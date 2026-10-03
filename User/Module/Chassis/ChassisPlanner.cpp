#include "ChassisPlanner.hpp"
#include "ChassisStateMachine.hpp"
#include "OmniCalculation.hpp"
#include "SlopePlanning.hpp"
#include "ChassisTypes.hpp"
#include "OperatorInput.hpp"

#include <cmath>

// 底盘正运动学对象由 RobotContext.cpp 创建。
// Controller 使用四个车轮的实际转速更新它，Planner 读取其速度作为里程计反馈。
extern Alg::CalculationBase::Omni_FK omni_fk;

namespace
{
constexpr float CONTROL_PERIOD_S = 0.001f;       // 控制任务周期：1 ms
constexpr float REMOTE_DEADZONE = 0.05f;         // 遥控器输入死区
constexpr float REMOTE_LINEAR_GAIN = 4.5f;       // 遥控最大平移速度：m/s
constexpr float REMOTE_ROTATION_GAIN = 13.0f;    // 遥控最大旋转速度：rad/s
constexpr float PATROL_SPEED_MPS = 1.1f;         // 自动巡航速度：m/s
constexpr float PATROL_STOP_LINEAR_MPS = 0.05f;  // 停稳判定：平移速度上限
constexpr float PATROL_STOP_ANGULAR_RADPS = 0.05f; // 停稳判定：角速度上限
constexpr unsigned PATROL_SETTLE_CYCLES = 200;     // 连续停稳 200 个 1 ms 周期
constexpr float PATROL_DISTANCE_M = 5.0f;          // 自动巡航单程距离：m
constexpr float VEHICLE_TURN_RADIUS_M = 0.3f;      // 底盘等效旋转半径：m
constexpr float MAX_WHEEL_SPEED_MPS = 5.2f;        // 车轮允许的最大线速度：m/s

// X、Y 两个平移方向各使用一个斜坡规划器，限制速度指令突变。
// 构造参数分别表示每个控制周期允许的最大增加量和最大减小量。
Alg::Utility::SlopePlanning translation_ramp[2] = {
    Alg::Utility::SlopePlanning(0.015f, 0.015f),
    Alg::Utility::SlopePlanning(0.015f, 0.015f)};

// 上一周期状态，用于检测是否刚刚进入自动巡航模式。
Enum_Chassis_States previous_state = Enum_Chassis_States::STOP;

// X 方向累计位置，单位为 m。
// 该值由正运动学计算出的实际 Vx 积分得到，是轮式里程计的当前位置。
float estimated_position_x = 0.0f;

// 本次进入自动巡航时的 X 方向位置，作为 0 m 起点。
float patrol_start_position_x = 0.0f;

// true 表示从起点驶向 5 m 端点，false 表示从 5 m 端点返回起点。
bool patrol_forward = true;
bool patrol_waiting_for_stop = false;
unsigned patrol_settle_cycles = 0;

// 对遥控器归一化输入应用死区，避免摇杆中位噪声引起底盘缓慢运动。
float ApplyDeadzone(float value)
{
    return std::fabs(value) < REMOTE_DEADZONE ? 0.0f : value;
}

/*
 * 对平移和旋转的合成目标进行统一限幅。
 * 旋转角速度乘以底盘等效半径后，换算成车轮所需的等效线速度；
 * 当平移速度与旋转等效速度之和超过车轮上限时，三个方向同比例缩小，
 * 从而保留原始运动方向和旋转比例。
 */
void LimitChassisTarget(ChassisTarget &target)
{
    const float translation_speed =
        std::sqrt(target.target_translation_x * target.target_translation_x +
                  target.target_translation_y * target.target_translation_y);   // 平移速度大小

    const float rotation_equivalent_speed = std::fabs(target.target_rotation) * VEHICLE_TURN_RADIUS_M;  // 将旋转角速度换算为车轮等效线速度
    
    const float required_wheel_speed = translation_speed + rotation_equivalent_speed; // 车轮所需的总线速度

    if (required_wheel_speed > MAX_WHEEL_SPEED_MPS)
    {
        const float scale = MAX_WHEEL_SPEED_MPS / required_wheel_speed;
        target.target_translation_x *= scale;
        target.target_translation_y *= scale;
        target.target_rotation *= scale;
    }
}

/*
 * 遥控模式规划：
 * 1. OperatorInput 中的三个输入范围均为 [-1, 1]；
 * 2. 先应用摇杆死区；
 * 3. 分别乘以平移、旋转增益，转换为 m/s 和 rad/s；
 * 4. 最后根据车轮最大线速度限制合成目标。
 */
ChassisTarget PlanRemoteTarget(const OperatorInput &input)
{
    ChassisTarget target;
    target.target_translation_x = REMOTE_LINEAR_GAIN   * ApplyDeadzone(input.move_x);
    target.target_translation_y = REMOTE_LINEAR_GAIN   * ApplyDeadzone(input.move_y);
    target.target_rotation      = REMOTE_ROTATION_GAIN * ApplyDeadzone(input.rotate);

    LimitChassisTarget(target);
    return target;
}

/*
 * 自动巡航模式规划：
 * - 巡航区间固定为本次起点到起点前方 5 m；
 * - 去程速度为 +1.3 m/s，回程速度为 -1.3 m/s；
 * - 到达端点后先减速至零，连续停稳 500 ms 后换向，不退出 PATROL 状态；
 * - Y 方向速度与自转角速度始终为零。
 */
ChassisTarget PlanPatrolTarget()
{
    ChassisTarget target;
    const float relative_position = estimated_position_x - patrol_start_position_x;

    // 相对位置使用固定起点计算，避免每次换向都重新累计造成端点漂移。
    // 锁存停车阶段，避免减速时的位置波动提前恢复行驶。
    if (!patrol_waiting_for_stop &&
        ((patrol_forward && relative_position >= PATROL_DISTANCE_M) ||
         (!patrol_forward && relative_position <= 0.0f)))
    {
        patrol_waiting_for_stop = true;
        patrol_settle_cycles = 0;
    }

    if (patrol_waiting_for_stop)
    {
        const float vx = omni_fk.GetChassisVx();
        const float vy = omni_fk.GetChassisVy();
        const bool stopped =
            vx * vx + vy * vy < PATROL_STOP_LINEAR_MPS * PATROL_STOP_LINEAR_MPS &&
            std::fabs(omni_fk.GetChassisVw()) < PATROL_STOP_ANGULAR_RADPS;
        patrol_settle_cycles = stopped ? patrol_settle_cycles + 1 : 0;

        if (patrol_settle_cycles >= PATROL_SETTLE_CYCLES)
        {
            patrol_forward = !patrol_forward;
            patrol_waiting_for_stop = false;
            patrol_settle_cycles = 0;
        }

        // 零目标仍经过斜坡和速度闭环；下一周期才开始反向加速。
        return target;
    }

    target.target_translation_x = patrol_forward ? PATROL_SPEED_MPS : -PATROL_SPEED_MPS;
    target.target_translation_y = 0.0f;
    target.target_rotation = 0.0f;
    return target;
}

/*
 * 对 X、Y 平移目标执行斜坡规划。
 * 第二个参数使用正运动学得到的实际底盘速度，使规划器以当前真实速度
 * 作为过渡起点。旋转目标当前不做斜坡处理。
 */
void ApplyTranslationRamp(ChassisTarget &target)
{
    translation_ramp[0].TIM_Calculate_PeriodElapsedCallback(target.target_translation_x, omni_fk.GetChassisVx());
    translation_ramp[1].TIM_Calculate_PeriodElapsedCallback(target.target_translation_y, omni_fk.GetChassisVy());

    target.target_translation_x = translation_ramp[0].GetOut();
    target.target_translation_y = translation_ramp[1].GetOut();
}
}

void InitChassisPlanner()
{
    // 上电或重新初始化控制系统时清空里程，并规定首次巡航从正方向开始。
    previous_state = Enum_Chassis_States::STOP;
    estimated_position_x = 0.0f;
    patrol_start_position_x = 0.0f;
    patrol_forward = true;
    patrol_waiting_for_stop = false;
    patrol_settle_cycles = 0;
}

ChassisTarget PlanChassisTarget(Enum_Chassis_States state, const OperatorInput &input)
{
    /*
     * 轮式里程计：x(k) = x(k-1) + vx(k) × dt。
     * omni_fk.GetChassisVx() 来自四个 3508 实际轮速的正运动学解算，
     * CONTROL_PERIOD_S 当前固定为 ControlTask 的 1 ms 调用周期。
     */
    estimated_position_x += omni_fk.GetChassisVx() * CONTROL_PERIOD_S;

    // 每次从其他状态进入 PATROL，都以当前位置作为新的起点并先向前行驶。
    if (state == Enum_Chassis_States::PATROL &&
        previous_state != Enum_Chassis_States::PATROL)
    {
        patrol_start_position_x = estimated_position_x;
        patrol_forward = true;
        patrol_waiting_for_stop = false;
        patrol_settle_cycles = 0;
    }
    previous_state = state;

    ChassisTarget target;
    switch (state)
    {
    case Enum_Chassis_States::REMOTE:
        // 遥控器直接决定底盘的 X、Y 平移速度和自转角速度。
        target = PlanRemoteTarget(input);
        break;

    case Enum_Chassis_States::PATROL:
        // 根据轮式里程计位置生成 ±1.3 m/s 的往返速度目标。
        target = PlanPatrolTarget();
        break;

    case Enum_Chassis_States::STOP:
    default:
        // 默认构造的目标三个方向均为零，未知状态也按停止处理。
        break;
    }

    // 所有状态（包括停止）都经过斜坡规划，避免速度指令阶跃。
    ApplyTranslationRamp(target);
    return target;
}

float GetChassisEstimatedPosition()
{
    // 返回上电以来累计的 X 方向轮式里程计位置，单位为 m。
    return estimated_position_x;
}

float GetPatrolTravelDistance()
{
    // 返回相对于本次巡航起点的位置，正常往返范围约为 0～5 m。
    return estimated_position_x - patrol_start_position_x;
}

bool IsPatrolMovingForward()
{
    // true：驶向 5 m 端点；false：返回巡航起点。
    return patrol_forward;
}
