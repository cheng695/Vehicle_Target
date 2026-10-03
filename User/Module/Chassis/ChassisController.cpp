#include "ChassisController.hpp"
#include "ChassisStateMachine.hpp"
#include "ChassisEventDetector.hpp"
#include "ChassisPlanner.hpp"
#include "OmniCalculation.hpp"
#include "RobotContext.hpp"
#include "pid.hpp"

#include <algorithm>

namespace
{
constexpr int CHASSIS_WHEEL_COUNT = 4;
constexpr float CHASSIS_RADIUS_M = 0.30f;
constexpr float WHEEL_RADIUS_M = 0.07f;
constexpr float MOTOR_CURRENT_LIMIT = 16384.0f;

/*
 * 底盘逆运动学对象。
 * 半径和轮径必须与 RobotContext.cpp 中的 omni_fk 完全一致，
 * 否则目标轮速与实际速度反馈将使用两套不同的底盘模型。
 */
Alg::CalculationBase::Omni_IK omni_ik(
    CHASSIS_RADIUS_M,
    WHEEL_RADIUS_M,
    chassis_wheel_azimuth,
    chassis_wheel_direction);

// 四个底盘 3508 分别使用独立的速度 PID。
ALG::PID::PID wheel_pid[CHASSIS_WHEEL_COUNT] = {
    ALG::PID::PID(700.0f, 0.5f, 0.0f, MOTOR_CURRENT_LIMIT, 2500.0f, 100.0f),
    ALG::PID::PID(700.0f, 0.5f, 0.0f, MOTOR_CURRENT_LIMIT, 2500.0f, 100.0f),
    ALG::PID::PID(700.0f, 0.5f, 0.0f, MOTOR_CURRENT_LIMIT, 2500.0f, 100.0f),
    ALG::PID::PID(700.0f, 0.5f, 0.0f, MOTOR_CURRENT_LIMIT, 2500.0f, 150.0f)};

/*
 * 每个控制周期都读取四个底盘电机的实际轮速并执行 FK。
 * FK 输出的 Vx、Vy、Vw 是底盘的实际速度反馈，其中 Vx 会被
 * ChassisPlanner 用于轮式里程计的位置积分。
 */
void UpdateChassisForwardKinematics()
{
    omni_fk.OmniForKinematics(
        Motor3508.getVelocityRads(1),
        Motor3508.getVelocityRads(2),
        Motor3508.getVelocityRads(3),
        Motor3508.getVelocityRads(4));
}

// 停止四个底盘电机，并清空速度 PID 的历史状态。
void StopChassis()
{
    for (int i = 0; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        wheel_pid[i].reset();
        chassis_output.out_wheel[i] = 0.0f;
    }
}

/*
 * 将 Planner 给出的底盘 Vx、Vy、Vw 目标通过 IK 转换为四个目标轮速，
 * 再使用各轮的实际转速进行速度闭环，最终输出 DJI 电调电流指令。
 */
void RunChassisController(const ChassisTarget &target)
{
    omni_ik.OmniInvKinematics(
        target.target_translation_x,
        target.target_translation_y,
        target.target_rotation,
        0.0f,
        1.0f,
        1.0f);

    for (int i = 0; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        const float target_wheel_speed = omni_ik.GetMotor(i);
        const float actual_wheel_speed = Motor3508.getVelocityRads(i + 1);

        wheel_pid[i].UpDate(target_wheel_speed, actual_wheel_speed);
        chassis_output.out_wheel[i] = std::clamp(wheel_pid[i].getOutput(), -MOTOR_CURRENT_LIMIT, MOTOR_CURRENT_LIMIT);
    }
}
}

void InitChassisControl()
{
    chassis_fsm.Init();
    InitChassisPlanner();

    for (int i = 0; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        wheel_pid[i].reset();
        chassis_output.out_wheel[i] = 0.0f;
    }
}

void ChassisControlLoop(bool is_online, const OperatorInput &input)
{
    // 将设备在线状态和遥控器拨杆组合转换为底盘状态事件。
    chassis_fsm.Dispatch(DetectChassisEvent(input, is_online));

    // 设备离线保护优先级最高，不允许其他事件覆盖 STOP。
    if (!is_online)
    {
        chassis_fsm.ForceStop();
    }

    // 无论当前是否停止，都更新 FK，保证速度反馈和轮式里程计连续有效。
    UpdateChassisForwardKinematics();

    // Planner 使用最新的 FK 速度反馈生成本周期运动目标。
    chassis_target = PlanChassisTarget(chassis_fsm.Get_Now_State(), input);

    switch (chassis_fsm.Get_Now_State())
    {
    case Enum_Chassis_States::REMOTE:
    case Enum_Chassis_States::PATROL:
        RunChassisController(chassis_target);
        break;

    case Enum_Chassis_States::STOP:
    default:
        StopChassis();
        break;
    }

    // 当前调用周期计入 FSM 状态运行时间。
    chassis_fsm.TIM_Update();

    for (int i = 0; i < 4; ++i)
    {
        M3508_velocity_rpm[i] = Motor3508.getVelocityRpm(i + 1);
    }
}
