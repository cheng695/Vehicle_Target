#pragma once

#include "ChassisStateMachine.hpp"
#include "ChassisConfig.hpp"
#include "ChassisTypes.hpp"
#include "OperatorInput.hpp"
#include "SlopePlanning.hpp"

/**
 * @brief 根据底盘状态生成运动目标，并维护自动巡航里程状态。
 */
class ChassisPlanner
{
public:
    /**
     * @brief 构造规划器并初始化平移速度斜坡规划器。
     */
    ChassisPlanner();

    /**
     * @brief 重置里程计、巡航状态和斜坡规划器。
     */
    void Init();

    /**
     * @brief 根据底盘状态和遥控器输入生成本周期的底盘目标。
     * @param state 当前底盘状态。
     * @param input 当前遥控器输入。
     * @return 本周期平移和旋转速度目标。
     */
    ChassisTarget Plan(Enum_Chassis_States state, const OperatorInput &input);

    /**
     * @brief 获取上电以来累计的 X 方向估算位置。
     * @return 轮式里程计位置，单位为米。
     */
    float GetEstimatedPosition() const;

    /**
     * @brief 获取相对于当前巡航起点的行驶距离。
     * @return 巡航相对位置，单位为米。
     */
    float GetPatrolTravelDistance() const;

    /**
     * @brief 查询自动巡航当前的行驶方向。
     * @return 正在驶向巡航端点时为 true，正在返回时为 false。
     */
    bool IsPatrolMovingForward() const;

private:
    // ===== 规划器运行状态：由规划逻辑维护，不是调参项 =====
    Alg::Utility::SlopePlanning translation_ramp_[2];                // X、Y 方向的平移斜坡规划器，用于限制目标变化速度。
    Enum_Chassis_States previous_state_ = Enum_Chassis_States::STOP; // 上一周期底盘状态，用于检测进入巡航状态。
    float estimated_position_x_ = 0.0f;                              // 轮式里程计累计的 X 方向位置，单位：米。
    float patrol_start_position_x_ = 0.0f;                           // 本次巡航起点的 X 方向位置，单位：米。
    bool patrol_forward_ = true;                                     // 巡航方向；true 为去程，false 为回程。
    bool patrol_waiting_for_stop_ = false;                           // 到达巡航端点后是否正在等待车辆停稳。
    unsigned patrol_settle_cycles_ = 0;                              // 当前连续满足停稳条件的周期计数。

    // ===== 内部规划步骤 =====
    /** @brief 对归一化遥控输入应用死区。 */
    static float ApplyDeadzone(float value);

    /** @brief 按车轮线速度上限同比例缩放平移和旋转目标。 */
    static void LimitTarget(ChassisTarget &target);

    /** @brief 将遥控器输入换算为底盘速度目标。 */
    ChassisTarget PlanRemote(const OperatorInput &input) const;

    /** @brief 根据里程计、巡航方向和端点停稳状态生成巡航目标。 */
    ChassisTarget PlanPatrol();

    /** @brief 对 X、Y 平移目标应用斜坡规划。 */
    void ApplyTranslationRamp(ChassisTarget &target);
};
