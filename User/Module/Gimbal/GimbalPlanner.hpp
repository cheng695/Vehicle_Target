#pragma once

#include "GimbalStateMachine.hpp"
#include "GimbalTypes.hpp"
#include "OperatorInput.hpp"
#include "GimbalConfig.hpp"

#include <cstdint>

/**
 * @brief 根据状态和遥控输入规划云台 Yaw 轴目标角速度。
 */
class GimbalYawPlanner
{
public:
    /**
     * @brief 重置 Yaw 规划器内部状态。
     */
    void Init();

    /**
     * @brief 计算当前控制周期的 Yaw 目标速度。
     * @param state 当前 Yaw 状态。
     * @param input 当前遥控输入。
     * @param time_s 当前控制时间，单位：秒。
     * @return 包含 Yaw 目标角速度的目标对象。
     */
    GimbalTarget Plan(GimbalYawState state, const OperatorInput &input, float time_s);

private:
    // ==================== 私有辅助函数 ====================
    static bool IsScrollPositive(float scroll);
    static bool IsScrollNegative(float scroll);
    static float GetVariableSpeed(float time_s);
    static void LimitTarget(GimbalTarget &target);

    // ==================== 运行状态：由规划器维护，不是调参项 ====================
    // 当前变速运动段的起始时间，单位：秒。
    float variable_start_time_s_ = 0.0f;

    // 上一周期的状态，用于检测模式切换。
    GimbalYawState previous_state_ = GimbalYawState::STOP;

    // 匀速模式当前的累积目标角速度，单位：rad/s。
    float constant_speed_target_ = 0.0f;

    // 变速模式方向：-1 反向，0 停止，1 正向。
    int8_t variable_direction_ = 0;

    // 滚轮事件锁存状态；要求滚轮回中后才接受下一次拨动。
    bool scroll_event_latched_ = false;
};

/**
 * @brief 根据状态、位置和限位规划云台升降机构目标角速度。
 */
class GimbalHightPlanner
{
public:
    /**
     * @brief 重置升降规划器的速度和时间状态。
     */
    void Init();

    /**
     * @brief 规划当前控制周期的升降机构目标角速度。
     * @param state 当前升降机构状态。
     * @param time_s 当前控制时间，单位：秒。
     * @param upper_limit_active 上限位是否触发。
     * @param lower_limit_active 下限位是否触发。
     * @param position_deg 相对下限位零点的位置，单位：度。
     * @param position_valid 位置零点是否已经校准有效。
     * @return 包含升降机构目标角速度的目标对象。
     */
    GimbalTarget Plan(GimbalHightState state, float time_s,
                      bool upper_limit_active, bool lower_limit_active,
                      float position_deg, bool position_valid);

private:
    // ==================== 私有辅助函数 ====================
    static float GetTargetSpeed(GimbalHightState state);
    static float LimitSpeedByPosition(float speed_radps, float position_deg,
                                      bool position_valid);

    // ==================== 运行状态：由规划器维护，不是调参项 ====================
    // 上一周期状态，供规划器记录状态变化。
    GimbalHightState previous_state_ = GimbalHightState::STOP;

    // 当前斜坡规划后的速度，单位：rad/s。
    float current_speed_radps_ = 0.0f;

    // 上一规划周期的时间，单位：秒。
    float previous_time_s_ = 0.0f;

    // 标记是否已完成首次时间初始化，避免首周期产生异常 dt。
    bool initialized_ = false;
};
