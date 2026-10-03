#pragma once

#include "core/Alg/FSM/FSMBase.hpp"

// Yaw 轴的转动模式。
// 状态机只负责区分转动模式，具体的速度指令由 Yaw 规划器生成。
enum class GimbalYawState : uint8_t
{
    STOP = 0,          // 不转动
    CONSTANT_SPEED,    // 匀速转动
    VARIABLE_SPEED,    // 变速转动
    STATUS_COUNT       // 状态数量
};

// Yaw 轴状态切换事件。
enum class GimbalYawEvent : uint8_t
{
    None = 0,                    // 无事件
    StopRequested,               // 请求停止转动
    ConstantSpeedRequested,      // 请求进入匀速转动
    VariableSpeedRequested       // 请求进入变速转动
};

// Yaw 轴状态机。
// 公共状态统计、状态运行时间和状态切换逻辑由 FSMBase 提供。
class GimbalYaw_FSM
    : public FSMBase<GimbalYawState,
                     static_cast<uint8_t>(GimbalYawState::STATUS_COUNT)>
{
public:
    // 初始化状态机，默认进入 STOP 状态。
    void Init();

    // 根据外部事件切换 Yaw 转动模式。
    void Dispatch(GimbalYawEvent event);

    // 强制停止 Yaw 转动。
    void ForceStop();

private:
    void TransitTo(GimbalYawState next_state);
};

// Hight 机构的运动状态。
// 正常运行时在上行和下行之间循环；收到停止请求后，受控返回下限位再停止。
enum class GimbalHightState : uint8_t
{
    STOP = 0,          // 已停在下限位
    MOVING_UP,         // 向上运动
    MOVING_DOWN,       // 向下运动
    RETURNING_HOME,    // 停止过程：受控返回下限位
    STATUS_COUNT       // 状态数量
};

// Hight 机构状态切换事件。
enum class GimbalHightEvent : uint8_t
{
    None = 0,             // 无事件
    StartRequested,       // 请求开始上下循环运动
    StopRequested,        // 请求结束循环并返回下限位
    UpperLimitTriggered,  // 上限位微触开关被触发
    LowerLimitTriggered,  // 下限位微触开关被触发
    LimitFault             // 上下限位同时触发
};

// Hight 机构状态机。
// 这里只决定运动阶段，具体的梯形速度规划由后续 Planner 实现。
class GimbalHight_FSM
    : public FSMBase<GimbalHightState,
                     static_cast<uint8_t>(GimbalHightState::STATUS_COUNT)>
{
public:
    // 初始化后默认认为机构位于下限位并进入 STOP 状态。
    void Init();

    // 根据启停请求和上下限位事件切换运动状态。
    void Dispatch(GimbalHightEvent event);

    // 紧急情况下立即进入 STOP，不执行自动回底过程。
    void ForceStop();

private:
    void TransitTo(GimbalHightState next_state);
};

// 对外统一使用的云台 FSM 名称。
using Gimbal_FSM = GimbalYaw_FSM;
