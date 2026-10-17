#include "GimbalStateMachine.hpp"

namespace
{
// 为每个状态提供调试名称和统计信息的初始值。
using GimbalYawFSMBase = FSMBase<
    GimbalYawState,
    static_cast<uint8_t>(GimbalYawState::STATUS_COUNT)>;

constexpr GimbalYawFSMBase::StateInfo YAW_STATE_INFO[] = {
    {"STOP", 0, 0},
    {"CONSTANT_SPEED", 0, 0},
    {"VARIABLE_SPEED", 0, 0}};

using GimbalHightFSMBase = FSMBase<
    GimbalHightState,
    static_cast<uint8_t>(GimbalHightState::STATUS_COUNT)>;

constexpr GimbalHightFSMBase::StateInfo HIGHT_STATE_INFO[] = {
    {"STOP", 0, 0},
    {"MOVING_UP", 0, 0},
    {"MOVING_DOWN", 0, 0},
    {"RETURNING_HOME", 0, 0}};
}

void GimbalYaw_FSM::Init()
{
    FSMBase::Init(GimbalYawState::STOP, YAW_STATE_INFO);
}

void GimbalYaw_FSM::TransitTo(GimbalYawState next_state)
{
    FSMBase::TransitTo(next_state);
}

void GimbalYaw_FSM::Dispatch(GimbalYawEvent event)
{
    switch (event)
    {
    case GimbalYawEvent::StopRequested:
        TransitTo(GimbalYawState::STOP);
        break;
    case GimbalYawEvent::ConstantSpeedRequested:
        TransitTo(GimbalYawState::CONSTANT_SPEED);
        break;
    case GimbalYawEvent::VariableSpeedRequested:
        TransitTo(GimbalYawState::VARIABLE_SPEED);
        break;
    case GimbalYawEvent::None:
    default:
        break;
    }
}

void GimbalYaw_FSM::ForceStop()
{
    TransitTo(GimbalYawState::STOP);
}

GimbalYawState GimbalYaw_FSM::GetCurrentState() const
{
    return GimbalYawFSMBase::Get_Now_State();
}

void GimbalYaw_FSM::UpdateTime()
{
    GimbalYawFSMBase::TIM_Update();
}

void GimbalHight_FSM::Init()
{
    // 上电时不假设机构已经在下限位，先执行一次受控回底。
    FSMBase::Init(GimbalHightState::RETURNING_HOME, HIGHT_STATE_INFO);
}

void GimbalHight_FSM::TransitTo(GimbalHightState next_state)
{
    FSMBase::TransitTo(next_state);
}

void GimbalHight_FSM::Dispatch(GimbalHightEvent event)
{
    switch (event)
    {
    case GimbalHightEvent::StartRequested:
        // 只有停止在下限位时才开始新一轮循环。
        if (Get_Now_State() == GimbalHightState::STOP)
            TransitTo(GimbalHightState::MOVING_UP);
        break;

    case GimbalHightEvent::StopRequested:
        // 运动过程中收到停止请求，先受控返回下限位。
        if (Get_Now_State() == GimbalHightState::MOVING_UP ||
            Get_Now_State() == GimbalHightState::MOVING_DOWN)
            TransitTo(GimbalHightState::RETURNING_HOME);
        break;

    case GimbalHightEvent::UpperLimitTriggered:
        // 上限位只在机构上行时生效；回零状态下不能阻止机构向下运动。
        if (Get_Now_State() == GimbalHightState::MOVING_UP)
            TransitTo(GimbalHightState::MOVING_DOWN);
        break;

    case GimbalHightEvent::LowerLimitTriggered:
        // 上行时下限位可能仍处于触发状态，机构离开下限位前不能中断上行。
        if (Get_Now_State() == GimbalHightState::MOVING_DOWN)
            TransitTo(GimbalHightState::MOVING_UP);
        else if (Get_Now_State() == GimbalHightState::RETURNING_HOME)
            TransitTo(GimbalHightState::STOP);
        break;

    case GimbalHightEvent::LimitFault:
        // 上下限位同时触发表示机械状态异常，立即停止。
        ForceStop();
        break;

    case GimbalHightEvent::None:
    default:
        break;
    }
}

void GimbalHight_FSM::ForceStop()
{
    TransitTo(GimbalHightState::STOP);
}

GimbalHightState GimbalHight_FSM::GetCurrentState() const
{
    return GimbalHightFSMBase::Get_Now_State();
}
