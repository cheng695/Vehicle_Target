#include "ChassisStateMachine.hpp"

namespace
{
using ChassisFSMBase = FSMBase<Enum_Chassis_States,
                               static_cast<uint8_t>(Enum_Chassis_States::STATUS_COUNT)>;

constexpr ChassisFSMBase::StateInfo STATE_INFO[] = {
    {"STOP", 0, 0}, {"REMOTE", 0, 0}, {"PATROL", 0, 0}};
}

void Chassis_FSM::Init()
{
    FSMBase::Init(Enum_Chassis_States::STOP, STATE_INFO);
}

void Chassis_FSM::TransitTo(Enum_Chassis_States next_state)
{
    FSMBase::TransitTo(next_state);
}

void Chassis_FSM::Dispatch(ChassisEvent event)
{
    switch (event)
    {
    case ChassisEvent::StopRequested: TransitTo(Enum_Chassis_States::STOP); break;
    case ChassisEvent::RemoteRequested: TransitTo(Enum_Chassis_States::REMOTE); break;
    case ChassisEvent::PatrolRequested: TransitTo(Enum_Chassis_States::PATROL); break;
    case ChassisEvent::None: default: break;
    }
}

void Chassis_FSM::ForceStop()
{
    TransitTo(Enum_Chassis_States::STOP);
}

Enum_Chassis_States Chassis_FSM::GetCurrentState() const
{
    return ChassisFSMBase::Get_Now_State();
}

void Chassis_FSM::Update()
{
    ChassisFSMBase::TIM_Update();
}
