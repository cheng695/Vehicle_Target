#include "ChassisStateMachine.hpp"

namespace
{
constexpr const char *STATE_NAMES[STATUS_COUNT] = {"STOP", "REMOTE", "PATROL"};
bool IsValidState(Enum_Chassis_States state) { return state >= STOP && state < STATUS_COUNT; }
}

void Chassis_FSM::Init()
{
    for (int i = 0; i < STATUS_COUNT; ++i)
    {
        Status[i] = {STATE_NAMES[i], 0, 0, nullptr};
        State_Run_Time[i] = 0;
    }
    State_chassis = STOP;
    Status[STOP].Enter_Count = 1;
}

void Chassis_FSM::TransitTo(Enum_Chassis_States next_state)
{
    if (!IsValidState(next_state)) next_state = STOP;
    if (State_chassis == next_state) return;

    Status[State_chassis].Total_Run_Time += State_Run_Time[State_chassis];
    State_Run_Time[State_chassis] = 0;
    State_chassis = next_state;
    ++Status[State_chassis].Enter_Count;
}

void Chassis_FSM::Dispatch(ChassisEvent event)
{
    switch (event)
    {
    case ChassisEvent::StopRequested: TransitTo(STOP); break;
    case ChassisEvent::RemoteRequested: TransitTo(REMOTE); break;
    case ChassisEvent::PatrolRequested: TransitTo(PATROL); break;
    case ChassisEvent::None: default: break;
    }
}

void Chassis_FSM::ForceStop() { TransitTo(STOP); }
void Chassis_FSM::TIM_Update() { ++State_Run_Time[State_chassis]; }

uint32_t Chassis_FSM::Get_State_Run_Time(Enum_Chassis_States state) const
{
    if (!IsValidState(state)) return 0;
    return Status[state].Total_Run_Time + (state == State_chassis ? State_Run_Time[state] : 0U);
}

uint32_t Chassis_FSM::Get_State_Enter_Count(Enum_Chassis_States state) const
{
    return IsValidState(state) ? Status[state].Enter_Count : 0U;
}

void Chassis_FSM::Reset_State_Statistics(Enum_Chassis_States state)
{
    if (!IsValidState(state)) return;
    Status[state].Enter_Count = state == State_chassis ? 1U : 0U;
    Status[state].Total_Run_Time = 0;
    State_Run_Time[state] = 0;
}
