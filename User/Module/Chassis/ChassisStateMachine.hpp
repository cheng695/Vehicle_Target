#pragma once

#include "core/Alg/FSM/FSMBase.hpp"
#include <cstdint>

enum class Enum_Chassis_States : uint8_t { STOP = 0, REMOTE, PATROL, STATUS_COUNT };

enum class ChassisEvent { None = 0, StopRequested, RemoteRequested, PatrolRequested };

class Chassis_FSM : public FSMBase<Enum_Chassis_States,
                                   static_cast<uint8_t>(Enum_Chassis_States::STATUS_COUNT)>
{
public:
    void Init();
    void Dispatch(ChassisEvent event);
    void ForceStop();

private:
    void TransitTo(Enum_Chassis_States next_state);
};
