#pragma once
#include "ChassisStateMachine.hpp"
#include "OperatorInput.hpp"
ChassisEvent ResolveChassisEvent(const OperatorInput &input, bool equipment_online);
ChassisEvent DetectChassisEvent(const OperatorInput &input, bool equipment_online);
