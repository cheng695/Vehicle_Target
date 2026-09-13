#pragma once

#include "ChassisStateMachine.hpp"
#include "ChassisTypes.hpp"
#include "OperatorInput.hpp"

// 初始化底盘目标规划器及自动巡航里程。
void InitChassisPlanner();

// 根据底盘状态和遥控器输入生成底盘速度目标。
ChassisTarget PlanChassisTarget(Enum_Chassis_States state,
                                const OperatorInput &input);

// 以下接口用于调试自动巡航里程和方向。
float GetChassisEstimatedPosition();
float GetPatrolTravelDistance();
bool IsPatrolMovingForward();
