#pragma once

#include "ChassisTypes.hpp"
#include "ChassisConfig.hpp"
#include "Gimbal/GimbalStateMachine.hpp"
#include "Gimbal/GimbalTypes.hpp"
#include "Gimbal/GimbalConfig.hpp"
#include "DT7.hpp"
#include "DjiMotor.hpp"
#include "DmMotor.hpp"
#include "OmniCalculation.hpp"
#include "OperatorInput.hpp"

/*
 * 跨任务共享对象集中声明。
 *
 * Task 层只负责调度，不再通过互相 include 对方的 Task 头文件来借用全局对象。
 */
extern BSP::REMOTE_CONTROL::RemoteController DT7;
extern BSP::Motor::Dji::GM3508<ChassisConfig::WHEEL_COUNT> Motor3508;
extern BSP::Motor::Dji::GM3508<GimbalConfig::HIGHT_MOTOR_COUNT> Motor3508_5;
extern BSP::Motor::DM::J4310<GimbalConfig::YAW_MOTOR_COUNT> MotorJ4310;

extern OperatorInput operator_input;

extern ChassisTarget chassis_target;
extern ChassisOutput chassis_output;

extern GimbalTarget gimbal_target;
extern GimbalOutput gimbal_output;

/*
 * 当前底盘速度反馈会被控制和调试发送共同使用，因此放在共享上下文。
 */
extern Alg::CalculationBase::Omni_FK omni_fk;

extern float M3508_velocity_rpm[ChassisConfig::WHEEL_COUNT];
extern float DM4310_velocity_rpm[1];
extern float M3508_5_velocity_rpm[1];
extern float M3508_5_position[1];


