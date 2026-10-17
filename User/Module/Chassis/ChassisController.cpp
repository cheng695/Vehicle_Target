#include "ChassisController.hpp"

#include "ChassisStateMachine.hpp"
#include "ChassisTypes.hpp"
#include "ChassisTuning.hpp"
#include "RobotContext.hpp"

#include <algorithm>

ChassisController chassis_controller;

ChassisController::ChassisController()
    : omni_ik_(
          ChassisConfig::CHASSIS_RADIUS_M,
          ChassisConfig::WHEEL_RADIUS_M,
          ChassisConfig::WHEEL_AZIMUTH_RAD,
          ChassisConfig::WHEEL_DIRECTION_RAD),
      wheel_pid_{
          ALG::PID::PID(ChassisConfig::WHEEL_PID[0].kp, ChassisConfig::WHEEL_PID[0].ki, ChassisConfig::WHEEL_PID[0].kd,
                        ChassisConfig::WHEEL_PID[0].output_limit, ChassisConfig::WHEEL_PID[0].integral_limit, ChassisConfig::WHEEL_PID[0].integral_separation_threshold),
          ALG::PID::PID(ChassisConfig::WHEEL_PID[1].kp, ChassisConfig::WHEEL_PID[1].ki, ChassisConfig::WHEEL_PID[1].kd,
                        ChassisConfig::WHEEL_PID[1].output_limit, ChassisConfig::WHEEL_PID[1].integral_limit, ChassisConfig::WHEEL_PID[1].integral_separation_threshold),
          ALG::PID::PID(ChassisConfig::WHEEL_PID[2].kp, ChassisConfig::WHEEL_PID[2].ki, ChassisConfig::WHEEL_PID[2].kd,
                        ChassisConfig::WHEEL_PID[2].output_limit, ChassisConfig::WHEEL_PID[2].integral_limit, ChassisConfig::WHEEL_PID[2].integral_separation_threshold),
          ALG::PID::PID(ChassisConfig::WHEEL_PID[3].kp, ChassisConfig::WHEEL_PID[3].ki, ChassisConfig::WHEEL_PID[3].kd,
                        ChassisConfig::WHEEL_PID[3].output_limit, ChassisConfig::WHEEL_PID[3].integral_limit, ChassisConfig::WHEEL_PID[3].integral_separation_threshold)}
{
}

void ChassisController::UpdateForwardKinematics()
{
    omni_fk.OmniForKinematics(
        Motor3508.getVelocityRads(1),
        Motor3508.getVelocityRads(2),
        Motor3508.getVelocityRads(3),
        Motor3508.getVelocityRads(4));
}

void ChassisController::Stop()
{
    for (int i = 0; i < ChassisConfig::WHEEL_COUNT; ++i)
    {
        wheel_pid_[i].reset();
        chassis_output.out_wheel[i] = 0.0f;
    }
}

void ChassisController::Run(const ChassisTarget &target)
{
    omni_ik_.OmniInvKinematics(
        target.target_translation_x,
        target.target_translation_y,
        target.target_rotation,
        0.0f,
        1.0f,
        1.0f);

    for (int i = 0; i < ChassisConfig::WHEEL_COUNT; ++i)
    {
        wheel_pid_[i].setK(ChassisTuning::WHEEL_PID_KP[i],
                           ChassisTuning::WHEEL_PID_KI[i],
                           ChassisTuning::WHEEL_PID_KD[i]);
        wheel_pid_[i].setMax(ChassisTuning::WHEEL_PID_OUTPUT_LIMIT[i]);
        wheel_pid_[i].setIntegralLimit(
            ChassisTuning::WHEEL_PID_INTEGRAL_LIMIT[i]);
        wheel_pid_[i].setIntegralSeparation(
            ChassisTuning::WHEEL_PID_INTEGRAL_SEPARATION[i]);
        const float target_wheel_speed = omni_ik_.GetMotor(i);
        const float actual_wheel_speed = Motor3508.getVelocityRads(i + 1);

        wheel_pid_[i].UpDate(target_wheel_speed, actual_wheel_speed);
        const float motor_current_limit = ChassisTuning::MOTOR_CURRENT_LIMIT;
        chassis_output.out_wheel[i] = std::clamp(
            wheel_pid_[i].getOutput(), -motor_current_limit, motor_current_limit);
    }
}

void ChassisController::Init()
{
    state_machine_.Init();
    planner_.Init();

    for (int i = 0; i < ChassisConfig::WHEEL_COUNT; ++i)
    {
        wheel_pid_[i].reset();
        chassis_output.out_wheel[i] = 0.0f;
    }
}

void ChassisController::ControlLoop(bool is_online, const OperatorInput &input)
{
    state_machine_.Dispatch(event_detector_.Detect(input, is_online));

    if (!is_online)
    {
        state_machine_.ForceStop();
    }

    UpdateForwardKinematics();

    const Enum_Chassis_States current_state = state_machine_.GetCurrentState();
    chassis_target = planner_.Plan(current_state, input);

    switch (current_state)
    {
    case Enum_Chassis_States::REMOTE:
    case Enum_Chassis_States::PATROL:
        Run(chassis_target);
        break;

    case Enum_Chassis_States::STOP:
    default:
        Stop();
        break;
    }

    state_machine_.Update();

    for (int i = 0; i < ChassisConfig::WHEEL_COUNT; ++i)
    {
        M3508_velocity_rpm[i] = Motor3508.getVelocityRpm(i + 1);
    }
}
