#include "ChassisTuning.hpp"

#include "ChassisConfig.hpp"

namespace ChassisTuning
{
volatile float WHEEL_PID_KP[4] = {
    ChassisConfig::WHEEL_PID[0].kp, ChassisConfig::WHEEL_PID[1].kp,
    ChassisConfig::WHEEL_PID[2].kp, ChassisConfig::WHEEL_PID[3].kp};
volatile float WHEEL_PID_KI[4] = {
    ChassisConfig::WHEEL_PID[0].ki, ChassisConfig::WHEEL_PID[1].ki,
    ChassisConfig::WHEEL_PID[2].ki, ChassisConfig::WHEEL_PID[3].ki};
volatile float WHEEL_PID_KD[4] = {
    ChassisConfig::WHEEL_PID[0].kd, ChassisConfig::WHEEL_PID[1].kd,
    ChassisConfig::WHEEL_PID[2].kd, ChassisConfig::WHEEL_PID[3].kd};
volatile float WHEEL_PID_OUTPUT_LIMIT[4] = {
    ChassisConfig::WHEEL_PID[0].output_limit, ChassisConfig::WHEEL_PID[1].output_limit,
    ChassisConfig::WHEEL_PID[2].output_limit, ChassisConfig::WHEEL_PID[3].output_limit};
volatile float WHEEL_PID_INTEGRAL_LIMIT[4] = {
    ChassisConfig::WHEEL_PID[0].integral_limit, ChassisConfig::WHEEL_PID[1].integral_limit,
    ChassisConfig::WHEEL_PID[2].integral_limit, ChassisConfig::WHEEL_PID[3].integral_limit};
volatile float WHEEL_PID_INTEGRAL_SEPARATION[4] = {
    ChassisConfig::WHEEL_PID[0].integral_separation_threshold,
    ChassisConfig::WHEEL_PID[1].integral_separation_threshold,
    ChassisConfig::WHEEL_PID[2].integral_separation_threshold,
    ChassisConfig::WHEEL_PID[3].integral_separation_threshold};
volatile float MOTOR_CURRENT_LIMIT = ChassisConfig::MOTOR_CURRENT_LIMIT;

volatile float TRANSLATION_RAMP_INCREASE = ChassisConfig::TRANSLATION_RAMP_INCREASE;
volatile float TRANSLATION_RAMP_DECREASE = ChassisConfig::TRANSLATION_RAMP_DECREASE;
volatile float REMOTE_DEADZONE = ChassisConfig::REMOTE_DEADZONE;
volatile float REMOTE_LINEAR_GAIN = ChassisConfig::REMOTE_LINEAR_GAIN;
volatile float REMOTE_ROTATION_GAIN = ChassisConfig::REMOTE_ROTATION_GAIN;

volatile float PATROL_SPEED_MPS = ChassisConfig::PATROL_SPEED_MPS;
volatile float PATROL_STOP_LINEAR_MPS = ChassisConfig::PATROL_STOP_LINEAR_MPS;
volatile float PATROL_STOP_ANGULAR_RADPS = ChassisConfig::PATROL_STOP_ANGULAR_RADPS;
volatile std::uint32_t PATROL_SETTLE_CYCLES = ChassisConfig::PATROL_SETTLE_CYCLES;
volatile float PATROL_DISTANCE_M = ChassisConfig::PATROL_DISTANCE_M;
volatile float VEHICLE_TURN_RADIUS_M = ChassisConfig::VEHICLE_TURN_RADIUS_M;
volatile float MAX_WHEEL_SPEED_MPS = ChassisConfig::MAX_WHEEL_SPEED_MPS;
} // namespace ChassisTuning
