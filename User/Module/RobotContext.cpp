#include "RobotContext.hpp"
#include "DjiMotor.hpp"
#include "ChassisConfig.hpp"

// 遥控器
BSP::REMOTE_CONTROL::RemoteController DT7;

// 电机
BSP::Motor::Dji::GM3508<ChassisConfig::WHEEL_COUNT> Motor3508(
    ChassisConfig::MOTOR_INIT_CAN_ID,
    ChassisConfig::MOTOR_RECEIVE_IDS,
    ChassisConfig::MOTOR_SEND_CAN_ID,
    BSP::Motor::Dji::Parameters(
        ChassisConfig::MOTOR_REDUCTION_RATIO,
        ChassisConfig::MOTOR_TORQUE_CONSTANT,
        ChassisConfig::MOTOR_FEEDBACK_CURRENT_MAX,
        ChassisConfig::MOTOR_MODEL_CURRENT_MAX,
        ChassisConfig::MOTOR_ENCODER_RESOLUTION));
BSP::Motor::Dji::GM3508<GimbalConfig::HIGHT_MOTOR_COUNT> Motor3508_5(
    GimbalConfig::HIGHT_MOTOR_INIT_CAN_ID,
    GimbalConfig::HIGHT_MOTOR_FEEDBACK_IDS,
    GimbalConfig::HIGHT_MOTOR_SEND_CAN_ID,
    BSP::Motor::Dji::Parameters(
        GimbalConfig::HIGHT_MOTOR_PARAMETERS.reduction_ratio,
        GimbalConfig::HIGHT_MOTOR_PARAMETERS.torque_constant_nm_per_a,
        GimbalConfig::HIGHT_MOTOR_PARAMETERS.feedback_current_max,
        GimbalConfig::HIGHT_MOTOR_PARAMETERS.motor_current_max_a,
        GimbalConfig::HIGHT_MOTOR_PARAMETERS.encoder_resolution));
BSP::Motor::DM::J4310<GimbalConfig::YAW_MOTOR_COUNT> MotorJ4310(
    GimbalConfig::YAW_MOTOR_INIT_CAN_ID,
    GimbalConfig::YAW_MOTOR_FEEDBACK_IDS,
    GimbalConfig::YAW_MOTOR_SEND_IDS,
    BSP::Motor::DM::Parameters(
        GimbalConfig::YAW_MOTOR_PARAMETERS.position_min_rad,
        GimbalConfig::YAW_MOTOR_PARAMETERS.position_max_rad,
        GimbalConfig::YAW_MOTOR_PARAMETERS.velocity_min_radps,
        GimbalConfig::YAW_MOTOR_PARAMETERS.velocity_max_radps,
        GimbalConfig::YAW_MOTOR_PARAMETERS.torque_min_nm,
        GimbalConfig::YAW_MOTOR_PARAMETERS.torque_max_nm,
        GimbalConfig::YAW_MOTOR_PARAMETERS.kp_min,
        GimbalConfig::YAW_MOTOR_PARAMETERS.kp_max,
        GimbalConfig::YAW_MOTOR_PARAMETERS.kd_min,
        GimbalConfig::YAW_MOTOR_PARAMETERS.kd_max));

// 输入
OperatorInput operator_input;

// 底盘目标和输出
ChassisTarget chassis_target;
ChassisOutput chassis_output;

// 云台状态机、目标值和电机输出。
GimbalTarget gimbal_target;
GimbalOutput gimbal_output;

Alg::CalculationBase::Omni_FK omni_fk(
    ChassisConfig::CHASSIS_RADIUS_M,
    ChassisConfig::WHEEL_RADIUS_M,
    static_cast<float>(ChassisConfig::WHEEL_COUNT),
    ChassisConfig::WHEEL_AZIMUTH_RAD,
    ChassisConfig::WHEEL_DIRECTION_RAD);

float M3508_velocity_rpm[ChassisConfig::WHEEL_COUNT] = {};
float DM4310_velocity_rpm[1] = {0.0f};
float M3508_5_velocity_rpm[1] = {0.0f};
float M3508_5_position[1] = {0.0f};
