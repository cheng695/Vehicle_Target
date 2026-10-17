#pragma once

#include <cstdint>

/**
 * @file GimbalConfig.hpp
 * @brief GENERATED FILE. Edit config/gimbal.yaml, then regenerate this file.
 * @details Rebuild and flash firmware after changing the YAML configuration.
 */
namespace GimbalConfig
{
/** @brief PID parameters: Kp, Ki, Kd, output limit, integral limit and separation threshold. */
struct PIDParameters { float kp; float ki; float kd; float output_limit; float integral_limit; float integral_separation_threshold; };
/** @brief DM motor MIT position, velocity, torque and gain ranges. */
struct DMMotorParameters { float position_min_rad; float position_max_rad; float velocity_min_radps; float velocity_max_radps; float torque_min_nm; float torque_max_nm; float kp_min; float kp_max; float kd_min; float kd_max; };
/** @brief DJI motor feedback and current unit conversion parameters. */
struct DJIMotorParameters { double reduction_ratio; double torque_constant_nm_per_a; double feedback_current_max; double motor_current_max_a; double encoder_resolution; };

// ===== Yaw axis speed planning =====
inline constexpr float YAW_TARGET_LIMIT_RADPS = 10.0f; ///< Yaw 目标角速度绝对值上限，单位：rad/s。
inline constexpr float YAW_CONSTANT_SPEED_STEP_RADPS = 2.6f; ///< 匀速模式每次滚轮拨动的速度步长，单位：rad/s。
inline constexpr float YAW_CONSTANT_SPEED_LIMIT_RADPS = 8.0f; ///< 匀速模式目标速度上限，单位：rad/s。
inline constexpr float YAW_SCROLL_THRESHOLD = 0.7f; ///< 滚轮有效拨动阈值；输入绝对值须大于此值。
inline constexpr float YAW_VARIABLE_BASE_SPEED_RADPS = 4.9f; ///< 变速曲线基础速度，单位：rad/s。
inline constexpr float YAW_VARIABLE_AMPLITUDE_1_RADPS = 1.5f; ///< 变速曲线第一项正弦振幅，单位：rad/s。
inline constexpr float YAW_VARIABLE_AMPLITUDE_2_RADPS = 0.6f; ///< 变速曲线第二项正弦振幅，单位：rad/s。
inline constexpr float YAW_VARIABLE_FREQUENCY_1_HZ = 1.5f; ///< 变速曲线第一项频率，单位：Hz。
inline constexpr float YAW_VARIABLE_FREQUENCY_2_HZ = 4.0f; ///< 变速曲线第二项频率，单位：Hz。
inline constexpr float YAW_VARIABLE_PHASE_1_RAD = -1.57079632679f; ///< 变速曲线第一项初相位，单位：rad。
inline constexpr float YAW_VARIABLE_PHASE_2_RAD = -1.57079632679f; ///< 变速曲线第二项初相位，单位：rad。

// ===== Hight mechanism speed planning =====
inline constexpr float HIGHT_TRAVEL_DEG = 670.0f; ///< 升降总行程；底部为零点、向上为正，单位：减速器输出端累计角度（度）。
inline constexpr float HIGHT_UP_SPEED_RADPS = 40.0f; ///< 升降上行目标角速度，单位：rad/s。
inline constexpr float HIGHT_DOWN_SPEED_RADPS = 40.0f; ///< 升降下行目标角速度绝对值，单位：rad/s。
inline constexpr float HIGHT_HOME_SPEED_RADPS = -3.0f; ///< 升降返回下限位速度，单位：rad/s，方向为负。
inline constexpr float HIGHT_ACCELERATION_RADPS2 = 5000.0f; ///< 升降速度斜坡加速度，单位：rad/s^2。
inline constexpr float HIGHT_MAX_SPEED_RADPS = 40.0f; ///< 升降规划速度绝对值上限，单位：rad/s。
inline constexpr float HIGHT_SLOWDOWN_START_RATIO = 0.7f; ///< 升降到达行程此比例后开始减速，范围通常为 [0, 1]。
inline constexpr float HIGHT_LIMIT_SPEED_RADPS = 3.0f; ///< 升降减速区寻找限位开关的低速，单位：rad/s。
inline constexpr float HIGHT_MIN_CONTROL_DT_S = 0.0005f; ///< 升降斜坡计算使用的最小控制周期，单位：秒。
inline constexpr float HIGHT_MAX_CONTROL_DT_S = 0.02f; ///< 升降斜坡计算使用的最大控制周期，单位：秒。

// ===== PID and current limits =====
inline constexpr float HIGHT_CURRENT_MAX = 16384.0f; ///< 升降最终电流命令绝对值上限。
inline constexpr float HIGHT_GRAVITY_FEEDFORWARD = 1100.0f; ///< 升降运动时叠加的重力前馈电流指令；有装甲板用 850，无装甲板用 500。
inline constexpr PIDParameters YAW_PID = {0.0f, 0.0f, 1.6f, 0.0f, 0.0f, 0.0f}; ///< Yaw PID 参数：Kp、Ki、Kd、输出限幅、积分限幅和积分分离阈值；当前流程使用 Kp/Kd 作为 MIT 指令参数。
inline constexpr PIDParameters HIGHT_PID = {3000.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f}; ///< 升降速度 PID 参数：Kp、Ki、Kd、输出限幅、积分限幅和积分分离阈值。

// ===== Limit switches and stall detection =====
inline constexpr bool HIGHT_LOWER_LIMIT_ACTIVE_LOW = true; ///< 下限位接 GPIO B；当前为低电平（GPIO_PIN_RESET）触发。true 表示低电平触发，false 表示高电平触发。
inline constexpr bool HIGHT_UPPER_LIMIT_ACTIVE_LOW = true; ///< 上限位接 GPIO A；当前为低电平（GPIO_PIN_RESET）触发。true 表示低电平触发，false 表示高电平触发。
inline constexpr bool HIGHT_STALL_DETECTION_ENABLED = false; ///< 堵转检测开关；当前检测器 Update 调用处于注释状态，尚未启用。
inline constexpr float HIGHT_STALL_MIN_TARGET_RADPS = 0.5f; ///< 堵转检测启用所需的最小目标角速度，单位：rad/s。
inline constexpr float HIGHT_STALL_MAX_ACTUAL_RADPS = 0.1f; ///< 判定升降机构未运动的实际角速度上限，单位：rad/s。
inline constexpr float HIGHT_STALL_MIN_CURRENT = 500.0f; ///< 堵转检测启用所需的最小电流命令绝对值。
inline constexpr std::uint32_t HIGHT_STALL_DURATION_MS = 500; ///< 堵转候选条件连续成立时间，单位：ms；当前检测未启用。

// ===== Hight GM3508 motor and CAN configuration =====
inline constexpr std::uint8_t HIGHT_MOTOR_COUNT = 1; ///< 升降 GM3508 电机数量。
inline constexpr std::uint16_t HIGHT_MOTOR_INIT_CAN_ID = 512; ///< 升降 GM3508 接收组 CAN ID。
inline constexpr std::uint8_t HIGHT_MOTOR_FEEDBACK_IDS[1] = {5}; ///< 升降 GM3508 反馈 ID 列表。
inline constexpr std::uint32_t HIGHT_MOTOR_SEND_CAN_ID = 511; ///< 升降 GM3508 控制组 CAN ID。
inline constexpr std::uint8_t HIGHT_MOTOR_CHANNEL = 1; ///< 升降 GM3508 单电机数组内的 1-based 通道号。
inline constexpr std::uint8_t HIGHT_MOTOR_RING_ID = 5; ///< 升降电机离线蜂鸣提示使用的反馈 ID。
inline constexpr DJIMotorParameters HIGHT_MOTOR_PARAMETERS = {15.7647058823529, 0.3, 16384, 20, 8192}; ///< GM3508 减速比、力矩常数、电流量程和编码器分辨率，用于反馈单位换算。

// ===== Yaw J4310 motor and CAN configuration =====
inline constexpr std::uint8_t YAW_MOTOR_COUNT = 1; ///< Yaw J4310 电机数量。
inline constexpr std::uint16_t YAW_MOTOR_INIT_CAN_ID = 0; ///< Yaw J4310 初始化 CAN ID。
inline constexpr std::uint8_t YAW_MOTOR_FEEDBACK_IDS[1] = {1}; ///< Yaw J4310 反馈 ID 列表。
inline constexpr std::uint32_t YAW_MOTOR_SEND_IDS[1] = {1}; ///< Yaw J4310 控制 CAN ID 列表。
inline constexpr std::uint8_t YAW_MOTOR_CHANNEL = 1; ///< Yaw J4310 单电机数组内的 1-based 通道号。
inline constexpr DMMotorParameters YAW_MOTOR_PARAMETERS = {-12.56f, 12.56f, -30.0f, 30.0f, -10.0f, 10.0f, 0.0f, 500.0f, 0.0f, 5.0f}; ///< J4310 MIT 参数范围，依次为位置、速度、力矩、Kp 和 Kd 的最小值与最大值。
inline constexpr std::uint32_t YAW_MOTOR_COMMAND_INTERVAL_MS = 10; ///< Yaw 电机使能、关闭及清错命令间隔，单位：ms。
inline constexpr std::uint32_t YAW_MOTOR_FEEDBACK_TIMEOUT_MS = 100; ///< Yaw 电机等待使能反馈及运行反馈的超时时间，单位：ms。
} // namespace GimbalConfig
