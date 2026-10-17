#pragma once

#include <cstdint>

/**
 * @file ChassisConfig.hpp
 * @brief GENERATED FILE. Edit config/chassis.yaml, then regenerate this file.
 * @details Rebuild and flash firmware after changing the YAML configuration.
 */
namespace ChassisConfig
{
// ===== Wheel count and geometry (FK and IK must use matching values) =====
inline constexpr std::uint8_t WHEEL_COUNT = 4; ///< 当前正、逆运动学实现固定处理四个车轮；轮序须与几何、PID 和电机 ID 对齐。
static_assert(WHEEL_COUNT == 4, "Current OmniCalculation implementation requires exactly four wheels.");
inline constexpr float CHASSIS_RADIUS_M = 0.3f; ///< 底盘中心到车轮安装点的等效距离，单位：米；FK 与 IK 必须使用同一值。
inline constexpr float WHEEL_RADIUS_M = 0.07f; ///< 车轮半径，单位：米；FK 与 IK 必须使用同一值。

inline constexpr float WHEEL_AZIMUTH_RAD[WHEEL_COUNT] = {-0.7853981634f, -2.35619449f, 0.7853981634f, 2.35619449f}; ///< 各车轮滚子允许自由滚动方向相对底盘 X 轴的角度，单位：弧度；顺序对应电机 ID。
inline constexpr float WHEEL_DIRECTION_RAD[WHEEL_COUNT] = {0.7853981634f, -0.7853981634f, 2.35619449f, -2.35619449f}; ///< 底盘中心指向各车轮安装点的方向角，单位：弧度；顺序对应电机 ID。

/** @brief 每个车轮的速度 PID 参数，依次为 Kp、Ki、Kd、输出限幅、积分限幅和积分分离阈值。 */
struct WheelPid { float kp; float ki; float kd; float output_limit; float integral_limit; float integral_separation_threshold; };
inline constexpr float MOTOR_CURRENT_LIMIT = 16384.0f; ///< 轮速 PID 的输出限幅，单位：电调控制值；须与所用电机驱动量程相符。
inline constexpr WheelPid WHEEL_PID[WHEEL_COUNT] = {
    {700.0f, 0.5f, 0.0f, 16384.0f, 2500.0f, 100.0f},
    {700.0f, 0.5f, 0.0f, 16384.0f, 2500.0f, 100.0f},
    {700.0f, 0.5f, 0.0f, 16384.0f, 2500.0f, 100.0f},
    {700.0f, 0.5f, 0.0f, 16384.0f, 2500.0f, 100.0f}
};

inline constexpr float TRANSLATION_RAMP_INCREASE = 0.015f; ///< X/Y 平移目标每个控制周期允许的最大增加量。
inline constexpr float TRANSLATION_RAMP_DECREASE = 0.015f; ///< X/Y 平移目标每个控制周期允许的最大减少量。

inline constexpr float CONTROL_PERIOD_S = 0.001f; ///< 控制周期，单位：秒；须与 ControlTask 的实际调用周期一致。
inline constexpr float REMOTE_DEADZONE = 0.05f; ///< 遥控输入死区；输入绝对值小于该值时按零处理，输入范围为 [-1, 1]。
inline constexpr float REMOTE_LINEAR_GAIN = 4.5f; ///< 遥控平移速度增益，单位：米/秒；输入为 ±1 时对应目标速度。
inline constexpr float REMOTE_ROTATION_GAIN = 13.0f; ///< 遥控旋转速度增益，单位：弧度/秒；输入为 ±1 时对应目标角速度。
inline constexpr float PATROL_SPEED_MPS = 1.3f; ///< 自动巡航直线速度，单位：米/秒；去程和回程使用相反符号。
inline constexpr float PATROL_STOP_LINEAR_MPS = 0.05f; ///< 巡航端点停稳判定的平移速度阈值，单位：米/秒。
inline constexpr float PATROL_STOP_ANGULAR_RADPS = 0.05f; ///< 巡航端点停稳判定的角速度阈值，单位：弧度/秒。
inline constexpr unsigned PATROL_SETTLE_CYCLES = 200; ///< 端点连续满足停稳条件的周期数；等待时长约为本值乘以控制周期。
inline constexpr float PATROL_DISTANCE_M = 5.0f; ///< 自动巡航单程距离，单位：米。
inline constexpr float VEHICLE_TURN_RADIUS_M = 0.3f; ///< 将角速度换算为车轮等效线速度时使用的转弯半径，单位：米。
inline constexpr float MAX_WHEEL_SPEED_MPS = 5.2f; ///< 平移与旋转合成后的车轮线速度上限，单位：米/秒。

inline constexpr std::uint16_t MOTOR_INIT_CAN_ID = 512; ///< 底盘 3508 电机组的 CAN 接收帧 ID。
inline constexpr std::uint8_t MOTOR_RECEIVE_IDS[WHEEL_COUNT] = {1, 2, 3, 4}; ///< 底盘各车轮电机的接收 ID；数组顺序须与车轮布局和 PID 顺序一致。
inline constexpr std::uint32_t MOTOR_SEND_CAN_ID = 512; ///< 底盘 3508 电机组的 CAN 控制帧 ID。

inline constexpr double MOTOR_REDUCTION_RATIO = 15.7647058823529; ///< GM3508 电机减速比，用于电机反馈单位换算。
inline constexpr double MOTOR_TORQUE_CONSTANT = 0.3; ///< 电机力矩常数，单位：牛·米/安培。
inline constexpr double MOTOR_FEEDBACK_CURRENT_MAX = 16384; ///< 电机反馈电流的原始量程参数。
inline constexpr double MOTOR_MODEL_CURRENT_MAX = 20; ///< GM3508 电机模型最大电流参数，单位：安培。
inline constexpr double MOTOR_ENCODER_RESOLUTION = 8192; ///< 电机编码器分辨率，单位：编码器计数/圈。
} // namespace ChassisConfig
