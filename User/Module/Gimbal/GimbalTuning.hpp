#pragma once

#include <cstdint>

/**
 * @brief 云台运行时调参变量。
 * @details 变量位于 RAM 并声明为 volatile，可在 Ozone 调试会话中直接修改；
 *          修改只在当前运行期间有效，重启后恢复为 YAML 生成的默认值。
 */
namespace GimbalTuning
{
extern volatile float YAW_TARGET_LIMIT_RADPS;              ///< Yaw 目标角速度上限。
extern volatile float YAW_CONSTANT_SPEED_STEP_RADPS;       ///< Yaw 匀速模式滚轮调速步长。
extern volatile float YAW_CONSTANT_SPEED_LIMIT_RADPS;      ///< Yaw 匀速模式速度上限。
extern volatile float YAW_SCROLL_THRESHOLD;                ///< 滚轮有效输入阈值。
extern volatile float YAW_VARIABLE_BASE_SPEED_RADPS;       ///< Yaw 变速曲线基础速度。
extern volatile float YAW_VARIABLE_AMPLITUDE_1_RADPS;      ///< Yaw 变速曲线第一项振幅。
extern volatile float YAW_VARIABLE_AMPLITUDE_2_RADPS;      ///< Yaw 变速曲线第二项振幅。
extern volatile float YAW_VARIABLE_FREQUENCY_1_HZ;         ///< Yaw 变速曲线第一项频率。
extern volatile float YAW_VARIABLE_FREQUENCY_2_HZ;         ///< Yaw 变速曲线第二项频率。
extern volatile float YAW_VARIABLE_PHASE_1_RAD;            ///< Yaw 变速曲线第一项相位。
extern volatile float YAW_VARIABLE_PHASE_2_RAD;            ///< Yaw 变速曲线第二项相位。

extern volatile float HIGHT_TRAVEL_DEG;                     ///< 升降总行程，减速器输出端角度。
extern volatile float HIGHT_UP_SPEED_RADPS;                ///< 升降上行速度。
extern volatile float HIGHT_DOWN_SPEED_RADPS;              ///< 升降下行速度。
extern volatile float HIGHT_HOME_SPEED_RADPS;              ///< 升降回下限位速度。
extern volatile float HIGHT_ACCELERATION_RADPS2;            ///< 升降速度斜坡加速度。
extern volatile float HIGHT_MAX_SPEED_RADPS;               ///< 升降规划速度上限。
extern volatile float HIGHT_SLOWDOWN_START_RATIO;           ///< 升降减速起点比例。
extern volatile float HIGHT_LIMIT_SPEED_RADPS;             ///< 升降限位搜索速度。
extern volatile float HIGHT_MIN_CONTROL_DT_S;              ///< 升降斜坡最小计算周期。
extern volatile float HIGHT_MAX_CONTROL_DT_S;              ///< 升降斜坡最大计算周期。

extern volatile float HIGHT_CURRENT_MAX;                    ///< 升降电流指令限幅。
extern volatile float HIGHT_GRAVITY_FEEDFORWARD;            ///< 升降重力前馈电流指令。

extern volatile float YAW_PID_KP;                           ///< Yaw MIT 控制的 Kp。
extern volatile float YAW_PID_KI;                           ///< Yaw PID Ki（当前控制流程未使用）。
extern volatile float YAW_PID_KD;                           ///< Yaw MIT 控制的 Kd。
extern volatile float YAW_PID_OUTPUT_LIMIT;                 ///< Yaw PID 输出限幅参数。
extern volatile float YAW_PID_INTEGRAL_LIMIT;               ///< Yaw PID 积分限幅参数。
extern volatile float YAW_PID_INTEGRAL_SEPARATION;          ///< Yaw PID 积分分离阈值。
extern volatile float HIGHT_PID_KP;                         ///< 升降速度 PID 的 Kp。
extern volatile float HIGHT_PID_KI;                         ///< 升降速度 PID 的 Ki。
extern volatile float HIGHT_PID_KD;                         ///< 升降速度 PID 的 Kd。
extern volatile float HIGHT_PID_OUTPUT_LIMIT;               ///< 升降速度 PID 输出限幅。
extern volatile float HIGHT_PID_INTEGRAL_LIMIT;             ///< 升降速度 PID 积分限幅。
extern volatile float HIGHT_PID_INTEGRAL_SEPARATION;        ///< 升降速度 PID 积分分离阈值。

} // namespace GimbalTuning
