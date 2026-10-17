#pragma once

#include <cstdint>

/**
 * @brief 底盘运行时调参变量。
 * @details 变量位于 RAM 并声明为 volatile，可在 Ozone 调试会话中直接修改；
 *          修改只在当前运行期间有效，重启后恢复为 YAML 生成的默认值。
 */
namespace ChassisTuning
{
extern volatile float WHEEL_PID_KP[4];                    ///< 每个车轮速度 PID 的比例增益；下标 0..3 对应车轮 1..4。
extern volatile float WHEEL_PID_KI[4];                    ///< 每个车轮速度 PID 的积分增益；下标 0..3 对应车轮 1..4。
extern volatile float WHEEL_PID_KD[4];                    ///< 每个车轮速度 PID 的微分增益；下标 0..3 对应车轮 1..4。
extern volatile float WHEEL_PID_OUTPUT_LIMIT[4];          ///< 每个车轮 PID 的输出限幅。
extern volatile float WHEEL_PID_INTEGRAL_LIMIT[4];        ///< 每个车轮 PID 的积分限幅。
extern volatile float WHEEL_PID_INTEGRAL_SEPARATION[4];   ///< 每个车轮 PID 的积分分离阈值。
extern volatile float MOTOR_CURRENT_LIMIT;                ///< 底盘电机电流指令限幅。

extern volatile float TRANSLATION_RAMP_INCREASE;           ///< 平移目标每周期最大增量。
extern volatile float TRANSLATION_RAMP_DECREASE;           ///< 平移目标每周期最大减量。
extern volatile float REMOTE_DEADZONE;                     ///< 遥控输入死区。
extern volatile float REMOTE_LINEAR_GAIN;                  ///< 遥控平移速度增益。
extern volatile float REMOTE_ROTATION_GAIN;                ///< 遥控旋转速度增益。

extern volatile float PATROL_SPEED_MPS;                    ///< 自动巡航速度。
extern volatile float PATROL_STOP_LINEAR_MPS;              ///< 巡航端点平移停稳阈值。
extern volatile float PATROL_STOP_ANGULAR_RADPS;           ///< 巡航端点角速度停稳阈值。
extern volatile std::uint32_t PATROL_SETTLE_CYCLES;        ///< 巡航端点停稳确认周期数。
extern volatile float PATROL_DISTANCE_M;                   ///< 自动巡航单程距离。
extern volatile float VEHICLE_TURN_RADIUS_M;               ///< 角速度换算等效线速度时使用的转弯半径。
extern volatile float MAX_WHEEL_SPEED_MPS;                 ///< 合成车轮线速度上限。
} // namespace ChassisTuning
