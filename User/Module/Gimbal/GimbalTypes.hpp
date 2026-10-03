#pragma once

struct GimbalTarget
{
    float target_yaw_velocity = 0.0f;        // Yaw 轴目标速度，单位为 rad/s
    float target_hight_velocity = 0.0f;      // 高度 目标速度，单位为 rad/s
};


struct GimbalOutput
{
    float out_gimbal_yaw[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // Yaw 电机输出 pos vel kp kd tor
    float out_gimbal_hight = 0.0f; // 高度电机输出
};
