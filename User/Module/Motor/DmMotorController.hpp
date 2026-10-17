#pragma once

#include "../../core/BSP/Motor/DM/DmMotor.hpp"
#include "../Gimbal/GimbalTypes.hpp"
#include "../Gimbal/GimbalConfig.hpp"

class DmMotorController
{
public:
    explicit DmMotorController(BSP::Motor::DM::J4310<GimbalConfig::YAW_MOTOR_COUNT> &motor)
        : motor_(motor)
    {
    }

    void Update(const GimbalOutput &gimbal_output);

private:
    void SendMitZero();
    void SendEnable();
    void RecoverEnable(uint32_t now);
    void SendMitTarget(const GimbalOutput &gimbal_output);

    BSP::Motor::DM::J4310<GimbalConfig::YAW_MOTOR_COUNT> &motor_;
    bool previous_enable_request_ = false; // 检测使能请求的下降沿
    enum class Stage { SendEnable, WaitFeedback, SendDisable, ClearError, Running };
    Stage stage_ = Stage::SendEnable;
    uint32_t stage_tick_ = 0;
    uint32_t enable_feedback_sequence_ = 0;
};
