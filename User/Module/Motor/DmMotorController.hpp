#pragma once

#include "../../core/BSP/Motor/DM/DmMotor.hpp"
#include "../Gimbal/GimbalTypes.hpp"

class DmMotorController
{
public:
    explicit DmMotorController(BSP::Motor::DM::J4310<1> &motor)
        : motor_(motor)
    {
    }

    void Update(const GimbalOutput &gimbal_output);

private:
    void SendMitZero();
    void SendEnable();
    void RecoverEnable(uint32_t now);
    void SendMitTarget(const GimbalOutput &gimbal_output);

    BSP::Motor::DM::J4310<1> &motor_;
    bool previous_enable_request_ = false; // 检测使能请求的下降沿
    enum class Stage { SendEnable, WaitFeedback, SendDisable, ClearError, Running };
    Stage stage_ = Stage::SendEnable;
    uint32_t stage_tick_ = 0;
    uint32_t enable_feedback_sequence_ = 0;
    // 重试间隔和反馈超时，单位 ms；后续可根据实机调整。
    static constexpr uint32_t COMMAND_INTERVAL_MS = 10;
    static constexpr uint32_t FEEDBACK_TIMEOUT_MS = 100;
};
