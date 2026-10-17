#include "DmMotorController.hpp"
#include "Gimbal/GimbalConfig.hpp"

void DmMotorController::SendMitZero()
{
    motor_.ctrl_Mit(GimbalConfig::YAW_MOTOR_CHANNEL, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

void DmMotorController::SendEnable()
{
    enable_feedback_sequence_ = motor_.getFeedbackSequence(GimbalConfig::YAW_MOTOR_CHANNEL);
    motor_.On(GimbalConfig::YAW_MOTOR_CHANNEL, BSP::Motor::DM::MIT);
    stage_tick_ = HAL_GetTick();
    stage_ = Stage::WaitFeedback;
}

void DmMotorController::RecoverEnable(uint32_t now)
{
    // 分周期发送，给电机处理命令和反馈留出时间。
    if (stage_ == Stage::SendDisable)
    {
        motor_.Off(GimbalConfig::YAW_MOTOR_CHANNEL, BSP::Motor::DM::MIT);
        stage_tick_ = now;
        stage_ = Stage::ClearError;
    }
    else if (stage_ == Stage::ClearError &&
             static_cast<uint32_t>(now - stage_tick_) >= GimbalConfig::YAW_MOTOR_COMMAND_INTERVAL_MS)
    {
        motor_.ClearErr(GimbalConfig::YAW_MOTOR_CHANNEL, BSP::Motor::DM::MIT);
        stage_tick_ = now;
        stage_ = Stage::SendEnable;
    }
}

void DmMotorController::SendMitTarget(const GimbalOutput &gimbal_output)
{
    motor_.ctrl_Mit(GimbalConfig::YAW_MOTOR_CHANNEL,
                    gimbal_output.out_gimbal_yaw[0],
                    gimbal_output.out_gimbal_yaw[1],
                    gimbal_output.out_gimbal_yaw[2],
                    gimbal_output.out_gimbal_yaw[3],
                    gimbal_output.out_gimbal_yaw[4]);
}

void DmMotorController::Update(const GimbalOutput &gimbal_output)
{
    if (!motor_.getIsenable())
    {
        stage_ = Stage::SendEnable;
        stage_tick_ = HAL_GetTick() - GimbalConfig::YAW_MOTOR_COMMAND_INTERVAL_MS;
        // 请求从使能切到失能时先发一次 Off，后续周期发送 MIT 全零。
        if (previous_enable_request_)
        {
            motor_.Off(GimbalConfig::YAW_MOTOR_CHANNEL, BSP::Motor::DM::MIT);
            previous_enable_request_ = false;
            return;
        }
        SendMitZero();
        return;
    }

    previous_enable_request_ = true;
    const uint32_t now = HAL_GetTick();
    switch (stage_)
    {
    case Stage::SendEnable:
        if (static_cast<uint32_t>(now - stage_tick_) >=
            GimbalConfig::YAW_MOTOR_COMMAND_INTERVAL_MS)
            SendEnable();
        break;
    case Stage::WaitFeedback:
        // 只接受本次使能命令之后收到的反馈，旧 err=1 不能作为成功依据。
        if (motor_.getFeedbackSequence(GimbalConfig::YAW_MOTOR_CHANNEL) != enable_feedback_sequence_ &&
            motor_.isFeedbackFresh(GimbalConfig::YAW_MOTOR_CHANNEL, now,
                                   GimbalConfig::YAW_MOTOR_FEEDBACK_TIMEOUT_MS))
        {
            stage_ = motor_.getError(GimbalConfig::YAW_MOTOR_CHANNEL) == 1 ? Stage::Running : Stage::SendDisable;
        }
        else if (static_cast<uint32_t>(now - stage_tick_) >=
                 GimbalConfig::YAW_MOTOR_FEEDBACK_TIMEOUT_MS)
        {
            stage_ = Stage::SendDisable;
        }
        break;
    case Stage::SendDisable:
    case Stage::ClearError:
        RecoverEnable(now);
        break;
    case Stage::Running:
        if (!motor_.isFeedbackFresh(GimbalConfig::YAW_MOTOR_CHANNEL, now,
                                   GimbalConfig::YAW_MOTOR_FEEDBACK_TIMEOUT_MS) ||
            motor_.getError(GimbalConfig::YAW_MOTOR_CHANNEL) != 1)
        {
            SendMitZero();
            stage_ = Stage::SendDisable;
        }
        else
        {
            SendMitTarget(gimbal_output);
        }
        break;
    }
}
