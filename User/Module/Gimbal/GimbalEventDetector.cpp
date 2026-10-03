#include "GimbalEventDetector.hpp"

#include "DT7.hpp"

namespace
{
using RemoteController = BSP::REMOTE_CONTROL::RemoteController;

bool IsValidSwitch(uint8_t position)
{
    return position == RemoteController::UP ||
           position == RemoteController::MIDDLE ||
           position == RemoteController::DOWN;
}
}

GimbalYawEvent ResolveGimbalYawEvent(const OperatorInput &input)
{
    // 拨杆数据异常时，Yaw 默认停止，保证异常输入下不会继续转动。
    if (!IsValidSwitch(input.left_switch) ||
        !IsValidSwitch(input.right_switch))
    {
        return GimbalYawEvent::StopRequested;
    }

    // 左下挡：右下停止，右中匀速，右上变速。
    if (input.left_switch == RemoteController::DOWN)
    {
        switch (input.right_switch)
        {
        case RemoteController::DOWN:
            return GimbalYawEvent::StopRequested;
        case RemoteController::MIDDLE:
            return GimbalYawEvent::ConstantSpeedRequested;
        case RemoteController::UP:
            return GimbalYawEvent::VariableSpeedRequested;
        default:
            return GimbalYawEvent::StopRequested;
        }
    }

    // 左中挡：右下停止，右中匀速，右上变速。
    if (input.left_switch == RemoteController::MIDDLE)
    {
        switch (input.right_switch)
        {
        case RemoteController::DOWN:
            return GimbalYawEvent::StopRequested;
        case RemoteController::MIDDLE:
            return GimbalYawEvent::ConstantSpeedRequested;
        case RemoteController::UP:
            return GimbalYawEvent::VariableSpeedRequested;
        default:
            return GimbalYawEvent::StopRequested;
        }
    }

    // 左上挡：右下停止，右中匀速，右上仍按匀速处理。
    if (input.left_switch == RemoteController::UP)
    {
        switch (input.right_switch)
        {
        case RemoteController::DOWN:
            return GimbalYawEvent::StopRequested;
        case RemoteController::MIDDLE:
        case RemoteController::UP:
            return GimbalYawEvent::ConstantSpeedRequested;
        default:
            return GimbalYawEvent::StopRequested;
        }
    }

    return GimbalYawEvent::StopRequested;
}

GimbalYawEvent DetectGimbalYawEvent(const OperatorInput &input)
{
    static uint8_t last_left_switch = 0;
    static uint8_t last_right_switch = 0;

    const bool input_changed =
        input.left_switch != last_left_switch ||
        input.right_switch != last_right_switch;

    last_left_switch = input.left_switch;
    last_right_switch = input.right_switch;

    if (!input_changed)
    {
        return GimbalYawEvent::None;
    }

    return ResolveGimbalYawEvent(input);
}

GimbalHightEvent ResolveGimbalHightEvent(const OperatorInput &input)
{
    // 拨杆数据异常时请求停止循环，并由状态机受控返回下限位。
    if (!IsValidSwitch(input.left_switch) ||
        !IsValidSwitch(input.right_switch))
    {
        return GimbalHightEvent::StopRequested;
    }

    // 左中右中、左中右上、左上右上时启动上下循环运动。
    const bool start_requested =
        (input.left_switch == RemoteController::MIDDLE &&
         (input.right_switch == RemoteController::MIDDLE ||
          input.right_switch == RemoteController::UP)) ||
        (input.left_switch == RemoteController::UP &&
         input.right_switch == RemoteController::UP);

    return start_requested
               ? GimbalHightEvent::StartRequested
               : GimbalHightEvent::StopRequested;
}

GimbalHightEvent DetectGimbalHightEvent(const OperatorInput &input)
{
    static uint8_t last_left_switch = 0;
    static uint8_t last_right_switch = 0;

    const bool input_changed =
        input.left_switch != last_left_switch ||
        input.right_switch != last_right_switch;

    last_left_switch = input.left_switch;
    last_right_switch = input.right_switch;

    if (!input_changed)
    {
        return GimbalHightEvent::None;
    }

    return ResolveGimbalHightEvent(input);
}

GimbalHightEvent ResolveGimbalHightLimitEvent(bool upper_limit_active,
                                              bool lower_limit_active)
{
    if (upper_limit_active && lower_limit_active)
        return GimbalHightEvent::LimitFault;
    if (upper_limit_active)
        return GimbalHightEvent::UpperLimitTriggered;
    if (lower_limit_active)
        return GimbalHightEvent::LowerLimitTriggered;
    return GimbalHightEvent::None;
}
