#include "GimbalEventDetector.hpp"

#include "DT7.hpp"

namespace
{
using RemoteController = BSP::REMOTE_CONTROL::RemoteController;
}

bool GimbalEventDetector::IsValidSwitch(uint8_t position)
{
    return position == RemoteController::UP ||
           position == RemoteController::MIDDLE ||
           position == RemoteController::DOWN;
}

GimbalYawEvent GimbalEventDetector::ResolveYawEvent(const OperatorInput &input) const
{
    // 拨杆数据异常时令 Yaw 停止，避免异常输入下继续转动。
    if (!IsValidSwitch(input.left_switch) || !IsValidSwitch(input.right_switch))
        return GimbalYawEvent::StopRequested;

    // 左杆在下或中：右杆下停止、中匀速、上变速。
    if (input.left_switch == RemoteController::DOWN ||
        input.left_switch == RemoteController::MIDDLE)
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

    // 左杆在上：右杆下停止，中或上均按匀速处理。
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

GimbalYawEvent GimbalEventDetector::DetectYawEvent(const OperatorInput &input)
{
    const bool input_changed = input.left_switch != yaw_last_left_switch_ ||
                               input.right_switch != yaw_last_right_switch_;
    yaw_last_left_switch_ = input.left_switch;
    yaw_last_right_switch_ = input.right_switch;

    return input_changed ? ResolveYawEvent(input) : GimbalYawEvent::None;
}

GimbalHightEvent GimbalEventDetector::ResolveHightEvent(const OperatorInput &input) const
{
    // 拨杆数据异常时请求停止循环，并由状态机控制返回下限位。
    if (!IsValidSwitch(input.left_switch) || !IsValidSwitch(input.right_switch))
        return GimbalHightEvent::StopRequested;

    // 左中右中、左中右上、左上右上时启动升降循环。
    const bool start_requested =
        (input.left_switch == RemoteController::MIDDLE &&
         (input.right_switch == RemoteController::MIDDLE ||
          input.right_switch == RemoteController::UP)) ||
        (input.left_switch == RemoteController::UP &&
         input.right_switch == RemoteController::UP);

    return start_requested ? GimbalHightEvent::StartRequested
                           : GimbalHightEvent::StopRequested;
}

GimbalHightEvent GimbalEventDetector::DetectHightEvent(const OperatorInput &input)
{
    const bool input_changed = input.left_switch != hight_last_left_switch_ ||
                               input.right_switch != hight_last_right_switch_;
    hight_last_left_switch_ = input.left_switch;
    hight_last_right_switch_ = input.right_switch;

    return input_changed ? ResolveHightEvent(input) : GimbalHightEvent::None;
}

GimbalHightEvent GimbalEventDetector::ResolveHightLimitEvent(bool upper_limit_active,
                                                            bool lower_limit_active) const
{
    if (upper_limit_active && lower_limit_active)
        return GimbalHightEvent::LimitFault;
    if (upper_limit_active)
        return GimbalHightEvent::UpperLimitTriggered;
    if (lower_limit_active)
        return GimbalHightEvent::LowerLimitTriggered;
    return GimbalHightEvent::None;
}

void GimbalEventDetector::Reset()
{
    yaw_last_left_switch_ = 0;
    yaw_last_right_switch_ = 0;
    hight_last_left_switch_ = 0;
    hight_last_right_switch_ = 0;
}
