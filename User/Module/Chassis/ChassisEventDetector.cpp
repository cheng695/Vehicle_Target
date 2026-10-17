#include "ChassisEventDetector.hpp"

#include "DT7.hpp"

using RemoteController = BSP::REMOTE_CONTROL::RemoteController;

bool ChassisEventDetector::IsValidSwitch(std::uint8_t position)
{
    return position == RemoteController::UP ||
           position == RemoteController::MIDDLE ||
           position == RemoteController::DOWN;
}

ChassisEvent ChassisEventDetector::Resolve(
    const OperatorInput &input,
    bool equipment_online) const
{
    // 安全条件具有最高优先级，任意关键设备离线时底盘必须停止。
    if (!equipment_online)
    {
        return ChassisEvent::StopRequested;
    }

    // 拨杆数据无效或遥控器数据不完整时，按照停止状态处理。
    if (!IsValidSwitch(input.left_switch) ||
        !IsValidSwitch(input.right_switch))
    {
        return ChassisEvent::StopRequested;
    }

    // 左拨杆上档：底盘进入自动巡航。
    // 在这一组模式中，右拨杆只负责选择云台工作方式，不影响底盘状态。
    if (input.left_switch == RemoteController::UP)
    {
        return ChassisEvent::PatrolRequested;
    }

    // 左拨杆中档：无论右拨杆处于哪一档，底盘都由遥控器控制。
    if (input.left_switch == RemoteController::MIDDLE)
    {
        return ChassisEvent::RemoteRequested;
    }

    // 左拨杆下档且右拨杆下档：整车停止。
    if (input.right_switch == RemoteController::DOWN)
    {
        return ChassisEvent::StopRequested;
    }

    // 左拨杆下档且右拨杆处于中档或上档：底盘由遥控器控制。
    return ChassisEvent::RemoteRequested;
}

ChassisEvent ChassisEventDetector::Detect(
    const OperatorInput &input,
    bool equipment_online)
{
    const bool input_changed =
        input.left_switch != last_left_switch_ ||
        input.right_switch != last_right_switch_ ||
        equipment_online != last_online_;

    last_left_switch_ = input.left_switch;
    last_right_switch_ = input.right_switch;
    last_online_ = equipment_online;

    if (!input_changed)
    {
        return ChassisEvent::None;
    }

    return Resolve(input, equipment_online);
}
