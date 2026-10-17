#pragma once

#include "ChassisStateMachine.hpp"
#include "OperatorInput.hpp"

#include <cstdint>

/**
 * @brief 根据遥控器输入和设备在线状态检测底盘状态事件。
 * @details 保存上一周期输入，仅在输入或在线状态变化时产生事件。
 */
class ChassisEventDetector
{
public:
    /**
     * @brief 检测本周期是否产生底盘状态事件。
     * @param input 当前遥控器输入。
     * @param equipment_online 底盘控制依赖设备是否在线。
     * @return 输入状态变化对应的事件；没有变化时返回 ChassisEvent::None。
     */
    ChassisEvent Detect(const OperatorInput &input, bool equipment_online);

private:
    std::uint8_t last_left_switch_ = 0;
    std::uint8_t last_right_switch_ = 0;
    bool last_online_ = false;

    /**
     * @brief 按安全优先级将当前输入映射为底盘事件。
     * @param input 当前遥控器输入。
     * @param equipment_online 底盘控制依赖设备是否在线。
     * @return 当前输入对应的底盘事件。
     */
    ChassisEvent Resolve(const OperatorInput &input, bool equipment_online) const;

    /**
     * @brief 判断遥控器拨杆值是否属于有效档位。
     * @param position 待检查的拨杆档位值。
     * @return 档位值有效时为 true，否则为 false。
     */
    static bool IsValidSwitch(std::uint8_t position);
};
