#pragma once

#include "GimbalStateMachine.hpp"
#include "OperatorInput.hpp"

#include <cstdint>

/**
 * @brief 将遥控器拨杆与升降机构限位状态转换为云台状态机事件。
 */
class GimbalEventDetector
{
public:
    /**
     * @brief 根据当前拨杆档位解析 Yaw 状态机事件。
     * @param input 当前遥控器输入。
     * @return 对应的 Yaw 事件；无效档位按停止事件处理。
     */
    GimbalYawEvent ResolveYawEvent(const OperatorInput &input) const;

    /**
     * @brief 仅在拨杆档位变化时生成 Yaw 事件。
     * @param input 当前遥控器输入。
     * @return 档位变化对应的 Yaw 事件；无变化时返回 None。
     */
    GimbalYawEvent DetectYawEvent(const OperatorInput &input);

    /**
     * @brief 根据当前拨杆档位解析升降机构启停事件。
     * @param input 当前遥控器输入。
     * @return 启动或停止升降循环的事件；无效档位按停止事件处理。
     */
    GimbalHightEvent ResolveHightEvent(const OperatorInput &input) const;

    /**
     * @brief 仅在拨杆档位变化时生成升降机构启停事件。
     * @param input 当前遥控器输入。
     * @return 档位变化对应的升降事件；无变化时返回 None。
     */
    GimbalHightEvent DetectHightEvent(const OperatorInput &input);

    /**
     * @brief 根据上下限位状态生成升降机构限位事件。
     * @param upper_limit_active 上限位是否触发。
     * @param lower_limit_active 下限位是否触发。
     * @return 对应的限位事件；两个限位同时触发时返回 LimitFault。
     */
    GimbalHightEvent ResolveHightLimitEvent(bool upper_limit_active,
                                           bool lower_limit_active) const;

    /**
     * @brief 清除档位变化检测器记录的上一次输入。
     */
    void Reset();

private:
    static bool IsValidSwitch(uint8_t position);

    // 分别记录 Yaw 与升降事件检测的前次输入，保留原先独立检测的行为。
    uint8_t yaw_last_left_switch_ = 0;
    uint8_t yaw_last_right_switch_ = 0;
    uint8_t hight_last_left_switch_ = 0;
    uint8_t hight_last_right_switch_ = 0;
};
