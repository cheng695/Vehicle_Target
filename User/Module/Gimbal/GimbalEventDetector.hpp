#pragma once

#include "GimbalStateMachine.hpp"
#include "OperatorInput.hpp"

// 根据遥控器左右拨杆挡位，解析 Yaw 状态机事件。
GimbalYawEvent ResolveGimbalYawEvent(const OperatorInput &input);

// 仅在拨杆挡位发生变化时返回事件；没有变化时返回 None。
GimbalYawEvent DetectGimbalYawEvent(const OperatorInput &input);

// 根据遥控器左右拨杆挡位，解析 Hight 机构的启停请求。
GimbalHightEvent ResolveGimbalHightEvent(const OperatorInput &input);

// 仅在拨杆挡位发生变化时返回 Hight 事件；没有变化时返回 None。
GimbalHightEvent DetectGimbalHightEvent(const OperatorInput &input);

// 根据上下限位的当前电平生成限位事件；同时触发时返回故障事件。
GimbalHightEvent ResolveGimbalHightLimitEvent(bool upper_limit_active,
                                              bool lower_limit_active);
