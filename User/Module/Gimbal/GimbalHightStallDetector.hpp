#pragma once

#include "GimbalConfig.hpp"

#include <cmath>
#include <cstdint>

class GimbalHightStallDetector
{
public:
    void Reset()
    {
        timing_ = false;
        fault_ = false;
        direction_ = 0;
        start_ms_ = 0;
    }

    bool HasFault() const { return fault_; }

    bool Update(bool enabled, float target_radps, float actual_radps,
                float current_command, uint32_t now_ms)
    {
        if (fault_)
            return true;

        // 使用最终电流指令，限位已经切断输出时不累计堵转时间。
        const bool suspected = enabled &&
            std::fabs(target_radps) >= GimbalConfig::HIGHT_STALL_MIN_TARGET_RADPS &&
            std::fabs(actual_radps) < GimbalConfig::HIGHT_STALL_MAX_ACTUAL_RADPS &&
            std::fabs(current_command) >= GimbalConfig::HIGHT_STALL_MIN_CURRENT;
        if (!suspected)
        {
            timing_ = false;
            return false;
        }

        const int direction = target_radps > 0.0f ? 1 : -1;
        // 换向时重新计时，避免把两个方向的低速时间累计到一起。
        if (!timing_ || direction != direction_)
        {
            timing_ = true;
            direction_ = direction;
            start_ms_ = now_ms;
        }

        // 无符号差值允许毫秒计数正常回绕。
        fault_ = static_cast<uint32_t>(now_ms - start_ms_) >=
                 GimbalConfig::HIGHT_STALL_DURATION_MS;
        return fault_;
    }

private:
    // ==================== 运行状态：由检测逻辑维护，不是调参项 ====================
    // 当前是否正在累计堵转候选时间。
    bool timing_ = false;

    // 堵转故障锁存标志；故障置位后由 Reset() 清除。
    bool fault_ = false;

    // 当前计时方向：1 表示正向，-1 表示反向，0 表示尚未开始计时。
    int direction_ = 0;

    // 本轮堵转计时的起始系统时刻，单位：毫秒。
    uint32_t start_ms_ = 0;
};
