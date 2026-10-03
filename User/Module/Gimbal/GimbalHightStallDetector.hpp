#pragma once

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
            std::fabs(target_radps) >= MIN_TARGET_RADPS &&
            std::fabs(actual_radps) < MAX_STALLED_RADPS &&
            std::fabs(current_command) >= MIN_CURRENT_COMMAND;
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
        fault_ = static_cast<uint32_t>(now_ms - start_ms_) >= STALL_DURATION_MS;
        return fault_;
    }

private:
    // 初始阈值需结合负载、正常启动时间和速度噪声实测调整。
    static constexpr float MIN_TARGET_RADPS = 0.5f;       // 判定堵转所需的最小目标速度，单位：rad/s
    static constexpr float MAX_STALLED_RADPS = 0.1f;     // 判定为未运动的最大实际速度，单位：rad/s
    static constexpr float MIN_CURRENT_COMMAND = 500.0f; // 判定堵转所需的最小电流指令绝对值
    static constexpr uint32_t STALL_DURATION_MS = 500;   // 堵转条件必须持续的时间，单位：ms

    bool timing_ = false;
    bool fault_ = false;
    int direction_ = 0;
    uint32_t start_ms_ = 0;
};
