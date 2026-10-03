#pragma once

#include <stdint.h>

// 通用有限状态机基类。
// State 必须是 enum class，StateCount 为状态数量。
template <typename State, uint8_t StateCount>
class FSMBase
{
public:
    struct StateInfo
    {
        const char *name = nullptr;
        uint32_t enter_count = 0;
        uint32_t total_run_time = 0;
    };

    void Init(State initial_state, const StateInfo *state_info)
    {
        current_state_ = IsValid(initial_state) ? initial_state : State{};

        for (uint8_t i = 0; i < StateCount; ++i)
        {
            states_[i] = state_info[i];
            states_[i].enter_count = 0;
            states_[i].total_run_time = 0;
            current_run_time_[i] = 0;
        }

        states_[ToIndex(current_state_)].enter_count = 1;
    }

    void TIM_Update()
    {
        ++current_run_time_[ToIndex(current_state_)];
    }

    State Get_Now_State() const { return current_state_; }

    const char *Get_Now_State_Name() const
    {
        return states_[ToIndex(current_state_)].name;
    }

    uint32_t Get_State_Run_Time(State state) const
    {
        if (!IsValid(state))
            return 0;

        const uint8_t index = ToIndex(state);
        return states_[index].total_run_time +
               (state == current_state_ ? current_run_time_[index] : 0U);
    }

    uint32_t Get_State_Enter_Count(State state) const
    {
        return IsValid(state) ? states_[ToIndex(state)].enter_count : 0U;
    }

    void Reset_State_Statistics(State state)
    {
        if (!IsValid(state))
            return;

        const uint8_t index = ToIndex(state);
        states_[index].enter_count = (state == current_state_) ? 1U : 0U;
        states_[index].total_run_time = 0;
        current_run_time_[index] = 0;
    }

protected:
    void TransitTo(State next_state)
    {
        if (!IsValid(next_state) || next_state == current_state_)
            return;

        const uint8_t old_index = ToIndex(current_state_);
        states_[old_index].total_run_time += current_run_time_[old_index];
        current_run_time_[old_index] = 0;

        current_state_ = next_state;
        ++states_[ToIndex(current_state_)].enter_count;
    }

    StateInfo states_[StateCount]{};
    State current_state_{};

private:
    static constexpr uint8_t ToIndex(State state)
    {
        return static_cast<uint8_t>(state);
    }

    static constexpr bool IsValid(State state)
    {
        return ToIndex(state) < StateCount;
    }

    uint32_t current_run_time_[StateCount]{};
};
