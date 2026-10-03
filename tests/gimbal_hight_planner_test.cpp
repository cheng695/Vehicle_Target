#include "../User/Module/Gimbal/GimbalPlanner.hpp"
#include <cassert>
#include <cmath>

static float SettledSpeed(GimbalHightState state, float position, bool valid = true,
                          bool upper = false, bool lower = false)
{
    GimbalHightPlanner planner;
    planner.Init();
    float speed = 0.0f;
    for (int i = 0; i <= 1000; ++i)
        speed = planner.Plan(state, i * 0.001f, upper, lower, position, valid).target_hight_velocity;
    return speed;
}

static void Near(float actual, float expected)
{
    assert(std::fabs(actual - expected) < 0.001f);
}

int main()
{
    using State = GimbalHightState;
    // 上下行分别在本方向走完 70% 后减速，减速区中点为 2 rad/s。
    Near(SettledSpeed(State::MOVING_UP, 0.0f), 3.0f);
    Near(SettledSpeed(State::MOVING_UP, 469.0f), 3.0f);
    Near(SettledSpeed(State::MOVING_UP, 569.5f), 2.0f);
    Near(SettledSpeed(State::MOVING_UP, 670.0f), 1.0f);
    Near(SettledSpeed(State::MOVING_UP, 700.0f), 1.0f);
    Near(SettledSpeed(State::MOVING_DOWN, 670.0f), -3.0f);
    Near(SettledSpeed(State::MOVING_DOWN, 201.0f), -3.0f);
    Near(SettledSpeed(State::MOVING_DOWN, 100.5f), -2.0f);
    Near(SettledSpeed(State::MOVING_DOWN, 0.0f), -1.0f);
    Near(SettledSpeed(State::MOVING_DOWN, -30.0f), -1.0f);
    Near(SettledSpeed(State::RETURNING_HOME, 100.0f), -1.0f);
    Near(SettledSpeed(State::MOVING_UP, 0.0f, false), 1.0f);
    Near(SettledSpeed(State::MOVING_DOWN, 670.0f, false), -1.0f);

    // 触发限位后，方向保护仍优先于位置减速。
    Near(SettledSpeed(State::MOVING_UP, 670.0f, true, true), 0.0f);
    Near(SettledSpeed(State::MOVING_DOWN, 0.0f, true, false, true), 0.0f);
    Near(SettledSpeed(State::MOVING_UP, 0.0f, true, false, true), 3.0f);
    Near(SettledSpeed(State::MOVING_DOWN, 670.0f, true, true), -3.0f);
    Near(SettledSpeed(State::MOVING_UP, 0.0f, true, true, true), 0.0f);

    // 模拟真实位置随目标速度变化，检查减速区内速度单调下降及终点速度。
    GimbalHightPlanner planner;
    float position = 0.0f;
    float previous_speed = 3.0f;
    float speed = 0.0f;
    for (int i = 0; i < 20000 && position < 670.0f; ++i)
    {
        speed = planner.Plan(State::MOVING_UP, i * 0.001f, false, false,
                             position, true).target_hight_velocity;
        if (position > 470.0f)
            assert(speed <= previous_speed + 0.0001f);
        previous_speed = speed;
        position += speed * 0.001f * 180.0f / 3.14159265f;
    }
    assert(position >= 670.0f);
    assert(speed >= 1.0f && speed < 1.01f);
    planner.Init();
    Near(planner.Plan(State::STOP, 30.0f, false, false, position, true).target_hight_velocity, 0.0f);
}
