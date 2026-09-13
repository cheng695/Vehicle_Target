#pragma once

#include <stdint.h>

struct OperatorInput
{
    float move_x = 0.0f;
    float move_y = 0.0f;
    float rotate = 0.0f;

    bool keyboard_mode = false;
    bool spin = false;
    bool follow_enabled = true;

    uint8_t left_switch = 2;
    uint8_t right_switch = 2;
};
