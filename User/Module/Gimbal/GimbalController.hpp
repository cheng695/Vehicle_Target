#pragma once

#include "GimbalPlanner.hpp"
#include "GimbalHightStallDetector.hpp"
#include "GimbalTypes.hpp"
#include "OperatorInput.hpp"
#include "pid.hpp"

class GimbalController
{
public:
    GimbalController();

    void Init();

    void UpdateYaw(GimbalYawState state, const OperatorInput &input, float time_s, GimbalTarget &target, GimbalOutput &output);

    void UpdateHight(GimbalTarget &target, GimbalOutput &output, float time_s);

    void StopHight(GimbalTarget &target, GimbalOutput &output);
    bool HasHightStallFault() const { return hight_stall_detector_.HasFault(); }

private:
    GimbalYawPlanner yaw_planner_;
    GimbalHightPlanner hight_planner_;
    GimbalHightStallDetector hight_stall_detector_;
    // 位置基准独立于速度斜坡，普通 STOP 不清除。
    float hight_zero_angle_deg_ = 0.0f;
    bool hight_position_valid_ = false;
    bool hight_previous_lower_limit_ = false;
    bool hight_previous_upper_limit_ = false;
};

void InitGimbalControl();

void GimbalControlLoop(bool is_online,
                       const OperatorInput &input,
                       float time_s);
