#include "ChassisPlanner.hpp"
#include "ChassisTuning.hpp"

#include "OmniCalculation.hpp"

#include <cmath>

// 正运动学对象由 RobotContext.cpp 创建，Controller 每周期更新其实际速度反馈。
extern Alg::CalculationBase::Omni_FK omni_fk;

ChassisPlanner::ChassisPlanner()
    : translation_ramp_{
          Alg::Utility::SlopePlanning(ChassisConfig::TRANSLATION_RAMP_INCREASE, ChassisConfig::TRANSLATION_RAMP_DECREASE),
          Alg::Utility::SlopePlanning(ChassisConfig::TRANSLATION_RAMP_INCREASE, ChassisConfig::TRANSLATION_RAMP_DECREASE)}
{
}

float ChassisPlanner::ApplyDeadzone(float value)
{
    return std::fabs(value) < ChassisTuning::REMOTE_DEADZONE ? 0.0f : value;
}

void ChassisPlanner::LimitTarget(ChassisTarget &target)
{
    const float translation_speed =
        std::sqrt(target.target_translation_x * target.target_translation_x +
                  target.target_translation_y * target.target_translation_y);
    const float rotation_equivalent_speed =
        std::fabs(target.target_rotation) * ChassisTuning::VEHICLE_TURN_RADIUS_M;
    const float required_wheel_speed = translation_speed + rotation_equivalent_speed;

    if (required_wheel_speed > ChassisTuning::MAX_WHEEL_SPEED_MPS)
    {
        const float scale = ChassisTuning::MAX_WHEEL_SPEED_MPS / required_wheel_speed;
        target.target_translation_x *= scale;
        target.target_translation_y *= scale;
        target.target_rotation *= scale;
    }
}

ChassisTarget ChassisPlanner::PlanRemote(const OperatorInput &input) const
{
    ChassisTarget target;
    target.target_translation_x = ChassisTuning::REMOTE_LINEAR_GAIN * ApplyDeadzone(input.move_x);
    target.target_translation_y = ChassisTuning::REMOTE_LINEAR_GAIN * ApplyDeadzone(input.move_y);
    target.target_rotation = ChassisTuning::REMOTE_ROTATION_GAIN * ApplyDeadzone(input.rotate);

    LimitTarget(target);
    return target;
}

ChassisTarget ChassisPlanner::PlanPatrol()
{
    ChassisTarget target;
    const float relative_position = estimated_position_x_ - patrol_start_position_x_;

    // 锁存停车阶段，避免减速过程中的位置波动提前恢复行驶。
    if (!patrol_waiting_for_stop_ &&
        ((patrol_forward_ && relative_position >= ChassisTuning::PATROL_DISTANCE_M) ||
         (!patrol_forward_ && relative_position <= 0.0f)))
    {
        patrol_waiting_for_stop_ = true;
        patrol_settle_cycles_ = 0;
    }

    if (patrol_waiting_for_stop_)
    {
        const float vx = omni_fk.GetChassisVx();
        const float vy = omni_fk.GetChassisVy();
        const bool stopped =
            vx * vx + vy * vy < ChassisTuning::PATROL_STOP_LINEAR_MPS * ChassisTuning::PATROL_STOP_LINEAR_MPS &&
            std::fabs(omni_fk.GetChassisVw()) < ChassisTuning::PATROL_STOP_ANGULAR_RADPS;
        patrol_settle_cycles_ = stopped ? patrol_settle_cycles_ + 1 : 0;

        if (patrol_settle_cycles_ >= ChassisTuning::PATROL_SETTLE_CYCLES)
        {
            patrol_forward_ = !patrol_forward_;
            patrol_waiting_for_stop_ = false;
            patrol_settle_cycles_ = 0;
        }

        // 零目标仍经过斜坡和速度闭环；下一周期才开始反向加速。
        return target;
    }

    target.target_translation_x = patrol_forward_ ? ChassisTuning::PATROL_SPEED_MPS : -ChassisTuning::PATROL_SPEED_MPS;
    target.target_translation_y = 0.0f;
    target.target_rotation = 0.0f;
    return target;
}

void ChassisPlanner::ApplyTranslationRamp(ChassisTarget &target)
{
    for (auto &ramp : translation_ramp_)
    {
        ramp.SetIncreaseValue(ChassisTuning::TRANSLATION_RAMP_INCREASE);
        ramp.SetDecreaseValue(ChassisTuning::TRANSLATION_RAMP_DECREASE);
    }
    translation_ramp_[0].TIM_Calculate_PeriodElapsedCallback(
        target.target_translation_x, omni_fk.GetChassisVx());
    translation_ramp_[1].TIM_Calculate_PeriodElapsedCallback(
        target.target_translation_y, omni_fk.GetChassisVy());

    target.target_translation_x = translation_ramp_[0].GetOut();
    target.target_translation_y = translation_ramp_[1].GetOut();
}

void ChassisPlanner::Init()
{
    previous_state_ = Enum_Chassis_States::STOP;
    estimated_position_x_ = 0.0f;
    patrol_start_position_x_ = 0.0f;
    patrol_forward_ = true;
    patrol_waiting_for_stop_ = false;
    patrol_settle_cycles_ = 0;

}

ChassisTarget ChassisPlanner::Plan(Enum_Chassis_States state, const OperatorInput &input)
{
    // 轮式里程计：x(k) = x(k-1) + vx(k) × dt。
    estimated_position_x_ += omni_fk.GetChassisVx() * ChassisConfig::CONTROL_PERIOD_S;

    // 每次进入巡航都以当前位置为起点，并先向正方向行驶。
    if (state == Enum_Chassis_States::PATROL &&
        previous_state_ != Enum_Chassis_States::PATROL)
    {
        patrol_start_position_x_ = estimated_position_x_;
        patrol_forward_ = true;
        patrol_waiting_for_stop_ = false;
        patrol_settle_cycles_ = 0;
    }
    previous_state_ = state;

    ChassisTarget target;
    switch (state)
    {
    case Enum_Chassis_States::REMOTE:
        target = PlanRemote(input);
        break;

    case Enum_Chassis_States::PATROL:
        target = PlanPatrol();
        break;

    case Enum_Chassis_States::STOP:
    default:
        break;
    }

    ApplyTranslationRamp(target);
    return target;
}

float ChassisPlanner::GetEstimatedPosition() const
{
    return estimated_position_x_;
}

float ChassisPlanner::GetPatrolTravelDistance() const
{
    return estimated_position_x_ - patrol_start_position_x_;
}

bool ChassisPlanner::IsPatrolMovingForward() const
{
    return patrol_forward_;
}
