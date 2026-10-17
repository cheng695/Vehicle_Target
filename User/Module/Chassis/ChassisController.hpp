#pragma once

#include "OperatorInput.hpp"
#include "ChassisEventDetector.hpp"
#include "ChassisPlanner.hpp"
#include "ChassisStateMachine.hpp"
#include "ChassisConfig.hpp"
#include "OmniCalculation.hpp"
#include "pid.hpp"

struct ChassisTarget;

/**
 * @brief 四轮全向底盘控制器。
 * @details 控制器负责底盘逆运动学、轮速 PID 闭环，以及控制周期调度。
 *          底盘状态和输出通过 RobotContext 中的共享对象与其他模块交互。
 */
class ChassisController
{
public:
    /**
     * @brief 构造底盘控制器并初始化运动学和各车轮 PID。
     */
    ChassisController();

    /**
     * @brief 初始化底盘状态机、规划器和轮速 PID。
     */
    void Init();

    /**
     * @brief 执行一个底盘控制周期。
     * @param is_online 底盘控制依赖设备的在线状态；为 false 时强制进入停止状态。
     * @param input 当前周期的遥控器输入。
     */
    void ControlLoop(bool is_online, const OperatorInput &input);

private:
    // ===== 控制器内部对象：不作为调参项 =====
    Alg::CalculationBase::Omni_IK omni_ik_;                   // 底盘逆运动学对象，将底盘速度目标换算为各轮目标转速。
    ALG::PID::PID wheel_pid_[ChassisConfig::WHEEL_COUNT];     // 各车轮独立的速度 PID；参数来自 ChassisConfig.hpp。
    Chassis_FSM state_machine_;                               // 底盘状态机，管理 STOP、REMOTE 和 PATROL 状态。
    ChassisEventDetector event_detector_;                     // 遥控器输入及设备状态事件检测器。
    ChassisPlanner planner_;                                  // 底盘目标规划器及自动巡航状态。

    /**
     * @brief 根据四个车轮的实际转速更新底盘正运动学反馈。
     */
    void UpdateForwardKinematics();

    /**
     * @brief 清零车轮电流输出并重置各轮 PID 状态。
     */
    void Stop();

    /**
     * @brief 根据底盘目标计算各轮速度，并执行轮速闭环控制。
     * @param target 规划器生成的底盘平移与旋转速度目标。
     */
    void Run(const ChassisTarget &target);
};

/**
 * @brief 底盘控制器的全局实例，供控制任务调用。
 */
extern ChassisController chassis_controller;
