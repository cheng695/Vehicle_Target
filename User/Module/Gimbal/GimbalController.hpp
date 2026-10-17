#pragma once

#include "GimbalPlanner.hpp"
#include "GimbalHightStallDetector.hpp"
#include "GimbalTypes.hpp"
#include "GimbalStateMachine.hpp"
#include "GimbalEventDetector.hpp"
#include "GimbalConfig.hpp"
#include "OperatorInput.hpp"
#include "pid.hpp"

class GimbalController
{
public:
    /**
     * @brief 构造云台控制器并初始化 Yaw 与升降机构的 PID 参数。
     */
    GimbalController();

    /**
     * @brief 重置规划器、PID、状态机和升降机构限位位置状态。
     */
    void Init();

    /**
     * @brief 执行一周期云台控制，包括状态机更新、Yaw 和升降机构控制及反馈采集。
     * @param is_online 云台相关设备是否在线。
     * @param input 当前遥控器输入。
     * @param time_s 当前控制时间，单位：秒。
     * @param target 云台目标速度；函数会更新该对象。
     * @param output 发送给电机的控制输出；函数会更新该对象。
     */
    void ControlLoop(bool is_online, const OperatorInput &input, float time_s,
                     GimbalTarget &target, GimbalOutput &output);

private:
    /**
     * @brief 根据 Yaw 状态和遥控输入更新 Yaw 目标速度及电机输出参数。
     * @param state 当前 Yaw 状态。
     * @param input 当前遥控器输入。
     * @param time_s 当前控制时间，单位：秒。
     * @param target 云台目标值；函数会更新 Yaw 目标速度。
     * @param output 云台电机输出；函数会更新 Yaw 电机控制参数。
     */
    void UpdateYaw(GimbalYawState state, const OperatorInput &input, float time_s,
                   GimbalTarget &target, GimbalOutput &output);

    /**
     * @brief 根据限位、位置和状态机状态更新升降机构电机输出。
     * @param target 云台目标值；函数会更新升降机构目标速度。
     * @param output 云台电机输出；函数会更新升降机构电流命令。
     * @param time_s 当前控制时间，单位：秒。
     */
    void UpdateHight(GimbalTarget &target, GimbalOutput &output, float time_s);

    /**
     * @brief 停止升降机构并重置规划器和 PID，同时更新堵转检测计时。
     * @param target 云台目标值；函数会将升降机构目标速度清零。
     * @param output 云台电机输出；函数会将升降机构电流命令清零。
     */
    void StopHight(GimbalTarget &target, GimbalOutput &output);

    /**
     * @brief 查询升降机构是否存在已锁存的堵转故障。
     * @return 存在堵转故障时返回 true，否则返回 false。
     */
    bool HasHightStallFault() const { return hight_stall_detector_.HasFault(); }

    /**
     * @brief 禁用 Yaw 电机并清零 Yaw 目标速度与输出参数。
     * @param target 云台目标值；函数会将 Yaw 目标速度清零。
     * @param output 云台电机输出；函数会将 Yaw 电机控制参数清零。
     */
    void StopYaw(GimbalTarget &target, GimbalOutput &output);

    // 由 GimbalConfig 集中定义的 PID 参数实例。
    ALG::PID::PID yaw_pid_;
    ALG::PID::PID hight_pid_;

    // ==================== 内部控制模块：通常无需在此处调参 ====================
    // Yaw 与升降机构状态机，保存当前运动状态及状态切换信息。
    Gimbal_FSM yaw_fsm_;
    GimbalHight_FSM hight_fsm_;
    GimbalEventDetector event_detector_;

    // 目标速度规划器；具体速度、加速度等规划参数定义在 GimbalPlanner.hpp。
    GimbalYawPlanner yaw_planner_;
    GimbalHightPlanner hight_planner_;

    // 升降机构堵转故障检测器；检测阈值和时间参数定义在 GimbalHightStallDetector 中。
    GimbalHightStallDetector hight_stall_detector_;

    // ==================== 运行状态：由控制逻辑维护，不是调参项 ====================
    // 升降机构位置零点，单位：度；触发上下限位时自动校准，普通 STOP 不清除。
    float hight_zero_angle_deg_ = 0.0f;

    // 当前零点是否有效；限位校准后置为 true。
    bool hight_position_valid_ = false;

    // 上一控制周期的限位状态，用于检测限位开关的新触发沿。
    bool hight_previous_lower_limit_ = false;
    bool hight_previous_upper_limit_ = false;
};

/**
 * @brief 初始化全局云台控制器并清零目标值和电机输出。
 */
void InitGimbalControl();

/**
 * @brief 执行一周期全局云台控制。
 * @param is_online 云台相关设备是否在线。
 * @param input 当前遥控器输入。
 * @param time_s 当前控制时间，单位：秒。
 */
void GimbalControlLoop(bool is_online,
                       const OperatorInput &input,
                       float time_s);
