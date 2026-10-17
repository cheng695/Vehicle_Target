#pragma once

#include "core/Alg/FSM/FSMBase.hpp"
#include <cstdint>

enum class Enum_Chassis_States : uint8_t { STOP = 0, REMOTE, PATROL, STATUS_COUNT };

enum class ChassisEvent { None = 0, StopRequested, RemoteRequested, PatrolRequested };

/**
 * @brief 管理底盘停止、遥控和自动巡航状态转换。
 * @details 通过私有继承隐藏通用 FSM 接口，只公开底盘所需的状态操作。
 */
class Chassis_FSM : private FSMBase<Enum_Chassis_States,
                                    static_cast<uint8_t>(Enum_Chassis_States::STATUS_COUNT)>
{
public:
    /** @brief 初始化状态机并进入 STOP 状态。 */
    void Init();

    /** @brief 根据事件执行底盘状态转换。 */
    void Dispatch(ChassisEvent event);

    /** @brief 无条件请求进入 STOP 状态。 */
    void ForceStop();

    /** @brief 获取当前底盘状态。 */
    Enum_Chassis_States GetCurrentState() const;

    /** @brief 累加当前状态的运行周期计数。 */
    void Update();

private:
    /** @brief 执行状态转换。 */
    void TransitTo(Enum_Chassis_States next_state);
};
