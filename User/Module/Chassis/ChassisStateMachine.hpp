#pragma once
#include <stdint.h>

enum Enum_Chassis_States { 
    STOP = 0,   // 停止
    REMOTE,     // 遥控器控制
    PATROL,     // 自动
    STATUS_COUNT };
    
enum class ChassisEvent { None = 0, StopRequested, RemoteRequested, PatrolRequested };

struct Struct_Status
{
    const char *Name;
    uint32_t Enter_Count;
    uint32_t Total_Run_Time;
    void *User_Data;
};

class Chassis_FSM
{
public:
    Struct_Status Status[STATUS_COUNT];
    Enum_Chassis_States State_chassis = STOP;

    void Init();
    void Dispatch(ChassisEvent event);
    void ForceStop();
    void TIM_Update();
    Enum_Chassis_States Get_Now_State() const { return State_chassis; }
    const char *Get_Now_State_Name() const { return Status[State_chassis].Name; }
    uint32_t Get_State_Run_Time(Enum_Chassis_States state) const;
    uint32_t Get_State_Enter_Count(Enum_Chassis_States state) const;
    void Reset_State_Statistics(Enum_Chassis_States state);

private:
    uint32_t State_Run_Time[STATUS_COUNT] = {0};
    void TransitTo(Enum_Chassis_States next_state);
};
