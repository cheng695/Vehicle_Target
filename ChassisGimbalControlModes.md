# 靶车底盘与云台控制模式

## 遥控器拨杆定义

系统使用 DT7 遥控器的左右三档拨杆共同决定底盘和云台的运行模式。

- 左拨杆主要决定整车停止、底盘遥控或底盘自动。
- 右拨杆在当前左拨杆模式下决定云台旋转及 Z 轴升降方式。
- 任意关键设备离线或出现故障时，应忽略拨杆状态并强制进入停止模式。

## 模式组合表

| 左拨杆 | 右拨杆 | 底盘模式 | Yaw 旋转模式 | Z 轴模式 | 整体行为 |
|:---:|:---:|:---:|:---:|:---:|---|
| 下 | 下 | 停止 | 停止 | 停止 | 整车停止 |
| 下 | 中 | 遥控 | 匀速旋转 | 停止 | 遥控底盘，云台匀速旋转 |
| 下 | 上 | 遥控 | 变速旋转 | 停止 | 遥控底盘，云台按变速曲线旋转 |
| 中 | 下 | 遥控 | 停止 | 停止 | 仅遥控底盘，云台不运动 |
| 中 | 中 | 遥控 | 匀速旋转 | 上下往复 | 遥控底盘，云台匀速旋转并上下运动 |
| 中 | 上 | 遥控 | 变速旋转 | 上下往复 | 遥控底盘，云台变速旋转并上下运动 |
| 上 | 下 | 自动 | 停止 | 停止 | 底盘自动往返，云台不运动 |
| 上 | 中 | 自动 | 匀速旋转 | 停止 | 底盘自动往返，云台匀速旋转 |
| 上 | 上 | 自动 | 匀速旋转 | 上下往复 | 底盘自动往返，云台匀速旋转并上下运动 |

> 注意：按照当前控制逻辑，左上右上是“匀速旋转 + 上下运动”，不是变速旋转。

## 建议的状态拆分

### 底盘模式

```cpp
enum class ChassisMode
{
    Stop,
    Remote,
    AutoPatrol
};
```

自动巡航模式下，底盘以 `1 m/s` 在相距 `5 m` 的两个端点之间循环往返。

### Yaw 旋转模式

```cpp
enum class YawMode
{
    Stop,
    ConstantSpeed,
    VariableSpeed
};
```

变速旋转建议使用带基础速度的正弦速度曲线：

```text
yaw_speed = speed_gain × (base_speed + amplitude × sin(omega × time + phase))
```

如果要求始终沿同一方向旋转，应满足：

```text
base_speed > abs(amplitude)
```

### Z 轴模式

```cpp
enum class ZAxisMode
{
    Stop,
    Reciprocating
};
```

上下往复模式由两个限位开关控制：

- 触发上限位后切换为向下运动。
- 触发下限位后切换为向上运动。
- 两个限位同时触发时进入故障停止。
- 限位开关需要进行软件消抖。
- 已触发限位时只禁止继续朝该限位方向运动，允许反向脱离。

## 滚轮速度增益

DT7 滚轮用于调整云台旋转速度增益。设定：

```cpp
constexpr float MIN_SPEED_GAIN = 0.5f;
constexpr float MAX_SPEED_GAIN = 2.0f;
```

将归一化后的滚轮输入 `scroll` 从 `[-1, 1]` 映射到增益范围：

```cpp
float CalculateSpeedGain(float scroll)
{
    scroll = std::clamp(scroll, -1.0f, 1.0f);

    const float ratio = (scroll + 1.0f) * 0.5f;
    return MIN_SPEED_GAIN +
           ratio * (MAX_SPEED_GAIN - MIN_SPEED_GAIN);
}
```

对应关系：

| 滚轮位置 | 归一化输入 | 旋转速度增益 |
|:---:|:---:|:---:|
| 最小端 | `-1.0` | `MIN_SPEED_GAIN` |
| 中位 | `0.0` | `(MIN_SPEED_GAIN + MAX_SPEED_GAIN) / 2` |
| 最大端 | `1.0` | `MAX_SPEED_GAIN` |

最终匀速旋转指令：

```text
yaw_speed = speed_gain × constant_speed
```

最终变速旋转指令：

```text
yaw_speed = speed_gain × (base_speed + amplitude × sin(omega × time + phase))
```

所有输出还应经过 4310 电机允许的最大速度限幅。

## 安全优先级

安全条件的优先级高于所有拨杆组合：

1. 遥控器离线：底盘、Yaw 和 Z 轴全部停止。
2. 对应电机离线：相关运动轴立即停止。
3. 两个 Z 轴限位同时触发：Z 轴进入故障停止。
4. 状态组合无效：默认进入整车停止模式。
