# Vehicle_Target

面向 STM32F427 系列控制板的机器人靶车控制固件，包含四轮全向底盘、Yaw 云台和升降机构控制。项目基于 STM32CubeMX 生成的 HAL 与 FreeRTOS 工程，使用 CMake、Ninja 和 ARM GNU Toolchain 构建。

## 功能概览

- 四轮全向底盘运动学与车轮速度 PID 闭环。
- 遥控底盘控制和自动往返巡航。
- Yaw 停止、匀速旋转和变速旋转控制，支持遥控滚轮调速。
- 升降机构速度规划、PID 和上下限位处理。
- 基于 CAN、UART 的电机及 DT7 遥控器接口。
- 通过 YAML 管理底盘和云台默认参数，构建时生成 C++ 配置头文件。
- 离线网页编辑参数，以及通过 Ozone 在运行时修改调参变量。

> 升降堵转检测参数已纳入配置，但当前检测器的周期更新调用尚未启用。请勿将其视为正在工作的堵转保护。

## 目录结构

| 目录/文件 | 说明 |
|---|---|
| `Core/`、`Drivers/`、`Middlewares/` | STM32 HAL、CMSIS、FreeRTOS 及芯片相关代码。 |
| `User/Task/` | RTOS 任务、遥控输入和通信调度。 |
| `User/Module/` | 底盘、云台、电机控制模块及共享上下文。 |
| `User/core/` | 项目算法、BSP 和 HAL 抽象。 |
| `config/` | 底盘和云台 YAML 默认配置。 |
| `tools/` | YAML 生成器和离线配置网页。 |
| `docs/` | 项目说明、参数配置和调参指南。 |
| `tests/` | 部分控制逻辑的测试源文件。 |
| `build.ps1` | 配置并构建 Debug/Release 固件。 |
| `download.ps1` | 通过 SEGGER J-Link 下载 ELF 固件。 |

## 开发环境

Windows 开发机需要安装并配置：

- CMake 3.22 或更新版本。
- Ninja。
- ARM GNU Toolchain，且 `arm-none-eabi-gcc`、`arm-none-eabi-g++`、`arm-none-eabi-objcopy` 和 `arm-none-eabi-size` 位于 `PATH`。
- Python 3.8 或更新版本及 PyYAML。
- 烧录时还需要 SEGGER J-Link Software、J-Link 调试器和目标板。

安装 Python 依赖（在仓库根目录执行）：

```powershell
python -m pip install -r tools/requirements.txt
```

## 构建

在仓库根目录打开 PowerShell：

```powershell
./build.ps1
```

默认构建 Debug。构建 Release 或清理后构建：

```powershell
./build.ps1 -Config Release
./build.ps1 -Config Debug -Clean
```

构建输出位于 `build/Debug/` 或 `build/Release/`，包括 `Vehicle_Target.elf`、`Vehicle_Target.hex` 和 `Vehicle_Target.bin`。换电脑后从仓库源码重新配置构建，不要复用另一台电脑的 `build/` 目录。

## 烧录

连接目标板与 J-Link 后，在仓库根目录运行：

```powershell
./download.ps1
```

默认下载 Debug 固件，设备型号为 `STM32F427II`，使用 SWD 接口，速度为 4000 kHz。可传入构建配置、芯片型号、接口和速度：

```powershell
./download.ps1 -Config Release -Device STM32F427II -Interface SWD -Speed 4000
```

请根据实际板卡核对芯片型号和调试接口。`JLink.exe` 需要在 `PATH` 或 SEGGER 的常见安装目录中。

## 修改控制参数

底盘和云台参数分别维护在：

- `config/chassis.yaml`
- `config/gimbal.yaml`

CMake 会在构建时调用 `tools/` 中的生成脚本，更新 `User/Module/Chassis/ChassisConfig.hpp` 和 `User/Module/Gimbal/GimbalConfig.hpp`。请编辑 YAML 源文件，不要直接修改生成头文件。参数修改后重新构建并烧录，才会成为固件默认值。

### 离线网页编辑器

```powershell
python tools/chassis_config_editor.py
```

启动后在浏览器访问 <http://127.0.0.1:8765>。网页提供底盘配置、云台配置和挡位说明，可保存 YAML 到项目或导出 YAML。编辑器仅在本机修改配置文件，不会连接机器人或直接改变运行中的固件。

### Ozone 实时调参

底盘和云台运行时变量分别位于 `ChassisTuning`、`GimbalTuning` 命名空间。使用 Debug ELF 在 Ozone Watch 窗口中查看和修改，例如：

```text
ChassisTuning::WHEEL_PID_KP[0]
ChassisTuning::REMOTE_LINEAR_GAIN
GimbalTuning::HIGHT_PID_KP
GimbalTuning::HIGHT_GRAVITY_FEEDFORWARD
```

Ozone 修改仅在当前运行期间有效；复位或断电后恢复为 YAML 配置生成的默认值。确认参数后请将其写回 YAML，再构建固件。

## 遥控器挡位

DT7 左右三档拨杆组合决定底盘、Yaw 和升降状态。九种组合的行为表及滚轮调速说明见[底盘与云台控制模式说明](ChassisGimbalControlModes.md)。

## 文档

- [项目结构、构建与烧录说明](docs/项目说明.md)
- [配置、离线编辑器与 Ozone 调参](docs/配置与调参.md)
- [底盘与云台挡位说明](ChassisGimbalControlModes.md)

