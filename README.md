# Quadcopter-FC · 自主四旋翼飞行控制系统

基于 STM32F4 从零实现的自主四旋翼飞控源码与 MATLAB 仿真项目。第一版已经完成姿态估计、DShot300 输出、串级 PID、安全状态机、激光高度估计等固件模块，并在同一非线性模型上完成串级 PID 与 LQR 的对比仿真实验。

## 项目成果

### 固件源码（`firmware/`）

| 模块 | 文件 | 说明 |
| --- | --- | --- |
| IMU 寄存器驱动 | `fc_icm20948.c/.h` | ICM-20948 芯片校验、量程配置、SI 单位换算，通过 I2C 回调接入 HAL |
| DShot300 编码 | `fc_dshot300.c/.h` | 16 位 DShot 帧、CRC 校验、归一化油门到 DShot 值 |
| 姿态估计 | `fc_attitude_estimator.c/.h` | 互补滤波，输出 roll/pitch/yaw |
| 串级 PID 控制 | `fc_controller.c/.h` | 角度环→角速度环→混控轴指令 |
| 通用 PID | `fc_pid.c/.h` | 积分限幅与抗饱和 |
| 四旋翼混控 | `fc_mixer.c/.h` | Quad-X 分配与输出归一化 |
| 安全状态机 | `fc_safety.c/.h` | 解锁/锁定/缓降/故障保护 |
| 高度估计 | `fc_height.c/.h` | 激光高度滤波与垂直速度估计 |

### MATLAB 仿真（`simulation/matlab/`）

- 非线性 12 状态四旋翼动力学模型；
- 串级 PID 姿态跟踪仿真；
- LQR 姿态控制器设计（无工具箱时自动使用解析解）与仿真；
- 同一指令、同一扰动条件下对比两类控制器输出。

## 仓库结构

```text
quadcopter-fc/
├── docs/                      # 项目文档
│   ├── 00_项目总览.md
│   ├── 01_硬件平台.md
│   ├── 02_软件架构.md
│   ├── 03_算法设计.md
│   ├── 04_开发路线与验收.md
│   └── 05_安全与调参.md
├── firmware/                  # STM32 固件源码
│   ├── app/include/           # 自研模块头文件
│   ├── app/src/               # 自研模块源码
│   └── project/               # CubeMX/Keil 工程生成说明
└── simulation/
    └── matlab/                # 模型与 PID/LQR 仿真
```

## 快速开始

### 运行 MATLAB 仿真

在 MATLAB 中打开 `simulation/matlab`，执行：

```matlab
run_all_simulations
```

会生成串级 PID 与 LQR 的姿态跟踪曲线、控制力矩曲线，用于对比控制器性能。

### 构建固件

1. 使用 STM32CubeMX 按 `firmware/project/README.md` 生成 STM32F405 工程；
2. 把 `firmware/app/src` 和 `firmware/app/include` 加入 Keil；
3. 由工程层实现 I2C、UART、DShot 底层时序，并调用 app 模块；
4. 所有实机步骤按 `docs/04_开发路线与验收.md` 执行，拆桨优先。

## 算法与源码说明

- 姿态：陀螺积分 + 加速度修正的互补滤波，源码见 `fc_attitude_estimator.c`；
- 控制：角度外环 P + 角速度内环 PID，串级结构源码见 `fc_controller.c` 与 `fc_pid.c`；
- 执行：DShot300 帧编码与 Quad-X 混控，源码见 `fc_dshot300.c`、`fc_mixer.c`；
- 仿真：`quadcopter_dynamics.m` 为非线性模型，`pid_cascade_sim.m` 与 `lqr_sim.m` 为两类控制器实验。

完整公式与说明见 `docs/03_算法设计.md`。

## 硬件平台

当前开发与验证平台：STM32F405RGT6 + 板载 ICM-20948（I2C）+ DShot300 电调，向上激光测距用于定高，光流 / 二维雷达用于定点扩展。资源分配见 `docs/01_硬件平台.md`。

## 安全与调参

源码与仿真之外，实机联调必须遵守 `docs/05_安全与调参.md`：拆桨调试、装桨前验收门槛、故障保护状态机、可复现调参记录。这些规则已经落到 `fc_safety.c` 的状态机实现中。

## License

MIT License，署名见 [LICENSE](LICENSE)。
