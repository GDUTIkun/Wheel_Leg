# 轮腿机器人工程结构整改架构文档

## 1. 文档目标

本文档用于固定 STM32 接入前的正式工程结构，约束仓库目录、ROS2 包边界、构建入口和仿真/实机切换方式。

本文档服务于两个直接目标：

- 多设备同步同一仓库进度，但不同设备只构建自己需要的部分。
- 在不重写控制主链的前提下，为后续仿真/实机顺滑切换和 `ros2_control` 接入预留结构。

## 2. 总体原则

- 采用单仓库组织，不拆分为多个仓库。
- ROS2 代码统一收口到 `ros2_ws/src/`。
- 固件、仿真资源、脚本、文档与 ROS2 包分层存放。
- `transplant/` 仅作为迁移参考区，不承载新增正式能力。
- 当前阶段继续保持 Topic 闭环主线，不在本轮直接重构为完整 `ros2_control`。

## 3. 目标目录结构

```text
wheel_leg_project/
├── ros2_ws/
│   ├── src/
│   │   ├── wheel_leg_msgs/
│   │   ├── wheel_leg_control/
│   │   ├── wheel_leg_bringup/
│   │   ├── wheel_leg_hw/
│   │   ├── wheel_leg_stm32_bridge/
│   │   ├── wheel_leg_sim/
│   │   ├── wheel_leg_rc/
│   │   ├── wheel_leg_common/
│   │   └── wheel_leg_bridge/
│   ├── build/
│   ├── install/
│   └── log/
├── firmware/
│   └── stm32/
├── sim/
│   └── mujoco/
│       ├── runtime/
│       ├── models/
│       ├── scenes/
│       ├── assets/
│       └── README.md
├── scripts/
│   ├── build.sh
│   ├── run.sh
│   ├── flash_stm32.sh
│   └── env.sh
├── docs/
│   ├── README.md
│   ├── architecture.md
│   ├── protocol.md
│   ├── runbook.md
│   ├── progress.md
│   ├── stm32_hardware_integration.md
│   └── 100hz_balance_notes.md
├── transplant/
└── README.md
```

## 4. 各层职责

### 4.1 `ros2_ws/`

ROS2 正式开发工作区。

约束：

- 所有正式 ROS2 包只放在 `ros2_ws/src/`。
- `build/ install/ log/` 只存在于 `ros2_ws/` 内。
- 不在仓库根目录直接堆放新的 ROS2 包。

### 4.2 `firmware/stm32/`

STM32 固件区。

职责：

- 存放 STM32 工程、驱动、中断、协议处理和板级配置。
- 作为树莓派侧通信协议的对端实现。

约束：

- 不放 ROS2 包。
- 不放 MuJoCo 仿真逻辑。

### 4.3 `sim/mujoco/`

MuJoCo 模型和场景资源区。

职责：

- `runtime/`：MuJoCo 可执行仿真程序源码与构建入口。
- `models/`：机器人主体模型定义。
- `scenes/`：场景入口文件。
- `assets/`：网格、材质和贴图等资源。

约束：

- 仿真资源和 ROS2 包解耦。
- 不把模型文件继续埋在 `transplant/` 里作为正式入口。
- 不把仿真程序入口继续留在 `transplant/` 里。

### 4.4 `scripts/`

统一开发入口。

职责：

- 统一 `build` 和 `run` 的用户操作。
- 按设备和场景选择性构建。
- 固化环境变量和常用命令，降低协作摩擦。

### 4.5 `docs/`

正式文档区。

职责：

- 维护结构、协议、运行入口、当前进度和当前阶段任务文档。
- 保留对后续硬件接入仍有直接价值的调参记录。
- 不再维护旧的 `docs/doc/` 平行规划体系。

### 4.6 `transplant/`

迁移参考区。

职责：

- 保留旧 MuJoCo 工程、算法参考、早期实验材料。

约束：

- 不继续往这里写正式新代码。
- 默认不作为正式构建入口。

## 5. ROS2 包职责

### 5.1 `wheel_leg_msgs`

- 维护自定义消息。
- 统一 `/joint_command`、`/robot_state`、遥控相关消息定义。

### 5.2 `wheel_leg_control`

- 维护控制编排、状态处理、控制算法调用。
- 长期不直接依赖 MuJoCo API、串口实现、STM32 协议细节。

### 5.3 `wheel_leg_bringup`

- 维护 launch、参数装配、仿真/实机入口切换。
- 负责把同一套控制主链接到不同 backend。

### 5.4 `wheel_leg_hw`

- 维护统一硬件抽象。
- 固定 joint、imu、actuator 的命名、单位和语义。
- 作为后续 `ros2_control` 接口收口点。

### 5.5 `wheel_leg_stm32_bridge`

- 维护树莓派与 STM32 的通信节点。
- 负责协议编解码、状态上送、控制命令下发。

### 5.6 `wheel_leg_sim`

- 维护 MuJoCo 运行时适配。
- 负责仿真状态采样、命令写入和 sim-only helper。

### 5.7 `wheel_leg_rc`

- 维护遥控器输入接入与统一命令发布。

### 5.8 `wheel_leg_common`

- 维护共享类型、数学工具、命名映射和轻量公共结构。

### 5.9 `wheel_leg_bridge`

- 保留现有消息转换和通信边界胶水代码。
- 在过渡期继续服务现有主链。

## 6. 控制主链与切换原则

当前阶段统一主链固定为：

```text
state source -> /robot_state -> wheel_leg_controller -> /joint_command -> actuator sink
```

### 6.1 仿真模式

- `wheel_leg_sim` 发布 `/robot_state`
- `wheel_leg_controller_node` 订阅 `/robot_state`
- `wheel_leg_controller_node` 发布 `/joint_command`
- `wheel_leg_sim` 消费 `/joint_command`

### 6.2 实机模式

- `wheel_leg_stm32_bridge` 发布 `/robot_state`
- `wheel_leg_controller_node` 订阅 `/robot_state`
- `wheel_leg_controller_node` 发布 `/joint_command`
- `wheel_leg_stm32_bridge` 消费 `/joint_command`

切换时不更换控制器，只更换状态源和命令去向。

## 7. 与 `ros2_control` 的关系

本轮不直接迁到完整 `ros2_control`，但结构必须为后续接入预留。

后续目标：

- MuJoCo 对接 `WheelLegMujocoSystem`
- STM32 对接 `WheelLegSTM32System`
- 控制器视情况迁成 controller plugin，或保留外部节点过渡

本轮必须先固定：

- joint 命名
- imu 命名
- actuator 命名
- 单位
- 方向
- 零位约定

## 8. 当前状态说明

当前仓库已经完成的结构性动作：

- 原 `src/` 已迁入 `ros2_ws/src/`
- 原 `build/ install/ log/` 已迁入 `ros2_ws/`
- 旧 `docs/doc/` 拆分文档已清理，当前只保留 `docs/` 根目录主线文档

后续重构应基于本文件继续推进，不再回到“根目录直接堆 ROS2 工作区产物”的方式。
