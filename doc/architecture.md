# 轮腿机器人系统总体架构

## 1. 文档目标

本文档记录轮腿机器人系统的长期分层架构、当前已确认的正式包边界，以及从 `transplant/` 迁往正式 ROS2 工程结构的组织原则。

当前已确认原则：

- ROS2 是统一通信层。
- MuJoCo 是当前主要仿真环境。
- Raspberry Pi 是 ROS2 主控目标平台。
- STM32 是真机实时硬件层。
- 控制器长期不直接依赖 MuJoCo API。
- 控制器长期不直接访问 STM32 串口或 CAN。
- 当前阶段优先延续 Topic 桥接，不直接引入完整 `ros2_control`。
- `transplant/` 只作为迁移来源和参考区，不作为长期运行时归宿。

当前未确认内容在本文末尾集中记录，后续确认后应更新本文档或记录到 `doc/decisions.md`。

## 2. 系统总体架构

长期目标架构如下：

```text
遥控器 / 调试工具 / 上层规划
  ↓
ROS2 输入层
  ↓
状态机 / 控制模式管理
  ↓
控制编排层
  ↓
控制算法层（LQR / MPC / WBC / RL / 轨迹跟踪等）
  ↓
统一机器人接口
  ↓
硬件接口层
  ├── MuJoCo 仿真接口
  └── STM32 真机接口
```

该架构将控制算法、控制流程组织、命令来源和底层硬件实现分离。上层控制逻辑只处理统一状态与统一命令，不关心底层当前来自 MuJoCo、STM32、遥控器还是调试脚本。

## 3. 硬件分层

### 3.1 桌面仿真环境

桌面 Linux 用于运行 MuJoCo 仿真和 ROS2 开发环境。

职责：

- 运行 MuJoCo 模型。
- 通过仿真适配层采样状态并执行命令。
- 支持 ROS2 控制流程在仿真环境中验证。
- 作为控制器迁移和动力学模型替换前的验证环境。

### 3.2 Raspberry Pi 主控

Raspberry Pi 是真机部署时的 ROS2 主控。

职责：

- 运行 ROS2 节点。
- 运行控制编排、状态估计、控制器和模式管理。
- 接收遥控器输入。
- 与 STM32 通信。
- 维护与仿真一致的状态和命令接口。

### 3.3 STM32 实时硬件层

STM32 是真机底层实时控制与安全保护单元。

职责：

- 电机驱动。
- 编码器读取。
- IMU 读取。
- 电流采样。
- 底层限幅和安全保护。
- 通过串口或 CAN 与 Raspberry Pi 通信。

STM32 通信协议、通信频率和安全策略尚未进入当前迭代，不在本文档中展开。

## 4. 正式软件结构

正式工程结构固定为：

```text
src/
  wheel_leg_msgs/
  wheel_leg_bridge/
  wheel_leg_control/
  wheel_leg_sim/
  wheel_leg_common/
```

### 4.1 `wheel_leg_msgs`

职责：

- 维护 ROS2 消息定义。
- 保留当前 `/joint_command` 消息。
- 后续可扩展状态消息、模式消息和调试消息。

### 4.2 `wheel_leg_bridge`

职责：

- 承担 ROS2 通信边界。
- 负责状态消息和命令消息转换。
- 向外维持 `/joint_states`、`/imu`、`/joint_command` 等接口。

约束：

- 不承载控制算法。
- 不承载长期控制流程组织。

### 4.3 `wheel_leg_control`

职责：

- 承担控制编排。
- 承担状态聚合。
- 承担模式切换入口。
- 承担后续 controller node。

约束：

- 长期只依赖 ROS2 状态接口和 `wheel_leg_common` 公共结构。
- 不直接读取 `mjModel`、`mjData`。

### 4.4 `wheel_leg_sim`

职责：

- 承担 MuJoCo 运行时适配。
- 承担仿真入口和 sim-only helper。
- 承担 step hook 接入层。
- 对外暴露采样状态、执行命令和仿真 step 边界接口。

约束：

- 不承载长期控制流程宿主职责。

### 4.5 `wheel_leg_common`

职责：

- 承担仿真和实机共享的数学工具。
- 承担命名表和参数结构。
- 承担共享数据结构和轻量控制类型。

## 5. 迁移期参考区

当前迁移来源主要包括：

```text
transplant/mujoco_win/simulate/wheel_leg/
transplant/mujoco_win/simulate/tools/
```

其中：

- `wheel_leg_hooks.cc` 当前承载 MuJoCo 仿真流程内的控制组织逻辑。
- `ros2_bridge.cc` 当前承载已完成的最小 bridge 实现。
- `tools/*.cc` 当前包含 PID、LQR、VMC、sensor、actuator、数学工具等内容。

迁移原则：

- `transplant/` 在迁移期继续存在。
- 其代码只作为来源和参考。
- 真正会长期保留的能力，后续要按职责拆入正式包。

## 6. 软件分层

```text
应用输入层
  - 遥控器输入节点
  - 调试命令节点
  - 后续上层规划节点

模式管理层
  - 控制模式管理
  - 状态机
  - 安全状态切换

控制编排层
  - state assembly
  - control pipeline orchestration
  - controller node

控制算法层
  - LQR
  - MPC
  - WBC
  - RL policy
  - 轨迹跟踪控制

统一接口层
  - JointStateSample
  - ImuSample
  - RobotStateSnapshot
  - ControlCommand
  - Joint Command Topic

硬件适配层
  - MuJoCo sim adapter
  - 后续 STM32 adapter
```

当前迭代不再只讨论 bridge，而是细化：

- ROS2 bridge 边界
- MuJoCo sim adapter 边界
- controller orchestration 边界
- algorithm library 提取边界

## 7. ROS2 节点划分

### 7.1 当前阶段涉及节点

推荐节点：

| 节点 | 当前定位 | 当前迭代状态 |
| --- | --- | --- |
| MuJoCo bridge 节点 | 连接仿真状态与 ROS2 topic | 已有基础能力，进入长期边界重整 |
| controller node | 长期控制流程入口 | 当前迭代定义职责，不直接实现完整迁移 |
| 调试命令节点 | 手动发布测试命令 | 当前仍可作为验证手段 |

### 7.2 后续节点概览

| 节点 | 后续定位 | 当前处理方式 |
| --- | --- | --- |
| 控制器节点 | 运行控制编排和算法调用 | 当前迭代定义边界 |
| 控制模式节点 | 管理站立、失能、速度控制等模式 | 当前迭代不展开 |
| 遥控器输入节点 | 将接收机输入转换为统一命令 | 当前迭代不展开 |
| STM32 通信节点 | 与真实硬件通信 | 当前迭代不展开 |

## 8. 仿真环境与实机环境接口对应关系

| 统一接口 | 仿真来源或去向 | 实机来源或去向 |
| --- | --- | --- |
| `/joint_states` | MuJoCo 状态采样后发布 | STM32 编码器和电机反馈 |
| `/imu` | MuJoCo IMU 采样后发布 | STM32 IMU 数据 |
| `/joint_command` | 控制侧下发到 sim adapter | 下发到 STM32 电机命令 |
| `RobotStateSnapshot` | 由仿真状态聚合生成 | 由真机状态聚合生成 |
| `ControlCommand` | 写入 MuJoCo actuator | 下发到真实执行器 |

## 9. 当前闭环与目标闭环

当前已完成的最小闭环：

```text
MuJoCo 状态
  ↓
/joint_states, /imu
  ↓
ROS2
  ↓
/joint_command
  ↓
MuJoCo actuator
```

这表示当前 ROS2 侧已经能驱动电机力矩命令通路，但 PID、LQR、VMC 的主要控制编排仍在 MuJoCo 内部流程中，没有完成外移。

当前迭代目标闭环：

```text
MuJoCo sim adapter
  ↓
/joint_states, /imu
  ↓
ROS2 controller orchestration
  ↓
algorithm library
  ↓
/joint_command
  ↓
MuJoCo actuator
```

这里的关键变化不是立刻重写算法，而是先把控制流程入口迁到正式控制侧。

## 10. 控制流

当前迭代固定控制迁移顺序：

1. 抽出 `wheel_leg_hooks.cc` 中的控制流程图和数据结构。
2. 抽出 MuJoCo 状态读取和 actuator 写入边界。
3. 抽出 PID、LQR、VMC 的独立接口。
4. 建立 ROS2 controller node，逐步接管 hooks 中的控制调度。

该顺序意味着：

- MuJoCo 长期不再承载控制流程组织职责。
- 控制器外移先做 orchestration，再做算法实现替换。
- 后续新动力学模型替换建立在这套结构之上。

## 11. 模块边界

### `wheel_leg_bridge`

- 维护 ROS2 话题接口。
- 不承载控制算法。

### `wheel_leg_control`

- 维护控制流程入口。
- 不直接依赖 MuJoCo API。

### `wheel_leg_sim`

- 维护仿真状态采样和 actuator 执行。
- 不承载长期控制流程宿主职责。

### `wheel_leg_common`

- 维护共享数据结构和数学工具。
- 不依赖 ROS2 node 类型或 MuJoCo runtime hook。

## 12. 当前不做什么

当前阶段明确不做：

- 不更换动力学模型。
- 不重新设计 LQR 数学模型。
- 不引入 STM32 通信实现。
- 不引入遥控器输入实现。
- 不定义完整状态机。
- 不实现完整 `ros2_control`。
- 不在本轮删除 `transplant/`。

## 13. 当前未确认事项

- controller node 的最终 topic 组织是否需要 namespace。
- 后续 STM32 接口在 ROS2 层使用 topic、service 还是更底层 transport。
- 新动力学模型替换后，算法参数是否继续沿用当前结构。
