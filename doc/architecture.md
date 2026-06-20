# 轮腿机器人系统总体架构

## 1. 文档目标

本文档记录轮腿机器人系统的长期分层架构和当前已确认的模块边界。本文档不代表一次性完成全部功能，而是作为后续迭代设计的上层约束。

当前已确认原则：

- ROS2 是统一通信层。
- MuJoCo 是主要仿真环境。
- Raspberry Pi 是 ROS2 主控目标平台。
- STM32 是真机实时硬件层。
- 控制器不直接依赖 MuJoCo API。
- 控制器不直接访问 STM32 串口或 CAN。
- 第一阶段优先使用 Topic 桥接，不直接引入完整 `ros2_control`。

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
控制器（LQR / MPC / WBC / RL / 轨迹跟踪等）
  ↓
统一机器人接口
  ↓
硬件接口层
  ├── MuJoCo 仿真接口
  └── STM32 真机接口
```

该架构将控制算法、命令来源和底层硬件实现分离。上层控制器只处理统一状态与统一命令，不关心数据来自 MuJoCo、STM32、遥控器还是调试脚本。

## 3. 硬件分层

### 3.1 桌面仿真环境

桌面 Linux 用于运行 MuJoCo 仿真和 ROS2 开发环境。

职责：

- 运行 MuJoCo 模型。
- 发布仿真状态到 ROS2。
- 接收 ROS2 命令并转换为 MuJoCo actuator 控制量。
- 支持控制算法在仿真环境中验证。

### 3.2 Raspberry Pi 主控

Raspberry Pi 是真机部署时的 ROS2 主控。

职责：

- 运行 ROS2 节点。
- 运行状态估计、控制器和模式管理。
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

## 4. 软件分层

```text
应用输入层
  - 遥控器输入节点
  - 调试命令节点
  - 后续上层规划节点

模式管理层
  - 控制模式管理
  - 状态机
  - 安全状态切换

控制算法层
  - LQR
  - MPC
  - WBC
  - RL policy
  - 轨迹跟踪控制

统一接口层
  - Joint State
  - IMU
  - Robot State
  - Joint Command
  - Body Command

硬件适配层
  - MuJoCo ROS2 Topic 桥接
  - 后续 MuJoCoHardware
  - 后续 STM32Hardware
```

当前迭代只涉及硬件适配层中的 MuJoCo ROS2 Topic 桥接，以及它暴露给 ROS2 的状态和命令接口。

## 5. ROS2 节点划分

### 5.1 当前迭代涉及节点

推荐节点：

| 节点 | 当前定位 | 当前迭代状态 |
| --- | --- | --- |
| MuJoCo ROS2 桥接节点 | 连接 MuJoCo 与 ROS2 Topic | 当前迭代细化 |
| 调试命令节点 | 手动发布测试命令 | 当前迭代只作为验证手段 |

MuJoCo ROS2 桥接节点的职责：

- 从 MuJoCo 读取关节状态。
- 从 MuJoCo 读取 IMU 状态。
- 发布 `/joint_states`。
- 发布 `/imu`。
- 订阅 `/joint_command`。
- 将 `/joint_command` 转换为 MuJoCo actuator 控制量。

### 5.2 后续节点概览

| 节点 | 后续定位 | 当前处理方式 |
| --- | --- | --- |
| 控制器节点 | 运行 LQR、MPC 等控制算法 | 本迭代不展开 |
| 控制模式节点 | 管理站立、失能、速度控制等模式 | 本迭代不展开 |
| 遥控器输入节点 | 将接收机输入转换为统一命令 | 本迭代不展开 |
| STM32 通信节点 | 与真实硬件通信 | 本迭代不展开 |

## 6. STM32 与 Raspberry Pi 任务划分

| 功能 | Raspberry Pi | STM32 |
| --- | --- | --- |
| ROS2 通信 | 负责 | 不负责 |
| 控制算法 | 负责 | 不负责或只做底层保护 |
| 状态估计 | 负责 | 可提供原始数据 |
| 电机驱动 | 不直接驱动 | 负责 |
| 编码器读取 | 接收处理后数据 | 负责 |
| IMU 读取 | 接收处理后数据 | 负责 |
| 电流采样 | 接收状态 | 负责 |
| 底层安全保护 | 下发安全模式和处理状态 | 负责实时保护 |

当前迭代不实现 STM32 通信，只保留接口一致性的架构约束。

## 7. 仿真环境与实机环境接口对应关系

| 统一接口 | 仿真来源或去向 | 实机来源或去向 |
| --- | --- | --- |
| `/joint_states` | MuJoCo joint/sensor 数据 | STM32 编码器和电机反馈 |
| `/imu` | MuJoCo IMU sensor 数据 | STM32 IMU 数据 |
| `/joint_command` | 写入 MuJoCo actuator | 下发到 STM32 电机命令 |
| `/cmd_vel` | 控制器目标输入 | 控制器目标输入 |
| `/control_mode` | 控制器模式输入 | 控制器模式输入 |
| `/robot_state` | 后续由仿真状态估计生成 | 后续由真机状态估计生成 |

当前迭代只细化 `/joint_states`、`/imu` 和 `/joint_command`。

## 8. 数据流

当前迭代数据流：

```text
MuJoCo mjModel / mjData
  ↓
状态读取与转换
  ↓
/joint_states, /imu
  ↓
ROS2 调试工具或后续控制器
```

后续完整数据流：

```text
MuJoCo 或 STM32
  ↓
统一状态接口
  ↓
状态估计 / 控制器
  ↓
统一命令接口
  ↓
MuJoCo 或 STM32
```

## 9. 控制流

当前迭代控制流：

```text
ROS2 调试命令
  ↓
/joint_command
  ↓
MuJoCo ROS2 桥接节点
  ↓
actuator 映射
  ↓
MuJoCo d->ctrl
```

当前迭代只验证命令通路存在，不要求完成 LQR、VMC、PID 等控制器外移。

## 10. 模块边界

### 10.1 MuJoCo ROS2 桥接模块

允许：

- 读取 MuJoCo 模型与仿真数据。
- 读取 MuJoCo sensor 数据。
- 写入 MuJoCo actuator 控制量。
- 发布和订阅 ROS2 Topic。

禁止：

- 承担长期控制算法职责。
- 把控制器实现写入渲染 UI 模块。
- 将 MuJoCo 类型泄漏到控制器节点接口。

### 10.2 控制器模块

允许：

- 订阅统一状态和命令。
- 输出统一控制命令。

禁止：

- 直接读取 `mjData` 或 `mjModel`。
- 直接访问 STM32 串口或 CAN。
- 直接依赖遥控器协议。

### 10.3 STM32 硬件模块

当前只保留边界，不进入实现。

## 11. 当前不确定设计

以下内容尚未确认，不能在当前迭代中固定为实现约束：

- `/robot_state` 的具体 message 类型和字段。
- `/body_cmd` 的具体 message 类型和字段。
- `/joint_command` 的 ROS2 消息文件尚未创建。
- MuJoCo actuator 方向、限幅与统一关节命令的安全策略。
- 真机关节正方向、零位和限幅。
- STM32 通信协议和通信频率。
- 遥控器接收机协议。
- 后续是否以及何时引入完整 `ros2_control`。

## 12. 后续可能调整的部分

- 当前 Topic 桥接后续可能演进为 `MuJoCoHardware`。
- `/control_mode` 当前可使用简单字符串语义，后续可能改为枚举型自定义消息。
- `/joint_command` 当前使用 `wheel_leg_msgs/msg/JointCommand`，后续可根据控制模式扩展或替换字段。
- 当前控制器逐步外移策略可能根据仿真稳定性分多轮完成。
