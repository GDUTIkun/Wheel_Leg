# 详细设计文档

## 1. 文档目标

本文档根据 `doc/proposal.md`、`doc/architecture.md` 与各轮 `doc/iterations/iter-XXX.md` 编写，用于细化当前迭代涉及的正式工程结构、控制边界和实现顺序。

当前文档保留已完成的 `iter-002` 控制编排外移设计，同时追加 `iter-003` 的 `rc_ibus_node` 设计，用于描述树莓派侧 `iBUS` 输入链路。

当前迭代聚焦：

- 保持 `iter-001` 已完成的 `/joint_states`、`/imu`、`/joint_command` 和 actuator 边界不退化。
- 定义正式 ROS2 包结构。
- 定义 `sim adapter`、`controller orchestration`、`algorithm library` 三层边界。
- 固定从 `wheel_leg_hooks.cc` 迁出的顺序。
- 明确 `transplant/` 在迁移期只作为参考和过渡依赖，不再作为长期运行时归宿。

当前迭代不直接实现新动力学模型、不重写 LQR 数学模型、不落 STM32 通信实现。

## 2. 当前迭代模块划分

当前迭代按以下正式模块划分：

1. `wheel_leg_msgs`
2. `wheel_leg_bridge`
3. `wheel_leg_control`
4. `wheel_leg_sim`
5. `wheel_leg_common`
6. 迁移期参考区 `transplant/`

模块划分目标不是一次性替换全部旧代码，而是先把长期边界说清，再指导后续代码逐步迁出。

## 3. 正式工程结构

正式工程结构固定为：

```text
src/
  wheel_leg_msgs/
  wheel_leg_bridge/
  wheel_leg_control/
  wheel_leg_sim/
  wheel_leg_common/
```

说明：

- `wheel_leg_msgs` 保留消息定义。
- `wheel_leg_bridge` 只承担 ROS2 通信边界和消息转换。
- `wheel_leg_control` 承担控制编排、状态聚合和后续 controller node。
- `wheel_leg_sim` 承担 MuJoCo 仿真运行时适配。
- `wheel_leg_common` 承担仿真和实机共享的数据结构、命名表、参数结构和数学工具。

`transplant/` 不进入正式结构，只作为迁移来源和参考区。

## 4. Bridge 模块

### 4.1 模块目标

继续维护 ROS2 对外 topic 边界，使上层控制只依赖 ROS2 接口，不依赖 MuJoCo 内部类型。

### 4.2 模块输入

- 来自 `wheel_leg_sim` 的状态采样结果。
- 来自 `wheel_leg_control` 的控制命令。
- ROS2 运行环境。

### 4.3 模块输出

- `/joint_states`
- `/imu`
- `/joint_command`

### 4.4 模块内部职责

该模块长期只负责：

1. ROS2 node 生命周期。
2. 发布标准状态 topic。
3. 订阅控制命令 topic。
4. 在 ROS2 消息与内部通用数据结构之间转换。

该模块不负责：

- 控制流程组织。
- 模式切换。
- PID、LQR、VMC 数学实现。
- 直接访问 `mjModel`、`mjData`。

### 4.5 长期接口边界

- 对外继续维护 `/joint_states`、`/imu`、`/joint_command`。
- 对内只依赖 `wheel_leg_common` 中的通用状态与命令结构。

## 5. Sim Adapter 模块

### 5.1 模块目标

把 MuJoCo 仿真运行时相关逻辑从控制逻辑中拆开，使 MuJoCo 只承担仿真周期、状态采样、命令执行和 hook 接入。

### 5.2 模块输入

- `mjModel`
- `mjData`
- 当前仿真 step 边界
- 来自控制侧的 actuator 命令

### 5.3 模块输出

- `JointStateSample`
- `ImuSample`
- 必要时的 `RobotStateSnapshot` 原始组成部分
- actuator 执行结果

### 5.4 模块内部职责

该模块长期负责：

1. 在 MuJoCo step 边界采样状态。
2. 将 MuJoCo 状态组织为通用结构。
3. 接收通用命令并写入 actuator。
4. 提供过渡期 hook 接入点。

该模块不负责：

- 决定控制流程。
- 承载长期 controller host 职责。
- 承载 ROS2 控制器算法实现。

### 5.5 长期接口边界

`wheel_leg_sim` 只暴露：

- 采样状态
- 执行命令
- 仿真 step 边界

不向上暴露 MuJoCo 类型作为长期控制器接口。

## 6. Controller Orchestration 模块

### 6.1 模块目标

把当前 `wheel_leg_hooks.cc` 中的控制流程组织职责迁到正式 ROS2 控制侧，使 MuJoCo 不再长期承载“控制流程入口”。

### 6.2 模块输入

- `/joint_states`
- `/imu`
- 后续模式或目标输入
- 来自 `wheel_leg_common` 的共享参数和数据结构

### 6.3 模块输出

- `ControlCommand`
- 必要时映射为 `/joint_command`

### 6.4 模块内部职责

该模块长期负责：

1. 汇总关节和 IMU 状态。
2. 组装统一 `RobotStateSnapshot`。
3. 驱动控制 pipeline。
4. 组织模式切换入口。
5. 决定何时调用 PID、LQR、VMC 等算法接口。

长期控制数据流定义为：

```text
/joint_states, /imu
  ↓
state assembly
  ↓
control pipeline
  ↓
ControlCommand
  ↓
/joint_command or actuator command
```

### 6.5 长期接口边界

- 后续 controller node 只依赖 ROS2 状态接口。
- 不直接读取 `mjData`、`mjModel`。
- 可依赖 `wheel_leg_common` 中的通用数据结构和数学类型。

## 7. Algorithm Library 模块

### 7.1 模块目标

将 PID、LQR、VMC 从 MuJoCo 内嵌控制流程中拆出，形成可被仿真和后续实机共用的算法接口。

### 7.2 模块输入

- `RobotStateSnapshot`
- 控制目标
- 参数结构
- 通用数学类型

### 7.3 模块输出

- 中间控制量
- `ControlCommand`

### 7.4 模块内部职责

该模块长期负责：

1. 持有算法实现。
2. 持有算法参数结构。
3. 提供稳定接口给 `wheel_leg_control` 调用。

当前迭代允许：

- 初期公式和参数实现仍来源于 `transplant/mujoco_win/simulate/tools/*.cc`

当前迭代不要求：

- 立即重写全部数学实现
- 立即替换动力学模型

### 7.5 长期接口边界

- 算法接口允许依赖通用数学类型和控制状态结构。
- 禁止依赖 MuJoCo 类型。
- 禁止直接依赖 ROS2 node 类型。

## 8. Common 模块

### 8.1 模块目标

承载仿真和实机共享的通用结构，避免桥接层、控制层和仿真层各自维护一套名称和数据载体。

### 8.2 当前应承载的公共类型

- `JointNameMap`
- `ActuatorMap`
- `RobotStateSnapshot`
- `ControlCommand`
- `ImuSample`
- `JointStateSample`

### 8.3 当前职责说明

该模块可逐步吸收当前散落在 `transplant/mujoco_win/simulate/tools/` 中、具有长期复用价值的：

- 数学工具
- 命名表
- 参数结构
- 轻量数据结构

但不直接承载 MuJoCo runtime hook。

## 9. 迁移期参考区

### 9.1 当前定位

迁移期参考区主要包括：

- `transplant/mujoco_win/simulate/wheel_leg/wheel_leg_hooks.cc`
- `transplant/mujoco_win/simulate/wheel_leg/ros2_bridge.cc`
- `transplant/mujoco_win/simulate/tools/*.cc`

### 9.2 迁移期规则

- `wheel_leg_hooks.cc` 只保留为过渡入口，不作为长期控制宿主。
- `tools/*.cc` 中会长期保留的能力，后续按职责拆进 `wheel_leg_common`、`wheel_leg_control`、`wheel_leg_sim`。
- `transplant/` 继续存在，但不再作为未来正式工程组织基础。

## 10. 迁移顺序

当前迭代固定迁移顺序如下：

1. 从 `wheel_leg_hooks.cc` 中抽出控制流程图和数据结构，形成 `controller orchestration` 边界。
2. 提取 MuJoCo 状态读取和 actuator 写入为 `sim adapter` 边界。
3. 提取 PID、LQR、VMC 为独立算法库接口，初期允许实现仍来源于当前 transplant 公式。
4. 新建 ROS2 controller node，逐步接管原 hooks 中的控制调度。

依赖关系固定为：

- 先有正式结构和接口，再做代码迁移。
- 先迁控制编排，再迁算法实现。
- 新动力学模型替换发生在这之后，不与当前迭代耦合。

## 11. 当前迭代范围

当前迭代包含：

- 建立正式工程结构和迁移路线。
- 定义 `controller orchestration` 节点职责。
- 定义 `sim adapter` 职责和边界。
- 定义算法库提取边界。
- 定义从 `wheel_leg_hooks.cc` 迁出的顺序。
- 定义哪些 `transplant` 文件会被逐步替换。

当前迭代不包含：

- 更换动力学模型。
- 重新设计 LQR 数学模型。
- STM32 通信实现。
- 遥控器输入实现。
- 完整状态机实现。
- 完整 `ros2_control` 实现。
- 立即删除 `transplant/` 原文件。

## 12. 验收导向

后续实现验收应至少覆盖：

1. 结构验收：正式包结构能表达 bridge、control、sim、common 的职责边界。
2. 编排验收：`controller orchestration` 可在不直接访问 MuJoCo API 的情况下描述输入、处理、输出。
3. 迁移验收：`wheel_leg_hooks.cc` 中控制流程可被逐步替换，而不是一次性重写。
4. 回归验收：`iter-001` 已完成的 bridge 和 actuator 边界能力不退化。
5. 未来兼容验收：后续替换新动力学模型时，不需要重新设计 bridge 和 `sim adapter` 总体结构。

## 13. Iter-003：`rc_ibus_node` 详细设计

### 13.1 模块目标

`rc_ibus_node` 的目标是在 Raspberry Pi 上直接读取 `FlySky FS-iA6B` 的 `iBUS` 数据，不依赖地面站，在 ROS2 节点内完成：

1. 串口接收。
2. `iBUS` 帧解包与校验。
3. 原始通道值发布。
4. 通道预处理。
5. 统一控制指令映射。
6. failsafe 与诊断状态发布。

### 13.2 输入条件

- 接收机型号固定为 `FlySky FS-iA6B`。
- 接收协议固定为 `iBUS`。
- 树莓派串口固定为 `/dev/ttyAMA3`。
- 串口参数固定为 `115200 8N1`。
- 当前阶段不实现 `iBUS` telemetry 回传。

### 13.3 模块输出

- `/rc/channels_raw`
- `/rc/status`
- `/cmd_vel`
- `/control_mode`
- `/body_cmd`

### 13.4 内部分层

在进入正式 `iBUS` 解包前，需要先通过“是否有信息接收到”的前置测试确认串口链路联通。只有在树莓派已经确认能收到持续字节流后，才进入帧解析、通道标定和控制映射。

#### `uart_reader`

职责：

- 打开 `/dev/ttyAMA3`。
- 配置 `115200 8N1`。
- 连续读取字节流并交给 `ibus_frame_parser`。

错误处理：

- 串口打开失败时，节点进入遥控不可用状态。
- 串口读超时、断开或异常时，节点切换到 failsafe。

前置验证要求：

- 节点需要提供最小接收联通性测试能力，用于确认 `/dev/ttyAMA3` 上是否持续收到非空字节流。
- 在该测试未通过前，不进入“协议错误”或“通道映射”判断。

#### `ibus_frame_parser`

职责：

- 在字节流中搜索合法 `iBUS` 帧边界。
- 按 32-byte 帧结构解包。
- 按小端序提取通道值。
- 校验 `0xFFFF - 前30字节和`。

当前文档固定的协议事实：

- 帧长为 `32 bytes`。
- 通道值为 little-endian `uint16`。
- 常见原始通道范围按 `1000~2000` 建模，中位按 `1500` 建模。

错误处理：

- 长度异常、帧头不匹配、checksum 错误的帧必须整帧丢弃。
- 坏帧不得进入控制映射链。
- 解析失败计入 `/rc/status` 错误计数。

#### `channel_preprocessor`

职责：

- 根据参数执行每通道中位、最小值、最大值标定。
- 将连续量通道归一化到 `[-1, 1]`。
- 将拨杆或开关通道离散成有限模式值。
- 执行死区处理。
- 执行方向反转。

参数要求：

- 每个连续通道都需要独立的 `min / center / max / reverse / deadzone` 参数。
- 开关通道需要支持三段或两段离散阈值配置。

#### `command_mapper`

职责：

- 将主速度摇杆映射到 `/cmd_vel`。
- 将模式拨杆映射到 `/control_mode`。
- 将剩余摇杆或旋钮映射到 `/body_cmd`。
- 映射确认阶段需要用户现场配合，逐一拨动摇杆和开关，确认物理输入对应的通道编号和方向。

当前固定语义：

- `/cmd_vel`：主速度和转向主命令。
- `/control_mode`：站立、失能、速度控制等模式切换命令。
- `/body_cmd`：机身附加控制目标，当前优先承载高度与转向辅助控制量。

接口约束：

- 控制器只看统一命令，不看 `iBUS` 协议细节。
- `/rc/channels_raw` 只服务调试、标定和排障。

#### `failsafe_guard`

职责：

- 监测最近有效帧时间。
- 监测持续坏帧和串口异常。
- 失联时发布安全控制命令。

当前固定安全策略：

- 接收机断链、串口超时或持续坏帧时，清零运动类命令。
- `/cmd_vel` 清零。
- `/body_cmd` 退回安全零增量或默认姿态目标。
- `/control_mode` 退回安全可站立模式，不直接失能。

### 13.5 ROS2 接口要求

`/rc/channels_raw` 需要至少表达：

- 时间戳。
- 协议名。
- 通道数组。
- 帧有效标志。

`/rc/status` 需要至少表达：

- 串口在线状态。
- 最近有效帧时间。
- checksum 错误计数。
- 丢帧或超时状态。
- failsafe 状态。

`/cmd_vel`、`/control_mode`、`/body_cmd` 的命令来源必须对控制器透明。

### 13.6 仿真环境行为

- 即使 MuJoCo 仿真不依赖真实接收机，`rc_ibus_node` 仍应可作为独立命令源接入仿真控制链。
- 调试时优先观察 `/rc/channels_raw` 和 `/rc/status`，确认通道映射后再联动控制器。

### 13.7 实机环境行为

- `rc_ibus_node` 在 Raspberry Pi 上直接运行。
- 当前不经过 STM32 转发遥控通道。
- 真机与仿真共享 `/cmd_vel`、`/control_mode`、`/body_cmd` 语义。

### 13.8 验证方法

1. 打开 `/dev/ttyAMA3` 并确认节点启动成功。
2. 先执行“是否有信息接收到”的前置测试，确认串口收到持续字节流。
3. 在前置测试通过后，再观察 `/rc/channels_raw` 是否持续收到合法帧。
4. 由用户现场单独拨动每个摇杆或开关，确认通道编号与数值变化。
5. 观察归一化、死区和反向配置是否生效。
6. 观察 `/cmd_vel`、`/control_mode`、`/body_cmd` 是否按预期更新。
7. 拔掉接收机或制造串口超时，确认进入 failsafe。

### 13.9 当前不做什么

- 不实现地面站 UI。
- 不实现 `iBUS` telemetry 回传。
- 不让控制器直接读取原始 UART 数据。
- 不在本轮定义复杂状态机。
