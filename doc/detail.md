# Iter-002 控制编排外移详细设计

## 1. 文档目标

本文档根据 `doc/proposal.md`、`doc/architecture.md`、`doc/iterations/iter-001.md` 和 `doc/iterations/iter-002.md` 编写，用于细化当前迭代涉及的正式工程结构、控制编排边界和迁移顺序。

当前迭代目标不是继续扩展 bridge 单点能力，而是把当前仍寄宿在 `transplant/mujoco_win/simulate` 内部的运行时组织方式，整理成一套可长期演进的 ROS2 包化结构。

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
