# Iteration 002: 控制编排外移与去 `transplant` 化工程结构重整

- 日期：2026-06-20
- 输入文档：`doc/architecture.md`、`doc/detail.md`、`doc/decisions.md`
- 输出文档：`doc/tasks/controller_orchestration.md`、`doc/tasks/sim_adapter_extraction.md`、`doc/tasks/algorithm_library_extraction.md`

## 1. 本次迭代目标

本次迭代目标是先完成“结构先行”的文档重构，把当前仍依赖 `transplant/mujoco_win/simulate` 的运行时组织、控制流程和算法引用，整理成一套正式的 ROS2 包化工程结构。

本次迭代的核心结论：

- 本轮先写 `iter-002` 文档，不继续扩展 `iter-001`。
- 先外移控制编排，不在第一步强行重写 PID、LQR、VMC 数学实现。
- 新正式代码根从一开始按 ROS2 包化规划。
- `transplant/` 只作为迁移来源和参考区，不再作为长期运行时归宿。

## 2. 当前闭环与当前问题

当前已完成的最小闭环为：

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

当前问题不在 bridge 是否存在，而在于：

- 控制流程组织仍在 `wheel_leg_hooks.cc` 内。
- PID、LQR、VMC 仍通过 MuJoCo 内部运行时被组织和调用。
- 工程结构仍依赖 `transplant/` 的移植目录形态，不适合作为长期正式代码根。

## 3. 本次迭代范围

本次迭代包含：

- 建立正式工程结构和迁移路线。
- 定义 `controller orchestration` 节点职责。
- 定义 `sim adapter` 职责和边界。
- 定义算法库提取边界。
- 定义从 `wheel_leg_hooks.cc` 迁出的顺序。
- 定义哪些 `transplant` 文件会被逐步替换。

## 4. 本次不做什么

本次迭代不做以下内容：

- 不更换动力学模型。
- 不重新设计 LQR 数学模型。
- 不引入 STM32 通信实现。
- 不引入遥控器输入实现。
- 不在本轮定义完整状态机。
- 不在本轮实现完整 `ros2_control`。
- 不在本轮删除 `transplant/` 原文件，只定义替代路线。

## 5. 涉及模块

本次迭代涉及模块：

- `wheel_leg_msgs`
- `wheel_leg_bridge`
- `wheel_leg_control`
- `wheel_leg_sim`
- `wheel_leg_common`
- `transplant/` 迁移参考区

## 6. 正式工程结构

正式结构固定为：

```text
src/
  wheel_leg_msgs/
  wheel_leg_bridge/
  wheel_leg_control/
  wheel_leg_sim/
  wheel_leg_common/
```

各包职责：

- `wheel_leg_msgs`：维护消息定义。
- `wheel_leg_bridge`：放 ROS2 通信边界和消息转换逻辑，不放控制算法。
- `wheel_leg_control`：放控制编排、状态聚合、模式切换入口、后续 controller node。
- `wheel_leg_sim`：放 MuJoCo 运行时适配、仿真入口、sim-only helper、hook 接入层。
- `wheel_leg_common`：放仿真/实机共享的数学工具、命名表、参数结构、数据结构。

## 7. 控制器外移的第一步

本轮只做“控制编排外移”的定义，不宣称完成整个控制器迁移。

当前 `wheel_leg_hooks.cc` 中的内容，在长期结构中拆成三层：

1. `sim adapter`
   - 读取 MuJoCo 状态
   - 写 actuator
   - 管理仿真周期回调
2. `controller orchestration`
   - 决定每一步控制数据流
   - 负责状态组装和控制流程组织
3. `algorithm library`
   - PID、LQR、VMC 的纯算法或近纯算法实现

本轮目标状态：

- MuJoCo 不再承载控制流程组织的长期职责。
- MuJoCo 只负责仿真周期、状态采样、命令执行。
- ROS2 controller node 成为长期控制流程入口。
- PID、LQR、VMC 初期允许复用现有公式和参数，但调用位置要从 MuJoCo 内部流程迁出去。

## 8. 长期接口约束

- `wheel_leg_bridge` 继续维护 `/joint_states`、`/imu`、`/joint_command`。
- `wheel_leg_control` 后续 controller node 只依赖 ROS2 状态接口，不读 `mjData`、`mjModel`。
- `wheel_leg_sim` 只暴露采样状态、执行命令、仿真 step 边界这类适配接口。
- `wheel_leg_common` 承载：
  - `JointNameMap`
  - `ActuatorMap`
  - `RobotStateSnapshot`
  - `ControlCommand`
  - `ImuSample`
  - `JointStateSample`
- PID、LQR、VMC 接口允许依赖通用数学类型和控制状态结构。
- PID、LQR、VMC 接口禁止依赖 MuJoCo 类型。
- PID、LQR、VMC 接口禁止直接依赖 ROS2 node 类型。

## 9. 迁移顺序

迁移顺序固定为：

1. 从 `wheel_leg_hooks.cc` 中抽出控制流程图和数据结构，形成 `controller orchestration` 文档边界。
2. 提取 MuJoCo 状态读取和 actuator 写入为 `sim adapter` 边界。
3. 提取 PID、LQR、VMC 为独立算法库接口，初期允许实现仍来源于现有 transplant 公式。
4. 新建 ROS2 controller node，逐步接管原 hooks 中的控制调度。

依赖关系：

- 先有正式结构和接口，再做代码迁移。
- 先迁控制编排，再迁算法实现。
- 新动力学模型替换发生在这之后，不与本轮耦合。

## 10. 输出结果

本次迭代输出结果：

- `doc/detail.md` 升级为本轮详细设计文档。
- `doc/architecture.md` 更新正式工程结构和包边界。
- `doc/tasks/progress.md` 切到 `iter-002` 总览。
- `doc/tasks/controller_orchestration.md`
- `doc/tasks/sim_adapter_extraction.md`
- `doc/tasks/algorithm_library_extraction.md`
- `doc/decisions.md` 中新增结构和迁移顺序决策。

## 11. 后续实现验收导向

后续实现验收应至少覆盖：

1. 结构验收：正式包结构能表达 bridge、control、sim、common 的职责边界。
2. 编排验收：controller orchestration 可以在不直接访问 MuJoCo API 的情况下描述输入、处理、输出。
3. 迁移验收：`wheel_leg_hooks.cc` 中控制流程可被逐步替换，而不是一次性重写。
4. 回归验收：`iter-001` 已完成的 `/joint_states`、`/imu`、`/joint_command`、actuator 边界能力不退化。
5. 未来兼容验收：后续替换新动力学模型时，不需要重新设计 bridge 和 sim adapter 的总体结构。

## 12. 风险点

- 当前 `wheel_leg_hooks.cc` 中控制流程、运行时绑定和算法调用可能耦合较深。
- `transplant/tools/*.cc` 中部分工具代码可能同时包含仿真假设和算法实现，需要后续拆分。
- 如果先做代码迁移再补文档边界，容易把 MuJoCo 细节继续带入长期控制层。

## 13. 当前需要确认的问题

- 当前无新增阻塞问题。
- 后续进入代码阶段前，需要继续梳理 `wheel_leg_hooks.cc` 的实际数据流和类型依赖。
