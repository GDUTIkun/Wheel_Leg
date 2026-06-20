# 当前任务进度

## 1. 当前迭代

- 迭代编号：`iter-002`
- 迭代名称：控制编排外移与去 `transplant` 化工程重整
- 迭代文档：`doc/iterations/iter-002.md`
- 详细设计：`doc/detail.md`
- 上一轮验证记录：`doc/validation.md`

## 2. 状态说明

任务状态使用：

```text
[ ] 未开始
[~] 进行中
[x] 代码已完成
[!] 阻塞或待验证
[?] 待确认
[v] 已通过验证
```

注意：

- `[x] 代码已完成` 不等于 `[v] 已通过验证`。
- 本轮当前是文档重构迭代，允许任务先停留在设计完成状态，不要求同步有代码验证。
- `iter-001` 已完成的 bridge 能力视为当前迭代输入，不在本轮重复验收实现细节。

## 3. 迭代目标概览

`iter-002` 关注：

- 建立正式 ROS2 包化工程结构。
- 明确 `wheel_leg_bridge`、`wheel_leg_control`、`wheel_leg_sim`、`wheel_leg_common` 的职责边界。
- 定义控制编排外移的第一步。
- 定义 `sim adapter` 提取边界。
- 定义 PID、LQR、VMC 算法库提取边界。
- 明确 `transplant/` 的迁移期定位和替代顺序。

`iter-002` 不关注：

- 新动力学模型替换。
- STM32 通信实现。
- 遥控器输入实现。
- 完整状态机落地。
- 完整 `ros2_control` 落地。
- 立即删除 `transplant/`。

## 4. 模块任务总览

| 模块任务 | 文件 | 当前状态 | 说明 |
| --- | --- | --- | --- |
| 控制编排外移任务 | `doc/tasks/controller_orchestration.md` | `[ ] 未开始` | 定义 controller orchestration 的输入、处理、输出和迁移顺序 |
| MuJoCo sim adapter 提取任务 | `doc/tasks/sim_adapter_extraction.md` | `[ ] 未开始` | 定义状态采样、命令执行和 step 边界从 hooks 中拆出的方式 |
| 算法库提取任务 | `doc/tasks/algorithm_library_extraction.md` | `[ ] 未开始` | 定义 PID、LQR、VMC 从 transplant 中抽为正式接口的边界 |
| 正式工程结构与包边界 | `doc/architecture.md` | `[x] 已完成` | 已更新正式包结构、迁移期定位和长期职责边界 |
| 当前迭代详细设计 | `doc/detail.md` | `[x] 已完成` | 已从 bridge-only 升级为 bridge + sim + control + common 视角 |
| 技术决策记录 | `doc/decisions.md` | `[x] 已完成` | 已追加正式结构和迁移顺序决策 |

## 5. 推荐执行顺序

1. `[x]` 固定 `iter-002` 文档目标和非目标。
2. `[x]` 固定正式 ROS2 包结构。
3. `[x]` 固定 `controller orchestration` 的职责和数据流。
4. `[x]` 固定 `sim adapter` 的职责和暴露接口。
5. `[x]` 固定 PID、LQR、VMC 算法库提取边界。
6. `[x]` 固定从 `wheel_leg_hooks.cc` 迁出的顺序和依赖关系。
7. `[ ]` 后续再进入代码迁移任务拆分。

## 6. 当前输入与已完成基础

作为本轮输入，以下内容已存在：

- `iter-001` 已建立 `/joint_states`、`/imu`、`/joint_command` 和 actuator 边界。
- `wheel_leg_msgs/msg/JointCommand.msg` 已存在。
- 当前运行时仍大量依赖 `transplant/mujoco_win/simulate`。
- `wheel_leg_hooks.cc` 仍承载控制流程组织。

## 7. 当前阻塞或待确认问题

- 当前无新的架构方向阻塞项。
- 后续代码阶段需要进一步梳理 `wheel_leg_hooks.cc` 中哪些结构先迁入 `wheel_leg_common`，哪些先保留在 `wheel_leg_sim`。
