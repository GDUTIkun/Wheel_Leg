# 控制编排外移任务

## 1. 任务目标

定义正式 `controller orchestration` 边界，把当前仍内嵌在 `transplant/mujoco_win/simulate/wheel_leg/wheel_leg_hooks.cc` 中的控制流程组织职责，迁移为长期由 `wheel_leg_control` 承担的能力。

当前任务重点不是立即重写 PID、LQR、VMC，而是先明确：

- 控制输入来自哪里。
- 状态如何组装。
- 算法何时被调用。
- 命令如何下发。

## 2. 当前输入

- `/joint_states`
- `/imu`
- 后续模式输入或目标输入
- `wheel_leg_common` 中的共享参数和数据结构

## 3. 当前输出

- `RobotStateSnapshot`
- 控制 pipeline 的中间状态
- `ControlCommand`
- 对 bridge 层可下发的 `/joint_command`

## 4. 当前边界要求

- 控制编排长期由 `wheel_leg_control` 承担。
- controller node 只依赖 ROS2 状态接口。
- 不直接读取 `mjModel`、`mjData`。
- 不直接依赖 MuJoCo hook 生命周期。
- 允许调用 PID、LQR、VMC 算法接口。
- 不把算法细节混入 bridge 和 sim adapter。

## 5. 需要完成的文档化事项

1. 明确输入状态集合和更新节奏。
2. 明确 `state assembly` 的职责。
3. 明确控制 pipeline 的组织顺序。
4. 明确 controller node 与 bridge、sim adapter 的接口边界。
5. 明确从 `wheel_leg_hooks.cc` 迁出时，哪些内容先保留为过渡逻辑。

## 6. 推荐迁移顺序

1. 先抽出 `wheel_leg_hooks.cc` 中的控制流程图。
2. 再抽出控制过程中使用的共享数据结构。
3. 再为 PID、LQR、VMC 定义被编排层调用的接口。
4. 最后由 controller node 接管调度。

## 7. 完成标准

满足以下条件即可认为该任务文档完成：

- 能清楚描述控制数据流：

```text
/joint_states, /imu
  ↓
state assembly
  ↓
control pipeline
  ↓
ControlCommand
  ↓
/joint_command
```

- 能明确 `wheel_leg_control` 和 `wheel_leg_sim` 的职责分界。
- 能指导后续实现者在不直接触碰 MuJoCo API 的情况下设计 controller node。

## 8. 非目标

- 不在本任务中重写 PID、LQR、VMC 数学实现。
- 不在本任务中定义新动力学模型。
- 不在本任务中定义完整状态机。

## 9. 当前状态

- `[ ] 未开始`
