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

- `[v] 已通过验证`

## 10. 当前已落地进展

- `wheel_leg_hooks.cc` 中的主控制链已不再直接组织完整输出细节，核心流程已迁入正式 `wheel_leg_control/stand_control_pipeline.*`。
- 当前过渡编排层已经承载：
  - 腿长 PID 输出汇总
  - LQR 状态向量与目标向量组装
  - 转向 PID 与防劈叉 PID 结果汇总
  - VMC 输入组织
  - `ControlCommand` 输出组装
- 当前过渡编排层已改为通过正式算法接口组织调用：
  - `PidAlgorithm`
  - `LqrAlgorithm`
  - `VmcAlgorithm`
- `wheel_leg_hooks.cc` 当前通过正式 `wheel_leg_control/stand_control_runtime.*` 持有运行时目标、算法适配器和控制流水线入口；hook 侧只保留 legacy 回调绑定与生命周期接入。
- 默认站立目标和 legacy PID 配置也已前移到正式头 `wheel_leg_control/stand_runtime_defaults.hpp`，过渡 hook 仅保留对 legacy 初始化接口的翻译。
- `RobotSensorData -> StandControlState` 的字段映射已前移到正式 `wheel_leg_sim/control_state_bridge.*`，hook 侧不再手写控制状态组装细节。
- legacy PID 实例、初始化配置翻译、算法回调绑定和每步控制执行已进一步收敛到 `transplant/wheel_leg/legacy_stand_control_bridge.*`，`wheel_leg_hooks.cc` 只负责 MuJoCo 生命周期和仿真读写入口。
- ROS 接管分支、takeover 切换日志和 ROS actuator 写入调用也已进一步从 `wheel_leg_hooks.cc` 收进 `ros2_bridge.*`；legacy 控制执行与绘图所需最小可视状态则收进 `legacy_stand_control_bridge.*`，hook 侧现已更接近纯生命周期入口。
- 在继续收口后，legacy 绘图链也已删除；hook 侧不再承担 plotting 或本地状态打印职责，后续调试统一依赖 ROS topic、trace 和外部分析工具。
- `wheel_leg_control` 侧已经补上 `legacy_algorithms.*`，并新增正式 `/robot_state` 接口与 `state_message_conversions.*`；controller node 现已可直接订阅 `StandControlState` 并发布控制命令，`ros_control_state_estimator.*` 已退出主执行路径。
- `wheel_leg_hooks.cc` 当前更接近：

```text
MuJoCo step hook
  ↓
状态采样
  ↓
wheel_leg_control::stand_control_pipeline
  ↓
sim adapter / ROS2 bridge
```

## 11. 当前未完成部分

- 当前编排层虽然已面向正式算法接口，但实际实现仍经由正式通用适配器转接到 `transplant/tools/` 下的 PID、LQR、VMC 实现，算法库尚未正式抽包。
- `wheel_leg_hooks.cc` 仍保留过渡期的 hook 生命周期入口与少量仿真侧绘图/日志杂项。
- `wheel_leg_hooks.cc` 已明显收缩，当前仅保留 `OnModelLoaded` / `BeforeStep` / `AfterStep` 这类 MuJoCo 生命周期入口。

## 12. 本次结论

- 已完成一次隔离域 `10 s+` takeover 保持复测：controller `dt` 连续样本为 `0.002000000 s`，takeover 持续约 `12.43 s`，未出现 controller 发布前限幅、未生成 trace、未出现 `QACC` / unstable 告警。
- 这说明当前“正式 `/robot_state` + 正式控制编排 + ROS takeover”主链已满足本轮收口所需的代码与验证闭环。

## 13. 下一步建议

1. 保持 `wheel_leg_hooks.cc` 只承担 step hook 生命周期和过渡入口，并继续收缩其中的 legacy 运行时对象装配。
2. 继续把 PID、LQR、VMC 的实现和参数类型从 `transplant/tools/` 抽向正式包目录。
3. 在后续阶段将当前控制 pipeline 逐步对齐到 `wheel_leg_control` 的正式 controller node。
