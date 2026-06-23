# MuJoCo Sim Adapter 提取任务

## 1. 任务目标

定义 `wheel_leg_sim` 的长期职责边界，把 MuJoCo 状态读取、actuator 写入和 step 回调相关逻辑，从当前 `wheel_leg_hooks.cc` 的混合控制流程中拆出来。

当前任务重点是明确 MuJoCo 在长期工程中的角色：

- 负责仿真周期
- 负责状态采样
- 负责命令执行
- 不负责控制流程组织

## 2. 当前输入

- `mjModel`
- `mjData`
- 当前仿真 step 生命周期
- 来自 bridge 或 control 层的通用控制命令

## 3. 当前输出

- `JointStateSample`
- `ImuSample`
- 供上层状态聚合使用的原始状态结构
- actuator 执行边界

## 4. 当前边界要求

- `wheel_leg_sim` 只暴露采样状态、执行命令和 step 边界接口。
- MuJoCo 类型不应泄漏为长期 controller 接口。
- bridge 和 control 层不直接承担 MuJoCo 数据读写。
- `wheel_leg_hooks.cc` 只保留迁移期间的过渡入口定位。

## 5. 需要完成的文档化事项

1. 明确哪些逻辑属于状态采样。
2. 明确哪些逻辑属于 actuator 写入。
3. 明确哪些逻辑属于仿真 hook 调度。
4. 明确这些逻辑从 `wheel_leg_hooks.cc` 迁出后的归属目录。
5. 明确与 `wheel_leg_bridge`、`wheel_leg_control` 的接口形式。

## 6. 推荐迁移顺序

1. 先整理状态读取边界。
2. 再整理 actuator 写入边界。
3. 再整理 step hook 调用边界。
4. 最后缩减 `wheel_leg_hooks.cc` 为过渡接入层。

## 7. 完成标准

满足以下条件即可认为该任务文档完成：

- 能把 MuJoCo 长期职责限定在仿真适配层。
- 能指导后续实现者把状态采样和命令执行从控制逻辑中分离。
- 能保证 `iter-001` 已有 `/joint_states`、`/imu`、`/joint_command` 和 actuator 边界不会因结构迁移而退化。

## 8. 非目标

- 不在本任务中定义 STM32 adapter。
- 不在本任务中替换动力学模型。
- 不在本任务中完成完整 controller node。

## 9. 当前状态

- `[v] 已通过验证`

## 10. 当前已落地进展

- 过渡 `wheel_leg/sim_adapter.*` 已删除，不再作为 MuJoCo 仿真侧的额外胶水层存在。
- 关节状态、IMU 采样和 actuator 写入辅助已直接并入 `ros2_bridge.*` 所在桥接边界。
- legacy 控制执行所需的传感采样与命令写入已直接复用 `sensor.*` 与正式 `wheel_leg_sim` / `wheel_leg_control` helper。
- 仿真侧本地 plotting 和 `PrintSensors` 调试链已移除，避免再把调试职责留在 MuJoCo hook 壳层。
- 迁移早期曾在 `transplant/mujoco_win/simulate/wheel_leg/sim_adapter.*` 中建立过渡期 sim adapter；其职责现已回收，不再保留为独立层。
- 状态发布所需的 `JointStateSample`、`ImuSample` 采样能力已沉淀到当前正式读写路径中。
- `SensorAssemblyState` 曾从 `sensor.cc` 隐式 `static` 迁为显式状态；当前相关采样状态管理已随过渡层收口并回到现行 helper 链。
- actuator 命令执行已统一收敛到 `ApplyControlCommand(...)`。
- `transplant` 中的 ROS2 bridge 已通过正式 `wheel_leg_bridge` 做消息转换。
- `src/wheel_leg_sim` 已从纯接口包升级为实际可链接库，并已承载 joint mapping / sim time conversion / command preparation 等通用实现。

## 11. 当前未完成部分

- `wheel_leg_hooks.cc` 仍然是当前 MuJoCo hook 入口，但已收缩到过渡生命周期壳层。
- 后续仍可继续把少量历史命名和目录布局进一步向正式 `wheel_leg_sim` 对齐；这不阻塞本轮结项。

## 12. 本次结论

- `sim adapter` 这条过渡链已实质收口：仿真侧长期职责已经回到状态采样、命令执行与 bridge 读写边界本身，不再额外维持一层独立胶水实现。
- 在 `ROS_DOMAIN_ID=109` 的 `10 s+` takeover 保持复测中，bridge 累计写入 `6162` 次 `/joint_command`，未出现 `QACC` / unstable 告警，说明当前 `wheel_leg_sim` 相关读写路径满足本轮验收。
