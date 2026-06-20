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

- `[ ] 未开始`
