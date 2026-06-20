# 模块任务：actuator 命令写入

## 1. 模块名称

actuator 命令写入模块。

## 2. 当前迭代目标

将已接收并校验的 ROS2 命令转换为 MuJoCo actuator 控制量，并写入 MuJoCo 仿真边界。

## 3. 任务 checklist

- `[x]` 阅读 `doc/detail.md` 中 actuator 命令写入模块设计。
- `[x]` 确认 actuator 名称映射表。
- `[x]` 确认命令单位和控制模式：当前只表达 effort，单位为 Nm。
- `[x]` 确认力矩限幅策略：使用 MuJoCo actuator `ctrlrange`。
- `[x]` 确认命令超时后的行为：停止应用 ROS 命令，回到不接管 actuator 行为，不自动清零 actuator。
- `[x]` 确认 ROS2 命令与现有 MuJoCo 内部站立控制的优先级：默认不接管，`enable_ros_command=true` 时在 step 末尾覆盖对应 actuator。
- `[x]` 接收关节命令订阅模块提供的最新有效命令。
- `[x]` 查找目标 actuator。
- `[x]` 将命令值转换为 MuJoCo actuator 控制量。
- `[x]` 写入 `d->ctrl` 或现有 actuator helper 边界。
- `[v]` 完成后执行仿真验证，有效命令可写入 actuator 边界，无效命令不写入。

## 4. 每个任务的输入

- MuJoCo 模型对象。
- MuJoCo 数据对象。
- 最新有效 `/joint_command`。
- actuator 名称映射表。
- 命令单位：Nm。
- 限幅约定：使用 MuJoCo actuator `ctrlrange`。
- 接管开关：`enable_ros_command`。

## 5. 每个任务的输出

- 写入 MuJoCo actuator 的控制量。
- 写入结果状态。
- 写入失败或跳过原因。

## 6. 完成标准

- 有效命令可以映射到目标 actuator。
- 未知 actuator 不会被静默写错。
- 无效命令不会导致仿真崩溃。
- 写入逻辑不包含控制算法。
- 默认不覆盖现有内部控制输出；仅在 `enable_ros_command=true` 且命令有效、未超时时覆盖对应 actuator。

## 7. 验证方法

仿真验证：

- 发布单个 actuator 测试命令。
- 观察 MuJoCo actuator 控制量或机器人状态响应。
- 发布未知 actuator 或无效命令，确认程序不崩溃。
- 回归确认现有 MuJoCo 基础站立能力未被破坏。

实机验证：

- 本模块不进行实机验证。
- 真机电机命令写入留到 STM32 通信层迭代。

## 8. 当前状态

`[v] 已通过最小仿真验证`

当前阻塞：

- 暂无。后续可补充命令超时、限幅和长时间动力学响应验证。

## 9. 本次验证记录

记录时间：2026-06-20。

- 当前代码已包含关节名到 actuator 的映射、MuJoCo `ctrlrange` 限幅、`enable_ros_command` 默认关闭、`0.2 s` 超时和无效命令拒绝逻辑。
- 验证命令：设置 `ros2 param set /mujoco_bridge enable_ros_command true` 后，发布 `ros2 topic pub --times 5 --rate 10 /joint_command wheel_leg_msgs/msg/JointCommand "{joint_names: ['left_wheel'], efforts: [0.05]}"`。
- 观察结果：bridge 日志连续输出 `Applied /joint_command to 1 actuator(s)`，说明有效命令进入 actuator 写入边界。
- 无效命令验证：发布未知关节 `bad_joint` 后，bridge 日志输出 `Rejected invalid /joint_command; no actuator was written`。
- 尚未完成：命令超时、限幅和长时间动力学响应验证。
