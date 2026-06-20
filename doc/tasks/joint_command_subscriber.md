# 模块任务：关节命令订阅

## 1. 模块名称

关节命令订阅模块。

## 2. 当前迭代目标

从 ROS2 接收 `/joint_command`，作为调试工具或后续控制器向 MuJoCo 下发关节或电机命令的入口。

当前 `/joint_command` 已确认使用 `wheel_leg_msgs/msg/JointCommand`。消息字段为 `std_msgs/Header header`、`string[] joint_names`、`float64[] efforts`，`joint_names` 与 `efforts` 一一对应，`efforts` 单位为 Nm。

## 3. 任务 checklist

- `[x]` 阅读 `doc/detail.md` 中关节命令订阅模块设计。
- `[x]` 确认 `/joint_command` 具体消息类型：使用自定义消息。
- `[x]` 确认 `/joint_command` 自定义消息包名、消息名和字段。
- `[x]` 确认命令映射方式：使用关节名映射。
- `[x]` 确认命令单位是否只允许 Nm：当前消息只表达 effort，单位为 Nm。
- `[x]` 确认命令超时策略：`0.2 s`，超时后停止应用 ROS 命令，不自动清零 actuator。
- `[x]` 确认命令限幅策略：使用 MuJoCo actuator `ctrlrange`。
- `[x]` 确认命令处理频率：每个 MuJoCo step 处理一次最新有效命令。
- `[x]` 确认 ROS2 命令与现有 MuJoCo 内部站立控制的优先级：默认不接管，`enable_ros_command=true` 时在 step 末尾覆盖对应 actuator。
- `[x]` 创建 `/joint_command` 订阅器。
- `[x]` 接收并缓存最新命令。
- `[x]` 向 actuator 命令写入模块提供命令。
- `[v]` 完成后执行仿真验证，有效命令可触发 bridge 写入边界，无效命令被拒绝。

## 4. 每个任务的输入

- Topic：`/joint_command`
- Message：`wheel_leg_msgs/msg/JointCommand`
- 字段：`std_msgs/Header header`、`string[] joint_names`、`float64[] efforts`。
- ROS2 回调上下文。
- 关节命名约定。
- 命令处理频率：每个 MuJoCo step 处理一次最新有效命令。
- 命令超时：`0.2 s`。
- 无效命令策略：整条拒绝，不写入任何 actuator。

## 5. 每个任务的输出

- 最新命令缓存。
- 命令时间戳。
- 命令有效性状态。
- 传递给 actuator 写入模块的命令数据。

## 6. 完成标准

- ROS2 发布 `/joint_command` 后，桥接层可以接收命令。
- 无效命令不会导致仿真崩溃。
- 命令解析逻辑使用 `joint_names` 与 `efforts` 一一对应的关节名映射。
- 该模块不直接写入 MuJoCo actuator，写入由 actuator 命令写入模块负责。
- `joint_names` 与 `efforts` 长度不一致、未知 joint name、NaN 或 Inf 均视为无效命令。

## 7. 验证方法

仿真验证：

- 启动 MuJoCo 仿真和 ROS2 桥接。
- 使用 ROS2 CLI 或调试节点发布 `/joint_command`。
- 确认桥接层能接收命令。
- 发布无效命令，确认程序不崩溃。

实机验证：

- 本模块不进行实机验证。
- 真机命令订阅或转发留到 STM32 通信层迭代。

## 8. 当前状态

`[v] 已通过最小仿真验证`

当前阻塞：

- 暂无。后续可补充更多无效命令组合和超时行为验证。

## 9. 本次验证记录

记录时间：2026-06-20。

- 构建通过：`colcon build --packages-select wheel_leg_msgs`，以及 MuJoCo bridge 目标编译通过。
- 端点验证命令：短时启动仿真后执行 `ros2 topic info /joint_command -v`。
- 观察结果：`/joint_command` 类型为 `wheel_leg_msgs/msg/JointCommand`，订阅者数量为 1，订阅节点为 `/mujoco_bridge`。
- 有效命令验证：设置 `ros2 param set /mujoco_bridge enable_ros_command true` 后，发布 `left_wheel=0.05`，bridge 日志输出 `Applied /joint_command to 1 actuator(s)`。
- 无效命令验证：发布未知关节 `bad_joint`，bridge 日志输出 `Rejected invalid /joint_command; no actuator was written`。
- 尚未完成：命令超时行为的独立验证。
