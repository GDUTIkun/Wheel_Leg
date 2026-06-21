# 模块任务：关节状态发布

## 1. 模块名称

关节状态发布模块。

## 2. 当前迭代目标

从 MuJoCo 读取关节状态，并通过 ROS2 标准 topic `/joint_states` 发布 `sensor_msgs/msg/JointState`。

## 3. 任务 checklist

- `[x]` 阅读 `doc/detail.md` 中关节状态发布模块设计。
- `[x]` 确认关节命名表。
- `[x]` 确认关节发布顺序。
- `[x]` 确认 `/joint_states` 是否填充 effort：当前不填充 effort。
- `[x]` 确认时间戳使用仿真时间还是 ROS2 当前时间：使用 MuJoCo 仿真时间 `d->time`。
- `[x]` 确认状态发布频率：每个 MuJoCo step，当前约 `500 Hz`。
- `[x]` 从 MuJoCo 数据中读取关节位置。
- `[x]` 从 MuJoCo 数据中读取关节速度。
- `[x]` 组装 `sensor_msgs/msg/JointState`。
- `[x]` 发布 `/joint_states`。
- `[v]` 完成后执行仿真验证，ROS2 CLI 可 echo 到 `/joint_states`。

## 4. 每个任务的输入

- MuJoCo 模型对象。
- MuJoCo 数据对象。
- 当前仿真时间。
- 关节命名表：`left_hip`、`left_knee`、`left_wheel`、`right_hip`、`right_knee`、`right_wheel`。
- 关节顺序约定：按关节命名表顺序发布。
- 发布频率：每个 MuJoCo step，当前约 `500 Hz`。
- 时间戳策略：使用 MuJoCo 仿真时间 `d->time`。

## 5. 每个任务的输出

- Topic：`/joint_states`
- Message：`sensor_msgs/msg/JointState`
- 字段语义：关节名称、位置、速度；当前不填充 effort。

## 6. 完成标准

- ROS2 可以订阅 `/joint_states`。
- `name`、`position`、`velocity` 数组长度一致。
- 关节状态随 MuJoCo 仿真变化。
- 缺失关节不会导致仿真程序崩溃。
- 当前不填充 effort。

## 7. 验证方法

仿真验证：

- 启动 MuJoCo 仿真和 ROS2 桥接。
- 执行 `ros2 topic echo /joint_states`。
- 检查 `name`、`position`、`velocity` 是否存在且长度一致。
- 观察机器人状态变化时数值是否变化。

实机验证：

- 本模块不进行实机验证。
- 实机 `/joint_states` 留到 STM32 通信层迭代。

## 8. 当前状态

`[v] 已通过最小仿真烟测`

当前待确认：

- 暂无。后续可补充长时间运行验证。

## 9. 本次验证记录

记录时间：2026-06-20。

- 构建命令：`source /opt/ros/jazzy/setup.bash && source install/local_setup.bash && cmake --build build/wheel_leg_simulate_ros2 -j2`。
- 仿真命令：`timeout -k 2s 12s build/wheel_leg_simulate_ros2/wheel_leg_simulate transplant/mujoco_win/model/scence.xml`。
- 验证命令：`ros2 topic echo --once /joint_states`。
- 观察结果：消息包含 6 个关节名称，`position` 和 `velocity` 均为 6 项，`effort` 为空，符合当前策略。
- 频率验证：当前实现按每个 MuJoCo step 发布，`ros2 topic hz /joint_states` 统计约 `500 Hz`。
- 修正说明：旧实现曾通过 `0.01 s` 周期调度把 `/joint_states` 限在 `100 Hz`；现已移除该限流，直接随 `0.002 s` MuJoCo timestep 发布。
