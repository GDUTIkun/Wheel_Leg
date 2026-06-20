# 验证记录

## 1. 文档说明

本文档记录本项目仿真和实机验证结果。根据 `doc/AGENTS.md` 规则，未经实际仿真或实机验证的功能不能标记为已通过。

当前对应迭代：

- 迭代：`iter-001`
- 迭代名称：ROS2-MuJoCo Topic 桥接
- 详细设计：`doc/detail.md`
- 当前状态：ROS2-MuJoCo 最小桥接代码已实现，`/joint_states`、`/imu`、`/joint_command` 和 actuator 写入边界已完成短时仿真烟测；`/joint_states` 和 `/imu` 频率已验证约 `100 Hz`；长时间站立回归、IMU 坐标语义、命令超时和限幅验证尚未执行；实机验证不在本迭代范围内。

## 2. 当前验证结论

| 验证项 | 环境 | 当前状态 | 说明 |
| --- | --- | --- | --- |
| `/joint_states` 状态发布 | 仿真 | 已通过 | 可收到 6 关节消息，修正发布调度后 `ros2 topic hz` 约 `100 Hz` |
| `/imu` 状态发布 | 仿真 | 已通过 | 可收到 `base_link` IMU 消息，修正发布调度后 `ros2 topic hz` 约 `100 Hz` |
| `/joint_command` 命令接收 | 仿真 | 已通过 | 有效命令可触发 bridge 写入边界；未知关节命令被拒绝 |
| MuJoCo actuator 写入 | 仿真 | 已通过 | `enable_ros_command=true` 时有效命令进入 actuator 写入边界；无效命令不写入 |
| `wheel_leg_msgs/msg/JointCommand` 消息包 | 构建 | 已通过 | `colcon build --packages-select wheel_leg_msgs` 成功，`colcon list` 正确识别为 `ros.ament_cmake` |
| 现有 MuJoCo 基础站立能力不被破坏 | 仿真 | 部分通过 | 短时启动和状态发布正常；尚未做长时间站立行为回归 |
| STM32 真机通信 | 实机 | 不适用 | 不属于 `iter-001` |
| 遥控器输入 | 实机 / 仿真 | 不适用 | 不属于 `iter-001` |

## 3. Iteration 001 验证计划

### 3.1 仿真验证目标

后续完成代码实现后，应验证 ROS2 与 MuJoCo 之间的最小 Topic 桥接闭环：

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

### 3.2 仿真验证前置条件

- MuJoCo 仿真可以正常启动。
- ROS2 Jazzy 环境可以正常启动。
- ROS2-MuJoCo 桥接代码已经实现。
- `/joint_command` 已确认使用 `wheel_leg_msgs/msg/JointCommand`。
- actuator 映射表已经确认。
- `/joint_states` 和 `/imu` 发布频率按 `100 Hz` 验证。
- 状态消息时间戳使用 MuJoCo 仿真时间 `d->time`。
- `/joint_command` 命令处理跟随 MuJoCo step。
- `/imu.header.frame_id` 使用 `base_link`。
- `/imu.orientation` 按 ROS `x,y,z,w` 顺序验证。
- 若现有 MuJoCo 内部站立控制仍启用，应明确其与 ROS2 命令的优先级关系。

### 3.3 仿真验证步骤

1. 启动 ROS2 环境。
2. 启动 MuJoCo 仿真。
3. 启动 ROS2-MuJoCo 桥接节点或内置桥接逻辑。
4. 使用 ROS2 CLI 查看 `/joint_states` 是否存在。
5. 使用 ROS2 CLI 查看 `/joint_states` 是否持续发布。
6. 使用 ROS2 CLI 查看 `/imu` 是否存在。
7. 使用 ROS2 CLI 查看 `/imu` 是否持续发布。
8. 发布一条最小 `/joint_command` 测试命令。
9. 观察桥接层是否接收到命令。
10. 观察 MuJoCo actuator 或机器人状态是否有响应。
11. 停止发布命令，观察仿真是否保持稳定。
12. 回归确认现有 MuJoCo 基础站立能力是否仍可运行。

### 3.4 仿真通过标准

- ROS2 可以订阅到 `/joint_states`。
- `/joint_states` 中关节名称、位置、速度数组长度一致。
- ROS2 可以订阅到 `/imu`。
- `/imu` 中姿态、角速度和线加速度数据随仿真状态变化。
- ROS2 发布 `/joint_command` 后，桥接层可以接收命令。
- 有效命令可以被映射到 MuJoCo actuator 写入边界。
- 无效命令不会导致仿真崩溃。
- 新增桥接逻辑不破坏现有 MuJoCo 仿真启动。

### 3.5 实机验证说明

`iter-001` 不进行实机验证。STM32 通信、真实传感器读取、电机驱动和实机安全保护留到后续迭代。

## 4. 验证记录

### VAL-001: `/joint_states` 状态发布验证

- 日期：2026-06-20
- 对应迭代：`iter-001`
- 测试环境：仿真
- 测试目标：验证 MuJoCo 是否能向 ROS2 发布 `/joint_states`
- 测试步骤：
  1. 执行 `source /opt/ros/jazzy/setup.bash && colcon build --packages-select wheel_leg_msgs`
  2. 执行 `source /opt/ros/jazzy/setup.bash && source install/local_setup.bash`
  3. 执行 `cmake -S transplant/mujoco_win/simulate -B build/wheel_leg_simulate_ros2 -DWHEEL_LEG_ENABLE_ROS2=ON`
  4. 执行 `cmake --build build/wheel_leg_simulate_ros2 -j2`
  5. 短时启动 `timeout -k 2s 12s build/wheel_leg_simulate_ros2/wheel_leg_simulate transplant/mujoco_win/model/scence.xml`
  6. 执行 `ros2 topic echo --once /joint_states`
- 观察结果：收到 `/joint_states` 消息，`name` 包含 `left_hip`、`left_knee`、`left_wheel`、`right_hip`、`right_knee`、`right_wheel`；`position` 和 `velocity` 均为 6 项；`effort` 为空。
- 是否通过：已通过
- 问题现象：`ros2 topic echo` 出现一次 `A message was lost` 提示，但随后成功收到消息。
- 可能原因：echo 订阅启动瞬间的 DDS 队列现象，不影响本次最小消息可达性结论。
- 下一步建议：后续补充长时间运行验证。

### VAL-002: `/imu` 状态发布验证

- 日期：2026-06-20
- 对应迭代：`iter-001`
- 测试环境：仿真
- 测试目标：验证 MuJoCo 是否能向 ROS2 发布 `/imu`
- 测试步骤：
  1. 在 VAL-001 相同构建和仿真环境下短时启动 MuJoCo。
  2. 执行 `ros2 topic echo --once /imu`。
- 观察结果：收到 `/imu` 消息，`header.frame_id` 为 `base_link`；消息包含 `orientation`、`angular_velocity`、`linear_acceleration`；covariance 为全 0。
- 是否通过：已通过
- 问题现象：`ros2 topic echo` 出现一次 `A message was lost` 提示，但随后成功收到消息；本次只验证消息可达和字段形态。
- 可能原因：echo 订阅启动瞬间的 DDS 队列现象，不影响本次最小消息可达性结论。
- 下一步建议：后续通过可控姿态动作或可视化验证 IMU 坐标系语义。

### VAL-003: `/joint_command` 命令接收验证

- 日期：2026-06-20
- 对应迭代：`iter-001`
- 测试环境：仿真
- 测试目标：验证 MuJoCo 桥接层是否能接收 ROS2 `/joint_command`
- 测试步骤：
  1. 在 VAL-001 相同构建和仿真环境下短时启动 MuJoCo。
  2. 执行 `ros2 topic info /joint_command -v`。
  3. 执行 `ros2 param set /mujoco_bridge enable_ros_command true`。
  4. 执行 `ros2 topic pub --times 5 --rate 10 /joint_command wheel_leg_msgs/msg/JointCommand "{joint_names: ['left_wheel'], efforts: [0.05]}"`。
  5. 执行 `ros2 topic pub --once /joint_command wheel_leg_msgs/msg/JointCommand "{joint_names: ['bad_joint'], efforts: [0.05]}"`。
- 观察结果：
  - `/joint_command` 类型为 `wheel_leg_msgs/msg/JointCommand`，订阅者数量为 1，订阅节点为 `/mujoco_bridge`。
  - 有效命令发布后，bridge 日志输出 `Applied /joint_command to 1 actuator(s)`。
  - 未知关节命令发布后，bridge 日志输出 `Rejected invalid /joint_command; no actuator was written`。
- 是否通过：已通过
- 问题现象：本次尚未独立验证命令超时行为。
- 可能原因：本次目标是命令订阅和无效命令拒绝的最小验证，超时验证留到后续。
- 下一步建议：补充 `0.2 s` 超时后停止接管 actuator 的验证。

### VAL-004: MuJoCo actuator 写入验证

- 日期：2026-06-20
- 对应迭代：`iter-001`
- 测试环境：仿真
- 测试目标：验证 ROS2 命令是否能被映射并写入 MuJoCo actuator
- 测试步骤：
  1. 在 VAL-001 相同构建和仿真环境下短时启动 MuJoCo。
  2. 执行 `ros2 param set /mujoco_bridge enable_ros_command true`。
  3. 发布 `left_wheel=0.05` 的 `/joint_command`，连续 5 次。
  4. 发布未知关节 `bad_joint` 的 `/joint_command`。
- 观察结果：
  - 有效命令触发 bridge actuator 写入边界，日志输出 `Applied /joint_command to 1 actuator(s)`。
  - 无效命令被拒绝，日志输出 `Rejected invalid /joint_command; no actuator was written`。
- 是否通过：已通过
- 问题现象：曾用 `left_wheel=2.0` 做探索时，仿真在约 6.9s 出现 `Nan, Inf or huge value in QACC` unstable warning；该探索命令不作为通过用例。
- 可能原因：较大 wheel effort 与现有内部站立控制叠加后的动力学稳定性需要单独评估。
- 下一步建议：后续验证限幅、超时和更长时间动力学响应时，从小力矩开始，避免把桥接连通性验证和控制稳定性验证混在一起。

### VAL-005: MuJoCo 基础站立能力回归验证

- 日期：2026-06-20
- 对应迭代：`iter-001`
- 测试环境：仿真
- 测试目标：验证新增 ROS2-MuJoCo 桥接逻辑不会破坏现有基础站立能力
- 测试步骤：
  1. 短时启动 `build/wheel_leg_simulate_ros2/wheel_leg_simulate transplant/mujoco_win/model/scence.xml`。
  2. 不发布 `/joint_command`。
  3. 观察程序是否能加载模型、创建 bridge 并持续发布状态。
- 观察结果：程序成功加载模型，日志显示 `wheel_leg simulate ready` 和 `ROS2 MuJoCo bridge ready`；`/joint_states` 和 `/imu` 可收到消息。
- 是否通过：部分通过
- 问题现象：短时测试结束时需要 `timeout -k` 强制清理进程；未做长时间 GUI 站立行为观察。
- 可能原因：MuJoCo GUI 程序对普通 SIGTERM 不立即退出；这属于测试清理方式问题，不代表仿真启动失败。
- 下一步建议：后续人工或带显示环境长时间运行，确认基础站立行为未退化。

### VAL-006: STM32 真机通信验证

- 日期：2026-06-20
- 对应迭代：后续迭代
- 测试环境：实机
- 测试目标：验证 Raspberry Pi 与 STM32 通信
- 测试步骤：不属于当前迭代
- 观察结果：未执行
- 是否通过：不适用
- 问题现象：无
- 可能原因：STM32 通信层尚未进入迭代
- 下一步建议：进入 STM32 通信层迭代后重新定义实机验证步骤

### VAL-007: `wheel_leg_msgs/msg/JointCommand` 消息包构建验证

- 日期：2026-06-20
- 对应迭代：`iter-001`
- 测试环境：构建
- 测试目标：验证 `wheel_leg_msgs/msg/JointCommand` 可以生成 ROS2 接口
- 测试步骤：
  1. 执行 `colcon build --packages-select wheel_leg_msgs`
  2. 执行 `source install/wheel_leg_msgs/share/wheel_leg_msgs/local_setup.bash`
  3. 执行 `ros2 interface show wheel_leg_msgs/msg/JointCommand`
  4. 用 Python 导入并实例化 `wheel_leg_msgs.msg.JointCommand`
- 观察结果：
  - `colcon build --packages-select wheel_leg_msgs` 成功。
  - `ros2 interface show wheel_leg_msgs/msg/JointCommand` 显示 `header`、`joint_names`、`efforts` 字段。
  - Python 可以实例化 `JointCommand`。
  - 修复 `package.xml` 中缺失的 `ament_cmake` build type 后，`colcon list` 正确识别 `wheel_leg_msgs` 为 `ros.ament_cmake`。
- 是否通过：已通过
- 问题现象：修复前 `wheel_leg_msgs` 曾被 `colcon list` 识别为 `ros.catkin`；根目录误生成的空 `AMENT_IGNORE` 曾导致 `colcon list` 无输出。
- 可能原因：`package.xml` 缺少 `<export><build_type>ament_cmake</build_type></export>`；根目录存在生成残留文件。
- 下一步建议：保持工作区根目录不放置 `AMENT_IGNORE`；后续若重新生成构建目录，优先使用 `build/` 下的独立目录。

### VAL-008: `/joint_states` 和 `/imu` 发布频率验证

- 日期：2026-06-20
- 对应迭代：`iter-001`
- 测试环境：仿真
- 测试目标：验证 `/joint_states` 和 `/imu` 发布频率是否符合 `100 Hz` 约束
- 测试步骤：
  1. 执行 `source /opt/ros/jazzy/setup.bash && source install/local_setup.bash`
  2. 执行 `cmake --build build/wheel_leg_simulate_ros2 -j2`
  3. 短时启动 `timeout -k 2s 12s build/wheel_leg_simulate_ros2/wheel_leg_simulate transplant/mujoco_win/model/scence.xml`
  4. 并行执行 `timeout 6s ros2 topic hz /joint_states` 和 `timeout 6s ros2 topic hz /imu`
- 初始观察结果：
  - 初始并行统计中 `/joint_states` 约 `83.31 Hz`。
  - 初始并行统计中 `/imu` 约 `83.35 Hz`。
- 问题现象：两个 topic 同步低于目标 `100 Hz`。
- 可能原因：MuJoCo timestep 为 `0.002 s`，初始实现使用 `d->time - last_publish_time_ < 0.01` 做浮点差值判断，可能因浮点误差每 6 个 step 才发布一次，即约 `0.012 s`、`83.3 Hz`。
- 修复方式：将 bridge 状态发布调度改为 `next_publish_time` 边界，按仿真时间推进下一次发布时间。
- 修复后观察结果：
  - `/joint_states` 统计约 `100.0 Hz`，示例输出包含 `average rate: 100.007`。
  - `/imu` 统计约 `100.0 Hz`，示例输出包含 `average rate: 100.015`。
- 是否通过：已通过
- 下一步建议：长时间运行时可再次统计频率漂移，同时观察 GUI 站立稳定性。

## 5. 待确认问题

以下问题需要在后续实现或验证前确认：

- `/imu` 坐标系语义仍需通过仿真观察验证。
- `/joint_command` 超时和限幅策略仍需单独验证。
