# 验证记录

## 1. 文档说明

本文档记录本项目仿真和实机验证结果。根据 `doc/AGENTS.md` 规则，未经实际仿真或实机验证的功能不能标记为已通过。

当前对应迭代：

- 迭代：`iter-001`
- 迭代名称：ROS2-MuJoCo Topic 桥接
- 详细设计：`doc/detail.md`
- 当前状态：ROS2-MuJoCo 最小桥接代码已实现，`/joint_states`、`/imu`、`/joint_command` 和 actuator 写入边界已完成短时仿真烟测；`/joint_states`、`/imu` 和 `/robot_state` 当前已验证约 `500 Hz`；ROS controller 已切到 `/robot_state` 驱动且 dt 日志已验证为约 `0.002 s`；`/joint_command` 超时和限幅行为已验证；headless 长时间站立回归已验证；IMU 坐标语义已验证；实机验证不在本迭代范围内。
- 当前状态：ROS2-MuJoCo 最小桥接代码已实现，`/joint_states`、`/imu`、`/joint_command` 和 actuator 写入边界已完成短时仿真烟测；`/joint_states`、`/imu` 和 `/robot_state` 当前已验证约 `500 Hz`；ROS controller 已切到 `/robot_state` 驱动且 dt 日志已验证为约 `0.002 s`；`100 Hz` 降采样接管实验已验证当前参数下仍会失稳，因此主线继续保持 `500 Hz`；`/joint_command` 超时和限幅行为已验证；headless 长时间站立回归已验证；IMU 坐标语义已验证；实机验证不在本迭代范围内。

补充说明：

- `iter-002` 当前已追加一次最小运行烟测，用于确认控制编排外移和正式算法接口适配后，MuJoCo 启动链、`OnModelLoaded` 和 ROS2 bridge 初始化链未被破坏。

## 2. 当前验证结论

| 验证项 | 环境 | 当前状态 | 说明 |
| --- | --- | --- | --- |
| `/joint_states` 状态发布 | 仿真 | 已通过 | 可收到 6 关节消息；当前 `ros2 topic hz` 约 `500 Hz` |
| `/imu` 状态发布 | 仿真 | 已通过 | 可收到 `base_link` IMU 消息，频率约 `500 Hz`，模型挂载和运行时样本均支持当前坐标语义 |
| `/robot_state` 控制状态发布 | 仿真 | 已通过 | 可收到正式 `StandControlState` 消息；发布路径 round-trip 校验通过；当前 `ros2 topic hz` 约 `500 Hz` |
| `/joint_command` 命令接收 | 仿真 | 已通过 | 有效命令可触发 bridge 写入边界；未知关节命令被拒绝 |
| MuJoCo actuator 写入 | 仿真 | 已通过 | 有效命令进入 actuator 写入边界；无效命令不写入；超时后暂停写入；超范围命令按 `ctrlrange` 限幅 |
| `wheel_leg_msgs/msg/JointCommand` 消息包 | 构建 | 已通过 | `colcon build --packages-select wheel_leg_msgs` 成功，`colcon list` 正确识别为 `ros.ament_cmake` |
| 现有 MuJoCo 基础站立能力不被破坏 | 仿真 | 已通过 | headless 无命令运行 `24 s` 无不稳定告警，中途 topic 仍持续可读；GUI 观感验证仍可补充 |
| `iter-002` 过渡编排最小启动验证 | 仿真 | 已通过 | 过渡编排切到正式算法接口后，MuJoCo 仍可加载模型并完成 `OnModelLoaded` 与 ROS2 bridge 初始化 |
| `rc_ibus_node` 串口接收与 `iBUS` 解包 | 实机 | 已通过 | `/dev/ttyAMA3` 可稳定接收合法 `iBUS` 帧，`checksum_errors=0`，`/rc/channels_raw` 可持续发布 |
| 遥控器统一命令映射 | 实机 | 已通过 | 已确认真实通道布局，`/cmd_vel`、`/control_mode`、`/body_cmd` 与急停映射符合预期 |
| 遥控器统一命令接入仿真控制链 | 实机 / 仿真 | 已通过 | `rc_ibus_node`、`wheel_leg_controller_node` 与 `mujoco_bridge` 已联通；`disabled`、`stand`、`velocity` 与主速度摇杆映射在仿真联调中符合预期 |
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
- `/joint_states`、`/imu` 和 `/robot_state` 发布频率按每个 MuJoCo step、约 `500 Hz` 验证。
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
- 超时观察结果：发布单条 `left_wheel=0.05` 后，bridge 先输出 `Applied /joint_command to 1 actuator(s)`，约 `0.2 s` 后输出 `/joint_command timed out after 0.200 s; actuator writes are suspended`。
- 是否通过：已通过
- 下一步建议：补充更多无效命令组合验证。

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
  - 命令超时后，日志输出 `/joint_command timed out after 0.200 s; actuator writes are suspended`。
  - 超范围命令按 MuJoCo actuator `ctrlrange` 限幅。
- 是否通过：已通过
- 问题现象：曾用 `left_wheel=2.0` 做探索时，仿真在约 6.9s 出现 `Nan, Inf or huge value in QACC` unstable warning；该探索命令不作为通过用例。
- 可能原因：较大 wheel effort 与现有内部站立控制叠加后的动力学稳定性需要单独评估。
- 下一步建议：后续验证更长时间动力学响应时，从小力矩开始，避免把桥接连通性验证和控制稳定性验证混在一起。

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
- 长时回归结果：在无 `/joint_command` 条件下，headless 运行 `24 s`，中途 `/joint_states` 与 `/imu` 仍可正常读取，日志中未出现新的 `Nan, Inf or huge value in QACC` 等不稳定告警。
- 是否通过：已通过
- 问题现象：测试结束时需要 `timeout -k` 强制清理进程；尚未做人眼 GUI 站立观感检查。
- 可能原因：MuJoCo GUI 程序对普通 SIGTERM 不立即退出；这属于测试清理方式问题，不代表仿真启动失败。
- 下一步建议：后续若需要补充 GUI 观感验证，可在带显示环境长时间运行确认站立姿态观感。

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
- 测试目标：验证 `/joint_states` 和 `/imu` 发布频率是否符合每个 MuJoCo step、约 `500 Hz` 约束
- 测试步骤：
  1. 执行 `source /opt/ros/jazzy/setup.bash && source install/local_setup.bash`
  2. 执行 `cmake --build build/wheel_leg_simulate_ros2 -j2`
  3. 短时启动 `timeout -k 2s 12s build/wheel_leg_simulate_ros2/wheel_leg_simulate transplant/mujoco_win/model/scence.xml`
  4. 并行执行 `timeout 6s ros2 topic hz /joint_states` 和 `timeout 6s ros2 topic hz /imu`
- 初始观察结果：
  - 初始并行统计中 `/joint_states` 约 `83.31 Hz`。
  - 初始并行统计中 `/imu` 约 `83.35 Hz`。
- 问题现象：旧实现中两个 topic 同步低于目标 `100 Hz`。
- 可能原因：MuJoCo timestep 为 `0.002 s`，初始实现使用 `d->time - last_publish_time_ < 0.01` 做浮点差值判断，可能因浮点误差每 6 个 step 才发布一次，即约 `0.012 s`、`83.3 Hz`。
- 修复方式：旧版先通过 `next_publish_time` 边界恢复到 `100 Hz`；当前控制接管阶段已进一步移除 `100 Hz` 限流，改为每个 MuJoCo step 直接发布。
- 当前观察结果：
  - `/joint_states` 统计约 `500.0 Hz`，示例输出包含 `average rate: 499.997`。
  - `/imu` 统计约 `500.0 Hz`，示例输出包含 `average rate: 500.286`。
- 是否通过：已通过
- 下一步建议：长时间运行时可再次统计频率漂移，同时观察 GUI 站立稳定性。

### VAL-009: `/joint_command` 限幅验证

- 日期：2026-06-20
- 对应迭代：`iter-001`
- 测试环境：仿真
- 测试目标：验证 `/joint_command` 超出 MuJoCo actuator `ctrlrange` 时是否按模型范围限幅
- 测试步骤：
  1. 执行 `source /opt/ros/jazzy/setup.bash && source install/local_setup.bash`
  2. 执行 `cmake --build build/wheel_leg_simulate_ros2 -j2`
  3. 短时启动 `timeout -k 2s 8s build/wheel_leg_simulate_ros2/wheel_leg_simulate transplant/mujoco_win/model/scence.xml`
  4. 执行 `ros2 param set /mujoco_bridge enable_ros_command true`
  5. 执行 `ros2 topic pub --once /joint_command wheel_leg_msgs/msg/JointCommand "{joint_names: ['left_wheel'], efforts: [1000.0]}"`
- 观察结果：
  - bridge 日志输出 `Clamped /joint_command for left_wheel from 1000.000 to 400.000`。

### VAL-010: `rc_ibus_node` 串口接收与 `iBUS` 解包验证

- 日期：2026-06-22
- 对应迭代：`iter-003`
- 测试环境：实机
- 测试目标：验证 Raspberry Pi 上的 `rc_ibus_node` 能通过 `/dev/ttyAMA3` 稳定接收并解包 `FlySky FS-iA6B` 的 `iBUS` 数据
- 测试步骤：
  1. 执行 `sudo bash -lc 'source /opt/ros/jazzy/setup.bash && source /home/tan/Wheel_Leg/install/setup.bash && ros2 run wheel_leg_rc rc_ibus_node'`
  2. 观察节点日志中的 `valid_frames`、`checksum_errors` 与 `sync_loss`
  3. 执行 `sudo bash -lc 'source /opt/ros/jazzy/setup.bash && source /home/tan/Wheel_Leg/install/setup.bash && ros2 topic echo --once /rc/channels_raw'`
- 观察结果：
  - 节点成功打开 `/dev/ttyAMA3`，日志持续输出 `iBUS online`
  - 日志样例显示 `valid_frames=3752 checksum_errors=0 sync_loss=26`
  - `/rc/channels_raw` 可读取到合法 `iBUS` 通道数据，例如 `[1500, 1498, 1000, 1500, 1000, 1000, ...]`
- 是否通过：已通过
- 问题现象：普通用户直接打开 `/dev/ttyAMA3` 会遇到 `Permission denied`，当前实机验证通过 `sudo` 执行
- 可能原因：当前用户尚未加入 `dialout` 组
- 下一步建议：将运行用户加入 `dialout` 后补一次普通用户运行回归，并继续做控制链联调

### VAL-011: 遥控器统一命令映射验证

- 日期：2026-06-22
- 对应迭代：`iter-003`
- 测试环境：实机
- 测试目标：验证真实遥控器通道与 `/cmd_vel`、`/control_mode`、`/body_cmd` 的默认映射是否正确
- 测试步骤：
  1. 在 `rc_ibus_node` 在线状态下运行 `tools/rc_channel_watch.py`，逐一拨动摇杆和开关
  2. 记录每个物理输入对应的 `CH1~CH6` 变化
  3. 通过 `ros2 topic echo /control_mode` 与 `ros2 topic echo /cmd_vel` 验证模式切换、急停和主速度命令
  4. 通过 `ros2 topic echo /body_cmd` 验证机身附加命令
- 观察结果：
  - 已确认通道布局：`CH2` 右摇杆上下、`CH1` 右摇杆左右、`CH3` 左摇杆上下、`CH4` 左摇杆左右、`CH5` 开关、`CH6` 三段开关
  - `CH5` 可在 `disabled` 与 `stand` 之间切换，急停链路正常
  - `CH6` 可在 `stand` 与 `velocity` 之间切换
  - `CH2` 上推时 `/cmd_vel.linear.x = +1`，`CH4` 左打时 `/cmd_vel.angular.z = +1`
  - `CH3` 与 `CH1` 对应的 `/body_cmd.body_height_offset`、`/body_cmd.yaw_rate_assist` 可正常变化
- 是否通过：已通过
- 下一步建议：进入统一命令到控制器/仿真链路的联调，验证 `velocity` 模式下的整机运动响应与 failsafe 退回行为
  - bridge 日志输出 `Applied /joint_command to 1 actuator(s)`。
- 是否通过：已通过
- 问题现象：命令被限幅到 `400.0` 后，MuJoCo 输出 `Nan, Inf or huge value in QACC` unstable warning。
- 可能原因：`400.0` 是当前模型 actuator `ctrlrange` 上限，但该上限命令仍可能使当前站立控制和动力学进入不稳定状态。
- 结论边界：本验证只确认 actuator 写入边界使用 MuJoCo `ctrlrange` 限幅，不证明上限命令下机器人动力学稳定。
- 下一步建议：长时间动力学响应验证应从小力矩命令开始，并把控制稳定性作为独立问题处理。

### VAL-010: `iter-002` 过渡编排最小运行烟测

- 日期：2026-06-21
- 对应迭代：`iter-002`
- 测试环境：仿真
- 测试目标：验证控制编排经正式算法接口适配后，MuJoCo 仿真是否仍能成功启动并走通模型加载与初始化链
- 测试步骤：
  1. 执行 `source /opt/ros/jazzy/setup.bash && source install/setup.bash`
  2. 执行 `colcon build --packages-select wheel_leg_control wheel_leg_simulate --event-handlers console_direct+`
  3. 执行 `timeout 8s build/wheel_leg_simulate/wheel_leg_simulate transplant/mujoco_win/model/scence.xml`
- 观察结果：
  - 程序输出 `MuJoCo version 3.7.0`
  - 程序输出 `wheel_leg simulate ready: nq=17, nv=16, nu=6`
  - 程序输出固定站立目标初始化日志
  - 程序输出 `ROS2 MuJoCo bridge ready: node=mujoco_bridge, topics=/joint_states,/imu,/joint_command, enable_ros_command=true`
  - 在后续将过渡 stand 控制流水线整体迁入正式 `wheel_leg_control/stand_control_pipeline.*` 后，重复执行同一烟测，以上启动日志仍保持一致。
  - 在进一步将控制目标、算法适配器和 `ControlAlgorithmSet` 装配迁入正式 `wheel_leg_control/stand_control_runtime.*` 后，再次重复同一烟测，以上启动日志仍保持一致。
  - 在继续将 legacy PID 实例、初始化配置翻译和每步控制执行收敛到 `transplant/wheel_leg/legacy_stand_control_bridge.*` 后，再次重复同一烟测，以上启动日志仍保持一致。
- 是否通过：已通过
- 结论边界：
  - 本次只验证“能启动、初始化链未断、短时运行未立即崩溃”
  - 本次不验证 GUI 站立观感
  - 本次不验证控制稳定性、长时间动力学表现或 ROS2 命令回归
- 问题现象：直接从未加载工作区环境的 shell 启动时，首次出现 `libwheel_leg_msgs__rosidl_generator_c.so` 缺失；补充 `source /opt/ros/jazzy/setup.bash && source install/setup.bash` 后恢复正常。
- 可能原因：`wheel_leg_simulate` 运行时依赖已安装 ROS2 接口库，裸 shell 未带上工作区动态库搜索路径。
- 清理说明：本次 `timeout` 未自动结束 GUI 进程，后续手动清理进程；这属于 GUI 程序测试清理问题，不代表启动链失败。
- 下一步建议：后续在继续抽离 PID 参数和初始化配置后，再补一次带 topic/命令的短时回归。

### VAL-012: `iter-002` ROS 侧 controller node 控制发布烟测

- 日期：2026-06-21
- 对应迭代：`iter-002`
- 测试环境：仿真 + ROS2
- 测试目标：验证 `wheel_leg_controller_node` 是否已可在 ROS 进程内依据正式 `/robot_state` 直接发布真实 `/joint_command`
- 测试步骤：
  1. 执行 `source /opt/ros/jazzy/setup.bash && source install/setup.bash`
  2. 启动 `build/wheel_leg_simulate_ros2/wheel_leg_simulate transplant/mujoco_win/model/scence.xml`
  3. 启动 `ros2 run wheel_leg_control wheel_leg_controller_node`
  4. 执行 `ros2 topic echo --once /joint_command`
  5. 执行 `ros2 node info /wheel_leg_controller`
- 观察结果：
  - 成功收到一条 `/joint_command`
  - 样本消息中 `joint_names` 包含 `right_hip`、`right_knee`、`left_hip`、`left_knee`、`right_wheel`、`left_wheel`
  - 样本消息中 `efforts` 为非零值，示例包含 `right_knee=3.2935`、`left_knee=2.9819`、`right_wheel=-0.0408`
  - `ros2 node info /wheel_leg_controller` 显示 controller 订阅 `/robot_state`，发布 `/joint_command`
- 是否通过：已通过
- 结论边界：
  - 本次确认 ROS controller node 已不再依赖 `/joint_states`、`/imu` 的二次重建路径，而是直接消费正式控制状态接口
  - 本次不证明长时间闭环稳定性
- 下一步建议：在保持 `enable_ros_command=false` 的前提下，继续验证 `/robot_state` 与 MuJoCo 内部控制状态的一致性，再评估是否重新开启受控接管。

### VAL-013: `iter-002` ROS 控制接管路径烟测

- 日期：2026-06-21
- 对应迭代：`iter-002`
- 测试环境：仿真 + ROS2
- 测试目标：验证仿真侧在检测到活跃 ROS `/joint_command` 时，会旁路内部 legacy stand control，并优先执行 ROS controller 命令
- 测试步骤：
  1. 执行 `source /opt/ros/jazzy/setup.bash && source install/setup.bash`
  2. 执行 `cmake -S transplant/mujoco_win/simulate -B build/wheel_leg_simulate_ros2 -DWHEEL_LEG_ENABLE_ROS2=ON`
  3. 执行 `cmake --build build/wheel_leg_simulate_ros2 -j2`
  4. 在隔离环境中启动仿真：`ROS_DOMAIN_ID=77 timeout -k 2s 15s build/wheel_leg_simulate_ros2/wheel_leg_simulate transplant/mujoco_win/model/scence.xml`
  5. 在相同 `ROS_DOMAIN_ID=77` 下启动 `ros2 run wheel_leg_control wheel_leg_controller_node`
  6. 执行 `ros2 topic info /joint_command -v`
  7. 执行 `ros2 topic echo --once /joint_command`
- 观察结果：
  - `/joint_command` 在隔离 domain 中只有 1 个 publisher `/wheel_leg_controller` 和 1 个 subscriber `/mujoco_bridge`
  - 本条旧记录中的 `100 Hz` 仅反映当时 controller 仍由定时器驱动的过渡实现，不再作为当前闭环基线
  - `/joint_command` 样本消息为 6 关节非零 effort
  - 仿真日志输出 `ROS2 MuJoCo bridge ready: ... enable_ros_command=true`
  - 仿真日志输出 `ROS command takeover active: bypassing legacy stand control.`
  - 随后 bridge 持续输出 `Applied /joint_command to 6 actuator(s)`，说明 actuator 写入已经优先走 ROS 命令通道
  - 停止命令约 `0.2 s` 后，日志输出 `/joint_command timed out after 0.200 s; actuator writes are suspended`，随后输出 `ROS command takeover released: returning to legacy stand control.`
- 是否通过：已通过
- 结论边界：
  - 本次确认“ROS 命令活跃时旁路 legacy 控制”的接管路径已经接通
  - 本次不证明 ROS controller 当前输出已经动力学稳定
- 问题现象：
  - 当前 ROS controller 烟测样本中 wheel effort 可明显超过仿真 actuator `ctrlrange`，bridge 日志出现 `Clamped /joint_command ... to 400.000`
  - 本次运行在约仿真时间 `3.61 s` 出现 `Nan, Inf or huge value in QACC` 不稳定告警
- 可能原因：
  - 接管链路本身已通，但 ROS 侧控制参数和状态重建仍是过渡版本，输出力矩对当前模型偏激进
- 下一步建议：在继续推进 ROS 接管前，优先补 controller 输出限幅/参数收敛与更温和的默认目标，避免把“链路已接通”和“控制已稳定”混为同一验收项。

### VAL-014: `iter-002` `/robot_state` 正式控制状态接口验证

- 日期：2026-06-21
- 对应迭代：`iter-002`
- 测试环境：仿真 + ROS2
- 测试目标：验证 MuJoCo 内部稳定控制链使用的 `StandControlState` 已通过正式 `/robot_state` 接口发布给 ROS，并且消息映射无字段漂移
- 测试步骤：
  1. 执行 `source /opt/ros/jazzy/setup.bash && source install/setup.bash`
  2. 执行 `colcon build --packages-select wheel_leg_msgs wheel_leg_bridge wheel_leg_control wheel_leg_sim --event-handlers console_direct+`
  3. 执行 `cmake -S transplant/mujoco_win/simulate -B build/wheel_leg_simulate_ros2 -DWHEEL_LEG_ENABLE_ROS2=ON`
  4. 执行 `cmake --build build/wheel_leg_simulate_ros2 -j2`
  5. 在隔离环境中启动仿真：`ROS_DOMAIN_ID=83 timeout -k 2s 10s build/wheel_leg_simulate_ros2/wheel_leg_simulate transplant/mujoco_win/model/scence.xml`
  6. 执行 `ros2 interface show wheel_leg_msgs/msg/StandControlState`
  7. 执行 `ros2 topic info /robot_state -v`
  8. 执行 `ros2 topic echo --once /robot_state`
- 观察结果：
  - `wheel_leg_msgs/msg/StandControlState` 字段包含 body、left_leg、right_leg 的平铺控制状态量
  - `/robot_state` 类型为 `wheel_leg_msgs/msg/StandControlState`，publisher 为 `/mujoco_bridge`
  - `/robot_state` 样本中收到非零控制状态，例如 `body_velocity=0.0291`、`left_leg_length=0.24967`、`right_phi=1.67650`
  - 仿真日志输出 `Robot state round-trip verified for /robot_state publish path.`
- 是否通过：已通过
- 结论边界：
  - 本次确认 `/robot_state` 已从 MuJoCo 内部控制状态直接发布，并通过发布前 round-trip 校验确认字段映射一致
  - 本次不证明 DDS 接收样本与日志中的首个样本必须来自同一仿真时刻
  - 本次不重新开启 ROS actuator takeover
- 下一步建议：继续在 `enable_ros_command=false` 条件下积累 `/robot_state` 样本与控制输出对照，再决定是否恢复受控接管实验。

### VAL-015: `iter-002` `/robot_state` 驱动下的非接管控制输出幅值验证

- 日期：2026-06-21
- 对应迭代：`iter-002`
- 测试环境：仿真 + ROS2
- 测试目标：验证在 `enable_ros_command=false` 条件下，ROS controller 基于正式 `/robot_state` 运行时，其 `/joint_command` 输出幅值是否明显收敛，避免重现此前 takeover 烟测中的大力矩异常
- 测试步骤：
  1. 执行 `source /opt/ros/jazzy/setup.bash && source install/setup.bash`
  2. 在隔离环境中启动仿真：`ROS_DOMAIN_ID=84 timeout -k 2s 12s build/wheel_leg_simulate_ros2/wheel_leg_simulate transplant/mujoco_win/model/scence.xml`
  3. 在相同 `ROS_DOMAIN_ID=84` 下启动 `ros2 run wheel_leg_control wheel_leg_controller_node`
  4. 执行短时订阅统计脚本，采集约 `4 s` `/joint_command` 样本
  5. 执行 `ros2 topic hz /robot_state`
- 观察结果：
  - 本条旧记录中的 `/robot_state` `100 Hz` 结果仅对应旧版 bridge 发布节拍，不再作为当前闭环基线
  - 约 `4 s` 采样窗口内，`/joint_command` 共收到 `401` 条样本
  - `joint_command_abs_max` 统计结果为：
    - `left_hip=0.3563`
    - `left_knee=3.3932`
    - `left_wheel=0.2610`
    - `right_hip=0.3507`
    - `right_knee=2.9687`
    - `right_wheel=0.2351`
- 是否通过：已通过
- 结论边界：
  - 本次确认在不接管 actuator 的条件下，ROS controller 基于 `/robot_state` 的控制输出量级已明显低于此前 takeover 烟测中的异常大力矩
  - 本次不证明重新开启 `enable_ros_command=true` 后闭环一定稳定
- 结论提示：
  - 当前更像是“状态接口已对齐后，controller 开环输出已基本收敛”
  - 后续若重新开启接管仍不稳，问题更可能集中在接管时序、参数初始化或闭环耦合，而不是 `/robot_state` 字段映射
- 下一步建议：在继续保持 `/robot_state` 为主状态接口的前提下，进入一次“小心限幅 + 短时接管”的恢复验证。

### VAL-016: `iter-002` 限幅后的短时接管恢复验证

- 日期：2026-06-21
- 对应迭代：`iter-002`
- 测试环境：仿真 + ROS2
- 测试目标：验证在 controller 发布前增加保守限幅后，短时重新开启 `enable_ros_command=true` 是否能避免立即失稳
- 测试步骤：
  1. 在 `wheel_leg_controller_node` 中启用发布前限幅，默认参数：
     - `hip_effort_limit=50`
     - `knee_effort_limit=50`
     - `wheel_effort_limit=20`
  2. 执行 `source /opt/ros/jazzy/setup.bash && source install/setup.bash`
  3. 在隔离环境中启动仿真：`ROS_DOMAIN_ID=86 timeout -k 2s 20s build/wheel_leg_simulate_ros2/wheel_leg_simulate transplant/mujoco_win/model/scence.xml`
  4. 在相同 `ROS_DOMAIN_ID=86` 下启动 `ros2 run wheel_leg_control wheel_leg_controller_node`
  5. 确认节点存在后执行 `ros2 param set /mujoco_bridge enable_ros_command true`
  6. 执行 `ros2 topic echo --once /joint_command`
  7. 观察仿真日志中的 takeover、clamp 和稳定性告警
- 观察结果：
  - controller 日志首先输出 `Clamped controller output for right_knee from -94.173 to -50.000 before publishing /joint_command`
  - takeover 打开后，bridge 持续输出 `Applied /joint_command to 6 actuator(s)`
  - 本轮未出现 bridge 侧 `Clamped /joint_command ... to 400.000`，说明 MuJoCo actuator `ctrlrange` 限幅不再是首要问题
  - 采样到的 `/joint_command` 样本在约 `16.0 s` 时已顶到发布前限幅边界：
    - `right_hip=-50`
    - `right_knee=-50`
    - `left_hip=-50`
    - `left_knee=-50`
    - `right_wheel=-20`
    - `left_wheel=-20`
  - 随后仿真输出 `Nan, Inf or huge value in QACC at DOF 7. Time = 16.0020.`
- 是否通过：未通过
- 结论边界：
  - 本次确认“正式 `/robot_state` + 发布前限幅”仍不足以保证 takeover 闭环稳定
  - 本次同时确认当前失稳已不再主要表现为 actuator `ctrlrange` 超限，而更像是 controller 启动/接管阶段的控制输出整体饱和
- 当前判断：
  - `/robot_state` 接口映射本身已通过
  - 问题已进一步收敛到 controller 启动瞬态、目标初始化、接管时序或控制参数耦合，而不是 ROS 状态重建漂移
- 下一步建议：
  - 在重新做 takeover 前，优先加入 controller “warm-up / settle” 保护：
    - 刚启动或刚接管的前若干周期内不直接发布控制命令，或只允许更小限幅
    - 等 `/robot_state` 连续稳定一段时间后，再进入正常控制输出
  - 如需更快定位，可同时记录 takeover 前 `0.5 s` 内的 `StandControlState` 与 `/joint_command` 序列，确认是哪一项状态或哪一路力矩最先把控制推到饱和。

### VAL-017: `iter-002` 状态 topic 提升到每步 `500 Hz` 验证

- 日期：2026-06-21
- 对应迭代：`iter-002`
- 测试环境：仿真 + ROS2
- 测试目标：验证 `/joint_states`、`/imu` 和 `/robot_state` 已去掉旧版 `100 Hz` 限流，改为每个 MuJoCo step 发布
- 测试步骤：
  1. 执行 `source /opt/ros/jazzy/setup.bash && source install/setup.bash`
  2. 执行 `cmake --build build/wheel_leg_simulate_ros2 -j2`
  3. 在隔离环境中启动仿真：`ROS_DOMAIN_ID=92 timeout -k 2s 20s build/wheel_leg_simulate_ros2/wheel_leg_simulate transplant/mujoco_win/model/scence.xml`
  4. 分别执行：
     - `ROS_DOMAIN_ID=92 bash -lc 'sleep 2; timeout 4s ros2 topic hz /joint_states'`
     - `ROS_DOMAIN_ID=92 bash -lc 'sleep 2; timeout 4s ros2 topic hz /imu'`
     - `ROS_DOMAIN_ID=92 bash -lc 'sleep 2; timeout 4s ros2 topic hz /robot_state'`
- 观察结果：
  - `/joint_states` 统计约 `499.997 Hz`
  - `/imu` 统计约 `500.286 Hz`
  - `/robot_state` 统计约 `499.922 Hz`
- 是否通过：已通过
- 结论边界：
  - 本次确认状态发布链已不再保留 `100 Hz` / `500 Hz` 混频
  - 统计命令在 `timeout` 结束时伴随 ROS2 context 清理告警，不影响频率结论

### VAL-018: `iter-002` controller 状态驱动与 dt 验证

- 日期：2026-06-21
- 对应迭代：`iter-002`
- 测试环境：仿真 + ROS2
- 测试目标：验证 ROS controller 已从 wall timer 切到 `/robot_state` 驱动，并使用消息时间戳计算 `dt`
- 测试步骤：
  1. 执行 `source /opt/ros/jazzy/setup.bash && source install/setup.bash`
  2. 在隔离环境中启动仿真：`ROS_DOMAIN_ID=93 timeout -k 2s 20s build/wheel_leg_simulate_ros2/wheel_leg_simulate transplant/mujoco_win/model/scence.xml`
  3. 在相同 `ROS_DOMAIN_ID=93` 下启动 `timeout -k 2s 12s ros2 run wheel_leg_control wheel_leg_controller_node`
  4. 观察 controller 启动日志
  5. 在另一轮隔离环境中执行 `ROS_DOMAIN_ID=94 bash -lc 'sleep 2; ros2 node info /wheel_leg_controller'`
- 观察结果：
  - controller 启动时输出 `Parameter publish_rate_hz is ignored`
  - controller 连续输出 5 条 `Controller dt sample` 日志，均为 `0.002000000 s`
  - `ros2 node info /wheel_leg_controller` 显示 controller 订阅 `/robot_state`，发布 `/joint_command`
  - 运行过程中出现一次 `Ignoring /robot_state sample with out-of-range dt=0.004000000` 告警，说明异常采样已按新规则被丢弃而非继续参与控制
- 是否通过：已通过
- 结论边界：
  - 本次确认 controller 真实控制节拍已来源于 `/robot_state` 时间戳，而不是 `publish_rate_hz` wall timer
  - 本次未单独形成 `/joint_command` 频率的新最终结论；后续 takeover 验证时可继续补采

### VAL-019: `iter-002` `100 Hz` 降采样接管稳定性实验

- 日期：2026-06-21
- 对应迭代：`iter-002`
- 测试环境：仿真 + ROS2
- 测试目标：验证在保持当前控制参数不变的前提下，将 ROS controller 主状态输入从 `500 Hz` 降采样到 `100 Hz` 后是否仍能稳定接管
- 测试步骤：
  1. 执行 `source /opt/ros/jazzy/setup.bash && source install/setup.bash`
  2. 准备临时降采样 relay，将 `/robot_state` 每 5 帧转发 1 帧到 `/robot_state_100hz`
  3. 在隔离环境中启动仿真：`ROS_DOMAIN_ID=97 timeout -k 2s 35s build/wheel_leg_simulate_ros2/wheel_leg_simulate transplant/mujoco_win/model/scence.xml`
  4. 在相同 `ROS_DOMAIN_ID=97` 下启动 relay
  5. 在相同 `ROS_DOMAIN_ID=97` 下启动 controller，并将 `/robot_state` remap 到 `/robot_state_100hz`
  6. 执行 `ros2 param set /mujoco_bridge enable_ros_command true`
  7. 执行：
     - `ROS_DOMAIN_ID=97 bash -lc 'sleep 2; timeout 4s ros2 topic hz /robot_state_100hz'`
     - `ROS_DOMAIN_ID=97 bash -lc 'sleep 2; timeout 4s ros2 topic hz /joint_command'`
  8. 观察 controller 和仿真日志
- 观察结果：
  - controller 连续输出 5 条 `Controller dt sample` 日志，均为 `0.010000000 s`
  - `/robot_state_100hz` 统计约 `99.996 Hz`
  - `/joint_command` 统计约 `100.017 Hz`
  - takeover 打开后，bridge 持续输出 `Applied /joint_command to 6 actuator(s)`
  - controller 日志出现 `Clamped controller output for right_hip from 57.884 to 50.000 before publishing /joint_command`
  - 随后仿真输出 `WARNING: Nan, Inf or huge value in QACC at DOF 0. The simulation is unstable. Time = 24.6900.`
- 是否通过：未通过
- 结论边界：
  - 本次确认“只把主状态输入降到 `100 Hz`、其余参数不重整定”的方案不能稳定接管
  - 本次实验仅用于排除当前参数下的 `100 Hz` 直降方案，不证明 `100 Hz` 在重新调参后绝对不可行
- 当前决策：
  - 主线继续保持 `500 Hz` 状态发布与 `0.002 s` controller dt 守门范围
  - 当前不再沿 `100 Hz` 路线继续调参或推进接管验收

### VAL-020: `iter-002` `500 Hz` 主线下的短时 takeover 复测

- 日期：2026-06-21
- 对应迭代：`iter-002`
- 测试环境：仿真 + ROS2
- 测试目标：验证在恢复 `500 Hz` 主线、`/robot_state` 驱动和 `0.002 s` dt 守门范围后，短时 actuator takeover 是否仍会立即重现此前的饱和和失稳
- 测试步骤：
  1. 执行 `source /opt/ros/jazzy/setup.bash && source install/setup.bash`
  2. 删除旧 trace：`rm -f /tmp/wheel_leg_takeover_trace.csv`
  3. 在隔离环境中启动仿真：`ROS_DOMAIN_ID=99 timeout -k 2s 30s build/wheel_leg_simulate_ros2/wheel_leg_simulate transplant/mujoco_win/model/scence.xml`
  4. 在相同 `ROS_DOMAIN_ID=99` 下启动 `ros2 run wheel_leg_control wheel_leg_controller_node`
  5. 执行 `ros2 param set /mujoco_bridge enable_ros_command true`
  6. 短时观察仿真与 controller 日志，并执行 `ros2 topic echo --once /joint_command`
- 观察结果：
  - controller 连续输出 5 条 `Controller dt sample` 日志，均为 `0.002000000 s`
  - takeover 打开后，bridge 持续输出 `Applied /joint_command to 6 actuator(s)`
  - 本轮未出现 controller 发布前限幅告警，因此未生成 `/tmp/wheel_leg_takeover_trace.csv`
  - 本轮未出现 `Nan, Inf or huge value in QACC` 不稳定告警
  - `/joint_command` 样本仍为非零控制输出，例如 `right_knee=4.0719`、`left_knee=3.2007`
  - 停止 controller 后，bridge 正常输出 `/joint_command timed out after 0.200 s; actuator writes are suspended`，并释放 takeover
- 是否通过：已通过
- 结论边界：
  - 本次只确认当前 `500 Hz` 主线下，短时 takeover 已不再立即重现此前“快速撞限幅并触发 QACC”的现象
  - 本次尚不证明长时间闭环完全稳定，也不证明所有参数都已收敛到最终可交付状态
- 下一步建议：
  - 继续做更长时间窗口的 takeover 保持验证
  - 若长时复测再次出现饱和，再依赖 `/tmp/wheel_leg_takeover_trace.csv` 精确定位最先冲高的状态和力矩通道

### VAL-021: `iter-002` `10 s+` takeover 保持验证

- 日期：2026-06-21
- 对应迭代：`iter-002`
- 测试环境：仿真 + ROS2
- 测试目标：验证在当前 `500 Hz` 主线、正式 `/robot_state` 驱动和 `0.002 s` dt 守门范围下，ROS actuator takeover 持续 `10 s` 以上时仍不重现限幅或 `QACC` 不稳定告警
- 测试步骤：
  1. 执行 `source /opt/ros/jazzy/setup.bash && source install/setup.bash`
  2. 删除旧 trace：`rm -f /tmp/wheel_leg_takeover_trace.csv`
  3. 在隔离环境中启动仿真：`ROS_DOMAIN_ID=109 timeout -k 2s 22s build/wheel_leg_simulate/wheel_leg_simulate transplant/mujoco_win/model/scence.xml`
  4. 在相同 `ROS_DOMAIN_ID=109` 下启动 controller：`timeout -k 2s 16s ros2 run wheel_leg_control wheel_leg_controller_node`
  5. 执行 `ros2 param set /mujoco_bridge enable_ros_command true`
  6. 执行 `timeout 4s ros2 topic echo --once /joint_command`
  7. 观察仿真日志中的 takeover、`Applied /joint_command`、timeout 与稳定性告警，并检查 `/tmp/wheel_leg_takeover_trace.csv` 是否生成
- 观察结果：
  - 仿真日志输出 `wheel_leg simulate ready`、`ROS2 MuJoCo bridge ready` 和 `Robot state round-trip verified for /robot_state publish path.`
  - controller 连续输出 5 条 `Controller dt sample` 日志，均为 `0.002000000 s`
  - controller 期间出现 1 次 `Ignoring /robot_state sample with out-of-range dt=0.004000000` 告警；样本被按守门规则丢弃，随后闭环继续稳定运行
  - takeover 生效后，bridge 首条 `Applied /joint_command to 6 actuator(s)` 时间戳为 `1782047663.978192639`，最后一条为 `1782047676.403446887`，持续约 `12.43 s`
  - 该窗口内 bridge 共记录 `6162` 次 `Applied /joint_command to 6 actuator(s)`，`/joint_command` 样本仍为非零控制输出，例如 `right_knee=4.0625`、`left_knee=3.1571`
  - 本轮未出现 controller 发布前限幅告警，因此未生成 `/tmp/wheel_leg_takeover_trace.csv`
  - 本轮未出现 `Nan, Inf or huge value in QACC`、`unstable` 或 bridge 侧 `Clamped /joint_command` 告警
  - controller 停止后约 `0.2 s`，bridge 正常输出 `/joint_command timed out after 0.200 s; actuator writes are suspended`，随后释放 takeover
- 是否通过：已通过
- 结论边界：
  - 本次确认当前主线配置下，ROS actuator takeover 已可稳定保持超过 `10 s`
  - 本次仍不证明“无限时长”或“最终参数完全定型”，但已经满足本轮迁移收口与闭环验收需要
- 下一步建议：
  - 本轮任务可在此结项
  - 后续若继续优化，可把关注点转向参数/目录长期整理，而非继续阻塞在本轮 takeover 基线验证上

### VAL-011: `/imu` 坐标语义验证

- 日期：2026-06-20
- 对应迭代：`iter-001`
- 测试环境：模型检查 + 仿真
- 测试目标：验证 `/imu` 当前“不做轴重映射，仅做四元数顺序重排”的坐标语义是否与 MuJoCo 模型一致
- 测试步骤：
  1. 检查 `transplant/mujoco_win/model/wheel_leg.xml` 中 `base_frame` 的挂载关系。
  2. 检查 `base_accel`、`base_gyro`、`base_quat` 的传感器定义。
  3. 检查 `transplant/mujoco_win/simulate/wheel_leg/ros2_bridge.cc` 中 `/imu` 填充逻辑。
  4. 短时启动仿真并执行 `ros2 topic echo --once /imu`。
- 模型观察结果：
  - `base_frame` site 直接挂在 `base_body` 上，`pos="0 0 0"`，未设置额外 `euler` 或 `quat`。
  - `base_accel`、`base_gyro`、`base_quat` 均直接以 `base_frame` 为参考。
  - bridge 对 `base_gyro`、`base_accel` 按分量直接发布，只对 `base_quat` 做 `w,x,y,z -> x,y,z,w` 顺序重排。
- 运行时观察结果：
  - `/imu.header.frame_id` 为 `base_link`。
  - 样本消息中 `orientation` 接近单位四元数。
  - 样本消息中 `linear_acceleration.z` 约为 `9.75`，`x/y` 相对更小，符合近直立机体系下重力主要落在 `+z` 方向的表现。
- 是否通过：已通过
- 结论：当前 `/imu` 使用 `base_link` 机体系直接发布、无需额外轴重映射的实现与模型挂载关系一致。
- 下一步建议：若后续状态估计或可视化对坐标语义提出更严格要求，可在带显示环境中补充姿态动作验证。

### VAL-022: `iter-003` 遥控器统一命令接入仿真控制链验证

- 日期：2026-06-22
- 对应迭代：`iter-003`
- 测试环境：实机 + 仿真 + ROS2
- 测试目标：验证真实遥控器经 `rc_ibus_node` 发布统一命令后，是否可驱动 `wheel_leg_controller_node` 与 `mujoco_bridge` 完成仿真控制链联调，并确认 `control_mode` 与主速度摇杆映射在仿真侧保持一致
- 测试步骤：
  1. 执行 `source /opt/ros/jazzy/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=120`
  2. 启动仿真：`build/wheel_leg_simulate/wheel_leg_simulate transplant/mujoco_win/model/scence.xml`
  3. 启动 controller：`ros2 run wheel_leg_control wheel_leg_controller_node`
  4. 执行 `ros2 param set /mujoco_bridge enable_ros_command true`
  5. 启动遥控输入：`ros2 run wheel_leg_rc rc_ibus_node`
  6. 观察 `/rc/status`、`/control_mode`、`/cmd_vel`，并现场拨动 `CH5`、`CH6`、`CH2`、`CH4`
- 观察结果：
  - 仿真日志出现 `ROS2 MuJoCo bridge ready`，controller 日志出现 `Controller dt sample ... 0.002...`，说明 `/robot_state -> controller -> /joint_command` 链路正常
  - `mujoco_bridge` 日志出现 `Applied /joint_command to 6 actuator(s)`，说明遥控统一命令已真实进入仿真 actuator 接管链
  - 测试过程中出现过一次 `/joint_command timed out after 0.200 s; actuator writes are suspended`，同时 controller 短时进入 `rc_status_timeout` fallback，随后恢复到 `disabled` 正常模式
  - `/rc/status` 观察正常，链路恢复后可持续工作
  - `CH5` 急停开关对应 `disabled <-> stand` 切换，`CH6` 三段开关对应 `stand <-> velocity` 切换，符合既定映射
  - 在 `velocity` 模式下，`CH2` 仅驱动 `/cmd_vel.linear.x` 正负变化，`CH4` 仅驱动 `/cmd_vel.angular.z` 正负变化，方向与预期一致
- 是否通过：已通过
- 结论边界：
  - 本次确认“真实遥控器 -> 统一命令 -> ROS controller -> MuJoCo bridge”联调链已接通，且关键模式/摇杆语义在仿真侧未漂移
  - 本次尚未验证长时间 `velocity` 模式下的整机动力学表现，也未验证断链后整机姿态恢复过程
- 下一步建议：
  - 继续在仿真中验证 `stand` 与 `velocity` 模式下的整机行为
  - 补一次更明确的 failsafe 场景测试，确认断链或停发后模式退回与机器人响应是否符合预期

## 5. 待确认问题

当前迭代暂无待确认问题。
