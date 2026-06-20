# 技术决策记录

## DEC-001: `/joint_command` 使用自定义消息

- 日期：2026-06-20
- 背景：`iter-001` 需要为 ROS2 到 MuJoCo actuator 的命令通路确认 `/joint_command` 消息类型，后续关节命令订阅模块和 actuator 写入模块都依赖该接口。
- 可选方案：`std_msgs/msg/Float64MultiArray`、`sensor_msgs/msg/JointState`、自定义消息。
- 最终选择：`/joint_command` 使用自定义消息 `wheel_leg_msgs/msg/JointCommand`，字段为 `std_msgs/Header header`、`string[] joint_names`、`float64[] efforts`。
- 选择理由：用户确认使用方案 A；当前机器人关节按左、右两侧组织，每侧包含髋部、膝部和轮部，使用 `joint_names` 与 `efforts` 一一对应的关节名映射可避免过早依赖固定数组顺序。`efforts` 当前只表达力矩命令，单位为 Nm。
- 影响范围：关节命令订阅模块、actuator 命令写入模块、接口与参数约束模块、后续 ROS2 消息包设计。
- 后续是否可修改：可修改。命令超时、限幅策略和 ROS2 命令与现有 MuJoCo 内部控制逻辑的优先级已在 DEC-005 中确认；若仿真验证发现不合适，应更新决策和验证记录。

## DEC-002: ROS 侧关节命名和 MuJoCo actuator 映射

- 日期：2026-06-20
- 背景：`/joint_command` 使用关节名映射，需要确认 ROS 侧 canonical 关节名，以及这些名称到 MuJoCo joint 和 actuator 的映射关系。用户说明仿真命名较粗略，可以重新命名。
- 可选方案：直接使用 MuJoCo 名称如 `left_hip_joint`；使用 ROS 侧短名如 `left_hip` 并维护映射表。
- 最终选择：ROS 侧使用 `left_hip`、`left_knee`、`left_wheel`、`right_hip`、`right_knee`、`right_wheel`，并映射到 MuJoCo 中的 `left_hip_motor`、`left_knee_motor`、`left_wheel_motor`、`right_hip_motor`、`right_knee_motor`、`right_wheel_motor`。
- 选择理由：ROS 侧短名更稳定、更接近机器人语义，不把 MuJoCo XML 的粗略命名直接泄漏到上层接口；映射表仍保留对当前仿真模型的精确对应。
- 影响范围：`/joint_states.name`、`/joint_command.joint_names`、actuator 写入模块、后续调试命令。
- 后续是否可修改：可修改。若真机关节命名、方向或 actuator 语义变化，应同步更新映射表和验证记录。

## DEC-003: 状态发布频率、命令处理频率和时间戳策略

- 日期：2026-06-20
- 背景：`iter-001` 后续代码实现需要确认 `/joint_states`、`/imu` 的发布节奏，以及 `/joint_command` 在 MuJoCo 仿真周期中的处理边界。
- 可选方案：状态发布频率与 MuJoCo step 完全一致；固定状态发布频率；使用 ROS 当前时间；使用 MuJoCo 仿真时间。
- 最终选择：`/joint_states` 和 `/imu` 按 `100 Hz` 发布；`/joint_command` 每个 MuJoCo step 处理一次最新有效命令；状态消息时间戳使用 MuJoCo 仿真时间 `d->time` 转换为 ROS time。
- 选择理由：`100 Hz` 足够支持当前调试和初步控制接入，不会让 ROS CLI 过度刷屏；命令处理跟随 MuJoCo step 可保持仿真执行边界清晰；状态消息使用仿真时间便于 rosbag、状态估计和回放。
- 影响范围：MuJoCo 桥接节点模块、关节状态发布模块、IMU 状态发布模块、关节命令订阅模块和后续验证步骤。
- 后续是否可修改：可修改。若后续控制器需要更高频率或实机部署需要同步真实硬件时钟，应重新确认并更新本文档。

## DEC-004: `/imu` sensor、frame id 和坐标系策略

- 日期：2026-06-20
- 背景：`/imu` 发布模块需要确认 MuJoCo sensor 名称、ROS `frame_id`、四元数顺序和 covariance 策略，避免实现时临时猜测。
- 可选方案：直接使用 MuJoCo sensor/site 名称作为 frame id；使用 ROS 侧语义名称 `base_link`；对 MuJoCo sensor 做轴重映射；先按机体系直通发布。
- 最终选择：MuJoCo sensor 使用 `base_quat`、`base_gyro`、`base_accel`；`/imu.header.frame_id` 使用 `base_link`；`base_quat` 从 MuJoCo `w,x,y,z` 转为 ROS `x,y,z,w`；当前不做轴重映射，按 `base_link` 机体系直接发布；covariance 全 0，表示未知。
- 选择理由：当前模型中 IMU sensor 均挂在 `base_frame` site，`base_link` 更适合作为 ROS 侧稳定语义名称；MuJoCo 已明确使用 `w,x,y,z` 顺序，而 ROS `sensor_msgs/msg/Imu` 使用 `x,y,z,w`；当前迭代先打通桥接，坐标轴语义通过仿真验证再调整。
- 影响范围：IMU 状态发布模块、MuJoCo 桥接节点模块、后续状态估计和验证步骤。
- 后续是否可修改：可修改。若仿真验证发现姿态、角速度或线加速度方向与 ROS 约定不一致，应记录到 `doc/validation.md` 后再调整坐标转换。

## DEC-005: 桥接节点运行策略和 ROS2 命令接管策略

- 日期：2026-06-20
- 背景：进入代码阶段前，需要确认节点名、topic namespace、ROS spin 策略、`/joint_states.effort`、命令接管、超时、限幅和无效命令处理策略。
- 可选方案：使用 namespace 或全局 topic；ROS 单独线程 spin 或 MuJoCo step 同线程处理；默认接管 actuator 或显式开关接管；代码自定义限幅或使用 MuJoCo `ctrlrange`。
- 最终选择：节点名为 `mujoco_bridge`；当前不使用 namespace，直接使用 `/joint_states`、`/imu`、`/joint_command`；ROS spin 与 MuJoCo step 同线程，在 step 边界调用 `rclcpp::spin_some` 或等价非阻塞处理；`/joint_states.effort` 当前不填充；使用 `enable_ros_command` 控制 ROS 命令是否接管 actuator，默认 `false`；`true` 时 ROS `/joint_command` 在 step 末尾覆盖对应 actuator；命令超时为 `0.2 s`，超时后停止应用 ROS 命令并回到不接管 actuator 行为，不自动清零 actuator；命令限幅使用 MuJoCo actuator `ctrlrange`；无效命令整条拒绝，不写入任何 actuator。
- 选择理由：全局 topic 和同线程 spin 是当前最小实现；默认不接管 actuator 可避免破坏现有站立控制；显式接管开关便于仿真验证；超时后不自动清零可以避免突变；使用 MuJoCo `ctrlrange` 避免在代码中发明新限幅。
- 影响范围：MuJoCo 桥接节点模块、关节状态发布模块、关节命令订阅模块、actuator 命令写入模块和仿真验证步骤。
- 后续是否可修改：可修改。若仿真验证发现同线程 spin 影响实时性，或控制器外移后需要默认接管 actuator，应更新决策和验证记录。
