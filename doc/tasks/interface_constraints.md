# 模块任务：接口与参数约束

## 1. 模块名称

接口与参数约束模块。

## 2. 当前迭代目标

集中管理 `iter-001` 中已确认和待确认的接口、参数、频率、命名和验证约束，避免实现时临时猜测。

## 3. 任务 checklist

- `[ ]` 阅读 `doc/detail.md` 中接口与参数约束模块设计。
- `[~]` 记录已确认接口。
- `[x]` 确认 `/joint_command` 消息类型：使用自定义消息。
- `[x]` 确认 `/joint_command` 映射方式：使用关节名映射。
- `[x]` 确认 `/joint_command` 自定义消息字段。
- `[x]` 确认关节命名表。
- `[x]` 确认 actuator 映射表。
- `[x]` 确认状态发布频率：`100 Hz`。
- `[x]` 确认命令处理频率：每个 MuJoCo step 处理一次最新有效命令。
- `[x]` 确认 `/imu` frame id 和坐标系定义。
- `[x]` 确认时间戳策略：状态消息使用 MuJoCo 仿真时间 `d->time`。
- `[x]` 确认是否需要记录到 `doc/decisions.md`。
- `[x]` 根据确认结果同步更新相关任务文档。

## 4. 每个任务的输入

- `doc/proposal.md`
- `doc/architecture.md`
- `doc/iterations/iter-001.md`
- `doc/detail.md`
- 用户确认结果。

## 5. 每个任务的输出

- 当前迭代已确认接口清单。
- 当前迭代待确认问题清单。
- 需要同步到任务文档或决策文档的设计选择。

## 6. 完成标准

- 已确认接口与 `doc/detail.md` 一致。
- 未确认接口明确标为 `[?] 待确认`。
- 不把未确认的消息类型、频率、映射表或限幅写成实现要求。
- 重要设计选择有明确记录位置。

## 7. 验证方法

文档验证：

- 检查 `doc/tasks/progress.md` 中待确认项是否完整。
- 检查各模块任务文件中不确定问题是否与 `doc/detail.md` 一致。
- 如果用户确认了关键接口，检查相关文档是否同步更新。

实机验证：

- 本模块是文档约束模块，不进行实机验证。

## 8. 当前状态

`[x] 已确认`

当前已确认：

- `/joint_states` 使用 `sensor_msgs/msg/JointState`。
- `/imu` 使用 `sensor_msgs/msg/Imu`。
- `/joint_command` 使用自定义消息 `wheel_leg_msgs/msg/JointCommand`。
- `/joint_command` 字段为 `std_msgs/Header header`、`string[] joint_names`、`float64[] efforts`。
- `/joint_command` 使用 `joint_names` 和 `efforts` 一一对应的关节名映射方式。
- `/joint_command` 的 `efforts` 单位为 Nm。
- 当前关节集合按左、右两侧组织，每侧包含髋部、膝部和轮部。
- ROS 侧 canonical 关节命名表和顺序：
  - `left_hip`
  - `left_knee`
  - `left_wheel`
  - `right_hip`
  - `right_knee`
  - `right_wheel`
- MuJoCo actuator 映射表：
  - `left_hip` -> `left_hip_joint` -> `left_hip_motor`
  - `left_knee` -> `left_knee_joint` -> `left_knee_motor`
  - `left_wheel` -> `left_wheel_joint` -> `left_wheel_motor`
  - `right_hip` -> `right_hip_joint` -> `right_hip_motor`
  - `right_knee` -> `right_knee_joint` -> `right_knee_motor`
  - `right_wheel` -> `right_wheel_joint` -> `right_wheel_motor`
- MuJoCo 中的 `left_connect2_joint`、`left_calf_joint`、`right_connect2_joint`、`right_calf_joint` 当前不进入 `/joint_command`。
- `/joint_states` 和 `/imu` 状态发布频率为 `100 Hz`。
- `/joint_command` 命令处理跟随 MuJoCo step，每个 step 处理一次最新有效命令。
- `/joint_states.header.stamp` 和 `/imu.header.stamp` 使用 MuJoCo 仿真时间 `d->time` 转换为 ROS time。
- `/imu` 使用 MuJoCo sensor：`base_quat`、`base_gyro`、`base_accel`。
- `/imu.header.frame_id` 使用 `base_link`。
- `/imu.orientation` 将 MuJoCo `base_quat` 的 `w,x,y,z` 顺序转换为 ROS `x,y,z,w`。
- `/imu.angular_velocity` 使用 `base_gyro`，`/imu.linear_acceleration` 使用 `base_accel`。
- `/imu` 当前不做轴重映射，按 `base_link` 机体系直接发布。
- `/imu` covariance 当前全 0，表示未知。
- ROS2 节点名：`mujoco_bridge`。
- 当前不使用 topic namespace，使用全局 topic：`/joint_states`、`/imu`、`/joint_command`。
- ROS spin 策略：同线程，在 MuJoCo step 边界调用 `rclcpp::spin_some` 或等价非阻塞处理。
- `/joint_states.effort` 当前不填充，只发布 `name`、`position`、`velocity`。
- ROS2 命令默认不接管 actuator；显式开关 `enable_ros_command=false` 时保持现有站立控制，`true` 时 ROS `/joint_command` 在 step 末尾覆盖对应 actuator。
- `/joint_command` 超时时间为 `0.2 s`；超时后停止应用 ROS 命令，回到不接管 actuator 行为；不自动清零 actuator。
- 命令限幅使用 MuJoCo actuator `ctrlrange`，不在代码中自定义固定限幅数值。
- 无效 `/joint_command` 整条拒绝，不写入任何 actuator；无效条件包括 `joint_names` 与 `efforts` 长度不一致、未知 joint name、NaN 或 Inf。
- 消息包：创建 `wheel_leg_msgs`，包含 `msg/JointCommand.msg`。

当前待确认：

- 暂无接口约束阻塞项。
