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

## DEC-006: 正式工程结构采用 ROS2 包化，并将 `transplant/` 定位为迁移参考区

- 日期：2026-06-20
- 背景：`iter-001` 已完成最小 bridge 闭环，但当前运行时、控制流程和算法引用仍大量依赖 `transplant/mujoco_win/simulate`。用户明确后续要删除这部分移植参考代码，因此需要先固定正式工程结构。
- 可选方案：继续围绕 `transplant/` 增量叠加；先做普通目录模块化；直接按 ROS2 包化建立正式代码根。
- 最终选择：正式工程结构固定为 `src/wheel_leg_msgs`、`src/wheel_leg_bridge`、`src/wheel_leg_control`、`src/wheel_leg_sim`、`src/wheel_leg_common` 五个 ROS2 包；`transplant/` 仅作为迁移来源和参考区，不再作为长期运行时归宿。
- 选择理由：后续系统目标明确是 ROS2 + MuJoCo + STM32 的长期工程，按 ROS2 包化建立正式代码根更利于节点边界、依赖方向和后续真机落地；继续围绕 `transplant/` 组织会把临时移植结构固化为长期包袱。
- 影响范围：`doc/architecture.md`、`doc/detail.md`、后续包创建顺序、从 `wheel_leg_hooks.cc` 和 `tools/*.cc` 的迁移归属。
- 后续是否可修改：可修改。若后续发现某些能力更适合并包或拆包，应在不破坏当前职责边界的前提下更新决策。

## DEC-007: 控制器外移先迁控制编排，再迁算法实现

- 日期：2026-06-20
- 背景：当前最小闭环已经形成，但 PID、LQR、VMC 的调用组织仍嵌在 `wheel_leg_hooks.cc` 所在的 MuJoCo 流程中。用户明确本轮目标不是立刻重写算法，而是先把工程结构和职责理顺。
- 可选方案：先直接重写 PID、LQR、VMC；先迁 MuJoCo 状态读取；先迁控制编排，再逐步迁算法实现。
- 最终选择：先迁 `controller orchestration`，再迁 `sim adapter` 边界，再抽出 PID、LQR、VMC 为独立算法接口，最后由 ROS2 controller node 逐步接管 hooks 中的控制调度。
- 选择理由：如果先重写算法实现，而控制流程入口仍留在 MuJoCo 内部，只会把新旧代码混在同一宿主里；先固定控制编排边界，才能让后续算法迁移和新动力学模型替换拥有稳定落点。
- 影响范围：`wheel_leg_control` 的职责定义、`wheel_leg_sim` 的接口边界、`transplant/tools/*.cc` 的迁移顺序、后续 controller node 实现方式。
- 后续是否可修改：可修改。若代码梳理后发现某个算法模块必须先抽出才能定义编排接口，应在不改变“MuJoCo 不长期承载控制流程组织职责”的原则下更新顺序。

## DEC-008: MuJoCo 长期只承担 sim adapter 职责，不再承载控制流程组织

- 日期：2026-06-20
- 背景：当前 `wheel_leg_hooks.cc` 同时承担仿真周期接入、状态读取、命令执行和控制流程组织，导致仿真运行时和控制层耦合过深。
- 可选方案：继续在 MuJoCo hooks 中承载控制流程；仅迁消息桥接；将 MuJoCo 限定为仿真适配层，并将控制流程入口迁往 ROS2 控制侧。
- 最终选择：MuJoCo 长期只负责仿真周期、状态采样、命令执行和过渡期 hook 接入；控制流程组织由 ROS2 controller node 所在的 `wheel_leg_control` 承担。
- 选择理由：后续还会接入 STM32 和新动力学模型，如果控制流程仍绑定 MuJoCo hook，控制侧就无法成为仿真与实机共享入口；把 MuJoCo 收缩为 sim adapter，才能让 bridge、control、sim、common 形成稳定边界。
- 影响范围：`wheel_leg_hooks.cc` 的长期定位、`wheel_leg_sim` 对外接口、控制节点设计方式、后续实机接口对齐方式。
- 后续是否可修改：原则上不建议修改。若后续需要保留少量 sim-only 调试逻辑在 MuJoCo 内部，应保证其不重新承担长期控制流程宿主职责。
