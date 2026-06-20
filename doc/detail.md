# ROS2-MuJoCo Topic 桥接详细设计

## 1. 文档目标

本文档根据 `doc/proposal.md`、`doc/architecture.md` 和 `doc/iterations/iter-001.md` 编写，只细化当前迭代涉及的 ROS2-MuJoCo Topic 桥接模块。

当前迭代目标：

- MuJoCo 向 ROS2 发布 `/joint_states`。
- MuJoCo 向 ROS2 发布 `/imu`。
- MuJoCo 订阅 ROS2 `/joint_command`。
- MuJoCo 将接收到的命令写入 actuator 边界。

当前迭代不实现遥控器、STM32 通信、完整 `ros2_control`、完整控制器外移，也不定义自定义消息字段。

## 2. 当前迭代模块划分

当前迭代按以下模块设计：

1. MuJoCo 桥接节点模块。
2. 关节状态发布模块。
3. IMU 状态发布模块。
4. 关节命令订阅模块。
5. actuator 命令写入模块。
6. 迭代外模块边界。

模块之间通过明确输入输出连接，避免把控制算法、硬件通信和遥控输入混入同一个实现。

## 3. MuJoCo 桥接节点模块

### 3.1 模块目标

建立 MuJoCo 与 ROS2 的运行时连接边界，负责 ROS2 节点初始化、发布器和订阅器创建，以及在 MuJoCo 仿真周期中触发状态发布和命令处理。

### 3.2 模块输入

- MuJoCo 模型对象。
- MuJoCo 数据对象。
- ROS2 运行环境。
- 当前仿真时间。

### 3.3 模块输出

- ROS2 节点。
- `/joint_states` 发布器。
- `/imu` 发布器。
- `/joint_command` 订阅器。
- 可供其他桥接子模块使用的上下文。

### 3.4 模块内部逻辑

推荐逻辑：

1. 模型加载后初始化桥接上下文。
2. 创建 ROS2 节点和 topic 端点。
3. 在 MuJoCo step 附近读取状态。
4. 将状态交给关节状态发布模块和 IMU 状态发布模块。
5. 处理 ROS2 回调中收到的命令。
6. 将最新命令交给 actuator 命令写入模块。

ROS2 spin 与 MuJoCo step 当前采用同线程策略，在 MuJoCo step 边界进行非阻塞 ROS 回调处理。

### 3.5 与其他模块的接口

| 对接模块 | 交互内容 |
| --- | --- |
| 关节状态发布模块 | 提供 MuJoCo 状态读取上下文 |
| IMU 状态发布模块 | 提供 MuJoCo 状态读取上下文 |
| 关节命令订阅模块 | 接收最新 ROS2 命令缓存 |
| actuator 命令写入模块 | 将命令应用到 MuJoCo actuator |

### 3.6 仿真环境下的行为

仿真环境下，该模块运行在 MuJoCo 程序内或其邻近桥接层中，可以访问 MuJoCo 类型和数据。该模块允许读取 `mjModel`、`mjData`，也允许将命令写入 MuJoCo actuator。

### 3.7 实机环境下的行为

实机环境下不运行该模块。真机后续由 STM32 通信节点或 `STM32Hardware` 提供等价接口。

### 3.8 参数配置

当前迭代不固定参数名。后续可能需要配置：

- ROS2 node name：`mujoco_bridge`。
- topic namespace：当前不使用 namespace，直接使用全局 topic。
- 状态发布频率：`100 Hz`。
- 命令超时时间：`0.2 s`。
- 是否启用 ROS2 命令接管 actuator：使用显式开关 `enable_ros_command`，默认 `false`。

以上当前迭代参数已确认；后续如需参数化，可在实现时保留为可配置项。

### 3.9 错误处理

应考虑的错误：

- ROS2 初始化失败。
- 发布器或订阅器创建失败。
- MuJoCo 模型未加载。
- MuJoCo 数据指针无效。
- ROS2 命令长时间未更新。

当前迭代只记录错误处理需求，不固定实现策略。

### 3.10 测试与验证方法

文档层验证：

- 确认该模块不承担控制算法职责。
- 确认该模块边界只服务 ROS2-MuJoCo 桥接。

后续代码验证：

- 启动 MuJoCo 后能创建 ROS2 节点。
- ROS2 CLI 能看到相关 topic。
- 未启动 ROS2 命令时不破坏原有仿真启动。

### 3.11 当前不确定问题

- 暂无。后续需通过仿真验证同线程 ROS spin 是否影响 MuJoCo step。

## 4. 关节状态发布模块

### 4.1 模块目标

将 MuJoCo 中的关节状态转换为 ROS2 标准 `/joint_states`，用于后续控制器、调试工具和日志系统订阅。

### 4.2 模块输入

- MuJoCo 模型对象。
- MuJoCo 数据对象。
- 当前仿真时间。
- 关节命名和顺序约定。

### 4.3 模块输出

- Topic：`/joint_states`
- Message：`sensor_msgs/msg/JointState`

### 4.4 模块内部逻辑

推荐逻辑：

1. 从 MuJoCo 模型或现有传感器组装逻辑中读取关节名称。
2. 从 MuJoCo 数据中读取关节位置。
3. 从 MuJoCo 数据中读取关节速度。
4. 如可获得 actuator 或传感器反馈力矩，则填充 effort。
5. 填充消息时间戳。
6. 发布 `/joint_states`。

当前已确认关节列表和顺序。`/joint_states` 发布频率为 `100 Hz`，`header.stamp` 使用 MuJoCo 仿真时间 `d->time`。是否填充 effort 待后续确认。

### 4.5 与其他模块的接口

| 对接模块 | 交互内容 |
| --- | --- |
| MuJoCo 桥接节点模块 | 获取 MuJoCo 状态上下文 |
| 后续控制器节点 | 发布标准关节状态 |
| 后续日志工具 | 提供统一关节状态数据 |

### 4.6 仿真环境下的行为

仿真环境下，该模块从 MuJoCo 读取关节状态并发布标准 ROS2 消息。该模块允许访问 MuJoCo 类型，但不应包含控制算法。

### 4.7 实机环境下的行为

实机环境下不使用 MuJoCo 读取逻辑。后续 STM32 通信层应发布同语义 `/joint_states`，使控制器无需区分仿真和真机。

### 4.8 参数配置

当前迭代不固定参数名。后续可能需要配置：

- 发布频率：`100 Hz`。
- 关节名称列表。
- 是否发布 effort。
- 是否启用特定关节过滤。

### 4.9 错误处理

应考虑的错误：

- 关节名称缺失。
- 关节数量与状态数组长度不一致。
- MuJoCo 中缺少某个预期关节。
- 位置或速度数据无效。

处理原则：

- 不因单个关节缺失导致整个仿真崩溃。
- 缺失项应有日志提示。
- 是否跳过缺失关节或停止发布待后续确认。

### 4.10 测试与验证方法

文档层验证：

- 确认使用 `sensor_msgs/msg/JointState`。
- 确认未定义新的自定义关节状态消息。

后续代码验证：

- 使用 `ros2 topic echo /joint_states` 查看消息。
- 检查 name、position、velocity 长度一致。
- 观察 MuJoCo 运动时 position 或 velocity 是否变化。

### 4.11 当前不确定问题

- effort 是否发布。

## 5. IMU 状态发布模块

### 5.1 模块目标

将 MuJoCo 中的机体姿态、角速度和线加速度转换为 ROS2 标准 `/imu`，用于后续状态估计、控制器和日志系统订阅。

### 5.2 模块输入

- MuJoCo 模型对象。
- MuJoCo 数据对象。
- 当前仿真时间。
- MuJoCo IMU sensor 数据。

### 5.3 模块输出

- Topic：`/imu`
- Message：`sensor_msgs/msg/Imu`

### 5.4 模块内部逻辑

推荐逻辑：

1. 从 MuJoCo sensor 数据中读取姿态。
2. 从 MuJoCo sensor 数据中读取角速度。
3. 从 MuJoCo sensor 数据中读取线加速度。
4. 将姿态转换为 ROS2 IMU 消息所需格式。
5. 填充时间戳和 frame id。
6. 发布 `/imu`。

当前已确认 IMU sensor、frame id 和四元数顺序。MuJoCo sensor 使用 `base_quat`、`base_gyro`、`base_accel`；ROS `header.frame_id` 使用 `base_link`；`base_quat` 从 MuJoCo `w,x,y,z` 转为 ROS `x,y,z,w`；当前不做轴重映射，按 `base_link` 机体系直接发布；covariance 全 0，表示未知。

### 5.5 与其他模块的接口

| 对接模块 | 交互内容 |
| --- | --- |
| MuJoCo 桥接节点模块 | 获取 MuJoCo 状态上下文 |
| 后续状态估计模块 | 发布 IMU 标准状态 |
| 后续控制器节点 | 提供机体姿态和角速度输入 |

### 5.6 仿真环境下的行为

仿真环境下，该模块从 MuJoCo sensor 中读取 IMU 相关数据。需要注意 MuJoCo 内部坐标系、四元数顺序和 ROS2 消息语义的一致性。

### 5.7 实机环境下的行为

实机环境下不使用 MuJoCo sensor。后续 STM32 通信层应发布同语义 `/imu`，使状态估计和控制器不依赖数据来源。

### 5.8 参数配置

当前迭代不固定参数名。后续可能需要配置：

- IMU sensor 名称：`base_quat`、`base_gyro`、`base_accel`。
- frame id：`base_link`。
- 发布频率：`100 Hz`。
- covariance：当前全 0，表示未知。
- 坐标系转换开关或方向参数：当前不做轴重映射。

### 5.9 错误处理

应考虑的错误：

- MuJoCo 中缺少 IMU sensor。
- sensor 数据维度不符合预期。
- 四元数无效。
- 线加速度或角速度为异常值。

处理原则：

- 缺少 IMU sensor 时应明确报错。
- 不应发布语义错误但看似正常的 IMU 数据。
- 坐标系语义如与仿真验证现象不一致，应先记录验证现象再调整转换策略。

### 5.10 测试与验证方法

文档层验证：

- 确认使用 `sensor_msgs/msg/Imu`。
- 确认未在本迭代定义自定义 IMU 消息。

后续代码验证：

- 使用 `ros2 topic echo /imu` 查看消息。
- 观察 MuJoCo 姿态变化时 orientation 或 angular_velocity 是否变化。
- 检查 frame id 是否稳定。

### 5.11 当前不确定问题

- 暂无。后续需通过仿真验证坐标系语义是否与可视化和状态估计期望一致。

## 6. 关节命令订阅模块

### 6.1 模块目标

从 ROS2 接收 `/joint_command`，作为后续控制器或调试工具向 MuJoCo 下发关节或电机命令的入口。

### 6.2 模块输入

- Topic：`/joint_command`
- Message：`wheel_leg_msgs/msg/JointCommand`
- Fields：`std_msgs/Header header`、`string[] joint_names`、`float64[] efforts`
- ROS2 回调上下文。

### 6.3 模块输出

- 最新关节或电机命令缓存。
- 命令时间戳。
- 命令有效性状态。

### 6.4 模块内部逻辑

推荐逻辑：

1. 订阅 `/joint_command`。
2. 在回调中读取命令。
3. 校验命令是否可映射到当前 MuJoCo actuator。
4. 缓存最新有效命令。
5. 将命令交给 actuator 命令写入模块。

当前已确认使用关节名映射方式。`joint_names` 与 `efforts` 一一对应，`efforts` 单位为 Nm。ROS 侧 canonical 关节名称为 `left_hip`、`left_knee`、`left_wheel`、`right_hip`、`right_knee`、`right_wheel`。
命令处理跟随 MuJoCo step，每个 step 处理一次最新有效命令。

### 6.5 与其他模块的接口

| 对接模块 | 交互内容 |
| --- | --- |
| MuJoCo 桥接节点模块 | 创建订阅器和维护回调 |
| actuator 命令写入模块 | 提供最新有效命令 |
| 后续控制器节点 | 后续发布关节或电机命令 |
| 调试命令节点 | 可用于手动发布测试命令 |

### 6.6 仿真环境下的行为

仿真环境下，该模块只负责接收 ROS2 命令，不直接写 `d->ctrl`。实际写入由 actuator 命令写入模块完成。

### 6.7 实机环境下的行为

实机环境下不使用该 MuJoCo 订阅逻辑。后续 STM32 通信层应订阅同语义命令或由统一控制接口转发到 STM32。

### 6.8 参数配置

当前迭代不固定参数名。后续可能需要配置：

- `/joint_command` topic 名称是否保持全局路径或进入 namespace。
- 命令超时时间。
- 命令是否覆盖 MuJoCo 内部控制逻辑。
- 命令限幅策略。
- 命令关节名称映射。
- 命令处理频率：每个 MuJoCo step 处理一次最新有效命令。

### 6.9 错误处理

应考虑的错误：

- 收到无法解析的命令。
- 命令关节数量不匹配。
- 命令中包含未知关节。
- 命令超时。
- 命令值超过 actuator 合理范围。

处理原则：

- 无效命令不应直接写入 actuator。
- 超时策略为 `0.2 s` 后停止应用 ROS 命令，不自动清零 actuator。
- 限幅策略使用 MuJoCo actuator `ctrlrange`，不在代码中自定义固定限幅数值。

### 6.10 测试与验证方法

文档层验证：

- 确认 `/joint_command` 使用自定义消息。
- 确认 `/joint_command` 字段、单位和映射方式已记录。

后续代码验证：

- 使用 ROS2 CLI 或调试节点发布测试命令。
- 确认桥接节点能接收命令。
- 确认无效命令不会导致仿真崩溃。

### 6.11 当前不确定问题

- 命令超时策略。
- 命令与现有 MuJoCo 内部站立控制的优先级关系。

## 7. actuator 命令写入模块

### 7.1 模块目标

将已接收并校验的 ROS2 命令转换为 MuJoCo actuator 控制量，并写入 MuJoCo 仿真。

### 7.2 模块输入

- MuJoCo 模型对象。
- MuJoCo 数据对象。
- 最新有效 `/joint_command`。
- actuator 名称或映射关系。

### 7.3 模块输出

- 写入 MuJoCo actuator 的控制量。
- 写入结果状态。

### 7.4 模块内部逻辑

推荐逻辑：

1. 获取最新有效命令。
2. 根据命令语义查找目标 actuator。
3. 将命令值转换为 MuJoCo actuator 控制量。
4. 写入 `d->ctrl` 或调用现有 actuator helper。
5. 记录写入成功、失败或跳过原因。

当前已确认 actuator 映射表，但不固定限幅策略。

| ROS joint name | MuJoCo joint | MuJoCo actuator |
| --- | --- | --- |
| `left_hip` | `left_hip_joint` | `left_hip_motor` |
| `left_knee` | `left_knee_joint` | `left_knee_motor` |
| `left_wheel` | `left_wheel_joint` | `left_wheel_motor` |
| `right_hip` | `right_hip_joint` | `right_hip_motor` |
| `right_knee` | `right_knee_joint` | `right_knee_motor` |
| `right_wheel` | `right_wheel_joint` | `right_wheel_motor` |

MuJoCo 中的 `left_connect2_joint`、`left_calf_joint`、`right_connect2_joint`、`right_calf_joint` 当前不进入 `/joint_command`。

ROS2 命令默认不接管 actuator。`enable_ros_command=false` 时保持现有内部站立控制输出；`enable_ros_command=true` 时，ROS `/joint_command` 在 step 末尾覆盖对应 actuator。命令超时时间为 `0.2 s`，超时后停止应用 ROS 命令并回到不接管 actuator 行为，不自动清零 actuator。命令限幅使用 MuJoCo actuator `ctrlrange`。无效命令整条拒绝，不写入任何 actuator；无效条件包括 `joint_names` 与 `efforts` 长度不一致、未知 joint name、NaN 或 Inf。

### 7.5 与其他模块的接口

| 对接模块 | 交互内容 |
| --- | --- |
| 关节命令订阅模块 | 接收最新有效命令 |
| MuJoCo 桥接节点模块 | 在仿真周期中调用写入逻辑 |
| MuJoCo 模型 | 写入 actuator 控制量 |

### 7.6 仿真环境下的行为

仿真环境下，该模块允许写入 MuJoCo actuator。写入时需要避免与现有 `wheel_leg_hooks.cc` 内部控制逻辑产生未定义冲突。

当前建议：

- 在详细实现前明确 ROS2 命令和现有站立控制的优先级。
- 在优先级未确认前，不假设 ROS2 命令必然覆盖所有内部控制输出。

### 7.7 实机环境下的行为

实机环境下不使用 MuJoCo actuator 写入逻辑。后续 STM32 通信层应将同语义命令转换为真实电机命令。

### 7.8 参数配置

当前迭代不固定参数名。后续可能需要配置：

- actuator 名称映射表。
- 力矩限幅。
- 命令单位。
- 写入模式。
- 与内部控制逻辑的优先级。

### 7.9 错误处理

应考虑的错误：

- actuator 名称不存在。
- 命令数量与 actuator 数量不一致。
- 命令值超出合理范围。
- MuJoCo 数据指针无效。
- 命令与内部控制逻辑冲突。

处理原则：

- 未找到 actuator 时应跳过该项并记录错误。
- 不应静默写入错误 actuator。
- 无效命令整条拒绝，不写入任何 actuator。
- 限幅使用 MuJoCo actuator `ctrlrange`，不在代码中自定义固定限幅数值。

### 7.10 测试与验证方法

文档层验证：

- 确认 actuator 映射表已记录。
- 确认未固定力矩限幅。

后续代码验证：

- 发布单个 actuator 测试命令。
- 观察 MuJoCo `d->ctrl` 或机器人响应。
- 测试未知 actuator 或无效命令不会导致程序崩溃。

### 7.11 当前不确定问题

- 力矩限幅。
- ROS2 命令与现有内部控制逻辑的优先级。
- 是否需要命令超时后清零。

## 8. 接口与参数约束模块

### 8.1 模块目标

集中记录当前迭代已经确认和未确认的接口约束，避免在实现时分散猜测。

### 8.2 模块输入

- `doc/proposal.md`。
- `doc/architecture.md`。
- `doc/iterations/iter-001.md`。
- 当前用户已确认的设计选择。

### 8.3 模块输出

- 当前迭代接口约束。
- 当前迭代待确认问题。
- 后续实现前必须确认的参数列表。

### 8.4 模块内部逻辑

该模块不是运行时代码模块，而是文档约束模块。它用于保证每个实现任务只使用已确认接口，不为了代码方便临时发明协议或字段。

### 8.5 与其他模块的接口

| 对接模块 | 交互内容 |
| --- | --- |
| 所有当前迭代模块 | 提供 topic、消息和待确认参数约束 |
| 后续任务文档 | 为任务拆分提供输入 |
| 后续决策文档 | 为 `doc/decisions.md` 提供候选决策 |

### 8.6 仿真环境下的行为

仿真环境下，所有桥接实现必须遵守本文档中已确认的接口语义。

### 8.7 实机环境下的行为

实机环境下，后续 STM32 接口应尽量复用相同状态和命令语义。

### 8.8 参数配置

当前已确认：

- ROS2 发行版：Jazzy。
- 操作系统目标：Ubuntu 24.04。
- `/joint_states` 使用 `sensor_msgs/msg/JointState`。
- `/imu` 使用 `sensor_msgs/msg/Imu`。
- `/joint_command` 使用 `wheel_leg_msgs/msg/JointCommand`。
- `/joint_command` 字段为 `std_msgs/Header header`、`string[] joint_names`、`float64[] efforts`。
- `/joint_command` 使用关节名映射，`joint_names` 与 `efforts` 一一对应。
- `/joint_command` 当前只表达 effort 命令，单位为 Nm。
- 当前关节集合按左、右两侧组织，每侧包含髋部、膝部和轮部。
- ROS 侧 canonical 关节命名表和顺序：`left_hip`、`left_knee`、`left_wheel`、`right_hip`、`right_knee`、`right_wheel`。
- actuator 映射表：`left_hip` -> `left_hip_motor`，`left_knee` -> `left_knee_motor`，`left_wheel` -> `left_wheel_motor`，`right_hip` -> `right_hip_motor`，`right_knee` -> `right_knee_motor`，`right_wheel` -> `right_wheel_motor`。
- `/joint_states` 和 `/imu` 发布频率为 `100 Hz`。
- `/joint_states.header.stamp` 和 `/imu.header.stamp` 使用 MuJoCo 仿真时间 `d->time`。
- `/joint_command` 命令处理跟随 MuJoCo step，每个 step 处理一次最新有效命令。
- `/imu` 使用 MuJoCo sensor：`base_quat`、`base_gyro`、`base_accel`。
- `/imu.header.frame_id` 使用 `base_link`。
- `/imu.orientation` 将 MuJoCo `base_quat` 的 `w,x,y,z` 顺序转换为 ROS `x,y,z,w`。
- `/imu` 当前不做轴重映射，按 `base_link` 机体系直接发布；covariance 全 0，表示未知。
- ROS2 节点名为 `mujoco_bridge`，当前不使用 topic namespace。
- `/joint_states.effort` 当前不填充。
- ROS spin 策略为同线程，在 MuJoCo step 边界调用 `rclcpp::spin_some` 或等价非阻塞处理。
- ROS2 命令接管使用 `enable_ros_command`，默认 `false`。
- `/joint_command` 超时时间为 `0.2 s`，超时后停止应用 ROS 命令，不自动清零 actuator。
- actuator 限幅使用 MuJoCo actuator `ctrlrange`。
- 无效 `/joint_command` 整条拒绝，不写入任何 actuator。

当前未确认：

- `/robot_state` 消息类型。
- `/body_cmd` 消息类型。

### 8.9 错误处理

如果实现中遇到未确认接口或参数，不应自行固定，应先更新设计问题并向用户确认。

### 8.10 测试与验证方法

文档层验证：

- 检查已确认接口是否与 `doc/proposal.md` 一致。
- 检查未确认项是否明确列出。

后续代码验证：

- 根据后续任务文档逐项验证。

### 8.11 当前不确定问题

- 是否需要在下一轮生成 `doc/decisions.md` 记录 Topic 桥接优先等设计选择。
- 是否需要为 `/joint_command` 单独开展接口设计迭代。
- 是否需要先建立关节命名与 actuator 映射文档。

## 9. 迭代外模块边界

### 9.1 控制器节点

控制器节点后续负责 LQR、MPC、WBC 等算法。本迭代只要求桥接接口不要把 MuJoCo 类型泄漏给控制器。

### 9.2 遥控器输入节点

遥控器输入节点后续负责把接收机输入转换为 `/cmd_vel`、`/body_cmd`、`/control_mode`。本迭代不指定遥控器协议。

### 9.3 STM32 通信节点

STM32 通信节点后续负责真实硬件状态上报和命令下发。本迭代不定义串口或 CAN 协议。

### 9.4 ros2_control 硬件接口

后续可将 MuJoCo 和 STM32 分别封装为 `MuJoCoHardware` 和 `STM32Hardware`。本迭代不实现完整 `ros2_control`。

## 10. 当前迭代通过标准

文档通过标准：

- 本文档只细化 ROS2-MuJoCo Topic 桥接。
- 当前迭代模块均包含 `doc/AGENTS.md` 要求的详细设计项。
- `/joint_states` 和 `/imu` 使用标准消息。
- `/joint_command` 使用 `wheel_leg_msgs/msg/JointCommand`。
- 频率、关节命名表和 actuator 映射表已记录。
- 命令超时、限幅和控制优先级已记录。
- 遥控器、STM32、`ros2_control` 未被写成当前迭代实现内容。

后续代码通过标准：

- ROS2 能订阅 `/joint_states`。
- ROS2 能订阅 `/imu`。
- ROS2 能发布 `/joint_command` 并被 MuJoCo 桥接层接收。
- MuJoCo actuator 写入边界可验证。
- 不破坏现有 MuJoCo 仿真启动和基础站立能力。

## 11. 后续建议任务

建议下一步按最小任务继续拆分：

1. 创建并验证 `wheel_leg_msgs/msg/JointCommand.msg`。
2. 选择 `doc/tasks/mujoco_bridge.md` 中一个最小任务。
3. 选择 `doc/tasks/joint_state_publisher.md` 中一个最小任务。
4. 选择 `doc/tasks/imu_publisher.md` 中一个最小任务。
5. 选择 `doc/tasks/joint_command_subscriber.md` 中一个最小任务。
