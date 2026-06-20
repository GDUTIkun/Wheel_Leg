# Iteration 001: ROS2-MuJoCo Topic 桥接

- 日期：2026-06-20
- 输入文档：`doc/proposal.md`、`doc/architecture.md`
- 输出文档：`doc/detail.md`

## 1. 本次迭代目标

本次迭代目标是完成 ROS2 与 MuJoCo 之间的最小 Topic 桥接设计，使 MuJoCo 能够向 ROS2 发布基础状态，并能够从 ROS2 接收关节或电机命令。

本迭代关注最小闭环：

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

本迭代是文档设计迭代，不实现代码。

## 2. 本次不做什么

本次迭代不做以下内容：

- 不实现遥控器接收机协议。
- 不定义 STM32 通信帧。
- 不实现完整 `ros2_control` 硬件接口。
- 不完成 LQR、VMC、PID 控制器外移。
- 不定义 `/robot_state`、`/body_cmd` 的自定义消息字段。
- 不固定 `/joint_command` 的具体消息类型。
- 不固定状态发布频率或控制频率。
- 不修改 `transplant/` 下的 MuJoCo 代码。

## 3. 涉及模块

当前迭代涉及模块：

- MuJoCo 状态读取模块。
- ROS2 状态发布模块。
- ROS2 命令订阅模块。
- MuJoCo actuator 写入模块。
- 接口与参数约束模块。

未进入当前迭代、只保留边界的模块：

- 控制器节点。
- 状态机与控制模式管理。
- 遥控器输入节点。
- STM32 通信节点。
- `ros2_control` 硬件接口。

## 4. 输入条件

当前迭代输入条件：

- `doc/proposal.md` 已存在并定义第一阶段目标。
- `doc/architecture.md` 已定义系统分层和模块边界。
- 当前 MuJoCo 工程已有基础模型和站立控制。
- 当前 MuJoCo 工程存在可用于读取传感器和写入 actuator 的代码基础。
- 目标 ROS2 环境为 ROS2 Jazzy 与 Ubuntu 24.04。

## 5. 输出结果

当前迭代输出结果：

- `doc/detail.md` 详细设计文档。
- 明确当前迭代模块边界。
- 明确 `/joint_states` 和 `/imu` 的标准消息方向。
- 明确 `/joint_command` 只作为语义占位，不固定消息类型。
- 明确当前待确认问题。

## 6. 最小实现任务

后续代码实现可拆成以下最小任务：

1. 在 MuJoCo 仿真侧建立 ROS2 节点初始化与生命周期边界。
2. 从 MuJoCo 数据中读取关节状态，并发布 `/joint_states`。
3. 从 MuJoCo 数据中读取 IMU 状态，并发布 `/imu`。
4. 订阅 `/joint_command`。
5. 将接收到的命令映射到 MuJoCo actuator。
6. 添加最小验证脚本或命令行测试步骤。

## 7. 仿真验证方法

后续实现完成后，仿真验证应至少包含：

- 启动 MuJoCo 仿真。
- 启动 ROS2 环境。
- 使用 ROS2 CLI 查看 `/joint_states` 是否持续发布。
- 使用 ROS2 CLI 查看 `/imu` 是否持续发布。
- 通过 ROS2 CLI 或调试节点发布 `/joint_command`。
- 观察 MuJoCo actuator 或机器人状态是否有响应。
- 确认现有 MuJoCo 基础站立能力未被破坏。

## 8. 实机验证方法

本次迭代不进行实机验证。

实机相关验证留到 STM32 通信层进入迭代后再定义。本迭代只要求接口设计不要阻碍后续真机适配。

## 9. 通过标准

文档层通过标准：

- `doc/detail.md` 存在。
- `doc/detail.md` 只细化当前迭代涉及模块。
- 当前迭代模块均包含目标、输入、输出、内部逻辑、接口、仿真行为、实机行为、参数配置、错误处理、验证方法和不确定问题。
- 未把遥控器、STM32、`ros2_control` 写成当前迭代实现内容。

后续代码层通过标准：

- ROS2 可订阅到 MuJoCo 发布的 `/joint_states`。
- ROS2 可订阅到 MuJoCo 发布的 `/imu`。
- ROS2 可向 MuJoCo 下发 `/joint_command`。
- MuJoCo 可根据命令写入 actuator。
- 当前命令通路不会破坏现有仿真启动。

## 10. 风险点

- ROS2 spin 与 MuJoCo step 线程关系未确认。
- `/joint_command` 消息类型未确认，后续实现前需要单独确认。
- 状态发布频率和控制频率未确认。
- actuator 名称、顺序、方向和限幅未形成完整接口表。
- 当前 MuJoCo 内部已有控制逻辑，命令桥接可能与现有控制输出冲突。
- 过早把控制器写入桥接层会造成新的耦合。

## 11. 需要用户确认的问题

- `/joint_command` 具体使用标准消息、数组消息还是自定义消息。
- 关节命名表和 actuator 映射表。
- 状态发布频率和命令处理频率。
- MuJoCo 内现有控制逻辑与 ROS2 命令的优先级关系。
- 是否需要为调试命令单独使用命名空间或测试 topic。
