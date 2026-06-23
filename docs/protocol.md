# 轮腿机器人统一接口与通信边界文档

## 1. 文档目标

本文档固定仿真与实机共用的上层接口语义，为后续 STM32 通信实现和 `ros2_control` 收口提供一致约束。

本轮不定义最终 STM32 二进制帧格式的每一个字节，但先固定上层接口和下层对接原则。

## 2. 上层公共 Topic

当前阶段公共 Topic 保持不变：

- `/robot_state`
- `/joint_command`
- `/joint_states`
- `/imu`
- `/cmd_vel`
- `/control_mode`

约束：

- 仿真和实机不得为同一语义发明两套 Topic 名称。
- 上层控制器只依赖这些公共接口，不直接感知底层是 MuJoCo 还是 STM32。

## 3. 状态接口语义

### 3.1 `/robot_state`

职责：

- 作为控制器主输入。
- 聚合控制所需的机体、腿部、轮部和模式相关状态。

来源：

- 仿真模式由 `wheel_leg_sim` 生成。
- 实机模式由 `wheel_leg_stm32_bridge` 生成。

### 3.2 `/joint_states`

职责：

- 对外提供标准关节位置、速度观测。
- 服务调试、可视化和通用 ROS 工具。

### 3.3 `/imu`

职责：

- 提供机体姿态、角速度和线加速度。

当前约束：

- `frame_id` 继续使用 `base_link`
- 四元数按 ROS 标准顺序发布

## 4. 命令接口语义

### 4.1 `/joint_command`

职责：

- 作为当前阶段控制器到执行层的统一命令接口。

消息约束：

- 使用 `wheel_leg_msgs/msg/JointCommand`
- `joint_names` 使用 ROS 侧规范命名
- `efforts` 当前语义为力矩命令

### 4.2 `/cmd_vel`

职责：

- 作为统一速度控制输入。
- 可以来自遥控器、调试脚本或后续规划器。

### 4.3 `/control_mode`

职责：

- 控制站立、失能、速度控制等模式切换。

## 5. 命名约束

ROS 侧关节规范名保持为：

- `left_hip`
- `left_knee`
- `left_wheel`
- `right_hip`
- `right_knee`
- `right_wheel`

约束：

- 该命名用于 `/joint_states.name` 和 `/joint_command.joint_names`
- MuJoCo XML 命名和 STM32 内部电机编号都通过映射层转换
- 上层控制代码不直接依赖仿真或固件内部命名

## 6. 单位与方向约束

本轮先固定原则，具体数值映射在 STM32 接入时细化：

- 位置使用弧度或米，禁止混用角度制进入控制主链
- 速度使用 `rad/s`、`m/s`
- 力矩使用 `Nm`
- IMU 角速度使用 `rad/s`
- 线加速度使用 `m/s^2`

方向约束：

- 同一关节在仿真和实机必须保持相同正方向语义
- 若 STM32 底层电机安装方向相反，应在 bridge 或 firmware 侧消解，不泄漏到上层控制器

## 7. STM32 通信分层原则

Raspberry Pi 与 STM32 的长期关系固定为：

```text
wheel_leg_controller
  ↓
/joint_command
  ↓
wheel_leg_stm32_bridge
  ↓
serial / can protocol
  ↓
STM32 firmware
```

分层约束：

- `wheel_leg_control` 不解析 STM32 二进制协议
- `wheel_leg_stm32_bridge` 负责协议编解码
- `firmware/stm32` 负责执行层实时细节

## 8. 与 `ros2_control` 的衔接

本轮不直接改成 `ros2_control`，但本文档中的命名和接口语义将作为后续 `SystemInterface` 的输入条件。

后续若接入 `ros2_control`：

- `wheel_leg_hw` 负责统一 hardware 语义
- MuJoCo 和 STM32 都实现同一套 joint/state/command 约定
- 控制器层不再区分仿真和实机 backend
