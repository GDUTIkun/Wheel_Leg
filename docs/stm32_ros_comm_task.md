# STM32 与 ROS 通信内容任务

## 1. 任务目标

本任务用于固定 STM32 层与 ROS 层第一版通信内容，为后续实现 `wheel_leg_stm32_bridge` 和 STM32 协议任务提供依据。

第一版边界：

- STM32 只上传原始或近原始硬件数据。
- ROS 侧完成全部控制计算。
- ROS 下发六电机力矩指令。
- STM32 负责协议收发、单位映射、限幅、斜率限制、安全停机和电机执行。

本任务不把平衡控制下放到 STM32，也不在本阶段引入完整 `ros2_control`。

## 2. 分层边界

通信主链保持为：

```text
STM32 firmware
  ↑↓
UART protocol
  ↑↓
wheel_leg_stm32_bridge
  ↑↓
/robot_state and /joint_command
  ↑↓
wheel_leg_controller
```

约束：

- `wheel_leg_control` 不解析 STM32 二进制协议。
- `wheel_leg_stm32_bridge` 负责协议编解码和 ROS 消息映射。
- `firmware/stm32` 负责实时采样、电机执行和底层安全保护。
- 上层控制器不感知底层是 MuJoCo 还是 STM32。

## 3. 通信周期与链路

第一版控制闭环周期按 `100Hz` 规划，但 STM32 到 ROS 的状态上报频率需要单独测试确认：

- ROS 侧控制器按 `100Hz` 消费状态并发布 `/joint_command`。
- ROS 到 STM32 的命令帧按最新 `/joint_command` 下发。
- STM32 必须基于命令超时进入安全输出，不依赖 ROS 主动发送停机帧。
- STM32 可以以高于 `100Hz` 的频率发送状态帧，让 ROS 侧获得更密的传感器观测。
- 当前实测 STM32 读完一次传感器约在 `3ms` 以内，因此优先测试 `200Hz` 状态上报。
- 最终状态上报频率由 STM32 传感器读取耗时、串口带宽、解包开销、丢包率和控制链稳定性共同决定。

链路明确使用 UART/串口通信。当前固件 USART 默认配置为 `115200 8N1`，后续实现前需要根据实际帧长度确认是否提高波特率。

由于串口是字节流，不提供天然消息边界，STM32 侧和 ROS bridge 侧都必须手动解包：

- 两侧都维护接收缓冲区和状态机。
- 通过帧头进行帧同步。
- 读取长度字段后等待完整 payload。
- 对帧类型、长度和 CRC 进行校验。
- 校验通过后再解析 payload 并更新状态或命令。
- 校验失败时丢弃坏帧并重新寻找下一帧头。

## 4. STM32 到 ROS 状态上报

状态帧至少包含以下内容。

### 4.1 时间与序号

- `seq`：状态帧递增序号，用于检测丢包和乱序。
- `stm_tick_ms` 或等价周期计数：用于估算 STM32 侧采样周期和 ROS 侧 `dt`。

### 4.2 IMU 数据

STM32 上传原始或近原始 IMU 数据，ROS 侧再映射到控制语义：

- 姿态角或四元数。
- `gyro_x`、`gyro_y`、`gyro_z`，单位最终进入 ROS 时为 `rad/s`。
- `acc_x`、`acc_y`、`acc_z`，单位最终进入 ROS 时为 `m/s^2`。

后续实现时需要记录 IMU 坐标轴、正方向和滤波口径，保证 `/robot_state` 中 `body_pitch`、`body_pitch_rate` 与仿真和 LQR 状态定义一致。

### 4.3 六电机反馈

电机顺序固定使用 ROS 侧规范命名：

- `left_hip`
- `left_knee`
- `left_wheel`
- `right_hip`
- `right_knee`
- `right_wheel`

每个电机至少包含：

- `position`：位置观测，进入 ROS 后统一为 `rad`。
- `velocity`：速度观测，进入 ROS 后统一为 `rad/s`。
- `feedback_torque` 或 `feedback_current`：反馈力矩或电流，用于调试和安全诊断。
- `online`：电机在线状态。

若 STM32 内部使用 CAN ID、电机原始角度、编码器计数或角度制，必须在 STM32 或 bridge 映射层消解，不得泄漏到上层控制器。

### 4.4 安全与诊断

状态帧必须能观察通信和执行层安全状态：

- `comm_rx_error_count`：STM32 接收命令解析错误计数。
- `comm_crc_error_count`：CRC 或校验错误计数。
- `last_command_timeout`：最近一次命令是否超时。
- `safety_state`：STM32 当前安全状态，例如 disabled、enabled、timeout、estop。
- `can_motor_online_mask`：六电机 CAN 在线位图。
- `can_error_count`：CAN 或电机反馈异常计数。

## 5. ROS 到 STM32 命令下发

命令帧由 `wheel_leg_stm32_bridge` 根据 `/joint_command` 生成。

命令帧至少包含：

- `seq`：命令帧递增序号。
- `enable`：控制使能位。
- `estop`：急停或强制失能位。
- 六电机目标力矩，语义对应 `/joint_command.efforts`。

六电机目标力矩顺序与状态上报一致：

```text
left_hip, left_knee, left_wheel, right_hip, right_knee, right_wheel
```

STM32 执行前必须处理：

- 力矩最大值限幅。
- 单周期变化率限制。
- 命令超时归零或失能。
- 急停时立即进入安全输出。
- 电机离线时禁止继续对该电机输出危险命令。

## 6. 单位、方向与映射规则

ROS 侧统一单位：

- 角度：`rad`
- 角速度：`rad/s`
- 力矩：`N*m`
- 线加速度：`m/s^2`

方向约束：

- 同一关节在仿真和实机必须保持相同正方向语义。
- 电机安装方向、零点、CAN ID 映射只能存在于 STM32 或 bridge 映射层。
- `/robot_state` 的语义必须与 `docs/protocol.md` 中的公共接口一致。
- `/joint_command.efforts` 的正方向必须与对应关节正方向一致。

当前阶段允许先以低力矩悬空测试确认方向，再决定具体映射放在 STM32 侧还是 bridge 侧；但最终不得让 `wheel_leg_control` 依赖底层电机方向细节。

## 7. 异常处理要求

### 7.1 命令超时

若 STM32 在超时时间内没有收到有效命令帧：

- 进入安全状态。
- 六电机目标输出归零或失能。
- 状态帧中置位 `last_command_timeout`。
- 保留错误计数，便于 ROS 侧记录。

### 7.2 通信错误

若收到非法帧、CRC 错误或长度错误：

- 丢弃该帧。
- 不更新有效命令。
- 增加对应错误计数。
- 保持上一安全策略，不因坏帧解除急停或恢复使能。

### 7.3 电机或 CAN 异常

若任一电机离线或 CAN 反馈异常：

- 状态帧中更新在线位图和错误计数。
- 对离线电机禁止继续输出危险命令。
- 是否整车失能由后续安全策略细化，但第一版必须可观测。

## 8. 实施清单

后续实现按以下顺序推进：

1. 定义 STM32 与 ROS 串口帧头、长度、类型、序号、payload、CRC。
2. 在 STM32 侧实现串口接收状态机，手动解包 ROS 命令帧。
3. 在 `wheel_leg_stm32_bridge` 中实现串口接收状态机，手动解包 STM32 状态帧。
4. 在 STM32 侧实现状态帧周期打包发送。
5. 在 `wheel_leg_stm32_bridge` 中实现命令帧打包发送、协议错误统计和重同步。
6. 将 STM32 状态帧映射为 `/robot_state`。
7. 将 `/joint_command` 映射为 STM32 六电机力矩命令帧。
8. 加入命令超时、安全状态、错误计数和低力矩调试参数。
9. 测试 STM32 状态上报频率，从 `100Hz`、`200Hz` 开始，记录传感器读取耗时、串口占用、ROS 解包延迟、丢包和错误计数。
10. 用悬空低力矩方式逐项验证六电机方向、单位、限幅和停机行为。

## 9. 验收场景

文档和后续实现按以下场景验收：

- 空帧或心跳通信稳定，可统计周期、丢包、CRC 错误和解析错误。
- 控制闭环保持 `100Hz` 时，测试 STM32 状态上报 `100Hz` 与 `200Hz` 的稳定性；若资源允许，再继续测试更高频率。
- STM32 上传 IMU 和六电机反馈，ROS 能稳定映射成 `/robot_state`。
- ROS 下发六电机小力矩，STM32 在悬空状态逐个验证方向、单位和限幅。
- 停止 ROS 命令后，STM32 在超时时间内进入安全输出。
- CAN 电机掉线、串口异常、failsafe 时，对应状态字段可观测。
- 控制器仍只依赖 `/robot_state` 和 `/joint_command`，不直接依赖 STM32 协议细节。

## 10. 与现有文档关系

- `docs/protocol.md` 继续固定仿真和实机共用的 Topic、命名、单位和方向边界。
- `docs/stm32_hardware_integration.md` 继续作为硬件接入与实机闭环总任务。
- 本文档专门记录 STM32 与 ROS 第一版通信内容、字段和验收任务。
