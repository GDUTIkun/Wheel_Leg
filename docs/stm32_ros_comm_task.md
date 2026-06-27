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

链路明确使用 UART/串口通信。当前 `USART2` 压测入口默认配置为 `921600 8N1`；若线缆、地线、树莓派串口和 STM32 侧错误计数都稳定，再继续测试 `1000000` 或 `2000000`。

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

当前硬件腿部角度口径：

- 仿真侧由原始关节角叠加 offset 得到 `hip_absolute`、`calf_absolute`。
- STM32 上报的髋和小腿角已经是世界坐标系 absolute 角。
- `left_hip` / `right_hip` 的 `position` 直接作为 `hip_absolute`。
- `left_knee` / `right_knee` 的 `position` 在控制语义中直接作为 `calf_absolute`。
- ROS bridge 不再对 STM32 上传的髋/小腿角叠加仿真 offset。

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

当前已先在 STM32 侧加入 `USART2` 接收解包压测入口，用于模拟 ROS 命令帧接收是否跟得上：

- 文件：`firmware/stm32/App/uart_protocol_test.cpp`
- 串口：`USART2`
- 帧格式：`0xA5 0x5A type len seq_lo seq_hi payload crc_lo crc_hi`
- CRC：`CRC16-CCITT`，初值 `0xFFFF`，覆盖 `type`、`len`、`seq` 和 `payload`
- 下行压测：STM32 中断接收并统计接收字节数、成功帧数、CRC 错误、长度错误、同步丢失、序号跳变、UART 错误、帧间隔；Keil Watch 可直接观察 `uart2_protocol_test_stats`
- 上行压测：STM32 同时以 `200Hz`（`5ms` 周期）主动回传 `type=0x81` 状态帧，payload 当前固定 `128` 字节，字段顺序如下：

```text
u32 stm_tick_ms
f32 roll
f32 pitch
f32 yaw
f32 gyro_x
f32 gyro_y
f32 gyro_z
f32 acc_x
f32 acc_y
f32 acc_z
6 * f32 joint_position
6 * f32 joint_velocity
6 * f32 joint_effort
u8  online_mask
u8  safety_state
u8  last_command_timeout
u8  reserved0
u32 comm_rx_error_count
u32 comm_crc_error_count
u32 can_error_count
```

- 编码：所有多字节字段当前都按 little-endian 打包
- 上位机模拟发送工具：`tools/uart_frame_sender.py`
- ROS 侧压测节点：`ros2_ws/src/wheel_leg_stm32_bridge/src/stm32_uart_stress_node.cpp`
- `stm32_uart_stress_node` 现已兼容旧 `48` 字节诊断状态帧和当前 `128` 字节正式状态帧，避免因版本切换误判链路失败

## 9. 验收场景

文档和后续实现按以下场景验收：

- 空帧或心跳通信稳定，可统计周期、丢包、CRC 错误和解析错误。
- 控制闭环保持 `100Hz` 时，测试 STM32 状态上报 `100Hz` 与 `200Hz` 的稳定性；若资源允许，再继续测试更高频率。
- STM32 上传 IMU 和六电机反馈，ROS 能稳定映射成 `/robot_state`。
- ROS 下发六电机小力矩，STM32 在悬空状态逐个验证方向、单位和限幅。
- 停止 ROS 命令后，STM32 在超时时间内进入安全输出。
- CAN 电机掉线、串口异常、failsafe 时，对应状态字段可观测。
- 控制器仍只依赖 `/robot_state` 和 `/joint_command`，不直接依赖 STM32 协议细节。

## 10. 当前 200Hz 状态上报瓶颈与优化方案

`2026-06-26` 的最新实测表明，STM32 侧把状态发送挂到 `Data_Task` 的 `5ms` 绝对周期后，单轮循环耗时约为 `5.58ms`。这意味着：

- `Data_Task` 自身已经超过 `5ms` 预算。
- `xTaskDelayUntil(..., 5)` 基本不会真实阻塞。
- 高优先级 `Data_Task` 会持续追赶节拍，低优先级任务更难获得 CPU。
- 当前问题已经从“调度位置不对”进一步收敛为“任务体本身太慢”。

### 10.1 当前主要耗时来源

当前确认有两块会直接挤占 `Data_Task` 周期预算：

1. 阻塞式 UART 状态帧发送

- 文件：`firmware/stm32/App/uart_protocol_test.cpp`
- 现状：状态帧发送使用 `HAL_UART_Transmit(&huart2, ...)`
- 问题：该调用是同步阻塞的，发送 `136` 字节左右状态帧时，任务会一直卡住直到发送完成或超时
- 影响：在 `921600 8N1` 下，单帧线速时间接近 `1.5ms`，这是 `5ms` 预算里最硬的一段阻塞

2. 软件 I2C 分散读取 IMU 寄存器

- 文件：`firmware/stm32/Hardware/MyI2C.c`
- 现状：`JY901S_Acc()`、`JY901S_Gyro()`、`JY901S_Angle()` 会分别多次调用 `I2C_ReadReg()`
- 问题：当前从 `0x34` 到 `0x3F` 的连续寄存器区间，被拆成 9 次独立 16 位寄存器读取
- 影响：每次 `I2C_ReadReg()` 都要走完整的 `start + address + reg + repeated start + read + stop` 流程；而软件 I2C 的 `SCL/SDA` 翻转还带 `Delay_us(1)`，累计开销明显

### 10.2 UART 发送优化方案

目标是把状态帧发送从“阻塞任务”改成“后台发送”，优先释放 `Data_Task` 的 CPU 占用。

建议方案：

1. 为 `USART2` 增加 TX DMA 通道

- 当前 `usart.c` 已初始化 `USART2` 和中断，但还没有为 `huart2` 绑定 DMA handle
- 需要补 `DMA_HandleTypeDef`
- 需要在 `HAL_UART_MspInit()` 中完成 DMA 初始化和 `__HAL_LINKDMA(&huart2, hdmatx, ...)`
- 需要补充对应 DMA IRQ

2. 将状态帧发送接口从阻塞发送改为非阻塞发送

- 首选：`HAL_UART_Transmit_DMA(&huart2, ...)`
- 过渡备选：`HAL_UART_Transmit_IT(&huart2, ...)`

3. 增加发送中的状态位，避免重入

- 增加类似 `g_tx_in_flight` 的标志
- 若上一帧尚未发送完成，本轮 `5ms` 周期直接跳过，避免多个发送重叠
- 统计跳过次数，便于后续评估链路余量

4. 将发送成功统计改到发送完成回调中

- 当前 `tx_frames`、`last_tx_seq` 等统计是在 `HAL_UART_Transmit()` 返回成功后立即更新
- 改为 DMA/IT 后，应在 `HAL_UART_TxCpltCallback()` 中确认本帧真正发完，再更新统计和序号

5. 避免发送时中断接收链路

- 当前实现为了绕过同步发送冲突，会在发送前 `HAL_UART_AbortReceive_IT()`，发送后再 `RestartReceive()`
- 如果改成 DMA/IT 发送，这种做法会让收发耦合更深，也更容易引入丢字节
- 更推荐保留 RX 中断持续工作，仅对 TX 帧缓冲做最小保护

预期收益：

- 直接去掉 `Data_Task` 里约 `1.5ms` 的同步串口阻塞
- 把状态帧发送的实际占用转为 DMA/中断后台完成
- 为 `5ms` 周期重新腾出最关键的一块预算

### 10.3 IMU 读取优化方案

目标是把当前“多次独立寄存器访问”压缩成“一次连续块读”，减少软件 I2C 事务数。

建议方案：

1. 在 `MyI2C` 层新增连续读接口

建议增加类似接口：

```c
void I2C_ReadRegs(uint8_t address, uint8_t start_reg, uint8_t *buf, uint16_t len);
```

行为要求：

- 先发送设备地址和起始寄存器地址
- 再 repeated start 切到读模式
- 连续读取 `len` 个字节
- 前 `len - 1` 个字节发送 ACK，最后 1 个字节发送 NACK
- 最后 `stop`

2. 在 `JY901S` 层新增一次性快照读取接口

建议增加类似接口：

```c
void JY901S_ReadSnapshot(...);
```

或直接一次性填充 `acc/gyro/angle` 九个量。

3. 直接读取 `0x34` 到 `0x3F`

这段地址本来就是连续区间：

- `0x34~0x36`：加速度
- `0x37~0x39`：角速度
- `0x3D~0x3F`：姿态角

当前实现虽然只实际使用其中部分寄存器，但先按整段连续读更简单稳妥。即使中间顺带读到未使用字段，也通常比多次小事务更省时。

4. 在任务里只做一次解析

- 连续读得到一段原始字节缓存
- 由 `JY901S` 层统一组合高低字节并完成比例换算
- `Data_Task` 只调用一次读取函数，不再逐项访问

预期收益：

- 明显减少 `start/stop/repeated-start` 次数
- 明显减少 `Delay_us(1)` 造成的累计开销
- 让 IMU 采样耗时从“多次小阻塞”收敛为“一次较短阻塞”

### 10.4 建议实施顺序

优先顺序建议固定为：

1. 先做 `USART2 TX DMA/IT`
- 这是当前最确定、最直接的减时项

2. 再做 `JY901S` 连续读
- 这是第二个主要耗时来源，也是让 `Data_Task` 真正回到 `5ms` 内的关键项

3. 两项完成后重新复测
- 记录 `Data_Task` 单轮耗时
- 记录 `200Hz` 状态帧是否稳定
- 记录 `tx_errors`、`crc_errors`、`sync_losses`、`rx_seq_gaps`

如果两项完成后 `Data_Task` 仍明显超过 `5ms`，再考虑把状态发送彻底挪出 `Data_Task`，单独放入独立 `State_Tx_Task`。

## 11. 树莓派 UART4 压测步骤

本阶段建议用树莓派 UART4 对接 STM32 USART2，在树莓派/ROS 环境里跑协议帧压测。目标是先确认物理串口链路、波特率和两侧手动解包稳定，再接入正式 `wheel_leg_stm32_bridge`。

### 11.1 接线

按 UART 交叉连接：

```text
Raspberry Pi UART4 TX  -> STM32 USART2 RX / PA3
Raspberry Pi UART4 RX  <- STM32 USART2 TX / PA2
Raspberry Pi GND       -- STM32 GND
```

注意：

- 必须共地。
- 两侧都应使用 `3.3V TTL` 电平。
- 不要把 TX 接 TX、RX 接 RX。
- 当前 STM32 USART2 默认 `921600 8N1`。

树莓派 UART 设备名以实际系统为准。当前工具默认会启用两个 UART overlay，并优先检测 `/dev/ttyAMA3`、`/dev/ttyAMA4`；本轮若指定 UART4，优先使用 `/dev/ttyAMA4`。

### 10.2 树莓派 UART 配置

在树莓派上执行：

```bash
cd ~/wheel_leg_ws
sudo ./tools/uart_loopback.py configure
sudo reboot
```

重启后查看 UART 状态：

```bash
cd ~/wheel_leg_ws
./tools/uart_loopback.py status
ls -l /dev/ttyAMA3 /dev/ttyAMA4
```

若 `/dev/ttyAMA4` 不存在，以 `status` 输出和 `/proc/device-tree/aliases` 为准确认 UART4 对应的 tty 设备。

### 10.3 ROS 环境下发送协议帧

加载 ROS/项目环境：

```bash
cd ~/wheel_leg_ws
source /opt/ros/jazzy/setup.bash
source ./ros2_ws/install/local_setup.bash
```

从树莓派 UART4 向 STM32 USART2 发送模拟 ROS 命令帧：

```bash
./tools/uart_frame_sender.py \
  --port /dev/ttyAMA4 \
  --baud 921600 \
  --rate-hz 200 \
  --payload-len 32 \
  --duration 60
```

如果系统中 UART4 不是 `/dev/ttyAMA4`，把 `--port` 改成实际设备。

如果希望直接在 ROS 2 图里运行压测节点，先编译：

```bash
cd ~/wheel_leg_ws/ros2_ws
colcon build --packages-select wheel_leg_stm32_bridge
source ./install/local_setup.bash
```

然后启动 ROS 侧串口压测节点：

```bash
ros2 run wheel_leg_stm32_bridge stm32_uart_stress_node \
  --ros-args \
  -p serial_device:=/dev/ttyAMA4 \
  -p baud_rate:=921600 \
  -p rate_hz:=200.0 \
  -p payload_len:=32 \
  -p report_period_sec:=1.0
```

该节点会持续发送与 STM32 当前测试入口一致的协议帧，并发布：

- `/stm32_bridge/uart_stress/status_text`
- `/stm32_bridge/uart_stress/counters`

其中 `counters` 字段顺序固定为：

```text
frames_attempted, frames_sent, bytes_sent, write_errors, partial_writes, deadline_misses, last_seq
```

ROS 侧重点关注：

- `frames_sent / frames_attempted` 是否持续接近 `1.0`
- `write_errors == 0`
- `partial_writes == 0`
- `deadline_misses` 是否持续增长

如果只看终端日志，不订阅 topic 也可以；节点每 2 秒会输出一次统计摘要。

当前 `stm32_uart_stress_node` 对 `/stm32_bridge/uart_stress/stm32_status` 的发布规则为：

- 若收到旧 `48` 字节诊断帧，字段顺序仍为：

```text
stm_tick_ms, rx_bytes, frames_ok, crc_errors, length_errors, sync_losses,
rx_seq_gaps, uart_errors, last_rx_seq, last_rx_type, last_rx_len,
min_frame_gap_ms, max_frame_gap_ms, last_rx_age_ms
```

- 若收到当前 `128` 字节正式状态帧，字段顺序为：

```text
stm_tick_ms, online_mask, safety_state, last_command_timeout,
comm_rx_error_count, comm_crc_error_count, can_error_count
```

### 10.4 STM32 侧观察项

在 Keil Watch 中观察：

```text
uart2_protocol_test_stats
```

重点看：

- `frames_ok`：成功解包帧数。
- `crc_errors`：CRC 错误计数。
- `length_errors`：长度错误计数。
- `sync_losses`：帧同步丢失计数。
- `uart_errors`：UART 硬件/HAL 错误计数。
- `last_seq`：最新帧序号。
- `min_frame_gap_ms` / `max_frame_gap_ms`：帧间隔范围。

正式测试前需要先清零统计。可以重新上电，或在调试器里调用：

```cpp
UartProtocolTest_ResetStats();
```

### 10.5 测试矩阵

先固定 `921600` 波特率，按以下顺序测试：

```bash
./tools/uart_frame_sender.py --port /dev/ttyAMA4 --baud 921600 --rate-hz 200 --payload-len 16 --duration 60
./tools/uart_frame_sender.py --port /dev/ttyAMA4 --baud 921600 --rate-hz 200 --payload-len 32 --duration 60
./tools/uart_frame_sender.py --port /dev/ttyAMA4 --baud 921600 --rate-hz 200 --payload-len 64 --duration 60
./tools/uart_frame_sender.py --port /dev/ttyAMA4 --baud 921600 --rate-hz 200 --payload-len 96 --duration 60
```

若 `200Hz` 稳定，再提高频率：

```bash
./tools/uart_frame_sender.py --port /dev/ttyAMA4 --baud 921600 --rate-hz 300 --payload-len 64 --duration 60
./tools/uart_frame_sender.py --port /dev/ttyAMA4 --baud 921600 --rate-hz 500 --payload-len 64 --duration 60
```

若 `921600` 稳定，再测试更高波特率。两侧必须同时改波特率：

```bash
./tools/uart_frame_sender.py --port /dev/ttyAMA4 --baud 1000000 --rate-hz 200 --payload-len 64 --duration 60
./tools/uart_frame_sender.py --port /dev/ttyAMA4 --baud 2000000 --rate-hz 200 --payload-len 64 --duration 60
```

### 10.6 判定标准

每轮测试结束后，记录发送端输出和 STM32 侧统计。

通过标准：

- `frames_ok` 接近发送端 `sent`。
- `crc_errors == 0`。
- `length_errors == 0`。
- `uart_errors == 0`。
- `last_seq` 持续递增，没有明显跳变。
- `max_frame_gap_ms` 没有异常大尖峰。

说明：

- 如果测试前用串口调试助手发过普通文本，`sync_losses` 可能已经增加；这不代表协议帧丢包。正式压测前先清零统计。
- 当前 STM32 接收实现是单字节中断接收，用于先验证链路和最坏情况下的解包开销。若高频或大 payload 下 `uart_errors` 增加，下一步改为 DMA 环形缓冲，但仍然保持手动解包。
- ROS 侧若在 `921600` 下已经出现 `write_errors`、`partial_writes` 或明显的 `deadline_misses`，先优先排查 UART4 设备名、串口权限、overlay 配置和串口占用情况。

## 11. 与现有文档关系

- `docs/protocol.md` 继续固定仿真和实机共用的 Topic、命名、单位和方向边界。
- `docs/stm32_hardware_integration.md` 继续作为硬件接入与实机闭环总任务。
- 本文档专门记录 STM32 与 ROS 第一版通信内容、字段和验收任务。

## 12. 实测记录

### 2026-06-25 UART4 <-> USART2

- 实测端口：树莓派 `/dev/ttyAMA4` 对接 STM32 `USART2`
- 实测波特率：`921600 8N1`
- 旧版 `stm32_uart_stress_node` 按 `48` 字节状态帧解包时表现为 `rx_ok=0`、`rx_sync` 持续增长，但原始串口抓包已能看到合法帧头 `a5 5a 81 80`，说明当时不是物理链路中断，而是 ROS 侧仍按旧状态帧格式解析
- 正式 `wheel_leg_stm32_bridge_node` 现场验证通过：`/stm32_bridge/status_text` 为 `state=ok state_stale=false`，`/stm32_bridge/counters` 观测到 `rx_frames_ok=849` 且 `rx_crc_errors=0`、`rx_length_errors=0`、`rx_sync_losses=0`
- 继续运行后，`/stm32_bridge/counters` 观测到 `rx_frames_ok=3101`，说明 STM32 到 ROS 上行通信稳定
- ROS 侧向 `/joint_command` 发布全零六关节力矩命令后，bridge 侧 `tx_frames_sent` 从 `0` 增加到 `4`，说明 ROS 到 STM32 下行写串口正常
- 当未持续发送命令时，状态中的 `safety=timeout` 为 STM32 `100ms` 命令超时保护的预期行为，不作为链路故障判据
- 本次结论：`wheel_leg_stm32_bridge_node` 已可与当前 STM32 固件正常通信；`stm32_uart_stress_node` 需要与 `128` 字节状态帧保持兼容，避免误报

### 12.1 2026-06-25 UART4 <-> USART2 首轮联调

测试环境：

- Raspberry Pi 串口设备：`/dev/ttyAMA4`
- STM32 串口：`USART2`
- 参数：`921600 8N1`
- ROS 侧压测节点：`stm32_uart_stress_node`
- 下行发送参数：`200Hz`、`payload_len=32`
- STM32 上行状态帧：`type=0x81`、`200Hz`、`payload_len=48`

第一次测试现象：

- 当时 Raspberry Pi 与 STM32 未共地。
- ROS 侧下行发送统计正常，`write_errors=0`、`partial_writes=0`。
- 但 ROS 侧无法按 `0xA5 0x5A` 帧格式解出任何合法 `type=0x81` 状态帧。
- `/dev/ttyAMA4` 上能读到原始字节流，但内容表现为噪声，说明物理链路参考地缺失会直接导致协议解包失败。

修正后结果：

- 补上 `Raspberry Pi GND -- STM32 GND` 共地后，链路恢复正常。
- ROS 侧可以稳定解出 STM32 回传的合法 `type=0x81` 状态帧。
- 这次联调确认了“必须共地”不仅是接线规范，也是本链路是否能建立协议通信的前置条件。

### 12.2 2026-06-25 5 秒复测结果

复测参数保持不变：`/dev/ttyAMA4`、`921600 8N1`、ROS 下行 `200Hz`、`payload_len=32`。

ROS 侧发送统计：

- `attempted=801`
- `sent=801`
- `write_errors=0`
- `partial_writes=0`
- `deadline_misses=0`

ROS 侧接收 STM32 状态帧统计：

- `valid_type81=1002`
- `crc_err=0`

从 STM32 `type=0x81` 状态帧解出的 5 秒窗口增量：

- `delta_stm_tick_ms = 5005`
- `delta_seq = 1001`
- `delta_frames_ok = 880`
- `delta_rx_bytes = 35200`
- `delta_crc_errors = 0`
- `delta_length_errors = 0`
- `delta_sync_losses = 0`
- `delta_rx_seq_gaps = 1`
- `delta_uart_errors = 0`

STM32 侧 `USART1` 调试打印观察：

- 早期一次误判记录中，STM32 侧表现为 `tx` 持续增长，但 `rx_ok`、`last_rx` 长时间不变；后续确认当时 ROS 侧压测节点未启动，因此只能证明 STM32 上行发送正常，不能据此判断链路整体异常。
- 在 ROS 压测节点正确启动后的稳定窗口内，STM32 侧打印可见 `rx_ok` 与 `tx` 同步持续增长，`last_rx`、`last_tx` 也连续前进，说明双向链路都在工作。
- 现场串口打印样例如下：

```text
uart2 rx_ok=4397 rx_crc=94 rx_gap=47 tx=4600 tx_err=0 last_rx=5991 last_tx=4599
uart2 rx_ok=4597 rx_crc=94 rx_gap=47 tx=4800 tx_err=0 last_rx=6191 last_tx=4799
uart2 rx_ok=4797 rx_crc=94 rx_gap=47 tx=5000 tx_err=0 last_rx=6391 last_tx=4999
uart2 rx_ok=4997 rx_crc=94 rx_gap=47 tx=5200 tx_err=0 last_rx=6591 last_tx=5199
uart2 rx_ok=5197 rx_crc=94 rx_gap=47 tx=5400 tx_err=0 last_rx=6791 last_tx=5399
uart2 rx_ok=5397 rx_crc=94 rx_gap=47 tx=5600 tx_err=0 last_rx=6991 last_tx=5599
uart2 rx_ok=5597 rx_crc=94 rx_gap=47 tx=5800 tx_err=0 last_rx=7191 last_tx=5799
uart2 rx_ok=5797 rx_crc=94 rx_gap=47 tx=6000 tx_err=0 last_rx=7391 last_tx=5999
uart2 rx_ok=5997 rx_crc=94 rx_gap=47 tx=6200 tx_err=0 last_rx=7591 last_tx=6199
uart2 rx_ok=6197 rx_crc=94 rx_gap=47 tx=6400 tx_err=0 last_rx=7791 last_tx=6399
uart2 rx_ok=6345 rx_crc=117 rx_gap=59 tx=6600 tx_err=0 last_rx=7988 last_tx=6599
```

由 STM32 侧打印可以直接得到：

- `tx_err=0`，说明 STM32 `USART2` 上行状态帧发送没有出现 HAL 发送失败。
- `rx_ok` 持续增长，说明 ROS 下行压测帧已被 STM32 持续成功解包。
- `last_rx` 与 `last_tx` 基本按固定步长推进，说明两侧周期配置和序号推进逻辑一致。
- `rx_crc`、`rx_gap` 虽然在稳定窗口内仍有小幅增长，但相对累计成功帧占比很低；当前状态可以认为“链路已打通，可进入后续 ROS 映射和长时间稳定性观察阶段”。

当前结论：

- UART4 到 STM32 USART2 的双向协议通信已打通。
- STM32 到 ROS 的 `type=0x81` 上行状态帧在本轮 5 秒复测中稳定。
- ROS 到 STM32 的命令帧发送链路正常，STM32 已持续成功解包下行命令帧。
- 本轮窗口内 `CRC`、长度、同步和 `UART` 错误新增均为 `0`，链路质量明显优于未共地时的失败状态。
- `delta_rx_seq_gaps = 1` 说明仍存在轻微偶发序号跳变，后续可在更长时间窗口继续观察，但当前已经满足“首轮链路打通”目标。

### 12.3 2026-06-25 正式通信节点 V1 代码落地

本轮已按“正式通信节点 V1”计划完成第一版代码接线，但当前结论仍属于“代码已落地，待板级联调验证”，不是最终硬件验收结论。

本轮已完成的代码改动：

- `wheel_leg_stm32_bridge` 已从占位节点切换为正式串口 bridge。
- ROS 侧 bridge 已实现 `0xA5 0x5A` 帧头、`type/len/seq/payload/crc16-ccitt` 解包与命令打包。
- ROS 侧 bridge 已发布：
  - `/robot_state`
  - `/imu`
  - `/joint_states`
- ROS 侧 bridge 已订阅：
  - `/joint_command`
  - `/rc/status`
- `wheel_leg_msgs/msg/RcStatus.msg` 已新增 `estop_active` 字段。
- `wheel_leg_rc` 已将急停默认通道固定为 `channel 7`，并按“低值端触发急停”输出 `estop_active=true`。
- bridge 已把 `RC ch7` 急停并入下行命令帧门控：
  - `estop_active=true` 时强制下发 `estop=1`
  - 同时强制 `enable=0`
  - 六路力矩清零
- `hw.launch.py` 已增加正式 bridge 的串口和发布参数透传。
- `rc.launch.py` 已增加 `estop_channel` 与 `estop_active_below` launch 参数，默认保持 `7 / true`。

STM32 侧本轮已完成的代码改动：

- `uart_protocol_test.cpp` 已从纯压测 payload 升级为正式协议对端。
- STM32 侧已实现下行命令帧解析：
  - `enable`
  - `estop`
  - 六路 `float32` 力矩命令
- STM32 侧已实现基础执行层保护：
  - 命令超时失能
  - 急停失能
  - 六路力矩限幅
  - 单周期斜率限制
- STM32 侧已改为周期回传真实状态帧：
  - `stm_tick_ms`
  - IMU 角度/角速度/加速度
  - 六电机位置/速度/反馈力矩
  - `online_mask`
  - `safety_state`
  - `last_command_timeout`
  - 通信与 CAN 诊断计数
- `Car.cpp` 已改为从通信模块读取六路执行力矩，而不是继续使用单一测试 `target`。

本轮已完成的软件验证：

- ROS 侧已在本机完成以下包编译通过：
  - `wheel_leg_msgs`
  - `wheel_leg_bridge`
  - `wheel_leg_rc`
  - `wheel_leg_stm32_bridge`
  - `wheel_leg_control`
  - `wheel_leg_bringup`
- `RcStatus` 新接口已可正确导出并被 ROS 侧包依赖。

本轮尚未完成的验证：

- 尚未在当前环境完成 STM32 工程编译验证。
- 尚未完成正式 `type=0x81` 新 payload 与 ROS bridge 的上板联调。
- 尚未完成 `RC ch7 -> bridge -> command estop -> STM32 safety_state` 的整链硬件验证。
- 尚未完成六电机方向、单位、限幅和斜率限制的悬空验证。
- 尚未完成 `/robot_state` 新来源下的 100Hz 实机闭环联调。

因此当前阶段状态应理解为：

- 压测链路：`[v] 已打通`
- 正式 V1 代码：`[x] 已实现`
- 正式 V1 硬件闭环：`[~] 待联调验证`

### 12.4 2026-06-25 正式状态帧发送失败问题定位

本节记录 `2026-06-25` 针对正式 `type=0x81` 状态帧发送异常的定位过程、结论以及临时绕过方案。

现象一：ROS 侧 `/stm32_bridge/counters` 表现异常

- 计数样例：`[0, 14576, 0, 1092082, 0, 0, 0]`
- 对应含义：
  - `rx_frames_ok = 0`
  - `rx_crc_errors` 持续增长
  - `rx_length_errors = 0`
  - `rx_sync_losses` 很大
  - `tx_frames_sent = 0`
- 这说明 ROS 侧没有成功解出任何一帧合法状态帧，但当时还不能仅凭这一点判断根因在 ROS 解包侧。

现象二：树莓派直接抓取 `/dev/ttyAMA4` 原始上行字节

- 在停掉 `hw.launch.py` 后，直接读取 `/dev/ttyAMA4` 的 256 字节原始流。
- 字节流中频繁出现 `a5 5a 81 80`，表面上看像是正式 `type=0x81` 状态帧。
- 但按当前源码约定的 `payload_len=128`、`CRC(type,len,seq,payload)` 去校验时，原始流无法通过 CRC。
- 同时，连续帧头实际出现的间距约为 `75` 到 `77` 字节，而不是新协议应有的完整帧长度。
- 因此，ROS 侧看到的是“看起来像新帧头”的字节流，但内容与当前正式协议定义并不一致。

现象三：STM32 调试打印直接显示 `USART2` 上行发送失败

调试打印样例：

```text
uart2 rx_ok=0 rx_crc=0 rx_gap=0 tx=0 tx_err=595 last_rx=0 last_tx=0
uart2 rx_ok=0 rx_crc=0 rx_gap=0 tx=0 tx_err=694 last_rx=0 last_tx=0
uart2 rx_ok=0 rx_crc=0 rx_gap=0 tx=0 tx_err=793 last_rx=0 last_tx=0
uart2 rx_ok=0 rx_crc=0 rx_gap=0 tx=0 tx_err=892 last_rx=0 last_tx=0
uart2 rx_ok=0 rx_crc=0 rx_gap=0 tx=0 tx_err=991 last_rx=0 last_tx=0
uart2 rx_ok=0 rx_crc=0 rx_gap=0 tx=0 tx_err=1090 last_rx=0 last_tx=0
uart2 rx_ok=0 rx_crc=0 rx_gap=0 tx=0 tx_err=1189 last_rx=0 last_tx=0
uart2 rx_ok=0 rx_crc=0 rx_gap=0 tx=0 tx_err=1288 last_rx=0 last_tx=0
uart2 rx_ok=0 rx_crc=0 rx_gap=0 tx=0 tx_err=1387 last_rx=0 last_tx=0
```

这组打印可以直接说明：

- `tx=0`：STM32 `USART2` 正式状态帧一帧都没有成功发出。
- `tx_err` 持续增长：发送阶段持续失败，失败点发生在 ROS 侧解包之前。
- `rx_ok=0`、`last_rx=0`：本轮同时也没有收到 ROS 下行的合法命令帧。

本轮结论：

- 这一次问题的根因不是 ROS 侧解包逻辑。
- 根因已经前移到 STM32 侧更前面的发送链路：`USART2` 正式状态帧发送本身持续失败。
- ROS 侧观察到的 CRC 错误和同步丢失，更应视为下游症状，而不是主因。

进一步诊断：

- 在 STM32 侧为 `HAL_UART_Transmit(&huart2, ...)` 增加 `HAL_StatusTypeDef`、`HAL_UART_GetError()`、`huart2.gState`、`huart2.RxState` 诊断打印后，失败样例表现为：

```text
uart2 rx_ok=0 rx_crc=0 rx_gap=0 tx=0 tx_err=2377 busy=0 to=2377 herr=0 tx_st=3 tx_ec=0 g=32 rxs=34 last_rx=0 last_tx=0
uart2 rx_ok=0 rx_crc=0 rx_gap=0 tx=0 tx_err=2476 busy=0 to=2476 herr=0 tx_st=3 tx_ec=0 g=32 rxs=34 last_rx=0 last_tx=0
uart2 rx_ok=0 rx_crc=0 rx_gap=0 tx=0 tx_err=2575 busy=0 to=2575 herr=0 tx_st=3 tx_ec=0 g=32 rxs=34 last_rx=0 last_tx=0
```

- 这说明失败类型不是 `HAL_BUSY` 或 `HAL_ERROR`，而是稳定的 `HAL_TIMEOUT`。
- `tx_ec=0` 说明当时没有新增 UART 硬件错误码。
- `g=32` 对应 `HAL_UART_STATE_READY`，`rxs=34` 对应 `HAL_UART_STATE_BUSY_RX`，也就是 `USART2` 正长期挂着 `HAL_UART_Receive_IT(..., 1)` 的接收状态。
- 该组合说明问题已进一步收敛到：同一个 `huart2` 上，持续 `Receive_IT` 与当前阻塞式 `HAL_UART_Transmit` 的组合会使发送侧在等待标志位时超时。

临时绕过修改：

- 在 `firmware/stm32/App/uart_protocol_test.cpp` 的 `TrySendStatusFrame()` 中，若发现 `huart2.RxState == HAL_UART_STATE_BUSY_RX`，先执行 `HAL_UART_AbortReceive_IT(&huart2)`。
- 状态帧发送完成后，再调用 `RestartReceive()` 恢复单字节中断接收。
- 同时把发送超时从 `5ms` 放宽到 `20ms`，并打印 `USART2->ISR`、`CR1`、`CR3` 快照。

应用绕过后的调试打印样例：

```text
uart2 rx_ok=0 rx_crc=0 rx_gap=0 tx=1519 tx_err=0 busy=0 to=0 herr=0 abort_rx=1519 tx_st=0 tx_ec=0 g=32 rxs=34 isr=006000d0 cr1=0000000d cr3=00000000 last_rx=0 last_tx=1518
uart2 rx_ok=0 rx_crc=0 rx_gap=0 tx=1585 tx_err=0 busy=0 to=0 herr=0 abort_rx=1585 tx_st=0 tx_ec=0 g=32 rxs=34 isr=006000d0 cr1=0000000d cr3=00000000 last_rx=0 last_tx=1584
uart2 rx_ok=0 rx_crc=0 rx_gap=0 tx=1651 tx_err=0 busy=0 to=0 herr=0 abort_rx=1651 tx_st=0 tx_ec=0 g=32 rxs=34 isr=006000d0 cr1=0000000d cr3=00000000 last_rx=0 last_tx=1650
uart2 rx_ok=0 rx_crc=0 rx_gap=0 tx=1717 tx_err=0 busy=0 to=0 herr=0 abort_rx=1717 tx_st=0 tx_ec=0 g=32 rxs=34 isr=006000d0 cr1=0000000d cr3=00000000 last_rx=0 last_tx=1716
```

绕过后的结论：

- `tx_err=0` 且 `tx` 持续增长，说明 STM32 `USART2` 正式状态帧上行发送已恢复正常。
- `abort_rx` 与 `tx` 基本同步增长，进一步支持“持续 `Receive_IT` 与阻塞式 `Transmit` 冲突”这一根因判断。
- 该修改证明本轮主要问题不在 CRC 算法、协议打包内容或物理 TX 线本身，而在 STM32 侧同一 UART 的收发方式组合。
- 当前日志仍保持 `rx_ok=0`、`last_rx=0`，说明剩余问题已转移到下行命令未进入 STM32，不能再把当前链路问题归因到上行发送。

当前记录状态：

- 已完成正式状态帧发送失败的根因缩小与临时绕过验证。
- 当前 `USART2` 上行状态发送恢复正常。
- 当前仍待继续排查 ROS/树莓派到 STM32 的下行命令接收链路。
- 当前“发送前 abort 接收、发送后重启接收”方案只作为联调阶段临时绕过，不建议直接作为最终正式实现。

### 10.5 `2026-06-26` 当晚继续跟进后的最终根因与修复

在继续把阻塞发送改为 `HAL_UART_Transmit_DMA(&huart2, ...)` 后，现场日志一度表现为：

```text
uart2 rx_ok=0 rx_crc=0 rx_gap=0 tx=0 tx_err=0 ... g=33 rxs=34 ... cr3=00000081 ... skip=1999
```

后续又补充了 DMA/DMAMUX 诊断字段，典型样例如下：

```text
uart2 rx_ok=0 rx_crc=0 rx_gap=0 tx=0 tx_err=0 ... g=33 rxs=34 ... cr3=00000081 ...
dma_st=2 ndtr=0 dcr=00000000 dfcr=00000000 lisr=00000000 hisr=00000000 mux=0000002c hdmatx=24006930 parent=24006808 skip=2199
```

这组数据说明：

- `g=33` 对应 `HAL_UART_STATE_BUSY_TX`，`rxs=34` 对应 `HAL_UART_STATE_BUSY_RX`。
- `skip` 持续增长，说明第一帧 DMA 发送请求发起后，`g_tx_in_flight` 长期没有被发送完成回调清零。
- 但 `ndtr=0`、`dcr=0`、`lisr=0`、`hisr=0` 又表明 DMA stream 自身并没有真正进入工作状态。
- `hdmatx` 和 `parent` 都非零，说明 `__HAL_LINKDMA()` 已经生效，不是 UART handle 没绑 DMA。

最终根因不是协议内容，也不是 `HAL_UART_Transmit_DMA()` 本身，而是 **实际参与构建的工程入口已经从 CubeMX 默认的 `main.c` 切到了自定义 `main.cpp`，但 `main.cpp` 没有同步调用 `MX_DMA_Init()`；同时 Keil 工程文件里也没有把 `dma.c` 编进目标**。

核对结果如下：

- Keil 工程实际入口文件：`firmware/stm32/Core/Src/main.cpp`
- `main.cpp` 原始版本初始化了 `GPIO/FDCAN/USART/TIM`，但没有调用 `MX_DMA_Init()`
- `firmware/stm32/MDK-ARM/wheel_leg_robort.uvprojx` 初始状态也未收录 `../Core/Src/dma.c`

最终修复内容：

1. 在 `firmware/stm32/Core/Src/main.cpp` 中补 `#include "dma.h"`。
2. 在外设初始化序列中补 `MX_DMA_Init();`，再初始化 `USART2`。
3. 在 `firmware/stm32/MDK-ARM/wheel_leg_robort.uvprojx` / `.uvoptx` 中把 `../Core/Src/dma.c` 正式加入工程。
4. 保留 `USART2_IRQHandler() -> HAL_UART_IRQHandler(&huart2)`，保证 DMA 发送完成后的 UART `TC` 收尾中断能进入 HAL。
5. 当前 HAL 版本里没有 `__HAL_RCC_DMAMUX1_CLK_ENABLE()` 宏，因此不要额外保留这行；本版本按 `MX_DMA_Init()` 中现有 `__HAL_RCC_DMA1_CLK_ENABLE()` 工作即可。

应用最终修复后的上行日志样例：

```text
uart2 rx_ok=0 rx_crc=0 rx_gap=0 tx=1799 tx_err=0 busy=0 to=0 herr=0 abort_rx=0 tx_st=0 tx_ec=0 g=33 rxs=34 isr=00600010 cr1=0000002d cr3=00000081 last_rx=0 last_tx=1798 dma_st=2 ndtr=130 dcr=0010045f dfcr=00000000 lisr=00000000 hisr=00000000 mux=0000002c hdmatx=24006930 parent=24006808 skip=0
uart2 rx_ok=0 rx_crc=0 rx_gap=0 tx=1999 tx_err=0 busy=0 to=0 herr=0 abort_rx=0 tx_st=0 tx_ec=0 g=33 rxs=34 isr=00600010 cr1=0000002d cr3=00000081 last_rx=0 last_tx=1998 dma_st=2 ndtr=129 dcr=0010045f dfcr=00000000 lisr=00000000 hisr=00000000 mux=0000002c hdmatx=24006930 parent=24006808 skip=0
```

最终结论：

- `tx` 与 `last_tx` 每秒增加约 `200`，说明 STM32 `USART2` 上行正式状态帧已经稳定跑到 `200Hz`。
- `tx_err=0`，说明 DMA 发送链路已恢复正常。
- `skip=0`，说明当前 `5ms` 周期下没有因为上一帧未发完而跳过周期。
- 日志中偶尔看到 `g=33`、`cr3=0x81`、`ndtr` 非零，属于采样瞬间落在 DMA 正在发送本帧的中间态，不再表示卡死。
- 在 ROS 侧未开启时，`rx_ok=0`、`last_rx=0` 是预期现象，不能再据此判断上行失败。

当前建议状态更新为：

- STM32 单边上行 `200Hz` 已验证通过。
- 前述“发送前 abort 接收、发送后重启接收”的联调绕过方案可以保留在文档中作为阶段性排查记录，但不再作为当前主线方案。
- 下一步应切回 ROS 侧联调，继续验证 `wheel_leg_stm32_bridge` 在 `200Hz` 上行下的接收稳定性、CRC/长度错误计数，以及 `/joint_command` 下行接收情况。

### 10.6 本阶段收口

截至 `2026-06-26`，本文档中“STM32 上行状态帧提频到 `200Hz`”这一部分可以正式收口，结论如下：

- `USART2` 状态帧上行已经稳定达到 `200Hz`
- 当前实现采用 `HAL_UART_Transmit_DMA(&huart2, ...)`
- `Data_Task` 内 IMU 读取已改为单次快照读取，不再保留 9 次分散寄存器访问
- 当前 `5ms` 周期下 `skip=0`，说明没有因为上一帧未发送完成而丢周期
- 当前阶段已不再需要依赖“发送前 abort RX、发送后重启 RX”的临时绕过方案

因此，`200Hz` 这一部分的后续工作不再是继续追查 STM32 单边上行发送本身，而是转入：

- ROS 侧 `wheel_leg_stm32_bridge` 接收稳定性验证
- ROS 下行 `/joint_command -> STM32` 接收链路验证
- 在 `200Hz` 上行、`100Hz` 控制闭环条件下的整链联调
