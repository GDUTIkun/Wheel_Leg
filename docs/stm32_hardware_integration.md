# STM32 硬件接入与实机闭环任务

## 1. 任务目标

在 100Hz 仿真平衡基线已经可以站稳的基础上，进入 STM32 / 实机硬件接入阶段。

本任务的目标不是立刻完成最终实机参数，而是先打通真实硬件状态上报、控制命令下发、安全保护和基础方向校验，让后续 100Hz 实机调参有可靠输入。

## 2. 当前输入

- 100Hz 仿真可站立基线。
- `100hz_balance_notes.md` 中记录的传感器、滤波、LQR 和执行器输出处理原则。
- 已冻结的 RC 遥控输入与 failsafe 链路。
- 当前 ROS2 控制接口与仿真接口边界。
- STM32 侧电机、编码器、IMU 和底层安全保护能力。

## 3. 当前状态

- `[~]` 进入 STM32 / 实机硬件接入阶段。
- `[!]` 100Hz 仿真参数等待硬件接入后重新调参。

当前判断：

- 100Hz 仿真已能稳定站立，先作为可运行基线冻结。
- 当前不继续只在仿真中追求最终参数。
- 实机参数应在真实 IMU、编码器、电机响应和通信延迟接入后重新小步调整。

## 4. 重点任务

1. 通信链路
   - Raspberry Pi 与 STM32 的通信收发稳定。
   - 能观察通信周期、丢包、超时和错误计数。

2. 状态上报
   - 编码器角度、角速度、电机反馈状态可映射到 ROS2。
   - IMU 姿态和角速度可映射到 ROS2。
   - 单位统一为控制器期望的 `rad`、`rad/s`、`m/s`、`N*m` 等口径。

3. 命令下发
   - `/joint_command` 能下发到 STM32。
   - 电机命令方向、限幅、斜率限制和停机行为可验证。
   - 控制器不直接依赖 STM32 协议细节。

4. 安全保护
   - 通信超时进入安全状态。
   - 遥控 failsafe 可以阻断或降级控制输出。
   - 电机输出有明确最大值和变化率限制。
   - 初次调试支持悬空、低力矩、单关节验证。

5. 100Hz 实机闭环准备
   - 实测控制周期接近 `0.01s`。
   - 周期抖动可记录。
   - 传感器滤波和速度估计使用实际 `dt`。

## 5. 上硬件前检查

上硬件前至少确认：

- pitch 正方向与仿真一致。
- pitch_rate 正方向与 LQR 状态定义一致。
- 当前角度解算口径：角度由角速度积分得到；实机观测为向左转动角度正向增加，向右转动角度反向减小。
- 左右腿 `phi` 与 `phi_rate` 方向一致。
- 轮子前进方向与 `base_link.velocity` 正方向一致。
- 髋关节、膝关节零点和仿真 offset 对齐。
- VMC 输出到电机后的力矩方向正确。
- 电机最大力矩和变化率不比仿真基线更激进。
- 控制使能前，滤波器状态能初始化到当前测量值。
- 急停、failsafe、通信超时都有可验证行为。

## 6. 初期调参原则

- 先悬空验证方向，再轻触地验证支撑力，最后再尝试站立。
- 先小力矩、小命令、短时间闭环，不直接恢复完整速度和转向能力。
- 先确认 `pitch_rate`、`phi_rate`、`base_velocity` 可信，再调 LQR 增益。
- 若出现 pitch 高频震荡，优先检查 IMU 方向、gyro 滤波、`pitch_rate` 增益和执行器斜率限制。
- 若出现缓慢前后跑偏，优先检查 `target_phi`、`target_pitch`、速度方向和距离积分。
- 若出现单侧压低，优先检查左右腿角度、roll、VMC 映射和左右电机方向。

## 7. 每轮记录模板

```text
- 轮次：
- 硬件状态：
- 修改参数/代码：
- 测试动作：
- 观察数据：
- 安全状态：
- 结论：
- 下一步：
```

## 8. 阶段完成标准

- STM32 通信链路稳定可观测。
- 真实传感器状态与 ROS2 状态接口语义一致。
- 电机命令方向、单位、限幅和安全停机已验证。
- 100Hz 实际控制周期和 `dt` 处理已确认。
- 可以在低风险条件下进入实机站立闭环参数调整。

## 9. 通信完成后的测试方法与流程

通信测试完成后，当前阶段不直接进入整机闭环，而是先做“悬空低力矩单关节验证”。

目标：

- 验证 `/joint_command -> bridge -> STM32 -> 电机` 下行链路持续有效。
- 验证六电机方向、单位、限幅和停机行为。
- 验证 `/joint_states`、`/imu`、`/robot_state` 的上行观测与实际动作一致。
- 验证 `RC ch7` 急停、命令超时和 bridge 侧状态监控工作正常。

### 9.1 测试前准备

执行前先确认：

- 机器人处于悬空或可靠支撑状态，轮子和腿部不会意外着地。
- 电机周围无工具、线缆和手部干涉。
- 树莓派与 STM32 已共地。
- 串口设备仍为 `/dev/ttyAMA4`，STM32 对应 `USART2`。
- 遥控器已上电，`RC ch7` 急停动作明确可用。
- bridge、STM32 固件和当前协议版本一致。
- 首次测试保持小力矩，例如 `0.1 ~ 0.2 N*m`。

### 9.2 启动流程

1. 启动正式 bridge，但先关闭控制器，避免 `/joint_command` 被控制器覆盖：

```bash
ros2 launch wheel_leg_bringup hw.launch.py use_controller:=false command_enable:=true
```

2. 在第二个终端观察 bridge 状态：

```bash
ros2 topic echo /stm32_bridge/status_text
```

3. 在第三个终端观察 bridge 计数器：

```bash
ros2 topic echo /stm32_bridge/counters
```

4. 在第四个终端观察关节反馈：

```bash
ros2 topic echo /joint_states
```

5. 如需同时观察机体状态，可额外打开：

```bash
ros2 topic echo /robot_state
```

### 9.3 单关节低力矩命令发送

当前推荐使用 `joint_command_probe_node` 做单关节验证。

固定正向小力矩示例：

```bash
ros2 run wheel_leg_stm32_bridge joint_command_probe_node --ros-args \
  -p joint_name:=left_hip \
  -p effort_nm:=0.2 \
  -p publish_rate_hz:=20.0 \
  -p mode:=constant
```

正负交替方波示例：

```bash
ros2 run wheel_leg_stm32_bridge joint_command_probe_node --ros-args \
  -p joint_name:=left_hip \
  -p effort_nm:=0.2 \
  -p publish_rate_hz:=20.0 \
  -p mode:=square \
  -p square_period_sec:=2.0
```

参数说明：

- `joint_name`：目标关节名，只允许规范名
- `effort_nm`：目标力矩幅值，建议从 `0.1` 或 `0.2` 开始
- `publish_rate_hz`：重复下发频率，当前建议 `20Hz`
- `mode=constant`：持续给定单一方向力矩
- `mode=square`：按周期在正负力矩之间切换
- `square_period_sec`：方波完整周期

### 9.4 推荐测试顺序

建议按以下顺序逐个测试，每次只动一个关节，其余五路保持零力矩：

1. `left_hip`
2. `left_knee`
3. `left_wheel`
4. `right_hip`
5. `right_knee`
6. `right_wheel`

每个关节建议按同样步骤执行：

1. 先用 `mode=constant` 验证正向小力矩。
2. 观察动作方向与 `/joint_states.velocity` 正方向是否一致。
3. 再改成负向小力矩，确认反向动作。
4. 最后用 `mode=square` 做短时间正负切换，观察限幅、响应和停机恢复是否稳定。

### 9.5 现场观察项

每轮测试至少观察以下内容：

- `/stm32_bridge/status_text` 中应保持 `state=ok`、`state_stale=false`
- `/stm32_bridge/counters` 中 `rx_frames_ok` 应持续增长
- `/stm32_bridge/counters` 中 `rx_crc_errors`、`rx_length_errors`、`rx_sync_losses` 不应明显持续增长
- `/joint_states` 中目标关节的位置和速度应与实际动作一致
- `/robot_state` 中与该动作相关的量不应出现明显跳变或异常漂移
- 若切到反向力矩，实际动作和速度符号也应同步反向
- 目标关节外的其他五个关节不应出现危险联动

### 9.6 安全项验证

在单关节方向确认后，必须补做以下安全验证：

1. 急停验证

- 维持单关节小力矩输出。
- 触发 `RC ch7` 急停。
- 预期行为：
  - bridge 状态中的 `estop=true`
  - 下行 `enable=0`
  - 电机输出立即归零或失能
  - 关节停止继续加速

2. 命令超时验证

- 关闭 `joint_command_probe_node`。
- 在 STM32 命令超时窗口后观察：
  - `safety=timeout` 或等价安全状态出现
  - 电机输出归零或失能
  - `last_command_timeout` 相关状态可观测

3. 限幅与斜率限制验证

- 将 `effort_nm` 从 `0.1` 逐步增加到 `0.2`、`0.3`
- 不直接大步跳到高力矩
- 观察实际响应是否平滑，是否存在异常冲击

### 9.7 异常时处理顺序

若测试中出现异常，按以下顺序排查：

1. 立即触发急停或关闭探针节点。
2. 确认机器人仍保持悬空和安全状态。
3. 查看 `/stm32_bridge/status_text` 是否变为 `state_stale=true`、`safety=timeout`、`estop=true` 等。
4. 查看 `/stm32_bridge/counters` 是否出现 CRC、长度、同步错误快速增长。
5. 若动作方向与预期相反，先记录关节名、正负力矩方向、实际动作方向，再回到映射层修正。
6. 若目标关节不动但计数器正常，优先排查 STM32 执行层、限幅、使能位和电机在线状态。
7. 若上行状态异常漂移，优先排查编码器零点、方向、IMU 方向和单位映射。

### 9.8 单轮记录建议

每个关节验证完成后，建议按下面格式记录：

```text
- 轮次：left_hip constant +0.2Nm
- 硬件状态：悬空 / 共地正常 / UART4<->USART2
- 修改参数/代码：无
- 测试动作：20Hz 持续下发 left_hip +0.2Nm
- 观察数据：joint_states.velocity 为正，bridge counters 正常
- 安全状态：RC ch7 急停有效，停止后 timeout 生效
- 结论：left_hip 正方向确认
- 下一步：验证 left_hip -0.2Nm 和 square 模式
```

### 9.9 通过标准

通信完成后的这一步，至少满足以下条件才进入下一阶段：

- 六个关节都完成单关节小力矩方向验证
- `/joint_states` 与实际运动方向一致
- bridge 计数器在测试窗口内稳定
- 急停和命令超时都已验证
- 未出现危险联动、异常冲击或持续通信错误

满足以上条件后，再进入“轻触地验证”和后续低风险实机闭环调参。
