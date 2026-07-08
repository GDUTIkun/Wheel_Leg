# 当前任务进度

## 1. 当前迭代

- 迭代编号：`iter-005`
- 迭代名称：STM32 硬件接入与 200Hz 实机闭环联调
- 当前任务文档：`stm32_hardware_integration.md` / `stm32_ros_comm_task.md` / `control_stm_migration_plan.md` / `hardware_control_200hz.md`
- 100Hz 仿真基线：`100hz_balance_notes.md`

## 2. 状态说明

任务状态使用：

```text
[ ] 未开始
[~] 进行中
[x] 代码已完成
[!] 阻塞或待验证
[?] 待确认
[v] 已通过验证
```

注意：

- `[x] 代码已完成` 不等于 `[v] 已通过验证`。
- 本轮当前是硬件接入准备迭代，允许任务先停留在通信、方向、单位和安全验证阶段，不要求立即完成实机站立。
- 旧迭代拆分文档已清理，已完成能力只作为当前阶段输入，不在本轮重复验收实现细节。

## 3. 迭代目标概览

`iter-005` 关注：

- 固定 100Hz 仿真平衡阶段先冻结，等待硬件接入后再调最终参数。
- 建立 Raspberry Pi 与 STM32 的真实通信闭环。
- 对齐真实 IMU、编码器、电机状态与 ROS2 状态接口。
- 对齐 ROS2 控制命令到 STM32 电机执行命令的方向、单位、限幅和安全状态。
- 将当前硬件控制主线口径切换并固定为 `200Hz`。
- 固定实机初期按悬空、低力矩、轻触地、站立闭环的顺序推进。
- 完成腿长控制阶段，完成 LQR + VMC 悬空输出方向复核，转入轻触地/落地稳定性验证。

## 3.1 当前进度快照（2026-07-07）

当前实机主线状态：

- 通信链路：STM32 状态上报和 ROS bridge 已跑通，硬件主线按 `200Hz` 消费 `/robot_state`。
- 状态装配：`/robot_state` 已包含 IMU、轮端、腿部 `hip_absolute/calf_absolute`、`leg_length`、腿摆杆角 `phi` 和 `phi_rate`。
- 腿长控制：腿长 PID + VMC 阶段已完成，`length_c > length_target` 时 knee 输出较大负力矩。
- LQR + VMC：悬空方向复核已完成，`phi_c < phi_target` 时 hip 输出较大正力矩。
- 当前硬件默认控制：`enable_vmc=true`、`enable_lqr=true`、`enable_leg_length_pid=true`、`enable_wheel_output=true`、`enable_hip_output=true`、`enable_knee_output=true`。
- LQR 调参状态：本轮已收口；`phi` / `phi_rate` 髋部增益按 `0.6` 缩放，`phi_rate_damping_kd=-0.2` 仅作用于髋部虚拟力矩，轮端力矩不叠加该项。
- 当前仍关闭的辅助环：`enable_heading_control=false`、`enable_anti_split=false`、`enable_roll_compensation=false`。
- 安全口径：`hw.launch.py` 默认 `command_enable=false`；ROS bridge 本地关节限位保护阈值默认为 `0.0`，即关闭本地限位保护。真正上电输出前仍必须显式确认 `command_enable`。
- 当前新增问题：`left_hip` 在实机输出打开后存在单侧抖动；右腿可正常控，左腿在实际 `phi` 接近当前 `phi_target` 时更容易触发高频抖动，改变 `phi_target` 后抖动区间会跟随目标移动。
- 当前下一步：在冻结本轮 LQR 参数的前提下进入轻触地/落地稳定性验证；若左腿目标附近仍抖动，优先看左腿 `phi_rate` / 关节速度估计质量、执行死区/回差、静摩擦释放、线束干涉和输出限幅/斜率限制。

本轮已定位并修复的问题：

- LQR 状态顺序没有错，保持 `[phi, phi_rate, distance, velocity, pitch, pitch_rate]`。
- MATLAB 中 `theta` 是腿摆杆角，对应 C++ 的 `leg.phi`；MATLAB 中 `phi/varphi` 是机体倾角，对应 C++ 的 `body.pitch`。
- hip 力矩过小的原因是 LQR 两路输出语义接错：腿摆杆恢复量在 LQR 第一行，原先被接到 wheel，导致 VMC 收到的 hip 虚拟转矩只有 `0.03 ~ 0.05 Nm`。
- 当前修正为 `wheel_torque = lqr_output[1]`，`hip_virtual_torque_for_vmc = -lqr_output[0]`。
- LQR 髋部输出行的 `phi` 和 `phi_rate` 增益已乘 `0.6`，用于降低目标附近腿角闭环激进程度；轮端输出行不做该缩放。
- `phi_rate_damping_kd` 已加入运行时参数，硬件默认 `-0.2`，只追加到髋部虚拟力矩，不作用于轮端力矩。
- VMC 已改为按二连杆世界角重算腿长和腿摆杆角，再映射虚拟腿长力/腿摆杆力矩到 hip/knee。
- ROS bridge 下行力矩极性已再次按单关节 probe 复核，当前左腿 `left_hip`、`left_knee` 命令极性均取反后与实物方向一致。

最近一次 `command_enable=false` 实测：

```text
默认 target_phi=97.1 deg，当前 phi_c≈104.48 deg：
  right_hip≈-2.20 Nm, left_hip≈-1.17 Nm
  right_knee≈-5.21 Nm, left_knee≈-4.56 Nm

临时 target_phi=115 deg，当前 phi_c≈104.48 deg，即 phi_c < phi_target：
  right_hip≈+1.89 Nm, left_hip≈+2.85 Nm
  right_knee≈-2.85 Nm, left_knee≈-2.23 Nm
```

2026-07-07 左腿抖动补充结论：

```text
- 现象：
  - right_leg 可正常跟踪，left_hip 在输出打开后更容易抖动。
  - 只手动向 `phi` 增大方向拉左腿时，更容易触发抖动。
  - 修改 `phi_target` 后，抖动不是固定出现在某个绝对角度，而是在实际 `phi` 接近新的 `phi_target` 附近出现。
- 已排除：
  - left_hip / left_knee 下行力矩极性已通过单关节 probe 修正。
  - 左腿 `phi`、`phi_rate` 主方向与关节速度主方向大体一致，不是整条状态链完全反号。
- 当前观测：
  - 记录脚本：tools/left_leg_oscillation_capture.py
  - `phi_increasing` 样本中：
    left_phi_rate max ≈ 6.00 rad/s
    left_phi_rate_raw max ≈ 10.13 rad/s
    raw_left_hip_vel max ≈ 7.32 rad/s
    raw_left_knee_vel max ≈ 8.19 rad/s
  - 同时右腿基本接近静止：
    right_phi_rate mean ≈ -0.013 rad/s
    raw_right_hip_vel mean ≈ -0.018 rad/s
- 当前判断：
  - 控制器在 `phi` 增大时对 left_hip 的恢复力矩方向是对的，不像单纯极性错误。
  - 因抖动区间会随 `phi_target` 移动，当前不再优先认为是某个固定机械角度点的问题。
  - 更像目标附近的局部闭环振荡：左腿速度状态过大/过吵、执行死区/回差或小范围静摩擦释放均需排查。
  - LQR 本轮已按保守口径收口，下一步优先拆解 left_phi_rate 的来源，并在接近 `phi_target` 的小扰动窗口内对比 left/right 的 `phi_rate`、`cmd_hip` 与实际关节速度。
```

`iter-005` 不关注：

- 继续只在 MuJoCo 中追求最终参数。
- 在硬件状态未校验前直接完整站立。
- 在通信和安全链路未闭环前扩大速度、转向或高度范围。
- 遥控器输入接口重设计。
- 地面站 UI。
- 完整 `ros2_control` 落地。

## 4. 模块任务总览

| 模块任务 | 文件 | 当前状态 | 说明 |
| --- | --- | --- | --- |
| 遥控输入与命令映射 | `wheel_leg_rc` / `wheel_leg_control` | `[v] 已通过验证` | 已完成真实遥控器通道确认，`/cmd_vel`、`/control_mode`、`/body_cmd` 与 failsafe 映射已实机验证 |
| 仿真控制参数基线 | `100hz_balance_notes.md` | `[v] 阶段冻结` | 100Hz 仿真已可稳定站立，参数等待硬件接入后重新小步调整 |
| 腿部角度映射 | `leg_angle_mapping.md` | `[v] 速度已验证` | 已按现场观测修正左右髋和左右膝世界角；四个关节角速度极性已实机确认，knee 角速度已正确叠加 hip 角速度 |
| STM32 硬件接入任务 | `stm32_hardware_integration.md` | `[~] 进行中` | 当前已完成通信、状态装配、腿长控制和 LQR+VMC 悬空方向复核，主线转入轻触地/落地稳定性验证 |
| STM32 与 ROS 通信内容 | `stm32_ros_comm_task.md` | `[~] 首轮联调已完成，V1 代码已落地` | 第一版固定为串口通信；`2026-06-26` 已确认 STM 状态上报稳定 `200Hz`，`2026-06-27` 起 ROS 硬件控制口径同步切到 `200Hz` |
| 实机 200Hz 控制口径 | `hardware_control_200hz.md` | `[x] 已记录` | 已明确当前硬件主线为 `200Hz`，并记录控制器 `dt` 参数、topic 频率和仿真/实机口径分离原则 |
| 控制层接入 STM 分阶段迁移 | `control_stm_migration_plan.md` | `[~] 落地稳定性阶段进行中` | 已完成阶段开关、硬件状态装配、传感器基础联调、腿长控制、LQR+VMC 悬空方向复核 |
| 工程结构与接口约束 | `architecture.md` / `protocol.md` | `[~] 持续维护` | 保留为当前文档入口，不再维护旧 `docs/doc/` 规划体系 |

## 5. 推荐执行顺序

1. `[x]` 完成真实遥控器到仿真控制链的最小联调闭环。
2. `[x]` 固定仿真控制参数基线。
3. `[x]` 将 500Hz 控制频率下调到 100Hz，并在仿真中恢复站立稳定。
4. `[x]` 冻结 100Hz 仿真阶段，等待硬件接入后再调最终参数。
5. `[~]` 进入 STM32 硬件接入任务，先按 `stm32_ros_comm_task.md` 固定通信内容，再验证通信、状态、命令和安全链路。
6. `[x]` 为控制层接入 STM 建立阶段开关、仿真/硬件参数入口和硬件状态装配模块。
7. `[x]` 按 `control_stm_migration_plan.md` 完成进入控制前所需的传感器基础联调。
8. `[x]` 完成腿长控制阶段，已验证可进入下一阶段联调。
9. `[x]` 按 `control_stm_migration_plan.md` 完成 LQR + VMC 悬空输出方向复核。
10. `[~]` 按 `control_stm_migration_plan.md` 进入轻触地与落地稳定性验证。
11. `[ ]` 按 `control_stm_migration_plan.md` 完成航向+抗劈叉与 roll 补偿验证。

## 6. 当前输入与已完成基础

作为本轮输入，以下内容已存在：

- 当前架构已建立 `/joint_states`、`/imu`、`/joint_command` 和 `/robot_state` 边界。
- 当前控制链已完成控制编排外移、`sim adapter` 收口和 ROS2 接口接入。
- 当前第二阶段硬件事实已经明确：`FlySky FS-iA6B`、`iBUS`、`/dev/ttyAMA3`、`GPIO8/9`。
- 当前项目没有地面站，需要在 ROS2 环境中直接完成调参与观察。
- 当前已完成真实遥控器到仿真控制链联调，`stand`、`velocity`、failsafe 与恢复复控能力已形成冻结仿真控制基线。
- 当前 100Hz 仿真平衡已经可以稳住，先作为阶段性基线冻结。
- 当前参数不视为实机最终参数，后续以 STM32 接入后的真实 IMU、编码器、电机响应和通信延迟重新调参。
- 当前 STM32 与 ROS 第一版通信边界已记录：使用串口通信，两侧手动解包；当前硬件主线按 `200Hz` 上报状态并按 `200Hz` 口径消费 `/robot_state`，STM 执行六电机力矩指令。
- 当前正式通信节点 V1 代码已接入仓库：ROS 侧 bridge 不再是占位节点，`RcStatus` 已新增 `estop_active`，默认 `RC channel 7` 低值端为急停，并由 bridge 直接门控下行 `enable/estop` 命令帧。
- 当前控制层接入 STM 的第一版代码已接入仓库：`wheel_leg_controller` 支持阶段开关，`hw.launch.py` 默认加载硬件保守参数，`sim.launch.py` 默认加载仿真全功能参数，硬件状态装配已从 bridge 节点拆出并加单元测试。
- `2026-06-27` 已将 `control_hw.yaml` 切换到 `expected_dt_sec = 0.005`、`accepted_dt_tolerance_sec = 0.002`，用于接受 `200Hz` 硬件状态样本。
- `2026-07-04` 硬件控制门控曾收口为“腿长 PID + VMC + 髋/膝输出”的单独验证模式，用于完成腿长控制阶段。
- `2026-07-07` 已完成 LQR + VMC 悬空方向复核：状态顺序保持 `[phi, phi_rate, distance, velocity, pitch, pitch_rate]`；修正 LQR 两路输出语义，第一路取反作为 VMC 腿摆杆虚拟力矩，第二路作为轮端输出。`command_enable=false` 下验证 `phi_c < phi_target` 时 hip 为较大正力矩，`length_c > length_target` 时 knee 为较大负力矩。
- `2026-07-07` LQR 调参收口：髋部输出行 `phi` / `phi_rate` 增益乘 `0.6`，新增 `phi_rate_damping_kd=-0.2` 只作用髋部虚拟力矩，不进入轮端力矩。
- `2026-07-07` 当前硬件默认配置已进入 LQR 阶段：VMC、LQR、腿长 PID、轮端输出、髋/膝输出均默认开启；heading、anti-split、roll compensation 仍默认关闭。

## 7. 当前阻塞或待确认问题

### 7.1 100Hz 仿真阶段

- 100Hz 控制频率下的 MuJoCo 仿真已经可以稳住。
- 该阶段先冻结为仿真可运行基线，不继续只在仿真中深调参数。
- 当前参数不视为实机最终参数；硬件接入后需要根据真实日志重新调整。

### 7.2 STM32 通信与状态上报

- STM32 状态上报 `200Hz` 已验证通过；ROS 硬件控制主线也已切到 `200Hz` 口径。
- `2026-06-25` 已完成 Raspberry Pi UART4 与 STM32 USART2 的首轮 5 秒双向联调：ROS 侧可稳定解出 `type=0x81` 状态帧，ROS 下行发送无 `write_errors`/`partial_writes`，复测窗口内 STM32 新增 `crc_errors`、`length_errors`、`sync_losses`、`uart_errors` 均为 `0`。
- 首次失败原因已确认是未共地；补上 `Raspberry Pi GND -- STM32 GND` 后通信恢复正常。
- `2026-06-25` 随后又定位到正式状态帧发送异常的根因收敛为：`USART2` 上持续 `HAL_UART_Receive_IT(..., 1)` 与阻塞式 `HAL_UART_Transmit()` 组合会导致发送侧稳定 `HAL_TIMEOUT`；加入“发送前 `HAL_UART_AbortReceive_IT()`、发送后重启接收”的联调绕过后，STM32 上行 `tx_err` 清零且状态帧恢复稳定发送。
- `2026-06-26` 继续实测后确认，当前把状态发送挂入 `Data_Task` 后单轮循环耗时约 `5.58ms`，已经超过 `5ms` 周期预算；当前主优化方向固定为两项：`USART2` 状态帧改为 DMA/中断非阻塞发送，以及 `JY901S` 从 `0x34~0x3F` 连续寄存器块读替代 9 次分散软件 I2C 读取。
- 当前剩余卡点已转移到 ROS/树莓派到 STM32 的下行命令接收链路；此时 `tx` 可持续增长，但 `rx_ok`、`last_rx` 仍可能保持为 `0`，需继续核对串口设备名、接线、发送时机与接收统计。
- 后续仍建议补一轮更长时间窗口测试，继续观察 `rx_seq_gaps` 是否偶发增长。
- 待确认 STM32 上传的编码器、IMU、电机反馈可以稳定映射到 ROS2 状态接口。
- 待确认时间戳和 `dt` 口径在实机整链下稳定围绕 `0.005s`。
- 待完成正式状态帧新 payload 与 ROS bridge 的上板联调，确认字段顺序、单位和字节序与当前代码一致。

### 7.3 命令下发与安全保护

- 待确认 `/joint_command` 下发到 STM32 后，各电机动作方向、单位、限幅和斜率限制正确。
- 待确认通信异常、遥控 failsafe 或控制异常时可以进入安全状态。
- 待确认初次调试时可以使用悬空、低力矩、单关节方式逐项验证。
- 待验证 `RC ch7 -> /rc/status.estop_active -> bridge estop=1 -> STM32 整车失能` 整链行为。
- `2026-06-26` 已在 `wheel_leg_stm32_bridge` 增加 ROS 侧本地限位保护：当 hip 世界角或 knee 相对角到达文档记录的机械限位，且该关节实际回传力矩绝对值大于阈值时，bridge 锁存 `local_estop` 并持续向 STM32 下发 `estop`。
- `2026-07-07` 为便于悬空观察内部力矩，`hw.launch.py` 默认将 `joint_limit_protection.effort_threshold_nm` 设为 `0.0`，即关闭 bridge 本地限位保护。

### 7.4 传感器方向与单位

- 待确认 pitch / pitch_rate 正方向与仿真一致。
- 当前角度解算使用角速度积分；实机观测为向左转动时角度为正向增加，向右转动时角度为反向减小，后续需要与 ROS2 / LQR 使用的姿态方向统一。
- 待确认左右腿 `phi` / `phi_rate` 正方向与 LQR 状态定义一致。
- 当前腿部世界角约定为右侧水平轴 `0 deg`、顺时针为正、竖直向下 `90 deg`、竖直向上 `270 deg`；STM32 上报到 ROS bridge 的髋关节和小腿角已经是世界坐标系 absolute 角，bridge 不再叠加仿真 offset。
- `2026-06-27` 已修正仿真侧和 STM32 bridge 的 `phi/leg_length` 解算：使用 `delta = hip_absolute - calf_absolute`、`knee_angle = pi - delta`、`lower_link_absolute = hip_absolute - pi + knee_angle`，再由大小腿三角形求腿对角边和世界系 `phi`。
- `2026-06-27` 已记录并接入实物侧目标限幅：`target_leg_length_min/max = 0.15/0.32 m`，`target_phi_min/max = 30/150 deg`。
- VMC 髋/膝力矩映射已复核：按二连杆世界角重算腿长与腿角，再映射虚拟腿长力/腿摆杆力矩到 hip/knee。
- 当前机械范围记录：hip 世界角活动范围为 `75 ~ 200 deg`；knee 自身相对角范围不包含 hip 转动，当前限位测试范围为 `-140 ~ -70 deg`；固件已加 `knee_limit_flag` 用于到达限位监测。
- 待确认轮子前进方向与 `base_link.velocity` 正方向一致。
- 待确认 `/robot_state` 中 `hip_absolute`、`calf_absolute` 与实物世界角一致。

### 7.5 实机调参入口

- 实机闭环初期优先观察 `pitch`、`pitch_rate`、`phi`、`phi_rate`、`base_velocity`、轮端力矩和髋关节力矩。
- 当前进度口径：腿长控制阶段已完成，LQR + VMC 悬空输出方向已复核，本轮 LQR 调参已收口；下一步主看 `pitch/pitch_rate/base_velocity`、轮端方向和轻触地稳定性。
- 轻触地前必须确认当前 launch 仍为 `command_enable=false`，先观察 `/joint_command`；准备实际输出时再显式改为 `command_enable=true` 并使用低限幅。
- 若出现 pitch 高频震荡，优先检查 IMU 方向、gyro 滤波、`pitch_rate` 增益和执行器斜率限制。
- 若出现缓慢前后跑偏，优先检查 `target_phi`、`target_pitch`、速度方向和距离积分。
- 若出现单侧压低，优先检查左右腿角度、roll、VMC 映射和左右电机方向。
