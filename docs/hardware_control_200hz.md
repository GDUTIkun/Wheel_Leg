# `200Hz` 实机控制口径记录

本文档记录自 `2026-06-27` 起，仓库主线硬件闭环采用的 `200Hz` 控制口径，避免继续把“`100Hz` 仿真基线”和“`200Hz` 实机联调口径”混在一起。

## 1. 当前结论

- STM32 `Data_Task` 以 `5ms` 周期运行，对应 `200Hz`。
- STM32 正式状态帧 `type=0x81` 已实测稳定跑到 `200Hz`。
- ROS 侧 `wheel_leg_stm32_bridge` 收到一帧状态就立即发布一次：
  - `/robot_state`
  - `/imu`
  - `/joint_states`
- ROS 侧 `wheel_leg_controller` 由 `/robot_state` 驱动，每收到一个满足 `dt` 条件的新样本就执行一步控制，并发布一次 `/joint_command`。
- 当前硬件参数已切到 `expected_dt_sec = 0.005`，即按 `200Hz` 接受控制样本。

## 2. 本次落地修改

本次仅切换**硬件主线**到 `200Hz`：

- 修改文件：`ros2_ws/src/wheel_leg_bringup/config/control_hw.yaml`
- 修改内容：
  - `debug_plot_publish_hz: 200.0`
  - `expected_dt_sec: 0.005`
  - `accepted_dt_tolerance_sec: 0.002`

当前 `accepted_dt_tolerance_sec = 0.002` 的接受窗口为：

- 最小 `dt = 0.003s`
- 最大 `dt = 0.007s`

该窗口用于覆盖串口、ROS 调度和时间戳抖动，但仍明确拒绝明显偏离 `200Hz` 主线的样本。

## 3. 当前频率口径

### 3.1 STM32 侧

- IMU 快照读取：`200Hz`
- 正式状态帧上报：`200Hz`
- 控制命令执行刷新：跟随下行 `/joint_command` 更新

### 3.2 ROS 硬件链

- `/robot_state`：约 `200Hz`
- `/imu`：约 `200Hz`
- `/joint_states`：约 `200Hz`
- `wheel_leg_controller` 有效控制步频：目标 `200Hz`
- `/joint_command`：目标 `200Hz`

### 3.3 RC 链

- `rc_ibus_node` 串口轮询周期：`5ms`，约 `200Hz`
- `/rc/channels_raw`：随有效遥控帧发布，通常接近 `200Hz`
- `/cmd_vel`、`/control_mode`、`/body_cmd`：随有效遥控帧更新
- `/rc/status`：`100ms`，即 `10Hz`

## 4. 与 `100Hz` 仿真基线的关系

以下内容**保持不变**：

- `100hz_balance_notes.md` 仍然是当前仿真平衡基线
- `control_sim.yaml` 仍按 `expected_dt_sec = 0.01` 运行
- MuJoCo 仿真主线仍是 `100Hz`

也就是说，当前仓库口径变为：

- 仿真主线：`100Hz`
- 实机硬件闭环主线：`200Hz`

后续如果需要把仿真也切到 `200Hz`，应单独立项，不要直接复用这次硬件参数切换结论。
