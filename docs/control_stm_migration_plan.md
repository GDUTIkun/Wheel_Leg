# 控制层接入 STM 迁移计划

## 1. 目标与边界

第一阶段采用 `ROS2 控制 + STM32 硬件后端`：

- `wheel_leg_control` 继续运行 VMC、LQR、航向、抗劈叉和 roll 补偿。
- `wheel_leg_stm32_bridge` 负责 STM32 协议编解码、状态映射、命令下发和 ROS 侧安全保护。
- `firmware/stm32` 负责真实传感器采样、电机执行、力矩限幅、斜率限制、命令超时和急停。
- 本阶段不把 VMC/LQR 下沉到 STM32，也不迁入完整 `ros2_control`。

公共主链保持不变：

```text
backend -> /robot_state -> wheel_leg_controller -> /joint_command -> backend
```

仿真和硬件只替换 backend：

- 仿真：`wheel_leg_sim` 发布 `/robot_state` 并消费 `/joint_command`
- 硬件：`wheel_leg_stm32_bridge` 发布 `/robot_state` 并消费 `/joint_command`

## 2. 分层与代码对应

| 层级 | 职责 | 主要代码 | 当前状态 |
| --- | --- | --- | --- |
| 控制阶段门控 | 按传感器、VMC、LQR、航向、抗劈叉、roll 逐段打开控制输出 | `wheel_leg_control` | `[v] LQR+VMC 悬空方向已验证，[~] 落地稳定性待验证` |
| 仿真参数入口 | 保持仿真全功能默认配置 | `wheel_leg_bringup/config/control_sim.yaml` | `[x] 代码已完成` |
| 硬件参数入口 | 当前硬件默认进入 LQR 阶段，VMC/LQR/腿长 PID/轮端/髋膝输出均开启，辅助环仍关闭 | `wheel_leg_bringup/config/control_hw.yaml` | `[x] 代码已完成，[~] 落地待验证` |
| 硬件状态装配 | 将 STM32 absolute 世界角/IMU 数据映射到控制器语义 | `wheel_leg_stm32_bridge/hardware_state_assembler` | `[x] 代码已完成，[!] 待实机验证` |
| STM32 协议与执行 | 状态帧、命令帧、限幅、斜率、超时、急停 | `wheel_leg_stm32_bridge` / `firmware/stm32` | `[~] 联调中` |

`wheel_leg_control` 不解析 STM32 二进制协议；STM32、串口、CAN 细节不得泄漏到控制层。

硬件腿部角度口径：

- 仿真侧仍由原始关节角叠加 offset 得到 `hip_absolute`、`calf_absolute`。
- STM32 上报到 bridge 的髋和小腿角已经是世界坐标系 absolute 角，bridge 不再叠加仿真 offset。
- 状态帧中的 `left_knee` / `right_knee` 位置槽位在控制语义里作为 `left_calf_absolute` / `right_calf_absolute` 使用。
- 腿部机构不是串联二连杆，不能使用 `hip_absolute + calf_absolute - pi` 推下连杆角。
- 腿部几何按实物平行四边形语义计算：`delta = hip_absolute - calf_absolute`，`knee_angle = pi - delta`，`lower_link_absolute = hip_absolute - pi + knee_angle`，再用大小腿三角形求腿对角边 `leg_length` 与世界系腿角 `phi`。
- `phi_rate` 由修正后的 `phi` 做角度差分和低通滤波。
- VMC 的髋/膝力矩映射已按同一套几何复核，避免 `phi/leg_length` 已修正但雅可比仍沿用旧串联模型。

## 3. 控制阶段参数

控制器新增以下参数，仿真默认全开；当前硬件默认配置已进入 LQR 阶段：

- 已开启：`VMC`、`LQR`、腿长 PID、轮端输出、髋/膝输出。
- 仍关闭：航向控制、抗劈叉、roll 补偿。
- `hw.launch.py` 默认 `command_enable=false`，即 ROS 内部会计算 `/joint_command`，但 bridge 不会向 STM32 下发实际非零力矩，除非显式改为 `command_enable:=true`。

| 参数 | 作用 | 硬件默认 |
| --- | --- | --- |
| `enable_vmc` | 启用腿端力/髋力矩到髋膝电机力矩的 VMC 映射 | `true` |
| `enable_lqr` | 启用 LQR 计算轮端力矩和髋关节虚拟力矩 | `true` |
| `enable_leg_length_pid` | 启用腿长 PID 和重力补偿 | `true` |
| `enable_heading_control` | 启用航向/yaw rate 控制与转向髋前馈 | `false` |
| `enable_anti_split` | 启用左右腿 `phi` 差的抗劈叉控制 | `false` |
| `enable_roll_compensation` | 启用 roll 平衡补偿 | `false` |
| `enable_wheel_output` | 允许发布左右轮力矩 | `true` |
| `enable_hip_output` | 允许发布左右髋力矩 | `true` |
| `enable_knee_output` | 允许发布左右膝力矩 | `true` |
| `phi_rate_damping_kd` | 在 LQR 髋部虚拟力矩处追加 `phi_rate * kd`，不作用于轮端力矩 | `-0.2` |
| `target_leg_length_min` | 实物侧腿长目标下限，单位 m | `0.15` |
| `target_leg_length_max` | 实物侧腿长目标上限，单位 m | `0.32` |
| `target_phi_min_deg` | 实物侧腿角目标下限，单位 deg | `30.0` |
| `target_phi_max_deg` | 实物侧腿角目标上限，单位 deg | `150.0` |

运行时可用 `ros2 param set /wheel_leg_controller <参数名> true/false` 逐段切换。

## 4. 推荐实机测试顺序

### 4.1 传感器阶段

目标：只验证 STM32 上行状态，不闭环控制。

启动：

```bash
ros2 launch wheel_leg_bringup hw.launch.py use_controller:=false command_enable:=false
```

观察：

- `/joint_states`
- `/imu`
- `/robot_state`
- `/stm32_bridge/status_text`
- `/stm32_bridge/counters`

确认项：

- `pitch`、`pitch_rate`、`roll`、`yaw_rate` 方向和单位正确。
- STM32 上传的 `hip` / `calf` 世界角无需二次 offset，`/robot_state` 中 `*_hip_absolute` 和 `*_calf_absolute` 应与实物世界角一致。
- `phi`、`phi_rate` 左右一致，且与 LQR 状态定义一致。
- `body_velocity` 与轮子前进方向一致。
- `/robot_state` 时间戳稳定，控制周期可接近 `0.01s`。

### 4.2 VMC 阶段（已完成）

目标：验证腿长力到髋/膝力矩映射，并完成进入 LQR 前的腿长控制闭环。

该阶段的历史验证模式：

- 裸启动 `wheel_leg_controller` 时，默认就是“腿长 PID + VMC”。
- `hw.launch.py` 对应的 `control_hw.yaml` 也已改成同样配置。
- 当时仅保留腿长 PID、VMC 和髋/膝电机输出，用于单独确认腿长闭环与 VMC 映射。
- 2026-07-07 当前硬件默认已进入 LQR + VMC 联合阶段：`enable_vmc=true`、`enable_lqr=true`、`enable_wheel_output=true`、`enable_hip_output=true`、`enable_knee_output=true`。

建议参数：

```bash
ros2 param set /wheel_leg_controller enable_hip_output true
ros2 param set /wheel_leg_controller enable_knee_output true
```

该阶段仅作为回溯记录，不作为当前硬件默认配置；当前默认见第 3 节。

确认项：

- 悬空低力矩下髋/膝力矩方向正确。
- 支撑力趋势正确。
- 急停、超时、限幅和斜率限制工作正常。

调参顺序：

1. 先固定 `target_leg_length`，不要一边改目标一边调 PID。
2. 先调 `leg_length_pid.kp`，让腿能“跟上”目标，但不出现连续上下弹跳。
3. 再调 `leg_length_pid.kd`，专门压制落地后或抬放腿时的振荡。
4. 最后再少量加入 `leg_length_pid.ki`，只用于消除静差，不用于“顶起”机器人。

观察重点：

- `leg_length` 能否稳定收敛到 `target_leg_length`
- `/debug/control/leg_length_output` 是否平滑，是否频繁顶到限幅
- 髋膝输出是否同向配合支撑，而不是一侧明显反着顶
- 落地后若出现高频抖动，优先减 `kp` 或加一点 `kd`
- 落地后若慢慢塌腿，再少量加 `ki`

实机建议：

- 第一轮把 `hip_effort_limit`、`knee_effort_limit` 保持在较小值，先确认方向对。
- 每次只改一个参数，单次改幅建议不超过 `10%` 到 `20%`。
- 若输出已频繁打满，先降 `kp`，不要继续加 `ki`。

当前进度补充：

- `2026-07-04` 已按该模式完成腿长控制阶段推进。
- 文档后续不再把 VMC 阶段视为“待开始”，当前主线已转入 LQR。

### 4.3 LQR 阶段

目标：在已完成腿长控制基础上，启用 VMC + LQR，验证平衡主环输出方向并逐步进入轻触地/短时落地联调。

建议参数：

```bash
ros2 param set /wheel_leg_controller enable_lqr true
ros2 param set /wheel_leg_controller enable_wheel_output true
```

保持：

- `enable_heading_control=false`
- `enable_anti_split=false`
- `enable_roll_compensation=false`

确认项：

- 先悬空观察轮端和髋关节输出符号。
- 再轻触地观察支撑趋势。
- 最后短时间落地，避免直接长时间闭环。

当前阶段关注：

- 优先确认 `pitch`、`pitch_rate`、`phi_rate`、`base_velocity` 和轮端输出符号一致。
- 若 LQR 打开后出现明显前后冲、轮子快速打满或髋部反向顶杆，先回头检查状态方向和滤波，再继续加大接地时间。

2026-07-07 实机复核结论：

- 本轮复核时必须保持 `command_enable:=false`，只观察 ROS 内部 `/joint_command` 与 debug topic，不让 STM32 执行力矩。
- MATLAB LQR 状态定义与 C++ 控制状态定义一致，状态顺序保持：

```text
[phi, phi_rate, distance, velocity, pitch, pitch_rate]
```

- 这里 `phi` 是腿摆杆角，对应 MATLAB 模型中的 `theta`；`pitch` 是机体倾角，对应 MATLAB 模型中的 `phi/varphi`。不要因为 MATLAB 符号名把两者换位。
- 本次 hip 力矩过小的真正原因不是状态顺序错，而是 LQR 两路输出语义接错：腿摆杆角恢复量主要在 LQR 第一行输出中，原 C++ 把它接到了 `wheel_torque`，导致送入 VMC 的 `hip_torque` 只有约 `0.03 ~ 0.05 Nm`。
- 修正后的映射为：LQR 第一行取反后作为 VMC 腿摆杆虚拟力矩，第二行作为 wheel 输出。
- VMC 映射按二连杆世界角重算腿长与腿角，再从虚拟腿长力/腿摆杆力矩映射到 hip/knee，避免上游 `leg_length/phi` 与关节角口径不一致时缩小 hip 输出。
- `hw.launch.py` 默认将 `joint_limit_protection.effort_threshold_nm` 设为 `0.0`，关闭 ROS bridge 的本地关节限位保护，避免只观察内部力矩时触发 local estop。

实测验证，`command_enable=false`、限位保护阈值 `0.0`：

```text
默认 target_phi=97.1 deg，当前 phi_c≈104.48 deg：
  right_hip≈-2.20 Nm, left_hip≈-1.17 Nm
  right_knee≈-5.21 Nm, left_knee≈-4.56 Nm

临时 target_phi=115 deg，当前 phi_c≈104.48 deg，即 phi_c < phi_target：
  right_hip≈+1.89 Nm, left_hip≈+2.85 Nm
  right_knee≈-2.85 Nm, left_knee≈-2.23 Nm
```

判据通过：`length_c > length_target` 时 knee 输出较大负力矩；`phi_c < phi_target` 时 hip 输出较大正力矩。

2026-07-07 LQR 调参收口：

- LQR 状态顺序和输出语义冻结：状态仍为 `[phi, phi_rate, distance, velocity, pitch, pitch_rate]`，第一行取反后作为 VMC 腿摆杆虚拟力矩，第二行作为轮端输出。
- 为降低目标附近腿角闭环激进程度，LQR 第一行中的 `phi` 与 `phi_rate` 两项增益统一乘以 `0.6`；轮端输出行的 `phi` / `phi_rate` 增益不乘该系数。
- 额外 `phi_rate_damping_kd` 只加在 LQR 髋部虚拟力矩处，当前硬件默认 `-0.2`。该项不进入 `wheel_torque`，避免把腿角速度阻尼混到轮端平衡力矩。
- 本轮不再继续扩大 LQR 增益调参范围。后续轻触地/落地测试若仍出现局部振荡，优先从速度估计质量、执行死区/回差、静摩擦释放、线束干涉和限幅/斜率限制排查；只有在这些现象被复核后再重新打开 LQR 增益调整。

### 4.4 落地稳定性阶段

目标：在低限幅下验证基础站立稳定性。

使用 `control_hw.yaml` 中的保守力矩限制作为起点：

- `hip_effort_limit: 3.0`
- `knee_effort_limit: 3.0`
- `wheel_effort_limit: 1.0`

观察：

- `/debug/control/balance`
- `/debug/control/wheel_effort`
- `/debug/control/leg_length_output`
- `pitch`、`pitch_rate`
- `phi`、`phi_rate`
- `body_velocity`

若出现 pitch 高频震荡，优先检查 IMU 方向、gyro 滤波、`phi_rate`、`base_velocity` 和执行器斜率限制。

2026-07-07 左腿单侧抖动实测补充：

- 当前右腿可正常控，左腿在 `phi` 增大方向更容易触发高频抖动。
- 调整 `phi_target` 后，抖动区间会跟随目标移动，不是固定出现在某个绝对机械角度。
- 单关节 probe 已确认左腿 `left_hip` / `left_knee` 下行力矩极性修正后与实物动作方向一致，因此当前问题不优先归因于命令极性。
- 采集脚本：`tools/left_leg_oscillation_capture.py`
- 当前实测中，`phi_increasing` 阶段左腿速度状态显著大于右腿：
  - `left_phi_rate max ≈ 6.00 rad/s`
  - `left_phi_rate_raw max ≈ 10.13 rad/s`
  - `raw_left_hip_vel max ≈ 7.32 rad/s`
  - `raw_left_knee_vel max ≈ 8.19 rad/s`
  - 同时 `right_phi_rate mean ≈ -0.013 rad/s`
- 当前优先级最高的排查方向：
  - 接近 `phi_target` 时，左腿 `phi_rate` 是否被髋/膝某一路速度单边放大
  - LQR 速度反馈或 VMC 映射在目标附近是否过激
  - 左腿执行侧是否存在死区、回差、静摩擦释放或线束干涉，导致目标附近小范围来回打

### 4.5 航向与抗劈叉阶段

目标：在基础站立可控后启用方向控制和左右腿约束。

建议参数：

```bash
ros2 param set /wheel_leg_controller enable_heading_control true
ros2 param set /wheel_leg_controller enable_anti_split true
```

确认项：

- 左右轮差速方向正确。
- 左右髋抗劈叉力矩方向正确。
- `left_phi - right_phi` 被拉回，而不是放大。

### 4.6 Roll 补偿阶段

目标：最后启用横滚补偿。

建议参数：

```bash
ros2 param set /wheel_leg_controller enable_roll_compensation true
```

确认项：

- 左右髋 roll 补偿方向正确。
- 不出现单侧持续压低。
- 关闭 roll 补偿后现象可复现退回，便于确认因果。

## 5. 验收与记录

每轮记录建议：

```text
- 轮次：
- 当前阶段：
- 启用参数：
- 硬件状态：
- 测试动作：
- 观察数据：
- 安全状态：
- 结论：
- 下一步：
```

阶段完成标准：

- `[v]` 传感器阶段：`/robot_state` 与真实姿态、关节、轮速方向一致。
- `[v]` VMC 阶段：髋/膝输出方向正确，低力矩下安全保护可验证。
- `[v]` LQR 阶段：轮端与髋关节平衡输出方向正确，可短时间低风险落地。
- `[v]` 落地稳定性阶段：低限幅下可维持基础站立或明确定位剩余方向/滤波/延迟问题。
- `[v]` 航向+抗劈叉阶段：差速与左右腿约束方向正确。
- `[v]` roll 阶段：横滚补偿方向正确，不引入单侧压低。

## 6. 自动验证

当前已加入自动测试：

- `wheel_leg_control/test/test_stand_control_pipeline.cpp`
  - 验证所有阶段开启时完整命令路径存在。
  - 验证所有阶段关闭时对应算法不被调用且输出为零。
  - 验证 wheel/hip/knee 输出组可单独门控。
- `wheel_leg_stm32_bridge/test/test_hardware_state_assembler.cpp`
  - 验证轮速到 `body_velocity/body_distance`。
  - 验证腿几何、`phi` 和 `phi_rate` 滤波。

建议每次修改控制门控或硬件状态装配后执行：

```bash
source ros2_ws/install/setup.bash
colcon --log-base ros2_ws/log build \
  --base-paths ros2_ws/src \
  --build-base ros2_ws/build \
  --install-base ros2_ws/install \
  --packages-select wheel_leg_control wheel_leg_stm32_bridge wheel_leg_bringup

colcon --log-base ros2_ws/log test \
  --base-paths ros2_ws/src \
  --build-base ros2_ws/build \
  --install-base ros2_ws/install \
  --packages-select wheel_leg_control wheel_leg_stm32_bridge
```

## 7. 当前进度

| 项目 | 状态 | 说明 |
| --- | --- | --- |
| 控制阶段参数与运行时切换 | `[x]` | 已接入 `wheel_leg_controller` |
| `RunStandControlStep` 阶段门控 | `[x]` | VMC/LQR/PID/航向/抗劈叉/roll/输出组已可单独关闭 |
| 仿真参数文件 | `[x]` | `control_sim.yaml` 默认全功能 |
| 硬件参数文件 | `[x]` | `control_hw.yaml` 当前默认开启 VMC/LQR/腿长 PID，`hw.launch.py` 默认关闭 bridge 本地限位保护 |
| launch 加载参数 | `[x]` | `sim.launch.py` / `hw.launch.py` 已加载各自参数 |
| 硬件状态 assembler | `[x]` | 已从 bridge 节点拆出并加测试 |
| 构建与单元测试 | `[v]` | `wheel_leg_control`、`wheel_leg_stm32_bridge`、`wheel_leg_bringup` 构建通过，新增测试通过 |
| 传感器阶段实机验证 | `[x]` | 已支撑腿长控制阶段推进，剩余问题转入 LQR 联调中继续复核 |
| VMC 阶段实机验证 | `[x]` | 腿长控制阶段已完成，当前已进入 LQR |
| LQR 阶段实机验证 | `[~]` | 已完成悬空方向复核并收口本轮 LQR 调参；仍需轻触地/落地验证 |
| 落地稳定性验证 | `[ ]` | 待现场测试 |
| 航向+抗劈叉验证 | `[ ]` | 待现场测试 |
| roll 补偿验证 | `[ ]` | 待现场测试 |
