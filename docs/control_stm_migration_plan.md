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
| 控制阶段门控 | 按传感器、VMC、LQR、航向、抗劈叉、roll 逐段打开控制输出 | `wheel_leg_control` | `[x] 代码已完成，[!] 待实机验证` |
| 仿真参数入口 | 保持仿真全功能默认配置 | `wheel_leg_bringup/config/control_sim.yaml` | `[x] 代码已完成` |
| 硬件参数入口 | 硬件默认保守配置，所有高风险环节默认关闭 | `wheel_leg_bringup/config/control_hw.yaml` | `[x] 代码已完成，[!] 待实机验证` |
| 硬件状态装配 | 将 STM32 原始关节/IMU 数据映射到控制器语义 | `wheel_leg_stm32_bridge/hardware_state_assembler` | `[x] 代码已完成，[!] 待实机验证` |
| STM32 协议与执行 | 状态帧、命令帧、限幅、斜率、超时、急停 | `wheel_leg_stm32_bridge` / `firmware/stm32` | `[~] 联调中` |

`wheel_leg_control` 不解析 STM32 二进制协议；STM32、串口、CAN 细节不得泄漏到控制层。

## 3. 控制阶段参数

控制器新增以下参数，仿真默认全开，硬件默认全关：

| 参数 | 作用 | 硬件默认 |
| --- | --- | --- |
| `enable_vmc` | 启用腿端力/髋力矩到髋膝电机力矩的 VMC 映射 | `false` |
| `enable_lqr` | 启用 LQR 计算轮端力矩和髋关节虚拟力矩 | `false` |
| `enable_leg_length_pid` | 启用腿长 PID 和重力补偿 | `false` |
| `enable_heading_control` | 启用航向/yaw rate 控制与转向髋前馈 | `false` |
| `enable_anti_split` | 启用左右腿 `phi` 差的抗劈叉控制 | `false` |
| `enable_roll_compensation` | 启用 roll 平衡补偿 | `false` |
| `enable_wheel_output` | 允许发布左右轮力矩 | `false` |
| `enable_hip_output` | 允许发布左右髋力矩 | `false` |
| `enable_knee_output` | 允许发布左右膝力矩 | `false` |

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
- `phi`、`phi_rate` 左右一致，且与 LQR 状态定义一致。
- `body_velocity` 与轮子前进方向一致。
- `/robot_state` 时间戳稳定，控制周期可接近 `0.01s`。

### 4.2 VMC 阶段

目标：只验证腿长力到髋/膝力矩映射。

建议参数：

```bash
ros2 param set /wheel_leg_controller enable_leg_length_pid true
ros2 param set /wheel_leg_controller enable_vmc true
ros2 param set /wheel_leg_controller enable_hip_output true
ros2 param set /wheel_leg_controller enable_knee_output true
```

保持：

- `enable_lqr=false`
- `enable_wheel_output=false`
- `enable_heading_control=false`
- `enable_anti_split=false`
- `enable_roll_compensation=false`

确认项：

- 悬空低力矩下髋/膝力矩方向正确。
- 支撑力趋势正确。
- 急停、超时、限幅和斜率限制工作正常。

### 4.3 LQR 阶段

目标：启用 VMC + LQR，先验证平衡主环输出方向。

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
| 硬件参数文件 | `[x]` | `control_hw.yaml` 默认保守全关 |
| launch 加载参数 | `[x]` | `sim.launch.py` / `hw.launch.py` 已加载各自参数 |
| 硬件状态 assembler | `[x]` | 已从 bridge 节点拆出并加测试 |
| 构建与单元测试 | `[v]` | `wheel_leg_control`、`wheel_leg_stm32_bridge`、`wheel_leg_bringup` 构建通过，新增测试通过 |
| 传感器阶段实机验证 | `[ ]` | 待现场测试 |
| VMC 阶段实机验证 | `[ ]` | 待现场测试 |
| LQR 阶段实机验证 | `[ ]` | 待现场测试 |
| 落地稳定性验证 | `[ ]` | 待现场测试 |
| 航向+抗劈叉验证 | `[ ]` | 待现场测试 |
| roll 补偿验证 | `[ ]` | 待现场测试 |
