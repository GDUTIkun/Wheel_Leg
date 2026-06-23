# 当前任务进度

## 1. 当前迭代

- 迭代编号：`iter-004`
- 迭代名称：遥控驱动下的控制算法调参与行为收敛
- 迭代文档：`doc/iterations/iter-004.md`
- 详细设计：`doc/detail.md`
- 上一轮验证记录：`doc/validation.md`

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
- 本轮当前是第二阶段文档冻结迭代，允许任务先停留在设计完成状态，不要求同步有代码验证。
- `iter-001` 与 `iter-002` 已完成能力视为当前迭代输入，不在本轮重复验收实现细节。

## 3. 迭代目标概览

`iter-004` 关注：

- 固定“STM32 之前先做一轮控制算法调参与行为收敛”的阶段边界。
- 固定 `velocity` 模式速度持续性问题作为本轮首要调参对象。
- 固定速度目标、转向响应与高度调节范围作为优先调参方向。
- 固定由用户现场主导调参、文档记录每轮现象与结论的流程。
- 固定在不破坏 `stand`、failsafe 与恢复复控能力前提下形成新的仿真控制参数基线。

`iter-004` 不关注：

- STM32 通信实现。
- 实机电机闭环落地。
- 遥控器输入接口重设计。
- 地面站 UI。
- 完整 `ros2_control` 落地。

## 4. 模块任务总览

| 模块任务 | 文件 | 当前状态 | 说明 |
| --- | --- | --- | --- |
| `rc_ibus_node` 设计任务 | `doc/tasks/rc_ibus_node.md` | `[v] 已通过验证` | 已完成 `/dev/ttyAMA3` 实机 `iBUS` 解包、原始通道发布与状态诊断验证 |
| 遥控命令映射任务 | `doc/tasks/rc_command_mapping.md` | `[v] 已通过验证` | 已完成真实遥控器通道确认，`/cmd_vel`、`/control_mode`、`/body_cmd` 与 failsafe 映射已实机验证 |
| 控制算法调参与行为收敛任务 | `doc/tasks/control_tuning.md` | `[v] 已通过验证` | 已冻结仿真遥控控制基线，RC 映射、PID、LQR 与速度/转向/高度参数已归档，可进入 STM32 通信阶段 |
| 树莓派串口联通性测试节点 | `src/wheel_leg_rc/src/rc_serial_probe_node.cpp` | `[x] 代码已完成` | 已新增 ROS2 串口探测节点，先验证 `/dev/ttyAMA3` 是否收到持续字节流，不依赖 `iBUS` 解包 |
| 第二阶段详细设计 | `doc/detail.md` | `[~] 文档已更新` | 已新增 `iter-003` 的 `rc_ibus_node` 详细设计章节 |
| 总体架构更新 | `doc/architecture.md` | `[~] 文档已更新` | 已补充 `rc_ibus_node` 的长期定位和 ROS2 接口边界 |
| 总需求更新 | `doc/proposal.md` | `[~] 文档已更新` | 已将第二阶段协议收口为 `FS-iA6B + iBUS + /dev/ttyAMA3`，并补前置测试要求 |

## 5. 推荐执行顺序

1. `[x]` 完成真实遥控器到仿真控制链的最小联调闭环。
2. `[x]` 固定当前 `velocity`、转向和高度调节表现中的主要问题。
3. `[x]` 先修正文档中的问题定性、默认实现口径与参数入口。
4. `[x]` 先做观测链路收口，优先解决 `rqt_plot` 直接消费高频调试流导致的卡顿问题，不动控制主频。
5. `[x]` 再修 `velocity` 模式的参考生成逻辑，包括模式切换、空杆保护与距离积分链路。
6. `[x]` 优先补充 `phi/pitch/roll` 观测并校准站立平衡点。
7. `[x]` 最后回到速度、转向和高度参数调优，并记录新的基线。

## 6. 当前输入与已完成基础

作为本轮输入，以下内容已存在：

- `iter-001` 已建立 `/joint_states`、`/imu`、`/joint_command` 和 actuator 边界。
- `iter-002` 已完成控制编排外移、`sim adapter` 收口和 `/robot_state` 正式接口建立。
- 当前第二阶段硬件事实已经明确：`FlySky FS-iA6B`、`iBUS`、`/dev/ttyAMA3`、`GPIO8/9`。
- 当前项目没有地面站，需要在 ROS2 环境中直接完成调参与观察。
- 当前已完成真实遥控器到仿真控制链联调，`stand`、`velocity`、failsafe 与恢复复控能力已形成冻结仿真控制基线。

## 7. 当前阻塞或待确认问题

### 7.1 观测问题

- 当前 `rqt_plot` 直接看高频调试流时帧率很低、曲线卡顿，已经影响对控制现象的判断。
- 已新增低频 `/debug/plot/*` 观测话题，现场确认 `rqt_plot` 卡顿问题已解决。
- 该问题按“观测链路过高频”处理，不等于控制主环掉频；控制主频继续保持当前 `500 Hz` 主线。

### 7.2 控制问题

- 当前 `velocity` 模式下，空杆观测到瞬时负速度，随后在未继续给速度指令时出现后跑和发散失控现象。
- 本轮定位到一个明确根因：回中时每帧把 `target_distance` 重锚到当前 `body.distance`，会让目标距离跟随实际位置漂移；若实际车体负向漂移，`ref_primary` 会以相同斜率持续负向变化。
- 已取消中位候选区的 `zero_hold_distance_ref` 锁存口径；当前 `velocity` 模式下 `target_distance` 只由 `filtered_target_velocity * dt` 积分生成。
- 当前现场反馈纯积分 `x_ref` 后速度链路已正常，下一步进入遥控器输入与滤波调整。

### 7.3 输入与滤波问题

- 当前进入遥控器调整阶段，优先观察速度、角速度和高度三组滤波前/滤波后目标对比。
- 当前 velocity 口径先固定为 `velocity_snap_threshold = 0.0`、`velocity_ref_lpf_rc = 0.4`、`velocity_ref_slew_rate = 0.8`。
- 当前遥控连续量输入统一按 `-1.0 ~ 1.0` 范围观察。
- 当前新增 `/debug/plot/ref_filter/velocity`、`/debug/plot/ref_filter/yaw_rate`、`/debug/plot/ref_filter/leg_length`，其中 `ref_primary` 为滤波前目标，`now_primary` 为滤波后目标。
- 当前文档先固定默认实现口径：`*_ref_lpf_rc` 越大，滤波越强；`*_ref_slew_rate` 越小，参考变化越平缓，设为 `0.0` 表示关闭斜率限制并保留低通结果。
- 当前文档先固定 `0.1 m/s` 的速度参考 snap 规则：当 `|filtered_target_velocity - commanded_target_velocity| <= 0.1` 时，速度参考直接贴到目标。
- 当前已将 `yaw_rate_assist_scale` 默认改为 `0.0`，避免 `CH1` 右摇杆左右通过 `/body_cmd.yaw_rate_assist` 叠加到转向。

### 7.4 站立平衡点偏置

- 当前观察到实际位移 `body.distance` 会长期和 `target_distance` 保持一段偏差，初始化落地后也会先向后跑一段。
- 当前优先怀疑 LQR 多状态耦合下的平衡点偏置：若 `target_phi = 90.0 deg` 或 `target_pitch = 0` 与真实稳定姿态不一致，控制器可能用 `distance` 偏差抵消 `phi/pitch` 姿态误差。
- 当前已补充 `/debug/plot/balance` 低频观测：`ref_primary=target_phi_deg`、`now_primary=avg_phi_deg`、`ref_secondary=target_pitch_deg`、`now_secondary=body_pitch_deg`。
- 当前已提供 `wheel_leg_controller_node` 参数 `target_phi_deg` 与 `target_pitch_deg` 作为站立平衡点校准入口。
- 当前现场临时基线已收口到 `target_phi = 97.1 deg`：该值可明显减小“启动先后退一段”和“回中后长期保留位移差”的现象。
- 当前 `97.1 deg` 基线仍存在稳态小幅振荡，表现仍不如 MuJoCo legacy stand control 平滑。
- 当前已确认 MuJoCo 默认保留 legacy stand control；ROS takeover 生效后会绕过 legacy 控制，仅执行 ROS `/joint_command`。
- 当前已确认 `body.distance` 是前向速度积分量，不是世界系绝对 `x`；启动阶段后退会直接形成初始负偏置。
- 在该问题收口前，不继续优先调遥控输入参数，也不优先放大速度目标。

### 7.5 转向侧压与 roll 补偿

- 当前左转右压问题已解决，最终根因收口为：双腿劈叉导致 `left_phi/right_phi` 不同，左右腿长相同的情况下仍会形成 body roll 偏差。
- 该问题不能只按腿长 PID 太软处理；单纯增大腿长 `kp` 不一定能把被压低的一侧腿恢复，因为侧倾载荷来自左右腿姿态差和 hip 差力矩耦合。
- 当前有效修正是加强 `anti_crash_pid` 抑制左右 `phi` 差，并增加 `roll_balance_pid` 对 body roll 做左右反号 hip 补偿。
- 当前腿长重力补偿已改为使用 roll 角，避免用 pitch 角解释横向侧倾载荷。
- 后续若继续优化转向手感，优先监控 `/debug/plot/anti_crash`、`/debug/plot/roll_balance`、`/debug/plot/turn_internal` 和左右腿长/phi 差，再小步回调转向前馈或转向 PID。

### 7.6 稳态振荡

- 当前即使把 `target_phi` 调到 `97.1 deg`，回中静站时仍存在持续小幅振荡。
- 当前判断该问题更像 LQR 站立平衡点附近的欠阻尼/极限环；最新参考逻辑已取消 zero-hold 一次锁存，改为纯 `velocity_ref` 积分。
- 当前 MATLAB LQR 权重为 `Q = diag([500 100 10 20 300 60])`、`R = diag([0.01 0.02])`。
- 当前现场反馈纯积分 `x_ref` 后整体已正常；后续优先处理遥控器映射与输入滤波。

### 7.7 调参协作要求

- 当前调参阶段仍由用户本人主导执行，但后续每轮记录不再只写“调了什么参数”，还应写清“当时问题属于观测、参考逻辑还是纯参数收敛”。
- 文档更新完成后，下一轮应先按文档结论进入代码修复，再继续做速度、转向和高度参数调优。
