# 控制算法调参与行为收敛任务

## 1. 任务目标

在进入 STM32 阶段前，围绕当前仿真遥控联调结果，对控制算法目标映射和关键参数做一轮集中调参，形成一版更可用的控制基线。

## 2. 当前输入

- `/cmd_vel`
- `/control_mode`
- `/body_cmd`
- `/robot_state`
- 当前 `wheel_leg_controller_node` 中与速度、转向、高度相关的参数
- 用户现场对仿真整机行为的观察

## 3. 当前输出

- 一版可接受的仿真控制参数基线
- 每轮调参记录
- 对后续代码修复与再调参顺序的明确结论

## 4. 当前重点问题

- `[x]` 速度参考纯积分链路验证正常
- `[~]` 遥控输入滤波参数观察与调整
- `[ ]` 速度目标幅值调大
- `[ ]` 转向响应降缓
- `[ ]` 高度调节范围扩大

## 5. 重点参数范围

当前优先关注以下参数或参数类别：

- `target_velocity_scale`
- `target_yaw_rate_scale`
- `yaw_rate_assist_scale`
- `body_height_offset_scale`
- `target_leg_length_min`
- `target_leg_length_max`
- `velocity_ref_lpf_rc`
- `velocity_ref_slew_rate`
- `yaw_rate_ref_lpf_rc`
- `yaw_rate_ref_slew_rate`
- `leg_length_ref_lpf_rc`
- `turn_hip_feedforward_scale`
- `velocity_snap_threshold`
- `velocity_zero_hold_threshold`
- `target_phi_deg`
- `target_pitch`
- `anti_crash_pid.*`
- `roll_balance_pid.*`
- 腿长 PID、转向 PID 及相关 legacy 控制参数
- 如有必要，再继续检查速度误差进入控制链的定义与计算位置

当前参数口径先固定为：

- `velocity_ref_lpf_rc` 越大，滤波越强。
- `velocity_ref_slew_rate` 越小，速度参考变化越平缓；设为 `0.0` 表示关闭斜率限制，仅保留低通结果。
- `yaw_rate_ref_lpf_rc` 越大，角速度参考滤波越强。
- `yaw_rate_ref_slew_rate` 越小，角速度参考变化越平缓；设为 `0.0` 表示关闭斜率限制，仅保留低通结果。
- `leg_length_ref_lpf_rc` 越大，高度对应的腿长目标滤波越强。
- `turn_hip_feedforward_scale` 越大，转向时叠加到左右髋差力矩上的前馈越强；设为 `0.0` 表示临时关闭这条前馈链，仅保留 steer PID、anti-crash 与 LQR 本体。
- `velocity_snap_threshold` 默认按 `0.1 m/s` 理解，判断口径固定为 `|filtered_target_velocity - commanded_target_velocity|`。
- `velocity_zero_hold_threshold` 为历史零速保持参数；当前纯积分口径下不再用于 `target_distance` 生成。
- `target_phi_deg` 后续应作为腿部姿态目标校准入口，默认仍以当前 `90.0 deg` 为基线。
- `target_pitch` 后续仅在确认 `body.pitch` 稳定偏离 `0` 后再引入或调整。
- `anti_crash_pid.*` 是限制左右 `phi` 劈叉的主入口；当双腿腿长相同但 `phi` 不同时，底盘仍可能产生 roll 偏差。
- `roll_balance_pid.*` 是当前新增的 roll 补偿环，输出叠加到左右 LQR hip 输出且左右反号，用于消除等腿长下由双腿劈叉引出的侧倾偏差。

当前现场临时基线先固定为：

- `wheel_leg_controller_node` 默认 `target_phi_deg = 97.1`
- MuJoCo 侧 stand 默认 `target_phi = 97.1 deg`
- 当前该基线的主要用途是先消除“仿真启动后先后退一段”和“回中后长期保留一段位移差”的明显现象

当前暂不调整以下遥控输入映射参数：

- `linear_x.center`
- `linear_x.deadzone`
- `linear_x.scale`
- `linear_x.reverse`

当前遥控调整阶段已固定：

- `velocity_snap_threshold = 0.0`
- `velocity_ref_lpf_rc = 0.4`
- `velocity_ref_slew_rate = 0.8`
- `yaw_rate_assist_scale` 默认改为 `0.0`，避免 `CH1` 右摇杆左右通过 `/body_cmd.yaw_rate_assist` 叠加到转向。
- 默认转向只由 `CH4 -> /cmd_vel.angular.z -> target_yaw_rate` 进入控制器。
- 当前已将转向髋部前馈重新收口为单一运行时参数 `turn_hip_feedforward_scale`，默认值 `3.0` 与当前 `swerving_speed_ff = 3.0 * steer_output` 行为等价。
- 当前优先验证点固定为：`turn_hip_feedforward_scale = 3.0` 和 `turn_hip_feedforward_scale = 0.0`。
- 遥控连续量输入当前统一按 `-1.0 ~ 1.0` 口径观察与调参。

## 6. 调参原则

1. 一次只改一组强相关参数。
2. 每轮都先验证 `stand` 模式不被破坏。
3. 每轮都先用小命令，再用中等命令，不直接上满杆。
4. 先记录现象，再判断是否继续保留参数。
5. 当前已初步判断仅靠参数不足，应先修参考逻辑，再继续调参。

## 6.1 当前已固定的实现判断

- 当前 `velocity` 模式问题不再只按“调几个增益”处理，而是按“参考生成逻辑 + 空杆保护 + 输入滤波策略”共同导致处理。
- 当前默认判断是：`velocity` 模式中的速度目标会进入“速度参考积分成距离参考”的链路，因此模式切换、空杆和恢复复控时都必须重点检查参考是否被错误继承。
- 当前已验证纯积分口径下速度链路恢复正常：`target_distance` 由 `filtered_target_velocity * dt` 积分生成，回中不再重新锚定到当前 `body.distance`。
- 当前实现前置目标不是先继续放大 `target_velocity_scale`，而是先消除错误参考、空杆漂移和观测链路噪声对判断的干扰。
- 当前进一步观察到：实际位移 `body.distance` 会长期和 `target_distance` 保持一段距离，且初始化落地后总要先向后跑一段。
- 当前优先判断该现象可能来自 LQR 多状态耦合下的站立平衡点偏置：控制器用 `distance` 误差补偿 `phi` 或 `body.pitch` 目标误差。
- 当前进一步确认 MuJoCo 仿真默认仍保留 legacy stand control；ROS takeover 生效后会绕过 legacy 控制，改为只执行 ROS 发布的 `/joint_command`。
- 当前进一步确认 `body.distance` 不是世界系绝对 `x`，而是按底盘前向速度积分得到的距离状态；因此启动阶段先后退的一段会直接积成初始负偏置。
- 当前已进入遥控器调整阶段，优先通过滤波前/滤波后的目标对比决定低通参数。

## 6.2 LQR 状态与目标说明

当前 LQR 状态向量包含：

- `phi`
- `phi_rate`
- `body.distance`
- `body.velocity`
- `body.pitch`
- `body.pitch_rate`

当前 LQR 目标向量包含：

- `target_phi`，当前默认来自 `90.0 deg`
- 当前现场 ROS 与 MuJoCo 联调基线已临时改为 `97.1 deg`
- `target_phi_rate = 0`
- `target_distance`
- `target_velocity`
- `target_pitch = 0`
- `target_pitch_rate = 0`

因此，`distance` 偏差不一定只代表位置没跟上，也可能是姿态目标未校准时的平衡补偿结果。

## 7. 推荐调参顺序

1. 文档收口
   - 先固定问题定性、默认实现口径和主要参数入口。
   - 先明确当前不是纯参数收敛问题。

2. 观测链路收口
   - 先解决 `rqt_plot` 直接看高频调试流带来的卡顿问题。
   - 后续应优先看低频 plot 话题，而不是直接看全速调试流。

3. 参考逻辑修复
   - 重点检查 `velocity` 模式下速度参考初始化、距离积分和空杆保护。
   - `target_distance` 应由 `filtered_target_velocity * dt` 积分生成，不在空杆时重新锚定到当前车体位置。

4. 遥控器滤波调整
   - 观察速度、角速度和高度三组滤波前/滤波后参考。
   - 优先用低频 plot 话题决定 `velocity_ref_lpf_rc`、`yaw_rate_ref_lpf_rc` 和 `leg_length_ref_lpf_rc`。
   - 若响应过慢，再联动检查对应 slew rate 参数。

5. 速度持续性与速度上限
   - 在参考逻辑修复后，再观察 `velocity` 模式下机器人是否能持续形成前进趋势。
   - 在基础稳定不退化的前提下，再提高 `target_velocity_scale`。

6. 转向响应
   - 优先降低 `target_yaw_rate_scale`。
   - `yaw_rate_assist_scale` 默认保持 `0.0`，需要右摇杆左右参与辅助转向时再显式打开。

7. 高度范围
   - 逐步提高 `body_height_offset_scale`。
   - 逐步放宽 `target_leg_length_min` 与 `target_leg_length_max`。

## 7.1 速度参考 snap 与距离参考积分口径

- 当 `|filtered_target_velocity - commanded_target_velocity| <= 0.1` 时，速度参考应直接贴到目标，不继续做慢速拖尾收敛。
- `velocity` 模式下 `target_distance` 固定按 `filtered_target_velocity * dt` 积分生成。
- 空杆或接近零速时，不再把 `target_distance` 重新锚定到当前 `body.distance`，也不再额外锁存 `zero_hold_distance_ref`。
- 回中后若 `filtered_target_velocity = 0`，`target_distance` 只会自然停在上一积分值。

## 8. 每轮验证方法

每轮调参建议至少执行：

1. `stand` 模式空杆稳定观察。
2. `velocity` 模式空杆稳定观察。
3. 小幅 `CH2` 前进/后退测试。
4. 中幅 `CH2` 前进/后退测试。
5. 小幅 `CH4` 左右转向测试。
6. 高度调节输入测试。
7. 断链/failsafe 回归。

补充观测建议：

- `rqt_plot` 后续应优先看低频 plot 话题，目标观测频率以 `50-100 Hz` 为主。
- 控制主频不应因为 plot 观测卡顿而下调，当前主线仍按 `500 Hz` 控制链理解。
- 当前已补充 `/debug/plot/balance`，口径固定为：
  - `ref_primary=target_phi_deg`
  - `now_primary=avg_phi_deg`
  - `ref_secondary=target_pitch_deg`
  - `now_secondary=body_pitch_deg`
- 保留 `/debug/plot/velocity/ref_primary` 与 `/debug/plot/velocity/now_primary`，用于观察 `target_distance` 与 `body.distance` 的长期偏差。
- 当前新增三组滤波对比低频话题，口径固定为 `ref_primary=滤波前目标`、`now_primary=滤波后目标`：
  - `/debug/plot/ref_filter/velocity`
  - `/debug/plot/ref_filter/yaw_rate`
  - `/debug/plot/ref_filter/leg_length`
- 当前新增 roll 补偿观测话题：
  - `/debug/control/roll_balance`
  - `/debug/plot/roll_balance`
  - 口径固定为 `ref_primary=0.0`、`now_primary=body.roll`、`ref_secondary=body.roll_rate`、`now_secondary=roll_balance_output`

## 8.1 站立平衡点校准流程

1. 先补观测项
   - 当前直接观察 `/debug/plot/balance`。
   - `target_phi/avg_phi` 与 `target_pitch/body.pitch` 均按角度制输出，便于现场校准。
   - 保留现有 `target_distance` 与 `body.distance` 观测。

2. 先静态站立观察
   - 不给速度命令。
   - 记录稳定后的 `body.pitch`、`left_phi`、`right_phi`、`avg_phi`。
   - 记录 `distance_error = target_distance - body.distance`。
   - 判断实际平衡姿态是否偏离 `target_phi = 90.0 deg` 或 `target_pitch = 0`。

3. 再校准姿态目标
   - 当前优先通过参数 `target_phi_deg` 小步校准；历史设计基线是 `90.0`，当前现场临时保留值是 `97.1`。
   - 如 `body.pitch` 稳定偏离 `0`，再通过参数 `target_pitch_deg` 小步校准。
   - 每轮只小步调整一个目标，先看站立偏移和落地后后跑是否改善。

4. 最后回到速度链路
   - 只有当静态站立距离偏差明显减小后，再继续调 `velocity_ref_lpf_rc`、`velocity_ref_slew_rate` 和 `target_velocity_scale`。
   - 转向和高度仍排在速度基础稳定之后。

## 8.2 左转右压定位命令

- 当前最终结论：左转右压主因不是单纯腿长 PID 不够，也不是轮差速符号错误；主要是双腿发生劈叉后 `left_phi/right_phi` 不同，导致在左右腿长相同的情况下仍出现 body roll 偏差。
- 当前有效修正：加强 `anti_crash_pid` 抑制左右 `phi` 差，同时增加 `roll_balance_pid` 将 roll 误差补偿叠加到左右 LQR hip 输出，左右符号相反。
- 当前腿长重力补偿已改为按 `roll` 角计算，避免用 `pitch` 角解释横向侧倾载荷。
- 后续转向调参应先保证左右 `phi` 差与 roll 偏差不过度积累，再回头微调 `turn_hip_feedforward_scale` 的转向手感。
- 启动 controller 并显式固定当前转向髋前馈基线：
  - `ros2 run wheel_leg_control wheel_leg_controller_node --ros-args -p turn_hip_feedforward_scale:=3.0`
- 在线临时关闭转向髋前馈，直接验证左转右压是否同步减轻：
  - `ros2 param set /wheel_leg_controller turn_hip_feedforward_scale 0.0`
- 在线恢复当前等价默认行为：
  - `ros2 param set /wheel_leg_controller turn_hip_feedforward_scale 3.0`
- 采集左右轮差速、anti-crash 与左右髋差值调试量：
  - `python3 tools/turn_sign_capture.py --output /tmp/wheel_leg_turn_sign_capture.csv`
- 当前 `turn_hip_feedforward_scale` 支持运行时更新；`steer_pid.*`、`anti_crash_pid.*`、`leg_length_pid.*` 与 `target_yaw_rate_scale` 仍按“改后重启 controller 才真正生效”处理。
- 当前 `roll_balance_pid.*` 也按“改后重启 controller 才真正生效”处理。

## 9. 每轮记录模板

每轮调参至少记录：

- 日期
- 修改参数
- 修改前数值
- 修改后数值
- 测试动作
- 观察现象
- 是否保留
- 是否引出新的问题

建议记录格式：

```text
- 轮次：
- 参数：
- 修改：
- 测试：
- 结果：
- 结论：
```

## 10. 完成标准

满足以下条件即可认为本轮调参任务达到阶段性完成：

- `velocity` 模式下前进命令的持续性明显改善。
- 当前速度上限比现状更高，且未明显破坏稳定性。
- 转向响应明显比当前更温和、更可控。
- 高度调节范围明显扩大，且基本稳定。
- 当前基线仍保持 `stand`、failsafe、恢复复控正常。

## 11. 非目标

- 不在本任务中完成 STM32 对接。
- 不在本任务中完成最终实机控制定型。
- 不在本任务中承诺彻底重写全部控制算法。

## 12. 当前状态

- `[v]` 仿真遥控控制基线已冻结，可进入 STM32 通信阶段

## 12.0 冻结基线参数

当前阶段已完成 RC 映射参数、PID 参数与 LQR 参数确认。以下参数已固化为当前仿真遥控控制基线：

```text
RC 映射：
- linear_x.channel = 2
- linear_x.scale = 1.0
- angular_z.channel = 4
- angular_z.scale = 1.0
- angular_z.reverse = true
- body_height.channel = 3
- body_height.scale = 1.0
- yaw_rate_assist.channel = 1
- yaw_rate_assist.scale = 1.0
- body_height_scale = 0.15

Controller 目标映射：
- target_velocity_scale = 0.6
- target_yaw_rate_scale = 2.0
- yaw_rate_assist_scale = 0.0
- body_height_offset_scale = 0.2
- target_leg_length_min = 0.23
- target_leg_length_max = 0.33
- target_phi_deg = 97.1
- target_pitch_deg = 0.0
- turn_hip_feedforward_scale = 3.0

输入参考滤波：
- velocity_ref_lpf_rc = 0.12
- velocity_ref_slew_rate = 1.5
- velocity_snap_threshold = 0.1
- yaw_rate_ref_lpf_rc = 0.4
- yaw_rate_ref_slew_rate = 1.8
- leg_length_ref_lpf_rc = 0.15

Legacy PID：
- leg_length_pid = kp 800.0, ki 50.0, kd 30.0, max_output 5000.0
- steer_pid = kp 6.0, ki 0.8, kd 0.0, max_output 50.0
- anti_crash_pid = kp 20.0, ki 0.5, kd 3.0, max_output 10.0
- roll_balance_pid = kp 20.0, ki 3.0, kd 0.2, max_output 10.0
```

阶段收口结论：

- `stand`、`velocity`、failsafe 与恢复复控保持为当前回归重点。
- `velocity` 回中参考漂移已修复为纯积分口径，不再每帧追随 `body.distance`。
- 左转右压已收敛，最终按“双腿劈叉导致 roll 偏差”处理，加强 `anti_crash_pid` 并增加 `roll_balance_pid`。
- 高度范围已通过 `body_height_offset_scale` 与 `target_leg_length_min/max` 固化到当前可用窗口。
- 这一阶段不再继续引入新的控制结构；后续进入 STM32 通信/实机闭环阶段。

回归记录：

```text
- 2026-06-24 colcon build 回归通过：
  colcon build --packages-select wheel_leg_msgs wheel_leg_common wheel_leg_bridge wheel_leg_control wheel_leg_sim wheel_leg_rc --event-handlers console_direct+
- 结果：
  6 packages finished
```

## 12.1 当前调试记录

```text
- 轮次：velocity zero-hold 参考漂移修复
- 参数/逻辑：
  - 新增低频 /debug/plot/* 观测链路，解决 rqt_plot 高频卡顿。
  - 新增 /debug/plot/wheel_effort，用于观察左右轮输出与 zero-hold 标志。
  - 将 zero-hold 距离参考从“每帧重锚到当前 body.distance”改为“进入中位候选区时锁存一次 zero_hold_distance_ref”。
- 测试：
  - 用户执行“上推 -> 回中 -> 下推 -> 回中”。
  - 观察 /debug/plot/velocity/ref_primary、now_primary、ref_secondary、now_secondary 与 /debug/plot/wheel_effort。
- 结果：
  - rqt_plot 卡顿问题已解决。
  - 已确认 /cmd_vel.linear.x 回中为 0.0，排除遥控中位偏差导致的持续负命令。
  - 已定位 ref_primary 回中后持续负斜率的原因是 target_distance 每帧追随 body.distance。
  - 改为锁存 zero_hold_distance_ref 后，当前现场反馈“先解决了”。
- 结论：
  - 该问题属于 velocity 参考生成逻辑问题，不是单纯滤波参数或遥控中位偏差问题。
  - 后续调参应在该修复基础上继续速度、转向和高度收敛。
```

```text
- 轮次：左转右压最终收敛结论
- 现象：
  - 左转时出现向右侧压、右腿被压低且仅增大腿长 kp 不能可靠恢复。
  - 双腿腿长相同的情况下，仍可能因为左右 phi 不同导致车体 roll 偏差。
- 根因：
  - 双腿劈叉导致 `left_phi/right_phi` 出现差异，等腿长并不等价于等横向支撑。
  - `phi` 差引出的 body roll 偏差会让一侧腿持续受压，表现为“右腿变短/回不去”。
- 修改：
  - 加强 `anti_crash_pid`，优先限制左右 `phi` 劈叉。
  - 增加 `roll_balance_pid`，以 body roll 为输入，输出叠加到左右 LQR hip 端且左右反号。
  - 腿长重力补偿改用 roll 角计算。
- 结论：
  - 问题已解决。
  - 后续不要优先把该现象归因到腿长 PID 太软；应先看左右 `phi` 差、`anti_crash_output`、`roll_balance_output` 与左右 hip 差力矩。
```

```text
- 轮次：velocity 纯积分 x_ref 验证与遥控器调整入口
- 参数/逻辑：
  - 取消中位候选区的 zero_hold_distance_ref 锁存口径。
  - velocity 模式下 target_distance 固定由 filtered_target_velocity * dt 积分生成。
  - yaw_rate_assist_scale 默认改为 0.0，避免 CH1 右摇杆左右叠加到转向。
  - 新增 /debug/plot/ref_filter/velocity、/debug/plot/ref_filter/yaw_rate、/debug/plot/ref_filter/leg_length。
- 测试：
  - 用户现场验证 velocity 纯积分 x_ref 后整机表现。
- 结果：
  - 当前反馈速度链路已正常。
  - 下一阶段进入遥控器输入滤波调参。
- 结论：
  - 后续通过三组 ref_filter 话题对比滤波前/滤波后目标，决定低通滤波参数。
```

```text
- 轮次：滤波参数运行时更新修复
- 参数/逻辑：
  - 将 velocity_ref_lpf_rc、velocity_ref_slew_rate、velocity_snap_threshold 纳入动态参数更新。
  - 将 yaw_rate_ref_lpf_rc、yaw_rate_ref_slew_rate、leg_length_ref_lpf_rc 纳入动态参数更新。
  - 将 yaw_rate_assist_scale 纳入动态参数更新。
- 现象：
  - ros2 param set velocity_ref_lpf_rc 10.0 显示成功，但 /debug/plot/ref_filter/velocity 中 now_primary 仍几乎贴着 ref_primary。
- 结论：
  - 原因是参数服务器值已改变，但控制器运行时成员变量没有同步刷新。
  - 修复后可直接用 ros2 param set 调低通参数，并通过 ref_filter 话题观察滤波前后差异。
```

```text
- 轮次：slew_rate 关闭语义修复
- 现象：
  - velocity_ref_lpf_rc 已显示为 10.0，velocity_ref_slew_rate 已显示为 0.0，但 now_primary 仍贴着 ref_primary。
- 原因：
  - 斜率限制关闭语义需要固定为“不限制低通候选值”，否则现场很难判断低通参数是否真的生效。
- 修改：
  - max_rate <= 0 时返回 target，表示关闭斜率限制并直接采用前一步低通候选值。
- 结论：
  - 若要观察纯一阶低通，设置 velocity_snap_threshold=0.0、velocity_ref_slew_rate=0.0，再调 velocity_ref_lpf_rc。
```

```text
- 轮次：slew_rate 斜率限制顺序修复
- 现象：
  - 将 velocity_ref_slew_rate 调得很小后，ref_filter 曲线仍主要表现为低通曲线，斜率变化不明显。
- 原因：
  - 旧实现先做低通，再以“低通后的当前值 -> 原始目标值”做 slew 限制。
  - 这样限制到的是剩余误差，不是“上一帧最终输出 -> 这一帧最终输出”的实际变化量。
- 修改：
  - 先生成低通候选值，再用上一帧最终输出对该候选值做 slew 限制。
- 结论：
  - 现在 velocity_ref_slew_rate / yaw_rate_ref_slew_rate 会真正限制最终输出斜率。
```

```text
- 轮次：velocity 参数与摇杆口径固定
- 参数：
  - velocity_snap_threshold = 0.0
  - velocity_ref_lpf_rc = 0.4
  - velocity_ref_slew_rate = 0.8
  - 遥控连续量输入范围按 -1.0 ~ 1.0 口径观察
- 结论：
  - 当前先固定 velocity 三个参数，后续优先继续看 yaw_rate 和 leg_length 的滤波表现。
```

```text
- 轮次：转向单侧压翻风险隔离
- 现象：
  - 向右转时整机可平稳转向，向左打时出现明显向右侧压、摇杆加大后向右倾倒。
- 判断：
  - 当前最可疑的是转向时叠加到左右髋关节上的 swerving/turn hip feedforward，而不是 yaw_rate 参考滤波本身。
- 结论：
  - 已按现场要求回退抗劈叉附近控制链修改。
  - 当前 swerving_speed_ff 默认仍等价于 3.0 * steer_output，但已重新暴露为单参数 `turn_hip_feedforward_scale`，方便直接做 3.0/0.0 对照验证。
```

```text
- 轮次：转向符号采集脚本与第一次数据结论
- 工具：
  - 新增 tools/turn_sign_capture.py，用于采集 yaw、yaw_ref_filter、anti_crash、wheel_effort 和 turn_internal 调试量。
- 第一次数据：
  - 后两段轻微左打接近侧翻，CSV 中 yaw_ref 为正，yaw_now 同号跟随。
  - yaw_ref 负方向段中 wheel_right - wheel_left 均值约为 -0.054。
  - 后两段 yaw_ref 正方向段中 wheel_right - wheel_left 均值约为 +0.157 和 +0.178。
- 初步结论：
  - 左右轮差速主链能随 yaw_ref 反号，不是当前首要嫌疑。
  - 下一轮重点观察新增 /debug/plot/turn_internal，直接确认 steer_output、swerving_speed_ff 和左右髋差值的符号关系。
```

```text
- 轮次：turn_internal 第二次采集结论
- 数据：
  - yaw_ref 负方向段：yaw_ref mean=-1.10，yaw_now mean=-1.09，wheel_R-L mean=-0.057。
  - yaw_ref 正方向段：yaw_ref mean=+0.36/+0.32，yaw_now 同号跟随，wheel_R-L mean=+0.126/+0.125。
  - yaw_ref 正方向段：steer_output mean=+0.235/+0.221，swerving_speed_ff mean=+0.705/+0.662，left_minus_right_hip_torque mean=+1.868/+1.701。
  - 回中后仍有 steer_output 正向残留，带来 swerving_speed_ff 正向残留。
- 结论：
  - 左右轮差速主链会随 yaw_ref 反号，暂不作为首要嫌疑。
  - 异常更集中在 steer_output -> swerving_speed_ff -> 左右髋差值链路。
  - 左打时 swerving_speed_ff 明显为正并推高 left_minus_right_hip_torque，和“持续向右侧压”的现象一致。
- 下一步：
  - 暂不继续改抗劈叉/转向髋部前馈控制链。
  - 若继续查左转侧压，优先通过观测链路确认符号和姿态状态，不先改 anti_crash/swerving 计算。
```

## 12.2 站立平衡点偏置记录

```text
- 轮次：站立平衡点偏置问题确认
- 现象：
  - 初始化落地后机器人总要先向后跑一段。
  - 站立或回中后，实际位移 body.distance 会和 target_distance 长期保持一段距离。
- 当前判断：
  - /cmd_vel.linear.x 回中已确认为 0.0，因此当前不优先归因到遥控器中位或死区。
  - LQR 同时使用 phi、distance、velocity、pitch 等状态；若 target_phi 或 target_pitch 未校准，distance 误差可能被用于抵消姿态误差。
- 当前优先级：
  - 先补充 phi/pitch 观测。
  - 再校准 target_phi/target_pitch。
  - 暂不调整 linear_x.center、linear_x.deadzone、scale/reverse 等遥控输入映射参数。
```

## 12.3 `target_phi = 97.1 deg` 基线记录

```text
- 轮次：站立平衡点临时基线固定
- 修改：
  - MuJoCo 侧 stand 默认 target_phi 调整到 97.1 deg。
  - ROS 侧 wheel_leg_controller_node 默认 target_phi_deg 调整到 97.1。
- 测试：
  - 先观察不接管 ROS 时仿真初始落地行为。
  - 再观察 ROS takeover 后 stand / velocity 回中静站行为。
- 结果：
  - 启动后“先向后退一段再停住”的现象明显减小。
  - ROS 接管后，回中静站时不再长期保留一段明显位移差。
  - 再大一点或再小一点 target_phi，会重新出现持续前跑/后跑，distance 持续积分。
  - 当前 97.1 deg 附近虽然能把大偏置压住，但稳态仍有小幅振荡，表现不如 MuJoCo legacy stand control 平滑。
- 结论：
  - 当前 target_phi 的主要问题已从“大偏置/后退”收口到“稳态小振荡”。
  - 下一步不应再把问题主要归因到 zero-hold 锁存逻辑，而应继续检查 LQR 增益与 x/dx 权重。
```

## 12.4 MuJoCo / ROS 接管关系与距离状态口径记录

```text
- 轮次：仿真接管关系与 distance 状态口径确认
- 现象：
  - 不开启 ROS 控制时，MuJoCo 仿真自身也能稳定站立。
  - 开启 ROS takeover 后，仿真控制表现会和 MuJoCo 默认站立控制表现不同。
  - 不接管时 target_distance 为 0，机器人也不一定回到积分意义上的 0。
- 当前判断：
  - MuJoCo 仿真默认保留 legacy stand control；ROS takeover 生效后，仿真会绕过 legacy 控制，改为只执行 ROS /joint_command。
  - 当前 body.distance 不是世界系绝对 x，而是由前向速度积分得到；启动阶段的短时后退会直接形成 distance 初始偏置。
  - 因此“distance 不回 0”不必然代表场景世界坐标没有回到原点，也可能只是启动阶段已经积掉一段距离。
- 结论：
  - 后续应把“站立平衡点是否正确”和“distance 积分状态是否有历史偏置”分开判断。
  - 当前优先目标仍是先消除 ROS takeover 后的稳态振荡，再决定是否需要额外引入世界系绝对 x 观测。
```

## 12.5 稳态振荡与 LQR 下一步记录

```text
- 轮次：稳态振荡问题确认
- 现象：
  - 在 target_phi=97.1 deg 基线下，机器人虽然不再明显后退，但回中静站时仍存在持续小幅振荡。
  - 调整 target_phi 到邻域其他数值时，要么恢复前后跑偏，要么稳态振荡仍无法完全消除。
- 当前判断：
  - 当前振荡更像站立平衡点附近的欠阻尼/极限环，而不是 zero-hold 一次锁存逻辑直接导致。
  - 仅继续微调 target_phi 已难同时兼顾“无明显偏置”和“低振荡”。
  - 当前 MATLAB LQR 权重为：
    Q = diag([500 100 10 20 300 60])
    R = diag([0.01 0.02])
  - 其中 x / dx 权重相对 phi / pitch 明显偏轻，后续需要优先尝试提高 x / dx 权重并重算增益。
- 下一步：
  - 先在 MATLAB 中提高 x / dx 权重，重新计算一版 LQR 增益。
  - 新增益验证通过前，当前 97.1 deg 基线只作为临时可运行版本，不视为最终参数基线。
```

## 13. 下一步建议

1. 保留当前冻结参数作为 STM32 通信阶段前的仿真控制基线。
2. 进入 STM32 阶段后，先保证 `/joint_command`、传感器状态与 failsafe 链路一致，再迁移控制参数。
3. 实机闭环初期不要同步放大速度、转向和高度范围；优先小命令验证。
4. 如实机出现新的稳态振荡或侧压，再按本轮记录优先检查 `phi` 差、roll 补偿和输出限幅。
