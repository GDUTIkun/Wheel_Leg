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

- `[ ]` 速度拉满后不能持续向前跑的问题定位与收敛
- `[ ]` 站立平衡点偏置定位与校准
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
- `velocity_snap_threshold`
- `velocity_zero_hold_threshold`
- `target_phi_deg`
- `target_pitch`
- 腿长 PID、转向 PID 及相关 legacy 控制参数
- 如有必要，再继续检查速度误差进入控制链的定义与计算位置

当前参数口径先固定为：

- `velocity_ref_lpf_rc` 越大，滤波越强。
- `velocity_ref_slew_rate` 越小，速度参考变化越平缓。
- `velocity_snap_threshold` 默认按 `0.1 m/s` 理解，判断口径固定为 `|filtered_target_velocity - commanded_target_velocity|`。
- `velocity_zero_hold_threshold` 默认按 `0.1 m/s` 理解，用于空杆或近零速时的距离参考重锚定保护。
- `target_phi_deg` 后续应作为腿部姿态目标校准入口，默认仍以当前 `90.0 deg` 为基线。
- `target_pitch` 后续仅在确认 `body.pitch` 稳定偏离 `0` 后再引入或调整。

当前现场临时基线先固定为：

- `wheel_leg_controller_node` 默认 `target_phi_deg = 97.1`
- MuJoCo 侧 stand 默认 `target_phi = 97.1 deg`
- 当前该基线的主要用途是先消除“仿真启动后先后退一段”和“回中后长期保留一段位移差”的明显现象

当前暂不调整以下遥控输入映射参数：

- `linear_x.center`
- `linear_x.deadzone`
- `linear_x.scale`
- `linear_x.reverse`

## 6. 调参原则

1. 一次只改一组强相关参数。
2. 每轮都先验证 `stand` 模式不被破坏。
3. 每轮都先用小命令，再用中等命令，不直接上满杆。
4. 先记录现象，再判断是否继续保留参数。
5. 当前已初步判断仅靠参数不足，应先修参考逻辑，再继续调参。

## 6.1 当前已固定的实现判断

- 当前 `velocity` 模式问题不再只按“调几个增益”处理，而是按“参考生成逻辑 + 空杆保护 + 输入滤波策略”共同导致处理。
- 当前默认判断是：`velocity` 模式中的速度目标会进入“速度参考积分成距离参考”的链路，因此模式切换、空杆和恢复复控时都必须重点检查参考是否被错误继承。
- 当前实现前置目标不是先继续放大 `target_velocity_scale`，而是先消除错误参考、空杆漂移和观测链路噪声对判断的干扰。
- 当前进一步观察到：实际位移 `body.distance` 会长期和 `target_distance` 保持一段距离，且初始化落地后总要先向后跑一段。
- 当前优先判断该现象可能来自 LQR 多状态耦合下的站立平衡点偏置：控制器用 `distance` 误差补偿 `phi` 或 `body.pitch` 目标误差。
- 当前进一步确认 MuJoCo 仿真默认仍保留 legacy stand control；ROS takeover 生效后会绕过 legacy 控制，改为只执行 ROS 发布的 `/joint_command`。
- 当前进一步确认 `body.distance` 不是世界系绝对 `x`，而是按底盘前向速度积分得到的距离状态；因此启动阶段先后退的一段会直接积成初始负偏置。
- 在完成站立平衡点校准前，不继续优先调整遥控输入中位、死区或速度输入映射。

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

4. 站立平衡点校准
   - 先补充 `phi/pitch` 观测，再判断实际稳定姿态是否偏离当前目标。
   - 优先校准 `target_phi`，必要时再考虑 `target_pitch`。
   - 在该阶段暂不调整遥控输入相关参数。

5. 速度持续性与速度上限
   - 在参考逻辑修复后，再观察 `velocity` 模式下机器人是否能持续形成前进趋势。
   - 在基础稳定不退化的前提下，再提高 `target_velocity_scale`。

6. 转向响应
   - 优先降低 `target_yaw_rate_scale`。
   - 如仍偏快，再看 `yaw_rate_assist_scale` 和转向相关控制参数。

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

- `[~]` 进行中，已完成观测链路收口与 `velocity` 回中参考漂移修复

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

1. 保留当前 `target_phi = 97.1 deg` 的临时可运行基线，用于后续 ROS takeover 验证。
2. 在 MATLAB 中优先提高 `x/dx` 权重，重新计算一版 LQR 增益。
3. 用新增益先验证稳态振荡是否明显减小，再决定是否继续微调 `target_phi/target_pitch`。
4. 站立平衡点与稳态振荡都明显改善后，再回到速度、转向和高度调参。
