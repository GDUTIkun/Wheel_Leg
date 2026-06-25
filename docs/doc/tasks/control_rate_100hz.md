# `100 Hz` ROS 控制降频阶段文档

## 1. 背景与目标

在正式接入硬件之前，需要先把 ROS 侧控制链从当前仿真主线的 `500 Hz` 收口到 `100 Hz`，以贴近后续实际通信节拍。

这一阶段的目标不是立即给出最终实机参数，也不是立刻改完整控制实现，而是先形成一份可直接执行的技术说明，明确：

- 哪些控制逻辑必须先改
- 哪些参数最可能因 `500 Hz -> 100 Hz` 失稳
- 为什么这些参数敏感
- 应该按什么顺序验证和调参
- 再次出现“直接失衡”时优先排查哪里

本阶段输出固定为：

- 一份 `100 Hz` 降频专项说明
- 需要调整的控制参数与逻辑清单
- 验证矩阵
- 首轮调参顺序
- 风险归因口径

## 2. 开篇先固定的三个事实

1. 当前 `/wheel_leg_controller` 不是靠内部定时器按 `500 Hz` 自己跑，而是“每收到一帧 `/robot_state` 就执行一步控制”。
2. 当前 controller 的 `dt` 守门范围写死在约 `0.002 s`，只接受 `0.0015 ~ 0.0035 s`；直接切到 `100 Hz` 的 `0.010 s` 会先触发这一层问题。
3. 仓库已有历史验证记录：`/robot_state` 降到约 `100 Hz` 时，controller 的 `dt` 已验证为 `0.010000000 s`，且当时“当前参数下仍会失稳”。

## 3. 当前实现基线

### 3.1 controller 当前如何跑

- `wheel_leg_controller_node` 订阅 `/robot_state`，每来一帧状态就执行一步控制。
- `wheel_leg_controller.publish_rate_hz` 当前只是被声明，但实现里已经明确写为 ignored，不再决定控制频率。
- 当前 `dt` 由 `/robot_state.header.stamp` 相邻两帧时间戳差值得到。

当前代码口径可归纳为：

```text
/robot_state
  -> controller 计算 dt
  -> dt 过守门范围才允许进入控制计算
  -> 执行 PID / LQR / VMC / 目标滤波
  -> 发布 /joint_command
```

### 3.2 当前 `500 Hz` 基线从哪里来

- MuJoCo bridge 当前按仿真步直接发布 `/robot_state`。
- 当前仿真主线下，controller 连续 `dt` 样本已经验证为约 `0.002000000 s`。
- 因此，当前“`500 Hz` 控制”本质上是“`500 Hz` 状态源频率 + `500 Hz` 控制步长”共同形成的基线。

### 3.3 硬件桥这边的当前默认值

- `wheel_leg_stm32_bridge.publish_rate_hz` 当前默认值为 `500.0`。
- 在正式硬件接入前，需要把状态发布节拍和控制节拍一起收口到 `100 Hz`。

### 3.4 现有历史验证结论

仓库已有两条与本任务直接相关的验证结论：

- `500 Hz` 主线当前已验证稳定，controller `dt` 连续样本约为 `0.002000000 s`。
- 之前做过 `/robot_state` 每 `5` 帧转发 `1` 帧的 `100 Hz` 降采样 takeover 实验：
  - `/robot_state_100hz` 约 `99.996 Hz`
  - controller `dt` 连续样本约 `0.010000000 s`
  - takeover 后出现输出限幅，随后仿真失稳

因此，这次任务的默认前提不是“证明 `100 Hz` 一定不行”，而是：

- “当前 `500 Hz` 参数不经重整定，直接切 `100 Hz` 不稳定”已被验证
- 本轮要做的是把需要调整的逻辑和参数明确列出来

## 4. 必调项分组

本轮先把潜在改动分成三组：

- `必须先改的逻辑项`
- `高优先参数项`
- `低优先但必须联查的项`

### 4.1 必须先改的逻辑项

这些项不先处理，后面的 PID 调参没有意义。

1. `dt` 守门范围
- 当前 controller 只接受约 `0.002 s` 的主线。
- 后续切到 `100 Hz` 时，需要允许 `0.010 s` 主线进入控制计算。
- 否则 controller 会直接把 `100 Hz` 状态样本当异常丢掉。

2. 与 `dt` 强耦合的 debug / 日志口径
- 当前异常日志里仍写着 `expected about 0.002 s`。
- 后续如果主线改为 `100 Hz`，这类提示必须同步更新，否则现场排查会被误导。

3. `100 Hz` 方案的口径说明
- 如果保留 MuJoCo `500 Hz` 物理步长，但只让 ROS 侧按 `100 Hz` 控制，需要明确这叫“状态降采样 + 控制降频”。
- 这不等于“仿真整体变慢”。
- 后续调参和复盘都必须基于这个口径描述问题。

## 5. 高优先参数项

这一组是本轮最可能因为 `500 Hz -> 100 Hz` 出现明显行为变化的参数。

### 5.1 `leg_length_pid.*`

当前优先级：

- 先看 `kd`
- 再看 `ki`
- `kp` 暂不作为第一优先

原因：

- 导数项包含 `/dt`，离散导数项和导数滤波行为会变
- 积分项按 `ki * error * dt` 累加，单步积分量会变大
- 腿长环直接关系到站立稳定和落地后的竖向振荡

### 5.2 `anti_crash_pid.*`

当前优先级：

- 先看 `kd`
- 再看 `ki`

原因：

- 这是转向和左右腿耦合的敏感环
- 历史问题里它与“左右 `phi` 劈叉”以及失衡直接相关
- 降频后，它的响应相位和抑制强度都可能变化

### 5.3 `roll_balance_pid.*`

当前优先级：

- 先看 `kd`
- 再看 `ki`
- 再联看 `output_lpf_rc`
- 再联看 `derivative_lpf_rc`

原因：

- 这个环直接基于 body roll 做左右反号补偿
- 降频后最值得警惕的是相位裕度下降
- 不能只盯 `kp/ki/kd`，因为它本身就启用了输出滤波和导数滤波

### 5.4 `steer_pid.*`

当前优先级：

- 主看 `ki`

原因：

- 当前 `kd = 0`
- 低频通信下更容易出现慢积分堆积、回中拖尾和残余转向输出

### 5.5 PID 内部滤波参数

这一组必须单独点出来，不要只把注意力放在 `kp/ki/kd`。

重点参数：

- `leg_length_pid.derivative_lpf_rc`
- `steer_pid.derivative_lpf_rc`
- `anti_crash_pid.derivative_lpf_rc`
- `roll_balance_pid.derivative_lpf_rc`
- `roll_balance_pid.output_lpf_rc`

原因：

- 它们与 `dt` 一起决定离散实现的真实动态
- 降频后，即使 `kp/ki/kd` 不改，滤波器等效响应也会变

## 6. 低优先但必须联查的项

这组通常不是“第一刀”要动的地方，但必须纳入联查，避免把问题都误判成 PID。

- `velocity_ref_lpf_rc`
- `velocity_ref_slew_rate`
- `yaw_rate_ref_lpf_rc`
- `yaw_rate_ref_slew_rate`
- `leg_length_ref_lpf_rc`
- `turn_hip_feedforward_scale`
- `velocity_zero_hold_threshold`
- `velocity_zero_hold_exit_threshold`
- `velocity_zero_hold_entry_delay_sec`
- `zero_hold_measurement_clamp_threshold`
- `zero_hold_wheel_effort_limit`
- `command_timeout_sec`

当前结论口径固定为：

- LQR 增益本身本阶段默认不作为首调对象，因为当前 LQR 计算本身不直接使用 `dt`
- 但 LQR 的目标输入和外围叠加环会变，因此不能把“LQR 不先调”理解成“LQR 链路完全不受影响”

## 7. 机理分析

这一轮不能只写成“先把 D 和 I 降一点试试”。

### 7.1 为什么不能只说“降 D、降 I”

当前控制链里，真正受采样频率变化影响的不止 PID 数值本身，还包括：

- PID 离散积分
- PID 离散导数
- 导数滤波
- 输出滤波
- 参考低通
- 参考斜率限制
- 若干按秒定义但按采样触发的辅助逻辑

### 7.2 积分项为什么敏感

PID 积分项当前实现口径是：

```text
i_term = ki * error * dt
```

因此，在同样误差下：

- 从 `0.002 s` 变到 `0.010 s`
- 单步积分量会增大
- 若不重整定，积分累积速度和饱和行为都可能变化

### 7.3 导数项为什么敏感

PID 导数项当前实现口径是：

```text
d_out = kd * (error - last_error) / dt
```

因此：

- 离散导数幅值会因 `dt` 变化而变化
- 即使观测噪声本身不变，导数支路的实际手感也会变
- 开启导数滤波时，滤波器与 `dt` 的相对关系也会变

### 7.4 为什么滤波和 slew 也要重看

当前目标滤波已经按 `dt` 归一：

- 一阶低通使用 `dt / (rc + dt)`
- slew limit 使用 `max_rate * dt`

这意味着它们在公式上不是“写死 `500 Hz`”。

但这并不等于：

- `100 Hz` 下手感一定与 `500 Hz` 等价
- 相位裕度一定不变
- 与 PID / LQR / 前馈叠加后的总体闭环一定不变

所以这些项不一定先改，但一定要复验。

### 7.5 按秒定义的逻辑为什么仍要联查

像下面这些参数：

- `velocity_zero_hold_entry_delay_sec`
- `command_timeout_sec`

理论上是按秒定义的，不需要简单按 `5` 倍或 `1/5` 倍重算。

但在 `100 Hz` 下：

- 触发颗粒度会更粗
- 一些状态切换可能更晚发生
- 对回中、空杆和短暂断链的感知会变

因此它们不一定是首调对象，但必须纳入联查。

## 8. 推荐调参顺序

### 第一步：先改 `dt` 守门与状态源节拍

- 先让 controller 真正接受 `0.010 s` 主线
- 先确认 `/robot_state` 实际约 `100 Hz`
- 先确认 controller 不再因 `out-of-range dt` 丢样

在这一步完成前，不进入 PID 调参。

### 第二步：只在纯 `stand` 模式看站立相关环

这一轮只看：

- `leg_length_pid.*`
- `roll_balance_pid.*`
- `anti_crash_pid.*`

优先策略：

- 优先压 `D`
- 谨慎压 `I`
- 先不要大动 `kp`

目的：

- 先把“静态站立就失衡”这类最基础问题压住

### 第三步：再看转向相关联动

这一轮再看：

- `steer_pid.*`
- `turn_hip_feedforward_scale`
- `anti_crash_pid.*`

重点不是先追求灵敏度，而是先防止：

- 左右转单侧压翻
- 回中后残余转向输出
- 转向时左右腿姿态差越积越大

### 第四步：最后再看参考滤波和 zero-hold

这一轮再看：

- `velocity_ref_*`
- `yaw_rate_ref_*`
- `leg_length_ref_lpf_rc`
- `velocity_zero_hold_*`

重点是确认：

- 降到 `100 Hz` 后是否出现额外慢相位
- 回中和零速保持是否变钝
- 速度、角速度和高度目标是否出现明显拖尾

### 第五步：再决定是否进一步限幅或降响应

如果前四步完成后仍没有明显失衡，再决定是否需要针对硬件通信阶段进一步做：

- 命令限幅收紧
- 参考变化速度再降
- 前馈项再减

## 9. 参数检查表

| 参数/参数组 | 所属环路 | 当前默认值 | 与 `dt` 的关系 | `100 Hz` 下首要关注方向 | 是否支持运行时更新 |
| --- | --- | --- | --- | --- | --- |
| `leg_length_pid.kp` | 腿长 PID | `800.0` | 间接受 `dt` 影响 | 暂不首调，先看 D/I 后再决定 | 否，按重启后生效处理 |
| `leg_length_pid.ki` | 腿长 PID | `50.0` | 积分项直接乘 `dt` | 谨慎下调，防止积分堆积 | 否，按重启后生效处理 |
| `leg_length_pid.kd` | 腿长 PID | `30.0` | 导数项直接除以 `dt` | 首先联调 | 否，按重启后生效处理 |
| `leg_length_pid.derivative_lpf_rc` | 腿长 PID 导数滤波 | `0.01` | 与 `dt` 共同决定导数滤波响应 | 与 `kd` 联查 | 否，按重启后生效处理 |
| `steer_pid.kp` | 转向速度环 | `6.0` | 间接受 `dt` 影响 | 非第一优先 | 否，按重启后生效处理 |
| `steer_pid.ki` | 转向速度环 | `0.8` | 积分项直接乘 `dt` | 首先联调，防止回中拖尾 | 否，按重启后生效处理 |
| `steer_pid.kd` | 转向速度环 | `0.0` | 当前未启用 | 暂无首调意义 | 否，按重启后生效处理 |
| `anti_crash_pid.kp` | 抗劈叉 | `20.0` | 间接受 `dt` 影响 | 非第一优先 | 否，按重启后生效处理 |
| `anti_crash_pid.ki` | 抗劈叉 | `0.5` | 积分项直接乘 `dt` | 高优先联调 | 否，按重启后生效处理 |
| `anti_crash_pid.kd` | 抗劈叉 | `0.5` | 导数项直接除以 `dt` | 高优先联调 | 否，按重启后生效处理 |
| `anti_crash_pid.derivative_lpf_rc` | 抗劈叉导数滤波 | `0.01` | 与 `dt` 强耦合 | 与 `kd` 联查 | 否，按重启后生效处理 |
| `roll_balance_pid.kp` | Roll 补偿 | `20.0` | 间接受 `dt` 影响 | 非第一优先 | 否，按重启后生效处理 |
| `roll_balance_pid.ki` | Roll 补偿 | `3.0` | 积分项直接乘 `dt` | 高优先联调 | 否，按重启后生效处理 |
| `roll_balance_pid.kd` | Roll 补偿 | `0.2` | 导数项直接除以 `dt` | 高优先联调 | 否，按重启后生效处理 |
| `roll_balance_pid.derivative_lpf_rc` | Roll 导数滤波 | `0.01` | 与 `dt` 强耦合 | 与 `kd` 联查 | 否，按重启后生效处理 |
| `roll_balance_pid.output_lpf_rc` | Roll 输出滤波 | `0.01` | 与 `dt` 强耦合 | 高优先联查 | 否，按重启后生效处理 |
| `velocity_ref_lpf_rc` | 速度目标滤波 | `0.12` | 已按 `dt` 归一 | 低优先复验手感 | 是 |
| `velocity_ref_slew_rate` | 速度目标斜率限制 | `1.5` | 已按 `dt` 归一 | 低优先复验慢相位 | 是 |
| `yaw_rate_ref_lpf_rc` | 角速度目标滤波 | `0.4` | 已按 `dt` 归一 | 低优先复验手感 | 是 |
| `yaw_rate_ref_slew_rate` | 角速度目标斜率限制 | `1.8` | 已按 `dt` 归一 | 低优先复验慢相位 | 是 |
| `leg_length_ref_lpf_rc` | 腿长目标滤波 | `0.15` | 已按 `dt` 归一 | 低优先复验响应迟滞 | 是 |
| `turn_hip_feedforward_scale` | 转向髋前馈 | `3.2` | 不直接用 `dt`，但受闭环动态影响 | 联查单侧压翻风险 | 是 |
| `velocity_zero_hold_entry_delay_sec` | 零速保持 | `0.08` | 按秒定义，采样颗粒度受频率影响 | 联查触发是否变钝 | 否 |
| `command_timeout_sec` | 指令超时 | `0.2` | 按秒定义，采样颗粒度受频率影响 | 联查误判超时与恢复节奏 | 否 |
| `yaw_rate_assist_scale` | 转向辅助映射 | `0.0` | 不直接由 `dt` 决定 | 低优先确认不引入额外输入 | 是 |

补充说明：

- 这里的“是否支持运行时更新”按当前代码现状记录。
- `turn_hip_feedforward_scale`、`velocity_ref_*`、`yaw_rate_ref_*`、`leg_length_ref_lpf_rc`、`yaw_rate_assist_scale` 当前支持运行时更新。
- `leg_length_pid.*`、`steer_pid.*`、`anti_crash_pid.*`、`roll_balance_pid.*` 当前仍按“改后重启 controller 才真正生效”处理。

## 10. 外部接口、类型与行为边界

本阶段只写文档，不修改这些接口；但后续 `100 Hz` 实施会直接影响它们的行为口径。

### 10.1 `/robot_state`

- 当前约 `500 Hz` 驱动 controller
- 后续要支持 `100 Hz` 主线
- controller 仍按消息时间戳计算 `dt`

### 10.2 `/wheel_leg_controller`

- 当前按消息时间戳计算 `dt`
- 后续会放宽 `dt` 接受范围
- 当前异常日志里的“expected about `0.002 s`”后续也应与新主线一致

### 10.3 `wheel_leg_stm32_bridge.publish_rate_hz`

- 当前默认 `500.0`
- 后续硬件阶段目标应切到 `100.0`

### 10.4 `wheel_leg_controller.publish_rate_hz`

- 当前该参数已被忽略
- 不能把修改这个参数误当成“已经完成降控制频率”

### 10.5 PID 参数族

本阶段后续实现最直接会动到的参数族包括：

- `leg_length_pid.*`
- `steer_pid.*`
- `anti_crash_pid.*`
- `roll_balance_pid.*`

### 10.6 LQR 相关边界

- 当前 LQR 计算输入不带 `dt`
- 因此本阶段默认不把 LQR 增益作为首调对象
- 但其目标值与外围叠加环会因降频而改变，后续验证时仍要把 LQR 闭环整体表现纳入观察

## 11. 固定验证矩阵

后续只要开始做 `100 Hz` 实现和调参，就按下面这套验证矩阵执行。

### 11.1 频率与 `dt` 验证

验证项：

- `/robot_state` 实际约 `100 Hz`
- controller 日志中的 `dt` 连续样本约 `0.010000000 s`
- 不再出现 `out-of-range dt` 告警

这一步没过，不进入控制参数调节。

### 11.2 静态站立验证

动作：

- `stand` 模式空杆保持 `10 s`

观察重点：

- 是否出现腿长振荡
- 是否出现 roll 左右摆动
- 是否出现原地缓慢漂移

优先看这些话题：

- `/debug/plot/leg_length_output`
- `/debug/plot/anti_crash`
- `/debug/plot/roll_balance`

### 11.3 小动作验证

动作顺序：

1. 小幅高度变化
2. 小幅左右转向
3. 小幅前后速度命令

执行规则：

- 每次只改一组强相关参数
- 先小动作，再中等动作
- 不直接上大杆量

### 11.4 再次失衡时的定位顺序

如果再次出现“仿真直接失衡”，优先按下面顺序排查：

1. `dt` 守门是否还在丢样
2. `roll_balance_pid.kd / derivative_lpf_rc`
3. `anti_crash_pid.kd / ki`
4. `leg_length_pid.kd / ki`
5. `steer_pid.ki`
6. `turn_hip_feedforward_scale`
7. `ref_filter` 与 `zero_hold` 逻辑是否引入慢相位

### 11.5 回归通过标准

满足以下条件，可认为 `100 Hz` 主线具备继续推进价值：

- `stand` 模式稳定
- 左右转不出现明显单侧压翻
- 小幅速度命令可收回，不出现持续漂移
- 从 `500 Hz` 仿真基线切到 `100 Hz` 后，没有“输入被丢样”与“直接失衡”这两类基础故障

## 12. 当前默认结论与边界

- 本阶段默认目标是“ROS 控制链与状态通信切到 `100 Hz`”，不是修改 MuJoCo 物理积分步长。
- 如果后续为了实验方便保留 MuJoCo `500 Hz` 物理步长，那也应明确表述为“状态降采样 + 控制降频”。
- 本阶段只写专项文档，不改现有 `control_tuning.md`。
- 本阶段文档粒度固定为“决策级方案”，不给具体新 PID 数值，只给参数优先级、机理和验证顺序。
- 本阶段默认 LQR 不是首调对象，但不会写成“LQR 链路完全不用看”。

## 13. 后续实现前的最小行动清单

真正开始做 `100 Hz` 实现前，建议先按下面顺序推进：

1. 放宽 controller `dt` 接受范围并同步修正日志口径
2. 准备 `100 Hz` 状态源
3. 先做 `stand` 模式静态验证
4. 再按优先级调 `leg_length_pid`、`roll_balance_pid`、`anti_crash_pid`
5. 再进入转向和参考滤波联调

## 14. 相关现状参考

当前专项文档与以下现状保持一致：

- controller 当前由 `/robot_state` 驱动，不再使用内部 `publish_rate_hz` 控制步频
- `wheel_leg_stm32_bridge.publish_rate_hz` 当前默认 `500.0`
- 当前 PID 默认值沿用正式 `stand_runtime_defaults.hpp`
- 已有历史验证表明：
  - `500 Hz` 主线稳定
  - `100 Hz` 降采样 takeover 在当前参数下会失稳
