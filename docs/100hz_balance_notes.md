# `kActuatorOutputLPFRC = 0.02`轮腿平衡控制从 500Hz 调整到 100Hz 的修改记录

本文档记录本工程从 500Hz 控制周期切换到 100Hz 控制周期后，为了让轮腿机器人重新保持站立平衡所做的关键修改。

最终目标不是锁住浮动基座自由度，而是在保持 `freejoint` 的前提下，让控制器、传感器估计、滤波器、LQR 增益和 MuJoCo 仿真步长都匹配 `0.01s` 的控制周期。

## 1. 背景

原始控制频率：

```text
500Hz, dt = 0.002s
```

目标控制频率：

```text
100Hz, dt = 0.01s
```

控制周期从 `0.002s` 变成 `0.01s` 后，采样周期变为原来的 5 倍。这个变化会直接影响：

- PID 的积分和微分计算
- 速度估计
- 低通滤波器截止频率
- LQR 离散化模型
- pitch / pitch_rate 反馈的相位裕度
- MuJoCo 接触求解稳定性
- 执行器输出变化速度

因此，不能只把主循环改成每 `0.01s` 执行一次。所有和时间有关的控制器、滤波器、传感器估计都需要一起修改。

## 2. 主要失稳现象

降到 100Hz 后出现过几类典型失稳现象：

1. 一落地直接发散。
2. 站立约 `0.5s` 后开始剧烈震荡。
3. 下盘能稳住，但浮动基座出现高频 pitch 抖动，然后发散。
4. 加入转向环、抗劈叉环后重新失衡。
5. 删除附加环后，单独 LQR + VMC 仍然可能因为 pitch 反馈过激而震荡。

最终判断：主要失稳来源不是自由度被放开本身，而是 100Hz 下 pitch / pitch_rate / velocity 等反馈项的离散延迟、速度噪声和滤波相位滞后共同作用，使 LQR 对浮动基座施加了过激力矩。

## 3. 控制周期修改

相关文件：

```text
simulate/wheel_leg/wheel_leg_hooks.cc
```

当前控制周期由 MuJoCo timestep 和控制降采样共同决定：

```cpp
constexpr int kControlDecimation = 1;
```

控制器实际 dt：

```cpp
const float control_dt =
    static_cast<float>(m->opt.timestep * kControlDecimation);
```

因为当前 MuJoCo timestep 已经是 `0.01s`，所以：

```text
control_dt = 0.01 * 1 = 0.01s
```

这表示控制器每个仿真步都运行一次，但由于仿真步长本身就是 `0.01s`，控制频率就是 100Hz。

## 4. PID 时间步长修改

相关文件：

```text
simulate/tools/pid.cc
simulate/tools/pid.h
simulate/wheel_leg/wheel_leg_hooks.cc
```

PID 不能继续使用固定的 `0.002s`。积分项和微分项都必须使用实际控制周期。

当前 PID 默认周期改成：

```cpp
float PIDCalculate(PIDInstance *pid, float measure, float ref) {
    return PIDCalculateWithDt(pid, measure, ref, 0.01f);
}
```

同时新增或使用带 dt 的接口：

```cpp
float PIDCalculateWithDt(PIDInstance *pid, float measure, float ref, float dt);
```

控制器中调用：

```cpp
PIDCalculateWithDt(&leglen_pid_l, left_leg.leg_length,
                   target_leg_length, control_dt);
```

这样积分计算按 `error * dt` 累积，微分计算按 `delta / dt` 计算，不会再隐含 500Hz 的时间尺度。

## 5. 低通滤波器从固定 alpha 改为 RC 形式

相关文件：

```text
simulate/tools/sensor.cc
```

原来类似：

```cpp
constexpr double kPhiRateLowPassAlpha = 0.95;
```

这类固定 alpha 和采样周期强相关。同样的 `alpha = 0.95`，在 `0.002s` 和 `0.01s` 下对应的物理截止频率完全不同。

现在改为 RC 形式：

```cpp
double LowPass(double previous, double current, double rc, double dt) {
  if (dt <= 0.0 || rc <= 0.0) {
    return current;
  }
  const double alpha = rc / (rc + dt);
  return alpha * previous + (1.0 - alpha) * current;
}
```

当前使用的 RC 参数：

```cpp
constexpr double kPhiRateLowPassRC = 0.02333333333333333;
constexpr double kBaseVelocityLowPassRC = 0.03;
constexpr double kGyroLowPassRC = 0.02;
```

在 `dt = 0.01s` 时，对应的 alpha 约为：

```text
phi_rate alpha = 0.02333 / (0.02333 + 0.01) = 0.70
base velocity alpha = 0.03 / (0.03 + 0.01) = 0.75
gyro alpha = 0.02 / (0.02 + 0.01) = 0.667
```

这样做的好处是：之后如果控制周期再次变化，只要继续使用实际 `dt`，滤波器仍然有明确的物理时间常数。

## 6. 传感器速度估计修改

相关文件：

```text
simulate/tools/sensor.cc
simulate/tools/sensor.h
```

100Hz 下速度估计比 500Hz 更敏感。特别是 LQR 状态中的：

- `phi_rate`
- `base_link.velocity`
- `base_link.pitch_rate`

这些量如果噪声大、延迟大或者方向错，会直接通过 LQR 反馈放大成力矩震荡。

这里要区分“角度状态”和“速度状态”。在当前仿真里，`phi` 和 `pitch` 角度本身通常不是最主要的高频噪声来源；真正容易把系统推到震荡的是：

```text
phi_rate
pitch_rate
base_link.velocity
```

也就是说，主要问题不是 LQR 看到了很吵的 `phi` 或 `pitch`，而是 100Hz 下由编码器/几何计算得到的腿角速度、由 gyro 得到的 pitch 角速度、以及基座前向速度更容易带噪声和相位滞后。LQR 对这些速度项的反馈一旦过强，就会把速度噪声放大成轮子和髋关节力矩抖动。

传感器部分是这次能稳住的关键之一。因为 LQR 本身只看状态量，它并不知道状态量是真实运动，还是由采样周期、差分噪声、IMU 噪声、坐标方向错误带来的假信号。500Hz 时这些问题可能被高频采样掩盖；到了 100Hz，每个错误状态会保持 `0.01s`，更容易把基座 pitch 推进震荡。

### 6.1 phi_rate

`phi_rate` 不再依赖简单差分，而是通过腿部几何和关节速度解析计算，再做低通滤波。

滤波函数：

```cpp
FilterPhiRate(&sensor_data.left_leg.kinematics,
              &filtered_left_phi_rate,
              &has_previous_left_phi_rate,
              dt);
```

这里要避免的错误是：

```text
phi_rate = (phi_now - phi_last) / dt
```

这种差分在 100Hz 下会明显放大量化噪声和角度抖动。腿部角度 `phi` 由多个关节角组合得到，任何一个关节角的小抖动都会被 `/ 0.01` 放大成速度抖动，再进入 LQR。

当前做法是先根据腿端几何变化计算角速度：

```text
phi_rate = (x * y_velocity - y * x_velocity) / (x^2 + y^2)
```

这个形式本质上是二维向量角度变化率，比直接对 `phi` 做差分更平滑，也更适合 100Hz。

随后再经过：

```cpp
constexpr double kPhiRateLowPassRC = 0.02333333333333333;
```

在 `dt = 0.01s` 下：

```text
alpha = 0.02333 / (0.02333 + 0.01) ~= 0.70
```

也就是当前速度估计保留约 70% 的上一时刻结果，混入约 30% 的新观测。这样可以压住速度噪声，同时延迟还不至于太大。

### 6.2 base forward velocity

基座前向速度优先读取平面关节速度；如果当前模型使用 `freejoint`，则用浮动基座在世界系下的速度投影到 yaw 方向：

```cpp
const double forward_x = std::cos(yaw);
const double forward_y = std::sin(yaw);
return d->qvel[0] * forward_x + d->qvel[1] * forward_y;
```

然后对前向速度做 RC 低通：

```cpp
filtered_base_forward_velocity =
    LowPass(filtered_base_forward_velocity,
            raw_base_forward_velocity,
            kBaseVelocityLowPassRC,
            dt);
```

距离状态不再从位置传感器直接读，而是用滤波后的速度积分：

```cpp
base_forward_distance += filtered_base_forward_velocity * dt;
```

这里有两个重点。

第一，`freejoint` 模型下基座的 `qvel[0]` 和 `qvel[1]` 是世界系平动速度，不是机器人机体系前向速度。因此要根据 yaw 做投影：

```text
base_forward_velocity = vx_world * cos(yaw) + vy_world * sin(yaw)
```

否则机器人转过一定角度后，LQR 看到的前进速度方向会错，轮子力矩会补偿到错误方向。

第二，距离状态用速度积分而不是直接依赖某个仿真位置关节，是为了让 `distance` 和 `velocity` 来自同一套前向定义。否则可能出现：

```text
distance 是世界 x
velocity 是机体前向
```

这会让 LQR 的位置和速度状态不一致。100Hz 下这种不一致也会表现成慢慢积累的力矩偏置。

当前速度低通：

```cpp
constexpr double kBaseVelocityLowPassRC = 0.03;
```

在 `dt = 0.01s` 下：

```text
alpha = 0.03 / (0.03 + 0.01) = 0.75
```

这比 `phi_rate` 更稳一点，因为 base velocity 直接影响轮子力矩。如果这个量噪声太大，轮子会前后快速抽动，再通过地面接触激发 pitch。

### 6.3 gyro / pitch_rate

`pitch_rate` 来自 `base_gyro[1]`，并经过 RC 低通：

```cpp
filtered_base_gyro[i] =
    LowPass(filtered_base_gyro[i], base_gyro[i], kGyroLowPassRC, dt);
```

这是 100Hz 能稳定的重要点之一。因为 pitch_rate 是 LQR 中最容易激发高频震荡的状态之一。

当前陀螺仪低通：

```cpp
constexpr double kGyroLowPassRC = 0.02;
```

在 `dt = 0.01s` 下：

```text
alpha = 0.02 / (0.02 + 0.01) ~= 0.667
```

`pitch_rate` 的处理比 `base_velocity` 稍微快一些，因为姿态角速度需要及时反映基座倾倒趋势。但它又不能完全不过滤，因为 LQR 对 `pitch_rate` 很敏感，未滤波的 gyro 噪声会直接变成高频轮子力矩和髋关节力矩。

这次最终还在 LQR 里把 `pitch_rate` 反馈缩放到 `0.35`，原因也是一样：100Hz 下 `pitch_rate` 同时存在测量噪声和相位滞后，高增益会让基座 pitch 进入自激振荡。

### 6.4 dt 必须从控制器传入传感器

当前传感器接口支持显式传入 dt：

```cpp
RobotSensorData AssembleSensorData(const mjModel* m, const mjData* d,
                                   double dt);
```

控制器中使用：

```cpp
const RobotSensorData sensor_data = AssembleSensorData(m, d, control_dt);
```

这个细节很重要。滤波器、速度积分、距离积分都必须使用控制器实际周期，而不是假设固定频率。

如果以后改成：

```text
仿真步长 0.002s
控制器每 5 步运行一次
控制周期 0.01s
```

那么传感器给控制器用的数据也应该按 `control_dt = 0.01s` 更新。否则会出现控制器 100Hz，传感器滤波却按 500Hz 的错配。

### 6.5 传感器状态初始化

传感器滤波中使用了静态变量保存上一时刻状态：

```cpp
static bool has_previous_left_phi_rate = false;
static bool has_previous_right_phi_rate = false;
static double filtered_left_phi_rate = 0.0;
static double filtered_right_phi_rate = 0.0;
static bool has_previous_base_velocity = false;
static bool has_previous_gyro = false;
static double filtered_base_forward_velocity = 0.0;
static std::array<double, 3> filtered_base_gyro = {0.0, 0.0, 0.0};
static double base_forward_distance = 0.0;
```

第一次读取时不直接滤波，而是用当前测量初始化：

```text
filtered = raw
has_previous = true
```

这样可以避免启动瞬间从 0 慢慢滤到真实值，导致控制器在前几帧看到假的速度或假的 pitch_rate。

上硬件时也要注意类似问题：IMU 和编码器滤波器最好在控制使能前先初始化到当前测量值，而不是上电默认 0。

### 6.6 传感器方向和单位检查

100Hz 稳定对传感器方向非常敏感，上硬件前必须确认：

```text
pitch 单位: rad
pitch_rate 单位: rad/s
phi 单位: rad
phi_rate 单位: rad/s
base velocity 单位: m/s
distance 单位: m
```

还要确认正方向：

```text
机器人向前倒时，pitch 符号是否和仿真一致
机器人向前倒得更快时，pitch_rate 符号是否和仿真一致
轮子向前滚动时，base velocity 是否为正
腿部 phi 增大方向是否和 LQR_K.m 建模一致
左右腿关节角正方向是否和 VMC 一致
```

如果某个速度方向反了，100Hz 下通常不会表现为温和偏差，而是会直接形成正反馈。例如 pitch_rate 符号反了，LQR 本来想阻尼基座俯仰，结果会变成主动推倒基座。

### 6.7 传感器和 LQR 状态的一一对应

当前 LQR 状态顺序是：

```text
x[0] = phi
x[1] = phi_rate
x[2] = distance
x[3] = velocity
x[4] = pitch
x[5] = pitch_rate
```

传感器输出必须严格对应这个顺序。尤其要避免以下错配：

```text
把 roll_rate 当 pitch_rate
把世界系 x 速度当机体前向速度
把角度单位 deg 传给期望 rad 的 LQR
左右腿 phi 定义不一致
distance 和 velocity 使用不同坐标系
```

这类错误在 500Hz 下可能还能短时间看起来能站住，但在 100Hz 下通常会更快暴露为 pitch 高频震荡。

### 6.8 上硬件时建议的传感器处理顺序

硬件上建议按下面顺序组织传感器数据：

1. 读取原始编码器、IMU、轮速。
2. 做零点校准和方向修正。
3. 统一单位到 rad、rad/s、m/s。
4. 用当前控制周期 `dt = 0.01s` 更新滤波器。
5. 计算腿部几何量 `phi`、`leg_length`。
6. 用关节速度解析计算 `phi_rate`。
7. 将 IMU 世界/机体系数据转换到控制器定义的坐标。
8. 得到 LQR 状态向量。
9. 再调用 LQR 和 VMC。

不要把滤波放在方向修正之前。方向修正、零点修正、单位换算应该先完成，再滤波。否则滤波器内部状态会混入错误坐标系下的历史数据。

### 6.9 传感器部分的最终原则

从 500Hz 到 100Hz，传感器部分的核心原则是：

```text
不用粗糙差分制造速度；
不用固定 alpha 隐含采样周期；
不用世界系速度冒充机体前向速度；
不用未滤波 pitch_rate 直接进 LQR；
不用和控制周期不一致的 dt 更新滤波器。
```

当前稳定版本能够站住，传感器侧主要依赖这几件事：

```text
phi_rate 解析计算 + RC 低通
base velocity 按 yaw 投影 + RC 低通
distance 由滤波后的前向速度积分
gyro / pitch_rate RC 低通
所有滤波器使用 control_dt = 0.01s
传感器状态和 LQR 状态顺序严格对应
```

## 7. MuJoCo timestep 和接触参数修改

相关文件：

```text
model/scence.xml
model/wheel_leg.xml
```

仿真步长改为：

```xml
<option timestep="0.01" gravity="0 0 -9.81" integrator="implicitfast"
        iterations="100" tolerance="1e-8" density="1.225" viscosity="1.8e-5" />
```

地面接触参数调整为偏软：

```xml
friction="0.8 0.02 0.001"
solref="0.04 1"
solimp="0.9 0.95 0.001"
```

默认关节也加入了适度阻尼和转子惯量：

```xml
<joint damping="0.03" armature="0.003"/>
```

这些修改的作用是降低 `0.01s` 大步长下接触瞬间的数值刚性，避免落地时把高频冲击直接注入基座 pitch。

## 8. 保持自由基座，不锁自由度

相关文件：

```text
model/wheel_leg.xml
```

当前基座保持自由浮动：

```xml
<body name="base_body" pos="0 0 0.28">
  <freejoint/>
</body>
```

曾经短暂尝试过限制自由度来判断问题来源，但最终稳定版本没有锁自由度。这样更接近真实硬件，因为真实机器人不会被仿真器额外约束 roll / pitch / yaw 或平移自由度。

需要注意：`freejoint` 下能稳住，才说明控制器本身对浮动基座是有效的；锁自由度得到的稳定结果不能直接代表真实硬件可行。

## 9. LQR 增益重新同步到 100Hz 版本

相关文件：

```text
simulate/matlab_function/LQR_K.m
simulate/tools/lqr_k.cc
```

LQR 增益必须按新的离散周期重新生成。500Hz 下的离散 LQR 直接拿到 100Hz 用，通常会出现相位裕度不足、速度反馈过激、pitch_rate 抖动放大等问题。

当前 `lqr_k.cc` 已按 `LQR_K.m` 的 100Hz 结果同步，并保持 MATLAB reshape 的列主序映射：

```cpp
return {{
    {{mt1, mt3, mt5, mt7, mt9, mt11}},
    {{mt2, mt4, mt6, mt8, mt10, mt12}},
}};
```

其中 LQR 状态顺序是：

```cpp
LqrVector BuildLqrStates(const LegState& leg,
                         const RobotSensorData& sensor_data) {
  return {{
      leg.kinematics.phi,
      leg.kinematics.phi_rate,
      sensor_data.base_link.distance,
      sensor_data.base_link.velocity,
      sensor_data.base_link.pitch,
      sensor_data.base_link.pitch_rate,
  }};
}
```

目标状态：

```cpp
return {{
    targets.target_phi,
    0.0,
    targets.target_distance,
    targets.target_velocity,
    0.0,
    0.0,
}};
```

## 10. 对 pitch 相关 LQR 反馈降幅

相关文件：

```text
simulate/tools/lqr_k.cc
```

最终稳定的关键之一，是降低 100Hz 下最容易激发基座高频震荡的 LQR 反馈项：

```cpp
constexpr double kPhiRateFeedbackScale = 0.6;
constexpr double kVelocityFeedbackScale = 1.0;
constexpr double kPitchFeedbackScale = 0.7;
constexpr double kPitchRateFeedbackScale = 0.35;
```

实际返回增益时：

```cpp
return {{
    {{mt1,
      mt3 * kPhiRateFeedbackScale,
      mt5,
      mt7 * kVelocityFeedbackScale,
      mt9 * kPitchFeedbackScale,
      mt11 * kPitchRateFeedbackScale}},
    {{mt2,
      mt4 * kPhiRateFeedbackScale,
      mt6,
      mt8 * kVelocityFeedbackScale,
      mt10 * kPitchFeedbackScale,
      mt12 * kPitchRateFeedbackScale}},
}};
```

含义：

- `phi_rate` 降到 60%
- `velocity` 保持 100%
- `pitch` 降到 70%
- `pitch_rate` 降到 35%

其中 `pitch_rate` 降幅最大，因为它在 100Hz 下最容易把陀螺仪噪声、滤波延迟和离散误差放大成高频力矩。

这不是额外增加一个姿态环，而是在现有 LQR 内部对敏感状态做保守化处理。

调试过程中曾经把 `velocity` 反馈降到 `0.5`，这样有利于先压住 pitch 高频震荡，但副作用是速度阻尼不足：机器人会先往前跑，距离误差变大后再被拉回，回到原点时速度仍然不为 0，形成低频来回摆。后续测试发现 `kVelocityFeedbackScale = 1.0` 更合适，说明在传感器滤波和 pitch_rate 增益已经压住以后，速度反馈需要恢复足够的阻尼能力。

## 11. 暂时只保留 LQR + VMC

相关文件：

```text
simulate/wheel_leg/wheel_leg_hooks.cc
```

当前站立稳定版本只保留：

- 腿长 PID
- LQR
- VMC 力矩映射
- 执行器输出限幅、低通和斜率限制

暂时不启用：

- 转向环
- 抗劈叉环
- 额外 roll / yaw 弱阻尼环

原因是附加环在 100Hz 下会带来额外相位滞后和耦合。如果基础 LQR + VMC 刚刚稳定，直接叠加转向环或抗劈叉环，很容易重新把系统推到失稳边缘。

建议后续加环顺序：

1. 先确认 LQR + VMC 在不同初始姿态下都能站稳。
2. 再加非常小增益的 yaw / 转向环。
3. 最后再加抗劈叉或左右腿同步环。
4. 每加一个环都记录 pitch、pitch_rate、wheel torque、hip torque。

## 12. 执行器输出平滑

相关文件：

```text
simulate/wheel_leg/wheel_leg_hooks.cc
```

当前执行器输出有力矩限幅、低通和斜率限制：

```cpp
constexpr double kJointTorqueLimit = 120.0;
constexpr double kWheelTorqueLimit = 80.0;
constexpr double kActuatorOutputLPFRC = 0.02;
constexpr double kActuatorOutputRateLimit = 600.0;
```

输出处理：

```cpp
const double alpha =
    kActuatorOutputLPFRC / (kActuatorOutputLPFRC + std::max(dt, 1e-6));
double output = alpha * *filtered_output + (1.0 - alpha) * clipped_output;

const double max_delta = kActuatorOutputRateLimit * std::max(dt, 0.0);
output = std::clamp(output, *filtered_output - max_delta,
                    *filtered_output + max_delta);
```

在 `dt = 0.01s` 下，最大单步变化量：

```text
600 * 0.01 = 6 N*m per step
```

这可以避免 LQR 输出突变直接打到关节和轮子上，降低 pitch 高频震荡风险。

## 13. 为什么 500Hz 稳，100Hz 初始不稳

500Hz 下：

- 采样周期短
- 速度估计延迟小
- 微分项更新快
- LQR 离散近似更接近连续系统
- 同样的力矩变化在每个周期内作用时间短

100Hz 下：

- 每次控制输出保持 `0.01s`
- 速度和角速度估计更粗
- 滤波器相位滞后更明显
- pitch_rate 噪声更容易被 LQR 放大
- 接触冲击在大步长下更容易激发数值震荡
- 原本在 500Hz 下还能接受的高增益，在 100Hz 下可能变成过激反馈

因此，100Hz 稳定不是简单降低频率，而是需要降低高敏感反馈项、重新配置滤波器，并让传感器估计和控制 dt 一致。

## 14. 与真实硬件的对应关系

当前稳定版本更接近真实硬件的地方：

- 保留 `freejoint`，没有用仿真约束强行稳定基座。
- 控制周期使用 `0.01s`，对应硬件 100Hz 控制循环。
- PID、滤波器、速度积分都显式使用实际 dt。
- 执行器输出有斜率限制，接近真实电机无法无限快改变力矩的事实。
- pitch_rate 经过滤波，接近真实 IMU 需要滤波后使用的情况。

仍然需要注意的差异：

- MuJoCo 中的接触参数和真实轮胎/地面不完全一致。
- 仿真里电机力矩响应仍然比真实硬件理想。
- 真实 IMU 会有零偏、延迟、安装误差和噪声。
- 真实编码器速度估计可能比仿真更脏。
- 电机驱动器本身可能还有电流环带宽、限流、死区和通信延迟。

因此，上硬件时建议先保持当前保守增益，不要一开始就恢复高 pitch_rate 增益。

## 15. 上硬件前建议检查

上硬件前建议重点确认：

1. 控制循环实际周期是否稳定在 `0.01s`。
2. IMU 的 pitch 正方向是否和仿真一致。
3. gyro pitch_rate 正方向是否和 LQR 状态定义一致。
4. 轮子速度正方向是否和 `base_link.velocity` 定义一致。
5. 髋关节、膝关节角度零点是否和仿真 offset 对齐。
6. VMC 输出到电机的力矩方向是否正确。
7. 电机最大力矩是否低于仿真中的限幅。
8. 电机力矩斜率是否不要比仿真中 `600 N*m/s` 更激进。
9. 初次上电时腿长目标和实际腿长不要差太多。
10. 先悬空验证关节方向，再轻触地验证支撑力方向，最后再尝试站立。

## 16. 调试时建议记录的变量

如果后续再次出现 pitch 震荡，建议优先打印或记录：

```text
time
pitch
pitch_rate
phi_left
phi_rate_left
phi_right
phi_rate_right
base_velocity
base_distance
left_wheel_torque
right_wheel_torque
left_hip_torque
right_hip_torque
left_leg_length
right_leg_length
```

判断方法：

- 如果 `pitch_rate` 先出现尖峰，再出现大力矩，通常是 gyro 噪声或 pitch_rate 增益过高。
- 如果 `base_velocity` 符号和机器人实际运动方向相反，可能是速度方向定义错。
- 如果 `wheel_torque` 在正负限幅间跳变，通常是 LQR 速度或 pitch_rate 反馈过激。
- 如果 `hip_torque` 先发散，可能是 VMC 力矩映射、腿角方向或 pitch 反馈耦合问题。
- 如果落地瞬间直接炸，优先看接触参数、腿长支撑力和执行器斜率限制。

## 17. 当前稳定版本总结

当前能在 100Hz 保持站立平衡的关键组合是：

```text
MuJoCo timestep = 0.01s
Control decimation = 1
Control dt = 0.01s
Base joint = freejoint
Controller = leg length PID + LQR + VMC
Sensor filter = RC filter with actual dt
LQR gain = 100Hz version from LQR_K.m
phi_rate / pitch / pitch_rate feedback = conservative scaling
velocity feedback = 1.0 after stability is recovered
Actuator output = torque limit + LPF + rate limit
Extra yaw / anti-split loops = disabled
```

一句话总结：

```text
从 500Hz 改到 100Hz 后，稳定性的关键不是锁自由度，而是让控制器、传感器速度估计、滤波器、LQR 增益和执行器输出都按 0.01s 的真实控制周期重新匹配，并降低 pitch_rate 这类高敏感反馈项。
```

## 18. 阶段冻结状态

当前 100Hz 控制频率下的仿真站立平衡已经可以稳住，本阶段先作为仿真可运行基线冻结。

冻结口径：

- 当前目标不是继续在 MuJoCo 中把所有参数调到最终形态。
- 当前参数只能证明 100Hz 控制链在仿真中具备基础稳定性。
- 真实硬件接入后，IMU 零偏、编码器速度噪声、电机响应、通信延迟、地面接触和结构装配误差都会改变最优参数。
- 后续参数调整应以硬件接入后的实测日志为准，尤其是 `pitch_rate`、`phi_rate`、`base_velocity`、轮端力矩和髋关节力矩。

因此，本阶段暂不继续扩大 100Hz 仿真调参范围。下一阶段转入 STM32 / 实机硬件接入，先打通真实状态上报、命令下发、安全停机和方向/单位校验，再在硬件闭环上重新小步调
