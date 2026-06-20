# 轮腿控制算法移植说明

本文说明 `simulate/tools` 中 `sensor`、`lqr_k`、`vmc` 三部分的意义，以及如何把当前仿真算法按已有嵌入式 C++ 工程的风格移植到板端。移植时不要机械照搬仿真目录结构，应优先融入原工程已有的 `controller`、`estimator`、`motor`、`config` 等模块组织方式。

## 1. 控制链路总览

当前控制链路可以理解为：

```text
IMU / 编码器 / 轮速或里程计
        |
        v
sensor: 组装 RobotSensorData
        |
        v
lqr_k: 根据腿长 L0 计算 LQR 增益，并输出轮力矩、髋虚拟力矩
        |
        v
腿长 PID / 转向 PID / 防劈叉修正
        |
        v
vmc: 将腿长方向力 + 髋虚拟力矩映射为髋、膝关节力矩
        |
        v
电机层: 方向、减速比、限幅、CAN/驱动协议
```

板端移植时，`lqr_k` 和 `vmc` 基本属于纯数学模块，可以保留核心公式；`sensor` 中 MuJoCo 读取接口不能直接移植，需要替换成实机 IMU、编码器和电机反馈接口。

## 2. 各模块意义

### 2.1 sensor

相关文件：`sensor.h`、`sensor.cc`。

`sensor` 的作用是把底盘和左右腿的原始传感器整理成控制器需要的统一状态：

- `BaseLinkState`：底盘前进距离、前进速度、roll/pitch/yaw、角速度和加速度。
- `JointState`：单个关节的位置和速度。
- `LegKinematics`：腿部绝对角、腿长 `leg_length`、腿角 `phi` 和 `phi_rate`。
- `LegState`：单条腿的髋、膝、轮、小腿关节状态和运动学结果。
- `RobotSensorData`：整车控制周期内使用的一帧状态。

仿真版 `AssembleSensorData(const mjModel* m, const mjData* d)` 做了几件事：

- 从 MuJoCo sensor 中读取 `base_quat`、`base_gyro`、`base_accel`。
- 将四元数转换成欧拉角，注意 MuJoCo `framequat` 顺序是 `w, x, y, z`。
- 根据 yaw 将世界系速度投影到底盘前进方向，并积分得到 `distance`。
- 读取左右腿髋、膝、轮、小腿编码器。
- 加入左右腿零位偏置，得到绝对关节角。
- 根据连杆长度计算腿长 `L0` 和腿角 `phi`。
- 通过差分和低通滤波得到 `phi_rate`。

移植到实机时，应保留的是数据结构、单位约定、角度转换、腿部运动学和 `phi_rate` 滤波思路；需要替换的是所有 MuJoCo 类型、`mj_name2id`、`d->sensordata`、仿真打印函数。

### 2.2 lqr_k

相关文件：`lqr_k.h`、`lqr_k.cc`。

`lqr_k` 是腿长相关 LQR 控制器。`LqrK(double leg_length)` 根据当前腿长 `L0` 计算一个 `2 x 6` 增益矩阵，公式来自 `simulate/matlab_function/LQR_K.m`。

LQR 状态向量顺序固定为：

```text
[phi, phi_rate, distance, velocity, pitch, pitch_rate]
```

`CalcLqrTorque(double leg_length, const LqrVector& target, const LqrVector& states)` 计算：

```text
output = -K * (states - target)
```

输出含义：

- `wheel_torque`：轮电机力矩，用于平衡、速度和位置控制。
- `hip_torque`：髋部虚拟力矩，后续会参与 VMC 映射到髋、膝关节。
- `torque_magnitude`：轮力矩和髋虚拟力矩的合成大小，当前主要用于调试或保护扩展。
- `fly_flag`：当前固定为 `false`，后续可扩展为离地保护标志。

板端移植时要特别保证状态顺序不能改。一旦顺序错位，算法仍会输出数值，但物理意义会完全错误。

### 2.3 vmc

相关文件：`vmc.h`、`vmc.cc`。

`vmc` 是虚拟模型控制映射。`SerialVMC(force, torque, leg_length, phi, theta1, theta2)` 将腿端虚拟量映射为串联腿两个关节力矩。

输入含义：

- `force`：腿长方向虚拟力，单位 N。
- `torque`：髋部虚拟力矩，单位 Nm。
- `leg_length`：当前腿长 `L0`，单位 m。
- `phi`：腿角，单位 rad。
- `theta1`：髋关节绝对角，单位 rad。
- `theta2`：小腿/等效第二连杆角，当前调用中使用 `calf_absolute`。

输出含义：

- `joint1_torque`：第一个关节力矩，当前用于髋电机。
- `joint2_torque`：第二个关节力矩，当前用于膝电机。

该公式来自 `simulate/matlab_function/VMC.m`。它内部包含 `1.0 / leg_length`，所以实机必须保证 `leg_length` 不为 0，并限制在机械允许和 LQR 标定有效范围内。

## 3. 按已有 C++ 工程风格移植

移植者应先阅读原工程，而不是先复制当前仿真文件。建议按下面顺序对齐：

1. 找到原工程控制周期入口，例如 `ControlUpdate()`、`Robot::Update()`、定时器回调或 RTOS task。
2. 找到传感器抽象层，确认 IMU、编码器、轮速、电机反馈当前怎样命名、怎样缓存、怎样更新时间戳。
3. 找到电机输出层，确认输出单位是电流、力矩、期望速度还是驱动器私有命令。
4. 找到参数管理方式，确认零位偏置、PID、限幅、连杆长度应该放在配置文件、参数表、宏定义还是类成员中。
5. 找到工程命名风格，是类风格、命名空间风格、C 风格 `Init/Update/Reset`，还是混合风格。

根据原工程风格选择封装方式：

- 如果原工程使用类：建议把 LQR、VMC、腿部运动学封到控制器类或子模块类里，例如 `WheelLegController::Update()`、`LegKinematics::Update()`。
- 如果原工程使用 C 风格模块：建议提供 `Init`、`Reset`、`Update` 三类接口，并把历史状态放入显式结构体。
- 如果原工程已经有数学工具库：角度归一化、弧度角度转换、低通滤波应复用原工程工具，不重复造接口。
- 如果原工程已有参数系统：零位偏置、杆长、PID、力矩限幅、腿长上下限必须进入参数系统，不要散落成魔法数字。

不要强行保留 `simulate/tools` 的文件名和目录结构。更重要的是保持算法输入输出一致，并符合原工程的代码组织和审查习惯。

## 4. 推荐接口约定

所有输入输出统一使用 SI 单位：

| 物理量 | 单位 |
| --- | --- |
| 角度 | rad |
| 角速度 | rad/s |
| 长度 | m |
| 速度 | m/s |
| 力 | N |
| 力矩 | Nm |

板端每个控制周期至少需要提供：

- IMU 姿态：roll、pitch、yaw，或四元数加统一转换函数。
- IMU 角速度：roll_rate、pitch_rate、yaw_rate。
- IMU 加速度：accel_x、accel_y、accel_z，用于状态估计或保护扩展。
- 左右髋、膝、轮编码器位置和速度。
- 前进距离和速度，可来自轮速积分、里程计、融合估计或上层状态估计器。

控制器输出建议保持为物理力矩：

- `left_hip_torque`
- `left_knee_torque`
- `left_wheel_torque`
- `right_hip_torque`
- `right_knee_torque`
- `right_wheel_torque`

方向修正、减速比、电流换算、驱动器限幅、CAN/串口协议应交给原工程电机层处理。这样算法层只关心物理控制量，便于和仿真输出对比。

## 5. 关键移植步骤

### 5.1 抽离纯数学部分

`lqr_k` 和 `vmc` 不依赖 MuJoCo，可以优先移植：

- 保留 `LqrK()` 的多项式系数。
- 保留 `CalcLqrTorque()` 的 `-K * (states - target)` 计算方向。
- 保留 `SerialVMC()` 的公式和输出顺序。
- 根据原工程风格决定是否继续使用 `std::array`，或改成原工程已有的向量/矩阵类型。

### 5.2 重写传感器适配层

不要移植 `ReadScalarSensor()`、`ReadQuaternionSensor()`、`ReadVectorSensor()`。实机版应从原工程传感器层读取：

- IMU 姿态和角速度。
- 髋、膝、轮编码器位置和速度。
- 轮速或估计器输出的底盘速度。

然后生成控制器需要的统一状态。实机版可以继续使用 `RobotSensorData` 这一类结构，也可以改成原工程已有状态结构，但字段含义和单位必须一致。

### 5.3 标定零位和方向

当前仿真中使用了左右腿零位偏置：

```text
left_hip_offset  = 143.944 deg
right_hip_offset = 145.56 deg
left_knee_offset = 26.04 deg
right_knee_offset = 33.843 deg
```

这些值不应直接当作实机最终值。实机要重新标定：

- 编码器零位。
- 髋、膝、轮正方向。
- 左右腿镜像关系。
- IMU 坐标系和机体坐标系关系。
- 电机正力矩方向。

标定完成后，再把偏置和方向系数写入原工程参数系统。

### 5.4 管理历史状态

仿真版 `AssembleSensorData()` 使用函数内 `static` 保存：

- 左右腿上一周期 `phi`。
- 左右腿滤波后的 `phi_rate`。
- 是否已有上一帧数据。
- 底盘前进距离积分。

实机不建议使用不可控的函数内 `static`。这些状态应放到控制器状态结构或类成员中，并提供：

- `Init()`：上电初始化。
- `Reset()`：急停、跌倒恢复、模式切换时清空历史状态。
- `Update(dt)`：每个控制周期显式传入 `dt`。

这样可以避免重启控制器、切模式或传感器异常后历史值污染下一次控制。

### 5.5 加入保护

板端必须在输出电机命令前加入保护：

- `dt <= 0` 或 `dt` 过大时跳过微分更新或进入安全输出。
- `leg_length` 小于下限或大于上限时限幅或停控。
- 所有关键输入输出做 `NaN/Inf` 检查。
- LQR 输出、VMC 输出、最终电机输出都做限幅。
- IMU 或编码器通信异常时进入安全状态。
- 急停、跌倒检测、离地检测应优先于正常控制输出。

## 6. 风险和注意点

- MuJoCo 四元数顺序是 `w, x, y, z`，实机 IMU 可能是 `x, y, z, w`，必须确认后再转换。
- 欧拉角正方向和旋转顺序必须和仿真一致，尤其是 pitch 和 yaw_rate。
- LQR 状态向量顺序必须固定为 `[phi, phi_rate, distance, velocity, pitch, pitch_rate]`。
- `SerialVMC()` 输入角度必须和腿部运动学中的绝对角定义一致。
- 左右腿镜像可能导致某些编码器、电机力矩方向相反，不能只看数值大小。
- `leg_length` 应限制在机械实际可达范围和 LQR 拟合有效范围内。
- `double` 默认更接近当前仿真结果；若目标板浮点性能不足，可以评估改为 `float`，但必须重新做输出对比。
- 调试初期先低力矩、悬空或支架测试，再逐步上地面闭环。

## 7. 对比测试清单

移植完成后，至少做以下测试：

1. 固定一组 `leg_length`，对比板端 `LqrK()` 输出和 PC 仿真输出。
2. 固定 `target` 和 `states`，对比 `CalcLqrTorque()` 的 `wheel_torque`、`hip_torque`。
3. 固定 `force`、`torque`、`leg_length`、`phi`、`theta1`、`theta2`，对比 `SerialVMC()` 的髋、膝输出。
4. 实机静止时检查 roll、pitch、yaw、关节角、腿长、`phi`、`phi_rate` 是否稳定合理。
5. 单独给正轮力矩、正髋力矩、正膝力矩，确认运动方向和仿真预期一致。
6. 模拟传感器断开、`dt` 异常、腿长越界、力矩饱和，确认控制器进入安全输出。

## 8. 给移植者或 AI 的提示词模板

如果让另一个工程师或 AI 根据已有工程风格移植，可以给它下面这段要求：

```text
请先阅读我的现有嵌入式 C++ 工程，识别控制周期入口、传感器接口、电机输出接口、参数系统和代码命名风格。然后把 wheel_leg 仿真工程中的 lqr_k、vmc 和 sensor 思路移植进去。

要求：
1. 不要机械照搬 simulate/tools 的目录结构，优先符合现有工程风格。
2. lqr_k 和 vmc 只保留纯数学核心，去掉 MuJoCo、打印和仿真依赖。
3. sensor 不移植 MuJoCo 读取函数，只移植状态含义、单位、腿部运动学、phi_rate 滤波和历史状态 reset 机制。
4. 所有输入输出使用 SI 单位：rad、rad/s、m、m/s、N、Nm。
5. LQR 状态顺序必须是 [phi, phi_rate, distance, velocity, pitch, pitch_rate]。
6. 输出保持物理力矩，由现有电机层处理方向、减速比、限幅和通信协议。
7. 零位偏置、连杆长度、PID、力矩限幅、腿长范围进入现有参数系统。
8. 增加 dt 异常、腿长越界、NaN/Inf、传感器掉线、急停和输出限幅保护。
9. 移植后用固定输入对比 PC 仿真输出，再做低力矩实机方向测试。
```

