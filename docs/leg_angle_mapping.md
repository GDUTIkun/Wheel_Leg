# 腿部电机角度映射记录

## 1. 标定输入

`leg_data.txt` 记录了一组实机姿态标定数据，单位为 `deg`：

```text
l_hip_angle  = -55.4412
r_hip_angle  =  63.052
world angle  = 190.81

l_knee_angle = -4.631
r_knee_angle =  4.678
link angle   =  63.86
```

原始标定含义：

- 左右髋关节电机读数在安装方向上相反，但该姿态下都应映射到世界坐标系中相对于水平线的 `190.81 deg`。
- 左右膝关节电机读数也在安装方向上相反，该姿态下对应大腿与膝关节电机连接摆杆的夹角幅值为 `63.86 deg`。按当前 knee 相对角和限位方向，固件中该相对角记为 `-63.86 deg`。

后续右髋现场补充观测：

```text
当前 rhip 输出：水平向前约 20 deg，竖直向下约 290 deg，水平向后约 200 deg
目标 rhip 输出：水平向前为 0 deg，竖直向下为 90 deg，水平向后为 180 deg
```

因此右髋需要从旧输出口径转换为：

```text
rhip_target = normalize_0_360(20 - rhip_old_output)
```

后续左髋现场补充观测：

```text
第一轮当前 lhip 输出：竖直向下约 204 deg，水平向后约 284 deg
第一轮目标 lhip 输出：竖直向下为 90 deg，水平向后为 180 deg
```

第一轮只修偏置后，现场反馈变成：

```text
当前 lhip 输出：竖直向下约 180 deg，水平向后约 90 deg
目标 lhip 输出：竖直向下为 90 deg，水平向后为 180 deg
```

这说明左髋方向也需要反过来，当前采用：

```text
lhip_target = normalize_0_360(270 - lhip_old_output)
```

## 2. 当前固件映射

映射位置：

```text
firmware/stm32/Hardware/dvc_motor_dji.cpp
```

当前世界角约定：

```text
右侧水平轴为 0 deg
顺时针旋转为正角度
第四象限竖直向下为 90 deg
第一象限竖直向上为 270 deg
角度范围按 0 ~ 360 deg 表示
```

`Class_Motor_DJI_GIM6010::Data_Process()` 先解析 GIM6010 原始电机角度，再按 CAN ID 做符号和零点映射。映射后，`Get_Now_Angle()` 返回的是机构角，单位仍为 `rad`；`Get_Now_Omega()` 的符号也按同一方向同步映射。

膝关节电机安装在 `hip_link` 上，因此膝关节输出角不是单独的膝电机相对角，而是：

```text
knee_world = hip_world + knee_relative + knee_offset
```

对应角速度也按同样链路叠加：

```text
knee_world_omega = hip_world_omega + knee_relative_omega
```

当前 CAN ID 对应关系：

```text
CAN_Motor_ID_0x4E: left hip
CAN_Motor_ID_0x2E: right hip
CAN_Motor_ID_0x6E: left knee
CAN_Motor_ID_0x8E: right knee
```

映射公式，角度单位为 `deg`：

```text
left_hip_world  = normalize_0_360(-left_hip_motor + 135.3688)
right_hip_world = normalize_0_360(right_hip_motor + 127.758)

left_knee_world  = normalize_0_360(left_hip_world - left_knee_motor - 68.491)
right_knee_world = normalize_0_360(right_hip_world + right_knee_motor - 68.538)
```

这里的常数就是当前安装状态下的偏置。左右髋、左膝和右膝均已按现场观测修正：

```text
left_hip_offset   = 190.81 + (-55.4412) = 135.3688
right_hip_offset  = 190.81 - 63.052     = 127.758
left_knee_offset  = -63.86 + (-4.631)   = -68.491
right_knee_offset = -63.86 - 4.678      = -68.538
```

代入 `leg_data.txt` 新标定：

```text
left_hip_world  = -(-55.4412) + 135.3688 = 190.81
right_hip_world = 63.052 + 127.758       = 190.81

left_knee_relative  = -(-4.631) - 68.491 = -63.86
right_knee_relative = 4.678 - 68.538     = -63.86

left_knee_world  = 190.81 - 63.86 = 126.95
right_knee_world = 190.81 - 63.86 = 126.95
```

角速度映射公式，单位为 `deg/s`：

```text
left_hip_world_omega  = -left_hip_motor_omega
right_hip_world_omega =  right_hip_motor_omega

left_knee_world_omega  = left_hip_world_omega - left_knee_motor_omega
right_knee_world_omega = right_hip_world_omega + right_knee_motor_omega
```

单关节悬空检查极性时，如果对应髋关节保持不动，正电机角速度应得到：

```text
left_hip_motor_omega  > 0 -> left_hip_omega  < 0
right_hip_motor_omega > 0 -> right_hip_omega > 0
left_knee_motor_omega > 0 -> left_knee_omega < 0
right_knee_motor_omega> 0 -> right_knee_omega> 0
```

2026-06-25 实机确认：

```text
四个关节的角速度极性正确。
knee 输出角速度已经正确叠加对应 hip 的世界角速度。
```

左膝现场修正：

```text
竖直向下时 lknee 输出 200 多 deg，说明原正偏置过大，应改为负偏置。
向前转 knee 时角度增大，但目标世界角应减小，说明左膝相对角极性应反号。
```

右膝现场修正：

```text
初始 rknee 叠加后约 215 deg，说明原偏置方向过大，应改为负偏置。
向第一象限转 knee 时角度增大，说明右膝相对角极性应保持正号。
```

代入左髋现场观测：

```text
old 180 -> 270 - 180 = 90
old 90  -> 270 - 90  = 180
```

代入右髋现场观测：

```text
old 20  -> 20 - 20  = 0
old 290 -> 20 - 290 = -270 -> 90
old 200 -> 20 - 200 = -180 -> 180
```

## 3. 机械活动范围

当前记录的活动范围，角度单位为 `deg`：

```text
hip_world_range = 75 ~ 200
```

`hip_world_range` 使用当前世界角约定，即水平向前为 `0 deg`、竖直向下为 `90 deg`、水平向后为 `180 deg`。

`knee` 的机械范围按自身相对角记录，不包含 `hip` 转动叠加后的世界角。当前实机限位测试范围改为：

```text
knee_relative_range = -140 ~ -70
```

后续做软件限位时，`hip` 应直接限制映射后的世界角；`knee` 应先使用不叠加 `hip` 的自身相对角做限位，再叠加到世界角用于状态输出。

当前固件测试变量：

```text
l_knee_relative_angle = normalize_180(l_knee_angle - l_hip_angle)
r_knee_relative_angle = normalize_180(r_knee_angle - r_hip_angle)
knee_limit_flag       = 1 if either knee_relative_angle reaches -140 or -70
```

## 4. 使用约定

- 固件内部仍使用 `rad`。
- `Car.cpp` 中用 `Basic_Math_Rad_To_Deg(Get_Now_Angle())` 打印时，髋关节读数应直接表示上述顺时针世界角。
- `Car.cpp` 中新增 `l_hip_omega`、`r_hip_omega`、`l_knee_omega`、`r_knee_omega`，单位为 `deg/s`，用于现场直接观察角速度极性。
- `Get_Now_Omega()` 在 GIM6010 上已经返回映射后的机构角速度；`Car.cpp` 只是把 `rad/s` 转成 `deg/s` 便于调试查看。
- 左髋和左膝的相对角速度会随角度映射一起反号；右髋和右膝按现场修正后不再反号，保证角度和角速度处于同一正方向。
- 膝关节当前输出已经叠加对应侧髋关节世界角，表示膝关节连接摆杆在世界坐标系下的绝对角度；膝关节角速度也已实机确认叠加对应侧髋关节世界角速度。
- 后续如果重新装配电机、调整零点或更换 CAN ID，需要重新采一组标定数据并更新上述偏置。
