# 模块任务：IMU 状态发布

## 1. 模块名称

IMU 状态发布模块。

## 2. 当前迭代目标

从 MuJoCo 读取机体姿态、角速度和线加速度，并通过 ROS2 标准 topic `/imu` 发布 `sensor_msgs/msg/Imu`。

## 3. 任务 checklist

- `[x]` 阅读 `doc/detail.md` 中 IMU 状态发布模块设计。
- `[x]` 确认 MuJoCo IMU sensor 名称：`base_quat`、`base_gyro`、`base_accel`。
- `[x]` 确认 `/imu` 的 frame id：`base_link`。
- `[x]` 确认 MuJoCo 到 ROS2 的坐标系转换约定：当前不做轴重映射，按 `base_link` 机体系直接发布。
- `[x]` 确认四元数顺序转换策略：MuJoCo `w,x,y,z` 转 ROS `x,y,z,w`。
- `[x]` 确认 covariance 是否填充：当前 covariance 全 0，表示未知。
- `[x]` 确认状态发布频率：`100 Hz`。
- `[x]` 确认时间戳策略：使用 MuJoCo 仿真时间 `d->time`。
- `[x]` 从 MuJoCo 读取姿态。
- `[x]` 从 MuJoCo 读取角速度。
- `[x]` 从 MuJoCo 读取线加速度。
- `[x]` 组装 `sensor_msgs/msg/Imu`。
- `[x]` 发布 `/imu`。
- `[v]` 完成后执行仿真验证，ROS2 CLI 可 echo 到 `/imu`。

## 4. 每个任务的输入

- MuJoCo 模型对象。
- MuJoCo 数据对象。
- MuJoCo IMU sensor 数据：`base_quat`、`base_gyro`、`base_accel`。
- 当前仿真时间。
- frame id：`base_link`。
- 坐标系约定：当前不做轴重映射，按 `base_link` 机体系直接发布。
- 四元数顺序：MuJoCo `w,x,y,z` 转 ROS `x,y,z,w`。
- covariance 策略：当前 covariance 全 0，表示未知。
- 发布频率：`100 Hz`。
- 时间戳策略：使用 MuJoCo 仿真时间 `d->time`。

## 5. 每个任务的输出

- Topic：`/imu`
- Message：`sensor_msgs/msg/Imu`
- 字段语义：姿态、角速度、线加速度。

## 6. 完成标准

- ROS2 可以订阅 `/imu`。
- 姿态、角速度和线加速度字段存在。
- MuJoCo 姿态变化时 `/imu` 数据随之变化。
- `header.frame_id` 为 `base_link`。
- 四元数按 ROS `x,y,z,w` 顺序发布。
- 缺失 IMU sensor 时有明确错误提示。

## 7. 验证方法

仿真验证：

- 启动 MuJoCo 仿真和 ROS2 桥接。
- 执行 `ros2 topic echo /imu`。
- 观察姿态或角速度字段是否随仿真变化。
- 检查 frame id 是否稳定。

实机验证：

- 本模块不进行实机验证。
- 实机 `/imu` 留到 STM32 通信层迭代。

## 8. 当前状态

`[v] 已通过最小仿真烟测`

当前待确认：

- 暂无。当前 IMU 坐标语义已按模型挂载关系和运行时消息表现完成验证。

## 9. 本次验证记录

记录时间：2026-06-20。

- 构建命令：`source /opt/ros/jazzy/setup.bash && source install/local_setup.bash && cmake --build build/wheel_leg_simulate_ros2 -j2`。
- 仿真命令：`timeout -k 2s 12s build/wheel_leg_simulate_ros2/wheel_leg_simulate transplant/mujoco_win/model/scence.xml`。
- 验证命令：`ros2 topic echo --once /imu`。
- 观察结果：消息包含 `orientation`、`angular_velocity`、`linear_acceleration`，`header.frame_id` 为 `base_link`，covariance 为全 0。
- 频率验证：修正仿真时间发布调度后，`ros2 topic hz /imu` 统计约 `100 Hz`。
- 修正说明：`/imu` 与 `/joint_states` 共用 bridge 发布调度；初始并行频率统计约 `83.3 Hz`，改为 `next_publish_time` 调度后恢复为约 `100 Hz`。
- 坐标语义验证：`base_frame` site 直接挂在 `base_body` 上且无额外旋转；`base_accel`、`base_gyro`、`base_quat` 都直接以该 site 为参考。bridge 只做四元数 `w,x,y,z -> x,y,z,w` 重排，不做轴重映射。运行时 `/imu` 样本中 `orientation` 接近单位四元数、`linear_acceleration.z` 接近 `+9.8`，与近直立机体系观测一致。
