# 当前任务进度

## 1. 当前迭代

- 迭代编号：`iter-001`
- 迭代名称：ROS2-MuJoCo Topic 桥接
- 迭代文档：`doc/iterations/iter-001.md`
- 详细设计：`doc/detail.md`
- 验证记录：`doc/validation.md`

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
- 未经过仿真或实机验证的任务不能标记为 `[v] 已通过验证`。
- 当前任务均为 `iter-001` 范围内任务，不包含遥控器、STM32、完整 `ros2_control` 或控制器完整外移。

## 3. 模块任务总览

| 模块任务 | 文件 | 当前状态 | 说明 |
| --- | --- | --- | --- |
| MuJoCo 桥接节点模块 | `doc/tasks/mujoco_bridge.md` | `[v] 已通过验证` | 已建立 ROS2 节点、发布器、订阅器和 MuJoCo step 边界；短时仿真可见节点和 topic |
| 关节状态发布模块 | `doc/tasks/joint_state_publisher.md` | `[v] 已通过验证` | `/joint_states` 可 echo 到消息，`name`、`position`、`velocity` 长度一致，`effort` 为空 |
| IMU 状态发布模块 | `doc/tasks/imu_publisher.md` | `[v] 已通过验证` | `/imu` 可 echo 到消息，`frame_id=base_link`，四元数按 ROS 字段发布 |
| 关节命令订阅模块 | `doc/tasks/joint_command_subscriber.md` | `[v] 已通过验证` | `/joint_command` 有效命令可被接收并触发写入边界；无效命令可被拒绝 |
| actuator 命令写入模块 | `doc/tasks/actuator_writer.md` | `[v] 已通过验证` | `enable_ros_command=true` 时有效命令写入 actuator 边界；无效命令不写入 |
| 接口与参数约束模块 | `doc/tasks/interface_constraints.md` | `[x] 已确认` | 当前代码实现前接口约束已确认 |

## 4. 推荐执行顺序

1. `[x]` 确认 `/joint_command` 消息类型：使用自定义消息。
2. `[x]` 确认 `/joint_command` 自定义消息字段、包名和消息名。
3. `[x]` 确认关节命名表和 actuator 映射表。
4. `[x]` 确认状态发布频率和命令处理频率。
5. `[x]` 确认 `/imu` frame id 和坐标系定义。
6. `[x]` 确认节点名、topic namespace、spin 策略、effort 策略、命令接管、超时、限幅和无效命令策略。
7. `[x]` 创建 `wheel_leg_msgs/msg/JointCommand.msg`。
8. `[v]` 实现并验证 MuJoCo 桥接节点模块。
9. `[v]` 实现并验证 `/joint_states` 发布。
10. `[v]` 实现并验证 `/imu` 发布。
11. `[v]` 实现并验证 `/joint_command` 订阅端点。
12. `[v]` 实现并验证 actuator 写入边界。
13. `[v]` 执行 IMU 坐标语义验证。

## 5. 当前阻塞或待确认问题

- 当前无接口确认阻塞项。
- `iter-001` 当前文档内验证项已完成。下一步可以进入下一轮迭代任务选择。
