# 当前任务进度

## 1. 当前迭代

- 迭代编号：`iter-003`
- 迭代名称：树莓派 iBUS 遥控输入节点
- 迭代文档：`doc/iterations/iter-003.md`
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

`iter-003` 关注：

- 固定 `FlySky FS-iA6B + iBUS` 遥控输入链路。
- 固定 Raspberry Pi `/dev/ttyAMA3` 串口接收边界。
- 固定“先验证串口是否收到数据，再进入协议解析”的前置测试。
- 固定 `rc_ibus_node` 的解包、原始通道发布和状态诊断职责。
- 固定从原始通道到 `/cmd_vel`、`/control_mode`、`/body_cmd` 的映射边界。
- 固定“遥控映射需要用户现场配合拨动摇杆和开关”的流程要求。
- 固定死区、归一化、方向反转和 failsafe 行为。

`iter-003` 不关注：

- STM32 通信实现。
- 地面站 UI。
- `iBUS` telemetry 回传。
- 完整状态机落地。
- 完整 `ros2_control` 落地。

## 4. 模块任务总览

| 模块任务 | 文件 | 当前状态 | 说明 |
| --- | --- | --- | --- |
| `rc_ibus_node` 设计任务 | `doc/tasks/rc_ibus_node.md` | `[~] 设计已冻结` | 已固定串口、前置接收测试、协议解包、原始通道发布和状态诊断边界 |
| 遥控命令映射任务 | `doc/tasks/rc_command_mapping.md` | `[~] 设计已冻结` | 已固定用户配合映射流程、`/cmd_vel`、`/control_mode`、`/body_cmd` 的映射职责与 failsafe |
| 树莓派串口联通性测试节点 | `src/wheel_leg_rc/src/rc_serial_probe_node.cpp` | `[x] 代码已完成` | 已新增 ROS2 串口探测节点，先验证 `/dev/ttyAMA3` 是否收到持续字节流，不依赖 `iBUS` 解包 |
| 第二阶段详细设计 | `doc/detail.md` | `[~] 文档已更新` | 已新增 `iter-003` 的 `rc_ibus_node` 详细设计章节 |
| 总体架构更新 | `doc/architecture.md` | `[~] 文档已更新` | 已补充 `rc_ibus_node` 的长期定位和 ROS2 接口边界 |
| 总需求更新 | `doc/proposal.md` | `[~] 文档已更新` | 已将第二阶段协议收口为 `FS-iA6B + iBUS + /dev/ttyAMA3`，并补前置测试要求 |

## 5. 推荐执行顺序

1. `[x]` 固定第二阶段硬件事实：`FS-iA6B + iBUS + /dev/ttyAMA3`。
2. `[x]` 固定“是否有信息接收到”的串口联通性前置测试。
3. `[x]` 固定 `rc_ibus_node` 的 UART / 协议 / 预处理 / 映射 / failsafe 边界。
4. `[x]` 固定 `/rc/channels_raw` 与 `/rc/status` 的调试和诊断定位。
5. `[x]` 固定用户现场配合的遥控映射确认流程。
6. `[x]` 固定 `/cmd_vel`、`/control_mode`、`/body_cmd` 的输入职责。
7. `[x]` 固定第二阶段仿真与实机验证导向。

## 6. 当前输入与已完成基础

作为本轮输入，以下内容已存在：

- `iter-001` 已建立 `/joint_states`、`/imu`、`/joint_command` 和 actuator 边界。
- `iter-002` 已完成控制编排外移、`sim adapter` 收口和 `/robot_state` 正式接口建立。
- 当前第二阶段硬件事实已经明确：`FlySky FS-iA6B`、`iBUS`、`/dev/ttyAMA3`、`GPIO8/9`。
- 当前项目没有地面站，需要在树莓派 ROS2 环境中直接完成通道确认。
- 当前已新增 `wheel_leg_rc` 包和 `rc_serial_probe_node`，可作为进入 `rc_ibus_node` 前的最小串口联通性验证入口。

## 7. 当前阻塞或待确认问题

- 前置串口联通性测试必须先通过，否则不得继续进入 `iBUS` 帧解析和遥控映射。
- 遥控映射确认阶段需要用户现场配合拨动摇杆和开关。
- 后续进入代码实现时，需要用真实遥控器实测最终通道编号分配和各通道端点。
