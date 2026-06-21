# Iteration 003: 树莓派 iBUS 遥控输入节点

- 日期：2026-06-21
- 输入文档：`doc/proposal.md`、`doc/architecture.md`
- 输出文档：`doc/detail.md`、`doc/tasks/rc_ibus_node.md`、`doc/tasks/rc_command_mapping.md`

## 1. 本次迭代目标

本次迭代目标是完成树莓派侧 `iBUS` 遥控输入节点设计，固定从 `FlySky FS-iA6B` 到统一 ROS2 控制命令的输入链路。

本迭代关注的最小闭环：

```text
FS-iA6B iBUS
  ↓
/dev/ttyAMA3
  ↓
rc_ibus_node
  ├── /rc/channels_raw
  ├── /rc/status
  ├── /cmd_vel
  ├── /control_mode
  └── /body_cmd
```

本迭代是文档设计与接口冻结迭代，不实现代码。

## 2. 本次不做什么

本次迭代不做以下内容：

- 不实现地面站 UI。
- 不实现 STM32 遥控转发链路。
- 不实现 `iBUS` telemetry 回传。
- 不实现复杂状态机。
- 不让控制器直接读取串口原始数据。
- 不在本轮定义最终自定义消息字段的完整 `.msg` 文件。

## 3. 涉及模块

当前迭代涉及模块：

- `rc_ibus_node`
- `uart_reader`
- `ibus_frame_parser`
- `channel_preprocessor`
- `command_mapper`
- `failsafe_guard`

未进入当前迭代、只保留边界的模块：

- STM32 通信节点。
- 遥测回传。
- 完整控制模式状态机。
- 上层规划输入。

## 4. 输入条件

当前迭代输入条件：

- 接收机型号已确认为 `FlySky FS-iA6B`。
- 接收机输出协议已确认为 `iBUS`。
- 树莓派使用 `PASS /dev/ttyAMA3`，对应 `GPIO8/9`。
- 当前项目没有地面站，需要在树莓派 ROS2 环境中直接完成通道确认。

## 5. 输出结果

当前迭代输出结果：

- `doc/detail.md` 中新增 `rc_ibus_node` 详细设计。
- 固定第二阶段链路、节点职责和接口边界。
- 固定 `/rc/channels_raw`、`/rc/status`、`/cmd_vel`、`/control_mode`、`/body_cmd` 的输出语义。
- 固定通道预处理和 failsafe 行为。
- 固定“先验证串口是否收到数据，再进入协议解析和遥控映射”的前置测试顺序。

## 6. 最小实现任务

后续代码实现可拆成以下最小任务：

1. 打开 `/dev/ttyAMA3` 并配置 `115200 8N1`。
2. 先完成“是否有信息接收到”的串口联通性测试。
3. 在 ROS2 节点中接收字节流并解包 32-byte `iBUS` 帧。
4. 校验 checksum 并提取 `CH1~CH6` 等原始通道值。
5. 发布 `/rc/channels_raw` 和 `/rc/status`。
6. 由用户现场配合拨动摇杆和开关，确认物理输入与通道编号映射。
7. 对通道执行归一化、死区、方向反转和开关离散化。
8. 将通道映射为 `/cmd_vel`、`/control_mode`、`/body_cmd`。
9. 实现串口超时、坏帧和失联场景下的 failsafe。

## 7. 仿真验证方法

后续实现完成后，仿真验证应至少包含：

- 在树莓派或等效 Linux 环境启动 `rc_ibus_node`。
- 先确认串口链路已有持续数据输入，再进入协议层验证。
- 观察 `/rc/channels_raw` 是否持续发布。
- 由用户现场单独拨动各摇杆和开关，确认通道变化与物理输入一致。
- 观察 `/cmd_vel`、`/control_mode`、`/body_cmd` 是否随映射更新。
- 让 MuJoCo 控制侧订阅统一命令，确认仿真可被遥控器驱动。

## 8. 实机验证方法

后续实机验证应至少包含：

- 接收机接入树莓派 `/dev/ttyAMA3`。
- 先验证串口打开后是否确实收到持续字节流。
- 再验证帧接收和通道读取稳定。
- 由用户现场配合完成摇杆和开关映射确认。
- 验证中位死区和方向反转配置。
- 验证接收机断链或串口异常时进入 failsafe。

## 9. 通过标准

文档层通过标准：

- `doc/detail.md` 已补充 `rc_ibus_node` 设计。
- `doc/tasks/rc_ibus_node.md` 和 `doc/tasks/rc_command_mapping.md` 存在。
- 第二阶段协议、串口和输出接口语义已固定。

后续代码层通过标准：

- 在不依赖帧语义的前提下，可先确认树莓派串口确实收到接收机数据。
- ROS2 节点可稳定读取并校验 `iBUS` 数据。
- 原始通道 topic 可用于通道确认和排障。
- `/cmd_vel`、`/control_mode`、`/body_cmd` 可按映射正常输出。
- 失联时命令进入安全状态。

## 10. 风险点

- 未经实测前，各物理摇杆与通道编号对应关系仍需现场确认。
- 通道端点和中位值可能与名义 `1000/1500/2000` 存在偏差，需要参数化标定。
- 开关通道可能存在两段或三段不同输出分布，需要映射时留阈值配置。
- 树莓派串口权限、占用或底层配置错误会直接导致节点不可用。

## 11. 需要用户确认的问题

- 遥控映射确认阶段需要用户现场配合拨动摇杆和开关。
- 进入代码实现前，需要以真实遥控器实测结果确认最终通道编号分配表。
