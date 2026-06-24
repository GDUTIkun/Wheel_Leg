# 文档入口

当前 `docs/` 只保留正在使用的主线文档。历史迭代拆分、旧任务清单和重复规划文档已经移除，避免后续按过期内容继续推进。

## 当前主线

- `progress.md`：当前阶段进度与待确认问题。
- `stm32_hardware_integration.md`：STM32 硬件接入与 100Hz 实机闭环准备。
- `stm32_ros_comm_task.md`：STM32 与 ROS 第一版通信内容、字段和验收任务。
- `100hz_balance_notes.md`：500Hz 调整到 100Hz 后的仿真平衡记录，作为上硬件前参考基线。
- `leg_angle_mapping.md`：腿部 GIM6010 电机角度到机构角/世界水平角的标定映射。

## 长期约束

- `architecture.md`：仓库结构、ROS2 包边界、仿真/实机切换原则。
- `protocol.md`：仿真和实机共用的 Topic、命名、单位、方向和通信边界。
- `runbook.md`：构建、启动和常用运行入口。

## 维护规则

- 新阶段只更新 `progress.md` 和对应的当前任务文档。
- 已完成阶段只保留对后续调试仍有直接价值的总结。
- 不再新增 `docs/doc/` 这类平行文档体系。
