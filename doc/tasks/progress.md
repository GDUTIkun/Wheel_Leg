# 当前任务进度

## 1. 当前迭代

- 迭代编号：`iter-002`
- 迭代名称：控制编排外移与去 `transplant` 化工程重整
- 迭代文档：`doc/iterations/iter-002.md`
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
- 本轮当前是文档重构迭代，允许任务先停留在设计完成状态，不要求同步有代码验证。
- `iter-001` 已完成的 bridge 能力视为当前迭代输入，不在本轮重复验收实现细节。

## 3. 迭代目标概览

`iter-002` 关注：

- 建立正式 ROS2 包化工程结构。
- 明确 `wheel_leg_bridge`、`wheel_leg_control`、`wheel_leg_sim`、`wheel_leg_common` 的职责边界。
- 定义控制编排外移的第一步。
- 定义 `sim adapter` 提取边界。
- 定义 PID、LQR、VMC 算法库提取边界。
- 明确 `transplant/` 的迁移期定位和替代顺序。

`iter-002` 不关注：

- 新动力学模型替换。
- STM32 通信实现。
- 遥控器输入实现。
- 完整状态机落地。
- 完整 `ros2_control` 落地。
- 立即删除 `transplant/`。

## 4. 模块任务总览

| 模块任务 | 文件 | 当前状态 | 说明 |
| --- | --- | --- | --- |
| 控制编排外移任务 | `doc/tasks/controller_orchestration.md` | `[v] 已通过验证` | `wheel_leg_hooks.cc` 主控制链与命令组装已迁入正式 `wheel_leg_control/stand_control_pipeline.*`，并已通过 `10 s+` takeover 仿真复测 |
| MuJoCo sim adapter 提取任务 | `doc/tasks/sim_adapter_extraction.md` | `[v] 已通过验证` | 状态采样、命令执行、ROS2 bridge 读写边界已收口到正式 `wheel_leg_sim`/`wheel_leg_bridge` 复用链，并已通过 `10 s+` 仿真复测 |
| 算法库提取任务 | `doc/tasks/algorithm_library_extraction.md` | `[v] 已通过验证` | 正式算法接口、legacy 运行时与 controller 侧调用链已收口，当前遗留仅为后续长期目录与参数统一工作 |
| 正式工程结构与包边界 | `doc/architecture.md` | `[x] 已完成` | 已更新正式包结构、迁移期定位和长期职责边界 |
| 当前迭代详细设计 | `doc/detail.md` | `[~] 进行中` | 文档边界已固定，代码迁移进展持续补充中 |
| 技术决策记录 | `doc/decisions.md` | `[x] 已完成` | 已追加正式结构和迁移顺序决策 |

## 5. 推荐执行顺序

1. `[x]` 固定 `iter-002` 文档目标和非目标。
2. `[x]` 固定正式 ROS2 包结构。
3. `[x]` 固定 `controller orchestration` 的职责和数据流。
4. `[x]` 固定 `sim adapter` 的职责和暴露接口。
5. `[x]` 固定 PID、LQR、VMC 算法库提取边界。
6. `[x]` 固定从 `wheel_leg_hooks.cc` 迁出的顺序和依赖关系。
7. `[v]` bridge、sim adapter 与控制编排迁移已收口，并已完成一次 `10 s+` takeover 保持验证。

## 6. 当前输入与已完成基础

作为本轮输入，以下内容已存在：

- `iter-001` 已建立 `/joint_states`、`/imu`、`/joint_command` 和 actuator 边界。
- `wheel_leg_msgs/msg/JointCommand.msg` 已存在。
- 当前运行时仍大量依赖 `transplant/mujoco_win/simulate`。
- `wheel_leg_hooks.cc` 仍承载控制流程组织。

当前已补充的代码迁移进展：

- 已建立 `wheel_leg_common`、`wheel_leg_bridge`、`wheel_leg_control`、`wheel_leg_sim` 四个正式包骨架并通过构建。
- `wheel_leg_bridge` 已提供 `/joint_states`、`/imu`、`/joint_command` 的双向消息转换。
- `wheel_leg_hooks.cc` 的主控制链已拆出到正式 `wheel_leg_control/stand_control_pipeline.*`。
- 过渡 `sim_adapter` 已接管状态采样、命令执行和内部采样状态复位。
- `transplant` 仿真工程已开始链接正式 `wheel_leg_bridge` 与 `wheel_leg_sim` 库，而不再只依赖头文件。
- `wheel_leg_sim` 已承载 joint mapping、sim time conversion、command preparation 和 state sample builder 等稳定实现。
- `sensor` 链的基础状态类型已迁入正式 `wheel_leg_sim/sensor_types.hpp`，`transplant/tools/sensor.h` 改为复用正式定义。
- 腿部运动学与 `phi_rate` 更新逻辑已迁入正式 `wheel_leg_sim/leg_kinematics.*`，`sensor.cc` 继续瘦身中。
- 姿态四元数转欧拉角与底盘前向速度投影逻辑已迁入正式 `wheel_leg_sim/attitude_utils.*`。
- 左右腿零位偏置、绝对角换算和腿状态组装 helper 已迁入正式 `wheel_leg_sim/leg_state_assembly.*`。
- 底盘状态填充与前向距离积分逻辑已迁入正式 `wheel_leg_sim/base_state_assembly.*`。
- `RobotSensorData` 总装配流程已迁入正式 `wheel_leg_sim/robot_state_assembly.*`，`sensor.cc` 进一步收缩为 MuJoCo 读数适配层。
- 过渡 `controller_orchestration` 已依赖正式 `wheel_leg_control/stand_control_types.hpp`，控制状态长期接口不再直接等同于 MuJoCo 传感器结构。
- 已新增正式算法接口头 `wheel_leg_control/stand_algorithm_interfaces.hpp`，并补上 `algorithm_adapters.*` 过渡适配层，让编排层按正式 `PidAlgorithm`、`LqrAlgorithm`、`VmcAlgorithm` 接口调用当前 transplant 实现。
- 已将通用算法适配器前移到正式头 `wheel_leg_control/function_algorithm_adapters.hpp`，`wheel_leg_hooks.cc` 仅在现场用 lambda 绑定当前 transplant PID/LQR/VMC 实现。
- 已将默认站立目标和 legacy PID 数值配置前移到正式头 `wheel_leg_control/stand_runtime_defaults.hpp`，`wheel_leg_hooks.cc` 当前只保留对 legacy `PIDInit` 的过渡翻译。
- 过渡 stand 控制流水线实现已整体前移到正式 `wheel_leg_control/stand_control_pipeline.*`，`transplant/wheel_leg/controller_orchestration.*` 已退出构建链。
- 已新增正式 `wheel_leg_control/stand_control_runtime.*` 统一持有控制目标、算法回调适配器和 `ControlAlgorithmSet` 装配，`wheel_leg_hooks.cc` 不再每步现场构造整套算法适配对象。
- 已新增正式 `wheel_leg_sim/control_state_bridge.*` 承接 `RobotSensorData -> StandControlState` 组装，`wheel_leg_hooks.cc` 不再手写控制状态字段映射。
- 已新增过渡 `transplant/wheel_leg/legacy_stand_control_bridge.*` 集中承接 legacy PID 实例、初始化配置翻译、算法回调绑定和每步控制执行，`wheel_leg_hooks.cc` 不再直接持有这批 legacy 运行时对象。
- `wheel_leg_control` 已新增 ROS 侧 `legacy_algorithms.*`，并已将 controller 主输入从 `/joint_states`、`/imu` 的二次重建迁到正式 `/robot_state` 接口；`wheel_leg_controller_node` 现可直接消费 `StandControlState` 并发布非零 `/joint_command`。
- `legacy_stand_control_bridge.*` 已继续收口为直接复用正式 `wheel_leg_control/legacy_algorithms.*`，仿真目标已不再直接编译或包含 `transplant/tools/pid.*`、`lqr_k.*`、`vmc.*`。
- `wheel_leg_hooks.cc` 已进一步瘦身：ROS takeover 分支和切换日志已并入 `ros2_bridge.*`，legacy 控制执行与 plotting 所需最小状态已并入 `legacy_stand_control_bridge.*`，hook 侧更接近纯 MuJoCo 生命周期入口。
- 过渡 `wheel_leg/sim_adapter.*` 已删除：其 joint state / imu / actuator 写入辅助已并入 `ros2_bridge.*`，legacy 控制执行所需传感采样与命令写入已直接复用 `sensor.*` 与正式 `wheel_leg_sim` helper。
- 仿真侧 plotting / `PrintSensors` / `math_utils` 这条历史调试链已整体移除，后续状态分析统一转到 ROS 侧工具链。
- `wheel_leg_msgs/msg/StandControlState.msg` 已新增，MuJoCo bridge 已开始发布正式 `/robot_state` 控制状态 topic，供后续仿真与硬件共用。
- `wheel_leg_controller_node` 已切到 `/robot_state` 驱动、按消息时间戳使用约 `0.002 s` 的真实 `dt`，并新增接管调试 trace：首次撞到发布前限幅时，可自动将最近一段 `StandControlState + raw/clamped /joint_command` 样本落到 `/tmp/wheel_leg_takeover_trace.csv`。
- 已完成一次恢复到 `500 Hz` 主线后的短时 takeover 复测：controller 未再立即撞发布前限幅，仿真短时窗口内也未重现 `QACC` 不稳定告警。
- 已完成一次 `10 s+` takeover 保持复测：隔离 `ROS_DOMAIN_ID=109` 下，takeover 持续约 `12.43 s`、bridge 累计写入 `6162` 次 `/joint_command`，未生成 `/tmp/wheel_leg_takeover_trace.csv`，日志中未出现 `QACC` / unstable 告警。
- 当前工作区已完成一次 `colcon build`，新增正式接口头、适配器层和过渡编排修改均已通过构建。
- 已完成一次 `iter-002` 最小运行烟测：在 `source /opt/ros/jazzy/setup.bash && source install/setup.bash` 环境下，`wheel_leg_simulate` 可成功加载 `transplant/mujoco_win/model/scence.xml`，并完成 `wheel_leg simulate ready` 与 ROS2 bridge 初始化。
- 已完成一次 `/robot_state` 正式接口烟测：`/mujoco_bridge` 可发布 `wheel_leg_msgs/msg/StandControlState`，controller 已确认订阅 `/robot_state` 并继续发布 `/joint_command`。

## 7. 当前阻塞或待确认问题

- 当前无新的架构方向阻塞项。
- `iter-002` 本轮“一次移植 + 仿真验证”范围内的收口项已完成，可按本次结论结束当前任务。
- 后续若继续推进，仅剩长期工程化工作：统一算法参数/目录布局、继续缩减 `transplant/` 保留面的历史过渡痕迹；这些不再阻塞本次任务结项。
