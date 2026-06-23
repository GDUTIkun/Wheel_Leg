# 工程结构迁移步骤文档

## 1. 文档目标

本文档用于固定“先文档、后重构”的实施顺序，避免重构过程变成边改边猜。

## 2. 迁移目标

从当前“ROS2 包、旧文档、MuJoCo 参考资源混放”的状态，迁到以下组织方式：

- ROS2 工作区固定在 `ros2_ws/`
- 固件固定在 `firmware/stm32/`
- MuJoCo 资源固定在 `sim/mujoco/`
- 构建与运行入口固定在 `scripts/`
- 正式文档固定在 `docs/`

## 3. 已完成动作

当前已经完成：

- `src/` -> `ros2_ws/src/`
- `build/ install/ log/` -> `ros2_ws/`
- `doc/` -> `docs/`

说明：

- 这些动作属于工作区骨架收口。
- 后续代码和脚本重构仍未完成，当前还不能视为迁移结束。

## 4. 后续迁移顺序

### 步骤 1：文档冻结

冻结以下文档作为正式约束：

- `docs/architecture.md`
- `docs/protocol.md`
- `docs/build_profiles.md`
- `docs/migration.md`

目标：

- 先把目录、包边界、构建入口、切换方式说清楚。

### 步骤 2：补正式包

新增并补齐以下正式包：

- `wheel_leg_bringup`
- `wheel_leg_hw`
- `wheel_leg_stm32_bridge`

目标：

- 先补结构落点，再填实现内容。

### 步骤 3：补脚本入口

新增并收口：

- `scripts/build.sh`
- `scripts/run.sh`
- `scripts/env.sh`
- `scripts/flash_stm32.sh`

目标：

- 固定不同设备的构建和启动方式。

### 步骤 4：收口仿真资源

将正式使用的 MuJoCo 资源迁到：

- `sim/mujoco/runtime/`
- `sim/mujoco/models/`
- `sim/mujoco/scenes/`
- `sim/mujoco/assets/`

目标：

- 不再通过 `transplant/mujoco_win/model/` 或 `transplant/mujoco_win/simulate/` 作为正式入口。

### 步骤 5：bringup 接管切换入口

新增并固定：

- `sim.launch.py`
- `hw.launch.py`

目标：

- 同一套控制主链切换不同 backend

### 步骤 6：STM32 bridge 占位实现

先完成最小桥接框架：

- 占位节点
- 基本参数
- `/robot_state` 发布接口
- `/joint_command` 订阅接口

目标：

- 先把接口落点占住，不在本轮就把通信协议做满

## 5. 迁移期间的禁止事项

- 不在 `transplant/` 新增正式运行时代码
- 不把 MuJoCo 头文件引入 `wheel_leg_control`
- 不把串口或 STM32 协议实现引入 `wheel_leg_control`
- 不通过注释 `CMakeLists.txt` 规避构建冲突
- 不在根目录重新创建新的 `build/ install/ log/`

## 6. 完成标准

迁移完成时应满足：

- `ros2_ws/src/` 只保留正式 ROS2 包
- `sim/mujoco/` 成为正式仿真资源入口
- `scripts/` 成为统一 build/run 入口
- `docs/` 明确记录结构与规则
- 仿真与实机切换只切状态源和命令去向，不切控制器主逻辑
