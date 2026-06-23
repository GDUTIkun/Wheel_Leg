# 构建与运行 Profile 文档

## 1. 文档目标

本文档固定后续重构时的构建入口和运行入口，避免不同设备手工敲不同 `colcon` 命令，或通过删包、注释 CMake 的方式规避构建冲突。

## 2. 基本原则

- 所有构建命令都从仓库根目录进入 `scripts/` 调用。
- 实际构建目录固定为 `ros2_ws/`。
- 通过“包白名单”选择构建内容。
- 设备专属依赖只允许出现在对应包中。

## 3. 计划中的构建入口

后续重构完成后，统一使用：

```bash
./scripts/build.sh core
./scripts/build.sh sim
./scripts/build.sh pi
./scripts/build.sh all
./scripts/build.sh clean
```

各 profile 语义如下。

### 3.1 `core`

构建范围：

- `wheel_leg_msgs`
- `wheel_leg_common`
- `wheel_leg_bridge`
- `wheel_leg_control`

适用场景：

- 纯控制侧开发
- 公共接口检查
- 不需要仿真和硬件依赖时的快速编译

### 3.2 `sim`

构建范围：

- `core`
- `wheel_leg_sim`
- `wheel_leg_bringup`

适用场景：

- 桌面仿真开发
- MuJoCo 与控制器联调

说明：

- MuJoCo 相关依赖只应影响该 profile

### 3.3 `pi`

构建范围：

- `core`
- `wheel_leg_rc`
- `wheel_leg_stm32_bridge`
- `wheel_leg_bringup`

适用场景：

- 树莓派侧开发
- 遥控器接入
- STM32 通信联调

说明：

- 不要求 MuJoCo 存在

### 3.4 `all`

构建范围：

- 所有正式 ROS2 包

适用场景：

- 完整开发环境
- CI 或一体机联调环境

### 3.5 `clean`

职责：

- 清理 `ros2_ws/build`
- 清理 `ros2_ws/install`
- 清理 `ros2_ws/log`

## 4. 计划中的运行入口

后续重构完成后，统一使用：

```bash
./scripts/run.sh sim
./scripts/run.sh sim_rc
./scripts/run.sh hw
./scripts/run.sh control_only
```

### 4.1 `sim`

- 启动 MuJoCo 仿真入口
- 启动控制器
- 不默认接入遥控器

### 4.2 `sim_rc`

- 启动 MuJoCo 仿真入口
- 启动控制器
- 启动遥控器输入节点

### 4.3 `hw`

- 启动控制器
- 启动 STM32 通信桥
- 启动遥控器输入节点

### 4.4 `control_only`

- 只启动控制器和必要最小依赖
- 用于纯控制逻辑测试

## 5. 包依赖约束

为避免构建冲突，后续重构时必须满足：

- `wheel_leg_control` 不依赖 MuJoCo
- `wheel_leg_control` 不依赖串口或 STM32 协议实现
- `wheel_leg_common` 不依赖 MuJoCo 或串口
- `wheel_leg_sim` 才能依赖 MuJoCo 相关实现
- `wheel_leg_stm32_bridge` 才能依赖 STM32 通信相关实现

## 6. 当前状态说明

当前仓库还未完成这些脚本实现，但后续重构必须以本文档为准，不再以“临时命令口径”推进。
