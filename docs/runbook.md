# 运行归档

## 工作区约定

- `ros2_ws/` 是正式 ROS2 工作区。
- `ros2_ws/build`、`ros2_ws/install`、`ros2_ws/log` 只服务 ROS2 构建，因此固定收口在 `ros2_ws/` 下。
- 仓库根目录同时包含 `firmware/`、`sim/`、`scripts/`、`docs/`，不再把根目录直接当成 ROS2 workspace。

这样做的目的：

- ROS2 构建产物和固件、仿真资源、文档分离。
- 不同设备同步同一仓库时，构建边界更清楚。
- `scripts/*.sh` 可以稳定地只针对 `ros2_ws/` 工作，而不污染仓库根目录。

## 目录职责

- `ros2_ws/`: ROS2 包、`colcon` 构建产物
- `firmware/`: STM32 固件
- `sim/mujoco/runtime/`: MuJoCo runtime 源码
- `sim/mujoco/scenes/scence.xml`: MuJoCo 场景入口
- `scripts/`: 统一构建和启动脚本

## 构建

在仓库根目录执行：

```bash
cd ~/wheel_leg_ws
```

核心控制链：

```bash
./scripts/build.sh core
```

主机构建仿真链：

```bash
./scripts/build.sh sim
```

树莓派构建遥控/硬件链：

```bash
./scripts/build.sh pi
```

全部正式包：

```bash
./scripts/build.sh all
```

清理 ROS2 构建产物：

```bash
./scripts/build.sh clean
```

## 启动分工

### 主机

仿真 + controller：

```bash
./scripts/run_host.sh sim
```

只跑 MuJoCo 仿真，不让 ROS controller 接管：

```bash
./scripts/run_host.sh sim_only
```

只跑 controller：

```bash
./scripts/run_host.sh control_only
```

### 树莓派

只跑遥控：

```bash
./scripts/run_pi.sh rc
```

硬件主链：

```bash
./scripts/run_pi.sh hw
```

只跑硬件控制主链，不起 STM32 bridge：

```bash
./scripts/run_pi.sh hw_core
```

## 手动方式

手动加载环境：

```bash
cd ~/wheel_leg_ws
source /opt/ros/jazzy/setup.bash
source ./ros2_ws/install/local_setup.bash
```

手动跑 MuJoCo 二进制：

```bash
./ros2_ws/build/wheel_leg_simulate_ros2/wheel_leg_simulate ./sim/mujoco/scenes/scence.xml
```

## 兼容入口

当前仍保留兼容脚本：

```bash
./scripts/run.sh sim
./scripts/run.sh sim_only
./scripts/run.sh hw
./scripts/run.sh rc
./scripts/run.sh control_only
```

但后续建议优先使用：

- `scripts/run_host.sh`
- `scripts/run_pi.sh`
