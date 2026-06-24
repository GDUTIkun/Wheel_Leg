# wheel_leg_project

当前仓库已按以下层次收口：

- `ros2_ws/`：正式 ROS2 工作区
- `firmware/`：STM32 固件
- `sim/`：MuJoCo 资源
- `scripts/`：统一 build/run 入口
- `docs/`：结构和迁移文档
- `transplant/`：迁移参考区

ROS2 构建产物固定放在 `ros2_ws/` 下：

- `ros2_ws/build`
- `ros2_ws/install`
- `ros2_ws/log`

这样 `ros2_ws/` 只承担 ROS2 workspace 职责，避免和 `firmware/`、`sim/`、`scripts/` 混放。

常用命令：

```bash
./scripts/build.sh core
./scripts/build.sh sim
./scripts/build.sh pi
./scripts/run_host.sh sim
./scripts/run_host.sh sim_only
./scripts/run_pi.sh rc
./scripts/run_pi.sh hw
```

完整文档入口见：

- [docs/README.md](docs/README.md)
