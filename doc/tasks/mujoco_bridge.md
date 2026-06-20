# 模块任务：MuJoCo 桥接节点

## 1. 模块名称

MuJoCo 桥接节点模块。

## 2. 当前迭代目标

在 MuJoCo 仿真侧建立 ROS2 Topic 桥接的运行时边界，为 `/joint_states`、`/imu` 和 `/joint_command` 提供统一节点上下文。

本模块只负责桥接节点生命周期和模块调度，不实现控制算法。

## 3. 任务 checklist

- `[x]` 阅读 `doc/detail.md` 中 MuJoCo 桥接节点模块设计。
- `[x]` 确认 MuJoCo 仿真程序中适合初始化桥接的入口：`OnModelLoaded`、`BeforeStep`、`AfterStep`。
- `[x]` 确认 ROS2 初始化与 MuJoCo 模型加载的先后关系：模型加载后初始化 bridge。
- `[x]` 确认 ROS2 spin 与 MuJoCo step 是否同线程：同线程，在 MuJoCo step 边界非阻塞处理 ROS 回调。
- `[x]` 确认桥接节点名称和 topic namespace：节点名 `mujoco_bridge`，当前不使用 namespace。
- `[x]` 确认状态发布频率：`100 Hz`。
- `[x]` 确认命令处理频率：每个 MuJoCo step 处理一次最新有效命令。
- `[x]` 确认状态消息时间戳策略：使用 MuJoCo 仿真时间 `d->time`。
- `[x]` 建立 ROS2 节点上下文。
- `[x]` 创建 `/joint_states` 发布器。
- `[x]` 创建 `/imu` 发布器。
- `[x]` 创建 `/joint_command` 订阅器前确认消息类型：使用自定义消息。
- `[x]` 创建 `/joint_command` 订阅器前确认自定义消息字段、包名和消息名。
- `[x]` 将状态发布和命令处理挂接到 MuJoCo 仿真周期边界。
- `[v]` 完成后执行仿真验证，ROS2 CLI 可见节点和 topic。

## 4. 每个任务的输入

- `doc/detail.md`
- `doc/iterations/iter-001.md`
- MuJoCo 模型对象。
- MuJoCo 数据对象。
- ROS2 Jazzy 运行环境。
- 当前仿真时间。
- 状态发布频率：`100 Hz`。
- 命令处理频率：每个 MuJoCo step 处理一次最新有效命令。
- 状态消息时间戳策略：使用 MuJoCo 仿真时间 `d->time`。

## 5. 每个任务的输出

- ROS2 节点。
- `/joint_states` 发布器。
- `/imu` 发布器。
- `/joint_command` 订阅器。
- 桥接上下文。
- 模块调度边界。

## 6. 完成标准

- MuJoCo 仿真启动后可以建立 ROS2 节点。
- ROS2 CLI 能看到当前迭代相关 topic。
- 桥接节点不包含 LQR、VMC、PID 等控制算法。
- 桥接节点不修改 MuJoCo UI 渲染逻辑。
- 未确认的接口不被硬编码为最终方案。

## 7. 验证方法

仿真验证：

- 启动 ROS2 环境。
- 启动 MuJoCo 仿真。
- 确认桥接节点创建成功。
- 使用 ROS2 CLI 查看 topic 是否存在。
- 确认未发布命令时不破坏现有仿真启动。

实机验证：

- 本模块不进行实机验证。
- 真机等价功能留到 STM32 通信层迭代。

## 8. 当前状态

`[v] 已通过最小仿真烟测`

当前待确认：

- 暂无。下一步进入 `/joint_command` 回调级验证和 actuator 响应验证。

## 9. 上下文恢复记录

记录时间：2026-06-20。

本节只用于恢复断开的 Codex 上下文，不代表继续推进下一步任务。

上一次上下文中的建议是先做“MuJoCo bridge 节点边界”的最小代码任务：只建立 ROS2 节点、发布器/订阅器和 MuJoCo step 边界，不先写完整状态映射，也不先写 actuator 覆盖。目标是最快让仿真侧出现 `/joint_states`、`/imu`、`/joint_command` 的 ROS 端点。

上一次上下文中已经识别到的接入点：

- 现有仿真入口是 `wheel_leg_hooks.cc` 的 `OnModelLoaded`、`BeforeStep`、`AfterStep`。
- bridge 生命周期适合挂在现有 hooks 上。
- ROS2 spin 与 MuJoCo step 保持同线程，在 MuJoCo step 边界非阻塞处理 ROS 回调。
- 节点名使用 `mujoco_bridge`，当前不使用 namespace。
- 状态发布频率暂定 `100 Hz`。
- 命令处理频率为每个 MuJoCo step 处理一次最新有效命令。
- 状态消息时间戳策略使用 MuJoCo 仿真时间 `d->time`。

上一次上下文中已经做过的尝试和结论：

- 新增过 `ros2_bridge.h` / `ros2_bridge.cc`。
- 修改过仿真 `CMakeLists.txt`，接入 ROS2、`sensor_msgs`、`wheel_leg_msgs` 等依赖。
- 修改过 `wheel_leg_hooks.cc`，把 bridge 调用接入 hooks。
- 使用 `source /opt/ros/jazzy/setup.bash && source install/local_setup.bash` 配置环境。
- `cmake -S transplant/mujoco_win/simulate -B build/wheel_leg_simulate_ros2 -DWHEEL_LEG_ENABLE_ROS2=ON` 配置时曾遇到 `ament_target_dependencies` 和 `target_link_libraries` 签名问题，并已按 ROS2 常用写法调整。
- `cmake --build build/wheel_leg_simulate_ros2 -j2` 曾编译通过。
- 显式传入模型路径 `transplant/mujoco_win/model/scence.xml` 后，短时运行曾能在 ROS2 CLI 中看到 `/joint_states`、`/imu`、`/joint_command` 和 `/mujoco_bridge`。
- 短时运行使用 `timeout` 时，仿真进程可能不响应普通 SIGTERM，需要用 `timeout -k` 或手动清理。
- 一次短时读取 `/joint_states`、`/imu` 消息的验证尚未形成最终结论。

当前暂停要求：

- 不继续做下一步代码实现。
- 不继续扩展状态映射。
- 不继续接管 actuator。
- 不继续做额外仿真验证。
- 等额度充足或用户明确要求后，再恢复该任务。

## 10. 本次恢复后的工程记录

记录时间：2026-06-20。

本次已恢复工程并完成最小 bridge 节点边界验证：

- 修复 `src/wheel_leg_msgs/package.xml`，补充 `ament_cmake` build type，使 `colcon list` 正确识别 `wheel_leg_msgs` 为 `ros.ament_cmake` 包。
- 清理工作区根目录误生成的空 `AMENT_IGNORE` 和 `CMakeFiles/`，恢复 `colcon` 包发现能力。
- `colcon build --packages-select wheel_leg_msgs` 通过。
- `cmake -S transplant/mujoco_win/simulate -B build/wheel_leg_simulate_ros2 -DWHEEL_LEG_ENABLE_ROS2=ON` 配置通过。
- `cmake --build build/wheel_leg_simulate_ros2 -j2` 编译通过。
- 短时启动 `build/wheel_leg_simulate_ros2/wheel_leg_simulate transplant/mujoco_win/model/scence.xml` 后，ROS2 CLI 可见节点 `/mujoco_bridge`。
- ROS2 CLI 可见 topic：`/joint_states`、`/imu`、`/joint_command`。
- `/joint_command` topic 类型为 `wheel_leg_msgs/msg/JointCommand`，订阅者为 `/mujoco_bridge`。
- 本次没有验证 `enable_ros_command=true` 时的 actuator 实际响应。
