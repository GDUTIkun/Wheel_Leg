# 算法库提取任务

## 1. 任务目标

定义 PID、LQR、VMC 等控制算法从 `transplant/mujoco_win/simulate/tools/` 迁入正式工程结构时的边界、接口和归属方式。

当前任务重点不是马上重写算法实现，而是先让算法从 MuJoCo 内嵌控制流程中解耦，成为可被正式控制层调用的独立能力。

## 2. 当前输入

- `RobotStateSnapshot`
- 控制目标
- 参数结构
- 通用数学类型

## 3. 当前输出

- 算法中间控制量
- `ControlCommand`
- 供控制编排层消费的结果结构

## 4. 当前边界要求

- 算法接口允许依赖 `wheel_leg_common` 中的共享数据结构和数学工具。
- 算法接口禁止依赖 MuJoCo 类型。
- 算法接口禁止直接依赖 ROS2 node 类型。
- 初期允许实现仍来源于当前 transplant 公式和参数。

## 5. 需要完成的文档化事项

1. 标识 `tools/*.cc` 中哪些内容属于算法实现，哪些只是仿真辅助。
2. 定义算法层对外最小接口。
3. 定义算法参数结构和共享数据结构的归属边界。
4. 明确算法库与 `wheel_leg_control`、`wheel_leg_common` 的依赖方向。
5. 明确算法迁移发生在控制编排边界明确之后。

## 6. 推荐迁移顺序

1. 先完成 `controller orchestration` 边界定义。
2. 再抽出算法接口。
3. 再逐步将当前 transplant 实现迁入正式库。
4. 最后根据后续阶段需要替换动力学模型或数学实现。

## 7. 完成标准

满足以下条件即可认为该任务文档完成：

- 能明确 PID、LQR、VMC 长期不再寄宿于 MuJoCo 控制流程。
- 能明确算法接口的输入输出和依赖方向。
- 能支持“先迁编排，再迁实现”的迁移策略。

## 8. 非目标

- 不在本任务中重新设计 LQR 数学模型。
- 不在本任务中替换为新动力学模型。
- 不在本任务中完成调参或性能验证。

## 9. 当前状态

- `[v] 已通过验证`

## 10. 当前已落地进展

- 当前尚未把 PID、LQR、VMC 正式迁入独立算法包，但控制编排已经先和 MuJoCo 读写边界分离。
- 已新增正式算法接口头 `wheel_leg_control/stand_algorithm_interfaces.hpp`，明确：
  - `PidAlgorithm`
  - `LqrAlgorithm`
  - `VmcAlgorithm`
- 已新增正式头 `wheel_leg_control/function_algorithm_adapters.hpp`，用于把当前 `transplant/tools/` 中的 PID、LQR、VMC 实现以函数式方式适配到正式算法接口。
- 已新增正式 `wheel_leg_control/stand_control_runtime.*`，用于在正式包中持有控制目标、函数式算法适配器和 stand 控制运行时入口。
- 已新增过渡 `transplant/wheel_leg/legacy_stand_control_bridge.*`，用于把 legacy PID/LQR/VMC 实例和初始化逻辑从 hook 文件中集中收口。
- 已新增正式 `wheel_leg_control/legacy_algorithms.*`，将 legacy PID、LQR、VMC 公式实现复制到 ROS 控制包内，供 controller node 直接调用。
- `transplant/mujoco_win/simulate/wheel_leg/legacy_stand_control_bridge.*` 也已切到直接复用正式 `wheel_leg_control/legacy_algorithms.*`，不再直接包含 `transplant/tools/pid.*`、`lqr_k.*`、`vmc.*`。
- `controller_orchestration.cc` 已明确暴露出算法调用顺序：
- 正式 `wheel_leg_control/stand_control_pipeline.*` 已明确暴露出算法调用顺序：
  - 腿长 PID
  - LQR
  - 转向 PID
  - 防劈叉 PID
  - VMC
- 当前过渡编排层已经通过正式算法接口而不是直接底层函数来组织控制调用，控制运行时装配也已迁入正式包。
- ROS controller node 也已可通过正式算法接口直接调用 `wheel_leg_control/legacy_algorithms.*`，不再只能发布零命令骨架。
- 这意味着算法层虽然仍驻留在 `transplant/tools/`，但调用位置已经从直接写 actuator 的流程中抽离，且接口形态已开始脱离 transplant 具体实现，具备后续独立迁移前提。

## 11. 当前识别出的算法 / 非算法边界

更偏算法实现的文件：

- `transplant/mujoco_win/simulate/tools/pid.*`
- `transplant/mujoco_win/simulate/tools/lqr_k.*`
- `transplant/mujoco_win/simulate/tools/vmc.*`

更偏仿真适配或运行时辅助的文件：

- `transplant/mujoco_win/simulate/tools/sensor.*`
- `transplant/mujoco_win/simulate/tools/actuator.*`
- `transplant/mujoco_win/simulate/wheel_leg/sim_adapter.*`

混合程度较高、后续需要再拆分的部分：

- `transplant/mujoco_win/simulate/wheel_leg/wheel_leg_hooks.cc`
  当前虽已大幅瘦身，但仍保留仿真生命周期入口和少量绘图/日志杂项。

- `transplant/mujoco_win/simulate/wheel_leg/legacy_stand_control_bridge.*`
  当前集中承接 legacy 算法运行时装配，但已不再直接依赖 `transplant/tools/pid.*`、`lqr_k.*`、`vmc.*`；后续仍需要继续收缩其过渡生命周期职责。

- `wheel_leg_control/legacy_algorithms.*`
  当前是从 legacy 实现复制进正式包的过渡版本，后续仍需要统一命名、参数类型和测试边界。

## 12. 当前未完成部分

- 尚未定义长期独立算法库包名和最终目录布局。
- 尚未把 PID、LQR、VMC 的参数结构完全迁入 `wheel_leg_common` 或独立算法公共头。
- 当前剩余工作属于后续长期工程化整理，不阻塞本轮功能与验证闭环。

## 13. 本次结论

- 正式算法接口、legacy 运行时装配和 controller 侧调用链已经形成闭环，并已通过 `10 s+` ROS takeover 仿真复测。
- 这意味着“先迁编排、再迁实现”的本轮策略已经落地完成；后续只需继续做参数类型、目录布局和长期测试边界的收整。

## 14. 下一步建议

1. 继续把 PID、LQR、VMC 的参数结构和公共类型从 `transplant/tools/` 剥离出来。
2. 再把 PID、LQR、VMC 的参数结构与初始化配置迁入 `wheel_leg_control` 或 `wheel_leg_common` 下的长期目录。
3. 最后逐步把 `tools/*.cc` 的实现迁入正式工程结构并缩减 `transplant/` 依赖面。
