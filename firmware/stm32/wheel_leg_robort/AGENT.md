# Kinesiology Migration Plan

## Goal

Move the reusable control and math code into `Kinesiology/` and keep
`Hardware/` focused on board devices and drivers. The migrated control layer
must build without MuJoCo or desktop-only dependencies.

## Scope

- Move `Hardware/alg_basic.h`, `Hardware/alg_basic.cpp`,
  `Hardware/alg_pid.h`, and `Hardware/alg_pid.cpp` into `Kinesiology/`.
- Use the existing `Class_PID` implementation as the project PID API.
- Transplant only `sensor`, `vmc`, and `lqr_k` from `transplant/`.
- Do not transplant `transplant/pid`, `actuator`, `plotter`, or `ekf` in this
  pass.

## File Plan

| Source | Destination | Notes |
| --- | --- | --- |
| `Hardware/alg_basic.*` | `Kinesiology/alg_basic.*` | Preserve existing math helpers, including deg/rad conversion. |
| `Hardware/alg_pid.*` | `Kinesiology/alg_pid.*` | Preserve `Class_PID`; do not add `PIDInstance`. |
| `transplant/lqr_k.*` | `Kinesiology/lqr_k.*` | Pure math transplant, no board dependency. |
| `transplant/vmc.*` | `Kinesiology/vmc.*` | Pure math transplant, no board dependency. |
| `transplant/sensor.*` | `Kinesiology/sensor.*` | Keep structures and kinematics; replace MuJoCo input with board-side explicit input. |

## Implementation Flow

1. Move `alg_basic` and `alg_pid` into `Kinesiology/`.
2. Add board-safe `lqr_k`, `vmc`, and `sensor` modules under `Kinesiology/`.
3. Update project include paths so `#include "alg_basic.h"` and
   `#include "alg_pid.h"` resolve from `Kinesiology/`.
4. Update the Keil project file:
   - Add `../Kinesiology` to `IncludePath`.
   - Remove `alg_basic.*` and `alg_pid.*` from the `Hardware` group.
   - Add a new `Kinesiology` group with the migrated and transplanted files.
5. Confirm no board build source includes MuJoCo, `iostream`, `simulate.h`, or
   `transplant/pid`.

## API Decisions

- PID API: keep `Class_PID`.
- LQR/VMC namespace: keep `wheel_leg`.
- LQR state vector order remains
  `[phi, phi_rate, distance, velocity, pitch, pitch_rate]`.
- Sensor input is explicit board data, not `mjModel` or `mjData`.
- Sensor history is stored in `SensorState` and reset with
  `ResetSensorState()`.

## Verification

- Search for stale `Hardware/alg_basic.*` and `Hardware/alg_pid.*` references.
- Search `Kinesiology/` for MuJoCo and desktop-only dependencies.
- Check the Keil project XML includes `Kinesiology` sources and no old
  `Hardware` algorithm entries.
- If the MDK toolchain is available, build the project and verify
  `dvc_motor_dji` and `App/Car` still compile.
