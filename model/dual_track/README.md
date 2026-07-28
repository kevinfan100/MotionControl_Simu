# model/dual_track/

Pure-MATLAB simulation driver that runs in parallel to the Simulink path
(`model/system_model.slx`). Both paths share the same physics model and EKF
state layout; the difference is integration (MATLAB-side ode4 loop vs
Simulink fixed-step solver) and **controller selection**:

- 本 driver **不經 `motion_control_law` dispatcher**。它以
  `config.eq17_variant` 字串分支：`'4state' | '5state' | '6state'`
  → `motion_control_law_eq17_{4state,5state,6state}`；未設 variant 時走
  7-state `motion_control_law_eq17_core`。
- `controller_type` ∈ {6, 17, 23}（eq6／eq17 wrapper／23-state legacy）
  只能在 **Simulink** track 跑（`test_script/run_simulation.m`）。
- 估測函數主線（powerlaw／expgain）不在本 driver：走
  `test_script/integration/run_5state_powerlaw.m`／`run_5state_expgain.m`
  （鏡像本 driver 的 step ordering，硬派發各自 controller）。

## Contents

- `step_dynamics.m` — single-step physics integrator (ode4 RK4)。
  controller-agnostic：吃力輸入，推進 plant state。
- `run_pure_simulation.m` — full-loop driver（ParamsBus → 每步 controller →
  軌跡與 EKF 診斷累積 → Simulink ToWorkspace 相容的 result struct）。

## History

2026-07-28 整理：移除 9 個 TEMP variant 死分支（指向已刪除的
`temp_motion_control_law_eq17_4state_*`）與已否證線的分支
（`'5state_aprime'`、`'gscalar'`，controller 已下沉
`model/controller/archive/`，舊路徑對照見
`reference/eq17_analysis/archive/MOVED.md`）。

Exercised by: `test_script/unit_tests/verify_eq17_unit_step_dynamics.m`
與 `test_script/integration/verify_eq17_4state.m` 等。
