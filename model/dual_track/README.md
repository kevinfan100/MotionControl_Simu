# model/dual_track/

Pure-MATLAB simulation driver that runs in parallel to the Simulink path
(`model/system_model.slx`). Both paths share the same physics model, EKF
state layout, and dispatcher; the only difference is integration method
(MATLAB-side ode4 loop vs Simulink fixed-step solver).

## Contents

- `step_dynamics.m` — single-step physics integrator (ode4 RK4). Pure
  controller-agnostic: takes a force input, advances the plant state.
- `run_pure_simulation.m` — full-loop driver. Reads ParamsBus, calls
  `motion_control_law` dispatcher each step (so it picks up
  controller_type=6/17/23 like Simulink does), accumulates trajectories
  and EKF diagnostics, returns a result struct.

## Cross-controller usage

Both files are designed to be controller-agnostic. They route through
`motion_control_law(del_pd, pd, p_m, params)` which is the same entry
point Simulink uses. Therefore the dual-track verification scope
extends to any controller_type the dispatcher recognizes.

Currently exercised by:
- `test_script/unit_tests/verify_eq17_unit_step_dynamics.m`
- `test_script/integration/verify_eq17_v2_h50_e2e.m`
- Other `verify_eq17_*` scripts under `test_script/integration/`

Future `verify_eq6_*` integration scripts can re-use `run_pure_simulation`
without modification — just set `config.controller_type = 6` before
calling `calc_simulation_params`.

## Why the rename from `pure_matlab/`

The previous name `pure_matlab/` was a generic placeholder that obscured
the intent. `dual_track` makes the relationship to Simulink explicit:
this is the second of two simulation entry points kept in sync.

## Caveats

- `step_dynamics` performance is much slower than Simulink for long
  simulations; intended for verification / unit-test scope, not
  production sweeps.
- Trajectory generator is called inline (no separate Simulink block);
  any change to `model/trajectory/` may affect this path too.
