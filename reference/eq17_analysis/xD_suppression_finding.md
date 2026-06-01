# x_D Suppression Study — Finding

> Concise summary. Diagnostic plots in `figures/suppress_xD_ramp_acov005/`, raw study drivers archived under `archive/sessions/suppress_xD_study/`.

## Hypothesis being tested

Under the Eq.17 7-state EKF, is the x_D state (slot 4) functioning as a **legitimate disturbance estimator** or as a **silent thermal-noise compensator** (effectively soaking up high-frequency noise the IIR isn't catching)?

If x_D is acting as thermal compensator, then **forcing x_D_hat_for_ctrl = 0 in the control law** (while still letting the EKF estimate x_D normally) should visibly degrade a-tracking because the EKF will not have a degree of freedom to "park" thermal noise into. If x_D is a true disturbance estimator, suppressing it from the control law should have minimal effect.

## Setup

- Scenario: ramp_descent trajectory, h: 50 → 5 over 20 s (z axis)
- a_pd = 0.05, a_cov = 0.005 (current scenario tuning)
- EKF runs unchanged; only the control law line `f_d = ... - x_D_hat / a_hat` is gated by `ctrl_const.suppress_xD`
- Driver: `temp_suppress_xD_ramp_apd05_acov005.m`
- Diagnostics: `temp_a_known_vs_a_hat.m`, `temp_ramp_diag_compare.m`, `temp_ramp_dynamic_diagnosis.m`

## Result

`xD_off.png` vs `xD_on.png`: a_x and a_z tracking are visually indistinguishable across the entire 20s ramp. Both estimated traces (red) sit on the true trace (green) with comparable jitter; the suppression toggle does not perceivably shift mean, variance, or lag behavior.

`diagnostic/z_diagnostic_4panel.png` confirms in detail:
- Panel 1 (a_z): KF (red) and truth (green) overlap, raw IIR (blue) noisier as expected
- Panel 2 (residuals): a_KF − a_true and a_IIR − a_true both small, theoretical IIR lag bias prediction matches observed bias
- Panel 3 (δa_z velocity state, slot 7): essentially flat near 0 throughout (forced by `force_Q77_zero=true`, expected)
- Panel 4 (h(t) ramp + K_h amplification near wall): K_h staying small until late ramp (h_bar ≥ 2 throughout most of the run); wall-region stress is not the main scenario

## Conclusion

In the tested ramp 50→5 / a_cov=0.005 scenario, **x_D in the control law is NOT acting as a silent thermal compensator** — removing it has no observable effect on a tracking. This means the EKF is correctly partitioning thermal noise into the IIR-derived a_m measurement noise budget (R(2,2) finite-alpha term), not silently leaking it into x_D.

By implication: the Eq.17 architecture (x_D as additive disturbance compensation) is doing the job it was designed for under positioning + slow ramp. The x_D term can stay in the control law for canonical eq17 deployments.

## Caveats / scope

- Tested only in the descent ramp regime where K_h stays small for most of the run. Near-wall (h_bar < 2) where K_h amplifies sharply was not directly probed by this study.
- Tested with `force_Q77_zero=true` (current production setting per `project_q77_zero_optimal.md`). Behaviour with non-zero Q77 floor may differ.
- Single-seed visual; not a multi-seed quantitative comparison. If a tighter quantification is needed, see `temp_a_known_vs_a_hat.m` for the raw .mat overlays.

## Files committed with this study

- `figures/suppress_xD_ramp_acov005/xD_off.png`, `xD_on.png` — top-level comparison
- `figures/suppress_xD_ramp_acov005/diagnostic/` — 9 PNG + 3 .mat detailed diagnostics
- `archive/sessions/suppress_xD_study/` — 4 driver scripts
