# IIR Prefill — t_warmup_kf Minimum-Stable Sweep (Agent D)

Date: 2026-05-05
Branch: test/eq17-5state-ekf

## Setup

- Configuration (per simulation):
  - trajectory_type='positioning', amplitude=0, frequency=0, t_hold=0
  - h_init=h_bottom=50 um, T_sim=15 s, lambda_c=0.7, controller_type=23
  - a_cov=0.05, a_pd=0.05, sigma2_w_fA=0, sigma2_w_fD=0
  - meas_noise_enable=true, meas_noise_std=[0.62e-3; 0.057e-3; 3.31e-3] um
  - thermal_enable=true, ctrl_enable=true
  - **iir_warmup_mode='prefill'** (key control)
  - **a_hat_freeze=[]** (no freeze, default)
- Sweep variable: t_warmup_kf in {0, 0.025, 0.05, 0.1, 0.2} s
- Seeds: {1, 2, 3} -> 5 x 3 = 15 sims, total wall time 7.2 min
- Metrics computed over steady-state region t >= 5 s

## Code modification

To allow per-run override of t_warmup_kf without touching motion_control_law / build_eq17_constants / user_config, a single passthrough hook was added in `model/pure_matlab/run_pure_simulation.m` after line 86:

```matlab
eq17_opts.t_warmup_kf = 0.2;
if isfield(config, 't_warmup_kf') && ~isempty(config.t_warmup_kf)
    eq17_opts.t_warmup_kf = config.t_warmup_kf;   % Agent D sweep override
end
```

Default behavior (no `config.t_warmup_kf` field) is unchanged; existing run_pure_simulation callers stay binary-identical. **Modification was NOT reverted** — left in place per task instructions for human review (NOT committed).

## Sweep table

| t_warmup_kf [s] | steps @ Ts=1/1600 | stable | mean RMSE_z [%] | mean trk_x [nm] | mean trk_z [nm] | mean min(a_hat_z) | mean max\|f_d\| [pN] | Wave 4 |
|---:|---:|:--:|---:|---:|---:|---:|---:|:--:|
| 0.000 | 0   | 3/3 | 0.0599 | 30.73 | 29.80 | 9.016e-03 | ~2.34 | 0 |
| 0.025 | 40  | 3/3 | 0.0596 | 30.31 | 29.83 | 1.115e-02 | ~2.42 | 0 |
| 0.050 | 80  | 3/3 | 0.0614 | 30.74 | 30.69 | 9.304e-03 | ~2.18 | 0 |
| 0.100 | 160 | 3/3 | 0.0604 | 30.56 | 30.22 | 1.038e-02 | ~2.40 | 0 |
| 0.200 | 320 | 3/3 | 0.0611 | 30.49 | 30.27 | 9.905e-03 | ~2.32 | 0 |

(Baseline: t_warmup_kf=0.2 gives RMSE_z=0.0611%; all other values within +/- 0.002 pp -> NO DEGRADED case.)

## Stability analysis

- **All 15 sims stable.** No NaN/inf, no a_hat <= 0 (no Wave 4 mode), all max|f_d| <= ~3 pN (well under 100 pN threshold).
- min(a_hat_z) stayed in 8.2e-03 .. 1.2e-02 across all conditions (vs nominal a_z ~ 1.4e-2).
- Earliest min(a_hat_z) location:
  - t_warmup_kf=0: ~0.017-0.029 s (first ~30-50 steps), but still positive and recovers immediately.
  - t_warmup_kf=0.025-0.20: typically 0.05-0.7 s — natural transient, not Wave 4.
- **Wave 4 mode (P_pred(3,6) cross-cov runaway, K_kf(6,1) -> a_hat negative) was NOT observed even at t_warmup_kf=0** with prefill mode active.

This confirms the hypothesis is wrong on one point and right on another:
- Wrong: Stage 10 Option A G1 was thought necessary independent of IIR. Empirically, with prefill providing a non-degenerate σ²_dxr_hat from step 1, the a_hat slot stays well-bounded even with no KF guard.
- Right: removing the IIR-derived justification (sigma2_dxr_hat unreliable) leaves no failure mode in this scenario.

Per-seed details (no UNSTABLE / DEGRADED cases — full per-run records in `iir_prefill_twarmup_sweep.mat`):

```
[OK] tw=0.000 seed=1  min_az=9.28e-03 @ t=0.029s  RMSE_z=0.060%  trkX=30.23nm trkZ=29.79nm  maxF=2.24pN
[OK] tw=0.000 seed=2  min_az=8.87e-03 @ t=0.027s  RMSE_z=0.058%  trkX=30.87nm trkZ=29.19nm  maxF=2.24pN
[OK] tw=0.000 seed=3  min_az=8.90e-03 @ t=0.017s  RMSE_z=0.061%  trkX=31.11nm trkZ=30.42nm  maxF=2.53pN
... (all other rows similar within run-to-run noise; see .mat)
```

## Recommendation

- **Smallest STABLE t_warmup_kf: 0.000 s** (G1 disabled entirely).
- **Smallest STABLE-and-not-DEGRADED t_warmup_kf: 0.000 s** (RMSE_z is flat across the sweep within +/- 0.002 pp).

With prefill mode active, the KF warm-up guard G1 is **not required** for h=50 positioning under the tested conditions. The Stage 10 Option A reasoning (P_pred(3,6) cross-cov buildup -> K_kf(6,1) drives a_hat negative around step ~120) does NOT manifest when σ²_dxr_hat is prefilled to its near-steady-state value, because the y_2 channel R(2,2) is correctly sized from step 1 and the KF gain stays in a sensible regime.

**Caveat**: this finding is scoped to h=50 positioning, lambda_c=0.7, T_sim=15 s. It does NOT cover:
- Motion / ramp scenarios (where prefill correctness depends on IIR tau >> ramp duration).
- Closer wall (h=2.5 / 5 um) where Q77 wall-coupling is large and the cross-cov dynamic is faster.
- Other lambda_c / a_cov choices.
Those should be checked before claiming t_warmup_kf=0 is universally safe.

## Verdict

PASS. With `iir_warmup_mode='prefill'`, `t_warmup_kf=0` is stable across all 3 tested seeds at h=50 positioning, with tracking std and a_hat std indistinguishable from the t_warmup_kf=0.2 baseline. Default could be reduced from 0.2 to a small value (e.g., 0.025 s = 40 steps) for a conservative belt-and-braces margin without measurable cost; or to 0 if the team is willing to rely on prefill alone for KF transient suppression in positioning scenarios.

## Files

- `reference/eq17_analysis/iir_prefill_twarmup_sweep.mat` — per-(t_warmup_kf, seed) records: full a_hat trajectories (x,z,y), min/max, RMSE, stability flags, plus aggregate struct.
- `reference/eq17_analysis/iir_prefill_twarmup_sweep.md` — this report.
- `model/pure_matlab/run_pure_simulation.m` — UNCOMMITTED edit adding 3-line `config.t_warmup_kf` passthrough (line 86-89). For human review.
