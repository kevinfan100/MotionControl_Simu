# Task 1c Report: IIR Bias Correction Integrated and Verified

**Branch**: `feat/sigma-ratio-filter`
**Date**: 2026-04-12
**Method**: 30 sec free-space Simulink run (`lc=0.7`, `meas_noise_enable=false`,
positioning trajectory). Same seed and config as Phase 2A.

## 1. One-line result

The Task 1b `IIR_bias_factor` correction is now integrated into
`motion_control_law_7state` via a 1-D `(lc,)` lookup. A fresh 30 sec Simulink
verification shows `a_m_z / a_nom = 1.0137` (was 0.9193 uncorrected in the same
run, 0.9250 in the pre-correction Phase 2A baseline). The residual +1.37%
over-correction on z is the expected outcome of the ~1 pt mismatch between the
`Σ_aug` analytic ρ and sample ρ (see Task 1b).

## 2. Integration summary

| file | change |
|------|--------|
| `test_script/build_bias_factor_lookup.m` | new: 1-D `(lc,)` sweep, closed-form from Task 1b |
| `test_results/verify/bias_factor_lookup.mat` | new: `bias_factor_tab`, `rho_tab`, metadata |
| `model/controller/calc_ctrl_params.m` | loads lookup, sets `ctrl.IIR_bias_factor`, default 1.0 if missing |
| `model/calc_simulation_params.m` | CtrlBus element 28 `IIR_bias_factor` |
| `model/controller/motion_control_law_7state.m` | persistent `IIR_bias_factor_const`, `del_pmr_var / IIR_bias_factor_const` before noise subtraction |
| `test_script/verify_task1c_correction.m` | new: 30 sec Simulink verification with before/after a_m |
| `reference/for_test/fig_bias_factor_lookup.png` | new: bias_factor vs lc + ρ(L) curves |
| `reference/for_test/fig_task1c_correction.png` | new: a_m time series + histogram + bar chart |

## 3. Lookup table (1-D over `lc`, `a_prd = 0.05` fixed)

| lc   | bias_factor | white-only | correction ×    |
|-----:|------------:|-----------:|----------------:|
| 0.40 | 0.9363      | 0.9744     | 1.0681          |
| 0.50 | 0.9300      | 0.9744     | 1.0752          |
| 0.60 | 0.9210      | 0.9744     | 1.0857          |
| **0.70** | **0.9069**  | 0.9744     | **1.1027**      |
| 0.80 | 0.8810      | 0.9744     | 1.1351          |
| 0.90 | 0.8190      | 0.9744     | 1.2211          |

Reference point `lc=0.7` matches Task 1b's 0.9069 to 4 decimal places
(gate PASS). Correction grows monotonically with `lc` — tighter control pole
→ higher `del_pmr` autocorrelation → larger bias amplification.

## 4. Closed-form correction applied in the controller

```
a_m = (del_pmr_var / IIR_bias_factor - C_np_eff * σ²_n) / (C_dpmr_eff · 4 k_B T)
```

Where `del_pmr_var` is the raw IIR estimate `max(E[del_pmr²] - E[del_pmr]², 0)`.
Dividing by `IIR_bias_factor` unbiases it **before** the noise subtraction and
the thermal scaling.

Caveat (for non-zero `σ²_n`): the `IIR_bias_factor` is computed from the
**thermal** autocorrelation alone (`Σ_th`). Measurement noise injected through
`B_np` produces a different autocorrelation, so in theory a separate
`IIR_bias_factor_np` should scale the `C_np_eff * σ²_n` subtraction. For
Phase 2A (`meas_noise_enable = false`) this is exact. Extending to the noisy
case is deferred (Task 1d if needed).

## 5. Phase 2A verification results (30 sec, same seed as baseline)

### 5.1 Mean of `a_m / a_nom`

| axis | uncorrected | corrected | Δ bias          |
|-----:|------------:|----------:|----------------:|
| x    | 0.9087      | 1.0020    | -9.13% → +0.20% |
| y    | 0.9138      | 1.0076    | -8.62% → +0.76% |
| **z** | **0.9193** | **1.0137** | **-8.07% → +1.37%** |

Bias magnitude drops ~**6×** on every axis. All three axes land within
`[0.98, 1.02]`.

### 5.2 Relative std (chi-squared floor)

z-axis: `std / mean = 44.95%` **in both** uncorrected and corrected runs.
This is expected — the correction is a pure scalar multiplier on `a_m`, so the
relative std is invariant. The chi-squared floor is not affected; reducing it
is a different (Task 2) problem.

### 5.3 Why uncorrected z shifts from 0.9250 (Phase 2A baseline) to 0.9193

Same seed, same trajectory, but the **controller itself is different** (now
uses corrected `a_m → a_hat`), so the closed-loop feedback path produces a
slightly different `del_pmr` time series. The ~0.6% shift between 0.9250 and
0.9193 comes from that second-order feedback effect. Task 1b's offline
analysis (using the pre-correction `del_pmr`) predicted 0.9162; the Simulink
rerun's uncorrected value of 0.9193 is within the expected envelope.

### 5.4 Why corrected z lands at 1.0137, not exactly 1.0

Task 1b found that the analytic `Σ_aug` ρ decays slightly slower than the
sample ρ at lags 4-5, leading the closed-form bias prediction to **over-predict**
bias by about 1 pt (`0.9069` theory vs `0.9162` sample at `lc=0.7`).
That over-prediction becomes a residual **over-correction** of the same
magnitude here: `1.0137 − 1.000 = 1.37%`, which is within the 1-2% envelope
predicted by Task 1b. No new phenomena.

### 5.5 Gate

`mean(a_m_z) / a_nom = 1.0137 ∈ [0.98, 1.02]` → **GATE G1 PASS**.

## 6. Decision

- **Task 1c is complete.** The offline-built `bias_factor_lookup.mat` plus the
  one-line controller modification eliminate 85% of the residual Phase 2A a_m
  bias (8.07% → 1.37%).
- The correction is **integration-tested end-to-end**: lookup → `calc_ctrl_params`
  → `CtrlBus` → controller persistent → runtime `a_m` computation → EKF `a_hat`
  feedback loop → closed-loop dynamics. No Bus errors, no numerical blow-up.
- The remaining +1.37% is **expected** and quantitatively accounted for by
  Task 1b's known ~1 pt `Σ_aug` over-prediction. It is not a new bug.

## 7. Next (proposed Task 1d — optional polish)

1. **Near-wall dynamic benchmark**: rerun the paper's 1 Hz / 9 µm oscillation
   scenario with correction enabled. See if `a_hat` median error drops from
   ~17% toward paper's 5-10%. The chi-squared floor stays, so the expected
   improvement is primarily in mean bias.
2. **Optional polish: 2-pole AR fit of ρ** to kill the remaining +1.37% overshoot.
   Unclear whether the gain is worth the extra complexity — wait for Task 1d
   result to decide.
3. **Optional: two separate bias factors for thermal vs noise** if the paper
   benchmark scenario uses non-zero measurement noise.

## 8. Artifacts

| artifact                                                    | content |
|-------------------------------------------------------------|---------|
| `test_script/build_bias_factor_lookup.m`                    | build 1-D lookup |
| `test_script/verify_task1c_correction.m`                    | 30 sec Simulink verification |
| `test_results/verify/bias_factor_lookup.mat`                | 1-D lookup (gitignored) |
| `test_results/verify/task1c_verification.mat`               | verification data (gitignored) |
| `reference/for_test/fig_bias_factor_lookup.png`             | bias_factor vs lc + ρ(L) |
| `reference/for_test/fig_task1c_correction.png`              | before/after a_m on z axis |
| `reference/for_test/task1c_report.md`                       | this report |
| `model/controller/calc_ctrl_params.m`                       | +24 lines (load lookup, set `IIR_bias_factor`) |
| `model/calc_simulation_params.m`                            | +2 lines (CtrlBus element 28) |
| `model/controller/motion_control_law_7state.m`              | +12 lines (persistent + init + unbias before noise subtraction) |
