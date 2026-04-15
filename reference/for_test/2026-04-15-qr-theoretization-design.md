# Q/R Theoretization — Design Doc

**Date**: 2026-04-15
**Branch (planned)**: `test/qr-paper-reference` (from `feat/sigma-ratio-filter`)
**Status**: Design, pending user approval → writing-plans

---

## 1. Background

### 1.1 Current state

The 7-state EKF in `model/controller/motion_control_law_7state.m` is a faithful
implementation of Meng et al., IEEE TIE 72(1), 2025 (near-wall motion control
paper), Eq.(14)-(21). State update (Eq.16), error dynamics `F_e` (Eq.18), and
covariance propagation (Eq.21) all match.

However the `Q` and `R` matrices in `model/config/user_config.m` deviate
from the paper:

```
Qz_diag_scaling = [0, 0, 1e4,   1e-1, 0, 1e-4, 0]
Rz_diag_scaling = [1e-2, 1.0]
```

Paper specifies (process noise from Eq.17, measurement variance from Eq.11-13):
- `Q(3,3) = Var(a*f_T) = sigma2_dXT` (scale = **1**, not 1e4)
- All other `Q` entries = **0** (no disturbance state process noise in paper)
- `R(1,1), R(2,2)` given by closed-form formulas derived from thermal + sensor
  noise interaction with IIR HP filter

The code's values are empirical tuning results, selected so that the EKF
stays stable in the near-wall regime (`h_bar in [2.5, 20] um`). Prior attempts
to use "physical Q" reportedly diverged (noted in `project_sigma_ratio_filter.md`).

### 1.2 Why this matters

1. **Writeup Section 3** (EKF introduction) currently cannot justify Q/R as
   anything other than "empirical tuning". Supervisor-facing document needs
   theory-backed values.
2. **Task 1d** diagnostic showed `a_hat` static rel std = 26% is a hard floor
   determined by current `(Q, R, a_cov)`. An MC sweep in Phase 1 found
   `(Q33=100, Q44=1e-2, R22=100)` gives 18% but diverges near wall.
3. The existence of a *theoretical* Q/R (from paper) was never tested directly
   in this simulation. The "physical Q diverges" claim is secondhand.

---

## 2. Goals

1. **Understand**: Map each `Q_ii` and `R_ii` entry to the physical quantity
   it represents in the paper's 7-state model. Identify which entries are
   theoretical and which are tuning.
2. **Verify**: Run the paper's theoretical Q/R values directly in this
   simulation and observe stability + performance vs current tuning.
3. **Decide**: Based on verification outcome, either
   (a) switch to paper values, or
   (b) justify incremental tuning from the theoretical baseline, or
   (c) document why the theoretical setup cannot work in this simulation.

Non-goal: re-tuning via MC sweep. This is a theory vs experiment comparison,
not a search for a better operating point.

---

## 3. Plan: Step (b) -> Step (a) -> Step (c)

### Step (b) — Paper R derivation (pure math, no code)

**Deliverable**: `reference/for_test/qr_theoretical_values.md`

**Actions**:
1. Read paper page 932, Eq.(11)-(13) carefully.
2. Write out the closed-form expressions for:
   - `sigma^2_{dx_m}[k]` (variance of the delayed-tracking measurement)
   - `sigma^2_{a_m}[k]` (variance of the IIR-derived gain measurement)
3. Translate paper notation to code notation (`lambda_c, a_nom, sigma2_dXT,
   sigma2_n`).
4. Substitute current parameter values:
   - `lambda_c = 0.7`
   - `sensor noise std = 0.01 um` -> `sigma2_n = 1e-4 um^2`
   - `sigma2_dXT = 4*k_B*T*Ts/gamma_N` (numerical value from `physical_constants.m`)
   - `a_cov, a_prd = 0.05`
5. Compute the numerical values of `R(1,1)_paper` and `R(2,2)_paper`, both
   in absolute units and as `R_scale_paper = R_paper / sigma2_dXT`.
6. Tabulate against current code values:

```
             Paper theory      Code current     Ratio
R(1,1)       ???*sigma2_dXT    1e-2*sigma2_dXT  ?
R(2,2)       ???*sigma2_dXT    1.0*sigma2_dXT   ?
```

**Acceptance**: User reviews `qr_theoretical_values.md` before Step (a).

**Estimated time**: 30-60 min.

**Risk**: Paper may present formulas for a specific configuration (e.g. single-
layer IIR, ignoring autocorrelation). If so, must cross-reference with our
Task 1b derivation of `beta` and reconcile. Worst case: paper formulas are
a subset; we extend them consistently.

---

### Step (a) — Simulink experiment with paper Q/R

**Prerequisites**: Step (b) complete and approved. Branch created.

**Dependency chain** (this is non-trivial and must be scripted carefully):

```
change Qz_diag_scaling / Rz_diag_scaling
  |
  v  DARE resolved with new Q/R -> new L_ss
  |
  v  A_aug changes -> Sigma_aug changes
  |
  v  cdpmr_eff_lookup.mat must be rebuilt
  |
  v  bias_factor_lookup.mat must be rebuilt (Task 1c beta depends on
     Sigma_aug autocorrelation rho)
  |
  v  Controller a_m reconstruction uses new C_dpmr_eff + new beta
  |
  v  Run Simulink test
```

**Script sequence**:

```
matlab> run test_script/build_cdpmr_eff_lookup   % rebuild with new Q/R
matlab> run test_script/build_bias_factor_lookup % rebuild with new Sigma_aug
matlab> run test_script/analyze_task1d_ahat_static  % main test
```

**Primary test**: `analyze_task1d_ahat_static.m`
- Scenario: 30s free-space positioning, lc=0.7, seed 12345, thermal ON,
  measurement noise OFF
- Matches baseline `test_results/verify/task1d_static_ahat.mat` exactly
- Produces: `a_hat_x/z rel std`, `autocorrelation time constant`, EKF
  smoothing factor, `V_IIR/V_theory` ratio

**Secondary test** (only if primary passes): `verify_task1d_paper_benchmark_mc.m`
- Scenario: lc=0.4, 1 Hz 2.5 um amplitude, 10 cycles, 10 seeds, near-wall
  dynamic
- Matches baseline `test_results/verify/task1d_paper_benchmark_mc.mat`
- Produces: 3D tracking RMSE, `a_hat_z` median error, cross-correlation lag

**Comparison table template**:

```
                       Baseline (tuning)     Paper (theoretical)
a_hat_z rel std        26.01 %               ?
a_hat_x rel std        19.14 %               ?
tc(a_hat_z)            84 samples            ?
V_IIR_z / V_theory     1.0137                ?
3D RMSE (dynamic)      40.9 nm               ?
a_hat_z median (dyn)   17.88 %               ?
a_hat_z lag (dyn)      31 ms                 ?
```

**Three-result decision tree**:

- **R1: Paper Q/R stable AND a_hat rel std not worse**
  -> Switch to paper values. Writeup Section 3 gets theoretical derivation.
     Skip Step (c). Reject the "physical Q diverges" claim as mis-tuning.
- **R2: Paper Q/R stable BUT rel std worse or dynamic lag worse**
  -> Quantify gap. Incremental Q scaling (1x -> 10x -> 100x) to find
     inflection point. Step (c) explains the inflation in physical terms.
- **R3: Paper Q/R diverges or produces NaN**
  -> Confirm "physical Q insufficient". Enter Step (c) to decompose why.
     Candidates: initial P matrix, R(1,1) from sensor spec vs paper formula,
     disturbance state as mandatory modeling element.

**Estimated time**: 1-2 hours (3 scripts + analysis).

**Risks**:
- Rebuild scripts may take longer than expected if DARE is ill-conditioned
  with the new Q/R (iterative Riccati has 30k max iters).
- If paper R(2,2) is much larger than 1.0, `a_m` measurement gets heavily
  discounted and EKF becomes essentially open-loop on gain -> may look
  like divergence but is just slow convergence.

---

### Step (c) — Disturbance state design intent (only if Step a needs it)

**Conditional on Step (a) result R2 or R3.**

**Actions**:
1. Re-read paper page 929-931 (Section II, plant model) for the motivation
   behind `x_D, delta_xD` states.
   - Is `x_D` defined as physical unknown disturbance (external force,
     actuator drift, creep)?
   - Or as a modeling-error sink (absorbing unmodeled dynamics)?
2. Compare against our simulation:
   - `system_model.slx` contains: thermal force, wall effect, sensor
     delay, sensor noise. No drift, no DC offset.
   - Does our simulation have anything that needs absorbing?
3. Branch decision:
   - **Disturbance state is physical** AND our simulation has
     corresponding source: derive `Q(4,4)` from physical bandwidth.
   - **Disturbance state is physical** AND our simulation has no source:
     `Q(4,4) = 0` is correct; disturbance state is redundant.
   - **Disturbance state is modeling sink**: `Q(4,4) = 0` unless we can
     quantify the modeling gap (linearization error, wall-effect nonlinearity).
4. Same analysis for `a, delta_a` random-walk states:
   - Estimate `da/dh * dh/dt` from trajectory
   - Take max `|delta_a|` per step squared as physical `Q(6,6)` upper bound

**Deliverable**: `reference/for_test/qr_design_rationale.md` — final
per-entry justification table for Q and R.

**Estimated time**: 30-90 min.

---

## 4. Branching and file hygiene

**Branch**: `test/qr-paper-reference`, created from `feat/sigma-ratio-filter`
HEAD `f1aa82d`.

**Files to change**:
- `model/config/user_config.m` — new Qz_diag_scaling, Rz_diag_scaling
- `test_results/verify/cdpmr_eff_lookup.mat` — regenerated
- `test_results/verify/bias_factor_lookup.mat` — regenerated
- Test script adjustments only if needed (ideally none)

**Files NOT to change**:
- Anything in `feat/sigma-ratio-filter` working tree (`writeup_architecture.tex`
  has uncommitted Sections 1-6 overhaul — do NOT touch)
- Any Simulink `.slx` file

**Output files**:
- `reference/for_test/qr_theoretical_values.md` (Step b)
- `reference/for_test/task_qr_reference_report.md` (Step a results)
- `reference/for_test/qr_design_rationale.md` (Step c, conditional)
- Optional figures: `reference/for_test/fig_qr_paper_vs_tuning_*.png`

**Merge policy**: Do NOT merge `test/qr-paper-reference` back to
`feat/sigma-ratio-filter` automatically. After experiment, if results
justify adoption, open a separate PR against the appropriate integration
branch.

---

## 5. Baseline data (already available)

| Baseline | File | Key values |
|---|---|---|
| Static (free-space, lc=0.7) | `test_results/verify/task1d_static_ahat.mat` | a_hat_z rel std=26.01%, a_hat_x=19.14%, tc=84 |
| Dynamic (near-wall, lc=0.4) | `test_results/verify/task1d_paper_benchmark_mc.mat` | z median err=17.88%, lag=31ms, 3D RMSE=40.9nm |

These will be loaded as reference in `task_qr_reference_report.md`.

---

## 6. Time estimate

| Step | Activity | Time |
|---|---|---|
| (b) | Paper R derivation | 30-60 min |
| (a) | Simulink experiment | 1-2 h |
| (c) | Disturbance state analysis (conditional) | 30-90 min |
| Total | | Half day to one day |

---

## 7. Success criteria

- Step (b) deliverable is accepted by user.
- Step (a) runs to completion without script errors.
- A decision table is produced mapping each `Q_ii` and `R_ii` entry to
  either a theoretical value, an incremental tuning with justification,
  or an explicit "cannot derive theoretically" note.
- Writeup Section 3 has source material for at least one of:
  (a) a fully theoretical Q/R with citation to paper Eq.(11-17),
  (b) a theoretical baseline plus documented incremental tuning,
  (c) a documented negative result explaining why theoretical Q/R fails.

---

## 8. Out of scope

- Q/R Monte Carlo optimization or gradient search
- Adaptive Q/R (innovation matching)
- ℋ∞ robust filter redesign
- Changing `F_e` structure or EKF state layout
- Changing IIR variance estimator architecture
