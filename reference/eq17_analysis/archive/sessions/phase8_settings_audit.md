# Phase 8 Settings Audit — feat/sigma-ratio-filter Branch

**Date**: 2026-04-29
**Branch audited**: `feat/sigma-ratio-filter` (positioning h=50 paper-level result)
**Purpose**: Extract validated settings to inform v2 eq17 implementation (Phase 8) on `test/eq17-5state-ekf` branch
**Mode**: read-only audit; no code changes

**Files audited**:
- `model/controller/motion_control_law_7state.m`
- `model/controller/calc_ctrl_params.m`
- `model/config/user_config.m`
- `model/config/apply_qr_preset.m`
- `model/calc_simulation_params.m`
- `test_script/run_simulation.m`
- `test_script/verify_qr_positioning_run.m`
- `test_script/verify_qr_positioning_aggregate.m`

**Reference reports**:
- `reference/qr_analysis/qr_verification_h50_final_2026-04-27.md` (Stage 4 final)
- `reference/eq17_analysis/design_v2.md`
- `reference/eq17_analysis/phase5_Q_matrix_derivation.md`
- `reference/eq17_analysis/phase6_R_matrix_derivation.md`

---

## Verdict legend
- ✓ 借鑑 — v2 直接沿用
- ⚠ 需調整 — v2 需修改
- ✗ 不適用 — sigma-ratio-filter 特有
- ❓ 待確認 — 需 review

---

## A. Warmup 設定

| Item | Value (sigma-ratio-filter) | v2 plan | Verdict |
|---|---|---|---|
| `warmup_count` (controller-side IIR warm-up) | **2 steps** (P6 fix, Session 7) | per P6 ⇒ 2 | ✓ 借鑑 |
| `f_d` during warmup | **= 0** (early return inside controller) | same | ✓ 借鑑 |
| IIR LP / mean / variance update during warmup | **YES** (delpmd, delpmrd, del_pmr2_avg all updated each step) | same | ✓ 借鑑 |
| EKF update during warmup | **NO** (skipped via early return) | same | ✓ 借鑑 |
| End-of-warmup seeding | YES — `del_p1_hat = del_p2_hat = del_p3_hat = del_pmd` (the IIR LP estimate) on the last warmup step | same | ✓ 借鑑 |
| Pre-fill of `del_pmr2_avg` (P5) | YES — `Cdpmr_eff·sigma2_dXT·(a_axis/a_nom) + Cnp_eff·sigma2_n` per axis at step 0 | should keep, value comes from Lyapunov; v2 has same form | ✓ 借鑑 |
| Pre-fill floor | `max(..., 0.01·sigma2_dXT)` to ensure gate stays open if lookup tiny | same | ✓ 借鑑 |
| Wall-aware `a_hat` init at step 0 (commit `9a0e8db`) | YES — `a_hat = [a_nom/c_para; a_nom/c_para; a_nom/c_perp]` using `h_bar_init = max(h_init/R, 1.001)` | v2 must mirror this for the 5-state `a_x` slot (Phase 5 §6 requires `â_x[0] ≠ a_nom` for near-wall) | ✓ 借鑑 (with state-index remap) |
| Initial values of EKF persistent states (other than `a_hat`) | All zeros except `a_hat` | v2 same; for 5-state, slots 1-3 (Δp delay), slot 4 (d), slot 5 (â_x) | ✓ 借鑑 |
| Delay buffers `pd_k1`, `pd_k2` init | both = current `pd` at step 0 | same | ✓ 借鑑 |

**Important note (sigma-ratio-filter Session 7 finding):** The original 320-step (0.2 sec) warmup was **harmful** — during no-control phase the IIR sees noise-only `δp_mr`, decaying its pre-filled value toward sensor-noise floor and biasing `a_m` low. With **P5 pre-fill + P6=2 warmup**, EMA cold-start is eliminated and 5-9% bias goes away. v2 must inherit both fixes together; reverting either alone reintroduces bias.

**Driver-side `t_warmup` (data-analysis warmup, not controller warmup):**
- `run_simulation.m` line 70: `t_warmup = 0.2` sec — used only to **trim plotting/stats windows**, NOT to gate controller behavior
- `verify_qr_positioning_run.m`: Session 7 P1 test override → **`t_warmup = 2` sec** for steady-state stats (see line 21-25, "Session 7 P1 test")
- These are independent of `warmup_count`

---

## B. KF gate 設定

**Important**: sigma-ratio-filter does **NOT** implement a runtime KF gate (no t_warmup_kf, no h_bar_safe, no NaN-guard, no wall-guard). It uses only the warm-up gate (Section A above) and an adaptive R(2,2) for thermal-quiet detection.

| Item | sigma-ratio-filter | v2 plan | Verdict |
|---|---|---|---|
| `t_warmup_kf` runtime gate | **not implemented** | v2 needs this if Phase 6 specifies | ❓ 待確認 |
| `h_bar_safe` wall gate | **not implemented** | v2 needs if eq17 redesign requires | ❓ 待確認 |
| NaN guard | **not implemented** in controller (only post-hoc `r.nan_count = sum(isnan(...))` in verify driver) | v2 should add explicit guard | ⚠ 需調整 |
| Adaptive R(2,2) on thermal-quiet | YES — `if del_pmr_var(i) < var_threshold then r_gain_i = 1e6` | review whether v2 keeps; sigma-ratio-filter author NOTES this is a band-aid: "adaptive R based on chi-squared statistics destabilizes the system because Q and R were co-tuned as a pair" | ⚠ 需調整 |
| `var_threshold` value | `sigma2_deltaXT * 0.001` | candidate for v2 if kept | ✓ 借鑑 (value) |
| Gate-triggered R(2,2) value | **`r_gain_i = 1e6` (NOT 1e10)** when triggered | v2 should pick consistent value | ❓ 待確認 |

**Specific source quote from `motion_control_law_7state.m`** (lines 207-216):
```
% Adaptive R: when del_pmr_var is too small (no thermal excitation),
% inflate gain channel noise to prevent EKF from trusting a_m = 0.
%
% Note: adaptive R(2,2) based on chi-squared statistics was tested but
% destabilizes the system because Q and R were co-tuned as a pair.
% Changing R alone breaks the balance. Proper adaptive R requires
% re-sweeping Q simultaneously (future work).
var_threshold = sigma2_deltaXT * 0.001;
```

This is a **3-line "guard"** triggering only the R(2,2) inflation, not full EKF skip.

---

## C. Pf_init 數值 (per-axis state ordering: [δp1; δp2; δp3; d; δd; a; δa])

**Final preset `frozen_correct` (and the `frozen_correct_peraxis*` family)**:
```
Pf_init_diag = [0; 0; 1e-4; 1e-4; 0; 1e-5; 0]    (7×1)
                δp1 δp2 δp3   d  δd   a  δa
```

| State | Pf_init | Comment | Verdict for v2 |
|---|---|---|---|
| Pf(1,1) δp1 | **0** | Seeded from IIR LP at warmup end → covariance starts confident | v2 (5-state) doesn't have δp1; may need analogous Δx[k-2] slot init = 0 | ✓ 借鑑 (concept) |
| Pf(2,2) δp2 | **0** | same | same | ✓ 借鑑 (concept) |
| Pf(3,3) δp3 | **1e-4** [um²] = 0.01 nm² | First "uncertain" state | v2: same magnitude for Δx̄ | ✓ 借鑑 |
| Pf(4,4) d | **1e-4** [um²] | Disturbance unknown at start | v2: same for d | ✓ 借鑑 |
| Pf(5,5) δd | **0** | Disturbance increment confident-zero start | v2 has no δd (single-d structure under eq17) | ✗ 不適用 |
| Pf(6,6) a | **1e-5** [um²/pN²] | corresponds to **σ_a/a_nom ratio**: `a_nom = Ts/gamma_N ≈ 0.0147` ⇒ σ_a = sqrt(1e-5)/0.0147 ≈ **21.5%** of a_nom (1σ) | v2: keep similar fraction-of-nominal scale, but apply on `a_x_init` not `a_nom` | ✓ 借鑑 (concept) |
| Pf(7,7) δa | **0** | δa truly = 0 in positioning ⇒ confident init | v2 has no δa (Phase 5 keeps Q77 instead of δa state) | ✗ 不適用 |

**Comparison across presets** (in `apply_qr_preset.m`):

| Preset | Pf(3,3) | Pf(4,4) | Pf(6,6) | a_cov |
|---|---|---|---|---|
| `frozen_correct` (FINAL) | 1e-4 | 1e-4 | **1e-5** | 0.005 |
| `frozen_correct_peraxisR11` | 1e-4 | 1e-4 | 1e-5 | 0.005 |
| `frozen_correct_peraxisR11_R22` | 1e-4 | 1e-4 | 1e-5 | 0.005 |
| `empirical` | 1e-4 | 1e-4 | **10·(0.0147)² = 2.16e-3** (≈10× larger) | 0.05 |
| `beta` | 1e-4 | 1e-4 | 2.16e-3 | 0.05 |

**Key observation**: The `frozen_correct` family uses `Pf(6,6) = 1e-5` (much tighter ⇒ trust initial wall-aware `â`), while `empirical` uses 200× looser. Tighter Pf(6,6) only works because **wall-aware init** (commit 9a0e8db) gives a good `a_hat[0]` near the truth.

For v2 5-state: `Pf_init(5,5)` (the `a_x` slot) should be **1e-5** if v2 also seeds `a_x_init = a_nom · c_inv(h_init/R)` (which it must, per design_v2).

---

## D. Q matrix 數值

**`Qz_diag_scaling` layout (9×1, since 2026-04-22):**
```
[Q1; Q2; Q3; Q4; Q5; Q6; Q7_x; Q7_y; Q7_z]
   ↑ shared across 3 axes ↑      ↑ per-axis ↑
```

**The actual Q matrix per axis is then built as:**
```
Q_axis(k,k) = sigma2_deltaXT · Qz_diag_scaling(k)   for k=1..6
Q_axis(7,7) = sigma2_deltaXT · Qz_diag_scaling(6+axis)   per-axis
```

**Final preset `frozen_correct` values:**
```
Qz_diag_scaling = [0; 0; 1; 0; 0; 1e-8; 1e-8; 1e-8; 1e-8]
```

| Element | Scaling | Absolute value | Physical meaning | v2 verdict |
|---|---|---|---|---|
| Q(1,1) δp1 | 0 | 0 | δp1 = δp2[k-1] (pure copy, no noise) | ✓ 借鑑 (matches Phase 5) |
| Q(2,2) δp2 | 0 | 0 | δp2 = δp3[k-1] (pure copy, no noise) | ✓ 借鑑 |
| Q(3,3) δp3 | **1** | **sigma2_deltaXT = 4kBT·a_nom** | Thermal injection on δp3 (= δx̄ in v2 notation) | ✓ 借鑑 |
| Q(4,4) d | 0 | 0 | d = d + δd, δd carries noise → d gets it via injection? Actually 0 is consistent with d as cumulative bias state | ❓ 待確認 (v2 Phase 5 says Q(d) = TermA + TermB, NOT 0) |
| Q(5,5) δd | 0 | 0 | Same logic | ✗ 不適用 (v2 has no δd) |
| Q(6,6) a (all 3 axes) | **1e-8** | sigma2_dXT · 1e-8 ≈ 5e-15 [um²/pN²] | "frozen" — discourages random drift in a | ⚠ 需調整 |
| Q(7,7) δa per axis | **1e-8** (originally) → **0 in Stage 3 (frozen_correct_peraxisR11_R22_Q770)** | This was the Stage 3 fix (2026-04-22) | ⚠ 需調整 (大不同 from v2) |

**Critical conflict with v2 design (Phase 5):**

`sigma-ratio-filter` final preset (Stage 3 fix, 2026-04-22) says **`Q(7,7) = 0` for all axes** because in positioning the true `δa = 0` (cf. final report §"Fix 3: Q(7,7) = 0"):
> "Positioning 下 a 真為常數、del_a 真為 0. Q(7,7) = 1e-8 給 del_a 隨機漂移，經 Fe(6,7) = 1 雙積分放大成 a 漂移（~1/L³）."

**v2 design_v2.md §6 Phase 5** instead derives:
```
Q77,i[k] = Δt⁴ · â_x²[k] · {(K_h² − K_h')² · ḣ⁴/(8·R⁴) + K_h² · ḧ²/(2·R²)}
```
which is **non-zero AND time-varying** during motion (it's the Term A + Term B model that captures wall-induced â variation).

**Reconciliation**:
- For positioning (ḣ = ḧ = 0), v2 Q77 → 0, which **matches** sigma-ratio-filter Stage 3.
- For motion (ḣ ≠ 0), v2 Q77 > 0 reflects real â_x dynamics, while sigma-ratio-filter `frozen_correct` (Q77 = 0) cannot track motion (its `empirical` preset with Q77 = 0 in slots 7-9 but Q66 = 1e-4 is a different patch).

**v2 conclusion**: the **frozen_correct preset matches v2 in the positioning limit** but v2's per-step Q77(k) is required for motion. ⇒ **borrow the positioning verification, but DO NOT lock Q to constants in v2**.

**Q66 conflict**: sigma-ratio-filter has `Q(6,6) = 1e-8` even in `frozen_correct`. v2 design treats `a` as a state with NO process noise (the increment `δa` carries the noise). For 5-state v2 (with no `δa` state), Q(5,5) = a-slot should follow Phase 5 derivation, NOT the legacy `1e-8`. ⇒ ⚠ 需調整.

---

## E. R matrix 數值

**`Rz_diag_scaling` layout (6×1, since 2026-04-21):**
```
[R_pos_x; R_pos_y; R_pos_z; R_gain_x; R_gain_y; R_gain_z]
```

**The actual R matrix per axis is:**
```
R_axis(1,1) = sigma2_deltaXT · Rz(axis)              (position channel)
R_axis(2,2) = sigma2_deltaXT · Rz(3+axis)            (a_m gain channel)
```
With dynamic override: `R_axis(2,2) ← sigma2_deltaXT · 1e6` when `del_pmr_var(axis) < sigma2_deltaXT · 0.001`.

**Stage 4 FINAL preset (`frozen_correct_peraxisR11_R22_Q770`):**
```
Rz_diag_scaling = [1.53e-3; 1.29e-5; 4.35e-2;  0.01881; 0.01881; 0.01784]
                       R_pos              R_gain
```

| Channel | Axis | Scaling | Absolute (sigma²) | sigma std |
|---|---|---|---|---|
| R_pos | x | 1.53e-3 | 1.53e-3 · sigma2_dXT ≈ 3.84e-7 [um²] | **0.62 nm** ✓ matches `meas_noise_std(1) = 0.00062 um` |
| R_pos | y | 1.29e-5 | 3.25e-9 [um²] | **0.057 nm** — verified below |
| R_pos | z | 4.35e-2 | 1.10e-5 [um²] | **3.31 nm** ✓ matches `meas_noise_std(3) = 0.00331 um` |
| R_gain | x | 0.01881 | (chi_sq · rho_a · a_x²) at h=50 | "intrinsic" R(2,2) per Phase 6 |
| R_gain | y | 0.01881 | same as x | (y axis = tangential, c_para like x) |
| R_gain | z | 0.01784 | (chi_sq · rho_a · a_z²) at h=50 | wall-normal axis |

**Y-axis 0.057 nm verification:** Per `verify_qr_positioning_run.m` line 90 and `run_simulation.m` line 63, `config.meas_noise_std = [0.00062; 0.000057; 0.00331]`. **0.000057 um = 0.057 nm — confirmed real, not a typo**. This is the project-wide spec for the lab sensor's tangential-y axis.

**ξ per-axis (computation pattern):**
- `ξ_axis = sigma2_n_axis / sigma2_deltaXT` is the dimensionless ratio σ²_n / Δt²·D
- This **is exactly** `Rz(axis)` (since R_pos is constructed as `Rz · sigma2_deltaXT`)
- ⇒ ξ_x = 1.53e-3, ξ_y = 1.29e-5, ξ_z = 4.35e-2 ⇒ ξ_z is **3000× larger than ξ_y**, fully justifying per-axis R (Stage 1 fix).

**3-guard / R behavior:**
- The only guard is the **var_threshold trigger** (Section B). When `del_pmr_var < sigma2_dXT · 0.001`, R(2,2) is inflated to `sigma2_deltaXT · 1e6` ≈ 2.5e-1 [um²/pN²]² (effectively ignore a_m).
- v2 needs to decide whether to keep this band-aid or replace with a principled gate.

**v2 verdict on R values:**
- ξ per-axis from sensor noise → **directly borrow** computation method (R_pos = sigma2_n_axis / sigma2_dXT); values may differ slightly under eq17 redefinition but the **structure** is identical.
- R(2,2) "intrinsic" value `chi_sq · rho_a · a²` → **borrow the formula** but per design_v2 Phase 6 §391: `R(2,2) = a_cov · IF_var · (â_x[k] + ξ)² + 5·Q77` which is more involved and **time-varying**. ⇒ ⚠ 需調整 (v2 has time-varying R(2,2); sigma-ratio-filter has constant).

---

## F. IIR coefficients

| Coefficient | sigma-ratio-filter `frozen_correct` | sigma-ratio-filter `empirical` | v2 design_v2.md §6 | Verdict |
|---|---|---|---|---|
| `a_pd` (LP for δp_md) | **0.05** (set in user_config.m line 79) | 0.05 | "inherit v1" → 0.05 | ✓ 借鑑 |
| `a_prd` (LP for δp_mrd mean) | **0.05** | 0.05 | "inherit v1" → 0.05 | ✓ 借鑑 |
| `a_cov` (LP for δp_mr² mean) | **0.005** (set in apply_qr_preset for frozen) | **0.05** | **0.05** (locked Phase 5) | ⚠ 需調整 (frozen used 0.005, but v2 says 0.05) |

**Critical conflict on `a_cov`:**
- sigma-ratio-filter `frozen_correct`: `a_cov = 0.005`
- v2 design_v2.md row 304: `a_cov = 0.05`

These differ by **10×**. The sigma-ratio-filter author chose `a_cov = 0.005` so that the IIR variance estimator `del_pmr2_avg` has a slow EMA (memory ~ 1/0.005 = 200 samples ≈ 0.125 sec at 1600 Hz) for low chi-squared sampling noise. v2 with `a_cov = 0.05` will give chi-squared noise **10× worse** in σ²_a_m, which feeds into R(2,2) and influences EKF a_hat std.

**Decision needed for v2**: either (i) lock to 0.05 per design_v2 and accept higher a_hat std, or (ii) revisit Phase 5 derivation to allow `a_cov = 0.005`. Phase 8 implementation should match the chosen value to the Q/R formula tied to `chi_sq = 2·a_cov / (2 - a_cov)`.

---

## G. Wall-aware init for `a_x`

**Implementation in `motion_control_law_7state.m` (lines 90-100, commit `9a0e8db`):**
```
p0_init       = params.common.p0;
w_hat_init    = params.wall.w_hat;
pz_init       = params.wall.pz;
R_init        = params.common.R;
h_init        = p0_init(:)' * w_hat_init(:) - pz_init;
h_bar_init    = max(h_init / R_init, 1.001);
[cpara_init, cperp_init] = calc_correction_functions(h_bar_init);
% For wall-normal w_hat = [0;0;1], this maps: x,y use c_para, z uses c_perp
a_hat = [a_nom / cpara_init; a_nom / cpara_init; a_nom / cperp_init];
```

**Commit reference confirmed**: `9a0e8db feat(controller): wall-aware initial a_hat for EKF` (in feat/sigma-ratio-filter history).

**v2 must mirror this** for the 5-state EKF's `a_x` slot. The tighter `Pf(6,6) = 1e-5` only works because `a_hat[0]` is already near truth; without wall-aware init, the EKF takes thousands of samples to converge through Pf-shrinkage and the controller produces large initial transients. ⇒ ✓ 借鑑.

**Edge case to preserve**: `h_bar_init = max(h_init/R, 1.001)` — clamp to avoid `c_para/c_perp` blowup at h_bar → 1.

---

## H. 5-seed CI driver flow

**Driver: `verify_qr_positioning_run.m`**

| Item | Value | Comment | Verdict |
|---|---|---|---|
| `T_sim` (default) | 30 sec | Session 6 reproduction | reasonable for v2 long-run |
| `T_sim` (P1 test) | **15 sec** | Session 7 P1 (used for Stage 4 final 2026-04-27 result) | ✓ 借鑑 for fast iteration |
| `t_warmup` (data-analysis stats window) | default 10 s; **P1 test: 2 s** | Steady-state window: stats over `t >= t_warmup` | ✓ 借鑑 |
| Seed list | hard-coded `[12345, 67890, 11111, 22222, 33333]` (5 seeds) | NOT randomized; reproducible | ✓ 借鑑 |
| Seeding mechanism | `config.thermal_seed = seed` (NOT `rng(seed)` directly — that's a known pitfall, see memory note `project_phase1_axm_audit.md`) | `calc_simulation_params` reads `thermal_seed` to set thermal RNG seed | ✓ 借鑑 |
| Sensor noise spec | `config.meas_noise_std = [0.00062; 0.000057; 0.00331]` ([um], main project lab spec) | Aligned with baseline (2026-04-18 fix) | ✓ 借鑑 |
| Scenario list | `near_wall_h25` (h_init=2.5), `free_space_h50` (h_init=50) | h=50 is the validated paper-level target | ✓ 借鑑 |
| Variant matrix | up to 8 presets (variants_all 1-8); P1 used `variant_filter = [3 6]` typically | v2 should run a similar A/B between v2-eq17 and v1-frozen_correct | ✓ 借鑑 (concept) |
| Output `.mat` structure (per run) | struct with fields: scenario, variant, seed, h_init, sim_time_s, idx, tracking_{mean,std}_{x,y,z}, tracking_rmse_3d, theory_std_dpmr_{x,z}_nm, ahat_{bias,std,max,rms}_{x,z}_pct, spike_{x,z}_pct, spike_z_t, nan_count, plus theoretical Lyapunov-based fields | structured, well-aligned for aggregator | ✓ 借鑑 (template) |
| Aggregator | `verify_qr_positioning_aggregate.m` produces md reports + summary png | full pipeline; v2 can clone | ✓ 借鑑 |
| Incremental checkpoint | `save(output_file, ...)` after every single run inside the `for j = 1:n_run` loop (line 70-72) | no run loss on crash | ✓ 借鑑 (must inherit) |

**Trajectory for h=50 positioning:**
```
config.h_init      = 50;       config.h_bottom = scenario.h_init;   config.amplitude = 0;
config.t_hold      = 0;        config.frequency = 1;     config.n_cycles = 1;
config.trajectory_type = 'positioning';     config.lambda_c = 0.7;
config.controller_type = 7;
```

This is the **exact configuration** that produced the paper-level final result (Stage 4, 2026-04-27). v2 should run the same config with `controller_type = <new v2 type>`.

---

## I. Bus extension summary (since 2026-04-22)

CtrlBus elements (from `calc_simulation_params.m`, 28 elements):
```
1-7   enable, lambda_c, gamma, Ts, meas_noise_{enable,std[3x1],seed}
8-13  a_pd, a_prd, a_cov, epsilon, k_B, T
14-15 sigma2_deltaXT, g_cov
16-19 controller_type, lambda_e, beta, lamdaF
20    sigma2_noise[3x1]   ← per-axis sensor noise
21    Pf_init_diag[7x1]
22    Qz_diag_scaling[9x1]   ← was 7x1 pre-2026-04-22; per-axis Q77
23    Rz_diag_scaling[6x1]   ← was 2x1 pre-2026-04-21; per-axis R_pos and R_gain
24-25 kf_R, kf_L
26    C_dpmr_eff[3x1]   ← was 1x1 pre-2026-04-27; per-axis on-the-fly
27    C_np_eff[3x1]     ← same
28    IIR_bias_factor[3x1]   ← same
```

**v2 will need analogous Bus reshape** for 5-state structure; templates above are good blueprints. ⇒ ✓ 借鑑 (pattern).

---

## J. Critical Conflicts with v2 design (summary table)

| # | Topic | sigma-ratio-filter | v2 design_v2.md | Resolution |
|---|---|---|---|---|
| 1 | Q(7,7) [δa] in positioning | 0 (Stage 3 fix) | derived 0 when ḣ=0 (Phase 5 Term A+B both vanish) | **Match** in positioning limit |
| 2 | Q(7,7) [δa] in motion | still 0 (`frozen` cannot track) | **non-zero, time-varying** Δt⁴·â²·{Term A+B} | v2 must implement time-varying Q77; **don't borrow constant** |
| 3 | Q(6,6) [a slot] | 1e-8 (small but non-zero) | derived from Phase 5 (process model) | ⚠ recompute for v2 5-state |
| 4 | a_cov | 0.005 (frozen_correct family) | 0.05 (locked) | ⚠ **decision needed**: lock at 0.05 (per v2) or revisit (impacts R(2,2) structure) |
| 5 | R(2,2) | constant `chi_sq · rho_a · a²` per axis | time-varying `a_cov · IF_var · (â_x[k] + ξ)² + 5·Q77` | v2 implements time-varying; structurally similar but per-step compute |
| 6 | KF runtime gate (h_bar_safe, NaN, t_warmup_kf) | none — only var_threshold band-aid | Phase 6 may require | ❓ confirm with Phase 6 final spec |
| 7 | Adaptive R(2,2) on thermal-quiet | yes (1e6 inflation when var < 0.001·sigma2_dXT) | not explicitly in v2 spec | ❓ keep as fallback or replace |
| 8 | Number of states | 7 (with δp1, δp2, δp3, d, δd, a, δa) | 5 (eq17 form: drops δd and δa, keeps Q77 as process noise) | structural difference; cannot directly map |
| 9 | Pf_init(δa) slot | 0 | n/a (no δa state) | drop |
| 10 | Pf_init(δd) slot | 0 | n/a (no δd state) | drop |

---

## K. Open Questions for Reviewer

1. **a_cov 0.005 vs 0.05** (Conflict 4): which is canonical for Phase 8? The 10× difference materially affects chi-squared sampling noise and R(2,2). The frozen_correct result uses 0.005; design_v2.md says 0.05.
2. **Adaptive R(2,2) band-aid**: keep the var_threshold trigger from sigma-ratio-filter, or replace with a principled mechanism (e.g., explicit thermal-quiet detection)?
3. **KF runtime gate**: Phase 6 mentions a gate, but I did not find its full spec. Should Phase 8 implement t_warmup_kf + h_bar_safe + NaN guard, or is the 2-step warmup + var_threshold sufficient (matching sigma-ratio-filter)?
4. **Q66 / a-slot process noise**: is the v2 Q for the `a_x` state derived independently of v1's `1e-8`, or should v2 pin `Q(5,5) = 0` (since Phase 5 says all noise is in δa, which doesn't exist as a separate state)? This impacts how `a_x` drifts in the absence of measurement updates.
5. **gate-trigger R(2,2) value**: sigma-ratio-filter uses `1e6 · sigma2_dXT` (≈ 2.5e-1) when triggered. The task brief mentions `1e10` — confirm what value v2 uses (different magnitudes ⇒ different "ignore measurement" stiffness).
6. **Q(4,4) [d slot]**: sigma-ratio-filter has it = 0. v2 Phase 5 derives Q(d) > 0 for disturbance random walk. Confirm v2 5-state Q(4,4) is non-zero per design_v2.

---

## L. Recommendations for Phase 8 implementation

**Direct ports (verbatim or near-verbatim):**
- Wall-aware `a_hat[0]` init (commit 9a0e8db pattern) — Section G
- P5 + P6 IIR pre-fill + warmup_count = 2 — Section A
- Per-axis sensor noise → R_pos: `R_pos_axis = sigma2_n_axis` — Section E
- 5-seed driver structure (verify_qr_positioning_run.m template) — Section H
- Incremental checkpoint pattern in batch loop — Section H
- Per-axis on-the-fly C_dpmr / C_n / IIR_bias_factor compute (calc_ctrl_params.m for ax = 1:3 loop) — Section I

**Recompute / redesign (do NOT borrow values):**
- All Q matrix entries: derive per Phase 5 for the 5-state structure
- R(2,2): derive per Phase 6 (time-varying) instead of borrowing constant 0.01881
- a_cov: lock to 0.05 per design_v2 (or revisit), and then derive R(2,2) consistent with that

**Confirm with reviewer (questions K above) before starting:**
- a_cov value
- adaptive R(2,2) keep/replace decision
- KF runtime gate spec (t_warmup_kf, h_bar_safe)
- gate-triggered R(2,2) inflation value (1e6 vs 1e10)

---

**End of audit.**
