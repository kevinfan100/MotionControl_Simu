# 4-state AR(1): Decoupling a_det / a' from the Known Wall Model — Status

**Line**: can the 4-state AR(1) controller's `a_det[k]` (AR(1) reversion target)
and `a'` (feedforward slope) be computed WITHOUT `c(h̄)` (unknown-wall case),
while staying in the 4-state architecture (no state augmentation, unlike the
5-state_aprime line)? **Branch**: `test/motion-test`. **Date**: 2026-07-04/06.
**Status**: exploratory / diagnostic — no formal derivation `.tex` yet; three
experiments run, one stable non-circular result found, Q44/R22 re-derivation
in progress (see §6).

---

## 1. Files (all TEMP, not yet committed — see §7)

| Role | Path |
|---|---|
| Exp. 03 — a_det:=â_x (closed loop, diverges) | `model/controller/temp_motion_control_law_eq17_4state_selfdet.m` |
| a_det known-stable, a' from self-diff â_x | `model/controller/temp_motion_control_law_eq17_4state_aprime_ff.m` |
| Exp. nocheat — a_det:=a_m_det, a' from self-diff â_x | `model/controller/temp_motion_control_law_eq17_4state_nocheat.m` |
| Driver dispatch (3 temp branches + `diag.a_prime_hat`) | `model/dual_track/run_pure_simulation.m` (`eq17_variant='4state_selfdet'\|'4state_aprime_ff'\|'4state_nocheat'`) |
| Test/figure scripts | `test_script/integration/temp_*.m` (several; see file headers) |

Offline scalars reuse `build_eq17_6state_constants` (dimension-agnostic, same as
all Vpersonal variants).

---

## 2. Why this line exists

The production 4-state AR(1) controller (`motion_control_law_eq17_4state.m`,
`use_q44_ar1=true`, see `derivation/4state_del_hd_ar1.tex`) reads the known wall
model `c(h̄)` in **two separate places**:

1. `a_det[k] = a_nom/c(h̄_d[k])` at the **desired** height — the AR(1) reversion
   target `(1−λc)·a_det[k]`, and the feedforward `Δa_x[k]=a_det[k]−a_det[k−1]`.
2. `K_h_axis` at the **measured** height — feeds `Q44` (`var_da_ram`) and the
   `R(2,2)` delay-leak term.

This line asks: can (1) be replaced by something that doesn't need `c(h̄)`,
while keeping the 4-state (not 5-state) architecture? (2) is out of scope here
(left on the known wall model in all three experiments below).

---

## 3. Three experiments (all on the `osc_1hz` scenario, see §4)

### Exp. 01 — circular (not valid evidence)

Differencing the **production** 4-state's own `â_x` (which is already fed by
the known-wall `a_det`/`Δa_x`) against a gated+EWMA slope estimator:
`corr_z=0.996`, `relerr_z=5.5%`. **Invalid as evidence** — `â_x ≈ a_det` to
within 0.3–1% (AR(1) pins it there), so differencing `â_x` just reads the
known answer back through a longer, noisier path. Confirmed by: switching the
raw-material to the **5-state**'s (`δa_x` free random walk, no known-wall
feedforward) `â_x` collapses the result (Exp. 02).

### Exp. 02 — fair, open-loop (diagnostic only, not fed back to control)

Same gated+EWMA differencing, applied to the 5-state's `â_x` (43% own relative
noise, no known-wall info): `corr_z=0.387`, `relerr_z=79.8%`. Honest measure of
"how good is naive self-differencing," decoupled from the a_det question.

### Exp. 03 — closed loop, `a_det:=â_x` (diverges)

`temp_motion_control_law_eq17_4state_selfdet.m`: sets **both** the AR(1)
reversion target and the feedforward source to the EKF's own `â_x`/self-diff,
fed back into the real predict step. Tested raw / gate-only (`|Δh_d|>1e-3`) /
EWMA-only (`β=0.05`) / gate+EWMA — **all 4 damping configurations diverge on
3/3 seeds** (some crash into the wall). Root cause (algebraic): with
`a_det_k:=â_x`, `λc·â_x+(1−λc)·â_x=â_x` — the AR(1) restoring force cancels
identically regardless of how the feedforward is damped; the reversion target
has no independent anchor. First test run showed false "stability" due to
MATLAB `persistent` state carrying over between separate `run_pure_simulation`
calls in the same session — **fixed by `clear <ctrl-fn>` before every single
run**, after which the divergence is confirmed and reproducible.

---

## 4. Verification scenario (all experiments)

1 Hz near-wall osc — `h_init=50`, `h_bottom=2.7` (h̄ trough ≈1.2), `amplitude=2.5`,
`t_hold=0.5`, `t_descend=1.0`, `T_sim=4`, `λc=0.7`, `a_pd=a_cov=0.05`,
`meas_noise_std=[.62,.57,3.31]e-3 µm`, seeds 1–3. `selfdet_gate_thresh=1e-3 µm`,
`selfdet_beta=0.05` (EWMA on the self-differenced slope) unless noted.

---

## 5. ⭐ Exp. nocheat — a_det from a model-free source: first stable result

`temp_motion_control_law_eq17_4state_nocheat.m`: `a_det[k] := a_m_det`
(persistent, EWMA-smoothed `a_xm` — **already exists** in the codebase as the
`use_am_lpf` feature's variable; needs only `λc + 4kBT`, no `c(h̄)` — the
"model-free Brownian thermometer" established early in this line of work).
`a'` unchanged from Exp. "aprime_ff" (gated+EWMA self-diff of `â_x`).

Because `a_m_det` is driven by an **independent measurement stream** (`a_xm` ←
`dx_r` ← position residuals), not by `â_x` itself, the AR(1) reversion keeps a
genuine external anchor — no algebraic self-cancellation like Exp. 03.

| Metric | Result |
|---|---|
| Divergence | **0/3** (first stable "neither side cheats" result in this line) |
| Tracking z | 34.9 nm (vs 24.1 nm known-wall baseline) |
| a_hat std z / bias z | 35.6% / **+12.2%** (baseline: 49.4% / 0%) |
| `a_det` (`a_m_det`) vs true `a_det`, corr / relerr | x 0.44/16%, y 0.66/11%, **z 0.70/26%** |
| `a'` (self-diff) vs true `a'`, corr / relerr | x 0.18/186%, y −0.05/207%, z 0.27/114% |
| EWMA ablation (z, `β=1` vs `0.05`) | corr **0.36 → 0.997** — the EWMA smoothing is doing almost all the work of the "circular" 0.99 numbers seen elsewhere; without it the raw signal is near-noise |

Figures (gitignored, `test_results/verify_4state/osc_1hz/`):
`fig1_aprime_ff_closedloop_osc_1hz.png` (Exp. aprime_ff a' vs true),
`fig1_aprime_ewma_compare_osc_1hz.png` (EWMA on/off ablation),
`fig1_nocheat_osc_1hz.png` (4-tile: a_det / a' / resulting a_hat / z-tracking-error).

**Reading**: `a_det` via `a_m_det` is a workable non-circular candidate
(direct level measurement, no differentiation). `a'` via naive self-differencing
is not — confirms Exp. 02's finding that differencing is the weak link,
independent of how good the `a_det` side is.

---

## 6. Next: re-derive Q44 / R22 within the 4-state structure (in progress)

**User decision (2026-07-06): stay in the 4-state state-space (no augmented
state); do NOT adopt the `5state_aprime` architecture as the base** — this is
a parameter-level fix, not an architecture change.

`F_e` / `H` structure is unchanged (still 4×4 / 2×4) — `a_m_det` and the
self-diff `a'` are exogenous inputs, not new states. Only `Q44`/`R22` need new
additive terms. Re-derived error dynamics (let `e_amdet:=a_det−a_m_det`,
`e_aprime:=a'−â'`):

```
e_ax[k+1] = λc·e_ax[k] + (1−λc)·e_amdet[k] + e_aprime[k]·Δh_d[k] + a'ε[k]

Q44_new = (3−2λc²)a'²σ²_δh (original)  +  (1−λc)²·Var(e_amdet)  +  Δh_d²·Var(e_aprime)
R22_new = R22_intrinsic + delay-leak (original)  +  ΔH_d²·Var(e_aprime)   [sum_da_ff now uses â', no longer exact]
```

- `Var(e_amdet)`: derivable now with the existing EWMA toolkit (same
  variance-gain machinery as `K_var`/`g_E` in the R22 derivation) — splits into
  a noise-averaging term `α/(2−α)·R(2,2)` (α=`a_det_lp`, currently 0.005,
  **never tuned for this role**) and an EWMA-lag bias term. Suspect the lag
  term dominates at α=0.005 for a 1 Hz trajectory (effective window ≈200
  steps ≈ ⅛ period) — **α-sweep is the immediate next actionable step**,
  same methodology as the `5state_aprime` κ-sweep.
- `Var(e_aprime)`: harder — `â'` is differenced from `â_x`'s own history, so
  `e_aprime` is correlated with `e_ax` itself (self-referential, same
  structural family as Exp. 03's instability, but at the noise-statistics
  level rather than the divergence level). No closed form yet; treat as an
  independent approximate term for a first pass, flag as open.

> **Cross-reference (2026-07-06, from the parallel `5state_aprime` line, now
> paused — see `reference/eq17_analysis/derivation/5state_aprime_unified.tex`
> Status box): that line hit the two issues this section anticipates, with
> concrete numbers that may save time here.**
> 1. **The α-sweep will likely show a real trade-off, not a clean optimum.**
>    Sweeping `Q_aprime_factor` (their analogue of α) over
>    `[0.1, 0.3, 1, 3, 10]` on the same 1 Hz near-wall scenario: correlation/
>    accuracy improved monotonically with the gain, but tracking cost and
>    instability risk ALSO worsened monotonically — `kappa=3,10` reintroduced
>    closed-loop divergence. Expect a bias-variance shape here too, not a
>    single best value.
> 2. **A too-generous or context-blind noise-inflation term can look fine
>    open-loop and only break in REAL closed loop.** Their honest `Q(a')`
>    anchor (no `c(h̄)`) had no wall-distance awareness and over-inflated
>    ~1e4–1e5x during fast motion far from the wall — invisible under
>    open-loop scoring, but caused real z-axis divergence once closed
>    (control law using `â_x`, not the true gain). Given `e_aprime`'s
>    self-referential structure is already flagged above as risky (same
>    family as Exp. 03's divergence), worth testing whatever `Var(e_aprime)`
>    formula is derived in BOTH open- and closed-loop modes before trusting
>    an open-loop-only characterization.

---

## 7. Known limits / open issues

1. `Var(e_aprime)` has no rigorous closed form yet (§6) — biggest open item.
2. `a_det_lp` (α) has never been tuned for the "AR(1) reversion target" role;
   only ever tuned (and found harmful in a different role) for `use_am_lpf`
   feeding the EKF measurement directly — see `project_am_lpf_study_2026-06-17`
   memory. α-sweep needed before trusting a_m_det's accuracy numbers as final.
3. `K_h_axis`/`Q44`/`R22`'s existing delay-leak term still reads the known
   wall model at the **measured** height (item (2) in §2) — untouched, out of
   scope for this line so far.
4. All `temp_*.m` files and the `run_pure_simulation.m` dispatch additions are
   diagnostic scaffolding, not committed, and not yet cleaned up per the
   project's `temp_*.m`-delete-after-use convention — pending until the Q44/R22
   re-derivation is complete enough to decide what (if anything) becomes a
   real, named controller variant.

All `.m` checkcode-clean. Verification is dynamic-only; figures land in
`test_results/verify_4state/osc_1hz/` (gitignored).

---

## 8. MAIN CAUSE identified and triple-confirmed (2026-07-06/07)

**Goal refinement (user decision)**: wall POSITION known, `c(h̄)` unknown;
base = the AR(1) variant (not RW); the deliverable is a complete, correct
theory that predicts and guides design ("α 該設多少" is tuning, NOT the cause).

### 8.1 The cause (one sentence)

The `a_xm` inversion `a_xm = (σ̂²_δxr − C_n·σ²_n)/(C_dpmr·4kBT)` silently
assumes `g := a_true/â = 1` (control at its design point); it cannot
distinguish "a changed" from "tracking degraded because â is wrong". Feeding
this confounded measurement back as the AR(1) reversion anchor
(`a_det_k := a_m_det`, fixed weight `(1−λc)`, no KF damping) creates a
positive-feedback anchor loop whose gain scales with `α`.

**Consequence for §6**: the "e_amdet is exogenous" assumption in the
`4state_del_hd_ar1.tex` correction is FALSIFIED — `e_amdet` couples to `e_ax`
through the real tracking error. `Q44_new`'s `Var(e_amdet)` additive term was
tested (`4state_adet_only_newq`) and does nothing (bias 11.9→12.2%), because
the injection path bypasses the Kalman gain entirely.

### 8.2 The corrected closed loop (g-mismatch form, VALIDATED)

```
δx[k+1] = λc·δx[k] + (1−g)(1−λc)·δx[k−d] + (1−g)·D[k] − ε_g[k]
D[k]    = p_d[k+1] − λc·p_d[k] − (1−λc)·p_d[k−d]      (known; =0 during holds)
ε_g[k]  = a·f_T[k] + (1−λc)·Σᵢ a·f_T[k−i] − g(1−λc)·n_x[k]
```

- g=1 self-check: reduces to `λc·δx − ε`; `Var(ε)` matches the code's `Q33_ss`.
- The `(1−g)·D[k]` deterministic feedthrough is exactly what `dx_bar_m`
  (δp_md) measures — **p_md is a discarded, signed, first-moment g-observation
  channel** (δx_det ≈ (1−g)·D/(g(1−λc)) quasi-statically). This answers the
  "is p_md hidden information" question: yes.

### 8.3 Evidence chain (all numerical gates PASSED)

| # | Test | File | Result |
|---|---|---|---|
| 1 | Loop-cut interventions | `temp_loop_cut_interventions.m` | CUT-PHYSICS (control on true gain) fully stabilizes α=0.010 (0/3); CUT-Y2 is bit-identical to closed-loop (max ratio 759.06 both) → y2/Q44/R22 irrelevant; predict path alone carries the loop. Open-loop a_m_det bias only 1.6–3.7% → **"information floor" hypothesis overturned** (~8 of the 12 bias points are loop-equilibrium shift). |
| 2 | Deterministic feedthrough | `temp_g_probe_validation.m` Part A (noise-free osc, fixed g via TEMP `opts.true_gain_scale`) | corr=1.0000, amplitude ratio 1.000, relRMS 0.2–0.3% at g=0.5 and g=2.0. |
| 3 | Stochastic variance map | same, Part B (hold h̄=2.22, augmented 6-state Lyapunov → `Var(dx_r)=c_w(g)·4kBT·a + c_n(g)·σ²_n`) | all five g within 0.1–0.5%; g=1 reproduces the C_dpmr formula to ratio 1.0000. Confound strength: a_xm/a_true = 1.22 @ g=0.6, 0.93 @ g=1.67; hold-scenario DC loop gain S≈0.31<1 (statically stable → cliff must be dynamic). |
| 4 | Mean-path surrogate | `temp_loop_surrogate.m` | reproduces the closed-loop BIAS (26.1%/10.6% vs measured 25.5%/11.9% at α=0.002/0.005) but NOT the cliff (stable to 0.05) → bias = mean-path lag+loop effect. |
| 5 | Stochastic surrogate | `temp_loop_surrogate_v2.m` (adds only correlated noise sampling) | **reproduces the cliff at the measured location**: 0.007→0/5, 0.008→1/5, 0.010→3/5, 0.020→5/5 (real sim: 0.005 stable / 0.008 blown) → cliff = noise-triggered escape (χ² fluctuations of σ̂², rel std ~46%, amplified by the nonlinear g-loop). |
| 6 | **Counter-side (surgical removal)** | `temp_gfix_counterside_test.m` + `temp_motion_control_law_eq17_4state_adet_only_gfix.m` (oracle g-corrected inversion: subtract predicted deterministic leak, divide by c_w(g) instead of C_dpmr; loop fully closed) | bias @0.005: 11.9%→**4.1%** (predicted open-loop floor 2–4%); cliff removed: 0.008→0/3 (28.3nm), 0.010→0/3 (28.8nm); 0.020→1/3 (residual escape consistent with the correction's quasi-static approximation). |

Residual: v2 surrogate over-predicts bias slightly (16.3 vs 11.9% @0.005) —
the omitted y1-channel correction of slot 4 is mildly stabilizing; secondary.

### 8.4 Files added for §8 (all TEMP, uncommitted)

- `model/controller/temp_motion_control_law_eq17_4state_adet_only.m` — a_det-only ablation
- `model/controller/temp_motion_control_law_eq17_4state_adet_only_newq.m` — Var(e_amdet) Q44 test (falsified)
- `model/controller/temp_motion_control_law_eq17_4state_adet_only_gfix.m` — oracle g-correction (counter-side)
- `model/dual_track/run_pure_simulation.m` — 3 TEMP dispatch branches + `opts.true_gain_scale`
- `test_script/integration/temp_decompose_adet_vs_aprime.m` — 4-way ablation (a' has ZERO effect; a_det carries ~all of nocheat's degradation)
- `test_script/integration/temp_adet_only_alpha_newq_sweep.m`, `temp_adet_only_alpha_fine_sweep.m` — α cliff mapping (0.005 stable / 0.008 blown)
- `test_script/integration/temp_check_sigma2_dxr_coupling.m`, `temp_check_q_r_height_scaling.m` — intermediate diagnostics (the scalar-KF-from-existing-Q/R idea was CHECKED AND REJECTED: near-wall gain would be ~600x the far-field one, wrong direction)
- `test_script/integration/temp_loop_cut_interventions.m`, `temp_g_probe_validation.m`, `temp_loop_surrogate.m`, `temp_loop_surrogate_v2.m`, `temp_gfix_counterside_test.m` — the §8.3 evidence chain

### 8.5 Next steps

1. ~~Write the g-mismatch derivation (§8.2) + validation into
   `derivation/4state_del_hd_ar1.tex`~~ **DONE (2026-07-07)** — see §9.
2. ~~Design the PRODUCTION (non-oracle) remedy~~ **DONE (2026-07-07)** — see §9.
3. Only then revisit Q44/R22 numbers (they were proven irrelevant to THIS
   failure mode, but the g-corrected a_xm changes `R22`'s operating point).

---

## 9. DEPLOYABLE controller completed (2026-07-07)

`temp_motion_control_law_eq17_4state_deploy.m` — fully model-free (wall
position known, `c(h̄)` unknown): a' self-diff + `a_det := a_m_det` anchor +
**ĝ-from-δp_md regression estimator** + g-corrected inversion. No oracle
anywhere. Full derivation + validation written into
`derivation/4state_del_hd_ar1.tex` (8 new sections after the Var(e_aprime)
treatment; 18 pages, compiles clean).

### 9.1 The ĝ estimator (design iteration matters)

- Pointwise ratio `u = (1−λc)·δp_md/D̄` **FAILED** (bias +52.9%, worse than
  uncorrected): per-sample SNR < 1, ratio rectification bias.
- **Regression form WORKS**: `Sxy = EWMA_βg(D̄·δp_md)`, `Sxx = EWMA_βg(D̄²)`,
  `u = (1−λc)·Sxy/Sxx`, `ĝ = clamp(1/(1+u), [0.5, 2])`, hold while
  `Sxx ≤ γ_D²`. `D̄ = LP_apd(D[k−d])` (matched-lag regressor; D̄ deterministic
  → `E[D̄·noise] = 0`, unbiased). Knobs: `β_g=0.05`, `γ_D=2e-3 µm`.
- Clamp tightening [0.2,5]→[0.5,2] was the stability enabler (α=0.010 blew
  up before); slower `β_g=0.02` REJECTED (estimator lag destabilizes).

### 9.2 Final validation (osc_1hz, 5 seeds)

| α | blown | track x/y/z (nm) | bias x/y/z (%) |
|---|---|---|---|
| **0.005 (operating point)** | **0/5** | 28.2 / 27.9 / 30.5 | +5.5 / −2.4 / +3.2 |
| 0.008 | 1/5 | 28.5/27.6/34.0 | +4.6/−4.0/+0.4 |
| 0.010 | 1/5 | 28.5/27.6/47.6 | +4.1/−4.3/+0.1 |
| 0.020 | 5/5 | blown | — |

vs uncorrected (+12.2%/34.9nm, cliff 0.008) and oracle upper bound
(+4.1%/27.9nm). The deployable version recovers most of the oracle's gain;
margin to the cliff ≈ 1.6× at the operating point.

### 9.3 Files added for §9 (TEMP, uncommitted)

- `model/controller/temp_motion_control_law_eq17_4state_deploy.m`
- `model/dual_track/run_pure_simulation.m`: deploy dispatch + `g_gate_thresh`/
  `g_beta` passthrough + `diag_log.g_hat`/`a_xm_corr`
- `test_script/integration/temp_deploy_{validation,knob_sweep,final_validation}.m`
- `derivation/4state_del_hd_ar1.tex/.pdf` — 8 new sections (FALSIFICATION →
  g-mismatch loop → variance map → mechanism closure → ĝ estimator →
  corrected inversion → complete controller → end-to-end table)

Residual known-model dependencies (declared out of scope): `K_h` inside the
Q44/R22 random-gain floor, and the wall-aware `â_x[0]` seed.

---

## 10. Lag/accuracy characterization + main-line KF experiment (2026-07-07, post-deploy)

### 10.1 a_hat error characterized (user observation: inaccurate + lagged)

Lag chain decomposition (z, osc window, 3 seeds): `a_xm_corr` lag 1.7 ms /
43.3% relRMS -> `a_m_det` (EWMA alpha=0.005 anchor) lag **78.8 ms** / 26.9%
-> `a_hat` **78.3 ms / 26.9% (identical to the anchor -- the EKF contributes
nothing; y2 gain negligible under AR(1)-bounded P44)**. Removing the pure lag
leaves 17.8% (chi-square floor through the EWMA). Phase decomposition: the
error cause DIFFERS per phase -- hold1 (far) 9.1% pure noise floor; descent
+7.8% bias = lag-on-a-ramp; osc bias ~0, lag-as-phase-shift ~1/3 of 28.7% RMS;
hold2 (near wall) 47% = transient tail (decays in 0.5 s) + stale-g_hat
persistent bias + near-wall relative inflation (a_true 7x smaller).

### 10.2 hold2 fix: slow g_hat decay-to-1

Freezing g_hat during holds leaves a persistent −12..−20% long-hold bias
(stale g=0.5 while true g recovers to 1); FAST decay (0.01) is worse (turns
the correction off before the real g recovers). **Slow decay g_decay=0.001**
(~390 ms, matched to the hold recovery rate) fixes the long-hold windows
(19.6->13.0%, 23.8->12.6%, late 27.6->20.8%) without touching the early
transient or osc. Set as deploy default. Sweep: temp_gdecay_sweep.m,
temp_hold2_settle_test.m.

### 10.3 Main line tested and FALSIFIED: KF-weighting cannot replace the anchor (4-state)

`temp_motion_control_law_eq17_4state_deploy_kf.m` (y2 := a_xm_corr, RW gain
state, anchor removed from predict): lag collapses 78->8 ms and hold2 halves
(47.9->23.4%), BUT oscRMS 28.7->57.6%, a_std 39.6->71.9%, track 28.9->48.5 nm.
kf_q44_scale sweep {1..0.003} maps the frontier: **no point beats the anchor**
-- even at matched bandwidth (scale 0.003, lag 62 ms) oscRMS is 40.5% vs
28.7%, and small scales grow bias (a'-ff drift accumulates in RW; the anchor's
(1−λc) reversion kills that drift every step). Feeding a_xm_corr to y2 within
the anchor architecture: bit-identical (y2 gain negligible). **Conclusion: the
anchor architecture is ON the achievable frontier under the 4-state
constraint; the 79 ms lag is the price of the chi-square noise level, not an
architectural mistake.** Structural escape would need a level+rate gain model
(state augmentation -- ruled out by scope) or better raw-measurement SNR.
Files: temp_deploy_kf_test.m, temp_deploy_kf_q44_sweep.m,
temp_deploy_y2corr_test.m; deploy grew `y2_use_corr` (default false) and
deploy_kf grew `kf_q44_scale` knobs.
