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
