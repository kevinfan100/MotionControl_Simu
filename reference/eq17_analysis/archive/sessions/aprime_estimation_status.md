# a' (gain-slope) Estimation — Status & Derivation↔Assumption↔Controller Map

**Line**: estimate the per-axis gain slope `a'_x = da_x/dh` for an UNKNOWN wall
(no `c(h̄)` in the estimator), by promoting `a'_x` to an estimated EKF state.
**Branch**: `test/motion-test`. **Date**: 2026-06-24/25.
**Derivation**: [`derivation/5state_est_aprime.tex`](derivation/5state_est_aprime.tex)
(+`.pdf`). This file maps that derivation to the code and the verification.

---

## 1. Files

| Role | Path |
|---|---|
| Derivation (math) | `reference/eq17_analysis/derivation/5state_est_aprime.tex` (+`.pdf`) |
| Controller | `model/controller/motion_control_law_eq17_5state_aprime.m` |
| F_e builder (standalone, testable) | `model/controller/build_F_e_5state_aprime.m` |
| Driver dispatch + knobs | `model/dual_track/run_pure_simulation.m` (`eq17_variant='5state_aprime'`) |
| L0 front-verification | `test_script/integration/verify_eq17_5state_aprime_L0.m` |
| L1 estimability + L2 closed-loop | `test_script/integration/verify_eq17_5state_aprime_L1.m` |
| Observability check | `test_script/integration/check_observability_5state_aprime.m` |

Offline scalars reuse `build_eq17_6state_constants` (dimension-agnostic).

---

## 2. Derivation ↔ assumptions ↔ controller

Per-axis augmented state: **`x = [δx_1, δx_2, δx_3, a_x, a'_x]`** (slot 5 = `a'_x`).

| Derivation (5state_est_aprime.tex) | Assumption | Controller realization |
|---|---|---|
| `a_x[k+1]=a_x[k]+a'_x[k]·Δh_d[k]+δa_ram` | gain advances by **slope × KNOWN desired height step**; `Δh_d=dot(del_pd, w_hat)` | predict `x_pred(4)=x(4)+x(5)*Δh_d` |
| `a'_x[k+1]=a'_x[k]+w_a'` | slope is **slowly-varying random walk** (no model feedforward — `c(h̄)` unknown) | predict `x_pred(5)=x(5)`; `Q(5,5)=Var(w_a')` |
| `F_e(4,5)=Δh_d[k]` | a' couples to a_x only through height motion | `build_F_e_5state_aprime` col 5 row 4 |
| `F_e(3,5)=dF_dx^h=(1−λc)Σ(h_d[k]−h_d[k−i])f_d[k−i]` | a' enters δx_3 via control history × height shift | col 5 row 3 |
| `H(2,5)=−ΔH_d[k]=−(h_d[k]−h_d[k−d])` | delayed gain measurement carries a' (d-step height span) | time-varying `H_full(2,5)` |
| slope const over the d-step delay window | small-d approximation for the H(2,5) collapse | inherent to the `−ΔH_d` form |
| `δx_D ≡ 0` | disturbance not modelled | no x_D state (as in 5-state) |
| d = 2 | 2-step sensor delay | `ctrl_const.d` |
| seed `a'_x[0] = −a_x·K_h/R` at h_init | **one-time** wall-aware seed; ongoing estimation never re-reads `c(h̄)` | `aprime_init` in init block |

**Q55 baseline** (the one genuinely new knob): `a'' = (a_x/R²)(K_h² − K_h')`,
`Q55 = Q_aprime_factor · [ (a''·Δh_d)² + a''²·σ²_dh ]`. Uses MEASURED h̄ (bias-free,
like Q44/R22). It sets the estimator **bandwidth** on a'_x; it does NOT leak a'_x
into the estimate (a'_x is read purely from a_xm).

---

## 3. Controller settings (and what verification used)

| Knob (ctrl_const / config) | Default | Verification value | Note |
|---|---|---|---|
| `Q_aprime_factor` (κ, scales Q55) | 1 | **10** (L1/L2/obs) | baseline (κ=1) too conservative for 1 Hz → κ≈10 optimal (see §5) |
| `Pf_aprime_scale` (a'_x[0] prior) | 1 | 1 | `Pf(5,5)=scale·((a'_init)²+1e-6)` |
| `freeze_aprime` | false | true (L0b only) | holds a'_x at seed, Q55=0, no slot-5 update |
| `lambda_c` | — | 0.7 | |
| gates G1/G2/G3 | warmup / NaN / `h̄<h̄_safe` | `h_bar_safe=1` (unreachable here) | **NOT yet an excitation gate** — see §6 |
| open-loop hook | `a_ctrl_override` via `opts.use_true_gain` | true=L1 (open), false=L2 (closed) | open-loop = control uses true gain, EKF estimates a' on the side |

**Verification scenario** (all): 1 Hz near-wall osc — `h_init=50`, `h_bottom=2.7`
(h̄ trough ≈1.2), `amplitude=2.5`, `t_hold=0.5`, `t_descend=1.0`, `T_sim=4`,
`λc=0.7`, `a_pd=a_cov=0.05`, `meas_noise_std=[.62,.57,3.31]e-3 µm`, 5 seeds.

---

## 4. Verification results

- **L0 (front, PASS)**: L0a finite-difference of the REAL maps → `F_e(4,5)=Δh_d`,
  `F_e(3,5)=dF_dx^h`, `H(2,5)=−ΔH_d` confirmed (J err ~1e-11); L0b frozen-a'
  x/y ≈ 4-state (base intact); L0c perfect-track 0.2 nm.
- **L1 (open-loop estimability)**: â' tracks true `a'(h)=−a·K_h/R` during the
  oscillation. Best κ=10: corr_z **0.73**, near-wall |relerr|_z 35%, spread 27%.
  Per-axis corr (x,y,z)=0.42,0.51,0.73 (z best = wall-normal motion direction).
  Q55 κ-sweep is a textbook lag(κ small)→track(κ=10)→noise(κ=100). Faster
  trajectory helps: 1 Hz (0.73) > 0.5 Hz (0.45) because a' signal ∝ speed.
- **L2 (closed-loop)**: stable (no divergence); â' survives the loop
  (corr_z 0.78 ≥ open-loop; the +11% a_xm confound bites the LEVEL a, not the
  SLOPE a'); tracking z 29.8 nm (no a') → **27.3 nm (estimated a')** → 24.4 nm
  (known-model 4-state) ⇒ estimating a' recovers ~half the gap.
- **Observability** (`check_observability_5state_aprime`): **rank(O_N)=5 ⟺
  Δh_d≠0; rank=4 during holds/turning points, null direction = exactly slot 5
  (a'_x)**. The a' observability strength ‖O(:,5)‖ ∝ |Δh_d| = speed. Even when
  observable it is WEAK (σ5/σ1 ~ 1e-2) → the ~35% accuracy floor is structural.

---

## 5. Results ↔ assumptions (why the numbers are what they are)

- The assumption "a' enters only via `a'·Δh`" ⟹ **a' observable iff Δh_d≠0**
  (verified: rank 5 motion / 4 hold). This is the structural root of: L1's
  three-phase picture (hold â'≈0 / osc tracks / static-hold collapse),
  faster-is-better (obs ∝ speed), the descent-onset dip (weak obs at small Δh),
  and the turning-point noise (obs dips at velocity zero-crossings).
- The Q55 baseline assumption (covers the per-step drift `(a''·Δh_d)²`)
  UNDER-covers the 1 Hz `a'(h)` variation ⟹ empirically need κ≈10× bandwidth.
- "Weak observability" (σ5/σ1 ~1e-2) ⟹ the SNR-limited ~35% accuracy is a hard
  structural floor, not a tuning artifact.

---

## 6. Known limits & open issues / next

1. **Static-hold bias + descent-onset dip** = unobservability / weak-excitation
   (structural, NOT bugs). Both fixable with an **excitation gate** (freeze a'
   update when |Δh_d| < threshold, or via the σ5 metric) — **NOT yet implemented**.
2. **`Q_aprime_factor` default is 1** but verification favors **≈10** for 1 Hz;
   consider per-axis κ and a default change after a fine sweep.
3. **"Any-trajectory" recommendation (not implemented)**: convert real-time a'
   tracking → **persistent `a'(h̄)` spatial learning** (map or gray-box wall
   model), gated by observability, optionally with a small probing dither for
   static-dominated tasks. Merges with the gscalar / scale-calib line (learn the
   unknown wall model from data).
4. **Derivation `.tex` Remarks** are qualitative; could add the quantitative
   observability result (rank↔Δh, ‖O(:,5)‖∝speed).
5. Robustness sweeps pending: 0.5/2 Hz, 8–10 seeds; merge target TBD.

All `.m` checkcode-clean. Verification is dynamic-only; figures land in
`test_results/verify_5state_aprime_L1/` (gitignored).
