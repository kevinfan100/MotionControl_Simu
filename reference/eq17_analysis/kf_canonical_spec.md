# 6-state EKF Canonical Spec (Vpersonal-aligned, λc)

**SSOT for the locked-down 6-state Kalman filter flow.** This document fixes the KF
algorithm skeleton and the per-controller "ingredients". Notation follows
`RevisedConrol_Vpersonal.pdf` (NOT the IEEE TIE 2025 paper). Pole symbol is **λc (lc)**,
not k_c.

- Source of truth: `RevisedConrol_Vpersonal.pdf` (structure) + `derivation/*.tex` (constants)
- Code anchor: `model/controller/motion_control_law_eq17_6state.m`,
  `model/controller/build_eq17_6state_constants.m`
- Comparison baseline (different controller): IEEE TIE 2025 (eq6, 7-state) — cite k_c↔λc only
- Created: 2026-06-09. Living document — extend at "Section 8 (往後增加)".
- Fidelity (2026-06-09 audit): infrastructure (plant/mobility/thermal/sensor-delay/output) +
  estimator (f_d/F_e/H/Φ/Q/R/init) all FAITHFUL to Vpersonal element-by-element. Sole structural
  deviation = S=0 (§8). Zero code bugs. Validated h̄>1.5: tracking ~31 nm, bias <1.1%, R22/Q55 emp≈1.

---

## 0. Locked decisions (2026-06-09)

| # | Decision | Choice | Rationale |
|---|---|---|---|
| D1 | Φ vs F_e | **Separate ingredients** | Vpersonal splits them p.4 (estimator) vs p.5 (F_e) |
| D2 | S = E{q·rᵀ} | **Keep S slot, default 0** | Vpersonal p.5 writes −L·r explicitly; near-wall fills S |
| D3 | Q33 composition | **3-component (full ε)** | Vpersonal p.4 `q3 = −ε` (thermal hist + randgain + n_x). Diverges from IEEE paper Path-C-strict (thermal only) — deliberate |
| D4 | Posterior P | **Joseph form** | numerical robustness (our enhancement; Vpersonal-compatible) |
| D5 | Pf init | **DARE steady-state** | Vpersonal silent on init |
| N1 | IIR poles | **a_pd / a_cov separate** | mean EWMA (a_pd) ≠ variance EWMA (a_cov) |
| N2 | C_dpmr / C_n | **full a_pd-dependent** | 3.16 / 1.11 (λc=0.7, a_pd=0.05); not the a_pd→0 simplification 3.96/1.18 |
| N3 | Disturbance state | **δx_D^d** (combined) | Vpersonal pre-combines x_D + (1−λc)·Σδx_D[k−i] |
| — | Pole symbol | **λc (lc)** | Vpersonal + codebase; k_c is IEEE-paper-only |

---

## 1. Fixed KF flow (invariant 5-step skeleton)

Per axis (x, y, z independent). `n`=state dim, `m`=measurement dim. The flow NEVER changes;
only the ingredients in §2 change per controller.

```
Enter step k:  (x̂⁻[k], P_f[k])      a-priori estimate + forecast covariance

(1) Gain        L[k]  = (F_e[k]·P_f[k]·Hᵀ + S[k]) · (H·P_f[k]·Hᵀ + R[k])⁻¹
                        └ S=0 default (D2); reduces to L = P_f·Hᵀ·(H·P_f·Hᵀ+R)⁻¹
(2) Update mean x̂[k]  = x̂⁻[k] + L[k]·( y[k] − H·x̂⁻[k] )
(3) Update cov  P[k]  = (I−L[k]H)·P_f[k]·(I−L[k]H)ᵀ + L[k]·R[k]·L[k]ᵀ      (Joseph, D4)
(4) Predict m   x̂⁻[k+1] = Φ[k]·x̂[k]                                       (deterministic map)
(5) Predict cov P_f[k+1] = F_e[k]·P[k]·F_eᵀ[k] + Q[k]

Exit: (x̂⁻[k+1], P_f[k+1])
```

Notes:
- Filter form (we store a-posteriori x̂[k|k]). Vpersonal predictor form merges (2)+(4) into
  the estimator eq (p.4) and (3)+(5) into the covariance recursion — algebraically identical.
- Innovation `y − H·x̂⁻` uses a-priori estimate; `e_y1 = δx_m − δx̂_1`, `e_y2 = a_xm − (â_x − d·δâ_x)`.
- Time-varying entries (F_e(3,5)/(3,6) via f_d history; Q[k], R[k] via â_x, h̄) are CONTENT
  updates, not flow changes.
- Code: predict-map L393-398, P_pred L399, gain L417, Joseph L427, per-axis loop L377-442.

---

## 2. Per-controller ingredients (what you "plug in")

Deciding dimensions is necessary but NOT sufficient. You also fix the CONTENTS of six items:

| Ingredient | Symbol | Role | 6-state value |
|---|---|---|---|
| dims | n, m | state / meas size | 6, 2 |
| mean map | **Φ** | propagate estimate | §4 |
| error Jacobian | **F_e** | propagate covariance | §4 (≠ Φ, see §3) |
| output map | H | which states measured | `[1 0 0 0 0 0; 0 0 0 0 1 −d]` |
| process noise | Q[k] | §5 | 3-component Q33, Q55 |
| meas noise | R[k] | §6 | R11, R22 (exact IF_eff) |
| init | P_f₀ | DARE steady-state | `solve_dare_kf_local` L158 |
| (cross-cov) | S[k] | q-r correlation | 0 (near-wall: §8) |

---

## 3. Φ ≠ F_e — the critical pair (Vpersonal p.4 vs p.5)

The mean-propagation map Φ and the error Jacobian F_e are TWO different matrices.

| | Vpersonal location | Content | Code |
|---|---|---|---|
| True state | **p.3 "State equation"** | `δx_3[k+1]=λc·δx_3 − F_dx·e_ax + dF_dx·e_δax − e_δafd − ε` | plant |
| **Φ** (mean) | **p.4 "Closed-loop Estimator"** (drop ℓ·e) | `δx̂_3[k+1]=λc·δx̂_3` | `x_pred` L393-398 |
| **F_e** (cov) | **p.5 F_e matrix** | Row3 = `[0 0 λc −1 −F_dx dF_dx]` | `build_F_e_6state` L507-524 |

```
Φ  (from p.4 estimator)              F_e  (p.5)
[0 1 0  0 0 0]                       [0 1 0  0   0     0    ]
[0 0 1  0 0 0]                       [0 0 1  0   0     0    ]
[0 0 λc 0 0 0]   ← λc only           [0 0 λc −1  −F_dx  dF_dx]   ← + error-coupling
[0 0 0  1 0 0]                       [0 0 0  1   0     0    ]
[0 0 0  0 1 1]                       [0 0 0  0   1     1    ]
[0 0 0  0 0 1]                       [0 0 0  0   0     1    ]

F_e − Φ : only Row 3 cols {4,5,6} = [−1, −F_dx, dF_dx]
```

**Why they differ:** the terms `−1·x_D`, `−F_dx·e_ax`, `dF_dx·e_δax` in the TRUE δx_3 (p.3)
multiply estimation errors (e_ax, e_δax, e_xD), which are zero-mean. The estimator (p.4) cannot
reproduce them, so Φ Row 3 keeps only λc. The error e = x − x̂ inherits them (p.3 true − p.4
estimate), so F_e (which propagates e) retains them.

**Consequence (deterministic-map predict):** the mean uses Φ (drops error couplings); the
covariance uses F_e (keeps them). Using F_e·x̂ for the mean would inject estimate values where
errors belong → the 7-state a_hat bias. This is the single most important structural choice.

Control-law dependence of F_e(3,3): eq17 (δx_m direct feedback) → F_e(3,3)=λc. eq6/IEEE-paper
(δx̂ feedback) → F_e(3,3)=1 (control law adds (1−λc)·e₃ back). Both correct for their law.

Helper sums (Vpersonal p.5): `F_dx = f_d[k] + (1−λc)·F_1`, `dF_dx = (1−λc)·F_2`,
`F_1 = Σ_{i=1}^d f_d[k−i]`, `F_2 = Σ_{i=1}^d i·f_d[k−i]`. Code L389-395.

---

## 4. State vector, Φ, F_e, H (6-state)

State (Vpersonal p.3), per axis:
```
x = [ δx_1, δx_2, δx_3, δx_D^d, a_x, δa_x ]ᵀ
      δx[k-2] δx[k-1] δx[k]  combined  gain  gain-rate
δx_D^d := δx_D[k] + (1−λc)·Σ_{i=1}^d δx_D[k−i]      (pre-combined, N3)
```

Φ, F_e: see §3. H (Vpersonal p.3):
```
H = [1 0 0 0 0 0;       y_1 = δx_m = δx_1 + n_x
     0 0 0 0 1 −d]      y_2 = a_xm ≈ a_x − d·δa_x + (n_a − Σ δa_ram[k−i])
```
(IEEE paper uses H(2,·)=[..1 0]; our −d models the 2-step gain delay explicitly.)

---

## 5. Q matrix (D3: 3-component Q33, Q55 closed form)

6×6 diagonal, two nonzero entries; per axis, per step. Code L316-343.

```
Q33 = Var(ε)  (3 independent components, h=50 independence approx):
   thermal :  4kBT·( â_x[k] + (1−λc)²·(â_x[k−1] + â_x[k−2]) )
   randgain:  (1−λc)²·Σ_{j=1}^d (d+1−j)²·f_d[k−j]²·Var(δa_ram[k−j])     weights {4,1} for d=2
   n_x     :  (1−λc)²·σ²_n_x
   Q33 = thermal + randgain + n_x

Q55 = Var(δa_ram) = [2/(1+λc)]·(â_x·K_h/R)²·σ²_dh                       (N2-style closed form)
   σ²_dh = 4kBT·a_perp           (wall-normal thermal var, shared 3 axes)
   K_h   = K_h_para (x,y) / K_h_perp (z)
   2/(1+λc) absorbs C_dpmr level amplification × (1−ρ1) increment factor (see §7 derivation)

Q(1,1)=Q(2,2)=Q(4,4)=Q(6,6)=0
```

Divergence note: IEEE paper / Path-C-strict uses Q33 = 4kBT·â_x[k] only. We follow Vpersonal
p.4 ε full form (D3). Empirically validated (review_findings §3.2, emp/closed ≈ 1.0 at h50/h10).

---

## 6. R matrix (R22 exact per-step IF_eff)

2×2 per axis, per step. Code L346-374.

```
R11 = σ²_n_x                                                          (sensor spec)
R22 = K_var·IF_eff·(â_x + ξ)² + Σ_{i=1}^d Var(δa_ram[k−i])            (intrinsic + buffered delay)
   K_var  = 2·a_cov/(2−a_cov)                                         (Isserlis × EWMA gain)
   IF_eff = 1 + 2·(s_xT²·A + 2·s_xT·s_nx·B + s_nx²·C)/(C_dpmr·s_xT + C_n·s_nx)²
            s_xT = 4kBT·â_x,  s_nx = σ²_n_x,  [A;B;C] = IF_abc (offline)   (R22_derivation S4-S6)
   ξ      = (C_n/C_dpmr)·σ²_n_x/(4kBT)                                (per-axis sensor floor)

3-guard (OR → R22 = 1e10, drop y_2):
   G1: t < t_warmup_kf            G2: σ̂²_δxr ≤ C_n·σ²_n_x (NaN)       G3: h̄ < h_bar_safe (wall)
```

Delay term uses per-step buffered sum {1,1}, NOT the stale 7-state `5·Q77` ({4,1}). The {1,1}
vs Q33-randgain {4,1} asymmetry models r_2 = n_a − Σδa_ram[k−i] (linear) vs process accumulation
(squared) — TODO: one-line r_2 derivation to confirm (§8).

---

## 7. Constants (N2: full a_pd-dependent)

`build_eq17_6state_constants.m`:
```
C_dpmr (full a_pd) = (1−a_pd)²·[ 2(1−a_pd)(1−λc)/(1−(1−a_pd)λc)
                                 + (2/(2−a_pd))·1/((1+λc)(1−(1−a_pd)λc)) ]      L103-105
   λc=0.7, a_pd=0.05 → 3.161   (a_pd→0 simplification = 2+1/(1−λc²) = 3.96; NOT used)
C_n (full a_pd)     = ...                                                       L107-110
   λc=0.7, a_pd=0.05 → 1.109   (simplification = 2/(1+λc) = 1.176; NOT used)
K_var = 2·a_cov/(2−a_cov)                                                       L117
var_da_increment_factor = 2/(1+λc)                                             L133
IF_abc = [A;B;C]  (s-weighted ACF sums, exact per-step IF_eff)                  compute_if_abc L197-229
xi_per_axis = (C_n/C_dpmr)·σ²_n_s/(4kBT)                                        L159
```
IIR (N1, separate): a_pd = mean EWMA pole (δx̄_m); a_cov = variance EWMA pole (σ̂²_δxr).

---

## 8. 往後增加 (open items / TODO)

**Framing — one correlation-structure family.** The near-wall Phase-A items below are NOT
independent bugs; they are the four off-diagonal/temporal blocks of the joint (q, r) noise
structure that the simple KF zeros out, all rooted in the SAME gain-position coupling
`δa_ram = (a·K_h/R)·Δx_ram`. Hence all scale with the wall sensitivity K_h(h̄):

| Item | Neglected correlation | KF assumption | K_h scaling |
|---|---|---|---|
| S=0 | E{q·rᵀ} (δa_ram in q3/q5 & r2; n_x in q3 & r1) | q ⊥ r | δa_ram ∝ K_h² (n_x part const ~0.16%) |
| Cov(q3,q5) | Q off-diagonal (δa_ram in q3 randgain & q5) | Q diagonal | ∝ K_h² |
| P(5,5) over-conf | a_xm innovation autocorr (ρ≈0.99) | innovation white | ∝ K_h (coupling strength) |
| Q55 near-wall | C_dpmr/ρ1 in 2/(1+λc) assume quasi-static K_h | K_h slow | cancellation valid only K_h small/slow |

K_h→0 far from wall (h̄>2-3) → all four → 0 → benign; K_h→∞ near wall (h̄→1) → all grow together.
The h_bar_safe=1.5 guard IS this boundary. Safe-region (h̄>1.5) effect: only optimality / P_a
trustworthiness / near-wall accuracy — NOT tracking or â-unbiasedness (both validated). One fix
family: model the correlation (correlated-noise predictor + off-diagonal Q + colored-meas KF +
near-wall C_dpmr/ρ1) instead of assuming white/diagonal/independent. Backlog:

- **Near-wall S = E{q·rᵀ}** — fill the S[k] slot in §1 step (1). Sources: n_x in q3 & r1;
  δa_ram in q3/q5 & r2. Scale ∝ K_h/(1−λc). ~0.16% safe region, blocking near wall.
- **r_2 weighting derivation** — confirm R22 delay {1,1} vs Q33 randgain {4,1} (§6).
- **Q33 canonical write-up** — formalize 3-component (D3) vs IEEE Path-C; ε MA(2) structure.
- **Observability rank** — `test_observability_eq17_v2.m` never executed for the −1 / combined
  form (phase4 predictions inherited from 7-state).
- **Lyapunov closed-loop oracle** — σ²_δx·(1−λc²) = C_dpmr·σ²_dXT + C_n·σ²_n_x; phase7 block-
  triangular; predict_closed_loop_var_eq17_v2 pending.
- **EKF P(5,5) over-confidence** — 10–30× under-estimate (colored a_xm innovation ρ1≈0.99).
- **1/â_x floor (LB-1)** — no divide-by-zero/negative guard in control law.
- **d=1 generalization** — Cdpmr/R22 hard-coded d=2; code has d=1 branches, transfer functions
  not derived.

---

## Notation table (Vpersonal-aligned, λc)

| Concept | Canonical (writeup) | Code var | IEEE paper |
|---|---|---|---|
| closed-loop pole | **λc** | `lambda_c` | k_c |
| sampling interval | Δt | `Ts` | Δt |
| motion gain / est | a_x / â_x | `a_x` / `a_hat` | a_x / â_x |
| desired force | f_d (f_dx) | `f_d` | f_dx |
| tracking error | δx | `delta_x` | δx |
| measured / random | δx_m / δx_r | `delta_x_m` / `dx_r` | δx_m / δx_r |
| measured gain | a_xm | `a_xm` | a_xm |
| delayed errors | δx_1, δx_2, δx_3 | `x_e(1:3)` | δx_1..3 |
| combined disturbance | **δx_D^d** | `xD_comb` / `x_e(4)` | (x_D, δx_D) separate |
| control-history sums | F_dx, dF_dx, F_1, F_2 | `Fe3_a, Fe3_da, F_1_i, F_2_i` | — |
| innovations | e_y1, e_y2 | `innov(1:2)` | e_x1, e_ax |
| gain random increment | δa_ram | `var_da_ram` (its var) | — |
| matrices | Φ, F_e, H, L, P_f, P, Q, R, S | — | F_e, H, L, P_f, P, Q, R |
| thermal var | σ²_dXT = 4kBT·a_x | `sigma2_dXT` | σ²_dxT |
| wall-normal thermal | σ²_dh = 4kBT·a_perp | `sigma2_dh` | — |
| IIR poles | a_pd (mean), a_cov (var) | `a_pd`, `a_cov` | a_var (single) |
| structural constants | C_dpmr, C_n (full a_pd) | `C_dpmr`, `C_n` | (Eq.11/12 inline) |
| color inflation | IF_eff, IF_var, IF_abc | `IF_eff`, `IF_abc` | — |
| sensor floor | ξ | `xi_per_axis` | — |
| wall sensitivity | K_h, K_h' | `K_h_axis`, `derivs.K_h_*` | — |
