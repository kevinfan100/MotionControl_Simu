# Q/R Theory — Complete Algebraic Derivation

**Date**: 2026-04-22
**Scope**: Pure algebra. Positioning, 7-state EKF, per-axis physics.
**Companion docs** (numerical verification, code ↔ formula mapping):
- `qr_verification_h50_final_2026-04-22.md` — 5-seed CI at h=50
- `qr_parameter_audit_2026-04-20.md` — design-choice audit

**Global assumptions** (Phase 1 linearization, positioning):
- f_dx sensitivity `Fe(3,6) = -f_dx[k]` frozen at `f_0 = 0` (free-space DARE).
- Gaussian δx_mr (required for §9 Gaussian 4th-moment reduction).
- Q(7,7) = 0 for positioning (δa truly constant when a is stationary).
- Scalar Riccati approximation for the (a, δa) EKF block (valid under Q(7,7)=0; see §14).

---

## 1. Plant dynamics and control law (per axis)

Continuous-time plant:

```
γ(x)·ẋ = f_dx + f_T
a_x[k] ≡ T_s / γ(x[k])
```

Discrete recursion (a·f_T is the per-step thermal driver):

```
x[k+1] = x[k] + a_x[k]·(f_dx[k] + f_T[k])
```

Einstein per-axis thermal variance:

```
Var(a_x · f_T) = 4·k_B·T · a_x[k] = (a_x / a_nom) · σ²_dXT ≡ ar · σ²_dXT
σ²_dXT ≡ 4·k_B·T · a_nom,   a_nom ≡ T_s / γ_N
```

Control law (δx̂_i, x̂_D from EKF; see §3):

```
f_dx[k] = (1/â[k]) · [(x_d[k+1] − x_d[k]) + (1−λ_c)·δx̂_3[k] − x̂_D[k]]
```

Closed-loop δx_3 recursion (fluctuation frame, f_dx sensitivity dropped per scope note):

```
δx_3[k+1] = λ_c·δx_3[k] + (1−λ_c)·e_{x3}[k] − e_{xD}[k] − a_x·f_T[k]
```

---

## 2. Measurement chain and a_m estimator

Sensor-delayed position error:

```
δx_m[k] = x_d[k−2] − x_m[k]
```

IIR low-pass (pole 1−a_pd):

```
δx_md[k]  = (1−a_pd)·δx_md[k−1]  + a_pd·δx_m[k]
δx_mr[k]  = δx_m[k] − δx_md[k]
```

Secondary LP (pole 1−a_prd) and EMA variance (pole 1−a_cov):

```
δx_mrd[k]       = (1−a_prd)·δx_mrd[k−1] + a_prd·δx_mr[k]
avg_sq[k]       = (1−a_cov)·avg_sq[k−1] + a_cov·(δx_mr[k])²
σ²_δxmr[k]      = avg_sq[k] − (δx_mrd[k])²
```

a_m estimator (bias correction deferred to §10):

```
a_m[k] = (σ²_δxmr[k] − C_n·σ²_n) / (C_dpmr · 4·k_B·T)
```

`C_dpmr` and `C_n` are derived in §7.

---

## 3. 7-state EKF

State and measurement:

```
x[k] = [δx_1, δx_2, δx_3, x_D, δx_D, a, δa]ᵀ
y[k] = [δx_m[k], a_m[k]]ᵀ

       ┌ 0 1 0  0 0  0       0 ┐                    ┌ 1 0 0 0 0 0 0 ┐
       │ 0 0 1  0 0  0       0 │                    │ 0 0 0 0 0 1 0 │
       │ 0 0 1 −1 0 −f_dx    0 │                    └               ┘
Fe  =  │ 0 0 0  1 1  0       0 │           H  =  (above 2×7)
       │ 0 0 0  0 1  0       0 │
       │ 0 0 0  0 0  1       1 │
       │ 0 0 0  0 0  0       1 │
       └                       ┘
```

Steady-state gain from DARE(Fe, H, Q_kf, R_kf):

```
L  = P'·Hᵀ·(H·P'·Hᵀ + R_kf)⁻¹
P  = (I − L·H)·P'
P' = Fe·P·Feᵀ + Q_kf                          (Pf converged)
```

Error dynamics (e ≡ x − x̂, closed loop):

```
e[k+1] = A_e·e[k] − Fe·L·v[k] + w[k],     A_e ≡ Fe·(I − L·H)
```

Q_kf and R_kf are the **designer** covariances (see §12 for their relation to actual driver variance).

---

## 4. Augmented state x_aug (11-dim)

```
x_aug[k] = [δx, δx_{d1}, δx_{d2}, e_1, e_2, e_3, e_{xD}, e_{δxD}, e_a, e_{δa}, pmd_prev]ᵀ
```

Index map:

```
idx_δx    = 1     δx[k] = δx_3[k] (current tracking error)
idx_{d1}  = 2     δx[k−1]
idx_{d2}  = 3     δx[k−2]    (what δx_m sees)
idx_e     = 4..10 seven EKF error states e_1..e_7
idx_p     = 11    δx_md[k−1] (IIR LP prev state)
```

---

## 5. A_aug block structure and noise drivers

With `f_0 = 0` (scope note):

```
Row idx_δx  : A_aug(1, 1)          = λ_c
              A_aug(1, idx_e(3))   = 1 − λ_c         (e_3 at index 6)
              A_aug(1, idx_e(4))   = −1              (e_4 at index 7)

Row idx_{d1}: A_aug(2, 1)          = 1               (pure delay)
Row idx_{d2}: A_aug(3, 2)          = 1               (pure delay)

Rows idx_e  : A_aug(idx_e, idx_e)  = A_e             (7×7 EKF error block)

Row idx_p   : A_aug(11, 11)        = 1 − a_pd
              A_aug(11, 3)         = a_pd            (IIR LP: feeds on δx_{d2})
```

Noise drivers (11×1, unit-variance):

```
B_T(idx_δx)    = −1                                  (a·f_T enters δx and e_3)
B_T(idx_e(3))  = −1

B_n(idx_e)     = −Fe·L(:, 1)                         (n_p through EKF gain col 1)
B_n(idx_p)     = a_pd                                (direct into IIR LP)

B_na(idx_e)    = −Fe·L(:, 2)                         (n_a through EKF gain col 2)
```

---

## 6. Lyapunov unit-solves

For each unit-variance driver:

```
Σ_T   = A_aug · Σ_T  · A_augᵀ + B_T  · B_Tᵀ
Σ_n   = A_aug · Σ_n  · A_augᵀ + B_n  · B_nᵀ
Σ_na  = A_aug · Σ_na · A_augᵀ + B_na · B_naᵀ
```

Each Σ_X is the closed-loop state variance induced by a unit-variance injection of its corresponding driver. Physical variances are restored in §11.

---

## 7. C_dpmr and C_n from Σ_T, Σ_n (closes §2)

From §2:

```
δx_mr[k] = (1 − a_pd) · ( δx_{d2}[k] − pmd_prev[k] − n_p[k] )
```

Selector extraction (n_p independent of x_aug):

```
Var(δx_mr) = (1−a_pd)² · [ Σ(d2,d2) + Σ(p,p) − 2·Σ(d2,p) ]  +  (1−a_pd)² · σ²_n
```

Split Σ into thermal + sensor drivers (the a_m self-feedback goes into rho_a, §9):

```
Σ_{T,n} = ar·σ²_dXT · Σ_T  +  σ²_n · Σ_n
```

Match `Var(δx_mr) = C_dpmr · ar · σ²_dXT + C_n · σ²_n`:

```
┌────────────────────────────────────────────────────────────────────────────┐
│ C_dpmr = (1−a_pd)² · [ Σ_T(d2,d2) + Σ_T(p,p) − 2·Σ_T(d2,p) ]               │
│                                                                            │
│ C_n    = (1−a_pd)² · [ Σ_n(d2,d2) + Σ_n(p,p) − 2·Σ_n(d2,p) + 1 ]           │
└────────────────────────────────────────────────────────────────────────────┘
```

The `+1` in `C_n` is the direct n_p contribution to δx_mr (does not pass through e).

---

## 8. Autocorrelation ρ(L)

Selector c ∈ ℝ¹¹ for the state part of δx_mr:

```
c(idx_{d2}) =  gain,   c(idx_p) = −gain,   gain ≡ 1 − a_pd
```

State-contribution variance and lag-L autocovariance:

```
V_state  = cᵀ · Σ · c
C_state(L) = cᵀ · A_augᴸ · Σ · c          (state part only)
```

Exogenous n_p is white → no autocovariance at L ≥ 1; it only inflates the variance:

```
V_noise = gain² · σ²_n
V_total = V_state + V_noise
```

Normalized autocorrelation for L ≥ 1:

```
ρ(L) = C_state(L) / V_total
```

---

## 9. rho_a (a_m variance amplification from IIR of (δx_mr)²)

**Gaussian 4th-moment reduction (Wick/Isserlis).** For zero-mean Gaussian δx_mr let
`y[k] ≡ (δx_mr[k])² − Var(δx_mr)`. Then:

```
Cov(y[k], y[k−L]) = 2 · Cov(δx_mr[k], δx_mr[k−L])² = 2 · Var²(δx_mr) · ρ²(L)
Var(y)            = 2 · Var²(δx_mr)
⇒  ρ_y(L) = ρ²(L)                     (ρ squared from Gaussian 4th moment)
```

**IIR of autocorrelated y** (pole 1 − a_cov, gain a_cov):

```
Var(IIR_output) / Var(y) = (a_cov / (2 − a_cov)) · [ 1 + 2 · Σ_{L≥1} ρ_y(L) · (1−a_cov)ᴸ ]
```

(Derived from the double-sum identity
`Σ_{i,j≥0} (1−g)^(i+j)·ρ_y(|i−j|) = 1/(1−(1−g)²) · [1 + 2·Σ_{L≥1} ρ_y(L)·(1−g)ᴸ]`
with `g = a_cov` and `1/(1−(1−a_cov)²) = 1/(a_cov·(2−a_cov))`.)

Substitute `ρ_y(L) = ρ²(L)` and define:

```
┌────────────────────────────────────────────────────────────────┐
│ chi_sq ≡ 2 · a_cov / (2 − a_cov)                               │
│ rho_a  ≡ 1 + 2 · Σ_{L=1}^∞ ρ²(L) · (1 − a_cov)ᴸ                │
└────────────────────────────────────────────────────────────────┘
```

Combined with `Var(y) = 2·Var²(δx_mr)`:

```
┌────────────────────────────────────────────────────────────────┐
│ Var(a_m noise) = chi_sq · rho_a · Var²(δx_mr)                  │
│                                                                │
│ rel_std(a_m)/a = √( chi_sq · rho_a )                           │
│   (using Var(δx_mr)/a ≈ C_dpmr · 4·k_B·T at σ²_n = 0)          │
└────────────────────────────────────────────────────────────────┘
```

Implementation note: ρ(L) in rho_a is evaluated on Σ = Σ_aug_phys (§11), which itself contains the `chi_sq · rho_a · a² · Σ_na` term. A single fixed-point iteration (starting from rho_a = 4 to build Σ, then recomputing rho_a) converges.

---

## 10. β bias factor and bias-corrected a_m

**Bias source.** The §2 construction satisfies
`E[σ²_δxmr] = Var(δx_mr) − Var(δx_mrd)`, so it is biased low. Define:

```
β ≡ E[σ²_δxmr] / Var(δx_mr) = 1 − Var(δx_mrd)/Var(δx_mr)
```

**Var(δx_mrd)** via the same IIR identity as §9 but applied to the **1st-order** LP of δx_mr
(no squaring ⇒ linear ρ instead of ρ²):

```
Var(δx_mrd) = (a_prd / (2 − a_prd)) · Var(δx_mr) · [ 1 + 2 · Σ_{m=1}^∞ ρ(m) · (1−a_prd)ᵐ ]
```

Substitute:

```
┌────────────────────────────────────────────────────────────────────────┐
│ β(λ_c, a_prd) = 1 − (a_prd / (2 − a_prd)) ·                            │
│                     [ 1 + 2 · Σ_{m=1}^∞ ρ(m) · (1−a_prd)ᵐ ]            │
└────────────────────────────────────────────────────────────────────────┘
```

**Bias-corrected a_m (runtime)**:

```
┌────────────────────────────────────────────────────────────────┐
│ a_m[k] = ( σ²_δxmr[k] / β − C_n · σ²_n ) / ( C_dpmr · 4·k_B·T )│
└────────────────────────────────────────────────────────────────┘
```

**Asymmetry with §9**: β uses linear ρ (δx_mrd is a 1st-order IIR of δx_mr);
rho_a uses ρ² (a_m is an IIR of (δx_mr)², a 2nd-order quantity). Different dampings
too: β uses (1−a_prd); rho_a uses (1−a_cov).

---

## 11. Physical driver variances and Σ_aug_phys

Per-axis physical driver variances:

```
Var(a_axis · f_T_axis) = ar · σ²_dXT          (Einstein per axis: Var(f_T_axis) ∝ γ_axis)
Var(n_p_axis)          = σ²_n_axis            (sensor spec, per axis)
Var(n_a_axis)          = chi_sq · rho_a(axis) · a²_axis     (from §9)
```

Physical combination of §6 unit-solves:

```
┌────────────────────────────────────────────────────────────────────────────────┐
│ Σ_aug_phys = ar · σ²_dXT · Σ_T                                                 │
│            + σ²_n       · Σ_n                                                  │
│            + chi_sq · rho_a · a² · Σ_na                                        │
└────────────────────────────────────────────────────────────────────────────────┘
```

Designer Q_kf / R_kf (which shape A_e via DARE) are **not** pre-multiplied by `ar`,
`σ²_n_axis`, or `a²`. See §12 for the two-role split.

---

## 12. Q/R per-axis design (two roles)

Two distinct matrices appear. Keeping them separate avoids confusion.

**(a) Designer covariances — input to DARE, determines L.**

```
Q_kf = σ²_dXT · diag([ 0, 0, 1, 0, 0, Q66_scale, Q77_scale ])

  Q_kf(3,3) = σ²_dXT                   (shared across axes; NOT ar-scaled)
  Q_kf(6,6) = Q66_scale · σ²_dXT       (random-walk on a, design parameter)
  Q_kf(7,7) = 0                         (δa truly constant in positioning)

R_kf(axis) = σ²_dXT · diag([ R11_scale(axis), R22_scale(axis) ])

  R11_scale(axis) = σ²_n(axis) / σ²_dXT                          (sensor noise)
  R22_scale(axis) = chi_sq · rho_a(axis) · a²(axis) / σ²_dXT    (a_m noise)
```

**(b) Actual driver variances — input to Σ_aug_phys (§11).**

```
Var(a·f_T) = ar · σ²_dXT           (per-axis physics, linear ar)
Var(n_p)   = σ²_n_axis             (per-axis sensor)
Var(n_a)   = chi_sq · rho_a · a²_axis
```

**Sub-optimality note.** Designer `Q_kf(3,3) = σ²_dXT` shared across axes
does **not** equal actual `ar · σ²_dXT` when `ar ≠ 1`. The DARE-optimal `L` is
obtained only at `ar = 1` (free space, h → ∞). At h = 50, `ar ≈ 1` (near-optimal).
At h = 2.5, `ar ≈ 0.1` (significant mismatch, suboptimal L but still stable).
`Σ_aug_phys` in §11 gives the **true** closed-loop variance regardless.

---

## 13. Tracking std per axis

```
┌──────────────────────────────────────────────────────────────────┐
│ Var(δx_axis[k]) = Σ_aug_phys( idx_δx, idx_δx )                   │
│                                                                  │
│ std(δx_axis)    = √ Var(δx_axis[k])                              │
└──────────────────────────────────────────────────────────────────┘
```

---

## 14. a_hat std per axis

Two additive contributions, assumed independent:

### (I) EKF estimation term (cascaded LP approximation)

Scalar Riccati on the (a, δa) EKF block, valid when `Q(7,7) = 0`:

```
┌────────────────────────────────────────────┐
│ L_eff  ≡  √( Q(6,6) / R(2,2) )             │
└────────────────────────────────────────────┘
```

**Remark.** `L_eff` is a scalar approximation to the 2×2 DARE gain on the
(a, δa) block. Validity was verified against the full 7-state DARE gain `L(6,2)`
under `Q(7,7) = 0`; mismatch at `Q(7,7) > 0` can be large (see handoff §Fix 3).

Cascaded LP transfer (a_m pole 1 − a_cov feeding EKF LP with effective gain L_eff):

```
Var(â) / Var(a_m input) = L_eff / (L_eff + a_cov)
```

Combined with `Var(a_m)/a² = chi_sq · rho_a` (§9):

```
┌────────────────────────────────────────────────────────────────────────┐
│ (rel_std(â)_EKF)² = chi_sq · rho_a · L_eff / (L_eff + a_cov)           │
└────────────────────────────────────────────────────────────────────────┘
```

### (II) Wall-sensitivity term

`a_true(h̄) = a_nom / c(h̄)`. Linearize around `h̄_0`:

```
(a_true − a_true_0) / a_true_0  ≈  − (dc/dh̄ / c) · (h̄ − h̄_0)

h̄ = (pz + δx_z) / R      ⇒   δh̄ = δx_z / R
```

```
┌────────────────────────────────────────────────────────────────┐
│ (rel_std(â)_wall)² = (|dc/dh̄| / c)² · Var(δx_z) / R²           │
└────────────────────────────────────────────────────────────────┘
```

**Scope for `std(δx_z)`**: z-axis **tracking error** std — not 3D RMSE, not
`p_m_z` magnitude. `std(p_m_z) = std(δx_z)` holds for positioning (no
z-trajectory). With an active z-trajectory, `std(p_m_z)` absorbs trajectory
fluctuations and this formula must be re-derived.

### Total (I + II under independence)

```
┌────────────────────────────────────────────────────────────────────────────────┐
│ rel_std²(â)_axis                                                               │
│   = chi_sq · rho_a · L_eff / (L_eff + a_cov)                                   │
│   + (|dc_axis/dh̄| / c_axis)² · Var(δx_z) / R²                                  │
└────────────────────────────────────────────────────────────────────────────────┘
```

Axis mapping: `c_para` for x, y; `c_perp` for z (wall-normal convention
`w_hat = [0; 0; 1]` at θ = φ = 0).

---

## Assumptions (consolidated)

| # | Assumption | Where used | Breaks when |
|---|---|---|---|
| 1 | `Fe(3,6) = -f_dx` frozen at `f_0 = 0` | §5 A_aug | trajectory tracking; quantified by §14(II) |
| 2 | Gaussian δx_mr | §9 Wick/Isserlis | near-wall non-Gaussian tail (empirical ~5% residual) |
| 3 | `Q(7,7) = 0` | §12, §14(I) | trajectory mode (a time-varying) |
| 4 | Scalar Riccati for (a, δa) | §14(I) L_eff | `Q(7,7) > 0` (multi-state DARE dominates) |
| 5 | Fixed-point iteration converged for rho_a | §9 implementation note | typically converges in 1 iteration |
