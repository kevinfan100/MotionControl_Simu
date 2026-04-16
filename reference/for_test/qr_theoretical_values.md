# Q/R Theoretical Derivation — Entry-by-Entry

**Date**: 2026-04-15 (revised 2026-04-16)
**Branch**: `test/qr-paper-reference`
**Paper**: Meng et al., IEEE TIE 72(1), Jan 2025, pp. 929-938.
**Purpose**: Derive the theoretical value of every Q(i,i) and R(i,i) in the
7-state EKF, entry-by-entry, from paper equations and first principles.
Each entry gets: paper basis, derivation, theoretical numerical value,
and comparison to the code's current value.

---

## 1. Physical constants and derived quantities

From `physical_constants.m` and `user_config.m`:

| Symbol | Value | Units | Source |
|---|---|---|---|
| k_B | 1.3806503e-5 | pN*um/K | physical_constants.m |
| T | 310.15 | K | physical_constants.m |
| gamma_N | 0.0425 | pN*sec/um | physical_constants.m |
| Ts | 6.25e-4 | sec | physical_constants.m |
| a_nom = Ts/gamma_N | 1.47059e-2 | um/pN | motion gain (free-space) |
| 4*k_B*T | 1.71291e-2 | pN*um | |
| sigma2_deltaXT = 4*k_B*T*a_nom | 2.5190e-4 | um^2 | calc_ctrl_params.m:40-41 |
| lambda_c | 0.7 | - | user_config.m |
| a_cov | 0.05 | - | user_config.m |
| sigma2_n (noise ON) | 1.0e-4 | um^2 | meas_noise_std(i)^2 |
| sigma2_n (noise OFF) | 0 | - | baseline test scenario |

### 1.1 sigma2_deltaXT derivation

```
sigma2_deltaXT := 4 * k_B * T * Ts / gamma_N
               = 4 * k_B * T * a_nom
               = 2.5190e-4 um^2
```

Physical meaning: the per-step position variance from thermal force.
The simulation's thermal model (`calc_thermal_force.m:67-72`) generates
```
f_th = sqrt(4 * k_B * T * gamma_N / Ts) * randn
```
so `Var(f_th) = 4*k_B*T*gamma_N/Ts`. Through the plant dynamics
(`Delta_x = a_nom * f_th`):
```
Var(Delta_x) = a_nom^2 * Var(f_th) = (Ts/gamma_N)^2 * (4*k_B*T*gamma_N/Ts)
             = 4*k_B*T*Ts/gamma_N = sigma2_deltaXT
```

This is the natural unit for both Q and R entries. All `Qz_diag_scaling`
and `Rz_diag_scaling` values in `user_config.m` are dimensionless
multipliers on sigma2_deltaXT.

### 1.2 Paper Eq.(11)-(13) structural constants

| Symbol | Formula | Value |
|---|---|---|
| C_dpmr | 2 + 2/(1 - lambda_c^2) | 5.92157 |
| C_n (Eq.11) | (1 - lambda_c)/(1 + lambda_c) | 0.17647 |
| C_n (Eq.12) | 2/(1 + lambda_c) | 1.17647 |

These are the paper's coefficients for the IIR-filtered tracking error
variance. Note: the paper's IIR uses pole = lambda_c = 0.7 (Eq.9);
the code uses a different architecture with a_pd = 0.05 (Section 5
of writeup_architecture.tex derives the corresponding C_dpmr_eff =
3.9242 for the code's dual-layer IIR via augmented Lyapunov).

---

## 2. Q matrix — entry-by-entry derivation

State vector (per axis):
```
x = [dx1, dx2, dx3, x_D, delta_x_D, a, delta_a]
```
where dx1 = dx[k-2], dx2 = dx[k-1], dx3 = dx[k] (tracking error delay chain).

Paper's process noise model from Eq.(17):
```
q[k] = a[k] * f_T[k] * [0, 0, -1, 0, 0, 0, 0]^T
Q[k] = E[q[k] * q[k]^T]
```

Only the 3rd state (dx3) has a stochastic driver. All others are
deterministic random-walk integrators.

---

### Q(1,1) — state dx1 = dx[k-2]

**Paper basis**: Eq.(14) line 1: `dx1[k+1] = dx2[k]`. Pure delay.
No process noise enters this equation.

**Theoretical value**: **0**

**Code current**: 0. Matches paper.

---

### Q(2,2) — state dx2 = dx[k-1]

**Paper basis**: Eq.(14) line 2: `dx2[k+1] = dx3[k]`. Pure delay.

**Theoretical value**: **0**

**Code current**: 0. Matches paper.

---

### Q(3,3) — state dx3 = dx[k] (tracking error)

**Paper basis**: Eq.(14) line 3 contains the thermal force term
`-a[k] * f_T[k]`. This is the only stochastic driver in the system.
Process noise q(3) = a[k] * f_T[k].

**Derivation**:
```
Q(3,3) = Var(a[k] * f_T[k])
       = a[k]^2 * Var(f_T[k])
```
From the simulation's thermal model (calc_thermal_force.m:67):
```
Var(f_T) = 4 * k_B * T * gamma_N / Ts
```
Substituting a_nom = Ts/gamma_N:
```
Q(3,3) = (Ts/gamma_N)^2 * (4*k_B*T*gamma_N/Ts)
       = 4*k_B*T*Ts/gamma_N
       = sigma2_deltaXT
       = 2.5190e-4 um^2
```

**Theoretical value**: **sigma2_deltaXT** (scaling = **1**)

**Code current**: 1e4 * sigma2_deltaXT (scaling = 1e4). 10000x inflation.

**Gap note**: Phase 3 MC sweep (documented in `project_sigma_ratio_filter.md`)
directly tested scaling = 1 and observed EKF divergence in the near-wall
regime h_bar in [2.5, 20]. The 10000x inflation is empirically necessary
for near-wall stability; it absorbs modeling errors not represented
in the linear F_e (wall-effect nonlinearity, c_perp(h_bar) variation,
linearization bias). Q(3,3) in practice is "physical thermal + model error",
not "physical thermal" alone.

---

### Q(4,4) — state x_D (disturbance position)

**Paper basis**: Eq.(14) line 4: `x_D[k+1] = x_D[k] + delta_x_D[k]`.
Pure random-walk integrator with no explicit process noise.
Paper's q(4) = 0.

**Theoretical value**: **0** (paper provides no theoretical basis for
a nonzero value)

**Code current**: 1e-1 * sigma2_deltaXT. Nonzero.

**Gap note**: Setting Q(4,4) = 0 tells the EKF that x_D is constant.
In practice, the "effective disturbance" includes: thermal-force DC drift,
wall-effect nonlinearity entering as slowly-varying model mismatch,
and linearization residuals. Q(4,4) > 0 allows the Riccati equation
to allocate nonzero Kalman gain to the disturbance channel, enabling
the estimator to track these unmodeled drifts. Paper's Q(4,4) = 0
works only when the plant model is exact — which it is in paper's
experiment but not in our near-wall simulation.

---

### Q(5,5) — state delta_x_D (disturbance rate)

**Paper basis**: Eq.(14) line 5: `delta_x_D[k+1] = delta_x_D[k]`.
Pure integrator, no noise.

**Theoretical value**: **0**

**Code current**: 0. Matches paper.

---

### Q(6,6) — state a (motion gain)

**Paper basis**: Eq.(14) line 6: `a[k+1] = a[k] + delta_a[k]`.
Pure random-walk integrator, no explicit noise.
Paper's q(6) = 0.

**Theoretical value**: **0** (paper provides no theoretical basis for
a nonzero value)

**Code current**: 1e-4 * sigma2_deltaXT. Nonzero.

**Gap note**: Setting Q(6,6) = 0 tells the EKF that the motion gain
is constant. In the near-wall regime, a_x(h_bar) = Ts/(gamma_N * c_perp(h_bar))
varies by ~3x as h_bar goes from 20 to 2.5. Q(6,6) > 0 allows the
estimator to track this time-varying gain. Without it, a_hat is frozen
at its initial value and the controller cannot adapt to wall-proximity
changes in the drag coefficient.

---

### Q(7,7) — state delta_a (gain rate)

**Paper basis**: Eq.(14) line 7: `delta_a[k+1] = delta_a[k]`.
Pure integrator, no noise.

**Theoretical value**: **0**

**Code current**: 0. Matches paper.

---

## 3. R matrix — entry-by-entry derivation

Measurement vector (Eq.15): `y[k] = H * x[k] + v[k]`
where `H = [1 0 0 0 0 0 0; 0 0 0 0 0 1 0]` selects state 1 (dx1)
and state 6 (a). R = Cov(v).

### Code-vs-paper measurement alignment

Code (`motion_control_law_7state.m:228`):
```matlab
meas_x = [del_pm(1); a_m_k1(1)];
```
- Channel 1: `del_pm = pd_k2 - p_m` = raw measured dx[k-2] (unfiltered)
- Channel 2: `a_m_k1` = Eq.(13) output from previous step

State 1 in the code is dx1_hat = EKF estimate of true dx[k-2].
Both the state and measurement represent the same physical quantity
(2-step-delayed tracking error), so the innovation = sensor noise.
This is consistent with writeup Section 3 (Convention A per-axis form).

---

### R(1,1) — measurement dx_m[k]

**Paper basis**: Measurement channel 1 observes `dx_m[k] = dx[k-2] + n[k-2]`
(raw, 2-step-delayed measured tracking error with sensor noise n).
State 1 = dx1[k] = true dx[k-2] (no noise). Innovation = n[k-2].

**Derivation**:
```
R(1,1) = Var(n[k-2]) = sigma2_n = meas_noise_std^2
```

**Theoretical value (noise ON)**: **sigma2_n = 1e-4 um^2**
In code scaling: `1e-4 / 2.5190e-4 = **0.397**`

**Theoretical value (noise OFF)**: **0** (degenerate; requires
regularization epsilon for KF to function)

**Code current**: 1e-2 * sigma2_deltaXT = 2.52e-6 um^2 (scaling = 0.01).

**Comparison (noise ON)**: paper 0.397 vs code 0.01. Code is **40x smaller**.
Current code's R(1,1) is very small, making the EKF heavily trust the
position measurement channel. Paper's value would moderate this trust.

**Note**: Paper Eq.(15) writes `y_1 = dx_m_bar` (IIR-filtered), which
would give R(1,1) = (1-lc)/(1+lc) * sigma2_n = 0.070 scaling (5.7x
smaller than the raw interpretation). But the code's measurement is
raw (unfiltered), and writeup Section 3 follows the code, so the
raw interpretation R(1,1) = sigma2_n is the one that matches our
architecture.

---

### R(2,2) — measurement a_m[k]

**Paper basis**: Measurement channel 2 observes `a_m[k]`, back-solved
from the IIR variance via Eq.(13). State 6 = a[k] = true motion gain.
Innovation = a_m - a = estimation error from the variance-based
back-calculation.

**Derivation**:

From Eq.(13):
```
a_m[k] = (V_meas[k] - C_n * sigma2_n) / (C_dpmr * 4*k_B*T)
```
where `V_meas` is a running IIR estimate of the tracking-error variance.
The estimation error delta_a_m = a_m - a comes entirely from the noise
in `V_meas` (in steady state E[V_meas] = V_true, so E[a_m] = a):
```
Var(a_m) = Var(V_est) / (C_dpmr * 4*k_B*T)^2
```

For a first-order IIR EMA variance estimator
`V_est[k+1] = (1 - a_cov)*V_est[k] + a_cov * x[k+1]^2`
applied to white Gaussian input x with true variance V:
```
Var(V_est) = (2*a_cov / (2 - a_cov)) * V^2
```
This is the exact steady-state result (derivation: geometric sum of
squared IIR weights times Var(x^2) = 2*V^2 for Gaussian x).

With `a_cov = 0.05`: coefficient = `2*0.05/1.95 = 0.05128`.

**Autocorrelation correction** (from Task 1d Appendix A, Layer 2):

The HP residual del_pmr is NOT white — it has autocorrelation inherited
from the lambda_c pole (~5 lags). Task 1d measured the raw a_m relative
std as 44% (empirical), vs the white-input prediction of 22.6% (exact
EMA formula above). The ratio 44/22.6 = 1.95x on std, i.e., **~4x on
variance**. This factor is structural (from del_pmr autocorrelation)
and does not depend on Q/R tuning.

**Numerical values**:

Denominator: `C_dpmr * 4*k_B*T = 5.92157 * 1.71291e-2 = 0.10143 pN*um`

| Scenario | V_nom | Var(V_est) white | Var(V_est) autocorr-corrected | Var(a_m) white | Var(a_m) corrected |
|---|---|---|---|---|---|
| noise OFF | 1.4913e-3 um^2 | 1.140e-7 um^4 | 4.561e-7 um^4 | 1.108e-5 (um/pN)^2 | 4.432e-5 (um/pN)^2 |
| noise ON  | 1.6090e-3 um^2 | 1.327e-7 um^4 | 5.310e-7 um^4 | 1.290e-5 (um/pN)^2 | 5.160e-5 (um/pN)^2 |

Relative std of a_m vs a_nom (= 1.4706e-2 um/pN):
| | White | Autocorr-corrected |
|---|---|---|
| noise OFF | 22.6% | 45.3% |
| noise ON  | 24.4% | 48.8% |

The autocorr-corrected 45.3% matches Phase 2's empirical raw a_m
rel std of 44% within 3%. Note: these are raw `a_m` values (EKF
input), NOT `a_hat` (EKF output). Task 1d's chi-sq chain (Layer 3)
shows the EKF smoothing reduces this to 26% (z-axis) / 19% (x-axis)
for `a_hat`, but R(2,2) governs the raw measurement noise, not the
filtered output.

**Theoretical value (noise OFF, white-input)**:
```
R(2,2) = Var(a_m) = 1.108e-5 (um/pN)^2
```
In code scaling: `1.108e-5 / 2.519e-4 = **0.044**`

**Theoretical value (noise OFF, autocorrelation-corrected)**:
```
R(2,2) = Var(a_m) = 4.432e-5 (um/pN)^2
```
In code scaling: `4.432e-5 / 2.519e-4 = **0.176**`

**Code current**: 1.0 * sigma2_deltaXT (scaling = 1.0).

**Comparison**: paper white 0.044, paper corrected 0.176, code 1.0.
Code is 5.7x to 22.7x larger than theory. Larger R(2,2) means the
EKF trusts a_m less and relies more on the dynamics channel (F_e
coupling through dx3). Phase 3 MC sweep confirmed that R(2,2) = 100
(ignoring a_m almost entirely) gives the best a_hat precision
far from wall (18.1% RMSE), while current R(2,2) = 1.0 is a
compromise needed for near-wall regime stability.

**Which R(2,2) to cite in writeup**: the white-input value 0.044 is
the direct paper-derivable number (from Eq.13 + chi-sq statistics).
The corrected 0.176 incorporates Task 1b/1d results (autocorrelation
amplification from Section 6 autocovariance analysis). Both are
citable; the corrected value is more honest.

---

## 4. Summary table

All values in `Qz_diag_scaling` / `Rz_diag_scaling` convention
(dimensionless multipliers on sigma2_deltaXT).

### Q matrix

| Entry | State | Paper theoretical | Code current | Derivable from paper? |
|---|---|---|---|---|
| Q(1,1) | dx1 (delay) | 0 | 0 | Yes: Eq.(14) line 1, no noise |
| Q(2,2) | dx2 (delay) | 0 | 0 | Yes: Eq.(14) line 2, no noise |
| Q(3,3) | dx3 (tracking err) | **1** | 1e4 | Yes: Eq.(14)+(17), thermal force |
| Q(4,4) | x_D (disturbance) | **0** | 1e-1 | **No**: paper assumes zero, no derivation for nonzero |
| Q(5,5) | delta_x_D (dist rate) | 0 | 0 | Yes: Eq.(14) line 5, no noise |
| Q(6,6) | a (gain) | **0** | 1e-4 | **No**: paper assumes zero, no derivation for nonzero |
| Q(7,7) | delta_a (gain rate) | 0 | 0 | Yes: Eq.(14) line 7, no noise |

### R matrix (noise ON, sigma2_n = 1e-4 um^2)

| Entry | Measurement | Paper (white) | Paper (autocorr-corrected) | Code current | Derivable? |
|---|---|---|---|---|---|
| R(1,1) | dx_m (position) | **0.397** | 0.397 | 0.01 | Yes: sensor spec |
| R(2,2) | a_m (gain est) | **0.044** | **0.176** | 1.0 | Yes: Eq.(13) + chi-sq |

### R matrix (noise OFF)

| Entry | Paper (white) | Paper (corrected) | Code current |
|---|---|---|---|
| R(1,1) | 0 + epsilon | 0 + epsilon | 0.01 |
| R(2,2) | 0.044 | 0.176 | 1.0 |

---

## 5. What this means for writeup Section 3

### Fully derivable entries (5 of 9)
Q(1,1), Q(2,2), Q(5,5), Q(7,7) = 0 (trivial, from state dynamics).
Q(3,3) = sigma2_deltaXT (from thermal model + Eq.14 process noise).
R(1,1) = sigma2_n (from sensor noise specification).
R(2,2) = 0.044-0.176 * sigma2_deltaXT (from Eq.13 + EMA chi-sq statistics).

These can be stated in Section 3 as "derived from paper Eq.(X)" with
full mathematical justification.

### Not derivable from paper (2 of 9)
Q(4,4) and Q(6,6). Paper sets both to 0 (implicit modeling choice:
disturbance and gain are assumed constant within the observation window).
Code uses nonzero values (1e-1 and 1e-4 respectively) to allow the
estimator to track slowly-varying states. **No closed-form derivation
exists in the paper** for what these values should be when nonzero.

Section 3 writeup strategy for these entries:
- State that paper assumes Q(4,4) = Q(6,6) = 0
- Explain that our simulation has time-varying a(h_bar) and unmodeled
  disturbances that require nonzero Q for estimator adaptability
- Cite current values as empirical (from MC optimization, Phase 3)
- Optionally note that Q(4,4) physically represents "allowable drift
  rate of the disturbance state" and Q(6,6) represents "allowable
  rate of change of the motion gain"

### Gap entries (Q(3,3), R(1,1), R(2,2))
Paper-derived values differ from code by 1-4 orders of magnitude.
The code values are not wrong — they are regime-stability tuning:
- Q(3,3) = 1e4: absorbs wall-effect model error beyond thermal noise
- R(1,1) = 0.01: tighter than sensor spec (noise OFF baseline makes
  physical R(1,1) ≈ 0, so code uses small positive value as regularization)
- R(2,2) = 1.0: intentionally larger than theory to reduce chi-sq
  noise injection into a_hat (dynamics channel provides better
  gain information when f_d != 0)

Section 3 can present these as "theoretical baseline X, simulation
tuning Y, rationale Z" — a three-column structure parallel to what
Task 1d did for the a_hat precision floor.

---

## 6. Files

- This file: `reference/for_test/qr_theoretical_values.md`
- No code changes. No .mat regeneration.
- Dependent on: paper pp. 931-932 (Eq.11-13, 14, 17, 21),
  writeup Section 5/6 (C_dpmr_eff, beta), Task 1d Appendix A (chi-sq chain),
  Phase 3 MC sweep results (project_sigma_ratio_filter.md).
