# EKF Q/R Matrix Analysis

23-state EKF 的 Process Noise (Q) 和 Measurement Noise (R) 矩陣推導分析。
Reference: `reference/controller/Estimator_and_Control__.pdf`

---

## 1. State Vector Structure

23 states in 9 groups (wall frame V):

| Group | States     | Dim | Symbol              | Meaning            |
|-------|-----------|-----|---------------------|--------------------|
| 1     | 1:3       | 3   | V_del_p1_hat        | Position error (k-2 delay) |
| 2     | 4:6       | 3   | V_del_p2_hat        | Position error (k-1 delay) |
| 3     | 7:9       | 3   | V_del_p3_hat        | Position error (current)   |
| 4     | 10:12     | 3   | V_d_hat             | Disturbance                |
| 5     | 13:15     | 3   | V_del_d_hat         | Disturbance rate           |
| 6     | 16:17     | 2   | lambda_hat          | Drag ratio [lambda_T; lambda_N] |
| 7     | 18:19     | 2   | del_lambda_hat      | Drag ratio rate            |
| 8     | 20:21     | 2   | theta_hat           | Wall orientation           |
| 9     | 22:23     | 2   | del_theta_hat       | Wall orientation rate      |

---

## 2. Lambda Convention

From the control law (PDF p.3):
```
Delta_u = (1/lambda_hat_T) * {del_u_d + (1-lambda_c)*del_u_hat - d_hat_u}
f_u = (gamma_N / Dt) * Delta_u
```

Actual position change (from Gamma_inv dynamics):
```
Dp_u = (Dt/(gamma_N * c_para)) * f_u = (1/c_para) * Delta_u
     = 1/(c_para * lambda_hat_T) * {stuff}
```

For perfect tracking: c_para * lambda_hat_T = 1, therefore:
```
lambda_hat_T = 1/c_para   (mobility factor, < 1 near wall)
lambda_hat_N = 1/c_perp   (mobility factor, < 1 near wall)
```

This is confirmed by Q33 consistency:
- Physical thermal variance: Var(Dp_u) = sigma2_dxT / c_para = sigma2_dxT * lambda_T
- Q33 = sigma2_dxT * diag(lambda_T, lambda_T, lambda_N)  [matches when lambda = 1/c]

---

## 3. Q Matrix Analysis

### Q = blkdiag(Q11, Q22, Q33, Q44, Q55, Q66, Q77, Q88, Q99)

### 3.1 Q11, Q22 — Confirmed: Zero

States del_p1, del_p2 are pure delay lines (from F matrix rows 1-2).
No process noise: Q11 = Q22 = 0.

### 3.2 Q33 — Confirmed: Thermal Noise

Source: fluctuation-dissipation theorem.

Thermal position increment variance per step (free particle):
```
sigma2_dxT = 4 * k_B * T * Dt / gamma_N = (0.01589 um)^2
```

With wall effect (anisotropic drag):
```
Q33 = sigma2_dxT * diag(lambda_T, lambda_T, lambda_N)
    = sigma2_dxT * diag(1/c_para, 1/c_para, 1/c_perp)
```

**Status: Confirmed from first principles.**

### 3.3 Q44, Q55 — Disturbance Process Noise

**PDF formula:** Q44 = Q55 = a_pd * a_prd * Q33

**Analysis approach:** Relate d_hat to EMA innovation.

From F matrix row 3 + state update consistency:
```
d = (1 - lambda_c) * del_p3 - G_lambda * del_lambda - G_theta * del_theta
```
So d_hat is NOT an independent physical quantity. It absorbs the difference
between open-loop F matrix dynamics and designed closed-loop pole lambda_c.

The per-step innovation in d_hat comes from:
- Thermal noise contribution: (1-lambda_c)^2 * Q33 = 0.09 * Q33
- This is the UPPER BOUND (instantaneous coupling)

The PDF uses a_pd * a_prd * Q33 = 0.01 * Q33, which is ~9x smaller.

**Physical interpretation:** a_pd * a_prd acts as a noise budget fraction,
limiting d_hat's bandwidth to match the EMA deterministic filter bandwidth.
This prevents d_hat from tracking high-frequency thermal noise.

EMA single-stage innovation variance:
```
Var(Delta_y_noise) = a^2 * sigma^2 * 2/(2-a)
```
For a = 0.1: factor = 0.01 * 1.053 = 0.01053
PDF approximation: a_pd * a_prd = 0.01 (ignores the 2/(2-a) correction, ~5% error).

**Status: Design heuristic. Not derivable from first principles.**

### 3.4 Q66, Q77 — Lambda Process Noise

**PDF formula:** Q66 = a_cov * sigma2_dxT * diag(lambda_T, lambda_N); Q77 = Q66

**Physics-based approach (chain rule):**

Lambda depends on h_bar: lambda_T = 1/c_para(h_bar).
Per-step change from thermal noise:
```
Q66_physical = (d_lambda/d_h_bar)^2 * sigma2_dxT * lambda_N / R^2
```

Numerical comparison at h_bar = 2.22 (h_init=5um, R=2.25um):
```
Q66_physical = 3.39e-7      (thermal chain rule)
Q66_PDF      = 1.89e-5      (PDF formula)
Ratio:         55.9x
```

The ratio increases with h_bar (at h_bar=10: ratio = 17000x).

**Why the large gap?**
The EKF uses a random walk model for lambda (doesn't know lambda = f(h_bar)).
Q66 must be large enough for the random walk to track lambda changes from:
1. Deterministic trajectory (sinusoidal motion changes h_bar)
2. Thermal noise (captured by physics derivation, but too small alone)
3. Model mismatch (random walk vs true nonlinear dynamics)

**Chi-squared measurement noise approach:**

Lambda is measured from squared normalized residuals:
```
lambda_Tm = (1-a_cov)*lambda_hat + a_cov * (du_nr^2 + dv_nr^2)/2
```
If du_nr ~ N(0, lambda_T), then Var((du_nr^2+dv_nr^2)/2) = lambda_T^2.
Measurement innovation variance: a_cov^2 * lambda_T^2 = 5.62e-3.

Three approaches compared:
```
Physics (thermal only):    3.39e-7    (too small)
PDF heuristic:             1.89e-5    (middle ground)
Chi-squared measurement:   5.62e-3    (too large for Q)
```

**Status: Design heuristic. PDF uses sigma2_dxT as a convenient scaling factor.**

### 3.5 Q88, Q99 — Theta Process Noise

**PDF formula:** Q88 = ?; Q99 = ? (undefined)
**Code placeholder:** Q88 = Q99 = 0.01 * Q66

**Physical analysis:**
Theta represents wall orientation angles. For a static wall:
theta_true = constant, so Q88_physical = 0.

**Theta measurement noise:**

From cross-product statistics:
```
theta_xm = theta_hat + a_cov * (dv_nr * dw_nr) / delta_lambda_m
Var(e_theta) = a_cov^2 * lambda_T * lambda_N / delta_lambda^2
```

At h_bar = 2.22: Var(e_theta) = 7.33e-2. Very large because delta_lambda
is small (weak anisotropy signal).

**Proposed formula:**
```
Q88 = alpha_theta * Q66
Q99 = alpha_theta * Q77
alpha_theta = design parameter (default: 0.01)
```

Rationale for alpha_theta = 0.01:
- Lambda changes as particle moves (dλ/dh ≈ 0.1 at h_bar=2.22)
- Theta doesn't change for static wall
- alpha_theta gives theta estimation bandwidth ~100x narrower than lambda
- Small enough to prevent theta from tracking measurement noise
- Large enough to prevent Pf theta entries from collapsing to zero

**Status: Design parameter. alpha_theta = 0.01 is reasonable for static wall.**

---

## 4. R Matrix Analysis

### R = blkdiag(R_pp, R_lambda, R_theta) (7x7, block diagonal)

### 4.1 R_pp (3x3) — Position Measurement Noise

**Code formula:** R_pp = g2_cov * I3

Where g2_cov = {2 + 1/(1-lambda_c^2)} * sigma2_dxT.

**Derivation:**
g2_cov represents the steady-state variance of the 2-step delayed position
error observation, for a free particle:
```
Var(del_pm) = sigma2_dxT * lambda * (2 + 1/(1-lambda_c^2))
```
For lambda = 1 (free particle): Var = g2_cov.

**Lambda-scaled version:**
```
R_pp = g2_cov * diag(lambda_T, lambda_T, lambda_N)
```

At h_bar = 2.22: code gives 1.00e-3, scaled gives 7.50e-4 (ratio 1.3).
The difference is moderate; code's constant approximation is acceptable.

**PDF alternative (adaptive):**
```
R_hat[k] = (1-a_cov) * R_hat[k-1] + a_cov * dp_mrr[k] * dp_mrr[k]'
```
This adaptive formula tracks the actual residual covariance. More robust but
only covers the 3x3 position block.

**Status: Code formula is a reasonable constant approximation.**

### 4.2 R_lambda (2x2) — Lambda Measurement Noise

**Code formula:** R_lambda = a_cov^2 * g2_cov * I2 = 1.00e-5 * I2

**Derived formula (chi-squared statistics):**

Lambda measurement uses squared normalized residuals:
```
e_lambda_T = a_cov * [(du_nr^2+dv_nr^2)/2 - lambda_T]
Var(e_lambda_T) = a_cov^2 * lambda_T^2
```
Since Var(X^2) = 2*sigma^4 for X ~ N(0, sigma^2).

```
R_lambda_derived = a_cov^2 * diag(lambda_T^2, lambda_N^2)
```

At h_bar = 2.22: derived = 5.62e-3, code = 1.00e-5 (ratio 0.0018).

**Important note on Q/R ratio:**
Despite the 560x absolute difference, the Q66/R_lambda ratio is similar:
- Code: Q66/R_lambda = 1.89e-5 / 1.00e-5 = 1.9
- Derived: would need consistent Q66 ≈ a_cov^2 * lambda_T^2 = 5.62e-3

The Kalman gain depends primarily on Q/R ratio, so the code's behavior
may be similar despite incorrect absolute values.

**Status: Code uses heuristic scaling. Derived formula available but
requires consistent Q66 adjustment to maintain Q/R balance.**

### 4.3 R_theta (2x2) — Theta Measurement Noise

**Code formula:** R_theta = a_cov^2 * g2_cov * I2 = 1.00e-5 * I2

**Derived formula (cross-product statistics):**

Theta measurement uses cross-products:
```
e_theta_x = a_cov * (dv_nr * dw_nr) / delta_lambda
Var(e_theta_x) = a_cov^2 * lambda_T * lambda_N / delta_lambda^2
```

At h_bar = 2.22: derived = 7.33e-2, code = 1.00e-5 (ratio 0.00014).

The derived R_theta is MUCH larger because:
1. Cross-product has high variance (product of two normals)
2. Division by delta_lambda (small when anisotropy is weak) amplifies noise

**Status: Code significantly underestimates theta measurement noise.
Derived formula is position-dependent through lambda and delta_lambda.**

---

## 5. Summary

### Confirmed (first-principles derivable)
| Block | Formula | Source |
|-------|---------|--------|
| Q11   | 0       | Pure delay, no noise |
| Q22   | 0       | Pure delay, no noise |
| Q33   | sigma2_dxT * diag(lam_T, lam_T, lam_N) | Fluctuation-dissipation |
| R_pp  | g2_cov * I3 | Steady-state residual variance |

### Design heuristics (PDF formulas)
| Block | PDF Formula | Reasoning |
|-------|------------|-----------|
| Q44   | a_pd * a_prd * Q33 | EMA bandwidth matching |
| Q55   | a_pd * a_prd * Q33 | Same as Q44 |
| Q66   | a_cov * sigma2_dxT * diag(lam_T, lam_N) | Noise scaling heuristic |
| Q77   | Q66 | Same structure |
| R_lam | a_cov^2 * g2_cov * I2 | Heuristic (not chi-squared) |
| R_theta | a_cov^2 * g2_cov * I2 | Heuristic (not cross-product) |

### Proposed formulas for undefined blocks
| Block | Proposed Formula | Rationale |
|-------|-----------------|-----------|
| Q88   | alpha_theta * Q66, alpha_theta=0.01 | Static wall, slow adaptation |
| Q99   | alpha_theta * Q77, alpha_theta=0.01 | Same as Q88 |

### Statistically derived R (for future comparison)
```
R_pp    = g2_cov * I3                               (current, acceptable)
R_lam   = a_cov^2 * diag(lambda_T^2, lambda_N^2)    (chi-squared)
R_theta = a_cov^2 * lam_T*lam_N/delta_lam^2 * I2   (cross-product)
```
Note: switching to derived R requires consistent Q adjustment to preserve
Q/R ratios. Not recommended as an isolated change.

---

## 6. Key Findings

1. **Only Q11-Q33 and R_pp are derivable from first principles.** All other
   Q and R entries are design parameters that control EKF bandwidth.

2. **Q44-Q77 use sigma2_dxT as a universal scaling factor**, providing a
   convenient small number proportional to the thermal noise level. The
   EMA parameters (a_pd, a_prd, a_cov) act as bandwidth selectors.

3. **Q88/Q99 = 0.01 * Q66** is a reasonable placeholder for static wall
   simulation. For real systems with potential wall movement, alpha_theta
   should be based on expected angular change rate.

4. **The R matrix has dimensional inconsistency** (mixing um^2 with
   dimensionless states), but this doesn't affect EKF behavior because
   the Kalman gain depends on Q/R ratios, not absolute values.

5. **Lambda convention: lambda = 1/c (mobility factor)**, not c (drag factor).
   The visualization code had a bug comparing lambda_hat with c_para instead
   of 1/c_para.

---

## 7. Simulation Test Results (2026-03-16)

### 7.1 Test Configuration

```
h_init = 10 um, amplitude = 1.0 um, frequency = 1 Hz
h_bar_init = 4.44, h_bar_min_trajectory = 4.00
lambda_c = 0.7, thermal = OFF, meas_noise = OFF
```

### 7.2 Observed Failure: Pf Covariance Divergence

**Symptom:** Simulation crashes with "h_bar must be > 1" (particle hits wall).

**Root cause:** `inv(H*Pf*H' + R)` in Kalman gain computation (motion_control_law.m:166)
becomes numerically singular. RCOND degrades monotonically:

```
Step ~0-50:     RCOND ~ 1e-17   (already poor)
Step ~50-200:   RCOND ~ 1e-19
Step ~200-500:  RCOND ~ 1e-22
Step ~500+:     RCOND ~ 1e-26   → inv() output corrupted
                                → control force diverges
                                → particle pushed to wall
```

**Cause chain:**
1. `Pf = 10 * eye(23)` initialization is oversized relative to state scales
   (position error ~0.01 um vs Pf diagonal = 10)
2. `P = (I - L*H) * Pf` uses simple form — numerical errors accumulate and
   break positive-definiteness of P
3. Once P loses positive-definiteness: Pf = F*P*F' + Q amplifies the error
4. Diverging Pf → singular (H*Pf*H' + R) → garbage Kalman gain → bad control

### 7.3 lambda_c Impact Analysis

lambda_c = 0.7 is NOT the primary failure cause, but it affects EKF parameters:

| lambda_c | g2_cov factor | (1-lambda_c) | Error settling (steps) |
|----------|---------------|--------------|------------------------|
| 0.3      | 2.10          | 0.7          | ~3 (aggressive)        |
| 0.5      | 2.33          | 0.5          | ~6                     |
| 0.7      | 3.96          | 0.3          | ~10                    |
| 0.9      | 7.26          | 0.1          | ~44 (smooth)           |

- Larger lambda_c → larger g2_cov → higher observation noise floor → harder for EKF
- lambda_c = 0.7 is standard for moderate convergence
- The real issue is Pf numerical stability, not lambda_c

### 7.4 Root Causes Identified

1. **Pf initialization `10*eye(23)` is too large**
   - Position states have scale ~0.01 um, lambda ~1, theta ~0
   - Pf=10 for position means 1000x overestimate of uncertainty
   - Causes initial Kalman gains to be extremely large

2. **Simple P update `P = (I-LH)*Pf` is numerically fragile**
   - Does not preserve symmetry or positive-definiteness
   - Error compounds each step through the Riccati recursion

3. **Q/R scale mismatch across state groups**
   - Position states: Q33 ~ 1e-4 um², R_pp ~ 1e-3 um²
   - Lambda states: Q66 ~ 1e-5 (um²?), R_lam ~ 1e-5 (um²?)
   - Theta states: Q88 ~ 1e-7, R_theta ~ 1e-5
   - Mixed units create ill-conditioned Pf matrix

---

## 8. Proposed Fix and Test Plan

### 8.1 Fix: Numerical Stability (A+B approach)

**A. Pf symmetrization after each update:**
```matlab
Pf = (Pf + Pf') / 2;  % enforce symmetry after Riccati step
```

**B. Joseph form for posterior covariance:**
```matlab
% Replace: P = (I - L*H) * Pf
% With:    P = (I - L*H) * Pf * (I - L*H)' + L * R * L'
ILH = eye(23) - L * H;
P = ILH * Pf * ILH' + L * R * L';
```
This is algebraically equivalent but numerically more stable because:
- Result is always symmetric
- L*R*L' term adds positive-definite contribution, preventing P from
  going negative-definite due to floating-point errors

**C. Better Pf initialization:**
```matlab
Pf = diag([...
    1e-2*ones(1,9),  ... % del_p1,p2,p3: position scale (~0.01 um)^2
    1e-4*ones(1,6),  ... % d, del_d: disturbance scale
    1e-1*ones(1,4),  ... % lambda, del_lambda: dimensionless ~0.1
    1e-2*ones(1,4)   ... % theta, del_theta: small angles
]);
```

### 8.2 Test Plan

| Test | Config | Purpose |
|------|--------|---------|
| T1   | thermal=OFF, noise=OFF, h_init=10, amp=1 | Verify Pf doesn't diverge |
| T2   | thermal=ON, noise=OFF, h_init=7, amp=2 | EKF tracks lambda with thermal |
| T3   | thermal=ON, noise=ON, h_init=5, amp=2.5 | Full config from run_simulation |
| T4   | Same as T3, lambda_c=0.5 | Compare convergence speed |
| T5   | Same as T3, lambda_c=0.9 | Compare smoothness |

**Success criteria for each test:**
- No singular matrix warnings
- h_bar stays above h_min throughout simulation
- lambda_hat converges toward 1/c_para (true lambda)
- Tracking error < 100 nm RMS in steady state

### 8.3 Test Results After Fixes (2026-03-16)

**Control law validation (known-lambda, no EKF):**
```
h_init=5, amplitude=2.5, thermal=ON, known lambda from h_bar
CL Tracking: max=0.0 nm, RMS=0.0 nm  (PERFECT)
OL Tracking: max=3665.9 nm, RMS=2024.1 nm
Force max: 1.73 pN (reasonable)
h_bar: [1.111, 3.333] (safe)
```
**Conclusion: control law is correct. Problem is entirely in the EKF.**

**EKF stability issues found:**
1. ~~Simulink Delay(2) on p_m path~~ — RESTORED: Delay(2) is correct.
   pm[k] in PDF = physical_position[k-2] (ADC pipeline delay).
   pd_k2 provides pd[k-2] to match → δpm = desired[k-2] - physical[k-2] (same time).
2. Without thermal noise, lambda_m decays to 0 (EKF requires noise to estimate lambda)
3. During EMA warmup, tracking error contaminates lambda measurement
4. Controller-EKF positive feedback: lambda_hat drift → wrong force → more error → more drift
5. Pf off-diagonal elements grow unboundedly even with diagonal clamping
6. Full warmup (L=0 for 100 steps) delays but doesn't prevent post-warmup divergence

**Applied fixes (in motion_control_law.m):**
- Joseph form P update (numerically stable)
- Pf symmetrization after forecast
- Scaled Pf initialization (per state group)
- Lambda clamp [0.05, inf] in state update
- Lambda safe clamp [0.1, inf] in control law
- Force saturation at 1 pN
- S regularization and NaN guard
- Warmup period (L=0 for first 100 steps)
- Pf diagonal clamping [1e-12, 1e2]

**These fixes prevent crashes but do NOT make the EKF converge.**
The fundamental issue is EKF stability, not just numerical robustness.

### 8.4 Verification After Fix

After passing T1-T3, re-examine:
1. Lambda_hat convergence plot (Tab 6) — now with correct 1/c_para comparison
2. Theta_hat convergence (Tab 7) — should stay near zero for static wall
3. Tracking error statistics (Tab 8) — quantify improvement
4. Pf condition number over time — should stabilize, not diverge
