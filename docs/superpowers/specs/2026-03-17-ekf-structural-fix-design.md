# EKF Structural Fix Design

## Problem

The 23-state EKF diverges in closed-loop due to 5 identified failure modes:

1. **g_cov overestimates by 2.4x** (EMA variance reduction not accounted for)
2. **R_lambda off by ~5000x** (used g_cov^2, needs chi-squared variance)
3. **Position EKF states destabilize control** (50x worse tracking with EKF)
4. **Double-integrator rate states drift** (del_d, del_lambda, del_theta accumulate)
5. **Theta estimation unstable at small anisotropy** (noise amplified by 1/del_lambda)

## Goal

Make the full 23-state EKF converge in closed-loop without divergence.
All states (position, disturbance, lambda, theta) must be estimated.
Performance tuning is a follow-up; this design targets stability first.

## Approach: Structural Fix (Method B)

Minimal changes to the existing 23-state architecture. Fix the specific
mechanisms that cause instability while preserving the thesis EKF structure.

### Change 0: Restore Full EKF Kalman Gain and State Update

**What:** Replace the current disabled-EKF path (L=0, direct-EMA lambda,
fixed theta) with the full 23-state Kalman update.

**Step [5] Kalman Gain Computation (restore):**
```matlab
idx_obs = [1:3, 16:17, 20:21];
Pf_HT  = Pf(:, idx_obs);                 % 23x7
HPf_HT = Pf(idx_obs, idx_obs);           % 7x7
S      = HPf_HT + R;                     % 7x7
S      = (S + S') / 2;                   % enforce symmetry
L      = Pf_HT / S;                      % 23x7, uses mldivide (not inv)
```

NaN guard retained: if `any(isnan(L(:)))`, set L=0.
Warmup gate retained: if `step_count <= warmup_steps`, set L=0.

**Step [6] State Update (restore all 9 groups):**
```matlab
V_del_p1_hat_kA1  = V_del_p2_hat                    + L(1:3,:)   * err;
V_del_p2_hat_kA1  = V_del_p3_hat                    + L(4:6,:)   * err;
V_del_p3_hat_kA1  = lambda_c * V_del_p3_hat         + L(7:9,:)   * err;
V_d_hat_kA1       = V_d_hat + V_del_d_hat           + L(10:12,:) * err;
V_del_d_hat_kA1   = V_del_d_hat                     + L(13:15,:) * err;
lamda_hat_kA1     = lamda_hat + del_lamda_hat        + L(16:17,:) * err;
del_lamda_hat_kA1 = del_lamda_hat                    + L(18:19,:) * err;
theta_hat_kA1     = theta_hat + del_theta_hat        + L(20:21,:) * err;
del_theta_hat_kA1 = del_theta_hat                    + L(22:23,:) * err;
```

This replaces the current hardcoded L=0 / direct-EMA / fixed-theta path.

### Change 1: Leaky Integrator for Rate States

**What:** Replace unit eigenvalues in F matrix for rate states with decay factor `rho_f`.

**Before:**
```matlab
F(13:15, 13:15) = eye(3);    % del_d: pure random walk
F(18:19, 18:19) = eye(2);    % del_lambda: pure random walk
F(22:23, 22:23) = eye(2);    % del_theta: pure random walk
```

**After:**
```matlab
F(13:15, 13:15) = rho_f * eye(3);    % del_d: leaky integrator
F(18:19, 18:19) = rho_f * eye(2);    % del_lambda: leaky integrator
F(22:23, 22:23) = rho_f * eye(2);    % del_theta: leaky integrator
```

**Parameter:** `rho_f = 0.995` (half-life = 138 steps = 86 ms).

**Rationale:** Rate states model the time derivative of slowly-varying
parameters. A pure random walk (rho=1) means corrections persist forever,
causing unbounded drift. Leaky integrator (rho<1) gives finite memory:
corrections decay naturally, preventing accumulation of biased estimates.
Half-life of 86 ms is fast enough to track 1 Hz trajectory dynamics but
slow enough to filter noise.

**Interaction with alpha_f:** The forgetting factor inflates Pf by
1/alpha_f per step, while rho_f damps F eigenvalues. For rate states,
the steady-state Pf contribution is proportional to rho_f^2/alpha_f
instead of 1/alpha_f, effectively reducing Pf growth for these states.
This is the desired behavior: less confidence in rate predictions.

**Plumbing:** `rho_f` added to user_config, calc_ctrl_params, CtrlBus (same
pattern as alpha_f).

### Change 2: Kalman Gain Decoupling

**What:** After computing L, zero out cross-coupling between position
measurements and parameter states.

```matlab
% Zero non-lambda columns for lambda states (keep only cols 4:5)
L(16:19, [1:3, 6:7]) = 0;
% Zero non-theta columns for theta states (keep only cols 6:7)
L(20:23, [1:3, 4:5]) = 0;
```

Position states (1:15) retain the full Kalman gain from all 7 channels.

**Rationale:** Position tracking errors (err_p) should not be interpreted
as lambda/theta changes. The position and parameter measurement channels
are physically independent (direct position vs chi-squared residual
statistics). Without decoupling, large transient tracking errors drive
lambda/theta estimates to extreme values through the off-diagonal Pf
correlations.

### Change 3: Adaptive g_cov (already implemented, preserve)

**Status:** Already in current code (lines 142-146 of motion_control_law.m).
No changes needed; listed here for completeness.

```matlab
del_pmrr_sq = mean(del_pmrr.^2);
g_cov_sq = (1 - a_cov) * g_cov_sq + a_cov * del_pmrr_sq;
g_cov = sqrt(max(g_cov_sq, 1e-20));
R(1:3, 1:3) = g_cov_sq * eye(3);
```

Initialized from the analytical formula in calc_ctrl_params.m (preserved
as seed value). The adaptive tracker converges to the correct value within
~10/a_cov = 100 steps.

### Change 4: State-Dependent R for Lambda/Theta (already implemented, preserve)

**Status:** Already in current code (lines 163-168). No changes needed.

```matlab
R(4,4) = a_cov^2 * max(lambda_hat(1), 0.1)^2;
R(5,5) = a_cov^2 * 2 * max(lambda_hat(2), 0.1)^2;
del_lam = max(abs(lambda_hat(1) - lambda_hat(2)), 0.01);
R(6,6) = a_cov^2 * max(lambda_hat(1),0.1) * max(lambda_hat(2),0.1) / del_lam^2;
R(7,7) = R(6,6);
```

### Change 5: Increased Theta Epsilon

**What:** Change anisotropy threshold from 0.01 to 0.05 in user_config.

The threshold check uses measured values (lamda_m), consistent with the
existing code:
```matlab
if abs(del_lamda_m / max(lamda_m(1), 0.01)) < epsilon
    theta_m = theta_hat;   % isotropic: no theta update
end
```

**Rationale:** At h=20, anisotropy |lambda_T - lambda_N|/lambda_T = 3%.
With epsilon=0.01, theta updates pass through and noise is amplified 33x
by the 1/del_lambda division. With epsilon=0.05, theta estimation is
disabled when anisotropy < 5%, preventing noise-driven divergence at large
h_bar. Value configurable in user_config.

### Unchanged Components

- **Q matrix:** All Q blocks (Q11-Q99) unchanged. Q88/Q99 = 0.01*Q66
  are placeholders for the static-wall case; adequate for initial
  stability testing, revisit if theta estimation shows issues.
- **H matrix:** Observation model unchanged
- **Control law:** Steps [1]-[2] unchanged
- **EMA chain:** Steps [3a]-[3c] unchanged (note: `del_pmrd_k1` stores
  `del_pmr` per the reference formula; variable name is misleading but
  the computation is correct)
- **Joseph form:** Step [7] unchanged
- **Forgetting factor:** alpha_f = 0.998 unchanged
- **Pf init:** diag([1e-2*9, 1e-4*6, 1e-4*4, 1e-4*4]) unchanged
- **Pf ceiling/floor:** [1e-12, 1e0] unchanged
- **Warmup:** ceil(3/a_pd + 3/a_prd) = 60 steps unchanged
- **State clamps:** lambda [0.05, 1.5], theta [-pi/6, pi/6] unchanged
- **Simulink model:** No changes
- **g_cov formula in calc_ctrl_params.m:** Preserved as seed for adaptive tracker

## Files Modified

| File | Changes |
|------|---------|
| `model/controller/motion_control_law.m` | Restore full 23-state EKF (Step 5: L computation, Step 6: all state updates); add rho_f to F matrix (Step 8); add L decoupling (Step 5); remove direct-EMA lambda path |
| `model/config/user_config.m` | Add `rho_f = 0.995`; change `epsilon = 0.05` |
| `model/controller/calc_ctrl_params.m` | Add `ctrl.rho_f = config.rho_f` (end of function, after g_cov) |
| `model/calc_simulation_params.m` | Add `rho_f` element to CtrlBus (element 17) |

## Test Plan

| Test | Config | Success Criteria |
|------|--------|-----------------|
| T1 | h=20, amp=1, thermal=ON, noise=OFF, 5 cycles | Completes; lambda within 20% of truth; no crash |
| T2 | h=7, amp=2, thermal=ON, noise=ON, 5 cycles | Completes; lambda converges; h_bar > 1.5 |
| T3 | h=5, amp=2.5, thermal=ON, noise=ON, 5 cycles | Completes; h_bar > h_min; tracking < 10 um RMS |
| T4 | h=7, amp=2, thermal=ON, noise=ON, **20 cycles** | Rate states bounded (no drift); Pf stable |

If T1-T3 fail, fallback: add staged activation (Method A) on top.
Staged activation phases: warmup(L=0) -> position only -> +disturbance
-> +lambda -> +theta, with configurable transition step counts.
