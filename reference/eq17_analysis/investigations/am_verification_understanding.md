# Unified Understanding: p_m → a_xm Chain and C_dpmr Closed-Loop Variance

**Purpose:** Reference document for designing the verification that p_m correctly
propagates through the IIR chain to a_xm, and that the formula
`sigma2_dxr = C_dpmr*4kBT*a + C_n*sigma2_n_s` holds in simulation data.

**Branch:** `test/motion-test`
**Controller:** 6-state EKF, `RevisedControl_Vpersonal` / paper-2023 Eq.17 control law

**Sources consolidated:** controller+axm-chain, driver+runner+logging,
gain-oracle-AB+branch, plant-physics+ground-truth, var-vs-a-theory,
existing-verify+figure-tooling.

---

## 1. 6-State Simulation — Overall Architecture

### 1.1 Key Files

| Role | Path |
|---|---|
| Time-stepping driver | `model/dual_track/run_pure_simulation.m` |
| 6-state EKF controller | `model/controller/motion_control_law_eq17_6state.m` (658 lines) |
| Offline constants builder | `model/controller/build_eq17_6state_constants.m` (229 lines) |
| ODE4 inner stepper | `model/dual_track/step_dynamics.m` |
| Wall corrections | `model/wall_effect/calc_correction_functions.m` |
| Mobility matrix | `model/wall_effect/calc_gamma_inv.m` |
| Thermal force | `model/thermal_force/calc_thermal_force.m` |
| Trajectory generator | `model/trajectory/trajectory_generator.m` |
| Physical constants | `model/config/physical_constants.m` |
| Default config | `model/config/user_config.m` |
| End-to-end integration test | `test_script/integration/verify_eq17_6state.m` |
| A/B oracle runner | `test_script/integration/compare_gain_oracle_6state.m` |
| A/B oracle analyzer | `test_script/integration/analyze_gain_oracle_6state.m` |

### 1.2 Data Flow for a Single Simulation

```
user_config()                         # default config struct
  + overrides (h_init, T_sim, lc, …)
  + opts (seed, collect_diag, gain_oracle)
           |
           v
run_pure_simulation(config, opts)
  [init] rng(opts.seed) -> deterministic seed chain
         calc_simulation_params(config) -> P struct
         build_eq17_6state_constants(…) -> ctrl_const   [C_dpmr, C_n, K_var, xi, …]
  [loop, k=1..N]
    (a) trajectory_generator(k,P) -> del_pd_k, pd_k     [p_d[k+1], p_d[k]]
    (b) p_m_delayed = p_m_buffer(:,1)                   [p_m[k-2] to controller]
    (c) prepare a_override_k (oracle arm A, or [] arm B)
    (d) [f_d_k, ekf_k, diag_k] = motion_control_law_eq17_6state(
             del_pd_k, pd_k, p_m_delayed, P, ctrl_const, a_override_k)
    (e) f_th_k = calc_thermal_force(p_curr, P)
    (f) F_total = f_d_k + f_th_k
    (g) p_curr = step_dynamics(p_curr, F_total, P, Ts)  [ODE4, 62-63 substeps]
    (h) p_m_raw = p_curr + meas_noise                   [n_i ~ N(0,sigma2_n_i)]
    (i) p_m_buffer shift: old[k-2] out, new raw in
    (j) log: p_true_out(k,:)=p_curr, p_m_out(k,:)=p_m_raw, f_d_out, F_th_out,
             ekf_out, a_true_out, diag (if opts.collect_diag)
```

Controller dispatch is `is_6state = strcmpi(config.eq17_variant, '6state')`
(`run_pure_simulation.m:184-202`).

### 1.3 Time-Series Outputs

#### Primary simOut fields (always available, mirror Simulink ToWorkspace)

| Field | Shape | Units | Contents |
|---|---|---|---|
| `tout` | [N×1] | s | t=0, Ts, …, (N-1)Ts |
| `p_d_out` | [N×3] | um | desired trajectory pd[k] |
| `p_m_out` | [N×3] | um | measured = p_true + noise (pre-delay buffer) |
| `p_true_out` | [N×3] | um | noise-free ground truth (post-ODE4) |
| `a_true_out` | [N×3] | um/pN | true gain at pre-integration position |
| `f_d_out` | [N×3] | pN | control force applied |
| `F_th_out` | [N×3] | pN | thermal force |
| `ekf_out` | [N×4] | um/pN, — | [a_hat_x, a_hat_z, a_hat_y, h_bar] — CAUTION: col 2=z, col 3=y |
| `ctrl_const` | struct | — | offline constants |
| `meta` | struct | — | config, params, seed, driver version |

#### simOut.diag fields (opts.collect_diag=true), all [N×3] unless noted

| Field | Units | Contents |
|---|---|---|
| `a_hat` | um/pN | EKF posterior a_x (slot 5) |
| `a_xm` | um/pN | IIR-inverted gain measurement y_2 |
| `sigma2_dxr_hat` | um² | EWMA variance of dx_r |
| `dx_r` | um | IIR HP residual: delta_x_m - dx_bar_m |
| `delta_x_m` | um | y1: p_d[k-2] - p_m[k] |
| `x_D_hat` | um | EKF posterior delta_x_D^d (slot 4) |
| `delta_a_hat` | um/pN | EKF posterior delta_a_x (slot 6) |
| `delta_x_hat_1` | um | EKF posterior dx_hat_1 (slot 1: dx[k-2] estimate) |
| `P_a` | (um/pN)² | P(5,5): posterior variance of a_x |
| `P_dx` | um² | P(3,3): posterior variance of dx_3 |
| `P_dx1` | um² | P(1,1): posterior variance of dx_1 |
| `innovation_y2` | um/pN | EKF innovation on y2 (0 when gated) |
| `K_kf_a_y2` | — | Kalman gain K(5,2) (0 when gated) |
| `K_kf_dx_y1` | — | Kalman gain K(3,1) |
| `gate_active` | bool | true = any guard firing, y2 off |
| `guards_individual` | [N×3×3] | dim2=guard index (1=G1,2=G2,3=G3), dim3=axis |
| `h_bar` | [N×1] | — | h_bar from measured p_m |
| `a_ctrl_used` | um/pN | gain actually used by control law |

### 1.4 RNG Seeding

```
rng(opts.seed)
  -> calc_simulation_params -> calc_thermal_params:  thermal.seed = randi(...)
                             -> calc_ctrl_params:    meas_noise_seed = randi(...)
  -> calc_thermal_force (first call): rng(thermal.seed), then consumes
  -> driver loop randn: global rng stream -> measurement noise
```

One `opts.seed` fully determines both streams. Deterministic run: set
`thermal_enable=false`, `meas_noise_enable=false` (used with `seed=0` in
`compare_gain_oracle_6state.m:89`).

---

## 2. The p_m → a_xm Chain

### 2.1 Step-by-Step with Exact Formulas

All steps happen inside `motion_control_law_eq17_6state.m`, Section [1]
(lines 303-306), after delta_x_m is formed at lines 267-275.

**Step 0 — Form delayed tracking error** (line 275):
```
delta_x_m[k] = pd_km2 - p_m        (3x1, [um])
```
`pd_km2` is `p_d[k-2]` from persistent buffer. Buffer shift: each step
`pd_km2 <- pd_km1 <- pd` (line 515). d_delay=2 hardwired.

**Step 1 — IIR LP mean removal** (line 303):
```
dx_bar_m[k] = (1 - a_pd) * dx_bar_m[k-1] + a_pd * delta_x_m[k]
```
EWMA with pole (1-a_pd). Default a_pd = 0.05. Time constant tau = 1/a_pd = 20
steps. Persistent dx_bar_m (3x1) initialized to zero (line 203); may be
prefilled for warm-up (see Section 2.4).

**Step 2 — HP residual** (line 304):
```
dx_r[k] = delta_x_m[k] - dx_bar_m[k]
```
Zero-mean fluctuation. HP transfer function: `H_HP(z) = (1-a_pd)*(1-z^-1) /
(1-(1-a_pd)*z^-1)`. Zero at z=1 kills DC.

**Step 3 — EWMA variance of dx_r** (line 305):
```
sigma2_dxr_hat[k] = (1 - a_cov) * sigma2_dxr_hat[k-1] + a_cov * dx_r[k].^2
```
Default a_cov = 0.05. Effective window ~1/a_cov = 20 samples (12.5 ms at
1600 Hz). Note: a_pd = a_cov by default (same coefficient for both EWMAs).
Persistent initialized via prefill (see Section 2.4).

**Step 4 — Linear inversion** (line 306):
```
a_xm[k] = (sigma2_dxr_hat[k] - C_n * sigma2_n_s) / (C_dpmr * 4 * kBT)
```
**Not** a sqrt — this is the linear inverse of the steady-state identity
`sigma2_dxr = C_dpmr * 4kBT * a_x + C_n * sigma2_n_s` (see Section 3).

### 2.2 Constants in the a_xm Formula

All computed once in `build_eq17_6state_constants.m`.

**C_dpmr and C_n — full a_pd-dependent closed form** (lines 98-110).
The 6-state builder uses the **full** form, not the a_pd->0 simplification:

```matlab
one_m_apd  = 1 - a_pd;
denom_pole = 1 - (1-a_pd)*lc;
C_dpmr = (1-a_pd)^2 * (
    2*(1-a_pd)*(1-lc)/denom_pole
  + (2/(2-a_pd)) * 1/((1+lc)*denom_pole) );

C_n = (2*(1-a_pd)^2/(2-a_pd)) * (
    1
  + (1-a_pd)^2*a_pd*(1-lc)/denom_pole
  + (1-lc)^2/((1+lc)*denom_pole) );
```

Numeric values at **lc=0.7, a_pd=0.05**:

| Constant | Full (a_pd=0.05) | Simplified (a_pd->0) | Error if wrong |
|---|---|---|---|
| C_dpmr | **3.1610** | 3.9608 | -20% denominator -> a_xm 25% high |
| C_n | **1.1093** | 1.1765 | small (< 1% effect on thermal-dominated a_xm) |

**4kBT**: `kBT = params.ctrl.k_B * params.ctrl.T` (line 116).
`k_B = 1.3806503e-5 pN*um/K`, `T = 310.15 K`. So `4kBT ≈ 1.714e-4 pN*um`.

**sigma2_n_s**: per-axis sensor noise variance, 3x1 (line 119).

**xi_per_axis** — sensor floor in gain units (line 159):
```
xi_per_axis = (C_n / C_dpmr) * sigma2_n_s / (4*kBT)     (3x1)
```
When `sigma2_dxr_hat <= C_n * sigma2_n_s`, a_xm <= 0 -> Guard G2 fires.

**K_var** — EWMA-squaring prefactor (line 117):
```
K_var = 2 * a_cov / (2 - a_cov)      % = 0.05128 at a_cov=0.05
```
Derived from Isserlis theorem: Var(X^2) = 2*sigma^4, times EWMA noise gain
a_cov/(2-a_cov).

**var_da_inc_factor** (line 133):
```
var_da_inc_factor = 2 / (1 + lc)     % = 1.1765 at lc=0.7
```
Absorbs the ARMA inflation of closed-loop variance times (1-rho1) increment
factor. Earlier i.i.d. reading used factor 2, over-estimating by (1+lc)=1.7x.

### 2.3 How a_xm Enters the EKF

**Measurement matrix H** (6x2, line 175 for Riccati init, line 438 per-step):
```
H = [1  0  0  0  0   0  ]    % row 1: observes slot 1 = delta_x[k-2]
    [0  0  0  0  1  -2  ]    % row 2: observes a_x - d*delta_a_x  (d=2)
```
Row 2 physical interpretation: a_xm reflects a_x[k-d] (the gain d steps ago,
from sensor delay). Linear model `a_x[k-d] ≈ a_x[k] - d*delta_a_x[k]`
yields H(2,:) = [0,0,0,0,1,-d].

**Measurement vector** (line 477 when gate off):
```
y_use = [delta_x_m(ax);  a_xm(ax)]
```

**Innovation for y2** (line 505):
```
innov_y2 = a_xm(ax) - (x_pred(5) - d_delay * x_pred(6))
```

**EKF Kalman gain for slot 5**: `K_kf(5,2)` = K_a_y2 (line 503). Logged
in `diag.K_kf_a_y2`.

### 2.4 IIR Warm-Up / Prefill

Default: `iir_warmup_mode = 'prefill'`, `t_warmup_kf = 0`.

In prefill mode, `sigma2_dxr_hat` is pre-seeded to its steady-state value
at initialization (line 209):
```
sigma2_dxr_hat_init = 4*kBT * a_x_init * C_dpmr + C_n * sigma2_n_s
```
This makes G1 effectively disabled (G1 = `t_now < 0`, never true).
Legacy mode (warmup_count=2): G1 fires for first 2 steps.

### 2.5 Three-Guard Gating of y2 (lines 418-428)

```matlab
G1 = (t_now < t_warmup_kf)                                  % warmup gate
G2 = (sigma2_dxr_hat_new(ax) - C_n*sigma2_n_s(ax) <= 0)    % NaN/sign guard
G3 = (h_bar < h_bar_safe)                                   % wall gate
gate_off(ax) = G1 || G2 || G3;
```

When `gate_off(ax)`:
```
R22_i = R_OFF = 1e10     [line 425]
```
EKF uses only H row 1 (line 473). Additionally during G1, K_kf rows 5 and 6
are zeroed (lines 488-490) — gain slots not updated from any measurement.

| Guard | Default trigger | Physical rationale |
|---|---|---|
| G1 | t < t_warmup_kf (=0 in prefill) | IIR transient; disabled by prefill |
| G2 | sigma2_dxr_hat <= C_n*sigma2_n_s | a_xm <= 0, nonsensical gain |
| G3 | h_bar < h_bar_safe (=1.5 default) | Lubrication regime: K_h large, a_x changes fast |

For Round-2 gate-free experiment: `h_bar_safe = 1` -> G3 never fires (trough
h_bar = 1.2 > 1).

**Gate duty cycle in well-conditioned runs:** G1 disabled (prefill), G2
fires only if variance transiently drops below noise floor, G3 fires only
near wall. In osc_aggr scenarios with h_bottom=2.7 um, G3 fires every cycle
near the trough (Round-1). In Round-2 gate-free, G3 never fires.

---

## 3. Var-of-Motion vs Gain (a) Theory

### 3.1 The Full Set of Variance Constants

#### C_delta_x = 2 + 1/(1-lc^2) — raw tracking error thermal coefficient

```
sigma2_dx = C_delta_x * sigma2_dXT + [(1-lc)/(1+lc)] * sigma2_n_s
C_delta_x = 2 + 1/(1-lc^2)       % = 3.961 at lc=0.7
```

`sigma2_dXT = 4*kBT*a_x` is the per-step thermal position variance.

Derivation: backsubstitute `delta_x[k] = -sum_i lc^i * epsilon[k-1-i]`.
`epsilon` is MA(2) on thermal: f_T[m] appears in epsilon[m], epsilon[m+1],
epsilon[m+2] with amplitude a_x. The 3 most recent past f_T each contribute
coefficient 1^2; older ones decay geometrically. Sum:
`{1^2 + 1^2 + sum_{j>=0} lc^{2j}} = 2 + 1/(1-lc^2)`.
(`phase2_C_dpmr_C_n_derivation.md` lines 181-188)

#### C_dpmr — full a_pd-dependent (canonical for code, EWMA-variance inversion)

```
sigma2_dxr = C_dpmr * sigma2_dXT + C_n * sigma2_n_s
```

`sigma2_dxr ≈ sigma2_dx + sigma2_n_s` (in steady state with HP filter
converged). C_dpmr is the thermal coefficient of the HP residual variance.

Full closed form: see Section 2.2, `Cdpmr_Cn_derivation.tex §3`.
At lc=0.7, a_pd=0.05: **C_dpmr = 3.161**.

#### Why C_dpmr < C_delta_x

The HP filter zero at z=1 and pole at z=1-a_pd removes some power from
delta_x before squaring:
- LP time constant tau_LP = 1/a_pd = 20 steps
- AR(1) corner of delta_x: (1-lc)/(2*pi) ~= 0.048 rad/sample
- HP corner: a_pd/(2*pi) ~= 0.008 rad/sample (well below AR(1) corner)

The HP removes only the ultra-low-frequency tail of delta_x spectrum:

| a_pd | C_dpmr | Reduction vs 3.961 |
|---|---|---|
| 0 (limit) | 3.961 | 0% |
| 0.05 | 3.161 | 20% |
| 0.10 | 2.96 | 25% |
| 0.20 | 2.64 | 33% |

The 20% reduction at a_pd=0.05 means using the simplified 3.961 in the
inversion denominator produces a_xm ~25% too high systematically. This was
the root cause of the historical -20% a_hat bias when the wrong C_dpmr
was used (`cdpmr_closed_form_verify_5seed.md`).

#### C_n — sensor contribution coefficient

Full form: see Section 2.2, `Cdpmr_Cn_derivation.tex §4`.
At lc=0.7, a_pd=0.05: **C_n = 1.109**. Simplified (a_pd->0): 1.176.
Sensor noise n_x is white, so the HP removes less of its power: C_n
changes less than C_dpmr with a_pd.

**Zero-mean theorem**: because H_HP(z=1)=0, dx_r is exactly zero-mean
regardless of a_pd. Therefore `E[sigma2_dxr_hat] = Var(dx_r)` without
bias (in steady state). The a_xm inversion is unbiased in expectation when
`a_hat = a_true`.

#### Q55 = Var(delta_a_ram) — 6-state gain random increment variance

```
Q55 = [2/(1+lc)] * (a_x * K_h_axis / R)^2 * sigma2_dh
sigma2_dh = 4*kBT*a_perp_meas     (wall-normal thermal variance per step)
K_h = (1/c)*(dc/d_hbar)           (wall sensitivity function)
```

Zero when wall disabled (K_h=0). Grows near wall where K_h diverges.
Empirically validated: emp/closed_form ratio = 0.989-1.043 across osc/gon/goff
windows at Round-2 dev 2 Hz (`eba78df` A1 dynamic check).

(The 7-state equivalent is Q77 = Dt^4 * a_x^2 * {Term A + Term B} from
`phase5_Q_matrix_derivation.md §6`; not used in 6-state.)

#### IF_eff — color inflation factor for R(2,2)

Exact form (`R22_derivation.tex §6`, eq.12):
```
IF_eff = 1 + 2*[rho^2(1)*s + rho^2(2)*s^2
              + c_a^2*(a^2*s)^3/(1-a^2*s)
              + 2*c_a*c_b*(a*b*s)^3/(1-a*b*s)
              + c_b^2*(b^2*s)^3/(1-b^2*s)]
where s = 1-a_cov,  a = 1-a_pd,  b = lc
```

The three-term geometric tail arises from two poles of F_T: alpha=(1-a_pd) and
beta=lc. Approximate: IF_eff ~= 3.84 at lc=0.7, a_pd=0.05, thermal-dominated
(`phase2_IF_var_dpr_derivation.md §4`). Computed per-step in code via
precomputed A,B,C sums (`build_eq17_6state_constants.m` lines 197-228).

### 3.2 Closed-Loop Dynamics (Eq.19 Form)

Under paper Eq.17 control law with Sigma-f_d retained (v2 "full" form),
the tracking error satisfies:
```
delta_x[k+1] = lc * delta_x[k] - epsilon[k]
```

`epsilon[k]` is MA(2) on thermal, white on sensor:
```
epsilon[k] = (1-lc)*n_x[k]                    (iv) current sensor
           + a_x*f_T[k]                        (i)  current thermal
           + (1-lc)*a_x*f_T[k-1]               (ii) lag-1 thermal
           + (1-lc)*a_x*f_T[k-2]               (iii) lag-2 thermal
```

Cross-step covariances of epsilon:
```
gamma_e(0)   = (1-lc)^2*sigma2_n + sigma2_dXT*{1 + 2*(1-lc)^2}
gamma_e(1)   = (1-lc)*sigma2_dXT*(2-lc)
gamma_e(2)   = (1-lc)*sigma2_dXT
gamma_e(>=3) = 0                   (MA(2) tail ends)
```

### 3.3 The sigma2_dxr Identity and When It Is Exact

**Identity:**
```
sigma2_dxr = C_dpmr * 4*kBT*a_x + C_n * sigma2_n_s
```

**Exact when ALL of the following hold:**

| Assumption | What breaks it |
|---|---|
| A1: a_hat = a_true, x_D_hat = x_D (ideal control) | Estimation error -> pole shifts from lc -> self-consistent bias (below) |
| A2: Sigma-f_d retained in control law (v2 form) | Without it, epsilon has different MA structure -> wrong C_dpmr/C_n |
| A3: Stationarity (constant a_x) | Oscillatory motion: a_x(t) time-varying; formula is instantaneous approximation |
| A4: sigma2_w_fD = 0 (no disturbance innovation) | x_D drift adds variance to delta_x |
| A5: Gaussian white noise sources | Non-Gaussian changes higher moments in R22 |
| A6: HP filter converged (IIR warm-up complete) | Transient: sigma2_dxr_hat below steady-state -> G1 or G2 activates |

**Self-consistent bias when a_hat != a_true (most important caveat):**

When a_hat != a_true, the control law applies force `f_d = (1/a_hat)*{bracket}`,
so the effective gain applied to the plant is `a_true/a_hat`. The actual
closed-loop pole shifts from lc. This changes sigma2_dx, which changes
sigma2_dxr_hat, which changes a_xm, which changes a_hat. The system reaches a
new equilibrium at a_hat != a_true.

In the 7-state implementation this caused 29-44% systematic bias because F_e's
Row 3 error-coupling terms (-F_dx*e_ax + dF_dx*e_dax) were used for mean
propagation, injecting spurious offsets. The 6-state Vpersonal architecture
explicitly separates Phi (deterministic-map predict, for mean) from F_e (full
Jacobian, for covariance only), eliminating this (`kf_canonical_spec.md §3`).
Result: 7-state a_hat bias ~6-8%, 6-state a_hat bias ~1%.

### 3.4 Why C_dpmr and C_delta_x Are Different

```
sigma2_dx  = C_delta_x * sigma2_dXT + [(1-lc)/(1+lc)] * sigma2_n_s
sigma2_dxr = sigma2_dx + sigma2_n_s
           = C_dpmr * sigma2_dXT + C_n * sigma2_n_s
```

Consistency check:
```
C_n = (1-lc)/(1+lc) + 1  (= 0.176 + 1 = 1.176 for lc=0.7, a_pd->0 limit)
```

For finite a_pd, the HP filter partially removes variance from delta_x before
adding sigma2_n_s back, so C_dpmr < C_delta_x but C_n approaches 1 + (1-lc)/(1+lc).
At a_pd=0.05: C_dpmr = 3.161 < C_delta_x = 3.961 (20% lower), while C_n = 1.109.

### 3.5 Lyapunov Oracle

**Structure** (`phase7_lyapunov_bench.md §3`): the augmented 11-dimensional
system is block-triangular. Because the Eq.17 control law uses raw delta_x_m
(not delta_x_hat_3), the KF estimation error of the delta_x chain does NOT
feed back into truth dynamics. This means:

**Step 1:** 4x4 sub-Lyapunov for estimation errors
[e_xD, e_dxD, e_a, e_da] (decoupled from truth dynamics) -> gives sigma2_e_xD
and sigma2_e_a.

**Step 2:** 3x3 truth Lyapunov for [dx_1, dx_2, dx_3]:
```
sigma2_dx * (1-lc^2) = C_dpmr * sigma2_dXT + C_n * sigma2_n_s
                     + sigma2_e_xD                          [disturbance error]
                     + E[bracket^2]/a_x^2 * sigma2_e_a      [gain error]
```

For positioning at h=50 um baseline: corrections from e_xD and e_a are < 0.1%.

**Primary oracle (positioning, ideal estimation):**
```
sigma_dx_oracle = sqrt( (C_dpmr * 4kBT*a_x + C_n * sigma2_n_s) / (1-lc^2) )
```

**a_xm oracle** (in steady state, perfect estimation):
```
E[a_xm] = a_x
E[sigma2_dxr_hat] = C_dpmr * 4*kBT*a_x + C_n * sigma2_n_s
```

**Relative noise of a_xm:**
```
sigma(a_xm) / a_x ~= sqrt(K_var * IF_eff)
                  ~= sqrt(0.00513 * 3.84)
                  ~= 14%   (thermal-dominated, lc=0.7, a_pd=0.05)
```
This is the chi-squared floor (~42% reported elsewhere refers to the chi-squared
contribution to sigma2_dxr relative to the mean, i.e. sqrt(2*K_var*IF_eff) /
the relative signal-to-noise in the squared domain).

---

## 4. a_true vs a_hat (A/B Arm) Infrastructure

### 4.1 Arm Definitions and Wiring

| | Arm A (oracle) | Arm B (production) |
|---|---|---|
| gain in control law | a_true[k] from p_curr (noise-free, zero-lag) | EKF posterior a_hat[k-1] |
| opt flag | `opts.gain_oracle = true` | `opts.gain_oracle = false` (default) |
| inside controller | `a_ctrl = a_ctrl_override` (lines 255-259) | `a_ctrl = a_hat` |
| EKF activity | runs fully, slot 5 estimates a_hat (not used in law) | same |
| diag.a_ctrl_used | = a_true_out[k] exactly | = a_hat[k-1] exactly |

Wiring in driver (`run_pure_simulation.m:319-330`):
```matlab
a_true_k = [a_nom_drv/c_para_k; a_nom_drv/c_para_k; a_nom_drv/c_perp_k];
if opts.gain_oracle
    a_override_k = a_true_k;   % Arm A
else
    a_override_k = [];          % Arm B
end
```

**Critical:** the EKF is always active regardless of arm. All IIR signals
(dx_bar_m, sigma2_dxr_hat, a_xm) are computed every step. Arm A isolates
control-law gain error from estimation.

**suppress_xD** (both arms, `compare_gain_oracle_6state.m:159`):
```matlab
cfg.suppress_xD = true;
```
Inside controller (lines 261-264): `xD_for_ctrl = zeros(3,1)` when true.
EKF slot 4 (delta_x_D^d) is still estimated normally; only its contribution
to the control law is zeroed.

### 4.2 CRN Seeds

Same seed -> same underlying randn sequences for both arms. CRN pairing enables
meaningful paired B/A ratios across seeds.

**Seeds:**
- verify_eq17_6state.m: seeds = 1:5 (five-seed regression)
- compare_gain_oracle_6state.m: default seeds = 1:100 (oracle experiment)
- Deterministic: seed = 0, thermal_enable=false, meas_noise_enable=false

### 4.3 det/ram Decomposition

**Fundamental identity** (design doc §6.0):
```
e[k]      = p_d_out[k+1] - p_true_out[k]          (aligned tracking error)
ram_s[k]  = e_s[k] - e_det[k]
          = p_true_det[k] - p_true_s[k]           (p_d cancels)
```

**det extraction — arm-asymmetric** (critical, design §12.0):
- **Arm A:** uses noise-free det run (thermal=false, noise=false). Valid because
  G2 does not latch (IIR sees true signal under oracle gain).
- **Arm B:** uses **ensemble mean of p_true over non-diverged noisy seeds**
  (pointwise per time index). The noise-free det run is INVALID for arm B
  because in a no-noise run, G2 latches (sigma2_dxr -> 0, y_2 shuts off),
  putting arm B on a structurally different (y1-only) path. Cross-validation
  showed 26-28 nm rms discrepancy at 2/5 Hz.

**Deflation correction** (design §12.0 item 2): ensemble mean is formed from
the same seeds it centres, so ram std is biased by factor sqrt(1 - 1/Ns).
All z-axis sd values divided by this factor. x-axis is exempt.

**x-direct ram** (design §12.3): mirror symmetry guarantees det_x ≡ 0, so the
raw aligned error IS the ram on axis 1 (x). No ensemble subtraction, no
deflation correction.

### 4.4 Trajectory (osc_aggr)

4-phase: hold 0.5 s -> cosine descent 1.0 s -> oscillation -> hold tail 0.5 s.

| param | Round-1 | Round-2 (gate-free) |
|---|---|---|
| h_init | 50 um (h_bar ~22) | same |
| h_bottom | 2.7 um (h_bar 1.2) | same |
| amplitude | 2.5 um | same |
| h_bar_safe | 1.5 (G3 crosses every cycle) | **1** (G3 never fires) |
| T_sim | 7.0 s | 4.0 s |
| seeds | 1:20 | 1:100 |
| n_cyc_per_s | 5 (osc = 5 s) | 2 (osc = 2 s) |

**Why gate-free for Round-2:** with h_bar_safe=1, G3 never fires even at
trough h_bar=1.2. EKF runs full dual-feedback throughout the near-wall region.
Arm-B det run diverges under this condition at all frequencies (confirmed by
probe) — this is a legitimate experimental finding about arm B, not an
infrastructure failure.

### 4.5 What Is Directly Reusable for a_xm Verification

All of the following are already in analysis.mat files from Round-1 (f=1/2/5
Hz, 20 seeds) and Round-2 dev (f=2 Hz, probe):

- `A.gain.a_pd` — a_pd skeleton along desired trajectory
  (`a_nom/C_i(h_bar_d)`, designer-known)
- `A.gain.Kh_pd` — wall sensitivity K_h along desired trajectory
- `A.ahat.ens_mean` — ensemble mean of a_hat (arm B noisy seeds)
- `A.noisy_det` — arm B ensemble det reconstruction
- `diag.a_xm` per run (accessible as `runs.B.noisy(s).simOut.diag.a_xm`)
- `diag.sigma2_dxr_hat` per run
- kBT, sigma2_n, C_delta_x = 3.961, C_n_fb = 0.176 already computed

Tasks 5-7 of the Round-2 plan are the only uncommitted items; all data
structures exist on disk.

---

## 5. Ground-Truth Physics

### 5.1 Physical Constants

From `model/config/physical_constants.m` lines 13-18:

```
R       = 2.25       um
gamma_N = 0.0425     pN*s/um    (free-space Stokes drag)
Ts      = 1/1600     s          (sample period, 625 us)
k_B     = 1.3806503e-5  pN*um/K
T       = 310.15     K          (37 C)
```

**a_nom = Ts / gamma_N = (1/1600) / 0.0425 ~= 1.471e-2 um/pN** (free space).
(`run_pure_simulation.m:253`, `motion_control_law_eq17_6state.m:153`)

### 5.2 Wall Correction Functions and a_true(h_bar)

`calc_correction_functions(h_bar)` with u=1/h_bar (`model/wall_effect/calc_correction_functions.m`):

```
D_para = 1 - (9/16)u + (1/8)u^3 - (45/256)u^4 - (1/16)u^5     [lines 74-76]
D_perp = 1 - (9/8)u + (1/2)u^3 - (57/100)u^4 + (1/5)u^5
             + (7/200)u^11 - (1/25)u^12                         [lines 79-83]
c_para = 1/D_para,   c_perp = 1/D_perp
```

Both -> 1 as h_bar -> inf; both -> inf as h_bar -> 1 (lubrication divergence).
`c_perp > c_para` at any finite h_bar.

**True gain** (`run_pure_simulation.m:319-325`):
```matlab
[c_para_k, c_perp_k] = calc_correction_functions(h_bar_true_k);
a_true_k = [a_nom/c_para_k;  a_nom/c_para_k;  a_nom/c_perp_k];
% a_true_x = a_true_y = Ts/(gamma_N*c_para(h_bar))
% a_true_z             = Ts/(gamma_N*c_perp(h_bar))
```

Near wall: c_para, c_perp both grow -> a_true shrinks (mobility decreases).

**Derivatives** (lines 100-127) — computed analytically:
```
K_h_para       = -(1/c_para)*d(c_para)/d(h_bar)
K_h_prime_para = d(K_h_para)/d(h_bar)
```
(Similarly for perp.) Used in Q55 and R22 in the 6-state controller.

**Post-hoc a_true from p_true_out** (scalar only, pattern from
`verify_eq17_6state.m:185-198`):
```matlab
for k = 1:N
    hb = max(h_true(k), 1.001);
    [cpa, cpe] = calc_correction_functions(hb);
    a_para(k) = a_nom / cpa;
    a_perp(k) = a_nom / cpe;
end
a_true = [a_para, a_para, a_perp];  % [N x 3]
```

**Alignment note:** `a_true_out` taps at pre-integration position, `p_true_out`
taps post-integration. For comparing a_hat vs a_true: use
`a_true_out(2:end,:)` paired with `a_hat(2:end,:)`.

### 5.3 Thermal Force and Per-Step Displacement Variance

From `calc_thermal_force.m` lines 66-68:
```matlab
variance_coeff = 4 * k_B * T * gamma_N / Ts;
Variance = variance_coeff * abs(C);   % C = c_para or c_perp per direction
f_th = sqrt(Variance) .* randn(3,1);
```

Converting to position-increment variance:
```
Var(dp_para) = [Ts/(gamma_N*c_para)]^2 * (4*kBT*gamma_N*c_para/Ts)
             = 4*kBT*Ts / (gamma_N*c_para)
             = 4*kBT * a_true_para
```

**Core ground-truth identity (confirmed analytically and by code structure):**
```
Var(thermal position increment)_i = 4*kBT * a_true_i(h_bar)    [um^2]
```

Convention: **single-sided** (4*kBT*a, not 2*kBT*a). Confirmed by
MSD ratio ~= 2 result in the 2026-05-08 session, user accepted.

Free-space reference: `sigma2_deltaXT = 4*kBT*Ts/gamma_N` (named in
`calc_ctrl_params.m:40`).

### 5.4 p_true vs p_m

Step ordering in `run_pure_simulation.m` (lines 306-411):
```
(g) p_curr = step_dynamics(p_curr, F_total, P, Ts)    # ODE4
(h) p_m_raw = p_curr + n_meas                          # n_i ~ N(0, sigma2_n_i)
(j) p_true_out(k,:) = p_curr                           # noise-free (line 382)
    p_m_out(k,:)    = p_m_raw                          # noisy, pre-delay (line 383)
```

Controller receives `p_m_delayed = p_m_buffer(:,1)` = p_m from d=2 steps
earlier (line 316). Measurement noise is additive Gaussian per-axis,
applied after physics, before delay buffer.

---

## 6. Existing Verification and Figure Tooling

### 6.1 Scripts to Reuse for a_xm Chain Verification

| Need | Script | Path | Required change for 6-state |
|---|---|---|---|
| Full chain: p_m -> sigma2_dxr_hat -> a_xm | `compare_am.m` | `test_script/learn_variance/` | Switch `controller_type` to `'6state'`; confirm diag fields populated |
| E[sigma2_dxr] = C_dpmr*4kBT*a + C_n*sigma2_n | `verify_Cdpmr_intrinsic.m` | `test_script/learn_variance/` | Re-run `compare_am.m` first |
| a_pd sweep (3 x 5 seeds x 3 axes x 10 segments) | `verify_Cdpmr_comprehensive.m` | `test_script/learn_variance/` | Switch controller; confirm C_dpmr formula matches |
| C_n isolated (thermal off) | `verify_Cn_thermal_off.m` | `test_script/learn_variance/` | Same |
| IIR algorithm unit test (no simulation) | `verify_eq17_unit_iir_centering_response.m` | `test_script/unit_tests/` | None — pure unit test |
| C_dpmr Lyapunov vs closed-form | `verify_eq17_unit_predict_closed_loop_var.m` | `test_script/unit_tests/` | Confirm `predict_closed_loop_var_eq6` (model/diag/) applies to 6-state |
| R22 = Var(sigma2_dxr_hat) | `verify_R22_intrinsic.m` | `test_script/learn_variance/` | Uses compare_am.mat; no change after re-running |
| rho(tau) of dx_r and IF_eff | `verify_rho_static.m` | `test_script/learn_variance/` | Same |
| Variance ordering: delta_x_m > dx1_true > dx1_hat | `verify_three_dx1_variances.m` | `test_script/learn_variance/` | Confirm 6-state populates diag.delta_x_hat_1, diag.P_dx1 |
| Thermal theory vs ram (trajectory-aware, C_delta_x form) | `thermal_theory_check` | inside `analyze_gain_oracle_6state.m` lines 677-738 | Already arm-type-independent |
| End-to-end: tracking std, a_hat bias, a_hat rel-std | `verify_eq17_6state.m` | `test_script/integration/` | No change needed; already 6-state |
| Publication-style gain + tracking figures | `make_eq17_6state_figures.m` | `test_script/integration/` | None |

**`compare_am.m` details:** runs a single 20-second ramp-descent, reconstructs
the chain offline in a sandbox path (feeding `simOut.diag.delta_x_m` through
a manually coded LP1 + EWMA), inverts to `a_xm_sandbox`, compares step-by-step
against `simOut.diag.a_xm`. Saves `compare_am.mat` to
`test_results/learn_variance/`. All downstream scripts (`verify_Cdpmr_*`,
`verify_R22_*`) load this cache.

**`verify_Cdpmr_intrinsic.m` details:** 10 segments across ramp, two theory
formulations (point-evaluated and trajectory-averaged), reports
`ratio = emp/theory` per segment (target 1.0). Key diagnostic for systematic
offset in C_dpmr formula. Produces `verify_Cdpmr_intrinsic.png` (4-panel:
means, ratio over time, scatter, full IIR vs theory).

**`thermal_theory_check` uses simplified C_delta_x** (`C_dx = 2 + 1/(1-lc^2)`,
a_pd->0), not the full closed form C_dpmr = 3.161. Validates whether motion
variance tracks the simplified formula across trajectory windows.

### 6.2 Committed Verification Findings

From `reference/eq17_analysis/investigations/cdpmr_closed_form_verify_5seed.md`:
- h=50, 5 seeds, T_sim=20s, a_cov=0.005
- Lyapunov C_dpmr_eff = 3.232 (x/y), 3.232 (z)
- Closed-form = 3.161 (isotropic)
- Difference ~2.2% (consistent with the Lyapunov including higher-order effects)
- Closed-form shifts a_hat bias from -4.3% (Lyapunov) to +1.6%/+2.7% (+6 pp),
  consistent with a smaller Cd scaling a_xm upward

### 6.3 Figure-Style Conventions

#### EXP/Thesis style (canonical for 6-state results)

Defined in `make_eq17_6state_figures.m` lines 38-43:

| Property | Value |
|---|---|
| Grid | off |
| Box | on |
| Font size (axes) | 18 pt bold |
| Font size (legend) | 14 pt bold |
| Legend location | northoutside, horizontal, Box=off |
| Line width: True | LR=3.0 |
| Line width: Estimated | LO=2.0 |
| Line width: Measured (a_xm) | LM=0.5 |
| Color: True | green [0 0.6 0] |
| Color: Estimated | red [0.8 0 0] |
| Color: Error | blue [0 0.2 0.8] |
| Color: Measured a_xm (light) | [0.45 0.55 0.95 0.30] (alpha) |
| Color: IIR LP | black [0 0 0] |
| Color: Disturbance | orange [0.85 0.45 0] |
| Export DPI | 150 via exportgraphics |
| Figure size | 1100x720 (gain), 1100x920 (tracking) |
| Title content | stats summary inline: 'bias %+.2f%%  std %.2f%%' |

#### Learn-variance sandbox style

Defined in `test_script/learn_variance/plot_style.m` + `apply_default_style.m`:

| Property | Value |
|---|---|
| Grid | on (GridAlpha=0.25) |
| Box | on |
| Font sizes | tick=9, label=10, title=11, legend=9 |
| Line widths | signal=0.8, baseline=1.4, theory=1.0 |
| Save DPI | 150 to test_results/learn_variance/ |
| Colors | black primary, red=ground truth, gray-dashed=theory |

#### Round-2 new figure spec (not yet implemented, Tasks 5-7)

**fig_gain_compare** (4 layers, x and z rows):

| layer | signal | color |
|---|---|---|
| 1 | a_xm raw IIR (arm B, single traj-seed) | light blue [0.45 0.55 0.95 0.30], LW 0.5 |
| 2 | a_pd = a_nom/C_i(h_bar_d) (desired trajectory) | green [0 0.6 0] |
| 3 | a_true ensemble mean (arm A, 100 seeds) | red [0.8 0 0] |
| 4 | a_hat ensemble mean (arm B, 100 seeds) | blue [0 0.2 0.9] |

**fig_motion_var** (pointwise variance vs theory):
```
Theory: sigma2_th,i(t) = C_delta_x * 4kBT * a_pd,i(t) + C_n_fb * sigma2_n,i
Arm A: pointwise var over seeds of ram_v2 (red)
Arm B: pointwise var over seeds of ram_v2 (blue)
```

**Style upgrade (Tasks 5-7):** FS=18, axes LineWidth=2.0, xlabel='Time (sec)',
RAM overlay: B arm blue drawn first/thick LW2.5, A arm red on top/thin LW1.0.
`fig_det_err` is FROZEN — no restyling.

---

## 7. motion-test Branch State

### 7.1 Committed Infrastructure (Round-2 Tasks 1-4)

| commit | task | content |
|---|---|---|
| `cb8cea6` | Task 1 | `config.h_bar_safe` plumbing in `run_pure_simulation.m` + unit test |
| `3fffcfd` | Task 2 | Runner defaults: freqs=[1,5,10], seeds=1:100, T_sim=4.0, n_cyc_per_s=2, h_bar_safe=1, out_root=gain_oracle_ab_nogate; Layer-0 gate-free assertion |
| `3e84fb5` | Task 2 fix | Guard T_sim >= hold+descend+osc in build_config |
| `aba2f14` | Task 3 | Analyzer: opts.data_root, per-cycle discard, SEM everywhere (A2), x-direct ram, arm-B det degradation path |
| `2c04e09` | Task 3 fix | SEM NaN robustness for degenerate seed counts |
| `4e28660` | Task 3 fix | Graceful all-arm-B-seeds divergence (NaN fields, no crash) |
| `eba78df` | Task 4 | A1 Q55 dynamic check, A3 desc-window a_hat stats, det-run de-coupling |

Current HEAD: `2c04e09 fix(eq17): SEM NaN robustness for degenerate seed counts`

### 7.2 Not Yet Committed (Tasks 5-9)

- Task 5: Restyle `fig_traj_det`, `fig_traj_ram` (new colors, FS=18, AXLW=2.0, stats-in-title)
- Task 6: New `fig_gain_compare` (4-layer a_xm/a_pd/a_true/a_hat)
- Task 7: New `fig_motion_var` (pointwise ensemble variance vs theory)
- Task 8: Sample render gate (stop point — user approval before production batch)
- Task 9: Production 100-seed batch at {1, 5, 10} Hz

### 7.3 Data on Disk (Gitignored)

| Directory | Status | Content |
|---|---|---|
| `test_results/gain_oracle_ab/f{1,2,5}Hz/` | Round-1 production | 20-seed runs.mat + analysis.mat + 3 PNGs + summary.md |
| `test_results/gain_oracle_ab/round2_dev/f2Hz/` | Round-2 dev render | analysis.mat + summary.md (no PNGs) |
| `test_results/gain_oracle_ab_nogate/f5Hz-smoke/` | Smoke test | Confirms Layer 0 pipeline |
| `test_results/gain_oracle_ab_nogate_probe/{f1,5,10}Hz/` | Probe (1 seed/freq) | Arm-B det divergence confirmed at all freqs |
| `test_results/gain_oracle_ab_nogate_probe/round2_dev/{f1,f5}Hz/` | Dev analysis | analysis.mat + summary.md |

### 7.4 Round-2 Verified Layer-1 Results (round2_dev f=2 Hz, 20 seeds)

| Check | Arm A z | Arm B z | Status |
|---|---|---|---|
| det error amplitude in W_osc | 0.10 nm | 45.20 nm | A PASS, B shows gain estimation cost |
| osc normvar (sig2_th formula) | 1.014 ± 0.009 | 1.127 | A PASS (1.4%); B 12.7% excess |
| A1 Q55 dynamic (osc/gon/goff) | ratio [0.989, 1.043] | — | PASS (first dynamic validation) |

---

## 8. Implications for Verifying p_m -> a_xm and C_dpmr

### 8.1 Source of Ground Truth

At each time step k:
```
a_true_i[k] = Ts / (gamma_N * c_i(h_bar_true[k]))    (from simOut.a_true_out)
h_bar_true[k] = dot(p_curr, w_hat) / R               (pre-integration, noise-free)
```

The formula `E[sigma2_dxr_hat[k]] = C_dpmr * 4*kBT * a_true_i[k] + C_n * sigma2_n_i`
can be verified by:
1. Plotting `simOut.diag.sigma2_dxr_hat` vs the right-hand side built from `simOut.a_true_out`.
2. Checking the ratio is ~1.0 segment by segment (see `verify_Cdpmr_intrinsic.m`).
3. Checking the inversion `simOut.diag.a_xm` has mean ~= a_true and relative
   std ~= 14% (the chi-squared floor).

### 8.2 Natural Decomposition Stages

```
Stage 1: Statistical foundation — does sigma2_dxr track the C_dpmr formula?
         (verifies the HP filter + EWMA var estimator chain as a statistics engine)

         Script: verify_Cdpmr_intrinsic.m (after compare_am.m with 6-state)
         Signal: sigma2_dxr_hat vs C_dpmr*4kBT*a_true + C_n*sigma2_n
         Arm: Arm A (oracle) is cleanest (no self-consistent bias)
         Condition: static positioning h=50 (a_x constant, stationary)

Stage 2: Estimator fidelity — does a_xm from the inversion recover a_true?
         (verifies the linear inversion formula with correct C_dpmr/C_n values)

         Signal: simOut.diag.a_xm vs simOut.a_true_out
         Statistics: bias = mean(a_xm - a_true)/mean(a_true); rel_std of a_xm
         Arm: Arm A preferred (no a_hat feedback to sigma2_dxr)
         Pitfall: 20% C_dpmr error produces 25% bias — must use 3.161 not 3.961

Stage 3: EKF fusion — does a_hat track a_true after KF smoothing of a_xm?
         (verifies that the KF posterior is consistent with theory)

         Signal: simOut.diag.a_hat vs simOut.a_true_out
         Statistics: bias < 5%, rel_std < 5% (from verify_eq17_6state.m thresholds)
         Arm: Arm B (uses a_hat in law — self-consistent test)
         Pitfall: self-consistent bias if Phi vs F_e separation broken
```

### 8.3 Why Arm A is the Cleanest Bed for Stage 1-2

In arm A, the control law uses the true gain `a_ctrl = a_true`. The closed-loop
pole is exactly lc (no shift from gain estimation error). Therefore:
- The tracking error exactly follows the Eq.19 form with the correct MA(2) epsilon
- sigma2_dx = C_delta_x * 4kBT * a_true + [(1-lc)/(1+lc)] * sigma2_n_s exactly
- sigma2_dxr = C_dpmr * 4kBT * a_true + C_n * sigma2_n_s exactly (at steady state)

The EKF's a_hat estimation errors do NOT feed back into the truth dynamics
(block-triangular Lyapunov structure). So sigma2_dxr is a clean function of
a_true alone, decoupled from the EKF's performance.

In arm B, a_hat != a_true creates a self-consistent loop that shifts the
equilibrium. Stage 1 and 2 verification against a_true is confounded.
Arm A's round2_dev result (normvar 1.014 vs theory) shows this: the C_delta_x
formula holds to 1.4% for arm A, while arm B shows 12.7% excess from the loop.

### 8.4 Known Pitfalls to Design Around

**1. Chi-squared floor (~42%):**
The relative standard deviation of a_xm is `sqrt(K_var * IF_eff) ~= 14%` in
gain units. The "42%" reported elsewhere refers to the relative std of
sigma2_dxr_hat itself: `sqrt(2 * K_var * IF_eff) ~= 20%`. In the chi-squared
domain (before the linear inversion that divides by C_dpmr*4kBT), the ratio
`sigma2_dxr_hat / (C_dpmr*4kBT*a) = 1 + noise` with coefficient of variation
~= sqrt(2 * K_var * IF_eff) ~= 20%. This is an irreducible finite-sample floor
at any single time step; averaging over a window of N_eff = 1/a_cov = 20
samples is already done by the EWMA.

**2. Finite-sample EWMA bias (~2.5% for white noise; ~9% for colored dx_r):**
The IIR EWMA variance estimator has a known finite-sample bias. For white noise
input and EWMA alpha=0.05, the bias factor is `1 - a_prd/(2-a_prd) ~= 2.56%`
(from `verification-notes.md`). For the colored `dx_r` (AR(1) correlation),
the bias amplifies ~3.5x -> ~9%. This explains the residual negative bias
before the C_n prefill correction. The prefill mode pre-seeds sigma2_dxr_hat
to its steady-state value to skip the transient, but the EWMA lag bias during
dynamic a_x changes (oscillatory motion) remains.

**3. G2 gate-latch in arm B det runs:**
In a no-noise (det) run for arm B, sensor noise is absent -> dx_r is unusually
small -> sigma2_dxr_hat -> 0 below C_n*sigma2_n_s -> G2 latches continuously ->
y_2 permanently off -> a_hat drifts on y_1 only. This is the structural
reason that arm B det runs cannot be used as the baseline for ram_s[k]
decomposition.

**4. Near-wall C_dpmr breakdown:**
The C_dpmr formula assumes a stationary AR(1) process with pole lc. Near the
wall (h_bar < 2), a_x(t) changes rapidly within a cycle. The IIR lag means
sigma2_dxr_hat tracks a_true with delay, and the effective C_dpmr is no longer
the static formula. Round-2 dev normvar checks for the gate-on window (arm A z,
h_bar in [1.2, 1.5] region) will show systematic deviation. This is the
"near-wall C_dpmr breakdown" in the gain-oracle design document.

**5. det subtraction asymmetry:**
For arm B in oscillatory motion, the ram_s[k] decomposition requires ensemble
mean of p_true (not det run). The det run for arm B places the EKF in a
structurally different (G2-locked, y1-only) mode that diverges from the noisy
ensemble. Using arm B det run as the "deterministic response" introduces 26-28
nm rms error in the ram. Use only the ensemble mean method for arm B.

### 8.5 Signals to Plot at Each Stage

**Stage 1 (C_dpmr statistical foundation):**
- Plot 1: `sigma2_dxr_hat` (z-axis, one seed) vs theoretical
  `C_dpmr*4kBT*a_true_z + C_n*sigma2_n_z` — both as time series
- Plot 2: per-segment ratio emp/theory (target 1.0, tolerance ±0.05)
- Plot 3: scatter emp vs theory across segments (should lie on 45-degree line)
- Key diagnostic: systematic offset from 1.0 indicates wrong C_dpmr value;
  time-varying offset during oscillation indicates time-varying a_x(t) lag effects

**Stage 2 (EWMA inversion to a_xm):**
- Plot 1: a_xm (arm A, multiple seeds, light alpha) vs a_true (solid green)
  vs a_pd = a_nom/C_i(h_bar_d) (dashed) — time series z-axis
- Plot 2: relative bias = mean(a_xm - a_true)/a_true per window (target ~0%)
- Plot 3: relative std of a_xm per window (target ~14% thermal-dominated)
- Gate indicator: shade gate-on regions to show where G3 fires

**Stage 3 (EKF a_hat):**
- The `fig_gain_compare` figure from Round-2 Task 6 shows all four layers:
  a_xm (light blue, noisy), a_pd (green), a_true_ens (red), a_hat_ens (blue)
- This directly illustrates the chain: how noisy a_xm is relative to a_true,
  and how much the KF smooths it (a_hat_ens between a_xm and a_pd)

**Stage cross-check identity table:**

| Quantity | Measured from simOut | Expected value |
|---|---|---|
| mean(sigma2_dxr_hat) / window | simOut.diag.sigma2_dxr_hat | C_dpmr*4kBT*a_true + C_n*sigma2_n |
| mean(a_xm) | simOut.diag.a_xm | a_true |
| std(a_xm)/a_true | simOut.diag.a_xm | ~14% (thermal-dominated) |
| mean(a_hat bias) | simOut.diag.a_hat vs a_true_out | <5% (production target) |
| normvar of ram (arm A) | ram_v2 / sqrt(sigma2_th) | 1.0 ± 0.15 (Layer 1 soft gate) |

---

*Document consolidated from six exploration summaries. For prior raw findings
see: `reference/eq17_analysis/investigations/cdpmr_closed_form_verify_5seed.md`,
`reference/eq17_analysis/gain_oracle_ab_plan_round2.md`,
`reference/eq17_analysis/kf_canonical_spec.md`,
`reference/eq17_analysis/derivation/` (Cdpmr_Cn_derivation.tex, R22_derivation.tex,
phase2_*.md, phase5_*.md, phase7_*.md).*
