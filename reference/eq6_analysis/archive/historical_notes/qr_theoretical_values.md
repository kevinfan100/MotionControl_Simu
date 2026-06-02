# Q/R Derivation for Our Simulation — Entry-by-Entry

**Date**: 2026-04-15 (revised 2026-04-17 — framework rewrite)
**Branch**: `test/qr-paper-reference`
**Paper reference**: Meng et al., IEEE TIE 72(1), Jan 2025, pp. 929-938.

---

## 0. Framing

### 0.1 This document's purpose

The paper **does not specify** numerical Q/R values. It specifies the
7-state model (Eq.14), the process noise structure (Eq.17), and the
measurement variance formulas (Eq.11-13). From these plus **our**
simulation's conditions (thermal model, sensor spec, IIR architecture),
Q and R can be derived entry-by-entry.

The goal is a principled derivation that gives each Q(i,i) and R(i,i) a
**unique** value (or a fixed-point equation) with clear basis. Entries
that cannot be derived are explicitly marked as tuning.

### 0.2 Key framing principles

1. **Closed-loop is required context.** Sigma_e (tracking error variance),
   C_dpmr_eff (augmented Lyapunov), and R(2,2) itself are all closed-loop
   quantities. Open-loop evaluation has no steady state.

2. **No tradeoff in the derivation.** Each entry is either a physical
   fact, a structural 0, or a closed-loop fixed-point solution. Tradeoffs
   only appear when Q/R is used as a tuning knob to compensate for model
   errors — that is explicitly outside the derivation pipeline.

3. **Ratios appear as derivation outputs, not tuning choices.** For
   example, `R(2,2)/sigma2_dXT = chi-sq · autocorr · (C_dpmr_eff·beta)²`
   is a product of structural constants; every factor is determined by
   the system, none is chosen.

4. **Tuning bucket** (explicitly separated from derivation):
   Q(4,4), Q(5,5) — the x_D / delta_x_D states have no clear physical
   meaning in our simulation (no actuator drift, no DC offset, wall
   effect already captured by time-varying a). These are initialized to
   0 and will be tuned empirically if needed. Q(4,4) is optimized first;
   Q(5,5) afterwards.

---

## 1. Physical constants and natural unit

From `model/config/physical_constants.m` and `user_config.m`:

| Symbol | Value | Units | Source |
|---|---|---|---|
| k_B | 1.3806503e-5 | pN·um/K | physical_constants.m |
| T | 310.15 | K | physical_constants.m |
| gamma_N | 0.0425 | pN·sec/um | physical_constants.m |
| Ts | 6.25e-4 | sec | physical_constants.m |
| R (radius) | 2.25 | um | physical_constants.m |
| a_nom = Ts/gamma_N | 1.4706e-2 | um/pN | motion gain (free-space) |
| 4·k_B·T | 1.7129e-2 | pN·um | |
| sigma2_dXT = 4·k_B·T·a_nom | 2.5190e-4 | um² | per-step thermal variance |
| lambda_c | 0.7 | — | closed-loop pole |
| a_cov | 0.05 | — | EMA smoothing for V_est |
| sigma2_n (noise ON) | 1.0e-4 | um² | (0.01 um std)² |
| sigma2_n (noise OFF) | 0 | um² | baseline test scenario |

### 1.1 Natural unit for Q/R scaling

`sigma2_dXT` is the per-step position variance from thermal force alone,
evaluated in free space (a = a_nom). All Q/R entries in
`user_config.m` are reported as dimensionless multipliers on `sigma2_dXT`.

Derivation:
```
f_th has Var(f_th) = 4·k_B·T·gamma_N / Ts       (calc_thermal_force.m:67)
Delta_x per step   = a_nom · f_th               (discrete-time convention)
Var(Delta_x)       = a_nom² · 4·k_B·T·gamma_N/Ts
                   = 4·k_B·T·Ts/gamma_N
                   = 4·k_B·T·a_nom
                   = sigma2_dXT
```

---

## 2. Measurement architecture constants

From the IIR / HP filter structure in the 7-state EKF:

| Symbol | Formula | Value (lc=0.7, a_cov=0.05) | Source |
|---|---|---|---|
| C_dpmr (paper Eq.11) | 2 + 2/(1 - lc²) | 5.9216 | Paper, single-layer IIR |
| C_n (paper Eq.11) | (1 - lc)/(1 + lc) | 0.1765 | Paper |
| C_dpmr_eff (our architecture) | augmented 11-dim Lyapunov | 3.924 @ a=a_nom, rises to ~4.99 extreme near-wall | writeup Sec.5, `cdpmr_eff_lookup.mat` |
| beta (finite-sample IIR bias) | Task 1c derivation | 0.907 @ lc=0.7 | writeup Sec.6, `bias_factor_lookup.mat` |
| chi-sq EMA coefficient | 2·a_cov/(2-a_cov) | 0.05128 @ a_cov=0.05 | Exact steady-state EMA variance (white input) |
| rho_a (autocorr amplification) | Var(V_est non-white) / Var(V_est white) | ~4 empirically; rigorous = Var(V_est) via augmented Lyapunov | Task 1d Appendix A Layer 2 |

`C_dpmr_eff` differs from paper `C_dpmr` because our architecture has two
IIR stages (a_pd HP residual LP + a_cov variance EMA). Writeup Section 5
derives the augmented-state Lyapunov correctly. Session 2 built a 2-D
lookup (`lc × a/a_nom`, 48 grid points, DARE self-consistent per point).

---

## 3. State model and derivation summary

State vector (per axis):
```
x = [dx1, dx2, dx3, x_D, delta_x_D, a, delta_a]
     [=dx[k-2]]                     [motion gain]
```

Process noise structure (paper Eq.17):
```
q[k] = a[k]·f_T[k] · [0, 0, -1, 0, 0, 0, 0]^T
```
— only dx3 has a direct physical stochastic driver in the paper's model.
This is **insufficient for our simulation** (see Q(7,7) derivation below).

### 3.1 Decision summary table

| Entry | State | Type | Derived value | Needs simulation? |
|---|---|---|---|---|
| Q(1,1) | dx1 | structural 0 | 0 | No |
| Q(2,2) | dx2 | structural 0 | 0 | No |
| Q(3,3) | dx3 | physical | sigma2_dXT·(a[k]/a_nom)² (adaptive, Eq.21) | No |
| Q(4,4) | x_D | **tuning bucket** | 0 initially | — |
| Q(5,5) | delta_x_D | **tuning bucket** | 0 initially | — |
| Q(6,6) | a | clean decomposition | 0 | No |
| Q(7,7) | delta_a | closed-loop trajectory | Var(a[k+2] - 2·a[k+1] + a[k]) along reference | **Yes** (reference-trajectory-based, but deterministic) |
| R(1,1) | dx_m channel | sensor spec | sigma2_n | No |
| R(2,2) | a_m channel | **self-consistent fixed point** | iterate DARE ↔ Lyapunov ↔ V_meas ↔ R(2,2) | Yes (offline script) |

---

## 4. Q matrix — entry-by-entry derivation

### 4.1 Q(1,1), Q(2,2) — structural zeros

Paper Eq.14 lines 1-2:
```
dx1[k+1] = dx2[k]
dx2[k+1] = dx3[k]
```
Pure delay chain. No stochastic input at these equations.

**Value**: **0** (exact, structural).

### 4.2 Q(3,3) — thermal injection (adaptive)

Paper Eq.14 line 3 contains `-a[k]·f_T[k]`. The process noise at dx3 is:
```
q_3[k] = a[k]·f_T[k]
Var(q_3[k]) = a[k]² · Var(f_T)
             = a[k]² · 4·k_B·T·gamma_N / Ts
             = 4·k_B·T · a[k]² / a_nom
             = sigma2_dXT · (a[k] / a_nom)²
```

**Value**: **Q(3,3)[k] = sigma2_dXT · (a[k]/a_nom)²** (time-varying in a).

**Paper consistency**: this is exactly Eq.21's time-varying form. The
code's current constant `Q(3,3) = 1e4·sigma2_dXT` is tuning compensation,
not derivation.

**Implementation note**: in practice a[k] is replaced by â[k] (EKF
estimate) or â[k|k-1]. The adaptive form honors the paper's Eq.21
literally. Stateflow syntactic constraints on this form are a separate
implementation issue (see `project_qr_theoretization.md` for prior
attempts). The correct *derivation* is adaptive regardless.

### 4.3 Q(4,4) — tuning bucket

x_D is the "disturbance state" in the paper's model. In our simulation
there is **no clear physical disturbance source**:
- No actuator drift.
- No DC offset.
- No creep.
- Wall effect enters through `a(h_bar)`, already captured by the a state.

Options that could force a derivation (rejected):
- "x_D absorbs F_e linearization residuals": ambiguous without a specific
  model of the residual structure, and path A (add c_perp(h_bar) to F_e)
  was rejected because it requires unknown wall position.
- "x_D represents unmodeled coupling": not present in our per-axis
  Convention A architecture after P2 fix (z-axis beta coupling disabled).

**Status**: **tuning bucket**, initial value **0**.

**Tuning order**: Q(4,4) is optimized first (before Q(5,5)) in a later
phase. Criterion TBD.

### 4.4 Q(5,5) — tuning bucket

delta_x_D is the rate-of-change of x_D. Same ambiguity as Q(4,4);
inherited status.

**Status**: **tuning bucket**, initial value **0**.

**Tuning order**: Q(5,5) is optimized after Q(4,4) is determined.

### 4.5 Q(6,6) — clean decomposition gives zero

State equations for (a, delta_a):
```
a[k+1]        = a[k] + delta_a[k] + q_a[k]       q_a ~ N(0, Q(6,6))
delta_a[k+1]  = delta_a[k]        + q_δa[k]      q_δa ~ N(0, Q(7,7))
```

**Clean decomposition**: Define delta_a_true[k] := a_true[k+1] - a_true[k]
(forward first difference of the true motion gain along the trajectory).
By construction:
```
a_true[k+1] = a_true[k] + delta_a_true[k]
```
holds **exactly** — no residual on the a equation. Hence:

**Value**: **Q(6,6) = 0** (derivation output, not structural).

All curvature of a(t) is absorbed into Q(7,7) via the delta_a equation.

### 4.6 Q(7,7) — closed-loop trajectory residual (path B)

From the clean decomposition:
```
q_δa[k] = delta_a_true[k+1] - delta_a_true[k]
        = (a_true[k+2] - a_true[k+1]) - (a_true[k+1] - a_true[k])
        = a_true[k+2] - 2·a_true[k+1] + a_true[k]
        = Δ²a_true[k]            (second forward difference)
```

**Value**: **Q(7,7) = Var(Δ²a_true[k])** evaluated along the closed-loop
trajectory.

**Why closed-loop**: a_true = Ts/(gamma_N · c_perp(h_bar(t))) depends on
the trajectory h_bar(t). h_bar(t) is set by the reference p_d(t) plus
tracking error (which depends on Q/R). Strictly this is self-consistent,
but because tracking error (≤ tens of nm) is much smaller than h_bar
excursion (um scale), h_bar(t) ≈ h_bar_ref(t) to high precision. Q(7,7)
can therefore be computed from the **reference trajectory** alone
without running Simulink.

**Continuous-time approximation** (sanity check):
```
Δ²a ≈ a''(t)·Ts²  as Ts → 0
Q(7,7) ≈ Ts⁴ · Var(a''(t) along trajectory)
```

For p_d(t) = p_0 + A·cos(2πf·t)·w_hat (our default 'osc' trajectory):
- h_bar(t) = (h_bottom + A·(1 + cos(2πf·t))/2) / R
- a(t) = Ts/(gamma_N · c_perp(h_bar(t)))
- a''(t) involves (da/dh_bar)², (d²a/dh_bar²), and (dh_bar/dt)²

Numerical value of Q(7,7) is computed by a small script (Section 6).

**Paper inconsistency**: paper sets Q(7,7) = 0, which states "delta_a is
perfectly constant" — but along any trajectory with h_bar variation,
delta_a is not constant. This is a paper modeling choice we explicitly
reject in favor of the correct derivation.

---

## 5. R matrix — entry-by-entry derivation

### 5.1 R(1,1) — sensor noise (direct)

Measurement channel 1: `dx_m[k] = dx[k-2] + n[k-2]` where n is sensor
noise. State dx1[k] represents true dx[k-2]. Innovation = n[k-2].

**Value**: **R(1,1) = sigma2_n** (sensor noise variance from spec).

Numerical values:
- Noise ON: R(1,1) = sigma2_n = 1.0e-4 um² → scaling = 0.397
- Noise OFF: R(1,1) = 0 (degenerate). In code implementation a small
  positive epsilon (e.g., 1e-8·sigma2_dXT) is required for DARE
  conditioning. This is regularization, not derivation.

### 5.2 R(2,2) — self-consistent closed-loop fixed point

This is the only entry where closed-loop self-consistency is intrinsic
to the derivation.

**Physical structure**:
```
a_m[k] = (V_meas[k] - C_n·sigma2_n) / (C_dpmr · 4·k_B·T)      (paper Eq.13)
```
where `V_meas[k]` is an IIR estimate of the HP-residual variance.
Expected value in closed-loop steady state:
```
E[V_meas] = beta · C_dpmr_eff · Sigma_e(3,3)
```
where:
- `Sigma_e(3,3)` is the closed-loop steady-state tracking error variance
- `beta` accounts for finite-sample IIR bias (Task 1c)
- `C_dpmr_eff` is the augmented-Lyapunov effective ratio for our
  two-stage IIR architecture (writeup Sec.5).

**Variance of a_m**: since V_meas varies around its expectation with
chi-sq-like statistics amplified by del_pmr autocorrelation:
```
Var(V_meas) = chi_sq · rho_a · (E[V_meas])²
            = [2·a_cov/(2-a_cov)] · rho_a · (beta · C_dpmr_eff · Sigma_e(3,3))²
```
and therefore:
```
R(2,2) = Var(a_m)
       = Var(V_meas) / (C_dpmr · 4·k_B·T)²
       = chi_sq · rho_a · (beta · C_dpmr_eff · Sigma_e(3,3))² / (C_dpmr · 4·k_B·T)²
```

**The self-referential loop**:
```
R(2,2)  →  DARE(F_aug, Q, H, R)  →  L_ss
L_ss    →  F_e(closed-loop error dynamics)
F_e     →  Sigma_e = Lyapunov(F_e, Q_driver)
Sigma_e →  V_meas (via beta · C_dpmr_eff)
V_meas  →  R(2,2)'    (new estimate)
iterate until R(2,2)' ≈ R(2,2)
```

R(2,2) is the unique fixed point of this map. Section 7 describes the
iteration algorithm.

**Important**: every factor in the R(2,2) expression is a derivation
output (chi_sq from a_cov; rho_a from lc via HP autocorrelation;
C_dpmr_eff from lookup; beta from finite-sample analysis; Sigma_e from
Lyapunov). Nothing is a free parameter. This is the "ratio of structural
constants" form the user identified.

---

## 6. Q(7,7) computation procedure

**Input**: reference trajectory parameters from `user_config.m`
(h_bottom, amplitude, frequency, n_cycles, t_hold, wall geometry,
c_perp polynomial coefficients).

**Steps**:

1. Generate the discrete-time reference `p_d[k]` for k = 0, …, N-1 at
   sample rate Ts.
2. Compute `h[k] = projection of p_d[k] onto w_hat minus wall location`.
3. Compute `h_bar[k] = h[k] / R`.
4. Evaluate `a_true[k] = Ts / (gamma_N · c_perp(h_bar[k]))`.
5. Compute second difference: `Δ²a[k] = a_true[k+2] - 2·a_true[k+1] + a_true[k]` for k = 0, …, N-3.
6. **Q(7,7) = var(Δ²a)** (sample variance over the full trajectory).

**Output**: a single numerical value (scalar) for Q(7,7) in um²/pN².
Normalized scaling = Q(7,7) / sigma2_dXT for use in `Qz_diag_scaling`.

**Notes**:
- The sample variance is taken over the full trajectory including hold,
  descent, and oscillation phases. The oscillation phase dominates.
- Per-axis: strictly the same procedure applied to each axis. For the
  w_hat-aligned trajectory, only the normal-direction axis sees
  variation; tangential axes have a(t) ≈ const and Q(7,7) ≈ 0.
- Normalized by sigma2_dXT, Q(7,7) scaling depends on trajectory
  (amplitude, frequency, h_bottom). Different operating scenarios yield
  different Q(7,7). This is correct — Q(7,7) is trajectory-specific.

---

## 7. R(2,2) self-consistency iteration

**Input**: lc, a_cov, a_pd, a_prd, sigma2_n, sensor_delay, all Q entries
(including Q(7,7) from Section 6 and Q(3,3) function of a).

**Iteration**:

```
0. Initialize R22 (e.g., use code's current 1.0·sigma2_dXT).
1. For current (Q, R), solve DARE on (F_aug, H, Q, R) for L_ss.
2. Construct closed-loop F_e from L_ss (11-dim augmented state including
   IIR filter states).
3. Solve Lyapunov: Sigma_e = F_e · Sigma_e · F_e' + Q_driver.
   (Q_driver injects Q(3,3) at dx3.)
4. Extract Sigma_e(3,3) = tracking error variance.
5. E[V_meas]_new = beta · C_dpmr_eff · Sigma_e(3,3).
6. Var(V_meas)_new = chi_sq · rho_a · (E[V_meas]_new)².
7. R22_new = Var(V_meas)_new / (C_dpmr · 4·k_B·T)².
8. If |R22_new - R22| / R22 < tol (e.g., 1e-4), converged; else R22 ← R22_new, goto 1.
```

**Infrastructure**: Session 2 already built essentially this loop in
`test_script/build_cdpmr_eff_lookup.m` for each (lc, a/a_nom) grid
point. That script resolves DARE self-consistently per grid point. It
can be repurposed to produce a single (Q,R) pair at a chosen operating
point.

**Output**: a single scalar R(2,2) and the corresponding Sigma_e, for
the chosen operating point (lc, a_ref). Normalized scaling = R(2,2) /
sigma2_dXT.

---

## 8. Side effect: Session 2 fragility resolved by correct Q(7,7)

Session 2 documented that Q(6,6) = 0 caused P(6,6) collapse in ~100
steps, leading to a_hat being permanently frozen. The mechanism:

```
L(6,2) = P(6,6) / (P(6,6) + R(2,2))
```

With Q(6,6) = 0 AND Q(7,7) = 0, P(6,6) has no injection and collapses
exponentially to 0. L(6,2) → 0. a_hat frozen.

**But this was the paper's Q(6,6) = Q(7,7) = 0 configuration.** In the
correct derivation, Q(7,7) > 0 (from path B). The covariance propagation
becomes:
```
P(7,7)[k+1] = P(7,7)[k] + Q(7,7)              linear growth
P(6,7)[k+1] = P(6,7)[k] + P(7,7)[k]           quadratic growth
P(6,6)[k+1] = P(6,6)[k] + 2·P(6,7) + P(7,7)   cubic growth (bounded by Kalman pull-back)
```

P(6,6) no longer collapses — it is sustained by cross-coupling from
P(7,7) through the `[1 1; 0 1]` block of F. L(6,2) stays bounded away
from 0. a_hat tracks the measurement channel properly.

**Implication**: the correct derivation of Q(7,7) (which paper sets to
0 but we derive as Var(Δ²a) > 0) **naturally resolves the coupled
fragility that Session 2 identified**. The fragility was not a Q/R
design problem — it was a consequence of paper's over-simplified
`delta_a = const` assumption, which fails the moment a(t) has curvature.

This is strong evidence the current derivation framework is correct.

---

## 9. Summary (all values as scalings on sigma2_dXT)

### Two interpretations of Q(6,6) / Q(7,7)

The state decomposition `a[k+1] = a[k] + δa[k] + q_a[k]` and
`δa[k+1] = δa[k] + q_δa[k]` is not unique in how noise is split
between `q_a` and `q_δa`. Two principled choices give different values:

**Interpretation 1 — Forward-diff clean decomposition (τ → 0 limit)**:
δa_true[k] := a_true[k+1] - a_true[k] → Q(6,6) = 0, Q(7,7) = Var(Δ²a).
Mathematically unique given the decomposition. At Ts = 6.25e-4, this is
operationally insufficient — Simulink shows `a_hat` tracking degrades.

**Interpretation 2 — Backward-diff β (strict but with rank-1 Q)**:
δa_true[k] := a_true[k] - a_true[k-1] → Q(6,6) = Q(7,7) = Var(Δ²a).
Same numerical magnitude but applied to both; diagonal approximation loses
rank-1 correlation.

**Interpretation 3 — B'-2 white-noise equivalent (time-scale self-consistent)**:
For deterministic signal with correlation time T_c = 1/(2πf₀):
`Q(6,6) = Var(a) · Ts / T_c`, `Q(7,7) = Var(δa) · Ts / T_c` along
oscillation phase. Captures the full-signal effective noise rather than
per-step residual. Numerically ~10⁷× larger Q(6,6) than interpretation 1/2.

### Numerical values at lc=0.7, a_pd=0.05, a_cov=0.05, default trajectory, noise ON

#### Q matrix

| Entry | Value | Interpretation |
|---|---|---|
| Q(1,1), Q(2,2), Q(5,5) | 0 | structural zeros |
| Q(3,3) | 1.0 (constant free-space approx) | adaptive (a/a_nom)² blocked by Stateflow |
| Q(4,4) | 0 (initial) | tuning bucket |
| Q(6,6) | 1.3444e-11 (β) / **1.3016e-04 (B'-2)** | B'-2 matches empirical Q(6,6) = 1e-4 |
| Q(7,7) | 1.3444e-11 (β) / **2.4368e-09 (B'-2)** | B'-2 nearly zero (matches empirical = 0) |

#### R matrix

| Entry | Value | Notes |
|---|---|---|
| R(1,1) | **0.3970** (= σ²_n/σ²_dXT) | sensor spec, noise ON |
| R(2,2) | **1.7016** (self-consistent theoretical) | derived |
| R(2,2) | 1.0 (empirical, used in code) | Simulink prefers lower R |

#### Structural constants (lc=0.7, ar=1.0)

| Symbol | Value |
|---|---|
| C_dpmr_paper | 5.9216 |
| C_dpmr_eff (augmented Lyapunov with Q77 > 0) | 4.0275 |
| β (IIR finite-sample bias) | 0.9024 |
| chi_sq = 2·a_cov/(2-a_cov) | 0.0513 |
| ρ_a (rigorous via A_aug autocorrelation) | 3.70 |
| Sigma_e(3,3) closed-loop | 1.327e-3 um² (std 36 nm) |
| V_meas = β·C_dpmr_eff·Sigma_e | 4.82e-3 um² |

### Simulink verification summary (2026-04-17 autonomous run)

Best configuration empirically: **B' Q + empirical R = 1.0** (not fully
theoretical). Achieves 3D RMSE 53 nm (vs empirical 57 nm) and `a_hat_z`
rel err 14.6% (vs empirical 21.7%). See `qr_verification_findings.md`
for full 5-variant comparison table.

---

## 10. Open implementation items

Items needed to produce concrete numerical values:

1. **Q(7,7) script** (Section 6): one-shot MATLAB function that reads
   trajectory parameters, returns scaling on sigma2_dXT.
2. **R(2,2) script** (Section 7): extract/repurpose
   `build_cdpmr_eff_lookup.m` into a single-point self-consistency
   solver.
3. **Adaptive Q(3,3) in controller**: Stateflow has syntactic
   constraints (see `project_qr_theoretization.md`). Implementation
   path separate from derivation.

Items in tuning phase (post-derivation):

4. **Q(4,4) optimization**: criterion to be defined. Options: minimize
   mean bias in a_hat, minimize 3D RMSE, minimize innovation
   whiteness-test residual.
5. **Q(5,5) optimization**: after Q(4,4) converged.

Items not in this derivation (reserved for later):

6. **Trajectory representativeness**: Q(7,7) and R(2,2) depend on
   trajectory. Baseline = default 'osc' trajectory from `user_config.m`.
   Other scenarios yield different (Q(7,7), R(2,2)).

---

## 11. Files and cross-references

**This document**: `reference/for_test/qr_theoretical_values.md`

**Dependencies** (prior derivations reused here):
- Writeup Section 5: C_dpmr_eff augmented Lyapunov derivation
- Writeup Section 6: F_e closed-loop error dynamics (per-axis, post-P2)
- Task 1b: finite-sample IIR bias beta
- Task 1c: beta integration into EKF
- Task 1d Appendix A: chi-sq chain decomposition
- Session 2 of this branch: coupled stability analysis,
  `build_cdpmr_eff_lookup.m` self-consistency infrastructure

**Referenced constants**:
- `model/config/physical_constants.m` (k_B, T, gamma_N, Ts, R)
- `model/config/user_config.m` (lambda_c, a_cov, meas_noise_std)
- `test_results/verify/cdpmr_eff_lookup.mat` (C_dpmr_eff lookup)
- `test_results/verify/bias_factor_lookup.mat` (beta lookup)

**Related reports**:
- `task_qr_reference_report.md` — Session 1 Simulink experiments with
  paper-direct Q/R (historical, pre-derivation)
- `project_qr_theoretization.md` (memory) — Session 2 findings that
  motivated this re-derivation

**Deprecated content from earlier versions of this file**: paper-vs-code
comparison tables have been removed since the framing shifted away from
"paper values as ground truth" to "derivation in our simulation's own
right."
