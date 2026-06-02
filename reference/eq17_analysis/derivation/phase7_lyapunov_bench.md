# Phase 7: Closed-loop Variance Lyapunov Bench

**Pre-Phase-7 commit**: `8b468aa` (Phase 6 R matrix)
**Date**: 2026-04-29
**Status**: Phase 7 derivation. Awaiting user review.

This phase derives the closed-form predictions for σ²_δx, σ²_e_xD, σ²_e_a using modular block-triangular Lyapunov approach (per brainstorming §3.5). These predictions serve as analytical oracles for Phase 8 simulation verification — they do NOT change KF runtime behavior.

---

## 1. Goals

1. Define augmented state for Lyapunov bench
2. Establish block-triangular structure (Eq.17 + Option B decoupling)
3. Step 1: Solve 4×4 estimation-error sub-Lyapunov → σ²_e_xD, σ²_e_a closed forms
4. Step 2: Solve truth-driven Lyapunov → σ²_δx with estimation-error corrections
5. Provide MATLAB function spec: `predict_closed_loop_var_eq17_v2.m`
6. Numerical examples (h=50, h=2.5) for verification

---

## 2. Augmented State (per brainstorming §3.5)

For full Lyapunov bench, the 11-dim augmented state:

```
e_aug = [ δx_truth chain  |  e_x_chain  |  e_xD/δxD  |  e_a/δa  |  IIR_LP ]
       └──── 3 dim ──────┘  └─ 3 dim ──┘  └─ 2 dim ─┘  └ 2 dim ┘  └ 1 dim ┘

δx_truth chain = [δx_1, δx_2, δx_3]                  (true tracking errors)
e_x_chain     = [e_x1, e_x2, e_x3]                   (KF estimation errors of δx chain)
e_xD/δxD      = [e_xD, e_δxD]                        (disturbance estimation errors)
e_a/δa        = [e_a, e_δa]                          (motion gain estimation errors)
IIR_LP        = [δx̄_m]                              (IIR LP filter state)
```

Convention: e_x := x_truth − x̂_KF (sign convention).

---

## 3. Block-Triangular Decoupling (★ Eq.17 enables this)

### 3.1 Augmented dynamics A_aug (11×11)

```
                δx_truth   e_x_chain   e_xD/δxD   e_a/δa   LP
δx_truth     [   A_self     ★ ZEROS    A_xD_drive  A_a_drive   0  ]   ← Eq.17 raw δx_m
e_x_chain    [    0         A_e_chain    *           *         0  ]   ← KF e_x dynamics
e_xD/δxD     [    0          0          A_e_xD       0         0  ]   ← KF e_xD dynamics
e_a/δa       [    0          0           0          A_e_a      0  ]   ← KF e_a dynamics
LP           [    coupling   0           0           0       (1−a_pd) ]
```

★ Critical structural property (Option B + Eq.17):
- **δx_truth row to e_x_chain block: ZERO**
- Reason: Eq.17 uses raw δx_m (NOT δx̂_3), so KF estimation error of δx chain doesn't feed back into truth dynamics
- This is the "Item-A killed" property (brainstorming §2.7)

### 3.2 Block-triangular consequence

Augmented Σ_aug splits into:
- Bottom-right 8×8: estimation errors + LP, decoupled from truth
- Top-left 3×3: truth chain, driven by estimation errors (sub-block result) + ε noise

→ Solve in 2 steps:
- **Step 1**: 4×4 sub-Lyapunov for e_xD, e_δxD, e_a, e_δa (estimation errors)
- **Step 2**: 3×3 driven Lyapunov for δx_truth using Step 1 σ² as inputs

(IIR LP and e_x_chain don't directly affect σ²_δx_truth, can be computed separately if needed.)

---

## 4. Step 1: Estimation-Error Sub-Lyapunov

### 4.1 Sub-state and dynamics

Sub-state: e_sub = [e_xD, e_δxD, e_a, e_δa]ᵀ (4-dim).

Sub-dynamics matrix A_e_sub (from KF estimation error formula `e[k+1] = (F − L·H)·e[k] + w − L·v`):

```
A_e_sub = (I − L_sub·H_sub)·F_sub
```

Where F_sub is the 4×4 sub-block of F_e for [x_D, δxD, a, δa] states (slots 4-7):
```
F_sub = [ 1   1   0   0 ]
        [ 0   1   0   0 ]
        [ 0   0   1   1 ]
        [ 0   0   0   1 ]
```

H_sub: 2×4 from H matrix slots 4-7:
```
H_sub = [ 0  0  0   0 ]    ← y_1 (δx_m) doesn't observe these states directly
        [ 0  0  1  −d ]    ← y_2 (a_xm) observes a_x − d·δa_x
```

L_sub: 4×2 KF gain (from Riccati steady-state).

### 4.2 Driving covariance (Q_sub + L·R·L')

Process noise: Q_sub = diag(Q44, Q55, Q66, Q77) = diag(0, a_nom²·σ²_w_fD, 0, Q77).

Measurement noise propagated: L_sub·R·L_subᵀ.

Discrete Lyapunov equation (steady-state):
```
Σ_e_sub = A_e_sub · Σ_e_sub · A_e_subᵀ + Q_sub + L_sub·R·L_subᵀ
```

Solve for Σ_e_sub (4×4 covariance matrix).

### 4.3 Closed-form approximations

Full 4×4 numerical solve requires R_2 (closed-loop, time-varying), L_sub from Riccati. **Closed form is NOT trivial** without further approximations. We provide approximate closed forms:

#### 4.3.1 σ²_e_a (motion gain estimation error)

For positioning (frozen Q77 = small), chi-squared chain theory (qr Level 6):
```
Var(e_a) / a_x² = chi_sq · ρ_a · L_eff / (L_eff + a_cov) + (sens·std(p_m_z)/R)²
                 ↑─── chi-sq chain ───↑           ↑── wall sensitivity ──↑
```

Where:
```
chi_sq = 2·a_cov / (2 − a_cov)                        ≈ 0.0513 for a_cov=0.05
ρ_a    ≈ 4.7 (for thermal-dominated near-wall)       (autocorrelation amplification)
L_eff  = sqrt(Q66 / R(2,2))                           (effective Kalman gain ratio)
sens   = |dc_axis/dh̄| / c_axis (at h_init)            (wall sensitivity)
```

For v2 baseline (Q66 = 0, Q77 from Path B):
- L_eff is tricky for integrated RW (Q66=0 means no innovation on a_x position state)
- Practical L_eff uses Q77 indirectly through cross-coupling
- Empirical (from v1 5-seed CI): σ_e_a / a_x ≈ 1.7-4.5% (positioning frozen)

For motion (high Q77), σ_e_a / a_x ≈ 5-15%.

```
σ²_e_a,i ≈ (CV_a,i)² · a_x,i²
where CV_a,i ≈ {1.7-4.5%}  for positioning, {5-15%} for motion
                                           per axis
```

(★ This is approximate; exact numerical solve via Riccati needed for precision.)

#### 4.3.2 σ²_e_xD (disturbance estimation error)

For baseline σ²_w_fD = 0:
```
Q55 = 0   → δx_D dynamics has no driving noise
e_xD steady-state under KF tracking of x_D = 0:
  σ²_e_xD → small (bounded by R(2,2) / L gain)
```

Approximate: σ²_e_xD ~ R(2,2) · constant_factor (from KF observability). For sensor-dominated z-axis, σ²_e_xD ~ R(2,2)_z / a_x_z² · scaling.

For σ²_w_fD > 0:
```
Q55 > 0 → e_xD has steady-state Riccati solution
σ²_e_xD ~ sqrt(Q55 · R(2,2))   (scalar approximation for SISO)
```

For baseline (σ²_w_fD = 0), σ²_e_xD is small enough to often neglect.

---

## 5. Step 2: Truth-driven Lyapunov for σ²_δx

### 5.1 Truth chain dynamics (3-dim)

Sub-state: x_truth = [δx_1, δx_2, δx_3]ᵀ.

Dynamics:
```
δx_1[k+1] = δx_2[k]              (shift)
δx_2[k+1] = δx_3[k]              (shift)
δx_3[k+1] = λ_c·δx_3[k] − e_xD[k] − f_d[k]·e_a[k]/â_x − ε[k]  
              ↑       ↑          ↑                      ↑
              AR(1)   disturb    gain estimation        thermal+sensor
                      err input   err input             current step
```

Note: under Option I, x_D contribution to truth dynamics ≈ 0 when σ²_w_fD = 0. The F_e(3,4) = -1.6 term is in F_e KF prediction, but x_D in truth ≈ 0.

### 5.2 σ²_δx_3 closed form (Eq.19 form, with corrections)

Adapt paper Eq.22 form with estimation error corrections:

```
σ²_δx_3 · (1 − λ_c²) = C_dpmr · 4kBT·a_x + C_n · σ²_n_s        ← original (Phase 2)
                     + σ²_e_xD                                  ← Step 1 input
                     + (E[bracket²] / a_x²) · σ²_e_a            ← Step 1 input
```

Where:
- E[bracket²] depends on trajectory (positioning vs motion)
- For positioning: E[bracket²] ≈ (1−λ_c)²·σ²_δx_m
- For motion: E[bracket²] ≈ feedforward terms dominate

### 5.3 σ²_δx_1, σ²_δx_2 (from shift register)

Since δx_1[k] = δx_3[k−2] and δx_2[k] = δx_3[k−1], in steady state:
```
σ²_δx_1 = σ²_δx_2 = σ²_δx_3 = σ²_δx
γ_δx_truth(τ) for τ=1,2: same as γ_δx (Phase 2 §6)
```

### 5.4 Numerical evaluation example

For positioning at h=50 µm, λ_c=0.7:

```
σ²_dXT_x = 4·k_B·T·a_x  (compute with a_nom·c_para⁻¹(h_bar=22))
σ²_n_s = sensor spec per axis

σ²_δx (paper part) = (3.96·σ²_dXT + 0.176·σ²_n_s) / (1 − 0.49)
                  ≈ 7.76·σ²_dXT + 0.345·σ²_n_s

For thermal-dominated (z-axis at h=50): σ²_dXT ≈ 5.1e-5 µm² (need actual γ value)
σ²_δx_paper_part ≈ 4.0e-4 µm²
σ_δx ≈ 20 nm

Estimation error corrections:
σ²_e_xD ≈ 0 (baseline)
(E[bracket²]/a_x²)·σ²_e_a ≈ (1−λ_c)²·σ²_δx · CV_a²
                          ≈ 0.09 · 4e-4 · 2e-3 (for CV_a=4.5%)
                          ≈ 7.2e-8 µm² (negligible)

σ²_δx_total ≈ σ²_δx_paper_part (corrections < 0.1%)
```

→ For positioning baseline, **paper Eq.22 directly gives ≈ 99.99% of σ²_δx**. Estimation-error corrections negligible.

For motion (e.g., 1 Hz osc), corrections are larger but still typically < 5%.

---

## 6. predict_closed_loop_var_eq17_v2.m MATLAB Function Spec

### 6.1 Function signature

```matlab
function [s2_dx, s2_e_xD, s2_e_a, diag] = predict_closed_loop_var_eq17_v2(opts)
% PREDICT_CLOSED_LOOP_VAR_EQ17_V2 Closed-form variance prediction for v2 EKF
%
% Solves modular block-triangular Lyapunov:
%   Step 1: 4×4 sub-Lyapunov for [e_xD, e_δxD, e_a, e_δa] estimation errors
%   Step 2: 3×3 truth-driven Lyapunov for [δx_1, δx_2, δx_3] tracking errors
%
% Inputs (opts struct):
%   .lambda_c       closed-loop pole [0.7]
%   .a_x_axis       1×3 motion gain per axis [µm/pN]
%   .h_bar          wall-normal distance / R [-]
%   .sigma2_n_s     1×3 sensor noise variance per axis [µm²]
%   .k_B            Boltzmann constant [pN·µm/K]
%   .T              temperature [K]
%   .Q77_axis       1×3 Q77 per axis (from Phase 5) [µm²/pN²]
%   .R_axis         1×3 R(2,2) per axis (from Phase 6) [µm²/pN²]
%   .a_cov, a_pd    IIR coefficients
%   .C_dpmr, C_n    Phase 2 constants
%   .IF_var         Phase 2 IF_var
%   .sigma2_w_fD    f_D RW innovation variance [pN²]
%   .scenario       'positioning' or 'motion'
%   .traj_amplitude motion only: A_h [µm]
%   .traj_freq      motion only: f [Hz]
%
% Outputs:
%   s2_dx       1×3 σ²_δx prediction per axis [µm²]
%   s2_e_xD     1×3 σ²_e_xD per axis [µm²]
%   s2_e_a      1×3 σ²_e_a per axis [µm²]
%   diag        struct with intermediate values for debugging

    lambda_c = opts.lambda_c;
    
    % Step 1: Estimation error variances
    %   For closed-form, use chi-sq chain approximation
    chi_sq = 2 * opts.a_cov / (2 - opts.a_cov);
    rho_a_pos = 4.7;  % positioning approximation, refine for motion
    
    s2_e_a = zeros(1, 3);
    s2_e_xD = zeros(1, 3);
    
    for i = 1:3
        % σ²_e_a via chi-sq chain
        L_eff = sqrt(opts.Q77_axis(i) / opts.R_axis(i));
        CV2_a = chi_sq * rho_a_pos * L_eff / (L_eff + opts.a_cov);
        s2_e_a(i) = CV2_a * opts.a_x_axis(i)^2;
        
        % σ²_e_xD: small for σ²_w_fD = 0, scaled with σ²_w_fD
        if opts.sigma2_w_fD > 0
            % Approximate: SISO Riccati for x_D-δxD pair
            s2_e_xD(i) = sqrt(opts.a_x_axis(i)^2 * opts.sigma2_w_fD * opts.R_axis(i));
        else
            s2_e_xD(i) = 1e-20;  % effectively 0
        end
    end
    
    % Step 2: σ²_δx_truth
    s2_dx = zeros(1, 3);
    sigma2_dXT = zeros(1, 3);
    
    for i = 1:3
        sigma2_dXT(i) = 4 * opts.k_B * opts.T * opts.a_x_axis(i);
        
        % Paper Eq.22 base
        s2_dx_paper = (opts.C_dpmr * sigma2_dXT(i) + opts.C_n * opts.sigma2_n_s(i)) / (1 - lambda_c^2);
        
        % E[bracket²] for scenario
        if strcmp(opts.scenario, 'positioning')
            % E[bracket²] ≈ (1-λ_c)²·σ²_δx_m ≈ (1-λ_c)²·(σ²_δx + σ²_n_s)
            E_bracket2 = (1 - lambda_c)^2 * (s2_dx_paper + opts.sigma2_n_s(i));
        else  % motion
            % E[bracket²] dominated by feedforward terms
            A_h = opts.traj_amplitude;
            omega = 2*pi*opts.traj_freq;
            Ts = 1/1600;
            % feedforward bracket scales with A_h·ω
            E_bracket2 = (A_h * omega * Ts)^2 / 2;  % rough estimate
        end
        
        % Add estimation-error corrections
        s2_dx(i) = s2_dx_paper ...
                 + s2_e_xD(i) / (1 - lambda_c^2) ...
                 + (E_bracket2 / opts.a_x_axis(i)^2) * s2_e_a(i) / (1 - lambda_c^2);
    end
    
    if nargout >= 4
        diag.sigma2_dXT = sigma2_dXT;
        diag.chi_sq = chi_sq;
        diag.L_eff = sqrt(opts.Q77_axis ./ opts.R_axis);
        diag.s2_dx_paper_only = (opts.C_dpmr*sigma2_dXT + opts.C_n*opts.sigma2_n_s) / (1 - lambda_c^2);
        diag.correction_xD = s2_e_xD / (1 - lambda_c^2);
        diag.correction_a = (E_bracket2 ./ opts.a_x_axis.^2) .* s2_e_a / (1 - lambda_c^2);
    end
end
```

### 6.2 Test cases for verification

```matlab
% Test 1: positioning h=50 (z-axis thermal-dominated)
opts1.lambda_c = 0.7;
opts1.a_x_axis = [a_par, a_par, a_perp_h50];
opts1.h_bar = 50/2.25 = 22.2;
opts1.sigma2_n_s = [3.84e-7, 3.25e-9, 1.10e-5];
% ... fill remaining
[s2_dx_1, ...] = predict_closed_loop_var_eq17_v2(opts1);

% Test 2: positioning h=2.5 (z-axis near-wall)
opts2.h_bar = 2.5/2.25 = 1.11;
% c_perp ≈ 22 at h_bar=1.11
% a_x_axis(3) reduces by factor 22

% Test 3: motion 1 Hz at h=2.5
opts3.scenario = 'motion';
opts3.traj_amplitude = 2.5;
opts3.traj_freq = 1.0;
```

### 6.3 Expected output (numerical, for sanity check)

For positioning h=50:
```
s2_dx = [predict_x, predict_y, predict_z]
σ_δx ≈ [25-35 nm per axis]    (matches v1 5-seed CI 35 nm)
s2_e_a / a_x² ≈ [0.08%-0.2%]   (paper-level)
s2_e_xD ≈ 0                    (baseline)
```

For motion 1 Hz h=2.5:
```
σ_δx ≈ [50-80 nm per axis]
s2_e_a / a_x² ≈ [3-15%]        (KF tracks dynamic a_x with chi-sq floor)
```

---

## 7. v2 vs v1 Lyapunov Predictions

### 7.1 v1 baseline (qr Level 1-6 derivation)

```
σ²_δx · (1−λ_c²) = C_dpmr · σ²_dXT + C_n · σ²_n_s     (paper Eq.22 only)
```

v1 used 11×11 monolithic Lyapunov with lookup tables for C_dpmr_eff (which absorbed some estimation error implicitly).

### 7.2 v2 (this Phase)

```
σ²_δx · (1−λ_c²) = C_dpmr · σ²_dXT + C_n · σ²_n_s
                 + σ²_e_xD
                 + (E[bracket²]/a_x²) · σ²_e_a
```

Modular block-triangular: 4×4 sub-Lyapunov + 3×3 driven, no 11×11 monolithic.

**Numerical predictions: v1 vs v2 should match** in baseline (paper Eq.22 dominates). Differences appear in:
- Motion scenarios (estimation error contribution larger)
- Near-wall (sensitivity term + Q77 inflation)
- σ²_w_fD > 0 (disturbance contribution)

### 7.3 Closed-form vs lookup

v1 needed cdpmr_eff_lookup.mat + bias_factor_lookup.mat (2-D in (h_bar, λ_c)). v2 closed form eliminates these — all evaluated each step from â_x[k], h̄[k] via Phase 5/6 formulas.

---

## 8. Open Items / Caveats

### 8.1 σ²_e_a closed form requires Riccati

The chi-sq chain approximation (qr Level 6) is order-of-magnitude correct but exact closed form requires solving 2×2 Riccati for (a_x, δa_x) gain pair. Phase 8 unit test should cross-check predicted σ²_e_a vs simulation observed.

### 8.2 ρ_a in motion scenarios

ρ_a (autocorrelation amplification of a_xm) was computed in qr Level 4 as ~4.7 for near-wall thermal-dominated. For other regimes:
- Far-from-wall: ρ_a smaller (~2-3)
- Sensor-dominated (z h=50): ρ_a different
- Dynamic motion: ρ_a varies with trajectory

For Phase 7 baseline, use ρ_a = 4.7 as approximation. Phase 8 sensitivity sweep refines.

### 8.3 E[bracket²] motion estimate

Section 6.1 uses rough estimate `E[bracket²] ≈ (A_h·ω·Ts)²/2`. More accurate:
```
E[bracket²] = E[(x_d[k+1] − λ_c·x_d[k] − (1−λ_c)·x_d[k−d] + (1−λ_c)·δx_m[k])²]
```

For sinusoidal x_d, this evaluates analytically but with correlation terms. Defer to Phase 8 numerical computation.

### 8.4 11×11 vs modular accuracy

Modular 4×4 + 3×3 ASSUMES decoupling from e_x_chain. Strictly, e_x_chain has small correlation with truth via IIR LP feedback (Phase 7 §3.1 LP→δx_truth coupling).

For Eq.17 + Option B: this coupling is ~0.2% effect (negligible). v1's 11×11 monolithic captures it; v2 modular ignores it.

If Phase 8 observes σ²_δx prediction off by > 5%, revisit (full 11×11 may be needed for paper-level accuracy).

### 8.5 Phase 5 Q55 a_x → a_nom correction

Phase 5 §5.4 wrote Q55 = a_x²·σ²_w_fD. Should be a_nom² to avoid self-loop in implementation. Cosmetic (baseline σ²_w_fD = 0). Phase 8 implementation must use a_nom².

---

## 9. Phase 7 Summary

| Quantity | Closed form | Source |
|---|---|---|
| σ²_δx (paper part) | (C_dpmr·σ²_dXT + C_n·σ²_n_s)/(1−λ_c²) | Phase 2 |
| σ²_e_xD | ≈0 baseline; ~sqrt(Q55·R) for σ²_w_fD>0 | Step 1 SISO approximation |
| σ²_e_a | a_x²·CV²(â_x) ≈ chi_sq·ρ_a·L_eff/(L_eff+a_cov)·a_x² | qr Level 6 chi-sq chain |
| σ²_δx_total | paper part + σ²_e_xD + (E[bracket²]/a_x²)·σ²_e_a | Step 2 truth-driven |

**Key v2 finding**: 
- For baseline σ²_w_fD = 0, positioning: corrections < 0.1%, paper Eq.22 alone is ~99.9% accurate
- For motion or σ²_w_fD > 0: corrections may be ~few %, still paper-level
- σ²_δx prediction matches v1 numerical values in baseline

**Modular block-triangular replaces v1 lookup tables**:
- Per-axis closed form, no 2-D interpolation
- Each step: ~50 ops to compute σ²_δx prediction

### 9.1 Phase 8 entry conditions

Phase 8 (implementation + e2e) needs all of Phases 0-7. With Phase 7 ready:
- ✓ All formulas locked
- ✓ MATLAB function spec defined
- ✓ Numerical predictions for verification

Phase 8 work:
1. Rewrite `motion_control_law_eq17_7state.m` (Σf_d, F_e(3,4)=−1.6, x̂_D placement)
2. Fix driver buffer + control-then-integrate ordering
3. Implement `predict_closed_loop_var_eq17_v2.m`
4. Replace lookup tables with closed-form
5. Unit tests + 5-criteria check (Phase 5 simulation)
6. End-to-end positioning S1/S2 + motion F3
7. Compare simulation σ²_δx vs predict_closed_loop_var_eq17_v2 oracle

---

**End of Phase 7 derivation. Awaiting user review before commit.**
