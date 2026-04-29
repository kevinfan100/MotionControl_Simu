# Phase 6: R Matrix Derivation (Sensor + IIR-Derived Measurement Noise)

**Pre-Phase-6 commit**: `6c0d9d7` (Phase 5 Q matrix)
**Date**: 2026-04-29
**Status**: Phase 6 derivation. Awaiting user review.

This phase derives the 2×2 R matrix (per-axis), assembles closed-form formulas from Phase 2 (C_dpmr, C_n, IF_var) and Phase 5 (Q77), and locks the 3-guard adaptive logic.

---

## 1. Goals

1. Derive R(1,1) per-axis from sensor spec (open-loop, const)
2. Assemble R(2,2)[k] from Phase 2 formulas (closed-loop, time-varying)
3. Compute per-axis ξ (sensor floor) closed form
4. Add d-step delay propagation (5·Q77) to R(2,2)
5. Lock 3-guard adaptive logic (warm-up / NaN / wall)
6. Document v2 vs v1 R comparison
7. Identify per-axis ξ values for x, y, z

---

## 2. Setup Recap

Per Phase 1 §10.5, H matrix (unchanged from v1):
```
H = [ 1  0  0  0  0   0    0 ]    ← y_1: δx_m → δx_1
    [ 0  0  0  0  0   1   −d ]    ← y_2: a_xm → a_x − d·δa_x

For d=2: H(2,7) = −2
```

R is 2×2 per-axis (off-diagonal = 0 since y_1 measures position, y_2 measures gain — independent noise sources):
```
R = [ R(1,1)   0     ]
    [ 0       R(2,2) ]
```

Per-axis: 3 independent KFs (x, y, z), each with own R.

---

## 3. R(1,1) — Sensor Noise on δx_m (Open-loop, Const)

### 3.1 Derivation

```
y_1[k] = δx_m[k] = δx[k−d] + n_x[k]
n_x[k] ~ N(0, σ²_n_s,i)    per-axis
```

```
R(1,1)_i = Var(n_x,i) = σ²_n_s,i        per-axis i ∈ {x, y, z}
```

### 3.2 Per-axis values (from v1 sensor spec)

```
σ_n_s,x = 0.00062 µm = 0.62 nm           σ²_n_s,x ≈ 3.84e-7 µm²
σ_n_s,y = 0.000057 µm = 0.057 nm  ★      σ²_n_s,y ≈ 3.25e-9 µm²
σ_n_s,z = 0.00331 µm = 3.31 nm           σ²_n_s,z ≈ 1.10e-5 µm²
```

★ y-axis 0.057 nm is **unusually low** (10× smaller than x). v1 audit (`eq17_simulation_DEEP_audit_2026-04-29.md` §6.2 #5) flagged this as suspicious — baseline test observed y-axis bias +332%.

**Phase 8 sanity check**: verify y-axis sensor spec is correct OR adjust to physical value.

### 3.3 5-criteria check (R(1,1))

| Criterion | R(1,1) = σ²_n_s | Pass? |
|---|---|---|
| 1. Symbol | σ²_n_s — sensor spec (physical) | ✓ |
| 2. Independence | Change ctrl law → R(1,1) unchanged | ✓ |
| 3. Dimension | [µm²] = state² ✓ | ✓ |
| 4. Open-loop sim | Run sensor noise injection only, measure | trivial (sensor spec given) |
| 5. Traceability | Sensor calibration data | ✓ |

```
┌─────────────────────────────────────────────────────┐
│  R(1,1)_i = σ²_n_s,i    (per-axis const, open-loop) │
└─────────────────────────────────────────────────────┘
```

---

## 4. R(2,2) — IIR-derived Measurement Noise on a_xm (Closed-loop, Time-Varying)

### 4.1 R_2_intrinsic — IIR estimation noise (Phase 2 result)

From Phase 2 §7:
```
Var(σ̂²_δxr) = a_cov · IF_var · (σ²_δxr)²
```

a_xm linkage (Phase 2 §5.3):
```
a_xm = (σ̂²_δxr − C_n·σ²_n_s) / (C_dpmr · 4·k_B·T)
```

Var(a_xm) propagated from Var(σ̂²_δxr):
```
Var(a_xm) = Var(σ̂²_δxr) / (C_dpmr · 4·k_B·T)²
         = a_cov · IF_var · (σ²_δxr / (C_dpmr · 4·k_B·T))²
```

Substituting σ²_δxr = C_dpmr · 4kBT · a_x + C_n · σ²_n_s:
```
σ²_δxr / (C_dpmr · 4kBT) = a_x + C_n · σ²_n_s / (C_dpmr · 4kBT)
                       = a_x + ξ
```

where:
```
ξ_i := (C_n / C_dpmr) · σ²_n_s,i / (4·k_B·T)    per-axis
```

So:
```
R_2_intrinsic[k]_i = Var(a_xm,i) = a_cov · IF_var · (â_x,i[k] + ξ_i)²
```

```
┌─────────────────────────────────────────────────────────────┐
│  R_2_intrinsic,i[k] = a_cov · IF_var · (â_x,i[k] + ξ_i)²    │
│  ξ_i = (C_n / C_dpmr) · σ²_n_s,i / (4·k_B·T)                 │
└─────────────────────────────────────────────────────────────┘
```

### 4.2 ξ per-axis values

For λ_c = 0.7:
```
C_n/C_dpmr = 1.176/3.961 ≈ 0.297
4·k_B·T (T=298 K) ≈ 4 · 1.381e-5 [pN·µm/K] · 298 [K] ≈ 1.646e-2 [pN·µm]
```

Wait — k_B in our simulation units:
```
k_B = 1.381e-23 J/K = 1.381e-23 [N·m/K] = 1.381e-23 · 1e6 · 1e6 [pN·µm/K] · 1e-12... 
```

Let me re-check unit. k_B = 1.381e-23 J/K. 1 J = 1e6 pN·µm = 1e12 pN·nm... actually:
```
1 J = 1 kg·m²/s² = 1e3 g · (1e3 mm)² / s² = ...
1 J = 1e12 pN·µm    (since 1 N·m = 1e12 pN·µm)
```

Wait let me redo: 1 N = 1e12 pN, 1 m = 1e6 µm, so 1 N·m = 1e12 pN · 1e6 µm = 1e18 pN·µm.

Hmm, let me check: 1 N = ? pN. 1 N = 10^12 pN since 1 pN = 10^-12 N. ✓
1 m = ? µm. 1 m = 10^6 µm. ✓
So 1 J = 1 N·m = 10^12 · 10^6 pN·µm = 10^18 pN·µm.

But that gives k_B = 1.381e-23 · 10^18 pN·µm / K = 1.381e-5 pN·µm / K.

For T = 298 K:
```
k_B·T = 1.381e-5 · 298 ≈ 4.116e-3 pN·µm
4·k_B·T ≈ 1.646e-2 pN·µm
```

Hmm but typical thermal force variance in our simulation (per design.md §1):
σ²_T = 4·k_B·T·γ/Δt where γ ≈ 60 pN·s/µm (ferromicroprobe in water)

Per Δt step thermal variance:
σ²_T per step = 4·k_B·T·γ/Δt = 4 · 4.116e-3 · 60 / (1/1600) = 1.581e+3 pN²

This seems high but ok.

For ξ:
```
ξ_x = 0.297 · σ²_n_s,x / (1.646e-2) = 0.297 · 3.84e-7 / 1.646e-2 = 6.93e-6 µm/pN
ξ_y = 0.297 · 3.25e-9 / 1.646e-2 = 5.86e-8 µm/pN
ξ_z = 0.297 · 1.10e-5 / 1.646e-2 = 1.98e-4 µm/pN
```

Compare to a_x (motion gain):
- a_nom_far_wall = Δt/γ_N ≈ (1/1600)/60 ≈ 1.04e-5 µm/pN

So ξ relative to a_x:
```
ξ_x / a_nom ≈ 6.93e-6 / 1.04e-5 ≈ 0.67 (dominant!)
ξ_y / a_nom ≈ 5.86e-8 / 1.04e-5 ≈ 0.0056 (negligible)
ξ_z / a_nom ≈ 1.98e-4 / 1.04e-5 ≈ 19 (sensor-dominated!)
```

These ratios indicate which regime each axis is in:
- x: mixed (ξ ~ a_nom, both contribute to R_2_intrinsic)
- y: thermal-dominated (sensor very low → ξ very small)
- z: sensor-dominated (sensor noise high → ξ dominates)

★ z-axis "sensor-dominated" is a notable v1/v2 characteristic. Per the IIR linear inversion, when ξ ≫ a_x (z), R(2,2) is large because a_xm is mostly noise.

### 4.3 5·Q77 — d-step Delay Propagation

From paper 2025 §II.F + Phase 5 §6:
```
y_2[k] = a_xm[k] = a_x[k−d] + n_a[k]
       = a_x[k] − d·δa_x[k] + Σ_{j=1}^{d}(d−j+1)·w_a[k−j] + n_a[k]
                                └────────┬─────────┘
                                Past w_a propagation through delay
```

Variance contribution from past w_a:
```
Var(Σ_{j=1}^{d}(d−j+1)·w_a[k−j]) = Σ_{j=1}^{d}(d−j+1)² · Var(w_a) = (Σ(d−j+1)²) · Q77

For d=2: Σ_{j=1}^{2}(d−j+1)² = (2−1+1)² + (2−2+1)² = 2² + 1² = 5
```

```
delay_R2_factor(d) = Σ_{j=1}^{d}(d−j+1)² = ?
  d=1: 1
  d=2: 5      ← our case
  d=3: 14
  ...
```

So:
```
R_2_eff[k] = R_2_intrinsic[k] + 5·Q77[k]    (for d=2)
```

### 4.4 Full R(2,2)[k] formula

```
┌──────────────────────────────────────────────────────────────────────────┐
│                                                                          │
│  R(2,2)_i[k] = R_2_intrinsic,i[k] + delay_R2_factor(d) · Q77,i[k]        │
│             = a_cov · IF_var · (â_x,i[k] + ξ_i)² + 5·Q77,i[k]            │
│                                                              (for d=2)   │
│                                                                          │
└──────────────────────────────────────────────────────────────────────────┘
```

Time-varying via:
- â_x,i[k] (KF estimate, per step)
- Q77,i[k] (Phase 5: also depends on â_x,i² and h̄[k])

So R(2,2) is **dual time-varying via â_x and h̄**.

### 4.5 5-criteria check (R(2,2))

R(2,2) is closed-loop, so brainstorming §3.8 distinguishes:
- "Functional form open-loop" (depends on physical / structural constants)
- "Runtime adaptive" (uses â_x at runtime)

For R(2,2):
- a_cov, IF_var, ξ_i, C_n, C_dpmr: structural constants (Category B)
- â_x: estimate (Category A runtime)
- Q77: also Category A (depends on â_x², h̄)

This is "double-Adaptive" structure. Different from R(1,1) which is open-loop pure.

**5-criteria not directly applicable**; instead verify R(2,2) is "correctly closed-loop":
- ✓ ρ_a structure (via IF_var) reflects δx_mr autocorrelation under closed-loop
- ✓ â_x² scaling matches Var(a_xm) ∝ a_x² physical relationship
- ✓ Past w_a delay propagation captures d-step measurement lag

---

## 5. 3-Guard Adaptive Logic (Phase 0 §4.4 inherits)

### 5.1 Three guards

When triggered, R(2,2)[k] = 1e10 (effectively disable y_2 measurement, KF degrades to single-feedback):

| Guard | Condition | Reason |
|---|---|---|
| G1 (warm-up) | t < t_warmup_kf = 0.2 sec | IIR not yet converged, σ̂²_δxr unreliable |
| G2 (NaN guard) | σ̂²_δxr ≤ C_n · σ²_n_s | Numerator of a_xm formula → 0 or negative, division → NaN/inf |
| G3 (wall guard) | h̄ < h_bar_safe = 1.5 | Near wall, IIR formula assumptions break down |

Trigger logic (pure OR, no hysteresis):
```
if (t < t_warmup_kf) OR (σ̂²_δxr ≤ C_n·σ²_n_s) OR (h̄ < h_bar_safe):
    R(2,2)[k] = 1e10
else:
    R(2,2)[k] = R_2_intrinsic[k] + 5·Q77[k]    (normal)
```

### 5.2 G1 (warm-up) rationale

At t=0, IIR LP/HP filters not yet at steady state. σ̂²_δxr is highly biased. Use t_warmup_kf = 0.2 sec ≈ 320 steps (10·τ_LP for a_pd=0.05).

**v1 inheritance**: Phase 4a/4b/4c implementation has this guard.

### 5.3 G2 (NaN guard) rationale

a_xm formula:
```
a_xm = (σ̂²_δxr − C_n·σ²_n_s) / (C_dpmr · 4·k_B·T)
```

If σ̂²_δxr ≤ C_n·σ²_n_s (e.g., low SNR transient), numerator ≤ 0:
- Numerator = 0 → a_xm = 0 (false zero)
- Numerator < 0 → a_xm < 0 (unphysical, motion gain must be positive)

Disable y_2 measurement to prevent KF from corrupting a_x estimate with bad data.

### 5.4 G3 (wall guard) rationale

Near wall (h̄ ~ 1.0-1.5):
- c_perp(h̄) lookup error increases (Faxén polynomial less accurate)
- a_x changes rapidly → IIR cannot track (window too long)
- σ²_δxr formula assumes slowly-varying a_x

Disable y_2 to prevent KF from learning wrong a_x.

### 5.5 No hysteresis / ramp-down (Phase 0 §3.5 simplification)

Pure OR triggering. When any guard releases, R(2,2) instantly returns to normal. No smoothing.

This is design.md §9.5 "MVP simplification" — known acceptable but may need ramp-down if observed transients cause chi-squared kicks. Deferred to Phase 8 if needed.

---

## 6. R Matrix Per-step Build

### 6.1 Compact form (per axis)

```
R,i[k] = diag(R(1,1),i, R(2,2),i[k])

where:
  R(1,1),i = σ²_n_s,i                                         (const)
  R(2,2),i[k] = R_2_intrinsic,i[k] + 5·Q77,i[k]               (time-varying)
                if any guard triggered:
                  R(2,2),i[k] = 1e10
```

### 6.2 Per-axis structure

3 independent KFs (one per axis), each with own R(1,1), R(2,2). All independent.

Per-axis ξ:
```
ξ_i = (C_n/C_dpmr) · σ²_n_s,i / (4·k_B·T)
ξ_x = 6.93e-6 µm/pN
ξ_y = 5.86e-8 µm/pN
ξ_z = 1.98e-4 µm/pN
```

### 6.3 Code mapping

```matlab
function R = build_R_eq17_v2(a_hat, h_bar, t, sigma2_dxr_hat, params, ctrl_const)
    % Per-axis R for v2 EKF
    R = zeros(2, 2, 3);  % 3 axes
    
    sigma2_n_s = params.config.meas_noise_std.^2;  % 3x1
    a_cov = ctrl_const.a_cov;
    IF_var = ctrl_const.IF_var;     % from build_eq17_constants
    C_n = ctrl_const.C_n;            % 1.176
    C_dpmr = ctrl_const.C_dpmr;      % 3.961
    kBT = params.ctrl.k_B * params.ctrl.T;
    delay_factor = ctrl_const.delay_R2_factor;  % 5 for d=2
    
    % Per-axis xi
    xi = (C_n/C_dpmr) * sigma2_n_s / (4*kBT);    % 3x1
    
    % Q77 (from Phase 5)
    Q77 = build_Q77_eq17_v2(a_hat, h_bar, params);  % 3x1
    
    % Build R per axis
    for axis = 1:3
        R(1,1, axis) = sigma2_n_s(axis);
        
        % Check guards
        gate_warmup = (t < ctrl_const.t_warmup_kf);
        gate_nan = (sigma2_dxr_hat(axis) <= C_n * sigma2_n_s(axis));
        gate_wall = (h_bar < ctrl_const.h_bar_safe);
        
        if gate_warmup || gate_nan || gate_wall
            R(2,2, axis) = 1e10;
        else
            R_2_intrinsic = a_cov * IF_var * (a_hat(axis) + xi(axis))^2;
            R(2,2, axis) = R_2_intrinsic + delay_factor * Q77(axis);
        end
    end
end
```

---

## 7. v2 vs v1 R Matrix Comparison

| Entry | v1 | v2 | Change? |
|---|---|---|---|
| R(1,1)_i | σ²_n_s,i | σ²_n_s,i | ✓ same |
| R(2,2) intrinsic | a_cov·IF_var·(â_x+ξ)² | a_cov·IF_var·(â_x+ξ)² | ✓ same |
| R(2,2) delay propagation | 5·Q77 | 5·Q77 | ✓ same |
| 3-guard logic | warm-up + NaN + wall | warm-up + NaN + wall | ✓ same |
| ξ per-axis values | x: 6.93e-6, y: 5.86e-8, z: 1.98e-4 | same | ✓ same |
| C_n, C_dpmr, IF_var values | 1.18, 3.96, 4.21 | same | ✓ same |

**R matrix unchanged from v1 numerically**. The v2 fixes are at controller layer (Σf_d) and F_e(3,4), not R. This is consistent with Phase 2 finding: paper Eq.22 form preserved → C_dpmr/C_n unchanged → R(2,2) closed form unchanged.

---

## 8. Caveats / Open Items

### 8.1 y-axis sensor noise abnormality

```
σ_n_s,y = 0.057 nm    ← unusually low
ξ_y = 5.86e-8 µm/pN   ← ~100× smaller than ξ_x
```

This makes y-axis extremely thermal-dominated. v1 baseline observed y-axis bias +332% (outlier, see deep audit). 

**Phase 8 sanity**: 
- Verify y-axis sensor spec is correct
- If actual sensor has y noise ≈ 0.62 nm (similar to x), update config

### 8.2 No hysteresis (potential transient issue)

3-guard triggers / releases instantly. If σ̂²_δxr crosses C_n·σ²_n_s threshold rapidly, R(2,2) flips between 1e10 and normal each step → KF gain kicks → estimation instability.

v1 observed this rarely. If Phase 8 sees it, add hysteresis:
```
if state == 'gated' and σ̂²_δxr > 2·C_n·σ²_n_s:
    state = 'normal'
elif state == 'normal' and σ̂²_δxr < C_n·σ²_n_s:
    state = 'gated'
```

### 8.3 No ramp-down

Going from R=1e10 to normal is sudden. KF P(6,6) snap might kick a_x estimate.

If Phase 8 sees chi-squared kick, add 1-time-constant ramp:
```
R(2,2)[k] = R(2,2)_ramp,k + (R_normal − R_ramp,0)·(1 − exp(−τ_ramp · t_since_release))
```

Defer until empirically motivated.

### 8.4 Q-R cross-cov S ≠ 0 (Phase 1 §6.9)

n_x[k] is in BOTH Q (via ε control feedback) and R(1,1). Cross-cov S = (1−λ_c)·σ²_n_x.

Standard KF assumes S = 0. Effect ~0.16% (per design.md §3.8). Acceptable to ignore.

Alternative (if needed): use cross-cov form Joseph update. Phase 8 only if observed issue.

### 8.5 Phase 5 Q55 minor correction note

In Phase 5 §5.4, Q55 = a_x²·σ²_w_fD. To avoid self-loop, Implementation should use **a_nom²·σ²_w_fD** (Category B const), not â_x²·σ²_w_fD. Phase 5 doc to be amended; baseline σ²_w_fD = 0 makes this a cosmetic fix.

---

## 9. Phase 6 Summary

| Item | Result |
|---|---|
| R(1,1)_i | σ²_n_s,i (per-axis const) |
| **R(2,2)_i[k]** | a_cov·IF_var·(â_x,i[k]+ξ_i)² + 5·Q77,i[k] |
| ξ per-axis | x: 6.93e-6, y: 5.86e-8, z: 1.98e-4 µm/pN |
| 3-guard | OR triggered → R(2,2) = 1e10 |
| Per-step time-varying | Only R(2,2) (via â_x[k] and h̄[k] through Q77) |
| v2 vs v1 | All R formulas / values unchanged |

**R matrix: 0 v2 changes from v1**. The v2 architecture revisions (Σf_d retained, F_e(3,4) = -1.6) live at controller and F_e level, not R level. Per Phase 2: paper Eq.22 form is preserved, so all derived R quantities also preserved.

### Phase 7 entry conditions

Phase 7 (Lyapunov bench) needs:
- ✓ F_e v2 (Phase 1)
- ✓ H (Phase 1 §10.5)
- ✓ Q v2 (Phase 5)
- ✓ R v2 (this Phase)
- ✓ ε structure + C_dpmr/C_n (Phase 2)

Ready to enter Phase 7.

---

**End of Phase 6 derivation. Awaiting user review before commit.**
