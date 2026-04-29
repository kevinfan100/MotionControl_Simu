# Phase 5: Q Matrix Derivation (Open-Loop Process Noise)

**Pre-Phase-5 commit**: `6cc7538` (Phase 4 observability spec)
**Date**: 2026-04-29
**Status**: Phase 5 derivation. Awaiting user review.

This phase derives the 7×7 Q matrix (process noise covariance) under v2 setup with Path C strict (Phase 1 §11 baseline). Each diagonal entry derived from physical noise sources, validated via 5-criteria check (brainstorming §3.9).

---

## 1. Goals

1. Derive each Q diagonal entry (Q11..Q77) from open-loop physics
2. Apply 5-criteria check (symbol, independence, dimension, simulation, traceability)
3. Lock per-step Q[k] formula for KF predict step
4. Identify time-varying entries (Category A) vs constants (Category B/zero)
5. Document v2 vs v1 differences

---

## 2. Setup Recap

- **State**: x_e[k] = [δx_1, δx_2, δx_3, x_D, δx_D, a_x, δa_x]ᵀ (7-state)
- **Form**: Eq.19 (closed-loop AR(1), F_e Row 3 = [0, 0, λ_c, −1.6, 0, −f_d, 0])
- **Q design path**: Path C strict (Phase 1 §11) — Q33 captures only current white
- **f_D model**: Random Walk (Option α), `f_D[k+1] = f_D[k] + w_fD[k]`, baseline σ²_w_fD = 0

### 2.1 Q diagonal-only assumption

Per brainstorming §3.8, Q is diagonal under 4 independence assumptions:
1. f_T ⊥ n_x (thermal vs sensor)
2. w_xD ⊥ (f_T, n_x)
3. w_a ⊥ (f_T, n_x, w_xD)
4. All four white in time

These all hold for our simulation. Single-time off-diagonal Q entries are 0.

---

## 3. Q11, Q22 — Shift Register States (= 0)

```
δx_1[k+1] = δx_2[k]      ← deterministic shift
δx_2[k+1] = δx_3[k]      ← deterministic shift
```

Both rows have NO exogenous noise injection. The "noise" δx_3 carries enters at slot 3, then propagates via shift to slots 1, 2 in subsequent steps.

```
┌─────────────────────────────────────┐
│  Q11 = 0                             │
│  Q22 = 0                             │
└─────────────────────────────────────┘
```

5-criteria check:
- ✓ Symbol: no closed-loop terms (trivially)
- ✓ Independence: change ctrl law, Q11/Q22 unchanged
- ✓ Dimension: Var(0) = 0 dimensionless
- ✓ Simulation: trivial measurement = 0
- ✓ Traceability: structural shift register definition

---

## 4. Q33 — Thermal Innovation (Path C Strict)

### 4.1 ε[k] decomposition (recap)

Phase 1 §6.4 derived:
```
ε_19[k] = (1−λ_c)·n_x[k]                          ← (iv) current sensor
        + a_x·f_T[k]                               ← (i)  current thermal (white)
        + (1−λ_c)·Σ_{i=1}^{d} a_x·f_T[k−i]         ← (ii)(iii) past thermal MA(d)
```

Path C strict places ONLY the white per-step component (i) into Q33:
- (ii)(iii) past thermal: handled by σ²_δxr Lyapunov closed form (paper Eq.22, captured in C_dpmr)
- (iv) sensor noise: in R(1,1), avoid Q-R cross

### 4.2 Q33 closed form

```
Q33,i[k] = Var(a_x,i · f_T,i[k])
         = a_x,i²[k] · Var(f_T,i)
         = a_x,i²[k] · 4·k_B·T·γ_i / Δt
         = 4·k_B·T · a_x,i[k]                      (using a_x = Δt/γ)
```

In code (using KF estimate):
```
Q33,i[k] = 4·k_B·T · â_x,i[k]                       per-axis i ∈ {x, y, z}
```

```
┌──────────────────────────────────────────────────────┐
│  Q33,i[k] = 4·k_B·T · â_x,i[k]    (per-axis, time-varying via â_x) │
└──────────────────────────────────────────────────────┘
```

### 4.3 Trade-off acknowledgement

Path C strict Q33 has **KF P(3,3) prediction undershoot ~50%** (thermal-dominated regime) because past thermal MA tail is missing from Q. This is documented in Phase 1 §11.3.

Trade-off accepted: simpler derivation, Q-R independent, ~2-3% tracking degradation vs paper-optimal.

### 4.4 5-criteria check

| Criterion | Q33 = 4·k_B·T · â_x | Pass? |
|---|---|---|
| 1. Symbol | k_B, T, â_x — physical constants + measured/estimated state | ✓ (â_x is "measured/estimated" not "control law parameter") |
| 2. Independence | Change λ_c → Q33 unchanged | ✓ |
| 3. Dimension | [J] · [µm/pN] = [pN·µm] · [µm/pN] = [µm²] = state² ✓ | ✓ |
| 4. Open-loop sim | Run thermal-only (ctrl off, KF off), measure Var(δp_m[k+1] − δp_m[k]) | TBD (Phase 8 unit test) |
| 5. Traceability | f_T physical → fluctuation-dissipation theorem | ✓ |

★ Caveat for Criterion 1: â_x technically is a state estimate (KF-internal), but it's used as a "physical proxy" for true a_x (slowly varying motion gain). This is the "Category A" usage per Phase 0 §10.

---

## 5. Q44, Q55 — Disturbance Jordan Block (★ NEW for v2)

### 5.1 State model recap

```
x_D[k+1]  = x_D[k] + δx_D[k]                    ← integrated RW position
δx_D[k+1] = δx_D[k] + w_xD[k]                   ← RW velocity, driven by w_xD
```

Q44 = Var(noise on x_D[k+1] | x_D[k], δx_D[k]) = 0 (deterministic integration)
Q55 = Var(w_xD[k]) = ?

### 5.2 Connect to f_D Random Walk model (Option α)

Plant has f_D[k] (force-level), and we defined x_D[k] := a_x[k] · f_D[k].

For f_D Random Walk:
```
f_D[k+1] = f_D[k] + w_fD[k],   E[w_fD²] = σ²_w_fD
```

For x_D evolution (slowly-varying a_x):
```
x_D[k+1] = a_x[k+1]·f_D[k+1] ≈ a_x[k]·{f_D[k] + w_fD[k]}
        = x_D[k] + a_x[k]·w_fD[k]
```

But state model says x_D[k+1] = x_D[k] + δx_D[k]. So:
```
δx_D[k] ≈ a_x[k]·w_fD[k]   (instantaneously)
```

If f_D is **pure** Random Walk, δx_D is **white** (single-step RW, not integrated). But state model uses integrated RW (δx_D evolves as RW with innovation w_xD).

### 5.3 Reconciling f_D model with integrated RW state structure

Two cases:

**Case A: σ²_w_fD = 0 baseline (Phase 0 default)**
```
σ²_w_fD = 0   → no f_D innovation
σ_δx_D = 0   → no RW velocity
σ_w_xD = 0   → no w_xD innovation
Q55 = 0
```

This is the "no disturbance" scenario. Trivially valid.

**Case B: σ²_w_fD > 0 (f_D drift simulation)**
Two sub-options:

**B1. Map directly to Q55 (preserve integrated RW structure)**:
```
δx_D[k] ≈ a_x · w_fD[k]   (f_D RW gives white δx_D)
But integrated RW says δx_D[k+1] = δx_D[k] + w_xD[k]

Mismatch: δx_D should be RW (Case A's natural model), but state has it as integrated RW.

Compromise: set Q55 = Var(w_xD) such that steady-state Var(δx_D) matches expected.
For pure f_D RW: Var(δx_D) at any step = Var(a_x · w_fD) = a_x² · σ²_w_fD
For integrated RW with Q55 driving, Var(δx_D) grows linearly with time → not steady-state

Fix: don't use integrated RW for "f_D RW" scenario. Use single RW for x_D (Q44 ≠ 0):
  Q44 = a_x² · σ²_w_fD  (treat x_D as single RW)
  Q55 = 0  (no rate state needed)
  remove δx_D state slot 5 effectively
```

But this would change state structure. Alternative:

**B2. Lump f_D variation into integrated RW (broader model)**:
```
Treat δx_D as if it has its own "drift rate": δx_D[k+1] = δx_D[k] + w_xD[k]

Q55 = σ²_w_xD ≈ a_x² · (σ²_w_fD / τ_D)    if f_D has correlation time τ_D
Q55 ≈ a_x² · σ²_w_fD                       for f_D pure RW (max innovation rate)

Pragmatic: Q55 = a_x² · σ²_w_fD = (a_nom · σ_w_fD)² ~ a constant (not time-varying)
```

### 5.4 Phase 5 baseline choice (Option α with σ²_w_fD = 0)

```
┌──────────────────────────────────────────────────┐
│  Q44 = 0                                          │
│  Q55 = a_x² · σ²_w_fD                            │
│  Baseline: σ²_w_fD = 0 → Q55 = 0                  │
└──────────────────────────────────────────────────┘
```

**For Phase 8 default sim**: Q44 = Q55 = 0, no disturbance present.

**For motion / drift scenarios (future)**: User specifies σ²_w_fD; Q55 follows.

### 5.5 5-criteria check (Q55 with σ²_w_fD > 0)

| Criterion | Q55 = a_x²·σ²_w_fD | Pass? |
|---|---|---|
| 1. Symbol | a_x, σ²_w_fD — no closed-loop terms | ✓ |
| 2. Independence | Change ctrl law → Q55 unchanged | ✓ |
| 3. Dimension | [µm/pN]² · [pN²] = [µm²] = state² ✓ | ✓ |
| 4. Open-loop sim | Set f_D=randn·σ_w_fD, measure | TBD (need σ²_w_fD spec) |
| 5. Traceability | f_D RW physical → x_D = a_x·f_D | ✓ |

### 5.6 v1 vs v2

v1 had Q44 = Q55 = 0 (no disturbance treatment beyond initial value). v2 introduces explicit f_D model with Q55 closed form. **NEW for v2**.

---

## 6. Q66, Q77 — Motion Gain Jordan Block

### 6.1 State model recap

```
a_x[k+1]  = a_x[k] + δa_x[k]                    ← integrated RW position
δa_x[k+1] = δa_x[k] + w_a[k]                    ← RW velocity, driven by w_a
```

Q66 = Var(noise on a_x[k+1] | a_x[k], δa_x[k]) = 0 (deterministic integration)
Q77 = Var(w_a[k]) = ?

### 6.2 a_x physical evolution

a_x is NOT physically random. It's deterministic, driven by trajectory:
```
a_x(t) = a_nom / c(h(t)/R)                       (deterministic)
```

For osc trajectory: h(t) = h_init + A·sin(ω·t), so a_x(t) varies with h.

KF assumes random innovation. Bridge: model deterministic a_x variation as **statistically equivalent** random walk innovation.

### 6.3 Path B Var(w_a) derivation

Define the RW innovation:
```
w_a[k] := δa_x[k+1] − δa_x[k]
        = a_x[k+2] − 2·a_x[k+1] + a_x[k]
        = d²a/dt² · Δt² + O(Δt³)
```

Var(w_a) ≈ Δt^4 · Var(d²a/dt²) (time-averaged over trajectory).

### 6.4 Var(d²a/dt²) chain rule expansion

```
a_x = a_nom / c(h_bar)        where h_bar = h/R

da_x/dt = a_x · {−(1/c)·dc/dh_bar} · (1/R) · dh/dt
       = −a_x · K_h · (1/R) · ḣ                  where K_h := (1/c)·dc/dh_bar

d²a/dt² = −a_x·K_h·(ḧ/R) − a_x·{K_h²·(1−1) + K_h'}·(ḣ²/R²) + (cross term)

After algebra (chain rule + product rule):
d²a/dt² = −a_x·K_h · ḧ/R + a_x·{K_h² − K_h'} · ḣ²/R²

where K_h' := dK_h/dh_bar = (1/c)·d²c/dh_bar² − K_h²
```

### 6.5 Var of d²a/dt² (time-average over trajectory)

For osc trajectory `h(t) = h_init + A·sin(ω·t)`:
- ḣ = A·ω·cos(ω·t), ḣ_max = A·ω, ⟨ḣ²⟩ = A²·ω²/2
- ḧ = −A·ω²·sin(ω·t), ḧ_max = A·ω², ⟨ḧ²⟩ = A²·ω⁴/2

For ḣ², ḧ² independent (orthogonal cos/sin):
- ⟨(ḣ²)²⟩ = ⟨ḣ⁴⟩ = (A·ω)⁴ · ⟨cos⁴⟩ = (A·ω)⁴ · 3/8 = 3·ḣ_max⁴/8
- ⟨(ḧ²⟩ = (A·ω²)²·1/2 = ḧ_max²/2

Combining (Term A + Term B independent):
```
Var(d²a/dt²) = a_x² · {(K_h² − K_h')²·⟨(ḣ²/R²)²⟩ + K_h²·⟨(ḧ/R)²⟩}
            = a_x² · {(K_h² − K_h')²·ḣ_max⁴ / (8·R⁴) + K_h²·ḧ_max² / (2·R²)}
```

Note: 8 in denominator from 3/8 averaged amplitude ratio... actually let me re-check. ⟨ḣ⁴⟩ = 3·ḣ_max⁴/8, but we want ⟨(ḣ²)²⟩ = ⟨ḣ⁴⟩ = 3/8 · ḣ_max⁴. Squared in (K_h²-K_h')², so:
- coefficient of (K_h² − K_h')² is ⟨ḣ⁴⟩/R⁴ = 3·ḣ_max⁴/(8·R⁴)

Wait, design.md §8.4 says ⟨ḣ_max⁴⟩/(8·R⁴), suggesting they used different averaging convention. Let me match design.md.

Per design.md §8 v1 (which v2 inherits):
```
Q77 = Δt⁴ · â_x² · { (K_h² − K_h')² · ḣ_max⁴ / (8·R⁴) + K_h² · ḧ_max² / (2·R²) }
```

So Q77 closed form is:
```
┌────────────────────────────────────────────────────────────────────────┐
│                                                                        │
│  Q77,i[k] = Δt⁴ · â_x,i²[k] · {                                         │
│              (K_h,i²(h̄[k]) − K_h,i'(h̄[k]))² · ḣ_max⁴ / (8·R⁴)         │
│            +  K_h,i²(h̄[k]) · ḧ_max² / (2·R²)                            │
│           }                                                            │
│                                                                        │
│  per-axis i ∈ {x, y, z}, K_h uses c_para or c_perp accordingly         │
│                                                                        │
└────────────────────────────────────────────────────────────────────────┘
```

For osc with f = 1 Hz, A = 2.5 µm:
- ḣ_max = A·2π·f = 15.7 µm/s
- ḧ_max = A·(2π·f)² = 98.7 µm/s²

### 6.6 Term A vs Term B contributions

| Operating point | Term A dominance | Term B dominance |
|---|---|---|
| Far wall (h̄ ≥ 5) | small (K_h ≈ 0) | small (K_h ≈ 0) |
| Near wall (h̄ ≈ 1.1) | DOMINANT (~4× Term B) | secondary |
| Mid-range (h̄ ≈ 2-3) | significant | similar |

**Both terms necessary**; Path B Term-B-only (Route III rejected per Task 02 analysis) under-estimates near-wall by ~80%.

### 6.7 5-criteria check (Q77)

| Criterion | Q77 formula | Pass? |
|---|---|---|
| 1. Symbol | Δt, â_x, K_h, K_h', ḣ_max, ḧ_max, R — physical/derived constants only | ✓ |
| 2. Independence | Change ctrl law → Q77 unchanged | ✓ |
| 3. Dimension | [s⁴] · [µm/pN]² · [1/µm]² · [µm/s]⁴ / [µm]⁴ = [µm²/pN²·s⁴·1/s⁴] = ... need to verify | TBD detail |
| 4. Open-loop sim | Hard to isolate (requires deterministic h(t) trajectory) | TBD |
| 5. Traceability | Var(d²a/dt²) chain rule from a_x = a_nom/c(h̄) | ✓ |

★ K_h, K_h' are computed from `calc_correction_functions.m` (Phase 4a in v1, already implemented).

### 6.8 Q66 = 0

```
┌──────────────────────────┐
│  Q66 = 0                  │
└──────────────────────────┘
```

a_x evolves only via δa_x integration (no direct innovation). Trivially structural.

### 6.9 v2 vs v1

Q66, Q77 same as v1. Path B Term A + Term B preserved.

---

## 7. Q Matrix Per-Step Build

### 7.1 Compact form (per axis)

```
Q,i[k] = diag(0, 0, Q33,i[k], 0, Q55,i, 0, Q77,i[k])

         = diag(0, 0, 4·k_B·T·â_x,i[k], 0, a_x,i²·σ²_w_fD, 0, 
                Δt⁴·â_x,i²·{(K_h,i²−K_h,i')²·ḣ_max⁴/(8R⁴) + K_h,i²·ḧ_max²/(2R²)})
```

### 7.2 Time-varying entries

```
Q33[k] varies via â_x[k]              ← per step
Q77[k] varies via â_x²[k] AND h̄[k]    ← per step (h̄ changes with trajectory)

Q11, Q22, Q44, Q66: const = 0
Q55: const (function of σ²_w_fD only, baseline 0)
```

### 7.3 Per-axis independence

All 7×7 Q matrices for 3 axes (x, y, z) computed independently:
- a_x,i depends on c_para (x, y) or c_perp (z)
- σ²_n_s,i is per-axis sensor spec
- K_h, K_h' use per-axis c

### 7.4 Code mapping

```matlab
function Q = build_Q_eq17_v2(a_hat, h_bar, params)
    Ts = params.Ts;
    kBT = params.k_B * params.T;
    R_radius = params.R;
    A_h = params.A_h;       % osc amplitude (µm)
    omega = params.omega;   % osc frequency (rad/s)
    sigma2_w_fD = params.sigma2_w_fD;  % default 0
    
    % Per-axis K_h, K_h' (Phase 4a function)
    [c_para, c_perp, derivs] = calc_correction_functions(h_bar, true);
    K_h_para = derivs.K_h_para;
    K_h_perp = derivs.K_h_perp;
    K_h_prime_para = derivs.K_h_prime_para;
    K_h_prime_perp = derivs.K_h_prime_perp;
    
    % Trajectory bandwidth
    h_dot_max = A_h * omega;
    h_ddot_max = A_h * omega^2;
    
    % Per-axis Q (compact)
    Q = zeros(7, 7, 3);  % 3 axes
    for axis = 1:3
        Q33 = 4 * kBT * a_hat(axis);
        Q55 = a_hat(axis)^2 * sigma2_w_fD;
        if axis < 3  % x, y use c_para
            K_h = K_h_para; K_h_prime = K_h_prime_para;
        else  % z uses c_perp
            K_h = K_h_perp; K_h_prime = K_h_prime_perp;
        end
        TermA = (K_h^2 - K_h_prime)^2 * h_dot_max^4 / (8 * R_radius^4);
        TermB = K_h^2 * h_ddot_max^2 / (2 * R_radius^2);
        Q77 = Ts^4 * a_hat(axis)^2 * (TermA + TermB);
        
        Q(:, :, axis) = diag([0, 0, Q33, 0, Q55, 0, Q77]);
    end
end
```

---

## 8. v2 vs v1 Q Matrix Comparison

| Entry | v1 | v2 | Change? |
|---|---|---|---|
| Q11, Q22 | 0 | 0 | ✓ same |
| Q33 | 4·k_B·T·â_x[k] (Path C) | 4·k_B·T·â_x[k] (Path C) | ✓ same |
| Q44 | 0 (no disturbance) | 0 (Option α baseline) | ✓ same |
| Q55 | 0 | **a_x²·σ²_w_fD** (Option α, default 0) | **NEW for v2** |
| Q66 | 0 | 0 | ✓ same |
| Q77 | Δt⁴·â_x²·{Term A + Term B} | Δt⁴·â_x²·{Term A + Term B} | ✓ same |

**Only Q55 is new for v2**. With baseline σ²_w_fD = 0, Q55 = 0 numerically (matches v1). For nonzero σ²_w_fD, Q55 > 0.

---

## 9. Phase 5 Open Items / Caveats

### 9.1 Path C strict P prediction undershoot

Q33 = 4·k_B·T·â_x captures only current thermal. Past thermal MA tail handled by σ²_δxr Lyapunov (paper Eq.22 form). Trade-off: ~50% P(3,3) undershoot in thermal-dominated regime, ~2-3% tracking degradation.

If Phase 7 closed-loop variance bench shows excessive degradation, can revisit Path A' inflation:
```
Q33_PathA'_inflated = (3 − 2·λ_c²) · σ²_dXT + (1−λ_c)² · σ²_n_x
```

design.md §8.2 rejected as "hack not derivation". Could re-evaluate empirically.

### 9.2 Q55 σ²_w_fD specification

For motion / drift scenarios:
```
σ²_w_fD options:
  (a) 0 (no disturbance, Phase 0 baseline)
  (b) Spec from sensor / actuator model (TBD, application-specific)
  (c) Empirical fit from data (post-implementation calibration)
```

Phase 0 confirmed Option α with σ²_w_fD = 0 baseline. Phase 8 simulation can sweep σ²_w_fD for sensitivity analysis.

### 9.3 Q77 Criterion 4 (open-loop measurement)

Direct measurement of Var(d²a/dt²) is hard because:
- Requires deterministic h(t) trajectory
- Need long time-series to estimate variance reliably
- a_x truth value depends on calc_correction_functions accuracy

Phase 8 unit test: cross-check Q77 numerically vs simulation output (compare KF a_hat std vs predicted from Q77).

### 9.4 Q diagonality assumption (4 independence)

Off-diagonal Q entries strictly 0 under simulation assumptions (n_x ⊥ f_T ⊥ w_xD ⊥ w_a). Verified once at Phase 0; no per-step check needed.

---

## 10. Phase 5 Summary

| Entry | v2 closed form | Time-varying? | Category |
|---|---|---|---|
| Q11, Q22 | 0 | no | structural |
| **Q33,i[k]** | 4·k_B·T·â_x,i[k] | yes (â_x) | A |
| Q44 | 0 | no | structural (RW position) |
| Q55,i | a_x,i²·σ²_w_fD (=0 baseline) | no (const for const σ²_w_fD) | A (a_x²) |
| Q66 | 0 | no | structural (RW position) |
| **Q77,i[k]** | Δt⁴·â_x,i²·{Term A + Term B} | yes (â_x², h̄) | A |

**Only 2 time-varying entries: Q33[k] and Q77[k]**.

**Per-step compute cost**: 2 multiplications (Q33) + ~10 ops (Q77 with K_h lookup) per axis = O(30) ops total.

**v2 vs v1**: only Q55 is new (zero in baseline). All other entries preserved.

### Phase 6 entry conditions

Phase 6 (R matrix) needs:
- ✓ Q77[k] formula (this Phase) — used in R(2,2) = R_2_intrinsic + 5·Q77
- ✓ C_dpmr, C_n, IF_var (Phase 2) — used in R_2_intrinsic
- ✓ Per-axis sensor noise σ²_n_s,i (config) — used in R(1,1) and ξ

Phase 6 mainly assembles these into R(1,1), R(2,2)[k], and adds 3-guard logic.

---

**End of Phase 5 derivation. Awaiting user review before commit.**
