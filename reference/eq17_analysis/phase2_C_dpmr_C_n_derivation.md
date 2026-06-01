# Phase 2: C_dpmr / C_n / IF_var Closed-form Derivation

**Pre-Phase-2 commit**: `b50bea3` (Phase 1 F_e derivation)
**Date**: 2026-04-29
**Status**: Phase 2 derivation. Awaiting user review.

This memo derives three structural constants used in IIR-derived a_xm and KF R(2,2) under v2 control law (paper Eq.17 + Σf_d retained). Per Phase 0 §10 categorization, all three are Category B (a_x-independent, offline-locked). Option I (slowly-varying x_D, σ²_w_fD = 0 baseline) used throughout.

---

## 1. Goals

1. Derive σ²_δx (steady-state tracking error variance) under ε_19 MA(2) structure
2. Derive σ²_δxr (IIR HP residual variance)
3. Identify closed-form coefficients C_dpmr (thermal) and C_n (sensor)
4. Derive ρ_δxr(τ) autocorrelation
5. Derive IF_var (Option A MA(2) full vs Option B AR(1) approximation)
6. Verify against v1 numerical values (3.96, 1.18, 4.224)

---

## 2. Setup Recap (from Phase 1)

Closed-loop tracking error dynamics (Eq.19 form, ignoring (x_D − x̂_D) which is handled separately):
```
δx[k+1] = λ_c · δx[k] − ε[k]

ε[k] = (1−λ_c)·n_x[k]                          ← current sensor noise
     + a_x·f_T[k]                              ← current thermal
     + (1−λ_c)·Σ_{i=1}^{d} a_x·f_T[k−i]        ← past thermal MA(d)
```

Per-step thermal variance (using fluctuation-dissipation):
```
σ²_dXT := Var(a_x · f_T) = a_x² · 4·k_B·T·γ/Δt = 4·k_B·T·a_x
σ²_n   := Var(n_x) = σ²_n_s
```

Independence: f_T ⊥ n_x. f_T white in time. n_x white in time.

---

## 3. Var(ε) Per Step + Cross-step Covariances

### 3.1 Single-step variance

Three independent contributions (n_x ⊥ f_T):
```
Var(ε)  = (1−λ_c)²·σ²_n          ← sensor noise
        + σ²_dXT                  ← current thermal
        + d·(1−λ_c)²·σ²_dXT       ← past thermal MA(d) [Σ over i=1..d, each (1−λ_c)²·σ²_dXT]
        
        = (1−λ_c)²·σ²_n + σ²_dXT·{1 + d·(1−λ_c)²}
```

For d=2, λ_c=0.7: (1−λ_c)² = 0.09
```
Var(ε) = 0.09·σ²_n + σ²_dXT·{1 + 0.18}
       = 0.09·σ²_n + 1.18·σ²_dXT
```

### 3.2 Cross-step covariance E[ε[k]·ε[k+τ]]

For τ = 1, identify common thermal terms in ε[k] and ε[k+1]:

| ε[k] term | ε[k+1] term | Common source | Cov contribution |
|---|---|---|---|
| a_x·f_T[k] | (1−λ_c)·a_x·f_T[k] | f_T[k] | (1−λ_c)·σ²_dXT |
| (1−λ_c)·a_x·f_T[k−1] | (1−λ_c)·a_x·f_T[k−1] | f_T[k−1] | (1−λ_c)²·σ²_dXT |

n_x is white → no contribution.

```
E[ε[k]·ε[k+1]] = (1−λ_c)·σ²_dXT + (1−λ_c)²·σ²_dXT
              = (1−λ_c)·σ²_dXT · {1 + (1−λ_c)}
              = (1−λ_c)·σ²_dXT · (2−λ_c)
```

For λ_c=0.7: 0.3 · 1.3 = 0.39 → 0.39·σ²_dXT

For τ = 2, common thermal:

| ε[k] term | ε[k+2] term | Common source | Cov contribution |
|---|---|---|---|
| a_x·f_T[k] | (1−λ_c)·a_x·f_T[k] | f_T[k] | (1−λ_c)·σ²_dXT |

Only one common source (f_T[k] in ε[k] as current and in ε[k+2] as lag-2).

```
E[ε[k]·ε[k+2]] = (1−λ_c)·σ²_dXT
```

For λ_c=0.7: 0.3·σ²_dXT

For τ ≥ 3: no common thermal terms (MA(2) tail decays). E = 0.

### 3.3 Summary autocovariance γ_ε(τ)

```
γ_ε(0) = (1−λ_c)²·σ²_n + σ²_dXT·{1 + d·(1−λ_c)²}
γ_ε(1) = (1−λ_c)·σ²_dXT · (2−λ_c)
γ_ε(2) = (1−λ_c)·σ²_dXT
γ_ε(τ ≥ 3) = 0
```

ε is **MA(2) on thermal**, **white on sensor** (n_x only contributes to lag 0).

---

## 4. σ²_δx via Direct Coefficient Method

For AR(1) process δx[k+1] = λ_c·δx[k] − ε[k], solve backward:
```
δx[k] = −Σ_{i=0}^{∞} λ_c^i · ε[k−1−i]
```

Compute Var(δx) by tracking how each noise source enters δx[k].

### 4.1 Thermal source contributions

For f_T[m] (m ≤ k−1), trace appearances in ε:
- In ε[m] with coefficient a_x (current of step m)
- In ε[m+1] with coefficient (1−λ_c)·a_x (lag-1)
- In ε[m+2] with coefficient (1−λ_c)·a_x (lag-2)

Each ε[k−1−i] enters δx[k] with weight −λ_c^i. So coefficient of f_T[m] in δx[k]:

```
c_T(m) = −a_x · {λ_c^{k−1−m}              ← from ε[m]
                + (1−λ_c)·λ_c^{k−2−m}      ← from ε[m+1]  
                + (1−λ_c)·λ_c^{k−3−m}}     ← from ε[m+2]
```

For m = k−1 (most recent): only ε[k−1] contributes
```
c_T(k−1) = −a_x · λ_c^0 = −a_x
```

For m = k−2: ε[k−2] and ε[k−1] contribute
```
c_T(k−2) = −a_x · {λ_c + (1−λ_c)} = −a_x · 1 = −a_x
```

For m = k−3: all three contribute
```
c_T(k−3) = −a_x · {λ_c² + (1−λ_c)·λ_c + (1−λ_c)}
         = −a_x · {λ_c² + λ_c − λ_c² + 1 − λ_c}
         = −a_x · 1 = −a_x
```

For m ≤ k−4 (general past):
```
c_T(m) = −a_x · λ_c^{k−3−m} · {λ_c² + (1−λ_c)·λ_c + (1−λ_c)}
       = −a_x · λ_c^{k−3−m} · 1
       = −a_x · λ_c^{k−3−m}
```

**Pattern**: thermal coefficient is **−a_x** for the 3 most recent steps (m=k−1, k−2, k−3), and decays as **−a_x·λ_c^{k−3−m}** for older.

### 4.2 Total thermal Var contribution

```
Var(δx)_thermal = a_x²·σ²_fT · {  1²              ← f_T[k−1] coefficient²
                                + 1²              ← f_T[k−2]
                                + 1²              ← f_T[k−3]
                                + Σ_{j=0}^{∞} λ_c^{2j}  ← older, geometric sum
                                  ↑   (note: m=k−4 has λ_c¹, m=k−5 has λ_c², ...)
                                  but actually Σ starts from λ_c^0 if we re-index, see correction below
}
```

Re-index correctly: for m = k−3−j (j ≥ 0), coefficient is −a_x·λ_c^j. So:
- j=0 (m=k−3): coefficient = −a_x·1 = −a_x ✓
- j=1 (m=k−4): coefficient = −a_x·λ_c
- j=2 (m=k−5): coefficient = −a_x·λ_c²
- ...

Plus the special m=k−1, k−2 each contributing −a_x.

```
Var(δx)_thermal = a_x²·σ²_fT · {1² + 1² + Σ_{j=0}^{∞} (λ_c^j)²}
                = a_x²·σ²_fT · {2 + 1/(1−λ_c²)}
                = σ²_dXT · {2 + 1/(1−λ_c²)}
                = σ²_dXT · C_dpmr
```

★ **C_dpmr = 2 + 1/(1−λ_c²)**.

For λ_c=0.7: 2 + 1/0.51 = 2 + 1.961 = 3.961 ≈ **3.96** ✓ matches v1.

### 4.3 Sensor source contributions

n_x[m] only appears in ε[m] (current step) with coefficient (1−λ_c). So in δx[k]:
```
c_n(m) = −(1−λ_c) · λ_c^{k−1−m}    for m ≤ k−1
```

Re-index j = k−1−m:
- j=0: −(1−λ_c)
- j=1: −(1−λ_c)·λ_c
- ...

```
Var(δx)_sensor = (1−λ_c)²·σ²_n · Σ_{j=0}^{∞} λ_c^{2j}
              = (1−λ_c)²·σ²_n / (1−λ_c²)
              = σ²_n · (1−λ_c) / (1+λ_c)
```

### 4.4 Total σ²_δx

```
σ²_δx = σ²_dXT · {2 + 1/(1−λ_c²)} + σ²_n · (1−λ_c)/(1+λ_c)
      = σ²_dXT · C_dpmr + σ²_n · (1−λ_c)/(1+λ_c)
```

For λ_c=0.7:
```
σ²_δx = 3.96·σ²_dXT + 0.176·σ²_n
```

---

## 5. σ²_δxr (IIR HP Residual Variance)

### 5.1 Relation to σ²_δx

In steady state with fully developed IIR LP filter, the LP estimate `δx̄_m` converges to the mean of `δx_m` (= 0 in zero-mean stationary case). So the HP residual `δx_r = δx_m − δx̄_m` ≈ δx_m.

```
σ²_δxr ≈ σ²_δx_m
```

And δx_m[k] = δx[k−d] + n_x[k] with δx and n_x independent:
```
σ²_δx_m = Var(δx[k−d]) + Var(n_x) = σ²_δx + σ²_n
```

(In steady state, Var(δx[k−d]) = Var(δx[k]) = σ²_δx.)

### 5.2 Closed form

```
σ²_δxr = σ²_δx + σ²_n
       = σ²_dXT · C_dpmr + σ²_n · (1−λ_c)/(1+λ_c) + σ²_n
       = σ²_dXT · C_dpmr + σ²_n · {(1−λ_c)/(1+λ_c) + 1}
       = σ²_dXT · C_dpmr + σ²_n · {(1−λ_c + 1+λ_c) / (1+λ_c)}
       = σ²_dXT · C_dpmr + σ²_n · 2/(1+λ_c)
       = σ²_dXT · C_dpmr + σ²_n · C_n
```

★ **C_n = 2/(1+λ_c)**.

For λ_c=0.7: 2/1.7 ≈ **1.176 ≈ 1.18** ✓ matches v1.

### 5.3 Verification

```
σ²_δxr = C_dpmr · 4·k_B·T·a_x + C_n · σ²_n_s
       = (2 + 1/(1−λ_c²)) · 4kBT·a_x + 2/(1+λ_c) · σ²_n_s
```

For λ_c=0.7: σ²_δxr = 3.96·4kBT·a_x + 1.18·σ²_n_s

This is the form used in **paper 2025 Eq.13** for IIR linear inversion:
```
a_xm[k] = ( σ̂²_δxr[k] − C_n·σ²_n_s ) / ( C_dpmr · 4·k_B·T )
```

---

## 6. ρ_δxr(τ) Autocorrelation (for IF_var)

### 6.1 ρ_δx(τ) for τ = 1, 2, 3+

Using δx[k+1] = λ_c·δx[k] − ε[k] and ε MA(2) tail:

For τ ≥ 3: ρ_δx(τ) = λ_c · ρ_δx(τ−1) (geometric decay because ε MA(2) has died out)

For τ = 1:
```
γ_δx(1) = E[δx[k]·δx[k+1]] = E[δx[k]·(λ_c·δx[k] − ε[k])]
        = λ_c·γ_δx(0) − E[δx[k]·ε[k]]
```

Compute E[δx[k]·ε[k]]:
```
δx[k] = −ε[k−1] − λ_c·ε[k−2] − λ_c²·ε[k−3] − ...
ε[k] correlated with ε[k−1], ε[k−2] (MA(2))
```

```
E[δx[k]·ε[k]] = −γ_ε(1) − λ_c·γ_ε(2) − λ_c²·γ_ε(3) − ...
              = −γ_ε(1) − λ_c·γ_ε(2)            (γ_ε(τ≥3) = 0)
              = −(1−λ_c)·σ²_dXT·(2−λ_c) − λ_c·(1−λ_c)·σ²_dXT
              = −(1−λ_c)·σ²_dXT · {(2−λ_c) + λ_c}
              = −(1−λ_c)·σ²_dXT · 2
              = −2(1−λ_c)·σ²_dXT
```

For λ_c=0.7: −2·0.3·σ²_dXT = −0.6·σ²_dXT

So:
```
γ_δx(1) = λ_c·γ_δx(0) + 2(1−λ_c)·σ²_dXT
        = λ_c·σ²_δx + 2(1−λ_c)·σ²_dXT
```

For λ_c=0.7, thermal-dominated (σ²_n ≪ σ²_dXT, common case):
σ²_δx ≈ 3.96·σ²_dXT
γ_δx(1) ≈ 0.7·3.96·σ²_dXT + 2·0.3·σ²_dXT = 2.77·σ²_dXT + 0.6·σ²_dXT = 3.37·σ²_dXT

ρ_δx(1) = γ_δx(1)/σ²_δx ≈ 3.37/3.96 ≈ **0.85**

### 6.2 ρ_δxr vs ρ_δx

For δx_r ≈ δx_m = δx[k−d] + n_x[k]:
```
γ_δxr(τ) = E[δx_r[k]·δx_r[k+τ]]
         = E[(δx[k−d] + n_x[k])·(δx[k−d+τ] + n_x[k+τ])]
         = γ_δx(τ) + σ²_n · δ(τ)        (n_x white, indep of δx)
```

So:
```
γ_δxr(0) = σ²_δx + σ²_n = σ²_δxr  ✓
γ_δxr(τ ≥ 1) = γ_δx(τ)              (n_x contributes 0 for nonzero lag)
```

```
ρ_δxr(τ) = γ_δxr(τ)/γ_δxr(0) = γ_δx(τ)/σ²_δxr   for τ ≥ 1
ρ_δxr(0) = 1
```

For thermal-dominated, σ²_δxr ≈ σ²_δx:
ρ_δxr(τ) ≈ ρ_δx(τ).

For λ_c=0.7, design.md §9 quotes (after adjustments for sensor floor):
```
ρ_δxr(1) ≈ 0.85
ρ_δxr(2) ≈ 0.67
ρ_δxr(τ ≥ 3) ≈ ρ_δxr(2) · λ_c^{τ−2}
```

---

## 7. IF_var Closed Form

### 7.1 Definition

For Wick theorem on Gaussian δx_r, the autocorrelation of squared signal:
```
ρ_{δx_r²}(τ) = ρ_δxr²(τ)
```

IF_var is the inflation factor relating Var(σ̂²_δxr) to (σ²_δxr)² for IIR EWMA estimator with parameter a_cov:
```
Var(σ̂²_δxr) = a_cov · IF_var · (σ²_δxr)² · (correction terms ~9% bias due to small a_cov)
```

The leading-order IF_var:
```
IF_var = 1 + 2·Σ_{τ=1}^{∞} ρ²_δxr(τ)
```

### 7.2 Option A — MA(2) full

Using full ρ_δxr(τ) values:
- ρ²(1) ≈ 0.85² = 0.7225
- ρ²(2) ≈ 0.67² = 0.4489
- ρ²(τ ≥ 3) ≈ 0.67²·λ_c^{2(τ−2)} (geometric decay rate λ_c²)

```
IF_var_A = 1 + 2·(0.7225 + 0.4489 + 0.4489·Σ_{j=1}^{∞} λ_c^{2j})
        = 1 + 2·{0.7225 + 0.4489·(1 + λ_c²/(1−λ_c²))}
        = 1 + 2·{0.7225 + 0.4489·1/(1−λ_c²)}
```

For λ_c=0.7, 1/(1−λ_c²) = 1.961:
```
IF_var_A = 1 + 2·{0.7225 + 0.4489·1.961}
        = 1 + 2·{0.7225 + 0.8803}
        = 1 + 2·1.6028
        = 1 + 3.2056
        ≈ 4.206
```

Approximate match to v1 quoted **4.224** (small rounding from approximating ρ values).

### 7.3 Option B — AR(1) approximation (ρ_δxr(τ) ≈ ρ_1 · λ_c^{τ−1})

If we approximate ρ_δxr(τ) as pure AR(1) decay:
```
IF_var_B = 1 + 2·ρ²(1)·Σ_{j=0}^{∞} λ_c^{2j}
         = 1 + 2·0.7225 / (1−λ_c²)
         = 1 + 2·0.7225 · 1.961
         = 1 + 2.834
         ≈ 3.834
```

Hmm, that's higher than design.md's quoted Option B value of 2.922. Let me reconsider.

Actually, design.md Option B computes ρ_δxr based on AR(1) approximation of the dynamics, not just the decay. Specifically, if ε is treated as white (ignoring MA(2) tail), then:
- ρ_δx(τ) = λ_c^τ for τ ≥ 0 (pure AR(1))
- IF_var_B = 1 + 2·Σ λ_c^{2τ} = 1 + 2·λ_c²/(1−λ_c²) = (1+λ_c²)/(1−λ_c²)

For λ_c=0.7: (1+0.49)/0.51 = 1.49/0.51 = **2.922** ✓ matches v1.

So Option B uses ρ_δx(τ) = λ_c^τ assumption (ignores MA(2)).

### 7.4 v2 IF_var values (Option A baseline, Option B for comparison)

```
IF_var Option A (MA(2) full, recommended):  ≈ 4.224 (v1 quoted)
IF_var Option B (AR(1) approx, rejected):    ≈ 2.922
```

design.md §9 chose Option A — it's structurally correct (matches paper's Eq.19 form ε MA(2) tail). Option B under-estimates IF_var by ~30% which would under-estimate R(2,2), making KF over-confident in a_xm measurement.

v2 inherits Option A.

---

## 8. v1 vs v2 Comparison

| Constant | v1 value | v2 derived value | Diff |
|---|---|---|---|
| C_dpmr | 3.96 | 2 + 1/(1−λ_c²) ≈ 3.961 | ✓ same |
| C_n | 1.18 | 2/(1+λ_c) ≈ 1.176 | ✓ same |
| IF_var (Option A) | 4.224 | ≈ 4.206 (computed here, slight rounding) | ✓ same |

**All three constants preserve v1 numerical values in v2**.

### 8.1 Why same? (Verification)

v1 used a SIMPLIFIED controller (no Σf_d), but the Lyapunov derivation in design.md §9 ASSUMED the ε MA(2) tail structure that paper Eq.22 describes. Effectively v1 was using **paper Eq.22's predicted ε structure** as if Σf_d were present — but the v1 IMPLEMENTATION dropped Σf_d.

This created the v1 paradox:
- Lyapunov σ²_δxr formula: paper Eq.22 form (assuming Σf_d) → C_dpmr=3.96, C_n=1.18 correct
- Actual closed-loop ε: WITHOUT Σf_d, has different MA structure (extra terms not absorbed)
- Consequence: a_hat estimation tracks WRONG variance scale → 33-44% bias

v2 fixes this by ADDING Σf_d to controller. Now:
- Lyapunov σ²_δxr formula: paper Eq.22 form ← unchanged
- Actual closed-loop ε: matches paper Eq.22 prediction
- C_dpmr, C_n values: same (formula structurally preserved)
- a_hat estimation: should track correctly → bias should disappear

**Implication**: the C_dpmr/C_n VALUES were always right; only the controller had to be fixed to match the assumed ε structure.

---

## 9. Phase 2 Open Items

### 9.1 Disturbance contribution

This derivation assumes (x_D − x̂_D) ≈ 0 (KF tracks x_D correctly at convergence). In transient or imperfect estimation, additional disturbance estimation error contributes to σ²_δxr. Phase 7 Lyapunov bench will quantify σ²_e_xD (disturbance estimation error variance) and add corrections.

### 9.2 Eq.18 vs Eq.19 form

C_dpmr/C_n derived under Eq.19 form. Eq.18 form would give same values (algebraic equivalence per Phase 1 §7), but derivation path differs (B·u handling needed in Eq.18). Eq.19 is cleaner for closed-form.

### 9.3 σ²_w_fD > 0 (Option II)

Under nonzero disturbance variation σ²_w_fD > 0, Option II would introduce δx_D coupling to F_e Row 3 and additional past w_xD contribution to ε. This shifts:
- Var(ε) per step
- γ_ε(τ) for τ = 1, 2
- Hence σ²_δx, σ²_δxr, ρ_δxr, IF_var

For simulation baseline σ²_w_fD = 0, Option I exact. Phase 5 Q derivation revisits if σ²_w_fD > 0.

---

## 10. Phase 2 Summary

| Constant | Closed Form | Numerical (λ_c=0.7) | Matches v1? |
|---|---|---|---|
| C_dpmr | 2 + 1/(1−λ_c²) | 3.961 | ✓ |
| C_n | 2/(1+λ_c) | 1.176 | ✓ |
| IF_var (Option A) | 1 + 2·{ρ²(1) + ρ²(2)/(1−λ_c²)} | 4.206 | ✓ (≈ v1 4.224) |

Key formulas locked for Phase 5/6:
```
σ²_δxr = C_dpmr · 4·k_B·T·a_x + C_n · σ²_n_s
       (paper Eq.22 form, Category B in (λ_c) only)

a_xm[k] = ( σ̂²_δxr[k] − C_n·σ²_n_s ) / ( C_dpmr · 4·k_B·T )
       (paper 2025 Eq.13 IIR linear inversion, structure unchanged)

R_2_intrinsic = a_cov · IF_var · ( a_x + ξ )²    [Phase 6 will lock]
        ξ := C_n/C_dpmr · σ²_n_s / (4·k_B·T)
```

**Verification**: C_dpmr/C_n VALUES preserved v1→v2 because paper Eq.22 form was always the "correct" target; v1's Σf_d-missing implementation caused mismatch between assumed and actual ε. v2's Σf_d-restored controller eliminates this mismatch.

---

**End of Phase 2 derivation. Awaiting user review before commit.**
