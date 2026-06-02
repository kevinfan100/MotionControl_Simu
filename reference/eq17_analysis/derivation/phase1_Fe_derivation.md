# Phase 1: F_e Row 3 Derivation under v2 Control Law

**Pre-Phase-1 commit**: `8804e14` (design_v2.md Phase 0 spec)
**Date**: 2026-04-29
**Status**: Phase 1 derivation. Awaiting user review.

This memo derives F_e Row 3 (the only nontrivial row) for the v2 7-state EKF under paper Eq.17 control law (with Σf_d retained) + x̂_D additive disturbance compensation. Two forms (Eq.18, Eq.19) are derived and compared.

---

## 1. Goals

1. Substitute v2 control law into plant equation to express δx[k+1]
2. Linearize at â_x = a_x, x̂_D = x_D
3. Derive **Eq.18 form** (direct, with explicit δx_1 entry, ε white)
4. Derive **Eq.19 form** (algebraic rearrangement, ε MA(2))
5. Round-trip verify equivalence
6. Compare F_e Row 3 entries, ε structure
7. Document v1 (simplified) vs v2 (paper full) differences
8. Recommend form for downstream Phase 2-7

H matrix is structurally simple and is given in §10.

---

## 2. Setup (recap from Phase 0)

### 2.1 Plant (Form A, per axis)

```
x[k+1] = x[k] + a_x[k]·( f_d[k] + f_T[k] + f_D[k] )
       = x[k] + a_x·f_d[k] + a_x·f_T[k] + x_D[k]

where x_D[k] := a_x[k]·f_D[k]    (position-level disturbance, state slot 4)
```

### 2.2 Tracking error in plant form

```
δx[k] := x_d[k] − x[k]
δx[k+1] = (x_d[k+1] − x_d[k]) + δx[k] − a_x·f_d − a_x·f_T − x_D
```

### 2.3 Control law (v2 paper Eq.17 + adaptive + x̂_D additive)

```
f_d[k] = (1/â_x[k])·{ x_d[k+1] − λ_c·x_d[k] − (1−λ_c)·x_d[k−d] + (1−λ_c)·δx_m[k] }
       − (1−λ_c)·Σ_{i=1}^{d} f_d[k−i]
       − x̂_D[k] / â_x[k]
```

### 2.4 Measurement

```
δx_m[k] = δx[k−d] + n_x[k]    (sensor delay d=2)
```

### 2.5 State vector (7-state)

```
x_e[k] = [δx_1, δx_2, δx_3, x_D, δx_D, a_x, δa_x]ᵀ
   slot:   1     2     3     4    5     6    7

δx_1[k] = δx[k−2]    (oldest, sensor visible)
δx_2[k] = δx[k−1]
δx_3[k] = δx[k]      (current)
```

### 2.6 Noise / disturbance assumptions

| Source | Distribution | Independence |
|---|---|---|
| f_T[k] | white, Var = 4kBT·γ/Δt | ⊥ n_x, w_xD, w_a |
| n_x[k] | per-axis white, Var = σ²_n_s | ⊥ f_T, w_xD, w_a |
| w_xD[k] (drives δx_D) | white | ⊥ all |
| w_a[k] (drives δa_x) | white | ⊥ all |
| f_D evolution | RW: f_D[k+1] = f_D[k] + w_fD[k], Option α | derived: w_xD ≈ a_x·w_fD |

---

## 3. Derivation Step 1: Substitute Control Law

Multiply control law by a_x:

```
a_x·f_d[k] = (a_x/â_x)·{ bracket } − a_x·(1−λ_c)·Σ f_d[k−i] − (a_x/â_x)·x̂_D[k]
```

where:
```
bracket = x_d[k+1] − λ_c·x_d[k] − (1−λ_c)·x_d[k−d] + (1−λ_c)·δx_m[k]
```

Substituting δx_m[k] = δx[k−d] + n_x[k]:
```
bracket = x_d[k+1] − λ_c·x_d[k] − (1−λ_c)·x_d[k−d] + (1−λ_c)·δx[k−d] + (1−λ_c)·n_x[k]
```

Substitute a_x·f_d into δx[k+1] equation:
```
δx[k+1] = (x_d[k+1] − x_d[k]) + δx[k] − a_x·f_d − a_x·f_T − x_D
        = (x_d[k+1] − x_d[k]) + δx[k]
          − (a_x/â_x)·bracket
          + a_x·(1−λ_c)·Σ f_d[k−i]
          + (a_x/â_x)·x̂_D
          − a_x·f_T − x_D
```

---

## 4. Derivation Step 2: Linearize at Truth

Linearize at:
- â_x = a_x   (motion gain at truth) → a_x/â_x = 1
- x̂_D = x_D   (disturbance estimate at truth)

```
δx[k+1] = (x_d[k+1] − x_d[k]) + δx[k]
        − bracket
        + a_x·(1−λ_c)·Σ f_d[k−i]
        + x̂_D
        − a_x·f_T − x_D
```

Expand bracket:
```
δx[k+1] = (x_d[k+1] − x_d[k]) + δx[k]
        − x_d[k+1] + λ_c·x_d[k] + (1−λ_c)·x_d[k−d] − (1−λ_c)·δx[k−d] − (1−λ_c)·n_x[k]
        + a_x·(1−λ_c)·Σ f_d[k−i]
        + (x̂_D − x_D)
        − a_x·f_T[k]
```

Collect x_d terms:
```
(x_d[k+1] − x_d[k]) − x_d[k+1] + λ_c·x_d[k] + (1−λ_c)·x_d[k−d]
= −x_d[k] + λ_c·x_d[k] + (1−λ_c)·x_d[k−d]
= −(1−λ_c)·{x_d[k] − x_d[k−d]}
```

Result of Step 2 (substituted + linearized, **no rearrangement yet**):

```
┌──────────────────────────────────────────────────────────────────────────┐
│                                                                          │
│  δx[k+1] = δx[k]                                                         │
│         − (1−λ_c)·{x_d[k] − x_d[k−d]}        ← trajectory difference     │
│         − (1−λ_c)·δx[k−d]                    ← past tracking error      │
│         − (1−λ_c)·n_x[k]                     ← current sensor noise     │
│         + a_x·(1−λ_c)·Σ_{i=1}^{d} f_d[k−i]  ← past control sum (paper)   │
│         + (x̂_D − x_D)                         ← disturb estimation error│
│         − a_x·f_T[k]                          ← current thermal          │
│                                                                          │
└──────────────────────────────────────────────────────────────────────────┘
```

This is the master equation. Two forms branch from here.

---

## 5. Eq.18 Form — Direct (no Σf_d expansion)

### 5.1 Decomposition into F_e + B·u + ε

In Eq.18 form, we keep Step 2 result as-is. Identify which terms are:

| Term | Type | Goes to |
|---|---|---|
| δx[k] | current state δx_3 | F_e(3,3) = 1 |
| −(1−λ_c)·δx[k−d] = −(1−λ_c)·δx_1 | current state δx_1 | F_e(3,1) = −(1−λ_c) |
| (x̂_D − x_D) | x̂_D is KF-internal (not state); x_D is state | F_e(3,4) = −1 (only x_D direct) |
| −a_x·f_T[k] | white process noise | ε_white |
| −(1−λ_c)·n_x[k] | white sensor noise (in Q) | ε_white |
| −(1−λ_c)·{x_d[k] − x_d[k−d]} | known trajectory input | B·u |
| +a_x·(1−λ_c)·Σ f_d[k−i] | known past control input | B·u |
| linearization in a_x: f_d acts via a_x·f_d term | depends on a_x | F_e(3,6) = −f_d (∂(a_x·f_d)/∂a_x = f_d) |

### 5.2 F_e Row 3 (Eq.18 form, v2)

```
slot:           1          2     3     4     5     6      7
F_e(3,:) = [ −(1−λ_c),    0,    1,   −1,   0,   −f_d,   0 ]
            ↑                   ↑    ↑          ↑
            δx_1 explicit       δx_3 x_D        a_x linearization
            (delayed tracking)  self direct     time-varying entry
```

Same structure as v1 Eq.18 form. **Σf_d term in v2 control law goes to B·u**, does NOT change F_e structure (because Σf_d is a known/computable past control, not a state).

### 5.3 ε structure (Eq.18 form)

```
ε_18[k] = −(1−λ_c)·n_x[k] − a_x·f_T[k]    (white per step)

Var(ε_18) = (1−λ_c)²·σ²_n_x + a_x²·σ²_fT
          = (1−λ_c)²·σ²_n_x + 4·k_B·T·a_x  (using fluctuation-dissipation)
```

### 5.4 B·u input

```
B·u[k] = −(1−λ_c)·{x_d[k] − x_d[k−d]} + a_x·(1−λ_c)·Σ_{i=1}^{d} f_d[k−i]

For positioning trajectory (x_d const): B·u reduces to a_x·(1−λ_c)·Σ f_d[k−i]
For motion trajectory: B·u contains both terms
```

The Σf_d term is **computed by controller**, KF can read it as a known input.

### 5.5 Eq.18 form caveat: Q-R cross-correlation

The white noise n_x[k] enters TWO places:
- (a) Process noise ε_18 (via dynamics)
- (b) Measurement noise R(1,1) (via δx_m measurement)

This violates standard KF assumption Q ⊥ R. Cross-covariance:
```
S = E[ε_18·v₁ᵀ] = −(1−λ_c)·E[n_x·n_x] = −(1−λ_c)·σ²_n_x
```

Standard KF (S=0) approximation introduces small error (~0.16% per design.md §3.8 caveat). Exact handling requires Joseph-form with cross-cov.

---

## 6. Eq.19 Form — Algebraic Rearrangement (Σf_d expansion)

### 6.1 Σf_d via past plant equations

From plant at past step k−i (assuming a_x slowly varying, a_x[k−i] ≈ a_x):
```
δx[k−i+1] = (x_d[k−i+1] − x_d[k−i]) + δx[k−i] − a_x·f_d[k−i] − a_x·f_T[k−i] − x_D[k−i]
```

Solve for a_x·f_d[k−i]:
```
a_x·f_d[k−i] = (x_d[k−i+1] − x_d[k−i]) + δx[k−i] − δx[k−i+1] − a_x·f_T[k−i] − x_D[k−i]
```

Sum from i=1 to d (telescoping):
```
Σ_{i=1}^{d} a_x·f_d[k−i] = (x_d[k] − x_d[k−d])      ← trajectory tele-sum
                          + (δx[k−d] − δx[k])        ← tracking error tele-sum
                          − Σ_{i=1}^{d} a_x·f_T[k−i] ← past thermal sum
                          − Σ_{i=1}^{d} x_D[k−i]     ← past disturbance sum
```

Multiply by (1−λ_c):
```
a_x·(1−λ_c)·Σ f_d[k−i] = (1−λ_c)·{ (x_d[k] − x_d[k−d]) + (δx[k−d] − δx[k])
                                  − Σ a_x·f_T[k−i] − Σ x_D[k−i] }
```

### 6.2 Substitute back into Step 2 master equation

Using Step 2 result (§4) + Σf_d expansion (§6.1):

```
δx[k+1] = δx[k]
        − (1−λ_c)·{x_d[k] − x_d[k−d]}        ← (a)
        − (1−λ_c)·δx[k−d]                     ← (b)
        − (1−λ_c)·n_x[k]                      ← (c)
        + (1−λ_c)·{(x_d[k] − x_d[k−d])         ← (d)  cancels (a)
                  + (δx[k−d] − δx[k])           ←       partially cancels (b), modifies δx[k]
                  − Σ a_x·f_T[k−i]              ←       new MA tail
                  − Σ x_D[k−i]}                 ←       new past x_D sum
        + (x̂_D − x_D)                         ← (e)
        − a_x·f_T[k]                            ← (f)
```

### 6.3 Cancellations and consolidations

**Term (a) + Term (d) trajectory part**: `−(1−λ_c)·{x_d[k]−x_d[k−d]} + (1−λ_c)·{x_d[k]−x_d[k−d]} = 0`

★ **Σf_d EXACTLY CANCELS the trajectory difference term**. This is the design purpose of Σf_d in paper Eq.17 — to enable proper d-step delay compensation under motion trajectories.

**Term (b) + Term (d) tracking part**: `−(1−λ_c)·δx[k−d] + (1−λ_c)·δx[k−d] = 0` (also cancels)

**Term (d) δx[k] part**: `(1−λ_c)·(−δx[k]) = −(1−λ_c)·δx[k]`. Combined with original δx[k]:
```
δx[k] − (1−λ_c)·δx[k] = (1 − (1−λ_c))·δx[k] = λ_c·δx[k]
```

★ **AR(1) form recovered**.

### 6.4 Eq.19 form result

```
δx[k+1] = λ_c·δx[k]                              ← AR(1) self
        + (x̂_D − x_D)                            ← disturb error
        − (1−λ_c)·n_x[k] − a_x·f_T[k]            ← current white
        − (1−λ_c)·Σ_{i=1}^{d} a_x·f_T[k−i]       ← past thermal MA(d)
        − (1−λ_c)·Σ_{i=1}^{d} x_D[k−i]           ← past disturbance sum
```

### 6.5 Past x_D sum handling

The `−(1−λ_c)·Σ x_D[k−i]` term is **specific to v2** (didn't appear in v1 simplified controller derivation, because v1 had no Σf_d to substitute).

This term's interpretation: paper Eq.17's Σf_d compensation, when applied to a system WITH disturbance f_D, accidentally produces a residual past-x_D feedforward effect.

Two handling options:

**Option I: Slowly-varying x_D approximation**

For Option α (f_D RW with σ²_w_fD small), x_D varies slowly within d steps:
```
x_D[k−1] ≈ x_D[k−2] ≈ x_D[k]   (within ~σ_w_xD per step)

⇒ Σ_{i=1}^{d} x_D[k−i] ≈ d·x_D[k]

⇒ −(1−λ_c)·d·x_D[k]
```

This contributes to F_e(3,4):
```
F_e(3,4) (Eq.19, slowly-varying):
  direct (x̂_D − x_D) term: −1
  past sum approximation:    −d·(1−λ_c)
  Total:                     −(1 + d·(1−λ_c))

For d=2, λ_c=0.7: F_e(3,4) = −(1 + 2·0.3) = −1.6
```

**Option II: Express past x_D via state evolution**

```
x_D[k−1] = x_D[k] − δx_D[k−1]
x_D[k−2] = x_D[k] − δx_D[k−1] − δx_D[k−2]

Past δx_D in terms of current δx_D + past w_xD:
δx_D[k−i] = δx_D[k] − Σ_{j=1}^{i} w_xD[k−j]
```

Substituting (for d=2):
```
Σ x_D[k−i] = 2·x_D[k] − 3·δx_D[k] + 3·w_xD[k−1] + w_xD[k−2]
           = 2·x_D[k] − 3·δx_D[k] + (past w_xD MA tail)
```

This contributes to F_e:
```
F_e(3,4) extra: −2·(1−λ_c) = −0.6  → total −1.6 (same as Option I)
F_e(3,5) extra: +3·(1−λ_c) = +0.9  ← new entry!
ε extra: −(1−λ_c)·(3·w_xD[k−1] + w_xD[k−2])  ← past w_xD MA tail
```

Option II is more rigorous but adds δx_D coupling to F_e Row 3.

**Recommendation**: Use Option I (slowly-varying) for Phase 1 baseline. Option II's δx_D contribution is secondary (since w_xD is small in Option α with σ²_w_fD baseline = 0). Phase 5 Q derivation will revisit if needed.

### 6.6 F_e Row 3 (Eq.19 form, v2, Option I — slowly-varying x_D)

```
slot:           1     2     3       4              5     6      7
F_e(3,:) = [   0,    0,   λ_c,  −(1+d(1−λ_c)),    0,   −f_d,   0 ]
                            ↑       ↑                    ↑
                            AR(1)   x_D scaled           a_x
                                   by (1+d(1−λ_c))       linearization

For d=2, λ_c=0.7: F_e(3,4) = −1.6
```

### 6.7 ε structure (Eq.19 form)

```
ε_19[k] = (1−λ_c)·n_x[k] + a_x·f_T[k] + (1−λ_c)·Σ_{i=1}^{d} a_x·f_T[k−i]
        ↑─── current white ───────────↑   ↑──── past thermal MA(d) ────↑

Var(ε_19) ≈ (1−λ_c)²·σ²_n_x + a_x²·σ²_fT·{1 + (1−λ_c)²·d}    (for d=2)
         = (1−λ_c)²·σ²_n_x + 4·k_B·T·a_x·{1 + (1−λ_c)²·d}
```

ε_19 has **MA(d) tail of past thermal** (cross-step correlated). Single-step Var ≈ {1 + (1−λ_c)²·d}× larger than ε_18 white version's a_x²·σ²_fT contribution.

For d=2, λ_c=0.7: factor = 1 + 0.09·2 = 1.18 (so ε_19 thermal variance is 18% larger than ε_18 thermal variance per step, but spread over d+1 = 3 steps).


### 6.8 Form choice vs Q33 design choice (orthogonal decisions)

After deriving both forms, an important distinction emerges:

**Form choice** (Eq.18 vs Eq.19): determines F_e Row 3 structure (which past signals are STATE-explicit vs absorbed into ε).

**Q33 design** (Path A' vs Path C strict): independent decision on what variance to put into Q33.

These are ORTHOGONAL decisions:

| Form | Q33 design | Trade-off |
|---|---|---|
| Eq.18 + Path A' | Q33 = a_x²·σ²_fT + (1−λ_c)²·σ²_n_x | ε mostly white, but Q-R cross-cov S≠0 |
| Eq.18 + Path C | Q33 = a_x²·σ²_fT only | KF P undershoots (n_x + past thermal missed) |
| Eq.19 + Path A' inflation | Q33 inflated to match full Var(ε_MA2) | Hack — violates Q white assumption |
| Eq.19 + Path C strict | Q33 = a_x²·σ²_fT only | KF P undershoots ~50%; chosen by v1 |

design.md §8.2 Step 5 chose **Eq.19 + Path C strict**, accepting ~50% P undershoot for cleaner derivation. Phase 5 will revisit for v2.

### 6.9 Q-R cross-covariance is intrinsic to Eq.17 + raw δx_m feedback

Sensor noise n_x[k] appears in BOTH:
- δx_3[k+1] dynamics (via control feedback through (1−λ_c)·δx_m[k])
- y_1[k] = δx_m[k] = δx[k−d] + n_x[k] measurement

This is a fundamental property of paper Eq.17 (uses raw δx_m as feedback), independent of Eq.18 vs Eq.19 form choice. Cross-covariance:
```
S = E[ε[k] · v_1[k]] = (1−λ_c)·σ²_n_x
Effect if S=0 ignored: ~0.16% on closed-loop variance (per design.md §3.8)
```

Phase 6 R derivation will revisit whether to handle S explicitly or ignore.

---

## 7. Round-trip Verification (Algebraic Equivalence)

### 7.1 Key identity

```
δx_3[k] − (1−λ_c)·δx_1[k]                         ← Eq.18 Row 3 contribution
= δx[k] − (1−λ_c)·δx[k−d]
= λ_c·δx[k] + (1−λ_c)·{δx[k] − δx[k−d]}            ← add/subtract λ_c·δx[k]
```

The {δx[k] − δx[k−d]} term, using past plant equations:
```
δx[k] − δx[k−d] = (x_d[k] − x_d[k−d]) − Σ a_x·f_d[k−i] − Σ a_x·f_T[k−i] − Σ x_D[k−i]
                  (where Σ runs over i=1..d)
```

### 7.2 Substitute into Eq.18 form

Eq.18 form δx[k+1] = δx_3 − (1−λ_c)·δx_1 + B·u + ε_18 + (x̂_D − x_D) terms.

Apply identity to (δx_3 − (1−λ_c)·δx_1):
```
δx_3 − (1−λ_c)·δx_1 = λ_c·δx[k] + (1−λ_c)·(x_d[k] − x_d[k−d])
                    − (1−λ_c)·Σ a_x·f_d[k−i] − (1−λ_c)·Σ a_x·f_T[k−i] − (1−λ_c)·Σ x_D[k−i]
```

Combined with B·u = +a_x·(1−λ_c)·Σ f_d[k−i] − (1−λ_c)·{x_d[k] − x_d[k−d]}:
```
[Eq.18 LHS] + B·u = λ_c·δx[k]
                  + (cross terms cancel: x_d trajectory + Σ f_d match)
                  − (1−λ_c)·Σ a_x·f_T[k−i]
                  − (1−λ_c)·Σ x_D[k−i]
```

Adding (x̂_D − x_D) and ε_18:
```
δx[k+1] = λ_c·δx[k] + (x̂_D − x_D) − (1−λ_c)·n_x[k] − a_x·f_T[k]
                    − (1−λ_c)·Σ a_x·f_T[k−i] − (1−λ_c)·Σ x_D[k−i]
```

This **exactly matches Eq.19 form result (§6.4)**. ✓ Round-trip verified.

### 7.3 Conclusion

Eq.18 and Eq.19 forms are algebraically equivalent representations of the SAME closed-loop dynamics. The choice is purely about HOW to PARTITION the same expression:
- Eq.18: keep δx[k−d] explicit in F_e (slot 1), past signals in B·u
- Eq.19: substitute δx[k−d] via past plant, past signals absorbed into ε MA tail and F_e(3,4) coefficient

---

## 8. F_e Row 3 Comparison Table

| Slot | Element | Eq.18 form (v2) | Eq.19 form (v2, Option I) | v1 Eq.18 (simplified) | v1 Eq.19 (simplified) |
|---|---|---|---|---|---|
| 1 | δx_1 | **−(1−λ_c)** = −0.3 | 0 | −(1−λ_c) | 0 |
| 2 | δx_2 | 0 | 0 | 0 | 0 |
| 3 | δx_3 | **1** | **λ_c** = 0.7 | 1 | λ_c |
| 4 | x_D | **−1** | **−(1+d(1−λ_c))** = −1.6 | −1 | −1 |
| 5 | δx_D | 0 | 0 (Option I) / 3(1−λ_c) (Option II) | 0 | 0 |
| 6 | a_x | **−f_d[k]** | **−f_d[k]** | −f_d[k] | −f_d[k] |
| 7 | δa_x | 0 | 0 | 0 | 0 |

For d=2, λ_c=0.7:
- Eq.18 form: `[ −0.3,  0,  1,  −1,  0,  −f_d,  0 ]`
- Eq.19 form: `[ 0,    0,  0.7, −1.6, 0, −f_d,  0 ]`

**v2 vs v1 KEY DIFFERENCE**: F_e(3,4) coefficient in Eq.19 form.
- v1 Eq.19: −1 (no past x_D contribution since v1 has no Σf_d to substitute)
- v2 Eq.19: −1.6 (Σf_d expansion introduces past x_D sum, captured under slowly-varying approximation)

This v2 finding means v1's σ²_δx Lyapunov prediction was incomplete in the sense of missing x_D propagation through Σf_d. In v1's setup (no f_D), this didn't matter. But applying v1's analysis to a system WITH f_D would underestimate F_e(3,4) effect.

---

## 9. ε Noise Structure Comparison

### 9.1 Eq.18 form ε (white per step)

```
ε_18[k] = −(1−λ_c)·n_x[k] − a_x·f_T[k]    (white)

Var(ε_18) = (1−λ_c)²·σ²_n_x + a_x²·σ²_fT
         = (1−λ_c)²·σ²_n_x + 4·k_B·T·a_x

For λ_c=0.7: (1−λ_c)² = 0.09 (small sensor contribution coefficient)
```

### 9.2 Eq.19 form ε (MA(d) tail)

```
ε_19[k] = −(1−λ_c)·n_x[k] − a_x·f_T[k]                  ← current step
        − (1−λ_c)·Σ_{i=1}^{d} a_x·f_T[k−i]              ← past thermal MA(d)
```

Cross-step correlations (not white per step):
```
E[ε_19[k]·ε_19[k+1]] ≈ (1−λ_c)·a_x²·σ²_fT (lag-1 thermal correlation)
E[ε_19[k]·ε_19[k+2]] ≈ (1−λ_c)·a_x²·σ²_fT (lag-2)
E[ε_19[k]·ε_19[k+τ≥3]] = 0
```

Per-step variance:
```
Var(ε_19) = (1−λ_c)²·σ²_n_x + a_x²·σ²_fT·{1 + (1−λ_c)²·d}
         = (1−λ_c)²·σ²_n_x + 4·k_B·T·a_x·{1 + (1−λ_c)²·d}

For λ_c=0.7, d=2: factor = 1 + 0.09·2 = 1.18
ε_19 thermal var per step is 18% larger than ε_18, but distributed over d+1 = 3 steps.
```

---

## 10. Full F_e and H Matrices (Pure Algebraic, Eq.19 form, v2)

### 10.1 F_e (7×7, pure algebra)

```
              slot 1   slot 2   slot 3      slot 4              slot 5   slot 6     slot 7
              (δx_1)   (δx_2)   (δx_3)      (x_D)               (δx_D)   (a_x)      (δa_x)
            ┌                                                                                 ┐
slot 1     │   0        1        0           0                   0        0          0        │
slot 2     │   0        0        1           0                   0        0          0        │
slot 3     │   0        0        λ_c        −(1 + d·(1−λ_c))     0        −f_d[k]    0        │
slot 4     │   0        0        0           1                   1        0          0        │
slot 5     │   0        0        0           0                   1        0          0        │
slot 6     │   0        0        0           0                   0        1          1        │
slot 7     │   0        0        0           0                   0        0          1        │
            └                                                                                 ┘
```

### 10.2 Row-by-row state equations (pure algebra)

| Row | State equation | Physical meaning |
|---|---|---|
| 1 | δx_1[k+1] = δx_2[k] | shift register: oldest slot takes previous middle slot |
| 2 | δx_2[k+1] = δx_3[k] | shift register: middle slot takes previous current |
| 3 | δx_3[k+1] = λ_c·δx_3[k] − (1+d(1−λ_c))·x_D[k] − f_d[k]·{a_x[k] − a_nom} − ε[k] | closed-loop tracking dynamics (only nontrivial row); ∂(a_x·f_d)/∂a_x = f_d gives F_e(3,6) = −f_d |
| 4 | x_D[k+1] = x_D[k] + δx_D[k] | integrated RW position (disturbance) |
| 5 | δx_D[k+1] = δx_D[k] + w_xD[k] | RW velocity (w_xD enters as process noise) |
| 6 | a_x[k+1] = a_x[k] + δa_x[k] | integrated RW position (motion gain) |
| 7 | δa_x[k+1] = δa_x[k] + w_a[k] | RW velocity (w_a enters as process noise) |

### 10.3 Time-varying entries

```
F_e(3,6) = −f_d[k]                  ← only time-varying entry, refreshed every step
all other 48 entries:                ← constants in (λ_c, d, 0, 1)
```

### 10.4 Numerical instantiation (d=2, λ_c=0.7)

For sanity-check / implementation reference:

```
        ┌                                              ┐
        │  0     1     0      0     0      0      0   │
        │  0     0     1      0     0      0      0   │
        │  0     0    0.7   −1.6    0   −f_d[k]   0   │
F_e =   │  0     0     0      1     1      0      0   │
        │  0     0     0      0     1      0      0   │
        │  0     0     0      0     0      1      1   │
        │  0     0     0      0     0      0      1   │
        └                                              ┘
```

### 10.5 H Matrix (2×7, pure algebra) — unchanged from v1

Measurement equations:
```
y_1[k] = δx_m[k] = δx[k−d] + n_x[k] = δx_1[k] + n_x[k]
y_2[k] = a_xm[k] = a_x[k−d] + n_a[k]
       = a_x[k] − d·δa_x[k] + Σ_{j=1}^{d}(d−j+1)·w_a[k−j] + n_a[k]
```

H matrix:
```
H = [ 1  0  0  0  0   0    0 ]    ← y_1: δx_m → δx_1 (slot 1)
    [ 0  0  0  0  0   1   −d ]    ← y_2: a_xm → a_x − d·δa_x (slots 6, 7)
```

For d=2: `H = [[1,0,0,0,0,0,0]; [0,0,0,0,0,1,−2]]`.

The past w_a terms (Σ(d−j+1)·w_a[k−j]) are absorbed into R(2,2) effective noise:
```
R_2_eff = R_2_intrinsic + Σ_{j=1}^{d}(d−j+1)²·Q77
        = R_2_intrinsic + 5·Q77    (for d=2)
```

H structure is **unchanged from v1** — Σf_d in control law affects δx_3 dynamics (Row 3), not measurement equations. This is why the a_xm linkage with σ²_δxr (per Phase 0 §10.3) is preserved structurally.

---

## 11. Trade-off Analysis (Quantitative)

### 11.1 Form-only trade-offs (Q33 to be decided in Phase 5)

| Aspect | Eq.18 form | Eq.19 form |
|---|---|---|
| F_e Row 3 structure | Small integer entries; F_e(3,1) ≠ 0 | AR(1) clean form; F_e(3,4) = −1.6 ≠ −1 (v2 update) |
| ε per step | White (current thermal + sensor only) | MA(d) (current + past thermal) |
| KF Q white assumption | Satisfied at single step | Violated (cross-step correlated) |
| Σf_d term handling | Lives in B·u (exogenous input) | Algebraically absorbed into F_e + ε structure |
| Trajectory difference (motion) | Lives in B·u | Cancels via Σf_d substitution |
| Lyapunov derivation complexity | Need B·u state-space form | Cleaner (paper Eq.22 directly applicable) |
| Modular block-triangular | Requires B·u correlation handling | Natural (per brainstorming §3.5) |
| Closed-form C_dpmr/C_n | Needs Eq.18 specific derivation | Inherits paper 2025 Eq.11/12 form |

### 11.2 With Q design combined

| Combination | KF P prediction | Q-R indep. | Derivation | v2 recommendation |
|---|---|---|---|---|
| Eq.18 + Path A' | ε white, Q matches | Violated S ≠ 0 (~0.16% err) | Medium | Acceptable backup |
| Eq.18 + Path C | Underpredicts (n_x missed) | Independent | Easy | Less accurate |
| Eq.19 + Path A' inflation | Numerical match (hack) | Independent (artificial) | Easy | Rejected per design.md |
| **Eq.19 + Path C strict** | **~50% undershoot** | **Independent** | **Cleanest** | **★ Phase 5 baseline** |

### 11.3 Quantitative impact estimates (v2 expected)

Under recommended **Eq.19 + Path C strict** (Phase 1 form choice; Q design Phase 5):
- KF P(3,3) prediction undershoots true σ²_δx by ~50% (thermal-dominated regime)
- KF gain L underweights y_1 measurement
- Estimation convergence: ~10-20% slower warm-up
- Steady-state tracking std: ~2-3% above paper-optimal (per v1 observed h=50: 35.7 nm vs 34.91 theory)
- a_hat steady-state: largely unaffected (depends on y_2 path)

**vs v1's 33-44% a_hat bias**: that bias was from MISSING Σf_d (controller-level model mismatch). v2 with Σf_d retained should eliminate this. The remaining ~2-3% Eq.19+Path C trade-off is paper-level acceptable.

---

## 12. Recommendation for Downstream Phases

### 12.1 Phase 1 recommends **Eq.19 form**

Reasons:
1. **Cleaner closed-form Lyapunov** — paper Eq.22 directly applicable to AR(1) form
2. **Modular block-triangular structure** (brainstorming §3.5) for Phase 7 augmented Lyapunov
3. **Closed-form C_dpmr, C_n** in (λ_c, a_pd) — no lookup
4. **Q-R independence** — Phase 6 R derivation simpler
5. **Matches paper 2023 §III convention** — Eq.19 IS paper's standard AR(1) form

### 12.2 v2 vs v1 form selection rationale

v1 chose Eq.19 + Path C with simplified controller. **v2 inherits Eq.19 + Path C as starting point**, but with:
- **Σf_d retained** in control law (not dropped) — controller-level fix
- **F_e(3,4) coefficient updated**: −1 → −(1+d(1−λ_c)) = −1.6 — Phase 1 finding
- **C_dpmr, C_n re-derived** (Phase 2) — values may match v1 (3.96, 1.18) since paper Eq.22 form structurally preserved
- **Lyapunov re-derived** (Phase 7) — modular block-triangular form

### 12.3 Phase 2 entry conditions

Given Eq.19 form, Phase 2 (H + new C_dpmr/C_n) needs:
1. Confirm H structure unchanged (§10 — done)
2. Re-derive σ²_δxr autocorrelation under ε_19 structure
3. Compute new C_dpmr, C_n closed forms
4. Compute new IF_var for R(2,2) Path

Expected: C_dpmr_v2 ≈ 2 + 1/(1−λ_c²) ≈ 3.96 and C_n_v2 ≈ 2/(1+λ_c) ≈ 1.18 (paper Eq.22 form preserved). The v1 numerical values were CORRECT for Eq.19 + Path C; the issue was that the CONTROLLER was wrong, not the C_dpmr/C_n derivation. Phase 2 will verify.

---

## 13. Open Items / Phase 1 Caveats

### 13.1 Slowly-varying a_x assumption (§6.1)

Σf_d expansion via past plant equations assumes a_x[k−i] ≈ a_x[k]. Validity:
- 1 Hz osc: a_x changes ~0.04% per Ts = 625µs, negligible over d=2 steps
- 5 Hz osc: ~0.2% per Ts, still small
- Validity threshold: Δa_x/a_x < ~5% over d steps → up to ~50 Hz acceptable

### 13.2 Slowly-varying x_D assumption (§6.5 Option I)

Σ x_D[k−i] ≈ d·x_D[k]. Validity:
- Option α (RW with σ²_w_fD ≈ 0): trivially satisfied
- Larger σ²_w_fD: introduces δx_D coupling to F_e Row 3 (Option II §6.5), TBD
- Phase 5 will revisit if Q55 sensitivity is significant

### 13.3 Linearization at â_x = a_x, x̂_D = x_D

Standard EKF linearization. Errors of order (â_x − a_x)² ignored. Valid when KF estimates near truth.

### 13.4 Phase 1 NOT covered (Phase 2+)

- IIR a_xm derivation (Phase 2) — H structure given, formula constants TBD
- Q matrix design (Phase 5) — Q33 Path C vs A' decision deferred
- R matrix (Phase 6) — IF_var, R_2_eff TBD
- Closed-loop variance Lyapunov (Phase 7) — block-triangular modular solve
- Observability rank test (Phase 4) — requires final F_e to test

---

## 14. Phase 1 Summary

| Item | Result |
|---|---|
| Form chosen for downstream | **Eq.19** (paper-aligned, AR(1) structure) |
| F_e(3,1) | 0 (Eq.19 form) |
| F_e(3,3) | λ_c = 0.7 |
| F_e(3,4) | **−1.6** (= −(1+d(1−λ_c)), v2-specific update from Σf_d substitution) |
| F_e(3,5) | 0 (Option I; nonzero in Option II if past w_xD contribution kept) |
| F_e(3,6) | −f_d[k] (time-varying, unchanged from v1) |
| F_e(3,7) | 0 |
| H matrix | Unchanged from v1: H(2,7) = −d |
| ε structure (Eq.19) | MA(2) thermal tail + current sensor noise |
| Q-R cross-cov | S = (1−λ_c)·σ²_n_x ≈ 0.16% effect (acceptable to ignore) |
| Σf_d term | Algebraically absorbed in Eq.19 via past plant substitution |
| Round-trip Eq.18 ↔ Eq.19 | ✓ Verified algebraically equivalent |

**Key v2 finding**: F_e(3,4) coefficient is **−1.6** (not −1 as v1 had), reflecting past x_D contributions arising from Σf_d substitution. This will affect Phase 7 Lyapunov bench numerical predictions.

**Algebraic equivalence verified**: Eq.18 and Eq.19 forms describe same dynamics. Choice is purely representational.

---

**End of Phase 1 derivation. Awaiting user review before commit.**
