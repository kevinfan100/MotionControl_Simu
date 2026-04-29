# Eq.17 Controller Redesign — Phase 0 Design Spec (v2)

**Branch**: `test/eq17-5state-ekf`
**Pre-Phase-0 Tag**: `eq17-pre-redesign-2026-04-29` @ commit `527b89e`
**Date**: 2026-04-29
**Status**: Phase 0 fundamentals confirmed by user. Phase 1+ derivation pending.

This file (v2) supersedes `design.md` (v1) for the eq17 controller line. v1 is preserved as historical context, but its §3 "x̂_D 取代 Σf_T" framing is **rejected** — see §0 below.

---

## 0. Why v2 (Error Chain Record)

v1 (`motion_control_law_eq17_7state.m` + `design.md` §3) contains a fundamental controller-form error that propagated through all downstream Q/R/Lyapunov derivations.

### 0.1 Error 1 — OCR misread of paper Eq.17

Brainstorming (`~/Downloads/eq17_b_design_discussion_2026-04-28.md`) §3.1 transcribed paper Eq.17 as:
```
f_d[k] = (γ/Δt)·{...} − (1−λ_c)·Σ f_T[k−i]    ← misread (thermal history, unknown)
```

Actual paper Eq.17 (per user-uploaded image, prior conversation):
```
f_d[k] = (γ/Δt)·{...} − (1−λ_c)·Σ f_d[k−i]    ← control force history, KNOWN
```

The misread changed the term from "computable" (f_d, computed from past control) to "immeasurable" (f_T, thermal noise).

### 0.2 Error 2 — "drop because unknown" reasoning

§3.1 then reasoned: "f_T 不可知 所以 drop (自然進入 ε[k] 殘差)". This argument applies only to f_T, NOT f_d. f_d is computable.

§3.1 ALSO contained a separate decision: include `−x̂_D / â_x` for f_D (force-level external disturbance) compensation. This is a NEW additive feature, independent of the Σ term.

### 0.3 Error 3 — design.md §3 "替代" framing collapse

design.md §3 (v1) framed two independent decisions ("drop Σ" + "add x̂_D for f_D") as ONE replacement:

> `−x̂_D[k]` — lumped disturbance compensation (取代 paper 2023 原公式的 −(1−λ_c)·Σf_T[k−i])

This:
- Inherited the OCR error (still says f_T)
- Conflated two independent choices into one wrong "replaces" framing
- Resulted in implementation entirely dropping the Σ term with no equivalent compensation

### 0.4 Error 4 — Implementation Σ term entirely missing

`motion_control_law_eq17_7state.m` line 209-214:
```matlab
f_d = (1./â_x) · ( pd_kp1 − λ_c·pd − (1−λ_c)·pd_km_d + (1−λ_c)·δx_m − x̂_D )
                                                        ↑
                                             paper's Σf_d term entirely absent
```

### 0.5 Downstream Consequences

All observed bias is downstream of this controller-form error:

1. **Implemented controller is not paper Eq.17** — it's a paper-inspired variant with Σf_d removed
2. **All Lyapunov σ²_δx predictions in v1** (e.g., 9.55e-4) are for the simplified controller, not paper's
3. **C_dpmr=3.96, C_n=1.18** closed-form constants are from simplified ε structure
4. **F_e Row 3** in v1 §5 was derived for simplified form (no Σf_d in dynamics)
5. **a_hat 33-44% systematic bias** at KF self-equilibrium: KF correctly tracks the simplified system's a_x, but that's not the actual paper system's a_x
6. **Sensor delay buffer off-by-one** (commit 527b89e): independent error, also requires v2 fix
7. **H3 sanity test (z-axis +5475% blow-up)**: was due to placing Σf_d INSIDE 1/â_x bracket (wrong form). Paper's form has Σf_d OUTSIDE.

### 0.6 v2 Resolution

- Restore paper Eq.17 with Σf_d in correct (outside-bracket) position
- x̂_D as **separate** force-level f_D compensation (NOT a replacement)
- Stability via Strategy 1: Σf_d outside 1/â_x factor (paper's natural form)
- Re-derive F_e, H, Q, R, Lyapunov from corrected controller form

---

## 1. Plant (Form A: force-level f_D, 3 forces parallel)

### 1.1 Continuous

```
γ(h(t)) · ẋ(t) = f_d(t) + f_T(t) + f_D(t)
```

| Symbol | Definition | Units |
|---|---|---|
| x(t) | true position | µm |
| γ(h) = γ_N · c(h/R) | wall-distance-dependent drag | pN·s/µm |
| f_d(t) | desired control force | pN |
| f_T(t) | thermal Brownian force, white | pN |
| f_D(t) | external slow-varying disturbance, force-level | pN |
| γ_N | nominal (free-space) drag coefficient | pN·s/µm |
| h | wall-normal distance | µm |
| R | bead radius | µm |

### 1.2 Discrete (per axis i ∈ {x, y, z})

```
x_i[k+1] = x_i[k] + a_x,i[k] · ( f_d,i[k] + f_T,i[k] + f_D,i[k] )

a_x,i[k] = Δt / γ_i(h[k])

per axis:
  γ_x = γ_N · c_para(h_bar)
  γ_y = γ_N · c_para(h_bar)
  γ_z = γ_N · c_perp(h_bar)
  h_bar = h / R
```

### 1.3 f_T (thermal) statistics

```
Var( f_T,i per step ) = 4 · k_B · T · γ_i / Δt    (fluctuation-dissipation)
```

Per-axis γ_i uses c_para or c_perp accordingly.

### 1.4 f_D physical model — Random Walk (Option α)

```
f_D[k+1] = f_D[k] + w_fD[k]

E[w_fD[k]] = 0
E[w_fD[k] · w_fD[j]] = σ²_w_fD · δ(k − j)        (white)
E[w_fD · f_T] = 0,  E[w_fD · n_x] = 0             (independent of all other noises)
```

`σ²_w_fD` is a free parameter. Default in simulation: 0 (no disturbance baseline). Sensitivity sweeps later.

### 1.5 Sign / Notation Conventions

- `δx[k] := x_d[k] − x[k]` — tracking error (1-step calendar, matches v1 Convention A)
- `h_bar > 0` means away from wall
- Per-axis subscript i ∈ {x, y, z}

---

## 2. Wall Effect / a_x Definition

```
a_x,i[k] = Δt / (γ_N · c_i(h_bar[k]))

c_i ∈ {c_para, c_perp} per axis (Goldman-Cox-Brenner full polynomial)
```

Implemented in `model/wall_effect/calc_correction_functions.m`. Phase 4a (v1) extended with analytical derivatives `K_h, K_h'` (used in Q77 derivation).

**Inherits from v1 unchanged**.

---

## 3. Measurement Model

```
y_1[k] = δx_m[k] = δx[k − d] + n_x[k]            (raw delayed)
y_2[k] = a_xm[k]                                  (IIR-derived, paper 2025 Eq.13)
```

| Parameter | Value | Source |
|---|---|---|
| d | 2 | sensor pipeline delay (paper 2023 §III.A) |
| n_x[k] | per-axis white Gaussian, σ²_n_s,i | sensor spec (per-axis) |
| Ts = Δt | 1/1600 s | discrete sampling |

Independence: `n_x ⊥ f_T ⊥ f_D ⊥ w_fD`. Per-axis `n_x` independent draws.

### 3.1 a_xm IIR derivation (paper 2025 Eq.9-13, linear inversion)

```
δx̄_m[k+1]   = (1−a_pd) · δx̄_m[k] + a_pd · δx_m[k+1]              (LP, Eq.9)
δx_r[k]     = δx_m[k] − δx̄_m[k]                                   (HP residual)
σ̂²_δxr[k+1] = (1−a_cov) · σ̂²_δxr[k]
            + a_cov · ( δx_r[k+1]² − δ̄x_r[k+1]² )                 (cov, Eq.10)

a_xm[k] = ( σ̂²_δxr[k] − C_n · σ²_n_s ) / ( C_dpmr · 4 · k_B · T )
```

**C_dpmr, C_n closed forms re-derive in Phase 2** (v1 values 3.96 / 1.18 are for simplified controller, no longer used).

**Linkage `a_xm ← σ²_δxr` is preserved v1→v2** (see §10.3). The inversion form is independent of ε structure; only the values of C_dpmr, C_n shift due to the new ε from paper Eq.17 form. Σf_d affects these values but does NOT break the linkage form. C_dpmr, C_n are structural constants in (λ_c, a_pd, ε MA structure) — NOT functions of â_x (otherwise circular).

a_xm is a noisy measurement of `a_x[k − d]` (paper 2025 §II.F: a_xm inherits d-step delay from δx_m).

---

## 4. Control Law (★ paper Eq.17 + adaptive + f_D compensation)

### 4.1 Paper 2023 Eq.17 (true paper form)

```
f_d[k] = (γ/Δt) · { x_d[k+1] − λ_c·x_d[k] − (1−λ_c)·x_d[k−d] + (1−λ_c)·δx_m[k] }
       − (1−λ_c) · Σ_{i=1}^{d} f_d[k−i]
```

Paper assumes γ/Δt is known constant (no estimation). Σf_d is OUTSIDE the (γ/Δt) factor.

### 4.2 v2 Adaptive Form (paper Eq.17 + γ→â_x + x̂_D additive)

```
f_d[k] = (1/â_x[k]) · { x_d[k+1] − λ_c·x_d[k] − (1−λ_c)·x_d[k−d] + (1−λ_c)·δx_m[k] }
       − (1−λ_c) · Σ_{i=1}^{d} f_d[k−i]                ← paper's known compensation, KEPT
       − x̂_D[k] / â_x[k]                                ← f_D compensation, NEW (additive)
```

Three v1→v2 differences:
1. γ/Δt → 1/â_x (adaptive estimate of motion gain)
2. Σf_d term: at paper's natural location (OUTSIDE 1/â_x), structurally KEPT
3. `−x̂_D / â_x`: NEW additive term for f_D disturbance compensation, NOT replacing Σ

**Equivalent algebraic form** (Σ outside, x̂_D inside bracket):
```
f_d[k] = (1/â_x[k]) · { x_d[k+1] − λ_c·x_d[k] − (1−λ_c)·x_d[k−d] + (1−λ_c)·δx_m[k] − x̂_D[k] }
       − (1−λ_c) · Σ_{i=1}^{d} f_d[k−i]
```

(both forms produce identical f_d). Choose whichever simplifies derivation downstream.

### 4.3 Stability Strategy 1 (primary)

`(1/â_x) × Σf_d` is decoupled at the current step by paper's structure: Σ is OUTSIDE 1/â_x. No multiplicative coupling between current â_x estimate and current Σf_d term.

History-level feedback remains (past `f_d[k−i]` values depend on past `â_x[k−i]`), but this is "weak" feedback through accumulated history. Generally stable; will be empirically verified at Phase 8.

### 4.4 Stability Fallbacks (if Strategy 1 unstable in Phase 8)

| Strategy | Description |
|---|---|
| 2 | Replace past f_d[k−i] in Σ with `ideal_f_d[k−i]` computed via `a_lookup(h̄[k−i])` instead of using KF estimate |
| 3 | Anti-windup: saturate `(1−λ_c)·Σ_{i=1}^{d} f_d[k−i]` magnitude |
| 4 | Use slow-filtered â_x in Σ computation (frequency-domain decoupling) |

Layer order if needed: Strategy 1 → 4 → 2 → 3 (by invasiveness).

### 4.5 Why x̂_D / â_x (not just x̂_D)

x̂_D is a position-level state (integrated effect of f_D in position units). To convert to force-level compensation:
```
position-level disturbance to compensate: x̂_D                    [µm]
force needed per unit position via plant: 1/a_x                   [pN/µm]
multiply: x̂_D / a_x                                               [pN]
adaptive substitute: x̂_D / â_x                                    [pN]
```

Applied to plant: `a_x · (−x̂_D/â_x) ≈ −x̂_D` (when â_x ≈ a_x), exactly cancelling position-level disturbance.

---

## 5. State Vector

### 5.1 Baseline 7-state (Phase 0 starting structure)

```
x_e[k] = [ δx_1[k], δx_2[k], δx_3[k], x_D[k], δx_D[k], a_x[k], δa_x[k] ]^T
```

| Index | State | Meaning | Dynamics |
|---|---|---|---|
| 1 | δx_1[k] | δx[k−2] (oldest, sensor-visible) | shift register |
| 2 | δx_2[k] | δx[k−1] | shift register |
| 3 | δx_3[k] | δx[k] (current) | closed-loop (Phase 1 derive) |
| 4 | x_D[k] | position-level disturbance | x_D[k+1] = x_D[k] + δx_D[k] |
| 5 | δx_D[k] | x_D rate | random walk (driven by w_xD) |
| 6 | a_x[k] | motion gain | a_x[k+1] = a_x[k] + δa_x[k] |
| 7 | δa_x[k] | a_x rate | random walk (driven by w_a) |

### 5.2 x_D ↔ f_D relation

Under f_D random walk (§1.4):
```
x_D[k] := a_x[k] · f_D[k]    (definitional, position-level effect)

x_D[k+1] = a_x[k+1] · f_D[k+1]
        ≈ a_x[k] · ( f_D[k] + w_fD[k] )       (slowly-varying a_x)
        = x_D[k] + a_x[k] · w_fD[k]
```

State model uses integrated random walk (x_D + δx_D) abstraction, capturing both the f_D random walk component AND any small a_x time-variation × f_D coupling.

### 5.3 Phase 1 caveat — possible state extension

If F_e **Eq.18 form** (direct Jacobian, with Σf_d explicit in dynamics) is chosen, vector may need to extend by `d` slots to store `f_d[k−1] ... f_d[k−d]`, giving 9-state.

If F_e **Eq.19 form** (algebraic rearrangement absorbing Σ into ε) is chosen, 7-state typically retained.

Phase 1 will derive both forms, document trade-offs, and select.

### 5.4 Conventions

- `δx[k] = x_d[k] − x[k]` (tracking error)
- `e_x_i[k] := x_i[k] − x̂_i[k]` (KF estimation error, standard)
- δx_3 is NEWEST (paper convention; matches v1 design.md §4)

---

## 6. Sampling, Delay, Pole Parameters

| Parameter | Value | Notes |
|---|---|---|
| Δt = Ts | 1/1600 s | discrete sampling |
| d | 2 | sensor measurement delay |
| λ_c | 0.7 | closed-loop pole, user-selected |
| a_pd | inherit v1 | IIR LP coefficient |
| a_prd | inherit v1 | IIR cov LP coefficient |
| a_cov | 0.05 | IIR cov coefficient |

Specific a_pd, a_prd values to be locked from v1 `user_config.m` at Phase 8.

---

## 7. Numerical Environment (revisit at Phase 8)

v1 uses pure-MATLAB driver `run_pure_simulation.m` + `step_dynamics.m` (ode4 with 10 µs inner step). Phase 8 re-evaluates:
- Plant integration step size (10 µs vs Simulink 1 µs equivalence claim is UNVERIFIED — see deep audit doc)
- Sensor delay buffer alignment (Agent 3 reframe: control-then-integrate ordering structural to driver loop)
- RNG seed semantics (params build runs before `rng()` — minor reproducibility caveat)

These are downstream concerns. Phase 0 does NOT lock driver decisions.

---

## 8. Confirmed by User (2026-04-29)

| # | Item | Choice |
|---|---|---|
| 1 | Plant form | A (force-level f_D, 3 forces in plant) |
| 2 | Wall effect | Inherit v1 (Goldman-Cox-Brenner, Phase 4a verified) |
| 3 | Measurement | Inherit v1 (d=2, Ts=1/1600) |
| 4a | Control law form | Paper Eq.17 + Σf_d retained + x̂_D additive |
| 4b | Σ term content | f_d (control force history), NOT f_T |
| 4c | Stability strategy | 1 primary, 2/3/4 fallback |
| 5 | State vector | 7-state initial, Phase 1 may extend if Eq.18 form |
| 6 | f_D model | α (Random Walk) |
| 7 | Control parameters | Inherit v1 (λ_c=0.7, d=2, IIR coefficients) |

Tag reference: `eq17-pre-redesign-2026-04-29` @ `527b89e` (pre-Phase-0 baseline).

---

## 9. Phase 1-8 Plan Overview

```
Phase 0 (this doc) → Phase 1 (F_e) → Phase 2 (H + new C_dpmr/C_n)
                                          ↓
                                  Phase 3 (algebraic verify, paper-only)
                                          ↓
                                  Phase 4 (observability rank tests)
                                          ↓
                                  Phase 5 (Q open-loop)
                                          ↓
                                  Phase 6 (R closed-loop)
                                          ↓
                                  Phase 7 (Lyapunov closed-form bench)
                                          ↓
                                  Phase 8 (code rewrite + e2e + driver fix)
```

Each phase output is a markdown derivation memo, NOT code. Phase 8 is the only phase that touches code.

### Phase 1 entry — F_e Derivation (next)

Starting point:
- Plant: §1.2 (3 forces, force-level f_D)
- Control law: §4.2 (paper Eq.17 + Σf_d outside + x̂_D / â_x)
- State vector: §5.1 (7-state baseline)

Goals:
1. Substitute control law into plant
2. Linearize at â_x = a_x, x̂_D = x_D
3. Extract F_e Row 3 (only nontrivial row to derive; rows 1-2, 4-7 are structural)
4. Derive both Eq.18 (direct Jacobian, with Σ) and Eq.19 (rearranged) forms
5. Document trade-offs, select one for downstream

Deliverable: `reference/eq17_analysis/phase1_Fe_derivation.md`

---

## 10. Closed-form Methodology Principle (★ governs Phase 1-7)

User-requested principle: **prefer closed-form analytical expressions over numerical lookups**, evaluated as functions of â_x[k] where physically meaningful. v1 used `cdpmr_eff_lookup.mat` + `bias_factor_lookup.mat` for some quantities; v2 aims to replace them with closed-form.

### 10.1 Two categories of closed forms

**Category A — a_x-dependent (time-varying via â_x[k])**

Quantities that physically scale with a_x. In code, evaluate using â_x[k] every step.

| Quantity | Form (sketched, Phase to lock) | Why a_x-dependent |
|---|---|---|
| Q33 | `4·k_B·T·â_x[k]` | Var(thermal force × a_x) ∝ a_x |
| Q77 | `Δt⁴·â_x²[k]·{Term A + Term B}` | Var(d²a) chain via â_x · K_h |
| R(2,2) | `a_cov·IF_var·(â_x[k]+ξ)² + 5·Q77` | Var(IIR-derived a_xm) ∝ a_x² |
| σ²_δx (steady) | `C_dpmr·4kBT·â_x[k] + C_n·σ²_n_s` | Lyapunov, linear in a_x via thermal |
| F_e(3,6) | `−f_d[k]` | f_d itself depends on â_x via 1/â_x in control law |

**Category B — a_x-independent (structural constants, offline lock)**

Quantities depending only on (λ_c, a_pd, a_prd, a_cov, d, ε MA structure). Lock once at controller build time.

| Quantity | Form | Depends on |
|---|---|---|
| C_dpmr | function of λ_c, a_pd, ε structure | filter + pole + ε MA tail |
| C_n | function of λ_c, ε structure | pole + ε MA tail |
| IF_var | function of λ_c, ε structure | δx_r autocorrelation inflation |
| β IIR bias factor | function of λ_c, a_pd, a_prd | IIR sample-mean cancellation |
| ρ_a chi-sq chain ratio | function of λ_c, a_pd, a_cov | a_xm chi-squared autocorr |
| `delay_R2_factor` | function of d (e.g., 5 for d=2) | Σ(d-j+1)² past w_a propagation |

### 10.2 Phase 1-7 v2 commitments

| Phase | Quantity | Category | v2 vs v1 |
|---|---|---|---|
| 1 | F_e Row 3 entries | Mixed: (3,6)=−f_d (A), (3,3)=λ_c (B) | Re-derive; Eq.18/19 form choice |
| 2 | C_dpmr, C_n | B | Re-derive — **new values** from paper Eq.17 ε (v1 3.96/1.18 obsolete) |
| 5 | Q33, Q77 | A | Q33 form may absorb new ε terms from paper Σf_d expansion |
| 6 | R(1,1), R(2,2), IF_var | R(1,1)=B static, R(2,2)=A, IF_var=B re-derive | R(2,2) form likely unchanged structurally; only C_dpmr/C_n values shift |
| 7 | σ²_δx, σ²_e_xD, σ²_e_a | A (linear/quadratic in â_x) | Closed-form modular Lyapunov (block-triangular per brainstorming §3.5), no lookup |

### 10.3 a_xm ↔ σ²_δxr linkage (Phase 0 confirmed)

```
a_xm[k] = ( σ̂²_δxr[k] − C_n·σ²_n_s ) / ( C_dpmr · 4·k_B·T )
```

- **Linkage form is preserved v1→v2** (IIR processes δx_m, unaffected by control-side Σf_d)
- **σ²_δxr value DOES change** between v1 and v2 (different ε from paper Eq.17 form vs simplified)
- **C_dpmr, C_n are Category B** structural constants — NOT functions of â_x (otherwise circular: a_xm formula would depend on a_xm estimate)
- **Σf_d affects C_dpmr/C_n VALUES but not inversion FORM** — Phase 2 re-derives values

### 10.4 Why this matters for time-varying control

Brainstorming §3.7 lists 7 advantages of closed-form (vs v1 lookup) for dynamic trajectory:
- σ²_δx pointwise applicable: just plug â_x[k] into closed form — no 2-D lookup
- C_dpmr, β, ρ_a const w.r.t. a_x: motion全程 const = single offline value, no per-step lookup
- F_e structure simple: only (3,6) time-varying via control law, no estimation-quality dependence
- Lookup table count reduces to 0

This eliminates v1's `cdpmr_eff_lookup.mat` (per-axis 3.875/3.874/3.896 at h=50), replaced by single closed-form evaluation per step.

---

## Appendix A: Reference Documents at Phase 0 Tag

- `reference/eq17_analysis/design.md` (v1, **superseded** by this v2 doc)
- `reference/eq17_analysis/task01_math_observability_report.md` (v1, math observability — to re-verify in Phase 4)
- `agent_docs/eq17-architecture.md` (v1, will update post-Phase 1)
- `agent_docs/eq17-verification.md` (continues with v2 milestones)
- `~/Downloads/eq17_b_design_discussion_2026-04-28.md` (brainstorming, partial validity due to OCR error in §3.1)
- `~/Downloads/eq17_simulation_DEEP_audit_2026-04-29.md` (v1 audit)
- Tag `eq17-pre-redesign-2026-04-29` @ commit `527b89e` (pre-Phase-0 baseline)

---

## Appendix B: Notation Cross-Reference (Paper ↔ v2 Spec ↔ Code)

| Paper 2023 | v2 Spec | Code (current v1) | Meaning |
|---|---|---|---|
| x[k] | x[k] | p_m | true position |
| x_d[k] | x_d[k] | p_d | desired trajectory |
| δx[k] | δx[k] = x_d − x | del_p | tracking error |
| x_m[k] | x_m[k] | p_m_delayed | delayed measurement |
| δx_m[k] | δx_m[k] | del_pm | delayed tracking error meas |
| f_d[k] | f_d[k] | f_d / fdx | control force |
| f_T[k] | f_T[k] | F_th | thermal force |
| f_D[k] | f_D[k] | (NEW v2) | force-level slow disturbance |
| n_x[k] | n_x[k] | n_x | sensor noise |
| γ | γ = γ_N · c | gamma_N · c | drag coefficient |
| Δt | Δt = T_s | Ts | sampling period |
| a_x = Δt/γ | a_x = Δt/γ | a_axis | motion gain |
| λ_c | λ_c | lamda_c | closed-loop pole |
| (no symbol) | x_D[k] | x_D state slot 4 | position-level disturbance state |
| (no symbol) | δx_D[k] | δx_D state slot 5 | x_D rate state |
| (no symbol) | a_x state | a_x state slot 6 | motion gain state |
| (no symbol) | δa_x state | δa_x state slot 7 | a_x rate state |

State vector convention: paper convention (oldest at slot 1, current at slot 3).

---

**End of Phase 0 Spec (v2)**
