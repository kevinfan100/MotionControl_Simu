# 6-state (RevisedControl_Vpersonal) Controller — Review & Empirical Validation Findings

- **Date**: 2026-06-08
- **Branch**: `feat/eq17-6state` (HEAD `69ebc9f`)
- **Scope**: code↔Vpersonal fidelity review + parameter audit + plant↔controller physics
  consistency + empirical cross-checks of the R22 / Q55 / EKF formulas.
- **Source of truth**: `RevisedConrol＿Vpersonal.pdf` (structure) + `derivation/Cdpmr_Cn_derivation.tex`,
  `derivation/R22_derivation.tex`, `derivation/Q66_OL_R22_derivation.pdf` (tin.pdf) (constants/noise).

---

## 1. Fidelity verdict (code vs Vpersonal)

**Architecture is identical to Vpersonal — element by element — with ONE deliberate exception (S=0).**

| Vpersonal element | code | match |
|---|---|---|
| State `[δx1,δx2,δx3,δx_D^d,a_x,δa_x]` (p.3) | `x_e` (6) | ✓ |
| Control law `f_d` (p.2) | active form | ✓ |
| `F_e` Row3 `[0 0 λc −1 −F_dx dF_dx]` (p.5) | `build_F_e_6state` | ✓ |
| `H = [1 0 0 0 0 0; 0 0 0 0 1 −d]` (p.3) | `H_full` | ✓ |
| `y_2 = a_xm` IIR (p.3) | `a_xm` | ✓ |
| `q3 = −ε`, `q5 = δa_ram` (p.4) | Q33 (3 terms), Q55 | ✓ |
| `r = [n_x; n_a − Σδa_ram]` (p.3) | R11, R22 | ✓ |
| predict `δx̂_3 = λc·δx̂_3` (deterministic map, p.4) | `x_pred` | ✓ |
| estimator innovations `e_y1, e_y2` (p.4) | `innov` | ✓ |
| all `Σ[k−i]` (p.1–3) | 8/8 per-step buffered | ✓ |
| plant (eq. of motion / d-delay meas / mobility / thermal, p.1) | `step_dynamics`/`calc_gamma_inv`/`calc_thermal_force` | ✓ + physics-consistent (§3.1) |
| **`−L·r` ⇒ S = E{q·rᵀ}** (p.5) | **S = 0** | ✗ deferred (Phase A) |

Q/R **values** carry the document's own approximations (A1–A5: dropped thermal history,
Taylor `a_xram`, neglected L-correction, single-sum `δx_δaf^d`, no process noise on `δx_D^d`)
plus the chosen safe-region approximations (h̄>1.5: Q-diagonal independence, open-loop gain
noise). These are documented modeling choices, not structural deviations.

---

## 2. Code fixes committed during this review

| commit | fix |
|---|---|
| `f4c7f1b` | Q55 `var(δa_ram) = 2·var(a_xram)` per Vpersonal p.2 (δa_ram = increment of a_xram); applied at the base so all 3 roles (Q55, Q33 randgain, R22 delay) pick it up |
| `fabbea6` | R22 delay term = per-step buffered `Σ var(δa_ram[k−i])` (was `d×current`); from the Σ audit |
| `8a237ec` | IF_eff = exact per-step closed form (R22_derivation S4–S6); replaces single-pole approx + drops 7-state `IF_eff_calibrated` override |
| `69ebc9f` | verify y sensor std `0.057 → 0.57 nm` typo |

C_dpmr/C_n docstring also corrected (`3.93/1.15 → 3.16/1.11`, MCP-verified against the .tex).

---

## 3. Empirical validations (MCP, h50 + h10 positioning unless noted)

### 3.1 Plant ↔ controller physics consistency

| # | check | result |
|---|---|---|
| #1 | thermal per-step position-increment variance = `4kBT·a` | ratio emp/theory `[0.998 1.003 0.987]` ✓ (single-sided 4kBT both sides) |
| #2 | mobility `diag(Γ⁻¹)·Ts = a_nom/c` | ratio `1.000000` ✓ (plant & controller share `calc_correction_functions`) |
| #3 | sensor delay (driver buffer `d+1=3`) | true **2-sample** delay (the +1 compensates end-of-step `p_true` indexing) ✓ |
| #5 | RNG | same-seed reproducible (diff=0); seeds independent; thermal⊥meas streams ✓ |

### 3.2 Formula cross-checks (5-seed)

| check | formula | h50 ratio (emp/formula) | h10 ratio | verdict |
|---|---|---|---|---|
| **R22 = Var(a_xm)** | `K_var·IF_eff·(â+ξ)²` | `[1.042 1.018 0.993]` | `[1.049 1.016 0.995]` | ✓ blue a_xm var = formula (<5%) |
| **IF_eff (V4 ACF)** | exact closed form | `[1.018 1.000 1.015]` | `[1.013 1.000 1.006]` | ✓ ~1% — **b2 fixed the old Phase-9 V4 ~10% over-prediction** |
| **A: C_dpmr/C_n** | `Var(dx_r)=C_dpmr·4kBT·â+C_n·σ²_nx` | `[1.007 1.019 1.010]` | `[1.012 1.019 1.016]` | ✓ constants validated directly |
| **C: a_xm unbiased** | `E[a_xm]=a_true` | `[1.013 1.004 1.023]` | `[1.011 1.004 1.019]` | ✓ inversion validated |
| **D: Q55** | `var(δa_ram)=2·(â·K_h/R)²·4kBT·a_perp` | `[0.58 0.61 0.58]` | `[0.59 0.61 0.59]` | ⚠ formula over-estimates ~1.7× (see §3.3) |
| **E: KF P(5,5)** | `Var(â−a_true)=P_a` | `[8.9 18.6 14.1]` | `[21 36 22]` | ⚠⚠ EKF over-confident 10–30× (see §3.3) |

### 3.3 The two real gaps (D, E)

**D — Q55 gain process noise (quantifies B4/B5):**
- **level**: empirical `Var(a_xram) ≈ 4× formula` → the gain-fluctuation level is ~4× the
  open-loop `σ²_δh`, because `h_true` is **closed-loop regulated** (variance ≈ C_dpm·σ²_δh),
  not free open-loop thermal motion (**B5**).
- **increment/level ratio = 0.29**, NOT 2 → the gain is **highly autocorrelated** (ρ₁≈0.86),
  i.e. NOT i.i.d.; the committed `2×` (i.i.d. increment) is physically too large (**B4**).
- **net**: `4 × (0.29/2) = 0.58` → the committed Q55 formula **over-estimates by ~1.7×**.
- **Impact**: NONE on h50/h10/ramp (K_h tiny → Q55≈0). A **near-wall (Phase A)** refinement:
  the correct near-wall Q55 should use the closed-loop level × autocorrelation, not open-loop×2.

**E — EKF over-confidence on a_x:**
- The filter's `P(5,5)` under-estimates the actual `Var(â − a_true)` by **10–30×** → the EKF
  thinks `â` is 3–6× more accurate (in std) than it is.
- Cause: the `â ↔ control ↔ a_xm` **closed-loop feedback** (self-consistent equilibrium)
  amplifies `â` variance beyond what the linear KF covariance models.
- **Impact**: does NOT break tracking or `â` unbiasedness (both validated); but `P_a` must not
  be trusted as the true `â` uncertainty. KF-consistency / Phase-A item.

---

## 4. Corrections to earlier notes

- **c_perp(h̄=4.44) ≈ 1.33** (NOT 4.87), so **a_z(h10) ≈ 1.1e-2** (NOT 1.28e-5). An earlier
  exploration agent miscalculated `c_perp` ~3.7×; this propagated into plan/memory. Confirmed
  by the sim: `a_hat_z(h10)=1.107e-2` matches ground truth to 0.7%.
- Therefore h10 is **not** "dramatically near-wall"; h̄=4.44 is moderate, all three axes have
  gains ~1.1–1.3e-2. The z-axis appears "harder" (higher a_hat rel-std, smaller r) **because of
  its large sensor noise (3.31 nm, ~30× the x/y variance)**, not because a_z is small.

---

## 5. Phase A (near-wall) backlog

Items that are negligible for h̄>1.5 but bite near the wall / in fast dynamics:

- **B1** Q-diagonal — add `Cov(δx_T^d, δx_δaf^d)` (∝K_h) off-diagonal.
- **B2** S = E{q·rᵀ} = 0 — the `−L·r` cross-covariance (n_x in q3+r_1, δa_ram in q3/q5+r_2);
  correlated-noise predict (Simon §7.1). S ∝ K_h/(1−λc).
- **B3** IF_eff time-variance — ✅ **DONE** (exact per-step closed form, validated §3.2).
- **B4 + B5** Q55 — now quantified (§3.3): use closed-loop level (≈4×) × autocorrelation
  (incr/level≈0.29), not open-loop × 2; net ~0.58× the current value near these operating points.
- **E** EKF P(5,5) over-confidence — needs closed-loop-feedback-aware covariance (or accept P as
  a lower bound).
- **LB-1** `1/a_hat` has no divide-by-zero / negative guard — add a positive floor before any
  near-wall use (a_hat can blow up there).

---

## 6. Bottom line

- Controller + simulation are **faithful to Vpersonal** (structure exact except S=0; plant
  physics consistent).
- **R22 derivation chain empirically validated** end-to-end (C_dpmr/C_n → σ²_δpmr → IF_eff →
  Var(a_xm)) at two operating points, all ratios ≈1.
- **b2 IF_eff** independently confirmed correct (fixes the historical Phase-9 V4 ~10% gap).
- Two real, quantified gaps (Q55 over-estimate 1.7×; EKF P over-confidence 10–30×) — both
  benign for the validated safe-region scenarios, both Phase-A / near-wall concerns.
  (Q55 now resolved with a closed form — see §8.)

---

## 7. Adversarial fidelity audit + sim-result back-verification (2026-06-08 cont.)

### 7.1 Block-by-block adversarial fidelity audit (code ↔ Vpersonal PDF)

7 derivation blocks, each independently re-derived from the PDF and tested to *refute* the
code. **Zero hard discrepancies.** Verdicts:

| block | verdict | key |
|---|---|---|
| control law (p.2) | faithful | telescoping `pd[k+1]−λc·pd−(1−λc)pd[k−d] = Δx_d^d` proved (d=1,2); past forces gain-weighted by â[k−i]; all terms inside 1/â |
| F_e (p.5) | faithful | every entry+sign: (3,4)=−1, (3,5)=−F_dx, (3,6)=**+**dF_dx, (5,6)=+1; F_2 is i-weighted |
| H + a_xm (p.3) | faithful* | H=[1 0 0 0 0 0; 0 0 0 0 1 −d] element-exact; a_xm magnitude = SSOT-split |
| predict-update (p.4) | faithful | deterministic-map x_pred = PDF estimator δx̂_3=λc·δx̂_3 slot-for-slot; P_pred uses full F_e; innov = PDF e_y1/e_y2 |
| Q (p.3-4) | faithful* | ε 3-component + (d+1−i)²={4,1} weights; Q44=Q66=0; var(δa_ram) factor → §8 |
| R (p.3) | faithful* | R11=σ²_nx; R22 delay = buffered Σ (coeff 1); σ²_na magnitude = SSOT-split |
| S (p.5) | **S=0 = the only structural deviation** | Joseph keeps L·R·Lᵀ → stable, sub-optimal |

The deterministic-map predict is **not** a deviation — the PDF estimator (p.4) itself writes
δx̂_3[k+1]=λc·δx̂_3[k] (the F_dx·e_ax / dF_dx·e_δax / ε terms are zero-mean estimation errors
that enter only P_pred via the full F_e).

### 7.2 Sim-result back-verification (V1–V5, MCP, h50+h10 5-seed)

| # | check | formula | result |
|---|---|---|---|
| **V1** | closed-loop tracking variance | `Var(δx_true) = C_δx·4kBT·a + (1−λc)/(1+λc)·σ²_nx`, `C_δx = 2+1/(1−λc²)=3.96` | ratio [1.01 0.99 1.03] ✓ (end-to-end capstone; **C_δx=3.96, NOT C_dpmr=3.16 — the difference is the a_xm IIR high-pass**) |
| **V2** | KF innovation whiteness | y_2 (a_xm) innovation ρ(1)≈**0.99** strongly colored | measurement-level root of finding E: KF treats correlated a_xm as white → P over-confident |
| **V3** | EWMA estimator unbiased (mean) | `E[σ̂²_dxr] = C_dpmr·4kBT·â + C_n·σ²_nx` | ratio [1.01 0.99 1.02] ✓ |
| **V4** | delay-chain + disturbance | corr(δx̂_1, δx_m)=1.00; \|x̂_D\|~0.1 nm | healthy ✓ |
| **V5** | plant/setup sanity | f_d zero-mean; p_d constant; guards 0% fired; no startup transient | all clean ✓ |

`ρ_δx(1)=0.85 ≠ λc=0.7` is expected — the signature of the colored MA(2) ε (the same reason
C_δx=3.96 not the white-noise 1/(1−λc²)=1.96). V1's σ_dx_true [31.4 31.0 31.2] nm matches the
formula at both operating points.

---

## 8. Q55 closed form + remaining differences (post-fix)

### 8.1 The Q55 = Var(δa_ram) closed form (derivation)

`δa_ram[k] = a_x^ram[k+1] − a_x^ram[k]` is the one-step increment of the random gain
fluctuation `a_x^ram = (a·K_h/R)·x_ram`, where `x_ram` = wall-normal random position deviation
(= the closed-loop tracking error δx in the h-direction). Three layers:

```
(A) chain rule : Var(a_x^ram) = (a·K_h/R)² · Var(x_ram)
(B) level      : Var(x_ram)   = C_δx · σ²_δh ,  C_δx = 2+1/(1−λc²)   (closed-loop accumulation, NOT single kick)
(C) increment  : Var(δa_ram)  = 2(1−ρ₁) · Var(a_x^ram) ,  ρ₁ = lag-1 autocorr of a_x^ram (=ρ_δx(1))
```

ρ₁ in closed form (from δx[k+1]=λc·δx[k]−ε[k], ε the MA(2) thermal tail):

```
R_ε(0)=g[1+2(1−λc)²], R_ε(1)=g(1−λc)(2−λc), R_ε(2)=g(1−λc),  g=4kBT·a=σ²_δh
R(0)=Var(δx)=[R_ε(0)+2λc R_ε(1)+2λc² R_ε(2)]/(1−λc²) = C_δx·g ,  numerator = 3−2λc²
ρ₁ = λc + 2(1−λc)/C_δx   →   1−ρ₁ = 1/[(1+λc)·C_δx]
```

The **collapse** — C_δx (amplification) and (1−ρ₁) (smoothing) cancel:

```
Q55 = 2(1−ρ₁)·C_δx·(a·K_h/R)²·σ²_δh = [2/(1+λc)] · (a·K_h/R)² · σ²_δh
```

So the entire correction is **factor 2 → 2/(1+λc)** (=1.176 at λc=0.7). The old i.i.d. `2×`
assumed ρ₁=0 (a_x^ram white) AND open-loop level; net over-estimate = (1+λc) = 1.7×.

**Direct validation** (measured true gain increment Var(δa_ram) from p_true, h50+h10 5-seed):
ρ₁ emp 0.855 (closed-form 0.8515); Var(a_ram)/[C_δx·level] = 1.028; **Var(δa_ram) emp / closed
= 0.998–1.001**; emp / old-2× = 0.587 (= 1/(1+λc)). Closed form is exact to <0.2%.

### 8.2 Code change (committed with this doc)

`build_eq17_6state_constants.m`: new `var_da_increment_factor = 2/(1+λc)`.
`motion_control_law_eq17_6state.m`: `var_da_ram` base factor `2 → var_da_inc_factor` (init +
per-step). **Q55, Q33_randgain, and the R22 delay term all read the same `var_da_ram` base** →
one fix corrects all three. 3-trajectory regression: h50 **PASS unchanged** (trk [31.4 31.0 31.3]
nm, bias [+0.8 −1.1 +0.8]%); ramp z rel-err 4.9→4.7% (descending end, where K_h grows). Zero
safe-region regression (K_h≈0 → var_da_ram≈0 regardless of factor). Closed form is rigorous in
the safe region (thermal-dominated, quasi-static h); near-wall re-validation of C_δx/ρ₁ is a
Phase-A item.

### 8.3 Remaining differences vs Vpersonal (after the Q55 fix)

The noise-**value** approximation family (Q55 / Q33_randgain / R22 delay) is now closed-form.
What remains is the noise-**correlation-structure** family — all the same physical origin
(unmodeled q-r / temporal correlation), all near-wall / closed-loop, all Phase A:

| item | dropped correlation | bites | type |
|---|---|---|---|
| **S=0** (only structural deviation) | S(3,1)[n_x q3↔r1], S(3,2)/S(5,2)[δa_ram q3/q5↔r2] | ∝(1−λc),K_h → near-wall | needs correlated-noise predictor (Simon §7.1), structural add |
| **diagonal Q drops Cov(q₃,q₅)** | ε contains past δa_ram; q₅=current δa_ram (autocorrelated) | ∝K_h → near-wall | sibling of S |
| **E / V2: EKF P over-confidence** | a_xm innovation ρ≈0.99 treated white → P(5,5) under-states â error 10–30× | closed-loop everywhere (does NOT break tracking/unbiasedness) | needs colored-measurement KF; same root |

Notes (not "errors"): **SSOT-split** — a_xm magnitude (C_dpmr/C_n/IF_eff/ξ/σ²_na) is from
supporting docs (paper2025/Cdpmr_Cn/R22), not Vpersonal (empirically verified §3.2). **Q33
thermal uses â** (true a unobservable). **Q55 closed-form near-wall caveat** (§8.2).
**Add-ons** (warmup gate, 3-guard, prefill/Riccati init) preserve PDF structure (Task 03/04).
**LB-1** `1/â` has no clamp (near-wall). **Zero code bugs** — every remaining difference is the
S/correlation family, an SSOT-split provenance note, or a benign design choice.
