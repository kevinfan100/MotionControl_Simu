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
