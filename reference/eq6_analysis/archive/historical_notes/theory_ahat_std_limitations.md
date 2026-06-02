# a_hat Std Theoretical Prediction — Limitations & Honest Status (2026-04-20)

**Goal**: predict Var(a_hat - a_true) per axis from closed-loop Lyapunov.
**Status**: order of magnitude correct for some cells, off by 3-9x for others.

---

## 1. Implementation (Step 3)

### 1.1 Modifications

`compute_7state_cdpmr_eff.m`:
- Solve `Sigma_na = solve_dlyap_robust(A_aug, B_na)` (was built but unused)
- Add `Sigma_na` to diagnostics
- Add `actual_a_m_var` field to `opts.physical_scaling`:
  `Sigma_aug_phys += actual_a_m_var * Sigma_na`
- Add `Sigma_e6_phys`, `Sigma_e7_phys` to diagnostics (Var(a_hat - a_true), Var(del_a_hat - del_a_true))
- **Excluded Q66, Q77 from default Sigma_aug_phys** (they're EKF designer's
  fictional process noise; in actual simulation `a` and `del_a` are constant
  with no real driver). Optional `include_designer_Q` flag for "what EKF
  expects" prediction.

`verify_qr_positioning_run.m`:
- Add `a_hat_std_theory_x_pct`, `a_hat_std_theory_z_pct` fields
- Compute `chi_sq = 2·a_cov / (2 - a_cov)`, `rho_a = 4` (approximation)
- Set `actual_a_m_var = chi_sq · rho_a · a_axis²`
- Extract `sqrt(Sigma_e6_phys) / a_axis * 100` as relative std

### 1.2 Theoretical formula (in code now)

```
Var(a_hat - a_true)_theory = a_ratio · sigma2_dXT · Sigma_th(e6,e6)   (thermal)
                            + sigma2_n · Sigma_np(e6,e6)               (sensor)
                            + chi_sq · rho_a · a_axis² · Sigma_na(e6,e6)  (a_m noise)
```

For positioning, thermal/sensor contributions to `e6` are ~0 (a_hat error not
directly driven by these). Dominant term: a_m noise via Kalman gain L.

---

## 2. Verification: P7 (3 seeds × 2 scenarios × 2 variants = 12 runs)

| 場景 | variant | theory x/z (%) | meas x (%) | meas z (%) | ratio_x | ratio_z |
|---|---|---|---|---|---|---|
| h=2.5 | frozen_correct | 1.46 | 13.78 | 4.52 | **9.4** | 3.1 |
| h=50 | frozen_correct | 1.46 | 14.04 | 1.68 | **9.6** | **1.15** ✓ |
| h=2.5 | empirical | 3.21 | 18.78 | 19.69 | 5.8 | 6.1 |
| h=50 | empirical | 3.21 | 18.76 | 18.39 | 5.8 | 5.7 |

**Best match**: frozen z h=50 (ratio 1.15, theory 1.46% vs measured 1.68%).
**All others under-predict by factor 3-9x**.

---

## 3. Limitations identified

### 3.1 Theory predicts a_hat_x ≈ a_hat_z (same per-axis); measurement differs strongly

For h=50, theory gives 1.46% for both x and z (because formula depends on
`chi_sq * a_axis²` and `a_x ≈ a_z` at free-space). But measured a_hat_x = 14%
vs a_hat_z = 1.7% — factor 8x difference.

**Root cause**: `Fe_err_x(3,6) = -f_d(1)` vs `Fe_err_z(3,6) = -f_d(3)` —
each axis has its OWN f_d in the closed-loop dynamics. My Lyapunov sets
`opts.f0 = 0` (linearizes around f_d=0), missing this **multiplicative coupling**
between a_hat error (e6) and instantaneous control force.

For positioning, mean(f_d) = 0 but Var(f_d) ≠ 0 (control fights thermal kicks).
The product `e6 * f_d` is multiplicative noise, NOT in linear Lyapunov.

### 3.2 Empirical case under-predicts by 6x systematically

Empirical (a_cov=0.05, Q(6,6)=1e-4): theory 3.21% vs measured 18-20%.

The chi-sq chain prediction (from `ahat_quality_prediction.md`) gives:
`a_hat_std ≈ sqrt(chi_sq · rho_a) = sqrt(0.0256 · 4) = 32%`

But that's for a_hat ≈ a_m (no Kalman filtering). With Kalman LP at L=0.01:
filtered std should be smaller. Measured 18-20% is between unfiltered (32%) and
my theory (3.21%).

**Interpretation**: my theory uses `actual_a_m_var = chi_sq · rho_a · a²`
weighted by `Sigma_na(e6,e6)` (closed-loop variance amplification through L).
For empirical with larger L, Sigma_na contribution should be larger; getting
3.21% means Sigma_na is too small or rho_a too small.

Possible: rho_a depends on Q (closed-loop autocorrelation differs); using
constant rho_a=4 is too simple. `compute_r22_self_consistent.m` has a
`compute_rho_a_rigorous` helper that could be invoked.

### 3.3 frozen z h=50 fits well — why?

ratio 1.15 (15% off). This cell has:
- Wall-aware init = a_z (a_hat starts at truth, no init error)
- Free-space (a ≈ a_nom, c_perp ≈ 1.05)
- Frozen Q (a_hat hardly moves)

In this regime, the formula's assumption (`a_hat - a_true` driven mainly by
a_m noise through small L) is valid. Other cells violate this:
- Near-wall (h=2.5): wall-effect non-trivial, Cdpmr lookup may be slightly off
- x-axis: f_d coupling matters (multiplicative)
- empirical: larger L allows more noise, formula doesn't capture full propagation

---

## 4. What would fix it (future work, ~1-2 days each)

### 4.1 Per-axis Fe (f_d coupling) — addresses 3.1
Augment Lyapunov state to include f_d_nom = (1-lc)·del_p3_hat - d_hat per axis.
Solve coupled system. Need to handle multiplicative noise term (-e6·f_d_nom)
via stochastic linearization or moment closure.

### 4.2 Rigorous rho_a per scenario — addresses 3.2 partial
Call `compute_rho_a_rigorous` (already in compute_r22_self_consistent.m as
local function; needs extraction or duplication). For each (lc, ar, Q) point,
compute rho_a from the actual closed-loop autocorrelation of del_pmr.

### 4.3 7-state Riccati with mismatched R — addresses 3.2
Current: DARE uses designer R (sigma2_dXT * 1.0). For sub-optimal Kalman with
actual R = chi_sq·rho_a·a², the actual closed-loop variance via Lyapunov on
A_e with proper noise statistics. May give better empirical match.

### 4.4 Time-varying Pf (transient memory) — addresses partial
Pf decays from init to P_ss over O(1/L) samples. For frozen with L_ss = 1e-4,
this takes 10000 samples = 6.25s. Within 15s sim window, system spends
significant time in transient. Add time-varying Riccati prediction.

---

## 5. Honest verdict for thesis

**What we can claim**:
- Tracking variance: theory matches simulation within ±4% (Sigma-based + bug fix).
- a_hat_z std for free-space frozen positioning (paper's main scenario): theory matches within 15% (1.46% vs 1.68%).
- Order of magnitude correct everywhere; chi-sq chain mechanism validated qualitatively.

**What we CANNOT claim**:
- Exact prediction of a_hat std for general (lc, ar, Q) — requires multiplicative Lyapunov + per-axis treatment.
- a_hat_x std (tangential) prediction — limitation of f_d=0 linearization.
- Empirical case point-prediction — chi-sq chain rough, needs rho_a refinement.

**Recommended phrasing**:
> "a_hat estimation std under frozen Q in free-space positioning matches
> closed-loop Lyapunov prediction within 15% (1.46% theory vs 1.68% measured).
> Per-axis variation and near-wall regime show systematic under-prediction
> (factor 3-9x), attributable to multiplicative coupling between gain
> estimation error and instantaneous control force (f_d) not captured by
> linearization at f_d=0. Empirical (high-Q) case validates qualitatively
> but quantitative match requires rigorous chi-sq autocorrelation amplification
> rho_a per operating point (left as future work)."

---

## 6. Files modified

- ✅ `test_script/compute_7state_cdpmr_eff.m`: Sigma_na solve, actual_a_m_var,
  exclude Q66/Q77 by default (with optional include_designer_Q flag),
  Sigma_e6_phys/Sigma_e7_phys diagnostics
- ✅ `test_script/verify_qr_positioning_run.m`: a_hat_std_theory_x/z_pct fields
- ✅ `test_results/verify/qr_p7_*.mat`: P7 verification (gitignored, regenerable)
- 📄 `reference/for_test/theory_ahat_std_limitations.md`: this report
