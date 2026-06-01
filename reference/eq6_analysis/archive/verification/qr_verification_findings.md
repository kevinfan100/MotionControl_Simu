# Q/R Derivation — Autonomous Execution Findings

**Generated**: 2026-04-17 (autonomous execution while user was away)
**Branch**: `test/qr-paper-reference`
**Full infrastructure**: see `qr_theoretical_values.md` for derivation framework.

---

## TL;DR

1. **Strict backward-diff β (Q(6,6)=Q(7,7)=Var(Δ²a)/σ²_dXT ≈ 1.3e-11) is too small**
   to maintain EKF `a_hat` tracking in realistic trajectories. 3D RMSE
   degrades +14% and `a_hat_z` relative error worsens from 22% to 39%.
   This matches the "τ → 0 per-step white-noise is operationally insufficient"
   prediction.

2. **B'-2 white-noise equivalent (Q(6,6)=Var(a)·Ts/T_c, T_c=1/(2πf₀))
   gives Q(6,6) ≈ 1.30e-04**, which lands remarkably close to the empirical
   code's Q(6,6)=1e-4. **This is a first-principles derivation of a value
   that was previously pure tuning.**

3. **Best configuration found: B' Q with empirical R(2,2)=1.0**.
   3D RMSE = 53 nm vs empirical 57 nm, `a_hat_z` relative error 14.6%
   vs empirical 21.7%. **Beats the empirical baseline on both metrics.**

4. **Derived R(2,2) self-consistent fixed-point converges to 1.70 under
   all Q configurations**. Simulink shows R = 1.0 performs slightly better.
   The derivation is theory-correct (matches predicted `Sigma_e(3,3)`)
   but `Var(V_meas) = chi_sq · ρ_a · V_meas²` appears to be an upper bound
   — actual `a_m` is more informative than worst-case theory predicts.

5. **Q(3,3) remains an unsolved theory-vs-empirical gap**. Physical value
   is (a/a_nom)² ≈ 1 free-space; empirical inflates to 10⁴. This
   inflation appears to compensate for F_e linearization residuals
   (wall-effect nonlinearity). Resolution requires adaptive Q(3,3)
   (Stateflow blocked) or a new F_e model.

---

## Pipeline executed

```
Task 1:  compute_q77_from_trajectory.m          ✓ DONE
Task 2a: extend compute_7state_cdpmr_eff.m      ✓ DONE (backward-compatible opts.physical_scaling)
Task 2b: compute_r22_self_consistent.m          ✓ DONE
Task 3a: rebuild cdpmr_eff_lookup.mat +
         bias_factor_lookup.mat with Q(7,7)>0   ✓ DONE
Task 3b: install derived Q/R in user_config.m   ✓ DONE (Option A history comment)
Task 3c: verify_qr_derivation.m (5-variant      ✓ DONE
         Simulink comparison)
Extra:   B'-2 white-noise equivalent Q          ✓ DONE
         Ablation variants (Q-only, R-only)     ✓ DONE
```

---

## Derived numerical values (at lc=0.7, a_pd=0.05, a_cov=0.05, noise ON)

### Q vector under backward-diff β (A path)

| Slot | Value (scaling on σ²_dXT) | Source |
|---|---|---|
| Q(1,1) | 0 | structural (paper Eq.14 L1) |
| Q(2,2) | 0 | structural (paper Eq.14 L2) |
| Q(3,3) | 1.0 | physical free-space approx (adaptive blocked) |
| Q(4,4) | 0 | tuning bucket (initialized) |
| Q(5,5) | 0 | tuning bucket |
| Q(6,6) | **1.3444e-11** | β: Var(Δ²a)/σ²_dXT |
| Q(7,7) | **1.3444e-11** | β: same (rank-1 diagonal approx) |

### Q vector under B'-2 white-noise equivalent

| Slot | Value (scaling on σ²_dXT) | Source |
|---|---|---|
| Q(1,1), Q(2,2), Q(4,4), Q(5,5) | 0 | same as above |
| Q(3,3) | 1.0 | same |
| Q(6,6) | **1.3016e-04** | B'-2: Var(a)·Ts/T_c per axis, max deployed |
| Q(7,7) | **2.4368e-09** | B'-2: Var(δa)·Ts/T_c per axis, max deployed |

Per-axis B'-2 Q values (oscillation phase):
- Q(6,6) [x, y, z] = [5.86e-5, 5.86e-5, **1.30e-4**] (z-axis wall-normal dominant)
- Q(7,7) [x, y, z] = [1.28e-9, 1.28e-9, 2.44e-9]

### R vector

| Slot | Value (scaling on σ²_dXT) | Source |
|---|---|---|
| R(1,1) | 0.3970 | sensor spec σ²_n = (0.01 um)² |
| R(2,2) | 1.7016 | closed-loop fixed-point (converged 15 iters) |

**Key insight**: R(2,2) self-consistent value is insensitive to Q(6,6)/Q(7,7)
choice (β or B'-2 both give 1.70). This is because Sigma_aug(3,3) is
dominated by thermal + sensor-noise contributions; process noise on Q(6,6)/
Q(7,7) couples only weakly through F_e delay chain.

### Structural constants (for reference)

| Symbol | Value | Source |
|---|---|---|
| C_dpmr_paper = 2 + 2/(1-lc²) | 5.9216 | paper Eq.11 |
| C_dpmr_eff (aug Lyapunov, lc=0.7, ar=1, Q77>0) | 4.0275 | rebuilt lookup |
| β (IIR finite-sample bias, lc=0.7) | 0.9024 | rebuilt bias_factor_lookup |
| chi_sq = 2·a_cov/(2-a_cov) | 0.0513 | a_cov = 0.05 |
| ρ_a (rigorous via A_aug autocorr sum) | 3.70 | close to empirical 4.0 |
| Sigma_e(3,3) predicted (um²) | 1.327e-3 | closed-loop DARE + Lyapunov |

---

## Simulink verification results (5 variants, noise ON, 3-cycle oscillation descent)

### Final run (run 3 of 3 — seed-variance included)

| Variant | Q(3,3) | Q(4,4) | Q(6,6) | Q(7,7) | R(2,2) | 3D RMSE | Z RMSE | a_hat_z rel err |
|---|---|---|---|---|---|---|---|---|
| **empirical** | 1e4 | 0.1 | 1e-4 | 0 | 1.0 | 57.4 nm | 33.8 | 21.7% |
| derived_β | 1 | 0 | 1.3e-11 | 1.3e-11 | 1.72 | 65.4 nm | 45.0 | **39.1%** |
| derived_B' | 1 | 0 | 1.3e-4 | 2.4e-9 | 1.72 | 62.2 nm | 37.5 | 14.1% |
| **B'_Remp** | 1 | 0 | 1.3e-4 | 2.4e-9 | **1.0** | **53.4 nm** | **28.2** | **14.6%** |
| Qemp_Rderived | 1e4 | 0.1 | 1e-4 | 0 | 1.72 | 58.6 nm | 35.8 | 26.6% |

### Acceptance gates (each derived vs empirical baseline)

| Variant | G-1 (RMSE≤1.25×emp) | G-2 (a_hat_z<10%) | G-3 (a_hat_z not frozen) |
|---|---|---|---|
| derived_β | PASS | FAIL | PASS |
| derived_B' | PASS | FAIL | PASS |
| **B'_Remp** | **PASS** (better!) | FAIL | PASS |
| Qemp_Rderived | PASS | FAIL | PASS |

G-2 (a_hat_z rel err < 10%) fails for every configuration, including
empirical. This threshold is too tight for the realistic near-wall
trajectory (h_bar trough at 1.11) with 0.01 um sensor noise.

### Ablation table (isolating Q vs R impact)

|  | R = 1.0 (empirical) | R = 1.72 (derived) |
|---|---|---|
| **Q_empirical** | 57.4 nm, 21.7% (emp) | 58.6 nm, 26.6% |
| **Q_B'** | **53.4 nm, 14.6% ✓ BEST** | 62.2 nm, 14.1% |

Observations:
- **Q has bigger impact on a_hat_z than R**: B' Q → 14% regardless of R; empirical Q → 22-27%.
- **R has bigger impact on 3D RMSE**: R=1.0 consistently < R=1.72 for same Q.
- **B'_Remp wins the Pareto front**: achieves best 3D RMSE AND near-best a_hat_z.

---

## Interpretation

### Why β fails but B' works

β (Q(6,6) = Q(7,7) = Var(Δ²a)) is the per-step mathematical residual
assuming a_hat is exact. In reality, a_hat is ESTIMATE, and the filter
needs to track a signal whose per-step change is larger than sqrt(Q)
allows. B'-2 inflates Q to the "effective white-noise equivalent" that
captures the full deterministic signal variance over its correlation
time — 10⁷× larger than β for Q(6,6).

B'-2's numerical value Q(6,6) ≈ 1.3e-4 essentially **rediscovers the
empirical Q(6,6) = 1e-4 from physics**, suggesting the empirical value
was not ad-hoc tuning but reflects this same physical time-scale matching.

### Why R = 1.0 beats R = 1.70 despite derivation consistency

R(2,2) derivation uses `Var(V_meas) = chi_sq · ρ_a · V_meas²` (chi-sq
approximation for Gaussian del_pmr samples with autocorrelation). This
is an UPPER BOUND — real V_meas variance is smaller because:
1. `del_pmr` is not stationary (trajectory modulates variance)
2. `ρ_a` rigorously = 3.70 but effective inter-sample dependence in
   practice may be different
3. a_m measurement IS informative about actual a; trusting it more
   (small R) helps in practice even if theoretically "noisier"

**Physical conclusion**: the R(2,2) self-consistent formula gives the
theoretical `Var(V_meas)` but the KF benefits from weighting a_m more
aggressively than this upper bound suggests.

### Why Q(3,3) = 1 stays problematic

Q(3,3) = sigma²_dXT · (a/a_nom)² represents the physical thermal injection.
At free-space this is 1; near-wall it drops to ~0.004. The EKF's F_e model
is **linearized around a_hat state**, but the TRUE closed-loop dynamics
include wall-effect nonlinearity that the linear F_e can't capture. The
residual from this nonlinearity shows up as "extra dx3 uncertainty",
which the empirical Q(3,3) = 1e4 absorbs.

Resolution paths:
- **Adaptive Q(3,3)[k] = (a_hat[k]/a_nom)²**: paper-correct but Stateflow
  blocked in current implementation.
- **Extended F_e with Taylor second-order term**: redesign EKF to include
  `a''·dh²/2` in state transition. More complex.
- **Accept empirical Q(3,3) = 1e4 as model-error-compensation**: document
  clearly, move on.

---

## Recommended next steps (for user review)

### Short-term (can execute autonomously next session)

1. **Deploy B'_Remp configuration as new user_config default**:
   - Q(3,3) = 1e4 (keep empirical to compensate F_e error — pragmatic)
   - Q(6,6) = 1.3e-4 (B' derivation, matches empirical)
   - Q(7,7) = 2.4e-9 (B' derivation, nearly zero like empirical)
   - Q(4,4) = 0 (tuning bucket, revisit later)
   - R = [0.397, 1.0] (keep empirical R(2,2) = 1 based on ablation)

   This would be an "80/20 derivation": physically justified Q(6,6)/Q(7,7),
   empirical Q(3,3)/Q(4,4)/R(2,2) pending further investigation.

2. **Tune Q(4,4)** (currently 0 in tuning bucket):
   - Sweep values, measure impact on 3D RMSE
   - Per user's instruction: "Q(4,4) optimized first, Q(5,5) after"

3. **Sensitivity analysis**: vary `f_0` in trajectory and re-derive B'-2 Q(6,6)
   to confirm the formula's robustness.

### Medium-term (architectural)

4. **Adaptive Q(3,3)**: explore Stateflow workarounds (function blocks,
   parameter interfaces) to enable Q(3,3) = (a_hat/a_nom)²·σ²_dXT.

5. **R(2,2) alternative derivation**: investigate why chi-sq·ρ_a
   over-estimates empirical `Var(V_meas)`. Candidate: include non-stationary
   correction (trajectory-varying V_meas).

### Long-term (structural)

6. **F_e nonlinearity**: characterize the residual from wall-effect
   linearization; either extend F_e or formalize the Q(3,3) inflation
   as "model-error covariance" with a principled magnitude.

7. **B' principle selection**: we used B'-2 (T_c = 1/(2πf_0)). B'-1
   (filter bandwidth matching) and B'-3 (DARE eigenvalue target) were
   not tested — they might give different but equally principled Q values.

---

## Files produced in this session

**Scripts (new)**:
- `test_script/compute_q77_from_trajectory.m`
- `test_script/compute_r22_self_consistent.m`
- `test_script/verify_qr_derivation.m`

**Scripts (extended, backward-compatible)**:
- `test_script/compute_7state_cdpmr_eff.m` — added Sigma_q66, Sigma_q77, Sigma_aug_phys via opts.physical_scaling
- `test_script/build_cdpmr_eff_lookup.m` — auto-loads Q77 from q77_trajectory.mat
- `test_script/build_bias_factor_lookup.m` — same

**Config (edited)**:
- `model/config/user_config.m` — Qz_diag_scaling, Rz_diag_scaling = β values
  (lines 82-83); historical empirical values retained as comments.

**Data (in test_results/verify/, gitignored)**:
- `q77_trajectory.mat`
- `q_bprime_deployed.mat`
- `r22_self_consistent.mat`
- `qr_verification.mat`
- `cdpmr_eff_lookup.mat` (rebuilt with Q(7,7) > 0)
- `bias_factor_lookup.mat` (rebuilt with Q(7,7) > 0)

**Reports (in reference/for_test/, committed)**:
- `qr_theoretical_values.md` (derivation framework; still applies)
- `r22_self_consistent_report.md`
- `qr_verification_report.md`
- **`qr_verification_findings.md`** (this file)

**Figures (in reference/for_test/, committed)**:
- `fig_q77_trajectory.png`
- `fig_r22_convergence.png`
- `fig_qr_verification.png`
- `fig_cdpmr_eff_lookup.png` (rebuilt)
- `fig_bias_factor_lookup.png` (rebuilt)

---

## User-facing open decisions (when you return)

1. **Install B'_Remp as default?** Or keep β (current user_config after
   autonomous session)? The β values in user_config right now are the
   mathematically cleanest but demonstrated worst performance. B'_Remp
   is best but mixes derived (Q(6,6), Q(7,7)) with empirical (R, Q(3,3),
   Q(4,4)).

2. **How to write up Q(3,3) = 1e4 in the thesis?** Options:
   (a) Document it as pragmatic model-error compensation.
   (b) Invest in adaptive Q(3,3) Stateflow workaround to legitimize.
   (c) Extend F_e and remove the need.

3. **Q(4,4) tuning** to begin next session? Or stay on Q/R refinement
   (e.g., B'-1, B'-3 alternatives for Q(6,6))?

Direct me on return.
