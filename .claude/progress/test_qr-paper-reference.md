# Q/R Theoretization — Progress Record

## 2026-04-18 (Session 6) — frozen_correct steady-state reveal: bias +6.4% → +1.4%

### Completed Parts
- ✅ Diagnostic trace of a_hat_z(t) in frozen_correct h=50 revealed IIR gate transient spike at k=1000
- ✅ Root cause identified: var_threshold gate in motion_control_law_7state.m opens suddenly when del_pmr_var crosses threshold (slower with small a_cov)
- ✅ Session 5 +6.4% bias at h=50 confirmed as measurement window artifact (not steady-state failure)
- ✅ Re-ran with T_sim=30s, t_warmup=10s (P2 standard) to capture true steady state
- ✅ frozen_correct h=50: bias 1.4 ± 0.6%, std 1.3 ± 0.7% (was +6.4%/+4.0%)
- ✅ Wall-aware init confirmed already in code (motion_control_law_7state.m lines 88-99, commit 9a0e8db)

### File Changes
**New Files:**
- `reference/for_test/session6_frozen_steady_state.md` — Session 6 full findings
- `test_results/verify/diag_frozen_correct_h50.mat` — diagnostic trace

**Modified Files:**
- `test_script/verify_qr_positioning_run.m`:
  - `T_sim`: 15 → 30 (allow transient decay)
  - `t_warmup`: 5 → 10 (P2 standard, also clear of transient)

### Key Results (Session 6, 12 runs with 3 seeds each)

| scenario | variant | 3D RMSE (nm) | bias (%) | std (%) |
|---|---|---|---|---|
| h=2.5 | empirical | 34.9 ± 0.2 | +1.8 ± 1.2 | 20.9 ± 1.0 |
| h=2.5 | frozen_correct | 35.7 ± 0.1 | **+1.4 ± 0.4** | **4.6 ± 0.1** |
| h=50 | empirical | 60.5 ± 0.3 | +0.1 ± 1.2 | 19.9 ± 1.3 |
| h=50 | frozen_correct | 61.8 ± 0.2 | **+1.4 ± 0.6** | **1.3 ± 0.7** |

**frozen_correct is the optimal positioning Q/R**:
- a_hat_z std: 1-5% (below paper target 5-10%)
- a_hat_z bias: ~1.4% (essentially 0)
- 3D RMSE: within ±2 nm of empirical

### Testing Status
✅ Optimal positioning Q/R identified and validated under true steady state
- 4-parallel batches of 3 runs each, T_sim=30s, ~15 min wall time
- Steady-state window 20s captures post-transient true behavior
- Seed variability confirmed low (std of means < 1 nm 3D RMSE, < 2% bias)

### Next Steps
- [ ] Commit Session 6 deliverables + push
- [ ] Option (root cause): implement pre-fill IIR V_meas to eliminate gate transient (shorter warmup possible)
- [ ] Dynamic scenario test with frozen_correct to characterize breaking point
- [ ] Thesis writeup preparation

### Issues & Notes

⚠️ **Session 5 "+6.4% bias" conclusion was wrong root-cause** (I thought init mismatch, but init is wall-aware-correct since commit 9a0e8db). The bias was entirely from IIR gate transient not yet decayed within T_sim=15s window.

⚠️ **Wall-aware init already in code**: lines 88-99 of motion_control_law_7state.m compute h_bar from p0 + wall, then set a_hat = a_nom/c_perp/c_para accordingly. My "implement element 1" proposal was redundant.

💡 **True performance of frozen_correct**:
- In steady state, a_hat_z std drops from empirical's 20% to **1.3% (h=50)** or **4.6% (h=2.5)**.
- Bias stays at ~1.4% (was 0 for empirical but 1.4% is negligible and consistent).
- 3D tracking RMSE unchanged (Q/R-insensitive, confirmed again).

💡 **"Optimal Q/R for positioning" is clearly frozen_correct**:
```matlab
Qz_diag_scaling = [0; 0; 1; 0; 0; 1e-8; 1e-8];
Rz_diag_scaling = [0.397; 1.0];
Pf_init_diag    = [0; 0; 1e-4; 1e-4; 0; 1e-5; 0];
a_cov           = 0.005;
```

### Git Commit
(to be filled after commit)

---

## 2026-04-18 (Session 5) — a_hat quality discovery: 19% → 4%

### Completed Parts
- ✅ Theory chain validation (Task 1d Appendix A): empirical std 19% matches prediction exactly
- ✅ a_cov sweep: 0.05 → 0.005 reduces a_hat std 1.6× (from chi-sq formula)
- ✅ frozen mechanism explained and tested (Q ≈ 0 + small Pf → P decays → frozen)
- ✅ frozen_correct variant: a_hat std drops to **4.4%** (paper level achieved)
- ✅ frozen_smartPf variant tested (allow snap then freeze): unreliable due to chi-sq snap noise
- ✅ Per-variant Pf_init and a_cov support added to verify_qr_positioning_run.m
- ✅ 24 runs (4 variants × 2 sc × 3 seeds), all completed successfully
- ✅ ahat_quality_prediction.md (pre-sim prediction)
- ✅ session5_ahat_quality_findings.md (full findings + optimal route design)
- ⏸️ Optimal route (frozen_correct_init with wall-aware init) — needs EKF code modification, not yet tested

### File Changes
**New Files:**
- `reference/for_test/ahat_quality_prediction.md` — pre-simulation theoretical predictions for 3 variants
- `reference/for_test/session5_ahat_quality_findings.md` — comprehensive findings, optimal route design

**Modified Files:**
- `test_script/verify_qr_positioning_run.m` — added Pf and a_cov fields per variant, added 4 variant configs

### Key Results

**Theory validated for empirical**:
- Predicted a_hat_z std at lc=0.7, a_cov=0.05: 19%
- Measured: 18.0-20.1% across 12 runs
- Match within ±5%

**a_hat std reductions**:
| variant | a_hat_z std (h=2.5) | a_hat_z std (h=50) | reduction vs empirical |
|---|---|---|---|
| empirical | 20.1 ± 1.5% | 18.0 ± 0.4% | baseline |
| emp_acov005 | 12.6 ± 0.8% | 11.1 ± 1.8% | 1.6× |
| frozen_correct | **4.4 ± 0.2%** | **4.0 ± 0.4%** | **4-5×** |
| frozen_smartPf | 4.5 ± 0.2% | 4.4 ± 5.2% (high seed variance) | similar mean |

**Bias issue at h=50**:
- frozen_correct: +6.4% bias (init mismatch a_nom vs a_nom/c_perp(22.2))
- frozen_smartPf: +9.3% bias on average, but with seed 11111 outlier (+24%)
- Solution: wall-aware init (proposed for next session)

### Testing Status
✅ 24 positioning runs (4 variants × 2 sc × 3 seeds) all completed
- 3-parallel batches of 6 each, ~10 min wall time
- Incremental checkpoint per run
- Theory chain quantitatively validated for empirical baseline

### Next Steps
- [ ] **Optimal route**: Implement wall-aware init in motion_control_law_7state.m
- [ ] Test `frozen_correct_init` variant (predict bias < 1% at both scenarios, std ~4%)
- [ ] Refine theory for emp_acov005 prediction (account for EKF smoothing factor adaptation at low a_cov)
- [ ] Move to dynamic scenario testing for the optimal positioning Q/R

### Issues & Notes

⚠️ **emp_acov005 prediction inaccuracy**: predicted std 6%, measured 11-13%. EKF smoothing factor adapts when a_m gets cleaner — this needs to be incorporated into the theory.

⚠️ **frozen_smartPf snap unreliability**: 1 out of 3 seeds at h=50 had +24% bias due to chi-sq outlier in first measurement. Pf_init=1e-3 gives L=0.8 first update — too aggressive for noisy initial measurements.

💡 **a_hat std 19% is NOT a hard floor** — it's the chi-sq prediction at a_cov=0.05. Reducing a_cov directly reduces std.

💡 **Frozen mechanism is the biggest a_hat std lever**:
- 4-5× reduction (20% → 4%)
- Achieves paper-equivalent precision (5-10% target)
- But requires correct init (because frozen can't correct) → needs wall-aware init for h-far scenarios

💡 **3D RMSE Q/R-independent confirmed across 4 variants and 2 scenarios** — Q/R cannot improve tracking, only a_hat quality.

### Git Commit
`87e0295` - feat(test/qr): a_hat quality breakthrough - 20% to 4% via frozen + a_cov tuning

---

## 2026-04-18 (Session 4) — Positioning verification + baseline alignment

### Completed Parts
- ✅ Project verification convention surveyed (verify_p2_static_h25.m as template)
- ✅ verify_qr_positioning_run.m worker function (parameterizable subset, incremental checkpoint)
- ✅ verify_qr_positioning_aggregate.m combine batches into reports
- ✅ 5-variant × 2-scenario × 3-seed positioning matrix (27 runs successful, 1 crash, 2 missing due to batch crash)
- ✅ B' near-wall divergence root cause identified (particle hit wall, h_bar→1.0)
- ✅ β frozen behavior confirmed (a_hat std 5% h25, 2% h50)
- ✅ **Sensor noise spec correction**: identified that earlier qr verify used [0.01, 0.01, 0.01] um but main project uses [0.00062, 0.000057, 0.00331] um (z 3x smaller)
- ✅ 12-run focused empirical+beta comparison with corrected noise spec
- ✅ Empirical baseline NOW matches main project Task P2 numbers
- ✅ qr_positioning_summary.md final report

### File Changes
**New Files:**
- `test_script/verify_qr_positioning_run.m` (~225 lines)
  Worker function for positioning verification matrix. Reads idx range, hardcoded scenario+variant+seed lists. Incremental .mat checkpoint per run.
- `test_script/verify_qr_positioning_aggregate.m` (~280 lines)
  Loads all qr_pos_b*.mat batch outputs, aggregates per (scenario, variant) across seeds, generates per-scenario reports + figure.
- `reference/for_test/qr_positioning_summary.md` (latest, with corrected noise spec)
- `reference/for_test/qr_positioning_near_wall_h25_report.md`
- `reference/for_test/qr_positioning_free_space_h50_report.md`
- `reference/for_test/fig_qr_positioning_summary.png` (5-variant, older data)
- `reference/for_test/fig_baseline_empirical_zoom.png` (empirical EKF spike trajectory diagnostic)

### Key Findings (corrected noise spec)

**Empirical matches main project baseline**:
- h=2.5: 3D RMSE 35.0±0.3 nm, a_hat_z bias +1.6%, std 20.1%  (vs P2: -0.03%, 19.66%)
- h=50: 3D RMSE 60.6±0.5 nm, a_hat_z bias -0.2%, std 18.7%  (vs P2: +1.35%, 19.49%)

**β vs empirical (positioning)**:
- 3D RMSE: essentially identical (35 vs 36 nm at h=2.5; 60 vs 62 nm at h=50)
- a_hat_z std: β is 4-5x smaller (FROZEN) — useless for dynamic
- a_hat_z bias: β tends biased high at h=50 (+7%)

**Earlier (wrong noise spec) findings still valid in pattern**:
- B' has near-wall crash risk (particle hits wall, 1/3 seeds)
- All variants give similar 3D RMSE
- Only differentiation is in a_hat estimator behavior

### Testing Status
✅ Positioning verification framework solid
- Sensor noise spec aligned to main project hardware values
- 12-run baseline + comparison cycle ~10 min wall time (2-parallel)
- Project P2 baseline reproduced to <1% (mean) and <5% (std)

### Next Steps (user-defined direction)
- [ ] **a_hat estimation quality** is the user's stated focus
- [ ] Need to make derivation values "match" simulation (currently empirical works, B' theoretical crashes)
- [ ] Possible directions:
  - Q(6,6)/R(2,2) sweep to find a_hat std vs frozen sweet spot
  - Pf_init(6,6) reduction to lower spike
  - lc=0.4 + lc=0.7 cross-comparison aligned with main project baseline table
  - Theory-vs-simulation match for a_hat std specifically

### Issues & Notes

⚠️ **Critical setup error caught**: Earlier verify_qr_derivation.m used wrong meas_noise_std. All numerical comparisons in qr_verification_findings.md were affected. The PATTERN findings (B' divergence, frozen β, similar RMSE) are still valid because both variants used same wrong noise — the relative comparison holds.

⚠️ **Old qr_positioning_combined.mat (27-run) is from wrong noise spec era**. Headline numbers there are off by ~5-50% depending on scenario.

💡 **Empirical baseline vs theory (lc=0.7, h=50)**:
- a_hat_z bias ~0% (good — no systematic offset)
- a_hat_z std 19% — this is the chi-squared chain prediction (Task 1d derivation)
- Reproduced exactly, validating both code AND theory

💡 **The a_hat std 19% is INTRINSIC** to the chi-squared statistics of the IIR estimator (Task 1d Appendix A). Q/R cannot push it lower without freezing β-style. The "right" a_hat std is bounded below by physics.

### Git Commit
`8f9e924` - feat(test/qr): positioning verification framework + baseline alignment fix

---

## 2026-04-17 (Session 3) — Full Pipeline Autonomous Execution

### Completed Parts
- ✅ Task 1: compute_q77_from_trajectory.m (per-axis + gates G1/G2/G3 PASS)
- ✅ Task 2a: extended compute_7state_cdpmr_eff.m with backward-compat opts.physical_scaling (B_q66/B_q77/Sigma_aug_phys)
- ✅ Task 2b: compute_r22_self_consistent.m fixed-point solver (converged 15 iters)
- ✅ Task 3a: rebuilt cdpmr_eff_lookup.mat + bias_factor_lookup.mat with Q(7,7)>0
- ✅ Task 3b: user_config.m installed derived Q/R (β values, history comment preserved)
- ✅ Task 3c: verify_qr_derivation.m 5-variant Simulink comparison
- ✅ B'-2 white-noise equivalent Q derivation (bonus — time-scale self-consistent principle)
- ✅ Ablation variants (B' Q + empirical R, empirical Q + derived R) isolate Q vs R impact
- ✅ qr_theoretical_values.md Section 9 updated with concrete numerical values
- ✅ qr_verification_findings.md comprehensive session summary

### File Changes
**New Files:**
- `test_script/compute_q77_from_trajectory.m` (~280 lines)
  Purpose: Compute Q(6,6), Q(7,7) = Var(Δ²a) per-axis along reference trajectory.
- `test_script/compute_r22_self_consistent.m` (~330 lines)
  Purpose: R(2,2) fixed-point solver with rigorous rho_a via augmented Lyapunov autocorrelation.
- `test_script/verify_qr_derivation.m` (~450 lines)
  Purpose: 5-variant Simulink comparison (empirical / β / B' / B'_Remp / Qemp_Rderived).
- `reference/for_test/qr_verification_findings.md`
  Purpose: Full autonomous session writeup with tables, interpretation, next steps.
- `reference/for_test/qr_verification_report.md`
  Purpose: Auto-generated Simulink verification output.
- `reference/for_test/r22_self_consistent_report.md`
  Purpose: R(2,2) fixed-point iteration audit trail.
- `reference/for_test/fig_q77_trajectory.png`
- `reference/for_test/fig_qr_verification.png`
- `reference/for_test/fig_r22_convergence.png`

**Modified Files:**
- `model/config/user_config.m` (+~20 lines)
  Installed derived β Q/R as default, retained empirical as commented historical block.
- `test_script/compute_7state_cdpmr_eff.m` (+~50 lines)
  Added B_q66/B_q77 drivers, Sigma_q66/Sigma_q77 solves, opts.physical_scaling path for physical Sigma_aug.
- `test_script/build_cdpmr_eff_lookup.m` (+~15 lines)
  Auto-load Q77_scaling from q77_trajectory.mat, inject into Q_kf_scale slots 6/7.
- `test_script/build_bias_factor_lookup.m` (+~12 lines)
  Same auto-load pattern; Q_kf_scale updated to derived β form.
- `reference/for_test/qr_theoretical_values.md` (+~60 lines)
  Section 9 rewrite: three interpretations (forward, β, B'-2), concrete numerical values, Simulink summary.
- `reference/for_test/fig_cdpmr_eff_lookup.png` (regenerated with Q(7,7)>0)
- `reference/for_test/fig_bias_factor_lookup.png` (regenerated with Q(7,7)>0)

### Testing Status
✅ Full pipeline validated end-to-end
- Static checkcode on all new scripts: no issues
- Task 1 gates: G1 (x/y symmetry) PASS, G2 (Ts⁴ approx) PASS, G3 (min h_bar 1.11) OK
- Task 2b convergence: 15 iters, rel tol 1e-4, monotonic (G4 PASS)
- Lookup rebuild gates: C_dpmr_eff[0.7, 1.0] = 4.0275 in [3.95, 4.10] PASS
- bias_factor vs Task 1b: within widened tol (5e-3)
- Simulink 5-variant comparison: all run OK, no NaN, no DARE divergence

### Key Results (final run, noise ON, 3-cycle oscillation trajectory)

| Variant | Q(3,3) | Q(6,6) | Q(7,7) | R(2,2) | 3D RMSE | a_hat_z rel |
|---|---|---|---|---|---|---|
| empirical | 1e4 | 1e-4 | 0 | 1.0 | 57.4 nm | 21.7% |
| derived_β | 1 | 1.3e-11 | 1.3e-11 | 1.72 | 65.4 nm | 39.1% |
| derived_B' | 1 | 1.3e-4 | 2.4e-9 | 1.72 | 62.2 nm | 14.1% |
| **B'_Remp** ★ | 1 | 1.3e-4 | 2.4e-9 | 1.0 | **53.4 nm** | **14.6%** |
| Qemp_Rderived | 1e4 | 1e-4 | 0 | 1.72 | 58.6 nm | 26.6% |

★ Best configuration found: B' Q + empirical R beats empirical on both metrics.

### Next Steps
- [ ] Decide whether to install B'_Remp as user_config default (currently β installed)
- [ ] Investigate R(2,2) theoretical 1.70 vs empirical 1.0 gap (chi_sq·ρ_a upper bound issue)
- [ ] Q(4,4) tuning (user's stated next phase per D1)
- [ ] Q(3,3) model-error gap writeup strategy (empirical 1e4 vs physical 1)
- [ ] Optional: B'-1 (filter bandwidth) and B'-3 (DARE eigenvalue) alternatives

### Issues & Notes

⚠️ **No commit-pushes attempted** (per project rule "commit only when asked"). Two commits staged locally: code/config + progress log.

⚠️ **user_config.m currently installed β values** (1.3e-11). If you want to see empirical/B'_Remp performance, must switch back. Use `verify_qr_derivation.m` for any 5-way re-test.

💡 **B'-2 first-principles Q(6,6) ≈ empirical Q(6,6)**: Var(a)·Ts/T_c with T_c=1/(2πf₀) gives 1.30e-4, nearly identical to empirical 1e-4. Significant theoretical result — physically justifies a previously ad-hoc tuning value.

💡 **R(2,2) derivation is SELF-CONSISTENT but SIMULINK prefers R=1 over R=1.70**: Sigma_e(3,3) theoretical 36 nm matches empirical Z RMSE 34 nm (Lyapunov calc verified), but chi_sq·ρ_a·V_meas² seems to overestimate actual Var(V_meas). Suggests non-stationary correction needed.

💡 **Q(3,3) remains the theory-empirical gap**. Physical (a/a_nom)²≈1 vs empirical 1e4 (10000× inflation). Inflation compensates for F_e linearization residuals (wall-effect nonlinearity). Resolution requires either Stateflow workaround, F_e redesign, or pragmatic acceptance.

💡 **None of the 5 variants reach a_hat_z < 10% threshold** — likely the threshold was set too aggressively for the realistic near-wall trajectory with sensor noise 0.01um. Even empirical gets 21%.

### Git Commit
`42c77e8` - WIP(test/qr): full Q/R derivation pipeline + B'-2 white-noise equivalent + Simulink verification

---

## 2026-04-16 (Session 2) — Q/R Coupled Stability Analysis

### Completed Parts
- ✅ Step (b): Q/R entry-by-entry derivation (8/9 from physics)
- ✅ Step (a): Paper Q/R simulation + seed instability discovery
- ✅ 5 near-wall problem points identified and severity-ranked
- ✅ Self-consistent C_dpmr_eff lookup rebuilt (lc x a/a_nom, 48 DARE points)
- ✅ Warmup drift quantification: z 1sig=+-22%, x 1sig=+-11%
- ✅ h=50um free-space verification: paper Q/R ALSO unstable (NOT wall-effect specific)
- ✅ Q(6,6)-R(2,2) coupled stability mechanism identified and documented
- ✅ Forgetting factor sweep (lf=0.99/0.98/0.95): lf=0.99 stabilizes z (spread 6pp) but not x
- ⏸️ Adaptive Q(3,3)/C_dpmr_eff implementation attempted, blocked by Stateflow limitations
- ⏸️ Combined Q(6,6)+R(2,2) coupled fix not yet tested

### File Changes
**Modified Files:**
- `test_script/build_cdpmr_eff_lookup.m` (+94/-72 lines)
  Changed: f0 axis -> aratio axis, self-consistent Q/R per grid point
- `model/controller/calc_ctrl_params.m` (+2/-2 lines)
  Changed: read last column (free-space) instead of first column
- `reference/for_test/fig_cdpmr_eff_lookup.png` (regenerated for aratio lookup)
- `reference/for_test/fig_bias_factor_lookup.png` (regenerated)

### Testing Status
⏸️ Extensive testing completed, key findings documented
- Tested: paper Q/R multi-seed (h=2.5, h=50) ✅
- Tested: forgetting factor lf=0.99/0.98/0.95 ✅
- Tested: adaptive Q(3,3) + C_dpmr_eff (Stateflow blocked) ✅
- Pending: Q(6,6)+R(2,2) coupled sweep ⬜
- Pending: lf=0.99 + R(2,2)=1.0 combined test ⬜

### Next Steps
- [ ] Test lf=0.99 + R(2,2)=1.0 (coupled fix, config-only, no code change)
- [ ] Resolve Stateflow compatibility for adaptive Q(3,3)/C_dpmr_eff
- [ ] Update qr_theoretical_values.md with Q(6,6)-R(2,2) coupling analysis
- [ ] Update task_qr_reference_report.md with all test results
- [ ] Decide writeup narrative: physical derivation + coupled stability story

### Issues & Notes
⚠️ **Critical finding**: Q(6,6)=0 is structurally fragile at ANY h (not wall-specific)
- P(6,6) collapses in ~100 steps -> L(6,2)->0 -> a_hat permanently frozen
- Combined with small R(2,2)=0.176 -> L(6,2) approx 1 during critical window -> a_hat slammed by noise

⚠️ **Stateflow limitation**: if/else branches, evalin, load, local functions all fail compilation
- Workaround: analytic C_dpmr approximation passed but overall still unstable

💡 **Key insight**: Q(6,6) and R(2,2) are a COUPLED stability pair
- Physical values (both small) -> fragile estimator
- Code values (both inflated) -> robust but conservative
- Forgetting factor lf<1 can replace Q(6,6) but NOT R(2,2)

💡 **Writeup value**: physical Q/R derivation is correct but insufficient without stability mechanisms

### Git Commit
`c3214c2` - WIP(test/qr): Q/R coupled stability analysis + forgetting factor + free-space verification

---

## 2026-04-16 (Session 1) — Step (b) derivation + Step (a) simulation

### Git Commit
`8a212eb` - WIP(test/qr): Q/R theoretical derivation + simulation verification + adaptive Q plan

(See previous progress entry for details)

---
