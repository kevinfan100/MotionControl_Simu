# QR-Paper-Reference Branch Cleanup + Merge Plan

**Date**: 2026-04-28
**Author**: Kevin Fan + Claude
**Branch in scope**: `test/qr-paper-reference` (worktree: `../MotionControl_Simu_qr`)
**Target outcome**: Clean qr branch to "final result only" state, then prepare for merge into `feat/sigma-ratio-filter`.

---

## 1. Goal

Aggressive cleanup of `test/qr-paper-reference` such that only the **final QR derivation pipeline + figure-producing code + canonical documentation** remain. Historical / exploratory / predecessor work is removed (code) or archived (documents).

The cleaned branch will then be merged into `feat/sigma-ratio-filter` (qr → current direction; merge plan is a separate spec, not in this document).

---

## 2. Principles

| Layer | Rule |
|---|---|
| Code (`.m`) | Final pipeline only. **No `archive/code/` subfolder** — historical code is deleted outright. |
| Docs (`.md`, `.tex`) | Canonical at top-level + historical preserved in `archive/` subfolder. |
| Figures (`.png`) | Canonical at top-level + historical preserved in `archive/figs/`. |
| Folder name | `reference/for_test/` → `reference/qr_analysis/` (semantic rename). |
| Commit history | C-mode: 35 commits → 5 logical commits via `git reset --soft` reflow. |
| Backup | Archive branch `test/qr-paper-reference-archive` pushed to remote before any destructive op. |

---

## 3. Final State

### 3.1 `test_script/` — 11 .m files

**Pipeline 1: Q/R lookup builders (6)**
- `compute_q77_from_trajectory.m` — Q(7,7) from trajectory + `fig_q77_trajectory.png`
- `compute_r22_self_consistent.m` — R(2,2) fixed-point + `fig_r22_convergence.png`
- `compute_rho_a_rigorous.m` — per-scenario ρ_a helper
- `compute_7state_cdpmr_eff.m` — 11-dim Lyapunov utility (per-axis Einstein)
- `build_cdpmr_eff_lookup.m` — `cdpmr_eff_lookup.mat` + `fig_cdpmr_eff_lookup.png`
- `build_bias_factor_lookup.m` — `bias_factor_lookup.mat` + `fig_bias_factor_lookup.png`

**Pipeline 2: QR verification + figure generators (4)**
- `verify_qr_derivation.m` — 5-variant Simulink → `fig_qr_verification.png`
- `verify_qr_positioning_run.m` — positioning worker (batches)
- `verify_qr_positioning_aggregate.m` — `fig_qr_positioning_summary.png` + `fig_baseline_empirical_zoom.png`
- `plot_qr_h50_figures.m` — `fig_qr_h50_tracking_std.png` + `fig_qr_h50_ahat_std.png` + `fig_qr_h50_verification/{fig1,fig2}.png` (writeup §8 figures)

**Main entry (1)**
- `run_simulation.m` — Simulink top-level

### 3.2 `model/` — unchanged from current qr branch top

```
model/
├── calc_simulation_params.m         [Bus schema 3×1 per-axis]
├── system_model.slx
├── config/
│   ├── apply_qr_preset.m            [NEW]
│   ├── physical_constants.m
│   └── user_config.m                [qr_preset field]
├── controller/
│   ├── calc_ctrl_params.m           [per-axis on-the-fly C_dpmr/Cn/beta]
│   ├── motion_control_law.m         [router]
│   ├── motion_control_law_1.m       [type=1, Eq.17]
│   ├── motion_control_law_2.m       [type=2, observer]
│   ├── motion_control_law_4.m       [type=4, KF observer]
│   ├── motion_control_law_5state.m  [type=5, 5-state KF]
│   └── motion_control_law_7state.m  [type=7, 7-state EKF + Bus + a_m exposed]
├── wall_effect/
├── thermal_force/
└── trajectory/
```

### 3.3 `reference/qr_analysis/` — folder structure

```
qr_analysis/
│
├── Canonical docs (top-level, 5 .md + writeup .tex/.pdf)
│   qr_theory_complete_final.md
│   qr_verification_h50_final_2026-04-27.md          [merged: 4-22 framework + 4-27 lookup fix]
│   qr_known_limitations.md                          [renamed from qr_parameter_audit_2026-04-20.md]
│   r22_self_consistent_report.md                    [supporting]
│   0427_kf_paras.md                                 [KF parameter inventory]
│   writeup_architecture.tex / .pdf                  [from current branch via merge later — not in qr cleanup scope]
│
├── Legacy reference docs (already in for_test/, kept)
│   variance_recursion.tex / .pdf
│   kf_derivation.tex
│   Observer-1.pdf
│   Tracking Error Variance -1.pdf
│
├── Canonical figures (top-level)
│   fig_qr_h50_tracking_std.png        ← writeup §8
│   fig_qr_h50_ahat_std.png             ← writeup §8
│   fig_qr_h50_verification/{fig1_gain_estimation, fig2_tracking_error}.png
│   fig_qr_positioning_summary.png
│   fig_qr_verification.png
│   fig_q77_trajectory.png
│   fig_r22_convergence.png
│   fig_baseline_empirical_zoom.png
│   fig_bias_factor_lookup.png
│   fig_cdpmr_eff_lookup.png
│   fig_p2_h_bin.png                    [from current branch via merge later]
│
└── archive/
    ├── theory/        [7 .md]
    ├── verification/  [6 .md]
    ├── sessions/      [4 .md + 2 temp_*.md]
    └── figs/          [~25 historical .png]
```

---

## 4. Detailed File Operations

### 4.1 Files to DELETE on qr branch (no archive)

**Code (19 .m in `test_script/`)**:

| Group | Files | Reason |
|---|---|---|
| G1: Σ_e abandoned | `verify_sigma_mc_1d.m`, `verify_sigma_recursion.m` | Σ_e tool replaced by 11-dim augmented Lyapunov |
| G2: 5-state predecessor | `analyze_5state_frozen.m` | Replaced by 7-state EKF |
| G3: Task 1b/c/d predecessors | `analyze_task1b_iir_bias.m`, `analyze_task1d_ahat_static.m`, `verify_task1c_correction.m`, `verify_task1d_paper_benchmark_mc.m` | Subsumed by §6 (β) and §8 (CV²) closed-form |
| G4: Phase 2/3 exploration | `phase2_chisquared_mc.m`, `phase2_paper_benchmark.m`, `phase3_acov_validate.m`, `phase3b_acov_sweep.m`, `phase3b_multilag_autocorr.m` | a_cov/chi-sq exploration finalized in `qr_theory_complete_final.md` §13 |
| G5: Early C_dpmr | `verify_variance.m`, `verify_cdpmr_eff_sanity.m`, `verify_cdpmr_eff_simulink.m` | Replaced by `verify_qr_derivation.m` |
| G6: Sanity / regression | `regression_7state_new_cdpmr.m`, `dryrun_7state_with_cdpmr_eff.m` | Not in production pipeline |
| P2: P2 verification | `verify_p2_static_h25.m`, `analyze_p2_h_bin.m` | User chose (ii): delete on qr; current branch's versions survive merge |

**Docs (2 .md)**:

| File | Reason |
|---|---|
| `2026-04-15-qr-theoretization-design.md` | Implementation complete; design spec no longer needed |

### 4.2 Files to RENAME

| From | To | Action |
|---|---|---|
| `reference/for_test/` | `reference/qr_analysis/` | `git mv` directory |
| `qr_parameter_audit_2026-04-20.md` | `qr_known_limitations.md` | `git mv` |
| `qr_verification_h50_final_2026-04-22.md` + content of `qr_lookup_fix_2026-04-27.md` | `qr_verification_h50_final_2026-04-27.md` | Read both → write merged → delete originals |

### 4.3 Files to MOVE to `archive/`

**`archive/theory/` (7 .md)**:
- `theory_complete_v2.md`
- `theory_tracking_and_ahat_v1.md`
- `theory_correction_analysis.md`
- `theory_summary_2026-04-20.md`
- `theory_ahat_std_limitations.md`
- `ahat_quality_prediction.md`
- `qr_theoretical_values.md`

**`archive/verification/` (6 .md)**:
- `qr_positioning_summary.md`
- `qr_positioning_free_space_h50_report.md`
- `qr_positioning_near_wall_h25_report.md`
- `qr_verification_findings.md`
- `qr_verification_report.md`
- `task_qr_reference_report.md`

**`archive/sessions/` (4 .md + 2 temp)**:
- `session5_ahat_quality_findings.md`
- `session6_frozen_steady_state.md`
- `session6_handoff_for_continuation.md`
- `session7_p1_prefill_iir.md`
- `temp_5state_analysis_notes.md`
- `temp_kf_analysis_summary.md`

**`archive/figs/` (~25 .png from intermediate / deleted-script outputs)**:
- `fig_5state_correct_cdpmr.png` (G2)
- `fig_6state_L1_QR_contour.png`, `fig_6state_L1_fd_sweep.png`, `fig_6state_mc_sweep.png` (Σ_e exploration)
- `fig_innovation_step12_Sratio.png` (early KF)
- `fig_ratio_L2_oracle.png`, `fig_ratio_L3_closedloop.png`, `fig_ratio_L4_5state.png`, `fig_ratio_L4a_matched.png` (early ratio filter)
- `fig_sigma_mc_1d_timeseries.png`, `fig_sigma_sync_result.png` (G1)
- `fig_simulink_qr_compare.png` (early comparison)
- `fig_phase2_chisquared.png`, `fig_phase2_paper_benchmark.png` (G4)
- `fig_phase3b_multilag.png` (G4)
- `fig_task1b_iir_bias.png`, `fig_task1c_correction.png`, `fig_task1d_ahat_vs_am_static.png`, `fig_task1d_paper_benchmark.png` (G3)
- `fig_ctrl1_C_dpm.png`, `fig_ctrl1_C_dpmr.png`, `fig_ctrl2_C_dpm.png`, `fig_ctrl2_C_dpmr.png`, `fig_ctrl4_C_dpm_vs_rho.png`, `fig_ctrl4_C_dpm_with_noise.png`, `fig_ctrl4_C_kf_vs_le.png`, `fig_ctrl4_le_g_contribution.png`, `fig_ctrl4_le_g_near_wall.png`, `fig_ctrl4_le_vs_rho.png`, `fig_ctrl4_variance_convergence.png`, `fig_ctrl7_C_dpm.png`, `fig_ctrl7_C_dpmr.png` (G5)

### 4.4 Files to KEEP at top-level (canonical)

**Top-level docs (after rename `qr_parameter_audit` → `qr_known_limitations`)**:
- `qr_theory_complete_final.md`
- `qr_verification_h50_final_2026-04-27.md` (merged)
- `qr_known_limitations.md`
- `r22_self_consistent_report.md`
- `0427_kf_paras.md`

**Top-level legacy refs**:
- `variance_recursion.tex/.pdf`, `kf_derivation.tex`, `Observer-1.pdf`, `Tracking Error Variance -1.pdf`

**Top-level figures** (10 + 1 subdirectory):
- `fig_qr_h50_tracking_std.png`, `fig_qr_h50_ahat_std.png`
- `fig_qr_h50_verification/` subdirectory
- `fig_qr_positioning_summary.png`, `fig_qr_verification.png`
- `fig_q77_trajectory.png`, `fig_r22_convergence.png`
- `fig_baseline_empirical_zoom.png`
- `fig_bias_factor_lookup.png`, `fig_cdpmr_eff_lookup.png`

### 4.5 Hardcoded path updates (`for_test` → `qr_analysis`)

7 KEEP-list .m files need path replacement (verified by grep on qr worktree):

- `model/config/apply_qr_preset.m`
- `test_script/build_bias_factor_lookup.m`
- `test_script/build_cdpmr_eff_lookup.m`
- `test_script/compute_q77_from_trajectory.m`
- `test_script/plot_qr_h50_figures.m`
- `test_script/verify_qr_derivation.m`
- `test_script/verify_qr_positioning_aggregate.m`

(4 other DELETE-list scripts also have hardcoded `for_test/` paths, but they're being removed so no edits needed: `verify_task1d_paper_benchmark_mc.m`, `verify_sigma_recursion.m`, `verify_sigma_mc_1d.m`, `analyze_p2_h_bin.m`)

**Plus docs that reference `for_test/` paths**:
- All 5 canonical .md files (mostly self-referential, e.g., "see `reference/for_test/...md`")
- `agent_docs/verification-notes.md`
- `CLAUDE.md` (project structure section)
- `.claude/rules/research-workflow.md`

---

## 5. Execution Plan

### 5.1 Pre-flight (non-destructive)

```bash
# In primary repo (any worktree works)
git branch test/qr-paper-reference-archive 5af8cad
git push -u origin test/qr-paper-reference-archive
```

### 5.2 Cleanup on qr worktree

```bash
cd ../MotionControl_Simu_qr

# Reset to common ancestor (working tree intact)
git reset --mixed b271cee

# At this point: working tree has all 35-commit changes as unstaged.
# Index is empty.

# A. Delete 19 .m files (G1-G6 + P2)
rm test_script/{verify_sigma_mc_1d.m, verify_sigma_recursion.m, ...}

# B. Delete design doc + temp_*.md (will be recreated under archive)
rm reference/for_test/2026-04-15-qr-theoretization-design.md

# C. Rename folder
git mv reference/for_test reference/qr_analysis  # may fail since not staged; use plain mv

# D. Create archive subdirs and move files
mkdir -p reference/qr_analysis/archive/{theory,verification,sessions,figs}
mv reference/qr_analysis/theory_*.md reference/qr_analysis/archive/theory/
mv reference/qr_analysis/qr_theoretical_values.md reference/qr_analysis/archive/theory/
mv reference/qr_analysis/ahat_quality_prediction.md reference/qr_analysis/archive/theory/
# ... (full list per §4.3)

# E. Rename canonical doc
mv reference/qr_analysis/qr_parameter_audit_2026-04-20.md reference/qr_analysis/qr_known_limitations.md

# F. Merge lookup fix into 4-22
# Read both files; write merged to qr_verification_h50_final_2026-04-27.md
# Delete originals.

# G. Update hardcoded paths in 7+1 .m files (sed-like; specific edits in §4.5)

# H. Stage and commit in 5 logical chunks
```

### 5.3 Five Logical Commits

**C1: `feat(controller): per-axis Q/R Bus extension`**
- `model/calc_simulation_params.m`
- `model/controller/motion_control_law_5state.m`
- `model/controller/motion_control_law_7state.m`

**C2: `feat(controller): per-axis on-the-fly C_dpmr/Cn/beta`**
- `model/controller/calc_ctrl_params.m`
- `test_script/compute_7state_cdpmr_eff.m`

**C3: `feat(config): apply_qr_preset + frozen_correct preset`**
- `model/config/apply_qr_preset.m` (new)
- `model/config/user_config.m`
- `test_script/run_simulation.m`

**C4: `feat(test): Q/R derivation verification framework`**
- `test_script/build_bias_factor_lookup.m`
- `test_script/build_cdpmr_eff_lookup.m`
- `test_script/compute_q77_from_trajectory.m`
- `test_script/compute_r22_self_consistent.m`
- `test_script/compute_rho_a_rigorous.m`
- `test_script/verify_qr_derivation.m`
- `test_script/verify_qr_positioning_run.m`
- `test_script/verify_qr_positioning_aggregate.m`
- `test_script/plot_qr_h50_figures.m`

**C5: `docs: Q/R canonical theory + verification + KF inventory + archive`**
- `reference/qr_analysis/` (new directory; 5 canonical .md + figures + archive/)
- Path updates in `agent_docs/verification-notes.md`, `CLAUDE.md`, `.claude/rules/research-workflow.md`

### 5.4 Force-push (DESTRUCTIVE)

```bash
git push --force-with-lease origin test/qr-paper-reference
```

`--force-with-lease` aborts if remote was updated by someone else.

---

## 6. Risks & Rollback

### Risks
- **Force-push**: rewrites public branch history. Archive branch preserves original.
- **Path updates incomplete**: scripts may break at runtime if a path was missed. Mitigation: grep verification after path edit; smoke-test by running `verify_qr_positioning_run.m` and `plot_qr_h50_figures.m`.
- **Bus schema commit (C1) without runtime test**: no Simulink test on the temporary branch state. Final state should be identical to pre-cleanup since we're reflowing existing code.

### Rollback
```bash
# Restore qr branch to original 35-commit history
git -C ../MotionControl_Simu_qr fetch origin test/qr-paper-reference-archive
git -C ../MotionControl_Simu_qr reset --hard origin/test/qr-paper-reference-archive
git push --force-with-lease origin test/qr-paper-reference
```

---

## 7. Out of Scope (Follow-up Specs)

- **Merge `qr → feat/sigma-ratio-filter`**: separate plan after qr cleanup verified clean.
- **`feat/sigma-ratio-filter` cleanup**: 3 uncommitted files (`.claude/settings.local.json`, `fig_bias_factor_lookup.png`, `single_AB_run.m`) handled when merge plan begins.
- **Integration of `writeup_architecture.tex` h=50 figs from current branch into qr_analysis canonical layout**: handled at merge time.
- **Final Merge to main**: future, requires separate review.

---

## 8. Acceptance Criteria

- [ ] `test/qr-paper-reference-archive` exists on remote
- [ ] qr branch has 5 commits ahead of `b271cee`
- [ ] `test_script/` contains exactly 11 .m files
- [ ] `reference/qr_analysis/` exists with archive/{theory,verification,sessions,figs}/
- [ ] No hardcoded `reference/for_test/` paths remain in `*.m` files (grep returns 0)
- [ ] `verify_qr_positioning_run.m` runs end-to-end without path errors
- [ ] `plot_qr_h50_figures.m` regenerates `fig_qr_h50_*.png` without errors
