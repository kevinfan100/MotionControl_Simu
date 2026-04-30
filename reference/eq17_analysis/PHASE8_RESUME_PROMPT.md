# Phase 8 Resume Prompt — for use after /compact

## Quick context

Branch: `test/eq17-5state-ekf`
Tag: `eq17-pre-redesign-2026-04-29` @ `527b89e` (v1 baseline, before redesign)

## Where we are

Phase 8 v2 redesign, ~98% done. Wave 4 v3 (Stage 11 Option I applied):
- ✓ Stability + tracking PASS (30 nm, similar to v2's 29 nm)
- △ a_hat bias **-11%** (improved 3× from v2's -31%, but still > 5% target)
- ✗ σ²_δx ratio 0.54 (worse than v2's 0.76 — see below)

**Status: PARTIAL PASS.** Stage 11 fix is correct in direction; magnitude
under-corrects because the Lyapunov helper `compute_7state_cdpmr_eff_v2.m`
Row 1 (lines 130-136) inherits v1 coefficients that don't match v2's
F_e(3,4) = -1.6 closed-loop coupling. See `phase8_e2e_h50_results_v3.md`.

**3 paths forward** (user decision):
- **Path A (rigorous)**: Re-derive A_aug Row 1 for v2 → expected bias < 2%
- **Path B (pragmatic)**: Empirical lookup calibration (qr-style)
- **Path C (accept)**: Document -11% as known limitation

## v2 implementation done (committed)

```
6add6da  Stage 11 Option I — per-axis C_dpmr_eff/C_np_eff (4 files +90 lines)
a6128d3  Phase 8 resume prompt for /compact continuation
eef59f9  Stage 11 partial — helper ported (untested at commit time)
8d6bbcb  Wave 4 v2 e2e results (WARN, bias 31% diagnostic)
990e157  Stage 10 Option A — G1 cross-coupling fix (PASS)
af7b055  Wave 1 audit + Wave 3 smoke
4233803  Wave 3 — 7 priority unit tests
115b0e8  Wave 2C — predict_closed_loop_var_eq17_v2 oracle
4a5bbfe  Wave 2 (B+D) — controller + driver implementation
4843e6f-8804e14  Phase 0-7 paper derivation (8 commits)
```

**Smoke + Wave 4 v3 verified** (this session, 2026-04-30):
- Smoke seed=1 T_sim=1s: bias [-3.06, -7.71, -10.09]% (PASS, no NaN)
- Wave 4 v3 5-seed T_sim=5s: bias [-11.10, -10.50, -11.89]% (PARTIAL PASS)
- Saved: `test_results/wave4_v3_5seed_tsim5_h50.mat`
- Doc:   `reference/eq17_analysis/phase8_e2e_h50_results_v3.md`

## Stage 11 Option I — a_hat bias fix path (this is what we're working on)

**Goal**: Port qr branch's per-axis on-the-fly C_dpmr_eff fix to v2.
v1 (qr branch) reduced h=50 bias from -2.85% to -0.80% via this fix.
v2 not yet ported. Stage 11 task.

**Already done (committed eef59f9)**:
- `test_script/compute_7state_cdpmr_eff_v2.m` (318 lines)
  - Helper function ported from qr branch
  - Adapted for v2 Eq.19 form (F_e(3,4) = -1.6)
  - 11-dim augmented Lyapunov solver
  - Phase 1 assumption: f_0 = 0 (positioning baseline)

**Remaining (~70%)**:
1. `model/controller/calc_ctrl_params.m`:
   - Per-axis call `compute_7state_cdpmr_eff_v2()` 
   - Build `ctrl.C_dpmr_eff_per_axis` (3x1) from result
   - Also `ctrl.C_np_eff_per_axis` (3x1)
   - Reference qr branch: `git show feat/sigma-ratio-filter:model/controller/calc_ctrl_params.m`

2. `model/calc_simulation_params.m`:
   - Extend CtrlBus by 2 elements (currently 23 → 25)
   - Add `C_dpmr_eff` (1x3) and `C_np_eff` (1x3)

3. `model/controller/build_eq17_constants.m`:
   - Pass through `ctrl.C_dpmr_eff_per_axis` and `ctrl.C_np_eff_per_axis`
   - Make available as `ctrl_const.C_dpmr_eff` (3x1)

4. `model/controller/motion_control_law_eq17_7state.m`:
   - In a_xm formula (~line 240 area):
     - Currently: `a_xm = (σ²_δxr - C_n*σ²_n_s) / (C_dpmr * 4kBT)` using paper closed-form
     - New: use per-axis `ctrl_const.C_dpmr_eff(i)` and `ctrl_const.C_np_eff(i)`
   - Per-axis loop already exists (3 KFs)

5. Smoke verification (1-seed, T_sim=1s):
   - Expect: stable, no NaN, a_hat bias < 5% (down from -31%)

6. Wave 4 v3 (after smoke pass):
   - Re-run `test_script/run_v2_h50_e2e.m`
   - 5-seed CI, T_sim=5s
   - Expect: bias < 5%, ratio ∈ [0.9, 1.1]

## How to dispatch Stage 11 finish (after /compact)

User says "繼續 Stage 11 finish" or similar.

I should:
1. Read `compute_7state_cdpmr_eff_v2.m` to refresh on signature
2. Read `git show feat/sigma-ratio-filter:model/controller/calc_ctrl_params.m` to see qr's call pattern
3. Implement integration in 4 files (mechanical, no derivation)
4. Run MATLAB checkcode on all
5. Smoke seed=1 T_sim=1s via `mcp__matlab__run_matlab_code`
6. If pass → commit, then dispatch Wave 4 v3 e2e (or wait for usage reset 12:40am Asia/Taipei)
7. If fail → analyze, may need adjustment to Eq.19 form Lyapunov

## Key files reference

- Phase 0 design spec: `reference/eq17_analysis/design_v2.md`
- Phase 1 F_e: `reference/eq17_analysis/phase1_Fe_derivation.md` (F_e(3,4) = -1.6)
- Phase 2 C_dpmr/C_n: `reference/eq17_analysis/phase2_C_dpmr_C_n_derivation.md`
- Phase 6 R + 3-guard: `reference/eq17_analysis/phase6_R_matrix_derivation.md`
- Phase 7 oracle: `reference/eq17_analysis/phase7_lyapunov_bench.md`
- Wave 4 v2 results: `reference/eq17_analysis/phase8_e2e_h50_results_v2.md`
- Stage 10 Option A: `reference/eq17_analysis/phase8_stage10_optionA.md`
- This resume prompt: `reference/eq17_analysis/PHASE8_RESUME_PROMPT.md`

## Critical context for understanding the bug

Don't re-discuss these — they're settled:
- Phase 0-7 derivations are 100% correct (paper Eq.17 + Σf_d + 7-state)
- F_e(3,4) = -1.6 (parameterized, not hardcoded)
- Stage 10 Option A: G1 期間 force L(6,:) = L(7,:) = 0 (cross-coupling fix)
- v2 actually performs BETTER than paper Eq.22 predicts (~37% less variance)
- a_xm formula must use C_dpmr_eff (matching actual closed-loop), not paper closed-form 3.96
- This is the same fix qr branch validated (-2.85% → -0.80% bias at h=50)

## After Wave 4 v3 PASS, remaining Phase 8

- Phase 6 doc update for Stage 11 (mention C_dpmr_eff replaces paper closed-form)
- T4 test threshold recalibration (cosmetic)
- Optional: extended scenarios (h=2.5 near-wall, motion 1Hz)

## Resume prompt template (paste this after /compact)

> 繼續 Phase 8 v2 redesign. Stage 11 Option I 還剩 ~70% integration 工作. helper function 已 ported (commit eef59f9). 詳見 `reference/eq17_analysis/PHASE8_RESUME_PROMPT.md`. 直接動手做 Stage 11 finish (Path C: 我手動完成 integration + smoke). 不需重新討論已 settled 的點 (Phase 0-7 derivation, Stage 10 Option A 等). 完成後 commit, 然後等 usage reset (12:40am Asia/Taipei) 跑 Wave 4 v3 verify.

---

**End of resume prompt. Use after /compact to continue Phase 8.**
