# Sigma_e Ratio Filter Development Progress

## 2026-04-06 22:10 - Level 0-2 Verification Complete

### Completed Parts
- ✅ Level 0: Mathematical verification (Lyapunov match, chi^2 distribution, linearity, noise)
- ✅ Level 1: Stationary oracle test (constant a, ctrl2 deadbeat)
- ✅ Level 2: Time-varying oracle test (MC 500, DECISION GATE PASSED)
- ⏸️ Level 3: Closed-loop self-update (pending)
- ⬜ Level 4: 5-state KF integration
- ⬜ Level 5: Simulink validation

### Key Results
**Level 0 (All Pass):**
- 7-state Lyapunov vs recursion: 0.000000% error
- Chi^2(1) distribution: mean=1.003, var=2.016
- Linearity: max deviation = 0.000000 (EXACT)
- C_dpmr (7-state, le=0, rho=0.05) = 4.3177

**Level 2 (DECISION GATE PASSED):**
| Method | RMSE | Corr |
|--------|------|------|
| IIR oracle (a_cov=0.05) | 42.93% | 0.41 |
| Ratio oracle (a_ratio=0.002) | **13.13%** | **0.92** |
| Improvement | **69.4%** | -- |

### File Changes
**New Files (gitignored - temp):**
- `test_script/temp_ratio_L0.m` (224 lines) -- Level 0 math verification
- `test_script/temp_ratio_L2.m` (324 lines) -- Level 1+2 oracle comparison

**New Files (to commit):**
- `reference/for_test/fig_ratio_L2_oracle.png` -- Oracle comparison figure

**Data (gitignored):**
- `test_results/verify/ratio_L2_results.mat` -- MC 500 results

### Testing Status
✅ Level 0-2 verified (MC 500 runs, all pass criteria met)
⬜ Level 3-5 pending

### Next Steps
- [ ] Level 3: Closed-loop test (a_hat self-updating, ctrl4)
- [ ] Level 4: 5-state KF integration with physical R(2,2)
- [ ] Level 5: Simulink validation
- [ ] Investigate Phase 1 bias (0.79) -- possibly IIR warmup artifact

### Issues & Notes
- Phase 1 stationary bias = 0.79 (both methods): likely IIR warmup or C_dpmr mismatch for ctrl2 structure. Does not affect ratio vs IIR comparison.
- Ratio filter a_ratio=0.002 gives best RMSE, but even a_ratio=0.01 (RMSE 21.7%) significantly beats IIR (42.9%).

### Branch
`feat/sigma-ratio-filter` (from `feat/formula-verification` @ 78485ae)

### Plan
`.claude/plans/moonlit-leaping-snowglobe.md`
