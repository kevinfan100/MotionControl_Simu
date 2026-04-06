# Sigma_e Ratio Filter Development Progress

## 2026-04-07 00:30 - Level 0-4 Complete

### Completed Parts
- ✅ Level 0: Mathematical verification (Lyapunov match, chi^2 distribution, linearity exact, noise)
- ✅ Level 1: Stationary oracle test (constant a, ctrl2 deadbeat)
- ✅ Level 2: Time-varying oracle test (MC 500, DECISION GATE PASSED)
- ✅ Level 3: Closed-loop self-update (ctrl4, converges, RMSE 25% vs IIR 27%)
- ✅ Level 4: 5-state KF integration (Sigma7 TV C_dpmr doesn't help; observer model mismatch)
- ⬜ Level 5: Simulink validation (deferred — see conclusions)

### Key Results

**Level 2 (Oracle, MC 500):**
| Method | RMSE | Corr |
|--------|------|------|
| IIR oracle (a_cov=0.05) | 42.93% | 0.41 |
| Ratio oracle (a_ratio=0.002) | **13.13%** | **0.92** |
| Improvement | **69.4%** | -- |

**Level 3 (Closed-loop, single realization):**
| Method | RMSE | Corr | Bias |
|--------|------|------|------|
| IIR + C_dpmr | 27.3% | 0.64 | 0.81 |
| Sigma_e sync | 28.0% | 0.62 | 0.85 |
| Sigma_e ratio (a_r=0.02, beta=0.005) | **25.0%** | 0.47 | 1.09 |

**Level 4 (5-state KF):**
- Sigma7 TV C_dpmr does NOT improve 5-state KF
- Root cause: Sigma7 models 3-state observer, but 5-state KF has Q(3,3)=10000x
- IIR const C_dpmr is sufficient (RMSE 45-50%)
- Dynamics channel (Fe coupling) dominates gain estimation in 5-state

### Conclusions
1. **Sigma_e ratio filter oracle is dramatically better** (69% improvement, Level 2)
2. **Closed-loop improvement is modest** (25% vs 27%, Level 3) because beta limits adaptation speed
3. **5-state KF doesn't benefit** from Sigma7 TV C_dpmr (Level 4) — observer model mismatch
4. **Best use case**: standalone a estimator (Level 3 architecture) for controllers without Fe coupling
5. **Level 5 Simulink deferred**: the 5-state KF's dynamics channel dominates, so the ratio filter adds little in the augmented-state framework

### Key Insight: Self-Referential Measurement Problem
Ratio-based a_m = a_hat * ratio_smooth is coupled to a_hat. In standalone mode (Level 3), this works via negative feedback. In the 5-state KF, the dynamics channel destabilizes a_hat faster than the ratio's IIR can correct it. Independent a_m (like IIR Eq.13) is required for the augmented-state KF.

### Files
**Temp scripts** (gitignored): temp_ratio_L0.m, temp_ratio_L2.m, temp_ratio_L3.m, temp_ratio_L4.m
**Committed figures**: fig_ratio_L2_oracle.png, fig_ratio_L3_closedloop.png, fig_ratio_L4_5state.png
**Data** (gitignored): ratio_L2_results.mat, ratio_L3_results.mat, ratio_L4_results.mat

### Git
Branch: `feat/sigma-ratio-filter`
Commits: 07d5ab2 (L0-2), 55bd65c (L3), 0f79c5f (L4)

### Plan
`.claude/plans/moonlit-leaping-snowglobe.md`
