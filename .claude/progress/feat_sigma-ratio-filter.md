# Sigma_e Ratio Filter & 6-State Analysis Progress

## 2026-04-07 — Complete: Ratio Filter L0-L4 + 6-State Analysis + MC Q/R Optimization

### Phase 1: Ratio Filter (L0-L4)

- ✅ L0: Math verification ALL PASS (linearity exact, chi^2 confirmed, Lyapunov 0%)
- ✅ L2: Oracle: ratio RMSE=13.1% vs IIR RMSE=42.9% (69% improvement)
- ✅ L3: Closed-loop: ratio 25% vs IIR 27% (modest, beta-limited)
- ✅ L4: 5-state KF: ratio filter FAILS (positive feedback from Sigma_e lag)

**Key finding**: Sigma_e ratio filter works as standalone estimator but NOT inside 5-state KF feedback loop. Root cause: structural lag in Sigma7 (noise propagates 2-3 steps through delay chain) creates positive feedback when a_hat changes.

### Phase 2: 6-State Sigma_e Analysis

- ✅ L0: A_cl correct (eigenvalues stable, one-step exact, Var(delta_x) MC match 1.6%)
- ✅ L0 limitation: Var(e_a) doesn't match MC (a_m noise linearization inadequate)
- ✅ L1: f_d coupling quantified (tracking error grows 3.5x at f_d=20pN)
- ✅ R_actual measured: 2.14e-5 (2.5x chi^2 theory, 12x smaller than R_kf)

**Key insight**: Var(delta_x) correct but Var(e_a) wrong because at f_d=0, delta_x is DECOUPLED from e_a. The a_m measurement noise is state-dependent (nonlinear), linearized model can't capture it.

### Phase 3: Theory-Guided MC Optimization

- ✅ R(2,2) sweep: optimal at 100x sigma2 (ignore a_m, dynamics channel sufficient)
- ✅ Q(3,3) x Q(4,4) sweep: optimal at Q33=100, Q44=0.01
- ✅ Final comparison:

| Config | std(delta_x) | e_a RMSE | corr |
|--------|-------------|----------|------|
| Current (Q33=1e4, R22=1.0) | 23.56 nm | 39.2% | 0.973 |
| **Optimal (Q33=100, R22=100)** | **21.96 nm** | **18.1%** | 0.939 |

### Overall Conclusions

1. **Sigma_e role**: Analysis tool for controller DESIGN, not for real-time use in KF
2. **f_d x e_a coupling**: Real and quantified. Gain error amplified by control force.
3. **Q(3,3) = 10000x is overkill**: Q33=100 sufficient, saves unnecessary noise injection
4. **R(2,2) should be large**: Dynamics channel dominates gain estimation. a_m adds noise.
5. **Gain estimation improves 2x** with optimized Q/R (39% → 18% RMSE)

### Pending
- [ ] Verify optimal Q/R in Simulink 3D (Level 5)
- [ ] Test with measurement noise (sigma2_n > 0)
- [ ] Investigate if adaptive R(2,2) based on f_d improves further

### Files
**Committed**: fig_ratio_L2_oracle.png, fig_ratio_L3_closedloop.png, fig_ratio_L4_5state.png, fig_ratio_L4a_matched.png, fig_6state_L1_fd_sweep.png, fig_6state_L1_QR_contour.png, fig_6state_mc_sweep.png
**Temp** (gitignored): temp_ratio_L0-L4.m, temp_6state_L0-L1.m, temp_6state_Ractual.m, temp_6state_mc_sweep.m
**Data** (gitignored): ratio_L2-L4_results.mat, mc_qr_sweep_results.mat

### Branch
`feat/sigma-ratio-filter` — 9 commits from 78485ae
