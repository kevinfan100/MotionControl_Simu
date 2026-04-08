# Sigma_e Analysis & Q/R Optimization — Final Progress

## 2026-04-08 — All Short-Term Tasks Complete

### Summary of All Work

#### Phase 1: Ratio Filter (L0-L4)
- ✅ Oracle: 69% improvement (ratio RMSE 13% vs IIR 43%)
- ✅ Closed-loop: modest (25% vs 27%, beta-limited)
- ✅ 5-state KF: fails (positive feedback from Sigma_e lag)
- **Conclusion**: Ratio filter works standalone, not inside 5-state KF feedback loop

#### Phase 2: 6-State Sigma_e Analysis
- ✅ A_cl verified (Var(delta_x) matches MC 1.6%)
- ✅ f_d coupling quantified (3.5x tracking error growth at f_d=20pN)
- ✅ Var(e_a) limitation identified (a_m noise linearization inadequate)
- **Conclusion**: Sigma_e for design/analysis, not real-time use

#### Phase 3: Theory-Guided MC Q/R Optimization
- ✅ MC sweep found optimal: Q33=100, Q44=0.01, R22=100
- ✅ Gain estimation 2x better (39→18% RMSE)
- ✅ 1D tracking 7% better (23.6→22.0nm)

#### Phase 4: Validation (NEW)
- ✅ **Simulink 3D**: 57% w-dir tracking improvement (53→22.6nm), 60% less force
- ✅ **Noise robustness**: optimal holds across sigma_n=0-10nm
- ✅ **Near-wall FAILURE**: optimal Q/R diverges at h=1.5-3um. Current heuristic (Q33=1e4) needed.

### Key Conclusions

1. **No universal optimal Q/R** — regime-dependent:
   - Standard trajectory (h=2.5-5um): Q33=100, R22=100 is better
   - Near-wall (h<3um): Q33=1e4, R22=1.0 is necessary
   - This motivates ADAPTIVE Q/R

2. **Simulink validates 1D findings with even bigger improvement** (57% vs 7%)

3. **The role of Sigma_e**:
   - Analysis tool for understanding f_d x e_a coupling
   - Guides MC parameter search
   - NOT for real-time KF integration

### Pending (Medium-Term)
- [ ] Adaptive Q/R: switch Q33 based on operating regime (h threshold)
- [ ] Test with ctrl7 (7-state) architecture
- [ ] Formal write-up in reference/for_test/

### Git
Branch `feat/sigma-ratio-filter` — 12 commits, pushed.
