# Formula Verification Development Progress

## 2026-03-30 — KF Observer Analysis Complete

### Completed Parts
- ✅ Controller 1 (Eq.17): C_dpm = 2+1/(1-lc^2), C_dpmr corrected formula
- ✅ Controller 2 (Observer): C_dpm = 3+1/(1-lc^2), K_eff(av) = 2+2*(1-av)^2/(2-av)
- ✅ C_dpmr (1-a_pd)^2 HP filter prefactor correction
- ✅ Controller 3 (EKF): no analytic formula (nonlinear)
- ✅ Controller 4 (KF observer): iterative implementation, rho->le->C_dpm chain
- ✅ Fe vs F design difference discovery and verification
- ✅ C_kf(lc, le) exact formula
- ✅ C_dpmr 3-term decomposition (ctrl2_term1 + le*g + term2)
- ✅ Near-wall le*g contribution analysis (h=2.5um, r=0.45)
- ✅ Measurement noise C_dpm verification
- ✅ Literature review (Kalata, Ekstrand, Berg-Sorensen)
- ✅ LaTeX derivation (kf_derivation.tex)
- ⏸️ Self-consistent verification (a_m recovery with correct C_dpmr)
- ⏸️ Full closed-loop analysis (5/7-state with a_x estimation)

### Testing Status
✅ Controller 1/2 C_dpm: Simulink vs theory < 4%
✅ Controller 1/2 C_dpmr: Lyapunov exact + Simulink < 6%
✅ Controller 4 C_dpm: Simulink vs Lyapunov < 5%
✅ Controller 4 C_dpmr: Simulink vs theory < 6%
⬜ Self-consistent a_m recovery
⬜ Full closed-loop 5/7-state

### Next Steps
- [ ] Approach 1: Self-consistent verification (a_m recovery using correct C_dpmr)
- [ ] Approach 2: Build 5-state closed-loop model (ctrl4 + a_x estimation)
- [ ] Connect to Near-Wall paper Eq.12-13
- [ ] Consider merge to main

### Issues & Notes
⚠️ Near-wall (h=2.5um): le*g contributes 5-10%, ctrl2 formula insufficient
💡 le_eff concept may be novel contribution (not found in literature)
💡 IIR-estimator-control feedback loop must be considered for motion gain estimation

### CRITICAL DISCOVERY (2026-03-30)
Self-consistent a_m recovery verification revealed:
- **thermal C_dpmr + Lyapunov noise_corr → a_m/a_x = 1.0000 (EXACT)**
- Old noise_corr `(2/(1+lc))*sigma2_n` is for del_pm (before IIR), 8.3x too large for del_pmr
- Correct noise_corr for del_pmr comes from Lyapunov with Q_noise only
- Analytic formula for noise_corr_dpmr: TBD
- This explains why the "old formula" accidentally worked: wrong C_dpmr compensated wrong noise_corr

### Self-Consistent Verification (2026-03-30)
- ✅ thermal C_dpmr + Lyapunov noise_corr → a_m/a_x = 1.0000 (EXACT)
- ✅ Confirmed: C_dpmr formula is correct, noise_corr needs IIR-aware version
- ✅ Core insight: A7 correct → C_dpmr correct → noise_corr_dpmr correct (same system, different Q)
- ⏸️ noise_corr_dpmr analytic formula: TBD (same symbolic Lyapunov, B_meas instead of B_thermal)

### 5-State KF Estimator (2026-03-31)

**Architecture**: 5-state dual measurement (del_pm + Eq.13 a_m)
- States: [del_p1, del_p2, del_p3, a, del_a] (simplified 7-state, no disturbance)
- controller_type = 5

**Key finding: single-measurement Plan B diverges**
- f_d = (1/a_hat)*stuff creates circular feedback: a_hat error -> f_d change -> Fe coupling -> worse a_hat
- Fe(3,3)=1 is mathematically correct for error dynamics, but L4 << L1 (~0.05 vs ~0.99) makes gain correction fragile
- f_d threshold + Pf freeze insufficient: Pf accumulates during inactive, explodes on activation
- Solution: add Eq.13 as 2nd measurement channel (stable gain baseline)

**Scenario B1 (near-wall osc, wall ON, meas noise OFF)**:
- Gain RMSE: 9.7% (cycle1) -> 5.9% (cycle3)
- Tracking z: 31 nm (cycle1) -> 27 nm (cycle3)
- a_hat/a_true correlation: r=0.94, lag=27 samples (6.1 deg at 1Hz)
- a_hat/a_true ratio range: [0.40, 1.74] (turning point lag)

**Frozen-f_d analysis**:
- det(O) = f_d^2 confirmed (observability depends on coupling)
- DARE fails for 5-state (symplectic spectrum near unit circle), iterative fallback works
- Pf_ss(4,4) drops 3700x from f_d=0 to f_d=5 pN

**Pending**:
- [ ] ctrl7 benchmark comparison (same B1 config)
- [ ] Measurement noise ON test
- [ ] Q/R sensitivity sweep
- [ ] Scenario B2/B3 (deeper wall / higher frequency)

### Git
Branch: feat/formula-verification
Last commit: `881f282` - feat(control): add 5-state KF estimator with dual measurement
