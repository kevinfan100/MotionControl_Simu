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

### Git
Branch: feat/formula-verification (18 commits, all pushed)
Last commit: `6c89bc5` - docs: add C_dpmr 3-term decomposition, le*g analysis, and near-wall figures
