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

---

## 2026-04-01 — Time-Varying Variance Recursion (Sigma_e)

### Completed Parts
- ✅ 推導 4x4 Σ_e recursion (Controller 4 + wall effect + trajectory)
- ✅ 證明 F_e 在 a[k] time-varying 時仍為常數
- ✅ Notation 統一（δx, a[k], Σ_e, 4k_BT·a[k]）並寫入規則
- ✅ Phase 1 驗證：stationary, recursion vs Lyapunov = 0.002%
- ✅ Phase 2 驗證：time-varying, MC 500, mean error 5.05%, R^2 = 0.915
- ✅ 發現 controller 用固定 a_nom 時 F_e 變 time-varying（65% error）
- ✅ 釐清 C_dpm 是 Σ_e recursion 的 stationary 特例
- ✅ 提出 Σ_e 取代 Eq.13/C_dpmr 的新方法
- ⏸️ 5-state + Σ_e 修正法 — 計畫已寫，待實作

### Key Findings
- Σ_e[k+1] = A[k]Σ_e[k]A[k]' + 4k_BT·a[k]·bb' (R2)
- C_dpm = (Σ_e)_11_ss / (4k_BT·a) 是 stationary 特例
- Time-varying 下無法分離結構(C_dpm)和噪音(sigma2_deltaXT)
- Quasi-static C_dpm[k] 是無記憶近似，a[k] 變化慢時有效
- 新方法：IIR(Var_measured) vs Σ_e(Var_predicted) → 修正 â，不需要 C_dpmr

### File Changes
**Committed (5d1b978):**
- `motion_control_law_4.m` — ekf_out 輸出 L1,L2,L3
- `verify_sigma_recursion.m` (409 lines) — Simulink 驗證腳本
- `temp_variance_recursion.tex` — 數學推導 LaTeX
- `fig_sigma_*.png` (3 files) — 驗證圖
- `verification-notes.md` — 加入 Σ_e 摘要

### ⚠️ Temp Files (未 commit)
- `temp_sigma_phase1.m` — Simulink Phase 1
- `temp_sigma_phase2.m` — Simulink Phase 2
- `temp_sigma_verify_1d.m` — 1D 離散驗證（agent 修過 delay buffer bug）

### Testing Status
✅ Σ_e recursion 數學正確性（stationary: 0.002%）
✅ Σ_e recursion time-varying 預測（MC 500: R^2=0.915）
⬜ 5-state + Σ_e 修正法

### Next Steps
- [x] 實作 5-state + Σ_e 修正法（temp_5state_sigma_verify.m）
- [x] Phase A: 5-state baseline（Eq.13 dual measurement）
- [x] Phase B: 5-state + Σ_e（取代 Eq.13）
- [x] Phase C: 比較 A vs B
- [ ] 清理 temp 檔案

### Issues & Notes
⚠️ temp_sigma_verify_1d.m 被 debug agent 修改（delay buffer bug），未 commit
💡 壁面幾何未知 → 必須從數據估計 a[k]
💡 Quasi-static C_dpm 可能已夠用（最簡單方案）

---

## 2026-04-01 — 5-State + Sigma_e Gain Estimation Verification

### Completed Parts
- ✅ 5-state KF 1D 離散模擬框架（oracle/eq13/sigma_e 三種模式）
- ✅ 修復 delay buffer、KF innovation timing、Fe coupling 等 bug
- ✅ 發現 Fe(3,4)=-f_d circular feedback → 加 fe_clamp
- ✅ 三階段驗證：Oracle(42%,0.74) / Eq.13(42%,0.48) / Sigma_e(46%,0.46)
- ⏸️ Sigma_e 未明顯優於 Eq.13

### Key Findings
- Fe(3,4)=-f_d coupling 是不穩定根源（和 ctrl5 單量測失敗一致）
- 需要 fe_clamp 限制耦合強度才能穩定
- 即使 oracle (a_m=a_true)，RMSE 也有 42% — fe_clamp 限制了 observability
- Sigma_e 和 quasi-static Eq.13 表現接近 → Eq.13 改用 C_dpm(rho) 可能已足夠

### Testing Status
✅ Oracle: RMSE=42.0%, corr=0.74
✅ Eq.13: RMSE=42.1%, corr=0.48
✅ Sigma_e: RMSE=45.6%, corr=0.46
⚠️ Sigma_e 未優於 Eq.13

### Next Steps
- [ ] 分析 fe_clamp 對 observability 的影響
- [ ] Quasi-static C_dpm 取代 K=2 近似（最簡單改進）
- [ ] 在 Simulink ctrl5/ctrl7 中驗證
- [ ] 清理 temp 檔案

### Git
Branch: feat/formula-verification
Last commit: `1dc0061` - docs(analysis): add 5-state Sigma_e gain estimation verification figures
