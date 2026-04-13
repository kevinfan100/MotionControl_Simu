# Verification Notes（跨電腦共享 Context）

## IIR Convention 差異

目前專案和外部專案 (motion_3D_1_2_w12_5) 的 IIR filter convention 不同：

| | LP 公式 | a_pd=0.05 的意義 |
|---|---|---|
| 目前專案 | del_pmd = (1-a_pd)*old + a_pd*new | 5% new, 95% old (slow) |
| 外部專案 | dzm_bar = Avar*old + (1-Avar)*new | 5% old, 95% new (fast) |

C_dpmr 公式基於**目前專案 convention** 推導。外部專案僅供 controller 邏輯參考。

## C_dpmr 修正發現

原始公式缺少 HP filter 的 (1-a_var)^2 prefactor。
修正為 universal，與信號頻譜和控制器架構無關。

## Fe vs F 設計差異

Kalman filter 必須用 Fe（closed-loop error dynamics, (3,3)=1）設計。
用 F（open-loop, (3,3)=lc）會得到次優 gain。
原因：controller 把 error dynamics 的 (3,3) 從 lc 補成 1。

## IIR Finite-Sample Bias (Task 1b, 2026-04-12)

Closed-form for the controller's EMA variance estimator applied to zero-mean
stationary x with autocorrelation ρ(L):

```
E[V_IIR]/γ(0) = 1 - (a_prd/(2-a_prd)) · (1 + 2 · Σ_{L>=1} ρ(L) · (1-a_prd)^L)
```

- White-noise baseline (`ρ(L)=0`): `1 - a_prd/(2-a_prd)` = 2.56% bias at a_prd=0.05.
- `del_pmr` autocorrelation (ρ(1)≈0.85, ρ(5)≈0.08) amplifies ~3.5× → ~9% bias.
- Verified against Phase 2A offline rerun: formula matches empirical within 0.1%
  across a_prd ∈ {0.005, 0.01, 0.02, 0.05, 0.1} when using sample ρ.
- Explains 100% of the residual 7.5% `a_m_z` negative bias left over after
  Phase 1's `C_dpmr_eff` fix.

ρ(L) from `Σ_aug`: `ρ_L = c' A_aug^L Σ_aug c / (c' Σ_aug c)` with
`c(idx_dx_d2)=1, c(idx_pmd_prev)=-1`. Σ_aug predicts ρ that decays slightly
slower than sample → over-predicts bias by ~1pt at a_prd=0.05 (0.9069 vs 0.9162).

**Task 1c (integrated)**: 1-D lookup `bias_factor_lookup.mat` over `lc`,
new Bus element `ctrl.IIR_bias_factor`, controller unbiases
`del_pmr_var /= IIR_bias_factor_const` before noise subtraction.
30 sec Simulink verification (lc=0.7, free space): z-axis
mean(a_m)/a_nom 0.9193 → 1.0137 (6x bias reduction). Residual +1.37%
matches Task 1b's known ~1 pt Σ_aug over-prediction.

Caveat: bias_factor uses Σ_th only. For measurement noise case (non-free-space),
a separate factor for C_np_eff term would be needed.

Artifacts: `reference/for_test/task1b_report.md`, `task1c_report.md`,
`test_script/analyze_task1b_iir_bias.m`, `build_bias_factor_lookup.m`,
`verify_task1c_correction.m`.

## Task 1d Diagnostic (2026-04-13): a_hat gap to paper

Static (lc=0.7, 30s free-space):
- `a_hat_z` rel std = 25.86% (EKF output)
- `a_m_corr_z` rel std = 44.95% (raw IIR, Task 1c applied)
- EKF smoothing factor = 1.74x
- EKF autocorrelation time constant ~52 ms

Dynamic near-wall (lc=0.4, 1 Hz 2.5 um, 10 seeds x 12 s):
- z-axis median |a_hat - a_true|/a_true = **17.88%** (pooled)
- z-axis 90-percentile = 53.26%
- z-axis cross-correlation lag = **+49.6 samples = 31.0 ms** (a_hat lags a_true)
- 3D tracking RMSE = 40.9 nm (matches paper; control is fine)
- Per-seed z-median range: 15.93% - 19.72% (robust)
- x-axis analysis had ground-truth bug (used c_perp instead of c_para); z numbers are correct

Gap decomposition:
- Bias (Task 1c fixed): <1%
- Spread: ~10-13% (chi-squared floor partially smoothed by EKF)
- Dynamic lag: ~7-10% at 11° phase @ 1 Hz
- Total: matches observed 17.88%

Verdict: Task 1c necessary but not sufficient. Architecture change required
because `C_dpmr_eff` is steady-state while true variance is time-varying
under dynamic `a(t)`. Proposed Task 1e: time-varying Σ_aug recursion closed
with `a_hat[k-1]` feedback. Design doc: `task1e_design.md`.

Artifacts: `task1d_diagnostic_report.md`, `task1e_design.md`,
`analyze_task1d_ahat_static.m`, `verify_task1d_paper_benchmark_mc.m`,
`fig_task1d_ahat_vs_am_static.png`, `fig_task1d_paper_benchmark.png`.

## Time-Varying Variance Recursion (Σ_e)

從 Controller 4 + wall effect + trajectory 推導出 4x4 error covariance recursion：
- e[k+1] = A[k]e[k] + q[k]，其中 e = [δx, e_1, e_2, e_3]'
- Σ_e[k+1] = A[k] Σ_e[k] A[k]' + 4k_BT·a[k]·bb'
- C_dpm 是此 recursion 的 stationary 特例
- Controller 用真值 a[k] 時 F_e 為常數；用固定 a_nom 時 F_e 變成 time-varying

驗證結果（1D 離散模擬，a[k] 用真值）：
- Stationary: recursion vs Lyapunov = 0.002%
- Time-varying (MC 500): mean error 5.05%, R^2 = 0.915

## 相關文件索引

- agent_docs/kf-observer-analysis.md — KF observer 完整推導
- agent_docs/literature-review.md — 文獻調查
- reference/for_test/verification_report.md — 驗證報告和數據
- reference/for_test/temp_variance_recursion.tex — Σ_e recursion 數學推導
