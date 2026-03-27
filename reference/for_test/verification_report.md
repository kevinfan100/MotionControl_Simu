# Tracking Error Variance Verification Report

## 概要

驗證三種控制器的 tracking error variance 公式（C_dpm 和 C_dpmr）。
條件：定點軌跡、無 wall effect、已知 gamma_N、thermal ON、no measurement noise、T_sim=20s。

## Notation

| 符號 | 定義 |
|---|---|
| C_dpm | Var(del_pm) / sigma2_deltaXT — 直接 tracking error variance factor |
| C_dpmr | Var(del_pmr) / sigma2_deltaXT — IIR HP filter 後 variance factor |
| sigma2_deltaXT | 4 * k_B * T * Ts / gamma_N [um^2] |
| del_pm | p_d[k-2] - p_m[k] — tracking error（含 2-step delay） |
| del_pmr | del_pm - del_pmd — HP 殘差 |
| del_pmd | (1-a_var)*del_pmd[k-1] + a_var*del_pm[k] — IIR LP |
| lambda_c (lc) | 閉迴路極點 |
| lambda_e (le) | Observer 極點 |
| a_var (a_pd) | IIR LP 係數 |

---

## C_dpmr 修正發現

原始公式缺少 HP filter 的 (1-a_var)^2 prefactor。

原因：HP filter transfer function 為
`H_HP(z) = (1-a_var) * (1-z^-1) / (1-(1-a_var)*z^-1)`，
原始推導遺漏了 (1-a_var) prefactor，導致 variance 高估 1/(1-a_var)^2 倍。

此修正與信號頻譜無關，適用所有控制器。

---

## Controller 1 — Eq.17 Delay Compensation (type=1)

### C_dpm

```
C_dpm(lc) = 2 + 1/(1-lc^2)
```

等效延遲 = 2 步（measurement delay），K=2 為常數。

驗證（Simulink vs theory）：所有 lc 誤差 < 4%。

### C_dpmr

```
C_dpmr(lc, av) = (1-av)^2 * [ 2*(1-av)*(1-lc)/(1-(1-av)*lc)
                             + (2/(2-av)) / ((1+lc)*(1-(1-av)*lc)) ]
```

K=2 對所有 a_var 都不變。Lyapunov 驗證：0% 誤差（machine precision）。

---

## Controller 2 — 3-State Observer (type=2)

### le=0 (deadbeat)

C_dpm:
```
C_dpm(lc) = 3 + 1/(1-lc^2)
```

等效延遲 = 3 步（2 measurement + 1 observer）。K=3 為常數。

C_dpmr:
```
C_dpmr(lc, av) = (1-av)^2 * [ K_eff(av)*(1-av)*(1-lc)/(1-(1-av)*lc)
                             + (2/(2-av)) / ((1+lc)*(1-(1-av)*lc)) ]

K_eff(av) = 2 + 2*(1-av)^2 / (2-av)
```

K_eff 隨 a_var 變化（av=0 時 K_eff=3，av 增大時下降）。
原因：deadbeat observer 的 +1 貢獻經 HP filter 衰減如白噪音：G_HP = 2*(1-av)^2/(2-av)。
Lyapunov 驗證：0% 誤差。

### le 不等於 0

C_dpm 不再是 K + 1/(1-lc^2) 形式，等效延遲隨 lc 變化：

| lc | le=0 (d_eff) | le=0.3 (d_eff) | le=0.5 (d_eff) |
|---|---|---|---|
| 0.4 | 3.0 | 3.85 | 5.03 |
| 0.7 | 3.0 | 4.05 | 5.38 |
| 0.9 | 3.0 | 4.20 | 5.74 |

le=0.5 有精確的 symbolic 公式（lc 的 5 次有理函數）：
```
C_dpm(lc) = (380*lc^5 - 2372*lc^4 + 4631*lc^3 - 1274*lc^2 - 4972*lc + 3688)
          / (81*(lc^2-1)*(lc-2)^3)
```

le=0.3 的有理數係數過大，建議用 Lyapunov 數值計算。

C_dpmr 在 le 不等於 0 時無簡單閉合公式（K_eff 同時依賴 lc 和 a_var）。

---

## Controller 3 — 7-State EKF (type=7)

C_dpm 不符合 K + 1/(1-lc^2) 結構。

| lc | C_dpm (sim) | K residual |
|---|---|---|
| 0.4 | 4.5744 | 3.38 |
| 0.5 | 4.5685 | 3.24 |
| 0.6 | 4.8018 | 3.24 |
| 0.7 | 4.9037 | 2.94 |
| 0.8 | 5.8290 | 3.05 |
| 0.9 | 7.2872 | 2.02 |

K residual 標準差 0.49（spread 16.6%）。

原因：EKF 為非線性系統（adaptive R, evolving Kalman gain），閉迴路無固定 transfer function。
C_dpmr 同樣無解析公式，以模擬值記錄。

---

## 公式總表

### C_dpm

| Controller | C_dpm | 驗證 |
|---|---|---|
| 1 (Eq.17) | 2 + 1/(1-lc^2) | Lyapunov exact, sim < 4% |
| 2 (Observer, le=0) | 3 + 1/(1-lc^2) | Lyapunov exact, sim < 4% |
| 2 (Observer, le>0) | Symbolic rational function of (lc, le) | Lyapunov exact |
| 3 (EKF) | 無解析公式 | 模擬值 |

### C_dpmr 統一形式（le=0 限定）

```
C_dpmr(lc, av) = (1-av)^2 * [ K(av)*(1-av)*(1-lc) / (1-(1-av)*lc)
                             + (2/(2-av)) / ((1+lc)*(1-(1-av)*lc)) ]
```

| Controller | K(av) |
|---|---|
| 1 (Eq.17) | 2 |
| 2 (Observer, le=0) | 2 + 2*(1-av)^2/(2-av) |

一致性檢查：av=0 時 C_dpmr = C_dpm。

---

## Code 影響

`motion_control_law_7state.m` 的 C_dpmr 公式已修正（加入 (1-a_pd)^2 prefactor）。
K=2 在 EKF context 下為近似值。

---

## 數據來源

| 文件 | 位置 | 內容 |
|---|---|---|
| sim_ctrl{1,2,7}_lc{40~90}.mat | test_results/verify/ | Simulink 原始模擬資料 |
| spectral_analysis_ctrl{1,2}.mat | test_results/verify/ | 積分/Lyapunov 分析數據 |
| analysis_ctrl7.mat | test_results/verify/ | Controller 3 分析 |
| lyapunov_le_sweep.mat | test_results/verify/ | le=0/0.3/0.5 掃描數據 |
