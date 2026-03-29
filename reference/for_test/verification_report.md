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

---

## Controller 4 — 3-State KF Observer (type=4)

用 Kalman filter 取代 pole-placement 設計 observer gain。

### Fe vs F 設計差異

Kalman filter 必須用 Fe（closed-loop error dynamics, (3,3)=1），
不能用 F（open-loop state dynamics, (3,3)=lc）。

用 F 設計會得到次優 gain（C_dpm 偏大 30-140%），
因為 KF 不知道 controller 的存在。

### rho → le 解析公式

```
rho = R / sigma2_deltaXT    (量測噪音 / thermal noise 比例)
a = (1 + sqrt(1 + 4*rho)) / 2
L = 1/a = 2 / (1 + sqrt(1 + 4*rho))
le = 1 - L = (sqrt(1+4*rho) - 1) / (sqrt(1+4*rho) + 1)
```

Fe-based KF 的特殊性質：L1=L2=L3=L（三個 gain 相等），
observer eigenvalues = (0, 0, le)，部分 deadbeat。

### Simulink 驗證（lc=0.7）

| rho | le | C_dpm(sim) | C_dpm(theory) | Error |
|---|---|---|---|---|
| 0.01 | 0.010 | 5.04 | 4.97 | +1.4% |
| 0.1 | 0.084 | 4.97 | 5.04 | -1.3% |
| 1 | 0.382 | 5.42 | 5.44 | -0.5% |
| 10 | 0.730 | 6.47 | 6.81 | -5.0% |
| 100 | 0.905 | 10.88 | 10.55 | +3.1% |

### KF vs Pole-placement

| | Fe-based KF | Pole-placement |
|---|---|---|
| Observer eigenvalues | (0, 0, le) | (le, le, le) |
| le=0 | 相同（deadbeat） | 相同 |
| le>0 | C_dpm 更小 | C_dpm 更大 |

### C_kf 分解

```
C_dpm = C_kf(lc, le) + 1/(1-lc^2)

C_kf(lc, le) = [2*lc^2*le^3 - 3*lc^2*le + 2*lc*le^3 - 4*lc*le^2 - lc*le + 3*lc - 2*le^2 + 3]
             / [(le^2-1)(lc*le-1)(lc+1)]

C_kf(lc, 0) = 3
```

### 含量測噪音驗證（r=0.0435, z 軸）

sigma_n = 0.00331 um, r = sigma2_n / sigma2_deltaXT = 0.0435

理論 Lyapunov 含噪音項: Q_total = B_th*B_th'*sigma2_deltaXT + B_meas*B_meas'*sigma2_n

| lc | C_dpm(sim) | C_dpm(theory) | Error |
|---|---|---|---|
| 0.4 | 4.345 | 4.232 | +2.7% |
| 0.5 | 4.331 | 4.375 | -1.0% |
| 0.6 | 4.625 | 4.604 | +0.5% |
| 0.7 | 4.855 | 5.003 | -3.0% |
| 0.8 | 5.978 | 5.820 | +2.7% |
| 0.9 | 8.208 | 8.305 | -1.2% |

### 專案量測噪音對應的 r

| 軸 | noise_std [um] | r | le | C_kf (lc=0.7) |
|---|---|---|---|---|
| x | 0.00062 | 0.0015 | 0.002 | 3.001 |
| y | 0.000057 | 0.00001 | ~0 | 3.000 |
| z | 0.00331 | 0.0435 | 0.040 | 3.034 |

r < 0.05 → C_kf 接近 3 → 和 deadbeat 差 < 1%

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
| riccati_analysis.mat | test_results/verify/ | KF rho sweep 數據 |
| verify_kf_ctrl4.mat | test_results/verify/ | KF Simulink 驗證數據 |
