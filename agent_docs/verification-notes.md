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

HP filter: H_HP(z) = (1-a_var)*(1-z^-1)/(1-(1-a_var)*z^-1)

原始推導遺漏了 (1-a_var) prefactor，導致 variance 高估 1/(1-a_var)^2。
修正為 universal，與信號頻譜和控制器架構無關。

### 正確公式

C_dpmr(lc, av) = (1-av)^2 * [K(av)*(1-av)*(1-lc)/(1-(1-av)*lc) + (2/(2-av))/((1+lc)*(1-(1-av)*lc))]

- Controller 1 (Eq.17): K = 2 (constant)
- Controller 2 (Observer, le=0): K_eff(av) = 2 + 2*(1-av)^2/(2-av)
- Controller 3 (EKF): 無解析公式

### le_eff 概念

Deadbeat observer (le=0): C_dpm 完美分解為 K + 1/(1-lc^2)，K=3 constant。
Non-deadbeat (le>0): 分解不成立，等效延遲隨 lc 變化。
le_eff（等效 observer pole from variance analysis）在現有文獻中未見標準做法，可能是 novel contribution。

## 文獻搜尋摘要

### 關鍵參考

| 作者 | 標題 | 相關性 |
|---|---|---|
| Simon, D. (2006) | Optimal State Estimation, Ch. 5.5 | Scalar DARE 閉合公式 |
| Kalata (1984) | The Tracking Index, IEEE Trans. AES | Q/R → gains 的單參數映射 |
| Ekstrand | Poles and zeros of alpha-beta-gamma filters | Gains → observer poles 映射 |
| Anderson & Moore (1979) | Optimal Filtering, Ch. 5-6 | DARE 收斂、steady-state gains |
| Berg-Sorensen & Flyvbjerg (2004) | Power spectrum analysis for optical tweezers | Fluctuation-based calibration (和我們的 mobility recovery 類似) |
| Oppenheim & Schafer (2009) | Discrete-Time Signal Processing, Ch. 2, 4 | Parseval, IIR variance propagation |

### 我們要建立的推導鏈

```
Q/R → DARE → L_ss → observer poles → le_eff → C_dpm(lc, le_eff) → C_dpmr(lc, le_eff, av)
```

每一步的理論基礎：
- Q/R → L_ss: Riccati equation (Simon Ch. 5.5, Kalata tracking index)
- L_ss → poles: 特徵多項式 (Ekstrand)
- poles → le_eff: 單極點近似（可能為 novel contribution）
- le_eff → C_dpm: Lyapunov equation (Anderson & Moore)
- C_dpm → C_dpmr: IIR variance propagation + (1-av)^2 correction (Parseval)

### Gap

"poles → le_eff" 的映射（多極點 observer 降成等效單極點）在文獻中沒有標準結果。
最接近的是 Kalata 的 tracking index（單參數描述 alpha-beta-gamma），
但明確用於 closed-loop variance analysis 的 le_eff 映射不是標準做法。
