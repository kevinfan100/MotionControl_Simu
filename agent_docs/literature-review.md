# Literature Review: Kalman Filter Variance Analysis

## 調查目的

尋找和以下主題相關的學理基礎和論文：
1. EKF/KF 的 tracking error variance 解析描述
2. Kalman gain, observer bandwidth, closed-loop variance 的關係
3. Steady-state Kalman filter 的 Q/R 對 error covariance 的影響
4. IIR filter 對 tracking error 的 variance propagation
5. Observer/estimator 引入的 "等效延遲" 概念

## 關鍵參考文獻

### 教科書

| 作者 | 書名 | 年份 | 相關章節 |
|---|---|---|---|
| Simon, D. | Optimal State Estimation | 2006 | Ch. 5.5: Scalar DARE 閉合公式; Ch. 5.6: 低階系統的顯式 gain 公式 |
| Anderson & Moore | Optimal Filtering | 1979 | Ch. 5: DARE 收斂; Ch. 6: Steady-state gains, pole 分析 |
| Kailath, Sayed, Hassibi | Linear Estimation | 2000 | Ch. 4: Innovation whiteness; Ch. 5: Riccati 解, Q/R 效應 |
| Oppenheim & Schafer | Discrete-Time Signal Processing | 2009 | Ch. 2: 噪音通過 LTI 系統; Ch. 4: Parseval/DTFT |
| Haykin, S. | Adaptive Filter Theory | 2014 | Ch. 3-4: 隨機過程, filter noise gain |
| Brookner, E. | Tracking and Kalman Filtering Made Easy | 2005 | Ch. 2: alpha-beta-gamma 作為 steady-state KF |

### 關鍵論文

| 作者 | 標題 | 年份 | 相關性 |
|---|---|---|---|
| Kalata | The Tracking Index: A Generalized Parameter for alpha-beta and alpha-beta-gamma Target Trackers | 1984 | Q/R → gains 的單參數映射（tracking index），最接近我們 rho→le 的概念 |
| Ekstrand | Poles and zeros of alpha-beta and alpha-beta-gamma Kalman filters | - | Gains → observer poles 的直接映射 |
| Berg-Sorensen & Flyvbjerg | Power spectrum analysis for optical tweezers | 2004 | Fluctuation-based calibration — 從 thermal variance 反推物理參數，和我們的 mobility recovery 高度類似 |
| Benedict & Bordner | Synthesis of optimal tracking filters | 1962 | beta = 2*alpha*(2-alpha) 關係 |

### 微奈米尺度相關

| 作者 | 標題 | 相關性 |
|---|---|---|
| Brenner (1961) | The slow motion of a sphere through a viscous fluid towards a plane surface | Wall effect 垂直修正 |
| Goldman, Cox & Brenner (1967) | Slow viscous motion parallel to a plane wall | 平行/垂直修正 |
| Happel & Brenner (1973) | Low Reynolds Number Hydrodynamics | Wall effect 教科書 |

## 和我們工作的對應

### 推導鏈的理論基礎

| 推導步驟 | 理論 | 參考文獻 |
|---|---|---|
| Q/R → L_ss | Scalar/低階 DARE 閉合公式 | Simon Ch. 5.5; Kalata (1984) |
| L_ss → poles | 特徵多項式 | Ekstrand |
| rho → le | 二次方程解析解 | 我們的推導（基於 Fe 特殊結構） |
| le → C_dpm | Steady-state Lyapunov equation | Anderson & Moore Ch. 5 |
| C_dpm → C_dpmr | IIR filter variance propagation (Parseval) | Oppenheim & Schafer Ch. 2, 4 |
| C_dpmr → mobility | Fluctuation-based calibration | Berg-Sorensen & Flyvbjerg (2004) |

### 文獻中的 Gap

**"poles → le_eff" 映射**（多極點 observer 降成等效單極點用於 variance analysis）
在現有文獻中沒有標準做法。

最接近的概念：Kalata 的 tracking index（單參數描述 alpha-beta-gamma 行為）。

但我們的分析超越了這個 gap：
- Fe-based KF 的 L1=L2=L3 性質讓 le=1-L 是精確的，不需要近似
- rho → le 公式是 3-state delay observer 在 Fe 結構下的精確解析結果
- 這對應 Kalata tracking index 的特定應用

### Fluctuation-based calibration 的類比

| Optical tweezers | 我們的系統 |
|---|---|
| Trap stiffness kappa | Mobility a_z = Ts/gamma |
| Thermal variance = k_B*T/kappa | Tracking error variance = C_dpm * sigma2_deltaXT |
| PSD Lorentzian fit | IIR filter + C_dpmr 公式 |
| 從 variance 反推 kappa | 從 del_pmr variance 反推 a_z (Eq.13) |
