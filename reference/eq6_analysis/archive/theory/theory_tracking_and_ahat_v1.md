# Theoretical Prediction: Tracking Variance & a_hat Std (v1, 2026-04-20)

**Scope**: positioning, controller_type=7, frozen_correct preset (P5+P6 controller).
**Reference data**: P6 verification (3 seeds × 2 scenarios), `qr_p6_b{1,2}.mat`

---

## Notation

| 符號 | 定義 | 單位 |
|---|---|---|
| Ts | 取樣週期 = 1/1600 | s |
| gamma_N | Stokes drag (free-space) = 6πη·R | N·s/m |
| a_nom | Ts/gamma_N (free-space gain) | um/pN |
| sigma2_dXT | 4·k_B·T·a_nom (thermal force impulse 變異, 單軸) | um² |
| sigma2_n | sensor noise variance per axis = (meas_noise_std)² | um² |
| ar | a_axis/a_nom = 1/c_para (x,y) 或 1/c_perp (z), 隨 h_bar 變 | 無因次 |
| lc | 閉迴路極點 = 0.7 | 無因次 |
| a_pd | IIR LP smoothing = 0.05 | 無因次 |
| a_cov | IIR variance EMA smoothing = 0.005 | 無因次 |
| Cdpmr_eff(lc, ar) | Lyapunov 預算 thermal contribution (查表) | 無因次 |
| Cnp_eff(lc, ar) | Lyapunov 預算 sensor noise contribution (查表) | 無因次 |
| chi_sq | 2·a_cov/(2-a_cov) | 無因次 |
| rho_a | autocorrelation amplification (≈ 4) | 無因次 |

---

## Part 1: Tracking Error Variance Theory

### 1.1 What current theory captures

`compute_7state_cdpmr_eff.m` 解 11-dim 增廣 Lyapunov, 含:
- True tracking error `delta_x` 的 closed-loop 動態
- 7-state EKF estimation errors (e1..e7)
- Position delay chain (dx_d1, dx_d2)
- IIR LP state (pmd_prev)

雜訊輸入 (4 個獨立來源):
1. Thermal force `a·f_T` → drives delta_x and e3
2. Position measurement noise `n_p` → drives EKF errors and IIR
3. Q(6,6) process noise on `a`
4. Q(7,7) process noise on `del_a`

**未含**:
- Gain measurement noise `n_a` 對 EKF errors 的注入 (B_na 算了但沒進 Sigma 解)
- f_d 非零的耦合 (預設 f0=0)
- a_hat noise 反饋進 control law 的二階效應

### 1.2 Formula (per axis)

```
Var(del_pmr)_theory = Cdpmr_eff(lc, ar) · sigma2_dXT · ar
                    + Cnp_eff(lc, ar)   · sigma2_n
```

Cdpmr_eff/Cnp_eff 從 lookup `cdpmr_eff_lookup.mat` interp at (lc, ar).

`del_pmr` 是 IIR HP residual:
```
del_pmr[k] = (1-a_pd) · (del_pm[k] - pmd_prev[k])
```

### 1.3 從 del_pmr 到 tracking error 的轉換

對 positioning, `del_pm[k] = pd - p_m[k] = -tracking_error[k]`.

del_pmr 是 del_pm 的 HP 部分。對 thermal-dominated 寬頻噪訊 (能量在 6 Hz 以上, IIR cutoff a_pd=0.05 處), HP 通過大部分能量但有 (1-a_pd) 衰減:

```
Var(del_pm) ≈ Var(del_pmr) / (1-a_pd)²    (for thermal-dominated spectrum)
            = Cdpmr_eff·sigma2_dXT·ar / (1-a_pd)² + Cnp_eff·sigma2_n / (1-a_pd)²
```

但 Cdpmr_eff 已經把 (1-a_pd)² 因子吸收進去了 (見 compute_7state_cdpmr_eff.m line 199):
```matlab
gain_sq = (1 - a_pd)^2;
C_dpmr_eff = gain_sq * var_state_th;
```

所以實際公式:
```
tracking_std_axis ≈ sqrt(Var(del_pmr)) / (1-a_pd)   ≈ theory_std_dpmr · 1.053
```

### 1.4 預測 vs 實測 (P6, 3 seeds 平均)

| 場景 | 軸 | theory_std_dpmr (nm) | 修 (1-a_pd) (nm) | 實測 (nm) | 比值 (實/修) |
|---|---|---|---|---|---|
| h=2.5 | X | 21.57 | 22.71 | 23.76 ± 0.55 | 1.046 |
| h=2.5 | Y | 21.57 | 22.71 | 23.91 ± 0.19 | 1.053 |
| h=2.5 | Z | 11.01 | 11.59 | 11.75 ± 0.14 | **1.014** |
| h=50 | X | 31.65 | 33.32 | 35.66 ± 0.90 | 1.070 |
| h=50 | Y | 31.65 | 33.32 | 35.88 ± 0.29 | 1.077 |
| h=50 | Z | 31.26 | 32.91 | 35.46 ± 0.53 | 1.078 |

**修正後**: gap 從 +10-13% 縮到 +1.4-7.8%。h=2.5 z-axis 已**完美匹配** (1.4%)。

剩餘 ~5-7% gap 在 free-space (h=50) 三軸 + h=2.5 x/y。**這個 gap 是真正的「未模型化」部分**。

### 1.5 缺失部分推導: a_hat noise → f_d 反饋

#### 1.5.1 問題機制

控制律 (paper Eq.17 變形, line 215):
```
f_d = (1 / a_hat) · [ del_pd + (1-lc)·del_p3_hat - d_hat ]
```

對 positioning, `del_pd = 0`, `d_hat → 0` (穩態), 所以:
```
f_d ≈ (1-lc) · del_p3_hat / a_hat
```

如 a_hat = a_true·(1+δ) 其中 δ = (a_hat - a_true)/a_true (相對誤差):
```
f_d ≈ (1-lc) · del_p3_hat / [a_true·(1+δ)]
    ≈ (1-lc) · del_p3_hat / a_true · (1 - δ)         (一階展開)
    = f_d_ideal · (1 - δ)
```

「多餘」的力 = `Δf_d = -f_d_ideal · δ`. 這個 Δf_d 由 EKF gain 誤差**乘上**正常 control 信號決定, **時變**且與 del_p3_hat 自相關。

#### 1.5.2 對 tracking 變異的貢獻

closed-loop 系統:
```
delta_x[k+1] = lc·delta_x[k] + (1-lc)·e3[k] - e4[k] + a_true·(f_T[k] - Δf_d[k])
```

其中 `Δf_d[k] = -(1-lc)·del_p3_hat[k]/a_true · δ[k]`.

代入:
```
delta_x[k+1] = lc·delta_x[k] + (1-lc)·e3[k] - e4[k] + a_true·f_T[k] + (1-lc)·del_p3_hat[k]·δ[k]
```

最後一項是新增的耦合 (a_hat noise × del_p3_hat)。**計算其變異需 a_hat 與 del_p3_hat 的 cross-covariance**, 兩者都是 closed-loop 變量, 通常**不獨立**。

#### 1.5.3 簡化估計 (假設 δ 與 del_p3_hat 獨立)

```
Var( extra term ) = (1-lc)² · Var(del_p3_hat) · Var(δ)
```

對 frozen_correct:
- (1-lc)² = 0.09
- Var(del_p3_hat) ≈ Var(del_p3) ≈ Cdpmr·sigma2_dXT·ar (close to del_pmr 變異)
- Var(δ) = (a_hat std as fraction)² → e.g. 0.045² = 2e-3 for h=2.5 z, 0.017² = 3e-4 for h=50

對 h=2.5 z (Var(del_p3) ≈ 1.1e-4 um², Var(δ)=2e-3):
- Var(extra) = 0.09 · 1.1e-4 · 2e-3 = 2e-8 um² → std = 1.4e-4 um = 0.14 nm

跟測得 ~10 nm 比, 才 1.4% 相對。**太小, 不能解釋 +5% gap**。

對 h=50 z (Var(del_p3) ≈ 9.5e-4, Var(δ)=3e-4):
- Var(extra) = 0.09 · 9.5e-4 · 3e-4 = 2.6e-8 → std = 1.6e-4 um = 0.16 nm
- 跟測得 35 nm 比, 0.5% 相對 → **更不能解釋**

#### 1.5.4 結論: a_hat 反饋**不是**主要 gap 來源

對 frozen_correct (a_hat 雜訊 1-5%), 反饋對 tracking std 的貢獻 < 0.5%. **+5% gap 來自其他**:

候選:
1. **f0 ≠ 0 in steady state**: 控制力非零 → Fe(3,6)·f0 項激活, 11-dim Lyapunov 設 f0=0 漏了
2. **EKF Riccati 沒收斂到 designer's L_ss**: 實際 EKF 的 Pf 受 IIR 影響可能未達理論穩態
3. **Var(del_pmr) ≠ Var(del_pm)/(1-a_pd)² 嚴格等式失效**: 真正的 spectrum shape 影響轉換因子
4. **discretization errors** 在連續/離散介面 (整合器 ode4 vs 離散 control)

### 1.6 改良預測公式 (proposed)

```
tracking_std_axis = sqrt(Var(del_pmr)/(1-a_pd)² + Var_extra) · 1000 [nm]
Var_extra ≈ (1-lc)² · Var(del_p3) · Var(δ)
         ≈ 0 (negligible for frozen_correct)
```

**目前**: theory_std × 1.053 修正後仍有 +5-7% gap (h=50, h=2.5 x/y 軸), **未完全解釋**。需更深入: 解 augmented Lyapunov 含 f_d ≠ 0 項 + a_hat-control 耦合二階項。

---

## Part 2: a_hat Estimation Std Theory

### 2.1 What current theory captures

**Chi-sq chain (Task 1d Appendix A)**:
1. IIR EMA estimates Var(del_pmr) with chi-sq error
2. a_m derived from del_pmr_var by linear formula
3. a_hat = LP filter (Kalman gain L) of a_m

對 **empirical** (a_cov=0.05, Q=1e-4, L_ss=0.01): chain 給 19% 預測 ≈ 19-21% 實測 ✓

對 **frozen** (a_cov=0.005, Q=1e-8, L_ss=1e-4): 需精細化, 因 L_ss 極小, a_hat 接近常數但非完全。

### 2.2 缺失部分推導: closed-loop Lyapunov on (a_hat - a_true)

對純量近似 (忽略 EKF cross-coupling):

#### 2.2.1 模型
```
true:  a[k+1] = a[k]                      (positioning, a 不變)
KF model: a_hat[k+1|k] = a_hat[k|k] + 0,  with Q(6,6) injected
measurement: a_m[k] = a[k] + noise_a[k]
```

`noise_a` 雜訊變異 = chi_sq · rho_a · a²:
```
Var(noise_a) = (2·a_cov/(2-a_cov)) · rho_a · a_true²
             = chi_sq · rho_a · a_true²       (per axis)
```

對 a_cov=0.005, rho_a=4: chi_sq · rho_a = 0.02 → std(noise_a) = 0.141·a_true (14.1% relative).

#### 2.2.2 Kalman 解 (純量)
更新方程:
```
a_hat[k+1] = (1-L)·a_hat[k] + L·a_m[k]
           = (1-L)·a_hat[k] + L·a_true + L·noise_a[k]   (positioning)
```

定義 `e[k] = a_hat[k] - a_true`:
```
e[k+1] = (1-L)·e[k] + L·noise_a[k]
```

LP filter 變異:
```
Var(e) = L²·Var(noise_a) / (1 - (1-L)²)
       = L²·Var(noise_a) / (2L - L²)
       ≈ L/2 · Var(noise_a)              (for small L)
```

#### 2.2.3 加上 Q 驅動的 random walk

實際 Riccati 解 (純量穩態):
```
P_ss = [Q + sqrt(Q² + 4·Q·R_designer)] / 2 ≈ sqrt(Q·R_designer)   (Q << R)
L_ss = P_ss / (P_ss + R_designer) ≈ sqrt(Q/R_designer)             (Q << R)
```

對 frozen_correct:
- Q_abs = sigma2_dXT · 1e-8
- R_designer_abs = sigma2_dXT · 1.0
- L_ss = sqrt(1e-8 / 1.0) = **1e-4**
- P_ss = sigma2_dXT · sqrt(1e-8 · 1.0) = sigma2_dXT · 1e-4

如**用 designer's R** (Riccati 結果): `Var(e) = P_ss = sigma2_dXT · 1e-4`

如 sigma2_dXT ≈ 2.5e-4 (um/pN)²:
- Var(e) = 2.5e-4 · 1e-4 = 2.5e-8
- std(e) = 1.58e-4 um/pN
- relative std (h=50, a_true_z=0.014): **1.13%**
- relative std (h=2.5, a_true_z=0.0015): **10.5%**

#### 2.2.4 但實際 a_m 噪訊 ≠ designer's R!

設計用 R = sigma2_dXT · 1.0, 但**實際** Var(noise_a) = chi_sq · rho_a · a_true² (per axis, 對 a 大小敏感).

對 a_cov=0.005, rho_a=4: actual_R = 0.02 · a_true²

| 場景 | a_true_z | actual_R | designer_R (≈sigma2_dXT) | actual/designer |
|---|---|---|---|---|
| h=50 | 0.014 | 0.02·0.014² = 3.92e-6 | 2.5e-4 | 0.016 (62×小於 designer) |
| h=2.5 | 0.0015 | 0.02·0.0015² = 4.5e-8 | 2.5e-4 | 1.8e-4 (5500×小於) |

**Designer 過度估了 a_m noise** (因 R = sigma2_dXT 是 thermal scale, 不是 chi-sq scale)。所以:
- L_ss (用 designer R) = 1e-4 是「保守」過低
- 實際 a_hat 比 Riccati 預測**更被信號拉住**, 但**也更慢吸收 a_m**
- 結果: Var(a_hat - a_true) 介於兩者之間, **per-scenario** 變化

#### 2.2.5 修正預測 (with mismatched R)

對 sub-optimal Kalman with designer L_ss but actual noise:
```
Var(e) ≈ L_ss/2 · actual_R      (LP filter 變異)
       = 1e-4/2 · 0.02·a_true²
       = 1e-6 · a_true²
relative std = sqrt(1e-6) = 0.001 = 0.1%       (per axis, **不依賴 a_true**)
```

太小 (預測 0.1% vs 測得 1.7-4.5%). **缺漏 Q 驅動 random walk**.

#### 2.2.6 完整 Var(e) for frozen

對 sub-optimal Kalman + 實際 a_m noise + Q 驅動:
```
Var(e)_total = Var(e)_a_m_noise + Var(e)_Q_drift
             = L_ss/2·actual_R   +   Q_abs/(2·L_ss) · scaling
```

第二項 (Q-drift):
```
e[k+1] = (1-L)·e[k] + L·noise_a + delta_a[k]
```
其中 delta_a 是 process noise (Q(6,6) 注入). 純量 LP 對 Q 噪訊:
```
Var(e)_Q ≈ Q / (2L - L²) ≈ Q/(2L)    for small L
```

對 frozen_correct:
- Q_abs = sigma2_dXT · 1e-8 = 2.5e-4 · 1e-8 = 2.5e-12
- L_ss = 1e-4
- Var(e)_Q = 2.5e-12 / (2·1e-4) = 1.25e-8 (um/pN)²
- std(e)_Q = 1.12e-4 um/pN
- relative (h=50): 1.12e-4/0.014 = **0.8%**
- relative (h=2.5): 1.12e-4/0.0015 = **7.5%**

加 a_m noise 部分 (0.1%): 總共
- h=50: sqrt(0.8² + 0.1²) ≈ **0.81%**
- h=2.5: sqrt(7.5² + 0.1²) ≈ **7.5%**

**對比實測**: h=50 1.7%, h=2.5 4.5%. **預測 h=50 偏小 (因子 2), h=2.5 偏大 (因子 1.7)**.

### 2.3 為何預測仍不完美?

可能原因:
1. **rho_a = 4 是粗估**: 實際 autocorrelation 可能 per-scenario 變
2. **Pf(6,6) 實際非 P_ss**: 從 init 1e-5 衰減到 P_ss = 2.5e-8, 但**不一定到達** (時間常數很慢)
3. **EKF cross-coupling**: state 6 (a) 跟 state 7 (del_a) 耦合, 純量近似漏
4. **transient memory**: 早期 chi-sq spike 在 a_hat 留 permanent shift, 看起來像 std 但實是 bias 隨時間飄
5. **wall-effect lookup 準確度**: ar 算錯一點會系統性偏

### 2.4 改良預測公式

對 frozen positioning:
```
a_hat std (relative) ≈ sqrt[ Q_abs/(2·L_ss·a_true²) + L_ss/2 · chi_sq · rho_a ]
                     = sqrt[ sigma2_dXT/2 · 1e-8/L_ss / a_true² + L_ss/2 · 0.02 ]
                     = sqrt[ Q-drift term + chi-sq term ]
```

**Q-drift term**: dominant (因 L_ss 極小放大 Q 的隨機行走)
**chi-sq term**: negligible for frozen

實作: 用 lookup 表裡的 L_ss 而非簡化 sqrt(Q/R), 加 Pf init 衰減校正, 加 EKF 二維 Riccati.

---

## Part 3: 整體理論狀態總結

### 3.1 OK 的部分

| 量 | 理論方法 | 嚴謹度 | 預測準度 |
|---|---|---|---|
| Tracking std (含 (1-a_pd) 修正) | 11-dim Lyapunov | 完整 (含 EKF, IIR, delay) | **+5-7%** |
| a_hat std (empirical, high Q) | Chi-sq chain (a_m ≈ a_hat) | 半解析 | TIGHT 19-21% |
| a_hat std (frozen) Q-drift | LP variance Q/(2L) | 純量近似 | range 對 (0.8-7.5% vs 測 1.7-4.5%) |

### 3.2 不夠的部分 (需要做)

| 缺口 | 影響 | 推導方向 |
|---|---|---|
| Tracking f0≠0 修正 | +5% gap (h=50 三軸, h=2.5 xy) | 12-dim Lyapunov 含 f_d 反饋 |
| a_hat 與 actual_R 失配 | predict h=2.5 偏大 | 用 actual_R = chi_sq·rho_a·a_true² 重解 Riccati |
| a_hat Pf 未達 P_ss | 殘餘 transient | 含 Pf init 衰減的 time-varying 解 |
| EKF state 6/7 cross-coupling | per-axis 系統偏 | 完整 7-state Riccati (非純量) |
| rho_a per-scenario | a_m noise 估錯 | 從 11-dim Sigma 抽 a_m 自相關 |

### 3.3 工程建議

**目前可寫 thesis 的精度等級**:
- Tracking std: theory ≈ measured 在 **±10% 內** (good engineering)
- a_hat_z std: theory 預測 frozen 範圍對 (1-10%), 數值對 free-space, 偏大 near-wall

**推薦在 thesis 寫**:
> "Closed-loop tracking variance 透過 11-dim 增廣 Lyapunov 預測, 與模擬實測差距 +5-7%, 主要源於 a_hat estimation noise 經控制律的二階回饋, 以及 f_d ≠ 0 的耦合項; 兩者均為下一階優化目標。a_hat estimation std 在 frozen 模式下由 Q 驅動的 random walk 主導, 半解析 LP filter 模型給出 0.8-7.5% 的 free-space-vs-near-wall 範圍, 與實測 1.7-4.5% 量級一致。"

### 3.4 短期可做的細化 (~2-4 hours each)

1. **啟用 IIR_bias_factor lookup** (existing): 看 +5% gap 是否縮
2. **Per-axis Cdpmr/Cnp 重 interp** (現用標量): 確認 h=2.5 x/y/z 差別
3. **Pf time-varying 模擬**: 測 transient 期 a_hat std vs steady 後比例
4. **f0 ≠ 0 12-dim Lyapunov** (extend compute_7state_cdpmr_eff): 完整解 closed-loop

### 3.5 中期 (~1 day each)

1. **a_hat full closed-loop Lyapunov**: 把 (a_hat - a_true) 加進 augmented state, 含 chi-sq 噪訊和 Q
2. **Per-axis EKF (3 個獨立 Riccati)**: 支援 actual_R 而非 designer_R
3. **rho_a 從 Sigma 抽**: 解 a_m 的 autocorrelation, 得到精確 chi-sq amplification

---

## Source files

- `compute_7state_cdpmr_eff.m` (11-dim Lyapunov 引擎)
- `build_cdpmr_eff_lookup.m` (預算 Cdpmr/Cnp 表)
- `verify_qr_positioning_run.m` (run_one 內 theory_std_dpmr 計算, line 188-220)
- `qr_p6_b{1,2}.mat` (P6 實測, source for ratios)
- `reference/for_test/ahat_quality_prediction.md` (chi-sq chain 文件)
- `reference/for_test/qr_theoretical_values.md` (整體推導框架)

---

## TODO

- [ ] 用 IIR_bias_factor lookup 啟用後重測, 看 +5% gap 是否縮小
- [ ] 做 Pf 時變模擬, 確認 a_hat_z 在前 5s 是否與後 10s 變異不同 (transient memory)
- [ ] 推 12-dim Lyapunov (含 f_d 反饋), 算 corrected tracking std
- [ ] 推 augmented (a_hat - a_true) Lyapunov, 算 frozen 模式 a_hat std
