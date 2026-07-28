# 5-state power-law controller — R2 / init / fdet 交接摘要

**Date** 2026-07-27 (Mac) · **Branch** `test/motion-test` · **Commits** `ae41395`, `6dd64a8`, `4db48b3`, `507fdda`, `306c89e` (全部已 push)
**File** `model/controller/motion_control_law_5state_powerlaw.m` · **Spec** `reference/eq17_analysis/derivation/5state_powerlaw_hd.tex`

三個修正，全部是「推導的代數對、但估測器的前提沒稽核」這一類。**沒有任何一個數字是調出來的。**

---

## 1. R2 — a_xm 是 AR(1)，不是白噪  (`ae41395`)

### 問題

從 controller 三行 code 直接推出的**恆等式**（實測殘差 `1.4e-17`，機器精度）：

```
sigma2_dxr_hat[k] = (1-a_cov)*sigma2_dxr_hat[k-1] + a_cov*dx_r[k]^2
a_xm[k]           = (sigma2_dxr_hat[k] - C_n*sig2_n)/(C_dpmr*4kBT)
=>  a_xm[k] = (1-a_cov)*a_xm[k-1] + a_cov*u[k]        pole = 1-a_cov = 0.95
    u[k] := (dx_r[k]^2 - C_n*sig2_n)/(C_dpmr*4kBT),   E[u[k]] = a_h[k-d]
```

推導把 a_xm 當白噪餵 KF，同一份噪聲被重複計入 **(2−a_cov)/a_cov = 39 倍**。

**關鍵區辨（這決定了修法）**：R2 的**邊際變異數是對的**（實測 `Var(a_xm − a_true)/R2 = 0.941`），錯的是**獨立性**。ACF 擬合 pole `0.9457`（預測 0.95），實測資訊高估 **33.6 倍**。

### 修法（無新參數）

```
y2 = a_xm[k] - (1-a_cov)*a_xm[k-1] = a_cov*u[k]
H row2 = a_cov * [0 0 0 1  -Delta_Hbar_d*(s_h/p)]
R2 = a_cov*(2-a_cov)*R2_int + a_cov^2*(sum_{j=1..d}(d-j+1)^2)*Q44     R2_int 不變
```

映射來自 AR(1) 穩態 `Var(a_xm) = a_cov*Var(u)/(2-a_cov)`。順帶修掉 a_xm 的 EWMA 群延遲 `(1-a_cov)/a_cov ≈ 19 步`（原本 H 只退 d=2）。白化後殘餘顏色實測 3.32 ≈ R2_int 裡本來就有的 IF_eff 3.365，自洽。

### 結果（6 seed，z 軸）

| | 前 | 後 |
|---|---|---|
| 下降段 p̂ 擺幅 | 0.2921 | 0.0089 |
| 誠實度（跨 seed 散佈 ÷ 宣稱 σ） | 7.9（過度自信） | 0.4（保守） |
| 撞 clamp | 3/6 seed | 0/6 |

### 驗收：a_cov 不變性

a_cov 是 IIR 內部參數，**不該出現在任何結論裡**。掃 10 倍，各項變動 **< 1.1%**（修正後 0.02%）。

---

## 2. p prior — 由兩個漸近極限釘死  (`ae41395`)

`Pf_p_std` 0.3 → **0.035**。不需要知道 c(h̄)：

- **近壁** Brenner 潤滑 `c_perp → 1/(h̄−1)` ⇒ p = 1（數值驗證 p_eff(1⁺) = 0.9994）
- **遠場** reflections `c_perp − 1 = (9/8)/h̄` ⇒ p = 1（p_eff(∞) = 1.0001）
- prior 只需涵蓋中間內插間隙：**全域 max|p_eff − 1| = 0.034 @ h̄ = 1.22**，與軌跡無關

其中 `p_eff(h̄) := -c'(h̄)·(h̄-1)/(c(h̄)-1)`。

**∥ 軸不適用**：Goldman 近壁是對數不是冪次，p_eff(1⁺) → 0.0055。這是 power law 對平行軸的結構性失效，不是 prior 寬度問題。

---

## 3. fdet — F_e(3,4) 用確定性鏡像  (`6dd64a8`)

### 問題：整流（rectification）

誤差動力學 `δh[k+1] = λc·δh − F_dh·e_ah − ε_h` 用**實現的 f_dh 是恆等式，方程本身沒錯**。錯在把它當 F_e 交給 KF：

```
f_dh[k] ∋ (1-λc)*δh_m[k]/â_h = (1-λc)*(δh[k-d] + n_h[k])/â_h
```

F_dh 裝著 ε_h 裡的同一份熱擾動 → P、L 與 innovation 相關 → `E[L·innov] ≠ 0`。**噪聲乘自己恆為單一符號**，所以累積不平均掉；F_e(3,4) 的負號讓它往下。

### 病徵：不是固定偏差，是不會停的下漂

純定位 a_hat/a_true：`T=2s: 0.9025 → T=5s: 0.8655 → T=12s: 0.8375`，還在走。

**貢獻分解**（純定位 12 s，帳平到小數第四位）：

```
y1 -24.3%  |  y2 +5.6%  |  predict +0.0%     (占 a_true)
```

### 實測否證的三個假設

| 假設 | 否證 |
|---|---|
| 感測噪聲 n_h 內生性 | σ_n 掃 80 倍，偏差不動（0.864 → 0.883，方向還相反） |
| `(1−λc)·a'·δĥ₃` 預測項 | 關掉它，六格結果**逐格相同** |
| 自我一致迴路 | â_h 跌 9.7%，實際閉迴路 trk std 只動 1.7%，a_xm 平在 0.945 無趨勢 |

標度隨 (1−λc)：λc 0.5 / 0.7 / 0.9 → 0.785 / 0.866 / 0.973。

### 修法

```matlab
% 控制律的無噪聲平行副本：delta_h_m := 0，跑自己的歷史 -> 純軌跡驅動、外生
sum_af_det = a_ctrl_km1 .* fdet_km1 + a_ctrl_km2 .* fdet_km2;
f_det      = inv_a_ctrl .* (pd_kp1 - lambda_c*pd - one_minus_lc*pd_km_d ...
                            - one_minus_lc*sum_af_det);
F_dh_det   = f_det + one_minus_lc*(fdet_km1 + fdet_km2);
F_e(3,4)   = -F_dh_det;
```

丟掉的乘性噪聲該進 Q33：`Var(F_noise)·E[e_ah²] ≈ 3e-7` vs `Q33 ≈ 2.4e-4 µm²`，**小三個數量級，不加**。

**先例**：`motion_control_law_eq17_4state.m` 2026-07-13 在 C2 通道踩過同一個坑（註解原文：*"rectifies a_hat downward regardless of noise sign (-70% crash)"*），修法相同，bias −3.49% → −0.37%。那邊還留了 `fe_eval_fdet` probe。

---

## 4. init — 遠場漸近種子，c(h̄) 完全離開 controller  (`6dd64a8`)

```matlab
h_bar_init = max((dot(p0, w_hat) - pz)/R, 1.001);          % 牆位已知
a_h_init   = [a_nom/(1 + (9/16)/h_bar_init);               % x  (∥)
              a_nom/(1 + (9/16)/h_bar_init);               % y  (∥)
              a_nom/(1 + (9/8) /h_bar_init)];              % z  (⊥)
Pf_a_frac  = min(max(5/h_bar_init^2, 0.002), 0.3);         % = 0.0101 @ h_bar_0 = 22.2
```

`(9/8)`、`(9/16)` 是 method of reflections 領頭項 —— **跟釘住 p = 1 是同一份已發表結果**，只需要 h̄₀。

**種子精度**（比對真曲線）：

```
h_bar_0     50      22.2      10        5
漸近種子   0.05%    0.25%    1.23%    4.91%      ~ 1.25/h_bar_0^2
平坦 a_nom 2.30%    5.33%   12.62%   28.51%
```

### prior 必須「匹配」不是「放寬」

| init 值 | Pf_a | 一致? | 定位 12 s |
|---|---|---|---|
| flat a_nom (+5.33%) | 0.01 | ✗ 太緊 | 1.0439（12 秒只修掉 0.9%） |
| flat a_nom (+5.33%) | 0.06 | ✓ | 1.0027 |
| asym (+0.25%) | 0.01 | ✓ | **1.0009** |
| asym (+0.25%) | 0.06 | ✗ 太鬆 | 0.9931 |

兩個壞掉的格子都是**錯配**，不是「prior 太小」。`Pf_a_frac = 5/h̄₀²` 是種子自己的殘差乘 4 倍餘裕。

### 關鍵驗證（沒有這一步，fdet 的結果不能信）

peek init 下 `a_h[0]` 就是真值，所以 fdet ON 拿到 0.998 只證明「沒走開」，**不證明「找得到」**。

換 flat a_nom（+5.33%）重測：**y₂ 單獨一條通道，一秒內把誤差清掉，t=12 收在 1.0027。**

→ **y1 那條路是偏差 100%、可用資訊 0%。** fdet 不是「刪掉問題」，是刪掉一個只會製造偏差的通道。

---

## 5. 綜合結果

**產線預設，6 seed，z 軸**

```
                    before(ae41395)    after(6dd64a8)
純定位 12 s            0.8375            1.0008     (不再漂)
振盪場景               0.9427            1.0178
a_cov 不變性             --              0.02%
跨 seed 散佈           0.0256            0.0380     (+48%)
追蹤 z                 23.8 nm           24.2 nm    (+1.8%)
```

**smoke（seed 7，hold → descend → 1 Hz osc）**

```
axis   追蹤 std     a_hat/a_true    p_hat mean(std)
x      26.77 nm       0.939         0.999 (0.001)
y      26.71 nm       0.913         1.001 (0.004)
z      23.47 nm       1.004         0.990 (0.003)
無 NaN、無 warning、gate 0%
```

**代價**：散佈 +48%、追蹤 +1.8%。都來自拿掉 y1 這條 gain 通道 —— 但那條路帶的是偏差不是資訊。

---

## 6. 現在的 controller（完全沒有作弊點）

```
calc_correction_functions   不在 controller 裡（唯一一次 init peek 已移除）
Q / R                       全部 c-free（只需 â_h, p̂, λc, a_cov, a_pd, kBT, σ_n）
a_h[0]                      a_nom/(1+(9/8)/h̄₀)，只需牆位
Pf_a_frac                   5/h̄₀²
Pf_p_std                    0.035
F_e(3,4)                    -F_dh_det（確定性鏡像）
y2                          白化增量
```

對照：其他 7 支 controller **每一步**都呼叫 `calc_correction_functions` 取 K_h / K_h' 餵 Q。

---

## 7. 開放項

| # | 項目 | 現況 |
|---|---|---|
| 1 | **family-wide `F_e(3,4)` 內生性** | `f_det` 只有 4-state 有（且只用在 C2）。**eq6 / 5state / 5state_aprime / 6state 四支 + powerlaw 修前，全用實現的 f_d** → memory 裡大量歷史 a_hat 數字可能含這份整流 |
| 2 | 振盪場景 z **+1.8%** | 純定位只有 +0.1%，所以與運動有關 |
| 3 | x/y 0.939 / 0.913 | power law 對 ∥ 的結構限制（近壁對數非冪次） |
| 4 | a_nom 校正誤差被放大 `a_o/(a_o−a_h)` | 遠場 19.8×；a_nom 低估 > 5.06% 讓 s_h 翻號。推導沒談 |
| 5 | `1.25/h̄₀²` 係數是離線比真曲線擬合的 | 嚴謹版應查反射級數下一階項係數 |
| 6 | C_dpmr regime 相依 | hold 窗 −7.7%、osc 窗 ~0% |
| 7 | Path C 讓 P(3,3) 低估 | 實測 1.68×（文件預測 2.02×） |

---

## 8. 方法論（同一個缺口出現三次）

| | 推導做對的 | 沒檢查的前提 |
|---|---|---|
| R2 | 正確算出 `Var(σ̂²_δhr)` | a_xm 是不是**白的**（它是 pole 0.95 的 AR(1)） |
| F_e(3,4) | 正確導出誤差動力學 | F_e 是不是**與噪聲獨立**（它裝著 n_h 和熱擾動） |
| Path C | 已知 ε_h 是 MA(d) 有色 | 明知故犯當白噪（有記錄） |

**代數嚴謹，但估測器的前提沒有稽核。** 寫完任何 F_e / R 之後應該固定問一次：**這個矩陣裡有沒有裝著噪聲。**

兩個可重用的驗收工具：

1. **內部參數洩漏 = 模型錯誤的指紋。** 濾波器內部參數（a_cov、a_pd、視窗長度）不該出現在任何結論裡；掃一遍，會動就是有東西寫錯。
2. **P[0] 是預算。** Q = 0 的 state 滿足 `E[(x̂∞ − x̂₀)²] = P[0] − P[∞]`，濾波器不自洽時這個上限會被撐破 —— 它獨立抓到了「只修 prior 不修 R2」那個組態不合法。

---

## 9. 重現方式

> 2026-07-28 整理：全部進版控（舊 temp 路徑對照見 `archive/MOVED.md`）。

```
test_script/integration/run_5state_powerlaw.m             driver（原 temp_run_5state_powerlaw）
test_script/integration/smoke_5state_powerlaw.m           canonical scenario，產推導文件那張圖的資料
test_script/integration/plot_5state_powerlaw.m            產 figures/powerlaw_5state_sim.png
test_script/integration/verify_powerlaw_regress_final.m   產線 regression + a_cov 不變性驗收
test_script/integration/verify_powerlaw_regress_A12.m     A1/A2 修復的驗收（a_cov 不變性）
test_script/integration/plot_p_prior_origin.m             p prior 0.035 來源圖
test_script/integration/verify_blocal_exp.m               產 figures/blocal_exp_check.png
test_script/scratch/temp_mcl_powerlaw_diag.m              instrumented fork（9 knob + P45/K2 分解 log）
test_script/scratch/temp_run_powerlaw_diag.m              上面那支的 driver
```
