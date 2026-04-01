# KF 內部分析總結

## 系統概要

探針在水中靠近壁面，drag coefficient gamma(h) 隨位置變化。
motion gain a[k] = T_s / gamma(h[k]) 需要即時估計。

控制律: f_d = (1/a_hat) × (delta_p_d + (1-lambda_c) × delta_x_hat_3)

---

## 1. KF 估計 a 的兩條資訊通道

### 通道 1: Dynamics（del_pm → a_hat）

a 影響 plant 的響應。a 錯 → f_d 效果不對 → tracking error 有系統性 pattern。

數學機制:
- Fe(3,4) = -f_d 耦合 → P^f 產生 cross-covariance P^f(4,1) → L(4,1) != 0
- 強度正比於 f_d。f_d=0 時通道關閉。

### 通道 2: Variance（IIR → a_m → a_hat）

thermal variance 正比於 a: Var(del_pmr) = C_dpmr × 4k_BT × a。
量 variance → 反推 a。

數學機制:
- IIR 計算 Var_measured（獨立於 a_hat，數據驅動）
- Eq.13: a_m = Var_measured / (C_dpmr × 4k_BT)
- a_m 作為量測輸入 KF

**通道 2 的 a_m 獨立於 a_hat** — 這是它比 innovation-based 方法好的原因。

---

## 2. C_dpmr 的問題與修正

### 問題

論文的 Eq.13 用 K=2 計算 C_dpmr:
  C_dpmr(K=2) = 3.16 (lc=0.7, a_pd=0.05)

但 Controller 4 (KF observer) 的正確 C_dpmr:
  C_dpmr(correct) = 4.32 (lc=0.7, a_pd=0.05, rho=0.05)

K=2 低估 C_dpmr 27% → a_m 高估 37%。

### 修正: quasi-static C_dpmr(rho)

C_dpmr 是 rho 的函數，其中 rho = sigma^2_n / (4k_BT × a_hat)。

計算方法: 7-state augmented Lyapunov
- 7 states: [del_p1, del_p2, del_p3, e_1, e_2, e_3, del_pmd]
- del_pmd 是 IIR LP 的 state
- Var(del_pmr) = P(1,1) + P(7,7) - 2×P(1,7)
- C_dpmr = Var(del_pmr) / (4k_BT × a) — 可以 precompute 為 lookup table

C_dpmr 對 a_hat 的敏感度很低（a_hat 錯 2 倍，C_dpmr 只差 ~5%）。
→ 即使 a_hat 不準確，a_m 仍然基本正確。

### 驗證結果

1D 離散模擬，3-state observer + IIR a_m:

| 方法 | RMSE | Corr | Bias |
|---|---|---|---|
| Oracle (a_m = a_true) | 13.4% | 0.77 | 1.02 |
| K=2 | 34.9% | 0.35 | 1.20 |
| Correct C_dpmr(rho) | 27.2% | 0.45 | 0.85 |

C_dpmr(rho) 比 K=2 改善 8 個百分點。bias 從 +20% 降到 -15%。

---

## 3. Sigma_e Recursion 框架

### 定義

4-state error vector: e = [delta_x, e_1, e_2, e_3]'

Error dynamics: e[k+1] = A[k] e[k] + q[k]

Covariance recursion:
  Sigma_e[k+1] = A[k] Sigma_e[k] A[k]' + 4k_BT × a[k] × b×b'

其中 b = [-1; 0; 0; -1]

### 意義

(Sigma_e)_11[k] = 在時刻 k，tracking error 的理論 variance。

### 和 C_dpm 的關係

Stationary case:
  Sigma_e → Lyapunov fixed point → C_dpm = (Sigma_e)_11_ss / (4k_BT × a)

Time-varying case:
  C_dpm 不存在。Sigma_e recursion 是完整描述。

### 驗證

- Stationary: recursion vs Lyapunov = 0.002%
- Time-varying (MC 500): mean error 5.05%, R^2 = 0.915

---

## 4. Q/R 的物理推導

### Q（process noise）

Q(3,3) = 4k_BT × a[k]
  → thermal force 對 e_3 的驅動（物理值）

Q(4,4) = (da/dh × dh/dt × T_s)^2
  → gain 每步的變化量（從軌跡 + wall model 計算）

### R（measurement noise）

R(1,1) = sigma^2_n
  → visual tracking noise（已知）

R(2,2) = Var(a_m - a_true)
  → 取決於 a_m 的計算方法
  → K=2: R(2,2) 大（包含 C_dpmr 偏差）
  → Correct C_dpmr: R(2,2) 小

### Innovation 診斷

設好 Q/R 後，用 innovation 驗證:
  S_predicted = P^f(1,1) + R
  S_actual = IIR(innov^2)

  S_actual ≈ S_predicted → Q/R consistent
  不匹配 → 調整物理推導中的假設

---

## 5. 嘗試過的方法與結論

### 方法 A: Innovation-based a_hat update（取代 Eq.13）

a_m = a_hat × S_actual / S_predicted

結果: RMSE=24%, corr=0.79
問題: a_m 依賴 a_hat（自我參照）→ 系統性 bias 20%

**結論: innovation-based 的 a_m 不提供獨立資訊，不如 IIR。**

### 方法 B: Joint 5-state KF

所有 state 在一個 KF: [delta_x_1..3, a_hat, delta_a_hat]

結果: Fe(3,4) = -f_d 耦合 → Pf 爆炸 → 不穩定
加 Fe clamp → 穩定但 observability 被限制 → 42% RMSE

**結論: joint KF 的 Fe 耦合是架構性問題。**

### 方法 C: IIR + K=2（論文現有方法）

結果: RMSE=35%, bias=+20%
問題: K=2 低估 C_dpmr 27%

### 方法 D: IIR + correct C_dpmr(rho)（改進方法）

結果: RMSE=27%, bias=-15%
改善: 比 K=2 好 8 個百分點

**結論: 這是最簡單有效的改進 — 只改 C_dpmr 公式。**

---

## 6. 關鍵洞察

1. **a_m 必須獨立於 a_hat**: IIR variance 是數據驅動的（好），innovation ratio 是自我參照的（壞）

2. **C_dpmr 不是常數**: 它依賴 rho（弱依賴 a_hat），可以用 7-state Lyapunov precompute

3. **Fe coupling 是雙面刃**: 提供 dynamics-based observability，但在 joint KF 中造成不穩定

4. **Dual architecture 更穩定**: 3-state observer（穩定）+ 獨立 a_hat 更新（避免 Fe 耦合）

5. **P^f recursion 就是 C_dpmr 的 generalization**: KF 內部已經「知道」innov 和 a 的關係

6. **Q/R 可以從物理推導**: 不需要 tuning，用 innovation 驗證 consistency

---

## 7. 後續方向

1. 把 correct C_dpmr(rho) 套用到 Simulink ctrl7 → 預期 RMSE 從 ~35% 降到 ~27%
2. 用 innovation 診斷調整 Q/R → 預期進一步改善
3. 結合 dynamics channel (通道 1) 和 variance channel (通道 2) 的最佳比例
4. 考慮 noise_corr 的修正（之前發現 IIR-filtered 的 noise correction 和 unfiltered 不同）
