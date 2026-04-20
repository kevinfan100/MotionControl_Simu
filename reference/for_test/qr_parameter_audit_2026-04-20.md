# Q/R/Pf Parameter Audit (2026-04-20)

深入理論推導完成後 (6 Levels + 11 commits) 回頭審視目前參數設定,
發現幾個**不符合物理推導**的地方, 記錄備案供未來修正。

---

## ★★★ 問題 1: R(1,1) 不是 per-axis (最嚴重)

### 當前狀態 (motion_control_law_7state.m line 240)

```matlab
r_pos_base = sigma2_deltaXT * Rz_scaling(1);   % 常數, 對 3 軸都一樣
```

對 x, y, z 三軸 EKF 用**同一個 R(1,1) = sigma2_dXT × 0.397 ≈ 1e-4 um²**。

### 實際 sensor noise 變異 (config.meas_noise_std)

| 軸 | std (um) | σ²_n (um²) | ratio vs designer |
|---|---|---|---|
| x | 0.00062 | **3.84e-7** | designer 高估 **260×** |
| y | 0.000057 | **3.25e-9** | designer 高估 **30000×** |
| z | 0.00331 | **1.10e-5** | designer 高估 **9×** |

### 物理意義 & 影響

**正確做法**: `R(1,1)_axis = σ²_n_axis / σ²_dXT` (per-axis)

| 軸 | R(1,1)_correct_scale |
|---|---|
| x | 3.84e-7 / 2.5e-4 = **0.00154** |
| y | 3.25e-9 / 2.5e-4 = **1.3e-5** |
| z | 1.10e-5 / 2.5e-4 = **0.044** |

**目前用 0.397 對 x 過估 260×, 對 z 過估 9×**。

**後果**:
- EKF 認為位置量測雜訊比實際大很多 → Kalman gain L(6,1) 偏小
- 對 x 軸特別嚴重 (designer 過估 260×), 使 EKF「**少信任**」x 位置量測
- 這可能是 **frozen x 軸 a_hat 5-7× 失準的主因** (原歸因 per-axis EKF coupling, 實際可能就是 R(1,1) 不對)

### 修正代價
- Code 改 ~5 行 (motion_control_law_7state.m)
- Bus 可能要新增 per-axis 欄位 (原 scalar → 3x1)
- 預期改善: **x 軸 a_hat 預測從 5-7× → 可能 2× 內**

---

## ★★ 問題 2: Q(3,3) linear vs quadratic ar (paper vs simulation 模型差)

### Paper Eq.21 vs Simulation 物理模型差

**Paper 假設**: thermal force Var(f_T) = **constant** (free-space, axis-independent)
→ 導出 Q(3,3) = **(a_axis/a_nom)² · σ²_dXT = ar² · σ²_dXT**

**我們 simulation (calc_thermal_force.m line 68)**: `Variance = variance_coeff * abs(C)`
其中 C 含 c_para, c_perp → **Einstein per-axis**, Var(f_T_axis) ∝ c_axis
→ 導出 Q(3,3) = **ar · σ²_dXT** (linear, **differs from paper**!)

### 對比 h=2.5 z 軸 (c_perp = 10.44)

| 模型 | Q(3,3) scaling | 比當前 1 |
|---|---|---|
| Paper Eq.21 (ar²) | (0.096)² = **0.0092** | 109× 小 |
| **我們 sim 物理 (ar)** | **0.096** | **10× 小** |
| 當前用 (frozen_correct) | 1.0 | - |
| 對 h=50 z (ar ≈ 0.95) | 0.90-0.95 (兩個近似) | ~1.1× 小 |

### 物理意義 & 影響

**Q(3,3) 控制 EKF 對「del_p3 狀態可能變多少」的先驗**。用 1 代表相信 del_p3 有 free-space thermal 變異, 但近壁實際 thermal 變異小 10 倍。

**後果**:
- 近壁 EKF 以為 del_p3 容易變 → L 對 del_p3 channel 偏大 → EKF 依賴 measurement 過多 → 吸收過多雜訊
- 影響 tracking std 和 a_hat std 預測準度 (尤其近壁)
- 目前已在理論 ±26% 內 (其他 levels 補償), 但**物理上 Q(3,3) 應 per-scenario**

### 修正路線

**階段 A** (簡單, 1 天): Per-axis static Q(3,3)
- 在 user_config 依 h_init 算 ar, 設 per-axis Q(3,3) scaling
- Controller 架構不改 (只是 bus 多一個 per-axis vector)
- 對 static positioning 正確

**階段 B** (時變, 3-5 天): Adaptive Q(3,3)[k]
- 每步從 p_m 推 h_bar → c → a_axis → Q(3,3)
- **不依賴 a_hat** (無 feedback), 穩定
- 需要 L 也時變 (DARE re-solve per step, 或 lookup L(h_bar))

**階段 C** (理論 consistent, 複雜): Q(3,3) 依 a_hat
- `Q(3,3)[k] = f(a_hat[k])`
- 自洽但有 feedback loop (Q → L → a_hat → Q)
- 僅在「不知牆壁位置」場景必要

### 附註: linear vs quadratic

兩種模型都「對」, 看誰的模型:
- 如要跟 paper claim 一致: 用 ar² (但跟我們 simulation 物理不合)
- 如要跟 simulation 物理一致 (Einstein per-axis): 用 ar (與 paper 不合)

**我們目前 simulation 是 Einstein per-axis 的, 所以 Q 應用 ar linear**。

---

## ★ 問題 3: Q(7,7) = 1e-8 vs 0 (frozen vs empirical)

### 當前
- Frozen preset: Q(7,7) = 1e-8
- Empirical preset: Q(7,7) = **0**

### 物理

對 positioning (a 真常數), del_a = 0 恆等成立, Q(7,7) 真實 = 0。

Q(7,7) > 0 會觸發 double-integrator random walk:
```
del_a[k+1] = del_a[k] + Q77 noise
a[k+1]    = a[k] + del_a[k]   → a 累積 del_a 噪訊
```

Sigma_q77(e6, e6) 的 closed-loop 變異放大 ~1/L³ (極大)。但實際 plant 沒 Q77 driver, 所以不應該發生 (這是 EKF 「假想」的)。

### 建議

Frozen preset 試 **Q(7,7) = 0** (跟 empirical 一致), 看是否改善 frozen x 軸預測。

---

## ★ 問題 4: Stateflow 不是 adaptive Q 的限制 (歷史誤傳)

### 原來的說法 (錯的)
「Adaptive Q blocked by Stateflow」— 在早期 memory 有提到。

### 實際

當前 7-state EKF 是 `motion_control_law_7state.m` **MATLAB Function Block**, NOT Stateflow. 技術上沒 structural limit 阻擋 adaptive Q。

### 真正的限制

| 項目 | 嚴重 |
|---|---|
| DARE 離線求 L (固定) | ★★ |
| Per-step DARE 計算量 at 1600 Hz | ★ |
| Simulink Bus Object 形狀固定 | 輕微 |
| Stateflow 限制 | **不存在 (誤傳)** |

**修正路線**: 用 lookup L(h_bar) (預先存 grid), 避免 per-step DARE。

---

## 問題優先級

| # | 問題 | 嚴重 | 工作量 | 預期改善 |
|---|---|---|---|---|
| 1 | R(1,1) 非 per-axis | ★★★ | 1 天 | x 軸 a_hat 5-7× → 可能 2× |
| 2 | Q(3,3) 非 per-axis | ★★ | 1 天 (static) | tracking 準度精確化 (目前已 ±4%) |
| 3 | Q(7,7) = 1e-8 for frozen | ★ | 10 分鐘 (preset 改值) | 或許改善 x frozen |
| 4 | Stateflow 誤傳 | - | done (memory 已修) | 無 |

---

## 最終建議

**短期 (寫 paper 時)**:
- 接受問題 1-3 現狀
- 列為 **"Known limitations, see qr_parameter_audit_2026-04-20.md"**
- Paper 主場景 (z 軸 frozen) 已 ±26% 準 (good enough)

**中期 (下輪迭代)**:
- **優先修 R(1,1) per-axis** — 解決 x 軸 a_hat 失準, ROI 最高
- 然後 Q(3,3) static per-axis (容易且有物理理由)
- Q(7,7) 小改動試 0 看效果

**長期**:
- 整體時變 KF + controller (含 adaptive Q, L lookup)
- 泛化到「不知牆壁位置」場景 (Q 用 a_hat)

---

## 相關 commits 和檔案

**Commits**:
- `9dca456` - Einstein per-axis 修正 (Sigma_aug_phys)
- `171aa1f` - Sigma-based tracking + per-axis sigma2_n in lookup (理論部分 fix 了)
- `ea9c3d4` - Cascaded LP filter (Level 6)

**檔案**:
- `model/controller/motion_control_law_7state.m` (line 240: r_pos_base 常數)
- `model/config/apply_qr_preset.m` (presets Q/R 值)
- `model/thermal_force/calc_thermal_force.m` (confirmed Einstein per-axis)
- `reference/for_test/theory_complete_v2.md` (理論最終狀態)

---

## 結語

目前 Q/R/Pf 絕大多數正確:
- Structural zeros ✓
- R(2,2) self-consistent ✓ (low by tuning factor 1.72 → 1.0)
- Pf_init 設計合理 ✓
- Q(6,6) frozen numerical floor ✓

**剩 2-3 項 per-axis 改進** 就能更接近完整物理推導。不影響當前 paper 主場景結果, 但對 x 軸預測和泛化性有幫助。
