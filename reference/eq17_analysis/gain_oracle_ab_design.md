# Gain Oracle A/B 對比實驗設計（eq17-6state）

Date: 2026-06-10
Branch: `test/motion-test`（自 feat/eq17-6state @ d6f95cc 分出，獨立 worktree）
Status: 設計定稿，待使用者 review 後進 implementation plan。

---

## 1. 目的

評估 6-state（RevisedControl_Vpersonal）控制器**本身**的控制能力，把 motion gain 估測品質的影響獨立出來量化：

> 同一條激進軌跡下，控制律拿到「正確的時變 gain」（oracle）vs「EKF 估測的 gain」，
> 控制結果（追蹤誤差）的 deterministic 分量與 random 分量各差多少？

- **Arm A（oracle）**＝估測完美時的性能上界，同時兼任分析 pipeline 的理論「對答案」錨點。
- **Arm B（estimated）**＝現役控制器原樣，代表目前端到端水準。
- A、B 的差距 = gain 估測誤差對控制的實際代價（det：系統性落後；ram：隨機誤差放大倍率）。

## 2. 實驗矩陣

每個頻率 f ∈ {1, 2, 5} Hz：

| | det run（噪聲全關）| ram runs（噪聲全開）|
|---|---|---|
| Arm A（oracle gain）| 1 run | seeds 1:5 |
| Arm B（EKF gain）| 1 run | seeds 1:5 |

- 合計 3 × 12 = 36 runs，全走 `run_pure_simulation`（pure-MATLAB），全部 `collect_diag = true`。
- 「噪聲全關」= `thermal_enable = false` 且 `meas_noise_enable = false`（模擬僅有的兩個隨機源）。det run 完全確定性，1 次即可、無 seed。
- 同 seed 的 A/B 共用同一條底層 randn 序列（common random numbers, paired design）。

### 兩臂共通設定

- `config.eq17_variant = '6state'`，λ_c = 0.7，a_pd = a_cov = 0.05
- `meas_noise_std = [0.00062; 0.00057; 0.00331]` μm（沿用 verify_eq17_6state）
- **`suppress_xD = true`（兩臂都設）**：控制力 f_d 不含 −x̂_D 項。
  - 理由：模擬中真實擾動 = 0，−x̂_D 只是噪聲注入通道；兩臂同時拿掉後 A/B 差異純粹來自 gain 來源。
  - EKF slot 4 照常估計、照常 log（當診斷信號）。

### 兩臂差異（唯一變因）

| | 控制律中的 gain（含過去項 Σ a[k−i]·f_d[k−i]）| EKF |
|---|---|---|
| Arm A | oracle `a_true[k]`（driver 每步從 p_curr 算）| 照常跑（slot 5 照估，但不進控制律）|
| Arm B | EKF 估測 `â[k]`（現役行為）| 照常跑 |

oracle 定義：`a_true,i[k] = a_nom / C_i(h̄_true[k])`，`a_nom = Ts/γ_N`，h̄_true 從**無噪聲真實位置** p_curr 算；x/y 用 C_∥、z 用 C_⊥。這是模擬特權（無延遲、無噪聲的真值），實機不可得，故為上界。

## 3. 軌跡（`osc_aggr`，激進版）

4-phase 既有形式（hold → cosine descent → oscillation → hold），參數：

| 參數 | 值 | 說明 |
|---|---|---|
| h_init | 50 μm（h̄ ≈ 22.2）| 高處出發 |
| t_hold | 0.5 s | IIR/EKF 暖機 |
| t_descend_override | 1.0 s | 與頻率脫鉤（避免 5 Hz 時預設 1/f = 0.2 s 衝底）|
| h_bottom | 2.7 μm（**h̄ = 1.2**）| 震盪波谷，低於 gate 邊界 h̄_safe = 1.5 |
| amplitude | 2.5 μm | h ∈ [2.7, 7.7] μm → h̄ ∈ [1.2, 3.42]，**每週期穿越 gate** |
| frequency | {1, 2, 5} Hz | sweep |
| n_cycles | 5·f（= 5/10/25）| 震盪時長固定 5.0 s，統計窗等長 |
| T_sim | 7.0 s | 0.5 + 1.0 + 5.0 + 0.5（尾段 hold at h̄=1.2）|
| config.h_min | 1.05·R = 2.3625 μm | scenario 內 override（全域預設 1.5R 會擋掉此軌跡；不動全域）|

z 軸 gain 動態範圍約 6×（h̄=22 的 ≈a_nom → h̄=1.2 的 a_nom/C_⊥）；同時壓「gain 追蹤」與「gate 每週期 on/off 切換」兩個機制。

實作前置檢查（step 0）：確認 `t_descend_override`、`h_min` 由 config → `calc_traj_params` 接通（`trajectory_generator` 已支援 `params.traj.t_descend_override`）。

## 4. 程式變更規格（全部向後相容）

### 4.1 `model/controller/motion_control_law_eq17_6state.m`

(a) 新增可選第 6 引數 `a_ctrl_override`（3×1 或 []）：
  - 非空時：控制律的 `1/â` 與過去項改用 override 值；新增獨立 persistent `a_ctrl_km1 / a_ctrl_km2` 緩衝存 override 歷史。
  - **EKF 一行不動**：Q33 的 thermal 項仍用 EKF 自己的 `a_hat_km1/km2`，slot 5 照估。
  - `diag` 新增 `a_ctrl_used`（3×1，控制律實際使用的 gain）——兩種模式都填（override 模式 = override 值；正常模式 = â）。

(b) 新增 `ctrl_const.suppress_xD` 支援（與 7-state core 同語意）：
  - true 時控制律的 `− xD_comb` 項歸零；EKF slot 4 照常 predict/update。

向後相容要求：不傳第 6 引數且不設 suppress_xD 時，輸出 bit-identical（以 h50 回歸驗證）。

### 4.2 `model/dual_track/run_pure_simulation.m`

- 新增 `opts.gain_oracle`（預設 false）。true 時每步從 p_curr 算 `a_true_k`（公式同 §2），以第 6 引數傳入 6-state 控制器（僅 6-state 路徑支援；7-state 路徑遇 gain_oracle=true 直接 error）。
- 無論模式，**每步都算並記錄 `simOut.a_true_out` [N×3]**（分析直接用，免去事後重算）。
  - **記錄點定義**：`a_true_out[k]` 在**積分前**的 p_curr（= 控制器在 step k 看到的真實位置）計算，與 `a_ctrl_used[k]` 精確同點；注意 `p_true_out[k]` 是積分後（t_k+Ts）的位置，兩者差一個樣本，屬既有 driver 慣例。
- `diag_log` 新增 `a_ctrl_used` [N×3]。

### 4.3 新增 `test_script/integration/compare_gain_oracle_6state.m`

單一入口：跑 §2 實驗矩陣 → Layer 0 assertions → det/ram 分析（§6）→ 出圖（§8）→ 寫 summary。
輸出至 `test_results/gain_oracle_ab/f<freq>Hz/`（gitignored），含 runs.mat、analysis.mat、summary.md、figs。
跨頻率總覽輸出至 `test_results/gain_oracle_ab/overview/`。

### 4.4 新增 `test_script/unit_tests/verify_eq17_unit_gain_override_6state.m`

- override 路徑：固定輸入下 f_d 與手算 Eq.17 公式逐項比對（gain = override 值、含過去項緩衝）。
- suppress_xD 路徑：f_d 中無 −x̂_D 項。
- 向後相容：無 override、無 suppress_xD 時與變更前輸出一致。

### 4.5 回歸

`run_eq17_6state_all`（h50 PASS gate）必須維持 PASS 且數字不變。

## 5. 信號清單

每 run 取出（[N×3]，N = 7 s × 1600 Hz + 1）：

| 信號 | 來源 | 用途 |
|---|---|---|
| `p_d_out` | driver | 誤差基準；12 runs 必須逐點相同 |
| `p_true_out` | driver（無噪聲真值探針）| **主路誤差** e = p_d − p_true |
| `p_m_out` | driver | 副路誤差 + 實機預演 |
| `a_true_out` | driver（新增）| 理論基準 / arm A 輸入 |
| `diag.a_ctrl_used` | controller（新增）| 接線驗證 |
| `diag.a_hat`, `diag.x_D_hat`, `diag.gate_active`, `diag.h_bar`, `f_d_out` | 既有 | 估測品質、診斷 |

控制器內部的 δx_m[k] = p_d[k−d] − p_m[k]（帶 d=2 延遲）**不用於**性能統計；分析一律用同步誤差。

## 6. 分析方法

### 6.0 拆解原則

det/ram 拆解**只在同臂內**做（每臂自帶 det 參考），跨臂只比統計量：

```
e[k]     = p_d[k] − p_true[k]                  （per run）
ram_s[k] = e_s[k] − e_det[k] = p_true_det[k] − p_true_s[k]   （p_d 消去）
```

**Log 對齊（重要）**：driver 慣例 `p_d_out[k]` 為 t_k、`p_true_out[k]`／`p_m_out[k]` 為積分後 t_k+Ts，差一個樣本。分析一律以

```
e[k] = p_d_out[k+1] − p_true_out[k]      （兩者同為 t_k+Ts 時刻）
```

對齊後再計算；否則 5 Hz 正弦擬合會帶 2πf·Ts ≈ 1.1° 的系統性相位偏移。A/B 同受影響故 ratio 不受害，但絕對相位會錯。

時間窗（由軌跡解析定義，全部 runs 共用）：

```
W_desc = [0.5, 1.5] s   |  discard [1.5, 2.5] s（前 f 個 cycle 暫態）
W_osc  = [2.5, 6.5] s   |  W_tail [6.5, 7.0] s（hold at h̄=1.2，觀察用，不進主統計）
gate 子窗 mask（確定性，不用 per-seed 的 diag.gate_active）：
  h̄_d(t) = (p_d·ŵ − p_z)/R；mask_on = (h̄_d < 1.5) ∧ W_osc；mask_off 反之
```

### 6.1 det 分量（每臂每頻率 1 條 e_det）

| 量 | 算法 |
|---|---|
| descent 峰值 | max\|e_det\| 於 W_desc（記號正負與時刻）|
| 正弦擬合 | W_osc 上最小二乘 X=[1, cos2πft, sin2πft] → 殘餘振幅 A_e、相位落後 φ、擬合殘差 rms_res（諧波/非線性指標）|
| cycle 平均波形 | reshape 成 [n_per_cycle × n_cycles] 取列平均（不假設正弦的形狀檢查）|
| trough bias | 各波谷時刻 ±5 樣本平均，再對 cycle 平均 |

### 6.2 ram 分量（每臂每頻率 5 條 ram_s）

| 量 | 算法 |
|---|---|
| 窗內 mean / std 分離 | μ_s(w) = mean(ram_s(w))（噪聲整流 bias，單獨成欄）；sd_s(w) = std(ram_s(w)) |
| 跨 seed 彙整 | mean ± [min, max]（5 seeds）|
| **A/B ratio（核心輸出）** | ratio_s(w) = sd_B_s(w)/sd_A_s(w)，**同 seed 配對相除**（common random numbers 紅利），報 mean ± range |
| 平穩性 | W_osc 內每 cycle 一個 std，前後半差 < 20% 為平穩；有趨勢（如 arm B 慢性漂移）標記單列討論 |

### 6.3 p_m 副路（cross-check + 實機預演）

noisy run 的量測版 ram：`ram_m_s = ram_s − n_s`。driver 迴路順序保證 n[k] 經 d=2 步延遲才回授影響位置 → 同時刻 Cov(ram, n) = 0，故：

```
Var(ram_m) = Var(ram_phys) + σ_n²    （精確）
std_phys_from_pm = sqrt( std(ram_m)² − σ_n² )
```

驗證：副路還原值與主路（p_true）的相對差 < 2%（soft，z 軸）；超出 = 噪聲注入或時序 bug（Layer 0 的延伸檢查）。z 軸修正量級 ~1%，x/y 可忽略。

### 6.4 a_hat 的同款拆解（arm B 專屬）

- 系統性：以 **ensemble mean**（5 seeds 的 a_hat 平均曲線）為主對照 a_true；det run 的 a_hat 降為輔助。
  - 原因（與 e 的處理唯一不對稱處）：det run 無噪聲 → σ²_δxr → 0 → Guard 2 全程觸發 → y₂ 關閉，a_hat 走 y₁-only 路徑，與 noisy 平均行為結構不同。
- 隨機：a_hat_s − a_hat_ensemble_mean 的窗內 std。
- 連接誤差鏈：gain 的 det 偏差 ↔ e_det^B 超出 e_det^A 的部分；gain 的 ram ↔ ram^B 超出 ram^A 的部分。

### 6.5 可選項（第一輪不做，留接口）

ram 的 Welch PSD（A/B 頻段分解，銜接 paper Fig.13 式分析）。

## 7. 三層驗證（寫進分析腳本自動執行）

**Layer 0 — 資料完整性（不過不進分析）**
1. 12 runs（同頻率）`p_d_out` 逐點相等（assert max|Δp_d| = 0）。
2. 接線斷言（注意時序）：
   - arm A：`a_ctrl_used[k] ≡ a_true_out[k]`（同記錄點，精確相等）。
   - arm B：`a_ctrl_used[k] ≡ â_post[k−1]`（控制律用的是進入 step k 時的 persistent 值 = 上一步 posterior；`diag.a_hat[k]` 是本步 posterior，**差一步是正確行為**，斷言按此寫）。
3. 發散偵測：h̄_true ≤ 1.001（撞牆）或 |e| > 0.5 μm → 該 run 標 diverged，剔除統計、報告單列。
   - 撞牆時 `calc_correction_functions`（要求 h̄ > 1）會直接 error 中斷模擬：跑批層以 try/catch 包住單一 run，error = diverged（保留已跑出的部分波形供診斷），不讓單一 run 中斷整批。

**Layer 1 — 物理錨點（arm A 對答案）**
4. arm A det：完美 gain + 無噪聲 ≈ paper 2023 Eq.18 理想閉迴路，e_det 應 ≈ 0。soft gate：W_osc 內 |e_det| 與 A_e < 1 nm；超出即先除錯再談結果。殘餘量 = Eq.17 延遲補償結構極限（本身就是「控制本事」的量測）。
5. arm A ram vs 理論包絡（V1 閉式，review findings §7.2）：

```
σ²_th,i(t) = C_δx·4k_B·T·a_true,i(t) + (1−λ_c)/(1+λ_c)·σ²_n,i
C_δx = 2 + 1/(1−λ_c²) = 3.96   (λ_c = 0.7)
z_s[k] = ram_s[k]/σ_th(t_k)；soft gate：std(z_s(W_osc)) ∈ 1 ± 0.15
```

   超出標註討論（已知偏差源：quasi-static 假設在 5 Hz trough 最弱；理論係數本身 ~10–15% 精度）。

**Layer 2 — 統計穩健性**
6. cycle-bin 平穩性（§6.2）。
7. across-seed spread 須明顯小於 A/B 差異本身（paired ratio 的 range 為結論不確定度）。
8. ensemble mean vs det run 差異欄（噪聲整流 sanity；μ_s ≈ 0 則 det 定義與 ensemble mean 一致）。

## 8. 報告與繪圖

風格：沿用 `make_eq17_6state_figures` 的 EXP/thesis style（role colors、grid off、tiledlayout compact、stats-in-title、legend northoutside）。色彩語意擴充：**True/理論 = 綠、arm B = 紅（沿用 "Estimated"）、arm A = 藍紫**；跨頻率總覽借 `learn_variance/plot_style.m` 三級 sweep palette（藍/橙/紫 = 1/2/5 Hz）。

每頻率一組：
1. `fig1_gain_tracking`：a_true vs arm B 的 â（z、x 兩列，套現有 fig1 模板）
2. `fig2_det_error`：e_det^A vs e_det^B 疊圖（z 軸，descent/osc 分段 shading）
3. `fig3_ram_std`：分窗 std bar（A vs B，per axis）+ paired ratio 標註
4. `fig4_theory_anchor`：arm A normalized ram（z_s）vs 1 ± 0.15 帶
跨頻率：
5. `fig5_freq_overview`：A/B ratio vs frequency（分窗、z 軸為主）

summary.md 主表（每頻率，per axis [x y z]）：

| 區塊 | 欄位 |
|---|---|
| det | descent peak、A_e、φ、rms_res、trough bias —— A、B 並列 |
| ram | 各窗 sd mean ± range（A、B）、paired ratio mean ± range、μ（整流 bias）|
| 驗證 | Layer 0/1/2 各項 PASS/FLAG、gate duty cycle、diverged run 清單 |
| 估測 | arm B：â ensemble-mean rel-err（trough/peak）、â ram std |

## 9. 風險與 caveats

1. **arm B 在 h̄ = 1.2 可能漂移甚至發散**（已知 near-wall 脆弱區；LF drift audit 的 gate-latched 失錨機制）——這是實驗結果不是實驗失敗；發散偵測保護整批統計。
2. **arm A 的 EKF model mismatch**：F_e Row 3 推導假設控制用 â；oracle 模式下該假設破。suppress_xD 後 x̂_D 不進迴路，影響只剩診斷信號的解讀（slot 4/5 估計值會偏，屬預期）。
3. **arm B det run 的 Guard 2 行為**（§6.4）——a_hat 系統性分析以 ensemble mean 為主。
4. **quasi-static 理論包絡在 5 Hz 最弱**——Layer 1 是 soft gate 不是 hard gate。
5. **h_min override 僅限本 scenario**——不動全域預設，避免影響既有 harness。

## 10. 工作流

- 分支 `test/motion-test`（worktree：`../MotionControl_Simu-motion-test`）；使用者同時在主 checkout 的 feat/eq17-6state 作業，互不干擾。
- MATLAB 執行：實驗跑批在 worktree 路徑下用獨立 MATLAB process（Bash batch）；互動 MATLAB/MCP 留在主 checkout。兩份 checkout 同名函數，同一 session 的 path 只能指向一份。
- Merge 回 feat/eq17-6state 的條件：unit tests + h50 回歸 PASS、Layer 0/1 驗證通過、使用者 review 結果。
- 變更全 additive（可選引數、opts、新檔案），衝突面小。

## 11. 驗收標準

| # | 項目 | 標準 |
|---|---|---|
| 1 | 向後相容 | 無 override / 無 suppress_xD 時 controller 輸出 bit-identical；`run_eq17_6state_all` h50 PASS 數字不變 |
| 2 | unit test | `verify_eq17_unit_gain_override_6state` 全 PASS |
| 3 | Layer 0 | 36 runs assertions 全過（diverged 屬合法結果，單列）|
| 4 | Layer 1 | arm A det < 1 nm（soft）；arm A normalized ram ∈ 1 ± 0.15（soft）；超出須有書面解釋 |
| 5 | 交付 | 3 組 per-freq 報告 + 跨頻率總覽 + 本設計文件對應的 findings 章節 |
