# Gain Compare A/B 對比實驗設計（eq17-6state）

Date: 2026-06-10（Round 1）/ 2026-06-11（Round 2，§12）
Branch: `test/motion-test`（自 feat/eq17-6state @ d6f95cc 分出，獨立 worktree）
Status: Round 1（§1–11）已實作完成並跑完 20-seed 生產批（部分細節被執行中的方法修正取代，見 §12.0）。Round 2（§12）= gate-free 新 scenario + 100 seeds + 新圖 + 分析升級，設計定稿待 review。

---

## 1. 目的

評估 6-state（RevisedControl_Vpersonal）控制器**本身**的控制能力，把 motion gain 估測品質的影響獨立出來量化：

> 同一條激進軌跡下，控制律拿到「正確的時變 gain」（true-gain control）vs「EKF 估測的 gain」，
> 控制結果（追蹤誤差）的 deterministic 分量與 random 分量各差多少？

- **a=a_true（true-gain control）**＝估測完美時的性能上界，同時兼任分析 pipeline 的理論「對答案」錨點。
- **a=â（estimated）**＝現役控制器原樣，代表目前端到端水準。
- A、B 的差距 = gain 估測誤差對控制的實際代價（det：系統性落後；ram：隨機誤差放大倍率）。

## 2. 實驗矩陣

每個頻率 f ∈ {1, 2, 5} Hz：

| | det run（噪聲全關）| ram runs（噪聲全開）|
|---|---|---|
| a=a_true（true gain）| 1 run | seeds 1:5 |
| a=â（EKF gain）| 1 run | seeds 1:5 |

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
| a=a_true | true-gain control `a_true[k]`（driver 每步從 p_curr 算）| 照常跑（slot 5 照估，但不進控制律）|
| a=â | EKF 估測 `â[k]`（現役行為）| 照常跑 |

true-gain control 定義：`a_true,i[k] = a_nom / C_i(h̄_true[k])`，`a_nom = Ts/γ_N`，h̄_true 從**無噪聲真實位置** p_curr 算；x/y 用 C_∥、z 用 C_⊥。這是模擬特權（無延遲、無噪聲的真值），實機不可得，故為上界。

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

- 新增 `opts.use_true_gain`（預設 false）。true 時每步從 p_curr 算 `a_true_k`（公式同 §2），以第 6 引數傳入 6-state 控制器（僅 6-state 路徑支援；7-state 路徑遇 use_true_gain=true 直接 error）。
- 無論模式，**每步都算並記錄 `simOut.a_true_out` [N×3]**（分析直接用，免去事後重算）。
  - **記錄點定義**：`a_true_out[k]` 在**積分前**的 p_curr（= 控制器在 step k 看到的真實位置）計算，與 `a_ctrl_used[k]` 精確同點；注意 `p_true_out[k]` 是積分後（t_k+Ts）的位置，兩者差一個樣本，屬既有 driver 慣例。
- `diag_log` 新增 `a_ctrl_used` [N×3]。

### 4.3 新增 `test_script/integration/compare_gain_6state.m`

單一入口：跑 §2 實驗矩陣 → Layer 0 assertions → det/ram 分析（§6）→ 出圖（§8）→ 寫 summary。
輸出至 `test_results/gain_compare/f<freq>Hz/`（gitignored），含 runs.mat、analysis.mat、summary.md、figs。
跨頻率總覽輸出至 `test_results/gain_compare/overview/`。

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
| `a_true_out` | driver（新增）| 理論基準 / a=a_true 輸入 |
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
| 平穩性 | W_osc 內每 cycle 一個 std，前後半差 < 20% 為平穩；有趨勢（如 a=â 慢性漂移）標記單列討論 |

### 6.3 p_m 副路（cross-check + 實機預演）

noisy run 的量測版 ram：`ram_m_s = ram_s − n_s`。driver 迴路順序保證 n[k] 經 d=2 步延遲才回授影響位置 → 同時刻 Cov(ram, n) = 0，故：

```
Var(ram_m) = Var(ram_phys) + σ_n²    （精確）
std_phys_from_pm = sqrt( std(ram_m)² − σ_n² )
```

驗證：副路還原值與主路（p_true）的相對差 < 2%（soft，z 軸）；超出 = 噪聲注入或時序 bug（Layer 0 的延伸檢查）。z 軸修正量級 ~1%，x/y 可忽略。

### 6.4 a_hat 的同款拆解（a=â 專屬）

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
   - a=a_true：`a_ctrl_used[k] ≡ a_true_out[k]`（同記錄點，精確相等）。
   - a=â：`a_ctrl_used[k] ≡ â_post[k−1]`（控制律用的是進入 step k 時的 persistent 值 = 上一步 posterior；`diag.a_hat[k]` 是本步 posterior，**差一步是正確行為**，斷言按此寫）。
3. 發散偵測：h̄_true ≤ 1.001（撞牆）或 |e| > 0.5 μm → 該 run 標 diverged，剔除統計、報告單列。
   - 撞牆時 `calc_correction_functions`（要求 h̄ > 1）會直接 error 中斷模擬：跑批層以 try/catch 包住單一 run，error = diverged（保留已跑出的部分波形供診斷），不讓單一 run 中斷整批。

**Layer 1 — 物理錨點（a=a_true 對答案）**
4. a=a_true det：完美 gain + 無噪聲 ≈ paper 2023 Eq.18 理想閉迴路，e_det 應 ≈ 0。soft gate：W_osc 內 |e_det| 與 A_e < 1 nm；超出即先除錯再談結果。殘餘量 = Eq.17 延遲補償結構極限（本身就是「控制本事」的量測）。
5. a=a_true ram vs 理論包絡（V1 閉式，review findings §7.2）：

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

風格：沿用 `make_eq17_6state_figures` 的 EXP/thesis style（role colors、grid off、tiledlayout compact、stats-in-title、legend northoutside）。色彩語意擴充：**True/理論 = 綠、a=â = 紅（沿用 "Estimated"）、a=a_true = 藍紫**；跨頻率總覽借 `learn_variance/plot_style.m` 三級 sweep palette（藍/橙/紫 = 1/2/5 Hz）。

每頻率一組：
1. `fig1_gain_tracking`：a_true vs a=â 的 â（z、x 兩列，套現有 fig1 模板）
2. `fig2_det_error`：e_det^A vs e_det^B 疊圖（z 軸，descent/osc 分段 shading）
3. `fig3_ram_std`：分窗 std bar（A vs B，per axis）+ paired ratio 標註
4. `fig4_theory_anchor`：a=a_true normalized ram（z_s）vs 1 ± 0.15 帶
跨頻率：
5. `fig5_freq_overview`：A/B ratio vs frequency（分窗、z 軸為主）

summary.md 主表（每頻率，per axis [x y z]）：

| 區塊 | 欄位 |
|---|---|
| det | descent peak、A_e、φ、rms_res、trough bias —— A、B 並列 |
| ram | 各窗 sd mean ± range（A、B）、paired ratio mean ± range、μ（整流 bias）|
| 驗證 | Layer 0/1/2 各項 PASS/FLAG、gate duty cycle、diverged run 清單 |
| 估測 | a=â：â ensemble-mean rel-err（trough/peak）、â ram std |

## 9. 風險與 caveats

1. **a=â 在 h̄ = 1.2 可能漂移甚至發散**（已知 near-wall 脆弱區；LF drift audit 的 gate-latched 失錨機制）——這是實驗結果不是實驗失敗；發散偵測保護整批統計。
2. **a=a_true 的 EKF model mismatch**：F_e Row 3 推導假設控制用 â；true-gain control 模式下該假設破。suppress_xD 後 x̂_D 不進迴路，影響只剩診斷信號的解讀（slot 4/5 估計值會偏，屬預期）。
3. **a=â det run 的 Guard 2 行為**（§6.4）——a_hat 系統性分析以 ensemble mean 為主。
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
| 4 | Layer 1 | a=a_true det < 1 nm（soft）；a=a_true normalized ram ∈ 1 ± 0.15（soft）；超出須有書面解釋。5 Hz 預期 FLAG（<2 nm）：近壁 quasi-static 近似殘差隨頻率增長，不影響 stochastic anchor（Layer 1 ram 檢查仍 PASS）。 |

---

## 12. Round 2 — gate-free 新 scenario + 100 seeds + 新圖 + 分析升級（2026-06-11）

### 12.0 Round 1 執行後的方法修正（已實作，記錄供讀者對照 §1–11）

Round 1 執行中經使用者質疑後修正的設計，git 歷史 905a1d8 / c6f17f4 / e5d4060 / 2a3c131：

1. **det 萃取改 ensemble 法**：noise-free det run 對 a=â 無效（無噪聲下 Guard 2 latch → y₂ 關 → â 走 y₁-only 路徑，與 noisy 行為結構不同；crossval 實證 z 系統差 26–28 nm rms @2/5Hz）。det := 跨 seed 逐時刻平均。**不對稱參考（使用者核准）**：a=a_true 用 noise-free run（對 A 有效且精確）、a=â 用 ensemble。§6.1 的 det run 表、§8 圖組對 a=â 部分被此取代。
2. **ram v2**：以 ensemble det 為參考重算（v1 的 ratio 1.6–1.7 是 det 洩漏 artifact；v2 headline：osc ≈ 1.0，超額僅 gate-on）。自減緊縮以 /(1−1/Ns) 精確補償。
3. **seeds 5 → 20**（Round 1 生產批實際為 20）。
4. **圖組重做**：§8 的 fig1–fig5 退役，改為 fig_traj_det / fig_traj_ram / fig_det_err 三張（git 12f6994 起）。
5. 誤差帳本：det 殘餘 σ/√N（無偏、不可補償、誤差棒呈現）；deflation（精確補償）；有限樣本散佈（範圍/誤差棒）。

### 12.1 Round 2 模擬配置（單一配置：B′ gate-free）

只跑一組配置，輸出 `test_results/gain_compare/f<freq>Hz/`（不覆蓋既有資料）：

| 項目 | Round 1 值 | Round 2 值 |
|---|---|---|
| 頻率 | {1, 2, 5} Hz | **{1, 5, 10} Hz** |
| 震盪時長 | 5.0 s（n_cycles = 5f）| **2.0 s（n_cycles = 2f）** |
| tail hold | 0.5 s | 0.5 s |
| T_sim | 7.0 s | **4.0 s**（0.5 + 1.0 + 2.0 + 0.5）|
| seeds | 20 | **100** |
| **h̄_safe** | 1.5（G3 每週期觸發，duty ~23%）| **1（G3 永不觸發；軌跡谷 h̄ = 1.2 > 1）= gate-free** |
| 其餘 | — | 不變（h_init 50、h_bottom 2.7、A 2.5、t_descend 1.0、h_min 1.05R、suppress_xD 兩臂、CRN 配對）|

- h̄_safe = 1 語意：僅真正碰壁才觸發，實質關閉 G3。G1 本來就關（t_warmup_kf = 0）、G2 在 noisy run 數學上不可觸發（熱訊號高於噪聲地板 ~10⁴ 倍）→ 本配置 = EKF 全程雙回授。**近壁 â 無 gate 保護是本實驗的目的**；發散偵測（§7 Layer 0 #3）保留，diverged 屬合法結果。
- driver/config plumbing：h̄_safe 需從 scenario config 接通到 `build_eq17_constants`（現為定值），additive、預設值不變。
- 預估成本：~4.5 s/run × 202 runs/頻率 ≈ 15 min/頻率，全矩陣 ~45 min；runs.mat ~630 MB/頻率。
- 風險：1 Hz 震盪段僅 2 週期（丟 1 留 1），窗統計薄，靠 100 seeds 補；10 Hz 的 quasi-static 近似最弱（Layer 1 soft gate 預期 FLAG 幅度更大，屬已知）。

### 12.2 分析窗更新（per-cycle discard）

discard 從固定 1.0 s 改為**丟震盪第 1 個週期**：

```
W_desc  = [0.5, 1.5] s
discard = [1.5, 1.5 + 1/f] s          （第 1 週期暫態）
W_osc   = [1.5 + 1/f, 3.5] s          （1 Hz: 1 cycle；5 Hz: 9；10 Hz: 19）
W_tail  = [3.5, 4.0] s                 （hold at h̄ = 1.2，觀察用）
gate 子窗 mask：照 §6 用 h̄_d < 1.5 幾何定義（gate-free 下仍是有意義的「近壁/遠壁」分窗，沿用名稱 near/far）
```

### 12.3 分析升級（既有 pipeline 上的四項）

1. **A2 — SEM 誤差棒**：所有跨 seed 彙整統計從 `mean [min, max]` 升級為 `mean ± SEM [min, max]`（SEM = 跨 seed std/√N）；paired ratio 同樣逐 seed 配對後報 mean ± SEM。
2. **x_ram 直取**：det_x ≡ 0（鏡射對稱，Round 1 雙重實證）→ x 軸 ram 直接用 x 本身，不做 ensemble 減法、無 deflation 修正；z 軸照 ensemble 法。
3. **A1 — Q55 閉式近壁動態首驗**：用 a=a_true 的 a_true stack（100 seeds）做逐 seed ram = a_true_s − mean_s(a_true)，分窗驗**兩個量**（eq17_6state_review_findings.md §8.1 三層鏈）：
   - **Level**：Var(a_ram) vs 閉式 `C_δx·(a·K_h/R)²·σ²_δh`（鏈 A+B）
   - **Increment（= Q55 本體）**：Var(δa_ram)（一步差分）vs 閉式 `[2/(1+λ_c)]·(a·K_h/R)²·σ²_δh`（鏈 C，C_δx 與 (1−ρ₁) 對消）
   - σ²_δh = 4k_B·T·a（per-step thermal kick）；理論逐點算（a、K_h 沿 h̄_d）再窗內平均。輸出 ratio 表進 summary。先前僅 h=50 靜態驗過（emp/closed = 0.998–1.001）。
4. **A3 — desc 窗 â 統計**：summary 的 a=â gain estimation 區塊補 desc 窗（â ensemble-mean rel-err + â ram std），與 osc/near/far 並列。

### 12.4 新圖 1 — 三層 gain 對比（`fig_gain_compare`）

x/z 兩列（a_x 用 C_∥、a_z 用 C_⊥），每列四層，由底至頂：

| 層 | 信號 | 來源 | 線型 |
|---|---|---|---|
| 1 | `a_xm`（IIR 反解的 gain 量測）| **B 臂**、單 seed raw（traj-figure seed）| 淡藍細線（Measured 慣例）|
| 2 | `a_pd = a_nom/C_i(h̄_d)` | 期望軌跡（確定性，分析端算）| 線型於樣張定 |
| 3 | `a_true` | **A 臂**（完美控制下的實際 gain）、100-seed ensemble mean | 同上 |
| 4 | `â` | **B 臂**（生產估測）、100-seed ensemble mean | 同上 |

- 語意：a_pd = 設計上應有的 gain（先驗、實機可算）；a_true(A) = 完美控制下實際發生的 gain；â(B) = 生產控制器以為的 gain；a_xm = 估測器看到的原始量測。
- 層 2–4 的顏色/粗細在樣張階段定（參考圖的 Measured/True/Estimated 風格 + §12.6 規範）。

### 12.5 新圖 2 — motion variance 對比（`fig_motion_var`）

x/z 兩列，每列三條曲線：

```
理論：σ²_th,i(t) = C_δx·4k_B·T·a_pd,i(t) + C_n_fb·σ²_n,i      （沿期望軌跡，確定性）
實測 ×2：a=a_true、a=â 的逐時刻 ensemble var
  z 軸：var_s over seeds of ram_v2(t_k)，× 1/(1−1/N) deflation 補償
  x 軸：var over seeds of x(t_k)（直取，無補償）
```

- **純 pointwise，不做時間平滑**（使用者定案）；100 seeds 下每點散佈 ~√(2/99) ≈ 14%，曲線毛刺屬無偏呈現。
- 不加第二條分解理論線（B 臂位置偏離效應不單獨拆；目標狀態 = 完美估測下該項趨零）。
- 單位 nm²；本圖 = §7 Layer 1 thermal_theory_check 的時間解析版，分窗數字表照常並行輸出。

### 12.6 圖風格全面定案（適用全部圖）

1. **標題**：stats-in-title（統計數值進標題；各圖放哪些數字於樣張階段微調）。
2. **時間軸**：全時段 [0, T_sim]，軸標 `Time (sec)`。
3. **字體/tick**：對照 `make_eq17_6state_figures` 參考風格（自然 tick 密度、~18pt）；**外框（box on）與 tick mark 線寬加粗**，與資料線對比清楚。
4. **ram 疊圖層次**：B 臂（â）= 藍 [0 0.2 0.9]、粗 2.5、底層先畫；A 臂（a_true）= 紅 [0.8 0 0]、細 1.0、疊上層（樣張 `fig_mockup/fig_traj_ram_mock_v3a_colorswap.png`）。讀法：藍色塊狀露出 = â 臂超額。
5. **圖例名稱**：去 `a = ` 前綴，統一 `Desired` / `a_{true}` / `â`。
6. **fig_det_err（δ_det 圖）凍結**：本輪不動（鎖範圍與標註留待後續）。

### 12.7 工作流與驗收（Round 2 增量）

- 順序：程式改動（driver plumbing + analyzer 升級 + 新圖 + 風格）→ **以既有 f2Hz 20-seed 資料出全套樣張供使用者過目** → 樣張定稿後跑 100-seed 批次（~45 min）→ 全套產出 review。
- 驗收增量：
  | # | 項目 | 標準 |
  |---|---|---|
  | 5 | 向後相容 | h̄_safe 預設值不變時既有 unit tests + h50 回歸 PASS 數字不變 |
  | 6 | gate-free 接線 | **noisy** runs 全程 `diag.gate_active ≡ false`（assert 進 Layer 0；G1 關、G2 noisy 下不可觸發、G3 被 h̄_safe=1 關）。det run 不在此斷言內：無噪聲下 G2 latch 屬已知行為（§12.0）|
  | 7 | A1 | Q55 閉式 ratio 表輸出（PASS 門檻不預設，近壁動態首驗屬探索性，數字單列討論）|
  | 8 | 樣張 gate | 新圖 1/2 + 風格改版經使用者核准後才跑批 |
| 5 | 交付 | 3 組 per-freq 報告 + 跨頻率總覽 + 本設計文件對應的 findings 章節 |
