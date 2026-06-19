# a_m LPF → a_m_det 設計 spec（6-state, R22 重推 + 三組對照圖）

Date: 2026-06-16
Branch: test/motion-test
Status: DESIGN — 待使用者 review 後進 execution
Scope: 只處理 **f1Hz**（200-seed）。f0.5/f2Hz 待 f1Hz 全管線跑通再決定是否擴張。

相關：
- 現行鏈驗證：[`am_verification_understanding.md`](am_verification_understanding.md)、`test_script/integration/verify_axm_cdpmr_6state.m`
- 前車之鑑：memory `a_m LPF experiment (2026-04-16)` — 7-state positioning 下「a_m 前置 EMA 進 EKF」得 a_hat std −40% 但 tracking 變化 <1%，當時結論「pre-EKF LPF 方向錯，應調 Q/R」。本 study = 在 **6-state + dynamic osc + 重推 R22 + det 分解** 下做正確版重測。

---

## 1. 目標

在 a_xm（gain 量測）算完後、進 KF 前**多串一層 EWMA(a_det)** 得 a_m_det，重新推導並驗證對應的 R22，然後產出三組對照圖，回答：

> LPF 讓 a_hat 更平滑/更準（②）→ 是否讓 tracking error 的**確定性成分**變小（③）？還是重現舊結論「a_hat 改善但 tracking 不動」？

**定位**：對照 study。產出 = 三組圖 + 結論，不是要把 LPF 變成 production 預設。

---

## 2. 架構變更

只在 a_xm 之後插一層 EWMA，KF 結構（state、H、predict/update、gating）**完全不動**：

```
現行 (baseline)                                   新增 (LPF config)
delta_x_m → EWMA(a_pd) → dx_bar_m            ┐
          → dx_r = delta_x_m − dx_bar_m       │ ← 這段不變
dx_r²     → EWMA(a_cov) → σ²_dxr_hat          │
          → affine → a_xm ──────────────► y₂  ┘ ──► EWMA(a_det) → a_m_det ──► y₂
                          │                                                     │
              R22 = K_var·IF_eff·(â+ξ)² + R22_delay      R22_new = R2_intrinsic_new + R22_delay
```

- **flag** `ctrl_const.use_am_lpf`（default false = baseline）切換兩 config，保證兩邊**只差這一層**。
- **a_det = 0.005**（固定，不 sweep；對應舊 sweet spot 量級）。
- a=a_true 臂控制律用真值、不碰量測 → LPF 對該臂無作用（重要：見 §6 P3 免重跑）。
- gating（3-guard）、H 維持原樣。f1Hz 慢軌 gate-free（h̄ 全程 > h̄_safe），所以 LPF 跨 gate 記憶問題在此資料上不發生。
- **LPF lag 不補償**（H 不改）。lag 造成的 a_hat bias 本身是 ③ 要量的東西，誠實留著。

---

## 3. R22_new 推導

記 β = a_det = 0.005，α = a_cov（現行 0.05），φ = 1 − α = 0.95。

### 3.1 cascade 變異

a_m_det[k] = (1−β)·a_m_det[k−1] + β·a_xm[k]

對平穩、自相關 ρ_axm(τ) 的輸入 a_xm，EWMA(β) 穩態輸出變異（標準結果）：

```
Var(a_m_det) = [β/(2−β)] · IF₂ · Var(a_xm)
IF₂          = 1 + 2·Σ_{τ≥1} (1−β)^τ · ρ_axm(τ)
Var(a_xm)    = K_var · IF_eff · (â+ξ)²      ← 現行（已被 verify_axm_cdpmr 驗過）
```

⟹

```
R2_intrinsic_new = [β/(2−β)] · IF₂ · K_var · IF_eff · (â+ξ)²
R22_new          = R2_intrinsic_new + R22_delay        ← delay 項 v1 不動（見 §8）
```

a_xm 是 σ²_dxr_hat 的 affine 映射，常數 offset 不影響變異與自相關，所以 ρ_axm = ρ(σ²_dxr_hat)。EWMA(α) 保均值 ⟹ mean(a_m_det)=mean(a_xm)=a+bias，**LPF 不引入穩態 mean bias**（dynamic lag 才會）。

### 3.2 v1：AR(1) 近似（先行）

σ²_dxr_hat 是 dx_r²（自相關 ρ_dxr² 經由 Isserlis）過 EWMA(α=0.05) 的輸出。EWMA 記憶 ~1/α≈20 步 ≫ dx_r 記憶 ~5 步 → EWMA 極點主導輸出自相關：

```
ρ_axm(τ) ≈ φ^τ = (1−a_cov)^τ          （a 無關 → IF₂ 為常數）
IF₂ ≈ 1 + 2·(1−β)φ / (1 − (1−β)φ)     （precompute 進 build_eq17_6state_constants）
```

數值（α=0.05, β=0.005）：(1−β)φ = 0.94525，IF₂ ≈ 35.5，[β/(2−β)] = 0.002506
→ **net factor on Var(a_xm) ≈ 0.089** → a_m_det std ≈ 0.30·a_xm std（≈ −70% std）。
（與舊 −40% 量級同序；6-state α 與 closed-loop 不同，數字不必一致。）

### 3.3 嚴格版（fallback，僅 P3 驗證不過才上）

若 P3 量到 ρ_axm(τ) 明顯偏離 φ^τ，或 Var(a_m_det) ratio 偏離 1.0，則把 IF₂ 升級成 **a-dependent**：把 `compute_if_abc` 的 s-weighted autocorrelation 機制延伸到「σ²_dxr_hat 的 lag-τ 自相關」再過第二層 EWMA。屆時 IF₂ 變成 if_eff_eval 的姊妹函數（per-step 隨 â 算）。v1 先不做。

---

## 4. 實作變更點

### 4.1 `model/controller/build_eq17_6state_constants.m`
- 新增輸入 `a_det`（default 0.005）、`use_am_lpf`（default false）。
- precompute：
  - `K_var2 = a_det/(2−a_det)`
  - `IF2 = 1 + 2*(1−a_det)*(1−a_cov) / (1 − (1−a_det)*(1−a_cov))`  （v1 常數）
  - `amlpf_var_factor = K_var2 * IF2`  （= R2_intrinsic_new / R2_intrinsic_old 的乘子）
- 寫進 `ctrl_const`：`a_det`、`use_am_lpf`、`amlpf_var_factor`。

### 4.2 `model/controller/motion_control_law_eq17_6state.m`
- persistent `a_m_det`（3×1）。prefill init = `a_x_init`（EWMA 保均值，與 a_xm prefill 同）；legacy = 0。
- a_xm 算完（現行 ~L306）後：
  ```matlab
  if ctrl_const.use_am_lpf
      a_m_det_new = (1 - a_det) .* a_m_det + a_det .* a_xm;   % 3x1
      a_meas = a_m_det_new;                                    % 進 KF 的 y₂
  else
      a_meas = a_xm;
  end
  ```
  用 `a_meas` 取代現行 y₂ 組裝處（gated/ungated 兩路都用 a_meas）。
- R22（~L407）：use_am_lpf 時 `R2_intrinsic_i = ctrl_const.amlpf_var_factor * K_var * IF_eff_i * (a_hat_i + xi)^2`。delay 項不動。
- persist `a_m_det = a_m_det_new`（只有 use_am_lpf 時）。
- 可選 diag 輸出：`a_xm`、`a_m_det` 兩者（給 ① 圖用）。

### 4.3 runner 串 flag
`compare_gain_6state`（及其呼叫 build/controller 的路徑）要能把 `use_am_lpf`、`a_det` 透傳到 ctrl_const。新增 opts 欄位。

### 4.4 驗證腳本（P3）
新檔 `test_script/integration/verify_r22_amlpf_6state.m`（verify_axm_cdpmr_6state 的姊妹）。

### 4.5 出圖腳本（P5）
新檔 `test_script/integration/plot_amlpf_compare_6state.m`（三組圖）。

---

## 5. 五 Phase 驗收標準

| Phase | 內容 | 通過標準 | 需 sim |
|---|---|---|---|
| P1 | R22_new 閉式（§3） | 數值自洽（手算 IF₂、net factor） | 否 |
| P2 | controller + constants + runner flag | `checkcode` 0 issues；baseline（use_am_lpf=false）行為與現行**逐位元相同**（回歸） | 否 |
| P3 | R22_new 驗證 | 見下 V-i/ii/iii | **否（reuse 現有 a=a_true 資料）** |
| P4 | f1Hz 200-seed a=â LPF 重跑 | 跑完、未發散、產出 runs.mat | **是** |
| P5 | 三組對照圖 | 圖產出、summary 數字成形 | 否 |

**P3 驗證三項**（跨 seed pointwise ensemble，沿用 verify_axm_cdpmr 方法論，a=a_true 臂）：
- **V-i**：離線重建 Var(a_xm) ≈ K_var·IF_eff·(a_true+ξ)²（re-confirm 現行；ratio≈1.0）
- **V-ii**：量 ρ_axm(τ) 與 (1−a_cov)^τ 比對（測 AR(1) 近似；偏離大 → §3.3 fallback）
- **V-iii（headline）**：Var(a_m_det) 跨 seed ≈ R2_intrinsic_new 理論，**ratio≈1.0（±幾 %）**

P3 全程**不跑 sim**：a=a_true 臂控制律不碰量測 → a_m_det 只是把現有 runs.mat 的 `diag.dx_r` 離線重放成 σ²_dxr_hat→a_xm→a_m_det（與控制器同 prefill init、同 a_cov/a_det）。跨 seed Var 自動消確定性成分。

---

## 6. 三組圖 + 輸出路徑

輸出根（`test_results/` 已 gitignore）：

```
test_results/am_lpf/f1Hz/
├── r22_verify/                          ← P3
│   ├── fig_r22_var_time.png                跨 seed Var(a_m_det) vs t + 理論線（+ Var(a_xm) 對照）
│   ├── fig_rho_axm.png                      ρ_axm(τ) 量測 vs (1−a_cov)^τ（V-ii）
│   ├── fig_r22_var_scatter.png             Var(a_m_det) vs a 分箱 + jackknife SEM + 理論
│   └── r22_verify_summary.md               V-i/ii/iii 的 ratio/bias 表
├── fig_am_vs_amdet.png                  ← ① 代表性 seed，a_xm vs a_m_det 時域，per-axis
├── fig_ahat_compare.png                ← ② a_hat：baseline vs LPF（a=â 臂，跨 seed mean±σ；bias+spread 分解）
├── fig_tracking_det_compare.png        ← ③ e=p_d−p_true：full（跨 seed std）+ det（跨 seed mean ⟨e⟩）；baseline / LPF / a=a_true 地板
└── am_lpf_summary.md                       ②③ 數字 + 對 2026-04-16 舊結論的對照
```

**定義**：
- ② a_hat bias = (mean(a_hat)−a_true)/a_true；spread = std((a_hat−a_true)/a_true)，跨 seed。
- ③ **det 成分 = 跨 seed ensemble mean ⟨e[k]⟩**（系統性，來自 gain mismatch）；stochastic = 跨 seed std。
- 圖樣式沿用既有：no grid/title、legend northoutside、â 用 Unicode（非 \hat）、scatter = a-值分箱 + jackknife SEM。

---

## 7. 執行順序與並行安排（快又正確）

**關鍵路徑（不可並行、必須先且對）**：P1 → P2。P2 一個 bug 會浪費整批 P4 重跑，所以 P1/P2 由**主執行緒親自做 + checkcode + baseline 回歸（逐位元）**，不外包。

P2 正確後（baseline 回歸通過）：

```
            ┌─ P4：f1Hz 200-seed a=â LPF 重跑（最久，最先排）
P1→P2 ──────┤
            ├─ P3：R22 驗證（reuse 現有資料，無 sim）   ┐ 與 P4 並行
            └─ P5：出圖腳本（先寫好，等 P4 資料）        ┘
```

- **P4 最久 → P2 一過立刻排**（由使用者本地跑，或經同意後我用 Bash/MCP 跑）。
- **P3、P5 用 agent 並行**：P3 驗證腳本、P5 出圖腳本可在 interface 鎖定後由 agent 草擬，主執行緒 review 正確性後跑。agent 只做文字/腳本草擬，不佔 MATLAB MCP；動態驗證主執行緒或使用者跑。
- **correctness gate**：P2 後（baseline 回歸）、P3 後（V-iii ratio≈1.0）兩個檢查點給使用者把關，再往下。

---

## 8. 已知 caveats

1. **colored-measurement 次優**：a_m_det 高度自相關，scalar R22 把它當白噪 → KF 形式上次優。**這是 study 要量的東西之一**，③ 會誠實顯示有沒有傷 tracking。嚴格解（state augmentation）是另一個更大實驗，本 study 不做。
2. **LPF lag 未補償**：a_m_det 落後 a(t)，dynamic osc 上造成 a_hat lag/bias，③ 觀察。
3. **IF₂ v1 為常數（AR(1) 近似）**：P3 V-ii 不過才升 a-dependent（§3.3）。
4. **R22_delay 不動**：delay 項相對 R2_intrinsic 小，v1 保持原樣；P3 的 var theory 本來就只比 R2_intrinsic（與 verify_axm_cdpmr 一致）。
5. **a_det 固定 0.005，不 sweep**：本 study 範圍。

---

## 9. 對照舊結論（2026-04-16）

| 項 | 2026-04-16（7-state, positioning） | 本 study（6-state, osc f1Hz） |
|---|---|---|
| a_hat std | −40%（alpha=0.005 sweet spot） | ② 量，看是否重現 |
| tracking error | 變化 <1%（沒改善） | ③ full + **det 成分**分開看 |
| R22 處理 | 未重推 | **重推 + 驗證（P3）** |
| 結論 | 「pre-EKF LPF 方向錯」 | 待 ③ 定（det 成分是新角度） |
```

