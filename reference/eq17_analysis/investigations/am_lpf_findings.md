# a_m LPF (a_m_det) study — findings (封板)

Date: 2026-06-18
Branch: test/motion-test (全部未 commit)
Scope: f1Hz, 6-state + 5-state(無 x_D)testbed
Spec: [`am_lpf_r22_design.md`](am_lpf_r22_design.md)

---

## 0. 一句話結論

> **dynamic 軌跡下,在 a_xm 後面加 pre-KF LPF(a_m_det)不該用:窗設錯(長)近壁有害、窗設對(=a_cov)無益且和 KF/a_cov 冗餘。a_hat 近壁的 ~15–20% 限制是「從熱噪聲量 a」的資訊底,LPF 碰不到——要更好得換資訊源(feedforward),不是後濾。**

延伸並印證 2026-04-16 舊結論(當時 7-state positioning;本次 6/5-state + dynamic + 重推 R22 + det 分解全驗過)。

---

## 1. 目標與設計

a_xm 後串一層 EWMA(a_det)→ a_m_det 再進 KF,重推 R22,出三組對照圖(① a_m vs a_m_det、② baseline vs LPF 的 a_hat、③ tracking error & det 成分)。決策:scalar R22 cascade 近似、f1Hz、det=跨 seed ensemble mean、重跑 a=â 臂。

實作:flag `use_am_lpf`(預設 false → **bit-identical baseline**:amlpf_var_factor=1 + a_meas=a_xm)。

---

## 2. 核心發現 1 — R22 沒壞,壞的是 LPF 的長窗 cascade

R22 ≠ R22_new:
```
R22     = Var(a_xm)                          ← 原始,已驗證
R22_new = R22 × [a_det/(2−a_det)]·IF₂        ← LPF 新增的 cascade 因子
```

驗證(a=a_true 臂,target ratio≈1.0,a_det=0.005):

| near-wall z | ratio |
|---|---|
| V-i: Var(a_xm) = R22(原始) | **1.003**(完美,連近壁) |
| V-iii: Var(a_m_det) = R22_new | **2.073**(偏 2×) |

→ **R22 從頭到尾正確(近壁也 1.003)。偏的只有 LPF 引進的 cascade 因子。**

**為什麼 cascade 近壁偏 2×**:不是自相關模型(AR(1))太粗——rigorous a-dependent IF₂(`compute_if2_rigorous.m`)只把 near z 2.07→1.87(沒救回),且把遠場過修到 0.88。真因是 **a_det 的平均窗(~125ms)≈ 近壁 a(t) 掃過谷底的時間 → 窗內 a 非平穩**,任何「用瞬時 a 的公式」都抓不到。對比 a_cov 窗(~12.5ms,短 10×)→ a 近壁也近似常數 → R22 驗得過。

---

## 3. 核心發現 2 — a_det sweep + α 地板 recipe

離線重建(a=a_true 臂)掃 a_det:

| a_det | 窗(ms) | std_ratio (a_m_det/a_xm) | near z ratio (crude/rig) |
|---|---|---|---|
| 0.005 | 125 | 0.30(−70%) | 2.07 / 1.87 ✗ |
| 0.020 | 31 | 0.53(−47%) | 1.21 / 1.10 |
| **0.050** | **12.5** | 0.71(−29%) | **1.12 / 1.03 ✓** |

**a_det=0.05(=a_cov)→ R22_new 全窗回正(near z rig 1.03)。** 證實 2× 純粹是長窗非平穩。

**α 地板 recipe(三個 α 共用約束)**:窗必須 ≪ a(t) 變化尺度 →
```
α_floor ≈ M·(2π·Ts)·|K_h(h̄_min)|·A·f / R     (M≈10,評在 K_h·速度 乘積峰值 ~h̄1.5)
```
代入 f=1Hz/A=2.5µm/h̄_min=1.2/R=2.25 → **≈0.058 ≈ 實測 0.05**。∝ f、A、|K_h|;換場景要縮放(2Hz→~0.1、0.5Hz→~0.025)。

**建議設定**:`a_pd = a_cov = α_floor`(≈0.05 @ f1Hz,取「還能追的最小值」= 噪聲最小又不 lag);**a_det 不要加**(冗餘),硬要就 `= a_cov`,絕不更小。

---

## 4. 核心發現 3 — 閉迴路(5-state, a_det=0.05, 200-seed):LPF 無益

窗修對後重跑(near 為 h̄<1.5,z 軸):

| z near | baseline | LPF a_det=0.05 | (對照)a_det=0.005 |
|---|---|---|---|
| tracking **det** RMS | 8.2 nm | **8.5(持平)** | 16.8(翻倍) |
| tracking stochastic | 16.4 nm | 16.9 | 19.3 |
| a_hat bias | +19.7% | +24.5% | +96% |
| a_hat spread | 31.0% | 35.9%(略升) | 34.3% |

- **近壁災難消失**(det 8.2→8.5,vs a_det=0.005 的翻倍)。
- **但 LPF 零好處**:tracking ±1nm 不動、a_hat spread 甚至略升(a_det=a_cov 冗餘 + KF 已平滑)。
- x 軸近壁幾乎不受影響(c_∥ 平緩);只有 z(c_⊥ 近壁陡)被咬。

---

## 5. 核心發現 4 — a_hat 近壁是「熱量測資訊底」,非後濾能解

a_xm 從熱運動變異反推 a → chi-squared 噪聲底 rel std ≈ √(2/N_eff)。
- **靜態 a**:窗可任意長 → 底→0 → 近乎完美。
- **動態近壁**:窗必須短(追 a(t))→ N_eff≈τ_a/Ts≈176 → √(2/176)≈11%(含相關性 ~15%)→ **對上實測 near z spread 31%**(=raw chi-squared √(2/20),KF 因 a 快變壓不下去)。
- **a 大(遠場)→ a_xm 接近 mean**:熱訊號 ≫ sensor floor(ξ/a→0,ξ=(C_n/C_dpmr)σ²_n/4kBT)+ a 慢 KF 能重平均 → far z spread 降到 9%。**a 小(近壁)→ a≈ξ 放大 + a 快 → 31%。**

**要逼近完美 a_hat 的真正槓桿(非 LPF)**:
1. **feedforward / model-based**:`a_nominal = Ts/(γ·c(h_measured))`(控制器已算,只是控制律沒用)→ 近壁 ~1%(δa/a=K_h·σ_n/R),但靠模型準(犧牲 robustness)。
2. 加 SNR / 放慢軌跡(降資訊底)。
3. 動 KF Q55(在迴路內、wall-aware,但有自己的 bias-variance)。

---

## 6. 檔案 / 圖 / 資料盤點(全未 commit)

**code(改)**:`build_eq17_6state_constants.m`(amlpf_var_factor)、`motion_control_law_eq17_6state.m` + `_5state.m`(a_m_det EWMA + a_meas + R22 scale)、`run_pure_simulation.m`(use_am_lpf/a_det pass-through)、`compare_gain_6state.m`(use_am_lpf/a_det/variant)。

**新增**:`model/controller/compute_if2_rigorous.m`(a-dependent IF₂);`test_script/integration/`:`verify_r22_amlpf_6state.m`、`run_amlpf_rerun_6state.m`、`plot_amlpf_compare_6state.m`(opts:lpf_file/det_only/no_movmean/show_seed_n);`reference/eq17_analysis/investigations/am_lpf_r22_design.md` + 本檔。

**資料/圖(test_results,gitignored)**:
- `am_lpf/f1Hz/`:6-state ②③(a_det=0.005)+ `r22_verify/`(crude/rig)+ `r22_verify_adet{0.005,0.02,0.05}/`。
- `am_lpf_5state/`:`baseline/`、`lpf/`(a_det=0.005)、`lpf_adet0.05/`、`compare_adet0.05/`(②③ 定案)、`compare_adet0.05_raw/`(無 movmean / 單 seed 探索)。

**圖樣式**:â 用 Unicode(非 \^a)、no grid/title、legend northoutside、exportgraphics 150dpi。

---

## 7. 後續(未做)

- feedforward / model-based a 混合(發現 5 的方向)——能真正逼近「近乎完美 a_hat」。
- 換頻率(0.5/2Hz)用 α 地板 recipe 縮放後驗證。
- merge / commit。
