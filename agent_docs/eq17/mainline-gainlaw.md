# 估測函數主線現況（gain-law family）

**每 session 自動載入的主線快照**。章程與約束見 CLAUDE.md「研究主線」節；本檔補檔案級對應。
更新時機：形式、入口、缺陷狀態任一改變時。

## 兩個對等形式

| | powerlaw | expgain |
|---|---|---|
| gain law | c(h̄) = 1 + K/(h̄−1)^p | a_h(h̄) = a_o·[1 − h̄^(−b)]，φ = ln h̄ |
| state | [δh̄₁ δh̄₂ δh̄₃, a_h, p] | 7b: […, a_h, b]／7a: […, a_o, b]（gain 代數求值） |
| controller | `motion_control_law_5state_powerlaw.m` | `_5state_expgain.m`(7b)／`_5state_expgain_alg.m`(7a) |
| driver | `test_script/integration/run_5state_powerlaw.m` | `test_script/integration/run_5state_expgain.m`（opts.variant='diff'/'alg'） |
| 推導 | `derivation/5state_powerlaw_hd.tex` | `derivation/5state_expgain_hd.tex` ⚠ 正文停在 7b，code 以 7a 檔為準 |
| 長短處 | ⊥ 最準（sup 0.034）；∥ 結構性失效（Goldman 對數非冪次） | 三軸可用（∥ h̄≥2 0.034）；⊥ 差 2.8× |

- prior：兩漸近錨（Brenner 近壁 + method-of-reflections 遠場）都逼出 p=1／b=1；
  √P[0] = 內插間隙 sup|θ_eff−1|（p: 0.035、b: 0.10），全域、與軌跡無關、不讀 c
- 種子：反射係數 9/8（⊥）、9/16（∥）；種子誤差係數 = A²（推導非擬合）→ 應 per-axis
- 兩形式 run-time 完全 c-free（全家唯一不呼叫 `calc_correction_functions` 的 controller）

## 驗收工具（形狀或 prior 改動後必跑）

- `verify_shape_exponent_bound.m` — sup|θ_eff−1| ≲ √P[0] ⟺ Q_θ=0 誠實（PASS/TIGHT/FAIL）
- a_cov 不變性 — `verify_5state_expgain_acov.m`／`verify_powerlaw_regress_A12.m`
- P[0] 預算 — E[(x̂∞−x̂₀)²] ≤ P[0]−P[∞]

## 第三形式：Form B（2026-07-31 起，w̄_s 能力線）

- gain law：ā = 1 − (1 + (w̄−w̄_s)/b)^(−p)，θ=(b,p,w̄_s) 錨 (9/8, 1, 1)；全正規化（/R、/a_o、f̄=a_o f）
- controller `motion_control_law_formB_ws.m`（7-state、lock 旗標階梯）；driver `run_formB_ws.m`（包絡 prior run-time 自推）
- 推導 `derivation/formB_ws.tex` + `formB_ws_ref.tex`（含 P[0] 包絡修訂、D3 Q 容器、末三頁 = c2/c3 進度圖）
- Tier-1：anchor-lock desc 1.20%；注意「鎖錨勝」在 sim 為對答案（真值≈錨族），正確讀法 = 錨對時估測器仍被慢偏差拖壞 → c2/c3（估 w̄_s）為當前工作線

## 現況（2026-08-01）

- 缺陷 1 已由 7a 代數式修復（descent 11.03% → 5.05%）
- **缺陷 2 已撤案**（2026-08-01，N=48：+1.33% n.s.、運動框架溶解、6-seed = 1-in-18 高抽樣；
  詳 memory `project-formB-tier1-defect2-retraction-2026-08-01`）。**Cdpmr_Cn 含運動重推無標的，勿再立案**
- **ε_w MA(2) 增廣已落地（08-01，`ma2_aug` default TRUE）**：白 Q 低估 DC 功率
  (1+2α)²/(1+2α²)=2.17×；m₁/m₂ 記憶州（slots 8/9）+ rank-2 Q（Q₃₈/Q₄₈ 承重）；
  innov_y₁ 白化 [0.30,0.24]→[0.003,0.015]、C1 b 預算轉綠。β_w 推導下沉 archive
- **y₂ 自迴音修正已落地（08-01，`y2_echo_corr` default TRUE）**：讀數跟隨施加增益
  （迴路極點位移），S=0.32（Lyapunov 推導 0.319 / 配對實測 0.323±0.043 互證）；
  H₂×(1−S)；ŵ_s honesty 1.42→0.93。「y₂ 凍結偏差」= seeds-1:20 幽靈（全新 40 seed
  否證，讀數鏈 χ² 級乾淨）
- **單 run 已在 CRLB（08-02 效率審計）**：配對靈敏度界限 ~0.09 vs 濾波器 0.080；
  散布 = 誠實後驗實現。**根治 = 校正鏈（c5）**：後驗遞移 8 runs ±8.9%→±2.0% 貼 √P、
  作業 run desc 1.69% ≈ 鎖定地板。注入機制 `opts.ws_inject` 就位
- **開發計畫（08-02 五 Stage，使用者核准）**：0 收尾 ✅ → **1a ∥ 軸法則 ✅（08-02）**：
  Goldman 對數 ⇒ ∥ 原點必在牆內（x/y 不能報牆位）；driver init 包絡自推
  (b,p,ws0)_∥=(0.5217, 0.9770, 0.5918)、x/y 鎖 5–7 槽、原點單向掛 z 的 ŵ_s（+Δ_∥）；
  x/y 增益健康 40×（desc 34.8→0.92%、hold −25→−0.04%），z 逐位不變，legacy `_nopar`；
  c6 圖頁 → 1b 融合對決實驗（z-only vs z+xy 鏈）→ 2 共享 ŵ_s → 3 校正正名+S11 →
  4 D3 prior 物理錨＋漂移 q（等使用者物理數）
- **讀數鏈數字紀律**：6-seed SEM ≈ 2.6%，1% 級宣稱需 N~200；07-28 頻率/a_pd 梯視為未解析
- ⚠ 家族未修項：predict 一步時序 lead（formB 已修；expgain/powerlaw 同構未修）
- 兩形式 prior margin 皆 TIGHT（1.02–1.06×）→ 對照與定案門檻見
  `reference/eq17_analysis/shape_ledger.md`
