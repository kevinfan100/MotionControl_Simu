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

## a′_true 作弊臂（2026-09-02/03，formC_b，z 軸）

目的：斜率是真的（ā′_true 由 c(h̄) 算、讀在估測器自己的高度 w̄_d − δŵ̄₃）時，估測器還會不會自己偏。
答案：不會，到 10 seeds 的解析度（牆邊 ā 的 0.3%）為止。

- **配方**（全 default-off 旗標）：`law_exact_step`（已知步長精確積分）＋ `ap_known_at='est'`
  ＋ `pred_mean2`（predict 二階均值：Jensen／起點差／曲率差，P 為主）＋ **`nw_mcorr`**
  （相關 process／量測雜訊 KF：控制器對同一份 y₁ 雜訊反應 ⇒ M = R₁g_n ≠ 0；predict 加輸入
  g_n(y₁ − x̂₁)、F_e(:,1) − g_n、Q − R₁g_ng_nᵀ）。`pred_mean2_kr1`／`_full` 是撤回的過度補償，勿開。
- **驗收**（同 10 seeds、canon deep 與 Meng ramp、hold 拉長 4 s）：hold 偏差 −0.00035 ± 0.00026／
  +0.00017 ± 0.00068；長 hold 斜率 −0.014 ± 0.041／−0.023 ± 0.075（e-6/步，修前 +0.239／+0.126）；
  散布 = √P 不變；追蹤誤差不變。收官 100-seed（<0.1%）未跑。
- **文件**：`derivation/0903_aptrue_4state_from_true.tex`（母本，斜率寫在命令高度＋補償項，等價於
  code 的 @est；最後一頁 = 最終圖）、`0903_aptrue_4state_from_true_mcorr.tex`（同順序含 nw_mcorr，
  待併回母本）、`0902_formC_aptrue_4state.tex`（header = 整段歷史）、`0903_formC_aptrue_update_half.tex`
  （update 半步、引理 ℓ₄₁ = −ā′ℓ₃₁、相關雜訊 KF；獨立驗算：4-state 線性內 M 只解釋量到 Cov 的 1/20，
  「M → 漂移」因果鏈待 9-state 版）。
- **延伸 b_true**：搬同四塊，多一行 −(∂ā′/∂â)e₄·步長（均值 +2b(1−â)[(1−λc)P₃₄ + F_dwP₄₄]），先推導再接線。

## 現況（2026-08-03）

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
- **⚠ 缺陷 3 立案（08-03，長 run regime；08-04 量值下修）**：w̄_s 長 run 下偏——
  **方向確立**（合併 27 seeds 22/27 低於真值 p~1.5e-3＋無注入臂 4/4＋結構機制證明），
  **量值 band 依賴且重尾**（兩個 12–15 seed band 給 −5.35% vs −0.85%，Welch p 0.004；
  單 band 的 −0.15%/s／12/12 不可再引用）。三臂消融排除 ridge/echo 元兇/更新飢餓；
  **主項已定罪（08-04 derivB）：y₁ 的 1 Hz 反相整流**——K_ws,y1∝命令速度 ×
  ν₁ 未建模追蹤 lag 殘差（δw̄ predict 鏈無命令運動確定性輸入）= −κ⟨ẇ²⟩，
  結構性、非資訊性（估測在真值下方仍下拉）、自限 ∝P_ws·A²ω²（＝「凍結在錯值」）；
  y₂ 為同根下游迴音（derivA 三臂判讀數鏈無罪、缺陷 2 不翻案；正 innov 全來自估測偏低，
  y₂ 修 â̂ 卻經 K(7)<0 再壓 ŵ_s；消融拿掉 ~16–30% 與此相容）。
  放大條件 = Q77=0＋hold 不可觀＋ā 永不重錨。
  audit 另揭 **y 軸 meas_noise 0.00057 vs 規格 5.7e-5 差 10×**（07-14 起全家族）。
  **4.8s 場景審計結論不變**；c5 校正鏈不受影響；長 run 辨識路線修復前不可用。
  **⚠ 修復路線一號（g_det 確定性前饋）已否證（08-04，停在 Stage-2 閘門，production 未動）**：
  H_loop≡1 精確恆等（前饋完全反演模型，信念增益對時零確定性 lag）；1 Hz 殘差 = 參數誤差
  信號本身（τ = 5.333·Ts·|ε|，隨 ε 翻號）⇒ 前饋會致盲 w_s，結構自我矛盾；
  「1.5e-3 R lag 特徵」370× 是雜訊實現（跨 24 seed 可重複部分 = 0）。
  **新主線索 = 響應赤字**：10.8s 回收僅 44%（t=−4.0）< 4.8s 的 59%，反 Bayes 趨勢——
  比漂移率統計上硬。機制歸因重開，下一步 = 紙筆推導「參數誤差信號×K∝速度的期望不對稱」。
  窗速率 per-seed sd ~0.37%/s ⇒ 分項量值宣稱需 ~15 配對以上。
  詳 memory `project-formB-longrun-ws-drift-defect-2026-08-03`＋`project-gdet-feedforward-falsified-2026-08-04`
- **開發計畫（08-02 五 Stage，使用者核准）**：0 收尾 ✅ → **1a ∥ 軸法則 ✅（08-02）**：
  Goldman 對數 ⇒ ∥ 原點必在牆內（x/y 不能報牆位）；driver init 包絡自推
  (b,p,ws0)_∥=(0.5217, 0.9770, 0.5918)、x/y 鎖 5–7 槽、原點單向掛 z 的 ŵ_s（+Δ_∥）；
  x/y 增益健康 40×（desc 34.8→0.92%、hold −25→−0.04%），z 逐位不變，legacy `_nopar`；
  c6 圖頁 → **1b ✅ 判決 ws 維持 z-only**（x/y ws 資訊僅 z 的 6–7%/軸，
  功效分析否決融合；`par_ws_free` 留作重開儀器；Stage 2 取消；c7 圖頁）→
  **3 校正正名+S11 ✅（08-02，exam v2 全綠）**：chain 協議 SSOT
  `run_formB_ws_calibration.m`＋考卷 `verify_formB_s11_calibration.m`；判準修訂
  （V1 對「真值+健康暫態」、V3b 對「1σ-牆位鎖定地板」）；V1± / V2 / V3a / V3b 全 PASS；
  V1− 殘餘 +0.9%±0.5% 帶內觀察項。**「停滯」= 本輪第四隻幽靈**（零基準框架錯 + 小 N
  post-hoc；40 鏈儀器否決：κ=宣稱、零截距；二階曲率帳 +7e-5 = 1.3% 已豁免，修正式備查）→
  4 D3 prior 物理錨＋漂移 q（等使用者物理數）
- **讀數鏈數字紀律**：6-seed SEM ≈ 2.6%，1% 級宣稱需 N~200；07-28 頻率/a_pd 梯視為未解析
- ⚠ 家族未修項：predict 一步時序 lead（formB 已修；expgain/powerlaw 同構未修）
- 兩形式 prior margin 皆 TIGHT（1.02–1.06×）→ 對照與定案門檻見
  `reference/eq17_analysis/shape_ledger.md`
