# 估測函數主線現況（gain-law family）

**每 session 自動載入的主線快照**。章程與約束見 CLAUDE.md「研究主線」節；本檔補檔案級對應。 已結案的舊線在 `mainline-history.md`（不自動載入）。
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

## 勿再提（已否證，沒有新證據不得重提；細節在 `mainline-history.md` 與假說帳）

- b 上加任何容器（R51／R57–R63、Q₅₅、δa、κ、E1／E2、b_ceil、O26）；κ 在所有讀法下否證，production κ = 1。
- [2/2] 四常數當「track 慢」的解（C59）；單一 s 當所有牆通用族（R64）。
- 缺陷 2、「y₂ 凍結偏差」已撤案；Cdpmr_Cn 含運動重推無標的，勿再立案。
- δ 法則只對平面平移族有效（C70），不再停留。

## [1/1] Padé 兩常數線（2026-09-16/17 起，當前工作線）

**裁示**：b 當常數從推導到建模都是根本錯誤（平面 b_true 牆邊 1 → 0.867 → 8/9）。新原則 = 估的參數在建模時就必須是常數，
先拿 plant 驗常數性再建。**notation 全用 Padé 係數 (c₀, d₁)，不再出現 b**（對舊圖只用 d₁ = c₀ + 1/b 換算）。
**座標（09-18 裁示，C61）：w̄_s = 牆面位置**（平面 0；細胞 = 可見表面），w̄ − w̄_s = h̄，接觸在 1，法則 ā = ((w̄ − w̄_s) + c₀)/((w̄ − w̄_s) + d₁)
不含 −1；常數：平面 (−0.984, 0.161)、細胞 (0.056, 1.197)，κ = d₁ − c₀ = 1.145／1.141；−c₀ = 增益歸零的中心高度（細胞 −0.06 ⇒ 真牆在可見表面下 1 R）。
09-16/17 的圖與帳（C54–C59）用接觸高度座標（w̄_s = 1），常數各 +1，模擬逐位相同。controller 的 `ws0_perp` 直接收 w̄_s。
**C62（09-18 晚）：細胞 = 精確平面多項式往下移 δ = 1.05 R，零形狀參數**（`probe_plane_offset_law.m`：rms 0.0061、2.6%，與 [1/1] 同水準；加 s 沒買到；
球面才要形狀）。細胞的牆種是位置型 ⇒ 候選 = 精確多項式 + 一個位置型 state δ（舊 ŵ_s 線的想法配對的法則）；未實作。R64 細胞段修正。


> 09-16 ～ 09-21 的 [1/1] 兩版、定高、速度、[2/2]、δ 法則、版本二細節已移到 `mainline-history.md`。

  **裁示（09-21，C70）：δ 法則只對平面平移族有效、對球面無效 ⇒ 不再停留在 δ。測試協定 = 估測器一律 init 在平面、牆種只給 plant、三牆並列
  （`.claude/rules/wall-test-protocol.md`）；seed = plant 與中點 prior 降為參考臂。下一步 = 從「對任一牆 ā 準」從頭判斷估測器設計。**
  **09-22：F2 兩常數斜率式 ā′ = (1−c)/(h+δ)² 落地（C75，旗標 `plaw_dc_slope`，default off，ALL PASS；runner law 'f2'／'f2p'，plant 'sphere_pt'）**：只吃 h 的兩參數式裡只有帶平移 δ 的三牆常數走幅 ≤ 0.2 且細胞精確；離線指標改為累積斜率誤差 max|∫e_ā′ dh|（`probe_slope_accum_walls.m`；c 必須放：只放 δ 球面 0.051）。推導 `0922_estb_f2_slope_dc.tex`（含 19 項展開頁，sympy 33/33）：∂ā′/∂δ = ∂ā′/∂h ⇒ F_e(4,7) = −高度份、H₂(7) = −H₂(3)；(c, δ) 凍結係數 det ≡ 0，兩高度才分。`verify_obs_f2.m` 對 code 1e-16、rank 6/6、c/δ motion-only（cell c 0.89）。canon 平面 init：plane +0.031／−0.005、**cell +0.047／+0.015（v2 +0.146／+0.083）**、sphere −0.030／−0.003；Meng cell 與 v2 持平（慢坡 = 位準資訊，O42）。開放：遠場 δ̂ 晃動（O40，plane／sphere 谷底相對 32–35%）、hold 位準是讀數鏈的（O41，y₂ innovation ≈ 0）。球面尾巴對照 plant `sphere_cperp_fig11_ptail.m`：近壁結論不依賴尾巴。圖 `f2_walls_PI{,_meng}`、`_spheretails`。
  **09-23 換座標 (δ, p)（C76，`plaw_dp_slope`，runner 'f4'，`0923_estb_f4_slope_dp.tex`，sympy 30/30）**：ā′ = 1/(h+δ)^p，接觸斜率釘 1。四五關兩常數都變好（p 0.14／0.28／0.14、δ 0.013／0.22／0.010）；**ā 準度三牆兩軌跡與 F2 相同**；p̂ 六格收到各牆 fit ±0.02–0.06、δ̂ 更靠 fit；誠實比 1.5–2.4。座標買常數不買 ā。O40 判別（R67）：種子交叉項不是遠場晃動的來源。圖 `f4_walls_PI{,_meng}`。
  **09-23/24 教授式 (c, p) 兩常數 → c-only（p = 2）→ c 槽遺忘（C77／C78、R68、O43／O44；旗標 `plaw_cp_slope`、`r2_scale`、`lambda_f_b_{chan,info}` 都 default off，ALL PASS；runner law 'f1'／'f1p2'、traj 'ramp'；memory `project_cp_slope_law_obs_and_forgetting_2026-09-24`）**：
  可觀性（obs_dump 的 G，只看 σ_i/v_i，負控制 slot 6）——常數的資訊只在 h_d ≲ 3–5 R 且在運動中，定點 22.2 R 與 hold 零貢獻；(c, p) 弱方向恆為 0.7c − 0.7p、細胞對平面的差全在其上 ⇒ 一趟釘不住 p；速度只換精度不換均值（T_d 三速 × A/B × plane/cell）。
  c-only 細胞 hold +0.114（牆邊 +0.131），與 prior、T_d 無關（ĉ 0.33 對 1.1 R 要的 0.74）；根 = c_eff(h) = 1 − h²ā′_true 在細胞走 −0.02 → +0.74，而模型 c[k+1] = c[k]、Q_cc = 0 ⇒ **P_cc 只縮不長是 KF 估常數的正常行為，不是缺陷（使用者 09-24 定調）**；1/P 100 → 1500、L_c ∝ P_cc ⇒ ĉ 追不上。R₂ 整體 ÷2 無效。
  遺忘（congruence P₅₅ ÷ λ、交叉 ÷ √λ，λ_k 記在 `lam_b_out`）：uniform 0.999 遠場沒資訊照忘 ⇒ windup（√P 0.10 → 0.31）、hold ×2.2／×8；Meng 博論 (6.11) 單樣本 NIS 觸發每步 32% 與 α 無關 ⇒ 盲（R68）；**資訊驅動 λ_k = max(exp(−3 s_k), 0.99)，s_k = 1 − P₅₅⁺/P₅₅⁻（只由 P⁻、H、R 定）：細胞 hold +0.008（牆邊 +0.012）、ĉ 0.60、平面 hold +0.005，代價近壁散布 2–3×、hold 內 √P 0.118 → 0.154（O44）**。等價 Q_cc ≈ β s_k P_cc。未跑候選：β 掃 {2,3,5}、ib3 + 3 s hold、視窗 NIS、導出 Q_cc = (c_eff′(h)M)²（O43）。
  文件：`0924_report.tex`（與 Layer-1 session 共享；規範 = 一節一頁、只放標題＋模型 align*＋圖、不放讀數；§3–§7 (c,p) 與 c-only 的可觀性＋模擬、§8 P 遞迴與 λ 進入點、§9 細胞三欄 β = 3｜0.999｜λ = 1）、讀本 `0923_obs_check_core.tex`。使用者收尾：回頭從頭重講 P 與 λ_f（卡點見 `feedback_lf_explanation_sticking_points_2026-09-24`）。
  **09-27~29 理解階段：可觀性矩陣從頭讀本（C79／C80／C81、R69、O45／O46；memory `project_obs_matrix_from_scratch_2026-09-29`）**：使用者定「先理解到能自己講，再談 λ_f 配方」。
  文件 `0928_obs_c_only_v1.tex`、`0928_obs_cp_v1.tex`（§1 模型／§2 矩陣／§3 what it shows，只放式子，斜率式 SSOT 符號，格式同 Layer-1 的 `0927_obs_first_order_v1.tex`；§4 加權平均／§5 KF 未寫）。
  要點：c 只在 F_e(4,5)、H₂(5) 兩格；c 欄 = 走過的累積 s^c = 1/ĥ − 1/ĥ₀（與 T_d 無關）；ā 欄平 ⇒ 分開 ⇔ β ≠ 0；平的那份歸 ā；hold M = (1−λc)δŵ̄₃ 抖動不累積；**首階省略合法因 F_e44 與 F_e43 迴路相消**（單留一項錯 2.5×，殘餘細胞 7%／平面 25%）；(c,p) 3×3 det ∝ Δln ĥ、近壁 r = s^p/s^c ≈ 0.9 ⇒ 弱方向 (0.7, −0.7)，細胞的差 0.82/0.83 在弱方向。
  判別：**谷底不是槓桿**（細胞 1.10→1.05→1.04 ĉ 0.331→0.345→0.348；1.03 以下碰牆、平面撞增益底線 0.05）；**ā 誤差 99% 來自 c 塊**（+0.272，二階 −0.003，濾波器拉回 51% ⇒ +0.131）；**R₂ ÷3／÷10 否證**（均值不動、hold −0.035、ĉ 散布 ×2、NIS₂ 2.85；c 資訊在近壁走 y₁）。
  講法：欄的形狀比較、KF 兩條軌（P/L 排程不看資料，x̂ 那行才看）、`c_slope_walk.png`（c 是斜率參數，走過才成值差）。code：`run_formC_b` `opts.allow_below_floor`、`run_plaw_walls` `opts.h_bottom`。未跑：lock-c 判別（O45）。
  **09-30～10-02 c 槽的 Q 設計 = Q/R 比（C82–C85、R70–R74、O47–O51；memory `project_q55_ratio_design_2026-10-02`；文件 `0930_q55_ratio_design.tex` 15 頁；旗標 `q55_adapt` ∈ {ratio, rhoconst, r2ratio, ratiogate} 全 default off，production defaults ALL PASS）**：
  I_k = 1/P₅₅⁺ − 1/P₅₅⁻（這步兩通道給 c 的資訊）、R_k = 1/I_k、ρ_k = (P₀/R_k)² ⇔ **Q₅₅ = P₀²·I_k ⇒ P₅₅ 的平衡點 = P₀**（補回 = 站在 P₀ 時會被拿走的量；沒資訊兩邊皆 0、不夾住）。讀法：ĉ = 最近 1/P₀ 份證據的平均，P₀ = ĉ 的步伐。
  √P₀ 0.1（prior）細胞牆邊 +0.073／**0.2 +0.016 ± 0.034、hold +0.011**／0.3 −0.001 ± 0.046（base +0.131／+0.114；ib3 +0.012；作弊 +0.017）；一個常數跨 T_d 0.3–5 s 成立（0.2 s 撞牆）。對手全量：ρ 常數炸（R70）、字面 R₂ 比例四個 ρ 蹺蹺板＋hold 長（R71）、Q₄₄／R₂/3 對振盪頂端無效（R73）、波形無差（R74）、帶內跟 c_eff 擺反而壞（R72）。
  ĉ 對誰：紅線 c_eff(h_d) = 1 − h_d²ā′_true（局部真值）、常數的位準中性值 c* = Σ c_eff Δs^c／Σ Δs^c（cell 0.483，隨起點變）；band fit 已從圖上拿掉。振盪 = 路徑可逆的位能，任何常數在 1.1–3.3 R 擺 ±0.026（C83）。
  軌跡：canon 下坡是半餘弦 ⇒ 新 `osc_linear`／`osc_tri`（預設不變），runner `traj 'osc'` 配 `descent_linear`／`wave`／`f_osc`／`h_bottom`／`h_top`。等速 + sin：谷底與 hold 由 ratio 修到 ±0.01、頂端留 +0.03；窄帶 1.05–2.0 hold −0.000 ± 0.043、ĉ 第二谷底 0.75；0.5 s 下坡 + 1.10–2.0 R 1 Hz：cell hold −0.018 ± 0.044，plane 第一頂端 −0.037（暫態把 ĉ 拉到 −0.1）。
  **ratio × innovation 閘**（|mean ν₁|₁₀₀ > 0.2）：plane 第一頂端 −0.004、ĉ sd 減半、cell 振盪 hold −0.002 ± 0.028；代價 = 單趟等速閘只開 18% ⇒ cell 牆邊 +0.055（C85）。同軌跡排序：閘 > ratio 常開 > R₂ 比例。
  未跑：motion gate（hold 漂，O49）、放法 C = Δc_fam²|M|/ĥ²（O50）、共形版 ratio／逐 seed 位能（O48）、閘靈敏度 thr 0.1／CUSUM（O51）、P₀ 的錨（O47）。c-only 的二階 E[e×e] 補丁（pred_mean2／nw_mcorr）全關，量級 1e-3（C81），c 塊壓到 0.016 後是剩餘的 20%。
  **10-04～05 ramp_tri 上的 c 的 Q（C88–C89、R76–R78、O52；memory `project_q55_memory_by_height_2026-10-05`；文件 = `motion_ctrl.tex` part2，1004 已 SUPERSEDED）**：P₀ 無關（定 √P₀ 0.1）；單一 √P_fix 是取捨線（0.15 折衷）；innovation／covmatch／加法漂移全否證；**rationew（新高度 0.15／重訪 0.05，`q55_adapt_Prev`）** 首次跳出取捨線；三角段誤差 80–90% 是 ĉ 的 seed 散布（c 已知臂 0.009／0.003），5 Hz 同帶配重訪不忘 plane 0.019 → 0.011、cell ĉ 散布減半但偏差 +0.011。
  **結論（O52，使用者 10-05 問「是不是根本方向錯」）**：c_eff 隨高度變，常數 c 本該 Q = 0；Q₅₅ 的取捨、分段、無推導的 0.15／0.05 都是補模型誤差 ⇒ 停止調 c 的 Q，下一步 = F2 (c, δ)（`plaw_dc_slope`）在 ramp_tri 三牆、Q = 0 對 rationew（未跑）。
- 之後候選（在新協定下重評）：估函數 vs 估值配對（6-state 對 δ 法則，同 case，皆平面 init）；O37 中速過度自信的來源（y2_whiten 或 innovation 自相關判別）；遠場 y₂ 著色進 R₂（縮近壁尖峰，O8 族）；O35 下坡後 P 縮過頭的判別（法則 plant 上看誠實比）；階梯／第二趟快速下坡；O34 定高收斂機制；O32 決定 [1/1] 線接回哪些修補；O30 球面（要形狀）；O33 換牆偵測＋重置。

## Layer-1 對照線：沒有 a′ 的 5-state（2026-09-20~22，收工）

- 論文階梯的對照組：state [δx₁ δx₂ δx₃ a δa]，a 純運動學（Q44 = Q55 = 0），不讀 c、不讀 a′、不用 Δw̄_d；
  a′-free 修正全接（白化 y₂、predict 時序、先 update 再算力、n_x 相關噪聲、MA(2)、prior P[0]、IF_white）。
- controller `motion_control_law_eq17_5state_noap.m`（`config.eq17_variant = '5state_noap'`）；推導 `derivation/0920_5state_noaprime.tex`
  （完整，§15 收官）與 `0921_5state_noap_simple.tex`（老師鏈、零新符號）；五格 battery `test_script/integration/run_5state_noap_battery.m`
  （定點 22.2/2.0/1.10 R 遠場 init、canon、meng 作弊 init；`opts.plant = 'cell'` 換細胞牆）。
- 終版：Ljung 記憶排程（命令速度×高度，clamp 0.98）、P44[0] 依 seed、√P55[0] = a_nom/40000。結論：定點散布平台 5–9%（讀數流結構，
  非迴音、非濾波器）；慢坡運動段 ±0.04、hold 0.006；1 Hz 轉折尖峰 0.09–0.15 = 記憶極限。**再往下要 a′·Δw̄_d = Layer 2**，用同一套 battery 對照。
- **一階 Q44 故事線（09-29~10-02，`gain_order 1`，memory `project_first_order_q44_story_2026-10-02`）**：Q44 = 0 的 KF = 全部 y₂ 讀數的 1/R₂ 加權平均
  （`derivation/0929_kf_two_tracks.tex` §4，c_{k,j} = P^f_aa[k]/R₂[j]）⇒ 下坡必 lag、拉長 T_d 無用（停點 e_ā −0.65）；Q44 = ε R₂、**ε = 1e-5 定案**（記憶 1/√ε 步，停點 −0.077、遠場 sd 0.077）。
  Δâ = ℓ41 e_y1 + ℓ42 e_y2 精確拆分：追蹤靠 y₁（ε 讓 P44 活著，ℓ41 大 30×），誤差集中在 y₁ 失明的轉折點／停點；4 Hz 不比 1 Hz 差、T_d 0.5 對 5 s 近壁只多 45%。
  a_m 偏高 = 控制增益錯的迴音（極點 1 − (1−λc)/s，Stein 預測對上）。新旗標 `q44_r2_frac`（ε）、`q44_mse_opt`、`q44_ratio_{std,cap,rel}`、`lf_info_beta` 等全 default off、逐位相同。
  ⚠ ε 與文件熱力項 ε[k] 同名，寫文件前要改一個。
