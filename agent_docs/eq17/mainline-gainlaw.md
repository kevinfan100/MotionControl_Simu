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

- **配方（09-04 晚起為 production 預設，`motion_control_law_formC_b.m`）**：`law_exact_step`（已知步長精確積分）＋ `ap_known_at='est'`
  ＋ `pred_mean2`（predict 二階均值：Jensen／起點差／曲率差，P 為主）＋ **`nw_mcorr`**
  （相關 process／量測雜訊 KF：控制器對同一份 y₁ 雜訊反應 ⇒ M = R₁g_n ≠ 0；predict 加輸入
  g_n(y₁ − x̂₁)、F_e(:,1) − g_n、Q − R₁g_ng_nᵀ）＋ **`pred_mean2_e4`**（讀 â 的起點差項 e₄ 行與 e_b 行）。
  四旗標 **預設 ON**；相依旗標預設跟上游（pred_mean2 ← law_exact_step，pred_mean2_e4 ← pred_mean2），
  單設 `law_exact_step=false` 即回舊 Euler 配方。旋鈕 `fe44_Aa_scale`（κ，P 傳播裡法則自我敏感度 A_a·M 的權重）
  **production 留 1**：κ = 0.5 只在 b_true 臂校準成立（見下段），常數 b 的臂會塌。驗收腳本 `test_script/integration/verify_formC_b_production_defaults.m`
  （預設 = 顯式全開逐位相同、顯式全關可跑、各臂 smoke）。09-04 晚之前的 scratch 假設 nw_mcorr／pred_mean2_e4 預設關，
  要重現須顯式設 false（當日的 runner 已改顯式）。`pred_mean2_kr1`／`_full` 是撤回的過度補償，勿開。
- **驗收**（同 10 seeds、canon deep 與 Meng ramp、hold 拉長 4 s）：hold 偏差 −0.00035 ± 0.00026／
  +0.00017 ± 0.00068；長 hold 斜率 −0.014 ± 0.041／−0.023 ± 0.075（e-6/步，修前 +0.239／+0.126）；
  散布 = √P 不變；追蹤誤差不變。收官 100-seed（<0.1%）未跑。
- **文件（09-04 統一）**：SSOT = `derivation/0903_aptrue_4state_from_true.tex`（寫法 A：斜率讀估測高度
  w̄_d − δŵ̄₃；九節含 nw_mcorr；§10 = b_true 臂；末兩頁 = b_true 三臂圖、a′_true 最終圖；獨立驗算 09-04
  全 PASS，三處記法修正已套）。`_mcorr` 版已併入並搬 `derivation/archive/`；`0902_formC_aptrue_4state.tex`
  （階梯／撤回／hold 位準模態／code 行）與 `0903_formC_aptrue_update_half.tex`（update 半步、引理、相關雜訊 KF）
  標 STATUS: HISTORY。但書不變：4-state 線性內「M → 漂移」未證，待 9-state。
- **b_true 臂（09-04，第三塊接完）**：斜率讀估測增益 ā′ = b_true(1−â)² ⇒ 起點差項應為 (∂ā′/∂â)Cov(e₄,u)，
  pred_mean2 用的是讀高度的 −ā″Cov(e₃,u)；兩者差 = (∂ā′/∂â)Cov(e₄+ā′e₃, u)，hold 內 ≈ 0（P₃₄ = −ā′P₃₃），
  近壁下坡 −3.5／−0.5 e-6/步（canon／Meng），累積 −0.0026／−0.0040。旗標 **`pred_mean2_e4`**（default off、
  需 pred_mean2；unit 16/16、關閉逐位相同）。第三臂（`run_btrue_e4.m`，同 10 seeds、hold +4 s）：配對響應
  下坡末端 +0.0010／+0.0004，運動中迴路 ~0.2–1 s 就吸掉，hold 位準、斜率（−0.013／−0.008 e-6/步）、散布皆不變。
  **b_true 臂 10 seeds 解析度不足**：hold sd 0.0047／0.0062（a′_true 的 1.8×，法則沿下坡放大增益誤差
  F_e(4,4) = 1 + (∂ā′/∂â)Δw̄），位準 SEM 0.0016／0.0022、斜率 SEM 0.27／0.24 e-6/步；base hold +0.0012／+0.0002
  與 0 相容。nw_mcorr 在此臂配對效果 −0.095／−0.113（a′_true 臂 −0.252／−0.150），base 斜率本身未解析。
  儀器：`run_btrue_nw_mcorr.m`、`probe_btrue_e4_line.m`（F_dw 近似，精確注入量用 pm2 差）、`plot_btrue_e4.m`。
  **下坡暫態（09-04 收案，`probe_btrue_descent_dip.m`，30 seeds）**：讀 â 的臂在快速接近牆時估值短暫低於真值：
  canon 1.42 s 一瞬 −0.0105 ± 0.0038（牆邊 12%，0.2 s 內回復，中位數 = 均值）；Meng 7–10 s 窗 −0.0011 ± 0.0026 不成均值、
  最低點 −0.0058 ± 0.0043、偏態 −0.74（負尾較重）。收支：predict 先掉（â 低 ⇒ 斜率陡 ⇒ F_e(4,4) > 1 正回饋）、y₂ 補回；
  a′_true@est 臂 predict 與 y₁ 精確互抵、無此事。判定為讀 â 的暫態響應，主要是散布不是位準；估測器端只能靠改軌跡（近壁放慢）壓它。
  **下坡暫態的根與修法（09-04 深夜，`fe44_Aa_scale`）**：近壁散布不是遠場誤差被放大（相關 0.09），是快速段新生：法則把位置雜訊
  映進增益（predict sd 0.025），a′_true 臂靠 ℓ₄₁ = −ā′ℓ₃₁ 每步抵消（相關 −0.99），b_true 臂在快速段 ℓ₄₁ 翻號（+0.036 對 −0.157）
  抵消失效（相關 −0.40）。翻號原因 = P₄₄ 沿下坡經 A_a·M 長 16 倍、P₄₁ 被 −F_dw·P₄₄ 主導。誠實比 κ=1 時 0.66/0.81（P 太大），
  κ=0 時 1.24–2.07（太小）⇒ κ 掃 {0,.25,.5,.75,1} 幾何平均誠實比 1.64/1.27/**0.96**/0.81/0.84 ⇒ κ* = 0.5，驗證 seeds 11:20：
  最差瞬間 sd 0.024→0.0026（canon）、0.027→0.0082（Meng），hold sd 0.010→0.0056／0.009→0.008，hold 位準 SEM 內。
  **但 κ = 0.5 不能進 production**（驗收 arm smoke 抓到，C.9 的坑）：常數 b 的臂 P₄₄ 變小 ⇒ y₂ 對模型誤差的拉回變弱 ⇒
  鎖 8/9 canon 10/10 seeds 撞地板、hold −0.0197 ± 0.0007（κ=1：+0.0006）；估 b̂ 臂 hold 沒壞（+0.0030）但暫態均值沒改善
  （−0.031 對 −0.034）、b̂ 被拉到 0.85 且 √P55 塌到 0.004。預設退回 1，旋鈕留給 b_true 臂與閉式推導。
  否證的候選：步長偏差（E[e_Δ] = 0）、e₄ 行補償誤差（+0.0008 反向）、分布偏斜（中位數 = 均值）。y₂ 權重：NIS₂ 全段 0.27–0.34 均勻
  （IF 色彩設計）、NIS₁ = 1.00，不動。推導債：自由／被綁兩部分的閉式（SSOT §12）。
  **κ 三判別（09-06）**：(1) 精確步的列 4 Jacobian 閉式（`jac_exact_step`，default off；sympy 驗算 PASS）接上後三臂兩軌跡配對差 ≤ 0.0002 ⇒ 推導正確但**無效**；
  (2) R₂÷3（K_var÷3）：快速段 √P₄₄ 不動、hold 過度自信 1.65–1.71 ⇒ 否證；(3) **散布 = prior P₄₄[0] 被法則自我敏感度放大**（Π(1−â⁺)²/(1−â)² = ā′_wall/ā′_entry）：
  seed-at-truth 臂把 P₄₄[0] 縮到起點實況（3e-4／1e-5），κ=1 最差瞬間 sd 0.0323→0.0051（Meng）、0.0264→0.0035（canon），E_l → 0，hold 誠實 1.01；
  給 Meng 一個真的 +0.0031 初始偏差：κ=1 hold +0.0007（回收），κ=0.5 hold **+0.0147 ± 0.0026**、最差瞬間 +0.048（放大 15×未修）⇒ **κ 在所有讀法下否證**，
  近壁散布 = 初始增益不確定 × 法則敏感度的誠實代價，槓桿在種子（校正鏈）不在估測器。速度否證（0.2–8.5 R/s 無效）；Meng 顯眼 = 同機制多待 13× 時間。
  儀器 `run_jac_exact_arms.m`／`run_btrue_r2_scale.m`／`run_btrue_prior_vs_offset.m`／`run_btrue_speed_swap.m`；圖 `btrue_prior_vs_offset.png`、`btrue_speed_swap.png`。
  **b_true 格收官（09-06 使用者裁示）**：估測器對初始值可修（真實 +0.0031：最差瞬間 84% 已修、hold 全修）、對初始信任度 P₄₄[0] 敏感（散布 ∝ P₄₄[0]，兩向）；
  production 不動（κ=1、四塊、`jac_exact_step` 關）。剩兩項入帳：快速段誠實比 0.66–0.77 來源未追（假說帳 O19）、hold 位準／斜率 10-seed 解析度（O2）。
  最終圖 `ladder_btest_4row_abs_p0.png`（現配方）／`ladder_btp0_4row_abs_p0.png`（prior 對回起點）。假說帳 R45–R48、C41–C42。
  **b_true 格結案（09-04）**：b 已知時讀 â 的估測器在 10 seeds 解析度內無 hold 偏差；剩解析度（散布 1.8×，κ=0.5 後 canon hold 縮 26%）
  與常數 b 對曲線 b 的配對 hold 漂移 −0.17e-6/步兩項入帳。

- **production 這一格（09-04，b 當常數 state）**：與 b_true 臂只差 b 是常數（鎖 8/9 或從 8/9 估）。
  §11 推導：常數 b 的模型誤差 f = (1−ā)²(b_true − b̂) 沿下坡經法則自身放大 F_e(4,4) 累積成
  μ = (1−ā_wall)²∫(b_true − b̂)dw̄，開環 −0.06（Meng 起點 6.67 R）／−0.09（canon 22.2 R，估值低於真值）——
  第一版只積 f 得 0.008 是錯的（驗算抓到）；定常 hold 一階不漂。**量到**（同 10 seeds、四塊全開、配對 b_true 曲線臂）：
  鎖 8/9 下坡末端 −0.0030／−0.0005、hold 起點 −0.0005／−0.0009 ⇒ y₂ 在運動中吃掉 ≥95%；估 b̂（production）與鎖 8/9
  配對差 +0.0014 ± 0.0009／−0.00002 ± 0.00013，b̂ 留在種子 ±0.01、√P55 0.039 → 0.023／0.027；production 四塊 hold 位準
  **+0.0012 ± 0.0016／+0.0004 ± 0.0023**（舊 production +0.019／+0.010）。**未解**：常數 b 對曲線 b 的配對 hold 漂移
  −0.176／−0.184 e-6/步（兩軌跡、12σ、5 s 積 −0.0014）；b′ 交叉矩否證（P 上 <0.002e-6）；b 鎖牆值的判別臂 canon 歸零、
  Meng 不變，機制開放。code：`pred_mean2_e4` 在 slot 5 自由時多加 (∂ā′/∂b̂)Cov(e_b,u)。儀器 `run_prod_ladder.m`
  （臂 lockb／lockw／prod／hist）、`plot_prod_ladder.m`、`probe_lockb_bprime_line.m`。圖 = 母本倒數第三頁。

## 未知牆面線與估 b̂ 延伸（2026-09-07）

- **三牆 = 法則家族的兩個數**：Meng Fig 11 實驗曲線數位化（`reference/eq17_analysis/data/`）擬合 1/(1−ā) = b(w̄ − w̄_s)：細胞 = 平面平移 −1 R（b 同）、球面 = b +33%。
  三牆 plant（`run_three_walls.m`）：production 在細胞崩（hold −0.08），prior 放寬到牆家族（Pf_b 0.15、Pf_w0 1 R，房規值）三牆都活；平面代價散布 3×。
  遠場 hold 沒有增益資訊，種子錯只能靠運動修 ⇒ prior 必須承認種子可能錯（假說帳 O 條）。
- **估 b̂ 延伸走完設計空間**（⚠ 09-08 撤，R55／R56：這一段的估 b̂ 臂都用了歸零點在谷底之上的種子線，「b̂ 被拉到 0.96 鎖死」「innovation 要有家」是 init 假象；弦種法下三容器全輸。保留只為讀舊圖）：b_true 乾淨結果（散布 0.005）不能直接延伸到估 b̂（同 prior 下 hold −0.011、b̂ 被拉到 0.96 鎖死）；
  b̂ 被拉的通道量到（canon y₁ 經 P₅₁、Meng y₂ 狀態路）；Q55 容器、E1 封 y₁、E2 δa 槽（五關 PASS）都能停止 b̂ 被拉，但各付散布（近壁 2×／1.5×／遠場 3–4×）；
  谷底常數 b 的均值誤差 −1～−2% 誰都拿不掉；兩錨曲線比常數差。**估測器到地板，production 是最均衡的一格**。旗標 `q55_path`、`l51_off`、`da_slot`、`jac_exact_step` 全 default off。
- 文件：`0907_estb_5state_core.tex`（估 b̂ 一階 F_e 讀本，0831 版型）；SSOT §11 加 Q55 與 E2 小節；假說帳 R45–R53、C41–C46、O19–O23。
- **牆種判別 3×3（09-08，`run_wall_mm_3x3.m`／`plot_wall_mm_3x3.m`）**：把「估 b」改成「判斷是哪種牆」。三 plant × 三 prior（各牆的 (b, w̄_s) 當種子、窄 prior、各自閉迴路）：
  對角線三牆都健康（canon hold +0.0014／+0.0021／−0.0040，Meng −0.0001／+0.0002／+0.0075）；只用 y₁ 的 innovation 似然差 ΔΛ₁ 在第一次下坡內選出正確牆（canon 過 ln 3 於 0.97–1.33 s、Meng 0.5–6.9 s），
  最弱一對 = 球面 plant 上的 plane prior；導出牆位 ŵ_wall = ŵ̄ − â/(b̂(1−â)) 對的 prior 平、錯的走。錯 prior 代價（運動段）：plane↔sphere ±0.015–0.03、細胞 0.02–0.08（最差 0.17–0.25）。
  **兩個儀器發現**：(1) controller `b_ceil` 預設 1.05 —— 球面 b 1.156 不可表示，09-07 wide 臂球面 b̂ 貼頂 1549 步（C44 數字以 R54 修正）；3×3 傳 b_ceil 1.5。
  (2) y₂ 的 per-step 高斯似然不可用（ln S₂ 主導，NIS₂ 0.3 設計值、R₂ 隨 â）；bank 要用 y₁ 或白化 y₂（O24）。共用迴路的 bank 未實作。
  常數 b 的代價量到是第一趟的（C48：回訪 ≤ 0.003）⇒ 「b 對高度」的記憶沒東西可撿，缺口在出發前的 prior。log 新增 S₁／S₂（`S1_out`／`S2_out`，logging only；plane×plane 對 09-07 逐位相同）。圖 `wall_mm_3x3_{canon,meng}.png`。
- **種子線假象（09-08 下午，R55／C49）**：seed-at-truth 只對 â[0]、b̂[0] 留 8/9，合成的直線在 canon 於 1.106 R 歸零（谷底 1.10 之上）；法則沿 21 R 把 â 搬到 floor，P₄₄[0] 小故 y₁ 經 ℓ₅₁ 把 b̂ 踢到 0.97、hold −0.0106。
  **09-02～09-07 所有 seed-at-truth 的估 b̂ 臂都帶這個 init**，其結論（b̂ 被拉、innovation 要有家、E1/E2/Q55）列 O25 重驗；餵 b_true 的臂不受影響。正確種法 = 弦：b̂[0] = (1/(1−â[0]) − 1)/(w̄₀ − w_c)（`run_three_walls` 臂 bseed/bseed0）。
  儀器規則：跑前印種子線歸零高度 w̄₀ − (1/(1−â[0]) − 1)/b̂[0]，必須低於軌跡最低點。production 自己的線（w₀ = 0、8/9）歸零 1.125，靠大 P₄₄[0] 由 y₂ 拉回，剩第一次下坡 −0.035 的坑；改錨接觸高度 canon −0.022、Meng 翻號 +0.025（O26）。
  **「起點＋牆位已知、c(h̄) 未知」這一格結案（C50）**：弦種法＋P₄₄[0] 起點實況下，常數 b̂ 對餵 b_true(w̄) 只差牆邊一瞬 ±0.017、回訪 ±0.003、散布 5–10×；法則形狀不是問題。
  圖：`seed_line_mismatch_canon.png`、`ladder_{btseed0,bseed0}_4row_abs_tw_p0.png`（平面）、`_cellb.png`（Brenner 下移 1.03 R 的細胞）；儀器 `plot_ladder_tw_abs.m`、`plot_three_walls_arms.m`、driver `opts.plant_cperp`（函數柄 plant）。
  **b(w̄) 可觀性（09-08 晚，C51／O27，`verify_obs_b_of_w.m`）**：canon 上線性 b 與 4 個高度格都結構可辨識（CRLB/prior 0.016–0.14，ā_w 不受害）；Meng 單趟只有常數 b（0.57）。「常數 b」在 canon 是回報上界（回訪 ±0.003）的決定，不是可觀性的決定；在 Meng 是可觀性的決定。
  **F_e／H／R／Q 審計（09-09，C52，`audit_fehrq_consistency.m`）**：Joseph 重建 P_upd 誤差 0、Q 由 P_pred − F P_upd F′ 精確回收（對稱、PSD、Q₅₅ = 0、增益塊 rank 1）、重放誤差 6e-14 ⇒ 接線無誤。**把同一組矩陣的 Q 設 0 重放，√P₅₅[end] 0.0087 → 0.00002（415×）**⇒ 可觀性工具的樂觀完全來自「G 不含 Q」，O10 結案；誠實的可學量 = 濾波器自己的 √P₅₅ 0.039 → 0.0087（canon）／0.0149（Meng），σ 只縮 4.5×／2.6×。b 的資訊幾乎全在 y₁。
  **b 按高度分格（09-09，R59，`b_bins_on` default off，驗收 ALL PASS）**：swap 近似（換格存回／載入、交叉協方差歸零）四判準全敗——各格 b̂ 仍停在弦 ±0.01、canon 末段位準 +0.0115、√P₅₅ 不縮；因為每格只拿 1/4 資料且 16 次換格把 y₁→b 的協方差路徑丟光。**C51 可觀性 PASS 的是完整增廣（4 state 同時在場、保留 P），未實作（O29）**。圖 `b_bins_plane.png`。
  **directional forgetting 落地並否證（09-09，R58，`p55_floor_on` default off，驗收 ALL PASS）**：P₅₅ 底線 = prior 0.039²，Meng 不 windup 但 ramp 上 b̂ 只到 0.98／0.97（真 1.16）、canon 振盪 σ 2.3×、hold 變差 ⇒ 遺忘家族全關；b 每趟近壁只換得到 0.05–0.2，是資訊率不是容器。剩 CUSUM 換牆偵測 + per-wall map。圖 `pf_ramp.png`、`pf_plane.png`。
  **遺忘因子重驗（09-09，R57）＋文獻（O28，`reference/eq17_analysis/literature_varying_parameter_2026-09-09.md`）**：固定／驚訝觸發的 slot-5 遺忘在 ramp plant（b 沿高度 1.16→0.87）不買追蹤、Meng windup；文獻無人線上估牆參數，同構問題 = 路面摩擦（Gustafsson 1997：Q=0 + CUSUM + 觸發重置），該做的是 directional forgetting（只在有資訊的步放 P₅₅）→ CUSUM 換牆偵測 + per-wall map。
  **b 的可觀性讀本（09-09，C52，`0907_estb_5state_core.tex` §5–§7 ＋ 末頁圖）**：量測 H（含 H₂₄、H₂₅）、Q/R、可觀性三塊補進 5-state 核心；判準寫成純奇異值——白化 O_w = R_w^(−1/2)O、O_w v_i = σ_i u_i、沿 v_i 的誤差標準差 = 1/σ_i、v₅ → e_b 時 1/σ₅ 就是 b 的誤差棒、1/σ₅ ≶ √P₅₅[0]。canon 下坡 σ = [13526, 948, 680, 680, 241]（σ₃ = σ₄ = 1/(σ_n/R) 恆定，y₁ 直接量 δw̄₁、δw̄₂），1/σ₅ = 0.0042 對 prior 0.0389；hold σ₅ = 14.1 ⇒ 0.071 輸 1.8×。b 是五個狀態裡唯一「誤差棒 ÷ 自身 prior」會超過 1 的（其他四個全程 0.09–0.45）；資料贏的視窗 canon 7/15、Meng 2/39，對應 b̂ 實走 0.008／0.0013。σ_min/σ_max 只能抓別名不能當判準（贏輸區間重疊）。5-state 對 production 的差 = nw_mcorr 欄（LOO：下坡 0.009 → 0.116，13×；MA(2) 兩槽只差 10%）。儀器 `test_script/integration/plot_obs_b_5state_two_traj.m`（產文件末頁圖）、scratch `obs_walk_5state.m`／`plot_obs_b_walk_one_window.m`／`plot_obs_b_signature_walk.m`／`plot_obs_sigma_two_traj.m`。
  **O25 重驗（09-08 晚，R56）**：Q55／E1／E2 換弦種法後全部比純估測器差（canon 最差 −0.017 → −0.025／−0.025／−0.040，σ 2.8×／1.8×／9×）⇒ 三旗標退場，「innovation 要有家」撤。driver 每 run 印 `SEED LINE:` 檢查；規則 stacked-fix-audit B.8。

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
