# formC_b 疊加態審計 ledger（簽名庫＋定罪表）

> 對象：`model/controller/motion_control_law_formC_b.m` @ `10e51db`（2026-08-24 18:29 +0800，
> reflog `logs/HEAD` 末筆：「feat(eq17): a wall that is not a constant, and R2's delay term finally has a knob」）
> 驅動：`test_script/integration/run_formC_b.m`；場景 SSOT：`model/config/canonical_scenario.m`。
> 建檔 2026-08-25（Mac；內容以 test/motion-test worktree @10e51db 為準）。每個數字附出處；查不到的標 [未核]。
> 08-25 增補：Stage A（bmid vs bmid＋ap_known）結果 L22–L24、§4 1b 求積棘輪、§5 vii predict replay 登記、§6 #11。
> 08-25 二次增補：replay 閉合／Route A／配對指紋 L25–L28、liveness 普查 §1.10（含 `da_known` 死臂）、a_nom 檔案值、閘普查、§6 #12–13。
> 08-25 三次增補：replay 終報 L29–L32——號差 CLOSED（端點配對，driver `:1113`）、§4 1c 求積後殘差 OPEN、1d driver 衛生項、§5 viii Stage B 登記 stub、§6 #11/#12 結案。
> 08-25 四次增補：Layer-2 LOO 電池 L33–L38（ma2 重現保留／echo pending／fe4 代理過期但承重／二階非可加／x/y 釘頂）、§3 三列更新、§4 #16–17、§5 i–v 狀態。
> 08-30 增補：正規化邊界審計 L42–L45（V1 換單位制／V2 κ_T 對 plant／V3 ε_w 對容器／V4 ξ 識別式，全 PASS）；L18′「Q33 對物理式」由 L44 結案。同日 tex 更新：`formC_state_b.tex` 補 MA(2) 州／rank-2 Q／S8 在 ā̂／notation 行（a_o vs a_nom、f_R）；規則 `.claude/rules/normalization-boundary.md`。
> 08-25 五次增補：echo descent/osc 窗重判 L39–L40 → echo **重現保留**；首輪 LOO 收官（ma2 ✓、echo ✓、fe4 代理過期但承重待直接簽名、echo×ma2 非可加）。
> 08-25 六次增補：形狀判準循環 C-1（L41–L41‴、§4 #18、§6 #14–15）——production Pf_b_std 窄 2.9×，A-1 重跑立案。
> 08-30 增補：Stage A 端點修正後重跑 L42——偏差存活 +21.65 vs +21.05，斜率家族對均值排除、只管散布；§4 1c 改寫（+9 求積 ＋ +12 非斜率 OPEN）、
> 1d 已修、#2 降級、#19 LOO 基底過期（1a70599）；§6 #16。
> 對應 memory：`project_formC_b_layered_audit_2026-08-25.md`。
> 所有 `controller:NNN`／`driver:NNN` 行號皆對 motion-test 那份 @10e51db。

---

## §0 用途與規則

本檔是 formC_b 疊加態審計的兩份底冊：**Layer 1 簽名庫**（§2）收錄與旗標無關的恆等式量測——
Var 識別式、R22 三層、Kalman 更新恆等式、誤差帳——這些是「不管哪個旗標拔掉都必須成立」的尺，
拔旗標後先拿它們驗儀器再談物理；**Layer 2 定罪表**（§3）逐一列出現行 production 配方裡的每個成員、
它當初被定罪的基底（law／帶／λc）、定罪數字，以及**是否已在 formC deep 終基底重驗過自己的簽名**。
§4 是兩個 session（追 â 高估的 R22 線、追 b 估不動的 b 線）合併後的未解清單；§5 是後續每個
判別 run 的預登記模板，並先替六個已規劃的 run 填好事實欄。

規則來源：`.claude/rules/stacked-fix-audit.md`（Meng ch4 戰役定稿，A1–D12 十二條）與
`.claude/rules/derivation-workflow.md`（規則 6「因果宣稱必須有對手」、規則 7「儀器先驗證」、
規則 9「判準用 seed 無關的簽名」）。方法先例：`reference/eq17_analysis/meng_ch4_spec_ledger.md`
§35–46（fdet 翻案、LOO 三件保住、二階 NOWHSC、關帳審計全表）。核心紀律三句：
(1) 每件修復的判準是**它自己當初的定罪數字**，不是總分；(2) **任何成員變動後全鏈 LOO 重跑**；
(3) 標 [假說] 的東西不得進結論鏈。
⚠ `stacked-fix-audit.md` **只存在於 test/meng-ch4 worktree**（`ls .claude/rules/` 於
motion-test 只有 canonical-trajectory／derivation-workflow／dual-machine-workflow／figure-style／
matlab-conventions／observability-workflow 六檔）；本檔引用的是 meng-ch4 那份，
merge 前 motion-test 分支上的讀者看不到它。

---

## §1 Production 快照 @10e51db

### 1.1 狀態向量（真實 slot map；檔頭 1–13 行已過期，見 1.7）

```
x = [dw1 dw2 dw3 | ā | b | (inert) (inert) | m1 m2]      n_state = 9（ma2_aug=true；=7 若關）
      1   2   3    4   5     6       7       8  9
```
- slot 4 = ā（正規化增益，(0,1)）；**slot 5 = b**（斜率倍率，ā′ = b(1−ā)²；controller:214 註解、
  :1006–1010 讀值 `b_hat_i = clamp(x_curr(5), b_floor, b_ceil)`、:1057 `local_gain_law_formC(a_bar_sl, enable_wall, b_hat_i)`）
- slot 6/7：永久鎖定、P0=0、Jacobian=0、K=0（:598–607 `lock_mask_g = [lock_b; true; true]`）；
  slot 7 於 init 被寫成 w0_law 當「inert reporter」（:792）
- slot 8/9：MA(2) 記憶州 m₁=w_T[k−1]、m₂=w_T[k−2]（:1101–1115 rank-2 Q、:1191–1208 predict）
- F_e 第 5 列純恆等 `[0 0 0 0 1]`（:1500–1529 `local_build_F_e_formC`；memory 08-24 b 線量到確認）

### 1.2 場景（deep SSOT，`canonical_scenario.m`，2026-08-19 `16631ef` 起預設）

| 量 | 值 | 出處 |
|---|---|---|
| 形狀 | hold 0.5 s @ h=50 μm（w̄ 22.2）→ descend 1.0 s → 1 Hz × 2 cycles，半振幅 2.5 μm → hold 至 T=4.8 s | `canonical_scenario.m:55–63` |
| 谷底 | w̄ = 1.10（h_bottom = 1.10·R；R = 2.25 μm）；帶 [1.10, 3.32]，拖曳 11.45× | `:12–13, :67` |
| h_min | 1.10·R（prior 包絡下限 = two-sphere 級數有效下限） | `:63`；driver `H_BAR_MIN_PRIOR=1.1`（`run_formC_b.m:161`） |
| h_bar_safe | **1.00**（deep；shallow 為 1.5 house value） | `:68, :72` |
| λc / a_pd / a_cov | 0.7 / 0.05 / 0.05（driver `A_COV_BASE=0.05`×`a_cov_scale`） | `:74–76`；driver `:170` |
| meas_noise_std | [0.00062; 0.00057; 0.00331] μm（見 1.9） | `:78` |
| a_nom | **0.01470588 μm/pN**（= Ts/γ_N，driver `:980` `a_nom_drv`；檔案值）。⚠ 手打 0.014696 差 1.00067×——Stage A W2 的「1.00067」就是這個 typo，不是接線誤差 | driver `:980`；`stageA_bmid_*.mat` 的 `a_nom` 欄 |
| 種子 | house 6-seed [7 11 23 42 101 777]；100-seed 研究用 1:100、400-seed 用 1:400 | driver `:160`；scratch `run_formC_b_true_a_arm.m:13` |
| 指標窗 | descent [t_hold, t_hold+1.0]；osc [+0.1 s settle, t3]；hold [t3+0.3 s, 4.8]；scratch 工具另用 hold(far) [0.05,0.45]／descend [0.5,1.5]／oscillate [1.75,3.5]／hold(trough) [3.75,4.8] | driver `:173–174, :531–539`；scratch `check_y1_leg_decompose.m:13` |
| 真值 log 夾制 | `h_bar_floor_drv = 1.001`（08-19 修，原借用 1.10 使尺凍住讀高 16%） | driver `:1013` |
| shallow 對照 | 谷底 w̄ 2.00、h_bar_safe 1.5；**跨帶不得比百分比**（谷底 ā 0.47 vs 0.087，分母差 5.4×） | `canonical_scenario.m:30–36` |

### 1.3 旗標總表（production 預設 = controller `get_field_default` 值；driver 覆寫另註）

| 旗標 | production 預設 | controller 行 | driver 覆寫／備註 |
|---|---|---|---|
| `y2_whiten` | true | :361；用於 :922–929（y2 = ā_wm − (1−a_cov)ā_wm[k−1]，H2_scale=a_cov） | — |
| `fe_row4_full` | true | :362；用於 :1160–1165（M_row4 = Δw̄_d[k−1] + (1−λc)·dw₃̂） | — |
| `obs_dump` | false | :368 | 只在 observability 工具開 |
| `use_fdet` | true | :371；用於 :1162–1167（F_dw ← 去噪鏡像 F_dw_det；false = 實現力 F_dw_raw） | scratch `run_fdet_off_arm_z.m` 用 `ctrl_const_override.use_fdet=false` |
| `y2_off` | false | :372；用於 :1243（y2 更新整段跳過） | driver `opts.y2_on=false` → `ov.y2_off=true`（`:317–318`，**formC_b 已接線**；formC_dist 08-12 曾未接） |
| `q33_dc_match` | false | :378–383；與 ma2_aug 互斥（:424–429 error） | 診斷臂 (b) |
| `y2_echo_corr` | true | :393；S_T/S_n **init 一次**由 6-state Lyapunov 在 g=1±1e-4 有限差分算出（:395–419）；每步 S=(S_T·ā+S_n·ξ̄)/(ā+ξ̄)，H2 整列 ×(1−S)（:1259–1273）、y2_pred 的斜率回退項 ×(1−S)（:1280–1281） | assert d=2（:396） |
| `ma2_aug` | true | :423；n_state 9（:431–435）；Q rank-2（:1101–1115）；5×5 DARE P0（:720–738） | — |
| `y1_gain_off` | false | :436；用於 :1221（K1(4:7)=0） | 消融用 |
| `t2_pure_prop` | false | :443；用於 :1176（F_dw=0、Q=0） | T2 hook |
| `ap_src` | 'post' | :473；分支 :1020–1049（post/pred/ewma/cmd/act） | driver `opts.ap_src`（`:130`） |
| `law_b_formC` | 1 | :474 **讀入後全檔無第二次使用**（`grep -c` = 2 = persistent 宣告 + 賦值）→ 死旗標，見 1.6 | driver `ov.law_b_formC = opts.law_b`（`:282`）**對濾波器無作用** |
| `ap_ewma_a` | 0.05 | :475 | — |
| `lambda_f` | 1（off） | :479；用於 :1314–1317（整個 P /λ_f） | 08-12/13 否證 12/12（:454–460 註解；commit `dba3cb4` 2026-08-13） |
| `lambda_f_b` | 1（off） | :505；用於 :1319–1333（slot-5 congruence） | 診斷 |
| `lambda_f_b_alpha` / `_floor` | 0 / 0.99 | :550 / :554 | 自適應版，08-20 結構性退化（memory slope_source） |
| `Q_theta_floor` | 0 | :564；用於 :1130–1134 | — |
| `par_law` | false（controller）／**true**（driver `opts.par_law`, `:151`） | :574–589 需 `w0_par`、`Pf_a_floor_par` | driver `local_parallel_law_package_dist` 供給（`:325–329`） |
| `lock_b` | false | :598（變數名仍叫 `lock_da`） | driver arm 表：b1/bmid → true；best/b98/bfree1 → false（`:291–309`） |
| `b_init` | **9/8**（controller） | :649 | driver 一律覆寫：b1→1、bmid/best→b_mid（=8/9 自 08-18）、b98→8/9、bfree1→1 |
| `Pf_b_std` | 0（controller） | :650 | driver `(~lock_b)·b_half`，b_half = sup\|b_true−8/9\| 於包絡（`:310`, `:578–598`）；**真值推導 ⇒ 形狀判準循環、比漸近寬度窄 2.9×**（L41、§4 #18） |
| `Pf_w0_std` | 0.111 | :669 | driver `opts.Pf_w0_std=0.111`（`:150`） |
| `Pf_a_floor` | 0.0066（controller） | :666–670 | driver：`floor_from_envelope=false`（`:145`）→ **seed-local** `local_seed_floor(env_hi−1, W0_PLANE, b_init)`（`:312–317`）；envelope sup 兩帶皆 0.03058（memory canonical_deep）；seed-local 數值 [未核] |
| `Pf_da_std` | 承 dist 寫法，**在 b 寫法未使用**（變數名 `Pf_da_std_unused`） | :667 | driver 仍印 `da PRIOR`（`:359`）— 顯示層殘留 |
| `r22_delay_scale` | 1.0 | :342–346；用於 `compute_R2_formB` 第 14 引數（:1141–1144, :1553） | 08-24 第一版 `nargin<15` 死旋鈕，配對差精確 0 抓到；已修 |
| `amlpf_var_factor` | 1（由 `build_eq17_6state_constants` 給） | :355；乘進 R2_int（:1552） | 見 1.8 |
| `t_warmup_kf` / `h_bar_safe` | 0 / cfg.h_bar_safe（deep 1.0） | :351–352；G1/G3 於 :1147–1149 | driver `:951–955` |
| `ws0_perp` | 1（w0_nominal=0，平面） | :641 | driver `ov.ws0_perp=1`（`:279`） |
| `plant_law_b` | NaN（plant 端，非估測器） | — | driver `:143, :376–389, :1299 local_plant_cperp_law`；純量或 `[b_wall,b_far,w_c,Δ]` logistic 曲線（10e51db） |

### 1.4 Clamps（數值護欄）

| 護欄 | 值 | 行 | 備註 |
|---|---|---|---|
| `a_bar_floor` / `a_bar_ceil` | 0.05 / 1−1e−4 | :610 / :618；施加於 :1005（法則輸入）、:979（a_ctrl 下限）、:1302（後驗） | 註解：從不 bind（forward-Euler 越界需 dw≈2/步 vs 0.0043） |
| `b_floor` / `b_ceil` | **0.60 / 1.05** | :651 / :652；施加於 :1010（法則輸入）、:1312（後驗，僅 slot 5 free 時） | 08-17 `2d1b9d1` 從 [0.5, 2.0] 收窄（memory state_b_multiplicative）；⚠ **controller 自身 b_init 預設 9/8 = 1.125 > b_ceil 1.05**（直接呼叫 controller 會釘在上界；driver 覆寫 8/9 所以 production 安全） |
| `ws_margin` / `gap_floor` | 1e−3 | :623–624；用於 seed level（:1494）與 cmd/act 分支（:1046–1047） | — |
| y2 閘 G1/G2/G3 | t<t_warmup_kf（=0 永不）；(σ̂²−C_n σ²_n)≤0；h_bar<h_bar_safe | :1147–1149 | deep 400 seeds：G2 全程 0.013%、末端 hold 0%、G3 0%（memory var_identity §四個解釋）；08-25 普查：G2/G3 命中比例 **0.000**（stageA 兩臂 100 seeds＋stack_deep400） |

### 1.5 P0 與種子（:754–792）

- ā 種子 = 1 − 1/[b(w̄₀−w0)]（`local_seed_level_formC`, :1475–1497），w0 = 0 平面；**種子與斜率同一個 b**（08-13 缺陷修復）
- P44[0] = (dA/dw0)²·Pf_w0_std² + free·(dA/db)²·Pf_b_std² + Pf_a_floor²，dA/db = +(1−ā_seed)/b（:765–768）
- P45[0] = free·dA/db·Pf_b_std²（非零；對照 dist 寫法結構性為 0）；P55[0] = free·Pf_b_std²
- 位置/記憶塊：5×5 DARE 於 [dw1 dw2 dw3 m1 m2] → slots [1 2 3 8 9]（:720–738）
- chol PD 檢查於 free block（:778–788）

### 1.6 死旗標 `law_b_formC`

`controller:474` 讀入 persistent，全檔再無使用（`grep -c law_b_formC` = 2）。估測器的 b 一律來自 slot 5。
driver `opts.law_b`（`:136`）經 `ov.law_b_formC`（`:282`）傳入**無效**；它只在 `ap_law_bias` 診斷臂
（driver `:1126` `ap_known_k = [0;0;(1−ab_off)²/law_b]`）還活著。與 08-24 `nargin<15` 同類：新增旗標沒接到消費端。
（來源：memory nearwall_pole_feedback「聯合審查」段；本檔 grep 於 10e51db 重驗 = 2）

**同類新陷阱 `opts.da_known`（driver `:128`）**：driver 每步照算 `da_known_k`（`:1100–1111`）並以第 7 引數傳入，
但 controller 簽名第 7 引數是 `~`（:14）且 `has_da_known = false` 寫死（:254）⇒ **在 formC_b 上靜默 no-op**。
liveness 普查：`da_known=true` 對 baseline 逐位零差（預期 LIVE、實測 DEAD）。它是 dist 寫法的 known-disturbance 臂，
fork 時 driver 端留下、controller 端拔掉。

### 1.10 旗標 liveness 普查（2026-08-25，seed 7，arm best，deep；`test_script/scratch/l0_flag_liveness.m`）

63 臂，判準 = max|Δā_hat|（全軸全程）或 max|Δb̂| > 0 即 LIVE；負控制（空 override）逐位零差 PASS。
**DEAD 9 項**：`law_b_formC`、`Pf_da_std`、`ws_margin`（deep 上不 bind，0.5 仍零差）、`obs_dump`（設計如此）、
`ap_ewma_a` 單獨（只在 ap_src=ewma 下消費）、`lambda_f_b_floor` 單獨（只在 alpha>0 下消費）、driver `da_known=true`（上段）、
driver `law_b` 單獨、driver `Pf_da_std`。**其餘 52 項 LIVE**（含 `r22_delay_scale=1e6` 正控制、四個 clamp 於強迫 bind 值、
`y2_echo_corr=false`、`ma2_aug=false`、`fe_row4_full=false`、`use_fdet=false`、`y2_off`、`h_bar_safe` 經 cc 與 cfg 兩路同值）。
輸出 `/Users/kevin/.claude/jobs/8581427c/tmp/l0b/flag_liveness.txt`（job tmp，不持久；表格已抄要點於此）。
⚠ `b_ceil=0.9` 標 LIVE 但只是「b̂ 曾越過 0.90」的下界證據；`lambda_f_b_floor|alpha` 的 LIVE 同樣是條件式。

### 1.7 過期檔頭（controller :1–13 與 :14–237 docstring）

- :8 「Slot map: 5 = da (ADDITIVE ...)」— 是 fork 來源 `formC_dist` 的；真實 slot 5 = b（:214、:1006）
- :14–237 整段 docstring 沿用 dist 寫法（`MOTION_CONTROL_LAW_FORMC_DIST`、`formC_state_dist.tex`、
  「da[k+1]=da[k]」、「F_e(4,5)=1 constant」、「H(2,5)=−d_delay」）— **全部不是本檔現況**：本檔
  F_e(4,5) = dap_db·M ∝ 位移（:1090, :1525）、H(2,5) = −Grad·dap_db（:1268–1273）
- :214–218 寫 b_floor/b_ceil 預設 (0.5, 2.0) 且「b_true ∈ [1.1258, 1.1534]」— 這是**舊 /b 約定**；
  現行 ×b 約定 b_true ∈ [0.867, 0.928]（memory slope_source）／驅動註解 [0.866978, 0.888225]（driver `:222–225`，
  兩者範圍不一致，見 §6 矛盾 #7），code 護欄 0.60/1.05
- :1453–1463 `local_gain_law_formC` 註解「law_b = 9/8 is the asymptotic anchor」— 錨已改 8/9（08-18 `843474b`）
- driver `:127` 預設 arm = 'best'，但 docstring `:15–17, :62` 仍寫 'dist'/'base'（arm 名早已換成 b1/bmid/best/b98/bfree1）
- `empty_diag_formB` :1651 註解「slot 5 = da, seed 0」

### 1.8 `use_am_lpf` 潛在陷阱

`build_eq17_6state_constants.m:89` 讀 `opts.use_am_lpf`（預設 false）；`:131–137` 若 true 則
`amlpf_var_factor = K_var2·IF2`（≈0.089），controller `:1552` 乘進 R2_int。formC **沒有 a_m LPF**
（無 a_m_det 通道），打開後 R2 縮 11× 而讀數完全沒被濾 → 跑得完、靜默錯。目前 driver 不設此欄（`:943–957`），
所以 production 安全，但沒有 error 攔截。（memory am_r22_whitened §兩個儀器發現）

### 1.9 y 軸 meas_noise 10× 備註

`user_config.m:43–63`：三軸 [0.62; 0.57; 3.31] nm 於 2025-12-12 `f5334d7` 進 repo，**無 datasheet／量測／引用**；
y 原為 0.057 nm（比 x 低 10×），Phase 6 判「切向軸物理上不合理」，`69ebc9f`（2026-06-08）在單一檔案改成 0.57 nm，
repo 因此分裂為兩族兩個月；2026-08-12 統一為 0.57 nm。**OPEN：三軸都沒有硬體來源**，是自洽 scenario 不是 spec。
`mainline-gainlaw.md` 的「y 軸 0.00057 vs 規格 5.7e-5 差 10×」指的是這段歷史，方向與 user_config 註解一致
（現行值是被**調高**的那個）。

---

## §2 Layer 1 簽名庫（恆等式，與疊加態無關）

所有量都是「模型下必須成立」的尺，不是旗標的功效。**帶別與臂別逐列標明，不可跨列合併。**
deep = 谷底 w̄ 1.10；shallow = 2.00。「部署臂」= arm best／ap_src post／a_ctrl = â；「true-a 臂」= `a_ctrl_override='true'`。

| # | 量 | 數字 | 臂 | seeds | 帶 | 日期 | 腳本 | 資料檔 |
|---|---|---|---|---|---|---|---|---|
| L1 | Var(δw̄_r) 識別式 實測/公式，分段 z | hold-start 0.995／descend 1.002／osc 1.006／**hold-end 1.052** | 部署 | 400 (1:400) | deep | 08-20 | `check_formC_var_identity.m` | `test_results/formC_cdpmr_var_check/raw_seeds.mat` |
| L1′ | Var(δx) 識別式（C_δx=3.9608）分段 z | 0.996／0.996／1.008／**1.108** | 部署 | 400 | deep | 08-20 | 同上 | 同上 |
| L1″ | 同上逐 bin 隱含常數 | a/a_o 0.949：C_dpmr 3.1694±0.0168、C_δx 3.9602±0.0223；0.083：3.5620／4.6788；a/a_o>0.4 平均 +0.31%／+0.18% | 部署 | 400 | deep | 08-20 | 同上；誤差棒 = delete-one-seed jackknife（`check_var_bin_errorbar_arbiter.m`） | 同上 |
| L2 | Var(δw̄_r) 識別式 末端 hold，true-a 臂 | z 1.060→**1.006**；x 1.064→1.004；y 1.068→1.009（分段 A 臂：0.993/1.004/1.006/1.010） | true-a vs 部署（配對） | 100 (1:100) | deep | 08-21 | `run_formC_b_true_a_arm.m`＋`judge_true_a_arm.m` | `test_results/am_r22_deep/truea_100.mat` |
| L3 | Var(ā_wm)／[K_var·IF_eff·(ā+ξ̄)²]（R22 第一層） | descent 1.009／osc 1.002／hold 0.982（agg） | 部署 | 200 | **shallow** | 08-19 | `verify_formC_am_r22.m`（`add0904`） | `test_results/am_r22/stack_200.mat` |
| L3′ | 同上末端 hold，true-a 臂 | z 1.297→**1.016**；x 1.287→1.000；y 1.354→1.054 | true-a vs 部署 | 100 | deep | 08-21 | `judge_true_a_arm.m` | `truea_100.mat`＋`raw_seeds.mat` |
| L4 | Var(y₂)／[2a_cov²(ā+ξ̄)²]（每步式，不含 IF） | 0.994／0.997／0.987（agg） | 部署 | 200 | shallow | 08-19 | `verify_formC_am_r22.m` | `stack_200.mat` |
| L4′ | 同上末端 hold，true-a 臂 | z 1.119→**1.010**；x 1.127→0.998；y 1.168→1.037 | true-a vs 部署 | 100 | deep | 08-21 | `judge_true_a_arm.m` | 同 L3′ |
| L4″ | 同上，a_true 上分兩半驗（每步式三軸貼合；IF_eff 形狀對） | 每步式三軸 log-log 與絕對值疊圖皆過；IF_eff 量測/模型 **1.10–1.16**（三軸三視窗） | true-a | 100 | deep | 08-24 | `plot_r22_atrue_validation.m`、`plot_r22_z_reference_style.m` | `fig11_r22_atrue_validation.png`、`fig12_r22_z_styled.png` |
| L5 | R2_used／Var(y₂) = IF_eff（R 蓄意膨脹＝有效樣本懲罰） | 3.366／3.340／3.370（= IF_eff 3.29–3.44） | 部署 | 200 | shallow | 08-19 | `verify_formC_am_r22.m` | `stack_200.mat` |
| L6 | acf 總帳 1+2Σρ(y₂) vs IF_eff 使用值 | **3.581 vs 3.294**（IF 偏小 8.0%，遠場） | 部署 | 200 | shallow | 08-19 | 同上 | 同上 |
| L6′ | 同上 deep：NIS E[innov²/S₂] vs 1+2Σρ | NIS 0.2909（看似 R₂ 大 3.44×）；1+2Σρ = **3.687**（ρ 0.649/0.362/0.140）⇒ 相符，總帳誠實 | 部署 | 400 | deep | 08-20 | `check_if_innovation_route.m`、`check_r22_operational.m` | `am_r22_deep/stack_deep400.mat`＋`raw_seeds.mat` |
| L6″ | IF 四量正名（IF_var／IF_smpl／IF_inno／實際施加） | 遠場低充氣 8% 無抵消；谷底看似完美因 R₂ 建在 â 高 +19% 的點上 | 部署 | 400 | deep | 08-20 | `check_if_innovation_route.m` | 同上 |
| L7 | ρ_y2(τ) = ρ_dwr(τ)²（Isserlis） | 400 seeds 驗到 0.1–1%；來源為 1−a_pd 與 λc 極點，非 a_cov、非 d=2 | 部署 | 400 | deep | 08-20 | `battery_I_identity.m` I4 | `raw_seeds.mat` |
| L8 | y₁ 誠實比 Var(innov₁)/S₁（S₁ 由 K1(4)=P41/S1 反推）＋白噪 | 0.88–0.99 四視窗；innov₁ 自相關全在 ±0.06；偏差極小（osc t=−5.97 但只 −0.117 nm） | 部署 | 100 | deep | 08-24 | `check_y1_channel_honesty.m` | `am_r22_deep/baseline_budget_100.mat` |
| L9 | Kalman 更新恆等式閉合 Δâ = predict + K1(4)innov₁ + K2(4)innov₂ | 閉合 **1.7e−18**（08-21 記）／「1e−17」（08-24 記）— 同一檢查兩次記錄 | 部署 | 100 | deep | 08-21/24 | `plot_formC_kf_gains.m`（08-21）；08-24 誤差帳腳本 [未核，memory 未指名] | `raw_seeds_budget.mat`（08-21）／`baseline_budget_100.mat`（08-24） |
| L10 | 誤差帳（絕對 ā 單位，整段行程，z） | 真值 −0.86232；法則 **−0.91273**；y₁ **+0.07786**；y₂ **−0.01152**；淨 **+0.01593** = 行程 1.8% = 谷底 +18.3% | 部署 | 100 | deep | 08-21 | 同 L9 | `raw_seeds_budget.mat` |
| L10′ | 分段 t 值（法則／y₁／y₂） | 遠場 hold −0.3/+0.5/−0.1；降落 **−13.9/+11.9/+3.8**；振盪 −1.6/+5.2/−3.4；末端 hold −1.5/+2.4/**−17.1** | 部署 | 100 | deep | 08-21 | 同上 | 同上 |
| L10″ | 法則斜率直接量 a_prime_out／a_prime_true_out | 降落 1.044、振盪 0.982、谷底 0.930（換號） | 部署 | 100 | deep | 08-21 | 同上 | 同上 |
| L11 | y₁ leg 分解 E[K1]E[i1] + Cov(K1,i1)，分窗 | descend +0.01372 = +0.00191 + **+0.01181（Cov 86.0%）**；osc +0.04012 = +0.03152 + +0.00860（21.4%）；hold(trough) +0.01517 = +0.01507 + +0.00010（0.6%，E[K1] = −0.496 持續帶號） | 部署 | 100 | deep | 08-24 | `check_y1_leg_decompose.m` | `baseline_budget_100.mat` |
| L12 | F_e(4,3) = (1−λc)·a′(â)（ap_src=post ⇒ J_d3=0，無自由參數，恆非負）分窗均值 | hold(far) 0.00068／descend 0.02834／oscillate 0.08038／**hold(trough) 0.21568 = 317×** | 部署 | 100 | deep | 08-24 | `check_Fe43_growth.m` | 同上 |
| L13 | 讀數第二路徑重建（LP→HP→EWMA→仿射反解 vs log） | (A) my dw_r vs dx_r_out、(B) my ā_wm vs a_xm_out，含 shift −1/0/+1 off-by-one 探針；結果 [未核，memory 只記「儀器驗證」通過，未存數字] | 部署 | 20 (of 400) | deep | 08-20 | `check_readout_reconstruction.m` | `raw_seeds.mat` |
| L14 | E[a_m]/a_true − 1 分段（讀數高報） | 初始 hold **−0.53%**／末端 hold **+6.09%**（對方 +5.98%）；逐 bin 由內而外 +6.9/+7.2/+7.2/+7.8/+10.6/+3.6/+1.7/…/−0.4%，a/a_o ~0.6 歸零 | 部署 | 400 | deep | 08-20 | `battery_I_identity.m` I3、`0525cc3` first-moment | `raw_seeds.mat` |
| L14′ | 同上 true-a 臂 末端 hold | z +7.21→**+0.81%**；x +6.44→+0.36%；y +6.97→+0.94% | true-a vs 部署 | 100 | deep | 08-21 | `judge_true_a_arm.m` | `truea_100.mat` |
| L14″ | 殘差 acf τ=1..4（模型 0.825/0.716） | x 0.838→0.823；z 0.739→0.718 | true-a vs 部署 | 100 | deep | 08-21 | 同上 | 同上 |
| L15 | fdet 消融配對（use_fdet false vs true） | descend Cov 佔比 86.0% → **93.9%**；末端 hold 偏差 +0.01589 → +0.01827；配對差 **+0.00243，t=+4.43**；偏差比例 20.9% vs 18.2%（fdet 貢獻 ≈13%） | 部署 | 100 配對 (1:100) | deep | 08-24 | `run_fdet_off_arm_z.m`、`judge_fdet_ablation.m`、預登記 `fdet_ablation_prediction.txt`（H_A 命中） | `fdet_off_budget_100.mat`＋`baseline_budget_100.mat` |
| L16a | K 通道不平衡 K_a_y2/K_a_y1（中位數） | **0.0087**（y₁ 力道 115×） | arm best | [未核 seeds] | **10 s Meng 單調降 h̄ 6.67→1.111，t_hold=0（非 canonical）** | 08-24 | b 線腳本 [未核名稱] | [未核] |
| L16b | 同現象另一量法：\|K₁\|/\|K₂\| | **43×** | 部署 | 100 | deep | 08-21 | `plot_formC_kf_gains.m` | `raw_seeds_budget.mat` |
| L16c | 同現象誤差帳份額：y₂ 佔 â 更新量 | ~1%（predict 96%、y₁ 大部分剩下） | 部署 | 100 | deep | 08-24 | L9 腳本 | `baseline_budget_100.mat` |
| L16d | K₁(4) 與 \|P(4,1)\| hold vs 運動中位數 | K₁(4) 1.93e−3 → 3.12e−1（**162×**）；\|P41\| 1.00e−7 → 1.57e−5（156×）；K₁(4)≈0.31 vs K₂(4)≈7e−4（~442×） | **formC_dist** dist 臂 | 8 | **shallow** | 08-12 | `run_formC_dist` 系列 | [未核] |
| L17 | b 搭便車簽名（hold 期 ℒ₅₂/ℒ₄₂ 比值） | hold −0.7413±0.0285（sd/mean 3.85%）vs 振盪 2.98±1.45；hold 期 max\|Δw̄_d\| = 0.000e+00 | 部署 | 100 | deep | 08-21 | `plot_formC_kf_gains.m`（ℒ₃₂/ℒ₅₁/ℒ₅₂ 新增 log） | `raw_seeds_gains_b.mat`；圖 `var_kf_b_ride_along.png` |
| L18 | code-vs-tex 逐項（僅用 obs_dump） | F(4,3) 2.5e−16、F(4,5) 機器精度（2.6e−17）、H(2,5) 5.4e−16 號正確、Q34 9.4e−14、Q44 3.3e−10、Q55=Q45=0、P45[0] 逐位；H row2 尺度 0.03428/0.05 ⇒ S=0.3144 | arm best | seed 7 | canonical（08-18 = shallow） | 08-18 | `audit_formC_b_code_vs_tex.m` | obs_dump 記錄（未存檔） |
| L18′ | 未觸及項 | H(2,4) 絕對值、Q33 對物理式、P44[0] 三項、F(4,4)×F(4,5) 聯合（補償性錯誤會漏） | — | — | — | 08-18 | 同上 | — |
| L19 | 可觀性（W=500 步×15 段，CRLB/prior 最佳窗） | formC_b best：rank 8/8、ā_w 0.021 PASS、b **0.86**（修 clamp 後；原 0.82 是釘死版）；飽和曲線 W 125→2000：4.71/1.92/1.10/0.866/0.861 | best | seed 7 | canonical 08-17（shallow） | 08-17/18 | `verify_state_observability.m`＋`obs_dump.m` | — |
| L20 | Var 識別式非高斯檢查 | 跨 seed 峰度四段全 2.971（高斯 3.00）；Jensen（逐 seed a_true）動 0.19% | 部署 | 400 | deep | 08-20 | `check_formC_var_identity.m` | `raw_seeds.mat` |
| L21 | 極點代換預測 λ_eff = 1−g(1−λc) 對 Var 超額 | a/a_o 0.0827：g 0.8388、λ_eff 0.7484、dx 實測 1.1766 vs 預測 1.0728；末端 hold 超額 dx 10.8pp 中 7.2pp、dxr 5.2pp 中 3.9pp（2/3–3/4）；三路線 acf 擬合 λ_eff 0.760／由 â 算 0.746 | 部署 | 400 | deep | 08-20 | `check_formC_var_identity.m` 後段 | `raw_seeds.mat` |
| L22 | **Stage A** 谷底 hold z 偏差（(â−a_true/a_nom)/(a_true/a_nom)，t≥3.75）：bmid 基準 vs bmid＋ap_known | 基準 **+20.56% ± 8.81**（sd）；ap_known **−22.11% ± 0.47**；配對差 **−42.67%，t = −47.9**；逐 seed 散布塌縮 **19×**（8.81→0.47）⇒ **換號**，非收縮 | bmid（lock_b, b=8/9）vs bmid＋ap_known | 100 配對 (1:100) | deep | 08-24 深夜 | `run_stageA_apknown_pair.m`、`judge_stageA_apknown.m`；預登記 `stageA_prereg.txt`（W1–W4 接線檢查、P1–P5） | `am_r22_deep/stageA_bmid_base_100.mat`、`stageA_bmid_apknown_100.mat` |
| L23 | A_a·M 刪除確認有作用（ap_known 臂 A_a=0，:1082）：√P44 比 ap_known/base 分窗 | far hold **1.001**／descend **0.415**／osc **0.626**／trough **0.566**；\|K1\| descend 比 **1.623**（未塌縮）⇒ P44 確實變小（過度自信條件成立），但 K1 沒跟著塌 | 同 L22 | 10（P44 近確定性） | deep | 08-24/25 | `check_apknown_P44.m` | 同 L22 |
| L24 | 求積檢查：真值斜率沿真值高度積分，谷底 hold 對真值 ā | trapezoid **0.08716** vs 真值 **0.08717**（−0.01%，儀器檢查過）；**left-endpoint 0.11398（+30.8%）**；a′ 沿軌跡跨 **370×**，估測器每步加 a′·Δw̄ 且 a′ 取左端點 | ap_known 臂 log（bmid） | 3（確定性積分題） | deep | 08-25 | `check_quadrature_on_true_slope.m` | 即時 run，未存 .mat |
| L25 | **predict replay 閉合**（§5 vii；obs_dump 全狀態，bit-identical 開關 max\|diff\| 0 於 seed 7） | Kalman 恆等式閉合 **1.1e−16**（兩臂）；索引約定 `x_upd(4)[i] == a_bar_hat_out[i+1]`、`x_upd(3)[i]·R == delta_x_hat_3_out[i+1]` **精確**；MA 記憶項 α(m̂₁+m̂₂) 對 M_row4 的 rms 比 1e−4（far/desc/osc）～8e−3（trough），累積貢獻 ≤1% a_T（base −0.42% trough／−0.82% 全程；ap_known +0.97%／+1.06%）；Route B 加 driver log 後 bit-check max\|diff\| = 0 | bmid base 與 bmid＋ap_known | seed 7（閉合）；100（量值） | deep | 08-25 | `l2_replay_routeA.m`、`l2_replay_routeB.m`、`l2_replay_bitcheck.m`、`l2_replay_wiring_guard.m`；預登記 `l2_replay_prereg.txt` | `am_r22_deep/l2_routeA_results.mat` |
| L26 | **Route A 離散化項** D = Σleft − Σtrap（z，descend＋osc 窗，對 a_T 百分比） | base **+9.115%**（sd 0.195，SEM 0.019）；ap_known **−9.365%**（sd 0.185）；midpoint 與 trapezoid 差 <0.2 pp；x/y D ≈ ±0.5% 且 sd 大（無命令運動＝控制組）；谷底 hold 偏差重現 +20.44±8.82／−22.25±0.47 | 同上 | 100 × 2 臂配對 | deep | 08-25 | `l2_replay_routeA.m` | `l2_routeA_results.mat` |
| L27 | Route A 配對指紋（斜率取值點） | base：a′[k] == (8/9)(1−â[k−1])² 到 **1e−16**；ap_known：a′[k] == a′_true[k] 到 **1e−16**（對 a′_true[k−1] 偏 2e−2）⇒ base 是左端點（步 Δw̄_d[k−1] 起點的 â）、ap_known 是**當前真高度**的真值斜率（= 該步終點） | 同上 | 100 | deep | 08-25 | `l0_pairing_fingerprint.m`／`l2_replay_wiring_guard.m` | 同上 |
| L29 | Route A == Route B（離散化項 D，z，descend＋osc，% a_T） | D = +9.115 ± 0.195（SEM 0.019）base／−9.365 ± 0.185 ap_known；midpoint 與 trapezoid 差 <0.2 pp；**Route A 與 Route B 差 ≤0.001 pp**；trough-only D +0.71／−1.32 | bmid base／bmid＋ap_known | 100 配對 | deep | 08-25 | `l2_replay_routeA.m`、`l2_replay_routeB.m` | `l2_routeA_results.mat`；圖 `l2_replay_base.png`、`l2_replay_apknown.png` |
| L30 | **端點判別（§4 1b 假說 → [量到]）** | ap_known 改餵 a′_true[k−1] → D = **+9.365**（與 base +9.115 差 0.25 pp 內）；base 改用 â[k] 的法則 → D = **−9.115**（負控制）。成因：base a′[k] = (8/9)(1−â[k−1])² = 步**起點**斜率（左法則，1.1e−16）；ap_known a′[k] = a′_true[k]（1.1e−16）= 步**終點**，因 driver `:1113` 在 ap_known 區塊（`:1117`）**之前**已把 hb_prev 覆寫成當前高度 ⇒ 右法則 | 同上 | 100 配對 | deep | 08-25 | `l2_replay_rerun_pair.m`、`l2_replay_wiring_guard.m` | 同上 |
| L31 | 真值路徑求積表（a′_true × Δh_true，descend＋osc 累積和，**% of a_T**（谷底真值），與 L29 同約定） | right −1007.22 ± 5.28；left −967.42 ± 5.11；trapezoid **−987.317 vs Δa_true −987.305**；left/right 各偏 ±19.9 ⇒ L24 的 +30.8% 是**同一物件在不同窗**（trough 累積、3 seeds）；濾波器只看到 ±9.4，因 M_row4 **不含** Brownian 二次變差 Σa″·dh² | 真值路徑 | 100 | deep | 08-25 | `l2_replay_routeA.m` | 同上 |
| L32 | MA 記憶項（由閉合反推）＋接線護欄 | base +0.579 ± 0.547（SEM 0.055）／ap_known −1.476 ± 0.500 pp of a_T；rms 比 3.6e−3／3.3e−3；護欄 ma2_aug=false 殘差 1.8e−16／2.2e−16（錯 shift 9.6e−4、錯單位 2.7e−1 會露餡）；B-exact 1.6e−16；driver 三行 log（`run_formC_b.m` :1055/:1196/:1257，`delta_x_hat_3_out`）後 100 seeds 兩臂 bit-identical = 0 | 同上 | 100 | deep | 08-25 | `l2_replay_bexact.m`、`l2_replay_bitcheck.m`、`l2_replay_wiring_guard.m` | 同上 |
| L33 | **LOO 電池基準**（arm best，z）：a_T 0.08735；hold 偏差 **+19.52 ± 9.06**（SEM 0.91）；desc peak 22.80；osc RMS 5.75；innov₁ acf hold lag1 −0.016／lag2 +0.056；fixture bit-identical | best（production） | 100 (1:100)，五臂配對 | deep | 08-25 | `l2_loo_run_arms.m`、`l2_loo_judge.m`；預登記 `l2_loo_prereg.txt` | `test_results/l2_loo/l2_base_100.mat`；console `l2_loo_judge_console.txt` |
| L34 | LOO `ma2_aug=false`（OFF−ON） | innov₁ acf hold lag1 −0.016 → **+0.226**（t 217）、lag2 +0.056 → **+0.243**（t 143）；osc lag1 +0.0035 → +0.302 ⇒ **重現 formB 定罪 [0.30, 0.24]**；hold 偏差 +0.677 pp（t 2.45）、desc peak +0.87（t 2.84）、**osc RMS −0.40（t −5.70，關掉反而低，照記）**；\|K_a_y2\| 比 0.669；driver 指標 23.67／5.35／20.34 | best vs ma2_off | 100 配對 | deep | 08-25 | 同上 | `l2_ma2_off_100.mat`；圖 `l2_loo_ma2_off.png` |
| L35 | LOO `y2_echo_corr=false`（OFF−ON，hold 窗） | innov₂ 均值 −5.498e−4 → −5.579e−4（t −2.55）；\|K_a_y2\| 比 **0.969（t −6.1，往下不是往上）**；hold 偏差 +0.275 pp（t 2.54）；osc RMS +0.10（t 2.6）；driver 23.08／5.85／19.91。⚠ **hold 簽名結構性盲**：echo_fac 只乘 back-off 項（:1280–1281），Grad=0 時整項消失 ⇒ hold 窗量不到 echo；osc/descent 窗重判**待 agent** | best vs echo_off | 100 配對 | deep | 08-25 | 同上 | `l2_echo_off_100.mat`；圖 `l2_loo_echo_off.png` |
| L36 | LOO `fe_row4_full=false`（OFF−ON） | 代理簽名：\|K_b_y1\| 比 OFF/ON 轉折點 ±10 步 **0.959 vs osc 窗 0.960**（無轉折點特有衰減）；\|K_b_y2\| 0.908／0.890 ⇒ 登記簽名**代理上過期**；但**承重**：hold 偏差 **+1.991 pp（t 4.44，三者最大）**、OFF 臂 seed sd 9.06 → 7.08、desc peak +0.115（t 2.1）；driver 22.92／5.83／21.63。⚠ 代理 = P 累積的 K_b 且 ma2 開（M_tot 從不精確為 0）；直接量 M_tot／F_e(4,5) 於轉折點**未 log** | best vs fe4_off | 100 配對 | deep | 08-25 | 同上 | `l2_fe4_off_100.mat`；圖 `l2_loo_fe4_off.png` |
| L37 | 二階 echo×ma2（both_off） | hold 偏差交互 **−0.628 pp（SEM 0.036，t −17.4）非可加**（兩者同拔比只拔 ma2 動得少）；innov₂ 交互 t +16.4；acf 交互 +3.5e−4（可忽略）；osc RMS 交互 t −4.5；driver 23.61／5.37／19.98 | best vs both_off | 100 配對 | deep | 08-25 | 同上 | `l2_both_off_100.mat`；圖 `l2_loo_both_off.png` |
| L38 | Clamp／gate 普查（post-clamp，下界；100 seeds × 7680 步） | ā floor 0.05 於 z：3–5 樣本／1 seed（可忽略）；**ā ceiling 0.9999 於 x：2575 樣本／95 seeds／每 seed 最多 84；y：2857／99／84**（x/y 幾乎每顆 seed 都釘上界 ~25–29 樣本/seed）；z 0；b floor/ceil 0；gate 0；NaN 0 | best | 100 | deep | 08-25 | `l2_loo_judge.m`（census 段） | `l2_base_100.mat` |
| L39 | **echo 窗重判**（descent／osc；post-hoc 改窗，登記為 deviation） | S 槓桿上限 \|ā′·Grad\|/â：descent 均值 0.00325（max 0.0282）、osc 0.00443（max 0.0143）、hold 0 ⇒ (1−S) 最多動 ŷ₂ **0.9%**（均 0.1–0.15%）。OFF−ON：descent innov₂ 均值 +1.874e−4 → +1.679e−4（**t −19.98**）、\|K_a_y2\| 比 **1.314（t +168）**、cum K2(4)·innov₂ +6.27 → +7.68 pp（diff +1.41，SEM 0.42）；osc innov₂ t −3.01、\|K_a_y2\| 比 **1.203（t +60）**、cum y₂ 貢獻 −2.91 → −3.58 pp（diff −0.67，SEM 0.12）；Var(innov₂)/R2 = 0.298 全窗全臂（= 1/IF，設計如此）；hold â 效應 +0.275 pp 不變 ⇒ **REPRODUCED，保留**。⚠ 圖第 2 列（osc innov₂）是 console 數字非可見特徵（均值位移 5e−6） | best vs echo_off | 100 配對 | deep | 08-25 | `l2_loo_judge_windows.m`；console 已附加 | `l2_echo_off_100.mat` |
| L40 | 窗重判一致性列（ma2_off／fe4_off 於 descent/osc） | ma2_off：descent acf lag1/lag2 0.003/0.017 → **0.311/0.251**、osc 0.003/0.029 → **0.302/0.253**；osc 窗偏差 +1.637 → +1.290（t −8.9）。fe4_off：任一窗 K_b 衰減皆 <4%；descent 偏差 −0.300 → −0.314（t −6.7） | 同上 | 100 配對 | deep | 08-25 | 同上 | `l2_ma2_off_100.mat`、`l2_fe4_off_100.mat` |
| L41 | **形狀判準循環（C-1）**：章程判準 sup\|θ_eff−θ₀\| ≲ √P[0] 的分母來源（code 已核） | formC：driver `local_envelope_b_range`（`:578–598`）`b_half = max\|b_true − 8/9\|`，b_true = a′_true/(1−a_true)² 讀 `calc_correction_functions` ⇒ **分子 ≡ 分母，比值恆 1.0000**；formB：`run_formB_ws.m local_envelope_priors`（`:581–626`）sPbb = max\|b_eff−9/8\|、sPpp = max\|p_eff−1\|（b_eff=(c−1)(w̄−1)、p_eff=−((w̄−1)+9/8)c′/(c(c−1)））⇒ 同樣循環；expgain：`_5state_expgain_alg.m:164` 預設 0.10 = 真值 sup 0.0942 手圓（tex Stage 2b），循環退一步。⇒ `shape_ledger.md` 的「TIGHT 1.02–1.06×」**無資訊** | 三形式 | — | deep [1.100, 23.222]／shallow [1.900, 23.222] | 08-25 | `test_script/integration/verify_shape_exponent_bound.m`（**NEW, DRAFT**，檔頭列 13 條假設；`res = verify_shape_exponent_bound('all')`） | 圖 `test_results/shape_bound/shape_bound_{formc,formb,expgain}_{deep,shallow}_perp.png` |
| L41′ | 真值無關分母兩種：(A) 兩錨差 Brenner 近壁 c=1/ε vs Faxén 倒數遠場 c=1/(1−9/8u+½u³)；(B) 次階複合 Cox–Brenner (1/ε+⅕ln(1/ε)+0.971) 於交叉點 w̄ 1.9409 以下／Faxén 以上 | (A)：formC b \|1−8/9\| = **0.1111**；formB b, p, ws 皆 **0.125**；expgain **0**（退化，兩極限皆 b=1）。(B)：formC 0.338、formB p 0.295、expgain 0.267 | — | — | — | 08-25 | 同上 | — |
| L41″ | 判準結果（⊥ 真值；PASS <0.9／TIGHT ≤1.1／FAIL >1.1；* = 循環） | **formC deep** sup 0.03889 @ w̄ 1.100 → driver 1.000*／A **0.350 PASS**／B 0.115；shallow 0.02191 @ 2.193 → 1.000*／0.197／0.065。**formB deep** b 0.07977 @ 1.100 → 1.000*／0.638／0.567；p 0.03728 @ 1.100 → 1.000*／0.298／0.126；ws 0.10677 @ 23.222 → 0.962（driver 0.111 非循環但 caller 給）／0.854／0.794；shallow b 0.01568 → A 0.125、p 0.02447 → 0.196、ws 同 0.854。**expgain deep** 0.09417 @ 1.424 → driver 0.942 TIGHT／A 退化／B 0.352；shallow 0.08469 → 0.847／退化／0.317。帶號輪廓：formC deep +0.039 @1.10（b_true→1 接觸）、內部極小 −0.022 @2.193；formB ws 單調 −0.008 → +0.107（錨定法則遠場 ws 結構上 9/8，「校正」0.111 即該漸近） | 三形式 | — | deep／shallow | 08-25 | 同上 | 同上 |
| L41‴ | ledger 重現檢查 | expgain ⊥ [1.1,10] sup 0.0942 @ 1.424 **對上 tex**（0.094 @ 1.424）；∥ [1.1,10] **0.568 vs tex 0.778 MISMATCH**（tex 定義域下限低於 1.1，未明寫） | expgain | — | [1.1,10] | 08-25 | 同上 | — |
| L42 | **正規化邊界 V1：換單位制不變性**（um→nm：R、k_B ×1e3、γ_N ÷1e3、h_init／amplitude／meas_noise_std ×1e3；`physical_constants` 影子函數） | 27 欄位全 PASS：無因次 log（ā̂、Q33、Q44、R2、innov₁₂、h̄、f̄、b̂、K_*）最大相對差 **1.7e-12**；長度／增益 log 恰 ×1e3、力 log ×1 ⇒ U0–U4 無一處漏 R 或 a_o；a_o = Ts/(γ_N R) 的 1/R 與 κ_T 的 /R 是一組 | best | seed 7 | deep | 08-30 | `test_script/scratch/l4_norm_v1_unit_invariance.m` | console |
| L43 | **正規化邊界 V2：κ_T 對 plant** | 遠場／3.32／1.10 三高度 × 三軸九個比值 Var(Δw̄_T)/(κ_T ā) = **0.994–1.007**（N=1e5，1σ 0.45%，最大偏 1.6σ）；κ_T = 4.976e-5 = Einstein 4DTs/R² 逐位；錯誤定義 a_o = Ts/γ_N 會給 0.444 ⇒ 排除。預登記門檻 ±0.5% 比抽樣 σ 還緊，改 ±2σ 照記 | plant only | 1e5 | — | 08-30 | `l4_norm_v2_kappaT_plant.m` | console |
| L44 | **正規化邊界 V3：實現 ε_w 對容器（L18′ 的「Q33 對物理式」）** | 由真值鏈重建 ε_w = −(e⁺ − λc e) − [f̄ e_ā]_k − α Σ[f̄ e_ā]_{k−i}；四窗 Var(ε_w)/Q_full = **0.987／0.930／1.007／0.984**（hold1／desc／osc／hold2，判準 ±10%）；對 tex S8 截斷式 κ_T ā 比 **1.17–1.21**，S3 預測 1.18–1.23 ⇒ ref tex「低估 1.18×」實測坐實；確定項在 osc 只值 0.1%。⚠ 樣本對齊：遠場變異數分不出 shift 0/1（0.987 vs 0.986），descent 偏差分得出（\|mean/sd\| 0.02 vs 0.56）⇒ shift 0 為物理對齊 | best | seed 7 | deep | 08-30 | `l4_norm_v3v4_epsw_container.m` | console |
| L45 | **正規化邊界 V4：ξ 讀數鏈識別式** | hold 段 Var(δw̄_r)/[C_dpmr κ_T (ā+ξ)]：hold1（遠場，ā 0.949）**0.975**；hold2（帶頂，ā 0.087）**1.095**（判準 ±10% 內但貼邊；與 08-26「λ_eff≠λc、R₂ 近壁未建模 ~30%」同向）；ξ(z) = 1.5265e-2 對定案 1.53e-2 | best | seed 7 | deep | 08-30 | 同上 | console |
| L46 | **地毯式三路對帳（08-31）**：主迴圈 vs tex／init-seed vs tex／tex 獨立重推（sympy） | **執行路徑零錯誤**：F_e 49 格、Q 每格外積、H、predict、R₂、取值點（clamped posterior）、buffer index 全對；tex 數學零錯誤（限值有理數精確、a_o²γ_N/Δt ≡ a_o/R 恆等）。實質項：(i) `b_init` 預設 9/8 撞 `b_ceil` 1.05（:757/:760，08-24 舊案仍在，production 傳 8/9 不活）；(ii) ŵ_s 語意缺口——code `w0_nominal = ws0_perp − 1`（:748，pole 名義 0），tex 未寫 nominal；(iii) 過期註解群（:132/:1305/:1397 的 da 敘事、:858 段、`local_seed_level` 檔頭）＝ §1.7 家族殘留；(iv) 休眠診斷不一致：`q44_scale≠1` 時 R₂ 延遲項吃縮放前 Q44（:1260 vs :1353，已驗）；warmup 凍結漏 K(8:9)（t_warmup=0 不活）；(v) slot 7 帶 w0_law inert reporter，tex 說 6–7 空；tex 側 4 個 IMPRECISE：f̄_dw 無定義、Var(w_T)=κ_T ā 的條件化（Itô ~0.15%，同 08-26 a′ 取值點族）未宣告、DARE 種子塊丟 −F_dw e_ā 的獨立性未宣告、R 塊符號未指 SSOT | best | — | — | 08-31 | 三個 fork agent＋本體覆核 | console |
| L42 | **Stage A 重跑（端點修正後）**：bmid vs bmid＋ap_known，谷底 hold z 偏差 | base **+21.046 ± 8.846**（sd）vs ap_known **+21.653 ± 0.537**；配對差 **+0.61%，t = +0.69**，n=100 ⇒ **P2：偏差存活**。斜率家族 {斜率, F_e(4,3) 開口, A_a·M} **對均值排除**，只解釋逐 seed **散布**（sd 8.8 → 0.54，16×）。P4：F_e(4,3) trough 比 apknown/base 1.087。P5 y₁ leg trough：base +0.01493（系統 +0.01471／cov +0.00022，1.4%）、apknown +0.01503（系統 +0.01716／cov −0.00213，14.2%）——總量不變、拆分移動；y₂ 佔 â 行程 base 4.29%／apknown 0.34%。接線：b 精確 8/9 兩臂；a_prime_out／a_prime_true[k−1] 中位 **1.000000**（左端點）；a_true 兩臂差 1.3e−4（plant 依控制而異，預登記「必為 0」不可達、作廢）；init 列非全零 | bmid（端點修正：driver hb_prev 覆寫移至 `:1185`，ap_known 區塊之後） | 100 配對 (1:100) | deep | **08-30**（tree `2047696` ＋ 未 commit 的 predict_quad 編輯，fixture 0） | `judge_stageA_apknown.m` 逐字重跑 | `stageA_bmid_{base,apknown}_100.mat`（**08-30 覆寫**，08-24 版不存） |
| L28 | Stage A 接線檢查 W1/W2（`stageA_prereg.txt`） | W1 max\|b̂ − 8/9\| = **0 精確**（兩臂）；W2 median a_prime_out／a_prime_true[k−1] = 1.00067 = a_nom 檔案值 0.01470588／手打 0.014696 ⇒ 接線 **1.0000**；W3（a_true 跨臂相同）與 W4（ā_hat 第 1 列為零）**登記寫錯、作廢**（W4 見儀器陷阱 1） | 同 L22 | 100 | deep | 08-24/25 | `judge_stageA_apknown.m` | `stageA_bmid_*.mat` |

儀器陷阱（讀這些 .mat 前必看，memory nearwall 08-24 儀器交接）：
1. `baseline_budget_100.mat`／`fdet_off_budget_100.mat`／`truea_100.mat`／`stageA_*.mat` 存**原始 log 含 init-only 第 1 列**，judge 腳本讀取時 `(2:end,:)`；`stack_deep400.mat` 存檔前已丟。⚠ 第 1 列**不是全部為零**：`a_bar_hat_out` 第 1 列 = **種子值**（controller init 回傳 `diag.a_bar_hat = a_bar_seed_v`，:828）、`a_xm_out` 第 1 列 = 0、`h_bar_true_out` 是 **N×1**（單一牆法向高度，非 per-axis）、`a_prime_out` 是**物理單位**（`/a_nom` 才是 dā/dw̄）。「全零列」只是 a_xm/P 欄的露餡簽名，不能拿 ā_hat 欄判。索引約定（L25 精確驗證）：controller 第 i 步的後驗 `x_upd` 存在 log 第 i+1 列。
2. `a_true_out` 是物理單位，需 `/a_nom` 才對 `a_bar_hat_out`；`a_prime_out` 需 `/a_nom`（`check_Fe43_growth.m:12`）。
3. `a_bar_hat_out` 是 post-clamp；`b_hat_out`/`P_b_out`/`K_b_*` 四份都沒存（driver 有 log，08-24 才補 `K_b_y1/y2_out`）。
4. `a_true_out(k)` 屬於 `p_true_out(k−1)`（step_dynamics 前後求值）；`user_config()` 預設 meas_noise 10 nm，重組常數一律從 `r.ctrl_const` 讀。
5. 誤差棒：seed 才是重抽單位（delete-one-seed jackknife），bin 內 τ_eff 法大 16.6×。

---

## §3 Layer 2 定罪表（疊加態成員）

「定罪基底」= 該成員當初被證明有效時的 law／帶／λc／controller。「formC deep 重驗」= 在 formC_b ×b 寫法、deep 帶、
現行全旗標疊加態下，用**它自己的簽名**（非總分）LOO 重驗。✓ 已量到簽名重現；半 = 只有間接證據或非 LOO；
✗ 未量 = 從未在終基底拔過。
⚠ **08-30 註記**：下表所有 08-25 LOO 的 ✓（ma2、echo、fe4 代理）量在 `l2_base_100.mat`，該基底早於 production `1a70599`
（seed 7 對現行臂 max|diff| 8.06e−3，§4 #19）⇒ 依規則 A-2，這些 ✓ 是「前一基底」的簽名，新基底重跑前不算收斂。

| 成員 | 登陸日期 / commit | 定罪基底（law, band, λc） | 定罪量與數字 | formC deep 基底重驗 | 交互嫌疑 |
|---|---|---|---|---|---|
| `y2_whiten` | 2026-07-27 `ae4caf`（powerlaw 首發）；formB/formC 繼承 | powerlaw 5-state；canonical shallow；λc 0.7 | a_xm AR(1) → R₂ 高估 39×；â 0.92–0.95 是 R₂ 假象（memory powerlaw_r2_whitening）；Meng 線 §28–29：x 慢游走 ~2.4×、誠實比 7–14× | **半**：L4/L5 驗了白化後每步式 0.99（shallow 200）與 IF 記帳（deep 400）；y₂ 自相關 0.644→0 五步內歸零（shallow）；**未做 whiten=false LOO**。註：controller `:922–929` 白化是純量（Meng 線 §45 有 per-axis 版，formC 沒有） | 與 `y2_echo_corr` 共用 H2_scale=a_cov（:1268）；與 IF_eff 的 s^τ 加權（§4 #3）；Meng §44 B 場景「z 過衝 = whiten 高通 × 實現力配對」二環鏈——formC 未檢查 |
| `y2_echo_corr` | 2026-08-01 `2c8ebe`（formB_ws）；formC 繼承 | formB_ws 7-state；canonical **shallow**（遠場 h̄ 22.2 配對強制失配）；λc 0.7 | S = 0.323±0.043 實測 vs Lyapunov 0.319；ŵ_s honesty 1.12→0.93、RMS 0.0697→0.0650（N=20 C2）；S(h̄) 近常數 0.310–0.319 | **半**：L18 從 H row2 尺度反推 S=0.3144（seed 7，shallow）；08-19 判別「(1−S) 若作用於噪聲則 Var(y₂) 比恆 0.46」實測 0.98–1.02 ⇒ R2 不需 (1−S)²（shallow）；08-12 formC_dist：拿掉 S 對 hold 漂移 −0.18±0.31 %/s t=−1.65 n.s.、K₂ ×1.47（shallow 8 seeds）。**✓ 簽名重現（08-25 窗重判 L39），保留**：hold 窗登記簽名結構性盲（echo_fac 只乘 back-off 項，Grad=0 整項消失，L35）；改 descent/osc 窗：innov₂ t −19.98／−3.01、拔掉後 \|K_a_y2\| ×1.314／×1.203（t +168／+60）、cum y₂ 貢獻 +1.41／−0.67 pp。**槓桿極小**（(1−S) 最多動 ŷ₂ 0.9%，均 0.1–0.15%），機制只在命令運動時作用；hold â 效應 +0.275 pp。仍未量：S_T/S_n 於 init 一次計算（:395–419），近壁極點非 λc（L21）時 S 前提是否成立；`check_formB_echo_S_measure.m` 結果 [未核] | 與 L21 極點正回饋同機制（S 就是「讀數跟隨施加增益」）；與 R2（(1−S) 不進 R2，`am_r22` 帳 3）；與 `y2_whiten`（同一 H2 列） |
| `ma2_aug` | 2026-08-01 `2c8ebe`（formB_ws） | formB_ws；canonical shallow；λc 0.7 | 三臂 N=20 配對：白 Q b 預算 1.963 FAIL／dc_match 1.074／增廣 **1.088 PASS**；innov_y₁ lag1-2 白化 [0.299,0.243]→[0.003,0.015]；desc 5.68→3.85%；DC 實測 2.149 vs 預測 2.162 | **✓ 簽名重現（08-25 LOO，L34）**：拔掉後 innov₁ acf hold lag1/lag2 −0.016/+0.056 → +0.226/+0.243（t 217/143），與 formB 定罪 [0.30, 0.24] 同量級 ⇒ **保留**。副作用照記：hold 偏差 +0.68 pp、desc +0.87、**osc RMS −0.40（關掉反而低）**、K_a_y2 ×0.669。前置證據：L8 deep 誠實 0.88–0.99 | 與 `fe_row4_full`（M_tot 含 α(m₁+m₂)，:1170）；與 Q44=a′²Q33 容器；與 y₁ 誠實比 |
| `use_fdet` | 6-state 時代（2026-06）；formB/formC 繼承 | eq17 4/6-state；**祖本 −24.3% drag-down**（controller :951–953 註解）；Meng 線 ch4 λc 0.4 | Meng §19–20 定罪整流 +11–17%；**Meng §35 翻案**：stale_ff 落地後 fdet 應關（z 帶 3.2–4.9→2.1–3.2%） | **✓（方向）**：L15 deep 100 配對——關掉 fdet 末端 hold 偏差 +0.00243 t=+4.43 顯著變差、Cov 佔比 86→93.9% ⇒ 在 formC deep **保留**，但只擋 13%。⚠ **08-12 formC_dist shallow 相反**：fdet off 使 hold 漂移 +3.08→+1.03 %/s、seed 11 RMS 10.69→6.32（見 §6 矛盾 #2） | 與 `fe_row4_full` 同為 F_e 第 3/4 列的 F_dw；與 P(4,1) 路徑（L11/L12）；Meng 線證明 fdet 判決依賴 stale-lead 修復狀態 ⇒ 與 timing-lead fix 二階交互**未量** |
| `fe_row4_full` ＋ timing-lead fix | 2026-08-01（formB_ws；`Delta_wbar_d_km1`/`F_dw_km1` 配對，:1153–1165） | formB_ws；shallow | 「Dropping (1−lc)·dw₃̂ zeroes the parameter columns at every turning point」（:1159–1160 註解）；定罪數字 [未核，memory 未存] | **08-25 LOO（L36）：登記簽名於代理上過期**（\|K_b_y1\| 轉折點 ±10 步 0.959 vs osc 窗 0.960，無轉折點特有衰減）**但承重**（hold 偏差 +1.991 pp t 4.44 三者最大、OFF 臂 sd 9.06→7.08）。⚠ 代理 = P 累積 K_b 且 ma2 開；直接量 M_tot／F_e(4,5) 於轉折點未 log ⇒ **決定前須用直接簽名重登記**。家族註記 expgain/powerlaw 同構未修 | 與 `ma2_aug`（M_tot）；與 `ap_src`（M 乘 a′(â)）；與 `use_fdet`（F_dw_km1） |
| `ap_src='post'` | 2026-08-13 `c870eb2`（true-slope arm ＋ pred/ewma）；`3e37de0` 08-20 加 cmd/act | formC_dist／formC_b；shallow（08-19 8 seeds）與 deep（08-20 8 seeds） | 08-19：牆偏 20% 代價 +0.25 pp（post）vs +2.29 pp（cmd）⇒ **主機制非缺陷**；08-20 deep：鎖 b 時 post 14.25 vs cmd 23.10；自由 b 時 post 13.82 vs act **7.10** | **✓**（deep 8 seeds 兩欄表）；但 L12 顯示 post 讓 F_e(4,3) 隨 a′(â) 暴增 317×，且「隱含對沖消掉 b 可辨識性」（08-24 b 線）——**效益與病灶同源，尚無分離實驗** | 與 b 估測（對沖搶 b 的資訊）；與 F_e(4,4) 的 A_a·M（:1524）；與 `ap_known` 臂（§5 vi） |
| `y2_on` | driver 層（`:152, :317–318`） | — | 08-12 四臂（formC_dist base，shallow，seeds 7/11）：y₂ 佔 hold 漂移 40–54%；08-21 deep：拿掉 y₂ 誤差 +0.0159→+0.0274（**y₂ 移除 42%**）；末端 hold y₂ t=−17.1 | **半**：deep 只有誤差帳反算（L10），**未跑 y2_off 配對 LOO**；08-12 消融在 formC_dist shallow | 與 G2/G3 閘（deep G3 需 h_bar_safe=1.0）；與 L16 K 通道不平衡 |
| `par_law` | 2026-08-02（formB Stage 1a）；formC 版 = w0_par 單常數 | formB_ws；shallow | x/y desc 34.8→0.92%、hold −25→−0.04%，z 逐位不變（formB） | **✗ 未量**（formC deep：x/y 末端 hold â 偏 +18%、散布 1.6–1.7×√P，L2/L3′ x/y 數字全在 par_law=true 下量）；「z 逐位不變」在 formC 未重驗 | 對 z 理論上解耦（Γ_inv 對角）；但 x/y 是 L2/L14′ 的免費控制組，par_law 錯會汙染控制組 |
| `a_bar_floor`/`a_bar_ceil` | 承 formB | — | 註解「never binds」（:611–617） | **✗ 未量**：`a_bar_hat_out` 為 post-clamp，觸發次數只有下界；deep 谷底 ā 0.087 vs floor 0.05 margin 1.7× | 與 true-a 臂 â 崩掉（desc 峰 54%）時可能觸底 |
| `b_floor`/`b_ceil` | 2026-08-17 `2d1b9d1`；08-18 收窄 0.60/1.05 | formC_b；shallow | 08-14 塌陷事件：舊 [−1,1] δa clamp 釘 b_init 1.14 於上界，露餡 = x_upd(5) 動 P55 不動 | **半**：08-18 恆等性驗（b_old 9/8 / b_new 8/9 鎖定臂 6.1e−16）；deep 上 b̂ 行程 0.213（08-19）在 [0.60,1.05] 內；**觸發計數未 log** | ⚠ controller 預設 b_init 9/8 > b_ceil（1.7）；放寬 prior 臂（√P55 2.14）必撞界 |
| `Pf_a_floor` seed-local 規則 | 2026-08-18 `843474b`（`floor_from_envelope=false`） | formC_b；shallow | 包絡 sup 0.03058 vs seed-local；「包絡 sup 設 t=0 = 同一個錯」；種子改錨代價 +1.81 pp t=+3.38 8/8（那是 b 種子，同批） | **✗ 未量**（deep 上 seed-local 值與 LOO 皆無；driver 每次印出 `:353–355`） | 與 P44[0] 三項（L18′ 未觸及）；與 b 預算判準 |
| `r22_delay_scale` | 2026-08-24 `10e51db` | formC_b；deep＋Meng 10 s | 四臂 0/1/1.105/2.5 配對差 0.000e+00（死旋鈕抓到）；修後 scale=1e6 → R2 +27%；Q44 median 1.92e−7、R2 8.84e−3、延遲項佔 **1.08e−7** | **✓（無標的）**：在現行軌跡量不出差別 | 與 Q44=a′²Q33（近壁 a′ 暴增時才有標的） |
| plant `b(w̄)` curve | 2026-08-24 `10e51db`（plant 端，非估測器） | — | b_wall=b_far 退化精確（max\|ΔB\| 0、CHECK 1 max\|Δā_hat\| 0）；sim wall b≈2.96× 斜率反而小 2.87–4.45× | **✓（退化恆等）**；非疊加態成員，列入是因為它改了「真值」的定義 | 與 `h_bar_floor_drv`、`local_a_prime_true`（driver `:1271–1297` 走 plant_cperp） |

---

## §4 未解清單（兩 session 合併，排序）

1. **â 近壁高估 +18–20%，87% 未解釋，鎖定 P(4,1)/F_e(4,3) 交叉協方差路徑**（L10–L12、L15）。
   已排除：R/Q/P 調校（P44 誠實 1.07）、y₁/y₂ 雜訊模型、fdet（僅 13%）。降落段 Cov(K1,i1) 整流 86%、
   谷底 hold E[K1]=−0.496 系統項 99%——**兩個機制共用一個通道名**。
   **Stage A 首跑（L22，08-24）**：餵真值斜率後 +20.56% → −22.11% 換號——後證為 driver 右端點接線（L30、1d）。
   **Stage A 重跑（L42，08-30，端點修正後）**：base +21.05 ± 8.85 vs ap_known **+21.65 ± 0.54**，配對差 +0.61%（t 0.69）
   ⇒ **P2 偏差存活：{斜率, F_e(4,3) 開口, A_a·M} 家族對 +21% 均值排除**；它只解釋逐 seed 散布（16×）。
   剩餘分帳（[量到]）：≈ **+9**（左端點求積，L29 base D）＋ ≈ **+12**（非斜率路徑，未解）。
   A_a·M 刪除有作用（L23）但只動散布不動均值。來源：`stageA_prereg.txt`、`judge_stageA_apknown.m`。
1b. **求積棘輪在 formC 重現，OPEN**（L24）：真值斜率沿真值高度用 left-endpoint 積分谷底 +30.8%，trapezoid −0.01%；
   估測器 predict 每步加 a′·Δw̄、a′ 取左端點（driver `:1128` 用 hb_prev；controller :1183–1187），而 a′ 沿軌跡跨 370×。
   這是 formB 線 2026-08-06「求積棘輪」（中點法則已推導、從未實作，memory `project_formB_7a_falsified_quadrature_ratchet`）
   在 formC 的再推導。**⚠ 必須以 open 陳述**：left-endpoint 真值斜率積分給 **+30.8%**，濾波器吃真值斜率卻給 **−22%**，
   **號相反**——不得寫成其中一個解釋另一個；差額（−22 − (+31) ≈ −53 pp）的來源是下一個判別標的（§5 vii）。
   **08-25 Route A 已量（L26）**：濾波器自己的 predict 離散化項 D = Σleft − Σtrap 在 descend＋osc 窗為
   base **+9.115%**、ap_known **−9.365%**（各 SEM 0.02）——同一求積規則兩臂**號相反**，量級對稱。
   **[量到] 換號 = 端點配對不對稱，號差 CLOSED（L30，08-25）**：base 的 a′ 取步起點的 â（左法則），ap_known 的 a′ 取
   當前真高度（右法則，driver `:1113` 在 `:1117` 之前覆寫 hb_prev）；判別兩支都中——ap_known 餵 a′_true[k−1] → +9.365（≈ base），
   base 改用 â[k] → −9.115（負控制）。對手「斜率本身的差」被排除（同一斜率只換端點即翻號）。
   L24 的 +30.8% 與 D 的 ±9.4 是**同一物件不同窗**（L31：真值路徑 trapezoid −987.317 vs Δa_true −987.305，left/right 各偏 ±19.9；
   濾波器只看到 ±9.4 因 M_row4 不含 Brownian 二次變差 Σa″·dh²）。
1c. **求積之後的殘差——08-25 版本作廢、08-30 改寫**：08-25 記的「base +11.3 vs ap_known −12.9 號相反、共同殘差否證」
   是在**右端點臂**上算的——那個 −22.1 是右端點臂整個鏡像迴路的結果，不是 D ＋ 共同殘差；lead 08-25 預測
   「ap_known ≈ −13 = 殘差」**錯**。端點一致後（L42）兩臂皆 ≈ +21 ⇒ 偏差 ≈ **+9（左端點求積，L29）＋ ≈ +12（非斜率路徑）**。
   **+12 為 OPEN**：不在斜率、不在 F_e(4,3) 開口、不在 A_a·M（三者已由 L42 排除）；候選回到 y₁ 整流／P(3,2)→P(4,1) 上游（§4 #1），
   y₁ leg trough 拆分在 ap_known 臂移動（cov 份額 1.4% → 14.2%）但總量不變（L42 P5）。
1d. ~~driver 層缺陷~~ **已修（C5，08-30）**：hb_prev 覆寫移至 ap_known 區塊之後（driver `:1185`），ap_known 現為左端點
   （a_prime_out／a_prime_true[k−1] 中位 1.000000）；Stage A 已重跑（L42）。08-24 的 L22 −22.11% 保留為歷史記錄，**不得再引用為斜率臂結果**。
2. **b 形式假說——對「+21% 偏差」已降級（08-30，L42）**：真值斜率（= 完美 b 曲線）不移除偏差 ⇒ b 形式只關乎**散布**與 **b 可辨識性**，
   不是均值偏差的成因。以下保留原案內容供散布線使用：b_true 沿帶 0.867（w̄ 2.193 內部極小）→0.888（遠場）→1（接觸）（driver `:222–225`、memory slope_source）；
   常數 b 配曲線真值 ⇒ 斜率誤差換號（L10″）；「b 差 0.28% → 谷底答案錯 52%」病態敏感（memory nearwall §F_e(4,3)）；
   b̂ 只追 26%（[未核：memory 未存此數，來自 lead 摘要]）；deep 上 b̂ 行程 0.213 vs 需要 2.14（08-20）。
   ⚠ 「b_true 0.888→0.928→1」（lead）與 driver 註解 [0.866978, 0.888225] 不一致，見 §6 #7。
3. **IF_eff 尺度**：遠場 8%（shallow 200，L6）／a_true deep 三軸 10–16%（L4″）；成因 = s^τ=(1−a_cov)^τ 加權沿用 EWMA 時代、
   白化後未重推（memory am_r22 帳 1）。記帳選擇非準度問題（修對只動 0.5%）。08-20 「近壁 16–19%」已於 08-21 撤回。
4. **F_e(4,3)（位置→增益）與 F_e(4,5)（b→增益）是否同源放大**：兩者皆 F_e 第 4 列滲漏欄、皆在近壁/位移下放大；
   若同源則「â 高估」與「b 估不動」是同一病態敏感度的兩個下游。兩線各自查，**未整合**（memory nearwall／b_wall_curve 交會段）。
5. **死旗標 `law_b_formC`＋ driver `da_known` 靜默 no-op**（§1.6、§1.10）：拔掉或接線，二選一；
   現在 `opts.law_b` 與 `opts.da_known` 都給讀者錯誤的「可調」印象。其餘 DEAD 7 項為條件式或設計如此，已列 §1.10。
6. **過期檔頭**（§1.7）：controller :1–13、:14–237、:214–218、:1453–1463、:1651；driver :15–17。
7. **`use_am_lpf` 陷阱**（§1.8）：無 error 攔截。
8. **meas_noise 三軸無硬體來源**（§1.9）：y 軸 10× 歷史已統一，但三軸皆為 scenario 非 spec。
9. **R2 延遲項係數三種寫法（1.0 / 1.105 / 2.5）無標的**：現行軌跡佔 R2 1.08e−7（memory b_wall_curve）；
   只有 Q33/Q44 變大的場景才可判。
10. **y₂ echo S 在近壁的前提**（§3 echo 列）：init 一次算、極點非 λc 時 S 的定義域已離開；未量。
    **08-26 盤點（λ_c 硬編路徑，memory `lambda_eff_treatment`）**：λ_eff = 1−g(1−λc) 只在兩條路一階處理——
    F_e(3,4) = −F_dw（:1563）與 H₂×(1−S)（:1296–1313）。C_dpmr 四消費者：seed（:826）零；讀數（:948）高報
    [推導] g=0.8388 精確 Lyapunov 1.065 vs 一階 1.059–1.063（殘差 0.2–0.6%），[量到] L14 +6.1–7.2% 由 S 吃掉；
    **R₂（:1168／:1572）沒跟**：[量到] Var(a_m)/公式 近壁 1.29–1.35（true-a 1.00–1.05）⇒ R₂ 近壁偏小 ~30%；ξ̄ 二階。
    同根：Q₃₃/Q₄₄ sensor 項 (1−λc)² 應為 (1−λ_eff)²（:771、:881），≈ Q 5%。λ_eff 需 a_true ⇒ 無 c-free 版本，
    R₂ 若補只能以 P₄₄ 做 g 的二階項。
11. **b 的 prior 寬度 √P55[0]=0.0389 單一構造**（sup\|b_true−8/9\| 於包絡），08-20 所有「估 b 不划算」結論掛在它上面（memory slope_source）。
12. **形狀驗收工具 `verify_shape_exponent_bound.m` 不存在**（章程指名；θ₀=錨時判準恆 1.0000）——mainline_audit_plan C-1。
13. **CRLB 124× 判別**（界限漏 Q vs 濾波器次佳）——mainline_audit_plan C-3。
14. **可觀性視窗 W=500 未做 10× 不變性掃描**——mainline_audit_plan B-1。
15. **`stacked-fix-audit.md` 只在 meng-ch4 分支**（§0）。
16. **x/y 軸 ā 釘在 ceiling 0.9999（L38，08-25）**：x 95/100 seeds、y 99/100 seeds 每 seed ~25–29 樣本（最多 84）觸頂；z 為 0。
   x/y 宣告 out of scope，但 **da_clamp 前例適用**：controller `:1302` 只夾 `x_upd(4)`、**P 不動**（已核 code），
   與 08-14 b 塌陷「狀態動 P 沒動」同形狀 ⇒ x/y 的 P44 在釘頂樣本上不誠實；x/y 又是 L2/L14′ 的控制組。
   待辦：(a) 數 pre-clamp 越界量（現 log 只有 post-clamp 下界）；(b) 決定 x/y 釘頂是否汙染 par_law 控制組結論。
17. **echo×ma2 二階非可加（L37）**：hold 偏差交互 −0.628 pp（t −17.4）——兩件不可各自單獨判，§3 交互欄已記；
   Meng §44 NOWHSC「≈可加」先例在 formC 上**不成立**。
18. **形狀判準循環 ⇒ production b prior 窄 2.9×，「估 b 沒用」判決基底待重跑（A-1）**（L41–L41″，[量到]）：
   driver 把 √P55[0] 定義成 sup\|b_true−8/9\|（真值曲線），判準分子 ≡ 分母，比值恆 1.0000（formC/formB），expgain 退一步循環；
   `shape_ledger.md` 的 TIGHT 1.02–1.06× 從未帶資訊（§4 #12 的「工具不存在」與此同根，工具現已有 DRAFT）。
   真值無關分母 (A) 兩錨差給 formC 0.1111 ⇒ 判準 0.350 PASS（餘裕 2.9×）；**production Pf_b_std（deep 0.0389）比漸近推的寬度窄 2.9×**，
   08-14／08-18／08-20 所有「b 買不到準度」判決都在這個 prior 下取得 ⇒ mainline_audit_plan **A-1 重跑**（寬 prior 臂，
   對照臂 b 固定 8/9 同 seed）。formB (b,p) 在 (A) 下 0.638/0.298 仍 PASS 但 ws 0.854 綁在錨定法則遠場 9/8 的結構；expgain (A) 退化為 0，
   只剩 (B) 0.352。**假設待標**：遠場取 Faxén 倒數 vs 加法 Lorentz 1+9/8u 會改 formB ws 列（錨差 1/8 vs 1）。
   也回收 §4 #11（b prior 寬度單一構造）。
19. **LOO 電池基底過期（規則 A-2，08-30）**：`l2_base_100.mat`（L33）早於 production commit `1a70599`（IF(1) 顏色因子）；
   Stage B agent 量到 seed 7 對現行左端點臂 max|diff| **8.06e−3**（非 bit-identical）⇒ L34–L40 的四件判決全在 **pre-1a70599 基底**，
   須待 Stage B judge 報出 production 變動量後在新基底重跑 LOO；在此之前 §3 的 ✓ 標記帶「基底過期」註記。

---

## §5 預登記模板

```
RUN 登記 #___  日期 ______  commit ______  執行者 ______
目的        ：（一句話，量什麼簽名）
臂          ：（旗標／override 全列；與 production 差異逐條）
同時改了什麼：（confound list —— 這個旗標動了幾件事？逐件列，附 controller 行號）
預期簽名    ：（數字＋方向；引用 §2/§3 哪一列；預登記在跑之前寫死）
中間退化值  ：（若兩項抵消，各自的預期量；「總分不動」不是通過）
停損        ：（什麼結果 ⇒ 停手入帳不加碼）
seeds       ：校準 seeds ______ ≠ 裁決 seeds ______（同 seed 配對；報 mean±SEM 與 t）
帶          ：deep / shallow（不得跨帶比 %）
出圖        ：（N 情境 N 組圖，左右並列共用 y 軸，`open`，逐圖指出看哪列）
判決        ：（跑後填；簽名重現 ✓／✗；引用回 §3 對應列）
```

**5.i–5.v 執行狀態（08-25 LOO 電池，`l2_loo_prereg.txt`，arm best，1:100 五臂配對，fixture bit-identical）**：
5.i echo → hold 簽名結構性盲（L35），descent/osc 窗重判**簽名重現，保留**（L39；post-hoc 改窗登記為 deviation）；
5.ii ma2 → **簽名重現，保留**（L34，descent/osc 一致 L40）；5.iii fe4 → 代理簽名過期但承重，**須直接簽名重登記**（L36/L40）；
5.iv 二階 → **非可加**（L37）；5.v 普查 → 跑了，x/y 釘頂新立案（L38、§4 #16）。**首輪 LOO 至此收官**；+20% 主偏差不在這四件裡。
⚠ 5.i 的預期「K_a_y2 ×1.4–1.5」在 hold 窗**沒中**（×0.969），在 descent/osc 窗中（×1.314／×1.203）——預填時沒先問「這個窗看得到嗎」
（規則 B5 的窗敏感性）。echo 圖第 2 列（osc innov₂）是 console 數字（均值位移 5e−6），圖上看不見，讀圖時勿誤判為無效應。

### 5.i `y2_echo_corr` LOO，deep，100 seeds 配對

- 目的：在 formC deep 終基底重驗 echo 修正自己的簽名（§3 echo 列 → ✓/✗）。
- 臂：`ctrl_const_override.y2_echo_corr=false`，其餘 production；配對 seeds 1:100（對 `baseline_budget_100.mat`）。
- 同時改了什麼：(a) H2 整列 echo_fac=1（:1259–1273）；(b) y2_pred 斜率回退項不再 ×(1−S)（:1280–1281）；
  (c) R2 **不變**（(1−S) 本來就不進 R2）；(d) K₂ 放大 ~1/(1−S)≈1.47（08-12 shallow 量到 ×1.47）。
- 預期簽名：K_a_y2 中位數 ×1.4–1.5；末端 hold y₂ leg（t=−17.1，L10′）量值放大；â 谷底偏差 [待填 by lead：方向——
  echo off 讓 y₂ 更相信讀數，而讀數高報 +6%（L14），預期偏差**變大**還是 y₂ 負號拉回**變小**？兩者對手]；
  L4′/L5 每步式與 IF 記帳**不得動**（純估測器側）。
- 中間退化值：若 â 偏差不動但 K₂ ×1.47 ⇒ echo 只改 P 的誠實不改位準（08-12 shallow 結果）。
- 停損：â 末端 hold 配對差 |t|<2 且 K₂ 未放大 ⇒ 旗標沒接線（先 grep :1259）。
- seeds：裁決 1:100；校準（S 值）來自 Lyapunov 不用 seed。
- 帶：deep。出圖：â−a_true 兩臂並列 + K_a_y2 時序 + 末端 hold 配對差直方圖。

### 5.ii `ma2_aug` LOO

- 臂：`ma2_aug=false`（n_state 回 7、Q33 白容器 :1117–1129、P0 3×3 DARE :740–747）。
- 同時改了什麼：(a) Q 由 rank-2 變 rank-1（Q33/Q34/Q44）；(b) predict 少掉 −α(m₁+m₂) 與 a′α(m₁+m₂)（:1191–1200）；
  (c) M_tot 少掉記憶 feedthrough（:1170）⇒ F_e(4,4) 的 A_a·M 與 F_e(4,5) 的 J_b·M 同步變；(d) P0 位置塊不同。
- 預期簽名：innov_y₁ lag1/lag2 自相關從 ≤0.06 回到 ~[0.30, 0.24]（08-01 formB 數字，λc 相同）；y₁ 誠實比離開 0.88–0.99；
  desc 峰值 [待填 by lead]。
- 中間退化值：(c) 讓 F_e(4,5) 的 M 變小 ⇒ b̂ 行程可能**減少**——這是交互不是 ma2 的簽名，要分開報。
- 停損：lag1 自相關不回升 ⇒ 旗標沒接（bit-identical 檢查）。
- seeds 1:100 配對；deep；出圖：innov₁ acf 兩臂並列、â 兩臂並列。

### 5.iii `fe_row4_full` LOO

- 臂：`fe_row4_full=false`（M_row4 = Δw̄_d[k−1] only，:1163–1164）。
- 同時改了什麼：只有 M_row4（一件），但 M_tot 進三處：F_e(4,4) A_a·M、F_e(4,5) J_b·M、predict **不變**
  （:1183–1187 predict 仍用完整 bracket）⇒ 這是 Jacobian-only 的改動，狀態軌跡只經 P/K 間接變。
- 預期簽名：hold 段（Δw̄_d=0）F_e(4,5)=0 兩臂皆然；**轉折點**（Δw̄_d 過零）F_e(4,5)、F_e(4,4)−1 在 off 臂精確歸零
  （註解 :1159–1160）；P55 縮減曲線在轉折點附近出現平台；b̂ 行程 [待填 by lead]。
- 停損：F_e(4,5) 在轉折點不歸零 ⇒ obs_dump 檢查接線。
- seeds 1:100；deep；需 `obs_dump=true` 單 seed 補 Jacobian 圖。

### 5.iv echo × ma2 二階（兩件同拔）

- 臂：`y2_echo_corr=false` ＋ `ma2_aug=false`。
- 預期：退化 ≈ 5.i + 5.ii 可加（Meng §44 NOWHSC 先例：實測略低於可加）；若非可加（|交互| > 各自 SEM 的 2×）記入 §3 交互欄。
- 判準：末端 hold 偏差配對差 = d_i + d_ii ± 2·SEM；innov₁ acf 與 K₂ 各自簽名同時重現。
- [待填 by lead：可加預測值，等 5.i/5.ii 跑完]。

### 5.v clamp／gate 普查

- 目的：補 §3 「✗ 未量」的三個護欄與 G1/G2/G3 觸發計數（deep，含 true-a 臂與放寬 prior 臂）。
- 臂：production ＋ driver 加 log `a_bar_pre_clamp`、`b_pre_clamp`、`diag.guards_individual`（controller :1416–1417 已有，driver `:1141` 只 log OR）。
- 同時改了什麼：純讀值 log，零行為改動（smoke：â[end] 逐位不變，08-21 慣例 0.107505；08-27 R₂ 色彩因子改 IF(1) 後 fixture = 0.108275）。
- 預期簽名：a_bar 觸底 0 次；b 撞界 0 次（best 臂）；G2 0.013%／G3 0%（重現 L 表）；true-a 臂 a_bar 觸底 [待填 by lead]。
- 停損：任一護欄在 best 臂觸發 >0 ⇒ 該護欄升級為疊加態成員，回 §3 補列。
- seeds 1:100；deep；出圖：觸發時刻 raster 三軸。

### 5.vi Stage A：'bmid' 基準 vs 'bmid'＋`ap_known`（無 `ap_law_bias`）— **已跑 2026-08-24 深夜（R22-am session），結果 L22/L23**

- 結果：基準 +20.56% ± 8.81 vs ap_known −22.11% ± 0.47；配對差 −42.67%，t = −47.9；散布塌 19×。**換號。**
- ⭐ **判準教訓（寫進規則）**：預登記只設了「偏差存活 (≥2/3)／消失 (≤1/3)／混合」三支，全部假設**單調收縮**；
  實測是換號，三支都不涵蓋 ⇒ fork 失格，run 不能被判讀成任一支。**每個預登記門檻都必須涵蓋換號**（至少四支：
  存活／消失／換號同量級／換號更大），不只單調縮小。這是 derivation-workflow 規則 7「儀器先驗證」在判準設計端的版本。
- 下方保留原登記內容供對照（哪些是跑前寫的、哪些是跑後才知道的）。
- 目的：判「法則斜率錯」vs「row-4 協方差機制」對 â 谷底 +18–20% 的貢獻，**以 b 鎖定為前提**。
- 臂：A0 = `opts.arm='bmid'`（lock_b, b=8/9 = tex 4-state）；A1 = 同 ＋ `opts.ap_known=true`（driver `:1117–1134`，
  a′ 取前一步真高度 `local_a_prime_true(hb_prev)/a_nom`）。配對 seeds 1:100，deep。**基準也要重跑 bmid**（現有 100-seed 檔全是 best）。
- 同時改了什麼（ap_known 的四個已知 confound，controller 行號）：
  1. predict 積分項 a′ 變真值斜率（:1081 `a_prime_i = ap_known(ax)`）— 唯一想測的；
  2. F_e(4,3) = (1−λc)·a′ 用真值斜率重建（:1523）— 滲漏開口改變；
  3. **A_a = 0**（:1082）⇒ F_e(4,4) 的 A_a·M 整項刪除（:1524）；
  4. `dap_db_i` 在 :1069 已由**法則**斜率凍結、早於 :1081 覆寫 ⇒ F_e(4,5)/H(2,5) 仍用法則斜率——**所以 b 必須鎖**（bmid），
     鎖後 J_b_fac=0（:1090 `double(~lm(1))`）、H(2,5)=0（:1272），此 confound 可證明 inert。
  存活的耦合：F_e(3,4) = −F_dw（:1522）與 F_e(4,4) 中的 a′·F_dw（:1524）**兩臂都在**，且現實中也在 ⇒ â 可讀
  （對比 true-a 臂：現實耦合消失、模型仍在 ⇒ â 崩 54%，不可讀）。
- 預期簽名：這是 {斜率, F_e(4,3) 開口, A_a·M} 的**聯合**測試。偏差消失 ⇒ 三者之一（分不出，需 Stage B 加兩個 default-off
  診斷旗標：`ap_known_keep_Aa`、`ap_known_keep_Fe43`，使用者裁決）；偏差存活 ⇒ 三者**全部排除**，主因在 y₁ 整流/P(3,2) 上游。
  [待填 by lead：A1 末端 hold 偏差預期值與方向；bmid 基準 vs best 基準的差（08-18 shallow：固定 8/9 2.20% vs 估 2.46%）]。
- 中間退化值：偏差減半 ⇒ 記「聯合貢獻 ~50%」不可歸給單項。
- 停損：A0 bmid 基準與 best 基準末端 hold 偏差差 >2×SEM ⇒ 先處理「b 鎖定本身改變偏差」再談 A1。
- seeds：裁決 1:100 配對；校準（b_mid、floor）由包絡推導不用 seed。帶 deep。
- 出圖：â−a_true 三臂並列（best／bmid／bmid+ap_known）共用 y 軸；F_e(4,3) 時序兩臂；y₁ leg Cov 佔比表（console）。
- 跑後補記（08-24 首跑）：W1/W2 通過、W3/W4 登記寫錯（L28）；bmid 基準 +20.56% 與 best 基準 +18–20% 差 <3 pp，
  預登記「b 自由跑可忽略」成立。**但首跑的 ap_known 臂是右端點（1d），其 −22.11% 不可讀為斜率臂結果。**
- **重跑（08-30，端點修正後，L42）**：base +21.05 ± 8.85 vs ap_known +21.65 ± 0.54，配對差 +0.61%（t 0.69）⇒ **P2 偏差存活**，
  斜率家族對均值排除、對散布負責（16×）。W3「a_true 兩臂必相同」在 plant 依控制而異的模擬下**不可達**（差 1.3e−4），作廢；
  P4 F_e(4,3) trough 比 1.087；P5 y₁ leg 總量不變、拆分移動。**Stage B（求積旗標，§5 viii）的預期「−9 pp」現在有了乾淨的基底**。

### 5.vii predict replay（純後處理，離線重建濾波器自己的 predict 增量）— **閉合與 Route A 已跑（08-25），結果 L25–L27**

- 跑後補記：MA 記憶州阻礙**不需改 production**——replay agent 用既有 `obs_dump` 拿到全狀態（開關 bit-identical，seed 7 max|diff| 0），
  α(m̂₁+m̂₂) 項直接量到 ≤1% a_T（L25）；Route B（加一行 driver log）也做了 bit-check = 0。閉合 1.1e−16 兩臂皆過。
- 已量：D = Σleft − Σtrap = +9.115%（base）／−9.365%（ap_known），midpoint≈trapezoid（L26）。
- 已結：換號機制 = 端點配對（L30，[量到]）；Route A == Route B ≤0.001 pp（L29）；MA 項與護欄（L32）。
- 未結：求積後殘差 +11.3 vs −12.9 號相反（§4 1c）。下方為原登記。

### 5.viii Stage B：中點／梯形法則作為 controller 的 default-off 診斷旗標（**改 production 檔，需 Kevin 核准**）

- 目的：把 L26/L30 量到的離散化項 D 從「離線 replay」變成「濾波器內的一個旗標」，直接量它對谷底偏差的貢獻。
- 臂：`ctrl_const.predict_quadrature ∈ {'left'(default, 現況), 'trap', 'mid'}` [名稱待 Kevin 定]；
  'trap' = a′ 取 ½(a′(â[k−1]) + a′(â_pred[k]))（需一次法則預評估）；'mid' = 半步高度重評。base 臂（bmid 或 best）配對 1:100。
- 同時改了什麼：predict 第 4 列積分項（:1183–1187）；**F_e(4,3)/F_e(4,4) 的 a′ 要不要跟著換**是第二件事，登記時二選一寫死
  （建議先只換 predict、Jacobian 不動，讓它是單因子）。
- 預期簽名：base 谷底偏差移動 ≈ **−D_trap ≈ −9 pp**（L26 base +9.115 於 descend＋osc；trough-only 另 +0.71），
  附開迴路 caveat：predict 變了 K 也會跟著變，實際移動可能小於 −9 pp（L32 的 step-mismatch 與修正反相關即是此效應）；
  ap_known 臂（修 1d 後）應同向移 ≈ +9 pp。
- 停損／儀器：旗標 off 時**bit-identical**（regression fixture `l0_regression_fixture.m`，max|diff| 0）；
  on 時 replay 用同法則重算的 D 必須 ≈ 0（閉合），否則旗標與 replay 定義不一致、不得讀偏差。
- seeds：閉合 seed 7；裁決 1:100 配對。帶 deep。
- 出圖：â−a_true 三條（left／trap／mid）共用 y 軸，兩臂各一張。

- 目的：分開 §4 1b 的兩個號相反的數字——「left-endpoint 真值斜率積分 +30.8%」與「濾波器吃真值斜率 −22%」——
  先確認**濾波器實際做的 predict 就是 left-endpoint 求積**（閉合檢查），再問 trapezoid 會把它推到哪裡。
- 臂：無新 run。用 `stageA_bmid_apknown_100.mat`（ap_known 臂，a′ 已是真值斜率）與 `stageA_bmid_base_100.mat`；
  可先用 3 seeds。
- 重建式（每步，z）：`Δā_pred[k] = a′[k]·(Δw̄_d[k−1] + (1−λc)·dw̄₃̂[k] + α(m̂₁+m̂₂))`（controller :1183–1200）；
  輸入全部來自 log：`a_prime_out/a_nom`（物理→正規化）、命令步長（`p_d_out` 差分 /R，注意用 **[k−1]** 的步，timing-lead 配對）、
  `delta_x_hat_3_out/R`（估計追蹤誤差）、MA 記憶州 **driver 目前不 log**（已核，見 §6）⇒ 先補一行純讀值 log
  `m_hat_out = x_e_per_axis([8 9],:)`，否則 α(m₁+m₂) 項只能標為未重建誤差、閉合必不過。
- 同時改了什麼：零（純讀值）。
- 閉合檢查（先於任何比較）：left-endpoint replay 的 Σ_k Δā_pred **必須逐步重現** log 的 predict 總和
  （= 誤差帳 L10 的「法則」項；或由 `a_bar_hat_out` 差分減去 K1(4)·innov₁ 與 K2(4)·innov₂ 反推）。
  閉合門檻 [待填 by lead；L9 恆等式的 1e−17 是同型檢查的量級]。閉合不過 ⇒ 重建式漏項（先查 MA 記憶項與 [k−1] 配對），不得往下讀。
- 預期簽名：(a) left-endpoint replay 閉合 ✓；(b) 換 trapezoid（a′ 取 ½(a′[k−1]+a′[k])，或中點高度重評）後 Σ Δā_pred
  的變化量 [待填 by lead：若求積是 −22% 的成因，trapezoid replay 應把 ap_known 臂終值推回 ≈ 0；若 replay 只動 +30 pp 而
  終值仍為負，則求積與 −22% 是兩個機制]；(c) 對 base 臂同做，看法則斜率下 left vs trap 的差是否同量級。
- 中間退化值：若 trapezoid replay 把 ap_known 終值推到 +8%（= −22+30）⇒ 求積解釋 +30.8 那塊、另有 −53 pp 未解，兩者都要入帳。
- 停損：閉合不過且查不出漏項 ⇒ 停，回頭補 driver log（m̂₁/m̂₂、predict 增量本身），不得用近似式硬比。
- seeds：閉合 3 seeds；量值 1:100（同 Stage A 配對）。帶 deep。
- 出圖：兩臂各一圖，三條曲線（logged â、left replay、trap replay）共用 y 軸；下列 = 三者對 a_true 的差。

---

## §6 來源間矛盾與 [未核] 清單

### 矛盾（兩邊都記，附帶／軌跡）

1. **Kalman 閉合**：1.7e−18（memory var_identity 08-21，`raw_seeds_budget.mat`）vs 「1e−17」（memory nearwall 08-24，
   `baseline_budget_100.mat`）。同一恆等式、可能不同資料檔；量級一致。
2. **fdet 方向**：08-12 formC_dist base 臂 shallow seeds 7/11——fdet off 使 hold 漂移 +3.08→+1.03 %/s、seed 11 RMS 10.69→6.32
   （**變好**）；08-24 formC_b best 臂 deep 100 配對——fdet off 末端 hold 偏差 +0.00243 t=+4.43（**變差**）。
   不同 controller（dist vs b）、不同帶、不同統計量（漂移率 vs 偏差）；Meng 線 §35 另證明 fdet 判決依賴疊加態。
3. **y₂ 的角色**：「y₂ 移除 42% 誤差、末端 hold t=−17.1 最紮實」（08-21）vs 「y₂ 只占 â 更新量 ~1%」（08-24）。
   同一資料族、兩種量法（淨貢獻對總誤差 vs 更新量份額對 predict）；不矛盾但引用時必須標明是哪個。
4. **K 通道不平衡量值**：442×（08-12 formC_dist shallow）／43×（08-21 formC_b deep 100）／115×＝0.0087（08-24 Meng 10 s 單調降）。
   三軌跡三數字，只有方向可轉移。
5. **true-a 臂殘餘**：「殘留 6–14%」（nearwall：指 Var 超額中未被極點解釋的份額）vs 「殘餘不到 1%」（var_identity 結案：指比值離 1）。
   分母不同。
6. **IF_eff 偏差**：8%（shallow 遠場，L6）／10–16%（deep a_true 三軸，L4″）／「近壁 16–19%」（08-20，**已撤回**）／
   「IF 3.29–3.37 該用 3.66–3.69」（08-21）。前兩者可能同一缺陷在兩帶的讀數；撤回項不得再引用。
7. **b_true 範圍**：driver `:222–225` 「0.866978（w̄ 2.193）→ 0.888225（包絡頂）」；memory slope_source 「[0.867, 0.928]」；
   lead 摘要「0.888→0.928→1」；controller :216 「[1.1258, 1.1534]」（舊 /b 約定）。0.928 是否為谷底 w̄ 1.10 的值 [未核]；
   driver 註解上限 0.888 可能是 08-18 shallow 包絡時寫的、未隨 deep 更新。
8. **â 谷底偏差**：+18.3%（08-21 誤差帳）／+19%（同日標題）／+19.5%（fdet 預登記）／+18–20%（nearwall）／20.9%→18.2%（fdet 消融）。
   同量，不同 run 與四捨五入。
9. **lambda_f 否證日期**：controller :454 寫 2026-08-12；reflog `dba3cb4` 為 2026-08-13 12:50 +0800。
10. **h_bar_safe**：driver `:952` 預設 1.5，deep cfg 覆寫 1.0（`canonical_scenario.m:68`）；08-12 memory 說 formC_dist 用 override 1。formC_b deep 實際 1.0。
11. ~~求積 vs Stage A 號相反~~ **已解（08-25，L30/L31）**：ap_known 不是 left-endpoint——driver `:1113` 讓它成了右法則，
   所以 +30.8%（左）與 −22%（含右法則的 −9.4）並非同一規則；號差 = 端點差，判別兩支皆中。殘餘 −12.9 另立 §4 1c。
12. ~~三個求積數字~~ **已解**：同一物件不同窗——真值路徑 left/right 各偏 ±19.9（descend＋osc）、trough 累積 +30.8（3 seeds）、
   濾波器看到的 ±9.4（M_row4 無 Σa″·dh² 項）。仍不可互相當「份額」，但關係已寫清（L31）。
13. **a_nom**：檔案 0.01470588 vs 手打 0.014696（Stage A W2 的 1.00067 全由此 typo 造成）；引用時一律讀 `.mat` 的 `a_nom` 欄。
14. **expgain ∥ 真值 sup**：`verify_shape_exponent_bound` 於 [1.1, 10] 給 **0.568**，`5state_expgain_hd.tex` 記 **0.778**（L41‴）；
   ⊥ 兩邊對上（0.0942 @ 1.424）⇒ 差異在 tex 的定義域下限低於 1.1 且未明寫。兩數字各附定義域後才可引用。
14b. **ap_known 臂兩個結果**：−22.11 ± 0.47（08-24，L22，**右端點** driver `:1113`）vs **+21.65 ± 0.54**（08-30，L42，左端點修正後）。
   同名臂、同 seeds、同檔名（08-30 覆寫）；差 ≈ 44 pp 全由端點接線造成（L30 判別已證）。引用 ap_known 一律附日期與端點。
   連帶：§4 1c 08-25 版的「殘差 −12.9」與 lead 08-25 預測「ap_known ≈ −13」皆作廢。
15. **shape_ledger.md「prior margin TIGHT 1.02–1.06×」vs L41 比值恆 1.0000**：前者是同一循環判準的不同 run 讀數，數值差只是 θ₀≠錨時的殘餘；
   兩者都不是形狀驗收證據，`shape_ledger.md` 該行待更正。

### [未核] 項目

- L13 讀數重建的數值結果（memory 只記「通過」）。
- L16a 的 seeds 數與腳本名（b 線 08-24 memory 未指名）。
- L9 08-24 誤差帳腳本名（memory 列了 y1 系列三支，未列帳本身）。
- `fe_row4_full` 當初定罪數字（08-01 formB，memory 未存）。
- `Pf_a_floor` seed-local 在 deep 的數值（driver 每 run 印出，未存）。
- `check_formB_echo_S_measure.m` 是否跑過、結果。
- b̂「只追 26%」：原始出處已找到 = `stageA_prereg.txt`「b_hat was measured to move only 26% of b_true」（量測 run 未指名）；
  「靈敏度 200%/1%」仍找不到，memory 有的是「b 差 0.28% → 谷底錯 52%」與 b̂ 行程 0.213。
- ~~Stage A W1–W4 接線檢查的實際輸出~~ 已核（L28）：W1 精確 0、W2 接線 1.0000（1.00067 = a_nom typo）、W3/W4 登記寫錯作廢。
- ~~driver 是否 log MA 記憶州~~ 已核：driver **不 log** m̂₁/m̂₂（只有 `delta_x_hat_3_out` :1055 與 `log_P_full` 的 `P_full_out` :1203）；
  但 replay 改走 `obs_dump` 取得全狀態，阻礙解除（L25），production 未動。
- ~~「換號 = 端點配對不對稱」[假說]~~ 已升 [量到]（L30）。
- L41 工具為 DRAFT，檔頭 13 條假設未逐條核（本檔只核了三個分母來源的 code 行）；遠場形式（Faxén 倒數 vs Lorentz 加法）對 formB ws 列的影響未量。
- ~~L31 真值路徑表的單位~~ 已核：% of a_T，descend＋osc 累積和，100 seeds（lead 08-25 確認）。
- §4 1c 殘差 −12.9 的 step-mismatch／修正 反相關分帳（replay agent 已量到 sd 但未報各自均值）。
- 0.928 作為 b_true 谷底值。
- `stacked-fix-audit.md` 是否已在其他分支 merge 進 motion-test（本檔以 ls 結果為準：未）。
