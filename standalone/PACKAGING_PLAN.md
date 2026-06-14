# 6-state Standalone Packaging Plan (SSOT)

**本文件是打包工作的唯一真值來源。** 每個工作 session 開頭先讀本文件；所有決定、邊界、
gate 定義以此為準。逐部分的決定紀錄在 [DECISIONS.md](DECISIONS.md)。

- Created: 2026-06-10 (第 0 段產出, branch `pack/6state-standalone`)
- 母 repo 真值: `feat/eq17-6state` 上的 `model/controller/motion_control_law_eq17_6state.m`
- Spec 對照: `reference/eq17_analysis/kf_canonical_spec.md` (KF flow + notation SSOT)
- 推導源頭: `reference/eq17_analysis/RevisedConrol＿Vpersonal.pdf`

---

## 1. 目的

建立獨立的 6-state Vpersonal 控制器 MATLAB 專案：

1. **可交付他人檢查** — implementation fidelity review（code 是否忠實於 spec）
2. **使用者完全掌握** — 對每一行程式碼的架構與內容可對 reviewer 講解
3. **自我建構 review** — 打包過程本身 = 系統性檢查「當初有沒有建構對」

第 (3) 點由每部分的「理論 gate」承擔（見 §6 雙 gate）。

## 2. 整體模擬框架（一頁版）

```
這個模擬回答的問題：
「6-state Vpersonal EKF 控制器能否在壁面附近 (h_bar > 1.2) 以 ~30 nm 精度
 操控磁珠定位，並同時即時估測時變 mobility a_x(h)？」

+---------------- 物理世界（連續, ode4 10 us）--------------------+
|  磁珠動力學  p_dot = Gamma_inv(p) * (f_d + f_T)                  |
|  Gamma_inv = 位置相關 mobility（wall effect c_para/c_perp）       |
|  f_T = 熱力白噪 (var = 4*kBT*gamma/Ts)                           |
+----------+--------------------------------------^---------------+
           | p (真實位置)                          | f_d (控制力, ZOH)
           v                                      |
   [感測: + 量測噪聲 n_x, d=2 步延遲]               |
           | p_m                                  |
+----------v--------- 控制器（離散 1600 Hz）--------+---------------+
|  量測鏈: delta_x_m -> IIR -> a_xm（mobility 的「第二個量測」）      |
|  控制律: Eq.17 ACTIVE form（delta_x_m 直接回授 + Sigma f_d 補償）  |
|  估測器: 6-state EKF [dx1 dx2 dx3 xD_d a_x da_x]                 |
|          paper predictor form + Joseph（D6）                     |
+------------------------------------------------------------------+
```

## 3. 適用範圍：h_bar > 1.2，等級 L2

| 層 | 宣告 | 依據 |
|---|---|---|
| 位置追蹤 | h_bar > 1.2 全域達標（h=50 定量 gate ~30 nm） | LF drift 審計: tracking 與 AR(1) λc 理論吻合 |
| â 估計 | h_bar ≳ 2 區 bias ~1%；**近壁 1.2–1.5 的 â 偏差是已知結構限制，如實文件化** | 審計: gate-open 近壁 Q55 random-walk 高估 LF 變異 237×；gate-latch h_bar=1.11 系統性 −25~−33% |
| 不在範圍 (L3) | â 在近壁全程達標 — 屬 Phase A 研究（mean-reverting gain model / S≠0），不是打包工作 | spec §8 backlog |

實作面：standalone config 設 `h_min = 1.2 * R = 2.7 um`（母 repo user_config 預設 1.5R；
安全檢查 `check_trajectory_safety` 邏輯不變，純參數）。

## 4. 已定案決定（12 項）

| # | 項目 | 定案 |
|---|---|---|
| 1 | 目的 | 他人可檢查 + 使用者高掌握度 + 自我建構 review |
| 2 | 適用範圍 | h_bar > 1.2、L2（§3） |
| 3 | 結構 | 11 檔（§7） |
| 4 | Controller 風格 | 23-state 模板：numbered steps [0]–[11]、persistent、4-arg 簽名 `(del_pd, pd, p_m, params)`、ekf_out 自然 (x,y,z) 序、無 diag 第三輸出 |
| 5 | KF form | D6 paper predictor form + Joseph（spec §1b 5 項 checklist）；persistent 存 a-priori 對 |
| 6 | 砍掉 | G1/G2/G3 + R_OFF + 1D/2D 切換（純移除、無替代保護；**package-only，母 repo 保留 guard**）、legacy warmup、d=1 分支（hardcode d=2）、diag struct、7-state 相容物（P77/Q77、(x,z,y) 軸序）、option 旋鈕（t_warmup_kf / h_bar_safe / iir_warmup_mode）、R22_prefactor alias、重複 default |
| 7 | 保留 | prefill + DARE init（a-priori 化）、wall-aware a_hat 種子、Q33 三成分、Q55 閉式 [2/(1+lc)]·(a·K_h/R)²·sigma2_dh、R22 = K_var·IF_eff·(a_hat+xi)² + buffered delay {1,1}、時變 F_e、Joseph、per-axis 迴圈、IF_abc 自檢 assert |
| 8 | Notation | 照 kf_canonical_spec notation 表（不自創）+ §8 的 ~10 處收斂 |
| 9 | 環境/語言 | MATLAB R2025b 對標（不做向下相容）、英文註解（風格照 23-state） |
| 10 | D6 順序 | 先母 repo（feat/eq17-6state 做 + 驗證），package 再 verbatim 複製 |
| 11 | 場景組 | h50（5-seed 定量 PASS）+ h10（描述性）+ ramp 50→2.7 um（h_bar=1.2 邊界實證 + 近壁 dwell 數值穩定） |
| 12 | Branch | D6 在 `feat/eq17-6state`；打包在 `pack/6state-standalone`（D6 後 merge 進來）；打包 commits 用 `pack(N):` 前綴 |

## 5. 三段式流程與狀態

> **⏩ 從這裡接續（2026-06-14, HEAD = `pack(9)`, 已 push origin）— 8 部分全部 DONE**
> **下一步 = §9a main_run 改腳本+設定區塊（清單已定案見 §9）→ §9c 軌跡/圖討論 → §9d 剪臍帶。**
> 部分 1–7 全部 DONE（走讀簽收陸續補確認，不影響接續）。controller_6state.m 是**完整
> EKF + Q/R**，已對母 repo 證明 rounding-floor 等價（h50/h10 ~3 ulps；唯一差異 = 近壁
> h̄<1.5 的 a_hat，L2 邊界）。git 全乾淨,7 個 gate 檔(part1–7)全 PASS。**下一步 = 部分 8
> 驗證層**:寫 verify_standalone（三場景 PASS 門檻）+ main_run（入口）+ make_figures +
> README（框架圖/notation 表/三個「不是 bug」預警/envelope 聲明，講解文字由使用者改寫定稿）。
> 新 session 開頭一句:「讀 standalone/PACKAGING_PLAN.md,做部分 8」即可接上。逐部分細節見
> [DECISIONS.md](DECISIONS.md)。

| 段 | 內容 | 狀態 |
|---|---|---|
| 第 0 段 | 本文件 + DECISIONS.md + 分支建立 | **DONE 2026-06-10** |
| 第 1 段 | 母 repo 前置：D6 遷移 → verify h50 gate → golden baseline → spec anchors → merge | **DONE 2026-06-11**（等價 gate 答案卷 = golden_post_d6.mat） |
| 第 2 段 | 8 部分 cycle（見下方逐部分進度） | **DONE（8/8）** |
| 收尾 | standalone/ 搬出成獨立 repo（剪臍帶）：歸檔母 repo 比對 gate(1/2/3/7) PASS transcript + git init + 母 repo 留 pointer | **NEXT** |

**第 2 段逐部分進度**（commit / gate 檔）:

| # | 部分 | 狀態 | commit | gate 檔 |
|---|---|---|---|---|
| 1 | 物理世界 | **DONE + 簽收** | pack(2) | gates_part1（E1–E4 bit-exact, T1–T6） |
| 2 | 模擬骨架 | **DONE + 簽收** | pack(3) | gates_part2（G1–G3 bit-exact, T1–T3） |
| 3 | 量測鏈 | **DONE + 簽收** | pack(4) | gates_part3（C1/C4 live；C2/C3 snapshot） |
| 4 | 控制律 | **DONE + 簽收** | pack(5) | gates_part4（D2 live；D1 snapshot） |
| 5 | EKF 遞迴 | **DONE + 簽收** | pack(6) | gates_part5（E1 DARE, E2 h50, E3 envelope） |
| 6 | Q/R 驗證輪 | **DONE**（走讀簽收待確認；不改 code） | pack(7) | gates_part6（F1/F2/F4/F5 + F3；F2/F5 ground truth） |
| 7 | 閉迴路等價 | **DONE**（走讀簽收待確認） | pack(8) | gates_part7（H1 h50/H2 h10 rounding-floor equiv；H3 ramp L2 邊界） |
| 8 | 驗證層 | **DONE**（走讀簽收待確認） | pack(9) | verify_standalone + make_figures + main_run + README |

**重要**:部分 5+6 的 code 在 pack(6) 一起落地(EKF 與 Q/R 執行上不可分,見 DECISIONS
「部分 5 前置決定」)。**部分 6 不再寫 code**,只驗 Q/R 內容物。

## 6. 第 2 段：8 部分清單與雙 gate

每部分流程：**討論**（方塊圖位置 + spec 對照 + code 預覽 + 為什麼）→ **實作** →
**檢查**（雙 gate + code-reviewer subagent 獨立審 + 走讀會 + 反向檢核 3–5 題）→
**DECISIONS.md 記一筆** → 使用者簽收 → 下一部分。每部分一個 `pack(N):` commit。

雙 gate 定義：
- **等價 gate**（vs 母 repo golden baseline）：implementation fidelity。搬移/改名段 bit-exact；
  行為變更段統計等價（同 seed 5 顆，tracking std / a_hat mean·std 容差 < 1%）。
- **理論 gate**（vs 閉式公式）：construction correctness — 服務「自我建構 review」目的。

| # | 部分 | 內容 | 理論 gate（例） |
|---|---|---|---|
| 1 | 物理世界 | gamma_inv、wall_corrections（含 K_h/K_h'）、thermal、ode4 step | c 函數 vs 有限差分 < 1e-5；MSD ratio ≈ 2.0（單邊積分慣例）；Gamma_inv 對稱/正定 |
| 2 | 模擬骨架 | driver 雙率迴圈（1600 Hz / 10 us）、d=2 量測延遲鏈、RNG 順序 | 延遲鏈逐步 k-表驗證；開迴路 Brownian 模擬統計 |
| 3 | 量測鏈 | delta_x_m、IIR（a_pd/a_cov 雙 pole）→ a_xm | 餵合成訊號驗 sigma2_dxr = C_dpmr·4kBT·a + C_n·sigma2_nx（emp/closed ≈ 1） |
| 4 | 控制律 | Eq.17 ACTIVE form（Sigma f_d 在 1/a_hat 括號外、過去力用過去 a_hat 加權） | 完美知識（a_hat=a_true, xD=0, 無噪）下 delta_x[k+1] = lc·delta_x[k] |
| 5 | EKF 遞迴 | D6 predictor form steps [3]–[10]、Phi ≠ F_e、Joseph | P 遞迴穩態 vs DARE；innovation 白化檢查（safe region） |
| 6 | Q/R | Q33 三成分（{4,1} 權重）、Q55、R11、R22 + IF_eff（{1,1} delay） | Q55 emp/closed ≈ 0.998 重現；IF_abc 自檢 R_fT(0)=C_dpmr |
| 7 | Init | constants（C_dpmr/C_n/K_var/IF_abc/xi）、DARE → P_f0 a-priori 化、prefill、wall-aware seed | **使用者親手重算 constants 對帳**（C_dpmr=3.1610, C_n=1.1093, K_var, xi） |
| 8 | 驗證層 | verify 三場景 + PASS 門檻、figures、README（含 envelope 聲明 + 三個「不是 bug」預警） | 完整 5-seed gate；**README 講解文字由使用者改寫定稿** |

部分 3–7 在 controller 單檔內逐節成形（[0]–[11] 編號結構先立骨架）；
部分 1–2 完成後即有可跑的開迴路模擬（第一個可檢查里程碑）。

README 必載的三個「不是 bug」預警：
1. Predict 用 Phi（Row 3 只 lc）不是 F_e — 刻意，spec §3（deterministic-map，7-state a_hat bias 的解法）
2. R22 delay {1,1} ≠ Q33 randgain {4,1} — 刻意不對稱，spec §6（r_2 線性 vs process 平方累積）
3. F_e(3,4) = −1 不是 −1.6 — 6-state 合併座標（delta_x_D^d），−1.6 是 7-state 座標

## 7. 目標檔案結構（11 檔）

```
eq17_6state_standalone/
├── README.md                 審查地圖：框架圖 + 閱讀順序 + notation 表 + 預警 + envelope
├── main_run.m                唯一入口（addpath + config → sim → figures）
├── config.m                  全部參數一處（含單位、h_min = 1.2R）
├── controller_6state.m       ◄ 控制器本體唯一一檔（~420–460 行）
│     主函數 [f_d, ekf_out] = controller_6state(del_pd, pd, p_m, params)
│     [0] init（constants + DARE a-priori + prefill, est 於首呼叫建立）
│     [1] control law  [2] measurement/IIR  [3] innovations  [4] R[k]
│     [5] gain  [6] state update (Phi·x + L·e)  [7] Joseph  [8] F_e[k]
│     [9] Q[k]  [10] forecast  [11] shifts + ekf_out
│     local: compute_if_abc / build_F_e / if_eff_eval / solve_dare_kf
├── physics/
│   ├── wall_corrections.m    c_para/c_perp + K_h/K_h'（controller 與 plant 共用 — 刻意）
│   ├── gamma_inv.m
│   ├── thermal_force.m
│   └── trajectory_ref.m
├── sim/
│   ├── run_simulation.m      時間迴圈（無 7-state dispatch）
│   └── step_dynamics.m       ode4 10 us
└── test/
    ├── verify_standalone.m   三場景 + PASS 門檻（等價 gate 腳本兼用）
    └── make_figures.m
```

Reviewer 動線：README → controller_6state.m（從頭讀到尾，對照 kf_canonical_spec）→
跑 verify → 自由探測。「檢查控制器整體」= 1 個 code 檔 + 1 份 spec。

## 8. Notation 收斂（在 kf_canonical_spec 表之上的修正）

照搬 spec 表不動：lambda_c, delta_x_m, dx_r, a_hat, a_xm, f_d, Fe3_a/Fe3_da/F_1_i/F_2_i,
var_da_ram, C_dpmr, C_n, K_var, IF_eff/IF_abc, xi_per_axis, K_h_axis, sigma2_dh。

| 修正 | 現況 → 定案 |
|---|---|
| Sensor noise 三名 | sigma2_n_s / sigma2_nx / sigma2_noise → 統一 `sigma2_nx` |
| 增量因子兩名 | → 統一 `var_da_increment_factor`（刪 controller fallback） |
| gamma_N_p | → `gamma_N` |
| d_delay vs d | → `d`（diag local 變數已不存在，無撞名） |
| slot 4 兩名 | xD_comb / x_D_hat → code 統一 `xD_comb`；輸出欄位 `xD_d_hat` |
| 4*kBT*a 字面式 | → 顯式中間量 `sigma2_dXT` |
| 3.96 的稱呼 | 註解一律 `C_dx`（= 2+1/(1-lc^2)），永不寫成 C_dpmr |
| Phi 隱形 | [6] banner 註明 Phi（Vpersonal p.4）≠ F_e（p.5），引 spec §3 |
| slot magic numbers | 檔頭 state layout 表 + 具名 index |
| ekf_out 軸序 | (x,z,y) legacy → 自然 (x,y,z) |

## 9. 細節風險清單（逐部分設防）

1. **D6 時序語意**：a-priori 化後控制律吃的 a_hat 不含當步量測；log 對 a_true 有一步位移（spec §1b item 3/4）— 第 1 段重點
2. **P_f0 方向**：DARE 後驗 → 先驗需多走 F·P·F' + Q 一步（§1b item 2）
3. **「移除 G123 = 無行為差」要實測**：B2 前在母 repo 跑三場景 guard 觸發統計。注意 ramp→2.7 在 h_bar<1.5 段 G3 必觸發 → 該場景 gate 分區比對：h_bar>1.5 段統計等價、h_bar<1.5 段記錄行為差異 + 數值穩定（無 K_h cap，h_bar→1.2 的 Q55/P 增長要實測）
4. **ekf_out 軸序**：grep 清查所有舊 (1,3,2) 取值點
5. **golden baseline config 逐欄寫死**：避開 apply_qr_preset a_cov=0.005 陷阱、noise-off 幻影 sigma2_n=1e-4（LF drift 審計列的 latent traps）
6. **RNG 順序**：rng(seed) 維持在 params 計算之前（994cd3f 教訓）
7. **f_d/delay buffer off-by-one**：task04 歷史雷區，列走讀重點
8. **IF_abc 自檢**保留為 assert（R_fT(0)=C_dpmr 是常數算對的免費保險）

## 10. 協作協定（Claude Code）

1. 純討論 session 開 plan mode；實作 session 正常模式
2. 使用者用 `@檔案` 把 code 拉進對話
3. MATLAB MCP 即時對數：討論公式當場驗算
4. **每部分一個 session**；session 開頭：「讀 standalone/PACKAGING_PLAN.md，做第 N 部分」
5. Gate 分工：快速檢查 Claude 經 MCP 跑；5-seed 完整 gate 使用者本地跑（親眼 PASS）
6. 每部分實作完先派 code-reviewer subagent（唯讀）獨立審，再走讀
7. 部分檢查協定可寫成可重用 workflow 腳本
8. Git：每部分一個 `pack(N):` commit；gate fail 即停
9. 掌握度機制：走讀會 + DECISIONS.md + 反向檢核 3–5 題簽收 + 使用者親手兩件事（constants 手算、README 改寫）

## 11. 不納入範圍（防 scope creep）

母 repo phase docs 的 supersession 註記與過時數值清理（~20 個 7-state 腳本 y noise）、
delta_x_D^d 正式推導補寫（文件討論另案）、Phase A 近壁系列（S≠0 / Q off-diagonal /
P(5,5) 修正 / mean-reverting gain）、6-state observability rank test、Simulink 路徑、
文獻/參考文件附帶範圍（使用者指定後議）。

---

## 9. 收尾與後續（part 8 之後，2026-06-14 規劃）

8 部分全部 DONE（pack(0)–pack(9)，已 push origin）。剩餘工作分三塊，**尚未實作**：

### 9a. main_run 改「腳本 + 頂部設定區塊」（使用者要求：在 code 裡改、不用指令）

main_run 改成 SCRIPT，頂部一個「模擬設定」區塊，使用者改值按 Run，不打指令、不進 config。
config.m 仍是完整參數定義 + 預設；main_run 頂部的覆蓋值透過「config 多接受一個 overrides 參數」
傳進去（對使用者透明；既有 gate 不傳 overrides，行為不變）。

**頂部設定區塊（露出的可調旋鈕，定案清單）：**
```
scenario   = 'h50';      % 'h50' | 'h10' | 'ramp2p7'
seed       = [];         % [] = 隨機（印出實際種子）; 或數字
T_sim      = [];         % [] = 場景預設; 或秒數
lambda_c   = 0.7;        % 閉迴路極點 (0~1)
a_pd       = 0.05;       % IIR 均值 EWMA pole
a_cov      = 0.05;       % IIR 變異 EWMA pole
meas_noise = true;       % 量測噪聲 開/關
thermal    = true;       % 熱力 開/關
```
- **seed = [] → 隨機**：main_run 內 `if isempty(seed); seed = randi(2^31-1); end` 並 fprintf 印出。
- config 留在內部、不露出的：物理常數 (R/gamma_N/Ts/kBT)、d=2、wall θ/φ/pz、t_hold、**meas_noise_std（使用者要求不露出，留 config）**。
- **軌跡參數（h_init/h_bottom/amplitude/frequency/t_hold）暫不放頂部 → 等 9c 軌跡討論定案。**
- 順帶補缺口：每次跑完把 out 存 `.mat`（目前只存圖、數值跑完即失）。輸出資料夾名待 9c 定。

實作接縫：config(scenario, overrides) 選用第二參數（nargin 檢查，向後相容）；
run_simulation(scenario, opts) 把 opts.overrides 轉給 config；main_run 蒐集頂部設定 → opts → 呼叫。
覆蓋值只限「不漣漪」的（lambda_c/a_pd/a_cov/T_sim/meas_noise*/thermal）；h_init/h_bottom 由 scenario 決定。

### 9b. 交付結構（使用者定案：test/ 完全不進交付包，連 verify 都不放）

交付包 = README + main_run + config + controller_6state + physics/ + sim/ + make_figures（從 test/ 搬出）
+ results/（輸出）+ docs/。**test/（gate_* + verify_standalone）只留開發端，不交付**；
交付改附 **docs/VERIFICATION.md**（一頁報告，摘要 8 部分所有 gate PASS 數字 = 交付前已驗證的證據）。
make_figures 必須搬出 test/（執行時要用）。

### 9c. ⚠ 兩個延後討論項（使用者明示：等 9a 做完再單獨討論，務必保留）

**這兩項尚未設計，只是登記為「待討論」，不要在 9a 順手做掉：**
1. **軌跡怎麼設定** — h_init/h_bottom/amplitude/frequency/n_cycles/t_hold 要不要進 main_run 頂部、
   要不要加 osc 振盪場景、軌跡參數的編輯介面長怎樣。（9a 頂部區塊「暫不放」軌跡參數就是為了等這個討論。）
2. **最後輸出的圖** — 目前只有 fig1 gain / fig2 tracking；要畫哪些、內容、數量、存到哪個資料夾、檔名規則。

### 9d. 剪臍帶（最後）

歸檔 gate 1/2/3/7 的 PASS transcript → 寫 VERIFICATION.md → 移除 test/ → git init standalone 成獨立 repo → 母 repo 留 pointer。

### 使用者親手工作（掌握度協定，未做）

- README ✎ 標記的講解 prose 由使用者改寫定稿。
- 親手重算 constants 對帳：C_dpmr=3.1610 / C_n=1.1093 / K_var / xi。

