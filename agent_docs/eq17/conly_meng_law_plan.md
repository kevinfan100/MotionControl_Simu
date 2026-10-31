# c-only 換成期刊控制律 (J6)：計畫與進度

> 分支 `test/conly-meng-law`，worktree `/Users/kevin/Code/MotionControl_Simu-conly-meng`，從 `7d715b4` 分出（2026-10-07）。
> 目的（使用者 10-06 定）：**與期刊一致**（Meng, Long, Menq, IEEE TIE 72(1) 2025），不是期待 ā 變準；
> 通過標準 = 三牆 e_ā 不變差。估測器留 c-only（ā_w + 常數 c，p = 2）；x̂_D 式子照寫、code 不加；λc 0.7。
> 推導 `reference/eq17_analysis/derivation/1007_conly_meng_law.tex`（式子）＋ `_ref.tex`（文字、對照、數字、驗收）。
> 每階段做完把結果填在「結果」欄；過關條件事先寫死，不事後改。

| 階段 | 做什麼 | 你會看到 | 過關 | 狀態 |
|---|---|---|---|---|
| 0 開獨立位置 | worktree + 分支；基準 run（現行控制律，f1p2，ramp_tri 1 Hz，plane/cell，seed 1）存 `test_results/temp_conly_meng/baseline_<plant>.mat` | 路徑、分支、基準數字 | 跑完、無 NaN | **完成** |
| 1 推導 | tex 九節 + 期刊式號；sympy 24 項；獨立 agent 複核；ref 含對照表、五處差異、讀數鏈數字、預登記驗收 | 兩份 PDF | 驗算全過、使用者同意 | 待使用者看 |
| 2 code | `formC_b.m` 旗標 `control_law = 'j6'`：順序對調、控制律、F_e 列 3/4、Q33、P[0] 閉式、C_n(K₁)／IF／S 查表；runner `opts.cc_extra` | 改動行號、三個檢查 | 旗標關逐位相同；unit suite；旗標開跑完無 NaN、min h > 1 | **完成** |
| 3 迴路 | â 鎖真值，定高 22.2／2.0／1.10 R × 3 s × 10 seeds，四個數字（衰減率 0.70、Var 對閉式、ā_m 偏差 ≈ 0、innovation 白） | 4 × 3 表 + δx_m 並列圖 | 四項全在 SEM 內（細則 ref §8） | **完成** |
| 4 三牆 | 可觀性 4、5 關；ramp_tri 三牆 plane init 10 seeds，新舊同 seed 配對 | 每牆一張並列圖 + 配對差表 | 谷底／頂端／三角段配對差在 SEM 內；δx_m std 不變差 | **跑完，字面判準未過（差 0.01 級、一致的位準平移），等使用者裁定** |
| 5 收尾 | 併入 motion_ctrl.tex、假說帳、memory；commit；等 Layer-1 commit 後合併回 test/motion-test | commit 清單、合併差異摘要 | 合併與 push 前問使用者 | — |

## 結果

### 階段 0（10-07）
- 基準（commit 7d715b4，現行控制律，Q55 = 0）：plane min h_true 1.0994、e_ā(end) +0.0015、ĉ(end) +0.009；
  cell min h_true 1.0880、e_ā(end) +0.053、ĉ(end) +0.416；NaN 0。
- 工具：`test_script/scratch/run_conly_meng_baseline.m`（`'compare', true` = 階段 2 的逐位相同閘）。
- 獨立 MATLAB 行程：`/Applications/MATLAB_R2025b.app/bin/matlab -batch`，不共用其他 session 的 MATLAB。

### 階段 1（10-07）
- `1007_conly_meng_law.tex` 6 頁、`_ref.tex` 3 頁；`test_script/scratch/check_conly_meng_law.py` 24/24。
- 推導的新結果：本迴路 C_dpmr = 2 + 1/(1−λc²) 不變，C_n = 1 + K₁(κ_T â, σ²_n)（原 2/(1+λc)）；
  位置 P[0] 有閉式（不再解 DARE）；閉迴路譜 = {λc, 1−K₁, 0×4}。
- 沿用舊常數的偏差（z 軸）：遠場 +0.9%、2 R +2.3%、谷底 +7.0%。穩定邊界 g_crit 3.37（現行 3.36）。
- 讀數常數工具 `model/controller/calc_readout_consts_kfloop.m`（K₁ 格點上 Lyapunov 精確算 C_T、C_N、IF 三和、S_T、S_n），
  自檢 `test_script/scratch/check_readout_consts_kfloop.m` 10/10（K₁ = 1 對 production 常數差 < 1e-11）。
  含 IIR 合成後沿用 production 常數的偏差：遠場 +0.7%、2 R +2.0%、谷底 +5.8%；熱力係數也隨 K₁ 動（3.16 → 3.36）。
- 獨立 agent 複核（10-07）：九項 PASS（含自己的 Lyapunov／Monte Carlo 重算），三項要改、已改：
  [A] 「先 update 再出力」是設計選擇不是 (J17)(J18) 的前提，期刊逐字讀是 pre-y[k]；0.77／1.16 的不穩是混合版（力吃上一拍、預測器照 λc 寫）；
  [B] 驗收 3(a) 改用後驗 δŵ̄₃ 的 lag-1 自相關 = λc（δw̄_m 的自相關 0.775／0.844 分不開）；
  [C] Gate 3 括號：hold 時 M = (1−λc)δŵ̄₃ ≠ 0，只有 y₁ 列歸零；f_det 的理由是經驗的，(4,3)(4,5) 的 M 照現行慣例保留。

### 階段 2（10-07）
- 改動（行號 = 本分支 `motion_control_law_formC_b.m`）：init 旗標＋assert＋讀數表 `:1252`；P[0] 閉式 `:1478`；讀數 C_T／C_N 查表 `:1758`；
  現行力作廢 `:1810`；Q33 `:2062`；R₂／ξ／S 查表 `:2086`；F_e(3,3)=1、(4,3) 去 ā′(1−λc) `:2151`；[3b] update 後出力 `:2715`；`local_j6_consts` `:2896`。
  runner `run_plaw_walls.m` 加 `opts.cc_extra`。
- 檢查：旗標關 vs 階段 0 基準 isequaln PASS（plane／cell）；`verify_formC_b_production_defaults`、`_law_err_q44`、`_amodel_fe` ALL PASS
  （log `test_results/temp_conly_meng/unit_suite_stage2.log`）；旗標開 seed 1：plane min h 1.0983、cell 1.0875、NaN 0、floor 0。

### 階段 3（10-07）
工具 `test_script/scratch/run_conly_meng_loopcheck.m`（定高、法則增益鎖真值 `a_ctrl_override 'true'`、plane、z、10 seeds、窗 t ≥ 1 s）；
圖 `test_results/temp_conly_meng/loopcheck_dxm.png`；log `loopcheck_stage3.log`。數字 = 平均 ± SEM，欄 = 22.2／2.0／1.10 R：

| 項 | eq17（現行） | j6 | 判準 |
|---|---|---|---|
| (a) ρ₁(δŵ̄₃) | 0.846／0.843／0.822 | 0.698／0.697／0.691 (±0.003) | 0.70；混合版 0.73 → 排除。谷底低 0.009 與 innovation ρ₁ −0.016 同源 |
| (b) Var(δw̄_m)/r 實測 vs 閉式 | 87.0 vs 87.6／43.7 vs 44.0／9.07 vs 9.13 | 87.8 vs 88.4／44.5 vs 44.8／9.59 vs 9.69（SEM 1.1／0.57／0.11） | 差 < 1 SEM（兩律皆 −0.7%，窗內去均值所致） |
| (c) ā_m/ā_true − 1 | −0.006／−0.006／−0.007 (±0.017) | −0.006／−0.006／−0.008 (±0.018) | ≈ 0 ✓ |
| (c′) j6 迴路用舊常數讀 | — | +0.002／+0.009／+0.047 | 相對 (c) 多 +0.8／+1.5／+5.4%，預測 +0.7／+2.0／+5.8% ✓ |
| (d) innovation ρ₁ | 0.31／0.29／0.23（有色：純形無補丁） | −0.003／−0.005／−0.016 | < 2/√N = 0.035 ✓ |
| (d′) Var(innov)/S₁ | 1.01／1.02／1.06 | 0.997／0.990／0.962 (±0.01) | 1 ± 0.1 ✓ |

結論：迴路對上推導；j6 不開補丁 innovation 就是白的，現行純形則有色（這就是三個補丁在補的東西）。

### 階段 4（10-07）
- 可觀性 4、5 關（`test_script/scratch/verify_obs_j6.m`，log `verify_obs_j6.log`）：三牆兩律 rank 5/5、負控制 Inf、c 的 CRLB < prior 全 PASS；
  dump 的 F_e(3,3) = 1、F_e(3,4)·â/Δw̄_d = −1、F_e(4,3)/F_e(4,5) 與 H₂(5)/H₂(3) 對 tex 的比值 1.0000（iqr 1e-11）。
  j6 的 c 的 CRLB 比 eq17 小約 10×（plane 0.0057 vs 0.064），因 F_e(3,3) = 1 讓 e₃ 不被 λc 收縮、y₁ 看得到更多（界限，非實測）。
- 三牆配對（`plot_conly_meng_walls.m`，圖 `walls_{plane,cell,sphere}_s1.png`，e_ā = ā_true − â，j6 − eq17 ± SEM，10 paired seeds）：

| 牆 | 谷底 1／2／3 s | 頂端 1.5／2.5 s | 三角段 mean|e_ā| | δx_m std [nm] |
|---|---|---|---|---|
| plane | −0.009／−0.008／−0.008 (±0.0005) | −0.005／−0.005 | +0.0027 (±0.0007) | +0.29 (17.97 → 18.26) |
| cell | −0.003／−0.008／−0.009 (±0.001) | −0.003／−0.005 | −0.0023 (±0.0008) | +0.22 (24.01 → 24.23) |
| sphere | −0.012／−0.010／−0.009 (±0.0007) | −0.005／−0.005 | −0.0049 (±0.0009) | +0.29 (18.69 → 18.99) |

- 讀法：j6 把 â 整體抬高約 0.005–0.01（三牆、谷底與頂端同號），不是形狀變化；eq17 在谷底低估（e_ā > 0）所以谷底變好、頂端變差；
  三角段 |e_ā| plane 略差、cell／sphere 略好。δx_m std 一致多 0.2–0.3 nm（+1.2–1.6%）= 推導的雜訊份 (1+K₁)r 對 2/(1+λc)r 的代價（帶內 q/r 2–8 預測變異數 +2–6%）。
- 字面判準（配對差在 SEM 內、δx_m 不變差）未過：差都顯著（|t| 3–40）但量級 0.01 級 ā、0.3 nm。等使用者裁定。
- [假說] 位準抬高來自 F_e(3,3) = 1 讓 y₁ 腿給 ā 更多修正（CRLB 10× 即此）；對手：R₂／迴音 S 查表改變 y₂ 腿權重。分開它們：j6 臂 `y2_off` 或 K_a_y1／K_a_y2 更新份拆分（既有 log），約 2 分鐘。未跑。

#### P_fix 臂（使用者 10-07 要求；q55 = P_fix²·I = 本分支的 `q55_adapt 'ratio'`，√P_fix 0.15；rationew 0.15/0.05 未 commit、不在此分支；T_d 1 s、1 Hz）
tag `pfix015_`，圖 `walls_pfix015_{plane,cell,sphere}_s1.png`，log `walls_pfix_stage4.log`。j6 − eq17 ± SEM：

| 牆 | 谷底 1／2／3 s | 頂端 1.5／2.5 s | 三角段 mean|e_ā| | δx_m std [nm] |
|---|---|---|---|---|
| plane | −0.012 (±0.0015)／+0.000／+0.000 | +0.017 (±0.008)／+0.003 | 0.026 → 0.019 (−0.007 ±0.0014) | +0.12 |
| cell | +0.027 (±0.004)／+0.019／+0.018 | −0.003／−0.006 | 0.021 → 0.019 (−0.002 ±0.0013, n.s.) | +0.12 |
| sphere | −0.016 (±0.0017)／−0.000／−0.000 | +0.016 (±0.008)／+0.003 | 0.027 → 0.019 (−0.008 ±0.0013) | +0.13 |

位準（eq17 → j6）：plane 谷底 1 s +0.019 → +0.007、頂端 1.5 s −0.016 → +0.001；cell 谷底 +0.022／+0.014／−0.006 → +0.049／+0.032／+0.012；sphere 同 plane。
健康：eq17 plane floor hits 6、sphere 4（ĉ 撞 0.999 上限的 seed）；j6 1／0／0，ĉ 範圍窄（上限 0.85–0.96 對 0.999）。
讀法：c 槽開著時 j6 在 plane／sphere 的三角段 |e_ā| 低 27%、第一個谷底與頂端都更靠 0；cell 的三個谷底偏高 +0.02–0.03（第 2、3 谷底 eq17 在 0 附近、j6 +0.03／+0.01），三角段均值無差。δx_m +0.1 nm。

#### ĉ 本身（`report_conly_meng_chat.m`，三角段 1–3 s 均值，10 seeds；c_eff 三角段均值 plane +0.058／cell +0.644／sphere −0.056）
| 臂 | 牆 | ĉ eq17 (seed sd) | ĉ j6 (seed sd) | j6 − eq17 | sd/√P eq17 → j6 |
|---|---|---|---|---|---|
| Q55 = 0 | plane | −0.001 (0.009) | +0.008 (0.008) | +0.009 ± 0.001 | 1.95 → 1.35 |
| Q55 = 0 | cell | +0.359 (0.029) | +0.368 (0.027) | +0.009 ± 0.001 | 1.70 → 1.15 |
| Q55 = 0 | sphere | +0.020 (0.013) | +0.032 (0.013) | +0.013 ± 0.001 | 2.23 → 1.52 |
| P_fix 0.15 | plane | +0.033 (0.083) | +0.071 (0.064) | +0.039 ± 0.012 | 0.56 → 0.43 |
| P_fix 0.15 | cell | +0.650 (0.129) | +0.595 (0.079) | −0.055 ± 0.017 | 0.86 → 0.52 |
| P_fix 0.15 | sphere | −0.089 (0.089) | −0.048 (0.066) | +0.042 ± 0.012 | 0.59 → 0.44 |

結論：Q55 = 0 時 ĉ 只整體 +0.01，常數 c 的限制（cell 0.36 對 0.64）兩律一樣；P_fix 時 j6 的 seed 散布小 25–40%（三牆），偏差 plane／sphere 持平或變好、cell 低 0.055（= cell 谷底 e_ā +0.02–0.03 的來源）。
P 的誠實比在 Q55 = 0 從 ~2 降到 ~1.3（j6 的 √P_c 較接近實際散布）。
[假說] 散布變小 = j6 下 y₁ innovation 白（ρ₁ 0.3 → 0），c 經 y₁ 腿的更新不再吃有色雜訊；對手 = R₂／S 查表改了 y₂ 腿權重。分開：K_b_y1／K_b_y2（= ℓ51／ℓ52）更新份拆分，log 已有，約 2 分鐘。未跑。
