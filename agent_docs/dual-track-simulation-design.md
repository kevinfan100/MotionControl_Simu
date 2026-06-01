# Hybrid Simulation Design: Pure-MATLAB Dev Engine + Simulink Oracle

Date: 2026-04-28
Status: 設計討論結束，未實作。等使用者 review 後進入 implementation plan 階段。

---

## 背景

目前所有模擬都綁在 `model/system_model.slx`，造成研究迭代瓶頸：

- **A. 單次模擬慢** — ode4 + step=1e-6，跑 10 秒模擬要分鐘級
- **B. `sim()` startup 重** — compile/init 開銷讓 5-seed CI / parameter sweep 很煩
- **C. 不易 debug** — 無法在 MATLAB Function block 內放 breakpoint，看內部訊號 (P, innovation, state) 不直觀
- **E. 無法 unit test 個別模組** — 想單獨測 EKF 或 trajectory 都得整個跑 Simulink

但 Simulink 架構非常完善，是研究最終整合對象 (publish/oracle)，不能放棄。

**需求**：在保留 Simulink 為 SSOT 的前提下，建立一條快速、可 debug、模組化的純 MATLAB 開發/驗證路徑。

---

## Pre-Discussion Finding

Explore agent 對 codebase 的快速 inventory 發現：

- **約 85% 模擬邏輯已經在 `.m` 檔裡**：7-state EKF (`motion_control_law_7state.m`)、trajectory、thermal force、wall effect、所有參數計算，全是純 MATLAB 函數
- **`.slx` 主要是 orchestrator/wrapper** — 多數 block 是呼叫 `.m` 檔，自己不藏邏輯
- **耦合點**：11+ 測試腳本呼叫 `sim('system_model')`；time-stepping loop / Bus Object / Measurement_Noise block 是 Simulink 專屬
- **預估**：純 MATLAB discrete-time loop 約 150–200 行新 code 即可取代主迴路

→ Pure-MATLAB 路徑技術上可行，重點在於選對切入點與等價標準。

---

## 決策一：架構 — Hybrid 雙軌, Simulink as SSOT

### 三條原則

1. **Simulink 不動** — `system_model.slx` 是 publish/oracle，研究最終結論由它背書
2. **單一 SSOT** — 所有物理 / 控制邏輯只有一份在 `.m` 函數裡。Pure-MATLAB 與 Simulink **共用同一份** `motion_control_law_7state.m` / `trajectory_generator.m` / `calc_thermal_force.m` / `calc_gamma_inv.m` / ...。改一次，兩路徑同步更新。
3. **Equivalence gate** — 一個 canonical scenario，斷言 pure-MATLAB ≈ Simulink。pass 才相信 pure-MATLAB 結論可外推到 Simulink (publish-grade)。

### 痛點 → 解法

| 痛點 | 解法 |
|---|---|
| A. 單次模擬慢 | 無 Simulink Bus / Block / ToWorkspace overhead，且積分步長放大 |
| B. `sim()` startup 重 | 直接呼叫函數，5-seed CI 從分鐘級變秒級 |
| C. 不易 debug | MATLAB 原生 breakpoint 進 EKF 內部，看 P / innovation / state |
| E. 無法 unit test | `test_*_unit.m` 個別測試 EKF / trajectory / thermal |

---

## 決策二：等價標準 — B (Statistical Equivalent)

驗收 pure-MATLAB 與 Simulink 「等價」的標準：

- **階段一 (確定性)**：時間序列 RMS 差異 < 0.1% (噪聲關閉)
- **階段二 (隨機)**：統計量 (mean, std, 研究關鍵 metric: C_dpmr, a_hat std) 差異 < 1%

### 為何不選 A (Bit-exact) 或 C (Behavioral)

- **A 不可行**：Simulink 的 RNG block 與 MATLAB `randn()` 使用不同的 RNG 引擎，同 seed 也不會給出相同隨機序列。即使連續積分能 bit-match，RNG 那關過不了。
- **C 太鬆**：dev 階段觀察到的細微改善 (例如 a_hat std 改善 8%) 可能在 Simulink 不重現，研究結論不可信。
- **B 剛好**：給清楚的 pass/fail 門檻，且只要 RMS < 1% 對研究問題 (C_dpmr 修正、Q/R 驗證、a_hat std) 已綽綽有餘。

---

## 決策三：連續動力學離散化 — 方案 2 (ode4, step = 10 µs)

### Drag_coeff_matrix + Integrator 是關鍵點

整個 Simulink 模型只有這個 block 是 CONTINUOUS — 其他都是 1/1600 Hz 的離散 block，pure-MATLAB 直接用 for-loop 呼叫 `.m` 函數即可。**連續部分是 pure-MATLAB 必須做出選擇的地方**。

數學：

```
p_dot(t) = Gamma_inv(p(t)) * F_total
p(t)     = ∫ p_dot(t) dt              (Integrator, IC=p0)
```

- F_total 在一個 Ts (625 µs) 內是常數 (ZOH from discrete control + thermal force)
- Gamma_inv(p) 是位置相關 mobility 矩陣 (Wall Effect) → ODE 非線性
- Simulink 用 ode4 + step=1e-6 → 一個 Ts 內跑 625 個 inner substeps

### 對你的物理，625 substeps 是 overkill

- 典型參數：f = 1 Hz，A = 2.5 µm → max velocity ≈ 15.7 µm/s
- 一個 Ts (625 µs) 內，position 最多變化 ≈ 10 nm
- 在 h = 50 µm 的工作點，10 nm 對 Gamma_inv 的影響 ≈ 2×10⁻⁴ 比例
- 也就是說：Gamma_inv 在一個 Ts 內幾乎是常數

### 各方案比較

| 屬性 | Simulink (現狀) | 方案 1 | 方案 2 (選定) | 方案 3 | 方案 5 |
|---|---|---|---|---|---|
| 積分方法 | ode4 (RK4) | ode4 | **ode4** | ode4 | Heun (RK2) |
| 步長 | 1 µs | 1 µs | **10 µs** | 625 µs (=Ts) | 625 µs |
| 一個 Ts 內 substeps | 625 | 625 | **62.5** | 1 | 1 |
| 每 Ts 的 RHS evals | 2500 | 2500 | **250** | 4 | 2 |
| 數值差異 vs Simulink | - | bit-equiv | **< 1e-10 RMS** | < 1e-5 RMS | < 1e-4 RMS |
| 整體速度 (相對 Simulink) | 1× | 2-3× | **8-12×** | 15-25× | 20-35× |

### 為何選方案 2 (而非 3 或 5)

- **與 Simulink 同算法 (ode4)**：等價論證最強，reviewer/同行容易接受
- **步長只放大 10×**：保險，對未來物理變動 (頻率上升、振幅變大、Wall Effect 模型修改) 仍有 margin
- **8-12× 加速**對 5-seed CI 已是質變 (分鐘級 → 秒級)
- 方案 3 雖再快 2×，但對未來物理變動 margin 較小
- 方案 5 雖最快，但**不同算法**讓等價論證較弱

---

## 速度估計

假設 Simulink 跑 10 秒模擬目前需 ~60 秒：

| Workflow | Simulink 現狀 | 方案 2 預估 |
|---|---|---|
| 單次 sim (10 sec sim time) | ~60 s | ~5–8 s |
| 5-seed CI (h=50 verification) | ~5 min | ~30–60 s |
| Parameter sweep (e.g. 20 條件) | ~20 min | ~2 min |
| Debug iteration cycle | 改 → wait → 改 | 改 → 立即看結果 |

---

## 雙軌 Workflow

```
[迭代開發 — 秒級]              [里程碑 — 分鐘級]            [最終 — publish-grade]
run_pure_simulation.m   →    verify_equivalence.m    →    sim('system_model')
test_*_unit.m                pure vs slx 通過 gate           run_simulation.m
                                                              verify_variance.m
```

開發階段：迭代用 pure-MATLAB，秒級回饋
里程碑：每完成一個改動或階段，跑 verify_equivalence 確認沒漂移
論文階段：所有結論用 Simulink 重跑作為最終背書

---

## 新增檔案 (最小集)

```
model/pure_matlab/
  run_pure_simulation.m       # pure-MATLAB driver, 取代 sim('system_model') 的 time-stepping loop
  step_dynamics.m             # 連續動力學一步 (ode4 step=10 µs, 62 substeps per Ts)

test_script/
  verify_equivalence.m        # pure vs slx 一致性 gate (兩階段)
  unit_tests/
    test_ekf.m                # 7-state EKF 單元測試
    test_trajectory.m         # (optional) trajectory 單元測試
```

---

## 不會動到的檔案

- `model/system_model.slx` 本體
- `model/calc_simulation_params.m` (兩路徑共用，driver 各自處理 base workspace 注入)
- `model/controller/` 全部 (motion_control_law_*.m, calc_ctrl_params.m, ...)
- `model/wall_effect/` (calc_correction_functions.m, calc_gamma_inv.m, calc_wall_params.m)
- `model/trajectory/` (trajectory_generator.m, calc_traj_params.m, ...)
- `model/thermal_force/` (calc_thermal_force.m, calc_thermal_params.m)
- `test_script/run_simulation.m`, `test_script/verify_variance.m` 繼續走 Simulink 路徑

---

## verify_equivalence.m 設計

### 階段一：確定性測試 (no noise)

```matlab
% 同 scenario, 關掉 thermal + measurement noise
config.thermal.disable = true
config.noise.disable = true

[t, p_m_slx]  = run_sim_simulink(config)
[t, p_m_pure] = run_pure_simulation(config, scheme=2)

% 比對時間序列
RMS_err = rms(p_m_slx - p_m_pure) / rms(p_m_slx)
assert(RMS_err < 0.001)  % 0.1% gate
```

預期 (方案 2)：RMS_err < 1e-10 (遠優於 0.1% 門檻)

通過 = 物理積分一致，與積分器/步長無關

### 階段二：隨機測試 (full noise, 5 seeds)

```matlab
for seed = 1:5
    out_slx{seed}  = run_sim_simulink(config, seed)
    out_pure{seed} = run_pure_simulation(config, seed, scheme=2)
end

% 比對統計量 (不要求逐 sample 一致, 因為 RNG 引擎不同)
assert(rel_diff(mean_p_m, ...) < 0.01)
assert(rel_diff(std_p_m, ...) < 0.01)

% 也比對研究關鍵 metric
assert(rel_diff(C_dpmr_factor, ...) < 0.01)
assert(rel_diff(a_hat_std, ...) < 0.01)
```

預期 (方案 2)：mean/std 差異 < 0.5%

通過 = 統計行為一致，pure-MATLAB 結論可外推到 Simulink

---

## 戰術決策

### 決策四：初期 controller_type 範圍 = 只支援 type=7 [DECIDED]

- pure-MATLAB driver 初期只支援 `controller_type=7` (7-state EKF, 現役研究焦點)
- type 1/2 維持只用 Simulink 路徑 (`run_simulation.m`, `verify_variance.m` 不變)
- `verify_equivalence.m` 也只測 type=7 場景
- 未來如需擴充 type 1/2，driver dispatch 處新增分支即可，不影響架構

### 決策五：Output schema = Mirror Simulink ToWorkspace [DECIDED]

- pure-MATLAB driver 輸出變數名與 dimension 完全對齊 Simulink ToWorkspace：
  - `p_d_out`, `f_d_out`, `F_th_out`, `p_m_out`
- 既有 `run_simulation.m` post-processing、`analyze_*.m`、`verify_*.m` 等全部 drop-in compatible，不需重寫
- 兩路徑的下游分析鏈共用，最大化既有工具鏈價值

### 決策六：RNG 策略 = 獨立 randn() [DECIDED]

- pure-MATLAB 使用 MATLAB 原生 `randn`，同概念 seed 但與 Simulink RandomStream 獨立
- B 等價標準允許：要求統計一致 (mean/std/spectrum/key metric)，不要求逐 sample 一致
- **代價**：同 seed 下兩路徑的個別 trajectory 不同（但 aggregate 統計一致）
- **好處**：實作簡潔，不需逆向工程 Simulink Random Number block 的內部 RNG 實作 (effort 大且未必可行)

### 隱含 trade-off (給未來的自己看)

如果哪天需要「同 seed 下兩路徑逐 sample 一致以便 debug 偏差來源」，要回頭重做決策六。屆時若 Simulink RandomStream 行為仍無法可靠複現，可改為兩路徑都從 MATLAB 注入隨機序列 (在 Simulink 端用 From Workspace 餵預先生成的 noise 序列)。這是 fallback 路線，非當前範圍。

---

## 下一步

1. 使用者 review 本 doc，補充或修改方向
2. (確認後) 進入 writing-plans skill，產出詳細 implementation plan，內含：
   - 檔案 stub 與函數 signature
   - 各步驟驗收標準
   - 預估工時與里程碑
3. (plan 確認後) 進入 execution
   - Step 1: 建 step_dynamics.m + run_pure_simulation.m skeleton
   - Step 2: 階段一 deterministic equivalence (driver/integration 對齊)
   - Step 3: 階段二 stochastic equivalence (RNG/noise 注入)
   - Step 4: 加 unit test harness
   - Step 5: 雙軌正式投入 dev workflow

---

## 風險與 mitigation

| 風險 | Mitigation |
|---|---|
| Pure-MATLAB 與 Simulink 漂移 (改了 .m 檔但忘了驗) | Equivalence gate 在 milestone 強制跑；每次重大改動前後跑一次 |
| 連續動力學 step=10 µs 在未來物理範圍下不夠 | 步長作為**參數**保留，可隨時切回 step=1 µs (方案 1) |
| 兩路徑分歧難 debug | 階段一 deterministic 測試已關噪聲，問題易定位；單元測試也可獨立驗證每個 .m 函數 |
| Pure-MATLAB 重複實作 Simulink 邏輯 | SSOT 原則：物理/控制邏輯只在 .m 檔，driver 是薄殼 |

---

## 相關 Memory / 文件

- `agent_docs/simulink-architecture.md` — Simulink 區塊圖、執行模式、Solver 設定
- `agent_docs/math-model.md` — 座標系統、Gamma_inv、單位
- `.claude/rules/research-workflow.md` — 討論優先原則、文件記錄要求、驗證標準
- `.claude/rules/agent-coordination.md` — MATLAB 並行策略
