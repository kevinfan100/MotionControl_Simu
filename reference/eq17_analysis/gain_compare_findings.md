# gain_compare 研究結果 — 現役資料集

Date: 2026-06-14（整理）
Branch: `test/motion-test`（未 merge 回 feat/eq17-6state）
實驗：6-state 控制器在「控制律餵真值 gain（a=a_true）vs 餵 EKF 估測 gain（a=â）」兩臂下，沿同一激進軌跡比較追蹤表現，拆成 det（系統性）/ ram（隨機）兩分量。
腳本：`compare_gain_6state.m`（跑批）+ `analyze_gain_6state.m`（分析/出圖）。

---

## 0. 現有資料集（磁碟上唯一保留的）

`test_results/gain_compare/{f0.5Hz, f1Hz, f2Hz}/`，每夾 = runs.mat + summary.md + 6 圖 + analysis.mat。

| 頻率 | seeds | 軌跡時間（hold / descend / osc / tail）| a=â 存活 |
|---|---|---|---|
| 0.5 Hz | 100 | 0.25 / 1.5 / 2.0(1 cyc) / 0.25 | 100/100 |
| 1 Hz | 200 | 0.5 / 1.0 / 2.0(2 cyc) / 0.5 | 100/100 |
| 2 Hz | 100 | 0.25 / 1.5 / 2.0(4 cyc) / 0.25 | 100/100 |

共通：gate-free（h̄_safe=1，G3 永不觸發）、h_init 50 / h_bottom 2.7 μm（h̄ 1.2↔3.42）、suppress_xD 兩臂、CRN 配對、熱力+量測噪聲全開。
（1 Hz 時間結構與 0.5/2 略不同——對 osc 窗分析無影響，osc 皆 2.0 s。）

---

## 1. 控制律本身 ≈ 完美（a=a_true 臂）

a=a_true（控制律餵真值 gain）的 det 誤差 e_det = p_d − p_true：z 軸 descent peak **0.1–0.2 nm**、osc A_e < 1 nm（三頻率）。完美 gain + Eq.17 延遲補償下，追蹤本身近乎零系統性誤差 → **控制律不是瓶頸**。

## 2. 估測代價（a=â 臂，系統性 det）

a=â（餵 EKF 估測）的 det 誤差（ensemble-det 參考）：z 軸 descent peak **~43–65 nm** + osc 週期波形。這是 â 在下降末端與近壁谷底跟不上 gain 驟變所致——估測誤差是端到端追蹤誤差的主要系統性來源。

## 3. 熱力理論驗證（V1/thermal closed form）

σ²_th,i(t) = C_δx·4k_BT·a_pd,i(t) + C_n·σ²_n,i 沿期望軌跡。a=a_true 臂：
- x 軸 var meas/theory：**0.99–1.00**（全窗全頻率）
- z 軸 normalized var：**~0.99**（osc）
→ 熱力基線吻合、資料品質乾淨可引用。
a=â 臂 z normalized var 超出 1（估測引入的超額擾動）：**1.05 / 1.04 / 1.44 @ 0.5 / 1 / 2 Hz** → 估測隨機代價隨頻率上升（2 Hz 已明顯，趨向高頻失穩）。

## 4. ⭐ Q55 閉式驗證（A1，現役主線）

Q55 = gain 隨機漲落的「一步增量」變異數，閉式 **Var(δa_ram) = [2/(1+λc)]·(a·K_h/R)²·σ²_δh**。
驗證法：a=a_true 的 100/200-seed `a_true_out` 堆疊 → 扣 ensemble 均值得 per-seed a_ram → 沿時間一步差分 → 跨 seed 取變異數 → 對閉式（gap-safe，§A1）。

實測 increment meas/theory（z 軸）：

| 頻率 | osc | near（近壁）|
|---|---|---|
| 0.5 Hz | 1.011 ± 0.005 | ~1.02 |
| 1 Hz | 1.014 ± 0.003 | ~1.02 |
| 2 Hz | 1.018 ± 0.004 | ~1.02 |

→ **Q55 閉式準到 ~1-2%**，h̄ 低至 1.2、慢軌 quasi-static 最強。對照 LF-drift audit 舊 random-walk 模型近壁高估 LF 變異 237× → Q55 閉式取代 heuristic 的數據基礎完備。

## 5. 穩定性（慢軌 gate-free）

a=â 存活率：0.5 Hz、1 Hz、2 Hz 皆 **100/100**（2 Hz 連 det run 都不發散）。失穩邊界在 2 Hz 以上（見 §7 歷史註記）。a=a_true 三頻率永遠全活。

---

## 6. 認知對齊備忘（毛毛 / a_true / Q55，討論中）

- fig_gain_compare 的 a_true 紅線 = 100-seed **ensemble mean**（平滑、無毛）；毛毛（Q55 物理）在「扣均值後的 per-seed 散佈」，圖上看不到但資料裡完整保留。
- **Level**（Var(a_ram)，離均值多遠）≠ **Increment**（Var(δa_ram)，一步跳多少 = Q55）。A1 驗的是 increment。
- 必用 a=a_true 臂驗（閉迴路嚴格在 λc）；a=â 臂有 self-consistent bias，僅作診斷。
- 待續：用乾淨資料把「均值 vs 散佈」做成並排圖；axm/C_dpmr 驗證腳本 `verify_axm_cdpmr_6state.m` 跑現役 gain_compare 資料。

## 7. 歷史註記（資料已刪，僅存結論）

整理前曾跑 {1, 5, 10} Hz 標準軌（hold 0.5/descend 1.0）100-seed gate-free 批，結論（資料已刪、可由 seed+code 重現）：
- **穩定性斷崖**：1 Hz 全活、5/10 Hz 全滅（|e| 3-4 μm 跟丟、無撞牆）→ G3 gate 低頻是累贅、高頻是剛需。
- **近壁 â 高估**：gated +42% → gate-free +20% → G3 蒙蔽佔近壁高估一半，殘餘為估測器近壁極限。
- 對應 design/plan 文件（gain_compare_design.md / _plan_round2.md）描述的是此標準軌 scenario，屬歷史設計記錄；現役慢軌（0.5/2Hz）為其 ad-hoc 延伸，無獨立 design。
