---
globs: ["**/*.m"]
---

# MATLAB 檔案慣例

## 命名規範
- 模擬腳本 `run_*.m`、分析 `analyze_*.m`、參數計算 `calc_*.m`、
  驗收 `verify_*.m`、產圖 `plot_*.m`、冒煙 `smoke_*.m`

## 草稿與輸出（「寫的進 git，跑出來的不進」）
- 診斷草稿一律開在 `test_script/scratch/`：正常命名（不用 temp_ 前綴）、
  **進版控**、commit 訊息用 `scratch:` 前綴
- `temp_*` 前綴 + gitignore **只用於可重生輸出**（.mat/.png）
- 永久產物（committed tex／驗收腳本／圖）**禁止**依賴 gitignored 或 untracked 檔案

## 圖與第三方檔案（2026-10-04，來源：19 天沒 commit、320 個變更、commit 時漏掉 35 張 tex 引用的圖）
- `reference/**/derivation/figures/` **只放有 tex 引用（`\includegraphics`）或假說帳「圖」欄點名的圖**。
  plot 腳本照舊寫進 figures/（不改 172 支腳本），探索用的圖收工時搬到 `test_results/figs_unused/`
  （gitignored、本機保留，要用時搬回）。
- tex 與它引用的圖**同一個 commit**；`figures/<name>.png` 與 `<name>`（graphicspath）兩種寫法都要對到已追蹤檔。
- 第三方檔案（論文 PDF、網頁截圖、gif）**不進 repo**：`reference/**/data/` 只放自己產的數位化資料（CSV）；
  原檔放 Thesis repo 或本機，路徑記在 memory。`.gitignore` 已擋 `reference/**/data/*.pdf|*.gif`。

## 收工檢查（session 收工、commit 前執行）
- `test_script/build_helpers/check_tree_hygiene.m`：**[HARD]** 已追蹤 tex 只引用已追蹤的圖（不過不准收工）；
  [INFO] 孤兒圖（`'move_orphans', true` 直接搬）、未追蹤腳本分「有人引用 → 跟文件一起 commit」與「沒人引用 → 走下表」、
  data 資料夾的第三方檔、距上次 commit 天數。共用工作樹時用 `'exclude', '<對方的路徑 regexp>'` 只看自己的。

## scratch 分流（session 收工時執行）
| 情況 | 動作 |
|---|---|
| 有永久產物依賴（tex 的圖、commit 的驗收條件） | `git mv` 進 `test_script/integration/` 正名 |
| 綁未解項、還要用 | 留在 scratch/，補 FORK 檔頭 |
| 結案 | `git mv` 進 `reference/eq17_analysis/archive/scratch/<date>-<topic>/` |
| 綁不到任何未解項 | `git rm`（git 歷史留檔，救得回） |

**未追蹤的腳本也走這張表**：結案的先搬進 archive 再 commit，**不要直接刪**（未追蹤檔刪了 git 救不回）；
被其他已追蹤腳本在註解裡引用為數字來源（例：`run_plaw_walls` 的 p = 1.9666 出自 `fit_advisor_law_walls.m`）= 有依賴，留 scratch 並 commit。

## 檔頭 schema（統一格式）
- FORK 檔頭（fork 自產線的診斷版必加）：
  `% FORK OF <path> @ <commit> | PURPOSE: <一句話> | EXPIRES: <結案條件> | 產線改動不會自動跟上`
- STATUS 檔頭（controller 與推導對應腳本）：
  `% STATUS: ACTIVE | SSOT | SUPERSEDED-BY <path> | FALSIFIED -- see memory <name>`

## 編碼規則
- 函數名 snake_case；避免 magic numbers；注意浮點精度
- 所有 Simulink Bus 元素必須為 double 型別
