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

## scratch 分流（session 收工時執行）
| 情況 | 動作 |
|---|---|
| 有永久產物依賴（tex 的圖、commit 的驗收條件） | `git mv` 進 `test_script/integration/` 正名 |
| 綁未解項、還要用 | 留在 scratch/，補 FORK 檔頭 |
| 結案 | `git mv` 進 `reference/eq17_analysis/archive/scratch/<date>-<topic>/` |
| 綁不到任何未解項 | `git rm`（git 歷史留檔，救得回） |

## 檔頭 schema（統一格式）
- FORK 檔頭（fork 自產線的診斷版必加）：
  `% FORK OF <path> @ <commit> | PURPOSE: <一句話> | EXPIRES: <結案條件> | 產線改動不會自動跟上`
- STATUS 檔頭（controller 與推導對應腳本）：
  `% STATUS: ACTIVE | SSOT | SUPERSEDED-BY <path> | FALSIFIED -- see memory <name>`

## 編碼規則
- 函數名 snake_case；避免 magic numbers；注意浮點精度
- 所有 Simulink Bus 元素必須為 double 型別
