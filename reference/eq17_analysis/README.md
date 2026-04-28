# reference/eq17_analysis/

驗證、討論、紀錄區，主軸：

- **控制律**：Meng & Menq 2023, IEEE/ASME T-Mech vol.28 no.1 — Eq.(17) d-step delay-compensated 形式
- **Estimator**：reduced-state EKF（5-state 起點，最簡 random walk 形式）
- **時變處理**：γ(h(t)) 隨壁面距離變動的 wall-effect 軌跡

平行於 qr 分支的 `reference/for_test/`，但本目錄專注於 Eq.17 路線，不繼承 7-state EKF 工作。

## 配套永久文件

- [`agent_docs/eq17-architecture.md`](../../agent_docs/eq17-architecture.md) — System equations、state vector、F_e 矩陣、H 量測選項、notation 對照
- [`agent_docs/eq17-verification.md`](../../agent_docs/eq17-verification.md) — 跨 session 驗證發現累積

## 內容規範

| 檔案類型 | 命名 | 用途 |
|---|---|---|
| Entry-point design | `design.md`, `design_v2.md`... | 各階段設計快照（不刪舊版） |
| Task report | `taskNN_<topic>_report.md` | 單一驗證 task 的結果報告 |
| Figure | `fig_<topic>.png` | Task report 引用的圖（committed） |
| Theory ref | `*.pdf` | 推導引用的 paper 或 textbook 截錄 |

Task report 格式遵循 qr 分支既有 `task1c_report.md` 慣例：
1. 標題 + 分支 / 日期 / 方法 header
2. 一行結果摘要
3. 整合表格（檔案 → 變更）
4. 數據表格（lookup 結果、sweep 結果）
5. 公式 + 邊界條件
6. Artifacts 清單（連結到 `test_script/` 與本目錄的 figures）

## 配套腳本與數據

- 驗證腳本：`test_script/verify_eq17_*.m` / `compute_eq17_*.m` / `analyze_eq17_*.m`
- 數據：`test_results/eq17_analysis/*.mat`（gitignored，由腳本重生）
