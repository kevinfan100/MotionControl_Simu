# MotionControl_Simu 專案規範

帶有 Wall Effect 的運動控制 MATLAB/Simulink 模擬系統，用於測試位置相關拖曳係數矩陣的即時估計。

**技術棧**: MATLAB R2025b + Simulink

---

## 核心規則

1. **實作前討論**：任何代碼變更前必須完整討論計畫並獲得明確許可
2. **功能完成原則**：只有功能完整、測試通過才能 commit

### Commit 訊息格式
```
<type>(<scope>): <subject>
Types: feat, fix, refactor, test, docs, chore
Scope: wall-effect, simulation, control, analysis, etc.
```

---

## 禁止事項

- 勿使用 Emoji（除非明確要求）
- Simulink 模型必須使用相對路徑
- 測試結果不 commit 到 Git（已在 .gitignore 中排除）
- 臨時檔案 `temp_*.m` 用完即刪

---

## Tool Usage

### matlab (MCP)
- 驗證參數計算與 Bus Object 定義
- 靜態分析 `.m` 檔案
- 執行模擬腳本、跑測試

### perplexity (MCP)
- 查物理公式（Wall Effect、Stokes drag）
- 搜尋論文與 MATLAB/Simulink 文件

---

## Workflow

- 大功能實作前使用 brainstorming + writing-plans skills
- 主模型: `model/system_model.slx`
- 主模擬腳本: `test_script/run_simulation.m`

---

## 雙 controller 架構

本專案維護兩個 paper-form 不同的 controller, 由 `params.ctrl.controller_type` 切換:

| controller_type | 路徑 | f_d 形式 | 主要文件 |
|---|---|---|---|
| 6 | `model/controller/motion_control_law_eq6.m` | Paper 2025 Eq.6: `f_d = (1/a) (del_pd + (1-lc) del_p3 - d)` | `reference/eq6_analysis/` |
| 17 | `model/controller/motion_control_law_eq17.m` (wrapper) + `_eq17_core.m` | Paper 2023 Eq.17 d-step delay-compensated: 含 Σ f_d[k-i] 在 1/a 括號外 + x_D 加項補償 | `reference/eq17_analysis/` |
| 23 | `model/controller/motion_control_law_23state.m` | Legacy 23-state EKF (paper full state) | — |

兩 controller 共用同一個 7-state per-axis EKF 結構 `[del_p1, del_p2, del_p3, d, del_d, a, del_a]`; 主要差異在 F_e Row 3 + control law f_d 形式。共通推導文件在 `reference/shared/writeup_architecture.tex`, controller-specific 章節 (`control_law_eq6.tex`, `control_law_eq17.tex`) 為 TODO 由 writeup 主檔抽出。

V5/V7 cross-branch 研究 (`reference/eq6_analysis/q66_value_dominance.md`) 發現 Q(6,6) 的「數值」(物理推導 1.89e-10 vs empirical 1e-8) 比 controller 架構更主導 a_hat std。on-the-fly 物理 Q/R 對兩 controller 都適用。

---

## 專案結構

主要目錄 `ls` 可見：`model/`（含 `controller/`, `wall_effect/`, `trajectory/`,
`thermal_force/`, `config/`）、`test_script/`、`agent_docs/`、`reference/`、`test_results/`。

以下幾點 `ls` 看不出來：

- `model/dual_track/` — 純 MATLAB driver，跨 controller，與 Simulink 並行（見 Detailed Docs）
- `model/diag/` — closed-loop variance oracles (per-controller)
- `reference/shared/writeup_architecture.tex` — 跨 controller 共通推導 SSOT
- `reference/branch-stories/` — sigma.md + eq17.md 歷史故事索引

歷史分支以 archive tag 凍結:
- `archive/sigma-pre-cleanup` → 整理前 sigma HEAD (origin remote)
- `archive/eq17-pre-cleanup` → 整理前 eq17 HEAD (origin remote)
- 對應 bundle 在 `D:\archives\MotionControl_Simu\` (離線備份)

---

## Detailed Docs

`@` 前綴 = 每個 session 開場自動載入；無前綴 = 指標，需要時才讀。
歷史紀錄與 legacy 分支文件一律不加 `@`（省 context，內容仍完整保留）。

### 跨 controller 共用
- @agent_docs/shared/simulink-architecture.md — Simulink 方塊圖、Block 模式、Solver、ToWorkspace
- @agent_docs/shared/math-model.md — 座標系統、單位、系統方程、Gamma_inv
- @agent_docs/shared/analysis-guide.md — GUI 分析 Tabs、建議測試參數
- agent_docs/shared/dual-track-simulation-design.md — pure-MATLAB vs Simulink 雙 track 設計決策 (已實作)

### eq17 controller
- @agent_docs/eq17/eq17-architecture.md — eq17 7-state EKF + Eq.17 控制律設計
- agent_docs/eq17/eq17-verification.md — eq17 task 01-04 驗證編年史 (2026-04, 歷史紀錄)

### eq6 (Paper 2025 Eq.6) 與 legacy 23-state
- agent_docs/eq6_or_23state/ekf-matrix-guide.md — 23-state EKF 矩陣完整文件
- agent_docs/eq6_or_23state/ekf-qr-analysis.md — 23-state Q/R 分析 (pointer to writeup)
- agent_docs/eq6_or_23state/kf-observer-analysis.md — KF observer 分析
- agent_docs/eq6_or_23state/literature-review.md — 文獻 review
- agent_docs/eq6_or_23state/verification-notes.md — 驗證筆記
