# MotionControl_Simu 專案規範

帶有 Wall Effect 的運動控制 MATLAB/Simulink 模擬系統：在 c(h̄) 未知的前提下即時估測
位置相關 motion gain。**技術棧**: MATLAB R2025b + Simulink。

---

## 研究主線：估測函數（2026-07 轉折）

目標：c(h̄) 未知前提下估準三軸 motion gain a（z 軸優先）。
方法：**不估增益值，估增益函數的常數** —— 低參數 gain law，參數 prior 由兩個已發表
漸近極限（Brenner 近壁 + method-of-reflections 遠場）推導釘死，零自由數字。

約束（07-13 章程之 07-28 修訂版）：
1. 零 tuning —— 每個數字有推導或錨
2. 推導完整 —— 無 trade-off（詳 `.claude/rules/derivation-workflow.md`）
3. run-time c-free —— Q/R/種子不讀 c(h̄)
4. 允許低參數函數形，合法性由形狀驗收判準擔保：
   `sup|θ_eff − θ₀| ≲ √P[0] ⟺ Q_θ = 0 誠實`
   （工具 `test_script/integration/verify_shape_exponent_bound.m`；
   prior 設計方法 `reference/shared/param_prior_rules.md`）

兩個**對等**形式（檔案級對應見 @agent_docs/eq17/mainline-gainlaw.md；
對照與定案門檻見 `reference/eq17_analysis/shape_ledger.md`）：
- **powerlaw** c = 1 + K/(h̄−1)^p —— ⊥ 最準；∥ 結構性失效
- **expgain** a_h = a_o[1 − h̄^(−b)] —— 三軸可用；7a 代數式已修缺陷 1
  （⚠ tex 正文落後 code：仍 7b 微分式，code 以 `_5state_expgain_alg.m` 為準）

未解主項 = **缺陷 2**：運動時變異數讀數 +4~5%（兩形式共用讀數鏈）
→ 重推 `reference/eq17_analysis/derivation/Cdpmr_Cn_derivation.tex`。

---

## 三條執行路徑

| 路徑 | 入口 | 選擇器 |
|---|---|---|
| 估測函數（主線） | `test_script/integration/run_5state_powerlaw.m`／`run_5state_expgain.m` | 直接呼叫，不經 dispatcher |
| pure-MATLAB 基準 | `model/dual_track/run_pure_simulation.m` | `config.eq17_variant` ∈ {4state, 5state, 6state} |
| Simulink | `test_script/run_simulation.m` → `model/system_model.slx` | `config.controller_type` ∈ {6, 17, 23}；6 已驗證健康，17 編譯可跑但數值未驗（⚠ 發散，eq17 走 pure track），23 編譯 OK |

---

## 核心規則

1. **實作前討論**：任何代碼變更前必須完整討論計畫並獲得明確許可
   （大功能先用 brainstorming + writing-plans skills）
2. **功能完成原則**：只有功能完整、測試通過才能 commit
3. **歷史下沉**：每一層目錄的 `archive/` = 歷史，其他 = 現役；結案的東西搬進去
4. **草稿進版控**：診斷腳本開在 `test_script/scratch/`（tracked，commit 用 `scratch:` 前綴）；
   `temp_*` + gitignore 只給 .mat/.png 可重生輸出（細則 `.claude/rules/matlab-conventions.md`）
5. **永久產物**（committed tex／驗收腳本／圖）禁止依賴 gitignored 或 untracked 檔案

### Commit 訊息格式
```
<type>(<scope>): <subject>
Types: feat, fix, refactor, test, docs, chore, scratch
Scope: wall-effect, simulation, control, analysis, etc.
```

## 禁止事項

- 勿使用 Emoji（除非明確要求）
- Simulink 模型必須使用相對路徑
- 測試結果不 commit 到 Git（已在 .gitignore 中排除）

## Tool Usage

- **matlab (MCP)**：驗證參數計算與 Bus Object、靜態分析 .m、執行模擬與測試
- **perplexity (MCP)**：查物理公式（Wall Effect、Stokes drag）、搜尋論文與文件

---

## 專案結構

主要目錄 `ls` 可見：`model/`（含 `controller/`, `wall_effect/`, `trajectory/`,
`thermal_force/`, `config/`, `dual_track/`, `diag/`）、`test_script/`（含 `integration/`,
`scratch/`, `unit_tests/`, `build_helpers/`, `learn_variance/`）、`agent_docs/`、
`reference/`、`test_results/`。

以下幾點 `ls` 看不出來：

- `reference/shared/writeup_architecture.tex` — 跨 controller 共通推導
- `reference/eq17_analysis/shape_ledger.md` — 兩形式統一指標對照 + 定案門檻
- `reference/eq17_analysis/archive/MOVED.md` — 2026-07-28 大整理的舊→新路徑對照表
- `reference/eq6_analysis/q66_value_dominance.md` — 歷史 cross-branch 發現（Q(6,6) 數值主導性）
- 歷史分支 archive tag：`archive/sigma-pre-cleanup`、`archive/eq17-pre-cleanup`
  （bundle 離線備份在 `D:\archives\MotionControl_Simu\`）

---

## Detailed Docs

`@` 前綴 = 每個 session 開場自動載入；無前綴 = 指標，需要時才讀。

### 主線（eager）
- @agent_docs/eq17/mainline-gainlaw.md — 兩形式對照、入口、驗收工具、缺陷狀態
- @agent_docs/shared/math-model.md — 座標系統、單位、系統方程、Gamma_inv

### 需要時才讀
- agent_docs/shared/simulink-architecture.md — Simulink 方塊圖、Block 模式、Solver、ToWorkspace
- agent_docs/shared/analysis-guide.md — GUI 分析 Tabs、建議測試參數
- agent_docs/shared/dual-track-simulation-design.md — 雙 track 設計決策（已實作）
- agent_docs/eq17/eq17-architecture.md — 2026-04 eq17 7-state 舊主線架構（歷史）
- agent_docs/eq17/eq17-verification.md — eq17 task 01-04 驗證編年史（2026-04 歷史）
- 更早的 eq6/23-state 文件 → `agent_docs/archive/`
