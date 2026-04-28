# MotionControl_Simu 專案規範

帶有 Wall Effect 的運動控制 MATLAB/Simulink 模擬系統，用於測試位置相關拖曳係數矩陣的即時估計。

**技術棧**: MATLAB R2025b + Simulink

---

## 核心規則

1. **實作前討論**：任何代碼變更前必須完整討論計畫並獲得明確許可
2. **功能完成原則**：只有功能完整、測試通過才能 commit
3. **結束工作時**：必須執行 `/save-progress`

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
- 公式驗證流程：討論(interview) → notation 定義 → 實作 → Lyapunov 驗證 → Simulink 驗證 → 出圖
- 主模型: `model/system_model.slx`
- 主模擬腳本: `test_script/run_simulation.m`
- 驗證腳本: `test_script/verify_variance.m`

---

## 控制器類型

| controller_type | 名稱 | 說明 |
|---|---|---|
| 1 | Eq.17 delay compensation | 簡單延遲補償控制律，K=2 |
| 2 | 3-state observer (le=0) | 三狀態觀測器（deadbeat），K=3 |
| 7 | 7-state EKF | 擴展卡爾曼濾波器，自適應增益，非線性 |

### C_dpmr 公式修正

IIR HP filter `H_HP(z) = (1-a_pd)*(1-z^-1)/(1-(1-a_pd)*z^-1)` 的 `(1-a_pd)` 前綴
在原始 C_dpmr 公式推導中被遺漏。修正後的公式需乘以 `(1-a_pd)^2` 前綴：

- Controller 1: `C_dpmr = (1-av)^2 * [2*(1-av)*(1-lc)/(1-(1-av)*lc) + (2/(2-av))/((1+lc)*(1-(1-av)*lc))]`
- Controller 2: 同結構但 `K_eff(av) = 2 + 2*(1-av)^2/(2-av)` 替代 K=2
- Controller 3: 無解析公式（EKF 為非線性系統）

詳見 `reference/qr_analysis/verification_report.md`。

## 專案結構

```
MotionControl_Simu/
├── model/                            # 核心模型
│   ├── calc_simulation_params.m         # 模擬參數計算 + Bus Object 定義
│   ├── system_model.slx                 # Simulink 主模型
│   ├── config/                        # 參數配置
│   ├── wall_effect/                   # Wall Effect 模組
│   ├── thermal_force/                 # 熱力（布朗運動）模組
│   ├── trajectory/                    # 軌跡模組
│   └── controller/                    # 控制器模組
├── test_script/                      # 模擬腳本
│   └── verify_variance.m               # 變異數驗證：C_dpm/C_dpmr vs 理論公式（3 種控制器）
├── agent_docs/                       # 詳細技術文件
├── reference/                        # 參考文件
│   └── qr_analysis/                      # 驗證用文件、報告、圖表（committed）
├── test_results/                     # 模擬結果（不納入 Git）
│   └── verify/                        # 驗證數據 .mat（可用 verify_variance.m 重生）
└── .claude/                          # Claude Code 配置
    ├── rules/                           # 自動載入規則
    └── commands/                        # Slash commands
```

---

## Detailed Docs

- @agent_docs/simulink-architecture.md — Simulink 方塊圖、Block 模式、Solver、ToWorkspace
- @agent_docs/math-model.md — 座標系統、單位、系統方程、Gamma_inv
- @agent_docs/analysis-guide.md — GUI 分析 Tabs、建議測試參數
