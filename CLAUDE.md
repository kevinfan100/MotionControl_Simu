# MotionControl_Simu 專案規範

## 專案概述
實現帶有 Wall Effect 的運動控制 MATLAB/Simulink 模擬系統，用於測試位置相關拖曳係數矩陣的即時估計。

**技術棧**: MATLAB R2024b/R2025b + Simulink

---

## 核心工作流程規則

### 必須遵守的流程
1. **開始工作前**：必須執行 `/resume`
2. **實作前討論**：任何代碼變更前必須完整討論計畫並獲得明確許可
3. **功能完成原則**：只有功能完整、測試通過才能 commit
4. **結束工作時**：必須執行 `/save-progress`

### Commit 訊息格式
```
<type>(<scope>): <subject>

Types: feat, fix, refactor, test, docs, chore
Scope: wall-effect, simulation, control, analysis, etc.
```

**範例**：
```
feat(wall-effect): Implement Gamma inverse calculation
test(wall-effect): Add correction function validation
docs(claude): Update project specifications
chore(deps): Update MATLAB toolbox dependencies
```

---

## 專案結構

```
MotionControl_Simu/
├── model/                         # 核心模型
│   ├── wall_effect/               # Wall Effect 模組
│   │   ├── calc_wall_params.m        # 參數計算（離線）
│   │   └── wall_effect_integrated.m  # Wall Effect 計算（即時，含 ZOH + 積分）
│   └── system_model.slx           # Simulink 主模型
│
├── test_script/                   # 測試腳本
│   └── run_wall_effect_test.m     # Wall Effect 單元測試
│
├── reference/                     # 參考文件
│   └── Drag_MATLAB.pdf            # Wall Effect 數學模型說明
│
├── test_results/                  # 測試結果（不納入 Git）
│
└── .claude/                       # Claude Code 配置
    ├── settings.local.json        # 權限與 hooks
    ├── commands/                  # Slash commands
    └── sessions/                  # 工作記錄
```

---

## 檔案命名規範

### MATLAB 腳本
- **臨時測試腳本**: `temp_*.m`（用完必須刪除）
- **測試腳本**: `run_*_test.m`
- **分析腳本**: `analyze_*.m`
- **參數計算**: `calc_*.m`

### Simulink 模型
- 主模型: `system_model.slx`
- 子模型: `subsystem_*.slx`

---

## 重要注意事項

1. **避免使用 Emoji**（除非明確要求）
2. **Simulink 模型使用相對路徑**（確保跨平台兼容）
3. **測試結果不 commit 到 Git**（已在 .gitignore 中排除）
4. **臨時檔案用完即刪**（`temp_*.m`）
5. **程式碼遵循 MATLAB 風格指南**

---

## 數學模型概要

（詳細內容將在後續討論中補充）

### 系統方程
```
ṗ = Γ⁻¹(p) · F
p = ∫ ṗ dt
```

其中 Γ⁻¹(p) 是位置相關的移動度矩陣（考慮 Wall Effect）。

---

## 開發指南

（將在後續討論中補充）
