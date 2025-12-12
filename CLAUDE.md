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
├── run_simulation.m               # 主執行腳本（含 GUI 分析）
│
├── model/                         # 核心模型
│   ├── calc_simulation_params.m      # 模擬參數計算
│   ├── system_model.slx              # Simulink 主模型
│   │
│   ├── wall_effect/                  # Wall Effect 模組
│   │   ├── calc_wall_params.m           # 參數計算（離線）
│   │   └── wall_effect_integrated.m     # Wall Effect 計算（即時）
│   │
│   └── controller/                   # 控制器模組
│       └── motion_control_law.m         # 運動控制律（含噪音濾波）
│
├── test_script/                   # 測試腳本
│   └── run_wall_effect_test.m        # Wall Effect 單元測試
│
├── reference/                     # 參考文件
│   └── Drag_MATLAB.pdf               # Wall Effect 數學模型說明
│
├── test_results/                  # 測試結果（不納入 Git）
│
└── .claude/                       # Claude Code 配置
    ├── settings.local.json           # 權限與 hooks（本地）
    ├── commands/                     # Slash commands
    └── sessions/                     # 工作記錄
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

### 系統方程
```
ṗ = Γ⁻¹(p) · F
p = ∫ ṗ dt
```

其中 Γ⁻¹(p) 是位置相關的移動度矩陣（考慮 Wall Effect）。

---

## 控制器模組

### 噪音濾波器
控制器反饋路徑包含可選的單級 IIR 低通濾波器：

```matlab
% 濾波器係數計算
alpha = Ts / (Ts + 1/(2*pi*fc))

% IIR 濾波器差分方程
y[k] = alpha * x[k] + (1 - alpha) * y[k-1]
```

**參數說明**：
| 參數 | 說明 | 預設值 |
|------|------|--------|
| `noise_filter_enable` | 啟用/關閉濾波器 | `false` |
| `noise_filter_cutoff` | 截止頻率 (Hz) | `10` |

### 測量噪音注入
支援在量測位置加入高斯白噪音（用於驗證/測試）：

| 參數 | 說明 | 預設值 |
|------|------|--------|
| `meas_noise_enable` | 啟用/關閉噪音注入 | `false` |
| `meas_noise_std` | 噪音標準差 (um) | `0.01` |

---

## 分析功能

### Deterministic/Random 分離分析
使用 FFT 將量測位置 (p_m) 分離為：
- **Deterministic**：低頻成分（< cutoff_freq），代表軌跡追蹤
- **Random**：高頻成分（> cutoff_freq），代表熱噪音

**用途**：驗證 Einstein 關係式、分析熱力學行為

### 分析 Tabs 總覽
| Tab | 內容 | 數據來源 |
|-----|------|---------|
| 1 | 軌跡總覽 (3D) | p_m, p_d |
| 2-4 | X/Y/Z 軸分析 | p_m, p_d, F |
| 5 | 開迴路熱力分析 | F（無 Wall Effect） |
| 6 | 閉迴路熱力分析 | F（含 Wall Effect） |
| 7 | 軌跡安全檢查 | p_m |
| 8 | Deterministic/Random (3軸) | p_m |
| 9 | Z軸 Deterministic/Random | p_m_z |

---

## 開發指南

### 軌跡類型
| 類型 | 說明 |
|------|------|
| `z_sine` | Z 軸正弦運動 |
| `circle_xy` | XY 平面圓周運動 |
| `helix` | 螺旋軌跡 |

### 建議測試參數
```matlab
frequency = 1;              % 軌跡頻率 (Hz)
noise_filter_enable = false; % 預設關閉
closedloop_cutoff_freq = 10; % 分析截止頻率
```
