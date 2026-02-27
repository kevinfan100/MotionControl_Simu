# MotionControl_Simu 專案規範

## 專案概述
實現帶有 Wall Effect 的運動控制 MATLAB/Simulink 模擬系統，用於測試位置相關拖曳係數矩陣的即時估計。

**技術棧**: MATLAB R2025b + Simulink

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

---

## 專案結構

```
MotionControl_Simu/
├── model/                            # 核心模型
│   ├── calc_simulation_params.m         # 模擬參數計算 + Bus Object 定義
│   ├── system_model.slx                 # Simulink 主模型
│   │
│   ├── wall_effect/                     # Wall Effect 模組
│   │   ├── calc_correction_functions.m     # c_para, c_perp 修正係數
│   │   └── calc_gamma_inv.m                # Gamma_inv 逆拖曳矩陣
│   │
│   ├── thermal_force/                   # 熱力（布朗運動）模組
│   │   └── calc_thermal_force.m            # 位置相關熱力生成
│   │
│   ├── trajectory/                      # 軌跡模組
│   │   ├── trajectory_generator.m          # 軌跡產生器（z_sine / xy_circle）
│   │   ├── calc_initial_position.m         # 初始位置計算
│   │   └── check_trajectory_safety.m       # 軌跡安全檢查
│   │
│   └── controller/                      # 控制器模組
│       └── motion_control_law.m            # 運動控制律（含噪音濾波）
│
├── test_script/                      # 模擬腳本
│   └── run_simulation.m                 # 主模擬腳本（含 GUI 分析）
│
├── reference/                        # 參考文件
│   └── Drag_MATLAB.pdf                  # Wall Effect 數學模型說明
│
├── test_results/                     # 模擬結果（不納入 Git）
│
└── .claude/                          # Claude Code 配置
    ├── settings.local.json              # 權限與 hooks（本地）
    └── commands/                        # Slash commands
```

---

## Simulink 模型架構

### 系統方塊圖
```
Trajectory_Generator (discrete, 1/1606 Hz)  --> p_d
Controller (discrete, 1/1606 Hz)            --> f_d
Thermal_Force (discrete, 1/1606 Hz)         --> F_th

f_d + F_th = F_total --> ZOH (1/1606) --> [ Gamma_inv(p) ] --> p_dot --> [ 1/s ] --> p
                                           ^    (continuous)                 |
                                           |________________________________|
```

### Block 執行模式
| Block | ChartUpdate | SampleTime | 說明 |
|-------|------------|------------|------|
| Trajectory_Generator | DISCRETE | 1/1606 | 離散軌跡產生 |
| Controller | DISCRETE | 1/1606 | 離散控制律 |
| Thermal_Force | DISCRETE | 1/1606 | 離散隨機力產生 |
| Drag coefficient matrix | CONTINUOUS | - | 連續動力學（每 solver step 更新） |
| Integrator | Continuous (ODE) | - | 連續積分 |

### Solver 設定
- Solver: Fixed-step, step size 1e-5
- 離散取樣率: 1606 Hz (Ts = 1/1606 sec)

---

## 檔案命名規範

### MATLAB 腳本
- **臨時測試腳本**: `temp_*.m`（用完必須刪除）
- **模擬腳本**: `run_*.m`
- **分析腳本**: `analyze_*.m`
- **參數計算**: `calc_*.m`

### Simulink 模型
- 主模型: `system_model.slx`

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
p_dot = Gamma_inv(p) * F_total
p = integral(p_dot) dt
```

其中 Gamma_inv(p) 是位置相關的移動度矩陣（考慮 Wall Effect）。

### Gamma_inv 計算
```
Gamma_inv(p) = 1/(gamma_N * c_para(h_bar)) * { I - [(c_perp(h_bar) - c_para(h_bar)) / c_perp(h_bar)] * W }
```
- h_bar = h/R（無因次化壁面距離）
- W = w_hat * w_hat'（壁面法線投影矩陣）
- c_para, c_perp：平行/垂直修正係數（h_bar 的多項式函數）

---

## 控制器模組

### 控制律
```
f_d[k] = (gamma/Ts) * { p_d[k] - lambda_c * p_d[k-1] - (1 - lambda_c) * p_feedback[k] }
```

### 噪音濾波器
控制器反饋路徑包含可選的單級 IIR 低通濾波器：

```matlab
alpha = Ts / (Ts + 1/(2*pi*fc))
y[k] = alpha * x[k] + (1 - alpha) * y[k-1]
```

| 參數 | 說明 | 預設值 |
|------|------|--------|
| `noise_filter_enable` | 啟用/關閉濾波器 | `false` |
| `noise_filter_cutoff` | 截止頻率 (Hz) | `10` |

### 測量噪音注入
| 參數 | 說明 | 預設值 |
|------|------|--------|
| `meas_noise_enable` | 啟用/關閉噪音注入 | `false` |
| `meas_noise_std` | 噪音標準差 (um) | `[0.00062; 0.000057; 0.00331]` |

---

## 分析功能

### GUI 分析 Tabs
| Tab | 內容 | 數據來源 |
|-----|------|---------|
| 1 | 軌跡總覽 (3D) | p_m, p_d |
| 2-4 | X/Y/Z 軸分析 | p_m, p_d, F |
| 5 | 位置 vs 時間 | p_m |
| 6 | 控制力 vs 時間 | f_d |
| 7 | 開迴路熱力分析（Deterministic/Random 分離） | F |
| 8 | 閉迴路熱力分析 | F |

---

## 開發指南

### 軌跡類型
| 類型 | 說明 |
|------|------|
| `z_sine` | Z 軸正弦運動 |
| `xy_circle` | XY 平面圓周運動 |

### 建議測試參數
```matlab
frequency = 1;              % 軌跡頻率 (Hz)
noise_filter_enable = false; % 預設關閉
closedloop_cutoff_freq = 10; % 分析截止頻率
```
