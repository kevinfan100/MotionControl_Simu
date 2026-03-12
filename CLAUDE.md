# MotionControl_Simu 專案規範

## 專案概述
實現帶有 Wall Effect 的運動控制 MATLAB/Simulink 模擬系統，用於測試位置相關拖曳係數矩陣的即時估計。

**技術棧**: MATLAB R2025b + Simulink

---

## 核心工作流程規則

### 必須遵守的流程
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

## 專案結構

```
MotionControl_Simu/
├── model/                            # 核心模型
│   ├── calc_simulation_params.m         # 模擬參數計算 + Bus Object 定義
│   ├── system_model.slx                 # Simulink 主模型
│   │
│   ├── config/                        # 參數配置
│   │   ├── physical_constants.m          # 物理常數 (R, gamma_N, Ts, k_B, T)
│   │   └── user_config.m                # 使用者可調參數預設值
│   │
│   ├── wall_effect/                   # Wall Effect 模組
│   │   ├── calc_wall_params.m            # 壁面幾何參數 (角度 deg→rad 轉換)
│   │   ├── calc_correction_functions.m   # c_para, c_perp 修正係數
│   │   └── calc_gamma_inv.m              # Gamma_inv 逆拖曳矩陣
│   │
│   ├── thermal_force/                 # 熱力（布朗運動）模組
│   │   └── calc_thermal_force.m          # 位置相關熱力生成
│   │
│   ├── trajectory/                    # 軌跡模組
│   │   ├── trajectory_generator.m        # 沿壁面法向 w_hat 的正弦軌跡
│   │   ├── calc_traj_params.m            # 軌跡參數打包
│   │   ├── calc_initial_position.m       # 初始位置計算
│   │   └── check_trajectory_safety.m     # 軌跡安全檢查
│   │
│   └── controller/                    # 控制器模組
│       └── motion_control_law.m          # 運動控制律（含噪音濾波）
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
params_const (Bus:ParamsBus, Ts=inf) → [Goto params] → From params_Traj/Ctrl/Thermal/Drag
Integrator output                    → [Goto p_m]    → From p_m_Ctrl/Thermal/Drag

Clock ────────────┐
From params_Traj ─┤→ Trajectory_Generator (DISCRETE 1/1600) → p_d ─┬→ p_d_out
                                                                    │
p_d ──────────────┐                                                 │
From p_m_Ctrl ────┤→ Controller (DISCRETE 1/1600) → f_d ───┬→ f_d_out
From params_Ctrl ─┘                                        │
                                                           │
From p_m_Thermal ────┐                                     │
From params_Thermal ─┤→ Thermal_Force (DISCRETE 1/1600) → F_th ─┬→ F_th_out
                                                                 │
                          f_d + F_th → [Sum_Forces: ++] → [ZOH 1/1600]
                                                              │
F_total (from ZOH) ──┐                                       │
From p_m_Drag ────────┤→ Drag_coeff_matrix (CONTINUOUS) → p_dot
From params_Drag ─────┘                                    │
                                                           ▼
                                    p_dot → [Integrator IC=p0] → p_m ──┬→ [Goto p_m]
                                                                       └→ p_m_out
```

### Block 執行模式
| Block | ChartUpdate | SampleTime | 說明 |
|-------|------------|------------|------|
| Trajectory_Generator | DISCRETE | 1/1600 | 離散軌跡產生 |
| Controller | DISCRETE | 1/1600 | 離散控制律 |
| Thermal_Force | DISCRETE | 1/1600 | 離散隨機力產生 |
| Drag coefficient matrix | CONTINUOUS | 0 | 連續動力學（每 solver step 更新） |
| Integrator | Continuous (ODE) | - | 連續積分，IC=p0 (workspace) |

### Solver 設定
- Solver: Fixed-step ode4 (Runge-Kutta 4th order), step size = 1e-6
- 離散取樣率: 1600 Hz (Ts = 1/1600 sec)

### 資料記錄（ToWorkspace）
| Block 名稱 | VariableName | SampleTime | 說明 |
|-----------|-------------|------------|------|
| p_d_out | p_d_out | 1/1600 | 期望軌跡（離散） |
| f_d_out | f_d_out | 1/1600 | 控制力（離散） |
| F_th_out | F_th_out | 1/1600 | 熱力（離散） |
| p_m_out | p_m_out | -1（繼承） | 測量位置（連續，每 solver step 記錄） |

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

## 座標系統與單位

### 壁面座標
- 壁面方向由 theta (方位角) 和 phi (仰角) 定義，**使用者設定單位為 deg**
- 內部自動轉 rad 計算
- w_hat = 壁面法向量，u_hat/v_hat = 壁面切向量
- 預設 theta=0, phi=0 時：w_hat = [0;0;1]（世界 Z 軸）

### 距離單位
- 使用者設定（h_init, amplitude, h_min）：**um（微米）**
- 模型內部物理計算：**h_bar = h/R（無因次）**
- 繪圖輸出：**世界座標 um**

### 軌跡定義
- 軌跡沿壁面法向 w_hat 做正弦振盪
- 設定的 amplitude 直接就是 h 方向的振幅（不受壁面角度影響）
- p_d(t) = p0 + amplitude * sin(2*pi*f*t) * w_hat（世界座標輸出）

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

### 初始位置
```
p0 = (pz + h_init) * w_hat
```
- pz：壁面沿 w_hat 方向的位移 [um]
- h_init：初始距離壁面 [um]

---

## 控制器模組

### 控制律
```
f_d[k] = (gamma/Ts) * { p_d[k] - lambda_c * p_d[k-1] - (1 - lambda_c) * p_m[k] }
```

### 測量噪音注入
| 參數 | 說明 | 預設值 |
|------|------|--------|
| `meas_noise_enable` | 啟用/關閉噪音注入 | `false` |
| `meas_noise_std` | 噪音標準差 (um) | `[0.01; 0.01; 0.01]` |

---

## 分析功能

### GUI 分析 Tabs
| Tab | 內容 | 數據來源 |
|-----|------|---------|
| 1 | 軌跡總覽 (3D) | p_m, p_d |
| 2-4 | X/Y/Z 軸分析 | p_m, p_d, F |
| 5 | 位置 vs 時間 | p_m |
| 6 | 控制力 vs 時間 | f_d |
| 7+ | 開迴路熱力分析 (條件觸發) | p_m |

---

## 開發指南

### 建議測試參數
```matlab
frequency = 1;              % 軌跡頻率 (Hz)
amplitude = 2.5;            % h 方向振幅 (um)
openloop_cutoff_freq = 5;   % 開迴路分析截止頻率
```
