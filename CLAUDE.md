# MotionControl_Simu 專案規範

## 專案概述
實現帶有 Wall Effect 的磁珠運動控制 MATLAB/Simulink 模擬系統，用於測試位置相關拖曳係數矩陣的即時估計。

**技術棧**: MATLAB R2024b/R2025b + Simulink

**實作狀態**: Phase 1 完成 (2024-12)

### 簡化假設
- 內迴路完美：f_d = f_m（磁力控制無誤差）
- 量測完美：p_m = p（視覺追蹤無誤差、無延遲）

### 混合架構
- **離散部分 @ 1606 Hz**：軌跡產生、控制器、Thermal Force
- **連續部分**：Particle Dynamics（ODE solver）
- 離散訊號經 ZOH 送入連續系統

### 快速參考：確認的參數值

| 參數 | 值 | 單位 | 說明 |
|------|-----|------|------|
| R | **2.25** | μm | 粒子半徑 |
| γ_N | 0.0425 | pN·sec/μm | Stokes drag |
| Δt | 1/1606 | sec | 取樣週期 |
| k_B | 1.3806503×10⁻⁵ | pN·μm/K | 波茲曼常數 |
| T | 310.15 | K | 溫度 (37°C) |
| h̄_min | 1.5 | - | 最小安全距離 |
| h_margin | 5 | μm | 額外安全餘量 |

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
Scope: wall-effect, thermal-force, trajectory, controller, simulation, etc.
```

---

## 專案結構

```
MotionControl_Simu/
├── CLAUDE.md                         # 專案規範
├── calc_simulation_params.m          # 參數計算
├── create_simulation_buses.m         # Bus Object 建立（Simulink 用）
├── convert_params_for_simulink.m     # 參數轉換（字串→數值編碼）
├── build_system_model.m              # Simulink 模型建構腳本
├── run_simulation.m                  # 主執行腳本（純 MATLAB）
├── run_simulation_slx.m              # Simulink 執行腳本
│
├── model/
│   ├── wall_effect/
│   │   ├── calc_correction_functions.m  # c_∥, c_⊥ 計算
│   │   └── calc_gamma_inv.m             # Γ⁻¹ 矩陣計算
│   │
│   ├── thermal_force/
│   │   └── calc_thermal_force.m         # F_th 生成
│   │
│   ├── trajectory/
│   │   ├── calc_initial_position.m      # 起始位置計算
│   │   ├── trajectory_generator.m       # 即時軌跡計算
│   │   └── check_trajectory_safety.m    # 安全檢查
│   │
│   ├── controller/
│   │   └── motion_control_law.m         # 控制律
│   │
│   └── system_model.slx                 # Simulink 主模型
│
├── test_script/
│   ├── run_wall_effect_test.m
│   ├── run_thermal_force_test.m
│   └── run_trajectory_test.m
│
├── test_results/                        # 測試結果（不納入 Git）
│   ├── wall_effect/
│   ├── thermal_force/
│   ├── trajectory/
│   └── simulation/
│
├── reference/
│   ├── Drag_MATLAB.pdf
│   ├── total_block_diagram.png
│   └── thermal_force.png
│
└── .claude/
    ├── settings.local.json
    ├── commands/
    ├── plans/
    └── sessions/
```

---

## 單位系統

| 物理量 | 單位 | 備註 |
|--------|------|------|
| 長度 | μm | |
| 力 | pN | |
| 時間 | sec | |
| 拖曳係數 | pN·sec/μm | |
| k_B | 1.3806503×10⁻⁵ pN·μm/K | SI 單位 ×10¹⁸ |
| T | 310.15 K | 37°C |
| γ_N | 0.0425 pN·sec/μm | Stokes drag |
| Δt | 1/1606 sec | 取樣週期 |

---

## 數學模型概要

### 系統方程
```
ṗ = Γ⁻¹(p) · (f_d + F_th)
p = ∫ ṗ dt
```

### Wall Effect

#### 正交基底向量
```matlab
% 法向量（垂直於牆面）
w_hat = [cos(theta)*sin(phi); sin(theta)*sin(phi); cos(phi)];

% 平行於牆面的兩個方向
u_hat = [-cos(theta)*cos(phi); -sin(theta)*cos(phi); sin(phi)];
v_hat = [sin(theta); -cos(theta); 0];
```

#### 修正函數
```
c_∥(h̄) = (1 - 9/16·(1/h̄) + 1/8·(1/h̄)³ - 45/256·(1/h̄)⁴ - 1/16·(1/h̄)⁵)⁻¹
c_⊥(h̄) = (1 - 9/8·(1/h̄) + 1/2·(1/h̄)³ - 57/100·(1/h̄)⁴ + 1/5·(1/h̄)⁵ + 7/200·(1/h̄)¹¹ - 1/25·(1/h̄)¹²)⁻¹
```

#### 移動度矩陣
```matlab
h = dot(p, w_hat) - pz;  % 粒子到平面距離
h_bar = h / R;           % 正規化距離
W = w_hat * w_hat';      % 投影矩陣
coeff = (c_perp - c_para) / c_perp;
Gamma_inv = (1 / (gamma_N * c_para)) * (eye(3) - coeff * W);
```

### Thermal Force
```matlab
C = c_para * (u_hat + v_hat) + c_perp * w_hat;
Variance = (4 * k_B * T * gamma_N / Ts) * (C.^2);
F_th = sqrt(Variance) .* randn(3,1);  % 離散 @ 1606 Hz
```

### 控制器（Position-Dependent Discrete-Time Motion Control）
```matlab
f_d[k] = (γ / Δt) · {p_d[k] - λ_c · p_d[k-1] - (1 - λ_c) · p[k]}
```

| 參數 | 說明 | 建議值 |
|------|------|--------|
| γ | 拖曳係數 | γ_N = 0.0425 |
| λ_c | 閉迴路極點 | 0.5 ~ 0.9 |

---

## 參數結構

```
params
├── common                          ← 所有模組共用
│   ├── R         = 2.25            % 粒子半徑 [μm]
│   ├── gamma_N   = 0.0425          % Stokes drag [pN·sec/μm]
│   ├── Ts        = 1/1606          % 取樣週期 [sec]
│   └── T_sim     = 5               % 模擬時間 [sec]
│
├── wall                            ← Wall Effect 模組
│   ├── theta, phi, pz              % 高階輸入
│   ├── h_bar_min = 1.5             % 安全距離
│   └── w_hat, u_hat, v_hat         % 計算得出
│
├── traj                            ← 軌跡產生器
│   ├── type                        % 'z_move' 或 'xy_circle'
│   ├── h_margin  = 5               % 額外安全餘量 [μm]
│   ├── delta_z, direction, speed   % z_move 參數
│   └── radius, n_circles, period   % xy_circle 參數
│
├── ctrl                            ← 控制器
│   ├── enable                      % 啟用/停用（閉迴路/開迴路）
│   ├── lambda_c  = 0.7             % 閉迴路極點
│   └── gamma     = gamma_N         % 拖曳係數
│
└── thermal                         ← Thermal Force
    ├── enable                      % 啟用/停用
    ├── k_B, T, Ts                  % 常數
    └── variance_coeff              % 預計算: 4·k_B·T·γ_N/Δt
```

### Bus Object 結構（Simulink 用）

```
ParamsBus
├── common (CommonBus)
├── wall (WallBus)
├── traj (TrajBus)      ← type/direction 為數值編碼
├── ctrl (CtrlBus)
└── thermal (ThermalBus)
```

字串→數值編碼（Simulink 限制）：
- `type`: `'z_move'` → `0`, `'xy_circle'` → `1`
- `direction`: `'away'` → `0`, `'toward'` → `1`

---

## 軌跡類型

| 類型 | 參數 | 說明 |
|------|------|------|
| z_move | delta_z, direction, speed | 沿法向量 ŵ 方向移動 |
| xy_circle | radius, n_circles, period | 在 û-v̂ 平面內繞圓 |

- `direction = 'away'`：遠離牆面（+ŵ）
- `direction = 'toward'`：靠近牆面（-ŵ）

---

## Simulink 設定

| 參數 | 建議值 |
|------|--------|
| Solver | ode45 (variable-step) |
| Max Step Size | 1e-4 sec |
| Relative Tolerance | 1e-6 |
| Absolute Tolerance | 1e-9 |

---

## 測試輸出結構

```
test_results/
├── wall_effect/
│   └── test_YYYYMMDD_HHMMSS/
│       ├── correction_functions.png
│       ├── gamma_inv_eigenvalues.png
│       └── result.mat
│
├── thermal_force/
│   └── test_YYYYMMDD_HHMMSS/
│       ├── distribution_histogram.png
│       ├── position_comparison.png
│       └── result.mat
│
└── trajectory/
    └── test_YYYYMMDD_HHMMSS/
        ├── z_move_3d.png
        ├── xy_circle_3d.png
        ├── h_bar_safety.png
        └── result.mat
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
6. **h̄_min = 1.5**（安全邊界，避免 h̄ ≤ 1）

---

## 參考文件

- `reference/Drag_MATLAB.pdf` - Wall Effect 數學模型
- `reference/total_block_diagram.png` - 完整系統架構圖
- `reference/thermal_force.png` - Thermal Force 公式
- `r_controller_package` - 參數管理與測試輸出架構參考
- Meng & Menq (2023) - 控制器設計來源
