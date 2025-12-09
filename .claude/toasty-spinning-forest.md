# MotionControl_Simu 實作計畫

## 專案目標
建立完整的磁珠運動控制模擬系統，包含 Wall Effect 的連續系統實作。

---

## 快速參考：確認的參數值

| 參數 | 值 | 單位 | 說明 |
|------|-----|------|------|
| R | **2.25** | μm | 粒子半徑（已修正）|
| γ_N | 0.0425 | pN·sec/μm | Stokes drag |
| Δt | 1/1606 | sec | 取樣週期 |
| k_B | 1.3806503×10⁻⁵ | pN·μm/K | 波茲曼常數 |
| T | 310.15 | K | 溫度 |
| h̄_min | 1.5 | - | 最小安全距離 |
| h_margin | 5 | μm | 額外安全餘量 |

---

## 目前專案狀態

### 已存在檔案
| 檔案 | 狀態 | 說明 |
|------|------|------|
| `CLAUDE.md` | 已完成 | 專案規範文件 |
| `.gitignore` | 已存在 | 基本完整 |
| `model/wall_effect/calc_wall_params.m` | 空殼 | 只有函數簽名 |
| `model/wall_effect/wall_effect_integrated.m` | 空殼 | 只有函數簽名 |
| `model/system_model.slx` | 存在 | 需確認內容 |
| `test_script/run_wall_effect_test.m` | 空殼 | 只有 clear 語句 |

### 尚未建立
- `model/thermal_force/` 目錄及檔案
- `model/trajectory/` 目錄及檔案
- `model/controller/` 目錄及檔案
- `test_results/` 子目錄
- `calc_simulation_params.m`
- `calc_initial_position.m`
- `run_simulation.m`
- 其他測試腳本

---

## 實作討論階段

### 階段總覽

| 階段 | 主題 | 狀態 | 討論重點 |
|------|------|------|----------|
| 1 | 專案基礎設施 | **已完成** | 目錄結構、路徑管理、初始化流程 |
| 2 | Wall Effect 核心函數 | **已完成** | 函數拆分、介面設計、邊界處理 |
| 3 | Wall Effect 測試 | **已完成** | 測試參數、圖形規格、Tabbed Figure |
| 4 | Thermal Force 模組 | **已完成** | 函數設計、RNG 處理、params 欄位 |
| 5 | Thermal Force 測試 | **已完成** | 統計驗證參數、Tabbed Figure 設計 |
| 6 | 軌跡產生器模組 | **已完成** | 即時計算、安全檢查邏輯、函數介面 |
| 7 | 軌跡產生器測試 | **已完成** | 3D 圖形設定、Tabbed Figure |
| 8 | 參數管理 | **已完成** | config/params 結構、預設值策略 |
| 9 | 控制器模組 | **已完成** | 狀態變數管理、初始化、Unit Delay |
| 10 | Simulink 整合 | **已完成** | Block 設定、信號連接、Solver |
| 11 | 整合測試 | **已完成** | 測試情境、run_simulation.m |

---

## 階段 1：專案基礎設施

### 1.1 討論項目
- [x] 目錄結構確認
- [x] 是否需要 `startup.m` 自動加入路徑？
- [x] `.gitignore` 是否需要補充？

### 1.2 最終決定

#### 目錄結構（分層）
```
MotionControl_Simu/
├── CLAUDE.md
├── calc_simulation_params.m          # 參數計算 + Bus Object
├── run_simulation.m                  # 主執行腳本
│
├── model/
│   ├── wall_effect/
│   │   ├── calc_correction_functions.m   # c_∥, c_⊥ 計算
│   │   └── calc_gamma_inv.m              # Γ⁻¹ 矩陣計算
│   │
│   ├── thermal_force/
│   │   └── calc_thermal_force.m          # F_th 生成
│   │
│   ├── trajectory/
│   │   ├── calc_initial_position.m       # 起始位置計算
│   │   ├── trajectory_generator.m        # 即時軌跡計算
│   │   └── check_trajectory_safety.m     # 安全檢查
│   │
│   ├── controller/
│   │   └── motion_control_law.m          # 控制律
│   │
│   └── system_model.slx                  # Simulink 主模型
│
├── test_script/
│   ├── run_wall_effect_test.m
│   ├── run_thermal_force_test.m
│   └── run_trajectory_test.m
│
├── test_results/                         # 不納入 Git
│   ├── wall_effect/
│   ├── thermal_force/
│   └── trajectory/
│
└── reference/
```

#### 路徑管理
- 不使用 `startup.m`
- 每個測試腳本及 `run_simulation.m` 開頭加入 addpath：
```matlab
addpath('model/wall_effect');
addpath('model/thermal_force');
addpath('model/trajectory');
addpath('model/controller');
```

#### 初始化流程
| 步驟 | 檔案 | 動作 |
|------|------|------|
| 1 | `calc_simulation_params.m` | 計算所有參數，產生 params 結構 |
| 2 | `calc_initial_position.m` | 根據 params 計算安全起始位置 p0 |
| 3 | `run_simulation.m` | 把 params, p0 放入 workspace |
| 4 | Simulink 積分器 | 讀取 p0 作為 Initial Condition |
| 5 | 各 MATLAB Function | 接收當前 p 做即時計算 |

#### 需要新建
- `model/thermal_force/`
- `model/trajectory/`
- `model/controller/`
- `test_results/wall_effect/`
- `test_results/thermal_force/`
- `test_results/trajectory/`

#### 需要刪除
- `model/wall_effect/calc_wall_params.m`（空殼）
- `model/wall_effect/wall_effect_integrated.m`（空殼）

#### 不變動
- `.gitignore`（已足夠）
- `model/system_model.slx`（後續處理）
- `test_script/run_wall_effect_test.m`（後續填入內容）

### 1.3 討論記錄
- 確認採用分層資料夾結構，不採用扁平結構
- 路徑管理：每個腳本開頭明確加入 addpath，不使用 startup.m
- p0 初始化：由 calc_initial_position.m 計算，積分器從 workspace 讀取

---

## 階段 2：Wall Effect 核心函數

### 2.1 討論項目
- [x] 函數拆分方式（單一函數 vs 多函數）
- [x] 現有空殼檔案如何處理（重新命名 or 刪除）
- [x] 輸入輸出介面設計
- [x] 邊界條件處理策略（h̄ ≤ 1）
- [x] 是否需要向量化（批次計算多個位置）

### 2.2 最終決定

#### 檔案結構
```
model/wall_effect/
├── calc_correction_functions.m   # c_∥, c_⊥ 計算
└── calc_gamma_inv.m              # Γ⁻¹ 矩陣計算

刪除：
├── calc_wall_params.m            # 空殼，由 calc_simulation_params.m 取代
└── wall_effect_integrated.m      # 空殼，由 calc_gamma_inv.m 取代
```

#### 函數 1：calc_correction_functions.m

**簽名**：
```matlab
function [c_para, c_perp] = calc_correction_functions(h_bar)
```

**輸入輸出**：
| 參數 | 型態 | 說明 |
|------|------|------|
| h_bar (輸入) | 純量 | 正規化距離 h̄ = h/R |
| c_para (輸出) | 純量 | 平行修正係數 c_∥ |
| c_perp (輸出) | 純量 | 垂直修正係數 c_⊥ |

**內部計算**：
```matlab
inv_h = 1 / h_bar;
c_para = 1 / (1 - 9/16*inv_h + 1/8*inv_h^3 - 45/256*inv_h^4 - 1/16*inv_h^5);
c_perp = 1 / (1 - 9/8*inv_h + 1/2*inv_h^3 - 57/100*inv_h^4 + 1/5*inv_h^5 ...
             + 7/200*inv_h^11 - 1/25*inv_h^12);
```

**邊界處理**：h̄ ≤ 1 時拋出錯誤

#### 函數 2：calc_gamma_inv.m

**簽名**：
```matlab
function [Gamma_inv, h_bar] = calc_gamma_inv(p, params)
```

**輸入輸出**：
| 參數 | 型態 | 說明 |
|------|------|------|
| p (輸入) | 3×1 向量 | 粒子位置 |
| params (輸入) | 結構體 | 包含 wall, common 子結構 |
| Gamma_inv (輸出) | 3×3 矩陣 | 移動度矩陣 |
| h_bar (輸出) | 純量 | 正規化距離 |

**需要的 params 欄位**：
- params.wall.w_hat (3×1)
- params.wall.pz (純量)
- params.common.R (純量)
- params.common.gamma_N (純量)

**內部計算**：
```matlab
h = dot(p, w_hat) - pz;
h_bar = h / R;
[c_para, c_perp] = calc_correction_functions(h_bar);
W = w_hat * w_hat';
coeff = (c_perp - c_para) / c_perp;
Gamma_inv = (1 / (gamma_N * c_para)) * (eye(3) - coeff * W);
```

#### 呼叫關係
```
Simulink → calc_gamma_inv(p, params) → calc_correction_functions(h_bar)
```

#### 邊界處理
| 條件 | 處理方式 |
|------|----------|
| h̄ ≤ 1 | 拋出錯誤 |
| h̄ > 1 | 正常計算 |

#### 設計決定
- 不需要向量化，每次處理單一位置
- 軌跡規劃安全餘量 h̄_min = 1.5（在軌跡模組處理）

### 2.3 討論記錄
- 確認拆分為兩個函數：calc_correction_functions + calc_gamma_inv
- 邊界條件：h̄ ≤ 1 拋出錯誤
- 後續銜接（F 輸入、ZOH、積分器）在階段 10 Simulink 整合討論

---

## 階段 3：Wall Effect 測試

### 3.1 討論項目
- [x] 測試圖規格（3 個 Tab）
- [x] 測試參數設定
- [x] 輸出格式
- [x] 參數正確性檢查

### 3.2 最終決定

#### 參數修正
| 參數 | 原值 | 修正值 | 來源 |
|------|------|--------|------|
| R | 2.8 μm | **2.25 μm** | 用戶確認 |
| Δt | 1/1606 | 1/1606 | 確認正確 |
| γ_N | 0.0425 | 0.0425 pN·sec/μm | PDF 確認 |
| k_B | 1.3806503×10⁻⁵ | 1.3806503×10⁻⁵ pN·μm/K | 單位轉換確認 |
| T | 310.15 K | 310.15 K | 確認正確 |

#### 測試腳本：run_wall_effect_test.m

**測試參數**：
```matlab
% 牆面參數（簡單情況）
theta = 0;
phi = 0;
pz = 0;

% 物理參數
R = 2.25;             % 粒子半徑 [μm]
gamma_N = 0.0425;     % Stokes drag [pN·sec/μm]

% 測試範圍
h_bar_range = linspace(1.1, 20, 200);
```

#### Tabbed Figure 設計（3 個 Tab）

| Tab | 名稱 | 內容 | 物理意義 |
|-----|------|------|----------|
| Tab 1 | Correction Functions | c_∥, c_⊥ vs h̄ | 修正係數的數學行為 |
| Tab 2 | Effective Drag | γ_∥, γ_⊥ vs h̄ | 實際拖曳力隨距離變化 |
| Tab 3 | Mobility | 1/(γ_N·c_∥), 1/(γ_N·c_⊥) vs h̄ | 移動度隨距離變化 |

#### Tab 內容詳細規格

**Tab 1: Correction Functions**
- X 軸：h̄ (1.1 ~ 20)
- Y 軸：修正係數
- 曲線：c_∥ (藍), c_⊥ (紅)
- 標註：h̄=1.5 安全線, h̄→∞ 漸近值 (=1)

**Tab 2: Effective Drag**
- X 軸：h̄ (1.1 ~ 20)
- Y 軸：有效拖曳係數 [pN·sec/μm]
- 曲線：γ_∥ = γ_N·c_∥ (藍), γ_⊥ = γ_N·c_⊥ (紅)
- 標註：γ_N 基準線

**Tab 3: Mobility**
- X 軸：h̄ (1.1 ~ 20)
- Y 軸：移動度 [μm/(pN·sec)]
- 曲線：1/(γ_N·c_∥) (藍), 1/(γ_N·c_⊥) (紅)
- 標註：1/γ_N 漸近值

#### 圖形格式（參考 r_controller_package）
```matlab
axis_linewidth = 1.5;
xlabel_fontsize = 14;
ylabel_fontsize = 14;
title_fontsize = 16;
tick_fontsize = 12;
legend_fontsize = 11;
line_width = 2;
```

#### 儲存方式
```matlab
% 分別切換 Tab 儲存 PNG
tabgroup.SelectedTab = tab1;
drawnow;
exportgraphics(fig, fullfile(output_dir, 'correction_functions.png'), 'Resolution', 150);

tabgroup.SelectedTab = tab2;
drawnow;
exportgraphics(fig, fullfile(output_dir, 'effective_drag.png'), 'Resolution', 150);

tabgroup.SelectedTab = tab3;
drawnow;
exportgraphics(fig, fullfile(output_dir, 'mobility.png'), 'Resolution', 150);
```

#### 輸出結構
```
test_results/wall_effect/test_YYYYMMDD_HHMMSS/
├── correction_functions.png
├── effective_drag.png
├── mobility.png
└── result.mat
```

### 3.3 討論記錄
- 採用 Tabbed Figure，3 個 Tab 呈現不同角度的物理意義
- 確認 R = 2.25 μm（原 2.8 為誤）
- 確認 Δt = 1/1606（thermal_force.png 的 1/1612 為舊值）
- 儲存格式：PNG（不存 .fig）

---

## 階段 4：Thermal Force 模組

### 4.1 討論項目
- [x] 函數設計
- [x] RNG seed 處理方式
- [x] params 欄位確認
- [x] 與 Wall Effect 的整合方式

### 4.2 最終決定

#### 函數：calc_thermal_force.m

**路徑**：`model/thermal_force/calc_thermal_force.m`

**簽名**：
```matlab
function F_th = calc_thermal_force(p, params)
```

**輸入輸出**：
| 參數 | 型態 | 說明 |
|------|------|------|
| p (輸入) | 3×1 向量 | 粒子位置 |
| params (輸入) | 結構體 | 包含 wall, thermal, common 子結構 |
| F_th (輸出) | 3×1 向量 | 隨機熱力 [pN] |

**需要的 params 欄位**：
```
params.wall.w_hat      (3×1)  % 法向量
params.wall.u_hat      (3×1)  % 平行向量 1
params.wall.v_hat      (3×1)  % 平行向量 2
params.wall.pz         (純量) % 平面位移
params.common.R        (純量) % 粒子半徑 = 2.25 μm
params.common.gamma_N  (純量) % Stokes drag = 0.0425 pN·sec/μm
params.thermal.k_B     (純量) % 波茲曼常數 = 1.3806503×10⁻⁵ pN·μm/K
params.thermal.T       (純量) % 溫度 = 310.15 K
params.thermal.Ts      (純量) % 取樣週期 = 1/1606 sec
```

**內部計算**：
```matlab
function F_th = calc_thermal_force(p, params)
    % 取出參數
    w_hat = params.wall.w_hat;
    u_hat = params.wall.u_hat;
    v_hat = params.wall.v_hat;
    pz = params.wall.pz;
    R = params.common.R;
    gamma_N = params.common.gamma_N;
    k_B = params.thermal.k_B;
    T = params.thermal.T;
    Ts = params.thermal.Ts;

    % 計算正規化距離
    h = dot(p, w_hat) - pz;
    h_bar = h / R;

    % 計算修正係數
    [c_para, c_perp] = calc_correction_functions(h_bar);

    % 計算 C 向量 (c_x, c_y, c_z)
    C = c_para * (u_hat + v_hat) + c_perp * w_hat;

    % 計算方差
    variance_coeff = 4 * k_B * T * gamma_N / Ts;
    Variance = variance_coeff * (C.^2);

    % 生成隨機熱力 (mean=0, variance=Variance)
    F_th = sqrt(Variance) .* randn(3, 1);
end
```

#### RNG 處理方式

- **函數內不設 seed**：保持函數乾淨
- **測試腳本開頭加註解提醒**：
```matlab
% run_thermal_force_test.m
clear; close all; clc;

% === RNG 設定 ===
% 如需重現測試結果，取消下行註解
% rng(42);

% ... 測試內容 ...
```

#### 統計特性確認

| 特性 | 理論值 | 實作方式 |
|------|--------|----------|
| Mean | [0; 0; 0] | randn 預設 mean=0 |
| Variance | (4·k_B·T·γ_N/Δt)·[c_x²; c_y²; c_z²] | sqrt(Variance) .* randn(3,1) |

### 4.3 討論記錄
- 確認 randn 預設 mean=0，符合需求
- RNG seed 不在函數內設定，由測試腳本控制（註解形式提醒）
- 確認 params 欄位結構

---

## 階段 5：Thermal Force 測試

### 5.1 討論項目
- [x] 測試樣本數量
- [x] 直方圖 bin 數量
- [x] 位置比較的 h̄ 值選擇
- [x] 統計驗證容許誤差

### 5.2 最終決定

#### 測試參數
| 參數 | 值 | 說明 |
|------|-----|------|
| N | 10000 | 樣本數量 |
| bins | 50 | 直方圖 bin 數量 |
| h̄_far | 10 | 遠離牆面測試位置 |
| h̄_near | 1.5 | 靠近牆面測試位置 |

#### h̄=10 vs h̄=1.5 差異驗證
| 項目 | h̄=10 | h̄=1.5 | 增加倍數 |
|------|-------|--------|----------|
| c_∥ | ~1.06 | ~1.62 | **1.5x** |
| c_⊥ | ~1.13 | ~3.21 | **2.8x** |
| σ_∥ | ~1.03 | ~1.27 | **1.2x** |
| σ_⊥ | ~1.06 | ~1.79 | **1.7x** |

**結論**：差異足夠明顯，尤其垂直方向有接近 3 倍差異

#### 測試腳本：run_thermal_force_test.m

**Tabbed Figure 設計（2 個 Tab）**

| Tab | 名稱 | 內容 |
|-----|------|------|
| Tab 1 | Distribution | 分布直方圖（h̄=10）|
| Tab 2 | Position Comparison | 位置相關性比較 |

#### Tab 1: Distribution（分布驗證）
**佈局**：3 個子圖（1×3）
- 子圖 1：F_x 直方圖 + 理論高斯曲線
- 子圖 2：F_y 直方圖 + 理論高斯曲線
- 子圖 3：F_z 直方圖 + 理論高斯曲線

**標註**（每個子圖下方 text box）：
```
理論: μ=0, σ=xx.xx pN
實測: μ=xx.xx, σ=xx.xx pN
誤差: Δμ=xx.xx%, Δσ=xx.xx%
```

#### Tab 2: Position Comparison（位置比較）
**佈局**：長條圖（grouped bar chart）
- X 軸：三組（σ_x, σ_y, σ_z）
- Y 軸：標準差 [pN]
- 每組兩個長條：h̄=10（藍）、h̄=1.5（紅）

**標註**：
- 圖例：h̄=10, h̄=1.5
- 每個長條上方顯示數值
- 圖下方 text box 顯示倍數比較：
```
σ_x 增加: 1.2x | σ_y 增加: 1.2x | σ_z 增加: 1.7x
```

#### 圖形格式（同 r_controller_package）
```matlab
axis_linewidth = 1.5;
xlabel_fontsize = 14;
ylabel_fontsize = 14;
title_fontsize = 16;
tick_fontsize = 12;
legend_fontsize = 11;
line_width = 2;
```

#### 直方圖樣式
```matlab
histogram(data, bins, 'Normalization', 'pdf', ...
    'FaceColor', [0.2 0.4 0.8], 'EdgeColor', 'white', 'FaceAlpha', 0.7);
```

#### 理論曲線疊加
```matlab
x_range = linspace(min(data)-3*std(data), max(data)+3*std(data), 200);
y_theory = normpdf(x_range, 0, sigma_theory);
plot(x_range, y_theory, 'r-', 'LineWidth', 2);
```

#### 儲存方式
```matlab
% 切換 Tab 儲存 PNG
tabgroup.SelectedTab = tab1;
drawnow;
exportgraphics(fig, fullfile(output_dir, 'distribution_histogram.png'), 'Resolution', 150);

tabgroup.SelectedTab = tab2;
drawnow;
exportgraphics(fig, fullfile(output_dir, 'position_comparison.png'), 'Resolution', 150);
```

#### 輸出結構
```
test_results/thermal_force/test_YYYYMMDD_HHMMSS/
├── distribution_histogram.png
├── position_comparison.png
└── result.mat
```

#### result.mat 內容
```matlab
result.params = params;           % 測試參數
result.N = N;                     % 樣本數
result.h_bar_test = [10, 1.5];    % 測試 h̄ 值
result.sigma_theory = [...];      % 理論標準差
result.sigma_measured = [...];    % 實測標準差
result.mean_measured = [...];     % 實測均值
```

### 5.3 討論記錄
- 確認 N=10000, bins=50
- 確認 h̄=10 vs h̄=1.5 比較（差異足夠明顯）
- 圖形為主，數值顯示在圖中（text box）
- 繪圖風格與 r_controller_package 一致

---

## 階段 6：軌跡產生器模組

### 6.1 討論項目
- [x] `trajectory_generator.m` 介面設計
- [x] `check_trajectory_safety.m` 設計
- [x] `calc_initial_position.m` 設計
- [x] 檔案位置調整
- [x] 錯誤處理方式

### 6.2 最終決定

#### 檔案結構（調整後）
```
model/trajectory/
├── calc_initial_position.m       # 起始位置計算（移入此資料夾）
├── trajectory_generator.m        # 即時軌跡計算
└── check_trajectory_safety.m     # 安全檢查
```

#### 函數 1：calc_initial_position.m

**路徑**：`model/trajectory/calc_initial_position.m`

**簽名**：
```matlab
function p0 = calc_initial_position(params)
```

**輸入輸出**：
| 參數 | 型態 | 說明 |
|------|------|------|
| params (輸入) | 結構體 | 包含 wall, common, traj 子結構 |
| p0 (輸出) | 3×1 向量 | 起始位置 [μm] |

**需要的 params 欄位**：
```
params.wall.w_hat        (3×1)  % 法向量
params.wall.pz           (純量) % 平面位移
params.wall.h_bar_min    (純量) % 最小安全距離（正規化）= 1.5
params.common.R          (純量) % 粒子半徑 = 2.25 μm
params.traj.h_margin     (純量) % 額外安全餘量 = 5 μm
```

**內部計算**：
```matlab
function p0 = calc_initial_position(params)
    w_hat = params.wall.w_hat;
    pz = params.wall.pz;
    h_bar_min = params.wall.h_bar_min;
    R = params.common.R;
    h_margin = params.traj.h_margin;

    % 計算安全距離
    h_safe = R * h_bar_min + h_margin;

    % 起始位置 = 牆面位置 + 安全距離沿法向量
    p0 = pz * w_hat + h_safe * w_hat;
end
```

#### 函數 2：trajectory_generator.m

**路徑**：`model/trajectory/trajectory_generator.m`

**簽名**：
```matlab
function p_d = trajectory_generator(t, p0, params)
```

**輸入輸出**：
| 參數 | 型態 | 說明 |
|------|------|------|
| t (輸入) | 純量 | 當前時間 [sec] |
| p0 (輸入) | 3×1 向量 | 起始位置 [μm] |
| params (輸入) | 結構體 | 包含 wall, traj 子結構 |
| p_d (輸出) | 3×1 向量 | 期望位置 [μm] |

**需要的 params 欄位**：
```
params.wall.w_hat        (3×1)  % 法向量
params.wall.u_hat        (3×1)  % 平行向量 1
params.wall.v_hat        (3×1)  % 平行向量 2
params.traj.type         (字串) % 'z_move' 或 'xy_circle'

% z_move 專用
params.traj.delta_z      (純量) % 移動距離 [μm]
params.traj.direction    (字串) % 'away' 或 'toward'
params.traj.speed        (純量) % 移動速度 [μm/sec]

% xy_circle 專用
params.traj.radius       (純量) % 圓半徑 [μm]
params.traj.period       (純量) % 週期 [sec]
params.traj.n_circles    (純量) % 圈數
```

**內部計算**：
```matlab
function p_d = trajectory_generator(t, p0, params)
    traj_type = params.traj.type;

    switch traj_type
        case 'z_move'
            % 沿法向量方向移動
            w_hat = params.wall.w_hat;
            delta_z = params.traj.delta_z;
            direction = params.traj.direction;
            speed = params.traj.speed;

            % 計算移動方向
            if strcmp(direction, 'away')
                dir_sign = 1;
            else  % 'toward'
                dir_sign = -1;
            end

            % 計算位移（有飽和）
            displacement = min(speed * t, delta_z);
            p_d = p0 + dir_sign * displacement * w_hat;

        case 'xy_circle'
            % 在牆面平行平面內繞圓
            u_hat = params.wall.u_hat;
            v_hat = params.wall.v_hat;
            radius = params.traj.radius;
            period = params.traj.period;
            n_circles = params.traj.n_circles;

            % 總時間
            T_total = period * n_circles;

            if t <= T_total
                omega = 2 * pi / period;
                p_d = p0 + radius * (cos(omega * t) - 1) * u_hat ...
                         + radius * sin(omega * t) * v_hat;
            else
                % 回到起點
                p_d = p0;
            end

        otherwise
            error('Unknown trajectory type: %s', traj_type);
    end
end
```

#### 函數 3：check_trajectory_safety.m

**路徑**：`model/trajectory/check_trajectory_safety.m`

**簽名**：
```matlab
function [is_safe, h_bar_min_actual, t_critical] = check_trajectory_safety(p0, params)
```

**輸入輸出**：
| 參數 | 型態 | 說明 |
|------|------|------|
| p0 (輸入) | 3×1 向量 | 起始位置 |
| params (輸入) | 結構體 | 參數結構體 |
| is_safe (輸出) | 邏輯值 | 是否安全 |
| h_bar_min_actual (輸出) | 純量 | 軌跡中最小 h̄ |
| t_critical (輸出) | 純量 | 最小 h̄ 發生時刻 |

**內部計算**：
```matlab
function [is_safe, h_bar_min_actual, t_critical] = check_trajectory_safety(p0, params)
    % 參數
    w_hat = params.wall.w_hat;
    pz = params.wall.pz;
    R = params.common.R;
    h_bar_min = params.wall.h_bar_min;
    Ts = params.common.Ts;
    T_sim = params.common.T_sim;

    % 生成時間序列
    t_vec = 0:Ts:T_sim;
    N = length(t_vec);

    % 計算所有軌跡點的 h_bar
    h_bar_vec = zeros(1, N);
    for i = 1:N
        p_d = trajectory_generator(t_vec(i), p0, params);
        h = dot(p_d, w_hat) - pz;
        h_bar_vec(i) = h / R;
    end

    % 找最小值
    [h_bar_min_actual, idx] = min(h_bar_vec);
    t_critical = t_vec(idx);

    % 判斷安全性
    is_safe = (h_bar_min_actual >= h_bar_min);

    % 輸出警告（如果不安全）
    if ~is_safe
        warning(['Trajectory unsafe! Min h_bar = %.2f at t = %.3f sec. ' ...
                 'Required h_bar_min = %.2f'], ...
                h_bar_min_actual, t_critical, h_bar_min);
    end
end
```

#### 呼叫關係
```
run_simulation.m
    │
    ├── calc_simulation_params(config) → params
    │
    ├── calc_initial_position(params) → p0
    │
    ├── check_trajectory_safety(p0, params) → 安全檢查
    │
    └── Simulink: trajectory_generator(t, p0, params) → p_d
```

#### 設計決定
| 項目 | 決定 |
|------|------|
| h_bar_min | 1.5（預設） |
| h_margin | 5 μm（預設） |
| 安全檢查方式 | 模擬前全點檢查 |
| 不安全時處理 | **error 直接中斷**（先檢驗再模擬）|

### 6.3 討論記錄
- calc_initial_position.m 移入 model/trajectory/ 資料夾
- trajectory_generator 接收 (t, p0, params) 作為輸入
- 軌跡類型由 params.traj.type 區分
- 安全檢查在模擬前執行，檢查所有時刻點
- 不安全時用 error 直接中斷（先檢驗再模擬的流程）
- 起點與路徑都確保在牆面法線正方向側

---

## 階段 7：軌跡產生器測試

### 7.1 討論項目
- [x] 3D 圖形設定
- [x] 測試軌跡參數
- [x] h̄(t) 圖的標示方式
- [x] 測試獨立性確認

### 7.2 最終決定

#### 測試腳本：run_trajectory_test.m

**Tabbed Figure 設計（3 個 Tab）**

| Tab | 名稱 | 內容 |
|-----|------|------|
| Tab 1 | z_move | z_move 軌跡 3D 圖 |
| Tab 2 | xy_circle | xy_circle 軌跡 3D 圖 |
| Tab 3 | Safety | h̄(t) 安全性曲線 |

#### 測試參數

**牆面參數（簡單情況）**：
```matlab
theta = 0;       % 方位角
phi = 0;         % 仰角 → 牆面平行於 xy 平面
pz = 0;          % 牆面位移
```

**z_move 軌跡參數**：
```matlab
traj_z.type = 'z_move';
traj_z.delta_z = 10;       % 移動距離 [μm]
traj_z.direction = 'away'; % 遠離牆面
traj_z.speed = 5;          % 速度 [μm/sec]
```

**xy_circle 軌跡參數**：
```matlab
traj_xy.type = 'xy_circle';
traj_xy.radius = 5;        % 圓半徑 [μm]
traj_xy.period = 1;        % 週期 [sec]
traj_xy.n_circles = 3;     % 圈數
```

#### Tab 1 & 2: 3D 軌跡圖

**元素**：
1. **牆面平面**：半透明 patch
   - 顏色：淺灰色 [0.8, 0.8, 0.8]
   - 透明度：FaceAlpha = 0.3
   - 大小：足夠容納軌跡（自動計算範圍 ±margin）

2. **軌跡線**：
   - 顏色：藍色
   - LineWidth = 2

3. **起點標記**：
   - 綠色圓點 'go', MarkerSize = 10, MarkerFaceColor = 'g'

4. **終點標記**：
   - 紅色方形 'rs', MarkerSize = 10, MarkerFaceColor = 'r'

5. **座標軸向量**（從起點發出）：
   - ŵ（法向量）：紅色箭頭
   - û（平行向量 1）：綠色箭頭
   - v̂（平行向量 2）：藍色箭頭
   - 箭頭長度：軌跡範圍的 20%

**視角設定**：
```matlab
view(30, 30);        % azimuth=30°, elevation=30°
axis equal;
grid on;
```

**標題與標籤**：
```matlab
title('z\_move Trajectory', 'FontSize', title_fontsize);
xlabel('x [\mum]', 'FontSize', xlabel_fontsize);
ylabel('y [\mum]', 'FontSize', ylabel_fontsize);
zlabel('z [\mum]', 'FontSize', xlabel_fontsize);
```

#### Tab 3: Safety 曲線

**佈局**：2 個子圖（2×1）

**子圖 1：z_move 的 h̄(t)**
- X 軸：時間 t [sec]
- Y 軸：h̄
- 曲線：藍色實線
- h̄_min 安全線：紅色虛線
- 標註：h̄_min = 1.5

**子圖 2：xy_circle 的 h̄(t)**
- 同上格式
- xy_circle 的 h̄ 應為常數（平行於牆面移動）

#### 圖形格式（同 r_controller_package）
```matlab
axis_linewidth = 1.5;
xlabel_fontsize = 14;
ylabel_fontsize = 14;
title_fontsize = 16;
tick_fontsize = 12;
legend_fontsize = 11;
line_width = 2;
```

#### 箭頭繪製（quiver3）
```matlab
% 從起點 p0 繪製向量
arrow_scale = 0.2 * (max(traj_range) - min(traj_range));
quiver3(p0(1), p0(2), p0(3), w_hat(1), w_hat(2), w_hat(3), ...
    arrow_scale, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
```

#### 儲存方式
```matlab
% 切換 Tab 儲存 PNG
tabgroup.SelectedTab = tab1;
drawnow;
exportgraphics(fig, fullfile(output_dir, 'z_move_3d.png'), 'Resolution', 150);

tabgroup.SelectedTab = tab2;
drawnow;
exportgraphics(fig, fullfile(output_dir, 'xy_circle_3d.png'), 'Resolution', 150);

tabgroup.SelectedTab = tab3;
drawnow;
exportgraphics(fig, fullfile(output_dir, 'h_bar_safety.png'), 'Resolution', 150);
```

#### 輸出結構
```
test_results/trajectory/test_YYYYMMDD_HHMMSS/
├── z_move_3d.png
├── xy_circle_3d.png
├── h_bar_safety.png
└── result.mat
```

#### result.mat 內容
```matlab
result.params_z = params_z;           % z_move 參數
result.params_xy = params_xy;         % xy_circle 參數
result.traj_z = traj_z_data;          % z_move 軌跡點
result.traj_xy = traj_xy_data;        % xy_circle 軌跡點
result.h_bar_z = h_bar_z_vec;         % z_move h̄(t)
result.h_bar_xy = h_bar_xy_vec;       % xy_circle h̄(t)
result.is_safe_z = is_safe_z;         % z_move 安全性
result.is_safe_xy = is_safe_xy;       % xy_circle 安全性
```

### 7.3 討論記錄
- 3 個 Tab：z_move、xy_circle、Safety
- 牆面以半透明 patch 呈現
- 座標軸向量從起點發出，箭頭長度為軌跡範圍的 20%
- h̄(t) 圖以紅色虛線標示 h̄_min
- **測試獨立性**：軌跡測試是純幾何測試，不依賴 Wall Effect / Thermal Force 模組
- 只依賴 `calc_simulation_params.m`（產生 params 結構）

---

## 階段 8：參數管理

### 8.1 討論項目
- [x] `calc_simulation_params.m` 設計
- [x] config 輸入結構
- [x] Bus Object 結構
- [x] 預設值策略

### 8.2 最終決定

#### 函數：calc_simulation_params.m

**路徑**：`calc_simulation_params.m`（根目錄）

**簽名**：
```matlab
function params = calc_simulation_params(config)
```

**輸入 config 結構**：
```matlab
config = struct(...
    % === Wall 參數 ===
    'theta', 0, ...              % 方位角 [rad]
    'phi', 0, ...                % 仰角 [rad]
    'pz', 0, ...                 % 平面位移 [μm]
    'h_bar_min', 1.5, ...        % 最小安全 h̄
    ...
    % === 軌跡參數 ===
    'traj_type', 'z_move', ...   % 'z_move' 或 'xy_circle'
    'h_margin', 5, ...           % 額外安全餘量 [μm]
    'delta_z', 10, ...           % z_move: 移動距離 [μm]
    'direction', 'away', ...     % z_move: 'away' 或 'toward'
    'speed', 5, ...              % z_move: 速度 [μm/sec]
    'radius', 5, ...             % xy_circle: 圓半徑 [μm]
    'period', 1, ...             % xy_circle: 週期 [sec]
    'n_circles', 3, ...          % xy_circle: 圈數
    ...
    % === 控制器參數 ===
    'lambda_c', 0.7, ...         % 閉迴路極點 (0 < λ_c < 1)
    ...
    % === Thermal Force 參數 ===
    'thermal_enable', true, ...  % 是否啟用熱力
    ...
    % === 模擬參數 ===
    'T_sim', 5 ...               % 模擬時間 [sec]
);
```

**輸出 params 結構**：
```matlab
params
├── common
│   ├── R          = 2.25            % 粒子半徑 [μm]（修正後）
│   ├── gamma_N    = 0.0425          % Stokes drag [pN·sec/μm]
│   ├── Ts         = 1/1606          % 取樣週期 [sec]
│   └── T_sim      = config.T_sim    % 模擬時間 [sec]
│
├── wall
│   ├── theta      = config.theta
│   ├── phi        = config.phi
│   ├── pz         = config.pz
│   ├── h_bar_min  = config.h_bar_min
│   ├── w_hat      = [計算得出]       % 法向量 (3×1)
│   ├── u_hat      = [計算得出]       % 平行向量 1 (3×1)
│   └── v_hat      = [計算得出]       % 平行向量 2 (3×1)
│
├── traj
│   ├── type       = config.traj_type
│   ├── h_margin   = config.h_margin
│   ├── delta_z    = config.delta_z
│   ├── direction  = config.direction
│   ├── speed      = config.speed
│   ├── radius     = config.radius
│   ├── period     = config.period
│   └── n_circles  = config.n_circles
│
├── ctrl
│   ├── lambda_c   = config.lambda_c
│   ├── gamma      = params.common.gamma_N  % 初始版本固定值
│   └── Ts         = params.common.Ts
│
└── thermal
    ├── enable     = config.thermal_enable
    ├── k_B        = 1.3806503e-5           % [pN·μm/K]
    ├── T          = 310.15                 % 溫度 [K]
    ├── Ts         = params.common.Ts
    └── variance_coeff = 4*k_B*T*gamma_N/Ts % 預計算
```

**內部計算**：
```matlab
function params = calc_simulation_params(config)
    %% 固定常數
    R = 2.25;                    % 粒子半徑 [μm]（修正後）
    gamma_N = 0.0425;            % Stokes drag [pN·sec/μm]
    Ts = 1/1606;                 % 取樣週期 [sec]
    k_B = 1.3806503e-5;          % 波茲曼常數 [pN·μm/K]
    T = 310.15;                  % 溫度 [K]

    %% common 參數
    params.common.R = R;
    params.common.gamma_N = gamma_N;
    params.common.Ts = Ts;
    params.common.T_sim = config.T_sim;

    %% wall 參數
    theta = config.theta;
    phi = config.phi;

    params.wall.theta = theta;
    params.wall.phi = phi;
    params.wall.pz = config.pz;
    params.wall.h_bar_min = config.h_bar_min;

    % 計算正交基底向量
    params.wall.w_hat = [cos(theta)*sin(phi); sin(theta)*sin(phi); cos(phi)];
    params.wall.u_hat = [-cos(theta)*cos(phi); -sin(theta)*cos(phi); sin(phi)];
    params.wall.v_hat = [sin(theta); -cos(theta); 0];

    %% traj 參數
    params.traj.type = config.traj_type;
    params.traj.h_margin = config.h_margin;
    params.traj.delta_z = config.delta_z;
    params.traj.direction = config.direction;
    params.traj.speed = config.speed;
    params.traj.radius = config.radius;
    params.traj.period = config.period;
    params.traj.n_circles = config.n_circles;

    %% ctrl 參數
    params.ctrl.lambda_c = config.lambda_c;
    params.ctrl.gamma = gamma_N;  % 初始版本固定值
    params.ctrl.Ts = Ts;

    %% thermal 參數
    params.thermal.enable = config.thermal_enable;
    params.thermal.k_B = k_B;
    params.thermal.T = T;
    params.thermal.Ts = Ts;
    params.thermal.variance_coeff = 4 * k_B * T * gamma_N / Ts;
end
```

#### 預設值策略
- 固定常數（R, gamma_N, Ts, k_B, T）寫死在函數內
- 使用者可調參數透過 config 傳入
- config 中未提供的欄位使用上述預設值

#### Nested Bus 設計（參考 r_controller_package）

**Bus 結構**：
```
ParamsBus
├── common (CommonBus)
│   ├── R          [1 1] double
│   ├── gamma_N    [1 1] double
│   ├── Ts         [1 1] double
│   └── T_sim      [1 1] double
│
├── wall (WallBus)
│   ├── theta      [1 1] double
│   ├── phi        [1 1] double
│   ├── pz         [1 1] double
│   ├── h_bar_min  [1 1] double
│   ├── w_hat      [3 1] double
│   ├── u_hat      [3 1] double
│   └── v_hat      [3 1] double
│
├── traj (TrajBus)
│   ├── type       [1 1] double    % 0=z_move, 1=xy_circle
│   ├── h_margin   [1 1] double
│   ├── delta_z    [1 1] double
│   ├── direction  [1 1] double    % 0=away, 1=toward
│   ├── speed      [1 1] double
│   ├── radius     [1 1] double
│   ├── period     [1 1] double
│   └── n_circles  [1 1] double
│
├── ctrl (CtrlBus)
│   ├── enable     [1 1] double    % 0=off（開迴路）, 1=on（閉迴路）
│   ├── lambda_c   [1 1] double
│   ├── gamma      [1 1] double
│   └── Ts         [1 1] double
│
└── thermal (ThermalBus)
    ├── enable     [1 1] double    % 0=off, 1=on
    ├── k_B        [1 1] double
    ├── T          [1 1] double
    ├── Ts         [1 1] double
    └── variance_coeff [1 1] double
```

**字串改為數值編碼**（Simulink Bus 不支援字串）：
- `type`: `'z_move'` → `0`, `'xy_circle'` → `1`
- `direction`: `'away'` → `0`, `'toward'` → `1`
- `enable`: `false` → `0`, `true` → `1`

**建構順序**：
1. 建立 5 個子 Bus（CommonBus, WallBus, TrajBus, CtrlBus, ThermalBus）
2. 各子 Bus assignin 到 base workspace
3. 建立父 Bus（ParamsBus），引用子 Bus
4. 用 Simulink.Parameter 包裝，設定 DataType = 'Bus: ParamsBus'

### 8.3 討論記錄
- R = 2.25 μm（已修正）
- 採用 Nested Bus 方案（參考 r_controller_package）
- 字串參數改為數值編碼（Simulink 限制）
- 固定常數寫死在函數內，使用者只需設定 config

---

## 階段 9：控制器模組

### 9.1 討論項目
- [x] `motion_control_law.m` 設計
- [x] p_d_prev 狀態管理方式
- [x] 初始化處理
- [x] γ 型態
- [x] 變數命名（p → p_m）

### 9.2 最終決定

#### 函數：motion_control_law.m

**路徑**：`model/controller/motion_control_law.m`

**簽名**：
```matlab
function f_d = motion_control_law(p_d, p_m, params)
```

**輸入輸出**：
| 參數 | 型態 | 說明 |
|------|------|------|
| p_d (輸入) | 3×1 向量 | 當前期望位置 [μm] |
| p_m (輸入) | 3×1 向量 | 當前量測位置 [μm] |
| params (輸入) | 結構體 | 包含 ctrl 子結構 |
| f_d (輸出) | 3×1 向量 | 控制力 [pN] |

**需要的 params 欄位**：
```
params.ctrl.gamma     (純量)  % 拖曳係數 = 0.0425 pN·sec/μm
params.ctrl.lambda_c  (純量)  % 閉迴路極點 (0 < λ_c < 1)
params.ctrl.Ts        (純量)  % 取樣週期 = 1/1606 sec
```

**控制律**：
```
f_d[k] = (γ / Δt) · {p_d[k] - λ_c · p_d[k-1] - (1 - λ_c) · p_m[k]}
```

**內部計算**：
```matlab
function f_d = motion_control_law(p_d, p_m, params)
    % 開迴路模式：直接返回零
    if ~params.ctrl.enable
        f_d = zeros(3, 1);
        return;
    end

    % 閉迴路模式
    persistent p_d_prev

    % 初始化（第一次呼叫時）
    if isempty(p_d_prev)
        p_d_prev = p_d;  % f_d[0] = 0
    end

    % 取出參數
    gamma = params.ctrl.gamma;
    lambda_c = params.ctrl.lambda_c;
    Ts = params.ctrl.Ts;

    % 控制律計算
    f_d = (gamma / Ts) * (p_d - lambda_c * p_d_prev - (1 - lambda_c) * p_m);

    % 更新狀態
    p_d_prev = p_d;
end
```

#### p_d_prev 狀態管理

**方案**：使用 persistent 變數在函數內部管理

**初始化邏輯**：
- 第一次呼叫時 `p_d_prev = p_d`
- 此時控制律計算結果：
  ```
  f_d[0] = (γ/Δt) · {p_d[0] - λ_c·p_d[0] - (1-λ_c)·p_m[0]}
         = (γ/Δt) · {(1-λ_c)·p_d[0] - (1-λ_c)·p_m[0]}
         = (γ/Δt) · (1-λ_c) · (p_d[0] - p_m[0])
  ```
- 若 p_d[0] = p_m[0] = p0（起始位置相同），則 **f_d[0] = 0**
- 這是正確的行為：系統在起始位置靜止

#### 設計決定
| 項目 | 決定 |
|------|------|
| γ 型態 | 純量（初始版本）|
| p_d_prev 管理 | persistent 變數（函數內部） |
| 初始值 | p_d_prev = p_d（第一次呼叫） |
| 變數命名 | p → p_m（量測位置） |

### 9.3 討論記錄
- 控制器使用 persistent 變數管理 p_d_prev 狀態
- 不使用 Simulink Unit Delay（簡化架構）
- 初始化設計確保 f_d[0] = 0（系統起始於平衡狀態）
- 變數名稱從 p 改為 p_m，明確表示「量測位置」
- 初始版本使用固定 γ = γ_N

---

## 階段 10：Simulink 整合

### 10.1 討論項目
- [x] Simulink 模型架構
- [x] MATLAB Function block 設定
- [x] 各 block 連接方式
- [x] 初始化流程
- [x] 輸出記錄設定

### 10.2 最終決定

#### Simulink 模型架構

```
┌──────────────────────────────────────────────────────────────────────────┐
│                        system_model.slx                                  │
│                                                                          │
│  params (Constant)                                                       │
│      │                                                                   │
│      ▼                                                                   │
│  ┌────────────────┐                                                      │
│  │   Trajectory   │──p_d─────────────────────────────┐                   │
│  │   Generator    │                                  │                   │
│  │  (MATLAB Func) │                                  ▼                   │
│  │  Ts=1/1606     │                            ┌──────────┐              │
│  └────────────────┘                            │Controller│              │
│         ▲                                      │(MATLAB   │──f_d──┐      │
│         │                                      │ Func)    │       │      │
│         │                                      │Ts=1/1606 │       │      │
│         │                                      │內含      │       │      │
│         │                                      │persistent│       │      │
│         │                                      └────▲─────┘       │      │
│         │                                           │             │      │
│         │                                           │ p_m         │      │
│         │                                           │             ▼      │
│  t (Clock)                                  ┌───────┴───┐    ┌────────┐  │
│                                             │           │    │  ZOH   │  │
│  ┌────────────────┐                         │           │    │        │  │
│  │ Thermal Force  │──F_th──┐                │           │    └───┬────┘  │
│  │  (MATLAB Func) │        │                │           │        │       │
│  │  Ts=1/1606     │        │                │           │        ▼       │
│  └───────▲────────┘        │                │           │    ┌────────┐  │
│          │                 │                │           │    │   +    │  │
│          │ p_m             ▼                │           │    └───┬────┘  │
│          │           ┌──────────┐           │           │        │       │
│          │           │   ZOH    │           │           │        ▼       │
│          │           └────┬─────┘           │           │  F_total       │
│          │                │                 │           │        │       │
│          │                │                 │           │        ▼       │
│          │                ▼                 │           │  ┌──────────┐  │
│          │           F_th_held              │           │  │ Γ⁻¹(p)·F │  │
│          │                │                 │           │  │(MATLAB   │  │
│          └────────────────┴─────────────────┤           │◀─│ Func)    │  │
│                                             │           │  │連續      │  │
│                                             │           │  └────┬─────┘  │
│                                             │           │       │ ṗ      │
│                                             │           │       ▼        │
│                                             │           │  ┌──────────┐  │
│                                             │           │  │Integrator│  │
│                                             └───────────┤◀─│   1/s    │  │
│                                                     p_m │  │  IC=p0   │  │
│                                                         │  └──────────┘  │
│                                                         │                │
│  ┌──────────────────────────────────────────────────────┴────────────┐   │
│  │                    To Workspace (p_m, p_d, f_d)                   │   │
│  └───────────────────────────────────────────────────────────────────┘   │
└──────────────────────────────────────────────────────────────────────────┘
```

**架構變更說明**（相較於先前版本）：
- 移除 Unit Delay block（p_d_prev 由控制器內部 persistent 變數管理）
- 積分器輸出信號命名為 p_m（量測位置）

#### Block 設定

| Block | 類型 | Sample Time | 說明 |
|-------|------|-------------|------|
| Trajectory Generator | MATLAB Function | 1/1606 | 輸入: t, p0, params |
| Controller | MATLAB Function | 1/1606 | 輸入: p_d, p_m, params（內含 persistent） |
| Thermal Force | MATLAB Function | 1/1606 | 輸入: p_m, params |
| ZOH (×2) | Zero-Order Hold | - | 離散→連續轉換 |
| Γ⁻¹(p)·F | MATLAB Function | -1 (連續) | 輸入: F_total, p_m, params |
| Integrator | Continuous | - | IC = p0 |

#### MATLAB Function Block 設定

**Trajectory Generator**：
```matlab
function p_d = trajectory_fcn(t, p0, params)
    p_d = trajectory_generator(t, p0, params);
end
```
- Sample Time: 1/1606
- 輸入: t (scalar), p0 (3×1), params (struct)
- 輸出: p_d (3×1)

**Controller**：
```matlab
function f_d = controller_fcn(p_d, p_m, params)
    f_d = motion_control_law(p_d, p_m, params);
end
```
- Sample Time: 1/1606
- 輸入: p_d, p_m (各 3×1), params (struct)
- 輸出: f_d (3×1)
- **注意**：motion_control_law 內部使用 persistent 變數管理 p_d_prev

**Thermal Force**：
```matlab
function F_th = thermal_fcn(p_m, params)
    if params.thermal.enable
        F_th = calc_thermal_force(p_m, params);
    else
        F_th = zeros(3, 1);
    end
end
```
- Sample Time: 1/1606
- 輸入: p_m (3×1), params (struct)
- 輸出: F_th (3×1)

**Particle Dynamics (Γ⁻¹·F)**：
```matlab
function p_dot = dynamics_fcn(F_total, p_m, params)
    [Gamma_inv, ~] = calc_gamma_inv(p_m, params);
    p_dot = Gamma_inv * F_total;
end
```
- Sample Time: -1 (inherited/continuous)
- 輸入: F_total (3×1), p_m (3×1), params (struct)
- 輸出: p_dot (3×1)

#### Solver 設定
| 參數 | 值 | 說明 |
|------|-----|------|
| Solver | ode45 | Variable-step，適合非剛性系統 |
| Type | Variable-step | - |
| Max Step Size | 6e-5 | 取樣時間的十分之一 (Ts/10) |
| Relative Tolerance | 1e-6 | 每步相對誤差 ≤ 0.0001% |

#### 初始化流程
```matlab
% run_simulation.m 中
params = calc_simulation_params(config);
p0 = calc_initial_position(params);

% 設為 base workspace 變數
assignin('base', 'params', params);
assignin('base', 'p0', p0);

% 執行模擬
simOut = sim('model/system_model');
```

#### 輸出記錄
使用 To Workspace blocks，格式：**Array**

| 信號名稱 | 維度 | 說明 |
|----------|------|------|
| `t_out` | 1×N | 時間向量 |
| `p_m_out` | 3×N | 量測位置軌跡 |
| `p_d_out` | 3×N | 期望位置軌跡 |
| `f_d_out` | 3×N | 控制力 |
| `F_th_out` | 3×N | 熱力雜訊 |
| `h_bar_out` | 1×N | 正規化距離（監控安全性）|

### 10.3 討論記錄
- Simulink 模型需要重新建立（現有空殼）
- 離散模組用 MATLAB Function + 指定 Sample Time
- 連續模組用 MATLAB Function (Sample Time = -1) + Integrator
- p_d_prev 由控制器內部 persistent 變數管理（不使用 Unit Delay）
- 所有位置信號統一命名為 p_m（量測位置）
- MATLAB Function block 直接呼叫外部函數（方案 A）
- Max Step Size = 6e-5（取樣時間的十分之一）
- 輸出格式使用 Array（非 Timeseries）
- 輸出信號：t, p_m, p_d, f_d, F_th, h_bar（不含 error）

---

## 階段 11：整合測試

### 11.1 討論項目
- [x] 測試情境
- [x] run_simulation.m 結構
- [x] 預期結果與驗證方式

### 11.2 最終決定

#### 測試情境

| 測試類型 | ctrl_enable | thermal_enable | 說明 |
|----------|-------------|----------------|------|
| 開迴路 | `false` | `true` | 無控制力，只有熱力干擾 |
| 閉迴路（無干擾）| `true` | `false` | 完整控制迴路，無干擾 |
| 閉迴路（有干擾）| `true` | `true` | 完整控制迴路 + 熱力干擾 |

#### run_simulation.m 結構

```matlab
%% run_simulation.m
clear; close all; clc;

%% SECTION 1: 高階參數配置
% === Wall 參數 ===
theta = 0;
phi = 0;
pz = 0;
h_bar_min = 1.5;

% === 軌跡參數 ===
traj_type = 'z_move';  % 'z_move' 或 'xy_circle'
h_margin = 5;
delta_z = 10;
direction = 'away';
speed = 5;
radius = 5;
period = 1;
n_circles = 3;

% === 控制器參數 ===
ctrl_enable = true;    % true=閉迴路, false=開迴路
lambda_c = 0.7;

% === Thermal Force ===
thermal_enable = true;

% === 模擬參數 ===
T_sim = 5;

%% SECTION 2: 打包配置 + 計算參數
config = struct(...
    'theta', theta, 'phi', phi, 'pz', pz, 'h_bar_min', h_bar_min, ...
    'traj_type', traj_type, 'h_margin', h_margin, ...
    'delta_z', delta_z, 'direction', direction, 'speed', speed, ...
    'radius', radius, 'period', period, 'n_circles', n_circles, ...
    'ctrl_enable', ctrl_enable, ...
    'lambda_c', lambda_c, ...
    'thermal_enable', thermal_enable, ...
    'T_sim', T_sim ...
);

% 加入路徑
addpath('model/wall_effect');
addpath('model/thermal_force');
addpath('model/trajectory');
addpath('model/controller');

% 計算參數
params = calc_simulation_params(config);

%% SECTION 3: 計算起始位置 + 安全檢查
p0 = calc_initial_position(params);
[is_safe, h_bar_min_actual, t_critical] = check_trajectory_safety(p0, params);

if ~is_safe
    warning('軌跡不安全！繼續執行但請檢查參數。');
end

%% SECTION 4: 執行模擬
% 載入到 base workspace
assignin('base', 'params', params);
assignin('base', 'p0', p0);

% 執行 Simulink 模擬
simOut = sim('model/system_model', 'StopTime', num2str(T_sim));

% 取出結果
t = simOut.t_out;
p_m = simOut.p_m_out;
p_d = simOut.p_d_out;
f_d = simOut.f_d_out;

%% SECTION 5: 結果繪圖
figure('Name', 'Simulation Results');

% 子圖 1：軌跡比較
subplot(2,2,1);
plot3(p_d(:,1), p_d(:,2), p_d(:,3), 'b-', 'LineWidth', 2);
hold on;
plot3(p_m(:,1), p_m(:,2), p_m(:,3), 'r--', 'LineWidth', 1.5);
legend('p_d (期望)', 'p_m (量測)');
xlabel('x [\mum]'); ylabel('y [\mum]'); zlabel('z [\mum]');
title('3D Trajectory');
grid on; axis equal; view(30, 30);

% 子圖 2：追蹤誤差
subplot(2,2,2);
error = vecnorm(p_m - p_d, 2, 2);
plot(t, error, 'k-', 'LineWidth', 1.5);
xlabel('Time [sec]'); ylabel('||e|| [\mum]');
title('Tracking Error');
grid on;

% 子圖 3：控制力
subplot(2,2,3);
plot(t, f_d(:,1), 'r-', t, f_d(:,2), 'g-', t, f_d(:,3), 'b-', 'LineWidth', 1.5);
legend('f_x', 'f_y', 'f_z');
xlabel('Time [sec]'); ylabel('Force [pN]');
title('Control Force');
grid on;

% 子圖 4：各軸位置
subplot(2,2,4);
plot(t, p_m(:,1), 'r-', t, p_m(:,2), 'g-', t, p_m(:,3), 'b-', 'LineWidth', 1.5);
legend('x', 'y', 'z');
xlabel('Time [sec]'); ylabel('Position [\mum]');
title('Position vs Time');
grid on;

%% SECTION 6: 儲存結果
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
output_dir = fullfile('test_results', 'simulation', ['sim_' timestamp]);
mkdir(output_dir);

savefig(fullfile(output_dir, 'results.fig'));
exportgraphics(gcf, fullfile(output_dir, 'results.png'), 'Resolution', 150);

result.t = t;
result.p_m = p_m;
result.p_d = p_d;
result.f_d = f_d;
result.params = params;
result.p0 = p0;
save(fullfile(output_dir, 'result.mat'), 'result');

fprintf('結果已儲存至: %s\n', output_dir);
```

#### 預期結果驗證

| 測試 | 預期結果 |
|------|----------|
| 開迴路 | p_m 不動（沒有控制力）|
| 閉迴路（無干擾）| p_m 追蹤 p_d，誤差趨近於 0 |
| 閉迴路（有干擾）| p_m 追蹤 p_d，有隨機波動 |

#### 驗證指標
- **追蹤誤差 RMSE**：應小於設定閾值
- **穩態誤差**：閉迴路應趨近於 0
- **控制力範圍**：應在合理範圍內（< 100 pN）

### 11.3 討論記錄
- 三種測試情境：開迴路、閉迴路無干擾、閉迴路有干擾
- 透過 `ctrl_enable` 和 `thermal_enable` 兩個開關切換測試模式
- 開迴路實現方式：控制器函數內判斷 `params.ctrl.enable`，若為 false 則返回 zeros(3,1)
- run_simulation.m 包含完整流程：參數設定→計算→模擬→繪圖→儲存
- 第一次實作以目視驗證為主，後續可加入數值驗證
- 結果繪圖 4 個子圖：3D軌跡、追蹤誤差、控制力、各軸位置

---

## 階段 12：Git 分支管理與開發流程

### 12.1 討論項目
- [x] 分支策略選擇
- [x] 開發順序與依賴關係
- [x] Commit 規範

### 12.2 目前分支狀況

```
origin/main ─────── 6458985 (初始化) ← 本地 main（目前位置，乾淨狀態）

origin/develop ──── 舊開發分支（不使用）
origin/develop2 ─── 舊開發分支（不使用）
```

**決定**：從 main 的乾淨狀態開始，不使用舊分支內容

### 12.3 分支策略

採用**單一 feature branch 順序開發**：

```
main
  │
  └── feature/phase1-implementation
         │
         ├── Step 1: 基礎設施 + 參數管理
         ├── Step 2: Wall Effect 模組 + 測試
         ├── Step 3: Thermal Force 模組 + 測試
         ├── Step 4: 軌跡產生器模組 + 測試
         ├── Step 5: 控制器模組
         ├── Step 6: Simulink 整合
         └── Step 7: 整合測試 + run_simulation.m
         │
         └── 完成後合併回 main
```

### 12.4 開發順序與依賴關係

```
Step 1: 基礎設施
├── 建立目錄結構
├── 更新 .gitignore
└── calc_simulation_params.m（所有模組的基礎）
        │
        ▼
Step 2: Wall Effect ←─────────────────────────┐
├── calc_correction_functions.m               │
├── calc_gamma_inv.m                          │ 可平行開發
└── run_wall_effect_test.m                    │ （但建議順序）
        │                                     │
        ▼                                     │
Step 3: Thermal Force                         │
├── calc_thermal_force.m（依賴 calc_correction_functions）
└── run_thermal_force_test.m                  │
        │                                     │
        ├─────────────────────────────────────┘
        ▼
Step 4: 軌跡產生器
├── calc_initial_position.m
├── trajectory_generator.m
├── check_trajectory_safety.m
└── run_trajectory_test.m
        │
        ▼
Step 5: 控制器
└── motion_control_law.m
        │
        ▼
Step 6: Simulink 整合
└── system_model.slx（重新建立）
        │
        ▼
Step 7: 整合測試
└── run_simulation.m
```

### 12.5 Commit 規範

遵循 CLAUDE.md 定義的格式：
```
<type>(<scope>): <subject>

Types: feat, fix, refactor, test, docs, chore
Scope: wall-effect, thermal-force, trajectory, controller, simulink, params
```

**建議的 Commit 點**：

| Step | Commit 訊息範例 |
|------|-----------------|
| 1a | `chore(init): Create directory structure` |
| 1b | `feat(params): Implement calc_simulation_params with Bus Object` |
| 2a | `feat(wall-effect): Implement correction functions` |
| 2b | `feat(wall-effect): Implement Gamma inverse calculation` |
| 2c | `test(wall-effect): Add wall effect validation test` |
| 3a | `feat(thermal-force): Implement thermal force generation` |
| 3b | `test(thermal-force): Add thermal force distribution test` |
| 4a | `feat(trajectory): Implement trajectory generator` |
| 4b | `feat(trajectory): Add safety check function` |
| 4c | `test(trajectory): Add trajectory visualization test` |
| 5 | `feat(controller): Implement motion control law` |
| 6 | `feat(simulink): Build system model` |
| 7 | `feat(simulation): Add run_simulation script` |

### 12.6 開發啟動指令

```bash
# 確保在 main 分支
git checkout main
git pull origin main

# 建立 feature branch
git checkout -b feature/phase1-implementation

# 開始開發...
```

### 12.7 完成後合併

```bash
# 確保所有測試通過後
git checkout main
git merge feature/phase1-implementation
git push origin main

# 可選：刪除 feature branch
git branch -d feature/phase1-implementation
```

### 12.8 實踐方式與檢查點

**檢查點設計**：每個 commit 點暫停讓用戶檢查

| Commit | 暫停點 | 用戶檢查內容 | 開發者雙重檢查 |
|--------|--------|-------------|---------------|
| 1a | 目錄結構建立後 | 確認目錄正確 | 對照計畫 + 獨立驗證 |
| 1b | calc_simulation_params.m 完成後 | 確認參數結構 | 對照計畫 + 測試呼叫 |
| 2a-c | Wall Effect 完成後 | 執行測試看圖 | 對照公式 + 數值驗證 |
| 3a-b | Thermal Force 完成後 | 執行測試看圖 | 對照統計理論 |
| 4a-c | Trajectory 完成後 | 執行測試看圖 | 對照幾何邏輯 |
| 5 | Controller 完成後 | 確認控制律 | 對照論文公式 |
| 6 | Simulink 完成後 | 確認模型架構 | 信號流檢查 |
| 7 | run_simulation.m 完成後 | 執行完整模擬 | 三種模式驗證 |

**雙重檢查方法**：
- 檢查 A（對照計畫）：函數簽名、params 欄位、輸出格式
- 檢查 B（獨立邏輯）：數學公式、邊界條件、單位、向量維度

### 12.9 討論記錄
- 從 main 乾淨狀態開始，不使用 develop/develop2 舊分支
- 採用單一 feature branch 順序開發
- 每個功能模組完成後做 commit
- 完成後合併回 main
- 每個 commit 點暫停讓用戶檢查
- 開發者執行雙重檢查：對照計畫 + 獨立邏輯驗證

---

## 遺漏檢查清單

### 數學模型
- [ ] c_∥, c_⊥ 公式係數正確性
- [ ] Γ⁻¹ 矩陣推導完整性
- [ ] Thermal Force 方差公式單位
- [ ] 控制器公式與論文一致

### 程式設計
- [ ] 所有函數輸入驗證
- [ ] 邊界條件處理
- [ ] 向量維度一致（column vector）
- [ ] 單位轉換正確

### 測試
- [ ] 每個模組對應測試
- [ ] 正常情況與邊界情況
- [ ] 測試輸出結構一致

### 整合
- [ ] Bus Object 與 Simulink 相容
- [ ] 離散/連續取樣時間
- [ ] 信號維度匹配

---

## 參考資料（數學模型摘要）

以下為先前討論確認的數學模型，供實作時參考：

## 系統架構總覽

### 簡化假設
- 內迴路完美：f_d = f_m（磁力控制無誤差）
- 量測完美：p_m = p（視覺追蹤無誤差、無延遲）

### 混合架構
- **離散部分 @ 1606 Hz**：軌跡產生、控制器、Thermal Force
- **連續部分**：Particle Dynamics（ODE solver）
- 離散訊號經 ZOH 送入連續系統

### 4 個模組
```
┌───────────┐      ┌────────────┐
│ 1. 軌跡   │─p_d─▶│ 2. 控制器  │─f_d─┐
│ Generator │      │ Controller │     │
└───────────┘      └────────────┘     │
      ▲                   ▲           │
      │              p_m  │           ▼
      │                   │    ┌─────────────┐
      │                   │    │ f_d + F_th  │
      │                   │    └──────┬──────┘
      │                   │           │
      │                   │    ┌──────▼──────┐
      │                   │    │ 4. Γ⁻¹(p)   │◀── 3. F_th (Thermal)
      │                   │    │ Particle    │
      │                   │    │ Dynamics    │
      │                   │    └──────┬──────┘
      │                   │           │
      └───────────────────┴───────────┘
                          p
```

### 單位系統
- 長度：μm
- 力：pN
- 時間：sec
- 拖曳係數：pN·sec/μm

---

## 討論階段規劃

### 階段 1：Wall Effect 數學模型
**目標**：確認 Γ⁻¹(p) 計算的所有細節

討論項目：
- [x] 1.1 座標系統定義
- [x] 1.2 傾斜平面參數 (θ, φ, p_z) 幾何意義
- [x] 1.3 法向量 ŵ 計算
- [x] 1.4 修正函數 c_∥(h̄), c_⊥(h̄) 公式確認
- [x] 1.5 Γ⁻¹(p) 矩陣推導
- [x] 1.6 邊界條件（h̄ → 1 時的處理）

### 階段 2：Thermal Force 數學模型
**目標**：確認布朗運動干擾的實作方式

討論項目：
- [x] 2.1 c_x, c_y, c_z 與 c_∥, c_⊥ 的關係
- [x] 2.2 Variance 矩陣計算
- [x] 2.3 離散 vs 連續：如何在連續系統中加入離散雜訊
- [x] 2.4 單位換算確認

### 階段 3：軌跡產生器設計
**目標**：定義軌跡類型與介面

討論項目：
- [x] 3.1 支援的軌跡類型
- [x] 3.2 起始位置自動生成
- [x] 3.3 軌跡安全檢查機制
- [x] 3.4 相對座標軌跡設計

### 階段 4：控制器設計
**目標**：定義控制器架構（可後續擴充）

討論項目：
- [x] 4.1 初始控制器類型（Position-Dependent Discrete-Time Motion Control）
- [x] 4.2 控制器輸入輸出定義
- [x] 4.3 可擴充架構設計（先簡化後擴充策略）

### 階段 5：Simulink 架構設計
**目標**：確定 Simulink 模型結構

討論項目：
- [x] 5.1 S-Function vs MATLAB Function 選擇（MATLAB Function + Integrator）
- [x] 5.2 各模組的 Simulink Block 設計
- [x] 5.3 Solver 設定（ode45, variable-step）
- [x] 5.4 信號連接方式（3×1 向量）
- [x] 5.5 初始化方式（p0 從 workspace）
- [x] 5.6 量測延遲處理（確認 d=0，無延遲）

### 階段 6：參數管理設計
**目標**：設計統一參數函數結構

討論項目：
- [x] 6.1 設計架構（雙層參數架構，參考 r_controller_package）
- [x] 6.2 params 結構定義（common, wall, traj, ctrl, thermal）
- [x] 6.3 參數傳遞方式（Bus Object）
- [x] 6.4 軌跡計算方式（即時計算 + 模擬前安全檢查）
- [x] 6.5 run_simulation.m 結構

### 階段 7：測試策略
**目標**：定義驗證方法（以圖形對比為主）

討論項目：
- [x] 7.1 測試對象（Wall Effect、Thermal Force、軌跡產生器）
- [x] 7.2 Wall Effect 測試圖設計
- [x] 7.3 Thermal Force 測試圖設計
- [x] 7.4 軌跡產生器測試圖設計
- [x] 7.5 測試輸出結構（參考 r_controller_package）
- [x] 7.6 測試腳本清單

---

## 討論記錄

### [階段 1] Wall Effect 數學模型
**狀態**：已完成

#### 1.1 座標系統
- 使用標準右手座標系 (x, y, z)
- 粒子位置 p = [p_x, p_y, p_z]ᵀ

#### 1.2 傾斜平面參數
- θ (theta)：方位角，x-y 平面旋轉角
- φ (phi)：仰角，與 z 軸夾角
- p_z：平面沿法向量方向的位移
- **模擬前設定，模擬中固定**

#### 1.3 正交基底向量
從角度 θ, φ 直接計算三個正交向量：
```matlab
% 法向量（垂直於牆面）
w_hat = [cos(theta)*sin(phi); sin(theta)*sin(phi); cos(phi)];

% 平行於牆面的兩個方向
u_hat = [-cos(theta)*cos(phi); -sin(theta)*cos(phi); sin(phi)];
v_hat = [sin(theta); -cos(theta); 0];
```
**已驗證**：ŵ·û = 0, ŵ·v̂ = 0, û·v̂ = 0

#### 1.4 修正函數
```
c_∥(h̄) = (1 - 9/16·(1/h̄) + 1/8·(1/h̄)³ - 45/256·(1/h̄)⁴ - 1/16·(1/h̄)⁵)⁻¹
c_⊥(h̄) = (1 - 9/8·(1/h̄) + 1/2·(1/h̄)³ - 57/100·(1/h̄)⁴ + 1/5·(1/h̄)⁵ + 7/200·(1/h̄)¹¹ - 1/25·(1/h̄)¹²)⁻¹
```

#### 1.5 Γ⁻¹(p) 計算
```matlab
h = dot(p, w_hat) - pz;  % 粒子到平面距離
h_bar = h / R;           % 正規化距離
W = w_hat * w_hat';      % 投影矩陣
coeff = (c_perp - c_para) / c_perp;
Gamma_inv = (1 / (gamma_N * c_para)) * (eye(3) - coeff * W);
```

#### 1.6 邊界條件
- **方案**：在軌跡規劃和初始位置設定時避免 h̄ ≤ 1
- 設定安全邊界（如 h̄_min = 1.5）

---

### [階段 2] Thermal Force 數學模型
**狀態**：已完成

#### 2.1 c_x, c_y, c_z 計算
將 c_∥ 和 c_⊥ 分配到世界座標系：
```matlab
C = c_para * (u_hat + v_hat) + c_perp * w_hat;
% C = [c_x; c_y; c_z]
```

#### 2.2 Variance 計算
```
Variance = (4·k_B·T·γ_N / Δt) · [c_x², c_y², c_z²]
```
- 對角矩陣形式，x, y, z 三個方向獨立

#### 2.3 Random Thermal Force 生成
```matlab
Std_dev = sqrt(Variance);
F_th = Std_dev .* randn(3,1);  % 離散 @ 1606 Hz
```

#### 2.4 單位系統（pN-μm-sec）
| 變數 | 值 | 單位 |
|------|-----|------|
| k_B | 1.3806503×10⁻⁵ | pN·μm/K |
| T | 310.15 | K |
| γ_N | 0.0425 | pN·sec/μm |
| Δt | 1/1606 | sec |

**注意**：k_B 需從 SI 單位轉換（×10¹⁸）

### [階段 3] 軌跡產生器設計
**狀態**：已完成

#### 3.1 支援的軌跡類型
| 類型 | 參數 | 說明 |
|------|------|------|
| z_move | delta_z, direction, speed | 沿法向量方向移動 |
| xy_circle | radius, n_circles, period | 在牆面平行平面內繞圓 |

#### 3.2 起始位置自動生成
```matlab
h_safe = R × h̄_min + h_margin
p0 = pz × ŵ + h_safe × ŵ
```
- `h_margin`：額外安全餘量（預設 5 μm）
- 起始位置自動在牆面「上方側」（法向量方向）

#### 3.3 軌跡安全檢查
- **方案 A**：生成前檢驗（採用）
- 計算所有軌跡點的 h̄，確保 h̄ ≥ h̄_min
- 不安全時提供修改建議

#### 3.4 相對座標軌跡設計
- 軌跡參數用**相對於起點**的方式設定
- `direction = 'away'`：遠離牆面（沿 +ŵ）
- `direction = 'toward'`：靠近牆面（沿 -ŵ）
- 圓形軌跡在 û-v̂ 平面內（平行於牆面）

#### 完整流程
```
設定牆面 → 設定軌跡參數 → 自動生成起點 → 生成軌跡 → 安全檢查
```

### [階段 4] 控制器設計
**狀態**：已完成

#### 4.1 控制器類型
採用 **Position-Dependent Discrete-Time Motion Control Law**
- 來源：Meng & Menq (2023) "Ultra-Precise High-Speed Untethered Manipulation"
- 論文方程式 17 的簡化版本

#### 4.2 控制律公式
```
f_d[k] = (γ / Δt) · {p_d[k] - λ_c · p_d[k-1] - (1 - λ_c) · p[k]}
```

| 變數 | 說明 |
|------|------|
| f_d[k] | 控制力輸出（3×1 向量）|
| p_d[k] | 當前期望位置 |
| p_d[k-1] | 前一時刻期望位置 |
| p[k] | 當前實際位置（量測值）|
| γ | 拖曳係數（純量或 3×1 向量）|
| λ_c | 閉迴路極點（0 < λ_c < 1）|
| Δt | 取樣週期（1/1606 sec）|

#### 4.3 設計策略：先簡化後擴充
**初始版本**：
- 使用固定 γ = γ_N（忽略 Wall Effect 對控制器的影響）
- 三軸使用相同拖曳係數

**擴充版本**（未來）：
- 可切換為位置相關 γ(p) = [γ_x, γ_y, γ_z]
- 根據 Wall Effect 即時更新

#### 4.4 輸入輸出定義
```matlab
function f_d = motion_control_law(p_d, p_d_prev, p, params)
% 輸入：
%   p_d      - 當前期望位置 (3×1)
%   p_d_prev - 前一時刻期望位置 (3×1)
%   p        - 當前實際位置 (3×1)
%   params   - 參數結構體
%     .gamma   - 拖曳係數 (純量 或 3×1)
%     .lambda_c - 閉迴路極點 (純量)
%     .Ts      - 取樣週期 (純量)
%
% 輸出：
%   f_d      - 控制力 (3×1) [pN]

    gamma = params.gamma;
    lambda_c = params.lambda_c;
    Ts = params.Ts;

    f_d = (gamma ./ Ts) .* (p_d - lambda_c * p_d_prev - (1 - lambda_c) * p);
end
```

#### 4.5 參數建議值
| 參數 | 值 | 說明 |
|------|-----|------|
| γ_N | 0.0425 pN·sec/μm | Stokes drag coefficient |
| λ_c | 0.5~0.9 | 越大追蹤越平滑但較慢 |
| Δt | 1/1606 sec | 取樣週期 |

#### 4.6 與原有程式碼的差異
原有程式碼分析：
- 有 "known wall" 和 "unknown wall" 兩種模式
- "known wall" 模式使用位置相關 γ_x, γ_y, γ_z
- "unknown wall" 模式使用固定 γ_N
- 包含大量未使用的 Wall Effect 計算程式碼

簡化後：
- 移除未使用的模式切換邏輯
- 保持介面一致性，方便未來擴充
- γ 可以是純量（三軸相同）或向量（三軸不同）

### [階段 5] Simulink 架構設計
**狀態**：進行中

#### 5.1 Block 類型選擇
- **離散模組**：MATLAB Function
- **連續模組**：MATLAB Function + Simulink Integrator
- **信號連接**：3×1 向量（簡單直接）

#### 5.2 Simulink 模型架構
```
┌─────────────────────────────────────────────────────────────────┐
│                    system_model.slx                             │
│                                                                 │
│  ┌───────────────┐     ┌───────────────┐                        │
│  │  Trajectory   │─p_d─▶│  Controller   │─f_d─┐                 │
│  │  Generator    │     │               │     │                 │
│  │ (MATLAB Func) │     │ (MATLAB Func) │     │                 │
│  │ Ts=1/1606     │     │ Ts=1/1606     │     │                 │
│  └───────┬───────┘     └───────▲───────┘     │                 │
│          │                     │             │                 │
│          │ p_d                 │ p           ▼                 │
│          │               ┌─────┴─────┐   ┌──────┐              │
│          │               │    ZOH    │   │  +   │◀── F_th      │
│          │               │           │   └──┬───┘   (3×1)      │
│          │               └─────▲─────┘      │                  │
│          │                     │            ▼ F_total          │
│          │                     │     ┌─────────────┐           │
│          │                     │     │  Γ⁻¹(p)·F   │           │
│          │                     │     │ (MATLAB Func)│           │
│          │                     │     │  連續       │           │
│          │                     │     └──────┬──────┘           │
│          │                     │            │ ṗ (3×1)          │
│          │                     │            ▼                  │
│          │                     │     ┌─────────────┐           │
│          │                     └─────│ Integrator  │           │
│          │                       p   │ 1/s         │           │
│          │                           └──────┬──────┘           │
│          │                                  │ p (3×1)          │
│          ▼                                  ▼                  │
│  ┌─────────────────────────────────────────────────┐           │
│  │               To Workspace / Scope              │           │
│  └─────────────────────────────────────────────────┘           │
└─────────────────────────────────────────────────────────────────┘
```

#### 5.3 各模組設定
| 模組 | Block 類型 | 取樣時間 | 輸入 | 輸出 |
|------|-----------|---------|------|------|
| Trajectory Generator | MATLAB Function | Ts = 1/1606 | t | p_d (3×1) |
| Controller | MATLAB Function | Ts = 1/1606 | p_d, p | f_d (3×1) |
| Thermal Force | MATLAB Function | Ts = 1/1606 | p | F_th (3×1) |
| Γ⁻¹(p)·F | MATLAB Function | 連續 (-1) | F_total, p | ṗ (3×1) |
| Integrator | Simulink Integrator | 連續 | ṗ | p |

#### 5.4 Solver 設定
| 參數 | 建議值 | 說明 |
|------|--------|------|
| Solver | ode45 | Variable-step，非剛性系統 |
| Max Step Size | 1e-4 sec | 離散/連續交界精度 |
| Relative Tolerance | 1e-6 | 相對誤差 |
| Absolute Tolerance | 1e-9 | 絕對誤差 |

#### 5.5 初始化
- p0 從 workspace 載入
- Thermal Force 使用 p 回授計算 h_bar
- 控制器內部狀態（p_d_prev 等）用程式碼暫存變數處理

#### 5.6 量測延遲
- 確認 d=0（無延遲）
- 控制器使用 p[k] 作為回授

### [階段 6] 參數管理設計
**狀態**：已完成

#### 6.1 設計架構（參考 r_controller_package）

**雙層參數架構**：
```
高階參數（使用者設定）
        │
        ▼
calc_simulation_params(config)  ← 計算衍生參數 + 建立 Bus Object
        │
        ▼
params 結構 + ParamsBus         ← 給 Simulink 使用
```

#### 6.2 params 結構定義

```
params
├── common                          ← 所有模組共用
│   ├── R         = 2.25            % 粒子半徑 [μm]（已修正）
│   ├── gamma_N   = 0.0425          % Stokes drag [pN·sec/μm]
│   ├── Ts        = 1/1606          % 取樣週期 [sec]
│   └── T_sim     = 5               % 模擬時間 [sec]
│
├── wall                            ← Wall Effect 模組
│   ├── theta, phi, pz              % 高階輸入
│   ├── h_bar_min                   % 高階輸入
│   ├── w_hat     = [3×1]           % 計算得出
│   ├── u_hat     = [3×1]           % 計算得出
│   └── v_hat     = [3×1]           % 計算得出
│
├── traj                            ← 軌跡產生器
│   ├── type, h_margin              % 高階輸入
│   ├── delta_z, direction, speed   % z_move 參數
│   └── radius, n_circles, period   % xy_circle 參數
│
├── ctrl                            ← 控制器
│   ├── lambda_c                    % 高階輸入
│   ├── gamma     = gamma_N         % 初始版本使用固定值
│   └── Ts                          % = common.Ts
│
└── thermal                         ← Thermal Force
    ├── enable                      % 高階輸入
    ├── k_B, T, Ts                  % 常數
    └── variance_coeff              % 預計算: 4·k_B·T·γ_N/Δt
```

#### 6.3 參數傳遞方式：Bus Object

採用與 r_controller_package 相同的做法：
- 在 `calc_simulation_params()` 中建立 `ParamsBus`（Simulink.Bus）
- 定義所有參數的 BusElement
- 用 `Simulink.Parameter` 包裝，設定 `DataType = 'Bus: ParamsBus'`
- Simulink 中用 Constant block 傳入整個 params

**優點**：
- 統一性高，所有參數一個端口傳入
- 已在 r_controller_package 驗證可行
- 支援 code generation

#### 6.4 軌跡計算方式

採用**即時計算**：
- Trajectory Generator 在每個時刻根據 t 和 params 計算 p_d
- 模擬前用獨立函數檢查軌跡安全性

#### 6.5 run_simulation.m 結構

```matlab
% run_simulation.m
%% SECTION 1: 高階參數配置（使用者修改這裡）
theta = 0; phi = 0; pz = 0; ...
traj_type = 'z_move'; ...
lambda_c = 0.7; ...

%% SECTION 2: 打包配置 + 計算參數
config = struct('theta', theta, ...);
params = calc_simulation_params(config);

%% SECTION 3: 計算起始位置 + 安全檢查
p0 = calc_initial_position(params);
check_trajectory_safety(params, p0);

%% SECTION 4: 載入參數 + 執行模擬
assignin('base', 'params', params);
assignin('base', 'p0', p0);
sim('system_model');

%% SECTION 5: 結果分析與繪圖
```

#### 6.6 各模組需要的參數

| 模組 | 需要的參數 |
|------|-----------|
| Trajectory Generator | params.traj.*, params.wall.(w_hat, u_hat, v_hat), p0 |
| Controller | params.ctrl.* |
| Thermal Force | params.thermal.*, params.wall.(w_hat, u_hat, v_hat) |
| Γ⁻¹(p)·F | params.wall.*, params.common.* |
| Integrator | p0 |

### [階段 7] 測試策略
**狀態**：已完成

#### 7.1 測試對象
只針對三個模組：Wall Effect、Thermal Force、軌跡產生器
（控制器暫不測試）

#### 7.2 Wall Effect 測試

**測試圖 1：修正函數曲線** (`correction_functions.png`)
- X 軸：h̄ (1.1 ~ 20)
- Y 軸：修正係數 c_∥, c_⊥
- 標註：h̄=1.5 安全線、h̄→∞ 漸近值
- 物理意義：驗證接近牆面時拖曳係數增大，且 c_⊥ > c_∥

**測試圖 2：Γ⁻¹ 特徵值 vs 位置** (`gamma_inv_eigenvalues.png`)
- X 軸：h̄
- Y 軸：特徵值 λ₁, λ₂, λ₃
- 標註：h̄→∞ 時的理論值 1/γ_N
- 物理意義：驗證移動度在各方向的變化

#### 7.3 Thermal Force 測試

**測試圖 1：分布直方圖** (`distribution_histogram.png`)
- 3 個子圖（F_x, F_y, F_z）
- 直方圖 + 疊加理論高斯曲線
- 標註：實測均值/標準差 vs 理論值
- 物理意義：驗證隨機力符合布朗運動統計特性

**測試圖 2：位置相關性比較** (`position_comparison.png`)
- 長條圖：h̄=10 vs h̄=1.5
- 三組長條：σ_x, σ_y, σ_z
- 物理意義：驗證接近牆面時垂直方向擾動增大

#### 7.4 軌跡產生器測試

**測試圖 1：z_move 軌跡** (`z_move_3d.png`)
- 3D 軌跡線 + 牆面平面（半透明）
- 起點/終點標記 + 法向量 ŵ 箭頭
- 物理意義：驗證軌跡沿法向量方向移動

**測試圖 2：xy_circle 軌跡** (`xy_circle_3d.png`)
- 3D 圓形軌跡 + 牆面平面
- û, v̂ 向量箭頭 + 起點標記
- 物理意義：驗證軌跡在牆面平行平面內

**測試圖 3：h̄(t) 安全性曲線** (`h_bar_safety.png`)
- X 軸：時間 t，Y 軸：h̄
- h̄_min 安全線（紅色虛線）
- 物理意義：驗證軌跡始終滿足安全距離

#### 7.5 測試輸出結構（參考 r_controller_package）

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

#### 7.6 測試腳本

| 腳本 | 輸出圖形 |
|------|---------|
| `run_wall_effect_test.m` | correction_functions.png, gamma_inv_eigenvalues.png |
| `run_thermal_force_test.m` | distribution_histogram.png, position_comparison.png |
| `run_trajectory_test.m` | z_move_3d.png, xy_circle_3d.png, h_bar_safety.png |

---

## 最終實作計畫

### 實作順序

#### Step 1：專案基礎設施
- [ ] 建立目錄結構
- [ ] 更新 CLAUDE.md
- [ ] 更新 .gitignore（加入 test_results/）

#### Step 2：參數管理
- [ ] `calc_simulation_params.m` - 參數計算 + Bus Object 建立
- [ ] `calc_initial_position.m` - 計算起始位置 p0

#### Step 3：Wall Effect 模組
- [ ] `model/wall_effect/calc_correction_functions.m` - c_∥, c_⊥ 計算
- [ ] `model/wall_effect/calc_gamma_inv.m` - Γ⁻¹ 矩陣計算
- [ ] `test_script/run_wall_effect_test.m` - 測試腳本

#### Step 4：Thermal Force 模組
- [ ] `model/thermal_force/calc_thermal_force.m` - F_th 生成
- [ ] `test_script/run_thermal_force_test.m` - 測試腳本

#### Step 5：軌跡產生器模組
- [ ] `model/trajectory/trajectory_generator.m` - 即時軌跡計算
- [ ] `model/trajectory/check_trajectory_safety.m` - 安全檢查
- [ ] `test_script/run_trajectory_test.m` - 測試腳本

#### Step 6：控制器模組
- [ ] `model/controller/motion_control_law.m` - 控制律

#### Step 7：Simulink 整合
- [ ] `model/system_model.slx` - 主模型
- [ ] `run_simulation.m` - 執行腳本

#### Step 8：整合測試
- [ ] 開迴路測試
- [ ] 閉迴路測試

### 檔案結構

```
MotionControl_Simu/
├── CLAUDE.md                         # 專案規範（更新）
├── calc_simulation_params.m          # 參數計算 + Bus Object
├── calc_initial_position.m           # 起始位置計算
├── run_simulation.m                  # 主執行腳本
│
├── model/
│   ├── wall_effect/
│   │   ├── calc_correction_functions.m
│   │   └── calc_gamma_inv.m
│   │
│   ├── thermal_force/
│   │   └── calc_thermal_force.m
│   │
│   ├── trajectory/
│   │   ├── trajectory_generator.m
│   │   └── check_trajectory_safety.m
│   │
│   ├── controller/
│   │   └── motion_control_law.m
│   │
│   └── system_model.slx
│
├── test_script/
│   ├── run_wall_effect_test.m
│   ├── run_thermal_force_test.m
│   └── run_trajectory_test.m
│
├── test_results/                     # 測試結果（不納入 Git）
│   ├── wall_effect/
│   ├── thermal_force/
│   └── trajectory/
│
└── reference/
    ├── Drag_MATLAB.pdf
    ├── total_block_diagram.png
    └── thermal_force.png
```

---

## CLAUDE.md 更新內容

實作時需要更新 CLAUDE.md，加入以下內容：

### 專案結構（更新）
```
MotionControl_Simu/
├── CLAUDE.md
├── calc_simulation_params.m          # 參數計算 + Bus Object
├── calc_initial_position.m           # 起始位置計算
├── run_simulation.m                  # 主執行腳本
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
│   └── trajectory/
│
└── reference/
```

### 數學模型概要（更新）

#### 系統方程
```
ṗ = Γ⁻¹(p) · (f_d + F_th)
p = ∫ ṗ dt
```

#### Wall Effect
- 修正函數：c_∥(h̄), c_⊥(h̄)
- 移動度矩陣：Γ⁻¹ = (1/γ_N·c_∥) · (I - coeff·W)
- 其中 coeff = (c_⊥ - c_∥) / c_⊥, W = ŵŵᵀ

#### Thermal Force
- 均值：0
- 方差：(4·k_B·T·γ_N / Δt) · [c_x², c_y², c_z²]
- 離散生成 @ 1606 Hz

#### 控制器
```
f_d[k] = (γ / Δt) · {p_d[k] - λ_c · p_d[k-1] - (1 - λ_c) · p[k]}
```

### 單位系統
| 物理量 | 單位 |
|--------|------|
| 長度 | μm |
| 力 | pN |
| 時間 | sec |
| 拖曳係數 | pN·sec/μm |
| k_B | 1.3806503×10⁻⁵ pN·μm/K |

---

## 參考文件
- reference/Drag_MATLAB.pdf - Wall Effect 數學模型
- reference/total_block_diagram.png - 完整系統架構圖
- reference/thermal_force.png - Thermal Force 公式
- r_controller_package - 參數管理與測試輸出架構參考
- Meng & Menq (2023) - Ultra-Precise High-Speed Untethered Manipulation - 控制器設計來源
