# MotionControl_Simu 改進計畫

## 文件目的
此文件詳細記錄目前實作與計畫的偏差，並提供具體的修正方案。

---

## 一、問題總覽

### 1.1 已確認的偏差

| 編號 | 問題 | 嚴重程度 | 狀態 |
|------|------|----------|------|
| P1 | 參數管理過度工程化（3 檔案應為 1 檔案）| 中 | 待修正 |
| P2 | Simulink 模型 MATLAB Function blocks 無程式碼 | 高 | 待修正 |
| P3 | run_simulation.m 使用純 MATLAB 而非 Simulink | 高 | 待修正 |
| P4 | 存在計畫外的額外檔案 | 低 | 待清理 |

### 1.2 現有檔案狀態

**符合計畫的檔案**（無需修改）：
```
model/wall_effect/calc_correction_functions.m  ✓
model/wall_effect/calc_gamma_inv.m             ✓
model/thermal_force/calc_thermal_force.m       ✓
model/trajectory/calc_initial_position.m       ✓
model/trajectory/trajectory_generator.m        ✓
model/trajectory/check_trajectory_safety.m     ✓
model/controller/motion_control_law.m          ✓
test_script/run_wall_effect_test.m             ✓
test_script/run_thermal_force_test.m           ✓
test_script/run_trajectory_test.m              ✓
```

**需要修改的檔案**：
```
calc_simulation_params.m      → 需整合 Bus Object 創建
run_simulation.m              → 需改用 sim('model/system_model')
model/system_model.slx        → 需配置 MATLAB Function blocks
```

**需要刪除的檔案**：
```
create_simulation_buses.m     → 功能應整合到 calc_simulation_params.m
convert_params_for_simulink.m → 功能應整合到 calc_simulation_params.m
build_system_model.m          → 計畫外檔案
run_simulation_slx.m          → 計畫外檔案
```

---

## 二、問題 P1：參數管理整合

### 2.1 問題描述

**計畫規定**（計畫第 79 行）：
```
calc_simulation_params.m          # 參數計算 + Bus Object
```

**實際狀況**：
- `calc_simulation_params.m`：只有參數計算
- `create_simulation_buses.m`：Bus Object 創建（獨立檔案）
- `convert_params_for_simulink.m`：字串轉數值（獨立檔案）

### 2.2 參考實作：r_controller_package

r_controller_package 的 `r_controller_calc_params.m` 結構：

```matlab
function params = r_controller_calc_params(fB_c, fB_e, fB_f)
    % ========================================
    % 1. 計算所有參數
    % ========================================
    params.k_o = 5.6695e-4;
    params.b = 0.9782;
    % ... 其他參數計算 ...

    % ========================================
    % 2. 建立 Bus Object 定義
    % ========================================
    ParamsBus = Simulink.Bus;
    ParamsBus.Description = 'R Controller Parameters Structure';

    elems(1) = Simulink.BusElement;
    elems(1).Name = 'k_o';
    elems(1).DataType = 'double';
    % ... 其他元素 ...

    ParamsBus.Elements = elems;
    assignin('base', 'ParamsBus', ParamsBus);

    % ========================================
    % 3. 包裝為 Simulink.Parameter
    % ========================================
    params_data = params;
    params = Simulink.Parameter(params_data);
    params.DataType = 'Bus: ParamsBus';
end
```

### 2.3 修正方案

**修改 `calc_simulation_params.m`**，整合以下功能：

1. **參數計算**（保留現有邏輯）
2. **字串轉數值**（從 convert_params_for_simulink.m 移入）
3. **Bus Object 創建**（從 create_simulation_buses.m 移入）
4. **Simulink.Parameter 包裝**

### 2.4 詳細實作規格

```matlab
function params = calc_simulation_params(config)
%CALC_SIMULATION_PARAMS 計算模擬參數並建立 Simulink Bus Object
%
%   params = calc_simulation_params(config)
%
%   此函數執行：
%   1. 從 config 計算所有模擬參數
%   2. 建立 Simulink Bus Object 定義
%   3. 將字串參數轉換為數值（Simulink 相容）
%   4. 返回 Simulink.Parameter 物件

    %% ========================================
    %% SECTION 1: 固定物理常數
    %% ========================================
    R = 2.25;                    % 粒子半徑 [um]
    gamma_N = 0.0425;            % Stokes drag [pN*sec/um]
    Ts = 1/1606;                 % 取樣週期 [sec]
    k_B = 1.3806503e-5;          % 波茲曼常數 [pN*um/K]
    T_temp = 310.15;             % 溫度 [K]

    %% ========================================
    %% SECTION 2: 預設值處理
    %% ========================================
    defaults = struct(...
        'theta', 0, 'phi', 0, 'pz', 0, 'h_bar_min', 1.5, ...
        'traj_type', 'z_move', 'h_margin', 5, ...
        'delta_z', 10, 'direction', 'away', 'speed', 5, ...
        'radius', 5, 'period', 1, 'n_circles', 3, ...
        'ctrl_enable', true, 'lambda_c', 0.7, ...
        'thermal_enable', true, 'T_sim', 5 ...
    );

    if nargin < 1 || isempty(config)
        config = struct();
    end

    fields = fieldnames(defaults);
    for i = 1:length(fields)
        if ~isfield(config, fields{i})
            config.(fields{i}) = defaults.(fields{i});
        end
    end

    %% ========================================
    %% SECTION 3: 計算衍生參數
    %% ========================================

    % --- common 子結構 ---
    params_data.common.R = R;
    params_data.common.gamma_N = gamma_N;
    params_data.common.Ts = Ts;
    params_data.common.T_sim = config.T_sim;

    % --- wall 子結構 ---
    theta = config.theta;
    phi = config.phi;

    params_data.wall.theta = theta;
    params_data.wall.phi = phi;
    params_data.wall.pz = config.pz;
    params_data.wall.h_bar_min = config.h_bar_min;
    params_data.wall.w_hat = [cos(theta)*sin(phi); sin(theta)*sin(phi); cos(phi)];
    params_data.wall.u_hat = [-cos(theta)*cos(phi); -sin(theta)*cos(phi); sin(phi)];
    params_data.wall.v_hat = [sin(theta); -cos(theta); 0];

    % --- traj 子結構（數值編碼）---
    % type: 'z_move' -> 0, 'xy_circle' -> 1
    switch config.traj_type
        case 'z_move'
            params_data.traj.type = 0;
        case 'xy_circle'
            params_data.traj.type = 1;
        otherwise
            error('Unknown trajectory type: %s', config.traj_type);
    end

    params_data.traj.h_margin = config.h_margin;
    params_data.traj.delta_z = config.delta_z;

    % direction: 'away' -> 0, 'toward' -> 1
    switch config.direction
        case 'away'
            params_data.traj.direction = 0;
        case 'toward'
            params_data.traj.direction = 1;
        otherwise
            error('Unknown direction: %s', config.direction);
    end

    params_data.traj.speed = config.speed;
    params_data.traj.radius = config.radius;
    params_data.traj.period = config.period;
    params_data.traj.n_circles = config.n_circles;

    % --- ctrl 子結構 ---
    params_data.ctrl.enable = double(config.ctrl_enable);  % 轉為 double
    params_data.ctrl.lambda_c = config.lambda_c;
    params_data.ctrl.gamma = gamma_N;
    params_data.ctrl.Ts = Ts;

    % --- thermal 子結構 ---
    params_data.thermal.enable = double(config.thermal_enable);  % 轉為 double
    params_data.thermal.k_B = k_B;
    params_data.thermal.T = T_temp;
    params_data.thermal.Ts = Ts;
    params_data.thermal.variance_coeff = 4 * k_B * T_temp * gamma_N / Ts;

    %% ========================================
    %% SECTION 4: 建立 Nested Bus Objects
    %% ========================================

    % --- CommonBus ---
    elems_common = Simulink.BusElement.empty(0, 4);
    elems_common(1) = Simulink.BusElement; elems_common(1).Name = 'R';
    elems_common(1).Dimensions = [1 1]; elems_common(1).DataType = 'double';
    elems_common(2) = Simulink.BusElement; elems_common(2).Name = 'gamma_N';
    elems_common(2).Dimensions = [1 1]; elems_common(2).DataType = 'double';
    elems_common(3) = Simulink.BusElement; elems_common(3).Name = 'Ts';
    elems_common(3).Dimensions = [1 1]; elems_common(3).DataType = 'double';
    elems_common(4) = Simulink.BusElement; elems_common(4).Name = 'T_sim';
    elems_common(4).Dimensions = [1 1]; elems_common(4).DataType = 'double';

    CommonBus = Simulink.Bus;
    CommonBus.Elements = elems_common;
    assignin('base', 'CommonBus', CommonBus);

    % --- WallBus ---
    elems_wall = Simulink.BusElement.empty(0, 7);
    elems_wall(1) = Simulink.BusElement; elems_wall(1).Name = 'theta';
    elems_wall(1).Dimensions = [1 1]; elems_wall(1).DataType = 'double';
    elems_wall(2) = Simulink.BusElement; elems_wall(2).Name = 'phi';
    elems_wall(2).Dimensions = [1 1]; elems_wall(2).DataType = 'double';
    elems_wall(3) = Simulink.BusElement; elems_wall(3).Name = 'pz';
    elems_wall(3).Dimensions = [1 1]; elems_wall(3).DataType = 'double';
    elems_wall(4) = Simulink.BusElement; elems_wall(4).Name = 'h_bar_min';
    elems_wall(4).Dimensions = [1 1]; elems_wall(4).DataType = 'double';
    elems_wall(5) = Simulink.BusElement; elems_wall(5).Name = 'w_hat';
    elems_wall(5).Dimensions = [3 1]; elems_wall(5).DataType = 'double';
    elems_wall(6) = Simulink.BusElement; elems_wall(6).Name = 'u_hat';
    elems_wall(6).Dimensions = [3 1]; elems_wall(6).DataType = 'double';
    elems_wall(7) = Simulink.BusElement; elems_wall(7).Name = 'v_hat';
    elems_wall(7).Dimensions = [3 1]; elems_wall(7).DataType = 'double';

    WallBus = Simulink.Bus;
    WallBus.Elements = elems_wall;
    assignin('base', 'WallBus', WallBus);

    % --- TrajBus ---
    elems_traj = Simulink.BusElement.empty(0, 8);
    elems_traj(1) = Simulink.BusElement; elems_traj(1).Name = 'type';
    elems_traj(1).Dimensions = [1 1]; elems_traj(1).DataType = 'double';
    elems_traj(2) = Simulink.BusElement; elems_traj(2).Name = 'h_margin';
    elems_traj(2).Dimensions = [1 1]; elems_traj(2).DataType = 'double';
    elems_traj(3) = Simulink.BusElement; elems_traj(3).Name = 'delta_z';
    elems_traj(3).Dimensions = [1 1]; elems_traj(3).DataType = 'double';
    elems_traj(4) = Simulink.BusElement; elems_traj(4).Name = 'direction';
    elems_traj(4).Dimensions = [1 1]; elems_traj(4).DataType = 'double';
    elems_traj(5) = Simulink.BusElement; elems_traj(5).Name = 'speed';
    elems_traj(5).Dimensions = [1 1]; elems_traj(5).DataType = 'double';
    elems_traj(6) = Simulink.BusElement; elems_traj(6).Name = 'radius';
    elems_traj(6).Dimensions = [1 1]; elems_traj(6).DataType = 'double';
    elems_traj(7) = Simulink.BusElement; elems_traj(7).Name = 'period';
    elems_traj(7).Dimensions = [1 1]; elems_traj(7).DataType = 'double';
    elems_traj(8) = Simulink.BusElement; elems_traj(8).Name = 'n_circles';
    elems_traj(8).Dimensions = [1 1]; elems_traj(8).DataType = 'double';

    TrajBus = Simulink.Bus;
    TrajBus.Elements = elems_traj;
    assignin('base', 'TrajBus', TrajBus);

    % --- CtrlBus ---
    elems_ctrl = Simulink.BusElement.empty(0, 4);
    elems_ctrl(1) = Simulink.BusElement; elems_ctrl(1).Name = 'enable';
    elems_ctrl(1).Dimensions = [1 1]; elems_ctrl(1).DataType = 'double';
    elems_ctrl(2) = Simulink.BusElement; elems_ctrl(2).Name = 'lambda_c';
    elems_ctrl(2).Dimensions = [1 1]; elems_ctrl(2).DataType = 'double';
    elems_ctrl(3) = Simulink.BusElement; elems_ctrl(3).Name = 'gamma';
    elems_ctrl(3).Dimensions = [1 1]; elems_ctrl(3).DataType = 'double';
    elems_ctrl(4) = Simulink.BusElement; elems_ctrl(4).Name = 'Ts';
    elems_ctrl(4).Dimensions = [1 1]; elems_ctrl(4).DataType = 'double';

    CtrlBus = Simulink.Bus;
    CtrlBus.Elements = elems_ctrl;
    assignin('base', 'CtrlBus', CtrlBus);

    % --- ThermalBus ---
    elems_thermal = Simulink.BusElement.empty(0, 5);
    elems_thermal(1) = Simulink.BusElement; elems_thermal(1).Name = 'enable';
    elems_thermal(1).Dimensions = [1 1]; elems_thermal(1).DataType = 'double';
    elems_thermal(2) = Simulink.BusElement; elems_thermal(2).Name = 'k_B';
    elems_thermal(2).Dimensions = [1 1]; elems_thermal(2).DataType = 'double';
    elems_thermal(3) = Simulink.BusElement; elems_thermal(3).Name = 'T';
    elems_thermal(3).Dimensions = [1 1]; elems_thermal(3).DataType = 'double';
    elems_thermal(4) = Simulink.BusElement; elems_thermal(4).Name = 'Ts';
    elems_thermal(4).Dimensions = [1 1]; elems_thermal(4).DataType = 'double';
    elems_thermal(5) = Simulink.BusElement; elems_thermal(5).Name = 'variance_coeff';
    elems_thermal(5).Dimensions = [1 1]; elems_thermal(5).DataType = 'double';

    ThermalBus = Simulink.Bus;
    ThermalBus.Elements = elems_thermal;
    assignin('base', 'ThermalBus', ThermalBus);

    % --- ParamsBus（父層）---
    elems_params = Simulink.BusElement.empty(0, 5);
    elems_params(1) = Simulink.BusElement; elems_params(1).Name = 'common';
    elems_params(1).Dimensions = [1 1]; elems_params(1).DataType = 'Bus: CommonBus';
    elems_params(2) = Simulink.BusElement; elems_params(2).Name = 'wall';
    elems_params(2).Dimensions = [1 1]; elems_params(2).DataType = 'Bus: WallBus';
    elems_params(3) = Simulink.BusElement; elems_params(3).Name = 'traj';
    elems_params(3).Dimensions = [1 1]; elems_params(3).DataType = 'Bus: TrajBus';
    elems_params(4) = Simulink.BusElement; elems_params(4).Name = 'ctrl';
    elems_params(4).Dimensions = [1 1]; elems_params(4).DataType = 'Bus: CtrlBus';
    elems_params(5) = Simulink.BusElement; elems_params(5).Name = 'thermal';
    elems_params(5).Dimensions = [1 1]; elems_params(5).DataType = 'Bus: ThermalBus';

    ParamsBus = Simulink.Bus;
    ParamsBus.Description = 'Motion Control Simulation Parameters';
    ParamsBus.Elements = elems_params;
    assignin('base', 'ParamsBus', ParamsBus);

    %% ========================================
    %% SECTION 5: 包裝為 Simulink.Parameter
    %% ========================================
    params = Simulink.Parameter(params_data);
    params.DataType = 'Bus: ParamsBus';
    params.Description = 'Motion Control Parameters for Simulink';

end
```

---

## 三、問題 P2：Simulink MATLAB Function Blocks 配置

### 3.1 問題描述

Simulink MATLAB Function blocks 無法透過程式自動填入程式碼，需要手動配置。

### 3.2 參考實作：r_controller_package

在 r_controller_package 中，MATLAB Function block 的程式碼格式：

```matlab
function [u, u_w1] = fcn(vd, vm, params)
    [u, u_w1] = r_controller_function_general(vd, vm, params);
end
```

**重點**：block 內只是簡單的 wrapper，呼叫外部 .m 函數。

### 3.3 MotionControl_Simu 的 MATLAB Function Blocks 設計

根據計畫第 1386-1432 行，需要 4 個 MATLAB Function blocks：

#### Block 1: Trajectory_Generator

**位置**：接收 Clock 輸出的 t
**Sample Time**：1/1606
**程式碼**：
```matlab
function p_d = fcn(t, p0, params)
%#codegen
    % 將數值類型轉回字串（給 trajectory_generator 使用）
    params_m = convert_params_to_matlab(params);
    p_d = trajectory_generator(t, p0, params_m);
end
```

#### Block 2: Controller

**位置**：接收 p_d 和 p_m
**Sample Time**：1/1606
**程式碼**：
```matlab
function f_d = fcn(p_d, p_m, params)
%#codegen
    % 將 enable 轉為 logical
    params_m = params;
    params_m.ctrl.enable = logical(params.ctrl.enable);
    f_d = motion_control_law(p_d, p_m, params_m);
end
```

#### Block 3: Thermal_Force

**位置**：接收 p_m
**Sample Time**：1/1606
**程式碼**：
```matlab
function F_th = fcn(p_m, params)
%#codegen
    if params.thermal.enable > 0.5
        % 將數值類型轉回內部格式
        params_m = convert_params_to_matlab(params);
        F_th = calc_thermal_force(p_m, params_m);
    else
        F_th = zeros(3, 1);
    end
end
```

#### Block 4: Particle_Dynamics

**位置**：接收 F_total 和 p_m
**Sample Time**：-1（連續）
**程式碼**：
```matlab
function p_dot = fcn(F_total, p_m, params)
%#codegen
    params_m = convert_params_to_matlab(params);
    [Gamma_inv, ~] = calc_gamma_inv(p_m, params_m);
    p_dot = Gamma_inv * F_total;
end
```

### 3.4 需要新增的輔助函數

**`convert_params_to_matlab.m`**（放在根目錄）：
```matlab
function params_m = convert_params_to_matlab(params)
%CONVERT_PARAMS_TO_MATLAB 將 Simulink 數值參數轉回 MATLAB 格式
%
%   將數值編碼的參數轉回字串格式，供 MATLAB 函數使用。
%
%   轉換規則：
%   - traj.type: 0 -> 'z_move', 1 -> 'xy_circle'
%   - traj.direction: 0 -> 'away', 1 -> 'toward'
%   - ctrl.enable: 0 -> false, 1 -> true
%   - thermal.enable: 0 -> false, 1 -> true

    params_m = params;

    % traj.type
    if params.traj.type == 0
        params_m.traj.type = 'z_move';
    else
        params_m.traj.type = 'xy_circle';
    end

    % traj.direction
    if params.traj.direction == 0
        params_m.traj.direction = 'away';
    else
        params_m.traj.direction = 'toward';
    end

    % ctrl.enable
    params_m.ctrl.enable = logical(params.ctrl.enable);

    % thermal.enable
    params_m.thermal.enable = logical(params.thermal.enable);
end
```

### 3.5 手動配置步驟

1. 在 MATLAB 中開啟 `model/system_model.slx`
2. 雙擊每個 MATLAB Function block
3. 貼入對應的程式碼
4. 設定 Sample Time（在 block properties 中）
5. 儲存模型

---

## 四、問題 P3：run_simulation.m 修正

### 4.1 問題描述

**計畫規定**（第 1560-1572 行）：
```matlab
% 執行 Simulink 模擬
simOut = sim('model/system_model', 'StopTime', num2str(T_sim));
% 取出結果
t = simOut.t_out;
p_m = simOut.p_m_out;
p_d = simOut.p_d_out;
f_d = simOut.f_d_out;
```

**實際狀況**：使用純 MATLAB for 迴圈 + Euler 積分。

### 4.2 修正後的 SECTION 4

```matlab
%% SECTION 4: Run Simulation
fprintf('\nStarting simulation...\n');
fprintf('  Mode: %s\n', ternary(ctrl_enable, 'Closed-loop', 'Open-loop'));
fprintf('  Thermal: %s\n', ternary(thermal_enable, 'Enabled', 'Disabled'));
fprintf('  Trajectory: %s\n', traj_type);
fprintf('  Duration: %.1f sec\n', T_sim);

% 載入模型
model_name = 'system_model';
model_path = fullfile('model', [model_name '.slx']);

if ~exist(model_path, 'file')
    error('Model file not found: %s', model_path);
end

if ~bdIsLoaded(model_name)
    load_system(model_path);
end

% 重置控制器 persistent 變數
clear motion_control_law

% 載入參數到 base workspace
assignin('base', 'params', params);
assignin('base', 'p0', p0);

% 設定模擬器參數
set_param(model_name, 'StopTime', num2str(T_sim));
set_param(model_name, 'Solver', 'ode45');
set_param(model_name, 'MaxStep', num2str(params.common.Ts / 10));
set_param(model_name, 'RelTol', '1e-6');

% 執行模擬
fprintf('  Running Simulink simulation...\n');
tic;
simOut = sim(model_name);
elapsed = toc;
fprintf('  Simulation completed in %.2f seconds.\n', elapsed);

% 取出結果
t = simOut.tout;
p_m = simOut.p_m_out';  % 轉置為 3×N
p_d = simOut.p_d_out';
f_d = simOut.f_d_out';

% 關閉模型
close_system(model_name, 0);
```

---

## 五、問題 P4：清理多餘檔案

### 5.1 需刪除的檔案

| 檔案 | 原因 |
|------|------|
| `create_simulation_buses.m` | 功能已整合到 calc_simulation_params.m |
| `convert_params_for_simulink.m` | 功能已整合到 calc_simulation_params.m |
| `build_system_model.m` | 計畫外檔案 |
| `run_simulation_slx.m` | 計畫外檔案 |

### 5.2 刪除順序

1. 先完成 calc_simulation_params.m 整合
2. 確認 Simulink 模型可正常運作
3. 確認 run_simulation.m 可使用 sim() 執行
4. 刪除多餘檔案
5. 執行測試確認

---

## 六、Simulink 模型架構詳細規格

### 6.1 Block 清單

| Block 名稱 | 類型 | 位置 | 輸入 | 輸出 |
|-----------|------|------|------|------|
| Clock | simulink/Sources/Clock | [50, 200] | - | t |
| params_const | simulink/Sources/Constant | [50, 50] | - | params |
| p0_const | simulink/Sources/Constant | [50, 130] | - | p0 |
| Trajectory_Generator | MATLAB Function | [200, 170] | t, p0, params | p_d |
| Controller | MATLAB Function | [450, 170] | p_d, p_m, params | f_d |
| Thermal_Force | MATLAB Function | [450, 300] | p_m, params | F_th |
| Sum_Forces | simulink/Math Operations/Add | [700, 220] | f_d, F_th | F_total |
| Particle_Dynamics | MATLAB Function | [800, 200] | F_total, p_m, params | p_dot |
| Integrator | simulink/Continuous/Integrator | [1020, 220] | p_dot | p_m |
| p_m_out | simulink/Sinks/To Workspace | [1150, 230] | p_m | - |
| p_d_out | simulink/Sinks/To Workspace | [450, 100] | p_d | - |
| f_d_out | simulink/Sinks/To Workspace | [700, 140] | f_d | - |

### 6.2 連線清單

| 來源 | 目標 |
|------|------|
| Clock/1 | Trajectory_Generator/1 |
| p0_const/1 | Trajectory_Generator/2 |
| params_const/1 | Trajectory_Generator/3 |
| Trajectory_Generator/1 | Controller/1 |
| Trajectory_Generator/1 | p_d_out/1 |
| params_const/1 | Controller/3 |
| params_const/1 | Thermal_Force/2 |
| params_const/1 | Particle_Dynamics/3 |
| Controller/1 | Sum_Forces/1 |
| Controller/1 | f_d_out/1 |
| Thermal_Force/1 | Sum_Forces/2 |
| Sum_Forces/1 | Particle_Dynamics/1 |
| Particle_Dynamics/1 | Integrator/1 |
| Integrator/1 | p_m_out/1 |
| Integrator/1 | Controller/2 |
| Integrator/1 | Thermal_Force/1 |
| Integrator/1 | Particle_Dynamics/2 |

### 6.3 Solver 設定

| 參數 | 值 |
|------|-----|
| Solver | ode45 |
| Type | Variable-step |
| Max Step Size | 6e-5 (Ts/10) |
| Relative Tolerance | 1e-6 |
| Stop Time | T_sim (from workspace) |

### 6.4 Constant Block 設定

**params_const**:
- Value: `params`（從 workspace 讀取）
- OutDataTypeStr: `Bus: ParamsBus`
- SampleTime: `inf`

**p0_const**:
- Value: `p0`（從 workspace 讀取）
- SampleTime: `inf`

### 6.5 Integrator 設定

- Initial Condition: `p0`（從 workspace 讀取）
- Initial Condition Source: `external` 或 `internal` 皆可

---

## 七、執行計畫

### 7.1 Phase 1：參數管理整合

1. [ ] 備份現有 calc_simulation_params.m
2. [ ] 重寫 calc_simulation_params.m（包含 Bus Object）
3. [ ] 新增 convert_params_to_matlab.m
4. [ ] 測試參數產生是否正確

### 7.2 Phase 2：Simulink 模型配置

1. [ ] 開啟 model/system_model.slx
2. [ ] 配置 params_const block（設定 Bus 類型）
3. [ ] 配置 4 個 MATLAB Function blocks 程式碼
4. [ ] 設定各 block 的 Sample Time
5. [ ] 儲存模型
6. [ ] 測試模型載入無錯誤

### 7.3 Phase 3：run_simulation.m 修正

1. [ ] 備份現有 run_simulation.m
2. [ ] 修改 SECTION 4 使用 sim()
3. [ ] 測試完整模擬流程

### 7.4 Phase 4：清理

1. [ ] 確認所有功能正常
2. [ ] 刪除 create_simulation_buses.m
3. [ ] 刪除 convert_params_for_simulink.m
4. [ ] 刪除 build_system_model.m
5. [ ] 刪除 run_simulation_slx.m
6. [ ] 執行完整測試確認

---

## 八、驗證檢查清單

### 8.1 參數驗證

- [ ] calc_simulation_params(config) 返回 Simulink.Parameter
- [ ] params.DataType == 'Bus: ParamsBus'
- [ ] Base workspace 包含所有子 Bus 定義
- [ ] 字串參數正確轉換為數值

### 8.2 Simulink 模型驗證

- [ ] 模型可載入無錯誤
- [ ] 所有 MATLAB Function blocks 有程式碼
- [ ] 信號維度匹配（3×1 向量）
- [ ] sim() 可執行無錯誤

### 8.3 整合驗證

- [ ] 開迴路模式：粒子不動（無控制力）
- [ ] 閉迴路（無干擾）：p_m 追蹤 p_d
- [ ] 閉迴路（有干擾）：p_m 追蹤 p_d，有隨機波動
- [ ] 追蹤誤差 RMSE < 0.1 um

---

## 九、風險與注意事項

### 9.1 已知風險

1. **MATLAB Function block 程式碼產生**
   - MATLAB Function block 需要手動配置程式碼
   - 無法完全自動化

2. **Bus Object 相容性**
   - Nested Bus 結構在不同 MATLAB 版本可能有差異
   - 建議在 R2024b 或 R2025b 測試

3. **persistent 變數重置**
   - motion_control_law 使用 persistent 變數
   - 每次模擬前需要 `clear motion_control_law`

### 9.2 備份建議

修改前備份以下檔案：
```
calc_simulation_params.m
run_simulation.m
model/system_model.slx
```

---

## 文件版本

| 版本 | 日期 | 作者 | 說明 |
|------|------|------|------|
| 1.0 | 2025-12-09 | Claude | 初版 |
