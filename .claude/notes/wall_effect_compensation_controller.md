# Wall Effect Compensation Controller

## 問題背景

原始控制律使用固定的拖曳係數 `gamma_N = 0.0425 pN*sec/um`，沒有考慮 Wall Effect。

當粒子靠近壁面時，實際拖曳係數會顯著增加：

| h/R | 實際 Gamma_zz | 與 gamma_N 比值 | 控制力不足比例 |
|-----|---------------|-----------------|----------------|
| 4.44 | 0.0566 | 1.33x | 25% |
| 2.22 | 0.0818 | 1.92x | 48% |
| 2.00 | 0.0903 | 2.12x | 53% |
| 1.78 | 0.1036 | 2.44x | 59% |

這導致控制力不足，粒子無法追蹤軌跡。

---

## 原始控制律（無 Wall Effect 補償）

```matlab
function f_d = motion_control_law(p_d, p_m, params)
    % 使用固定的 gamma
    gamma = params.ctrl.gamma;  % = gamma_N = 0.0425
    lambda_c = params.ctrl.lambda_c;
    Ts = params.ctrl.Ts;

    % 控制力公式
    f_d = (gamma / Ts) * (p_d - lambda_c * p_d_prev - (1 - lambda_c) * p_m);
end
```

**問題**：gamma 是固定值，不會隨位置變化。

---

## 修正後控制律（有 Wall Effect 補償）

```matlab
function f_d = motion_control_law(p_d, p_m, params)
    lambda_c = params.ctrl.lambda_c;
    Ts = params.ctrl.Ts;

    % 1. 計算期望速度
    v_d = (1 / Ts) * (p_d - lambda_c * p_d_prev - (1 - lambda_c) * p_m);

    % 2. 計算位置相關的拖曳係數矩陣 Gamma(p_m)
    [Gamma_inv, ~] = calc_gamma_inv(p_m, params);
    Gamma = inv(Gamma_inv);

    % 3. 使用位置相關的 Gamma 計算控制力
    f_d = Gamma * v_d;
end
```

**修正**：用 `Gamma(p_m)` 取代固定的 `gamma_N`。

---

## 物理意義

```
粒子動力學: p_dot = Gamma_inv(p) * F
控制目標:   p_dot = v_d (期望速度)

=> F = Gamma(p) * v_d
```

靠近壁面時 Gamma 增大 → 控制力自動增大 → 正確追蹤軌跡。

---

## 測試結果

| 條件 | 追蹤誤差 RMSE |
|------|---------------|
| 無熱力擾動 | 0.6 nm |
| 有熱力擾動 | 29.8 nm |

---

## 完整修正後程式碼

```matlab
function f_d = motion_control_law(p_d, p_m, params)
%MOTION_CONTROL_LAW Position-dependent discrete-time motion control
%
%   f_d = motion_control_law(p_d, p_m, params)
%
%   Implements the control law for magnetic bead position control with
%   Wall Effect compensation. Uses position-dependent drag coefficient
%   matrix Gamma(p_m) instead of fixed gamma_N.
%
%   Control Law (with Wall Effect compensation):
%       v_d[k] = (1/Ts) * {p_d[k] - lambda_c * p_d[k-1] - (1-lambda_c) * p_m[k]}
%       f_d[k] = Gamma(p_m[k]) * v_d[k]
%
%   where Gamma(p_m) = inv(Gamma_inv(p_m)) is the position-dependent
%   drag coefficient matrix that accounts for wall effects.

    % Check if control is enabled (0 = disabled, 1 = enabled)
    if params.ctrl.enable < 0.5
        % Open-loop mode: no control force
        f_d = zeros(3, 1);
        return;
    end

    % Closed-loop mode
    persistent p_d_prev initialized

    % Initialize on first call
    if isempty(initialized)
        initialized = true;
        p_d_prev = p_d;
    end

    % Extract control parameters
    lambda_c = params.ctrl.lambda_c;
    Ts = params.ctrl.Ts;

    % Compute desired velocity
    v_d = (1 / Ts) * (p_d - lambda_c * p_d_prev - (1 - lambda_c) * p_m);

    % Calculate position-dependent drag coefficient matrix Gamma(p_m)
    [Gamma_inv, ~] = calc_gamma_inv(p_m, params);
    Gamma = inv(Gamma_inv);

    % Compute control force with Wall Effect compensation
    f_d = Gamma * v_d;

    % Update state for next iteration
    p_d_prev = p_d;
end
```

---

## 啟用方式

將上述程式碼覆蓋 `model/controller/motion_control_law.m` 即可啟用 Wall Effect 補償。
