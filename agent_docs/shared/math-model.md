# 座標系統與數學模型

## 壁面座標（2026-10-15 起，與 Estimator_and_Control__.pdf p.6 同一套）
- 兩個座標：measurement coordinate（x, y, z）與 local coordinate（u, v 沿牆、w 法向）
- 壁面方向由 `config.theta_x`、`config.theta_y` 定義（**deg**）：先繞 x 轉 θ_x、再繞 y 轉 θ_y，
  V = R_y(θ_y)·R_x(θ_x) = [û v̂ ŵ]（`params.wall.V`）
- V 把 local 轉到 measurement：δp = V·ᵛδp；Vᵀ 把 measurement 轉到 local：ᵛδp = Vᵀ·δp
- 角度為 0 時 V = I：u = x、v = y、w = z。θ_y > 0：法向往 +x 傾（牆往 +x 降低）；θ_x > 0：法向往 −y 傾
- ŵ = (sinθ_y cosθ_x, −sinθ_x, cosθ_y cosθ_x)；增益在 local 座標為 𝚲̄ = diag(ā_T, ā_T, ā_N)，
  在 measurement 座標為 𝐀̄ = 𝐕·𝚲̄·𝐕ᵀ（T = tangential，N = normal；bar = 已除以 a_nom；同博論 (5.1) 𝐀 = 𝐕𝚲𝐕ᵀ）
- 字體依 IEEE TIE 2025：純量斜體（ā_T、θ_x、λ_c），向量／矩陣粗體正體（δ𝐩̄、𝐮 𝐯 𝐰、𝐕、𝐀̄、𝚲̄）；
  左上標 ᵛ = 估測的 local coordinate（博論 (5.7)，ᵛδ𝐩̄ = 𝐕̂ᵀδ𝐩̄）
- 舊參數（方位角 theta、極角 phi；角度 0 時 û = −x、v̂ = −y）已移除，設非零值會被 `calc_wall_params` 擋下並給換算：
  θ_x = −asin(sinθ·sinφ)，θ_y = atan2(cosθ·sinφ, cosφ)；切向正弦 amp_u、amp_v 的正負號因此翻轉（同一條路徑）
- 熱力（2026-10-15）：在 local 座標抽 √(係數·[c∥ c∥ c⊥])·randn，再乘 V 轉回量測座標，
  共變異 = 係數·V·diag(c∥, c∥, c⊥)·Vᵀ（與 Γ⁻¹ 同一個阻力矩陣）；舊寫法在斜牆時沒有共變異數
- 真值紀錄（`run_pure_simulation`）：a_true_out = 量測座標增益矩陣 A = V·diag(ā_T, ā_T, ā_N)·Vᵀ 的對角項（角度 0 時不變）；
  新增 a_true_mat_out（N×9，完整 A）、a_local_out（N×2，[a_T a_N]）
- 只對角度 0 有效、尚未改：現役逐軸控制器、use_true_gain／a_ctrl_override 的真值餵入、自己算 z 軸真值的 driver
- 回歸：`test_script/scratch/verify_wall_angle_xy.m`（z 軸與 x/y 正弦逐位相同、斜牆 ŵ 與舊定義一致、
  斜牆熱力共變異與一步熱位移主方向、A_true = Ts·Γ⁻¹）

## 距離單位
- 使用者設定（h_init, amplitude, h_min）：**um（微米）**
- 模型內部物理計算：**h_bar = h/R（無因次）**
- 繪圖輸出：**世界座標 um**

## 軌跡定義
- 軌跡沿壁面法向 w_hat 做正弦振盪
- 設定的 amplitude 直接就是 h 方向的振幅（不受壁面角度影響）
- p_d(t) = p0 + amplitude * sin(2*pi*f*t) * w_hat（世界座標輸出）

---

## 系統方程
```
p_dot = Gamma_inv(p) * F_total
p = integral(p_dot) dt
```

其中 Gamma_inv(p) 是位置相關的移動度矩陣（考慮 Wall Effect）。

## Gamma_inv 計算
```
Gamma_inv(p) = 1/(gamma_N * c_para(h_bar)) * { I - [(c_perp(h_bar) - c_para(h_bar)) / c_perp(h_bar)] * W }
```
- h_bar = h/R（無因次化壁面距離）
- W = w_hat * w_hat'（壁面法線投影矩陣）
- c_para, c_perp：平行/垂直修正係數（h_bar 的多項式函數）

## 初始位置
```
p0 = (pz + h_init) * w_hat
```
- pz：壁面沿 w_hat 方向的位移 [um]
- h_init：初始距離壁面 [um]
