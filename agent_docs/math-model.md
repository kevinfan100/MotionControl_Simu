# 座標系統與數學模型

## 壁面座標
- 壁面方向由 theta (方位角) 和 phi (仰角) 定義，**使用者設定單位為 deg**
- 內部自動轉 rad 計算
- w_hat = 壁面法向量，u_hat/v_hat = 壁面切向量
- 預設 theta=0, phi=0 時：w_hat = [0;0;1]（世界 Z 軸）

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
