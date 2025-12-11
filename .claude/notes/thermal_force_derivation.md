# Thermal Force 數學推導

## 1. 物理背景

### 1.1 Langevin 方程

粒子在流體中的運動由 Langevin 方程描述：

```
m * p'' = F_M + F_T - Gamma * p'
```

其中：
- `m` = 粒子質量
- `p''` = 加速度 (d²p/dt²)
- `p'` = 速度 (dp/dt)
- `F_M` = 外力（如磁力）
- `F_T` = 熱力（隨機布朗運動力）
- `Gamma` = 拖曳係數矩陣 (3x3)

### 1.2 Overdamped 近似

對於微米級膠體粒子（低雷諾數），慣性項可忽略，方程簡化為：

```
0 = F_M + F_T - Gamma * p'
```

即：

```
p' = Gamma_inv * (F_M + F_T)
```

這就是專案 PDF 文件 (Drag_MATLAB.pdf) 中的系統方程。

---

## 2. Fluctuation-Dissipation Theorem（漲落耗散定理）

### 2.1 基本原理

熱力 F_T 和拖曳係數 Gamma 來自同一個物理來源（流體分子的碰撞），因此它們之間存在基本關係。

### 2.2 連續時間形式

熱力的自相關函數：

```
<F_T(t) * F_T(t')^T> = 2 * k_B * T * Gamma * delta(t - t')
```

其中：
- `<...>` = 統計平均（ensemble average）
- `k_B` = 波茲曼常數 = 1.3806503 × 10^-23 J/K
- `T` = 絕對溫度 (K)
- `Gamma` = 拖曳係數矩陣 (3x3)
- `delta(t - t')` = Dirac delta 函數

### 2.3 離散時間形式

當使用時間步長 dt 進行數值模擬時，Dirac delta 函數離散化：

```
delta(t - t') --> 1/dt   (當 t = t' 時)
```

因此熱力的協方差矩陣變為：

```
Cov(F_T) = <F_T * F_T^T> = 2 * k_B * T * Gamma / dt
```

### 2.4 論文中的公式 [33]

論文使用的公式：

```
sigma_fr^2 = 4 * k_B * T * gamma / dt
```

係數為 4（而非 2）的原因：
- 這取決於功率譜密度的定義方式（單邊 vs 雙邊）
- 或特定的數值積分方案
- 在本專案中，我們遵循論文的 4 係數

---

## 3. Wall Effect 對 Thermal Force 的影響

### 3.1 拖曳係數矩陣

在靠近牆壁時，拖曳係數變成位置相關的 3x3 矩陣：

**世界座標系中：**
```
Gamma = gamma_N * [c_para * I + (c_perp - c_para) * W]
```

其中：
- `gamma_N` = 名義拖曳係數 (Stokes drag) = 6 * pi * eta * R
- `c_para` = 平行於牆方向的校正係數
- `c_perp` = 垂直於牆方向的校正係數
- `I` = 3x3 單位矩陣
- `W` = w_hat * w_hat' = 牆法向量的投影矩陣

**局部座標系中（對角化）：**
```
Gamma_local = gamma_N * diag([c_para, c_para, c_perp])
```

其中 (u, v, w) 是牆的局部座標系：
- `u_hat`, `v_hat` = 平行於牆的兩個正交單位向量
- `w_hat` = 垂直於牆的單位向量（法向量）

### 3.2 校正係數公式

來自 Drag_MATLAB.pdf：

**平行校正係數 c_para：**
```
c_para = 1 / (1 - 9/16*(1/h_bar) + 1/8*(1/h_bar)^3
              - 45/256*(1/h_bar)^4 - 1/16*(1/h_bar)^5)
```

**垂直校正係數 c_perp：**
```
c_perp = 1 / (1 - 9/8*(1/h_bar) + 1/2*(1/h_bar)^3
              - 57/100*(1/h_bar)^4 + 1/5*(1/h_bar)^5
              + 7/200*(1/h_bar)^11 - 1/25*(1/h_bar)^12)
```

其中 `h_bar = h / R` 是正規化距離（粒子中心到牆的距離除以粒子半徑）。

---

## 4. Thermal Force 計算方法

### 4.1 方法：局部座標系分解

由於 Gamma 矩陣在局部座標系中是對角的，可以簡化計算。

**Step 1: 計算各方向的變異數**

```
sigma_u^2 = 4 * k_B * T * gamma_N * c_para / dt
sigma_v^2 = 4 * k_B * T * gamma_N * c_para / dt
sigma_w^2 = 4 * k_B * T * gamma_N * c_perp / dt
```

**Step 2: 計算標準差**

```
sigma_para = sqrt(4 * k_B * T * gamma_N * c_para / dt)
sigma_perp = sqrt(4 * k_B * T * gamma_N * c_perp / dt)
```

**Step 3: 在局部座標系生成隨機力**

```
F_u = sigma_para * randn()   % u 方向（平行於牆）
F_v = sigma_para * randn()   % v 方向（平行於牆）
F_w = sigma_perp * randn()   % w 方向（垂直於牆）
```

**Step 4: 轉換到世界座標**

```
F_T = F_u * u_hat + F_v * v_hat + F_w * w_hat
```

### 4.2 完整 MATLAB 實作

```matlab
function F_th = calc_thermal_force(p, params)
%CALC_THERMAL_FORCE Generate random thermal force (Brownian motion)
%
%   基於 Fluctuation-Dissipation Theorem:
%   sigma^2 = 4 * k_B * T * gamma / dt
%
%   在有 Wall Effect 時，gamma 在不同方向有不同值：
%   - 平行於牆: gamma_para = gamma_N * c_para
%   - 垂直於牆: gamma_perp = gamma_N * c_perp

    % 提取參數
    w_hat = params.wall.w_hat;      % 牆法向量
    u_hat = params.wall.u_hat;      % 牆平行向量 1
    v_hat = params.wall.v_hat;      % 牆平行向量 2
    pz = params.wall.pz;            % 牆位移
    R = params.common.R;            % 粒子半徑
    gamma_N = params.common.gamma_N; % 名義拖曳係數
    k_B = params.thermal.k_B;       % 波茲曼常數
    T = params.thermal.T;           % 溫度
    dt = params.thermal.Ts;         % 取樣週期

    % 計算正規化距離
    h = dot(p, w_hat) - pz;
    h_bar = h / R;

    % 計算校正係數
    [c_para, c_perp] = calc_correction_functions(h_bar);

    % 計算各方向的標準差
    % sigma^2 = 4 * k_B * T * gamma / dt
    % 其中 gamma_para = gamma_N * c_para, gamma_perp = gamma_N * c_perp
    base_coeff = 4 * k_B * T * gamma_N / dt;
    sigma_para = sqrt(base_coeff * c_para);
    sigma_perp = sqrt(base_coeff * c_perp);

    % 在局部座標系生成隨機力
    F_u = sigma_para * randn();  % u 方向
    F_v = sigma_para * randn();  % v 方向
    F_w = sigma_perp * randn();  % w 方向

    % 轉換到世界座標
    F_th = F_u * u_hat + F_v * v_hat + F_w * w_hat;
end
```

---

## 5. 物理驗證

### 5.1 極限情況檢查

**遠離牆壁 (h_bar -> infinity)：**
- c_para -> 1, c_perp -> 1
- sigma_para = sigma_perp = sqrt(4 * k_B * T * gamma_N / dt)
- 熱力變成各向同性（等方性）

**靠近牆壁 (h_bar -> 1)：**
- c_para > 1, c_perp > 1, 且 c_perp > c_para
- sigma_perp > sigma_para
- 垂直於牆的熱力波動較大（因為拖曳係數較大）

### 5.2 能量均分定理

在熱平衡時，每個自由度的平均動能應為 (1/2) * k_B * T。

這可以通過模擬驗證：長時間平均的速度變異數應該滿足：
```
<v_i^2> = k_B * T / m   (對於每個方向 i)
```

---

## 6. 與原始程式碼的差異

### 6.1 原始程式碼（有問題）

```matlab
C = c_para * (u_hat + v_hat) + c_perp * w_hat;
Variance = variance_coeff * (C.^2);
F_th = sqrt(Variance) .* randn(3, 1);
```

**問題：**
1. `u_hat + v_hat` 沒有物理意義（兩個單位向量相加）
2. 對向量做 `.^2` 然後開根號的方式不正確
3. 沒有正確處理座標轉換

### 6.2 修正後的程式碼

見 Section 4.2 的完整實作。

**關鍵改進：**
1. 在局部座標系（u, v, w）中分別計算各方向的熱力
2. 使用正確的變異數公式：sigma^2 = 4 * k_B * T * gamma_N * c / dt
3. 通過基底向量正確轉換到世界座標

---

## 7. 參考文獻

1. 專案文件：Drag_MATLAB.pdf - Wall Effect 數學模型
2. 專案文件：thermal_force.png - 原始熱力公式
3. 論文：Ultra-Precise High-Speed Untethered Manipulation of Magnetic Scanning Microprobe in Aqueous Solutions
4. [Langevin Equation - Chemistry LibreTexts](https://chem.libretexts.org/Bookshelves/Biological_Chemistry/Concepts_in_Biophysical_Chemistry_(Tokmakoff)/03:_Diffusion/13:_Friction_and_the_Langevin_Equation/13.01:_Langevin_Equation)
5. [Langevin equation - Wikipedia](https://en.wikipedia.org/wiki/Langevin_equation)

---

## 8. 更新記錄

- 2025-12-10：初版，推導 thermal force 正確公式
