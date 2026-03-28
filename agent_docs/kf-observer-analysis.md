# 3-State Kalman Filter Observer Analysis

## 目的

建立 Q/R → L_ss → le_eff → C_dpm 的完整數學鏈條，理解 observer 收斂速度和 tracking error variance 的關係。

## 預期結果

1. 從 Kalman filter 的 Q/R 比例出發，推導出等效 observer pole le
2. 得到 rho → le 的解析公式
3. 得到 le → C_dpm(lc, le) 的解析公式
4. 驗證 Fe-based KF 和 pole-placement 在 le=0 時一致

---

## 物理系統

磁探針單軸離散運動方程：

```
z[k+1] = z[k] + a_z * (f_d[k] + f_T[k])
```

- a_z = Ts / gamma_N [um/pN] — mobility
- f_d — 控制力
- f_T — thermal force（白噪音，variance = sigma2_fT）

Tracking error: delta_z[k] = z_d[k] - z[k]
控制目標: delta_z[k+1] = lc * delta_z[k]，lc 是閉迴路極點

量測有 2-step delay: del_pm[k] = z_d[k-2] - z_m[k]

---

## Observer 結構

### 狀態定義

```
del_p1[k] = delta_z[k-2]   ← 可量測（= del_pm）
del_p2[k] = delta_z[k-1]   ← 不可量測
del_p3[k] = delta_z[k]     ← 不可量測（controller 需要這個）
```

### Observer 更新法則

```
innovation = del_pm[k] - del_p1_hat[k]

del_p1_hat[k+1] = del_p2_hat[k]           + L1 * innovation
del_p2_hat[k+1] = del_p3_hat[k]           + L2 * innovation
del_p3_hat[k+1] = lc * del_p3_hat[k]      + L3 * innovation
```

L1, L2, L3 是 observer gain，決定收斂速度。

### 控制律

```
f_d[k] = (1/a_z) * (del_pd + (1-lc) * del_p3_hat[k])
```

Controller 用估計的當前 error (del_p3_hat) 計算控制力。

---

## Observer Error Dynamics

定義 estimation error: e_i[k] = del_p_i[k] - del_p_i_hat[k]

推導 e3 的演化：

```
e3[k+1] = del_p3[k+1] - del_p3_hat[k+1]
        = [lc*del_p3 + (1-lc)*e3 - a_z*fT] - [lc*del_p3_hat + L3*innovation]
        = lc*(del_p3 - del_p3_hat) + (1-lc)*e3 - L3*e1 - a_z*fT
        = lc*e3 + (1-lc)*e3 - L3*e1 - a_z*fT
        = 1*e3 - L3*e1 - a_z*fT
```

注意：lc*e3 + (1-lc)*e3 = **1***e3。(1-lc)*e3 項來自 controller 的作用。

矩陣形式：

```
e[k+1] = (Fe - L*H) * e[k] + [0; 0; -a_z] * fT[k]

Fe = [0, 1, 0]     ← error dynamics matrix
     [0, 0, 1]
     [0, 0, 1]     ← (3,3) = 1（不是 lc，因為 controller 補償）

H = [1, 0, 0]      ← 只能量測 e1

L = [L1; L2; L3]   ← observer gain
```

### 為什麼 Fe(3,3) = 1 而非 lc

真實 state dynamics: F(3,3) = lc（open-loop plant dynamics）
Closed-loop error dynamics: Fe(3,3) = 1

因為 controller 使用 del_p3_hat 計算力，引入 (1-lc)*e3 項。
在 error dynamics 中：lc*e3 + (1-lc)*e3 = 1*e3。
controller 的作用把 lc 補成了 1。

---

## 設定 Observer Gain 的兩種方式

### 方式 1: Pole-placement

指定 Fe - L*H 的三個 eigenvalue 都在 le：

```
L1 = 1 - 3*le
L2 = 1 - 3*le + 3*le^2
L3 = (1 - le)^3
```

Observer error eigenvalues: (le, le, le) — 三重極點。

### 方式 2: Kalman filter

不指定極點，而是給定噪音統計特性 Q 和 R，由 DARE 自動計算最優 L。

---

## Kalman Filter 設計

### 設計用的系統矩陣

Kalman filter 必須用 **Fe**（closed-loop error dynamics），而非 F（open-loop state dynamics）。

原因：observer error 的實際演化遵循 Fe（controller 已在迴路中），
用 F 設計會得到對 closed-loop 次優的 gain。

### Q 和 R

Process noise covariance（每步 error 的隨機變動）：

```
Q = sigma2_deltaXT * diag([0, 0, 1])

sigma2_deltaXT = 4 * k_B * T * Ts / gamma_N [um^2]
```

只有 e3 被 thermal force 直接驅動。e1, e2 是 e3 的延遲，沒有新噪音。

Measurement noise covariance：

```
R = rho * sigma2_deltaXT
```

rho 是 tuning 參數（量測噪音 / thermal noise 的比例）：
- rho 小 → 量測可靠 → L 大 → observer 快 → le 小
- rho 大 → 量測有噪音 → L 小 → observer 慢 → le 大

### DARE（Discrete Algebraic Riccati Equation）

穩態時，forecast covariance Pf 滿足：

```
Predict:  Pf = Fe * P * Fe' + Q
Update:   S = H * Pf * H' + R         ← innovation variance
          L = Pf * H' / S             ← Kalman gain
          P = (I - L*H) * Pf          ← posterior covariance
```

合併成 DARE：

```
Pf = Fe * (Pf - Pf*H'*(H*Pf*H'+R)^{-1}*H*Pf) * Fe' + Q
```

收斂後 Pf 和 L 不再變化。

---

## Fe 特殊結構的結果

### L1 = L2 = L3 = L

因為 Fe 的前兩行是純延遲（[0,1,0] 和 [0,0,1]），DARE 的解讓三個 gain 自然相等。

### Pf 的結構

```
Pf = [a,   a,   a  ]     其中 a = Pf(1,1)
     [a,  a+1, a+1 ]
     [a,  a+1, a+2 ]
```

### Observer error eigenvalues

L1=L2=L3=L 時，Fe - L*ones(3,1)*H 的特徵值：

```
eigenvalues = {0, 0, 1-L}
```

兩個在 0（部分 deadbeat），一個在 le = 1-L。

這和 pole-placement 不同 — pole-placement 把三個 eigenvalue 都放在 le。

---

## rho → le 的解析推導

### 關鍵步驟

從 DARE 的 (3,3) 元素：

```
Pf(3,3) = Pf(3,3) - alpha + 1
=> alpha = 1
```

其中 alpha = L * Pf(1,1) = a^2/(a+rho)。

alpha = 1 代入：

```
a^2/(a + rho) = 1
a^2 - a - rho = 0
```

### 求解

```
a = (1 + sqrt(1 + 4*rho)) / 2
```

L = 1/a（因為 L = a/(a+rho) = a/a^2 = 1/a）：

```
L = 2 / (1 + sqrt(1 + 4*rho))
```

le = 1 - L：

```
le = (sqrt(1 + 4*rho) - 1) / (sqrt(1 + 4*rho) + 1)
```

### 驗證

| rho | a | L = 1/a | le = 1-L |
|---|---|---|---|
| 0 | 1 | 1 | 0（deadbeat）|
| 1 | (1+sqrt(5))/2 = 1.618 | 0.618（黃金比例）| 0.382 |
| 2 | 2 | 0.5 | 0.5 |
| 10 | 3.702 | 0.270 | 0.730 |
| 100 | 10.51 | 0.095 | 0.905 |

全部和 DARE 數值解吻合。

---

## le → C_dpm 的解析公式

### 方法

把 L1=L2=L3=1-le 代入 4-state augmented system，用 Lyapunov 方程求解。

4-state system（tracking error + observer error）：

```
A4 = [lc,       0,       0,    1-lc ]
     [ 0,   le-1,       1,       0  ]
     [ 0,   le-1,       0,       1  ]
     [ 0,   le-1,       0,       1  ]

B4 = [-1; 0; 0; -1]   (normalized by a_z)
```

C_dpm = P(1,1) from Lyapunov: P = A4*P*A4' + B4*B4'

### 結果

```
C_dpm(lc, le) = N(lc, le) / [(lc^2-1)(lc*le-1)(le-1)(le+1)]
```

分子 N 是 lc 和 le 的 3 次多項式（比 pole-placement 的 5 次簡單）。

le=0 時簡化為：

```
C_dpm = (4 - 3*lc^2) / (1 - lc^2) = 3 + 1/(1-lc^2)
```

---

## Fe-based KF vs Pole-placement 比較

| 比較項 | Fe-based KF | Pole-placement |
|---|---|---|
| 設計參數 | rho（Q/R 比例）| le（指定極點）|
| Gain 結構 | L1=L2=L3（三個相等）| L1 不等於 L2 不等於 L3 |
| Observer eigenvalues | (0, 0, le) | (le, le, le) |
| le=0 結果 | 相同（deadbeat）| 相同 |
| le>0 結果 | C_dpm 更小（部分 deadbeat 更有效）| C_dpm 更大 |

### 數值比較

| lc | le | C_dpm(KF) | C_dpm(PP) | 差異 |
|---|---|---|---|---|
| 0.4 | 0 | 4.190 | 4.190 | 0% |
| 0.4 | 0.3 | 4.439 | 5.040 | KF 低 12% |
| 0.7 | 0 | 4.961 | 4.961 | 0% |
| 0.7 | 0.5 | 5.716 | 7.340 | KF 低 22% |

Fe-based KF 在 le>0 時 C_dpm 更小，因為它保留了兩個 deadbeat pole（= 0），
只用一個 pole 在 le。Pole-placement 把三個 pole 都放在 le，收斂更慢。

---

## 完整推導鏈

```
rho（量測噪音 / thermal noise 比例）
  |  a = (1 + sqrt(1 + 4*rho)) / 2
  |  L = 1/a
  v
le = 1 - L（等效 observer pole）
  |  A4(L1=L2=L3=1-le), Lyapunov
  v
C_dpm(lc, le)（tracking error variance factor）
  |  * sigma2_deltaXT
  v
Var(del_pm) = C_dpm * sigma2_deltaXT [um^2]
```

---

## F vs Fe 設計差異

若誤用 F（open-loop state dynamics, (3,3)=lc）設計 Kalman filter：
- L1 不等於 L2 不等於 L3（三個 gain 不同）
- L 和 lc 耦合
- C_dpm 遠大於 pole-placement（次優）

這是因為 Kalman filter 不知道 controller 的存在，假設系統以 open-loop dynamics 演化。

正確做法：用 Fe（closed-loop error dynamics, (3,3)=1）設計。

---

## 數據來源

- Riccati 分析: test_results/verify/riccati_analysis.mat
- Lyapunov le sweep: test_results/verify/lyapunov_le_sweep.mat
- 驗證腳本: test_script/verify_variance.m
