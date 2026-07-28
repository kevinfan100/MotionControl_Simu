# EKF 矩陣解讀指南

本文件整理 `reference/controller/Estimation_Control.pdf` 中所有大矩陣的符號、維度、block 結構，並標註哪些會隨疊代步 [k] 改變。

---

## 符號慣例

| 符號 | 意義 |
|------|------|
| `I_{nxn}` | n x n 單位矩陣 |
| `0_{mxn}` | m x n 全零矩陣 |
| 上標 `V` (如 `V_dp1`) | 在壁面座標 (u,v,w) 下表示 |
| 無上標 (如 `dp1`) | 在世界座標 (x,y,z) 下表示 |
| hat (如 `theta_hat`) | 估計值 |
| delta (如 `d_lambda`) | 變化率 |
| 下標 m (如 `lambda_m`) | 量測值 |

---

## 1. 狀態向量 — 9 組，共 23 維

| Group | 符號 | 維度 | 物理意義 | 隨 [k] 變化 |
|-------|------|------|---------|:-----------:|
| 1 | `V_dp_hat_1[k]` | **3x1** | 位置誤差 delay 1（最舊） | YES |
| 2 | `V_dp_hat_2[k]` | **3x1** | 位置誤差 delay 2 | YES |
| 3 | `V_dp_hat_3[k]` | **3x1** | 位置誤差（當前，控制用） | YES |
| 4 | `V_d_hat[k]` | **3x1** | 干擾估計 | YES |
| 5 | `V_dd_hat[k]` | **3x1** | 干擾變化率估計 | YES |
| 6 | `lambda_hat[k]` | **2x1** | 拖曳係數 [lambda_T; lambda_N] | YES |
| 7 | `d_lambda_hat[k]` | **2x1** | 拖曳係數變化率 | YES |
| 8 | `theta_hat[k]` | **2x1** | 壁面角度 [theta_x; theta_y] | YES |
| 9 | `d_theta_hat[k]` | **2x1** | 壁面角度變化率 | YES |

**Group 1~5 各 3 維，Group 6~9 各 2 維，合計 3x5 + 2x4 = 23**

---

## 2. 觀測向量 — 3 組，共 7 維

| 觀測 | 維度 | 對應 State Group |
|------|------|-----------------|
| `V_dp_m[k]` (位置誤差量測) | 3x1 | Group 1 |
| `lambda_m[k]` (拖曳係數量測) | 2x1 | Group 6 |
| `theta_m[k]` (角度量測) | 2x1 | Group 8 |

---

## 3. 所有矩陣總覽

| 矩陣 | 大小 | Block 結構 | 隨 [k] 變化 | 說明 |
|------|------|-----------|:-----------:|------|
| **H** | 7 x 23 | 3x9 blocks | NO | 觀測挑選矩陣，固定結構 |
| **F[k]** | 23 x 23 | 9x9 blocks | YES | 狀態轉移矩陣，含 G_lambda, G_theta |
| **P** | 23 x 23 | 9x9 blocks | YES | 後驗誤差協方差 |
| **P^f** | 23 x 23 | 9x9 blocks | YES | 先驗（預測）誤差協方差 |
| **Q[k]** | 23 x 23 | block diagonal | YES | 過程雜訊協方差，依賴 lambda_hat[k] |
| **R[k]** | 7 x 7 | — | ? | 量測雜訊協方差（文件未明確定義） |
| **L[k]** | 23 x 7 | 9x3 blocks | YES | Kalman gain |
| **G** | 7 x 7 | 3x3 blocks | YES | Innovation covariance 的逆 |
| **P^f H^T** | 23 x 7 | 9x3 blocks | YES | 中間計算量 |
| **H P^f H^T** | 7 x 7 | 3x3 blocks | YES | Innovation covariance |
| **V[k]** | 3 x 3 | — | YES | 壁面旋轉矩陣，依賴 theta_hat[k] |
| **G_lambda[k]** | 3 x 2 | — | YES | 拖曳係數耦合矩陣 |
| **G_theta[k]** | 3 x 2 | — | YES | 角度耦合矩陣 |

---

## 4. H — 觀測矩陣 (7 x 23) — 固定不變

H 是「挑選矩陣」，從 23 維狀態中挑出 Group 1, 6, 8 組成 7 維觀測。

```
           G1(3)  G2(3)  G3(3)  G4(3)  G5(3)  G6(2)  G7(2)  G8(2)  G9(2)
row1 (3) [ I3x3   0      0      0      0      0      0      0      0    ]  <- 取出 Group 1
row2 (2) [ 0      0      0      0      0      I2x2   0      0      0    ]  <- 取出 Group 6
row3 (2) [ 0      0      0      0      0      0      0      I2x2   0    ]  <- 取出 Group 8
```

Block 大小對照：
- row1 的每個 block: 3x3 或 3x2（根據該 column group 維度）
- row2, row3 的每個 block: 2x3 或 2x2

---

## 5. F[k] — 狀態轉移矩陣 (23 x 23) — 每步更新

F[k] 是 9x9 block matrix，block(i,j) 大小 = (Group_i 維度) x (Group_j 維度)。

```
           G1(3)  G2(3)  G3(3)    G4(3)   G5(3)  G6(2)      G7(2)  G8(2)      G9(2)
G1 (3)  [  0      I3x3   0        0        0      0          0      0          0     ]
G2 (3)  [  0      0      I3x3     0        0      0          0      0          0     ]
G3 (3)  [  0      0      I3x3    -I3x3     0     -G_lam[k]   0     -G_th[k]    0     ]
G4 (3)  [  0      0      0        I3x3     I3x3   0          0      0          0     ]
G5 (3)  [  0      0      0        0        I3x3   0          0      0          0     ]
G6 (2)  [  0      0      0        0        0      I2x2       I2x2   0          0     ]
G7 (2)  [  0      0      0        0        0      0          I2x2   0          0     ]
G8 (2)  [  0      0      0        0        0      0          0      I2x2       I2x2  ]
G9 (2)  [  0      0      0        0        0      0          0      0          I2x2  ]
```

### 隨 [k] 變化的部分

F[k] 中只有 **第 3 行的兩個 coupling block** 隨 k 改變：

**G_lambda[k]** (3x2) — 拖曳係數對位置的影響：
```
G_lambda[k] = [ Delta_u[k]      0        ]
              [ Delta_v[k]      0        ]     切向 (u,v) 用 lambda_T，法向 (w) 用 lambda_N
              [     0       Delta_w[k]   ]
```

**G_theta[k]** (3x2) — 壁面角度對位置的影響：
```
                    D_lambda[k] = lambda_T_hat[k] - lambda_N_hat[k]

G_theta[k] = [          0                     -D_lambda[k] * Delta_w[k]  ]
             [  D_lambda[k] * Delta_w[k]              0                  ]
             [  D_lambda[k] * Delta_v[k]     -D_lambda[k] * Delta_u[k]  ]
```

注意：`Delta_u/v/w[k]` 是 Section 10.2 控制律計算的位置增量，不要與干擾估計 `d_u/v/w_hat` 混淆。

### 各行物理意義

| Row (Group) | 動態方程 | 解讀 |
|-------------|---------|------|
| 1 | dp1[k+1] = dp2[k] | delay chain：由 Group 2 推進 |
| 2 | dp2[k+1] = dp3[k] | delay chain：由 Group 3 推進 |
| **3** | dp3[k+1] = dp3 - d - G_lam*lambda - G_th*theta | **核心動態**：位置誤差含控制、干擾、拖曳、角度耦合 |
| 4 | d[k+1] = d + dd | 干擾 = 干擾 + 變化率 |
| 5 | dd[k+1] = dd | 干擾變化率維持（隨機遊走模型） |
| 6 | lambda[k+1] = lambda + d_lambda | 拖曳 + 變化率 |
| 7 | d_lambda[k+1] = d_lambda | 拖曳變化率維持 |
| 8 | theta[k+1] = theta + d_theta | 角度 + 變化率 |
| 9 | d_theta[k+1] = d_theta | 角度變化率維持 |

---

## 6. P^f 相關矩陣 — 每步更新

### P^f (23 x 23)

P^f 是對稱矩陣，block `P^f_{ij}` 表示第 i 組與第 j 組的 cross-covariance。

Block 大小規則：
```
P^f_{ij} 大小 = (Group_i 維度) x (Group_j 維度)
```

| 範例 | Block 位置 | 大小 |
|------|-----------|------|
| `P^f_{11}` | (Group1, Group1) | 3x3 |
| `P^f_{16}` | (Group1, Group6) | 3x2 |
| `P^f_{18}` | (Group1, Group8) | 3x2 |
| `P^f_{61}` | (Group6, Group1) | 2x3 |
| `P^f_{66}` | (Group6, Group6) | 2x2 |
| `P^f_{88}` | (Group8, Group8) | 2x2 |
| `P^f_{98}` | (Group9, Group8) | 2x2 |

### P^f * H^T (23 x 7)

H 只挑 Group 1, 6, 8，所以 `P^f * H^T` = P^f 的**第 1, 6, 8 column group**：

```
P^f * H^T =  [ P^f_{i,1}(ix3)   P^f_{i,6}(ix2)   P^f_{i,8}(ix2) ]   for i = 1..9

           col-block1(3)  col-block2(2)  col-block3(2)
G1 (3)  [  P^f_{11}       P^f_{16}       P^f_{18}     ]
G2 (3)  [  P^f_{21}       P^f_{26}       P^f_{28}     ]
G3 (3)  [  P^f_{31}       P^f_{36}       P^f_{38}     ]
G4 (3)  [  P^f_{41}       P^f_{46}       P^f_{48}     ]
G5 (3)  [  P^f_{51}       P^f_{56}       P^f_{58}     ]
G6 (2)  [  P^f_{61}       P^f_{66}       P^f_{68}     ]
G7 (2)  [  P^f_{71}       P^f_{76}       P^f_{78}     ]
G8 (2)  [  P^f_{81}       P^f_{86}       P^f_{88}     ]
G9 (2)  [  P^f_{91}       P^f_{96}       P^f_{98}     ]
```

### H * P^f * H^T (7 x 7)

進一步只取 row Group 1, 6, 8：

```
H*P^f*H^T =  [ P^f_{11}(3x3)   P^f_{16}(3x2)   P^f_{18}(3x2) ]
              [ P^f_{61}(2x3)   P^f_{66}(2x2)   P^f_{68}(2x2) ]
              [ P^f_{81}(2x3)   P^f_{86}(2x2)   P^f_{88}(2x2) ]
```

---

## 7. G 與 L[k] — Kalman Gain 計算 — 每步更新

### G — Innovation Covariance Inverse (7 x 7)

```
G = ( H * P^f * H^T + R[k] )^{-1}
  = (7x7 + 7x7)^{-1} = 7x7
```

### L[k] — Kalman Gain (23 x 7)

```
L[k] = P^f * H^T * G
     = (23x7) * (7x7) = 23x7
```

L 的 block 結構是 **9 row-blocks x 3 column-blocks**：

```
         col1(3)   col2(2)   col3(2)
         對應 e_p   對應 e_lam  對應 e_th
G1 (3) [ L11       L12       L13     ]
G2 (3) [ L21       L22       L23     ]
G3 (3) [ L31       L32       L33     ]
G4 (3) [ L41       L42       L43     ]
G5 (3) [ L51       L52       L53     ]
G6 (2) [ L61       L62       L63     ]
G7 (2) [ L71       L72       L73     ]
G8 (2) [ L81       L82       L83     ]
G9 (2) [ L91       L92       L93     ]
```

各 block `L_{ij}` 的大小：

| L block | 大小 | 意義 |
|---------|------|------|
| `L_{i1}` (i=1..5) | 3x3 | Group i 受位置誤差 e_p (3x1) 修正 |
| `L_{i2}` (i=1..5) | 3x2 | Group i 受拖曳誤差 e_lambda (2x1) 修正 |
| `L_{i3}` (i=1..5) | 3x2 | Group i 受角度誤差 e_theta (2x1) 修正 |
| `L_{i1}` (i=6..9) | 2x3 | Group i 受位置誤差修正 |
| `L_{i2}` (i=6..9) | 2x2 | Group i 受拖曳誤差修正 |
| `L_{i3}` (i=6..9) | 2x2 | Group i 受角度誤差修正 |

---

## 8. Q[k] — 過程雜訊協方差 (23 x 23) — 每步更新

Q[k] 是 **block diagonal** 矩陣，只有對角 block 非零。

其中 `sigma^2 = (0.01589 um)^2 = 4*kB*T*dt / gamma_N`

| Block | 對應 Group | 大小 | 值 | 隨 [k] 變化 |
|-------|-----------|------|-----|:-----------:|
| Q11 | 1 | 3x3 | **0** | NO |
| Q22 | 2 | 3x3 | **0** | NO |
| Q33 | 3 | 3x3 | `sigma^2 * diag(lam_T_hat, lam_T_hat, lam_N_hat)` | YES |
| Q44 | 4 | 3x3 | `a_pd * a_prd * Q33` | YES |
| Q55 | 5 | 3x3 | `a_pd * a_prd * Q33` | YES |
| Q66 | 6 | 2x2 | `a_cov * sigma^2 * diag(lam_T_hat, lam_N_hat)` | YES |
| Q77 | 7 | 2x2 | = Q66 | YES |
| Q88 | 8 | 2x2 | **? (未定義)** | ? |
| Q99 | 9 | 2x2 | **? (未定義)** | ? |

Q33 展開：
```
Q33[k] = (0.01589)^2 * [ lambda_T_hat[k]    0                0              ]
                        [       0         lambda_T_hat[k]     0              ]
                        [       0              0         lambda_N_hat[k]     ]
```

Q66 展開：
```
Q66[k] = a_cov * (0.01589)^2 * [ lambda_T_hat[k]     0             ]
                                [       0         lambda_N_hat[k]   ]
```

注意：Q33~Q77 都依賴 `lambda_hat[k]`，因此每步都會更新。

---

## 9. V[k] — 壁面旋轉矩陣 (3 x 3) — 每步更新

由 `theta_hat[k] = [theta_x; theta_y]` 決定：

```
cx = cos(theta_x_hat[k]),  sx = sin(theta_x_hat[k])
cy = cos(theta_y_hat[k]),  sy = sin(theta_y_hat[k])

V[k] = [  cy    sy*sx    sy*cx  ]        V^T[k] = [  cy      0     -sy    ]
       [   0      cx      -sx   ]                  [ sy*sx    cx    cy*sx  ]
       [ -sy    cy*sx    cy*cx  ]                  [ sy*cx   -sx    cy*cx  ]
```

用途：世界座標 <-> 壁面座標轉換
- 世界 -> 壁面：`V_x = V^T * x`
- 壁面 -> 世界：`x = V * V_x`

---

## 10. 控制律計算 — 每步 [k] 更新

### 10.1 期望位移轉壁面座標

```
[delta_u_d[k]]              [delta_x_d[k]]
[delta_v_d[k]] = V^T[k]  *  [delta_y_d[k]]       (3x1) = (3x3) * (3x1)
[delta_w_d[k]]              [delta_z_d[k]]
```

### 10.2 位置增量計算

```
Delta_u[k] = (1 / lambda_T_hat[k]) * { delta_u_d[k] + (1 - lambda_c) * delta_u_hat[k] - d_u_hat[k] }
Delta_v[k] = (1 / lambda_T_hat[k]) * { delta_v_d[k] + (1 - lambda_c) * delta_v_hat[k] - d_v_hat[k] }
Delta_w[k] = (1 / lambda_N_hat[k]) * { delta_w_d[k] + (1 - lambda_c) * delta_w_hat[k] - d_w_hat[k] }
```

其中：
- `delta_u_hat, delta_v_hat, delta_w_hat` = `V_dp_hat_3[k]` (Group 3，當前位置誤差估計)
- `d_u_hat, d_v_hat, d_w_hat` = `V_d_hat[k]` (Group 4，干擾估計)
- 切向 (u,v) 除以 `lambda_T_hat`，法向 (w) 除以 `lambda_N_hat`

### 10.3 控制力

```
f_u[k] = (gamma_N / dt) * Delta_u[k]
f_v[k] = (gamma_N / dt) * Delta_v[k]
f_w[k] = (gamma_N / dt) * Delta_w[k]
```

---

## 11. 量測處理鏈 — 每步 [k] 更新

### 11.1 位置誤差量測 (3-D, 含 2-step delay)

```
初始化：
  k = 1:  p_d[k-2] = p_d[k];    p_d[k-1] = p_d[k]
  k = 2:  p_d[k-2] = p_d[k-1];  p_d[k-1] = p_d[k-1]
  k >= 3: p_d[k-2] = p_d[k-2];  p_d[k-1] = p_d[k-1]    (使用已存的值)

量測誤差（世界座標）：
  delta_p_m[k] = p_d[k-2] - p_m[k]                        (3x1)

更新暫存：
  p_d[k-2] = p_d[k-1]
  p_d[k-1] = p_d[k]
```

### 11.2 轉壁面座標

```
V_delta_p_m[k] = [delta_u_m[k]]
                 [delta_v_m[k]] = V^T[k] * delta_p_m[k]   (3x1)
                 [delta_w_m[k]]
```

### 11.3 確定/隨機分量分離 (低通濾波)

```
delta_p_md[k]  = (1 - a_pd) * delta_p_md[k-1]  + a_pd  * delta_p_m[k]     (確定性分量, 3x1)
delta_p_mr[k]  = delta_p_m[k] - delta_p_md[k]                              (隨機分量, 3x1)
delta_p_mrd[k] = (1 - a_prd) * delta_p_mr[k-1] + a_prd * delta_p_mr[k]    (隨機確定, 3x1)
delta_p_mrr[k] = delta_p_mr[k] - delta_p_mrd[k]                            (隨機殘差, 3x1)
```

### 11.4 正規化殘差

```
delta_p_nr[k] = (1 / g_cov) * delta_p_mrr[k]                               (3x1)

[delta_u_nr[k]]
[delta_v_nr[k]] = V^T[k] * delta_p_nr[k]                                   (3x1)
[delta_w_nr[k]]
```

### 11.5 拖曳係數量測 lambda_m[k] (2x1)

```
[lambda_Tm[k]]            [lambda_T_hat[k-1]]            [ (delta_u_nr^2 + delta_v_nr^2) / 2 ]
[lambda_Nm[k]] = (1-a_cov)*[lambda_N_hat[k-1]] + a_cov * [         delta_w_nr^2               ]
```

### 11.6 角度量測 theta_m[k] (2x1)

```
D_lambda_m[k] = lambda_Tm[k] - lambda_Nm[k]

if |D_lambda_m[k] / lambda_Tm[k]| < epsilon :
    [theta_xm[k]]   [theta_x_hat[k-1]]
    [theta_ym[k]] = [theta_y_hat[k-1]]       (各向同性 -> 無法觀測角度，保持前一步估計值)

otherwise:
    [theta_xm[k]]   [theta_x_hat[k-1]]            [  delta_v_nr * delta_w_nr / D_lambda_m  ]
    [theta_ym[k]] = [theta_y_hat[k-1]] + a_cov * [ -delta_w_nr * delta_u_nr / D_lambda_m  ]
```

---

## 12. 誤差信號 — 每步 [k] 更新

```
e_p[k]      = V_delta_p_m[k] - V_dp_hat_1[k]                                (3x1)

                      { [ (delta_u_nr^2 + delta_v_nr^2)/2 ]   [lambda_mT] }
e_lambda[k] = a_cov * { [       delta_w_nr^2               ] - [lambda_mN] }   (2x1)

e_theta[k]  = a_cov * [ delta_v_nr * delta_w_nr / D_lambda_m  ]             (2x1)
                       [ -delta_w_nr * delta_u_nr / D_lambda_m ]
```

---

## 13. EKF 狀態更新方程 — 每步 [k] 更新（Kalman 修正）

所有更新先在壁面座標 (V) 下計算，再轉回世界座標。

### 13.1 壁面座標下更新 (9 組方程)

```
(G1) V_dp_hat_1[k+1] = V_dp_hat_2[k]                    + L11*e_p + L12*e_lambda + L13*e_theta
(G2) V_dp_hat_2[k+1] = V_dp_hat_3[k]                    + L21*e_p + L22*e_lambda + L23*e_theta
(G3) V_dp_hat_3[k+1] = lambda_c * V_dp_hat_3[k]         + L31*e_p + L32*e_lambda + L33*e_theta
(G4) V_d_hat[k+1]    = V_d_hat[k] + V_dd_hat[k]         + L41*e_p + L42*e_lambda + L43*e_theta
(G5) V_dd_hat[k+1]   = V_dd_hat[k]                      + L51*e_p + L52*e_lambda + L53*e_theta
(G6) lambda_hat[k+1] = lambda_hat[k] + d_lambda_hat[k]  + L61*e_p + L62*e_lambda + L63*e_theta
(G7) d_lambda_hat[k+1] = d_lambda_hat[k]                + L71*e_p + L72*e_lambda + L73*e_theta
(G8) theta_hat[k+1]  = theta_hat[k] + d_theta_hat[k]    + L81*e_p + L82*e_lambda + L83*e_theta
(G9) d_theta_hat[k+1] = d_theta_hat[k]                  + L91*e_p + L92*e_lambda + L93*e_theta
```

注意 G3 多乘了 `lambda_c`（控制極點），這是控制律嵌入的結果。

### 13.2 壁面 -> 世界座標轉換 (只對 Group 1~5)

```
dp_hat_1[k+1]  = V[k] * V_dp_hat_1[k+1]       (3x1) = (3x3) * (3x1)
dp_hat_2[k+1]  = V[k] * V_dp_hat_2[k+1]
dp_hat_3[k+1]  = V[k] * V_dp_hat_3[k+1]
d_hat[k+1]     = V[k] * V_d_hat[k+1]
dd_hat[k+1]    = V[k] * V_dd_hat[k+1]
```

Group 6~9 (lambda, theta) 不需轉換，直接使用更新值。

### 13.3 世界 -> 壁面座標轉換 (下一步開始時)

```
V_dp_hat_1[k] = V^T[k] * dp_hat_1[k]           (3x1) = (3x3) * (3x1)
V_dp_hat_2[k] = V^T[k] * dp_hat_2[k]
V_dp_hat_3[k] = V^T[k] * dp_hat_3[k]
V_d_hat[k]    = V^T[k] * d_hat[k]
V_dd_hat[k]   = V^T[k] * dd_hat[k]
```

---

## 14. P / P^f 更新方程 — 每步 [k] 更新

### 14.1 後驗更新 (Measurement Update)

```
P = (I_{23x23} - L[k] * H) * P^f

  = (23x23 - 23x7 * 7x23) * 23x23
  = 23x23
```

### 14.2 先驗預測 (Time Update)

```
P^f = F[k] * P * F^T[k] + Q[k]

    = (23x23) * (23x23) * (23x23) + (23x23)
    = 23x23
```

### 14.3 完整 Kalman Gain 計算流程

```
Step 1:  P^f * H^T                      (23x23) * (23x7)  = 23x7
Step 2:  H * P^f * H^T                  (7x23) * (23x7)   = 7x7
Step 3:  G = (H * P^f * H^T + R[k])^-1  (7x7 + 7x7)^-1   = 7x7
Step 4:  L[k] = (P^f * H^T) * G         (23x7) * (7x7)    = 23x7
```

---

## 15. V[k] 更新 — 每步 [k] 更新

```
theta_hat[k] 更新後 (Section 13.1 G8)：

  lambda_hat[k] = lambda_hat[k+1]        (用更新後的值覆蓋)
  d_lambda_hat[k] = d_lambda_hat[k+1]
  theta_hat[k] = theta_hat[k+1]          (用更新後的值覆蓋)
  d_theta_hat[k] = d_theta_hat[k+1]

重新計算旋轉矩陣：
  cx = cos(theta_x_hat[k]),  sx = sin(theta_x_hat[k])
  cy = cos(theta_y_hat[k]),  sy = sin(theta_y_hat[k])

  V[k+1] = V[k] = [  cy    sy*sx    sy*cx  ]
                   [   0      cx      -sx   ]
                   [ -sy    cy*sx    cy*cx  ]
```

---

## 16. F[k] 更新 — 每步 [k] 更新

F[k] 需要當前步的 Delta_u/v/w 和 lambda_hat：

```
D_lambda_hat[k] = lambda_T_hat[k] - lambda_N_hat[k]

G_lambda[k] = [ Delta_u[k]    0          ]
              [ Delta_v[k]    0          ]    (3x2)
              [    0       Delta_w[k]    ]

G_theta[k]  = [        0                    -D_lambda_hat[k] * Delta_w[k]  ]
              [  D_lambda_hat[k] * Delta_w[k]           0                  ]    (3x2)
              [  D_lambda_hat[k] * Delta_v[k]   -D_lambda_hat[k] * Delta_u[k]  ]

F[k] 的完整結構見 Section 5（只有第 3 行含 G_lambda, G_theta 會變）
```

---

## 17. EKF 單步完整執行順序

Code reference: `model/controller/motion_control_law.m`

```
輸入：del_pd (= p_d[k+1]-p_d[k]), pd (= p_d[k]), p_m (= p_m[k])

 [1] 座標轉換  (L108-109)
     V_del_pd = V_T * del_pd
     Uses persistent V_T[k]

 [2] 控制律  (L111-120, Section 10)
     del_u/v/w -> fu/fv/fw -> f_d = V * [fu;fv;fw]
     Uses persistent [k]: lamda_hat, V_del_p3_hat, V_d_hat, V

 [3] 量測處理  (L122-151, Section 11)
     [3a] del_pm = pd_k2 - p_m                          (L125)
     [3b] V_del_pm = V_T * del_pm                       (L128)
     [3c] EMA: del_pmd -> del_pmr -> del_pmrd -> del_pmrr  (L131-134)
     [3d] del_pnr -> V_del_nr -> del_unr/vnr/wnr        (L137-139)
     [3e] lamda_m                                        (L142)
     [3f] theta_m (epsilon threshold)                    (L145-151)

 [4] 誤差信號  (L154-157, Section 12)
     err_p, err_lamda, err_theta -> err (7x1)

 [5] Kalman Gain  (L160-164, Section 14.3)
     Pf_HT -> HPf_HT -> G -> L (23x7)

 [6] 狀態更新  (L166-184, Section 13)
     [6a] 9 groups wall frame: *_kA1 variables          (L169-177)
     [6b] World frame: del_p*_hat_kA1 = V * V_del_*     (L180-184)

 [7] 後驗協方差  (L186-187, Section 14.1)
     P = (I - L*H) * Pf

 [8] F[k] 更新  (L189-218, Section 16)
     del_lamda_hat_scalar from lamda_hat_kA1 (updated)
     G_lamda, G_theta use del_u/v/w from Step [2]
     F (23x23)

 [9] Q[k] 更新  (L222-231, Section 8)
     Uses lamda_hat_kA1 (updated)
     Q33, Q66 -> Q (23x23)

[10] 先驗預測  (L233-234, Section 14.2)
     Pf = F * P * F' + Q

[11] V[k] 更新 & 狀態重投影  (L236-251, Section 15, 13.3)
     [11a] V_new, V_T_new from theta_hat_kA1            (L239-243)
     [11b] Re-project: V_del_*_new = V_T_new * del_*_kA1  (L247-251)

Persistent Shifts  (L253-284)
     V, V_T, wall-frame states, scalar states, world copies,
     delay buffers (pd_k1/k2), EMA states (del_pmd_k1, del_pmrd_k1)
```

---

## 18. EKF 更新流程中的矩陣依賴

```
常數 / 只算一次          每步 [k] 更新
------------------      ----------------------------------
H (7x23)                V[k] (3x3)          <- theta_hat[k]
sigma^2                 G_lambda[k] (3x2)   <- Delta_u, Delta_v, Delta_w
a_pd, a_prd, a_cov      G_theta[k] (3x2)    <- D_lambda, Delta_u/v/w
lambda_c                F[k] (23x23)        <- G_lambda, G_theta
gamma_N, dt             Q[k] (23x23)        <- lambda_hat[k]
epsilon                 P^f (23x23)         <- F, P, Q
                        L[k] (23x7)         <- P^f, H, R
                        P (23x23)           <- L, H, P^f
                        所有狀態估計 (23x1)
                        所有誤差信號 (7x1)
```

---

## 19. 常數參數

| 符號 | 值 | 說明 |
|------|-----|------|
| kB | 1.3806503e-5 pN*um/K | 波茲曼常數 |
| T | 310.15 K | 溫度 (37 C) |
| gamma_N | 0.0425 pN*sec/um | Stokes 拖曳係數 (R=2.25 um) |
| dt | 1/1606 sec | 取樣週期 |
| sigma^2 | (0.01589 um)^2 | 熱擾動位移變異數 |
| lambda_c | 0.7 | 控制極點 |
| epsilon | 0.01 | 角度量測門檻 |
| a_pd | 0.1 | 確定性分量濾波器係數 |
| a_prd | 0.1 | 隨機分量濾波器係數 |
| a_cov | 0.1 | 協方差濾波器係數 |
