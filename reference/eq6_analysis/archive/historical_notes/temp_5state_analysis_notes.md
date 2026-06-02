# 5-State KF 解析框架（Time-Varying）

## 5-State 系統

States: [δx̂_1, δx̂_2, δx̂_3, â, δâ]
量測: [del_pm, a_m]

所有量都是 time-varying（不可假設 constant）。

## Time-Varying 量

| 量 | 依賴 | 公式 |
|---|---|---|
| a[k] | h[k] | T_s / gamma[k] |
| gamma[k] | h[k] | gamma_N × c_perp(h[k]/R) |
| f_d[k] | â[k], δx̂_3, 軌跡 | (1/â)(δp_d + (1-λ_c)δx̂_3) |
| Fe(3,4)[k] | f_d[k] | -f_d[k] |
| Q(3,3)[k] | â[k] | 4k_BT × â[k] |
| Q(4,4)[k] | 軌跡速度, wall 梯度 | (da/dh × dh/dt × T_s)² |
| L[k] | P^f[k], R | P^f H'/(H P^f H' + R) |
| P^f[k] | Fe[k], Q[k], L[k] | recursion |

## 兩條資訊通道

### 通道 1: del_pm → â（dynamics channel）

```
機制: a 錯 → f_d 效果不對 → tracking error 有系統性 pattern → del_pm 攜帶 a 資訊
數學: Fe(3,4) = -f_d 耦合 → P^f(4,1) ≠ 0 → L(4,1) ≠ 0
強度: 正比於 f_d（f_d=0 時通道關閉）
```

### 通道 2: a_m → â（variance channel）

```
機制: thermal variance ∝ a → 量 variance → 反推 a
數學: a_m 作為量測 → innovation_2 = a_m - â → L(4,2) × innovation_2 修正 â
強度: 正比於 Q(4,4)/R(2,2)
問題: a_m 怎麼算？需要 variance → a 的轉換關係
```

## Q 的物理推導（不是 tuning）

### Q(3,3)[k] = 4k_BT × â[k]

thermal force 對 estimation error e_3 的驅動。
物理值，但 KF 不知道真 a，用 â 代替。

### Q(4,4)[k] = (da/dh × dh/dt × T_s)²

a 每步的實際變化量的 variance。

da/dh = -T_s × c'_perp(h/R) / (gamma_N × c_perp² × R)
dh/dt = 軌跡速度（已知）

Time-varying:
- 軌跡極值（峰/谷）: dh/dt ≈ 0 → Q(4,4) ≈ 0
- 軌跡快速段: dh/dt 大 → Q(4,4) 大
- 近壁面: da/dh 大（梯度陡）→ Q(4,4) 更大

## R 的物理推導

### R(1,1) = σ²_n

visual tracking noise。物理量，已知。

### R(2,2) = Var(a_m - a_true)

取決於 a_m 的計算方法。**這是未解決的核心問題。**

## KF 的 P^f Recursion

```
L[k] = P^f[k] H' / (H P^f[k] H' + R)
P[k] = (I - L[k]H) P^f[k]
P^f[k+1] = Fe[k] P[k] Fe[k]' + Q[k]
```

P^f(4,4)[k] = KF 認為 â 的誤差 variance → â 的理論精度。

## Σ_e Recursion（驗證工具）

```
e = [δx, e_1, e_2, e_3, e_a, e_δa]' (6x1)
Σ_e[k+1] = A_6[k] Σ_e[k] A_6[k]' + noise covariance

(Σ_e)_11 = tracking error 的真實 variance
(Σ_e)_55 = gain estimation error 的真實 variance
```

Σ_e 和 P^f 的關係:
- P^f 是 KF 認為的 estimation error covariance
- Σ_e(2:6, 2:6) 是實際的 estimation error covariance
- 如果 Q, R 正確 → 兩者匹配（KF consistent）
- 如果不匹配 → Q 或 R 有誤

## 核心未解決問題

**a_m 怎麼算？**
- IIR + C_dpmr: C_dpmr 公式對 5/7-state EKF 不正確
- Σ_e inversion: 需要 â → 循環依賴
- 不用 a_m（單量測）: 只靠 f_d 耦合，需要足夠 excitation

## 驗證結果

### Σ_e recursion 驗證（a[k] 已知）
- Stationary: recursion vs Lyapunov = 0.002%
- Time-varying (MC 500): mean error 5.05%, R^2 = 0.915

### 5-state gain estimation（1D 模擬）
- Oracle: RMSE=42%, corr=0.74（受 Fe clamp 限制）
- Eq.13: RMSE=42%, corr=0.48
- Σ_e: RMSE=46%, corr=0.46
- 注意: 這是簡化的 1D 模擬，不代表 Simulink 的 ctrl5/ctrl7 性能
- Simulink ctrl5 已驗證: RMSE ~6%, corr=0.94（之前的結果）
