# 7-state EKF — 全部相關參數清單與意義 (h=50 final)

**Date**: 2026-04-27
**Branch**: `test/qr-paper-reference`
**對應圖檔**: `reference/qr_analysis/fig_qr_verification_h50/fig1_gain_estimation.png`、`fig2_tracking_error.png`
**Commit chain**:
- `add0876` — Stage 1-3 Q/R per-axis fix + Q(del_a)=0（驗證主體）
- `1c4755a` — 圖檔首次加入
- `0513741` — 圖加 ±1σ theory 參考線
- `b1ce941` — `ekf_out(3:4) = [a_m(1); a_m(3)]` 暴露 IIR Eq.13 量測（fig1 加 "Measured" 線）

**模擬主檔**: `test_script/verify_qr_positioning_run.m` 的 `variants_all(7)` (`frozen_peraxisR11_R22_Q770`)
**Run-level 設定**: `h_init=50 µm`、`trajectory_type='positioning'`、`T_sim=15 s`、`t_warmup=2 s`、`seeds = [12345, 67890, 11111, 22222, 33333]`

依「進入 EKF 的角色」分組，所有引用均對應 `model/controller/motion_control_law_7state.m`。

---

## A. State / Model 層（per-axis 各跑一份 7-state）

### A.1 狀態向量 (7×1)

```
x = [del_p1; del_p2; del_p3; d; del_d; a; del_a]
     1       2       3       4  5      6  7
```

| Idx | 變數 | 物理意義 |
|---|---|---|
| 1 | `del_p1` | 追蹤誤差延遲鏈 (k 時刻) |
| 2 | `del_p2` | 追蹤誤差延遲鏈 (k−1) |
| 3 | `del_p3` | 追蹤誤差延遲鏈 (k−2)；經 λ_c 一階 LP |
| 4 | `d` | 干擾估計（disturbance） |
| 5 | `del_d` | 干擾微分 |
| 6 | `a` | **運動增益** (=Ts/(γ_N·c_axis))，Stage 3 主鎖目標 |
| 7 | `del_a` | 增益微分；**Stage 3 將其 Q 設為 0** |

→ `motion_control_law_7state.m:9, :243-248, :384`

### A.2 量測向量 (2×1)

```
y = [del_pm_i; a_m_k1_i]
H = [1 0 0 0 0 0 0;     ← del_pm picks state(1) = del_p1
     0 0 0 0 0 1 0]     ← a_m   picks state(6) = a
```

a_m 量測 **使用上一步的值**（`a_m_k1`），對應 IIR 估計天然有一拍延遲 → `:251-253`。

### A.3 Process matrix `Fe_err` (7×7) — 誤差動態（非標準 Jordan-block）

```
Fe_err(3,3) = 1                    ← Eq.18 closed-loop 誤差自迴授（不是 0！）
Fe_err(3,6) = -f_d(axis)           ← 增益誤差透過控制力進入 del_p3
Fe_err(6,7) = 1                    ← a + del_a 雙積分結構
z 軸額外: Fe_err(4,4)=1+β, Fe_err(4,5)=-β, Fe_err(6,6)=1+β, Fe_err(6,7)=-β
```

→ `:277-298`。z 軸的 β coupling 是 chart-extension predictor，預設 β=0 退化為 x/y 形式。

---

## B. Q / R / Pf — KF 三大協方差矩陣

### B.1 Process noise Q (7×7) — 由 `Qz_diag_scaling` 構建

**Bus 維度: 9×1**（2026-04-22 擴充，原本 7×1）

```
Qz_diag_scaling = [Q1; Q2; Q3; Q4; Q5; Q6;  Q7_x; Q7_y; Q7_z]
                   ←─── 三軸共用 ─────→  ←─── per-axis ───→

Q_axis(i,i)[1..6] = sigma2_deltaXT * Qz_diag_scaling(i)
Q_axis(7,7)       = sigma2_deltaXT * Qz_diag_scaling(6+axis)
```

→ `:108-116`

| Slot | 對應狀態 | 物理意義 / h=50 final 值 |
|---|---|---|
| Q(1) | del_p1 | 0（無 process noise） |
| Q(2) | del_p2 | 0 |
| Q(3) | del_p3 | **1** — 吸收 thermal force 進 del_p3 通道 |
| Q(4) | d | 0 |
| Q(5) | del_d | 0 |
| Q(6) | a | **1e-8** — 給 a 微小漂移自由度（frozen 設定） |
| Q(7)_x/y/z | del_a | **0 / 0 / 0** — Stage 3 修正：positioning 下 del_a 真為常數 0，不灌虛假噪聲 |

### B.2 Measurement noise R (2×2 per-axis) — 由 `Rz_diag_scaling` 構建

**Bus 維度: 6×1**（2026-04-21 擴充，原本 2×1）

```
Rz_diag_scaling = [R_pos_x; R_pos_y; R_pos_z;  R_gain_x; R_gain_y; R_gain_z]
R(1,1)_axis = sigma2_deltaXT * Rz_diag_scaling(axis)        ← position channel
R(2,2)_axis = sigma2_deltaXT * Rz_diag_scaling(3+axis)      ← gain channel
```

→ `:263-274`

| Slot | 量測通道 | 物理意義 / h=50 final |
|---|---|---|
| R(1,1)_x | del_pm_x noise | σ²_n_x/σ²_dXT = **1.53e-3** |
| R(1,1)_y | del_pm_y noise | **1.29e-5** |
| R(1,1)_z | del_pm_z noise | **4.35e-2** |
| R(2,2)_x | a_m_x noise | chi_sq·ρ_a·a_x²/σ²_dXT = **0.01881** |
| R(2,2)_y | a_m_y noise | **0.01881** |
| R(2,2)_z | a_m_z noise | **0.01784** |

**Adaptive gate** (`:262-273`): 當 `del_pmr_var(i) < sigma2_dXT × 0.001` 時，`R_gain_i = 1e6` 強制把 a_m 通道閉掉（防止無 thermal 激擾時 EKF 信任 a_m=0）。

### B.3 Forecast covariance 初值 Pf_init (7×7 per-axis) — `Pf_init_diag`

```
Pf_init_diag = [0; 0; 1e-4; 1e-4; 0; 1e-5; 0]   ← h=50 final
```

→ `:101-105`

| Slot | 初值 | 意義 |
|---|---|---|
| (3,3) / (4,4) | 1e-4 | del_p3 與 d 給予不確定度啟動 |
| (6,6) | **1e-5** | a 起始極低不確定度 — frozen 設定的關鍵之一 |
| 其餘 | 0 | 完全相信初值 |

---

## C. Forgetting / Coupling

| 參數 | 值 | 出處 | 意義 |
|---|---|---|---|
| `lamdaF` | 1.0 | `:75, :309` | EKF 遺忘因子；Eq.20 後驗 P 除以 λ_F；x/y 強制 1，z 採 `lamdaF` 設定（仍 1.0 = 標準 KF，無遺忘） |
| `beta` | 0 | `:74, :291-298` | z 軸 chart-extension coupling；改 Fe_err(4,4)、Fe_err(6,6) 結構，預設 0 退化 |

---

## D. 影響 EKF 行為但住在 IIR / 控制層的參數

這些**不是 KF 矩陣**，但會改變送進 KF 的量測 a_m 統計，因此同樣是 KF 性能調諧的一部分。

| 參數 | 值 | 出處 | 對 KF 的角色 |
|---|---|---|---|
| `lambda_c` | 0.7 | `:68` | 閉迴路極點；進 Fe_err(3,3 chain 的 LP)、控制律、DARE 解；決定 del_p3 的 ρ_a |
| `a_pd` | 0.05 | `:69, :187` | IIR LP 平滑（提取 deterministic 分量） |
| `a_prd` | 0.05 | `:70, :190` | HP residual 均值 EMA |
| `a_cov` | **0.005** | `:71, :191` | HP residual **二階** EMA；決定 a_m 量測的 sampling noise 強度 — `chi_sq = 2·a_cov/(2−a_cov)` 直接出現在 R(2,2) 的物理推導 |
| `epsilon` | 0.01 | — | anisotropy threshold（θ 量測閾值，本場景未觸發） |
| `warmup_count` | 2 (P6) | `:166-173, :211-232` | IIR pre-fill 後跳過控制+EKF 的步數；舊版 320 步反而有害 |
| `var_threshold` | σ²_dXT·0.001 | `:262` | adaptive R(2,2) 閘門 |

---

## E. 物理常數（KF 矩陣的尺度錨）

| 變數 | 出處 | 值 / 公式 | 用途 |
|---|---|---|---|
| `sigma2_deltaXT` | `:77, :113` | 4·k_B·T·a_nom = **2.52e-4 µm²** | Q、R 的整體 scaling unit |
| `sigma2_noise` (3×1) | `:76, :157` | [σ²_n_x; σ²_n_y; σ²_n_z] | sensor 噪聲，物理推導 R(1,1) per-axis |
| `gamma_N`, `Ts` | `:66-67` | a_nom = Ts/γ_N | 無壁時的 nominal 增益 |
| `k_B`, `T_temp` | `:72-73` | — | thermal 力統計（影響 σ²_dXT 與 Eq.13 分母） |

---

## F. EKF init 的「看牆」處理

```matlab
h_bar_init    = (p0·w_hat − pz) / R
[c_para, c_perp] = calc_correction_functions(h_bar_init)
a_hat(0)  = [a_nom/c_para; a_nom/c_para; a_nom/c_perp]   ← per-axis
del_a_hat(0) = 0
```

→ `:88-99`。**避免 EKF 從 a_nom 慢慢爬到 a_nom/c_perp(h)** 的 transient。

---

## G. Eq.13 增益遞迴（量測 a_m 的計算路徑）

a_m 不是 sensor 直接讀，而是從 IIR HP residual 變異數反推：
```
a_m = max( (del_pmr_var/IIR_bias_factor − C_np·σ²_n) / (C_dpmr·4·k_B·T), 0 )
```

→ `:204-209`

| 常數 | 出處 | 來源 |
|---|---|---|
| `C_dpmr_eff_const` | `:120-130` | `cdpmr_eff_lookup.mat` 由 augmented Lyapunov 預算；fallback 為 K=2 公式 |
| `C_np_eff_const` | `:120-130` | 同上；fallback `2/(1+λ_c)` |
| `IIR_bias_factor_const` | `:135-139` | `bias_factor_lookup.mat`；finite-sample + autocorrelation 校正，fallback 1.0 |

這些 lookup 一旦改變，會直接改 a_m 量測值的尺度，間接改變 R(2,2) 的合適值 — 它們和 KF 是耦合校準關係，不是獨立可調。

---

## H. 一張總表 — h=50 final 全部數值

| 群組 | 參數 | 值 |
|---|---|---|
| **A** Model | states / 軸 | 7 |
| | β / λ_F | 0 / 1.0 |
| **B** Q | Qz_diag_scaling (9) | [0;0;1;0;0;1e-8; 0;0;0] |
| **B** R | Rz_diag_scaling (6) | [1.53e-3; 1.29e-5; 4.35e-2; 0.01881; 0.01881; 0.01784] |
| **B** Pf | Pf_init_diag (7) | [0;0;1e-4;1e-4;0;1e-5;0] |
| **C** | λ_c | 0.7 |
| **D** IIR | a_pd / a_prd / a_cov | 0.05 / 0.05 / 0.005 |
| **D** | warmup_count | 2 |
| **D** | var_threshold | σ²_dXT × 0.001 |
| **E** scale | σ²_dXT | 2.52e-4 µm² |
| **E** sensor | meas_noise_std | [0.00062; 0.000057; 0.00331] µm |

---

## I. 場景 / Run-level 設定（產生圖的條件）

| 類別 | 參數 | 值 |
|---|---|---|
| Trajectory | `trajectory_type` | `'positioning'` |
| | `h_init` = `h_bottom` | 50 µm |
| | `amplitude` / `t_hold` / `n_cycles` | 0 / 0 / 1 |
| Wall | `theta` / `phi` / `pz` | 0 / 0 / 0 (w_hat = ẑ) |
| | `enable_wall_effect` | true |
| | `h_min` | 1.1 × 2.25 = 2.475 µm |
| Controller | `controller_type` | 7 |
| | `ctrl_enable` | true |
| Noise | `meas_noise_enable` | true |
| | `meas_noise_std` | [0.00062; 0.000057; 0.00331] µm |
| | `thermal_enable` | true |
| Simulation | `T_sim` | 15 s |
| | `t_warmup` | 2 s（穩態評估視窗起點） |
| | `thermal_seed` | [12345, 67890, 11111, 22222, 33333] (5 seeds) |

---

## J. 已驗證性能（5-seed CI）

| 指標 | 軸 | Theory | Measured (mean ± std) | Ratio |
|---|---|---|---|---|
| Tracking std (nm) | x | 34.91 | 34.88 ± 0.45 | **0.999** ✓ |
| Tracking std (nm) | z | 34.59 | 34.73 ± 0.30 | **1.004** ✓ |
| a_hat std (%) | x | 5.34 | 4.92 ± 0.38 | **0.92** ✓ |
| a_hat std (%) | z | 5.38 | 4.50 ± 0.81 | **0.84** ✓ |
| 3D RMSE (nm) | — | — | 60.22 ± 0.42 | — |

四個 ratio 全部落在 [0.84, 1.00] — 工程級 theory↔sim 通過。

完整推導 / 公式 / 各 Stage 解析參見 `reference/qr_analysis/qr_verification_h50_final_2026-04-22.md`。

---

## K. 已知 GAP / 注意事項

1. **Stage 3 配置目前不是 `apply_qr_preset.m` 的 named preset**：最新 case 只到 `frozen_correct_peraxisR11_R22`（Q(7,7)=1e-8）。Stage 3 (Q(7,7)=0) 只活在 `verify_qr_positioning_run.m` 的 hardcoded `variants_all(7)`。要一鍵重生需另加 case `frozen_correct_peraxisR11_R22_Q770`。

2. **R(2,2) 沒有 h-相依**：當前 0.01881 / 0.01784 是為 h=50 量身（含 a_axis²），h=2.5 不適用。

3. **Q(7,7) = 0 只對 positioning 成立**：a 不變才能說 del_a=0。motion / oscillation 下需另推 Q(7,7)。

4. **z 軸 ratio 0.84 殘餘 14% 落差**：可能來自 ρ_a Lyapunov 殘差 ~5%、scalar L_eff 對 z 略 over-predict、closed-loop del_pm 非 Gaussian tail。

5. **出圖腳本本身未 commit**：圖是手動繪製或臨時 script，repo 內無法一鍵重生。
