# Theory Status Summary (2026-04-20)

完整整理目前 tracking std + a_hat std 的理論預測能力, 計算方式, 以及 KF
參數的推導 vs tuning 分類。

---

## A. 兩個指標的預測結果 (P7 verification, 3 seeds × 2 scenarios × 2 variants)

### A.1 Tracking error std (per axis, nm)

| 場景 | 軸 | Theory (Sigma-based) | Measured | Ratio |
|---|---|---|---|---|
| h=2.5 | X | 23.82 | 23.76 ± 0.55 | **0.997** |
| h=2.5 | Y | 23.82 | 23.91 ± 0.19 | **1.004** |
| h=2.5 | Z | 11.28 | 11.75 ± 0.14 | **1.041** |
| h=50 | X | 35.75 | 35.66 ± 0.90 | **0.997** |
| h=50 | Y | 35.75 | 35.88 ± 0.29 | **1.004** |
| h=50 | Z | 35.31 | 35.46 ± 0.53 | **1.004** |

**結論**: 5/6 cells **±0.4% 內**, 最壞 +4.1% (h=2.5 Z)。**接近完美匹配**。

### A.2 a_hat estimation std (relative %, per axis)

| 場景 | variant | Theory | Meas X | Meas Z | Ratio X | Ratio Z |
|---|---|---|---|---|---|---|
| h=2.5 | frozen | 1.46 | 13.78 | 4.52 | 9.4× | 3.1× |
| h=50 | frozen | 1.46 | **14.04** | **1.68** | 9.6× | **1.15× ✓** |
| h=2.5 | empirical | 3.21 | 18.78 | 19.69 | 5.8× | 6.1× |
| h=50 | empirical | 3.21 | 18.76 | 18.39 | 5.8× | 5.7× |

**結論**: 只有 **frozen z h=50 (paper 主場景) ±15% 對齊**。其他 3-9× 偏差。

---

## B. a_hat_x 9× 偏差的根因 (誠實版本)

之前歸因「f_d 耦合」**未充分驗證**。Diagnostic (frozen h=50 seed 12345):
- a_true_x 是常數 (std 0.002%)
- a_hat_x 範圍 [0.0094, 0.021], 真在大幅漂移 ±50%
- a_hat_z 範圍 [0.0135, 0.0141], 穩定 ±2%

**為什麼 frozen 下 x 漂移而 z 不漂移?** 三個候選**沒一個被排除**:

1. **Per-axis sensor noise 失配 (designer R 同一個)**
   - `R(1,1)_designer = sigma2_dXT * 0.397 ≈ 1e-4 um²` (固定)
   - 實際 sigma2_n_x = 6.2e-7 um² (designer 過估 160×)
   - 實際 sigma2_n_z = 1.1e-5 um² (designer 過估 9×)
   - x 軸 EKF 過度信任「位置 noisy」→ 把實際很小的位置擾動歸給 a 漂移
   
2. **f_d 軸別不同 (multiplicative coupling)**
   - Fe_err_x(3,6) = -f_d(1), Fe_err_z(3,6) = -f_d(3)
   - x 與 z 軸的 f_d 即時值不同 → A_e 動態不同
   - 我的 11-dim Lyapunov 用 opts.f0 = 0 漏這個
   
3. **Q(7,7) random walk 雙積分放大**
   - frozen 設 Q(7,7) = 1e-8 (small but nonzero)
   - del_a_hat 隨 Q77 walk → integrate → a_hat drift
   - L_ss(6,2) = 1e-4 弱抑制 → 雙積分變異放大 ~1/L³ = 36000
   - 實際模擬中 (沒有真實 Q77 driver), del_a 的 random walk 被 measurement 部分抑制, 但抑制程度 axis-dependent

**結論**: 9× 不是單一原因, 是上述三者組合。**最可能主導 #1 + #3** (sensor noise mismatch 讓 x EKF 把雜訊歸到 a, 然後 Q77 双積分放大)。

---

## C. 怎麼用程式計算理論預測

### C.1 Tracking error std (per axis)

```matlab
% Inputs needed:
%   lc = 0.7, a_pd = 0.05
%   Q_kf_scale = config.Qz_diag_scaling  (7x1)
%   R_kf_scale = config.Rz_diag_scaling  (2x1)
%   sigma2_dXT = 4*k_B*T*a_nom
%   a_phys_per_axis (use a_nom/c_para for x,y; a_nom/c_perp for z)
%   sigma2_n_per_axis = config.meas_noise_std(i)^2

opts.physical_scaling = struct( ...
    'sigma2_dXT', sigma2_dXT, ...
    'a_phys',     a_phys_per_axis, ...
    'a_nom',      a_nom, ...
    'sigma2_n',   sigma2_n_per_axis);
[~,~,~,~,dgn] = compute_7state_cdpmr_eff(lc, 0, a_pd, Q_kf_scale, R_kf_scale, opts);

tracking_std_axis_um = sqrt(dgn.Sigma_aug_phys(dgn.idx_dx, dgn.idx_dx));
% convert to nm
tracking_std_axis_nm = tracking_std_axis_um * 1000;
```

每軸算一次。h=50 ar 接近, x/y/z 預測接近; h=2.5 ar 不同, x/y vs z 預測差很多。

**精度: ±4% in-and-out**.

### C.2 a_hat std (per axis)

```matlab
% Inputs (extra):
%   a_cov = config.a_cov
%   chi_sq = 2*a_cov / (2 - a_cov)
%   rho_a = 4    (constant approx)
%   actual_a_m_var = chi_sq * rho_a * a_phys_per_axis^2

opts.physical_scaling.actual_a_m_var = actual_a_m_var;
% (re-compute as above)

ahat_std_per_axis_pct = 100 * sqrt(dgn.Sigma_e6_phys) / a_phys_per_axis;
```

**精度: 1.15× (best, frozen z h=50) 到 9× (worst, frozen x)**. 適用於**檢驗趨勢**而非 point prediction.

---

## D. KF 參數: DERIVED vs TUNING (frozen_correct preset)

### Q (7x1 process noise scaling), R (2x1 measurement noise scaling)

| idx | 變數 | 值 | 類別 | 推導/原因 |
|---|---|---|---|---|
| Q(1,1) | del_p1 | 0 | **STRUCTURAL** | paper Eq.14 delay chain: del_p1[k+1]=del_p2[k]+inj, 純確定性, 無 process noise |
| Q(2,2) | del_p2 | 0 | **STRUCTURAL** | 同上 |
| Q(3,3) | del_p3 | 1 | **DERIVED (近似)** | 物理 = (a/a_nom)²*sigma2_dXT (per-axis); 用常數 1 近似 free-space (Stateflow 阻 per-step adapt) |
| Q(4,4) | x_D | 0 | **TUNING (=0)** | x_D 是模擬中無實質 disturbance source |
| Q(5,5) | del_x_D | 0 | **TUNING (=0)** | 同上 |
| Q(6,6) | a | 1e-8 | **TUNING (frozen)** | KEY: 接近 0, 視 a 為常數; β interp 推導值 1.34e-11 但 simulation drift |
| Q(7,7) | del_a | 1e-8 | **TUNING (frozen)** | 同 Q(6,6) |
| R(1,1) | del_pm | 0.397 | **DERIVED** | = sigma2_n_z / sigma2_dXT (sensor spec 與熱物理比); 注意: 用 z-axis sensor noise, 對 x-axis 不準 |
| R(2,2) | a_m | 1.0 | **TUNING (低於理論)** | 理論 self-consistent fixed point 1.72; 但 1.0 表現更好 (Var(V_meas) 是 upper bound) |

### Pf_init (7x1 initial covariance)

| idx | 變數 | 值 | 類別 | 推導/原因 |
|---|---|---|---|---|
| (1) | del_p1 | 0 | **DERIVED** | positioning init 已知 = 0 |
| (2) | del_p2 | 0 | **DERIVED** | 同上 |
| (3) | del_p3 | 1e-4 | **TUNING (small)** | 給少量初始彈性 |
| (4) | x_D | 1e-4 | **TUNING (small)** | x_D 初始未知度 |
| (5) | del_x_D | 0 | **TUNING (=0)** | EKF 起始 |
| (6) | a | **1e-5** | **TUNING (frozen 配套)** | KEY: 小 → 高初始信任; 配 Q(6,6)≈0 → frozen |
| (7) | del_a | 0 | **TUNING (=0)** | 起始已知 = 0 |

### 其他 EKF 參數

| 參數 | 值 | 類別 | 原因 |
|---|---|---|---|
| **a_cov** | **0.005** | **TUNING** | KEY: 小 → 低 chi-sq 注入 → 低 a_hat std |
| a_pd | 0.05 | TUNING | IIR LP smoothing |
| a_prd | 0.05 | TUNING | HP residual mean smoothing |
| beta | 0 | DESIGN | canonical (paper 沒用 beta extension) |
| lamdaF | 1.0 | TUNING | 1.0 = standard KF |
| lambda_c | 0.7 | DESIGN | closed-loop pole |
| warmup_count | 2 | DESIGN | P6 修, 原 320 有害 |
| del_pmr2_avg init | per-axis steady (P5) | **DERIVED** | Cdpmr*sigma2_dXT*ar + Cnp*sigma2_n |

### 統計

| 類別 | 數量 | 列舉 |
|---|---|---|
| **STRUCTURAL** (零值, 不可改) | 2 | Q(1,1), Q(2,2) |
| **DERIVED** (有公式) | 5 | Q(3,3) (approx), R(1,1), Pf_init(1,1), Pf_init(2,2), del_pmr2_avg init |
| **TUNING (=0)** (無實質源) | 4 | Q(4,4), Q(5,5), Pf_init(5,5), Pf_init(7,7) |
| **TUNING (frozen 核心)** | 4 | Q(6,6), Q(7,7), Pf_init(6,6), a_cov |
| **TUNING (其他)** | 5 | R(2,2), Pf_init(3,3)/(4,4), a_pd, a_prd, lamdaF |
| **DESIGN (架構)** | 3 | beta, lambda_c, warmup_count |

---

## E. 工程級結論 + Thesis 可寫範圍

### E.1 可信任的部分 (寫進 thesis)
- **Tracking std** by 11-dim Lyapunov: ±4% across 6 (h, axis) cells
- **a_hat z std for free-space frozen**: ±15% (paper 主場景)
- **Cdpmr_eff lookup formula**: linear ar scaling consistent with thermal_force per-axis Einstein model

### E.2 部分可信 (有 caveat)
- a_hat z std for near-wall frozen: 3× 偏差, 應該歸因於 wall-effect lookup 內插誤差或 transient memory
- a_hat empirical 全部: 6× under-predict, 應歸因於 chi-sq chain 含 closed-loop autocorrelation (rho_a) 用常數 4 不足

### E.3 不可信 (列為 future work)
- a_hat_x std (任何場景): 9× 偏差。根因不確定, 可能 sensor R 設計 + Q77 雙積分 + f_d 耦合的組合
- 任何 per-axis non-symmetric prediction (theory 對 x 和 z 預測相同, 實測差很多)

### E.4 改進路徑 (各 1-2 day)
1. **Per-axis R(1,1) in EKF design**: 用實際 sigma2_n_axis 而非常數 0.397
2. **Rigorous rho_a per scenario**: 用 compute_rho_a_rigorous (existing helper)
3. **12-dim Lyapunov 含 f_d_nom**: 抓 multiplicative coupling
4. **Time-varying Pf**: 含 init transient

---

## F. 相關 commits

- `9dca456` — bug fix (ar² → ar) in compute_7state_cdpmr_eff
- `171aa1f` — Sigma-based tracking_std + per-axis sigma2_n in lookup
- `9ba1b02` — Sigma_na + a_hat std theoretical prediction (partial)

## G. 文件

- `theory_correction_analysis.md` — bug fix + 3-method tracking std comparison
- `theory_tracking_and_ahat_v1.md` — original derivation walkthrough
- `theory_ahat_std_limitations.md` — Step 3 limitations
- **`theory_summary_2026-04-20.md`** (this file) — comprehensive status + how-to-compute
