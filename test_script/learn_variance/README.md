# learn_variance — Variance Estimate 推導與驗證 Sandbox

脫鉤 Simulink 的 R22 (variance-of-variance) 閉式推導與驗證環境。
研究 controller IIR variance chain: LP1(a_pd) → HP1 → EWMA(a_cov)。

詳細推導記錄: `test_results/learn_variance/derivation_IF_eff.md` (本機，未入庫)
變更記錄: `test_results/learn_variance/CHANGE_RECORD.md` (本機，未入庫)

> **Port 注意 (2026-06-02)**: 本 sandbox 從 Mac 分支 learn/variance-walkthrough
> port 到新架構 (pure_matlab → dual_track, eq17_7state → eq17_core)。
> 純閉式腳本 (derive_*, confirm_*, verify_Cdpmr_intrinsic, verify_R22_intrinsic,
> verify_rho_static) 不依賴 controller 行為，結果不變。
> 但 `run_q_unification_h50.m`、`isolation_ahat_freeze.m`、`verify_three_dx1_variances.m`
> 等模擬類腳本依賴 Mac 端的 controller 行為變更 (Q66_OL/force_Q77_zero defaults +
> F_e Row-3 F_1/F_2)，該變更**尚未** port 進 eq17_core — 在行為 port 完成前，
> 這些腳本跑出的數值不等於 Mac 端驗證過的結果 (README「核心結果」一節的數字)。

---

## 核心結果

- C_dpmr、C_n、R_T(τ)、R_N(τ) 閉式 — 機器精度驗證 (<8e-15)
- IF_eff 三項幾何閉式 (paper Eq.21 H_T/H_n ∘ HP1 ∘ LP1, d=2)
- R22 ratio: 0.74 (legacy OptA) → 0.94 (closed-form) → 1.00 (freeze a_hat)
- isolation 測試確認: 5% production 殘差 = EKF a_hat 非線性反饋 (paper 未建模)

---

## 腳本分類

### 繪圖基礎設施
| 檔案 | 用途 |
|---|---|
| plot_style.m | 色票/字級/佈局規範 |
| apply_default_style.m | 套用樣式到 figure |
| save_figure.m | exportgraphics 存檔 helper |
| new_stacked_figure.m | 多層 stacked time-series figure |
| preview_plot_style.m | 樣式預覽 |
| preview_individual.m | 個別 case 預覽 |

### 輸入合成 / 訊號準備
| 檔案 | 用途 |
|---|---|
| get_ramp_delpm.m | 從 ramp 模擬取 del_pm 訊號 |
| analyze_ramp_delpm.m | ramp del_pm 分析 |
| compare_am.m | a_m / C_dpmr / C_n 閉式對照 |

### 閉式推導
| 檔案 | 用途 |
|---|---|
| derive_Cn_closed_v2.m | C_n 閉式推導 (canonical, sum-of-squares 法) |
| derive_rho_dpmr_step1.m | Phase 1: R_T(τ)/R_N(τ) 閉式 vs brute-force |
| derive_rho_dpmr_step2.m | Phase 2: ρ_δp_mr 加權合成 vs empirical |

### 驗證
| 檔案 | 用途 |
|---|---|
| verify_Cdpmr_intrinsic.m | C_dpmr intrinsic 驗證 |
| verify_Cdpmr_comprehensive.m | C_dpmr 跨參數驗證 |
| verify_Cdpmr_offline.m | C_dpmr offline IIR 驗證 |
| verify_Cn_thermal_off.m | C_n 驗證 (thermal off, 隔離 sensor noise) |
| verify_R22_intrinsic.m | R22 intrinsic 驗證 |
| verify_rho_static.m | static positioning ρ 驗證 (h=20um, 5 seeds) |
| verify_positioning_grid.m | 27-sim grid (3 h × 3 a_pd × 3 seeds × 3 軸) |
| verify_acov_and_tau.m | a_cov sweep + tau_max 敏感度 |

### 診斷 / 隔離
| 檔案 | 用途 |
|---|---|
| diagnose_IF_eff_empirical.m | empirical IF_eff 診斷 |
| isolation_ahat_freeze.m | freeze a_hat=a_true 隔離測試 (確認 5% 殘差來源) |

---

## 開放 TODO

- §8 IF_eff 三項幾何閉式取代 numerical truncated sum
- §9 整合 §8 進 production code
- (research) EKF a_hat 反饋包進 paper closed-loop → ratio→1.00
