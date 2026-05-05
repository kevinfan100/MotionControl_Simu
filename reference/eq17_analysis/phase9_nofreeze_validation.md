# Phase 9 No-Freeze Validation Results

**Date**: 2026-05-05
**Branch**: test/eq17-5state-ekf
**Sims**: 5 seeds × 30s × α=0.05 × σ²_n=baseline × h_init=50 × **NO a_hat freeze**

## Setup vs Phase 9 Wave 1

Same as Phase 9 Wave 1 but `scenario.freeze_a_hat = false`:
- `ctrl_const.a_hat_freeze` not set
- KF freely updates slot 6 (a_hat) each step
- All other conditions identical (positioning, σ²_w_fA=0, baseline σ²_n)

`run_R22_validation.m` extended with `scenario.freeze_a_hat` flag (default true for backward compat).

## Sanity Gates

| Gate | Result |
|---|---|
| `delta_a_hat (slot 7) max` | 0.000e+00 全程 ✓ (A1 結構性凍結仍成立) |
| `Q77 max` | 0 ✓ |
| `guards inactive` | ✓ |

Slot 7 zero proof robust under no-freeze (P_init(7,7)=0 + Q77=0 → structural).

## Key Findings

### 1. a_hat 系統性偏低 5.5-8% (跨 axes)

| Axis | a_TRUE [µm/pN] | mean(a_hat) [µm/pN] | bias |
|---|---|---|---|
| x | 1.4334e-2 | 1.3538e-2 | **-5.55%** |
| y | 1.4334e-2 | 1.3386e-2 | **-6.61%** |
| z | 1.3962e-2 | 1.2843e-2 | **-8.02%** |

a_hat std (per axis, cross-seed) ≈ 0.17-0.21% of mean → 數值穩定但 settled at biased value.

**This IS the Wave 4 v3 a_hat bias.** Phase 9 freeze 強制 a_hat=a_TRUE 一直 mask 它，no-freeze 揭露。

### 2. R(2,2) 公式對「給定 a_hat 操作點」仍正確

| Mode | V1 ratio (mean across axes) |
|---|---|
| freeze (Phase 9) | 0.97-0.99 ✓ |
| no-freeze (using mean(a_hat)) | 1.03-1.04 ✓ |

兩者 within ±5% PASS。**R(2,2) 公式正確性跟 a_hat bias 是獨立兩件事**。

### 3. V5 mean(a_xm) bias 隨之惡化

| Mode | V5 bias x | V5 bias z |
|---|---|---|
| freeze | -0.37% | -1.06% |
| no-freeze | -2.19% | **-3.44%** |

a_hat 偏低 → 1/a_hat 偏高 → f_d scale 偏 → closed-loop 偏 → mean(a_xm) shift。Stage 11 inversion 校準是針對 a_TRUE 設計，no-freeze a_hat 偏離後 inversion 不再完美。

### 4. Var(a_xm) no-freeze < freeze 約 7-9%

| axis | freeze Var | no-freeze Var | nofz/freeze |
|---|---|---|---|
| x | 3.587e-5 | 3.325e-5 | 0.927 |
| y | 3.553e-5 | 3.263e-5 | 0.918 |
| z | 3.328e-5 | 3.016e-5 | 0.906 |

a_hat 偏低 → (a_hat+ξ)² 偏小 → R(2,2) 預測值偏小 → empirical Var 也偏小（heteroscedastic noise 跟 a 連動）。

## 結論

### 對 Phase 9 R(2,2) 驗證

R(2,2) Layer 1 公式驗證通過 in BOTH:
- freeze conditions (controlled experiment)
- no-freeze conditions (production-like, using empirical operating point)

R(2,2) 公式本身**沒問題**。

### 對 Wave 4 v3 a_hat bias 調查

之前 V6 indistinguishable 已證偽「a_hat bias 來自 ξ choice」。
no-freeze test 進一步：
- 確認 a_hat bias 真實存在，量級 5-8% 跨軸一致
- 證明 a_hat bias **跟 R(2,2) 公式無關**（公式驗證通過）
- 嫌疑剩：Q77 motion estimate / motion C_dpmr_eff fixed-at-init / x_D 耦合 / 其他 closed-loop 機制

### 結論的價值

Phase 9 + no-freeze 一起完成了 R(2,2) Layer 1 production-realistic validation，並把 a_hat bias 真兇調查的搜尋空間**收窄**（排除 R(2,2)、ξ）。

## Files

**Tracked in commit**:
- `test_script/run_R22_validation.m` (+ `scenario.freeze_a_hat` option)
- `reference/eq17_analysis/phase9_nofreeze_validation.md` (this doc)

**Untracked (consistent with raw sim output convention)**:
- `reference/eq17_analysis/phase9_results_NoFreeze_seed{1..5}.mat` (5 × ~15 MB)

## Next Step Candidates

1. Wave 4 a_hat bias root cause investigation (highest priority — production blocker)
2. R(2,2) Layer 2 / Layer 3 validation (motion / edge cases)
3. X1 augmented Lyapunov closed form
