# Task 1b Handoff: Investigate IIR Finite-Sample Bias

> **目的**：讓下一個 session 的 Claude 冷啟動後能直接接續工作。不需要讀完整對話。

## 一句話現狀

> Phase 1 修正 `C_dpmr_eff` (26%→4.5% Var 預測誤差) 後，`a_m` 仍有 **~8% 系統性負偏**。Task 1a 驗證 `f_0 ≠ 0` hypothesis 被**駁回**。下一步 Task 1b：定量分析 **IIR smoother 的有限樣本 bias + autocorrelation 放大**，看它能否解釋 ~8% 偏差。

## Branch & Latest Commit

- Branch: `feat/sigma-ratio-filter`
- Latest: `b0026b6` — `refactor(analysis): 2-D (lc, f_0) lookup replaces dummy (lc, rho) axis`

## 必讀背景（依優先順序）

1. **本檔案**（完整讀）
2. `reference/for_test/phase4_final_report.md`（跳到 "Phase 2" 和 "Remaining work" 小節）
3. `~/.claude/projects/.../memory/project_phase1_cdpmr_eff.md`（Task 1 Result 小節）
4. `reference/for_test/phase2_report.md`（Experiment 2A 的數字）

其他檔案**按需**讀，不必預先載入。

## 關鍵數字（不要重新驗證）

### Phase 1 (C_dpmr_eff 修正後)
```
C_dpmr_eff(lc=0.7, f_0=0) = 3.9242   (vs K=2 舊公式 3.1610, +24%)
C_np_eff(lc=0.7, f_0=0)   = 1.1141
Var(del_pmr) 預測精度:    26% → 4.5% (Simulink verify, 6 points)
```

### Phase 2A (30s free-space, lc=0.7)
```
mean(a_m_z) / a_nom = 92.5%  ← 7.5% 負偏
std(a_m_z) / mean   = 43.6%  ← chi-squared noise + autocorrelation
chi-squared 理論    = 31.6%  (sqrt(2/N_eff), N_eff=20)
autocorrelation 放大 = ~1.38x
```

### Phase 1 Simulink (10s verify)
```
Var(del_pmr) empirical / theory = 1.045  ← sample variance 比理論高 4.5%
```

### 關鍵矛盾
```
Phase 1: Var_sample > Var_theory by 4.5% (sample var 高)
Phase 2: a_m = Var_IIR/C      → mean 92% of a_nom (IIR 低 8%)

→ Var_IIR / Var_sample ≈ 0.88  (IIR 比 sample 低 12%)
```

**這個 12% 落差是 Task 1b 要解釋的核心。** 它來自 IIR smoother 的有限樣本 bias + autocorrelation 放大的組合。

### Task 1a 駁回的 hypothesis
```
f_0 dependence of C_dpmr_eff (lc=0.70):
  f_0 = 0  pN: C = 3.9242
  f_0 = 5  pN: C = 3.9244  (+0.005%)
  f_0 = 10 pN: C = 3.9250  (+0.02%)
  f_0 = 20 pN: C = 3.9272  (+0.08%)
```
→ `f_0 ≠ 0` 對 `C_dpmr_eff` 的影響 **< 0.1%**，不可能解釋 8% bias。**不要再試 f_0 方向**。

## 當前 Hypothesis (Task 1b 要驗證的)

IIR EMA variance estimator 的數學特性：

```
V_IIR[k] = EMA(x^2, a_cov) - (EMA(x, a_prd))^2

對 stationary Gaussian x ~ N(0, sigma^2):
  E[EMA(x^2)]     = sigma^2                                (無偏)
  E[(EMA(x))^2]   = (a_prd/(2-a_prd)) * sigma^2            (finite-sample variance)
  E[V_IIR]        = sigma^2 * (2 - 2*a_prd) / (2 - a_prd)  (系統性低偏)
                  = 0.974 * sigma^2   (at a_prd=0.05)
                  = 2.6% 低偏
```

但 del_pmr 有 **autocorrelation**（從閉迴路 dynamics 繼承）。autocorrelation 讓 EMA 的有效 N 減小 ~3-4x，放大 bias 到 ~10%。

**如果這個 hypothesis 正確**：
- 可以從 autocorrelation structure（Sigma_e recursion 的 off-diagonal）解析算出 bias
- 可以設計一個 bias correction factor 作為 C_dpmr_eff 的倒數乘數
- 預期 a_m 從 92% → ~100% 改善，可能帶動 a_hat 從 17% → ~12-14%

## Task 1b 具體步驟（**完全 offline，不跑 Simulink**）

### 可複用的資料（不要重跑）

| File | 內容 |
|------|------|
| `test_results/verify/phase2_chisquared_mc.mat` | 30s 單次長模擬：`del_pmr` time series, IIR-smoothed `Var_est_IIR`, `a_m_IIR` 等 |
| `test_results/verify/sigma_mc_1d_results.mat` | 1D MC (ctrl4)：`Var_theory` (time-varying recursion), `Var_MC_10000` |
| `test_results/verify/cdpmr_eff_lookup.mat` | 2D lookup (lc × f_0)，Phase 1 + Task 1a |

### Step 1: 從 Phase 2A 的 del_pmr 直接算 bias

```matlab
% Load phase2_chisquared_mc data
data = load('test_results/verify/phase2_chisquared_mc.mat');
% Extract del_pmr time series (z-axis)
del_pmr_z = data.results.del_pmr(3, :);
ss = data.results.ss_start:length(del_pmr_z);

% Compute sample variance (no IIR, ground truth for stationary)
Var_sample = var(del_pmr_z(ss));

% Compute IIR variance with sweep over a_cov
a_cov_list = [0.005, 0.01, 0.02, 0.05, 0.1];
for a_cov = a_cov_list
    % Apply IIR: del_pmr2_avg, del_pmrd
    % Compute V_IIR in steady state
    % Measure mean(V_IIR[ss]) / Var_sample
end
```

**輸出**：a_cov vs observed_bias (empirical)
**對照**：a_cov vs theoretical_bias (公式 `(2-2a)/(2-a)`)
**看差距**：autocorrelation 放大因子是多少？

### Step 2: 從 autocorrelation 解析預測 bias

從 Phase 1 已算好的 `Sigma_aug` (augmented Lyapunov)，可以提取 del_pmr 的 autocorrelation function:

```
gamma_L = c' * A_aug^L * Sigma_aug * c    (for L = 0, 1, ..., 50)
```

然後用 autocorrelation 算 effective sample size：

```
N_eff_IIR = 1 / (a_cov * sum_{L=-∞}^∞ rho_L^2)
  where rho_L = gamma_L / gamma_0
```

預測 bias ≈ `(2 * N_eff_IIR - 2*N_eff_IIR*a_prd) / (2*N_eff_IIR - a_prd)` 的等效公式。

### Step 3: 比對理論 vs 實測

- 如果理論 bias ≈ 實測 bias (± 1%) → **hypothesis 確認**，設計 correction
- 如果理論 bias 和實測偏差大 → 另找原因

### Step 4 (conditional): 設計 bias correction

如果 Step 3 確認：

```
C_dpmr_eff_corrected = C_dpmr_eff_raw * bias_factor(a_cov, autocorr)
```

`bias_factor` 可以：
- 作為第三個 lookup 軸（lc, f_0, a_cov）
- 或 runtime 從 Sigma_aug 的 autocorr 計算

### 決策點

| Task 1b 結果 | 下一步 |
|-------------|--------|
| IIR bias 解釋 > 70% of 8% | Task 1c: 實作 correction + 重跑 Phase 2 benchmark |
| IIR bias 解釋 30%-70% | Task 1c (部分改善) + 並行尋找其他 bias source |
| IIR bias 解釋 < 30% | Hypothesis 駁回，回去檢查 `L_ss` 是否與 Simulink runtime 一致 |

## 可複用的 code 資產（不要重寫）

| File | 用途 |
|------|------|
| `test_script/compute_7state_cdpmr_eff.m` | 核心 augmented Lyapunov，可抽 Sigma_aug |
| `test_script/verify_sigma_mc_1d.m` | 1D MC 架構，可參考 offline 分析 pattern |
| `test_script/verify_cdpmr_eff_simulink.m` | Simulink verification harness (之後要用) |
| `test_script/phase2_chisquared_mc.m` | Phase 2A 生成腳本，可借 IIR code block |

## 絕對不要做

- ❌ 跑 Simulink（context 和時間昂貴，offline 就能解決 Task 1b）
- ❌ 改 `motion_control_law_7state.m`（等 Task 1b 結果決定 Task 1c 再動）
- ❌ 重建 `cdpmr_eff_lookup.mat`（已經是 2D 版，內容正確）
- ❌ 嘗試 Multi-axis joint estimation 方向（user 已拒絕）
- ❌ 嘗試 adaptive R(2,2)（Q/R 共同 tuned，已失敗）

## 冷啟動時的第一個 prompt (使用者貼這個)

```
讀 reference/for_test/task1b_handoff.md 完整內容。按照裡面的 Task 1b 步驟執行。
不用 Simulink，只做 offline 分析。目標：驗證 IIR finite-sample bias hypothesis
是否能解釋 Phase 2A 觀察到的 ~8% a_m 負偏。
```

## 完成 Task 1b 後的交付物

1. **.mat**：`test_results/verify/task1b_iir_bias_analysis.mat`
2. **.md report**：`reference/for_test/task1b_report.md`，包含：
   - 理論預測 vs 實測的數字表
   - autocorrelation 放大因子定量
   - hypothesis 驗證結論
   - 下一步建議（Task 1c 或新方向）
3. **Figure**：`fig_task1b_iir_bias.png`（a_cov sweep 理論 vs 實測）
4. **Memory 更新**：`project_phase1_cdpmr_eff.md` 加 Task 1b 結論
5. **Commit**

---

## Context 預算估計

新 session 讀完這份 handoff（~250 行）+ Phase 2 report 關鍵段落（~50 行）≈ **~5% context 預算**。剩下 95% 全部用於 Task 1b 工作。這比在舊 session 繼續要乾淨很多。
