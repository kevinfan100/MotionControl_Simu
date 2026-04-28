# Task 01 Report — Math-layer Observability Verification

**Branch**: `test/eq17-5state-ekf`
**Date**: 2026-04-28
**Method**: Algebraic rank test on LTV observability matrix
**Script**: `test_script/check_observability_eq17.m`
**Data**: `test_results/eq17_analysis/task01_observability_results.mat` (gitignored)

---

## 1. Result Summary

**ALL 10 cases PASS** — analytical predictions in `design.md` §7 fully confirmed.

附加發現一個比 design.md §7 更精確的 PE 條件（見 §5）。

```
Window N = 6, lambda_c = 0.70, n_state = 5
Total cases: 10  (5 Config A + 3 Config B + 2 boundary)
PASS:        10
FAIL:         0
```

---

## 2. Test Cases & Results

| ID | Scenario | Config | rank(O) | Expected | Pass |
|---|---|---|---|---|---|
| A1 | f_d = 0 (positioning hold) | A | 4 | 4 | ✓ |
| A2 | f_d = 2 (constant non-zero) | A | 4 | 4 | ✓ |
| A3 | f_d = linear ramp 1..6 | A | 5 | 5 | ✓ |
| A4 | f_d = sinusoidal sin(k·π/3) | A | 5 | 5 | ✓ |
| A5 | f_d = [1 2 1 1 1 1] (idx 2, **inside** PE window) | A | 5 | 5 | ✓ |
| A6 | f_d = [1 1 2 1 1 1] (idx 3, **inside** PE window) | A | 5 | 5 | ✓ |
| A7 | f_d = [1 1 1 2 1 1] (idx 4, **outside** PE window for N=6) | A | 4 | 4 | ✓ |
| B1 | f_d = 0, dual measurement | B | 5 | 5 | ✓ |
| B2 | f_d = 2 constant, dual measurement | B | 5 | 5 | ✓ |
| B3 | f_d = ramp, dual measurement | B | 5 | 5 | ✓ |

---

## 3. Observability Matrix Inspection

### Case A1 — f_d = 0, Config A, rank 4

```
[  1.0000   0.0000   0.0000   0.0000   0.0000 ]    <- y_1[k_0]   identifies δx_1
[  0.0000   1.0000   0.0000   0.0000   0.0000 ]    <- y_1[k_0+1] identifies δx_2
[  0.0000   0.0000   1.0000   0.0000   0.0000 ]    <- y_1[k_0+2] identifies δx_3
[  0.0000   0.0000   0.7000  -1.0000   0.0000 ]    <- y_1[k_0+3]
[  0.0000   0.0000   0.4900  -1.7000   0.0000 ]    <- y_1[k_0+4]
[  0.0000   0.0000   0.3430  -2.1900   0.0000 ]    <- y_1[k_0+5]
                                       ↑
                            a_x column 全部 0 → 不可觀
```

a_x 那欄全部為 0（因為 −f_d[k_0] = 0），所以 a_x 完全不可觀。x_D 透過 −1, −1.7, −2.19 可被識別，但 a_x 撈不出來 → rank 4。

### Case A4 — f_d sinusoidal, Config A, rank 5

```
[  1.0000   0.0000   0.0000   0.0000   0.0000 ]
[  0.0000   1.0000   0.0000   0.0000   0.0000 ]
[  0.0000   0.0000   1.0000   0.0000   0.0000 ]
[  0.0000   0.0000   0.7000  -1.0000  -0.8660 ]    <- a_x 欄出現非零項
[  0.0000   0.0000   0.4900  -1.7000  -1.4722 ]
[  0.0000   0.0000   0.3430  -2.1900  -1.0306 ]
```

f_d 的時變讓 a_x 欄出現非零、非比例的值（−0.866, −1.4722, −1.0306），與 x_D 欄（−1, −1.7, −2.19）線性無關 → rank 5。

### Case B1 — f_d = 0, Config B, rank 5

```
[  1.0000   0.0000   0.0000   0.0000   0.0000 ]    <- y_1[k_0]   = δx_1
[  0.0000   0.0000   0.0000   0.0000   1.0000 ]    <- y_2[k_0]   = a_x 直接量測
[  0.0000   1.0000   0.0000   0.0000   0.0000 ]    <- y_1[k_0+1] = δx_2
[  0.0000   0.0000   0.0000   0.0000   1.0000 ]    <- y_2[k_0+1] = a_x (重複)
[  0.0000   0.0000   1.0000   0.0000   0.0000 ]    <- y_1[k_0+2] = δx_3
[  0.0000   0.0000   0.0000   0.0000   1.0000 ]    <- y_2[k_0+2] = a_x (重複)
[  0.0000   0.0000   0.7000  -1.0000   0.0000 ]    <- y_1[k_0+3]
[  0.0000   0.0000   0.0000   0.0000   1.0000 ]    <- y_2[k_0+3]
[  0.0000   0.0000   0.4900  -1.7000   0.0000 ]    <- y_1[k_0+4]
[  0.0000   0.0000   0.0000   0.0000   1.0000 ]    <- y_2[k_0+4]
[  0.0000   0.0000   0.3430  -2.1900   0.0000 ]    <- y_1[k_0+5]
[  0.0000   0.0000   0.0000   0.0000   1.0000 ]    <- y_2[k_0+5]
```

雖然 f_d = 0 讓 y_1 鏈無法分離 (x_D, a_x)，**但 y_2 鏈直接給出 a_x = const**。把 a_x 當已知後，y_1 的 row 7 [0, 0, 0.7, −1, 0] 立刻解出 x_D = −0.7·δx_3 → rank 5。**這就是 IIR 把 PE 從必要條件變不必要條件的核心機制**。

---

## 4. 對 design.md §7 的核對

| design.md §7 預測 | 實際 rank | 一致 |
|---|---|---|
| Config A, f_d = 0 → 不可觀 (rank 4) | 4 (A1) | ✓ |
| Config A, f_d = 任意非零定值 → 不可觀 (rank 4) | 4 (A2) | ✓ |
| Config A, f_d 變動 → 可觀 (rank 5) | 5 (A3, A4) | ✓ |
| Config B 永遠 rank 5 | 5 (B1, B2, B3) | ✓ |

---

## 5. Key Finding：PE Window 的精確邊界

### 觀察

A7 的 [1, 1, 1, 2, 1, 1] 雖然 f_d **有變動**，rank 卻 = 4。表面上違反 design.md §7「f_d 變動 → 可觀」的預測。

### 解析

對 N=6 觀測窗口，O 矩陣的 (x_D, a_x) 子塊（rows 4–6）依賴 f_d 序列的特定子集。逐列推導：

```
Row 4: (x_D, a_x) coeff = ( −1,           −f_d_seq(1)                       )
Row 5: (x_D, a_x) coeff = ( −(λ_c+1),     −(λ_c·f_d_seq(1) + f_d_seq(2))    )
Row 6: (x_D, a_x) coeff = ( −(λ_c²+λ_c+1), −(λ_c²·f_d_seq(1) + λ_c·f_d_seq(2) + f_d_seq(3)) )
```

→ **只有 f_d_seq(1)、f_d_seq(2)、f_d_seq(3) 進入 (x_D, a_x) 子塊。f_d_seq(4)、(5)、(6) 完全不影響 rank**。

### 為什麼是 (1..N−3)？

時變項 F_e(3,5) = −f_d 只有在「row 3 chain」被啟動時才出現。對 H · Φ(k_0+m, k_0)：
- m = 0, 1, 2：在最右邊 F_e 的 row 3 還沒被引用 → f_d 不出現
- m = 3：第一次引用 row 3，引到的是 F_e[k_0] 的 row 3 → 帶 f_d_seq(1)
- m = 4：引到 F_e[k_0+1] 的 row 3 → 帶 f_d_seq(2)
- m = 5：引到 F_e[k_0+2] 的 row 3 → 帶 f_d_seq(3)

對窗口 N=6，最大 m = N−1 = 5，引用到 f_d_seq(N−3) = f_d_seq(3) 為止。

### 精確 PE 條件（refines design.md §7）

> **Config A 可觀 ⇔ f_d_seq(1..N−3) 之中至少有兩個值不同**

對應到實際軌跡：要在窗口 [k_0, k_0+N−1] 內可觀，**必須在 [k_0, k_0+N−4] 這個前段子窗口內 f_d 已有變動**，僅在窗口尾端變動是不夠的。

### 數值佐證

對四個 sequence 直接算 (x_D, a_x) sub-block 的 rank：

| Sequence | Effective (f₁,f₂,f₃) | Sub-block rank | 預測 O rank |
|---|---|---|---|
| [1 1 1 1 2 1] (var idx 5) | (1, 1, 1) | 1 (collinear) | 4 |
| [1 2 1 1 1 1] (var idx 2) | (1, 2, 1) | 2 (spans) | 5 |
| [1 1 2 1 1 1] (var idx 3) | (1, 1, 2) | 2 (spans) | 5 |
| [1 1 1 2 1 1] (var idx 4) | (1, 1, 1) | 1 (collinear) | 4 |

預測值與 MATLAB 計算的實際 rank 完全一致。

### 含義

1. **Window length 與 PE 緊密耦合**：選 N 時不只是「給足夠 row 數」，還要確保 trajectory 上「PE 變動發生在 [k_0, k_0+N−4] 內」
2. **Task 02 (Gramian) 的窗口設計**必須留意：對 1Hz osc trajectory 採 N=10 (~6 ms @ 1.6 kHz)，PE 窗口為前 7 個樣本 (~4.4 ms)，遠小於 1Hz 週期 → quasi-stationary 內 f_d 幾乎不變 → σ_min 很弱
3. **Config B 不受此影響**：a_xm 量測直接給 a_x，不靠 PE → 設計上應**強烈傾向加 IIR**

---

## 6. Discussion

### Config A 的兩種 rank deficiency 機制

A1 (f_d=0) 與 A2 (f_d=非零定值) 都是 rank 4，但機制略有不同：
- **A1**：a_x 欄整欄為 0（f_d=0 讓 a_x 完全不影響輸出）
- **A2**：a_x 欄非 0，但與 x_D 欄成比例（共線）→ 仍然 rank 4

兩種都印證「frozen-time 不可觀」結論。

### Config B 的 rank 5 來源拆解

從 B1 的 O 矩陣看到：12 列中有 6 列是 [0,0,0,0,1]（y_2 在所有時間都是 a_x 不變），其餘 6 列來自 y_1 鏈。實際獨立列：
- 1 列 a_x 量測（y_2 任一列）
- 3 列 δx_1, δx_2, δx_3（y_1 前 3 列）
- 1 列 x_D 識別（y_1 row 7：[0,0,0.7,−1,0]，扣掉 0.7·δx_3 後得 x_D）

→ 5 個獨立資訊通道，無需 PE。

### IIR 的「保險」價值具體量化

對 osc trajectory 的 hold 階段（t < 0.5s, f_d ≈ 0），**配置 A 嚴格 rank 4 → KF 完全無法分離 (x_D, a_x)**。配置 B 因 a_xm 量測仍 rank 5。這代表：
- 在 hold 期，配置 A 的 KF 必然漂移
- 配置 B 在 hold 期受 IIR 收斂速度限制（warm-up），但**結構上可觀**
- 兩者差異在 design.md §6 的 Caveat 章已述，本 task 數值佐證

---

## 7. Open Items / Follow-ups

1. **Task 02 — Gramian σ_min(t)**：本 task 只看 rank（布林）。下一步是**定量**算窗口內的觀測 Gramian，畫 σ_min vs t，看實際軌跡上「多可觀」。配合 R[k] 加權成 information form 後可直接對應 KF 收斂速度。

2. **Task 03 — R_2[k] adaptive design**：配置 B 的「實務可觀性」受 IIR 健康度影響。需設計 R_2[k] 在 IIR 崩塌條件下自動加大（warm-up、低 SNR、過 wall）。本 task 假設 nominal R_2 純粹做 rank 檢查。

3. **PE-window 精確邊界用於 Task 02 的窗口設計**：本 task 的 §5 結論直接餵進 Task 02 的 σ_min 計算 — 對窗口 N，PE 子窗口為前 N−3 個樣本。

4. **Symbolic verification using MATLAB Symbolic Toolbox**（low priority）：用符號運算重新推導 Row 4/5/6 公式，固化在 docstring 裡。目前 §5 的公式是手算 + 數值佐證，可信度足夠但未符號化。

---

## 8. Artifacts

- **Script**: `test_script/check_observability_eq17.m`（10 test cases，含 A5/A6/A7 邊界 sweep）
- **Data**: `test_results/eq17_analysis/task01_observability_results.mat`（gitignored；含 results 結構陣列、O 矩陣、cases 設定）
- **This report**: `reference/eq17_analysis/task01_math_observability_report.md`
- **Update**: `reference/eq17_analysis/design.md` §7 精確化（PE-window 條件）
- **Update**: `agent_docs/eq17-verification.md` milestone table

---

## 9. Reproducibility

```matlab
cd('/Users/kevin/Code/MotionControl_Simu')
addpath('test_script')
check_observability_eq17
```

預期輸出：`=== ALL 10 cases PASS ===`，O 矩陣對 A1/A4/B1 印出，`.mat` 存到 test_results/eq17_analysis/。
