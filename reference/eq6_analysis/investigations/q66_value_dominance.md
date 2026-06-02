# Q(6,6) 數值主導 a_hat std — V5/V6/V7 cross-branch study 發現

> 精煉版結論。完整 study 過程見 `transitions_report.md` §10-12, 視覺證據見 `fig_transitions/`。

## 核心 finding

**在 h=50 free-space positioning, a_hat std 的主要驅動是 Q(6,6) 的「數值」, 不是 controller 架構 (Paper 2025 Eq.6 vs Paper 2023 Eq.17 形式)。**

| 設定 | Q(6,6) 來源 | Q(6,6) 數值 (z 軸) | a_hat std (x / z) |
|---|---|---|---|
| V4 (sigma frozen_correct, Stage 4 final) | empirical tune | 1e-8 | 5-7% |
| V7 (sigma v1 controller + eq17 OL_mode Q/R port) | OL_mode physics `(â·K_h/R)²·σ²_δh,thermal` | 1.89e-10 | **1.10% / 1.41%** |

V7 把 eq17 那條分支推導出的物理 Q(6,6) 配方原樣 port 到 sigma v1 (Paper 2025 Eq.6) controller 上, a_hat std 不只「沒退步」, 還達到甚至超越 eq17 published baseline。差異 ~50×, controller F_e Row 3 形式 (sigma 原 form vs Eq.19 form) 在 h=50 free-space 不是主因。

## 三階段佐證

- **V5**: 直接把 eq17 OL_mode Q/R scaling 套到 sigma v1 controller, a_hat std 立刻往 eq17 水準靠攏 → 否證「a_hat std 是 controller 架構決定」假設
- **V6**: 只改 H(2,7) = -d (其他全照 V4), a_hat std 幾乎不動 → 證明不是 H 矩陣形式
- **V7**: 只改 Q(6,6) 從 1e-8 → 1.89e-10 (其他全照 V4), a_hat std 從 5-7% 降到 1-1.4% → 鎖定 Q(6,6) 數值是真主因

## Q(6,6) 物理推導 (V7 採用)

OL_mode runtime 公式:

```
Q(6,6) = (â · K_h / R)² · σ²_δh,thermal
       = (â · K_h / R)² · (4·k_B·T·Ts / (γ_N·c_perp))
```

在 h=50 free-space, K_h ≈ 0.0023 (極小, 因離壁面遠), 所以 Q(6,6) 的數量級降到 1e-10 量級。

Bus scaling (= Q(6,6)/σ²_dXT):
- Q(6,6)_x scaling ≈ 5.12e-11
- Q(6,6)_z scaling ≈ 1.89e-10

對比 V4 的 1e-8 empirical floor, 物理推導值小 50-200×。

## 物理意義

- **V4 1e-8** = engineering floor, 與 h 無關, 純讓 EKF 有「a 漂移自由度」
- **V7 1.89e-10** = OL_mode physics, 反映「壁面附近 a 因 h 變化導致 process noise 上限」。h_bar 越大 → K_h → 0 → Q(6,6) → 0, 自然符合 free-space EKF 應該「鎖住」 a_hat 的物理期望

## 對 new main 的設計指引

兩 controller 在 new main 共用同一套 on-the-fly Q/R 物理公式 (包含 Q(6,6) OL_mode mode), 不用 preset 機制。V7 已實證此設計對 sigma eq6 controller 工作良好。詳見 `reference/shared/writeup_architecture.tex` Q 矩陣章節 + `reference/eq17_analysis/control_law_eq17.tex` (Eq.17 specific F_e 推導)。

## 衍生開放議題

- V7 在 free-space (h=50) 驗證, 還沒測 wall-region (h_bar < 2): 那邊 K_h 暴增可能引發其他問題 (參考 `reference/eq17_analysis/paper_compare_report.md`, `Kh_cap_breakthrough.md`)
- Q(6,6) physics 推導本身的限制 (例如 c_perp 在 wall-region 變化敏感) 未充分 stress-tested

## 參考檔

- `transitions_report.md` — 完整研究筆記 (V1-V7 全程, 287 lines)
- `fig_transitions/fig_transitions_v1_v7_grid.png` — 視覺對比
- `fig_transitions/fig_v4_vs_v6_steadystate.png`, `fig_v4_vs_v5_steadystate.png` — 穩態對照
- `archive/sessions/V5_V7_study/motion_control_law_olmode.m` — 當時用的 isolation harness (已歸檔, 因 new main 雙 controller 結構讓此 harness 過時)
