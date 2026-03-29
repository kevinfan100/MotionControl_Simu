# Verification Notes（跨電腦共享 Context）

## IIR Convention 差異

目前專案和外部專案 (motion_3D_1_2_w12_5) 的 IIR filter convention 不同：

| | LP 公式 | a_pd=0.05 的意義 |
|---|---|---|
| 目前專案 | del_pmd = (1-a_pd)*old + a_pd*new | 5% new, 95% old (slow) |
| 外部專案 | dzm_bar = Avar*old + (1-Avar)*new | 5% old, 95% new (fast) |

C_dpmr 公式基於**目前專案 convention** 推導。外部專案僅供 controller 邏輯參考。

## C_dpmr 修正發現

原始公式缺少 HP filter 的 (1-a_var)^2 prefactor。
修正為 universal，與信號頻譜和控制器架構無關。

## Fe vs F 設計差異

Kalman filter 必須用 Fe（closed-loop error dynamics, (3,3)=1）設計。
用 F（open-loop, (3,3)=lc）會得到次優 gain。
原因：controller 把 error dynamics 的 (3,3) 從 lc 補成 1。

## 相關文件索引

- agent_docs/kf-observer-analysis.md — KF observer 完整推導
- agent_docs/literature-review.md — 文獻調查
- reference/for_test/verification_report.md — 驗證報告和數據
