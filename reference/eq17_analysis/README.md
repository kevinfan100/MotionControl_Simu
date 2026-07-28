# reference/eq17_analysis/

eq17 研究線（paper 2023 Eq.17 d-step 延遲補償控制律）的推導、驗證與調查記錄。
現役主線 = **估測函數**（gain-law 函數常數估測，powerlaw／expgain 對等並行）——
章程見 repo 根 `CLAUDE.md`「研究主線」節。

## 從哪裡開始

| 要做什麼 | 讀哪裡 |
|---|---|
| 了解主線現況與入口 | `agent_docs/eq17/mainline-gainlaw.md`（每 session 自動載入） |
| 兩形式對照、定案門檻 | **`shape_ledger.md`** |
| 6-state KF flow 結構 SSOT | `kf_canonical_spec.md`（Vpersonal-aligned，living document） |
| powerlaw 線最新交接 | `powerlaw_handoff_2026-07-27.md` |
| 推導文件 | `derivation/README.md`（現役 15 支索引 + archive 死因表） |
| 舊路徑對照（2026-07-28 大整理） | `archive/MOVED.md` |

## Layout

```
eq17_analysis/
├── README.md                     (this file)
├── shape_ledger.md               兩形式統一指標對照 + freeze gate
├── kf_canonical_spec.md          6-state KF flow SSOT
├── powerlaw_handoff_2026-07-27.md  powerlaw 線交接（R2 白化/fdet/init 三修正）
├── gain_compare_findings.md      gain_compare 資料集索引（test_results/gain_compare/）
│
├── derivation/                   現役推導 15 支 tex（見其 README）
│   └── archive/                  已否證/被取代 17 支 + phase1-7 md + drafts/
│
├── archive/
│   ├── MOVED.md                  2026-07-28 整理的舊→新路徑對照表
│   ├── scratch/                  結案診斷腳本（7 組日期主題，各含 README）
│   └── sessions/                 歷史 session 報告與已執行完的 plan
│
├── verification/                 2026-04/05 phase8/9 驗證（凍結）
├── investigations/               2026-04/05 專題調查（凍結）
├── figures/                      phase8/9 時代場景圖（凍結）
└── L0_basis_oracle/              L0 python oracle（時代不明，UNCERTAIN）
```

對照組 eq6 線：`../eq6_analysis/`（自有 README 與 archive 結構）。
跨 controller 共通推導：`../shared/writeup_architecture.tex`。
