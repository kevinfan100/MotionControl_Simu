# Packaging Decisions Ledger

每個部分完成時記一筆：做了什麼、為什麼、依據哪份 spec/文件。
本帳本是使用者掌握度文件 + 回答 reviewer 提問的依據。
框架層的 12 項定案見 [PACKAGING_PLAN.md](PACKAGING_PLAN.md) §4。

| 日期 | 段/部分 | 決定 | 理由 | 依據 |
|---|---|---|---|---|
| 2026-06-10 | 第 0 段 | 框架定案：目的三層、h_bar>1.2 L2、11 檔結構、controller 單檔 23-state 風格、D6 form、砍 G123、雙 gate 協定、分支策略（D6 在 feat / 打包在 pack） | 理解先於實作；一次只動一個變數；母 repo 與 package 不分岔 | PACKAGING_PLAN.md 全文; kf_canonical_spec.md §1b; LF drift 審計 (test_results/audit_lf_drift_2026-06-10.md) |
