# Packaging Decisions Ledger

每個部分完成時記一筆：做了什麼、為什麼、依據哪份 spec/文件。
本帳本是使用者掌握度文件 + 回答 reviewer 提問的依據。
框架層的 12 項定案見 [PACKAGING_PLAN.md](PACKAGING_PLAN.md) §4。

| 日期 | 段/部分 | 決定 | 理由 | 依據 |
|---|---|---|---|---|
| 2026-06-10 | 第 0 段 | 框架定案：目的三層、h_bar>1.2 L2、11 檔結構、controller 單檔 23-state 風格、D6 form、砍 G123、雙 gate 協定、分支策略（D6 在 feat / 打包在 pack） | 理解先於實作；一次只動一個變數；母 repo 與 package 不分岔 | PACKAGING_PLAN.md 全文; kf_canonical_spec.md §1b; LF drift 審計 (test_results/audit_lf_drift_2026-06-10.md) |
| 2026-06-11 | 第 1 段 (D6) | D6 遷移完成並 commit（feat 3cd07c7 + d87909e, merge 2e34308）。h50 5-seed OVERALL PASS（tracking 31.5 nm；bias 2.60/0.63/2.63%；rel-std 2.05–2.54%）。pre/post golden 比對：tracking delta +0.15 nm；**a_hat bias 系統性 +1.8 pt**，歸因實驗（暫時改 Phi*L 單 seed 重跑）分解：gain 位置 L vs Phi*L ~0.4 pt（paper 慣例，spec 預期接受）+ **舊 split form 的 F_e/Q 一步早用 covariance timing 被修正 ~1.4 pt（paper-strict）**。pre-D6 較低 bias 部分來自舊 timing 不一致的偶然有利 | 忠實於 Vpersonal paper-literal form 優先於湊較好看的 bias；delta 全量化、機制明確、gates 全過 | golden_pre_d6 / golden_post_d6 .mat（test_results/pack_baseline；post_d6 標 HEAD d6f95cc 但內容 = 3cd07c7 committed code）；compare_baselines 輸出；spec §1b item 5 |
| 2026-06-11 | 第 1 段簽收 | 使用者驗收通過（檢核題 1 經走讀展開後確認：Φ 推均值 / F_e 推共變異的分工與 D6 變更範圍釐清） | 走讀協定 | 對話紀錄 session "pack" |
| 2026-06-11 | 第 1 段附帶 | ramp 50→2.7 µm（h_bar=1.2）三 seeds 跑通無數值問題（pre/post 皆然），tracking ~30 nm 全程 | h_bar>1.2 envelope 第一個實證；近壁 dwell 數值穩定（帶 guard 的母 repo 行為） | golden baseline ramp2p7 runs |
