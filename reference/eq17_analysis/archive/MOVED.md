# MOVED — 2026-07-28 大整理路徑對照表

> 同 2026-06-02 port rename map 慣例：memory／舊文件引用的路徑用此表解讀。
> 一行一條：`舊路徑 → 新路徑`。git 歷史完整保留（untracked 檔為首次入版控）。

## temp 轉正（test_script/ → test_script/integration/，函數 temp_run_5state_powerlaw 更名 run_5state_powerlaw）

```
test_script/temp_run_5state_powerlaw.m      → test_script/integration/run_5state_powerlaw.m
test_script/temp_smoke_5state_powerlaw.m    → test_script/integration/smoke_5state_powerlaw.m
test_script/temp_plot_5state_powerlaw.m     → test_script/integration/plot_5state_powerlaw.m
test_script/temp_test_blocal_exp.m          → test_script/integration/verify_blocal_exp.m
test_script/temp_regress_powerlaw_A12.m     → test_script/integration/verify_powerlaw_regress_A12.m
test_script/temp_regress_powerlaw_final.m   → test_script/integration/verify_powerlaw_regress_final.m
test_script/temp_fig_p_prior_origin.m       → test_script/integration/plot_p_prior_origin.m
```

## 活草稿（test_script/ → test_script/scratch/，進版控、檔名不變）

```
test_script/temp_mcl_powerlaw_diag.m                    → test_script/scratch/（FORK 檔頭）
test_script/temp_run_powerlaw_diag.m                    → test_script/scratch/（FORK 檔頭）
test_script/temp_motion_control_law_eq17_4state_kfmeas.m → test_script/scratch/（FORK 檔頭）
test_script/temp_run_pure_sim_kfmeas.m                  → test_script/scratch/（FORK 檔頭）
test_script/temp_diag_powerlaw_cdpmr.m                  → test_script/scratch/（缺陷2 證據）
test_script/temp_diag_powerlaw_cdpmr2.m                 → test_script/scratch/（缺陷2 證據）
test_script/temp_verify_powerlaw_claims.m               → test_script/scratch/（缺陷2 證據）
```
