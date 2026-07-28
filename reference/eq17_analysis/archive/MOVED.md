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

## 結案證據下沉（→ reference/eq17_analysis/archive/scratch/<group>/）

```
test_script/integration/temp_{taylor_det_display,compare_5state,analyze_5state}.m → 2026-07-14-asstate-taylor/
test_script/temp_{motion_control_law_eq17_4state_centered,run_pure_sim_centered,var_centered_verify}.m → 2026-07-15-var-centered/
test_script/temp_{dither_pilot,dither_sweep,run_pure_sim_dither}.m → 2026-07-21-dither/
test_script/temp_diag_kfmeas_*.m (15 支) → 2026-07-22-kfmeas/
test_script/temp_{motion_control_law_eq17_aprime2nd,run_pure_sim_aprime2nd,aprime_order_compare,aprime_hd_order_compare,aprime_hd_cheat_vs_honest,aprime_p55_test}.m → 2026-07-23-curvature-aprime2nd/
test_script/temp_diag_powerlaw_{whiten,descent,fixes,init_q,loop,phase2,phase3}.m + temp_fig_powerlaw_phat_raw.m + temp_diag_init_5state.m → 2026-07-26-powerlaw-whitening/
test_script/temp_diag_{6pct_drift,6pct_positioning,a_anchor_2x2,fdet_test,fdet_honestinit,init_asym,init_vs_prior,powerlaw_2x2,powerlaw_honestinit,realistic_init,selfconsistent_loop,what_anchors_a,who_moves_ahat}.m + temp_fig_{fdet_convergence,fdet_z,honest_init_1seed,powerlaw_A12,powerlaw_final}.m → 2026-07-27-fdet-anchor-init/
```

## 刪除（快照 commit 後移除；`git show <snapshot>:<path>` 可救回）

```
repo root temp_{plot_ahat_relerr_pct,plot_ahat_zoom_z,plot_aram_overview,plot_aram_zoom3,plot_delpm_z_1hz,plot_true_vs_pm_z,test_4state_ar1_1hz,test_4state_dx3_1hz}.m → 刪 (快照在 026189f)
test_script/temp_{checkcode_dither,dither_check}.m → 刪 (快照在 026189f)
test_script/*.png/.mat 10 個輸出 → 刪 (可由已入版控的產生器重生, 未快照)
reference/eq17_analysis/derivation/figures/aprime_hd_oracle_compare.png → 刪 (產生器在 2026-07-23-curvature-aprime2nd/)
```
