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

## Stage 3 — 已否證線整線下沉（2026-07-28）

```
model/controller/motion_control_law_eq17_gscalar.m        → model/controller/archive/
model/controller/motion_control_law_eq17_5state_aprime.m  → model/controller/archive/
model/controller/build_F_e_5state_aprime.m                → model/controller/archive/
model/controller/motion_control_law_eq17_4state_para.m    → model/controller/archive/
model/controller/compute_if2_rigorous.m                   → model/controller/archive/
test_script/integration/{verify_eq17_5state_aprime_L0,L1,selfmod_L0,selfmod_L1,verify_5state_aprime_selfmod_signflip,check_observability_5state_aprime,verify_aprime_Qprime_assumptions,verify_aprime_blackbox_*x6,verify_eq17_gscalar_suite,verify_para_integrated,plot_para_aprime,verify_r22_amlpf_6state,run_amlpf_rerun_6state,plot_amlpf_compare_6state}.m → test_script/integration/archive/
test_script/unit_tests/verify_eq17_unit_gscalar_recursion.m → test_script/unit_tests/archive/
model/dual_track/run_pure_simulation.m 的 '5state_aprime'/'gscalar'/9 個 TEMP variant 分支 → 移除（未知 variant 現在直接 error）
```

## Stage 4 — 推導與文件下沉（2026-07-28）

```
derivation/{5state_aprime_var_esti,var_centered,var_chain,mirror,coupdate,axm_num,kf_meas,selfmod,unified,observability}.tex(+pdf) → derivation/archive/
derivation/{5state_est_aprime,aprime_blackbox_regression,5state_vs_6state_aprime,6state_curvature_hd,6state_curvature_taylor,4state_para_c,4state_del_hd_thetafit}.tex(+pdf) → derivation/archive/
derivation/{phase1..7_*.md x8, Q66_OL_R22_derivation.md, drafts/} → derivation/archive/
eq17_analysis/{design_v2,gain_compare_plan,gain_compare_plan_round2,gain_compare_design,4state_adet_aprime_nocheat_status,aprime_estimation_status,task01_math_observability_report,asstate_verify_report,c1c2_unknownc_audit_plan,eq17_6state_review_findings}.md → eq17_analysis/archive/sessions/
agent_docs/eq6_or_23state/*.md (5 支) → agent_docs/archive/
```
