# Archived legacy controllers

These controller variants were once dispatched from `motion_control_law.m` but are no longer in production. They are kept here as historical reference; none should be needed for re-running unless reproducing an old experiment.

| File | Original case | Description | Why archived |
|---|---|---|---|
| `motion_control_law_1.m` | case 1 | Old baseline single-axis controller | Superseded by 7-state per-axis EKF; no test/driver references |
| `motion_control_law_2.m` | case 2 | Observer-pole variant using configurable `lambda_e` (non-deadbeat observer) | Experimental alternate observer form; not used in production runs |
| `motion_control_law_4.m` | case 4 | KF observer with DARE-precomputed kf_L (Fe=[0 1 0;0 0 1;0 0 1], H_kf=[1 0 0], Q_kf=sigma2_dXT*diag([0 0 1])) | Iterative-update observer experiment; folded back into 7-state results |
| `motion_control_law_5state.m` | case 5 | 5-state KF dropping `d, del_d` (states `[del_p1, del_p2, del_p3, a, del_a]`) | Tests showed positive feedback inside KF loop when paired with Sigma_e ratio filter (memory: `project_sigma_ratio_filter.md`); kept for ablation only |

## Re-running

If reproducing an old result requires one of these, restore by copying back into `model/controller/` and re-adding the case to the dispatcher. The original commit history is reachable via the `archive/sigma-pre-cleanup` tag if more context is needed.

## Why not delete

These embody real design experiments (alternate observer forms, alternate state dimensions). Deleting would lose searchable context for "we tried X and it failed because Y". The volume (4 files, ~few hundred lines total) is negligible.
