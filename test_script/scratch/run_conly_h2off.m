% STATUS: SCRATCH -- 2026-10-06 (user: "set H23 and H25 to 0, what happens"). Flag h2_walk_cols_off: H2(3) = H2(5) = 0, y2_pred unchanged.
%   Single sqrt(P_fix) 0.15 ('ratio'), plane + cell, plane init, sqrt(P_0) 0.1, 10 seeds, two trajectories with baselines on disk:
%   ramp_tri T_d 1, 1.1-2 R at 1 Hz (baseline _sig010_rho15_rtri_f1)  |  T_d 0.5, 1.1-10 R at 2.5 Hz (baseline _sig010_rho15_rtri_f2p5_td0p5_top1000).
%   Then the production-defaults acceptance check (the flag is default off).
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root, 'test_script', 'scratch'));  addpath(fullfile(root, 'test_script', 'integration'));
base = struct('law', 'f1p2', 'traj', 'ramp_tri', 'seeds', 1:10, 'q55_adapt', 'ratio', 'q55_adapt_P0', 0.15, 'sig', 0.1, ...
              'plants', {{'cell', 'plane'}}, 'h2_walk_cols_off', true, 'tag_suffix', '_sig010_rho15_h2off');
o = base;  o.f_osc = 1;  run_plaw_walls(o);
o = base;  o.Td = 0.5;  o.f_osc = 2.5;  o.h_top_tri = 10;  run_plaw_walls(o);
disp('H2OFF DONE');
verify_formC_b_production_defaults;
disp('VERIFY DONE');
