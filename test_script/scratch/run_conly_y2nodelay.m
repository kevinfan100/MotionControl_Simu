% STATUS: SCRATCH -- 2026-10-07 (user: "test ignoring the delay entirely"). Flag y2_delay_off: H23 = H25 = 0 AND y2_pred = s * a_hat
%   (no walk-back a' Grad_d x_d). Single sqrt(P_fix) 0.15 ('ratio'), lambda_c 0.7 (canonical), plane + cell, plane init, 10 seeds.
%   Trajectories: ramp (hold 0.5 s, descent 22.2 -> 1.10 R in T_d 2 s, hold 0.5 s) | ramp_tri T_d 1, 1.1-2 R, 1 Hz | wide T_d 0.5, 1.1-10 R, 2.5 Hz.
%   Baselines (_sig010_rho15) exist for the last two; ramp baseline is run here. Then the production-defaults acceptance.
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root, 'test_script', 'scratch'));  addpath(fullfile(root, 'test_script', 'integration'));
base = struct('law', 'f1p2', 'seeds', 1:10, 'q55_adapt', 'ratio', 'q55_adapt_P0', 0.15, 'sig', 0.1, 'plants', {{'cell', 'plane'}});
for arm = 1:2
    o0 = base;
    if arm == 1; o0.tag_suffix = '_sig010_rho15'; else; o0.tag_suffix = '_sig010_rho15_y2nodelay'; o0.y2_delay_off = true; end
    o = o0;  o.traj = 'ramp';  o.Td = 2.0;  run_plaw_walls(o);
    if arm == 2
        o = o0;  o.traj = 'ramp_tri';  o.f_osc = 1;  run_plaw_walls(o);
        o = o0;  o.traj = 'ramp_tri';  o.Td = 0.5;  o.f_osc = 2.5;  o.h_top_tri = 10;  run_plaw_walls(o);
    end
end
disp('Y2NODELAY DONE');
verify_formC_b_production_defaults;
disp('VERIFY DONE');
