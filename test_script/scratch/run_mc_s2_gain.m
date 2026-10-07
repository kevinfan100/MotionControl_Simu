% STATUS: SCRATCH -- 2026-10-07, motion_ctrl Section 2 (effect of the motion gain on tracking): the control law uses
%   (a) the nominal motion gain a_nom (no estimation) or (b) the true motion gain (known wall model). The c-only estimator
%   still runs but does not enter the control law. ramp (hold 0.5 s, descent 22.2 -> 1.10 R in 2 s, hold 0.5 s),
%   plane wall, lambda_c 0.4, seed 1 (figure style 10-05). Writes test_results/temp_plaw_walls/f1p2_plane_PI_s1_ctrl{nom,true}_ramp2_lc040.mat.
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root, 'test_script', 'scratch'));  addpath(genpath(fullfile(root, 'model')));
cfg = canonical_scenario(0.05, 1.1, 'deep');
P = calc_simulation_params(cfg);  a_nom = P.Value.common.Ts / P.Value.common.gamma_N;   % [um/pN] nominal motion gain
fprintf('a_nom = %.5g um/pN\n', a_nom);
base = struct('law', 'f1p2', 'traj', 'ramp', 'Td', 2.0, 'seeds', 1, 'q55_adapt', 'ratio', 'q55_adapt_P0', 0.15, 'sig', 0.1, ...
              'plants', {{'plane'}}, 'lambda_c', 0.4);
o = base;  o.a_ctrl_override = a_nom * [1; 1; 1];  o.tag_suffix = '_s1_ctrlnom';  run_plaw_walls(o);
o = base;  o.a_ctrl_override = 'true';              o.tag_suffix = '_s1_ctrltrue'; run_plaw_walls(o);
base.traj = 'ramp_tri';  base.f_osc = 1;  base = rmfield(base, 'Td');   % 10-07: same mechanism as the paper's Fig. 5 on our z axis (fast motion near the wall)
o = base;  o.a_ctrl_override = a_nom * [1; 1; 1];  o.tag_suffix = '_s1_ctrlnom';  run_plaw_walls(o);
o = base;  o.a_ctrl_override = 'true';              o.tag_suffix = '_s1_ctrltrue'; run_plaw_walls(o);
disp('S2 DONE');
