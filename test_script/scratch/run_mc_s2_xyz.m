% STATUS: SCRATCH -- 2026-10-07, motion_ctrl Section 2, three-axis version of the paper's Fig. 5 (Scenario A shape, 5 s):
%   z ramps from 50 um to 2.475 um at constant velocity (h/R 22.2 -> 1.10, no hold; user 10-07: the z height used in later sections), x and y are 1 Hz sines of amplitude 9 um
%   (tangential sines ported from test/meng-ch4 on 10-07). Control law with the true motion gain (x/y parallel, z perpendicular
%   correction) vs the nominal motion gain; the estimator runs but does not enter the control law. Plane wall, lambda_c 0.4, seed 1.
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root, 'test_script', 'scratch'));  addpath(genpath(fullfile(root, 'model')));
cfg = canonical_scenario(0.05, 1.1, 'deep');
P = calc_simulation_params(cfg);  a_nom = P.Value.common.Ts / P.Value.common.gamma_N;
ov = struct('trajectory_type', 'ramp_descent', 'h_init', 50, 'h_bottom', 1.1 * 2.25, 't_hold', 0, 't_descend_override', 5, 'T_sim', 5, ...
            'amp_u', -9, 'freq_u', 1, 'amp_v', -9, 'freq_v', 1, 'tan_shape', 1);
base = struct('law', 'f1p2', 'traj', 'ramp', 'Td', 5, 'seeds', 1, 'q55_adapt', 'ratio', 'q55_adapt_P0', 0.15, 'sig', 0.1, ...
              'plants', {{'plane'}}, 'lambda_c', 0.4, 'ov_extra', ov);
o = base;  o.a_ctrl_override = 'true';              o.tag_suffix = '_s1_ctrltrue_xyzA';  run_plaw_walls(o);
o = base;  o.a_ctrl_override = a_nom * [1; 1; 1];  o.tag_suffix = '_s1_ctrlnom_xyzA';   run_plaw_walls(o);
disp('XYZ DONE');
