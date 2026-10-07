% STATUS: SCRATCH -- 2026-10-06 (user: "narrow band but speed close to the descent"). ramp_tri T_d 0.5 s (descent 42.2 R/s),
%   triangle 1.10 <-> 2.00 R at 23.5 Hz (2 x 0.9 x 23.5 = 42.3 R/s; n_cycles = 47), single sqrt(P_fix) 0.15 ('ratio'),
%   plane + cell, plane init, sqrt(P_0) 0.1, 10 seeds. Pairs with run_conly_rtri_wide_fast.m (narrow 5 Hz, wide 2.5 Hz).
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root, 'test_script', 'scratch'));
o = struct('law', 'f1p2', 'traj', 'ramp_tri', 'Td', 0.5, 'f_osc', 23.5, 'seeds', 1:10, 'q55_adapt', 'ratio', 'q55_adapt_P0', 0.15, ...
           'sig', 0.1, 'plants', {{'cell', 'plane'}}, 'tag_suffix', '_sig010_rho15');
run_plaw_walls(o);
disp('NARROW FAST DONE');
