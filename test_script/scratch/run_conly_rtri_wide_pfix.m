% STATUS: SCRATCH -- 2026-10-06 (user: "the wide + fast triangle gives c a chance to track; keep a single P_fix; can the oscillation track?").
%   ramp_tri T_d 0.5 s (42 R/s), triangle 1.10 -> 10 R at 2.5 Hz (44.5 R/s), single P_fix (q55_adapt 'ratio'), sqrt(P_fix) 0.25 and 0.35
%   (0.15 is already on disk from run_conly_rtri_wide_fast.m). Plane + cell, plane init, sqrt(P_0) 0.1, 10 seeds.
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root, 'test_script', 'scratch'));
for pf = [0.25 0.35]
    o = struct('law', 'f1p2', 'traj', 'ramp_tri', 'Td', 0.5, 'f_osc', 2.5, 'h_top_tri', 10, 'seeds', 1:10, ...
               'q55_adapt', 'ratio', 'q55_adapt_P0', pf, 'sig', 0.1, 'plants', {{'cell', 'plane'}}, ...
               'tag_suffix', sprintf('_sig010_rho%02d', round(100 * pf)));
    run_plaw_walls(o);
end
disp('WIDE PFIX DONE');
