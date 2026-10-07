% STATUS: SCRATCH -- runner for hypotheses_formC_b.md C88 (eps r2 on ramp_tri) (Layer-1 session, 2026-10-04/05; moved from the job tmp dir).
%   Writes test_results/temp_plaw_walls/*.mat; the analysis numbers in the ledger row are computed from those files.
% Starting point for the story: Q55 = rho * R2[k] (constant ratio to the y2 noise), ramp_tri, sqrt(P_0) = 0.1, plane + cell, 10 seeds.
root = '/Users/kevin/Code/MotionControl_Simu-motion-test';
addpath(fullfile(root, 'test_script', 'scratch'));
for rho = [0.03 0.3]
    o = struct('law', 'f1p2', 'traj', 'ramp_tri', 'f_osc', 1, 'seeds', 1:10, 'q55_adapt', 'r2ratio', 'q55_adapt_rho', rho, ...
               'sig', 0.1, 'plants', {{'cell', 'plane'}});
    o.tag_suffix = sprintf('_sig010_r2r%03d', round(100 * rho));
    run_plaw_walls(o);
end
disp('R2RATIO DONE');
