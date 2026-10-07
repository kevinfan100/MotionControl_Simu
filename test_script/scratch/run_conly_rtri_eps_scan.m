% STATUS: SCRATCH -- runner for hypotheses_formC_b.md C88 (eps r2 scan) (Layer-1 session, 2026-10-04/05; moved from the job tmp dir).
%   Writes test_results/temp_plaw_walls/*.mat; the analysis numbers in the ledger row are computed from those files.
% Q55 = eps * R2 scan between eps = 1e-5 (~ Q55 = 0) and 0.03 (windup): eps in {1e-4, 1e-3, 1e-2}, ramp_tri Td = 1 s, sqrt(P_0) = 0.1, plane + cell, 10 seeds.
root = '/Users/kevin/Code/MotionControl_Simu-motion-test';
addpath(fullfile(root, 'test_script', 'scratch'));
for e = [1e-4 1e-3 1e-2 1e-5]
    o = struct('law', 'f1p2', 'traj', 'ramp_tri', 'f_osc', 1, 'seeds', 1:10, 'q55_adapt', 'r2ratio', 'q55_adapt_rho', e, ...
               'sig', 0.1, 'plants', {{'cell', 'plane'}});
    o.tag_suffix = sprintf('_sig010_eps%de', -round(log10(e)));
    run_plaw_walls(o);
end
disp('EPS SCAN DONE');
