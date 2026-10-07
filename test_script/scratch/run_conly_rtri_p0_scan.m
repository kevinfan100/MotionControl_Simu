% STATUS: SCRATCH -- runner for hypotheses_formC_b.md C88 (P_0 scan) (Layer-1 session, 2026-10-04/05; moved from the job tmp dir).
%   Writes test_results/temp_plaw_walls/*.mat; the analysis numbers in the ledger row are computed from those files.
% P_0 (start value of P^f_55) scan on ramp_tri, c-only f1p2, P_fix fixed: sqrt(P_fix) = 0.2 (q55_adapt ratio).
root = '/Users/kevin/Code/MotionControl_Simu-motion-test';
addpath(fullfile(root, 'test_script', 'scratch'));
base = struct('law', 'f1p2', 'traj', 'ramp_tri', 'f_osc', 1, 'seeds', 1:10, 'q55_adapt', 'ratio', 'q55_adapt_P0', 0.2);
runs = {0.1, {'sphere'}; 0.2, {'plane', 'cell', 'sphere'}; 0.6, {'plane', 'cell', 'sphere'}};
for i = 1:size(runs, 1)
    o = base;  o.sig = runs{i, 1};  o.plants = runs{i, 2};
    o.tag_suffix = sprintf('_sig%03d_rho20', round(100 * o.sig));
    run_plaw_walls(o);
end
disp('P0 SCAN DONE');
