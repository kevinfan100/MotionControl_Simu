% STATUS: SCRATCH -- runner for hypotheses_formC_b.md C88 (P_fix 0.10) (Layer-1 session, 2026-10-04/05; moved from the job tmp dir).
%   Writes test_results/temp_plaw_walls/*.mat; the analysis numbers in the ledger row are computed from those files.
% P_fix (equilibrium of P^f_55) scan on ramp_tri, c-only f1p2, sqrt(P_0) = 0.1; sqrt(P_fix) = 0.2 already on disk.
root = '/Users/kevin/Code/MotionControl_Simu-motion-test';
addpath(fullfile(root, 'test_script', 'scratch'));
for pf = 0.10
    o = struct('law', 'f1p2', 'traj', 'ramp_tri', 'f_osc', 1, 'seeds', 1:10, 'q55_adapt', 'ratio', 'q55_adapt_P0', pf, ...
               'sig', 0.1, 'plants', {{'plane', 'cell'}});
    o.tag_suffix = sprintf('_sig010_rho%02d', round(100 * pf));
    run_plaw_walls(o);
end
disp('PFIX 010 DONE');
