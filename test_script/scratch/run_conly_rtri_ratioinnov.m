% STATUS: SCRATCH -- runner for hypotheses_formC_b.md R76 (innovation variants, first set) (Layer-1 session, 2026-10-04/05; moved from the job tmp dir).
%   Writes test_results/temp_plaw_walls/*.mat; the analysis numbers in the ledger row are computed from those files.
% ratioinnov on ramp_tri: Q55 = P_0^2 I (mean nu1/sigma)^2, sqrt(P_0) = sqrt(P^f_55[0]) = 0.1, base level sqrt(P_low) in {0.10, 0.15}.
root = '/Users/kevin/Code/MotionControl_Simu-motion-test';
addpath(fullfile(root, 'test_script', 'scratch'));
for pf = [0.10 0.15]
    o = struct('law', 'f1p2', 'traj', 'ramp_tri', 'f_osc', 1, 'seeds', 1:10, 'q55_adapt', 'ratioinnov', 'q55_adapt_P0', pf, ...
               'q55_adapt_N', 100, 'sig', 0.1, 'plants', {{'cell', 'plane'}});
    o.tag_suffix = sprintf('_sig010_ri%02d', round(100 * pf));
    run_plaw_walls(o);
end
disp('RATIOINNOV DONE');
addpath(fullfile(root, 'test_script', 'integration'));
verify_formC_b_production_defaults;
disp('VERIFY DONE');
