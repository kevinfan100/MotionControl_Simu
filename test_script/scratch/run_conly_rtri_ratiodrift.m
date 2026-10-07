% STATUS: SCRATCH -- runner for hypotheses_formC_b.md R77 (additive drift Q, winds up) (Layer-1 session, 2026-10-04/05; moved from the job tmp dir).
%   Writes test_results/temp_plaw_walls/*.mat; the analysis numbers in the ledger row are computed from those files.
% ratiodrift on ramp_tri: Q55 = P_fix^2 I + dc^2 |M| / h_hat^2, sqrt(P_fix) = 0.10, sqrt(P_0) = 0.1, dc in {0.2, 0.5, 0.8}, plane + cell, 10 seeds.
root = '/Users/kevin/Code/MotionControl_Simu-motion-test';
addpath(fullfile(root, 'test_script', 'scratch'));
% wiring check first: dc = 0 must reproduce ratio sqrt(P_fix) = 0.10 exactly (seeds 1:2)
o = struct('law', 'f1p2', 'traj', 'ramp_tri', 'f_osc', 1, 'seeds', 1:2, 'q55_adapt', 'ratiodrift', 'q55_adapt_P0', 0.10, 'q55_adapt_dc', 0, ...
           'sig', 0.1, 'plants', {{'cell'}}, 'tag_suffix', '_sig010_rd10chk');
run_plaw_walls(o);
A = load(fullfile(root, 'test_results', 'temp_plaw_walls', 'f1p2_cell_PI_sig010_rho10_rtri_f1.mat'));
B = load(fullfile(root, 'test_results', 'temp_plaw_walls', 'f1p2_cell_PI_sig010_rd10chk_rtri_f1.mat'));
fprintf('WIRING CHECK dc = 0 vs ratio 0.10: max|d a_hat| = %g\n', max(max(abs(A.a_hat(:, 1:2) - B.a_hat))));
for dc = [0.2 0.5 0.8]
    o = struct('law', 'f1p2', 'traj', 'ramp_tri', 'f_osc', 1, 'seeds', 1:10, 'q55_adapt', 'ratiodrift', 'q55_adapt_P0', 0.10, 'q55_adapt_dc', dc, ...
               'sig', 0.1, 'plants', {{'cell', 'plane'}});
    o.tag_suffix = sprintf('_sig010_rd10dc%02d', round(10 * dc));
    run_plaw_walls(o);
end
disp('RATIODRIFT DONE');
addpath(fullfile(root, 'test_script', 'integration'));
verify_formC_b_production_defaults;
disp('VERIFY DONE');
