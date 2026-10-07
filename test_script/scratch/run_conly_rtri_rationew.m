% STATUS: SCRATCH -- runner for hypotheses_formC_b.md C88 (P_fix by new height / revisit) (Layer-1 session, 2026-10-04/05; moved from the job tmp dir).
%   Writes test_results/temp_plaw_walls/*.mat; the analysis numbers in the ledger row are computed from those files.
% rationew on ramp_tri: P_fix = P_new on new ground (commanded h_d below the lowest so far), P_rev on revisits. sqrt(P_0) = 0.1, plane + cell, 10 seeds.
root = '/Users/kevin/Code/MotionControl_Simu-motion-test';
addpath(fullfile(root, 'test_script', 'scratch'));
% wiring check: P_rev = 0 must reproduce ratio sqrt(P_fix) = 0.15 exactly (seeds 1:2)
o = struct('law', 'f1p2', 'traj', 'ramp_tri', 'f_osc', 1, 'seeds', 1:2, 'q55_adapt', 'rationew', 'q55_adapt_P0', 0.15, 'q55_adapt_Prev', 0, ...
           'sig', 0.1, 'plants', {{'cell'}}, 'tag_suffix', '_sig010_rnchk');
run_plaw_walls(o);
A = load(fullfile(root, 'test_results', 'temp_plaw_walls', 'f1p2_cell_PI_sig010_rho15_rtri_f1.mat'));
B = load(fullfile(root, 'test_results', 'temp_plaw_walls', 'f1p2_cell_PI_sig010_rnchk_rtri_f1.mat'));
fprintf('WIRING CHECK P_rev = 0 vs ratio 0.15: max|d a_hat| = %g\n', max(max(abs(A.a_hat(:, 1:2) - B.a_hat))));
for pn = [0.15 0.20]
    for pr = [0.05 0.10]
        o = struct('law', 'f1p2', 'traj', 'ramp_tri', 'f_osc', 1, 'seeds', 1:10, 'q55_adapt', 'rationew', 'q55_adapt_P0', pn, 'q55_adapt_Prev', pr, ...
                   'sig', 0.1, 'plants', {{'cell', 'plane'}});
        o.tag_suffix = sprintf('_sig010_rn%02d_%02d', round(100 * pn), round(100 * pr));
        run_plaw_walls(o);
    end
end
disp('RATIONEW DONE');
addpath(fullfile(root, 'test_script', 'integration'));
verify_formC_b_production_defaults;
disp('VERIFY DONE');
