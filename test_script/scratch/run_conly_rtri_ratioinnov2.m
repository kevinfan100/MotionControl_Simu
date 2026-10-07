% STATUS: SCRATCH -- runner for hypotheses_formC_b.md R76 (innovation variants, N 25 / I-weighted / max1) (Layer-1 session, 2026-10-04/05; moved from the job tmp dir).
%   Writes test_results/temp_plaw_walls/*.mat; the analysis numbers in the ledger row are computed from those files.
% ratioinnov variants on ramp_tri, sqrt(P_low) = 0.10, sqrt(P_0) = 0.1, plane + cell, 10 seeds:
%   method 1 = short window N = 25 (plain mean / sigma), method 2 = I-weighted window N = 100; each with g = x ('lin') and g = max(1, x).
root = '/Users/kevin/Code/MotionControl_Simu-motion-test';
addpath(fullfile(root, 'test_script', 'scratch'));
arms = {'n25lin', 25, false, 'lin'; 'n25max', 25, false, 'max1'; 'wIlin', 100, true, 'lin'; 'wImax', 100, true, 'max1'};
for i = 1:size(arms, 1)
    o = struct('law', 'f1p2', 'traj', 'ramp_tri', 'f_osc', 1, 'seeds', 1:10, 'q55_adapt', 'ratioinnov', 'q55_adapt_P0', 0.10, ...
               'q55_adapt_N', arms{i, 2}, 'q55_adapt_wI', arms{i, 3}, 'q55_adapt_gfun', arms{i, 4}, 'sig', 0.1, 'plants', {{'cell', 'plane'}});
    o.tag_suffix = ['_sig010_ri10' arms{i, 1}];
    run_plaw_walls(o);
end
disp('RATIOINNOV2 DONE');
% wiring check: ratioinnov N = 100, lin, wI off must reproduce the earlier ri10 run exactly
o = struct('law', 'f1p2', 'traj', 'ramp_tri', 'f_osc', 1, 'seeds', 1:2, 'q55_adapt', 'ratioinnov', 'q55_adapt_P0', 0.10, 'q55_adapt_N', 100, ...
           'sig', 0.1, 'plants', {{'cell'}}, 'tag_suffix', '_sig010_ri10chk');
run_plaw_walls(o);
A = load(fullfile(root, 'test_results', 'temp_plaw_walls', 'f1p2_cell_PI_sig010_ri10_rtri_f1.mat'));
B = load(fullfile(root, 'test_results', 'temp_plaw_walls', 'f1p2_cell_PI_sig010_ri10chk_rtri_f1.mat'));
fprintf('WIRING CHECK max|d a_hat| (seeds 1:2 vs earlier ri10) = %g\n', max(max(abs(A.a_hat(:, 1:2) - B.a_hat))));
addpath(fullfile(root, 'test_script', 'integration'));
verify_formC_b_production_defaults;
disp('VERIFY DONE');
