% STATUS: SCRATCH -- runner for hypotheses_formC_b.md R76 (covariance matching) (Layer-1 session, 2026-10-04/05; moved from the job tmp dir).
%   Writes test_results/temp_plaw_walls/*.mat; the analysis numbers in the ledger row are computed from those files.
% covmatch on ramp_tri, sqrt(P_0) = 0.1, plane + cell, 10 seeds:
%   'ratio' form  Q55 = P_fix^2 I (D / Rm), sqrt(P_fix) = 0.10;   'pure' form  Q55 = D (no P_fix).  EWMA window N = 100.
root = '/Users/kevin/Code/MotionControl_Simu-motion-test';
addpath(fullfile(root, 'test_script', 'scratch'));
arms = {'cmratio', 'ratio'; 'cmpure', 'pure'};
for i = 1:2
    o = struct('law', 'f1p2', 'traj', 'ramp_tri', 'f_osc', 1, 'seeds', 1:10, 'q55_adapt', 'covmatch', 'q55_adapt_P0', 0.10, ...
               'q55_adapt_N', 100, 'q55_adapt_cmf', arms{i, 2}, 'sig', 0.1, 'plants', {{'cell', 'plane'}});
    o.tag_suffix = ['_sig010_' arms{i, 1}];
    run_plaw_walls(o);
end
disp('COVMATCH DONE');
addpath(fullfile(root, 'test_script', 'integration'));
verify_formC_b_production_defaults;
disp('VERIFY DONE');
