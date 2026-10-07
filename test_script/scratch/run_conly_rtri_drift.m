% STATUS: SCRATCH -- 2026-10-06 (user-approved decisive test of O50, ledger O52): Q55 from how much c may change, not from P_fix and I.
%   ramp_tri (T_d 1 s, triangle 1.10-2.0 R at 1 Hz), plane + cell, plane init, sqrt(P_0) 0.1, 10 seeds.
%   'drift'   Q55 = dc^2 |Delta w_d| / h_hat^2     (O50, random walk in 1/h)
%   'driftsq' Q55 = (dc |Delta w_d| / h_hat^2)^2   (deterministic step change squared)
%   dc = 0.76 (wall-family change of c_eff over one approach; the estimator does not know the wall).
%   Baseline (already on disk): single sqrt(P_fix) 0.15, tag _sig010_rho15.   Then the production-defaults acceptance check.
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root, 'test_script', 'scratch'));  addpath(fullfile(root, 'test_script', 'integration'));
ARMS = {{'drift', '_sig010_dr76'}, {'driftsq', '_sig010_dq76'}};
for j = 1:numel(ARMS)
    a = ARMS{j};
    o = struct('law', 'f1p2', 'traj', 'ramp_tri', 'f_osc', 1, 'seeds', 1:10, 'q55_adapt', a{1}, 'q55_adapt_dc', 0.76, ...
               'sig', 0.1, 'plants', {{'cell', 'plane'}}, 'tag_suffix', a{2});
    run_plaw_walls(o);
end
disp('DRIFT DONE');
verify_formC_b_production_defaults;
disp('VERIFY DONE');
