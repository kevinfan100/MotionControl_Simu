% STATUS: SCRATCH -- 2026-10-06 (user: "change lambda_c 0.7 -> 0.4, do it fully, verify, then show the simulation").
%   1 wiring: no lambda_c and explicit 0.7 must both be bit-identical to the cached 0.7 run (seeds 1:2, cell).
%   2 lambda_c 0.4 on ramp_tri (T_d 1, 1.1-2 R at 1 Hz): single sqrt(P_fix) 0.10 / 0.15 / 0.20, plane + cell, 10 seeds.
%   3 lambda_c 0.4 on the wide setup (T_d 0.5, 1.1-10 R at 2.5 Hz), sqrt(P_fix) 0.15.
%   4 echo S measured (paired +-10 % forced gain mismatch, hold, arms T and N, 8 seeds) at lambda_c 0.4 and 0.7.
%   5 production-defaults acceptance.
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root, 'test_script', 'scratch'));  addpath(fullfile(root, 'test_script', 'integration'));
id = fullfile(root, 'test_results', 'temp_plaw_walls');
base = struct('law', 'f1p2', 'traj', 'ramp_tri', 'f_osc', 1, 'q55_adapt', 'ratio', 'q55_adapt_P0', 0.15, 'sig', 0.1);

% 1 wiring
o = base;  o.seeds = 1:2;  o.plants = {'cell'};  o.tag_suffix = '_sig010_rho15_wire';  run_plaw_walls(o);
o.lambda_c = 0.7;  run_plaw_walls(o);
A = load(fullfile(id, 'f1p2_cell_PI_sig010_rho15_rtri_f1.mat'), 'a_hat');
B = load(fullfile(id, 'f1p2_cell_PI_sig010_rho15_wire_rtri_f1.mat'), 'a_hat');
C = load(fullfile(id, 'f1p2_cell_PI_sig010_rho15_wire_rtri_f1_lc070.mat'), 'a_hat');
fprintf('WIRING  no lambda_c vs cached: max|d a_hat| = %g\n', max(abs(A.a_hat(:, 1:2) - B.a_hat), [], 'all'));
fprintf('WIRING  lambda_c 0.7 vs cached: max|d a_hat| = %g\n', max(abs(A.a_hat(:, 1:2) - C.a_hat), [], 'all'));

% 2 ramp_tri at 0.4, P_fix scan
for pf = [0.10 0.15 0.20]
    o = base;  o.seeds = 1:10;  o.plants = {'cell', 'plane'};  o.lambda_c = 0.4;  o.q55_adapt_P0 = pf;
    o.tag_suffix = sprintf('_sig010_rho%02d', round(100 * pf));  run_plaw_walls(o);
end
% 3 wide setup at 0.4
o = base;  o.seeds = 1:10;  o.plants = {'cell', 'plane'};  o.lambda_c = 0.4;  o.Td = 0.5;  o.f_osc = 2.5;  o.h_top_tri = 10;
o.tag_suffix = '_sig010_rho15';  run_plaw_walls(o);
disp('LC040 RUNS DONE');

% 4 echo S
for lc = [0.4 0.7]
    E = check_formB_echo_S_measure(struct('lambda_c', lc, 'arms_sel', {{'T', 'N'}}, 'eps_list', 0.10, 'seeds', 1:8, 'save_fig', false, 'verbose', true));
    fprintf('ECHO lambda_c %.1f: model S_T %+.4f S_n %+.4f\n', lc, E.S_T_model, E.S_n_model);
    for ia = 1:numel(E.arms)
        r = E.arms{ia};  fprintf('ECHO lambda_c %.1f arm %s: measured S (x,y,z) = %+.4f %+.4f %+.4f  (SEM %.4f %.4f %.4f)\n', lc, r.tag, r.S(end, :), r.S_sem(end, :));
    end
end
disp('ECHO DONE');

% 5 acceptance
verify_formC_b_production_defaults;
disp('VERIFY DONE');
