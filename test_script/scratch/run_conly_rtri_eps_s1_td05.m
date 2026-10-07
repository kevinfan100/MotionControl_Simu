% STATUS: SCRATCH -- runner for hypotheses_formC_b.md C88 (eps per wall, seed 1, T_d 0.5) (Layer-1 session, 2026-10-04/05; moved from the job tmp dir).
%   Writes test_results/temp_plaw_walls/*.mat; the analysis numbers in the ledger row are computed from those files.
% House-style arms (ap model 10-05 spec): seed 1, ramp_tri with Td = 0.5 s, sqrt(P_0) = 0.1; each wall's best eps (plane 1e-3, cell 3e-3).
root = '/Users/kevin/Code/MotionControl_Simu-motion-test';
addpath(fullfile(root, 'test_script', 'scratch'));
A = {'plane', 1e-3, '_s1_eps1e3'; 'cell', 3e-3, '_s1_eps3e3'};
for i = 1:size(A, 1)
    o = struct('law', 'f1p2', 'sig', 0.10, 'seeds', 1, 'Td', 0.5, 'f_osc', 1, 'plants', {A(i, 1)}, ...
               'q55_adapt', 'r2ratio', 'q55_adapt_rho', A{i, 2}, 'tag_suffix', A{i, 3});
    run_plaw_walls(o);
end
disp('EPS S1 DONE');
