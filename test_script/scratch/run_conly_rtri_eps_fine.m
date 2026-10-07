% STATUS: SCRATCH -- runner for hypotheses_formC_b.md C88 (eps r2 fine scan) (Layer-1 session, 2026-10-04/05; moved from the job tmp dir).
%   Writes test_results/temp_plaw_walls/*.mat; the analysis numbers in the ledger row are computed from those files.
% Finer Q55 = eps * R2 scan on ramp_tri (Td = 1 s), sqrt(P_0) = 0.1, plane + cell, 10 seeds: eps in {3e-4, 3e-3, 5e-3, 2e-2}.
root = '/Users/kevin/Code/MotionControl_Simu-motion-test';
addpath(fullfile(root, 'test_script', 'scratch'));
for e = [3e-4 3e-3 5e-3 2e-2]
    o = struct('law', 'f1p2', 'traj', 'ramp_tri', 'f_osc', 1, 'seeds', 1:10, 'q55_adapt', 'r2ratio', 'q55_adapt_rho', e, ...
               'sig', 0.1, 'plants', {{'cell', 'plane'}});
    o.tag_suffix = ['_sig010_epsv' strrep(sprintf('%g', e), '.', 'p')];
    run_plaw_walls(o);
end
disp('EPS FINE DONE');
