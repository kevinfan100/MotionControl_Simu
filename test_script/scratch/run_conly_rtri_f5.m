% STATUS: SCRATCH -- runner for hypotheses_formC_b.md C89 (5 Hz triangle, rationew vs P_fix 0.15) (Layer-1 session, 2026-10-04/05; moved from the job tmp dir).
%   Writes test_results/temp_plaw_walls/*.mat; the analysis numbers in the ledger row are computed from those files.
% 10 seeds, ramp_tri Td 1 s, triangle 5 Hz (same band 1.10-2.00): rationew 0.15/0.05 and ratio 0.15, plane + cell.
root = '/Users/kevin/Code/MotionControl_Simu-motion-test';
addpath(fullfile(root, 'test_script', 'scratch'));
arms = {{'rationew', 0.05, '_sig010_rn15_05'}, {'ratio', 0, '_sig010_rho15'}};
for i = 1:numel(arms)
    a = arms{i};
    o = struct('law', 'f1p2', 'traj', 'ramp_tri', 'f_osc', 5, 'seeds', 1:10, 'q55_adapt', a{1}, 'q55_adapt_P0', 0.15, 'q55_adapt_Prev', a{2}, ...
               'sig', 0.1, 'plants', {{'cell', 'plane'}}, 'tag_suffix', a{3});
    run_plaw_walls(o);
end
disp('F5 DONE');
