% PURPOSE (2026-09-23 evening, user: "unify p = 2 and repeat the round we just did"): the c-only slope law a' = (1-c)/h^2
%   (plaw_cp_slope wiring with p pinned by a ~0 prior; runner law 'f1p2'), plane init c = 0.0533 (plane fit at p = 2),
%   c prior 0.6 (wall-family width) unless stated, walls plane and cell, one battery in this order:
%     1. fixed points 22.2 / 1.10 R, 3 s: observability (seed 7) + simulation (10 seeds)      -> obs_fixed_*_f1p2, f1p2_fixed_PI
%     2. ramp T_d = 2 s: observability + simulation (prior 0.6 and the narrow 0.10)             -> obs_ramp_f1p2_*_Td2, f1p2_walls_PI_ramp2, _sig010_ramp2
%     3. ramp T_d = 16 s: observability + simulation (prior 0.6)                                 -> obs_ramp_f1p2_*_Td16, f1p2_walls_PI_ramp16
%   Run in one MATLAB batch (about 55 min). EXPIRES: with the c-only line.
function run_cp2_battery()
    here = fileparts(mfilename('fullpath'));  addpath(here);
    t0 = tic;
    fprintf('\n##### 1. fixed points\n');
    probe_obs_fixed_height([], {'f1p2'});
    run_plaw_walls(struct('law', 'f1p2', 'plants', {{'plane', 'cell'}}, 'kind', 'fixed', 'heights', [22.2 1.10], 'T_hold', 3, 'seeds', 1:10));
    plot_f1_fixed('PI', 'f1p2');
    fprintf('##### fixed done (%.0f s)\n', toc(t0));
    fprintf('\n##### 2. ramp T_d = 2 s\n');
    probe_obs_ramp_f1(2, [], 'f1p2');
    run_plaw_walls(struct('law', 'f1p2', 'traj', 'ramp', 'Td', 2, 'plants', {{'plane', 'cell'}}, 'seeds', 1:10));
    run_plaw_walls(struct('law', 'f1p2', 'traj', 'ramp', 'Td', 2, 'plants', {{'plane', 'cell'}}, 'seeds', 1:10, 'sig', 0.10, 'tag_suffix', '_sig010'));
    plot_plaw_walls('PI', '_ramp2', {'plane', 'cell'}, 'f1p2');
    plot_plaw_walls('PI', '_sig010_ramp2', {'plane', 'cell'}, 'f1p2');
    fprintf('##### ramp 2 s done (%.0f s)\n', toc(t0));
    fprintf('\n##### 3. ramp T_d = 16 s\n');
    probe_obs_ramp_f1(16, [], 'f1p2');
    run_plaw_walls(struct('law', 'f1p2', 'traj', 'ramp', 'Td', 16, 'plants', {{'plane', 'cell'}}, 'seeds', 1:10));
    plot_plaw_walls('PI', '_ramp16', {'plane', 'cell'}, 'f1p2');
    fprintf('##### all done (%.0f s)\n', toc(t0));
end
