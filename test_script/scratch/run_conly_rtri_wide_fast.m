% STATUS: SCRATCH -- 2026-10-06 (user: "raise the oscillation range and make its speed close to the descent, then see whether c tracks").
%   ramp_tri, T_d 0.5 s (descent 22.2 -> 1.10 R at 42.2 R/s), triangle from 1.10 R up to h_top, 2 s, frequency chosen so the
%   triangle speed 2 (h_top - 1.10) f ~ the descent speed (n_cycles = 2 f must be an integer):
%     top 2.0  f 5   ->  9.0 R/s  (baseline: the user's figure)
%     top 5.0  f 5.5 -> 42.9 R/s
%     top 10   f 2.5 -> 44.5 R/s
%     top 22.2 f 1   -> 42.2 R/s  (every cycle is a full descent)
%   Two q55 settings: 'ratio' sqrt(P_fix) 0.15 every step (forgets everywhere: the arm that CAN track) and 'rationew' 0.15 / 0.05
%   (revisits barely forget). Plane + cell, plane init, sqrt(P_0) 0.1, 10 seeds.  Writes test_results/temp_plaw_walls/*.mat.
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root, 'test_script', 'scratch'));
CASES = [2.0 5; 5.0 5.5; 10 2.5; 22.2 1];
ARMS = {{'ratio', 0, '_sig010_rho15'}, {'rationew', 0.05, '_sig010_rn15_05'}};
for i = 1:size(CASES, 1)
    for j = 1:numel(ARMS)
        a = ARMS{j};
        o = struct('law', 'f1p2', 'traj', 'ramp_tri', 'Td', 0.5, 'f_osc', CASES(i, 2), 'seeds', 1:10, ...
                   'q55_adapt', a{1}, 'q55_adapt_P0', 0.15, 'q55_adapt_Prev', a{2}, 'sig', 0.1, ...
                   'plants', {{'cell', 'plane'}}, 'tag_suffix', a{3});
        if abs(CASES(i, 1) - 2.0) > 1e-9; o.h_top_tri = CASES(i, 1); end
        run_plaw_walls(o);
    end
end
disp('WIDE FAST DONE');
