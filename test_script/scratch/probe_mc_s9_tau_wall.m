% STATUS: SCRATCH -- 2026-10-15 (user, test 5 of the wrong-wall plan): does a slower approach (larger tau_a) let w_s-hat learn before the
%   level debt builds up? Realistic protocol: w_s-hat[0] = 0, true wall moved by DELTA = -1 R (plant side); plane a_d 0.07 (h* 1.078 R) and
%   cell a_d 0.52 (h* 1.174 R); S8 + fe_cmd_eval; tau_a 0.05 / 0.1 / 0.3 / 1.0 s; T_sim 6 s (10 s for tau_a 1.0); seeds 1:5 run one by one.
%   Prints: contacts, last 0.5 s mean (sd) of h - h* and of the w_s-hat error over the seeds that ran.
function probe_mc_s9_tau_wall()
    here = fileparts(mfilename('fullpath'));  od = fullfile(fileparts(fileparts(here)), 'test_results', 'temp_1009_ws');  addpath(here);
    E = struct('fe_cmd_eval', true);  K = {'plane', 0.07, 1.0781, -1; 'cell', 0.52, 1.1741, -2.05};  DE = -1;
    for i = 1:2
        fprintf('\n[%s, true wall moved %+g R] tau_a | contacts | h - h* mean (sd) [R] | w_s-hat error mean (sd)\n', K{i, 1}, DE);
        for tau = [0.05 0.1 0.3 1.0]
            T = 6;  if tau >= 1; T = 10; end
            nc = 0;  dh = [];  ew = [];
            for sd = 1:5
                tg = sprintf('_tw_s%d', sd);
                try
                    evalc('run_mc_s9_gainreg(K(i, 1), sd, K{i, 2}, tau, T, 0, E, tg, [], DE);');
                    S = load(fullfile(od, sprintf('gainreg_%s_tau%03d_T%g%s.mat', K{i, 1}, round(1000 * tau), T, tg)));
                    if min(S.h_true) <= 1.0011; nc = nc + 1; continue; end
                    dh(end + 1) = mean(S.h_true(S.t > T - 0.5)) - K{i, 3};  ew(end + 1) = S.w_hat(end) - K{i, 4}; %#ok<AGROW>
                catch
                    nc = nc + 1;
                end
            end
            fprintf('  %.2f | %d/5 | %+.3f (%.3f) | %+.3f (%.3f)\n', tau, nc, mean(dh), std(dh), mean(ew), std(ew));
        end
    end
end
