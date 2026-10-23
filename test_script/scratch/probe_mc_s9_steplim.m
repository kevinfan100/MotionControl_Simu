% STATUS: SCRATCH -- 2026-10-15 (user): fast approach (tau_a 0.05 s) with a step limit |Delta x_d| <= eta (h_d - w_s-hat) (run_formC_b
%   gain_reg.eta) vs the current tau_a 0.1 s without a limit. Realistic protocol: w_s-hat[0] = 0, true wall moved by DELTA (plant side).
%   Plane a_d 0.07 (h* 1.078 R), cell a_d 0.52 (h* 1.174 R); S8 + fe_cmd_eval; 6 s; seeds 1:5 one by one.
%   Prints per (wall, DELTA, arm): contacts, last 0.5 s mean (sd) of h - h* and of the w_s-hat error over the seeds that ran.
function probe_mc_s9_steplim(etas, deltas_plane, deltas_cell)
    if nargin < 1 || isempty(etas); etas = [0.01 0.02 0.05]; end
    if nargin < 2 || isempty(deltas_plane); deltas_plane = [-2 -1 0 2]; end
    if nargin < 3 || isempty(deltas_cell); deltas_cell = [-1 0 1]; end
    here = fileparts(mfilename('fullpath'));  od = fullfile(fileparts(fileparts(here)), 'test_results', 'temp_1009_ws');  addpath(here);
    E = struct('fe_cmd_eval', true);
    K = {'plane', 0.07, 1.0781, 0, deltas_plane; 'cell', 0.52, 1.1741, -1.05, deltas_cell};
    arms = [{struct('tau', 0.1, 'eta', 0, 'name', 'tau 0.10')}, arrayfun(@(e) struct('tau', 0.05, 'eta', e, 'name', sprintf('tau 0.05 eta %.2f', e)), etas, 'UniformOutput', false)];
    for i = 1:2
        fprintf('\n[%s, a_d %.2f, h* %.3f R] DELTA | arm | contacts | h - h* mean (sd) [R] | w_s-hat error mean (sd)\n', K{i, 1}, K{i, 2}, K{i, 3});
        for DE = K{i, 5}
            for ia = 1:numel(arms)
                a = arms{ia};  gx = struct();  if a.eta > 0; gx.eta = a.eta; end
                nc = 0;  dh = [];  ew = [];
                for sd = 1:5
                    tg = strrep(strrep(sprintf('_sl_%+g_%03d_s%d', DE, round(100 * a.eta), sd), '+', 'p'), '-', 'm');
                    try
                        evalc('run_mc_s9_gainreg(K(i, 1), sd, K{i, 2}, a.tau, 6, 0, E, tg, [], DE, gx);');
                        S = load(fullfile(od, sprintf('gainreg_%s_tau%03d_T6%s.mat', K{i, 1}, round(1000 * a.tau), tg)));
                        if min(S.h_true) <= 1.0011; nc = nc + 1; continue; end
                        dh(end + 1) = mean(S.h_true(S.t > 5.5)) - K{i, 3};  ew(end + 1) = S.w_hat(end) - (DE + K{i, 4}); %#ok<AGROW>
                    catch
                        nc = nc + 1;
                    end
                end
                fprintf('  %+3g | %-17s | %d/5 | %+.3f (%.3f) | %+.3f (%.3f)\n', DE, a.name, nc, mean(dh), std(dh), mean(ew), std(ew));
            end
        end
    end
end
