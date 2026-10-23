% STATUS: SCRATCH -- 2026-10-15 (user): process noise on the wall position (formC_b q77_adapt_P0 = sqrt P_fix, Q77 = P_fix^2 I7) under the
%   realistic wrong-wall protocol (w_s-hat[0] = 0, true wall moved by DELTA on the plant side). Section 9 regulation, S8 + fe_cmd_eval,
%   tau_a 0.1 s, 6 s, seeds 1:5 one by one. Plane a_d 0.07 (h* 1.078 R), cell a_d 0.52 (h* 1.174 R).
%   Prints per (wall, DELTA, sqrt P_fix): contacts, last 0.5 s mean (worst) of h - h*, w_s-hat error mean, honesty |e_w| / sqrt P_ws.
function probe_mc_s9_wsQ(pfix)
    if nargin < 1 || isempty(pfix); pfix = [0 0.06 0.245]; end
    here = fileparts(mfilename('fullpath'));  od = fullfile(fileparts(fileparts(here)), 'test_results', 'temp_1009_ws');  addpath(here);
    K = {'plane', 0.07, 1.0781, 0, [-1 1 2]; 'cell', 0.52, 1.1741, -1.05, [-1 1 2]};
    for i = 1:2
        fprintf('\n[%s] DELTA | sqrt P_fix | contacts | h - h* mean (worst) [R] | w_s-hat error | honesty_w\n', K{i, 1});
        for DE = K{i, 5}
            for pf = pfix
                E = struct('fe_cmd_eval', true);  if pf > 0; E.q77_adapt_P0 = pf; end
                nc = 0;  dh = [];  ew = [];  hw = [];
                for sd = 1:5
                    tg = strrep(strrep(sprintf('_wq_%+g_%03d_s%d', DE, round(1000 * pf), sd), '+', 'p'), '-', 'm');
                    try
                        evalc('run_mc_s9_gainreg(K(i, 1), sd, K{i, 2}, 0.1, 6, 0, E, tg, [], DE);');
                        S = load(fullfile(od, sprintf('gainreg_%s_tau100_T6%s.mat', K{i, 1}, tg)));
                        if min(S.h_true) <= 1.0011; nc = nc + 1; continue; end
                        e = S.w_hat(end) - (DE + K{i, 4});
                        dh(end + 1) = mean(S.h_true(S.t > 5.5)) - K{i, 3};  ew(end + 1) = e;  hw(end + 1) = abs(e) / S.sPw(end); %#ok<AGROW>
                    catch
                        nc = nc + 1;
                    end
                end
                if isempty(dh); fprintf('  %+g | %.3f | %d/5 |\n', DE, pf, nc);
                else; [~, w] = max(abs(dh)); fprintf('  %+g | %.3f | %d/5 | %+.3f (%+.3f) | %+.3f | %.1f\n', DE, pf, nc, mean(dh), dh(w), mean(ew), mean(hw)); end
            end
        end
    end
end
