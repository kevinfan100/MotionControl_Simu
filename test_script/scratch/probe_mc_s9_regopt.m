% STATUS: SCRATCH -- 2026-10-15 (user): can the gain regulation survive a wrong wall guess better? Realistic protocol: estimator always starts
%   at w_s-hat[0] = 0, the TRUE wall moved by DELTA on the plant side (ws_inject). S8 estimator with fe_cmd_eval, 6 s, seeds 1:5.
%   Arms: 'base' (tau_a 0.1 s), 'tau03' (tau_a 0.3 s), 'B1' (two stages: a_mid until 1.5 s, then a_d), 'B2' (B1 + known 0.05 R / 5 Hz command
%   dither during the first stage, 0.5-1.5 s). Plane a_d 0.07 (h* 1.078 R, a_mid 0.3); cell a_d 0.52 (h* 1.174 R, a_mid 0.7).
%   Prints per (wall, DELTA, arm): seeds that touch the wall, last 0.5 s mean (worst) of h_true - h* over the seeds that ran.
function probe_mc_s9_regopt(walls)
    if nargin < 1; walls = {'plane', 'cell'}; end
    here = fileparts(mfilename('fullpath'));  od = fullfile(fileparts(fileparts(here)), 'test_results', 'temp_1009_ws');  addpath(here);
    W = struct('plane', struct('ad', 0.07, 'hs', 1.0781, 'amid', 0.3, 'D', [-3 -2 -1 0 2 5 8]), ...
               'cell',  struct('ad', 0.52, 'hs', 1.1741, 'amid', 0.7, 'D', [-1 0 1 2 3]));
    A = {'base', 'tau03', 'B1', 'B2'};  E = struct('fe_cmd_eval', true);
    for iw = 1:numel(walls)
        w = W.(walls{iw});
        fprintf('\n[%s] a_d %.2f, h* %.3f R | per DELTA: contact seeds / 5, last 0.5 s mean (worst) of h - h* [R]\n', walls{iw}, w.ad, w.hs);
        for DE = w.D
            row = sprintf('%+3g |', DE);
            for ia = 1:numel(A)
                tau = 0.1;  gx = struct();  dith = [];
                if strcmp(A{ia}, 'tau03'); tau = 0.3; end
                if startsWith(A{ia}, 'B'); gx = struct('a_mid', w.amid, 't_switch', 1.5); end
                if strcmp(A{ia}, 'B2'); dith = struct('amp_R', 0.05, 'freq', 5, 't_on', 0.5, 't_off', 1.5); end
                nc = 0;  dh = [];
                for sd = 1:5
                    tg = sprintf('_ro_%s_%s_%+g_s%d', walls{iw}, A{ia}, DE, sd);  tg = strrep(strrep(tg, '+', 'p'), '-', 'm');
                    try
                        evalc('run_mc_s9_gainreg(walls(iw), sd, w.ad, tau, 6, 0, E, tg, dith, DE, gx);');
                        S = load(fullfile(od, sprintf('gainreg_%s_tau%03d_T6%s.mat', walls{iw}, round(1000 * tau), tg)));
                        if min(S.h_true) <= 1.0011; nc = nc + 1; else; dh(end + 1) = mean(S.h_true(S.t > 5.5)) - w.hs; end %#ok<AGROW>
                    catch
                        nc = nc + 1;
                    end
                end
                if isempty(dh); row = [row sprintf(' %-5s %d/5 contact            |', A{ia}, nc)];
                else; [~, iwst] = max(abs(dh));  row = [row sprintf(' %-5s %d/5  %+.3f (%+.3f) |', A{ia}, nc, mean(dh), dh(iwst))]; end
            end
            fprintf('%s\n', row);
        end
    end
end
