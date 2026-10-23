% STATUS: SCRATCH -- 2026-10-18: last 0.15 s before contact, u-law regulation of the a' state (run_mc_apreg_u settings).
%   Shortens T in 0.1 s steps until the seed survives, then prints h_d, h, a_true, a_hat, a'-hat, a'-true and the step.
function probe_mc_apreg_u_contact(w, ad, tau, seed, oracle)
    if nargin < 5; oracle = ''; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'temp_1009_ws');
    AP = struct('ap_slot', true, 'fe_cmd_eval', true, 'delta_init', 1 / 22.2^2, 'Pf_delta_std', 0.002, 'plaw_delta_lim', [-1 10], ...
                'plaw_seed_cross', false, 'q77_adapt_P0', 0.4, 'q77_hscale', 1.1);
    GX = struct('beta', 2 / 51, 'thr', 0.0066, 'ap_min', 1 / 22.2^2, 'coord', 'u');  if ~isempty(oracle); GX.oracle = oracle; end
    for T = [5.9:-0.1:1, 0.95:-0.05:0.05]
        try
            evalc('run_mc_s9_gainreg({w}, seed, ad, tau, T, 0, AP, ''_uc'', [], 0, GX);');
            f = fullfile(od, sprintf('gainreg_%s_tau%03d_T%g_uc.mat', w, round(1000 * tau), T));  S = load(f);  delete(f);  break;
        catch
        end
    end
    fprintf('%s a_d %.2f tau %.1f seed %d: survives %.1f s, contact before %.1f s\n', w, ad, tau, seed, T, T + 0.1);
    n = numel(S.t);  wd = S.wd;
    for k = unique([2:max(1, round(n / 12)):n, n])
        fprintf('  t %.3f | h_d %.3f | h %.3f | a_true %.3f | a_hat %.3f | a''_hat %.4f | a''_true %.3f | step %+.4f R\n', S.t(k), wd(k), S.h_true(k), ...
                S.a_true(k), S.a_hat(k), -S.w_hat(k), S.ap_true(k), wd(k) - wd(k - 1));
    end
end
