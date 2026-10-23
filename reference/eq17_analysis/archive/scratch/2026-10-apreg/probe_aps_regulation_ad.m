% STATUS: SCRATCH -- 2026-10-16 (user): the fig_mc_aps_regulation setting (a' as a state, ap_slot, q77_adapt_P0 0.4, q77_hscale 1.1,
%   fe_cmd_eval, prior 0.002, tau_a 0.1 s, 6 s, plane init) with smaller a_d. Per seed: contact or not; if not, mean over the last 0.5 s of
%   a_true - a_d, h_true - h*, a_true - a_hat, and min a'-hat over the run.
function probe_aps_regulation_ad()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration')); addpath(here);
    od = fullfile(root, 'test_results', 'temp_1009_ws');
    AP = struct('ap_slot', true, 'fe_cmd_eval', true, 'delta_init', 1 / 22.2^2, 'Pf_delta_std', 0.002, 'plaw_delta_lim', [-1 10], ...
                'plaw_seed_cross', false, 'q77_adapt_P0', 0.4, 'q77_hscale', 1.1);
    C = {'plane', 0.5, 2.128; 'plane', 0.3, NaN; 'plane', 0.1, 1.117; 'plane', 0.07, 1.078; ...
         'cell', 0.6, 1.654; 'cell', 0.55, 1.334; 'cell', 0.52, 1.174};
    fprintf('wall  a_d  h*    | seed | contact | a_true-a_d | h-h* [R] | a_true-a_hat | min a''-hat\n');
    for i = 1:size(C, 1)
        for s = 1:5
            tg = sprintf('_apsad_s%d', s);
            try
                clear run_formC_b motion_control_law_formC_b;
                evalc('run_mc_s9_gainreg(C(i, 1), s, C{i, 2}, 0.1, 6, 0, AP, tg);');
                S = load(fullfile(od, sprintf('gainreg_%s_tau100_T6%s.mat', C{i, 1}, tg)));  L = S.t > 5.5;
                fprintf('%-5s %.2f %5.3f | %d    | no      | %+.4f    | %+.4f  | %+.4f      | %+.4f\n', C{i, 1}, C{i, 2}, C{i, 3}, s, ...
                        mean(S.a_true(L)) - C{i, 2}, mean(S.h_true(L)) - C{i, 3}, mean(S.a_true(L) - S.a_hat(L)), min(-S.w_hat(2:end)));
            catch
                fprintf('%-5s %.2f %5.3f | %d    | YES\n', C{i, 1}, C{i, 2}, C{i, 3}, s);
            end
        end
    end
end
