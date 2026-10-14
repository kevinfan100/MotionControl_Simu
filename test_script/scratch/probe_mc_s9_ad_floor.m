% STATUS: SCRATCH -- 2026-10-14 (user): smallest a_d that the S9 gain regulation reaches without touching the wall, plane and cell.
%   Each a_d: run_mc_s9_gainreg (tau_a 0.1 s, w_s-hat[0] = 0, seeds 1:3, T_sim 3 s); touch = h_true at the plant floor 1.001
%   (the driver clamps h_bar_true there). Prints min h_true, steps at the floor, last-1-s mean a_true - a_d, end h_d.
function probe_mc_s9_ad_floor()
    here = fileparts(mfilename('fullpath'));  od = fullfile(fileparts(fileparts(here)), 'test_results', 'temp_1009_ws');  addpath(here);
    C = {'plane', [0.1 0.07 0.05 0.04 0.03 0.02 0.01]; 'cell', [0.52 0.51 0.50 0.495 0.49 0.485]};
    for ic = 1:2
        fprintf('\n[%s] a_d | min h_true per seed | steps at floor (all seeds) | last 1 s a_true - a_d | h_d end | a_hat NaN\n', C{ic, 1});
        for ad = C{ic, 2}
            try
                evalc('run_mc_s9_gainreg(C(ic, 1), 1:3, ad, 0.1, 3, 0);');
                S = load(fullfile(od, sprintf('gainreg_%s_tau100_T3.mat', C{ic, 1})));  L = S.t > 2;
                fprintf('%.3f | %s | %d | %+.4f | %.4f | %d\n', ad, sprintf('%.4f ', min(S.h_true)), nnz(S.h_true <= 1.0011), ...
                        mean(mean(S.a_true(L, :) - ad)), mean(S.wd(end, :)), nnz(~isfinite(S.a_hat)));
            catch ME
                fprintf('%.3f | FAILED: %s\n', ad, ME.message);
            end
        end
    end
end
