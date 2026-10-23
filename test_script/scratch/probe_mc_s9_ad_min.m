% STATUS: SCRATCH -- 2026-10-14 (user): smallest a_d usable for BOTH Section 9 figure types on each wall, with the current estimator
%   setting (formC_b fe_cmd_eval). Feasible = for w_s-hat[0] = -3, 0, +3 and seeds 1:3, 6 s: no contact (run completes), NaN 0;
%   plane also: a_hat never at the gain floor a_bar_floor = 0.05 after 1 s. Prints min h_true, a_hat floor hits, last 0.5 s a_true - a_d.
function probe_mc_s9_ad_min(plant, ads)
    here = fileparts(mfilename('fullpath'));  od = fullfile(fileparts(fileparts(here)), 'test_results', 'temp_1009_ws');  addpath(here);
    for ad = ads
        ok = true;  row = '';
        for w = [-3 0 3]
            tg = sprintf('_ad%04d_min', round(10000 * ad));
            try
                evalc('run_mc_s9_gainreg({plant}, 1:3, ad, 0.1, 6, w, struct(''fe_cmd_eval'', true), tg);');
                wt = '';  if w < 0; wt = '_wm3'; elseif w > 0; wt = '_wp3'; end
                S = load(fullfile(od, sprintf('gainreg_%s_tau100%s_T6%s.mat', plant, wt, tg)));  L = S.t > 1;
                nf = nnz(S.a_hat(L, :) <= 0.05 + 1e-9);  e = mean(mean(S.a_true(S.t > 5.5, :) - ad));
                row = [row, sprintf(' | %+d: min h %.4f, floor %d, end %+.4f', w, min(S.h_true(:)), nf, e)];
                if strcmp(plant, 'plane') && nf > 0; ok = false; end
            catch
                row = [row, sprintf(' | %+d: CONTACT', w)];  ok = false;
            end
        end
        fprintf('%s a_d %.3f %s%s\n', plant, ad, local_v(ok), row);
    end
end

function s = local_v(ok)
    s = 'NO ';  if ok; s = 'OK '; end
end
