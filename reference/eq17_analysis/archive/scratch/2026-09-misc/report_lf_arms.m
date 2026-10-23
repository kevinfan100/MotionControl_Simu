% PURPOSE (2026-09-24, user: "Meng's conditional forgetting as the main line"): one table per saved run_plaw_walls result:
%   e_a at the wall (descent end) and in the hold, c_hat at the descent end against c_eff(1.1) of that wall, the forgetting
%   factor's behaviour (mean lambda and fraction of steps with lambda < 1 in the descent and in the hold, first height at which
%   it fired), sqrt(P55) at the start and end of the hold (windup check), and NIS1 in the descent.
%   Usage: report_lf_arms({'f1p2_cell_PI_sig010_ramp2', 'f1p2_cell_PI_sig010_ma3_ramp2', ...})
function report_lf_arms(tags)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    rd = fullfile(root, 'test_results', 'temp_plaw_walls');
    CEFF = struct('plane', 0.065, 'cell', 0.743);      % c_eff(1.1) = 1 - h^2 a'_true(h) at the descent end (plot_slope_fit_c_only_p2)
    fprintf('\n%-36s | %-16s | %-16s | %-22s | %-15s | %-15s | %s\n', 'run', 'e_a wall / hold', 'c end (c_eff)', 'lambda desc: mean, frac<1', ...
            'hold: mean, frac', 'sqrtP55 hold s/e', 'first fire h_d | NIS1 desc');
    for k = 1:numel(tags)
        f = fullfile(rd, [tags{k} '.mat']);  if ~exist(f, 'file'); fprintf('%-36s | missing\n', tags{k}); continue; end
        r = load(f);  t = r.t;  seg = r.seg;  pl = r.plant;
        desc = t >= seg(1) & t < seg(2);  hold = t >= seg(3);  ie = find(desc, 1, 'last');
        ea = r.a_true - r.a_hat;  m = mean(ea, 2);
        ce = mean(r.c_hat(ie, :));
        if isfield(r, 'lam_b') && ~isempty(r.lam_b)
            L = r.lam_b;  ld = L(desc, :);  lh = L(hold, :);
            lam_d = mean(ld, 'all');  fr_d = mean(ld < 1, 'all');  lam_h = mean(lh, 'all');  fr_h = mean(lh < 1, 'all');
            fired = any(L < 1, 2);  i1 = find(fired & desc, 1, 'first');  h1 = NaN;  if ~isempty(i1); h1 = r.wd(i1); end
        else
            lam_d = 1;  fr_d = 0;  lam_h = 1;  fr_h = 0;  h1 = NaN;
        end
        sP = r.sP_c;  ih = find(hold);  sPs = mean(sP(ih(1), :));  sPe = mean(sP(ih(end), :));
        ok = isfinite(r.innov1) & isfinite(r.S1) & r.S1 > 0;  nis1 = r.innov1.^2 ./ r.S1;  nis1(~ok) = NaN;  n1 = mean(nis1(desc, :), 'all', 'omitnan');
        fprintf('%-36s | %+.3f / %+.3f  | %+.3f (%+.3f)  | %.5f, %4.1f%%          | %.5f, %4.1f%% | %.3f / %.3f   | %5.2f R | %.2f\n', ...
                tags{k}, m(ie), mean(m(hold)), ce, CEFF.(pl), lam_d, 100 * fr_d, lam_h, 100 * fr_h, sPs, sPe, h1, n1);
    end
end
