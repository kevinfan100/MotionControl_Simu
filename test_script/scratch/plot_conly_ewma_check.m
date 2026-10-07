% STATUS: SCRATCH -- 2026-10-06 (user: "show the mathematical basis of 'small R averages, large R follows', not a story fitted to plots").
%   Scalar-case recursion from the doc (part2 S5): c_hat[k+1] = c_hat[k] + g[k] (c_read[k] - c_hat[k]),
%   g[k] = l52 h25 = p^f I[k] / (1 + p^f I[k]),  p^f = P_fix,  E[c_read[k]] ~ c_eff(h_d[k]).
%   Prediction = this recursion driven by the logged I[k] (seed mean) and c_eff(h_d), started from the actual seed-mean c_hat[0].
%   It contains no fitted number. Compared with the actual seed-mean c_hat of the full filter.
%   Rows: narrow 1.1-2 R 5 Hz, wide 1.1-10 R 2.5 Hz (both T_d 0.5, sqrt(P_fix) 0.15); columns plane | cell.
function plot_conly_ewma_check()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_plaw_walls');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    F = {'f1p2_%s_PI_sig010_rho15_rtri_f5_td0p5.mat', 'narrow 1.1-2 R, 5 Hz', 0.15;
         'f1p2_%s_PI_sig010_rho15_rtri_f2p5_td0p5_top1000.mat', 'wide 1.1-10 R, 2.5 Hz', 0.15;
         'f1p2_%s_PI_sig010_rho35_rtri_f2p5_td0p5_top1000.mat', 'wide 1.1-10 R, 2.5 Hz', 0.35};
    WALLS = {'plane', 'cell'};  FS = 15;  LFS = 11;  AXLW = 1.8;  cT = [0.8 0 0];  cE = [0 0.2 0.9];
    f = figure('Position', [40 40 1600 1250], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(size(F, 1), 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for i = 1:size(F, 1)
        for iw = 1:2
            S = load(fullfile(id, sprintf(F{i, 1}, WALLS{iw})));  t = S.t(:);  I = mean(S.q55a_stat, 2);  ce = S.c_eff_d(:);
            ce(S.wd(:) < 1) = ce(find(S.wd(:) >= 1, 1));                   % t = 0 sample (h_d = 0 before the first step)
            Pf = F{i, 3}^2;  g = Pf * I ./ (1 + Pf * I);
            cm = mean(S.c_hat, 2);  cp = zeros(size(cm));  cp(1) = cm(1);
            for k = 1:numel(t) - 1; cp(k + 1) = cp(k) + g(k) * (ce(k) - cp(k)); end
            ax = nexttile(tl, (i - 1) * 2 + iw);  hold on; box on;
            plot(ax, t, ce, '-', 'Color', cT, 'LineWidth', 1.4, 'DisplayName', 'c_{eff}(h_d)');
            plot(ax, t, cm, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'c_{hat}: full filter, 10-seed mean');
            plot(ax, t, cp, 'k--', 'LineWidth', 1.6, 'DisplayName', 'c_{hat}[k+1] = c_{hat}[k] + g[k](c_{eff} - c_{hat}[k])');
            tri = t > 0.5;  r = corr(cp(tri), cm(tri));  rms_d = sqrt(mean((cp(tri) - cm(tri)).^2));
            fprintf('%-5s %-24s sqrtP_fix %.2f | recursion vs filter (t > 0.5 s): corr %.2f, rms diff %.3f | c_eff swing %.2f, filter swing %.2f, recursion swing %.2f\n', ...
                WALLS{iw}, F{i, 2}, F{i, 3}, r, rms_d, local_sw(ce(tri)), local_sw(cm(tri)), local_sw(cp(tri)));
            ylim(ax, [-0.4 1.0]);  xlim(ax, [0 t(end)]);  set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax, 'off');
            if i == 1
                lg = legend(ax, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'AutoUpdate', 'off');
                lg.Title.String = sprintf('%s wall, plane init', WALLS{iw});
            end
            if iw == 1; ylabel(ax, sprintf('c  (%s, \\surdP_{fix} %.2f)', F{i, 2}, F{i, 3}), 'FontSize', FS - 3, 'FontWeight', 'bold'); end
            if i == size(F, 1); xlabel(ax, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
        end
    end
    out = fullfile(fd, 'fig_conly_ewma_check.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end

function s = local_sw(x)
    s = prctile(x, 95) - prctile(x, 5);
end
