% PURPOSE (2026-10-04, user: "run the best P_0 / P_fix setting on the triangle trajectory, show plane and cell"): c-only f1p2 on
%   ramp_tri (1 s descent 22.2 -> 1.10 R, triangle 1.10 <-> 2.00 R 1 Hz for 2 s), plane init, Q55 = P_fix^2 I (q55_adapt 'ratio').
%   Prints the P_fix scan (sqrt(P_0) = 0.1, sqrt(P_fix) = 0.15 / 0.2 / 0.3) point by point, then draws the chosen setting in house
%   style: columns plane | cell, rows a_true (red) with a_hat mean +- sd over seeds (blue) | e_a = a_true - a_hat mean +- sd | c_hat mean +- sd with c_eff(h_d) = 1 - h_d^2 a'_true (red).
%   Reads temp_plaw_walls/f1p2_<wall>_PI_sig010_rho<15|20|30>_rtri_f1.mat.   STATUS: SCRATCH
function plot_conly_best_rtri(rho_best)
    if nargin < 1; rho_best = '20'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_plaw_walls');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    WALLS = {'plane', 'cell'};  RHOS = {'10', '15', '20', '30'};
    for iw = 1:numel(WALLS)
        fprintf('\n%s (sqrt P_0 = 0.1)\n', WALLS{iw});
        fprintf('  sqrtP_fix | troughs 1 / 2 / 3 s                          | tops 1.5 / 2.5 s            | mean|e_a| tri | mean sd_seed tri | worst |e_a| 0.9-1 s\n');
        for m = 1:numel(RHOS)
            fn = fullfile(id, sprintf('f1p2_%s_PI_sig010_rho%s_rtri_f1.mat', WALLS{iw}, RHOS{m}));
            if ~exist(fn, 'file'); fprintf('  missing %s\n', fn); continue; end
            S = load(fn);  t = S.t;  ea = S.a_true - S.a_hat;  at = @(tt) ea(find(t >= tt - 1e-9, 1), :);
            tr = [at(1); at(2); at(3)];  tp = [at(1.5); at(2.5)];  kt = t >= 1;  w = max(abs(ea(t >= 0.9 & t < 1, :)), [], 1);
            fprintf('  %.2f      | %+.3f+-%.3f %+.3f+-%.3f %+.3f+-%.3f | %+.3f+-%.3f %+.3f+-%.3f | %.4f        | %.4f           | %.3f+-%.3f\n', ...
                    str2double(RHOS{m}) / 100, mean(tr(1, :)), std(tr(1, :)), mean(tr(2, :)), std(tr(2, :)), mean(tr(3, :)), std(tr(3, :)), ...
                    mean(tp(1, :)), std(tp(1, :)), mean(tp(2, :)), std(tp(2, :)), mean(mean(abs(ea(kt, :)))), mean(std(ea(kt, :), 0, 2)), mean(w), std(w));
        end
    end
    FS = 18;  LFS = 13;  AXLW = 2.0;  cT = [0.8 0 0];  cE = [0 0.2 0.9];  cB = [0.45 0.55 0.95];
    f = figure('Position', [40 40 1600 1250], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(3, numel(WALLS), 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = gobjects(3, numel(WALLS));
    for iw = 1:numel(WALLS)
        S = load(fullfile(id, sprintf('f1p2_%s_PI_sig010_rho%s_rtri_f1.mat', WALLS{iw}, rho_best)));
        t = S.t(:)';  ea = S.a_true - S.a_hat;
        mh = mean(S.a_hat, 2)';  sh = std(S.a_hat, 0, 2)';  me = mean(ea, 2)';  se = std(ea, 0, 2)';
        ax(1, iw) = nexttile(tl, iw);  hold on; box on;
        hb = fill([t fliplr(t)], [mh + sh, fliplr(mh - sh)], cB, 'EdgeColor', 'none', 'FaceAlpha', 0.35, 'DisplayName', 'a_{hat} \pm sd');
        ht = plot(t, mean(S.a_true, 2), '-', 'Color', cT, 'LineWidth', 2.4, 'DisplayName', 'a_{true}');
        hh = plot(t, mh, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'a_{hat}');
        lg = legend([ht hh hb], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'AutoUpdate', 'off');
        lg.Title.String = sprintf('%s wall, plane init, \\surdP_0 = 0.1, \\surdP_{fix} = %.2f', WALLS{iw}, str2double(rho_best) / 100);
        ax(2, iw) = nexttile(tl, numel(WALLS) + iw);  hold on; box on;
        fill([t fliplr(t)], [me + se, fliplr(me - se)], cB, 'EdgeColor', 'none', 'FaceAlpha', 0.35);
        plot(t, me, '-', 'Color', cE, 'LineWidth', 1.8);  plot([0 3], [0 0], '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 0.8);
        mc = mean(S.c_hat, 2)';  sc = std(S.c_hat, 0, 2)';
        ax(3, iw) = nexttile(tl, 2 * numel(WALLS) + iw);  hold on; box on;
        fill([t fliplr(t)], [mc + sc, fliplr(mc - sc)], cB, 'EdgeColor', 'none', 'FaceAlpha', 0.35);
        ce = S.c_eff_d(:)';  ce(S.wd(:)' < 1) = NaN;  hr = plot(t, ce, '-', 'Color', cT, 'LineWidth', 2.0, 'DisplayName', 'c_{eff}(h_d)');
        hc = plot(t, mc, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'c_{hat}');
        legend([hr hc], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'AutoUpdate', 'off');
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        for r = 1:3
            yl = ylim(ax(r, iw));  plot(ax(r, iw), [1 1], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0);  ylim(ax(r, iw), yl);
            xlim(ax(r, iw), [0 3]);  set(ax(r, iw), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax(r, iw), 'off');
        end
    end
    ylabel(ax(1, 1), 'a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax(2, 1), 'e_a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
    linkaxes(ax(2, :), 'y');  ylabel(ax(3, 1), 'c', 'FontSize', FS, 'FontWeight', 'bold');  linkaxes(ax(3, :), 'y');
    out = fullfile(fd, sprintf('fig_conly_best_rtri_rho%s.png', rho_best));  exportgraphics(f, out, 'Resolution', 150);  fprintf('\nfigure: %s\n', out);
end
