% STATUS: SCRATCH -- 2026-10-06 (user: separate speed from range). ramp_tri T_d 0.5 s, single sqrt(P_fix) 0.15, plane init, 10 seeds.
%   Columns: narrow 1.1-2 R at 5 Hz (9 R/s) | narrow 1.1-2 R at 23.5 Hz (42 R/s) | wide 1.1-10 R at 2.5 Hz (44.5 R/s).
%   Rows: (1) h_d  (2) abar true red / est blue (seed mean)  (3) e_abar seed mean +- sd  (4) c_eff(h_d) red, c_hat seed mean blue.
%   One figure per wall: fig_conly_speed_range_<wall>.png.  Console: per column, info per leg, c_eff change per leg, c_hat swing,
%   corr(c_hat, c_eff), near-wall (h_d < 2) |e_abar|, mean, seed sd; min h, NaN.
function plot_conly_rtri_speed_range()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_plaw_walls');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    C = {'_rtri_f5_td0p5', 5, 'narrow 1.1-2 R, 5 Hz (9 R/s)';
         '_rtri_f23p5_td0p5', 23.5, 'narrow 1.1-2 R, 23.5 Hz (42 R/s)';
         '_rtri_f2p5_td0p5_top1000', 2.5, 'wide 1.1-10 R, 2.5 Hz (44.5 R/s)'};
    cT = [0.8 0 0];  cE = [0 0.2 0.9];  FS = 15;  AXLW = 1.8;  win = 1 / 0.15^2;
    for w = {'plane', 'cell'}
        f = figure('Position', [40 40 1900 1400], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
        tl = tiledlayout(4, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
        for ic = 1:3
            S = load(fullfile(id, sprintf('f1p2_%s_PI_sig010_rho15%s.mat', w{1}, C{ic, 1})));  t = S.t(:);
            ax = gobjects(4, 1);  for r = 1:4; ax(r) = nexttile(tl, (r - 1) * 3 + ic); hold(ax(r), 'on'); box(ax(r), 'on'); end
            plot(ax(1), t, S.wd, 'k-', 'LineWidth', 1.2);
            plot(ax(2), t, mean(S.a_true, 2), '-', 'Color', cT, 'LineWidth', 1.4);  plot(ax(2), t, mean(S.a_hat, 2), '-', 'Color', cE, 'LineWidth', 1.2);
            e = S.a_true - S.a_hat;  m = mean(e, 2);  s = std(e, 0, 2);
            fill(ax(3), [t; flipud(t)], [m - s; flipud(m + s)], cE, 'FaceAlpha', 0.2, 'EdgeColor', 'none');  plot(ax(3), t, m, '-', 'Color', cE, 'LineWidth', 1.3);
            plot(ax(3), [0 t(end)], [0 0], ':', 'Color', [0.3 0.3 0.3]);
            ce = S.c_eff_d(:);  ce(S.wd(:) < 1) = NaN;
            plot(ax(4), t, ce, '-', 'Color', cT, 'LineWidth', 1.3);  plot(ax(4), t, mean(S.c_hat, 2), '-', 'Color', cE, 'LineWidth', 1.6);
            ylim(ax(1), [0 23]);  ylim(ax(2), [0 1.1]);  ylim(ax(3), [-0.1 0.1]);  ylim(ax(4), [-0.4 1.0]);
            for r = 1:4; xlim(ax(r), [0 t(end)]); set(ax(r), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW); grid(ax(r), 'off'); end
            title(ax(1), '');  lg = legend(ax(1), {'h_d'}, 'Location', 'northoutside', 'FontSize', FS - 4, 'FontWeight', 'bold');
            lg.Title.String = sprintf('%s wall, plane init: %s', w{1}, C{ic, 3});
            xlabel(ax(4), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
            if ic == 1
                ylabel(ax(1), 'h_d', 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax(2), 'a_z / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
                ylabel(ax(3), 'e_a  (true - est)', 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax(4), 'c  (red c_{eff}, blue c_{hat})', 'FontSize', FS, 'FontWeight', 'bold');
            end
            tri = t > 0.5;  nw = tri & S.wd(:) < 2;  I = mean(S.q55a_stat, 2);  nleg = 2 * C{ic, 2} * 2;  cm = mean(S.c_hat, 2);
            ce2 = S.c_eff_d(:);
            fprintf('%-5s %-34s info/leg %6.1f (%.2f windows) | c_eff change/leg %.2f | c_hat swing %.2f corr %+.2f | near-wall |e_a| %.4f mean %+.4f seed-sd %.4f | min h %.3f NaN %d\n', ...
                w{1}, C{ic, 3}, sum(I(tri)) / nleg, sum(I(tri)) / nleg / win, max(ce2(tri)) - min(ce2(tri)), prctile(cm(tri), 95) - prctile(cm(tri), 5), ...
                corr(cm(tri), ce2(tri)), mean(abs(e(nw, :)), 'all'), mean(e(nw, :), 'all'), std(mean(e(nw, :), 1)), min(S.h_true(:)), nnz(isnan(S.a_hat)));
        end
        out = fullfile(fd, sprintf('fig_conly_speed_range_%s.png', w{1}));  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
    end
end
