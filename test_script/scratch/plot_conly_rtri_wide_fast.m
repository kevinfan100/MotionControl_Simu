% STATUS: SCRATCH -- 2026-10-06, plots run_conly_rtri_wide_fast.m (ramp_tri T_d 0.5, triangle top 2 / 5 / 10 / 22.2 R at ~descent speed).
%   Figure 1 fig_conly_wide_fast_c.png: rows = triangle top, columns plane | cell; c_eff(h_d) red, c_hat seed mean:
%     dark blue = sqrt(P_fix) 0.15 every step ('ratio'), light blue = 0.15 new / 0.05 revisit ('rationew'); grey = h_d (right axis).
%   Figure 2 fig_conly_wide_fast_ea.png: same layout, e_abar = abar_true - abar_hat, seed mean (line) +- seed sd (band).
%   Console: tracking in the triangle (t > T_d): corr(c_hat mean, c_eff), swing ratio sd(c_hat mean)/sd(c_eff),
%   |c_hat - c_eff| near the wall (h_d < 2) and everywhere, mean |e_abar|, seed sd, min h_true, NaN.
function plot_conly_rtri_wide_fast()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_plaw_walls');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    CASES = [2.0 5; 5.0 5.5; 10 2.5; 22.2 1];  WALLS = {'plane', 'cell'};
    ARMS = {'_sig010_rho15', '_sig010_rn15_05'};  ANAME = {'P_{fix} every step', 'P_{fix} new / revisit'};
    cA = {[0 0.2 0.9], [0.55 0.75 1.0]};  cT = [0.8 0 0];  cH = [0.6 0.6 0.6];  FS = 15;  LFS = 11;  AXLW = 1.8;  Td = 0.5;
    for fig = 1:2
        f = figure('Position', [40 40 1600 1500], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
        tl = tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
        for ic = 1:4
            for iw = 1:2
                ax = nexttile(tl, (ic - 1) * 2 + iw);  hold on; box on;  h = gobjects(0);
                for ia = 1:2
                    S = load(fullfile(id, local_name(WALLS{iw}, ARMS{ia}, CASES(ic, :))));  t = S.t(:);
                    if fig == 1
                        if ia == 1
                            ce = S.c_eff_d(:);  ce(S.wd(:) < 1) = NaN;
                            h(end + 1) = plot(ax, t, ce, '-', 'Color', cT, 'LineWidth', 1.4, 'DisplayName', 'c_{eff}(h_d)'); %#ok<AGROW>
                        end
                        h(end + 1) = plot(ax, t, mean(S.c_hat, 2), '-', 'Color', cA{ia}, 'LineWidth', 1.8, 'DisplayName', ['c_{hat}, ' ANAME{ia}]); %#ok<AGROW>
                    else
                        e = S.a_true - S.a_hat;  m = mean(e, 2);  s = std(e, 0, 2);
                        fill(ax, [t; flipud(t)], [m - s; flipud(m + s)], cA{ia}, 'FaceAlpha', 0.25, 'EdgeColor', 'none', 'HandleVisibility', 'off');
                        h(end + 1) = plot(ax, t, m, '-', 'Color', cA{ia}, 'LineWidth', 1.6, 'DisplayName', ANAME{ia}); %#ok<AGROW>
                    end
                    if fig == 1; local_metrics(S, WALLS{iw}, ANAME{ia}, CASES(ic, :), Td); end
                end
                if fig == 1; ylim(ax, [-0.4 1.0]); else; ylim(ax, [-0.08 0.08]); plot(ax, [0 t(end)], [0 0], ':', 'Color', [0.3 0.3 0.3], 'HandleVisibility', 'off'); end
                yyaxis(ax, 'right');  plot(ax, t, S.wd, '-', 'Color', cH, 'LineWidth', 0.8, 'HandleVisibility', 'off');
                ylim(ax, [0 max(24, 1.05 * CASES(ic, 1))]);  ax.YAxis(2).Color = cH;
                if iw == 2; ylabel(ax, 'h_d', 'FontSize', FS, 'FontWeight', 'bold'); end
                yyaxis(ax, 'left');  ax.YAxis(1).Color = [0 0 0];
                xlim(ax, [0 t(end)]);  set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax, 'off');
                if ic == 1
                    lg = legend(ax, h, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'AutoUpdate', 'off');
                    lg.Title.String = sprintf('%s wall, plane init', WALLS{iw});
                end
                if iw == 1
                    if fig == 1; yl = 'c'; else; yl = 'e_a'; end
                    ylabel(ax, sprintf('%s  (top %.1f R, %g Hz)', yl, CASES(ic, 1), CASES(ic, 2)), 'FontSize', FS - 2, 'FontWeight', 'bold');
                end
                if ic == 4; xlabel(ax, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
            end
        end
        nm = {'fig_conly_wide_fast_c.png', 'fig_conly_wide_fast_ea.png'};
        out = fullfile(fd, nm{fig});  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
    end
end

function n = local_name(wall, arm, cs)
    tg = strrep(sprintf('_rtri_f%g_td0p5', cs(2)), '.', 'p');
    if abs(cs(1) - 2.0) > 1e-9; tg = [tg sprintf('_top%03d', round(100 * cs(1)))]; end
    n = sprintf('f1p2_%s_PI%s%s.mat', wall, arm, tg);
end

function local_metrics(S, wall, aname, cs, Td)
    t = S.t(:);  tri = t > Td;  hd = S.wd(:);  nw = tri & hd < 2.0;
    ce = S.c_eff_d(:);  cm = mean(S.c_hat, 2);  e = S.a_true - S.a_hat;  m = mean(e(tri, :), 1);
    r = corr(cm(tri), ce(tri));  sw = std(cm(tri)) / std(ce(tri));
    fprintf('%-5s top %5.1f %4.1f Hz  %-22s corr %+.2f  swing %.2f  |c-ceff| h<2 %.3f all %.3f | |e_a| %.4f  bias %+.4f  seed-sd %.4f | sqrtP end %.3f  min h %.3f  NaN %d\n', ...
        wall, cs(1), cs(2), strrep(aname, '_{fix}', '_fix'), r, sw, mean(abs(cm(nw) - ce(nw))), mean(abs(cm(tri) - ce(tri))), ...
        mean(abs(e(tri, :)), 'all'), mean(m), std(m), mean(S.sP_c(end, :)), min(S.h_true(:)), nnz(isnan(S.a_hat)));
end
