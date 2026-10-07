% STATUS: SCRATCH -- 2026-10-06, plots run_conly_rtri_drift.m: ramp_tri (T_d 1, 1 Hz), plane | cell, 10 seeds, three Q55 settings:
%   single sqrt(P_fix) 0.15 (Q55 = P_fix^2 I), 'drift' dc 0.76 (Q55 = dc^2 |dw_d|/h^2, O50), 'driftsq' dc 0.76 (Q55 = (dc dw_d/h^2)^2).
%   Rows: (1) e_abar seed mean +- seed sd  (2) c_hat seed mean with c_eff(h_d) red  (3) sqrt(p55) seed mean (log).
%   Console: point criteria (descent end 1.0 s, troughs 2 / 3 s, tops 1.5 / 2.5 s), triangle mean |e_abar|, c_hat range, sqrt(P) end.
function plot_conly_rtri_drift()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_plaw_walls');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    WALLS = {'plane', 'cell'};  TAG = {'_sig010_rho15', '_sig010_dr76', '_sig010_dq76'};
    NAME = {'P_{fix}^2 I, \surdP_{fix} 0.15', 'drift: dc^2|\Deltaw_d|/h^2', 'driftsq: (dc \Deltaw_d/h^2)^2'};
    C = {[0 0.2 0.9], [0.9 0.5 0], [0.3 0.65 0.3]};  cT = [0.8 0 0];  FS = 16;  LFS = 11;  AXLW = 1.8;
    f = figure('Position', [40 40 1600 1250], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for iw = 1:2
        ax = gobjects(3, 1);  for r = 1:3; ax(r) = nexttile(tl, (r - 1) * 2 + iw); hold(ax(r), 'on'); box(ax(r), 'on'); end
        h = gobjects(0);
        for ia = 1:3
            S = load(fullfile(id, sprintf('f1p2_%s_PI%s_rtri_f1.mat', WALLS{iw}, TAG{ia})));  t = S.t(:);
            e = S.a_true - S.a_hat;  m = mean(e, 2);  s = std(e, 0, 2);
            fill(ax(1), [t; flipud(t)], [m - s; flipud(m + s)], C{ia}, 'FaceAlpha', 0.18, 'EdgeColor', 'none', 'HandleVisibility', 'off');
            h(end + 1) = plot(ax(1), t, m, '-', 'Color', C{ia}, 'LineWidth', 1.6, 'DisplayName', NAME{ia}); %#ok<AGROW>
            if ia == 1; ce = S.c_eff_d(:); ce(S.wd(:) < 1) = NaN; plot(ax(2), t, ce, '-', 'Color', cT, 'LineWidth', 1.4); end
            plot(ax(2), t, mean(S.c_hat, 2), '-', 'Color', C{ia}, 'LineWidth', 1.8);
            plot(ax(3), t, mean(S.sP_c, 2), '-', 'Color', C{ia}, 'LineWidth', 1.6);
            k = @(tt) find(t >= tt - 1e-9, 1);  pts = [1.0 1.5 2.0 2.5 3.0];  v = arrayfun(@(tt) m(min(k(tt), numel(t))), pts);
            tri = t > 1.0;  ch = S.c_hat(tri, :);
            fprintf('%-5s %-32s e_a @1.0 %+.3f top1.5 %+.3f trough2 %+.3f top2.5 %+.3f trough3 %+.3f | descent max|mean| %.3f | tri |e_a| %.4f seed-sd %.4f | c_hat [%+.2f %+.2f] | sqrtP end %.3f max %.3f | min h %.3f NaN %d\n', ...
                WALLS{iw}, strrep(strrep(NAME{ia}, '\surd', 'sqrt'), '\Delta', 'd'), v, max(abs(m(t > 0.5 & t <= 1.05))), mean(abs(e(tri, :)), 'all'), std(mean(e(tri, :), 1)), ...
                min(ch(:)), max(ch(:)), mean(S.sP_c(end, :)), max(S.sP_c(:)), min(S.h_true(:)), nnz(isnan(S.a_hat)));
        end
        plot(ax(1), [0 3], [0 0], ':', 'Color', [0.3 0.3 0.3], 'HandleVisibility', 'off');
        ylim(ax(1), [-0.15 0.15]);  ylim(ax(2), [-0.6 1.2]);  set(ax(3), 'YScale', 'log');  ylim(ax(3), [3e-3 1.5]);
        for r = 1:3
            yl = ylim(ax(r));  plot(ax(r), [1 1], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off');  ylim(ax(r), yl);
            xlim(ax(r), [0 3]);  set(ax(r), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax(r), 'off');
        end
        lg = legend(ax(1), h, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'AutoUpdate', 'off');
        lg.Title.String = sprintf('%s wall, plane init (red in row 2 = c_{eff}(h_d))', WALLS{iw});
        xlabel(ax(3), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        if iw == 1
            ylabel(ax(1), 'e_a  (true - est)', 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax(2), 'c', 'FontSize', FS, 'FontWeight', 'bold');
            ylabel(ax(3), '\surdp_{55}', 'FontSize', FS, 'FontWeight', 'bold');
        end
    end
    out = fullfile(fd, 'fig_conly_rtri_drift.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
