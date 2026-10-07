% PURPOSE (2026-10-05, user: "explain I on a figure, and what setting Q this way means"): ramp_tri, c-only, plane init, sqrt(P_0) 0.1,
%   sqrt(P_fix) 0.15 (Q55 = P_fix^2 I), 10 seeds, columns plane | cell.  Rows on one time axis:
%   (1) commanded height h_d  (2) I[k] (log)  (3) c_hat (mean) with c_eff(h_d) red  (4) Q55[k] added (log).
%   Reads f1p2_<wall>_PI_sig010_rho15_rtri_f1.mat (q55a_stat = I[k], q55a_q = Q55).   STATUS: SCRATCH
function plot_conly_I_meaning()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_plaw_walls');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    WALLS = {'plane', 'cell'};  FS = 17;  AXLW = 2.0;  cE = [0 0.2 0.9];  cT = [0.8 0 0];
    f = figure('Position', [40 40 1600 1300], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');  ax = gobjects(4, 2);
    for iw = 1:2
        S = load(fullfile(id, sprintf('f1p2_%s_PI_sig010_rho15_rtri_f1.mat', WALLS{iw})));  t = S.t;
        for r = 1:4; ax(r, iw) = nexttile(tl, (r - 1) * 2 + iw); hold on; box on; end
        hd = S.wd;  hd(hd < 1) = NaN;  plot(ax(1, iw), t, hd, 'k-', 'LineWidth', 2.0);  ylim(ax(1, iw), [0 23]);  yticks(ax(1, iw), [1.1 5 10 15 22.2]);
        plot(ax(2, iw), t, mean(S.q55a_stat, 2), '-', 'Color', cE, 'LineWidth', 1.6);  set(ax(2, iw), 'YScale', 'log');  ylim(ax(2, iw), [1e-4 3]);
        ce = S.c_eff_d(:);  ce(S.wd(:) < 1) = NaN;
        plot(ax(3, iw), t, ce, '-', 'Color', cT, 'LineWidth', 2.0);  plot(ax(3, iw), t, mean(S.c_hat, 2), '-', 'Color', cE, 'LineWidth', 1.8);  ylim(ax(3, iw), [-0.2 0.8]);
        plot(ax(4, iw), t, max(mean(S.q55a_q, 2), 1e-9), '-', 'Color', cE, 'LineWidth', 1.6);  set(ax(4, iw), 'YScale', 'log');  ylim(ax(4, iw), [1e-8 3e-3]);
        for r = 1:4
            yl = ylim(ax(r, iw));  plot(ax(r, iw), [1 1], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0);  ylim(ax(r, iw), yl);
            xlim(ax(r, iw), [0 3]);  set(ax(r, iw), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax(r, iw), 'off');
        end
        title(ax(1, iw), '');  lg = legend(ax(1, iw), sprintf('h_d  (%s wall, plane init, \\surdP_{fix} = 0.15)', WALLS{iw}), 'Location', 'northoutside', 'FontSize', FS - 5, 'FontWeight', 'bold', 'AutoUpdate', 'off');
        legend(ax(3, iw), {'c_{eff}(h_d)', 'c_{hat}'}, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 5, 'FontWeight', 'bold', 'AutoUpdate', 'off');
        xlabel(ax(4, iw), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        mI = mean(S.q55a_stat, 2);  mc = mean(S.c_hat, 2);  mq = mean(S.q55a_q, 2);
        for tt = [0.5 0.9 0.95 1.0 1.5 2.0]
            k = find(t >= tt, 1);  fprintf('%-5s t %.2f  h_d %5.2f  I %.3g  c_hat %.3f  Q55 %.2g\n', WALLS{iw}, tt, S.wd(k), mI(k), mc(k), mq(min(k + 1, end)));
        end
        fprintf('%-5s c_hat moves: 0-0.9 s %+.3f, 0.9-1.0 s %+.3f, 1-3 s %+.3f\n', WALLS{iw}, mc(find(t >= 0.9, 1)) - mc(1), mc(find(t >= 1, 1)) - mc(find(t >= 0.9, 1)), mc(end) - mc(find(t >= 1, 1)));
    end
    ylabel(ax(1, 1), 'h_d', 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax(2, 1), 'I[k]', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax(3, 1), 'c', 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax(4, 1), 'q_{55}[k]', 'FontSize', FS, 'FontWeight', 'bold');
    out = fullfile(fd, 'fig_conly_I_meaning.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
