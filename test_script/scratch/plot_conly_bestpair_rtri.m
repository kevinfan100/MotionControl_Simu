% PURPOSE (2026-10-05, user: "one figure with each wall's own best eps, one with the single P_fix setting for both"): ramp_tri (Td = 1 s),
%   c-only f1p2, plane init, sqrt(P_0) = 0.1, 10 seeds.  which = 'eps': plane column Q55 = 1e-3 R2, cell column Q55 = 3e-3 R2 (each wall's
%   best eps without guard hits, summarize_eps_r2_rtri.m); which = 'pfix': both columns P_fix new 0.15 / revisit 0.05.
%   Rows: a_true (red) and a_hat mean +- sd | e_a mean +- sd | c_hat mean +- sd with c_eff(h_d) (red) | sqrt(P^f_55) mean.
%   Same axis limits in both figures so they can be laid side by side.   STATUS: SCRATCH
function plot_conly_bestpair_rtri(which)
    if nargin < 1; which = 'eps'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_plaw_walls');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    WALLS = {'plane', 'cell'};
    switch which
        case 'eps';  TAG = {'eps3e', 'epsv0p003'};  LAB = {'Q_{55} = \epsilon R_2, \epsilon = 10^{-3} (best for plane)', 'Q_{55} = \epsilon R_2, \epsilon = 3\times10^{-3} (best for cell)'};
        case 'pfix'; TAG = {'rn15_05', 'rn15_05'};  LAB = {'P_{fix}: new 0.15 / revisit 0.05', 'P_{fix}: new 0.15 / revisit 0.05'};
    end
    YL = {[0 1], [-0.04 0.10], [-0.25 0.85], [0 0.25]};
    FS = 17;  LFS = 12;  AXLW = 2.0;  cT = [0.8 0 0];  cE = [0 0.2 0.9];  cB = [0.45 0.55 0.95];
    f = figure('Position', [40 40 1600 1300], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');  ax = gobjects(4, 2);
    for iw = 1:2
        S = load(fullfile(id, sprintf('f1p2_%s_PI_sig010_%s_rtri_f1.mat', WALLS{iw}, TAG{iw})));
        t = S.t(:)';  ea = S.a_true - S.a_hat;  band = @(M) deal(mean(M, 2)', std(M, 0, 2)');
        for r = 1:4; ax(r, iw) = nexttile(tl, (r - 1) * 2 + iw); hold on; box on; end
        [m, s] = band(S.a_hat);  fill(ax(1, iw), [t fliplr(t)], [m + s, fliplr(m - s)], cB, 'EdgeColor', 'none', 'FaceAlpha', 0.35);
        ht = plot(ax(1, iw), t, mean(S.a_true, 2), '-', 'Color', cT, 'LineWidth', 2.4, 'DisplayName', 'a_{true}');
        hh = plot(ax(1, iw), t, m, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'a_{hat} (mean \pm sd)');
        [m, s] = band(ea);  fill(ax(2, iw), [t fliplr(t)], [m + s, fliplr(m - s)], cB, 'EdgeColor', 'none', 'FaceAlpha', 0.35);
        plot(ax(2, iw), t, m, '-', 'Color', cE, 'LineWidth', 1.8);  plot(ax(2, iw), [0 3], [0 0], '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 0.8);
        [m, s] = band(S.c_hat);  fill(ax(3, iw), [t fliplr(t)], [m + s, fliplr(m - s)], cB, 'EdgeColor', 'none', 'FaceAlpha', 0.35);
        ce = S.c_eff_d(:)';  ce(S.wd(:)' < 1) = NaN;  plot(ax(3, iw), t, ce, '-', 'Color', cT, 'LineWidth', 2.0);  plot(ax(3, iw), t, m, '-', 'Color', cE, 'LineWidth', 1.8);
        plot(ax(4, iw), t, mean(S.sP_c, 2), '-', 'Color', cE, 'LineWidth', 1.8);
        for r = 1:4
            ylim(ax(r, iw), YL{r});  plot(ax(r, iw), [1 1], YL{r}, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0);
            xlim(ax(r, iw), [0 3]);  set(ax(r, iw), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax(r, iw), 'off');
        end
        xlabel(ax(4, iw), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        lg = legend(ax(1, iw), [ht hh], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'AutoUpdate', 'off');
        lg.Title.String = sprintf('%s wall, plane init: %s', WALLS{iw}, LAB{iw});
        at = @(tt) mean(ea(find(S.t >= tt - 1e-9, 1), :));
        fprintf('%-5s %-40s troughs %+.3f %+.3f %+.3f | tri mean|e_a| %.4f | sd %.4f | guard %d\n', WALLS{iw}, strrep(LAB{iw}, '\', ''), at(1), at(2), at(3), ...
                mean(mean(abs(ea(S.t >= 1, :)))), mean(std(ea(S.t >= 1, :), 0, 2)), nnz(S.c_hat > 0.95));
    end
    ylabel(ax(1, 1), 'a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax(2, 1), 'e_a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax(3, 1), 'c_{hat}  (red: c_{eff})', 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax(4, 1), '\surd P^f_{55}', 'FontSize', FS, 'FontWeight', 'bold');
    out = fullfile(fd, sprintf('fig_conly_bestpair_rtri_%s.png', which));  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
