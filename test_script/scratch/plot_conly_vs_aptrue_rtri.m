% PURPOSE (2026-10-04, user: "go back to the phenomenon and separate it"): ramp_tri, plane | cell, 10 seeds.
%   c-only ratio sqrt(P_fix) = 0.15 (slope (1 - c_hat)/h^2 learned) vs the same estimator blocks fed the TRUE slope a'_true at the
%   estimator height (ap_known 'est', slot 5 locked): the difference is what learning the slope costs.
%   Rows: a_true (red) with a_hat means | e_a mean +- sd over seeds.  Reads f1p2_<wall>_PI_sig010_rho15_rtri_f1.mat and aptrue_<wall>_rtri_f1.mat
%   (runner: job tmp run_aptrue_rtri.m).   STATUS: SCRATCH
function plot_conly_vs_aptrue_rtri()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_plaw_walls');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    WALLS = {'plane', 'cell'};
    ARMS = {'f1p2_%s_PI_sig010_rho15_rtri_f1.mat', 'learned slope: c-only ratio, \surdP_{fix} = 0.15', [0 0.2 0.9]; ...
            'aptrue_%s_rtri_f1.mat', 'true slope a''_{true} (diagnostic)', [0 0.6 0.3]};
    FS = 18;  LFS = 12;  AXLW = 2.0;  cT = [0.8 0 0];
    f = figure('Position', [40 40 1600 900], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(2, numel(WALLS), 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = gobjects(2, numel(WALLS));
    for iw = 1:numel(WALLS)
        ax(1, iw) = nexttile(tl, iw);  hold on; box on;  ax(2, iw) = nexttile(tl, numel(WALLS) + iw);  hold on; box on;
        hl = gobjects(0);
        for m = 1:size(ARMS, 1)
            S = load(fullfile(id, sprintf(ARMS{m, 1}, WALLS{iw})));  t = S.t(:)';  ea = S.a_true - S.a_hat;  me = mean(ea, 2)';  se = std(ea, 0, 2)';
            if m == 1; hl(end + 1) = plot(ax(1, iw), t, mean(S.a_true, 2), '-', 'Color', cT, 'LineWidth', 2.4, 'DisplayName', 'a_{true}'); end %#ok<AGROW>
            hl(end + 1) = plot(ax(1, iw), t, mean(S.a_hat, 2), '-', 'Color', ARMS{m, 3}, 'LineWidth', 1.6, 'DisplayName', ARMS{m, 2}); %#ok<AGROW>
            fill(ax(2, iw), [t fliplr(t)], [me + se, fliplr(me - se)], ARMS{m, 3}, 'EdgeColor', 'none', 'FaceAlpha', 0.18);
            plot(ax(2, iw), t, me, '-', 'Color', ARMS{m, 3}, 'LineWidth', 1.6);
        end
        plot(ax(2, iw), [0 3], [0 0], '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 0.8);
        for r = 1:2
            yl = ylim(ax(r, iw));  plot(ax(r, iw), [1 1], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0);  ylim(ax(r, iw), yl);
            xlim(ax(r, iw), [0 3]);  set(ax(r, iw), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax(r, iw), 'off');
        end
        xlabel(ax(2, iw), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        lg = legend(ax(1, iw), hl, 'Location', 'northoutside', 'Orientation', 'vertical', 'FontSize', LFS, 'FontWeight', 'bold', 'AutoUpdate', 'off');
        lg.Title.String = sprintf('%s wall, plane init', WALLS{iw});
    end
    ylabel(ax(1, 1), 'a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax(2, 1), 'e_a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
    linkaxes(ax(2, :), 'y');
    out = fullfile(fd, 'fig_conly_vs_aptrue_rtri.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
