% PURPOSE (2026-10-04, user: "is the green drift in fig_conly_vs_aptrue_rtri the missing E[e x e]?"): the true-slope arm
%   (ap_known 'est', slot 5 locked, c-only pure form) on ramp_tri with the missing blocks switched on one at a time:
%   A as run (law_exact_step / pred_mean2 / nw_mcorr off) | B + law_exact_step (Euler -> exact step) | C + pred_mean2 (+_e4)
%   (second-order mean, E[e x e]) | D + nw_mcorr (correlated noise). Rows: e_a mean over 10 seeds (plane | cell), band = seed sd of A.
%   Reads test_results/temp_plaw_walls/aptrue{,B,C,D}_<wall>_rtri_f1.mat (runner: job tmp run_aptrue_ladder.m).
%   Output reference/eq17_analysis/derivation/figures/aptrue_ladder_rtri.png   STATUS: SCRATCH | EXPIRES: with the c-only line
function plot_aptrue_ladder_rtri()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    rd = fullfile(root, 'test_results', 'temp_plaw_walls');  od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    WALLS = {'plane', 'cell'};
    ARMS = {'', 'A  pure form (as in the figure)', [0 0.6 0.3]; 'B', 'B  + law\_exact\_step (Euler -> exact step)', [0.85 0.45 0]; ...
            'C', 'C  + pred\_mean2 (E[e x e] terms)', [0 0.2 0.9]; 'D', 'D  + nw\_mcorr (correlated noise)', [0.5 0 0.6]};
    FS = 16;  LFS = 12;  AXLW = 2.0;
    f = figure('Position', [40 40 1500 560], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');  ax = gobjects(1, 2);
    for iw = 1:2
        ax(iw) = nexttile(tl, iw);  hold on;  box on;  hl = gobjects(0);
        for a = 1:size(ARMS, 1)
            r = load(fullfile(rd, sprintf('aptrue%s_%s_rtri_f1.mat', ARMS{a, 1}, WALLS{iw})));  t = r.t(:)';  e = r.a_true - r.a_hat;
            me = mean(e, 2)';
            if a == 1; se = std(e, 0, 2)'; fill([t fliplr(t)], [me + se, fliplr(me - se)], ARMS{a, 3}, 'EdgeColor', 'none', 'FaceAlpha', 0.12); end
            hl(end + 1) = plot(t, me, '-', 'Color', ARMS{a, 3}, 'LineWidth', 1.8, 'DisplayName', sprintf('%s   e_a(3 s) %+.4f', ARMS{a, 2}, me(end))); %#ok<AGROW>
        end
        plot([0 3], [0 0], '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 0.8);  plot([1 1], [-0.02 0.01], '--', 'Color', [0.5 0.5 0.5]);
        xlim([0 3]);  ylim([-0.02 0.01]);  xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        if iw == 1; ylabel('e_a / a_{nom}  (true slope, mean of 10 seeds)', 'FontSize', FS, 'FontWeight', 'bold'); end
        lg = legend(hl, 'Location', 'northoutside', 'Orientation', 'vertical', 'FontSize', LFS, 'FontWeight', 'bold');
        lg.Title.String = sprintf('%s wall, true slope a''_{true} at the estimator height, ramp\\_tri 1 Hz', upper(WALLS{iw}));
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid off;
    end
    out = fullfile(od, 'aptrue_ladder_rtri.png');  exportgraphics(f, out, 'Resolution', 150);  close(f);  fprintf('figure: %s\n', out);
end
