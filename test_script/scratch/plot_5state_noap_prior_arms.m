% PURPOSE (2026-09-28, user: "fixed points converge in 0.2 s, ramps do not -- contradiction?"): first order, lambda_f = 1,
%   cell wall, held at 1.10 R (a_true/a_nom = 0.50), seed a_nom, 5 seeds, 3 s. Only the initial confidence 1/P44[0] changes:
%   Pf_a_std_rel = 0.5 / 0.05 / 0.0143  ->  1/P44[0] = 4 / 400 / 4904 (the last = what the filter holds when a 5 s ramp reaches the wall).
%   Rows: a_true / a_hat | fraction of the initial error still left, e_a(t)/e_a(0), with the averaging prediction
%   (1/P44[0]) / (1/P44[t]) (dashed; 1/P44[t] read from the same run).  Runs: ladder_fixed_h1p10_cell_pa{0500,0050,0143}_o1.mat
%   STATUS: SCRATCH
function plot_5state_noap_prior_arms()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    arms = {'_pa0500', '1/P_{44}[0] = 4',    [0 0.2 0.9]; ...
            '_pa0050', '1/P_{44}[0] = 400',  [0.1 0.6 0.1]; ...
            '_pa0143', '1/P_{44}[0] = 4904', [0.85 0.45 0]};
    COL_TRUE = [0.8 0 0];  FS = 18;  LFS = 12;  AXLW = 2.0;
    f = figure('Position', [40 40 1150 950], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax1 = nexttile(tl, 1);  hold(ax1, 'on');  ax2 = nexttile(tl, 2);  hold(ax2, 'on');
    hh = gobjects(1, 3);  hp = gobjects(1, 1);
    fprintf('\n%-20s | left at 0.2 s (sim / pred) | left at 0.5 s | left at 3 s | a_hat at 0.2 / 0.5 / 3 s\n', 'arm');
    for k = 1:3
        r = load(fullfile(id, sprintf('ladder_fixed_h1p10_cell%s_o1.mat', arms{k, 1})));
        t = r.t(:);  at = mean(r.a_true, 2);  ah = mean(r.a_hat, 2);
        e0 = at(1) - ah(1);  left = (at - ah) / e0;
        pred = mean((1 ./ r.P_a(1, :)) ./ (1 ./ r.P_a), 2);
        if k == 1
            h0 = plot(ax1, nan, nan, 'w-', 'DisplayName', 'first order, \lambda_f = 1, cell wall, held at 1.10 R, seed a_{nom}');
            ht = plot(ax1, t, at, '-', 'Color', COL_TRUE, 'LineWidth', 2.4, 'DisplayName', 'a_{true} / a_{nom}');
        end
        hh(k) = plot(ax1, t, ah, '-', 'Color', arms{k, 3}, 'LineWidth', 2.0, 'DisplayName', ['a_{hat}, ' arms{k, 2}]);
        plot(ax2, t, left, '-', 'Color', arms{k, 3}, 'LineWidth', 2.0);
        hq = plot(ax2, t, pred, '--', 'Color', 'k', 'LineWidth', 1.4, 'DisplayName', 'prediction (1/P_{44}[0]) / (1/P_{44}[t])');
        if k == 1; hp = hq; end
        i02 = find(t >= 0.2, 1);  i05 = find(t >= 0.5, 1);
        fprintf('%-20s | %.3f / %.3f              | %.3f         | %.3f       | %.3f / %.3f / %.3f\n', arms{k, 2}, left(i02), pred(i02), left(i05), left(end), ah(i02), ah(i05), ah(end));
    end
    for ax = [ax1 ax2]
        xlim(ax, [0 t(end)]);  set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
    end
    ylim(ax1, [0 1.1]);  ylim(ax2, [-0.1 1.1]);
    plot(ax2, [0 t(end)], [0 0], '-', 'Color', [0.6 0.6 0.6]);
    ylabel(ax1, 'a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax2, 'e_a(t) / e_a(0)', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(ax2, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    legend(ax1, [h0 ht hh], 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'AutoUpdate', 'off');
    legend(ax2, hp, 'Location', 'northoutside', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'AutoUpdate', 'off');
    out = fullfile(fd, 'fig_noap_prior_arms_fixed_cell.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
