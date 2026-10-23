% PURPOSE (2026-09-27, user: "the speed part, and a plain map from O to the figure"): first-order gain model, lambda_f = 1,
%   plane, 5 seeds, T_d = 5 s and 15 s. The weight each step's readings add to the a estimate = the per-step increase of 1/P44.
%   y2's share is the hold slope (only y2 acts in a hold): 1/R2 = 0.26 per step. Everything above it during the descent is y1.
%   Left : weight added per STEP against h_d.  Right: weight added per unit HEIGHT (per R travelled), split y2 / y1.
%   Runs: test_results/temp_5state_noap/ladder_ramp16_Td{5,15}_lf1_o1.mat    STATUS: SCRATCH
function plot_5state_noap_weight_per_height()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    TD = [5 15];  COLS = {[0 0.2 0.9], [0.1 0.6 0.1]};  W2 = 0.26;  FS = 18;  LFS = 12;  AXLW = 2.0;
    f = figure('Position', [40 40 1700 620], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    axL = nexttile(tl, 1); hold(axL, 'on');  axR = nexttile(tl, 2); hold(axR, 'on');
    hL = gobjects(1, 3);  hR = gobjects(1, 4);
    fprintf('%-6s %-10s | per step: total  y2  y1 | per R: y2  y1\n', 'T_d', 'h window');
    for c = 1:2
        r = load(fullfile(id, sprintf('ladder_ramp16_Td%d_lf1_o1.mat', TD(c))));
        t = r.t(:);  Ts = t(2) - t(1);  hd = r.hd(:);  iP = mean(1 ./ r.P_a, 2);  seg = r.seg;
        nw = round(0.05 / Ts);  inc = [0; diff(movmean(iP, nw))];
        m = t > seg(1) + 0.05 & t < seg(2) - 0.02;                       % descent only
        steps_per_R = (TD(c) / Ts) / (hd(1) - hd(end));                  % constant velocity
        y1 = max(inc - W2, 0);
        hL(c) = plot(axL, hd(m), inc(m), '-', 'Color', COLS{c}, 'LineWidth', 2.0, 'DisplayName', sprintf('T_d = %g s', TD(c)));
        hR(2*c-1) = plot(axR, hd(m), W2 * steps_per_R * ones(nnz(m), 1), '--', 'Color', COLS{c}, 'LineWidth', 2.0, 'DisplayName', sprintf('y_2, T_d = %g s', TD(c)));
        hR(2*c)   = plot(axR, hd(m), y1(m) * steps_per_R, '-', 'Color', COLS{c}, 'LineWidth', 2.0, 'DisplayName', sprintf('y_1, T_d = %g s', TD(c)));
        for wnd = {[10 22.2], [3 10], [1.5 3], [1.1 1.5]}
            q = m & hd >= wnd{1}(1) & hd < wnd{1}(2);
            fprintf('%-6g %4.1f-%4.1f R | %6.2f  %.2f  %5.2f | %6.0f  %6.0f\n', TD(c), wnd{1}(1), wnd{1}(2), mean(inc(q)), W2, mean(y1(q)), W2 * steps_per_R, mean(y1(q)) * steps_per_R);
        end
    end
    hL(3) = plot(axL, [1.05 23], [W2 W2], 'r:', 'LineWidth', 1.6, 'DisplayName', 'y_2 alone = 1/R_2 = 0.26');
    for ax = [axL axR]
        set(ax, 'XScale', 'log', 'XDir', 'reverse', 'XLim', [1.05 23], 'XTick', [1.1 1.5 2 3 5 10 22.2], ...
            'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
        xlabel(ax, 'h_d  [R]  (descent runs left to right)', 'FontSize', FS, 'FontWeight', 'bold');
    end
    ylabel(axL, 'added to 1/P_{44} per step', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(axR, 'added to 1/P_{44} per R travelled', 'FontSize', FS, 'FontWeight', 'bold');
    legend(axL, hL, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'AutoUpdate', 'off');
    legend(axR, hR, 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'AutoUpdate', 'off');
    out = fullfile(fd, 'fig_noap_weight_per_height.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
