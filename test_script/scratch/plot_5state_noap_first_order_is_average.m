% PURPOSE (2026-09-27, user: "explain what comes after O, plainly"): first-order gain model, lambda_f = 1, plane, 5 seeds.
%   Shows that the filter's a_hat is (almost) the plain running average of all past truth values, with the prior counted as
%   N0 = (1/P44[0]) / (1/R2) readings of the seed value. 1/R2 = 0.26 per step is read off the hold slope of 1/P44 (y2 only).
%   Panels: T_d = 5 s | T_d = 15 s.  Lines: a_true (red), running average of a_true (black dashed), filter a_hat (blue).
%   Runs: test_results/temp_5state_noap/ladder_ramp16_Td{5,15}_lf1_o1.mat    STATUS: SCRATCH
function plot_5state_noap_first_order_is_average()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    TD = [5 15];  COL_TRUE = [0.8 0 0];  C2 = [0 0.2 0.9];  FS = 18;  LFS = 12;  AXLW = 2.0;
    f = figure('Position', [40 40 1700 560], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for c = 1:2
        r = load(fullfile(id, sprintf('ladder_ramp16_Td%d_lf1_o1.mat', TD(c))));
        t = r.t(:);  at = mean(r.a_true, 2);  ah = mean(r.a_hat, 2);  seg = r.seg;
        N0 = (1 / r.P_a(1, 1)) / 0.26;                                   % prior as N0 readings of the seed value
        avg = (N0 * ah(1) + cumsum(at)) ./ (N0 + (1:numel(at)).');
        ax = nexttile(tl, c);  hold(ax, 'on');
        h0 = plot(ax, nan, nan, 'w-', 'DisplayName', sprintf('first order, \\lambda_f = 1, T_d = %g s', TD(c)));
        ht = plot(ax, t, at, '-', 'Color', COL_TRUE, 'LineWidth', 2.4, 'DisplayName', 'a_{true} / a_{nom}');
        hv = plot(ax, t, avg, '--', 'Color', 'k', 'LineWidth', 2.0, 'DisplayName', 'running average of a_{true}');
        he = plot(ax, t, ah, '-', 'Color', C2, 'LineWidth', 1.8, 'DisplayName', 'a_{hat} (filter)');
        ylim(ax, [0 1.1]);  xlim(ax, [0 t(end)]);
        for s = seg(:).'; plot(ax, [s s], [0 1.1], '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2); end
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
        xlabel(ax, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        if c == 1; ylabel(ax, 'a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend(ax, [h0 ht hv he], 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'AutoUpdate', 'off');
        i_de = find(t <= seg(2), 1, 'last');
        fprintf('T_d %2d s | N0 = %.0f readings | descent end: true %.3f  average %.3f  filter %.3f | end: true %.3f  average %.3f  filter %.3f\n', ...
            TD(c), N0, at(i_de), avg(i_de), ah(i_de), at(end), avg(end), ah(end));
    end
    out = fullfile(fd, 'fig_noap_first_order_is_average.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
