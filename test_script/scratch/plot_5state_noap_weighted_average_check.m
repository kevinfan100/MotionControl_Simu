% PURPOSE (2026-09-27, user: "from O, show the a estimate is an average and why speed matters; derivation must match the figure"):
%   first-order gain model, lambda_f = 1, plane, 5 seeds, T_d = 5 s and 15 s.
%   Prediction (0927_kf_obs_stepwise.tex Part 5): a_hat[k] = (W0 a_hat[0] + sum_j w_j a_x[j]) / (W0 + sum_j w_j),
%   w_j = per-step increase of 1/P44 (= c_j^2/R_j summed over y1, y2), W0 = 1/P44[0]; no fitting, weights read from the filter's own P.
%   Compared with: the filter's a_hat, and the equal-weight (y2-only) average.
%   Rows: a_true / a_hat | e_a = a_true - a_hat.  Columns: T_d = 5 s | 15 s.
%   Runs: test_results/temp_5state_noap/ladder_ramp16_Td{5,15}_lf1_o1.mat    STATUS: SCRATCH
function plot_5state_noap_weighted_average_check()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    TD = [5 15];  COL_TRUE = [0.8 0 0];  C2 = [0 0.2 0.9];  CG = [0.1 0.6 0.1];  FS = 18;  LFS = 12;  AXLW = 2.0;  W2 = 0.26;
    f = figure('Position', [40 40 1700 950], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    fprintf('%-5s | %-28s | %-26s | %-26s\n', 'T_d', 'descent end: true filter wavg eqavg', 'e_a near: filter wavg eqavg', 'hold end: filter wavg eqavg');
    for c = 1:2
        r = load(fullfile(id, sprintf('ladder_ramp16_Td%d_lf1_o1.mat', TD(c))));
        t = r.t(:);  seg = r.seg;  hd = r.hd(:);
        nS = size(r.a_hat, 2);  wav = zeros(size(r.a_hat));  eqa = wav;
        for q = 1:nS                                                          % per seed, then average
            iP = 1 ./ r.P_a(:, q);  w = [0; diff(iP)];  w = max(w, 0);  W0 = iP(1);
            at = r.a_true(:, q);  a0 = r.a_hat(1, q);
            wav(:, q) = (W0 * a0 + cumsum(w .* at)) ./ (W0 + cumsum(w));
            eqa(:, q) = (W0 * a0 + cumsum(W2 * at)) ./ (W0 + W2 * (1:numel(at)).');
        end
        at = mean(r.a_true, 2);  ah = mean(r.a_hat, 2);  wv = mean(wav, 2);  eq = mean(eqa, 2);
        i_de = find(t <= seg(2), 1, 'last');  m_near = hd < 3 & t > seg(1) & t <= seg(2);
        fprintf('%-5g | %.3f %.3f %.3f %.3f        | %+.3f %+.3f %+.3f         | %+.3f %+.3f %+.3f\n', TD(c), at(i_de), ah(i_de), wv(i_de), eq(i_de), ...
            mean(at(m_near) - ah(m_near)), mean(at(m_near) - wv(m_near)), mean(at(m_near) - eq(m_near)), at(end) - ah(end), at(end) - wv(end), at(end) - eq(end));
        ax1 = nexttile(tl, c);  hold(ax1, 'on');
        h0 = plot(ax1, nan, nan, 'w-', 'DisplayName', sprintf('first order, \\lambda_f = 1, T_d = %g s', TD(c)));
        ht = plot(ax1, t, at, '-', 'Color', COL_TRUE, 'LineWidth', 2.4, 'DisplayName', 'a_{true} / a_{nom}');
        he = plot(ax1, t, eq, ':', 'Color', 'k', 'LineWidth', 2.0, 'DisplayName', 'equal-weight average (y_2 only)');
        hw = plot(ax1, t, wv, '--', 'Color', CG, 'LineWidth', 2.2, 'DisplayName', 'weighted average, w = \Delta(1/P_{44})');
        hf = plot(ax1, t, ah, '-', 'Color', C2, 'LineWidth', 1.6, 'DisplayName', 'a_{hat} (filter)');
        ylim(ax1, [0 1.1]);
        legend(ax1, [h0 ht he hw hf], 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'AutoUpdate', 'off');
        ax2 = nexttile(tl, 2 + c);  hold(ax2, 'on');
        plot(ax2, t, at - eq, ':', 'Color', 'k', 'LineWidth', 2.0);
        plot(ax2, t, at - wv, '--', 'Color', CG, 'LineWidth', 2.2);
        plot(ax2, t, at - ah, '-', 'Color', C2, 'LineWidth', 1.6);
        plot(ax2, [0 t(end)], [0 0], '-', 'Color', [0.6 0.6 0.6]);  ylim(ax2, [-0.9 0.1]);
        xlabel(ax2, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        for ax = [ax1 ax2]
            xlim(ax, [0 t(end)]);  yl = ylim(ax);
            for s = seg(:).'; plot(ax, [s s], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2); end
            set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
        end
        if c == 1
            ylabel(ax1, 'a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax2, 'e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold');
        end
    end
    out = fullfile(fd, 'fig_noap_weighted_average_check.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
