% PURPOSE (2026-09-24, user: "one figure: our lambda_f lag vs the current a'(h, c)"): same ramp (hold 0.5 s, 22.2 -> 1.10 R
%   constant velocity over T_d = 2 s, hold 0.5 s), plane plant, paired seeds 1:5.
%   Arms: Layer-1 (no a', second order, Ljung lambda_f schedule)          test_results/temp_5state_noap/ladder_ramp16_Td2_o2.mat
%         a'(h, c) law (1-c)/h^2, c state, plane init, lambda = 1        test_results/temp_plaw_walls/f1p2_plane_PI_sig010_ramp2.mat
%         a'(h, c) law + information forgetting beta = 3 (ap model)      test_results/temp_plaw_walls/f1p2_plane_PI_sig010_ib3_ramp2.mat
%   Rows: a_true / a_hat | e_a = a_true - a_hat (mean, +-sigma_seed band) | sqrt(P44) claimed vs sigma_seed.  STATUS: SCRATCH
function plot_5state_noap_vs_aprime()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    SEEDS = 1:5;
    A = {fullfile(root, 'test_results', 'temp_5state_noap', 'ladder_ramp16_Td2_o2.mat'),      'Layer-1, no a'', \lambda_f schedule', [0 0.2 0.9]; ...
         fullfile(root, 'test_results', 'temp_plaw_walls', 'f1p2_plane_PI_sig010_ramp2.mat'),     'a''(h, c) law, \lambda_f = 1',        [0.85 0.45 0]; ...
         fullfile(root, 'test_results', 'temp_plaw_walls', 'f1p2_plane_PI_sig010_ib3_ramp2.mat'), 'a''(h, c) law, info forgetting \beta = 3', [0.1 0.6 0.1]};
    COL_TRUE = [0.8 0 0]; FS = 18; LFS = 12; AXLW = 2.0;
    f = figure('Position', [40 40 1100 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = gobjects(1, 3);  for r = 1:3; ax(r) = nexttile(tl, r); hold(ax(r), 'on'); end
    h1 = gobjects(1, 3); h2 = gobjects(1, 3); h3 = gobjects(1, 3);
    fprintf('%-42s | near (h<3R) mean  sd | hold-end mean  sd | far (h>10R) sd | honesty near / end\n', 'arm');
    for k = 1:3
        r = load(A{k, 1});  t = r.t(:);  q = SEEDS;
        if k == 1; hb = r.hb(:, q); sP = sqrt(r.P_a(:, q)); else; hb = r.h_true(:, q); sP = r.sP_a(:, q); end
        at = r.a_true(:, q);  ah = r.a_hat(:, q);  ea = at - ah;  seg = r.seg;
        m = mean(ea, 2); s = std(ea, 0, 2); sp = mean(sP, 2); hm = mean(hb, 2);
        if k == 1; ht = plot(ax(1), t, mean(at, 2), '-', 'Color', COL_TRUE, 'LineWidth', 2.4, 'DisplayName', 'a_{true} / a_{nom}'); end
        h1(k) = plot(ax(1), t, mean(ah, 2), '-', 'Color', A{k, 3}, 'LineWidth', 1.8, 'DisplayName', A{k, 2});
        fill(ax(2), [t; flipud(t)], [m + s; flipud(m - s)], A{k, 3}, 'FaceAlpha', 0.15, 'EdgeColor', 'none');
        h2(k) = plot(ax(2), t, m, '-', 'Color', A{k, 3}, 'LineWidth', 1.8, 'DisplayName', A{k, 2});
        h3(k) = plot(ax(3), t, sp, '-', 'Color', A{k, 3}, 'LineWidth', 1.8, 'DisplayName', ['sqrt(P_{44}), ' A{k, 2}]);
        plot(ax(3), t, s, ':', 'Color', A{k, 3}, 'LineWidth', 1.6);
        m_near = hm < 3 & t > seg(1) & t <= seg(2);  m_end = t > seg(2);  m_far = t > seg(1) & hm > 10;
        fprintf('%-42s | %+.3f  %.3f | %+.3f  %.3f | %.3f | %.2f / %.2f\n', strrep(A{k, 2}, '\', ''), mean(m(m_near)), mean(s(m_near)), ...
            mean(m(m_end)), mean(s(m_end)), mean(s(m_far)), mean(s(m_near)) / mean(sp(m_near)), mean(s(m_end)) / mean(sp(m_end)));
    end
    seg = r.seg;
    for rr = 1:3
        xlim(ax(rr), [0 t(end)]);  yl = ylim(ax(rr));
        for sgm = seg(:).'; plot(ax(rr), [sgm sgm], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2); end
        set(ax(rr), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
    end
    plot(ax(2), [0 t(end)], [0 0], '-', 'Color', [0.6 0.6 0.6]);
    ylim(ax(1), [0 1.1]);
    ylabel(ax(1), 'a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax(2), 'e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax(3), '\sigma  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold');  set(ax(3), 'YScale', 'log', 'YLim', [1e-3 1], 'YTick', [1e-3 1e-2 1e-1 1]);
    xlabel(ax(3), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    legend(ax(1), [ht h1], 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    legend(ax(2), h2, 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 3, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    legend(ax(3), h3, 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 3, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    out = fullfile(fd, 'fig_noap_vs_aprime_Td2.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
