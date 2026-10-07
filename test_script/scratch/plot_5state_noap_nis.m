% PURPOSE (2026-09-24): does the residual carry the near-wall lag? Windowed NIS_i = mean(nu_i^2 / S_i) over 0.05 s,
%   seed-averaged, for three Layer-1 arms on the same ramp (22.2 -> 1.10 R over 5 s, second order):
%   lambda_f = 1 | constant 0.995 | Ljung schedule.  Rows: e_a | NIS_1 (y1) | NIS_2 (y2).  Reads ladder_*.mat only.
%   Meng (6.11) lambda = exp(-alpha nu^2) has a signal exactly where NIS rises above 1.   STATUS: SCRATCH
function plot_5state_noap_nis()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    tags = {'_Td5_lf1', '_Td5_c09950', '_Td5'};  names = {'\lambda_f = 1', 'constant \lambda_f = 0.995', '\lambda_f schedule'};
    COLS = {[0.85 0.45 0], [0.5 0.5 0.5], [0 0.2 0.9]};  FS = 18; LFS = 12; AXLW = 2.0;  W = 0.05;
    f = figure('Position', [40 40 1100 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = gobjects(1, 3);  for r = 1:3; ax(r) = nexttile(tl, r); hold(ax(r), 'on'); end
    hh = gobjects(1, 3);
    for c = 1:3
        r2 = load(fullfile(id, sprintf('ladder_ramp16%s_o2.mat', tags{c})));
        t = r2.t(:);  Ts = t(2) - t(1);  nw = max(1, round(W / Ts));
        ea = mean(r2.a_true - r2.a_hat, 2);
        n1 = movmean(mean(r2.innov1.^2 ./ r2.S_y1, 2), nw);
        g2 = r2.S_y2 > 0;  q2 = zeros(size(r2.innov2));  q2(g2) = r2.innov2(g2).^2 ./ r2.S_y2(g2);
        n2 = movmean(mean(q2, 2), nw);
        hh(c) = plot(ax(1), t, ea, '-', 'Color', COLS{c}, 'LineWidth', 1.8, 'DisplayName', names{c});
        plot(ax(2), t, n1, '-', 'Color', COLS{c}, 'LineWidth', 1.8);
        plot(ax(3), t, n2, '-', 'Color', COLS{c}, 'LineWidth', 1.8);
        seg = r2.seg;
        hm = mean(r2.hb, 2);  m_near = hm < 3 & t > seg(1) & t <= seg(2);  m_far = t > seg(1) & hm > 10;  m_end = t > seg(2);
        fprintf('%-26s | NIS1 far %.2f near %.2f hold-end %.2f | NIS2 far %.2f near %.2f hold-end %.2f | e_a near %+.3f\n', ...
            names{c}, mean(n1(m_far)), mean(n1(m_near)), mean(n1(m_end)), mean(n2(m_far)), mean(n2(m_near)), mean(n2(m_end)), mean(ea(m_near)));
    end
    seg = r2.seg;
    for r = 1:3
        yl = ylim(ax(r));  for s = seg(:).'; plot(ax(r), [s s], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2); end
        xlim(ax(r), [0 t(end)]);  set(ax(r), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
    end
    plot(ax(1), [0 t(end)], [0 0], '-', 'Color', [0.6 0.6 0.6]);
    plot(ax(2), [0 t(end)], [1 1], 'r-', 'LineWidth', 1.4);  plot(ax(3), [0 t(end)], [1 1], 'r-', 'LineWidth', 1.4);
    ylabel(ax(1), 'e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax(2), 'NIS_1 = \nu_1^2 / S_1', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax(3), 'NIS_2 = \nu_2^2 / S_2', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(ax(3), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    legend(ax(1), hh, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(ax(2), 'YScale', 'log');  set(ax(3), 'YScale', 'log');
    out = fullfile(fd, 'fig_noap_nis_three_arms.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
