% PURPOSE (2026-09-27, user: "with T_d longer, is the final a_hat the same? show three T_d"): second-order gain model,
%   lambda_f = 1 (no forgetting), plane, 5 seeds, ramp template (hold 0.5 s, 22.2 -> 1.10 R constant velocity over T_d, hold 0.5 s),
%   T_d = 2 / 5 / 15 s.  Left column against time, right column against the commanded height h_d (reversed, log):
%   if the estimate depends only on the path, the three curves on the right fall on top of each other during the descent;
%   the final hold at 1.10 R shows as a vertical stroke at the left end of the right column.
%   Rows: a_true / a_hat | e_a = a_true - a_hat.  Runs: test_results/temp_5state_noap/ladder_ramp16_Td{2,5,15}_lf1_o2.mat
%   STATUS: SCRATCH
function plot_5state_noap_td3_lf1()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    TD = [2 5 15];  COLS = {[0 0.2 0.9], [0.85 0.45 0], [0.1 0.6 0.1]};  STY = {'-', '--', '-.'};
    COL_TRUE = [0.8 0 0]; FS = 18; LFS = 12; AXLW = 2.0;
    f = figure('Position', [40 40 1700 900], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = gobjects(2, 2);  for q = 1:4; ax(q) = nexttile(tl, q); hold(ax(q), 'on'); end   % ax(1)=r1c1, ax(2)=r1c2 (row-major tiles)
    ax = reshape(ax, 2, 2).';                                                              % ax(row, col)
    hh = gobjects(1, 3);
    fprintf('%-6s | a_hat descent end  e_a descent end | e_a near (h<3R) | a_hat hold end  e_a hold end\n', 'T_d');
    for k = 1:numel(TD)
        r = load(fullfile(id, sprintf('ladder_ramp16_Td%d_lf1_o2.mat', TD(k))));
        t = r.t(:);  hd = r.hd(:);  at = mean(r.a_true, 2);  ah = mean(r.a_hat, 2);  ea = at - ah;  seg = r.seg;
        i_de = find(t <= seg(2), 1, 'last');  m_near = hd < 3 & t > seg(1) & t <= seg(2);
        fprintf('%-6g | %.3f              %+.3f           | %+.3f          | %.3f           %+.3f\n', TD(k), ah(i_de), ea(i_de), mean(ea(m_near)), ah(end), ea(end));
        if k == 1
            ht = plot(ax(1, 1), t, at, '-', 'Color', COL_TRUE, 'LineWidth', 2.4, 'DisplayName', 'a_{true} / a_{nom}');
            plot(ax(1, 2), hd, at, '-', 'Color', COL_TRUE, 'LineWidth', 2.4);
        else
            plot(ax(1, 1), t, at, '-', 'Color', COL_TRUE, 'LineWidth', 2.4);
        end
        hh(k) = plot(ax(1, 1), t, ah, STY{k}, 'Color', COLS{k}, 'LineWidth', 2.0, 'DisplayName', sprintf('a_{hat}, T_d = %g s', TD(k)));
        plot(ax(1, 2), hd, ah, STY{k}, 'Color', COLS{k}, 'LineWidth', 2.0);
        plot(ax(2, 1), t, ea, STY{k}, 'Color', COLS{k}, 'LineWidth', 2.0);
        plot(ax(2, 2), hd, ea, STY{k}, 'Color', COLS{k}, 'LineWidth', 2.0);
        plot(ax(1, 1), seg(2) * [1 1], [0 1.1], ':', 'Color', COLS{k}, 'LineWidth', 1.2);   % descent end of this T_d
        plot(ax(2, 1), seg(2) * [1 1], [-0.6 0.1], ':', 'Color', COLS{k}, 'LineWidth', 1.2);
    end
    for c = 1:2
        ylim(ax(1, c), [0 1.1]);  ylim(ax(2, c), [-0.6 0.1]);
        plot(ax(2, c), xlim(ax(2, c)), [0 0], '-', 'Color', [0.6 0.6 0.6]);
    end
    for rr = 1:2
        xlim(ax(rr, 1), [0 16]);
        set(ax(rr, 2), 'XScale', 'log', 'XDir', 'reverse', 'XLim', [1.05 23], 'XTick', [1.1 1.5 2 3 5 10 22.2]);
        for c = 1:2; set(ax(rr, c), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); end
    end
    ylabel(ax(1, 1), 'a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax(2, 1), 'e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(ax(2, 1), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(ax(2, 2), 'h_d  [R]  (descent runs left)', 'FontSize', FS, 'FontWeight', 'bold');
    h0 = plot(ax(1, 1), nan, nan, 'w-', 'DisplayName', 'second order, no forgetting (\lambda_f = 1), plane');
    legend(ax(1, 1), [h0 ht hh], 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 3, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'AutoUpdate', 'off');
    out = fullfile(fd, 'fig_noap_td3_lf1.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
