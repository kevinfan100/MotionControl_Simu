% PURPOSE (2026-09-23): figure of run_5state_noap_ladder.m, house style. Columns: fixed 22.2 R (cell) | fixed 1.10 R (cell)
%   | ramp 22.2 -> 1.10 R over T_d (plane). Rows: a_true / a_hat | e_a = a_true - a_hat | rate. Two arms overlaid:
%   with delta a (blue solid, filled +-sigma band, sqrt(P44) dashed) and without delta a (orange dashed mean, dotted +-sigma).
%   Output reference/eq17_analysis/derivation/figures/fig_noap_ladder_order.png   STATUS: ACTIVE (2026-09-23)
function plot_5state_noap_ladder(cells, T_d, tags, out_name, show_o1)
    if nargin < 1 || isempty(cells); cells = {'fixed_h22p2_cell', 'fixed_h1p10_cell', 'ramp16'}; end
    if nargin < 2 || isempty(T_d); T_d = 15 * ones(1, numel(cells)); end
    if nargin < 3 || isempty(tags); tags = repmat({''}, 1, numel(cells)); end   % per-column file tag (T_d scan)
    if nargin < 4 || isempty(out_name); out_name = 'fig_noap_ladder_order.png'; end
    if nargin < 5 || isempty(show_o1); show_o1 = true; end   % false = second order only, even when the o1 file exists
    if isscalar(T_d); T_d = T_d * ones(1, numel(cells)); end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    COL_TRUE = [0.8 0 0]; C2 = [0 0.2 0.9]; C1 = [0.85 0.45 0]; B2 = [0.45 0.55 0.95]; B1 = [0.95 0.75 0.45]; FS = 18; LFS = 12; AXLW = 2.0;
    nC = numel(cells);
    f = figure('Position', [40 40 max(1100, 800 * nC), 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(3, nC, 'TileSpacing', 'compact', 'Padding', 'compact');
    for c = 1:nC
        r2 = load(fullfile(id, sprintf('ladder_%s%s_o2.mat', cells{c}, tags{c})));
        f1 = fullfile(id, sprintf('ladder_%s%s_o1.mat', cells{c}, tags{c}));  has1 = show_o1 && exist(f1, 'file') == 2;   % order-1 arm optional
        if has1; r1 = load(f1); else; r1 = r2; end
        t = r2.t;  seg = r2.seg(r2.seg > 0);
        switch cells{c}
            case 'fixed_h22p2_cell'; desc = 'cell wall, hold 22.2 R, init a_{nom}, prior 0.5 a_{nom}';
            case 'fixed_h1p10_cell'; desc = 'cell wall, hold 1.10 R, init a_{nom}, prior 0.5 a_{nom}';
            otherwise;               desc = sprintf('plane, 22.2 -> 1.10 R constant velocity over %g s, init a(h_0)', T_d(c));
        end
        if contains(tags{c}, 'lf1'); desc = [desc ', no forgetting (\lambda_f = 1)'];
        elseif contains(tags{c}, '_c0'); desc = [desc sprintf(', constant \\lambda_f = 0.%s', extractAfter(tags{c}, '_c0'))];
        else; desc = [desc ', \lambda_f schedule']; end
        % row 1
        nexttile(c); hold on; box on;
        h0 = plot(NaN, NaN, 'w.', 'DisplayName', desc);
        ht = plot(t, mean(r2.a_true, 2), '-', 'Color', COL_TRUE, 'LineWidth', 2.2, 'DisplayName', 'a_{true} / a_{nom}');
        h2 = plot(t, mean(r2.a_hat, 2), '-', 'Color', C2, 'LineWidth', 1.6, 'DisplayName', 'a_{hat}, with \delta a');
        h1 = plot(t, mean(r1.a_hat, 2), '--', 'Color', C1, 'LineWidth', 1.8, 'DisplayName', 'a_{hat}, without \delta a', 'Visible', has1);
        local_seg(seg); xlim([0 t(end)]); ylim([0 1.1]);
        if c == 1; ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold'); end
        hl = [h0 ht h2]; if has1; hl = [hl h1]; end
        legend(hl, 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 1 + has1, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on'); local_ax(FS, AXLW);
        % row 2
        nexttile(nC + c); hold on; box on;
        e2 = r2.a_true - r2.a_hat;  e1 = r1.a_true - r1.a_hat;
        local_band(t, e2, B2);
        h2 = plot(t, mean(e2, 2), '-', 'Color', C2, 'LineWidth', 1.6, 'DisplayName', 'e_a, with \delta a  (band = \pm\sigma_{seed})');
        m1 = mean(e1, 2);  s1 = std(e1, 0, 2);
        h1 = plot(t, m1, '--', 'Color', C1, 'LineWidth', 1.8, 'DisplayName', 'e_a, without \delta a', 'Visible', has1);
        h3 = plot(t, m1 + s1, ':', 'Color', C1, 'LineWidth', 1.4, 'DisplayName', '\pm\sigma_{seed}, without \delta a', 'Visible', has1); plot(t, m1 - s1, ':', 'Color', C1, 'LineWidth', 1.4, 'Visible', has1);
        sp = mean(sqrt(r2.P_a), 2);
        h4 = plot(t, sp, '--', 'Color', C2, 'LineWidth', 1.0, 'DisplayName', '\pm sqrt(P_{44}), with \delta a'); plot(t, -sp, '--', 'Color', C2, 'LineWidth', 1.0);
        yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off'); local_seg(seg);
        m_y = t > 0.5;   % size the axis on the settled part; the init transient is visible in row 1
        yl = 1.3 * max([max(abs(mean(e2(m_y, :), 2)) + std(e2(m_y, :), 0, 2)); max(abs(mean(e1(m_y, :), 2)) + std(e1(m_y, :), 0, 2)); 0.02]);
        xlim([0 t(end)]); ylim([-yl yl]);
        if c == 1; ylabel('e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold'); end
        hl = [h2 h4]; if has1; hl = [hl h1 h3]; end
        legend(hl, 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on'); local_ax(FS, AXLW);
        % row 3
        nexttile(2 * nC + c); hold on; box on;
        local_band(t, r2.da_hat * 1e3, B2);
        ht = plot(t, movmean(mean(r2.da_true, 2), 401) * 1e3, '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'true increment (0.25 s mean)');
        h2 = plot(t, mean(r2.da_hat, 2) * 1e3, '-', 'Color', C2, 'LineWidth', 1.6, 'DisplayName', '\delta a_{hat}, with \delta a');
        dp = mean(sqrt(r2.P_da), 2) * 1e3;  dm = mean(r2.da_hat, 2) * 1e3;
        h4 = plot(t, dm + dp, '--', 'Color', C2, 'LineWidth', 1.0, 'DisplayName', '\pm sqrt(P_{55})'); plot(t, dm - dp, '--', 'Color', C2, 'LineWidth', 1.0);
        yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off'); local_seg(seg);
        m_r = t > 0.3;  yr = 1.3 * max([max(abs(movmean(mean(r2.da_true, 2), 401) * 1e3)); max(abs(dm(m_r)) + std(r2.da_hat(m_r, :), 0, 2) * 1e3); 0.01]);
        xlim([0 t(end)]); ylim([-yr yr]);
        if c == 1; ylabel('\delta a / a_{nom}  [10^{-3} / step]', 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        legend([h2 h4 ht], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on'); local_ax(FS, AXLW);
    end
    fn = fullfile(fd, out_name);
    exportgraphics(f, fn, 'Resolution', 150);  copyfile(fn, fullfile(id, out_name));  close(f);  fprintf('figure: %s\n', fn);
end
function local_band(t, Y, col)
    m = mean(Y, 2);  s = std(Y, 0, 2);
    fill([t; flipud(t)], [m - s; flipud(m + s)], col, 'FaceAlpha', 0.25, 'EdgeColor', 'none', 'HandleVisibility', 'off');
end
function local_seg(seg)
    for s = seg; xline(s, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
end
function local_ax(FS, AXLW)
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
end
