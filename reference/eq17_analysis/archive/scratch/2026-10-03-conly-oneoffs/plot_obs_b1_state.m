% FORK OF test_script/integration/plot_obs_b_5state_two_traj.m (style) (2026-09-14) | PURPOSE: gates 4-5 picture for b1 = db/dw
%   (verify_obs_b1_state.m output), Meng left, canon right, shared y per row. Rows: (1) commanded height w_d/R; (2) b1 CRLB/prior per
%   500-step window -- structural (b1 added at b1_hat = 0 on the constant-b run) and the implemented b1 run, gray dotted = the
%   anchor-consistent width over the b1 prior; (3) b CRLB/prior -- constant-b run vs with b1; (4) a_w CRLB/prior, same pair.
%   CRLB/prior < 1 = the data beat the prior in that window. | EXPIRES: when b1_state is closed | 產線改動不會自動跟上
function plot_obs_b1_state(win)
    if nargin < 1 || isempty(win); win = 500; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    TR = {'meng', 'canon'};
    COL_O = [0 0.2 0.9];  COL_B1 = [0.85 0.33 0.10];  FS = 15; LFS = 11; AXLW = 1.8;
    f = figure('Units', 'inches', 'Position', [0 0 13 13], 'Color', 'w', 'Visible', 'off');
    tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');  ax = gobjects(4, 2);
    for c = 1:2
        S = load(fullfile(od, sprintf('verify_obs_b1_state_%s_w%d.mat', TR{c}, win)));
        pr = S.priors;  T_END = ceil(S.t(end));
        ax(1, c) = nexttile(c); hold on;
        plot(S.t, S.hd, '-', 'Color', [0.2 0.2 0.2], 'LineWidth', 1.6);
        local_style(c == 1, 'w_d / R', false, T_END, FS, AXLW);  set(gca, 'YScale', 'log');
        % b1
        ax(2, c) = nexttile(2 + c); hold on;
        [t2, r2] = local_ratio(S.V2, 6, pr(3));  [t3, r3] = local_ratio(S.V3, 6, pr(3));
        h1 = plot(t2, r2, '-o', 'Color', COL_B1, 'LineWidth', 1.6, 'MarkerSize', 4);
        h2 = plot(t3, r3, '--s', 'Color', COL_B1, 'LineWidth', 1.4, 'MarkerSize', 4);
        yline(1, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
        h3 = yline(S.anchor_w / pr(3), ':', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.4);
        legend([h1 h2 h3], {'b_1  (added at \^b_1 = 0)', 'b_1  (b_1 run)', 'anchor-consistent width / prior'}, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_style(c == 1, 'CRLB / prior   b_1', false, T_END, FS, AXLW);  set(gca, 'YScale', 'log');
        % b
        ax(3, c) = nexttile(4 + c); hold on;
        [t1, r1] = local_ratio(S.V1, 5, pr(2));  [t2, r2] = local_ratio(S.V2, 5, pr(2));  [t3, r3] = local_ratio(S.V3, 5, pr(2));
        h1 = plot(t1, r1, '-o', 'Color', COL_O, 'LineWidth', 1.6, 'MarkerSize', 4);
        h2 = plot(t2, r2, '-o', 'Color', COL_B1, 'LineWidth', 1.6, 'MarkerSize', 4);
        h3 = plot(t3, r3, '--s', 'Color', COL_B1, 'LineWidth', 1.4, 'MarkerSize', 4);
        yline(1, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
        legend([h1 h2 h3], {'b  (constant b)', 'b  (b_1 added)', 'b  (b_1 run)'}, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_style(c == 1, 'CRLB / prior   b', false, T_END, FS, AXLW);  set(gca, 'YScale', 'log');
        % a_w
        ax(4, c) = nexttile(6 + c); hold on;
        [t1, r1] = local_ratio(S.V1, 4, pr(1));  [t2, r2] = local_ratio(S.V2, 4, pr(1));
        h1 = plot(t1, r1, '-o', 'Color', COL_O, 'LineWidth', 1.6, 'MarkerSize', 4);
        h2 = plot(t2, r2, '-o', 'Color', COL_B1, 'LineWidth', 1.6, 'MarkerSize', 4);
        yline(1, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
        legend([h1 h2], {'\^a_w  (constant b)', '\^a_w  (b_1 added)'}, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_style(c == 1, 'CRLB / prior   \^a_w', true, T_END, FS, AXLW);  set(gca, 'YScale', 'log');
    end
    for r = 2:4; linkaxes(ax(r, :), 'y'); end
    png = fullfile(od, sprintf('obs_b1_state_w%d.png', win));
    exportgraphics(f, png, 'Resolution', 150);  close(f);  fprintf('saved %s\n', png);
end

function [t, r] = local_ratio(R, slot, prior)
    j = find(R.free == slot);  t = R.t_win(:);  r = R.crlb(:, j, 1) / prior;  r(~isfinite(r)) = NaN;
end

function local_style(is_left, ylab, is_bottom, T_END, FS, AXLW)
    if is_left; ylabel(ylab, 'FontSize', FS, 'FontWeight', 'bold'); end
    if is_bottom; xlabel('time  [s]', 'FontSize', FS, 'FontWeight', 'bold'); end
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off; xlim([0 T_END]);
end
