% STATUS: SCRATCH -- 2026-10-13, single-seed figure in the motion_ctrl Section 6 style, one wall per figure, start w_s-hat[0] = 0.
%   Columns: S8 (numerator 1) | B (numerator 9/8, c pinned at -1/8) | C (B + q77_adapt_P0 0.06). Rows: z_d and z [um]; a-bar_z true and
%   estimate; w_s-hat with +- sqrt(P_w) band, red = wall position set in the plant (0), gray dashed = the w_s that makes num/(h - w_s)^2
%   equal the true slope at the true height (num = 1 or 9/8, per column); delta z_m [um]. Data: run_1009_wsonly.m.
function plot_1009_ws_arms_s1(wall, seed)
    if nargin < 1; wall = 'plane'; end
    if nargin < 2; seed = 1; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(genpath(fullfile(root, 'model')));  addpath(here);
    od = fullfile(root, 'test_results', 'temp_1009_ws');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    TG = {'', '_n98', '_n98q77'};  NUM = [1, 9/8, 9/8];  NM = {'S8: numerator 1', 'B: numerator 9/8', ['C: 9/8 + Q_{77}, ' char(8730) 'P_{fix} 0.06']};
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cB = [0.75 0.85 1];  cG = [0.45 0.45 0.45];  FS = 20;  LFS = 15;  R_UM = 2.25;
    AB = char(257);  HAT = char(770);  WB = ['w' char(772)];
    pcp = [];  if strcmp(wall, 'cell'); pcp = @cell_cperp_delta; end
    f = figure('Position', [40 40 2300 2100], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(4, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:3
        S = load(fullfile(od, sprintf('wsonly_%s_p0%s.mat', wall, TG{ic})));  q = find(S.seeds == seed, 1);  t = S.t;
        wd = S.wd;  wd(1) = wd(2);  ht = max(S.h_true(:, q), 1.001);  dh = 1e-4;
        apt = (local_a(ht + dh, pcp) - local_a(ht - dh, pcp)) / (2 * dh);  wneed = ht - sqrt(NUM(ic) ./ apt);
        ax = gobjects(4, 1);
        ax(1) = nexttile(tl, ic);  hold on; box on;
        h1 = plot(ax(1), t, wd * R_UM, '-', 'Color', cT, 'LineWidth', 5, 'DisplayName', 'z_d');
        h2 = plot(ax(1), t, S.h_true(:, q) * R_UM, '--', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'z');
        ylim(ax(1), [0 55]);  legend(ax(1), [h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        text(ax(1), 0.96, 0.9, NM{ic}, 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', FS - 2, 'FontWeight', 'bold');
        ax(2) = nexttile(tl, 3 + ic);  hold on; box on;
        h3 = plot(ax(2), t, S.a_true(:, q), '-', 'Color', cT, 'LineWidth', 5, 'DisplayName', [AB '_z']);
        h4 = plot(ax(2), t, S.a_hat(:, q), '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HAT '_z']);
        ylim(ax(2), [-0.2 1.6]);  legend(ax(2), [h3 h4], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        ax(3) = nexttile(tl, 6 + ic);  hold on; box on;
        wh = S.w_hat(:, q);  sp = S.sP_w(:, q);
        fill(ax(3), [t; flipud(t)], [wh + sp; flipud(wh - sp)], cB, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        h5 = plot(ax(3), t, zeros(size(t)), '-', 'Color', cT, 'LineWidth', 5, 'DisplayName', [WB '_s']);
        h6 = plot(ax(3), t, wneed, '--', 'Color', cG, 'LineWidth', 2.2, 'DisplayName', 'law');
        h7 = plot(ax(3), t, wh, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [WB HAT '_s']);
        ylim(ax(3), [-2 2]);  legend(ax(3), [h5 h6 h7], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        ax(4) = nexttile(tl, 9 + ic);  hold on; box on;
        plot(ax(4), t, S.dxm(:, q), '-', 'Color', cE, 'LineWidth', 1.0);
        ylim(ax(4), [-0.2 0.2]);  yticks(ax(4), [-0.1 0 0.1]);  xlabel(ax(4), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        for k = 1:4; xlim(ax(k), [0 1.2]);  set(ax(k), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2.2);  grid(ax(k), 'off'); end
        if ic == 1
            ylabel(ax(1), ['z  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax(2), [AB '_z'], 'FontSize', FS + 2, 'FontWeight', 'bold');
            ylabel(ax(3), [WB '_s  [R]'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax(4), [char(948) 'z_m  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
        end
        e = S.a_true(:, q) - S.a_hat(:, q);
        fprintf('%s %-30s seed %d | near-wall (h_d<=2) mean|e_a| %.4f | hold %.4f | end w_s-hat %+.3f sqrtP %.3f\n', wall, NM{ic}, seed, ...
                mean(abs(e(t <= 1 & wd <= 2))), mean(abs(e(t > 1))), wh(end), sp(end));
    end
    out = fullfile(fd, sprintf('fig_mc_s8_arms_%s_s%d.png', wall, seed));
    exportgraphics(f, out, 'Resolution', 130);  fprintf('figure: %s\n', out);
end

function a = local_a(h, pcp)
    h = max(h, 1.001);
    if isempty(pcp)
        a = zeros(size(h));  for i = 1:numel(h); [~, c] = calc_correction_functions(h(i)); a(i) = 1 / c; end
    else
        a = 1 ./ pcp(h);
    end
end
