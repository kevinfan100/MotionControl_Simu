% STATUS: SCRATCH -- 2026-10-13, figures for run_1009_wsonly.m (motion_ctrl Section 8): one wall per figure, columns = w_s-hat[0].
%   Rows (Section 6 style, seed 1): z_d and z [um]; a-bar_z true and estimate; w_s-hat with +- sqrt(P_w) (light blue band), red = the
%   wall position set in the plant (0), gray dashed = the w_s that makes 1/(h - w_s)^2 equal the true slope at the true height
%   (h_true - 1/sqrt(a'_true(h_true))); delta z_m [um]. Console: 10-seed w_s-hat at heights, honesty ratio, e_a windows.
function plot_1009_wsonly(wall, seed, suffix, num)
    if nargin < 1; wall = 'plane'; end
    if nargin < 2; seed = 1; end
    if nargin < 3; suffix = ''; end          % 10-13: '_n98' = arm B (numerator 9/8); figure name gets the suffix
    if nargin < 4; num = 1; end              %   numerator of the law (gray dashed = h - sqrt(num / a'_true))
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(here);
    od = fullfile(root, 'test_results', 'temp_1009_ws');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    IN = [-2 -1 -0.5 0 0.5];  nC = numel(IN);
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cB = [0.75 0.85 1];  cG = [0.45 0.45 0.45];  FS = 20;  LFS = 15;  AXLW = 2;  R_UM = 2.25;
    AB = char(257);  HAT = char(770);  WB = ['w' char(772)];
    pcp = [];  if strcmp(wall, 'cell'); pcp = @cell_cperp_delta; end
    f = figure('Position', [40 40 3000 2000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(4, nC, 'TileSpacing', 'compact', 'Padding', 'compact');
    fprintf('\n[%s] w_s-hat mean +- sd over seeds (sqrtP) at h_d = 6 / 3 / 2 / 1.10 R / end of hold | honesty rms(err)/mean sqrtP at end | e_a descent mean|.|, 0.9-1.0 s, hold\n', wall);
    for ic = 1:nC
        tag = strrep(strrep(sprintf('%+g', IN(ic)), '+', 'p'), '-', 'm');  tag = strrep(tag, '.', 'd');
        S = load(fullfile(od, sprintf('wsonly_%s_%s%s.mat', wall, tag, suffix)));  q = find(S.seeds == seed, 1);  t = S.t;
        wd = S.wd;  wd(1) = wd(2);  ht = max(S.h_true(:, q), 1.001);  dh = 1e-4;
        apt = (local_a(ht + dh, pcp) - local_a(ht - dh, pcp)) / (2 * dh);  wneed = ht - sqrt(num ./ apt);
        ax = gobjects(4, 1);
        ax(1) = nexttile(tl, ic);  hold on; box on;
        h1 = plot(ax(1), t, wd * R_UM, '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', 'z_d');
        h2 = plot(ax(1), t, S.h_true(:, q) * R_UM, '--', 'Color', cE, 'LineWidth', 1.6, 'DisplayName', 'z');
        ylim(ax(1), [0 55]);  legend(ax(1), [h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        text(ax(1), 0.96, 0.9, sprintf('%s%s_s[0] = %+g', WB, HAT, IN(ic)), 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', FS, 'FontWeight', 'bold');
        ax(2) = nexttile(tl, nC + ic);  hold on; box on;
        h3 = plot(ax(2), t, S.a_true(:, q), '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', [AB '_z']);
        h4 = plot(ax(2), t, S.a_hat(:, q), '-', 'Color', cE, 'LineWidth', 1.6, 'DisplayName', [AB HAT '_z']);
        ylim(ax(2), [-0.2 1.6]);  legend(ax(2), [h3 h4], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        ax(3) = nexttile(tl, 2 * nC + ic);  hold on; box on;
        wh = S.w_hat(:, q);  sp = S.sP_w(:, q);
        fill(ax(3), [t; flipud(t)], [wh + sp; flipud(wh - sp)], cB, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        h5 = plot(ax(3), t, zeros(size(t)), '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', [WB '_s']);
        h6 = plot(ax(3), t, wneed, '--', 'Color', cG, 'LineWidth', 2, 'DisplayName', 'law');
        h7 = plot(ax(3), t, wh, '-', 'Color', cE, 'LineWidth', 1.6, 'DisplayName', [WB HAT '_s']);
        ylim(ax(3), [-3 2]);  legend(ax(3), [h5 h6 h7], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        ax(4) = nexttile(tl, 3 * nC + ic);  hold on; box on;
        plot(ax(4), t, S.dxm(:, q), '-', 'Color', cE, 'LineWidth', 0.9);
        ylim(ax(4), [-0.2 0.2]);  yticks(ax(4), [-0.1 0 0.1]);  xlabel(ax(4), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        for k = 1:4; xlim(ax(k), [0 1.2]);  set(ax(k), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax(k), 'off'); end
        if ic == 1
            ylabel(ax(1), ['z  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax(2), [AB '_z'], 'FontSize', FS, 'FontWeight', 'bold');
            ylabel(ax(3), [WB '_s  [R]'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax(4), [char(948) 'z_m  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
        end
        % console (10 seeds)
        W = S.w_hat;  P = S.sP_w;  e = S.a_true - S.a_hat;  m = mean(e, 2);  row = '';
        for hv = [6 3 2 1.10]
            i = find(wd <= hv + 1e-9 & t > 0.001, 1);  row = [row sprintf(' %+.2f+-%.2f(%.2f)', mean(W(i, :)), std(W(i, :)), mean(P(i, :)))]; %#ok<AGROW>
        end
        row = [row sprintf(' | end %+.2f+-%.2f(%.2f)', mean(W(end, :)), std(W(end, :)), mean(P(end, :)))];
        hon = sqrt(mean((W(end, :) - mean(wneed(t > 1))).^2)) / mean(P(end, :));
        fprintf('  w0 %+4.1f |%s | honesty vs law %.2f | e_a %.4f, %+.4f, %+.4f\n', IN(ic), row, hon, mean(abs(m(t <= 1))), mean(m(t > 0.9 & t <= 1)), mean(m(t > 1)));
    end
    out = fullfile(fd, sprintf('fig_mc_s8_%s%s_s%d.png', wall, suffix, seed));
    exportgraphics(f, out, 'Resolution', 120);  fprintf('figure: %s\n', out);
end

function a = local_a(h, pcp)
    h = max(h, 1.001);
    if isempty(pcp)
        a = zeros(size(h));  for i = 1:numel(h); [~, c] = calc_correction_functions(h(i)); a(i) = 1 / c; end
    else
        a = 1 ./ pcp(h);
    end
end
