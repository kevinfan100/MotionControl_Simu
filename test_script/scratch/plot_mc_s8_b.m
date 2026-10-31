% STATUS: SCRATCH -- 2026-10-13, motion_ctrl Section 8 figures for arm B (numerator 9/8), one setting only (user 10-13: start w_s-hat[0] = 0).
%   which = 'main' : columns plane | cell, rows z | a-bar_z | w_s-hat (+- sqrt P_w, red 0, gray dashed = (9/8)-law w_s) | delta z_m, seed 1
%                    -> figures/fig_mc_s8_b_s1.png
%   which = 'aperr': |a-bar'_hat - a-bar'_true| along the descent, 10-seed mean, columns plane | cell, lines c-only (S6), S8 numerator 1,
%                    B numerator 9/8; a-bar'_hat = num / (h_d - w_s-hat)^2 (c-only: (1 - c-hat) / h_d^2), the estimator height taken as
%                    h_d (delta x-hat_3 ~ 0.01 R is not logged); a-bar'_true = slope of the plant at the true height -> test_results/temp_1009_ws.
function plot_mc_s8_b(which, suffix, num, st)
    if nargin < 1; which = 'main'; end
    if nargin < 2; suffix = '_n98'; end       % 10-13 (user): '' = S8 numerator 1 (the chosen setting), '_n98' = B numerator 9/8
    if nargin < 3; num = 9/8; end
    if nargin < 4; st = 'p0'; end               % 10-13 (user): start tag, 'm1' = w_s-hat[0] = -1 (wrong start)
    tagf = 'n1';  if contains(suffix, 'n98'); tagf = 'b'; end
    if contains(suffix, 'sw030'); tagf = [tagf '_sw030']; end   % 10-13 (user): prior sqrt P_w[0] = 0.3
    if ~strcmp(st, 'p0'); tagf = [tagf '_' st]; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(genpath(fullfile(root, 'model')));  addpath(here);
    od = fullfile(root, 'test_results', 'temp_1009_ws');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    WL = {'plane', 'cell'};  PC = {[], @cell_cperp_delta};
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cB = [0.75 0.85 1];  cG = [0.45 0.45 0.45];  FS = 22;  LFS = 17;  R_UM = 2.25;
    AB = char(257);  HAT = char(770);  WB = ['w' char(772)];
    if strcmp(which, 'main')
        f = figure('Position', [40 40 1600 2000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
        tl = tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
        for ic = 1:2
            S = load(fullfile(od, sprintf('wsonly_%s_%s%s.mat', WL{ic}, st, suffix)));  q = 1;  t = S.t;  wd = S.wd;  wd(1) = wd(2);
            ht = max(S.h_true(:, q), 1.001);  apt = local_ap(ht, PC{ic});  wneed = ht - sqrt(num ./ apt);
            ax = gobjects(4, 1);
            ax(1) = nexttile(tl, ic);  hold on; box on;
            h1 = plot(ax(1), t, wd * R_UM, '-', 'Color', cT, 'LineWidth', 5, 'DisplayName', 'z_d');
            h2 = plot(ax(1), t, S.h_true(:, q) * R_UM, '--', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'z');
            ylim(ax(1), [0 55]);  legend(ax(1), [h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            text(ax(1), 0.96, 0.9, WL{ic}, 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', FS, 'FontWeight', 'bold');
            ax(2) = nexttile(tl, 2 + ic);  hold on; box on;
            h3 = plot(ax(2), t, S.a_true(:, q), '-', 'Color', cT, 'LineWidth', 5, 'DisplayName', [AB '_z']);
            h4 = plot(ax(2), t, S.a_hat(:, q), '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HAT '_z']);
            ylim(ax(2), [-0.2 1.6]);  legend(ax(2), [h3 h4], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            ax(3) = nexttile(tl, 4 + ic);  hold on; box on;
            wh = S.w_hat(:, q);  sp = S.sP_w(:, q);
            fill(ax(3), [t; flipud(t)], [wh + sp; flipud(wh - sp)], cB, 'EdgeColor', 'none', 'HandleVisibility', 'off');
            weq = -1.05 * strcmp(WL{ic}, 'cell');   % 10-15 (user): red = equivalent wall (plane 0, cell -1.05); grey = visible surface (cell only)
            if weq ~= 0; plot(ax(3), t, zeros(size(t)), '-', 'Color', [0.55 0.55 0.55], 'LineWidth', 2, 'DisplayName', 'visible surface'); end
            h5 = plot(ax(3), t, weq * ones(size(t)), '-', 'Color', cT, 'LineWidth', 5, 'DisplayName', ['equivalent wall ' WB '_s']);
            h6 = plot(ax(3), t, wneed, '--', 'Color', cG, 'LineWidth', 2.2, 'DisplayName', 'law');
            h7 = plot(ax(3), t, wh, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [WB HAT '_s']);
            ylim(ax(3), [-2 2]);  legend(ax(3), 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            ax(4) = nexttile(tl, 6 + ic);  hold on; box on;
            plot(ax(4), t, S.dxm(:, q), '-', 'Color', cE, 'LineWidth', 1.0);
            ylim(ax(4), [-0.2 0.2]);  yticks(ax(4), [-0.1 0 0.1]);  xlabel(ax(4), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
            for k = 1:4; xlim(ax(k), [0 1.2]);  set(ax(k), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2.2);  grid(ax(k), 'off'); end
            if ic == 1
                ylabel(ax(1), ['z  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax(2), [AB '_z'], 'FontSize', FS + 2, 'FontWeight', 'bold');
                ylabel(ax(3), [WB '_s  [R]'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax(4), [char(948) 'z_m  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
            end
        end
        out = fullfile(fd, sprintf('fig_mc_s8_%s_s1.png', tagf));
    elseif strcmp(which, 'aperr1')                 % 10-13 (user): a' error of the chosen arm alone, plane | cell, seeds light + 10-seed mean
        f = figure('Position', [40 40 2000 800], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
        tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
        for ic = 1:2
            ax = nexttile(tl);  hold on; box on;
            S = load(fullfile(od, sprintf('wsonly_%s_%s%s.mat', WL{ic}, st, suffix)));  t = S.t;  wd = S.wd;  wd(1) = wd(2);  dsc = t <= 1;
            E = abs(num ./ (wd - S.w_hat).^2 - local_ap(max(S.h_true, 1.001), PC{ic}));
            plot(ax, t(dsc), E(dsc, :), '-', 'Color', [0.6 0.78 1], 'LineWidth', 0.8, 'HandleVisibility', 'off');
            plot(ax, t(dsc), local_ap(max(wd(dsc), 1.001), PC{ic}), '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', [AB '''  (true)']);
            plot(ax, t(dsc), mean(E(dsc, :), 2), '-', 'Color', cE, 'LineWidth', 2.5, 'DisplayName', ['|' AB HAT ''' - ' AB '''|  (10-seed mean)']);
            m = mean(E, 2);  fprintf('%s |a''-hat - a''| 10-seed mean: >6R %.5f | 6-2R %.4f | 2-1.1R %.4f\n', WL{ic}, mean(m(dsc & wd > 6)), mean(m(dsc & wd <= 6 & wd > 2)), mean(m(dsc & wd <= 2)));
            set(ax, 'YScale', 'log');  ylim(ax, [1e-5 1]);  xlim(ax, [0 1]);
            xlabel(ax, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');  text(ax, 0.03, 0.92, WL{ic}, 'Units', 'normalized', 'FontSize', FS, 'FontWeight', 'bold');
            if ic == 1; ylabel(ax, [AB ''''], 'FontSize', FS + 2, 'FontWeight', 'bold'); end
            legend(ax, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2.2);  grid(ax, 'off');
        end
        out = fullfile(fd, sprintf('fig_mc_s8_%s_aperr.png', tagf));
    else
        f = figure('Position', [40 40 2000 800], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
        tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
        CO = {'wallshift_p0', 'wallshift_m1.05'};
        for ic = 1:2
            ax = nexttile(tl);  hold on; box on;
            C = load(fullfile(od, [CO{ic} '.mat']));  S = load(fullfile(od, sprintf('wsonly_%s_p0.mat', WL{ic})));  B = load(fullfile(od, sprintf('wsonly_%s_p0_n98.mat', WL{ic})));
            t = B.t;  wd = B.wd;  wd(1) = wd(2);
            E = { abs((1 - C.c_hat) ./ wd.^2 - local_ap(max(C.h_true, 1.001), PC{ic})), ...
                  abs(1 ./ (wd - S.w_hat).^2 - local_ap(max(S.h_true, 1.001), PC{ic})), ...
                  abs((9/8) ./ (wd - B.w_hat).^2 - local_ap(max(B.h_true, 1.001), PC{ic})) };
            NM = {'c-only', 'S8: numerator 1', 'B: numerator 9/8'};  CL = {'k', [0 0.25 0.85], [0 0.6 0.25]};
            for a = 1:3
                m = mean(E{a}, 2);  m(t > 1) = NaN;  plot(ax, t, m, '-', 'Color', CL{a}, 'LineWidth', 2.4, 'DisplayName', NM{a});
                dsc = t <= 1;  fprintf('%s %-18s |a''-hat - a''-true| (10-seed mean): >6R %.5f | 6-2R %.5f | 2-1.1R %.4f\n', WL{ic}, NM{a}, ...
                        mean(m(dsc & wd > 6)), mean(m(dsc & wd <= 6 & wd > 2)), mean(m(dsc & wd <= 2)));
            end
            plot(ax, t, local_ap(max(wd, 1.001), PC{ic}) .* (t <= 1) ./ (t <= 1), ':', 'Color', cT, 'LineWidth', 2, 'DisplayName', [AB '''_{true}  (scale)']);
            set(ax, 'YScale', 'log');  ylim(ax, [1e-5 1]);  xlim(ax, [0 1]);
            xlabel(ax, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');  text(ax, 0.03, 0.92, WL{ic}, 'Units', 'normalized', 'FontSize', FS, 'FontWeight', 'bold');
            if ic == 1; ylabel(ax, ['|' AB HAT ''' - ' AB '''|   (10-seed mean)'], 'FontSize', FS - 2, 'FontWeight', 'bold'); end
            legend(ax, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS - 2, 'FontWeight', 'bold');
            set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 1.8);  grid(ax, 'off');
        end
        out = fullfile(od, 'fig_1009_aprime_error.png');
    end
    exportgraphics(f, out, 'Resolution', 140);  fprintf('figure: %s\n', out);
end

function ap = local_ap(h, pcp)
    d = 1e-4;  ap = (local_a(h + d, pcp) - local_a(h - d, pcp)) / (2 * d);
end

function a = local_a(h, pcp)
    h = max(h, 1.001);
    if isempty(pcp)
        a = zeros(size(h));  for i = 1:numel(h); [~, c] = calc_correction_functions(h(i)); a(i) = 1 / c; end
    else
        a = 1 ./ pcp(h);
    end
end
