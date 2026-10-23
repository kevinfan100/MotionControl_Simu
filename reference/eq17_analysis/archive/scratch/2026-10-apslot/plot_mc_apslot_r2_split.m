% STATUS: SCRATCH -- 2026-10-18 (user: same layout as motion_ctrl Fig. 5.2): a' as a state, raw y2 vs whitened y2, both q on a' = 0,
%   constant-velocity descent 22.2 -> 1.10 R in 1 s (run_mc_apslot_r2 arms raw_q0 / white_q0), plane wall, seed 1.
%   Row 1, accumulated from t = 0 (the three sums add up to a-hat - a-hat[0] exactly): red true change of a-bar_z, dark blue
%   prediction a-hat' M-hat (= d a_hat - corrections), grey y1 correction l41 e_x1, light blue y2 correction l42 e_am.
%   Row 2: |l41|, |l42| (50 ms mean, log axis); l42 = change of a-hat per unit change of the current reading a-bar_m[k] in both columns.
function plot_mc_apslot_r2_split(seed, arms, names, otag)
    if nargin < 1 || isempty(seed); seed = 1; end
    if nargin < 2 || isempty(arms); arms = {'raw_q0', 'white_q0'}; names = {'raw y_2,  q on a'' = 0', 'whitened y_2,  q on a'' = 0'}; end
    if nargin < 4; otag = '_q0'; end
    N_WIN = 80;                       % 50 ms at Ts = 1/1600 s
    od = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'test_results', 'temp_1009_ws');
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cM = [0.45 0.75 1.0];  cG = [0.55 0.55 0.55];  FS = 22;  LFS = 15;  AXLW = 2.2;
    AB = char(257);  HAT = char(770);  PR = char(8242);  EL = char(8467);
    f = figure('Position', [40 40 1700 1100], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:2
        S = load(fullfile(od, sprintf('apr2_%s_plane_desc.mat', arms{ic})));  t = S.t;  q = seed;
        c1 = S.da_y1(:, q);  c2 = S.da_y2(:, q);  c1(~isfinite(c1)) = 0;  c2(~isfinite(c2)) = 0;
        pred = [0; diff(S.a_hat(:, q))] - c1 - c2;
        K1 = abs(c1 ./ S.nu1(:, q));  K2 = abs(c2 ./ S.nu2(:, q));  K1(~isfinite(K1)) = NaN;  K2(~isfinite(K2)) = NaN;
        ax1 = nexttile(tl, ic);  hold on; box on;
        ht = plot(ax1, t, S.a_true(:, q) - S.a_true(1, q), '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', 'true');
        hp = plot(ax1, t, cumsum(pred), '-', 'Color', cE, 'LineWidth', 2.0, 'DisplayName', ['\Sigma ' AB HAT PR ' M' HAT]);
        h1 = plot(ax1, t, cumsum(c1), '-', 'Color', cG, 'LineWidth', 2.0, 'DisplayName', ['\Sigma ' EL '_{41} e_{x_1}']);
        h2 = plot(ax1, t, cumsum(c2), '-', 'Color', cM, 'LineWidth', 2.4, 'DisplayName', ['\Sigma ' EL '_{42} e_{a_m}']);
        yline(ax1, 0, ':', 'Color', [0.3 0.3 0.3], 'HandleVisibility', 'off');
        ylim(ax1, [-1 0.4]);
        text(ax1, 0.03, 0.08, names{ic}, 'Units', 'normalized', 'FontSize', LFS + 1, 'FontWeight', 'bold');
        legend(ax1, [ht hp h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        ax2 = nexttile(tl, 2 + ic);  hold on; box on;
        h41 = plot(ax2, t, movmean(K1, N_WIN, 'omitnan'), '-', 'Color', cG, 'LineWidth', 2.0, 'DisplayName', ['|' EL '_{41}| (50 ms mean)']);
        h42 = plot(ax2, t, movmean(K2, N_WIN, 'omitnan'), '-', 'Color', cM, 'LineWidth', 2.4, 'DisplayName', ['|' EL '_{42}| (50 ms mean)']);
        set(ax2, 'YScale', 'log');  ylim(ax2, [1e-5 1]);  yticks(ax2, 10.^(-5:0));
        legend(ax2, [h41 h42], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        xlabel(ax2, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        for a = [ax1 ax2]; xlim(a, [0 t(end)]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(a, 'off'); end
        if ic == 1
            ylabel(ax1, ['accumulated change of ' AB HAT '_z'], 'FontSize', FS - 2, 'FontWeight', 'bold');
            ylabel(ax2, ['|' EL '_{41}|,  |' EL '_{42}|'], 'FontSize', FS - 2, 'FontWeight', 'bold');
        end
        for tt = [0.6 0.9 1.0 1.2]
            k = find(t >= tt - 1e-9, 1);  if isempty(k); k = numel(t); end
            fprintf('%-8s t=%.1f  a_hat-a_hat0 %+.3f = pred %+.3f + y1 %+.3f + y2 %+.3f | true %+.3f\n', arms{ic}, tt, S.a_hat(k, q) - S.a_hat(1, q), ...
                    sum(pred(1:k)), sum(c1(1:k)), sum(c2(1:k)), S.a_true(k, q) - S.a_true(1, q));
        end
    end
    out = fullfile(od, sprintf('fig_mc_apslot_r2_split%s.png', otag));  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
