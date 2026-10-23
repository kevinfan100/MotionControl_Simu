% STATUS: SCRATCH -- 2026-10-18 (user: same logic as the R split, now Q with R right): a' as a state, whitened y2 in both columns,
%   q55 (process noise of a'; code slot 7, Q77 = P_fix^2 I7, sqrt P_fix 0.05) = 0 | on. Constant-velocity descent 22.2 -> 1.10 R in 1 s,
%   plane wall, seed 1 (run_mc_apslot_r2 arms white_q0 / white). State order in the text: [dx1 dx2 dx3 a a'] => a' = slot 5.
%   Row 1, accumulated from t = 0 (the three sums add up to a-hat - a-hat[0] exactly): red true change of a-bar_z, dark blue prediction
%          a-hat' M-hat, grey y1 correction l41 e_x1, light blue y2 correction l42 e_am.
%   Row 2: slope a-bar' true (red), a-hat' (blue).
%   Row 3: gains into a': |l51|, |l52| (50 ms mean, log axis), recovered from the logged slot-7 updates divided by the innovations.
function plot_mc_apslot_q_split(seed, arms, names, otag)
    if nargin < 1 || isempty(seed); seed = 1; end
    if nargin < 2 || isempty(arms); arms = {'white_q0', 'white'}; names = {'q_{55} = 0', 'q_{55} on'}; end
    if nargin < 4; otag = ''; end   % 10-18: {'white_q0', 'white_fin'} = final Q (sqrt P_fix 0.4 x (1.1/h_d)^2)
    N_WIN = 80;                       % 50 ms at Ts = 1/1600 s
    od = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'test_results', 'temp_1009_ws');
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cM = [0.45 0.75 1.0];  cG = [0.55 0.55 0.55];  FS = 22;  LFS = 15;  AXLW = 2.2;
    AB = char(257);  HAT = char(770);  PR = char(8242);  EL = char(8467);
    f = figure('Position', [40 40 1700 1500], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:2
        S = load(fullfile(od, sprintf('apr2_%s_plane_desc.mat', arms{ic})));  t = S.t;  q = seed;
        c1 = S.da_y1(:, q);  c2 = S.da_y2(:, q);  c1(~isfinite(c1)) = 0;  c2(~isfinite(c2)) = 0;
        pred = [0; diff(S.a_hat(:, q))] - c1 - c2;
        K51 = abs(S.dap_y1(:, q) ./ S.nu1(:, q));  K52 = abs(S.dap_y2(:, q) ./ S.nu2(:, q));
        K51(~isfinite(K51)) = NaN;  K52(~isfinite(K52)) = NaN;
        ax1 = nexttile(tl, ic);  hold on; box on;
        ht = plot(ax1, t, S.a_true(:, q) - S.a_true(1, q), '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', 'true');
        hp = plot(ax1, t, cumsum(pred), '-', 'Color', cE, 'LineWidth', 2.4, 'DisplayName', ['\Sigma ' AB HAT PR ' M' HAT]);
        h1 = plot(ax1, t, cumsum(c1), '-', 'Color', cG, 'LineWidth', 2.0, 'DisplayName', ['\Sigma ' EL '_{41} e_{x_1}']);
        h2 = plot(ax1, t, cumsum(c2), '-', 'Color', cM, 'LineWidth', 2.0, 'DisplayName', ['\Sigma ' EL '_{42} e_{a_m}']);
        yline(ax1, 0, ':', 'Color', [0.3 0.3 0.3], 'HandleVisibility', 'off');  ylim(ax1, [-1 0.4]);
        text(ax1, 0.03, 0.08, ['whitened y_2,  ' names{ic}], 'Units', 'normalized', 'FontSize', LFS + 1, 'FontWeight', 'bold');
        legend(ax1, [ht hp h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        ax2 = nexttile(tl, 2 + ic);  hold on; box on;
        plot(ax2, t, S.ap_true(:, q), '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', [AB PR ' true']);
        plot(ax2, t, S.ap_hat(:, q), '-', 'Color', cE, 'LineWidth', 2, 'DisplayName', [AB HAT PR]);
        ylim(ax2, [-0.1 0.9]);  legend(ax2, 'Location', 'northwest', 'FontSize', LFS, 'FontWeight', 'bold');
        ax3 = nexttile(tl, 4 + ic);  hold on; box on;
        h51 = plot(ax3, t, movmean(K51, N_WIN, 'omitnan'), '-', 'Color', cG, 'LineWidth', 2.0, 'DisplayName', ['|' EL '_{51}| (50 ms mean)']);
        h52 = plot(ax3, t, movmean(K52, N_WIN, 'omitnan'), '-', 'Color', cM, 'LineWidth', 2.4, 'DisplayName', ['|' EL '_{52}| (50 ms mean)']);
        set(ax3, 'YScale', 'log');  ylim(ax3, [1e-6 10]);  yticks(ax3, 10.^(-6:1));
        legend(ax3, [h51 h52], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        xlabel(ax3, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        for a = [ax1 ax2 ax3]; xlim(a, [0 t(end)]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(a, 'off'); end
        if ic == 1
            ylabel(ax1, ['accumulated change of ' AB HAT '_z'], 'FontSize', FS - 2, 'FontWeight', 'bold');
            ylabel(ax2, [AB PR], 'FontSize', FS, 'FontWeight', 'bold');
            ylabel(ax3, ['|' EL '_{51}|,  |' EL '_{52}|'], 'FontSize', FS - 2, 'FontWeight', 'bold');
        end
        far = t > 0.1 & t < 0.6;  nr = t > 0.8 & t <= 1.0;
        fprintf('%-8s |l51| far %.2e near %.2e | |l52| far %.2e near %.2e | sqrt p55 0.5 s %.4f 1.0 s %.4f\n', arms{ic}, ...
                mean(K51(far), 'omitnan'), mean(K51(nr), 'omitnan'), mean(K52(far), 'omitnan'), mean(K52(nr), 'omitnan'), ...
                S.sp77(find(t >= 0.5, 1), q), S.sp77(find(t >= 1.0, 1), q));
    end
    out = fullfile(od, ['fig_mc_apslot_q_split' otag '.png']);  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
