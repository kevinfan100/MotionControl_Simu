% STATUS: SCRATCH -- 2026-10-07, motion_ctrl Fig. 5.2 (user: explain why the estimate is so much steadier with a'; second version:
%   per-step row instead of sqrt(P44), the table the user could read). Fig. 5.1 runs (Section 2 trajectory, plane, lambda_c 0.4),
%   seed 1. Columns: first order | c-only (q55 = 0). Change of a_hat in one step = prediction a-hat' M-hat (0 in the first-order
%   model) + correction l41 e_x1 + l42 e_am (from the logged gain row and innovations; prediction = d a_hat - correction).
%   Row 1, accumulated from t = 0: red true change a-bar_z - a-bar_z[0], blue sum of the prediction, light blue sum of the correction.
%   Row 2 (user 10-07, replaces the per-step row): |l41| (per R of e_x1) and |l42| (per unit y2 innovation, y2 fed whitened in both codes), log axis.
function plot_mc_s5_split(seed)
    if nargin < 1; seed = 1; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
    fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    R1 = run_mc_s4_first_order({'prod_xyz'}, seed, {'plane'});  r = R1.prod_xyz.plane(seed);
    S = load(fullfile(root, 'test_results', 'temp_plaw_walls', 'f1p2_plane_PI_mcs5_desc_ramp5_lc040.mat'));
    D = {struct('t', r.t, 'a_true', r.a_true, 'a', r.a_hat, 'K1', r.K_a_y1, 'K2', r.K_a_y2, 'i1', r.innov1, 'i2', r.innov2), ...
         struct('t', S.t, 'a_true', S.a_true(:, seed), 'a', S.a_hat(:, seed), 'K1', S.K_a_y1(:, seed), 'K2', S.K_a_y2(:, seed), ...
                'i1', S.innov1(:, seed), 'i2', S.innov2(:, seed))};
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cM = [0.45 0.75 1.0];  FS = 22;  LFS = 16;  AXLW = 2.2;  U = 1e4;
    AB = char(257);  HAT = char(770);  PR = char(8242);  EL = char(8467);
    nmP = [AB HAT PR ' M' HAT];  nmC = [EL '_{41} e_{x_1} + ' EL '_{42} e_{a_m}'];
    f = figure('Position', [40 40 1600 1050], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    pc = physical_constants();  a_nom = pc.Ts / pc.gamma_N;  R_UM = pc.R;
    % l41, l42 in common units (user 10-07: bottom row = the gains): l41 = change of a-bar per unit e_x1 in R, l42 = change of a-bar
    % per unit of the y2 innovation (both codes feed y2 whitened). first order stores K/a_nom: x R for l41, x a_nom for l42.
    SC = {[R_UM, a_nom], [1, 1]};
    tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:2
        d = D{ic};  t = d.t;
        corr = d.K1 .* d.i1 + d.K2 .* d.i2;  corr(~isfinite(corr)) = 0;  pred = [0; diff(d.a)] - corr;
        if ic == 1; pred(:) = 0; end                                 % first order: no prediction term (measured 1e-17)
        dtrue = [0; diff(d.a_true)];
        ax1 = nexttile(tl, ic);  hold on; box on;
        ht = plot(ax1, t, d.a_true - d.a_true(1), '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', 'true');
        hc = plot(ax1, t, cumsum(corr), '-', 'Color', cM, 'LineWidth', 1.8, 'DisplayName', ['\Sigma (' nmC ')']);
        hp = plot(ax1, t, cumsum(pred), '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', ['\Sigma ' nmP]);
        ylim(ax1, [-1 0.3]);
        legend(ax1, [ht hp hc], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        ax2 = nexttile(tl, 2 + ic);  hold on; box on;
        l41 = movmean(abs(d.K1) * SC{ic}(1), 80);  l42 = movmean(abs(d.K2) * SC{ic}(2), 80);   % 50 ms mean of |l| (first-order l41 flips sign step to step)
        h41 = plot(ax2, t, l41, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', ['|' EL '_{41}| (50 ms mean)']);
        h42 = plot(ax2, t, l42, '-', 'Color', cM, 'LineWidth', 1.8, 'DisplayName', ['|' EL '_{42}| (50 ms mean)']);
        set(ax2, 'YScale', 'log');  ylim(ax2, [1e-5 3]);  yticks(ax2, 10.^(-5:0));
        legend(ax2, [h41 h42], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        xlabel(ax2, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        for a = [ax1 ax2]; xlim(a, [0 t(end)]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(a, 'off'); end
        if ic == 1
            ylabel(ax1, ['accumulated change of ' AB '_z'], 'FontSize', FS - 2, 'FontWeight', 'bold');
            ylabel(ax2, ['|' EL '_{41}|,  |' EL '_{42}|'], 'FontSize', FS - 2, 'FontWeight', 'bold');
        end
        for w = {[2 2.2], [4.7 4.8]}
            m = t > w{1}(1) & t <= w{1}(2);
            fprintf('col %d %.1f-%.1f s: true %+.2f | pred %+.2f | corr rms %.2f [x1e-4/step] | l41 %.3g | l42 %.3g\n', ic, w{1}, ...
                    mean(dtrue(m)) * U, mean(pred(m)) * U, rms(corr(m)) * U, mean(l41(m)), mean(l42(m)));
        end
    end
    out = fullfile(fd, sprintf('fig_mc_s5_split_s%d.png', seed));  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
