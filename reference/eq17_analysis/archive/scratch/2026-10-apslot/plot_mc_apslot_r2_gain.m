% STATUS: SCRATCH -- 2026-10-18 (user: see the KF gain difference raw vs whitened y2 directly). a' as a state, q on a' off by default,
%   constant-velocity descent, plane, seed 1 (run_mc_apslot_r2 arms raw_q0 / white_q0, or raw / white with qtag '').
%   Row 1: |l42| = change of a-hat per unit y2 innovation (per unit change of the current reading a-bar_m[k] in both).
%   Row 2: gain per NEW thermal sample u[j] (a_m[k] = (1-a_cov) a_m[k-1] + a_cov u[k]):
%          raw y2 = a_m carries u[j] into every later step with weight a_cov (1-a_cov)^n  =>  g[j] = sum_n l42[j+n] a_cov (1-a_cov)^n;
%          whitened y2 = a_cov u[k] carries it once                                       =>  g[j] = a_cov l42[j].
%   Row 3: a-bar_z true (red), estimate raw (grey), whitened (blue). Output test_results/temp_1009_ws/fig_mc_apslot_r2_gain.png.
function plot_mc_apslot_r2_gain(seed, qtag)
    if nargin < 1 || isempty(seed); seed = 1; end
    if nargin < 2; qtag = '_q0'; end   % 10-18 (user): default = q on a' off (arms raw_q0 / white_q0); '' = q on
    A_COV = 0.05;                     % EWMA weight of the gain readout (deep scenario value)
    N_TAIL = 200;                     % steps kept in the raw carry-over sum (0.95^200 = 3.5e-5)
    od = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'test_results', 'temp_1009_ws');
    D = {load(fullfile(od, ['apr2_raw' qtag '_plane_desc.mat'])), load(fullfile(od, ['apr2_white' qtag '_plane_desc.mat']))};
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cR = [0.55 0.55 0.55];  FS = 20;  LFS = 15;  AB = char(257);  HAT = char(770);  EL = char(8467);
    t = D{1}.t;  n = numel(t);  q = seed;  w = A_COV * (1 - A_COV).^(0:N_TAIL)';
    L = cell(1, 2);  g = cell(1, 2);
    for i = 1:2
        l = D{i}.da_y2(:, q) ./ D{i}.nu2(:, q);  l(~isfinite(l)) = 0;  L{i} = l;
        if i == 1
            gi = zeros(n, 1);
            for j = 1:n; m = min(N_TAIL + 1, n - j + 1); gi(j) = sum(l(j:j+m-1) .* w(1:m)); end
            g{i} = gi;
        else
            g{i} = A_COV * l;
        end
    end
    f = figure('Position', [40 40 1500 1300], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax1 = nexttile(tl);  hold on; box on;
    plot(ax1, t, movmean(abs(L{1}), 80), '-', 'Color', cR, 'LineWidth', 2, 'DisplayName', 'raw y_2');
    plot(ax1, t, movmean(abs(L{2}), 80), '-', 'Color', cE, 'LineWidth', 2, 'DisplayName', 'whitened y_2');
    set(ax1, 'YScale', 'log');  ylim(ax1, [1e-5 1e-1]);  ylabel(ax1, ['|' EL '_{42}|'], 'FontSize', FS, 'FontWeight', 'bold');
    legend(ax1, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    ax2 = nexttile(tl);  hold on; box on;
    plot(ax2, t, movmean(abs(g{1}), 80), '-', 'Color', cR, 'LineWidth', 2);
    plot(ax2, t, movmean(abs(g{2}), 80), '-', 'Color', cE, 'LineWidth', 2);
    set(ax2, 'YScale', 'log');  ylim(ax2, [1e-5 1e-1]);  ylabel(ax2, 'gain per new sample', 'FontSize', FS - 2, 'FontWeight', 'bold');
    ax3 = nexttile(tl);  hold on; box on;
    plot(ax3, t, D{2}.a_true(:, q), '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', [AB '_z true']);
    plot(ax3, t, D{1}.a_hat(:, q), '-', 'Color', cR, 'LineWidth', 1.6, 'DisplayName', [AB HAT '_z raw y_2']);
    plot(ax3, t, D{2}.a_hat(:, q), '-', 'Color', cE, 'LineWidth', 1.6, 'DisplayName', [AB HAT '_z whitened y_2']);
    ylim(ax3, [0 1.05]);  text(ax3, 0.98, 0.9, sprintf('q on a'' %s, seed %d', ternary_txt(isempty(qtag)), q), 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', LFS + 1, 'FontWeight', 'bold');  ylabel(ax3, [AB '_z'], 'FontSize', FS, 'FontWeight', 'bold');  xlabel(ax3, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    legend(ax3, 'Location', 'southwest', 'FontSize', LFS, 'FontWeight', 'bold');
    for a = [ax1 ax2 ax3]; xlim(a, [0 t(end)]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);  grid(a, 'off'); end
    far = t > 0.1 & t < 0.6;  nr = t > 0.8 & t <= 1.0;
    for i = 1:2
        fprintf('%-6s |l42| far %.2e near %.2e | per new sample far %.2e near %.2e\n', D{i}.arm, mean(abs(L{i}(far))), mean(abs(L{i}(nr))), mean(abs(g{i}(far))), mean(abs(g{i}(nr))));
    end
    out = fullfile(od, ['fig_mc_apslot_r2_gain' qtag '.png']);  exportgraphics(f, out, 'Resolution', 120);  fprintf('figure: %s\n', out);
end

function s = ternary_txt(q_on)
    if q_on; s = 'on'; else; s = '= 0'; end
end
