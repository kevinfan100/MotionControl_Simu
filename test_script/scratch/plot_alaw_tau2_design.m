% STATUS: SCRATCH -- 2026-10-19 (user): does the a law with tau = 2 s follow its designed response? Design: a - a_d = (a0 - a_d) e^(-t/tau).
%   Data alaw_tau2_long_<arm>_T40.mat (no cap, gate off, plane a_d 0.07, seed 1). Columns w_s | a' state; log y.
%   Black dashed = design; blue = a_hat - a_d (what the law regulates); red = a_true - a_d (0.1 s mean, the truth).
%   Output test_results/temp_1009_ws/fig_alaw_tau2_design.png. Console: time to 10 % / 1 % of the start gap, design vs runs (5 seeds).
function plot_alaw_tau2_design(seed)
    if nargin < 1; seed = 1; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'test_results', 'temp_1009_ws');  tau = 2;  ad = 0.07;
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  FS = 20;  LFS = 15;  AXLW = 2.2;  AB = char(257);  HAT = char(770);
    R = {load(fullfile(od, 'alaw_tau2_long_ws_T40.mat')), load(fullfile(od, 'alaw_tau2_long_ap_T40.mat'))};  NM = {['w' char(772) '_s state'], [AB ''' state']};
    f = figure('Position', [40 40 1800 1250], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for i = 1:2
        S = R{i};  t = S.t;  q = find(S.seeds == seed);  n01 = round(0.1 / (t(2) - t(1)));
        e0 = S.a_true(1, q) - ad;  et = movmean(S.a_true(:, q), n01) - ad;  eh = S.a_hat(:, q) - ad;
        ax = nexttile(tl, i);  hold on; box on;
        g1 = plot(ax, t, e0 * exp(-t / tau), '--', 'Color', 'k', 'LineWidth', 2.5, 'DisplayName', 'e^{-t/\tau_a}');
        g2 = plot(ax, t, abs(et), '-', 'Color', cT, 'LineWidth', 2.5, 'DisplayName', ['|' AB '_z - ' AB '_d|']);
        g3 = plot(ax, t, abs(eh), '-', 'Color', cE, 'LineWidth', 1.6, 'DisplayName', ['|' AB HAT '_z - ' AB '_d|']);
        set(ax, 'YScale', 'log');  ylim(ax, [1e-3 1]);  xlim(ax, [0 20]);
        lg = legend(ax, [g1 g2 g3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'NumColumns', 2);
        lg.Title.String = sprintf('plane wall, %s, %s_d = 0.07', NM{i}, AB);
        if i == 1; ylabel(ax, ['|' AB ' - ' AB '_d|'], 'FontSize', FS, 'FontWeight', 'bold'); end
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax, 'off');
        ax2 = nexttile(tl, 2 + i);  hold on; box on;          % 10-19 (user): the slope the law divides by, against the truth
        pc = physical_constants();  a_nom = pc.Ts / pc.gamma_N;  y = S.ap_hat(:, q) / a_nom;  y(y <= 0) = NaN;
        k1 = plot(ax2, t, S.ap_true(:, q) / a_nom, '-', 'Color', cT, 'LineWidth', 3.0, 'DisplayName', [AB '''_z']);
        k2 = plot(ax2, t, y, '-', 'Color', cE, 'LineWidth', 1.6, 'DisplayName', [AB HAT '''_z']);
        set(ax2, 'YScale', 'log');  ylim(ax2, [1e-3 3]);  xlim(ax2, [0 20]);
        legend(ax2, [k1 k2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        xlabel(ax2, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        if i == 1; ylabel(ax2, [AB ''''], 'FontSize', FS + 2, 'FontWeight', 'bold'); end
        set(ax2, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax2, 'off');
        T10 = zeros(1, numel(S.seeds));  T01 = T10;
        for j = 1:numel(S.seeds)
            e = movmean(S.a_true(:, j), n01) - ad;  e = e / e(1);
            T10(j) = t(find(e < 0.1, 1));  k = find(e < 0.01, 1);  if isempty(k); T01(j) = NaN; else; T01(j) = t(k); end
        end
        fprintf('%-9s | true gap to 10%%: %.1f s (design %.1f) | to 1%%: %.1f s (design %.1f) | per seed 10%%: %s\n', NM{i}, mean(T10), tau * log(10), ...
                mean(T01, 'omitnan'), tau * log(100), mat2str(T10, 2));
    end
    out = fullfile(od, 'fig_alaw_tau2_design.png');  exportgraphics(f, out, 'Resolution', 150);  close(f);  fprintf('figure: %s\n', out);
end
