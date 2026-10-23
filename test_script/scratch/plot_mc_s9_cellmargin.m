% STATUS: SCRATCH -- 2026-10-15 (user): cell wall, higher a_d buys tolerance to a wrong wall guess at the cost of accuracy.
%   Realistic protocol (w_s-hat[0] = 0, true wall moved by DELTA, plant side), S8 + fe_cmd_eval, tau_a 0.1 s, 6 s (files _cm_<ad>_100_<DELTA>_s<seed>).
%   Figure 1/2: DELTA = +2 / -1, seed 1, columns a_d 0.52 | 0.60 | 0.70; rows true gap h (blue, black dashed h*) and a-bar_z (red true, blue estimate, dashed a_d).
%   Figure 3: final h - h* (last 0.5 s, seed mean, bars = min..max over the seeds that did not touch) vs DELTA, one line per a_d; touching seeds counted in labels.
function plot_mc_s9_cellmargin()
    here = fileparts(mfilename('fullpath'));  od = fullfile(fileparts(fileparts(here)), 'test_results', 'temp_1009_ws');
    AD = [0.52 0.60 0.70];  HS = [1.1741 1.6540 2.6110];  D = [-1 1 2 3];  cT = [0.85 0 0];  cE = [0 0.25 0.85];  FS = 18;  LFS = 14;  AB = char(257);  HAT = char(770);
    for DE = [2 -1]
        f = figure('Position', [40 40 2100 1050], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');  tl = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
        for j = 1:3
            S = load(fullfile(od, local_file(AD(j), DE, 1)));  t = S.t;
            a1 = nexttile(tl, j);  hold on; box on;
            plot(a1, t, S.h_true, '-', 'Color', cE, 'LineWidth', 1.6, 'DisplayName', 'h');  yline(a1, HS(j), '--', 'Color', 'k', 'LineWidth', 1.8, 'DisplayName', 'h^* (gain = a_d)');
            yline(a1, 1, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'DisplayName', 'contact');  ylim(a1, [0.95 3.2]);
            text(a1, 0.97, 0.88, sprintf('a_d = %.2f,  \\Delta = %+g R', AD(j), DE), 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', LFS + 2, 'FontWeight', 'bold');
            a2 = nexttile(tl, 3 + j);  hold on; box on;
            plot(a2, t, S.a_true, '-', 'Color', cT, 'LineWidth', 3.5, 'DisplayName', [AB '_z']);  plot(a2, t, S.a_hat, '-', 'Color', cE, 'LineWidth', 1.6, 'DisplayName', [AB HAT '_z']);
            yline(a2, AD(j), '--', 'Color', 'k', 'LineWidth', 2, 'DisplayName', [AB '_d']);  ylim(a2, [0.45 0.85]);  xlabel(a2, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
            if j == 1; ylabel(a1, 'true gap h  [R]', 'FontSize', FS, 'FontWeight', 'bold');  ylabel(a2, [AB '_z'], 'FontSize', FS, 'FontWeight', 'bold'); end
            for a = [a1 a2]; xlim(a, [0 t(end)]);  legend(a, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
                set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);  grid(a, 'off'); end
        end
        out = fullfile(od, sprintf('fig_mc_s9_cellmargin_d%s.png', strrep(strrep(sprintf('%+g', DE), '-', 'm'), '+', 'p')));  exportgraphics(f, out, 'Resolution', 120);  fprintf('figure: %s\n', out);
    end
    f = figure('Position', [40 40 1300 800], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');  ax = axes(f);  hold on; box on;
    C = {[0.15 0.15 0.15], [0 0.25 0.85], [0.95 0.55 0.1]};
    for j = 1:3
        m = NaN(size(D));  lo = m;  hi = m;
        for k = 1:numel(D)
            v = [];  nc = 0;
            for sd = 1:5
                fn = fullfile(od, local_file(AD(j), D(k), sd));  if ~exist(fn, 'file'); nc = nc + 1; continue; end
                S = load(fn);  if min(S.h_true) <= 1.0011; nc = nc + 1; continue; end
                v(end + 1) = mean(S.h_true(S.t > 5.5)) - HS(j); %#ok<AGROW>
            end
            if ~isempty(v); m(k) = mean(v);  lo(k) = min(v);  hi(k) = max(v); end
            if nc > 0; text(ax, D(k) + 0.05 * (j - 2), 0.75 - 0.08 * j, sprintf('%d/5 touch', nc), 'Color', C{j}, 'FontSize', 13, 'FontWeight', 'bold', 'HorizontalAlignment', 'center'); end
        end
        errorbar(ax, D + 0.06 * (j - 2), m, m - lo, hi - m, 'o-', 'Color', C{j}, 'LineWidth', 2.2, 'MarkerFaceColor', C{j}, 'DisplayName', sprintf('a_d = %.2f  (h^* = %.2f R)', AD(j), HS(j)));
    end
    yline(ax, 0, '-', 'Color', [0.85 0 0], 'LineWidth', 1.5, 'HandleVisibility', 'off');
    xlabel(ax, '\Delta  (true wall moved toward the probe)  [R]', 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax, 'final  h - h^*  [R]', 'FontSize', FS, 'FontWeight', 'bold');
    xlim(ax, [-1.5 3.5]);  ylim(ax, [-0.8 0.8]);  xticks(ax, D);
    legend(ax, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');  set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);  grid(ax, 'off');
    out = fullfile(od, 'fig_mc_s9_cellmargin_summary.png');  exportgraphics(f, out, 'Resolution', 120);  fprintf('figure: %s\n', out);
end

function fn = local_file(ad, DE, sd)
    tg = strrep(strrep(sprintf('_cm_%03d_100_%+g_s%d', round(100 * ad), DE, sd), '+', 'p'), '-', 'm');
    fn = sprintf('gainreg_cell_tau100_T6%s.mat', tg);
end
