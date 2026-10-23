% STATUS: SCRATCH -- 2026-10-18 (user): the regulation law as a ruler. At h = 22.2 R the law takes the slope there and extends a
%   straight line to the target. Left: a-bar(h) (plane) with that line; right: u = 1/(1 - a-bar) with that line. Black dot = target.
%   Output test_results/temp_1009_ws/fig_mc_ruler.png.
function plot_mc_ruler()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'test_results', 'temp_1009_ws');  FS = 20;  LFS = 15;  AXLW = 2.2;  AB = char(257);  cT = [0.85 0 0];  cE = [0 0.25 0.85];
    h = linspace(1.02, 25, 500)';  a = local_a(h);  h0 = 22.2;  a0 = local_a(h0);  ap0 = (local_a(h0 + 1e-4) - local_a(h0 - 1e-4)) / 2e-4;
    ad = 0.07;  hs = 1.078;  u = 1 ./ (1 - a);  u0 = 1 / (1 - a0);  up0 = ap0 / (1 - a0)^2;  ud = 1 / (1 - ad);
    f = figure('Position', [40 40 1800 650], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = nexttile(tl, 1);  hold on; box on;
    plot(ax, h, a, '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', [AB '(h), plane']);
    hl = linspace(-400, h0, 50);  plot(ax, hl, a0 + ap0 * (hl - h0), '--', 'Color', cE, 'LineWidth', 2.5, 'DisplayName', 'straight line from the slope at 22.2 R');
    plot(ax, hs, ad, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 10, 'DisplayName', 'target');  yline(ax, ad, ':', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.5, 'HandleVisibility', 'off');
    xlim(ax, [0 25]);  ylim(ax, [0 1]);
    legend(ax, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    xlabel(ax, 'h [R]', 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax, AB, 'FontSize', FS + 2, 'FontWeight', 'bold');
    ax2 = nexttile(tl, 2);  hold on; box on;
    plot(ax2, h, u, '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', ['u = 1/(1 - ' AB '), plane']);
    hl = linspace(0, h0, 50);  plot(ax2, hl, u0 + up0 * (hl - h0), '--', 'Color', cE, 'LineWidth', 2.5, 'DisplayName', 'straight line from the slope at 22.2 R');
    plot(ax2, hs, ud, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 10, 'DisplayName', 'target');
    xlim(ax2, [0 25]);  ylim(ax2, [0 22]);
    legend(ax2, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    xlabel(ax2, 'h [R]', 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax2, 'u', 'FontSize', FS + 2, 'FontWeight', 'bold');
    for a_ = [ax ax2]; set(a_, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(a_, 'off'); end
    fprintf('a: at 22.2 R a = %.3f, slope %.4f, line reaches a_d at h = %.0f R (true 1.078)\n', a0, ap0, h0 - (a0 - ad) / ap0);
    fprintf('u: at 22.2 R u = %.2f, slope %.3f, line reaches u_d at h = %.2f R (true 1.078)\n', u0, up0, h0 - (u0 - ud) / up0);
    fprintf('u slope check: chain rule %.3f | finite difference (u(22.2) - u(10)) / 12.2 = %.3f\n', up0, (u0 - 1 / (1 - local_a(10))) / 12.2);
    out = fullfile(od, 'fig_mc_ruler.png');  exportgraphics(f, out, 'Resolution', 150);  close(f);  fprintf('figure: %s\n', out);
end

function a = local_a(h)
    a = zeros(size(h));  for i = 1:numel(h); [~, c] = calc_correction_functions(h(i)); a(i) = 1 / c; end
end
