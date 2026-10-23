% STATUS: SCRATCH -- 2026-10-19 (user): bridge from "the a law asks for a speed the a' state cannot follow" to the u law.
%   Commanded approach speed of each law with the TRUE a and a' (property of the law, no estimator), plane a_d 0.07 (h* 1.078 R):
%     a law:  v = (a - a_d) / (a' tau)              u law:  v = (u - u_d) / (u' tau) ~ (h - h*) / tau
%   against the speed both estimators were validated at (the 1 s constant-velocity descent, 21.1 R/s).
%   Output test_results/temp_1009_ws/fig_mc_law_speed.png.
function plot_mc_law_speed()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'test_results', 'temp_1009_ws');  FS = 20;  LFS = 15;  AXLW = 2.2;  AB = char(257);
    ad = 0.07;  hs = fzero(@(x) local_a(x) - ad, [1.001 30]);  h = linspace(hs + 0.05, 22.2, 500)';
    a = local_a(h);  ap = local_ap(h);  u = 1 ./ (1 - a);  up = ap ./ (1 - a).^2;  ud = 1 / (1 - ad);  vval = (22.2 - 1.10) / 1.0;
    f = figure('Position', [40 40 1500 800], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');  ax = axes(f);  hold on; box on;
    plot(ax, h, (a - ad) ./ (ap * 0.1), '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 3.5, 'DisplayName', [AB ' law, \tau = 0.1 s']);
    plot(ax, h, (u - ud) ./ (up * 0.1), '--', 'Color', [0 0.25 0.85], 'LineWidth', 3.0, 'DisplayName', 'u law, \tau = 0.1 s');
    plot(ax, h, (u - ud) ./ (up * 1.0), '-', 'Color', [0 0.25 0.85], 'LineWidth', 3.5, 'DisplayName', 'u law, \tau = 1 s');
    yline(ax, vval, ':', 'Color', 'k', 'LineWidth', 2.5, 'DisplayName', 'validated speed (1 s descent, 21 R/s)');
    set(ax, 'YScale', 'log');  xlim(ax, [1 23]);  ylim(ax, [0.05 1e4]);
    xlabel(ax, 'h [R]', 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax, 'commanded speed [R/s]', 'FontSize', FS, 'FontWeight', 'bold');
    lg = legend(ax, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'NumColumns', 2);
    lg.Title.String = sprintf('plane, %s_d = %.2f (h* = %.3f R), true %s and %s'' in the law', AB, ad, hs, AB, AB);
    set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax, 'off');
    for hq = [22.2 10 5 2]
        [~, i] = min(abs(h - hq));
        fprintf('h %5.1f | a law tau 0.1: %7.1f R/s | u law tau 0.1: %6.1f | u law tau 1: %5.1f | validated %.1f\n', h(i), (a(i) - ad) / (ap(i) * 0.1), (u(i) - ud) / (up(i) * 0.1), (u(i) - ud) / up(i), vval);
    end
    out = fullfile(od, 'fig_mc_law_speed.png');  exportgraphics(f, out, 'Resolution', 150);  close(f);  fprintf('figure: %s\n', out);
end

function ap = local_ap(h)
    d = 1e-4;  ap = (local_a(h + d) - local_a(h - d)) / (2 * d);
end

function a = local_a(h)
    a = zeros(size(h));  for i = 1:numel(h); [~, c] = calc_correction_functions(h(i)); a(i) = 1 / c; end
end
