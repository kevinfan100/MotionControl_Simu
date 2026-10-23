% STATUS: SCRATCH -- 2026-10-18 (user): property of the regulation LAW alone (true a and a' handed to the law, no estimator).
%   Law family: make g(a) go to g(a_d) as e^(-t/tau) by one linearized step, Delta h = (1 - lambda_a)(g_d - g(a)) / (g'(a) a'(h)).
%   The step moves the true g by g(h + Delta h) - g(h); the fraction of the remaining gap closed per step is (1 - lambda_a) x
%   secant/local, so tau_eff / tau = local slope / secant slope = g_h(h) / ((g(h) - g_d)/(h - h*)), with g_h = dg/dh.
%   g = a (current a law) vs g = u = 1/(1 - a) (u law), plane a_d 0.07 and cell a_d 0.60, x = h (linear).
%   Output test_results/temp_1009_ws/fig_mc_law_teff.png.
function plot_mc_law_teff()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'test_results', 'temp_1009_ws');  FS = 20;  LFS = 15;  AXLW = 2.2;  AB = char(257);
    C = {'plane', 0.07; 'cell', 0.60};  cAl = [0.5 0.5 0.5];  cU = [0 0.25 0.85];
    f = figure('Position', [40 40 1800 650], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:2
        w = C{ic, 1};  ad = C{ic, 2};  hs = fzero(@(x) local_a(x, w) - ad, [1.001 30]);
        h = linspace(hs + 0.02, 22.2, 600)';  a = local_a(h, w);  ap = local_ap(h, w);  u = 1 ./ (1 - a);  ud = 1 / (1 - ad);
        ra = ap ./ ((a - ad) ./ (h - hs));  ru = (ap ./ (1 - a).^2) ./ ((u - ud) ./ (h - hs));
        ax = nexttile(tl, ic);  hold on; box on;
        plot(ax, h, ra, '-', 'Color', cAl, 'LineWidth', 3.0, 'DisplayName', [AB ' law']);
        plot(ax, h, ru, '-', 'Color', cU, 'LineWidth', 3.0, 'DisplayName', 'u law');
        yline(ax, 1, '--', 'Color', 'k', 'LineWidth', 1.8, 'DisplayName', 'designed');
        set(ax, 'YScale', 'log');  xlim(ax, [1 23]);  xticks(ax, [1 5 10 15 20]);  ylim(ax, [0.03 2]);  yticks(ax, [0.03 0.1 0.3 1 2]);
        lg = legend(ax, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        lg.Title.String = sprintf('%s wall, %s_d = %.2f (h* = %.3f R), true %s and %s'' in the law', w, AB, ad, hs, AB, AB);
        xlabel(ax, 'h [R]', 'FontSize', FS, 'FontWeight', 'bold');
        if ic == 1; ylabel(ax, '\tau_{eff} / \tau', 'FontSize', FS, 'FontWeight', 'bold'); end
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax, 'off');
        for hq = [22.2 10 5 3 2 1.5]
            [~, i] = min(abs(h - hq));  fprintf('%-5s h %5.1f | tau_eff/tau  a law %.3f | u law %.3f\n', w, h(i), ra(i), ru(i));
        end
    end
    out = fullfile(od, 'fig_mc_law_teff.png');  exportgraphics(f, out, 'Resolution', 150);  close(f);  fprintf('figure: %s\n', out);
end

function ap = local_ap(h, wall)
    d = 1e-4;  ap = (local_a(h + d, wall) - local_a(h - d, wall)) / (2 * d);
end

function a = local_a(h, wall)
    if strcmp(wall, 'cell'); a = 1 ./ cell_cperp_delta(h); return; end
    a = zeros(size(h));  for i = 1:numel(h); [~, c] = calc_correction_functions(h(i)); a(i) = 1 / c; end
end
