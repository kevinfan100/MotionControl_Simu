% STATUS: SCRATCH -- 2026-10-16: closed-loop figure for run_apff_desc_l1 next to the a'-state run, constant-velocity descent
%   22.2 -> 1.10 R in 1 s + 0.2 s hold, seed 1. Columns plane | cell. Rows: a-bar_z (true red; first order without a' light blue;
%   first order + low-pass difference a' orange; a' state blue); e_a = a_true - a_hat (same colours, + first order with the true a'
%   grey); slope a-bar' on a log scale (true red, low-pass difference orange, state blue; negative values not drawn).
%   Output test_results/temp_1009_ws/fig_apff_desc_l1.png.
function plot_apff_desc_l1(seed)
    if nargin < 1; seed = 1; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'test_results', 'temp_1009_ws');  WL = {'plane', 'cell'};
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cL = [0.45 0.7 1];  cO = [0.95 0.55 0.1];  cK = [0.5 0.5 0.5];  FS = 18;  LFS = 14;  AB = char(257);  HAT = char(770);
    f = figure('Position', [40 40 2000 1500], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for iw = 1:2
        A = load(fullfile(od, sprintf('apcmp_apP002F400H110_%s_desc.mat', WL{iw})));
        r0 = load(fullfile(od, sprintf('apff_off_%s_s%d.mat', WL{iw}, seed)));  rl = load(fullfile(od, sprintf('apff_lpf_%s_s%d.mat', WL{iw}, seed)));
        rt = load(fullfile(od, sprintf('apff_true_%s_s%d.mat', WL{iw}, seed)));  t = r0.t;
        ax = nexttile(tl, iw);  hold on; box on;
        plot(t, r0.a_true, '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', [AB '_z']);
        plot(t, r0.a_hat, '-', 'Color', cL, 'LineWidth', 1.5, 'DisplayName', 'first order');
        plot(t, rl.a_hat, '-', 'Color', cO, 'LineWidth', 1.5, 'DisplayName', ['first order + low-pass ' AB '''']);
        plot(A.t, A.a_hat(:, seed), '-', 'Color', cE, 'LineWidth', 1.5, 'DisplayName', [AB ''' state']);
        ylabel([AB '_z']);  ylim([0 1]);  xlim([0 1.2]);  legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'NumColumns', 2);
        text(0.03, 0.08, WL{iw}, 'Units', 'normalized', 'FontSize', FS, 'FontWeight', 'bold');  set(ax, 'FontSize', FS, 'FontWeight', 'bold');
        ax = nexttile(tl, 2 + iw);  hold on; box on;
        plot(t, r0.a_true - r0.a_hat, '-', 'Color', cL, 'LineWidth', 1.5, 'DisplayName', 'first order');
        plot(t, rl.a_true - rl.a_hat, '-', 'Color', cO, 'LineWidth', 1.5, 'DisplayName', ['+ low-pass ' AB '''']);
        plot(t, rt.a_true - rt.a_hat, '-', 'Color', cK, 'LineWidth', 1.5, 'DisplayName', ['+ true ' AB '''']);
        plot(A.t, A.a_true(:, seed) - A.a_hat(:, seed), '-', 'Color', cE, 'LineWidth', 1.5, 'DisplayName', [AB ''' state']);
        yline(0, 'Color', [0.3 0.3 0.3], 'HandleVisibility', 'off');
        ylabel(['e_{' AB '}']);  ylim([-0.2 0.2]);  xlim([0 1.2]);  legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'NumColumns', 2);
        set(ax, 'FontSize', FS, 'FontWeight', 'bold');
        ax = nexttile(tl, 4 + iw);  hold on; box on;
        pos = @(x) x .* (x > 0) ./ (x > 0);
        plot(t, pos(rl.ap_hat), '-', 'Color', cO, 'LineWidth', 1.2, 'DisplayName', 'low-pass difference');
        plot(A.t, pos(-A.w_hat(:, seed)), '-', 'Color', cE, 'LineWidth', 2, 'DisplayName', [AB ''' state']);
        plot(t, local_ap(max(r0.h, 1.001), WL{iw}), '-', 'Color', cT, 'LineWidth', 3, 'DisplayName', [AB ''' true']);
        set(ax, 'YScale', 'log');  ylim([1e-4 3]);  xlim([0 1.2]);  ylabel([AB '''']);  xlabel('Time (sec)');
        legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS);  set(ax, 'FontSize', FS, 'FontWeight', 'bold');
    end
    exportgraphics(f, fullfile(od, 'fig_apff_desc_l1.png'), 'Resolution', 150);  close(f);
end

function ap = local_ap(h, wall)
    d = 1e-4;  ap = (local_a(h + d, wall) - local_a(h - d, wall)) / (2 * d);
end

function a = local_a(h, wall)
    if strcmp(wall, 'cell'); a = 1 ./ cell_cperp_delta(h); return; end
    a = zeros(size(h));  for i = 1:numel(h); [~, c] = calc_correction_functions(h(i)); a(i) = 1 / c; end
end
