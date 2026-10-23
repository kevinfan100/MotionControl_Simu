% STATUS: SCRATCH -- 2026-10-16: figure for probe_apdiff_desc_l1 (constant-velocity descent 22.2 -> 1.10 R in 1 s + 0.2 s hold), seed 1.
%   Columns plane | cell. Row 1: a-bar_z true (red), Layer-1 a_hat (light blue), a_hat of the a'-state run (blue).
%   Row 2 (log scale, negative values not drawn): true slope (red), a' state (blue), window difference N = 50 (orange),
%   low-pass difference N = 50 (green), both from the Layer-1 a_hat. Output test_results/temp_1009_ws/fig_apdiff_desc_l1.png.
function plot_apdiff_desc_l1(seed, N)
    if nargin < 1; seed = 1; end
    if nargin < 2; N = 50; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(genpath(fullfile(root, 'model')));  addpath(here);
    od = fullfile(root, 'test_results', 'temp_1009_ws');  WL = {'plane', 'cell'};
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cL = [0.45 0.7 1];  cO = [0.95 0.55 0.1];  cG = [0.1 0.6 0.2];  FS = 18;  LFS = 14;  AB = char(257);  HAT = char(770);
    f = figure('Position', [40 40 2000 1100], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for iw = 1:2
        A = load(fullfile(od, sprintf('apcmp_apP002F400H110_%s_desc.mat', WL{iw})));  r = load(fullfile(od, sprintf('apdiff_desc_l1_%s_s%d.mat', WL{iw}, seed)));
        ax = nexttile(tl, iw);  hold on; box on;
        plot(r.t, r.a_true, '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', [AB '_z']);
        plot(r.t, r.a_hat, '-', 'Color', cL, 'LineWidth', 1.5, 'DisplayName', [AB HAT '_z first order']);
        plot(A.t, A.a_hat(:, seed), '-', 'Color', cE, 'LineWidth', 1.5, 'DisplayName', [AB HAT '_z, ' AB ''' state']);
        ylabel([AB '_z']);  ylim([0 1]);  xlim([0 1.2]);  legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS);
        text(0.03, 0.08, WL{iw}, 'Units', 'normalized', 'FontSize', FS, 'FontWeight', 'bold');  set(ax, 'FontSize', FS, 'FontWeight', 'bold');
        ax = nexttile(tl, 2 + iw);  hold on; box on;
        apd = local_pos(local_win(r.a_hat, r.h_d, N));  apl = local_pos(local_lpf(r.a_hat, r.h_d, N));  aps = local_pos(-A.w_hat(:, seed));
        plot(r.t, apd, '-', 'Color', cO, 'LineWidth', 1.2, 'DisplayName', sprintf('difference, N = %d', N));
        plot(r.t, apl, '-', 'Color', cG, 'LineWidth', 1.2, 'DisplayName', sprintf('low-pass, N = %d', N));
        plot(A.t, aps, '-', 'Color', cE, 'LineWidth', 2, 'DisplayName', [AB ''' state']);
        plot(r.t, local_ap(max(r.h, 1.001), WL{iw}), '-', 'Color', cT, 'LineWidth', 3, 'DisplayName', [AB ''' true']);
        set(ax, 'YScale', 'log');  ylim([1e-4 3]);  xlim([0 1.2]);  ylabel([AB '''']);  xlabel('Time (sec)');
        legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'NumColumns', 2);  set(ax, 'FontSize', FS, 'FontWeight', 'bold');
    end
    exportgraphics(f, fullfile(od, 'fig_apdiff_desc_l1.png'), 'Resolution', 150);  close(f);
end

function x = local_pos(x); x(x <= 0) = NaN; end

function ap = local_win(a, hd, N)
    n = numel(a);  ap = NaN(n, 1);  s = NaN;
    for k = N+1:n; dh = hd(k) - hd(k-N); if abs(dh) > 1e-3; s = (a(k) - a(k-N)) / dh; end; ap(k) = s; end
end

function ap = local_lpf(a, hd, N)
    n = numel(a);  ap = NaN(n, 1);  b = 2 / (N + 1);  num = 0;  den = 0;  s = NaN;
    for k = 2:n
        num = (1 - b) * num + b * (a(k) - a(k-1));  den = (1 - b) * den + b * (hd(k) - hd(k-1));
        if k > N && abs(den) > 1e-3 / N; s = num / den; end;  ap(k) = s;
    end
end

function ap = local_ap(h, wall)
    d = 1e-4;  ap = (local_a(h + d, wall) - local_a(h - d, wall)) / (2 * d);
end

function a = local_a(h, wall)
    if strcmp(wall, 'cell'); a = 1 ./ cell_cperp_delta(h); return; end
    a = zeros(size(h));  for i = 1:numel(h); [~, c] = calc_correction_functions(h(i)); a(i) = 1 / c; end
end
