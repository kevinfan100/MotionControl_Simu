% STATUS: SCRATCH -- 2026-10-16: figure for probe_apdiff_offline_l1 (Step 0, offline a'-difference of the Layer-1 estimator), seed 1.
%   Columns plane | cell. Rows: h_d [R]; a-bar_z true (red) and Layer-1 estimate (blue); slope a-bar' true (red) and the difference
%   slope with N = 10 (light blue) and N = 100 (blue). Output test_results/temp_1009_ws/fig_apdiff_offline_l1.png.
function plot_apdiff_offline_l1(seed)
    if nargin < 1; seed = 1; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'test_results', 'temp_1009_ws');  S = load(fullfile(od, 'apdiff_offline_l1.mat'));
    WALLS = {'plane', 'cell'};  cT = [0.85 0 0];  cE = [0 0.25 0.85];  cL = [0.45 0.7 1];  FS = 18;  LFS = 14;  AB = char(257);  HAT = char(770);
    f = figure('Position', [40 40 2000 1300], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for iw = 1:2
        r = S.D.(WALLS{iw})(seed);  t = r.t;  h = max(r.h, 1.001);  apt = local_ap(h, WALLS{iw});
        ax = nexttile(tl, iw);  hold on; box on;
        plot(t, r.h_d, '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', 'h_d');  plot(t, h, '--', 'Color', cE, 'LineWidth', 1.5, 'DisplayName', 'h');
        ylabel('h [R]');  ylim([0 23]);  xlim([0 3]);  legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS);
        text(0.97, 0.9, WALLS{iw}, 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', FS, 'FontWeight', 'bold');
        set(ax, 'FontSize', FS, 'FontWeight', 'bold');
        ax = nexttile(tl, 2 + iw);  hold on; box on;
        plot(t, r.a_true, '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', [AB '_z']);  plot(t, r.a_hat, '-', 'Color', cE, 'LineWidth', 1.5, 'DisplayName', [AB HAT '_z']);
        ylabel([AB '_z']);  ylim([0 1]);  legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS);  set(ax, 'FontSize', FS, 'FontWeight', 'bold');
        ax = nexttile(tl, 4 + iw);  hold on; box on;
        plot(t, local_diff(r, 10), '-', 'Color', cL, 'LineWidth', 1, 'DisplayName', 'difference, N = 10');
        plot(t, local_diff(r, 100), '-', 'Color', cE, 'LineWidth', 1.5, 'DisplayName', 'difference, N = 100');
        plot(t, apt, '-', 'Color', cT, 'LineWidth', 3, 'DisplayName', [AB ''' true']);  yline(0, 'Color', [0.5 0.5 0.5], 'HandleVisibility', 'off');
        ylabel([AB '''']);  ylim([-0.4 1.2]);  xlabel('Time (sec)');
        legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS);  set(ax, 'FontSize', FS, 'FontWeight', 'bold');
    end
    exportgraphics(f, fullfile(od, 'fig_apdiff_offline_l1.png'), 'Resolution', 150);  close(f);
end

function apd = local_diff(r, N)
    n = numel(r.t);  apd = NaN(n, 1);  dh = r.h_d(N+1:end) - r.h_d(1:end-N);  da = r.a_hat(N+1:end) - r.a_hat(1:end-N);
    v = abs(dh) > 1e-6;  tmp = NaN(size(dh));  tmp(v) = da(v) ./ dh(v);  apd(N+1:end) = tmp;
end

function ap = local_ap(h, wall)
    d = 1e-4;  ap = (local_a(h + d, wall) - local_a(h - d, wall)) / (2 * d);
end

function a = local_a(h, wall)
    if strcmp(wall, 'cell'); a = 1 ./ cell_cperp_delta(h); return; end
    a = zeros(size(h));  for i = 1:numel(h); [~, c] = calc_correction_functions(h(i)); a(i) = 1 / c; end
end
