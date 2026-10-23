% STATUS: SCRATCH -- 2026-10-15 (user): is estimating a' directly the same as differencing a_hat? and can the noise be tuned?
%   Constant-velocity descent 22.2 -> 1.10 R in 1 s + 0.2 s hold, seed 1, columns plane | cell (run_mc_apslot_compare 'desc').
%   Row 1 (log scale): true slope a'(h_true) (red); a' as a state, best height-scaled setting apP002F400H110 (blue, slot 7 = -w_hat);
%     bare difference of the SAME run's a_hat, (a_hat[k] - a_hat[k-N]) / (h_d[k] - h_d[k-N]), N = 100 steps (orange; negative values not drawn).
%   Row 2: a-bar_z true (red), a_hat with a fixed P_fix (apP002F050, light blue) and with P_fix scaled by 1/h_d^2 (apP002F400H110, blue).
function plot_mc_slope_direct_vs_diff()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'test_results', 'temp_1009_ws');  WL = {'plane', 'cell'};  N = 100;
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cL = [0.45 0.7 0.95];  cO = [0.95 0.55 0.1];  FS = 18;  LFS = 14;  AB = char(257);  HAT = char(770);
    f = figure('Position', [40 40 2000 1100], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for iw = 1:2
        A = load(fullfile(od, sprintf('apcmp_apP002F400H110_%s_desc.mat', WL{iw})));  U = load(fullfile(od, sprintf('apcmp_apP002F050_%s_desc.mat', WL{iw})));
        t = A.t;  q = 1;  hd = A.wd(:, q);  hd(1) = hd(2);  ah = A.a_hat(:, q);
        apt = local_ap(max(A.h_true(:, q), 1.001), WL{iw});  apk = -A.w_hat(:, q);
        apd = NaN(size(t));  k = (N + 1):numel(t);  dh = hd(k) - hd(k - N);  ok = abs(dh) > 1e-3;
        apd(k(ok)) = (ah(k(ok)) - ah(k(ok) - N)) ./ dh(ok);  apd(apd <= 0) = NaN;
        a1 = nexttile(tl, iw);  hold on; box on;
        plot(a1, t, apt, '-', 'Color', cT, 'LineWidth', 3.5, 'DisplayName', [AB ''' true']);
        plot(a1, t, apd, '-', 'Color', cO, 'LineWidth', 1.2, 'DisplayName', sprintf('difference of %s%s, %d steps', AB, HAT, N));
        plot(a1, t, max(apk, 1e-4), '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB '''' HAT ' as a state']);
        set(a1, 'YScale', 'log');  ylim(a1, [1e-3 3]);  xlim(a1, [0 t(end)]);
        text(a1, 0.03, 0.9, WL{iw}, 'Units', 'normalized', 'FontSize', LFS + 4, 'FontWeight', 'bold');
        a2 = nexttile(tl, 2 + iw);  hold on; box on;
        plot(a2, t, A.a_true(:, q), '-', 'Color', cT, 'LineWidth', 3.5, 'DisplayName', [AB '_z']);
        plot(a2, t, U.a_hat(:, q), '-', 'Color', cL, 'LineWidth', 1.2, 'DisplayName', [AB HAT '_z, fixed P_{fix}']);
        plot(a2, t, ah, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HAT '_z, P_{fix} \propto 1/h_d^2']);
        ylim(a2, [0 1.05]);  xlim(a2, [0 t(end)]);  xlabel(a2, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        if iw == 1; ylabel(a1, [AB ''''], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(a2, [AB '_z'], 'FontSize', FS, 'FontWeight', 'bold'); end
        for a = [a1 a2]; legend(a, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);  grid(a, 'off'); end
        d = t <= 1;  lr = @(x) mean(abs(log(x(d & isfinite(x) & x > 0) ./ apt(d & isfinite(x) & x > 0))));
        fprintf('%s seed 1 descent: mean |ln(est/true)| state %.2f, difference %.2f (difference positive in %.0f%% of steps)\n', ...
                WL{iw}, lr(apk), lr(apd), 100 * mean(isfinite(apd(d & t > N * 6.25e-4))));
    end
    out = fullfile(od, 'fig_mc_slope_direct_vs_diff.png');  exportgraphics(f, out, 'Resolution', 120);  fprintf('figure: %s\n', out);
end

function ap = local_ap(h, plant)
    d = 1e-4;  ap = (local_a(h + d, plant) - local_a(h - d, plant)) / (2 * d);
end

function a = local_a(h, plant)
    if strcmp(plant, 'cell'); a = 1 ./ cell_cperp_delta(h); return; end
    a = zeros(size(h));  for i = 1:numel(h); [~, c] = calc_correction_functions(h(i)); a(i) = 1 / c; end
end
