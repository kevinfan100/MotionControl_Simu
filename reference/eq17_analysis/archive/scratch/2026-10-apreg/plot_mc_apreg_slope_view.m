% STATUS: SCRATCH -- 2026-10-18 (user): why the a' state does not follow the designed tau. u-law regulation, tau 0.5 s, seed 1, 4 s,
%   a' state (blue) vs w_s state (orange); files gainreg_<wall>_tau500_T4_sl{A,W}.mat (run_mc_s9_gainreg, see run_mc_apreg_u settings).
%   Rows: (1) u = 1/(1 - a_true) and the designed u_d + (u0 - u_d) e^(-t/tau) (black dashed), u_d grey dotted;
%         (2) slope the estimator hands the law, a-bar' = a_prime_out / a_nom (= diag.a_prime_bar), against the TRUE HEIGHT (x, linear, wall at
%             the right), so the true slope a-bar'(h) is one red curve for both runs;
%   Columns plane a_d 0.07 | cell a_d 0.60. Output test_results/temp_1009_ws/fig_mc_apreg_slope_view.png.
function plot_mc_apreg_slope_view()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'test_results', 'temp_1009_ws');  pc = physical_constants();  a_nom = pc.Ts / pc.gamma_N;  tau = 0.5;
    cT = [0.85 0 0];  cA = [0 0.25 0.85];  cW = [0.95 0.5 0.05];  FS = 20;  LFS = 15;  AXLW = 2.2;  AB = char(257);  HAT = char(770);
    C = {'plane', 0.07; 'cell', 0.60};
    f = figure('Position', [40 40 1900 1200], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:2
        w = C{ic, 1};  A = load(fullfile(od, sprintf('gainreg_%s_tau500_T4_slA.mat', w)));  W = load(fullfile(od, sprintf('gainreg_%s_tau500_T4_slW.mat', w)));
        t = A.t;  ud = 1 / (1 - C{ic, 2});  ua = 1 ./ (1 - A.a_true);  uw = 1 ./ (1 - W.a_true);
        apA = A.ap_hat / a_nom;  apW = W.ap_hat / a_nom;  ptA = local_ap(max(A.h_true, 1.001), w);  ptW = local_ap(max(W.h_true, 1.001), w);
        ax = nexttile(tl, ic);  hold on; box on;
        plot(ax, t, ud + (ua(1) - ud) * exp(-t / tau), '--', 'Color', 'k', 'LineWidth', 2.0, 'DisplayName', 'designed, e^{-t/\tau}');
        plot(ax, t, ua, '-', 'Color', cA, 'LineWidth', 2.2, 'DisplayName', [AB ''' state']);
        plot(ax, t, uw, '-', 'Color', cW, 'LineWidth', 2.2, 'DisplayName', 'w_s state');
        yline(ax, ud, ':', 'Color', [0.4 0.4 0.4], 'LineWidth', 2.0, 'DisplayName', 'u_d');
        ylim(ax, [0 21]);  lg = legend(ax, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'NumColumns', 2);
        lg.Title.String = sprintf('%s wall, %s_d = %.2f, seed 1, \\tau = %.1f s', w, AB, C{ic, 2}, tau);
        if ic == 1; ylabel(ax, ['u = 1/(1 - ' AB ')'], 'FontSize', FS, 'FontWeight', 'bold'); end
        ax2 = nexttile(tl, 2 + ic);  hold on; box on;            % 10-18: x = true height, so the true slope is ONE curve for both runs
        hg = logspace(log10(1.02), log10(22.2), 400)';
        plot(ax2, hg, local_ap(hg, w), '-', 'Color', cT, 'LineWidth', 4.0, 'DisplayName', [AB ''' true']);
        L = t <= 2.5;  y = apA;  y(y <= 0) = NaN;  plot(ax2, A.h_true(L), y(L), '-', 'Color', cA, 'LineWidth', 1.6, 'DisplayName', [AB HAT ''', ' AB ''' state']);
        y = apW;  y(y <= 0) = NaN;  plot(ax2, W.h_true(L), y(L), '-', 'Color', cW, 'LineWidth', 1.6, 'DisplayName', [AB HAT ''', w_s state']);
        set(ax2, 'YScale', 'log');  xlim(ax2, [1 23]);  xticks(ax2, [1 5 10 15 20]);  ylim(ax2, [1e-3 3]);
        legend(ax2, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        xlabel(ax2, 'h [R]  (true height)', 'FontSize', FS, 'FontWeight', 'bold');
        if ic == 1; ylabel(ax2, [AB ''''], 'FontSize', FS + 2, 'FontWeight', 'bold'); end
        xlim(ax, [0 2.5]);  xlabel(ax, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        for a = [ax ax2]; set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(a, 'off'); end
        BANDS = [5 22.3; 2 5; 1 2];
        for ib = 1:3
            LA = A.h_true > BANDS(ib, 1) & A.h_true <= BANDS(ib, 2) & t <= 2.5;  LW = W.h_true > BANDS(ib, 1) & W.h_true <= BANDS(ib, 2) & t <= 2.5;
            fprintf('%-5s h %4.1f-%4.1f R | a''-hat / a'' true: a'' state median %.2f (%%neg %2.0f) | w_s state median %.2f\n', w, BANDS(ib, 1), BANDS(ib, 2), ...
                    median(apA(LA) ./ ptA(LA)), 100 * mean(apA(LA) <= 0), median(apW(LW) ./ ptW(LW)));
        end
    end
    out = fullfile(od, 'fig_mc_apreg_slope_view.png');  exportgraphics(f, out, 'Resolution', 150);  close(f);  fprintf('figure: %s\n', out);
end

function ap = local_ap(h, wall)
    d = 1e-4;  ap = (local_a(h + d, wall) - local_a(h - d, wall)) / (2 * d);
end

function a = local_a(h, wall)
    if strcmp(wall, 'cell'); a = 1 ./ cell_cperp_delta(h); return; end
    a = zeros(size(h));  for i = 1:numel(h); [~, c] = calc_correction_functions(h(i)); a(i) = 1 / c; end
end
