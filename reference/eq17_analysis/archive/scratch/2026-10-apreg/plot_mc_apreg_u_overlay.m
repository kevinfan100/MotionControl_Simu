% STATUS: SCRATCH -- 2026-10-18 (user): u-law regulation response in u = 1/(1 - a), overlaid: a' state (blue) vs w_s state (orange),
%   true u(t) = 1/(1 - a_true(t)); black dashed = the designed response u_d + (u(0) - u_d) e^(-t/tau); grey dotted = u_d.
%   Columns plane a_d 0.07 (seed 1) | cell a_d 0.60 (seed 2); rows tau 0.5 | 1.0 s. Output test_results/temp_1009_ws/fig_mc_apreg_u_overlay.png.
function plot_mc_apreg_u_overlay()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_1009_ws');
    cA = [0 0.25 0.85];  cW = [0.95 0.5 0.05];  FS = 20;  LFS = 15;  AXLW = 2.2;  AB = char(257);
    CASES = {'plane', '', 1; 'cell', '_ad060', 2};  TAUS = [0.5 1.0];
    f = figure('Position', [40 40 1900 1300], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for it = 1:2
        for ic = 1:2
            w = CASES{ic, 1};  seed = CASES{ic, 3};  tau = TAUS(it);
            A = load(fullfile(od, sprintf('apreg_stateu%03d_%s%s.mat', round(100 * tau), w, CASES{ic, 2})));
            W = load(fullfile(od, sprintf('apreg_wsu%03d_%s%s.mat', round(100 * tau), w, CASES{ic, 2})));
            t = A.t;  ud = 1 / (1 - A.a_d);  qa = find(A.seeds == seed);  qw = find(W.seeds == seed);
            ua = 1 ./ (1 - A.a_true(:, qa));  uw = 1 ./ (1 - W.a_true(:, qw));
            ax = nexttile(tl, (it - 1) * 2 + ic);  hold on; box on;
            hr = plot(ax, t, ud + (ua(1) - ud) * exp(-t / tau), '--', 'Color', 'k', 'LineWidth', 2.0, 'DisplayName', 'u_d + (u_0 - u_d) e^{-t/\tau}');
            ha = plot(ax, t, ua, '-', 'Color', cA, 'LineWidth', 2.2, 'DisplayName', [AB ''' state']);
            hw = plot(ax, t, uw, '-', 'Color', cW, 'LineWidth', 2.2, 'DisplayName', 'w_s state');
            hd = yline(ax, ud, ':', 'Color', [0.4 0.4 0.4], 'LineWidth', 2.0, 'DisplayName', 'u_d');
            xlim(ax, [0 4]);  ylim(ax, [0 21]);
            lg = legend(ax, [ha hw hr hd], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'NumColumns', 2);
            lg.Title.String = sprintf('%s wall, %s_d = %.2f, seed %d, \\tau = %.1f s', w, AB, A.a_d, seed, tau);
            set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax, 'off');
            if ic == 1; ylabel(ax, ['u = 1/(1 - ' AB ')'], 'FontSize', FS, 'FontWeight', 'bold'); end
            if it == 2; xlabel(ax, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
        end
    end
    out = fullfile(od, 'fig_mc_apreg_u_overlay.png');  exportgraphics(f, out, 'Resolution', 150);  close(f);  fprintf('figure: %s\n', out);
end
