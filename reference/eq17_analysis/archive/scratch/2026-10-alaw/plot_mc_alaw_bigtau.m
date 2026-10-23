% STATUS: SCRATCH -- 2026-10-19 (user): the "bigger tau" option. a law with tau = 18.3 s (start speed = the validated 21.1 R/s), no speed
%   cap, slope gate off (thr 0: with tiny steps the gate would freeze the far-field slope), plane a_d 0.07, seed 1, 30 s; next to the
%   u law with tau = 1 s (run_mc_apreg_u, gate on). Black dashed = the a law with true a and a' (calc_alaw_bigtau_time).
%   Particle height h (log). Output test_results/temp_1009_ws/fig_mc_alaw_bigtau.png.
function plot_mc_alaw_bigtau()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'test_results', 'temp_1009_ws');  FS = 20;  LFS = 14;  AXLW = 2.2;  AB = char(257);
    cA = [0 0.25 0.85];  cW = [0.95 0.5 0.05];
    BA = load(fullfile(od, 'gainreg_plane_tau18300_T30_bt0AP.mat'));  BW = load(fullfile(od, 'gainreg_plane_tau18300_T30_bt0WS.mat'));
    UA = load(fullfile(od, 'apreg_stateu100_plane.mat'));  UW = load(fullfile(od, 'apreg_wsu100_plane.mat'));
    ad = 0.07;  d = 1e-4;  A = @(h) 1 / local_c(h);  tau = 18.3;  tt = 0:0.01:30;  hh = zeros(size(tt));  hh(1) = 22.2;
    for i = 2:numel(tt)
        h = hh(i - 1);  a = A(h);  ap = (A(h + d) - A(h - d)) / (2 * d);  hh(i) = h - (a - ad) / (ap * tau) * 0.01;
    end
    f = figure('Position', [40 40 1600 800], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');  ax = axes(f);  hold on; box on;
    plot(ax, tt, hh, '--', 'Color', 'k', 'LineWidth', 2.5, 'DisplayName', [AB ' law, \tau = 18.3 s, true ' AB ' and ' AB '''']);
    plot(ax, BW.t, BW.h_true, '-', 'Color', cW, 'LineWidth', 2.5, 'DisplayName', [AB ' law, \tau = 18.3 s, w_s state']);
    plot(ax, BA.t, BA.h_true, '-', 'Color', cA, 'LineWidth', 2.5, 'DisplayName', [AB ' law, \tau = 18.3 s, ' AB ''' state']);
    plot(ax, UW.t, UW.h_true(:, 1), ':', 'Color', cW, 'LineWidth', 3, 'DisplayName', 'u law, \tau = 1 s, w_s state');
    plot(ax, UA.t, UA.h_true(:, 1), ':', 'Color', cA, 'LineWidth', 3, 'DisplayName', ['u law, \tau = 1 s, ' AB ''' state']);
    yline(ax, 1.078, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.5, 'DisplayName', 'h*');
    set(ax, 'YScale', 'log');  xlim(ax, [0 30]);  ylim(ax, [1 25]);  yticks(ax, [1 1.5 2 3 5 10 20]);
    xlabel(ax, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax, 'h [R]  (particle)', 'FontSize', FS, 'FontWeight', 'bold');
    lg = legend(ax, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'NumColumns', 2);
    lg.Title.String = sprintf('plane, %s_d = 0.07 (h* = 1.078 R), seed 1, no speed cap', AB);
    set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax, 'off');
    out = fullfile(od, 'fig_mc_alaw_bigtau.png');  exportgraphics(f, out, 'Resolution', 150);  close(f);  fprintf('figure: %s\n', out);
end

function c = local_c(h)
    [~, c] = calc_correction_functions(h);
end
