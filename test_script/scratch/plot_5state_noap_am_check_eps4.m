% PURPOSE (2026-10-02, user: "matching the dx_r figure, draw a_true, a_hat, a_m for the two Q, two rows"): companion of
%   fig_noap_am_check_eps_td5_v3 -- same runs (plane, T_d = 5 s, stop at 5.5 s), same seed 1, same window 5.0-6.5 s.
%   Rows epsilon = 0 | 1e-5, same y axis: a_true (red), a_hat (blue), a_m (light blue), all / a_nom.
%   Reads amchk_plane_td5_<arm>_s1.mat.   STATUS: SCRATCH
function plot_5state_noap_am_check_eps4()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    pc = physical_constants();  a_nom = pc.Ts / pc.gamma_N;  WIN = [5.0 6.5];  YL = [0 1.0];
    ARMS = {'q0', '\epsilon = 0'; 'eps', '\epsilon = Q_{44}/R_2 = 10^{-5}'};
    FS = 18;  LFS = 12;  AXLW = 2.0;
    f = figure('Position', [40 40 1400 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for m = 1:2
        r = load(fullfile(id, sprintf('amchk_plane_td5_%s_s1.mat', ARMS{m, 1})));
        t = r.t;  k = t >= WIN(1) & t <= WIN(2);
        ax = nexttile(tl, m); hold(ax, 'on'); box(ax, 'on');
        hm = plot(ax, t(k), r.a_m(k) / a_nom, '-', 'Color', [0.55 0.75 1.0], 'LineWidth', 0.8, 'DisplayName', 'a_m');
        ht = plot(ax, t(k), r.a_true(k) / a_nom, '-', 'Color', [0.8 0 0], 'LineWidth', 2.4, 'DisplayName', 'a_{true}');
        hh = plot(ax, t(k), r.a_hat(k) / a_nom, '-', 'Color', [0 0.2 0.9], 'LineWidth', 2.2, 'DisplayName', ['a_{hat}, ' ARMS{m, 2}]);
        plot(ax, [5.5 5.5], YL, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2);
        xlim(ax, WIN);  ylim(ax, YL);  ylabel(ax, 'a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
        if m == 2; xlabel(ax, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend(ax, [ht hh hm], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');  grid(ax, 'off');
        h = t > 6.0;
        fprintf('%-4s seed 1, t > 6 s: a_true %.3f  a_hat %.3f  a_m %.3f (a_m/a_true %.2f)\n', ARMS{m, 1}, mean(r.a_true(h)) / a_nom, ...
            mean(r.a_hat(h)) / a_nom, mean(r.a_m(h)) / a_nom, mean(r.a_m(h)) / mean(r.a_true(h)));
    end
    out = fullfile(fd, 'fig_noap_am_check_eps_td5_v4.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
