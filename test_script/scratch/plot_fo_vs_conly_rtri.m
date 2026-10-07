% PURPOSE (2026-10-05, user: "compare against this set" = the first-order seed-1 figure fig_noap_rtri_td<Td>_f<f>_s1):
%   one figure per wall, columns = first order (eps = Q44/R2 = 1e-5) | c-only final (sqrt P_fix 0.15 new / 0.05 revisit),
%   same trajectory (T_d descent + 2 s triangle at f), seed 1, rows a_bar | e_a | delta_x_m [um], rows shared y.
%   Reads test_results/temp_5state_noap/rtri_<wall>_td<Td>_f<f>_s1.mat (plot_5state_noap_rtri) and
%   test_results/temp_plaw_walls/f1p2_<wall>_PI_s1_rn15_05_rtri_f<f>_td<Td>.mat (run_plaw_walls, seeds 1).   STATUS: SCRATCH
function plot_fo_vs_conly_rtri(wall, f, Td)
    if nargin < 2; f = 5; end
    if nargin < 3; Td = 0.5; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    pc = physical_constants();  a_nom = pc.Ts / pc.gamma_N;
    tg = strrep(sprintf('td%g_f%g', Td, f), '.', 'p');  tc = strrep(sprintf('_rtri_f%g_td%g', f, Td), '.', 'p');
    F = load(fullfile(root, 'test_results', 'temp_5state_noap', sprintf('rtri_%s_%s_s1.mat', wall, tg)));
    C = load(fullfile(root, 'test_results', 'temp_plaw_walls', sprintf('f1p2_%s_PI_s1_rn15_05%s.mat', wall, tc)));
    D = {struct('t', F.t, 'at', F.a_true / a_nom, 'ah', F.a_hat / a_nom, 'am', F.a_m / a_nom, 'dx', F.dxm, ...
                'lab', 'first order, \epsilon = Q_{44}/R_2 = 10^{-5}'), ...
         struct('t', C.t, 'at', C.a_true(:, 1), 'ah', C.a_hat(:, 1), 'am', C.a_m(:, 1), 'dx', C.dxm(:, 1), ...
                'lab', 'c-only, \surdP_{fix} = 0.15 new / 0.05 revisit')};
    AB = char(257);  HAT = char(770);
    COL_TRUE = [0.8 0 0];  C_EST = [0 0.2 0.9];  C_MEAS = [0.55 0.75 1.0];  FS = 18;  LFS = 12;  AXLW = 2.0;
    fig = figure('Position', [40 40 1700 1150], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(fig, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');  ax = gobjects(3, 2);
    for m = 1:2
        d = D{m};  t = d.t;  e = d.at - d.ah;
        for q = 1:3; ax(q, m) = nexttile(tl, (q - 1) * 2 + m); hold(ax(q, m), 'on'); end
        hm = plot(ax(1, m), t, d.am, '-', 'Color', C_MEAS, 'LineWidth', 0.6, 'DisplayName', [AB '_m']);
        ht = plot(ax(1, m), t, d.at, '-', 'Color', COL_TRUE, 'LineWidth', 2.4, 'DisplayName', [AB '_{true}']);
        hh = plot(ax(1, m), t, d.ah, '-', 'Color', C_EST, 'LineWidth', 1.8, 'DisplayName', [AB HAT]);
        ylim(ax(1, m), [-0.2 1.6]);
        lg = legend(ax(1, m), [ht hh hm], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'AutoUpdate', 'off');
        lg.Title.String = sprintf('%s wall, seed 1, %s', wall, d.lab);
        plot(ax(2, m), t, e, '-', 'Color', C_EST, 'LineWidth', 1.8);  yline(ax(2, m), 0, '-', 'Color', [0.6 0.6 0.6]);
        plot(ax(3, m), t, d.dx, '-', 'Color', C_MEAS, 'LineWidth', 0.8);  yline(ax(3, m), 0, '-', 'Color', [0.6 0.6 0.6]);
        xlabel(ax(3, m), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        kd = t >= 0.8 * Td & t < Td;  ko = t >= Td;
        fprintf('%-5s %-48s | e_a end of descent %+.3f | triangle mean %+.4f, mean|e_a| %.4f, worst %+.3f | dx_m rms triangle %.4f um\n', ...
            wall, d.lab, mean(e(kd)), mean(e(ko)), mean(abs(e(ko))), e(find(abs(e) == max(abs(e(ko))) & ko, 1)), rms(d.dx(ko)));
    end
    ylabel(ax(1, 1), AB, 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax(2, 1), ['e_{' AB '}'], 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax(3, 1), '\delta x_m  [\mum]', 'FontSize', FS, 'FontWeight', 'bold');
    for q = 2:3; linkaxes(ax(q, :), 'y'); end
    for q = 1:3
        for m = 1:2
            xlim(ax(q, m), [0 D{m}.t(end)]);  set(ax(q, m), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');  grid(ax(q, m), 'off');
        end
    end
    out = fullfile(fd, sprintf('fig_fo_vs_conly_%s_%s_s1.png', wall, tg));
    exportgraphics(fig, out, 'Resolution', 150);  close(fig);  fprintf('figure: %s\n', out);
end
