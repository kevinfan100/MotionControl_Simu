% FORK OF test_script/scratch/plot_conly_rtri_pair.m @ uncommitted (ap model, 10-05) | PURPOSE: same house style (seed 1, rows a / e_a / dx_m),
%   but columns = walls (plane | cell) for ONE design, so each design gets its own figure | EXPIRES: when the house style settles and
%   plot_conly_rtri_pair takes per-column walls | production style changes in the parent are not followed automatically.
%   Call: plot_conly_rtri_walls({'plane', file_arm, label; 'cell', file_arm, label}, traj, name)
%   e.g. plot_conly_rtri_walls({'plane','eps1e3','Q_{55} = \epsilon R_2, \epsilon = 10^{-3}'; 'cell','eps3e3','Q_{55} = \epsilon R_2, \epsilon = 3\times10^{-3}'}, '_rtri_f1_td0p5', 'besteps')
%   STATUS: SCRATCH
function plot_conly_rtri_walls(cols, traj, name)
    if nargin < 2; traj = '_rtri_f1_td0p5'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_plaw_walls');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    AB = char(257);  HAT = char(770);  nA = size(cols, 1);
    COL_TRUE = [0.8 0 0];  C_EST = [0 0.2 0.9];  C_MEAS = [0.55 0.75 1.0];  FS = 18;  LFS = 12;  AXLW = 2.0;
    fig = figure('Position', [40 40 1700 1150], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(fig, 3, nA, 'TileSpacing', 'compact', 'Padding', 'compact');  ax = gobjects(3, nA);
    for m = 1:nA
        r = load(fullfile(id, sprintf('f1p2_%s_PI_s1_%s%s.mat', cols{m, 1}, cols{m, 2}, traj)));  t = r.t;
        e = r.a_true(:, 1) - r.a_hat(:, 1);
        for q = 1:3; ax(q, m) = nexttile(tl, (q - 1) * nA + m); hold(ax(q, m), 'on'); end
        hm = plot(ax(1, m), t, r.a_m(:, 1), '-', 'Color', C_MEAS, 'LineWidth', 0.6, 'DisplayName', [AB '_m']);
        ht = plot(ax(1, m), t, r.a_true(:, 1), '-', 'Color', COL_TRUE, 'LineWidth', 2.4, 'DisplayName', [AB '_{true}']);
        hh = plot(ax(1, m), t, r.a_hat(:, 1), '-', 'Color', C_EST, 'LineWidth', 1.8, 'DisplayName', [AB HAT]);
        ylim(ax(1, m), [-0.2 1.6]);
        lg = legend(ax(1, m), [ht hh hm], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'AutoUpdate', 'off');
        lg.Title.String = sprintf('%s wall, seed 1, c-only, %s', cols{m, 1}, cols{m, 3});
        plot(ax(2, m), t, e, '-', 'Color', C_EST, 'LineWidth', 1.8);  yline(ax(2, m), 0, '-', 'Color', [0.6 0.6 0.6]);
        plot(ax(3, m), t, r.dxm(:, 1), '-', 'Color', C_MEAS, 'LineWidth', 0.8);  yline(ax(3, m), 0, '-', 'Color', [0.6 0.6 0.6]);
        xlabel(ax(3, m), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        Td = r.seg(2);  ko = t >= Td;
        fprintf('%-5s %-8s | triangle mean|e_a| %.3f, worst %+.3f | e_a at Td %+.3f\n', cols{m, 1}, cols{m, 2}, mean(abs(e(ko))), ...
                e(find(abs(e) == max(abs(e(ko))) & ko, 1)), e(find(t >= Td, 1)));
    end
    ylabel(ax(1, 1), AB, 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax(2, 1), ['e_{' AB '}'], 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax(3, 1), '\delta x_m  [\mum]', 'FontSize', FS, 'FontWeight', 'bold');
    for q = 2:3; linkaxes(ax(q, :), 'y'); end
    for q = 1:3
        for m = 1:nA
            xlim(ax(q, m), [0 t(end)]);  set(ax(q, m), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');  grid(ax(q, m), 'off');
        end
    end
    out = fullfile(fd, sprintf('fig_conly_%s_walls%s_s1.png', name, traj));
    exportgraphics(fig, out, 'Resolution', 150);  close(fig);  fprintf('figure: %s\n', out);
end
