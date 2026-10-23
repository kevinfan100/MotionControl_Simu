% STATUS: SCRATCH -- 2026-10-08, history ladder of the gain model without a' (user: first and second order only, find the key
%   mistakes). Fig. 4.3 trajectory (22.2 -> 1.10 R in 1 s, triangle 1.10 <-> 10 R at 2 Hz), plane wall, seed 1, 5state_noap.
%   which = 'first' : Q44 = 0 / raw y2 / old timing / lambda_c 0.7 -> + eps 1e-5 -> + y2 whitening -> + new timing -> + lambda_c 0.4
%   which = 'second': level + rate, lambda_f 1 -> + lambda_f 0.98 -> + whitening -> + new timing -> + lambda_c 0.4
%   Rows: a-bar_z true (red, thick, first) / estimate (blue); measured tracking error delta z_m [um]. Rung label = legend title.
function plot_history_ladder(which, seed)
    if nargin < 1; which = 'first'; end
    if nargin < 2; seed = 1; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
    fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    if strcmp(which, 'first')
        F = {'plain_eps0_lc70_oldorder_tri10', 'plain_lc70_oldorder_tri10', 'plain_y2_whiten_lc70_oldorder_tri10', 'plain_y2_whiten_lc70_tri10', 'plain_y2_whiten_tri10'};
        L = {'0: Q_{44} = 0, raw y_2', '1: + \epsilon = 10^{-5}', '2: + y_2 whitening', '3: + new timing', '4: + \lambda_c 0.4'};
    else
        F = {'plain_go2_eps0_lc70_oldorder_tri10', 'plain_go2_eps0_lf098_lc70_oldorder_tri10', 'plain_go2_eps0_lf098_y2_whiten_lc70_oldorder_tri10', ...
             'plain_go2_eps0_lf098_y2_whiten_lc70_tri10', 'plain_go2_eps0_lf098_y2_whiten_tri10'};
        L = {'0: level + rate, \lambda_f = 1', '1: + \lambda_f = 0.98', '2: + y_2 whitening', '3: + new timing', '4: + \lambda_c 0.4'};
    end
    R = run_mc_s4_first_order(F, seed, {'plane'});
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  FS = 16;  LFS = 12;  AXLW = 1.8;  AB = char(257);  HAT = char(770);
    n = numel(F);  f = figure('Position', [40 40 520 * n 900], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(2, n, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:n
        r = R.(matlab.lang.makeValidName(F{ic})).plane(seed);  t = r.t;
        ax = nexttile(tl, ic);  hold on; box on;
        ht = plot(ax, t, r.a_true, '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', [AB '_z']);
        hh = plot(ax, t, r.a_hat, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HAT '_z']);
        ylim(ax, [-0.2 1.6]);  xlim(ax, [0 t(end)]);
        lg = legend(ax, [ht hh], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        lg.Title.String = L{ic};
        a2 = nexttile(tl, n + ic);  hold on; box on;
        plot(a2, t, r.dxm, '-', 'Color', cE, 'LineWidth', 0.8);  ylim(a2, [-0.6 0.6]);  xlim(a2, [0 t(end)]);
        xlabel(a2, 'Time (sec)');
        if ic == 1; ylabel(ax, [AB '_z']);  ylabel(a2, [char(948) 'z_m  [' char(181) 'm]']); end
        for a = [ax a2]; set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(a, 'off'); end
    end
    out = fullfile(fd, sprintf('fig_history_ladder_%s_s%d.png', which, seed));  exportgraphics(f, out, 'Resolution', 130);  fprintf('figure: %s\n', out);
end
