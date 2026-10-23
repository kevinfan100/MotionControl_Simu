% STATUS: SCRATCH -- 2026-10-08, the repo's OLD estimators vs today's first/second order (whitening only), same trajectory
%   (Fig. 4.3), plane, lambda_c 0.7, seed 1. Data: run_history_old.m. Columns: 7-state (default eq17, Meng-like), 6-state
%   (Vpersonal; 5-state identical), 4-state (production taylor+ar1+fdet: reads the TRUE wall law c(h_d) at run time),
%   first order (noap, whitening only, eps 1e-5), second order (noap, whitening only, lambda_f 0.98).
%   Rows: a-bar_z true (red) / estimate (blue); true tracking error z_d - z [um] (p_d(k) vs p_true(k-1)).
function plot_history_old(seed)
    if nargin < 1; seed = 1; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
    fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    V = {'7state', '6state', '4state', 'noap1w', 'noap2w'};
    L = {'7-state (Meng-like)', '6-state (Vpersonal)', '4-state (wall law known)', 'first order, whitening', 'second order, whitening'};
    R = run_history_old(V, seed);
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  FS = 16;  LFS = 12;  AXLW = 1.8;  AB = char(257);  HAT = char(770);
    n = numel(V);  f = figure('Position', [40 40 520 * n 900], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(2, n, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:n
        r = R.(['v' V{ic}])(seed);  t = r.t;
        ax = nexttile(tl, ic);  hold on; box on;
        ht = plot(ax, t, r.a_true, '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', [AB '_z']);
        hh = plot(ax, t, r.a_hat, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HAT '_z']);
        ylim(ax, [-0.2 1.6]);  xlim(ax, [0 t(end)]);
        lg = legend(ax, [ht hh], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');  lg.Title.String = L{ic};
        a2 = nexttile(tl, n + ic);  hold on; box on;
        plot(a2, t, r.dxm, '-', 'Color', cE, 'LineWidth', 0.8);  ylim(a2, [-0.6 0.6]);  xlim(a2, [0 t(end)]);  xlabel(a2, 'Time (sec)');
        if ic == 1; ylabel(ax, [AB '_z']);  ylabel(a2, ['z_d - z  [' char(181) 'm]']); end
        for a = [ax a2]; set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(a, 'off'); end
    end
    out = fullfile(fd, sprintf('fig_history_old_s%d.png', seed));  exportgraphics(f, out, 'Resolution', 130);  fprintf('figure: %s\n', out);
end
