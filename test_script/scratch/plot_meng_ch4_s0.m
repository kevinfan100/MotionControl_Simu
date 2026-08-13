% STATUS: ACTIVE (scratch) | PURPOSE: Scenario-0 figures (Meng thesis Fig 28/29
%   counterparts) from run_meng_ch4_s0 output. House style: no grid/title,
%   True red / Estimate blue / Measured light blue, legend northoutside,
%   exportgraphics 150 dpi. | EXPIRES: with run_meng_ch4_s0.
function plot_meng_ch4_s0(out, out_dir)

    if nargin < 2 || isempty(out_dir)
        here = fileparts(mfilename('fullpath'));
        root = fileparts(fileparts(here));
        out_dir = fullfile(root, 'test_results', 'meng_ch4_s0');
    end
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end

    t  = out.t;  zd = out.zd;
    rN = out.N{1};  rE = out.E98{1};  rE1 = out.E1{1};
    C_TRUE = [0.85 0.10 0.10];  C_EST = [0.10 0.25 0.75];  C_MEAS = [0.55 0.75 0.95];

    % ---- Fig 1: Fig-28 counterpart — fixed-gain divergence vs (4.4)+lambda_f ----
    f1 = figure('Position', [80 80 900 720], 'Color', 'w');
    tl = tiledlayout(f1, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax1 = nexttile(tl);  hold(ax1, 'on');  box(ax1, 'on');
    plot(ax1, t, zd, '-', 'Color', C_TRUE, 'LineWidth', 1.2);
    plot(ax1, t, rN.p_true(:, 3), '-', 'Color', C_MEAS, 'LineWidth', 0.8);
    plot(ax1, t, rE.p_true(:, 3), '-', 'Color', C_EST, 'LineWidth', 0.8);
    yline(ax1, -3.2, 'k:');
    ylabel(ax1, 'z [\mum]');
    legend(ax1, {'z_d', 'z (arm N)', 'z (arm E98)', 'z_b'}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal');

    ax2 = nexttile(tl);  hold(ax2, 'on');  box(ax2, 'on');
    plot(ax2, t, zd - rN.p_true(:, 3), '-', 'Color', C_MEAS, 'LineWidth', 0.8);
    ylabel(ax2, '\deltaz arm N [\mum]');

    ax3 = nexttile(tl);  hold(ax3, 'on');  box(ax3, 'on');
    plot(ax3, t, 1e3 * (zd - rE.p_true(:, 3)), '-', 'Color', C_EST, 'LineWidth', 0.8);
    yline(ax3,  1e3 * 3 * out.sig11(2), 'k:');
    yline(ax3, -1e3 * 3 * out.sig11(2), 'k:');
    ylabel(ax3, '\deltaz arm E98 [nm]');
    xlabel(ax3, 't [s]');

    fp1 = fullfile(out_dir, 'meng_ch4_s0_fig28_counterpart.png');
    exportgraphics(f1, fp1, 'Resolution', 150);

    % ---- Fig 2: Fig-29 counterpart — a_hat_z tracks the gain step ----
    f2 = figure('Position', [120 120 900 520], 'Color', 'w');
    t2 = tiledlayout(f2, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    bx1 = nexttile(t2);  hold(bx1, 'on');  box(bx1, 'on');
    plot(bx1, t, rE.a_true_z, '-', 'Color', C_TRUE, 'LineWidth', 1.4);
    plot(bx1, t, rE.a_hat(:, 3), '-', 'Color', C_EST, 'LineWidth', 1.0);
    plot(bx1, t, rE1.a_hat(:, 3), '--', 'Color', C_EST, 'LineWidth', 1.0);
    ylabel(bx1, 'a_z [\mum/pN]');
    legend(bx1, {'a true', 'a hat (\lambda_f = 0.98)', 'a hat (\lambda_f = 1)'}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal');

    bx2 = nexttile(t2);  hold(bx2, 'on');  box(bx2, 'on');
    plot(bx2, t, 100 * (rE.a_hat(:, 3) ./ rE.a_true_z - 1), '-', ...
         'Color', C_EST, 'LineWidth', 0.9);
    plot(bx2, t, 100 * (rE1.a_hat(:, 3) ./ rE1.a_true_z - 1), '--', ...
         'Color', C_EST, 'LineWidth', 0.9);
    yline(bx2, 0, 'k:');
    ylabel(bx2, 'a\_hat/a\_true - 1 [%]');
    xlabel(bx2, 't [s]');

    fp2 = fullfile(out_dir, 'meng_ch4_s0_fig29_counterpart.png');
    exportgraphics(f2, fp2, 'Resolution', 150);

    fprintf('figures:\n  %s\n  %s\n', fp1, fp2);
end
