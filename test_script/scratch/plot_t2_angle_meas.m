% STATUS: SCRATCH -- 2026-10-22 (user: one gap, intuitive comparison of with / without the angle measurement). Runs tmgS<gap>_<arm>_s1
%   (seed 1, 20 s fixed point, 20 deg plane): with = A (wait) / B (motion), without = D (wait) / C (motion; theta_xm, theta_ym rows unused).
%   Columns = wait only | motion.
%   Row 1: theta-hat_y - theta_y, with (blue) and without (orange dashed) overlaid.
%   Row 2: run WITH the angle measurement, cumulative change of theta-hat_y from e_p (dp_m rows), e_lambda (a_Tm, a_Nm rows) and
%          e_theta (theta_xm, theta_ym rows) = the three terms L_81 e_p + L_82 e_lambda + L_83 e_theta of the notes p.5.
%   Row 3: sqrt P of theta_y, with / without overlaid (log axis).
%   which = 'err' (user 10-22: this is the presentation version): row 1 only -> fig_t2_angle_meas_<gap>_err.png
%   -> test_results/temp_3d_wsonly/fig_t2_angle_meas_<gap>.png | run: plot_t2_angle_meas(15)  or  plot_t2_angle_meas(15, 'err')
function plot_t2_angle_meas(gap, which)
    if nargin < 2; which = 'all'; end
    one = strcmp(which, 'err');  nrow = 3 - 2 * one;
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cO = [0.9 0.5 0];  cG = [0 0.55 0.2];  cP = [0.55 0.2 0.65];  cW = [0.5 0.5 0.5];
    FS = 18;  LFS = 14;  TH = char(952);  HT = char(770);  DG = char(176);  MU = char(181);  AB = char(257);  LA = char(955);
    ARM = {'A', 'D'; 'B', 'C'};  CL = {'wait only', 'motion'};
    f = figure('Position', [40 40 1400 450 * nrow], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(nrow, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:2
        W = load(fullfile(od, sprintf('temp_t2_tmgS%d_%s_s1.mat', gap, ARM{ic, 1})));  rw = W.R.runs{1};
        N = load(fullfile(od, sprintf('temp_t2_tmgS%d_%s_s1.mat', gap, ARM{ic, 2})));  rn = N.R.runs{1};
        k = 1:min(rw.kend, rn.kend);  t = rw.t(k);  ty = W.R.opts.theta_y;
        % ---- row 1: error ----
        ax = nexttile(tl, ic);  hold on;  box on;
        plot([0 t(end)], [0 0], '-', 'Color', cT, 'LineWidth', 2.5, 'HandleVisibility', 'off');
        plot(t, rw.thy(k) - ty, '-', 'Color', cE, 'LineWidth', 2, 'DisplayName', 'with angle measurement');
        plot(t, rn.thy(k) - ty, '--', 'Color', cO, 'LineWidth', 2, 'DisplayName', 'without angle measurement');
        ylim([-25 10]);  xlim([0 t(end)]);
        text(0.97, 0.1, sprintf('%d %sm,  %s', gap, MU, CL{ic}), 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', FS - 2, 'FontWeight', 'bold');
        if ic == 1; ylabel([TH HT '_y - ' TH '_y  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold'); end
        legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
        if one; xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); continue; end
        % ---- row 2: where the change comes from (run with the angle measurement) ----
        ax = nexttile(tl, 2 + ic);  hold on;  box on;
        plot([0 t(end)], [0 0], '-', 'Color', cT, 'LineWidth', 2.5, 'HandleVisibility', 'off');
        plot([0 t(end)], [20 20], ':', 'Color', cW, 'LineWidth', 2, 'DisplayName', 'needed change');
        plot(t, cumsum(rw.dth_dp(k, 2)), '-', 'Color', cO, 'LineWidth', 2, 'DisplayName', 'from  e_p');
        plot(t, cumsum(rw.dth_ag(k, 2)), '-', 'Color', cP, 'LineWidth', 2, 'DisplayName', ['from  e_{' LA '}']);
        plot(t, cumsum(rw.dth_ro(k, 2)), '-', 'Color', cG, 'LineWidth', 2, 'DisplayName', ['from  e_{' TH '}  (angle measurement)']);
        ylim([-5 25]);  xlim([0 t(end)]);
        if ic == 1; ylabel(['change of  ' TH HT '_y  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold'); end
        legend('Location', 'northoutside', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold');
        set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
        % ---- row 3: sqrt P ----
        ax = nexttile(tl, 4 + ic);  hold on;  box on;
        plot(t, rw.sP(k, 6), '-', 'Color', cE, 'LineWidth', 2, 'DisplayName', 'with angle measurement');
        plot(t, rn.sP(k, 6), '--', 'Color', cO, 'LineWidth', 2, 'DisplayName', 'without angle measurement');
        set(ax, 'YScale', 'log');  ylim([0.2 15]);  xlim([0 t(end)]);
        if ic == 1; ylabel(['\surdP_{' TH 'y}  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
        fprintf('%2d um %-9s: err at 20 s with %+.1f / without %+.1f deg | sqrtP at 20 s with %.2f / without %.2f deg | from e_p %+.1f, e_lambda %+.1f, e_theta %+.1f\n', ...
                gap, CL{ic}, rw.thy(k(end)) - ty, rn.thy(k(end)) - ty, rw.sP(k(end), 6), rn.sP(k(end), 6), sum(rw.dth_dp(k, 2)), sum(rw.dth_ag(k, 2)), sum(rw.dth_ro(k, 2)));
    end
    out = fullfile(od, sprintf('fig_t2_angle_meas_%d.png', gap));  if one; out = fullfile(od, sprintf('fig_t2_angle_meas_%d_err.png', gap)); end
    exportgraphics(f, out, 'Resolution', 100);  close(f);  fprintf('figure: %s\n', out);
end
