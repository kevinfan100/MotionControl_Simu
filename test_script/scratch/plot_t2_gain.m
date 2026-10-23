% STATUS: SCRATCH -- 2026-10-22 (user: simple figure; Kalman gain = the ACTUAL gain the filter used). Wait-only fixed points tmg<gap>_s1
%   (20 deg plane, seed 1, logs thm / Kth / dth_y1 / dth_ro of run_t2_3d_reg). Columns = gaps. theta_y only.
%   Row 1: single-sample angle reading theta_ym (light dots, every 20th step), theta-hat_y (blue), truth 20 deg (red).
%   Row 2: actual gain on the angle reading, K * H = fraction of (theta_ym - theta-hat_y) applied per step (log axis).
%   Row 3: where theta-hat_y's change came from: cumulative change from the angle-reading row and from the position rows (y1).
%   -> test_results/temp_3d_wsonly/fig_t2_gain.png | run: plot_t2_gain([50 15 10 5])
function plot_t2_gain(GAP)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cL = [0.65 0.75 0.95];  cGr = [0 0.55 0.2];  FS = 17;  LFS = 13;  TH = char(952);  HT = char(770);  DG = char(176);  MU = char(181);
    nc = numel(GAP);
    f = figure('Position', [40 40 560 * nc 1350], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(3, nc, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ig = 1:nc
        S = load(fullfile(od, sprintf('temp_t2_tmg%d_s1.mat', GAP(ig))));  r = S.R.runs{1};  k = 1:r.kend;  t = r.t(k);  ty = S.R.opts.theta_y;
        ax = nexttile(tl, ig);  hold on;  box on;
        kk = 1:20:numel(k);
        h1 = plot(t(kk), r.thm(k(kk), 2), '.', 'Color', cL, 'MarkerSize', 4, 'DisplayName', [TH '_{ym} (one sample)']);
        h2 = plot(t, r.thy(k), '-', 'Color', cE, 'LineWidth', 2, 'DisplayName', [TH HT '_y']);
        h3 = plot([0 t(end)], [ty ty], '-', 'Color', cT, 'LineWidth', 2.5, 'DisplayName', 'true');
        ylim([-200 200]);  xlim([0 t(end)]);
        text(0.04, 0.92, sprintf('%d %sm', GAP(ig), MU), 'Units', 'normalized', 'FontSize', FS, 'FontWeight', 'bold');
        if ig == 1; ylabel([TH '_y  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        ax(2) = nexttile(tl, nc + ig);  hold on;  box on;
        plot(t, abs(r.Kth(k, 2)), '-', 'Color', cE, 'LineWidth', 1.2);
        set(ax(2), 'YScale', 'log');  ylim([1e-7 1e-1]);  xlim([0 t(end)]);
        if ig == 1; ylabel(['gain on ' TH '_{ym}  (K H)'], 'FontSize', FS, 'FontWeight', 'bold'); end
        ax(3) = nexttile(tl, 2 * nc + ig);  hold on;  box on;
        g1 = plot(t, cumsum(r.dth_y1(k, 2)), '-', 'Color', cGr, 'LineWidth', 2, 'DisplayName', 'from position rows (y_1)');
        g2 = plot(t, cumsum(r.dth_ro(k, 2)), '-', 'Color', cE, 'LineWidth', 2, 'DisplayName', ['from the ' TH '_{ym} row']);
        plot([0 t(end)], [ty ty], ':', 'Color', cT, 'LineWidth', 2, 'HandleVisibility', 'off');
        ylim([-15 30]);  xlim([0 t(end)]);
        if ig == 1; ylabel(['change of ' TH HT '_y  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        legend([g1 g2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
        m1 = t >= 1;
        fprintf('%2d um: a_T - a_N %.3f | theta_ym spread (std after 1 s) %.0f deg | gain K H median %.1e | theta-hat_y change at 20 s: y1 %+.1f, readout %+.1f deg | end error %+.1f\n', ...
                GAP(ig), mean(r.aT_true(k) - r.aN_true(k)), std(r.thm(k(m1), 2), 'omitnan'), median(abs(r.Kth(k(m1), 2)), 'omitnan'), sum(r.dth_y1(k, 2)), sum(r.dth_ro(k, 2)), r.thy(k(end)) - ty);
    end
    out = fullfile(od, 'fig_t2_gain.png');
    exportgraphics(f, out, 'Resolution', 110);  close(f);  fprintf('figure: %s\n', out);
end
