% STATUS: SCRATCH -- 2026-10-22 (user: glass bead, approach (a): start above the bead top). Runs of run_t2_3d_reg with opts.wall = 'bead'
%   (bead radius 5 um, top at z = pz0 under the start, glass at z = pz0 - 10 um; nearest surface + plane law). Columns = arms (one seed).
%   Rows: side view x-z [um] of the scan part (bead, glass, gap h* R around the bead, probe path p solid) | true normal tilt theta_y (red) and
%   theta-hat_y (blue), theta_x / theta-hat_x (green) against time from the scan start | h with h* | a_N, a-hat_N, a_d.
%   Console per arm and seed: contact, the true tilt reached, max |theta_y error| and h range while the true tilt is below 30 / 60 / 90 deg.
%   -> test_results/temp_3d_wsonly/fig_t2_bead_<outtag>.png | run: plot_t2_bead({'bead_v1','bead_v4'}, {'40 um/s','10 um/s'}, 1, 1:3, 'speed')
function plot_t2_bead(PRE, LB, seed, SD, outtag)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');  addpath(genpath(fullfile(root, 'model')));  pc = physical_constants();  Rr = pc.R;
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cGr = [0 0.55 0.2];  cW = [0.45 0.45 0.45];  FS = 17;  LFS = 13;
    MU = char(181);  DG = char(176);  TH = char(952);  HT = char(770);  AB = char(257);
    nc = numel(PRE);  rb = 5;
    f = figure('Position', [40 40 900 * nc 1800], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(4, nc, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:nc
        for s = SD                                              % console numbers for every seed
            S = load(fullfile(od, sprintf('temp_t2_%s_s%d.mat', PRE{ic}, s)));  r = S.R.runs{1};  k = 1:r.kend;
            sc = k(r.t(k) >= S.R.opts.t_scan);  tilt = abs(r.thy_true(sc));  ey = r.thy(sc) - r.thy_true(sc);
            fprintf('%-8s s%d: contact %d (t %.3f) | max true tilt %.0f deg', PRE{ic}, s, r.contact, r.t_contact, max(tilt));
            for lim = [30 60 90]
                m = tilt < lim;  if any(m); fprintf(' | tilt<%d: max|e_thy| %.1f, h %.2f-%.2f', lim, max(abs(ey(m))), min(r.h(sc(m))), max(r.h(sc(m)))); end
            end
            fprintf('\n');
        end
        S = load(fullfile(od, sprintf('temp_t2_%s_s%d.mat', PRE{ic}, seed)));  r = S.R.runs{1};  k = 1:r.kend;  o = S.R.opts;
        sc = k(r.t(k) >= o.t_scan - 0.2);  t = r.t(sc) - o.t_scan;  p = r.p(sc, :);
        z0 = r.p(1, 3) - r.h(1) * Rr;  c = [0 z0 - rb];  zg = z0 - 2 * rb;   % bead top under the start
        ax = nexttile(tl, ic);  hold on;  box on;
        th = linspace(0, 2 * pi, 400);
        fill([-12 12 12 -12], [zg zg zg - 2 zg - 2], [0.9 0.9 0.9], 'EdgeColor', 'none', 'HandleVisibility', 'off');
        plot([-12 12], [zg zg], '-', 'Color', cW, 'LineWidth', 4, 'HandleVisibility', 'off');
        fill(c(1) + rb * cos(th), c(2) + rb * sin(th), [0.8 0.88 0.95], 'EdgeColor', cW, 'LineWidth', 3, 'HandleVisibility', 'off');
        hs = plot(c(1) + (rb + r.h_star * Rr) * cos(th), c(2) + (rb + r.h_star * Rr) * sin(th), '--', 'Color', cW, 'LineWidth', 1.5, 'DisplayName', 'h* R');
        hp = plot(p(:, 1), p(:, 3), '-', 'Color', cE, 'LineWidth', 2, 'DisplayName', 'p');
        plot(p(end, 1), p(end, 3), '^', 'Color', cE, 'MarkerFaceColor', cE, 'MarkerSize', 10, 'HandleVisibility', 'off');
        axis equal;  xlim([-12 12]);  ylim([zg - 1 c(2) + rb + 4]);
        text(0.03, 0.92, LB{ic}, 'Units', 'normalized', 'FontSize', FS, 'FontWeight', 'bold');
        xlabel(['x  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');  if ic == 1; ylabel(['z  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([hs hp], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
        ax = nexttile(tl, nc + ic);  hold on;  box on;
        h1 = plot(t, r.thy_true(sc), '-', 'Color', cT, 'LineWidth', 2.5, 'DisplayName', [TH '_y,  ' TH '_x']);
        plot(t, r.thx_true(sc), '-', 'Color', cT, 'LineWidth', 2.5, 'HandleVisibility', 'off');
        h2 = plot(t, r.thy(sc), '-', 'Color', cE, 'LineWidth', 1.5, 'DisplayName', [TH HT '_y']);
        h3 = plot(t, r.thx(sc), '-', 'Color', cGr, 'LineWidth', 1.5, 'DisplayName', [TH HT '_x']);
        ylim([-30 120]);  xlim([t(1) t(end)]);  if ic == 1; ylabel([TH '  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
        ax = nexttile(tl, 2 * nc + ic);  hold on;  box on;
        h1 = plot(t, r.h_star + 0 * t, '-', 'Color', cT, 'LineWidth', 2.5, 'DisplayName', 'h*');
        h2 = plot(t, r.h(sc), '-', 'Color', cE, 'LineWidth', 1.5, 'DisplayName', 'h');
        ylim([0.9 2.5]);  xlim([t(1) t(end)]);  if ic == 1; ylabel('h  [R]', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
        ax = nexttile(tl, 3 * nc + ic);  hold on;  box on;
        h1 = plot(t, r.aN_true(sc), '-', 'Color', cT, 'LineWidth', 2.5, 'DisplayName', [AB '_N']);
        h2 = plot(t, r.aN(sc), '-', 'Color', cE, 'LineWidth', 1.5, 'DisplayName', [AB HT '_N']);
        h3 = plot(t, o.a_d + 0 * t, '--', 'Color', [0 0 0], 'LineWidth', 2, 'DisplayName', sprintf('%s_d = %.1f', AB, o.a_d));
        ylim([0 0.4]);  xlim([t(1) t(end)]);  if ic == 1; ylabel([AB '_N'], 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel('time from the scan start  [s]', 'FontSize', FS, 'FontWeight', 'bold');
        legend([h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
    end
    out = fullfile(od, sprintf('fig_t2_bead_%s.png', outtag));
    exportgraphics(f, out, 'Resolution', 100);  close(f);  fprintf('figure: %s\n', out);
end
