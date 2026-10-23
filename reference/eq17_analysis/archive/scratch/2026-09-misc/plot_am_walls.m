% PURPOSE (2026-09-29, user: "show me the measured-gain signal a_m on the two walls"): c-only law (f1p2), plane init, prior 0.10,
%   lambda = 1, T_d = 2 s, 10 seeds; runs f1p2_{plane,cell}_PI_sig010_am_ramp2.mat (baseline rerun with a_m collected).
%   Columns plane | cell; rows: (1) a_true (red), a_hat (blue), a_m = measured gain (light blue), 10-seed means;
%   (2) a_m - a_true (what the reading gets wrong, mean +- sigma_seed) and a_hat - a_true; (3) innovation y2 - y2_hat (mean +- sigma).
%   House style (True red, Estimate blue, Measured light blue). Output reference/eq17_analysis/derivation/figures/am_walls.png
%   EXPIRES: with the c-only line
function plot_am_walls(sfx)
    if nargin < 1; sfx = '_sig010_am_ramp2'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    rd = fullfile(root, 'test_results', 'temp_plaw_walls');  od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    PL = {'plane', 'cell'};  COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; COL_M = [0.35 0.7 0.95]; BANDC = [0.45 0.55 0.95]; BANDM = [0.6 0.8 0.95];
    FS = 15; LFS = 11; AXLW = 2.0; CG = [0.5 0.5 0.5];
    f = figure('Position', [60 60 1250 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');  ax = gobjects(3, 2);
    fprintf('\n%-6s | %10s %10s | %10s %10s | %10s\n', 'plant', 'a_m-a_true', 'wall/hold', 'a_hat-a_tr', 'wall/hold', 'sd(a_m) wall');
    for ip = 1:2
        r = load(fullfile(rd, sprintf('f1p2_%s_PI%s.mat', PL{ip}, sfx)));
        t = r.t;  seg = r.seg;  desc = t >= seg(1) & t < seg(2);  hold_ = t >= seg(3);  ie = find(desc, 1, 'last');
        at = mean(r.a_true, 2);  am = mean(r.a_hat, 2);  mm = mean(r.a_m, 2);
        em = r.a_m - r.a_true;  eh = r.a_hat - r.a_true;  Em = mean(em, 2);  Sm = std(em, 0, 2);  Eh = mean(eh, 2);  Sh = std(eh, 0, 2);
        I2 = mean(r.innov2, 2);  S2 = std(r.innov2, 0, 2);
        fprintf('%-6s | %+10.3f %+10.3f | %+10.3f %+10.3f | %10.3f\n', PL{ip}, Em(ie), mean(Em(hold_)), Eh(ie), mean(Eh(hold_)), Sm(ie));
        ax(1, ip) = nexttile(ip); hold on; box on;
        h3 = plot(t, mm, '-', 'Color', COL_M, 'LineWidth', 1.4, 'DisplayName', 'a_m / a_{nom}  (measured gain = y_2 reading, 10-seed mean)');
        h1 = plot(t, at, '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'a_{true} / a_{nom}');
        h2 = plot(t, am, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', 'a_{hat} / a_{nom}  (estimate)');
        h0 = plot(NaN, NaN, 'w.', 'DisplayName', sprintf('%s wall, plane init', upper(PL{ip})));
        for s = seg; plot([s s], [0 1], '--', 'Color', CG, 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
        xlim([0 t(end)]); ylim([0 1]);  if ip == 1; ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([h0 h1 h2 h3], 'Location', 'northoutside', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
        ax(2, ip) = nexttile(2 + ip); hold on; box on;
        fill([t; flipud(t)], [Em - Sm; flipud(Em + Sm)], BANDM, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
        fill([t; flipud(t)], [Eh - Sh; flipud(Eh + Sh)], BANDC, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
        h4 = plot(t, Em, '-', 'Color', COL_M, 'LineWidth', 1.6, 'DisplayName', 'a_m - a_{true}  (reading error, mean \pm \sigma_{seed})');
        h5 = plot(t, Eh, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', 'a_{hat} - a_{true}  (estimate error)');
        yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
        for s = seg; plot([s s], [-0.2 0.2], '--', 'Color', CG, 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
        xlim([0 t(end)]); ylim([-0.2 0.2]);  if ip == 1; ylabel('error  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([h4 h5], 'Location', 'northoutside', 'NumColumns', 1, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
        ax(3, ip) = nexttile(4 + ip); hold on; box on;
        fill([t; flipud(t)], [I2 - S2; flipud(I2 + S2)], BANDC, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
        h6 = plot(t, I2, '-', 'Color', COL_HAT, 'LineWidth', 1.4, 'DisplayName', 'innovation  y_2 - y_2_{hat}  (mean \pm \sigma_{seed})');
        yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
        for s = seg; plot([s s], [-0.2 0.2], '--', 'Color', CG, 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
        xlim([0 t(end)]); ylim([-0.2 0.2]);  if ip == 1; ylabel('innovation  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        legend(h6, 'Location', 'northoutside', 'NumColumns', 1, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    end
    for rr = 1:3; linkaxes(ax(rr, :), 'y'); end
    fn = fullfile(od, 'am_walls.png');  exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
end
