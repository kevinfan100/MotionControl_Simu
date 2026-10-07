% STATUS: SCRATCH -- 2026-10-06 (user: "show the wide + fast configuration on its own"). ramp_tri T_d 0.5 s (42 R/s), triangle 1.10 -> 10 R
%   at 2.5 Hz (44.5 R/s), single P_fix (q55_adapt 'ratio'), plane init, 10 seeds; columns plane | cell.
%   Rows: (1) h_d  (2) abar_true red, abar_hat (seed mean)  (3) e_abar seed mean +- sd  (4) c_eff(h_d) red, c_hat (seed mean).
%   pfix_list: sqrt(P_fix) values to overlay (0.15 from run_conly_rtri_wide_fast.m; 0.25 / 0.35 from run_conly_rtri_wide_pfix.m).
%   Console: per sqrt(P_fix), the c_hat follow fraction per leg (c_hat change / c_eff change), near-wall |c_hat - c_eff| (h_d < 2),
%   e_abar at the troughs (mean of the 5 trough instants) and in 1.1-2 R, and the seed sd of c_hat at the troughs.
function plot_conly_rtri_wide_cfg(pfix_list, out_name)
    if nargin < 1; pfix_list = 0.15; end
    if nargin < 2; out_name = 'fig_conly_wide_cfg.png'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_plaw_walls');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    WALLS = {'plane', 'cell'};  cT = [0.8 0 0];  FS = 16;  LFS = 11;  AXLW = 1.8;
    CB = {[0 0.2 0.9], [0.45 0.25 0.85], [0.1 0.65 0.75]};
    f = figure('Position', [40 40 1600 1500], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for iw = 1:2
        ax = gobjects(4, 1);  for r = 1:4; ax(r) = nexttile(tl, (r - 1) * 2 + iw); hold(ax(r), 'on'); box(ax(r), 'on'); end
        h2 = gobjects(0);  h4 = gobjects(0);
        for ip = 1:numel(pfix_list)
            S = load(fullfile(id, sprintf('f1p2_%s_PI_sig010_rho%02d_rtri_f2p5_td0p5_top1000.mat', WALLS{iw}, round(100 * pfix_list(ip)))));
            t = S.t(:);  nm = sprintf('\\surdP_{fix} %.2f', pfix_list(ip));  cb = CB{ip};
            if ip == 1
                plot(ax(1), t, S.wd, 'k-', 'LineWidth', 1.6);
                h2(end + 1) = plot(ax(2), t, mean(S.a_true, 2), '-', 'Color', cT, 'LineWidth', 1.6, 'DisplayName', 'a_z/a_{nom} true'); %#ok<AGROW>
                ce = S.c_eff_d(:);  ce(S.wd(:) < 1) = NaN;
                h4(end + 1) = plot(ax(4), t, ce, '-', 'Color', cT, 'LineWidth', 1.6, 'DisplayName', 'c_{eff}(h_d)'); %#ok<AGROW>
            end
            h2(end + 1) = plot(ax(2), t, mean(S.a_hat, 2), '-', 'Color', cb, 'LineWidth', 1.4, 'DisplayName', ['est, ' nm]); %#ok<AGROW>
            e = S.a_true - S.a_hat;  m = mean(e, 2);  s = std(e, 0, 2);
            fill(ax(3), [t; flipud(t)], [m - s; flipud(m + s)], cb, 'FaceAlpha', 0.18, 'EdgeColor', 'none');
            plot(ax(3), t, m, '-', 'Color', cb, 'LineWidth', 1.5);
            h4(end + 1) = plot(ax(4), t, mean(S.c_hat, 2), '-', 'Color', cb, 'LineWidth', 1.8, 'DisplayName', ['c_{hat}, ' nm]); %#ok<AGROW>
            local_metrics(S, WALLS{iw}, pfix_list(ip));
        end
        plot(ax(3), [0 t(end)], [0 0], ':', 'Color', [0.3 0.3 0.3]);
        ylim(ax(1), [0 23]);  ylim(ax(2), [0 1.1]);  ylim(ax(3), [-0.1 0.1]);  ylim(ax(4), [-0.4 1.0]);
        for r = 1:4; xlim(ax(r), [0 t(end)]); set(ax(r), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW); grid(ax(r), 'off'); end
        lg = legend(ax(2), h2, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'AutoUpdate', 'off');
        lg.Title.String = sprintf('%s wall, plane init; 0.5 s descent + triangle 1.1-10 R at 2.5 Hz', WALLS{iw});
        legend(ax(4), h4, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'AutoUpdate', 'off');
        xlabel(ax(4), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        if iw == 1
            ylabel(ax(1), 'h_d', 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax(2), 'a_z / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
            ylabel(ax(3), 'e_a  (true - est)', 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax(4), 'c', 'FontSize', FS, 'FontWeight', 'bold');
        end
    end
    out = fullfile(fd, out_name);  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end

function local_metrics(S, wall, pf)
    t = S.t(:);  hd = S.wd(:);  tri = t > 0.5;  cm = mean(S.c_hat, 2);  ce = S.c_eff_d(:);
    up = [0; diff(hd)] > 0;  dcm = [0; diff(cm)];  dce = [0; diff(ce)];
    fol_up = sum(dcm(tri & up)) / sum(dce(tri & up));  fol_dn = sum(dcm(tri & ~up)) / sum(dce(tri & ~up));
    nw = tri & hd < 2;  e = S.a_true - S.a_hat;  me = mean(e, 2);
    tr = find(tri & [false; diff(sign([0; diff(hd)])) > 0]);           % trough instants (h_d turns from down to up)
    fprintf('%-5s sqrtP_fix %.2f | c_hat follows %.0f%% (up) %.0f%% (down) | near-wall |c_hat-c_eff| %.3f | e_a troughs %+.3f, 1.1-2 R |e_a| %.4f | c_hat seed sd at troughs %.3f | tri |e_a| %.4f\n', ...
        wall, pf, 100 * fol_up, 100 * fol_dn, mean(abs(cm(nw) - ce(nw))), mean(me(tr)), mean(abs(e(nw, :)), 'all'), mean(std(S.c_hat(tr, :), 0, 2)), mean(abs(e(tri, :)), 'all'));
end
