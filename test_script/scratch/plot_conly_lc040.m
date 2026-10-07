% STATUS: SCRATCH -- 2026-10-06, plots run_conly_lc040.m: lambda_c 0.7 (blue) vs 0.4 (orange), same 10 seeds, plane init, columns plane | cell.
%   Figure 1 fig_conly_lc040_rtri.png   ramp_tri (T_d 1, 1.1-2 R, 1 Hz), sqrt(P_fix) 0.15: rows abar true/est, e_abar mean +- sd, c_hat vs c_eff, tracking error R(h_d - h) [um] (seed 1).
%   Figure 2 fig_conly_lc040_pfix.png   lambda_c 0.4, sqrt(P_fix) 0.10 / 0.15 / 0.20 on ramp_tri: rows e_abar, c_hat.
%   Figure 3 fig_conly_lc040_wide.png   wide setup (T_d 0.5, 1.1-10 R, 2.5 Hz), 0.7 vs 0.4: same rows as figure 1.
%   Console: health (NaN, min h, c_hat range, NIS1, NIS2) and point criteria for every run.
function plot_conly_lc040()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_plaw_walls');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    W = {'plane', 'cell'};
    RUNS = {'_sig010_rho15_rtri_f1', 'lc 0.7  P_fix 0.15  ramp_tri', 1.0;
            '_sig010_rho15_rtri_f1_lc040', 'lc 0.4  P_fix 0.15  ramp_tri', 1.0;
            '_sig010_rho10_rtri_f1_lc040', 'lc 0.4  P_fix 0.10  ramp_tri', 1.0;
            '_sig010_rho20_rtri_f1_lc040', 'lc 0.4  P_fix 0.20  ramp_tri', 1.0;
            '_sig010_rho15_rtri_f2p5_td0p5_top1000', 'lc 0.7  P_fix 0.15  wide', 0.5;
            '_sig010_rho15_rtri_f2p5_td0p5_top1000_lc040', 'lc 0.4  P_fix 0.15  wide', 0.5};
    for w = W
        for i = 1:size(RUNS, 1)
            S = load(fullfile(id, sprintf('f1p2_%s_PI%s.mat', w{1}, RUNS{i, 1})));  t = S.t(:);  Td = RUNS{i, 3};
            e = S.a_true - S.a_hat;  m = mean(e, 2);  tri = t > Td;  nw = tri & S.wd(:) < 2;
            nis1 = mean(S.innov1(tri, :).^2 ./ S.S1(tri, :), 'all', 'omitnan');  nis2 = mean(S.innov2(tri, :).^2 ./ S.S2(tri, :), 'all', 'omitnan');
            k = @(tt) find(t >= tt - 1e-9, 1);
            if Td == 1; pts = sprintf('@1.0 %+.3f top1.5 %+.3f tr2 %+.3f top2.5 %+.3f tr3 %+.3f', m(k(1)), m(k(1.5)), m(k(2)), m(k(2.5)), m(end));
            else; pts = sprintf('@0.5 %+.3f', m(k(0.5))); end
            fprintf('%-5s %-30s | NaN %d min h %.3f c_hat [%+.2f %+.2f] NIS1 %.2f NIS2 %.2f | descent max|mean| %.3f | %s | tri |e_a| %.4f near-wall %.4f seed-sd %.4f | sqrtP end %.3f\n', ...
                w{1}, RUNS{i, 2}, nnz(isnan(S.a_hat)), min(S.h_true(:)), min(S.c_hat(:)), max(S.c_hat(:)), nis1, nis2, max(abs(m(t > 0.3 & t <= Td + 0.02))), pts, ...
                mean(abs(e(tri, :)), 'all'), mean(abs(e(nw, :)), 'all'), std(mean(e(tri, :), 1)), mean(S.sP_c(end, :)));
        end
    end
    local_pair(id, fd, W, {'_sig010_rho15_rtri_f1', '_sig010_rho15_rtri_f1_lc040'}, 'ramp\_tri 1.1-2 R, 1 Hz, T_d 1 s, \surdP_{fix} 0.15', 'fig_conly_lc040_rtri.png');
    local_pair(id, fd, W, {'_sig010_rho15_rtri_f2p5_td0p5_top1000', '_sig010_rho15_rtri_f2p5_td0p5_top1000_lc040'}, 'wide 1.1-10 R, 2.5 Hz, T_d 0.5 s, \surdP_{fix} 0.15', 'fig_conly_lc040_wide.png');
    local_pfix(id, fd, W);
end

function local_pair(id, fd, W, tags, ttl, out_name)
    C = {[0 0.2 0.9], [0.9 0.5 0]};  NM = {'\lambda_c 0.7', '\lambda_c 0.4'};  cT = [0.8 0 0];  FS = 15;  LFS = 11;  AXLW = 1.8;
    f = figure('Position', [40 40 1600 1500], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for iw = 1:2
        ax = gobjects(4, 1);  for r = 1:4; ax(r) = nexttile(tl, (r - 1) * 2 + iw); hold(ax(r), 'on'); box(ax(r), 'on'); end
        h1 = gobjects(0);  h3 = gobjects(0);
        for ia = 1:2
            S = load(fullfile(id, sprintf('f1p2_%s_PI%s.mat', W{iw}, tags{ia})));  t = S.t(:);
            if ia == 1; h1(end + 1) = plot(ax(1), t, mean(S.a_true, 2), '-', 'Color', cT, 'LineWidth', 1.5, 'DisplayName', 'a_z/a_{nom} true'); end %#ok<AGROW>
            h1(end + 1) = plot(ax(1), t, mean(S.a_hat, 2), '-', 'Color', C{ia}, 'LineWidth', 1.2, 'DisplayName', ['est, ' NM{ia}]); %#ok<AGROW>
            e = S.a_true - S.a_hat;  m = mean(e, 2);  s = std(e, 0, 2);
            fill(ax(2), [t; flipud(t)], [m - s; flipud(m + s)], C{ia}, 'FaceAlpha', 0.18, 'EdgeColor', 'none');  plot(ax(2), t, m, '-', 'Color', C{ia}, 'LineWidth', 1.4);
            if ia == 1; ce = S.c_eff_d(:); ce(S.wd(:) < 1) = NaN; h3(end + 1) = plot(ax(3), t, ce, '-', 'Color', cT, 'LineWidth', 1.4, 'DisplayName', 'c_{eff}(h_d)'); end %#ok<AGROW>
            h3(end + 1) = plot(ax(3), t, mean(S.c_hat, 2), '-', 'Color', C{ia}, 'LineWidth', 1.7, 'DisplayName', ['c_{hat}, ' NM{ia}]); %#ok<AGROW>
            te = 2250 * (S.wd(:) - S.h_true(:, 1));  te(S.wd(:) < 1) = NaN;   % tracking error R (h_d - h) [nm], seed 1, R = 2.25 um
            plot(ax(4), t, te, '-', 'Color', C{ia}, 'LineWidth', 0.6);
        end
        plot(ax(2), [0 t(end)], [0 0], ':', 'Color', [0.3 0.3 0.3]);
        ylim(ax(1), [0 1.1]);  ylim(ax(2), [-0.1 0.1]);  ylim(ax(3), [-0.4 1.0]);
        for r = 1:4; xlim(ax(r), [0 t(end)]); set(ax(r), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW); grid(ax(r), 'off'); end
        lg = legend(ax(1), h1, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'AutoUpdate', 'off');
        lg.Title.String = sprintf('%s wall, plane init; %s', W{iw}, ttl);
        legend(ax(3), h3, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'AutoUpdate', 'off');
        xlabel(ax(4), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        if iw == 1
            ylabel(ax(1), 'a_z / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax(2), 'e_a  (true - est)', 'FontSize', FS, 'FontWeight', 'bold');
            ylabel(ax(3), 'c', 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax(4), 'R(h_d - h)  [nm], seed 1', 'FontSize', FS, 'FontWeight', 'bold');
        end
    end
    out = fullfile(fd, out_name);  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end

function local_pfix(id, fd, W)
    PF = [10 15 20];  C = {[0.45 0.25 0.85], [0.9 0.5 0], [0.1 0.65 0.3]};  cT = [0.8 0 0];  FS = 15;  LFS = 11;  AXLW = 1.8;
    f = figure('Position', [40 40 1600 900], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for iw = 1:2
        a1 = nexttile(tl, iw);  hold on; box on;  a2 = nexttile(tl, 2 + iw);  hold on; box on;  h = gobjects(0);
        for ip = 1:3
            S = load(fullfile(id, sprintf('f1p2_%s_PI_sig010_rho%02d_rtri_f1_lc040.mat', W{iw}, PF(ip))));  t = S.t(:);
            e = S.a_true - S.a_hat;  m = mean(e, 2);  s = std(e, 0, 2);
            fill(a1, [t; flipud(t)], [m - s; flipud(m + s)], C{ip}, 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');
            h(end + 1) = plot(a1, t, m, '-', 'Color', C{ip}, 'LineWidth', 1.4, 'DisplayName', sprintf('\\surdP_{fix} %.2f', PF(ip) / 100)); %#ok<AGROW>
            if ip == 1; ce = S.c_eff_d(:); ce(S.wd(:) < 1) = NaN; plot(a2, t, ce, '-', 'Color', cT, 'LineWidth', 1.4); end
            plot(a2, t, mean(S.c_hat, 2), '-', 'Color', C{ip}, 'LineWidth', 1.7);
        end
        plot(a1, [0 3], [0 0], ':', 'Color', [0.3 0.3 0.3], 'HandleVisibility', 'off');
        ylim(a1, [-0.1 0.1]);  ylim(a2, [-0.4 1.0]);
        for ax = [a1 a2]; xlim(ax, [0 3]); set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW); grid(ax, 'off'); end
        lg = legend(a1, h, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'AutoUpdate', 'off');
        lg.Title.String = sprintf('%s wall, plane init; \\lambda_c 0.4, ramp\\_tri 1 Hz (red = c_{eff})', W{iw});
        xlabel(a2, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        if iw == 1; ylabel(a1, 'e_a  (true - est)', 'FontSize', FS, 'FontWeight', 'bold'); ylabel(a2, 'c', 'FontSize', FS, 'FontWeight', 'bold'); end
    end
    out = fullfile(fd, 'fig_conly_lc040_pfix.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
