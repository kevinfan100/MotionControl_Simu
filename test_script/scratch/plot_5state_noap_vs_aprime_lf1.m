% PURPOSE (2026-09-24, user: "two panels: lambda_f = 1 with delta a, and lambda_f = 1 with a'"): same ramp (hold 0.5 s,
%   22.2 -> 1.10 R constant velocity over T_d = 2 s, hold 0.5 s), plane plant, paired seeds 1:5, NO forgetting in either arm.
%   Left : Layer-1, no a', second order a + delta a, lambda_f = 1     test_results/temp_5state_noap/ladder_ramp16_Td2_lf1_o2.mat
%   Right: a'(h, c) law (1-c)/h^2, c state, plane init, lambda_f = 1  test_results/temp_plaw_walls/f1p2_plane_PI_sig010_ramp2.mat
%   Rows: a_true / a_hat | e_a = a_true - a_hat (mean, +-sigma_seed band, +-sqrt(P44) dashed) | the estimated parameter:
%   left delta a_hat vs true increment (+-sqrt(P55)), right c_hat vs the plane band-fit constant (+-sqrt(P55)).  STATUS: SCRATCH
function plot_5state_noap_vs_aprime_lf1()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    SEEDS = 1:5;
    A = {fullfile(root, 'test_results', 'temp_5state_noap', 'ladder_ramp16_Td2_lf1_o2.mat'),  'Layer-1: a + \delta a, no a'', \lambda_f = 1'; ...
         fullfile(root, 'test_results', 'temp_plaw_walls', 'f1p2_plane_PI_sig010_ramp2.mat'), 'a''(h, c) law, c state, \lambda_f = 1'};
    COL_TRUE = [0.8 0 0]; C2 = [0 0.2 0.9]; B2 = [0.45 0.55 0.95]; FS = 18; LFS = 12; AXLW = 2.0;
    f = figure('Position', [40 40 1600 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = gobjects(3, 2);
    fprintf('%-42s | near (h<3R) mean  sd | hold-end mean  sd | far (h>10R) sd | honesty near / end\n', 'arm');
    for c = 1:2
        r = load(A{c, 1});  t = r.t(:);  q = SEEDS;
        if c == 1; hb = r.hb(:, q); sP = sqrt(r.P_a(:, q)); else; hb = r.h_true(:, q); sP = r.sP_a(:, q); end
        at = r.a_true(:, q);  ah = r.a_hat(:, q);  ea = at - ah;  seg = r.seg;
        m = mean(ea, 2); s = std(ea, 0, 2); sp = mean(sP, 2); hm = mean(hb, 2);
        % row 1
        ax(1, c) = nexttile(tl, c); hold(ax(1, c), 'on');
        h0 = plot(ax(1, c), nan, nan, 'w-', 'DisplayName', A{c, 2});
        ht = plot(ax(1, c), t, mean(at, 2), '-', 'Color', COL_TRUE, 'LineWidth', 2.4, 'DisplayName', 'a_{true} / a_{nom}');
        h1 = plot(ax(1, c), t, mean(ah, 2), '-', 'Color', C2, 'LineWidth', 1.8, 'DisplayName', 'a_{hat}');
        ylim(ax(1, c), [0 1.1]);
        legend(ax(1, c), [h0 ht h1], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'AutoUpdate', 'off');
        % row 2
        ax(2, c) = nexttile(tl, 2 + c); hold(ax(2, c), 'on');
        fill(ax(2, c), [t; flipud(t)], [m + s; flipud(m - s)], B2, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
        h2 = plot(ax(2, c), t, m, '-', 'Color', C2, 'LineWidth', 1.8, 'DisplayName', 'e_a  (band = \pm\sigma_{seed})');
        h4 = plot(ax(2, c), t, sp, '--', 'Color', C2, 'LineWidth', 1.2, 'DisplayName', '\pm sqrt(P_{44})'); plot(ax(2, c), t, -sp, '--', 'Color', C2, 'LineWidth', 1.2);
        plot(ax(2, c), [0 t(end)], [0 0], '-', 'Color', [0.6 0.6 0.6]);
        ylim(ax(2, c), [-0.45 0.15]);
        legend(ax(2, c), [h2 h4], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'AutoUpdate', 'off');
        % row 3: the parameter each arm estimates (left: rate slot delta a; right: the law constant c)
        ax(3, c) = nexttile(tl, 4 + c); hold(ax(3, c), 'on');
        if c == 1
            da = r.da_hat(:, q) * 1e3;  sPd = sqrt(r.P_da(:, q)) * 1e3;  dtr = movmean(mean(r.da_true(:, q), 2), max(1, round(0.25 / (t(2) - t(1))))) * 1e3;
            md = mean(da, 2);  sd = std(da, 0, 2);  spd = mean(sPd, 2);
            fill(ax(3, c), [t; flipud(t)], [md + sd; flipud(md - sd)], B2, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
            h7 = plot(ax(3, c), t, dtr, '-', 'Color', COL_TRUE, 'LineWidth', 2.4, 'DisplayName', 'true increment (0.25 s mean)');
            h5 = plot(ax(3, c), t, md, '-', 'Color', C2, 'LineWidth', 1.8, 'DisplayName', '\delta a_{hat}  (band = \pm\sigma_{seed})');
            h6 = plot(ax(3, c), t, md + spd, '--', 'Color', C2, 'LineWidth', 1.2, 'DisplayName', '\pm sqrt(P_{55})'); plot(ax(3, c), t, md - spd, '--', 'Color', C2, 'LineWidth', 1.2);
            plot(ax(3, c), [0 t(end)], [0 0], '-', 'Color', [0.6 0.6 0.6]);
            ylabel(ax(3, c), '\delta a / a_{nom}  [10^{-3} / step]', 'FontSize', FS, 'FontWeight', 'bold');
        else
            ch = r.c_hat(:, q);  sPc = r.sP_c(:, q);  mc = mean(ch, 2);  sc = std(ch, 0, 2);  spc = mean(sPc, 2);
            fill(ax(3, c), [t; flipud(t)], [mc + sc; flipud(mc - sc)], B2, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
            c_fit = mean(ch(1, :));   % plane band fit at p = 2 = the constant the law is built on (also the init)
            h7 = plot(ax(3, c), [0 t(end)], [c_fit c_fit], '-', 'Color', COL_TRUE, 'LineWidth', 2.4, 'DisplayName', sprintf('c (plane band fit) = %.3f', c_fit));
            h5 = plot(ax(3, c), t, mc, '-', 'Color', C2, 'LineWidth', 1.8, 'DisplayName', 'c_{hat}  (band = \pm\sigma_{seed})');
            h6 = plot(ax(3, c), t, mc + spc, '--', 'Color', C2, 'LineWidth', 1.2, 'DisplayName', '\pm sqrt(P_{55})'); plot(ax(3, c), t, mc - spc, '--', 'Color', C2, 'LineWidth', 1.2);
            ylabel(ax(3, c), 'c', 'FontSize', FS, 'FontWeight', 'bold');  ylim(ax(3, c), [-0.3 0.3]);
            fprintf('   c_hat end %.3f +- %.3f (sqrtP %.3f), c_eff at 1.10 R %.3f, plane init %.3f\n', mean(ch(end, :)), std(ch(end, :)), spc(end), r.c_eff_d(end), mean(ch(1, :)));
        end
        legend(ax(3, c), [h7 h5 h6], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'AutoUpdate', 'off');
        xlabel(ax(3, c), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        for rr = 1:3
            xlim(ax(rr, c), [0 t(end)]);  yl = ylim(ax(rr, c));
            for sgm = seg(:).'; plot(ax(rr, c), [sgm sgm], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2); end
            set(ax(rr, c), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
        end
        m_near = hm < 3 & t > seg(1) & t <= seg(2);  m_end = t > seg(2);  m_far = t > seg(1) & hm > 10;
        fprintf('%-42s | %+.3f  %.3f | %+.3f  %.3f | %.3f | %.2f / %.2f\n', strrep(A{c, 2}, '\', ''), mean(m(m_near)), mean(s(m_near)), ...
            mean(m(m_end)), mean(s(m_end)), mean(s(m_far)), mean(s(m_near)) / mean(sp(m_near)), mean(s(m_end)) / mean(sp(m_end)));
    end
    ylabel(ax(1, 1), 'a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax(2, 1), 'e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold');
    out = fullfile(fd, 'fig_noap_vs_aprime_lf1_Td2.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
