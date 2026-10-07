% STATUS: SCRATCH -- 2026-10-06, plots run_conly_h2off.m: baseline H (blue) vs H23 = H25 = 0 (orange), single sqrt(P_fix) 0.15, 10 seeds.
%   One figure per trajectory (fig_conly_h2off_<traj>.png); columns plane | cell.
%   Rows: (1) e_abar seed mean +- sd  (2) c_hat seed mean with c_eff(h_d) red  (3) I[k] seed mean (log).
%   Console: wiring (max |difference| from baseline, must be > 0), descent end, troughs, triangle and near-wall |e_abar|,
%   seed sd, c_hat end, sqrt(p55) end, information in the descent / triangle.
function plot_conly_h2off()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_plaw_walls');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    TR = {'_rtri_f1', 1.0, 'narrow 1.1-2 R, 1 Hz, T_d 1 s', 'narrow';
          '_rtri_f2p5_td0p5_top1000', 0.5, 'wide 1.1-10 R, 2.5 Hz, T_d 0.5 s', 'wide'};
    ARM = {'_sig010_rho15', 'H as derived';  '_sig010_rho15_h2off', 'H_{23} = H_{25} = 0'};
    C = {[0 0.2 0.9], [0.9 0.5 0]};  cT = [0.8 0 0];  FS = 15;  LFS = 11;  AXLW = 1.8;  W = {'plane', 'cell'};
    for it = 1:2
        f = figure('Position', [40 40 1600 1250], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
        tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
        for iw = 1:2
            ax = gobjects(3, 1);  for r = 1:3; ax(r) = nexttile(tl, (r - 1) * 2 + iw); hold(ax(r), 'on'); box(ax(r), 'on'); end
            h = gobjects(0);  B = [];
            for ia = 1:2
                S = load(fullfile(id, sprintf('f1p2_%s_PI%s%s.mat', W{iw}, ARM{ia, 1}, TR{it, 1})));  t = S.t(:);  Td = TR{it, 2};
                e = S.a_true - S.a_hat;  m = mean(e, 2);  s = std(e, 0, 2);  I = mean(S.q55a_stat, 2);
                fill(ax(1), [t; flipud(t)], [m - s; flipud(m + s)], C{ia}, 'FaceAlpha', 0.18, 'EdgeColor', 'none', 'HandleVisibility', 'off');
                h(end + 1) = plot(ax(1), t, m, '-', 'Color', C{ia}, 'LineWidth', 1.5, 'DisplayName', ARM{ia, 2}); %#ok<AGROW>
                if ia == 1; ce = S.c_eff_d(:); ce(S.wd(:) < 1) = NaN; plot(ax(2), t, ce, '-', 'Color', cT, 'LineWidth', 1.3); end
                plot(ax(2), t, mean(S.c_hat, 2), '-', 'Color', C{ia}, 'LineWidth', 1.7);
                plot(ax(3), t, max(I, 1e-6), '-', 'Color', C{ia}, 'LineWidth', 1.2);
                tri = t > Td;  nw = tri & S.wd(:) < 2;  k = @(tt) find(t >= tt - 1e-9, 1);
                if ia == 1; B = S; dif = NaN; else; dif = max(abs(S.a_hat(:) - B.a_hat(:))); end
                fprintf('%-6s %-5s %-20s | wiring max|d a_hat| %.3g | e_a descent end %+.3f | tri |e_a| %.4f near-wall %.4f seed-sd %.4f | c_hat end %.3f | sqrtP end %.3f | info descent %.0f tri %.0f\n', ...
                    TR{it, 4}, W{iw}, strrep(strrep(ARM{ia, 2}, '_{', ''), '}', ''), dif, m(k(Td)), mean(abs(e(tri, :)), 'all'), mean(abs(e(nw, :)), 'all'), ...
                    std(mean(e(tri, :), 1)), mean(S.c_hat(end, :)), mean(S.sP_c(end, :)), sum(I(~tri)), sum(I(tri)));
            end
            plot(ax(1), [0 t(end)], [0 0], ':', 'Color', [0.3 0.3 0.3], 'HandleVisibility', 'off');
            ylim(ax(1), [-0.1 0.1]);  ylim(ax(2), [-0.4 1.0]);  set(ax(3), 'YScale', 'log');  ylim(ax(3), [1e-4 10]);
            for r = 1:3; xlim(ax(r), [0 t(end)]); set(ax(r), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW); grid(ax(r), 'off'); end
            lg = legend(ax(1), h, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'AutoUpdate', 'off');
            lg.Title.String = sprintf('%s wall, plane init; %s (red in row 2 = c_{eff})', W{iw}, TR{it, 3});
            xlabel(ax(3), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
            if iw == 1
                ylabel(ax(1), 'e_a  (true - est)', 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax(2), 'c', 'FontSize', FS, 'FontWeight', 'bold');
                ylabel(ax(3), 'I[k]', 'FontSize', FS, 'FontWeight', 'bold');
            end
        end
        out = fullfile(fd, sprintf('fig_conly_h2off_%s.png', TR{it, 4}));  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
    end
end
