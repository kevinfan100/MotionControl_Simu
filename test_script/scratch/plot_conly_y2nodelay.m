% STATUS: SCRATCH -- 2026-10-07, plots run_conly_y2nodelay.m: current filter (y2_pred walks the gain back d steps, full H; blue)
%   vs ignoring the y2 delay (H23 = H25 = 0, y2_pred = s a_hat; orange). Single sqrt(P_fix) 0.15, lambda_c 0.7, 10 seeds.
%   One figure per trajectory fig_conly_y2nodelay_<traj>.png, columns plane | cell.
%   Rows: (1) abar true red / est (seed mean)  (2) e_abar seed mean +- sd  (3) c_hat seed mean with c_eff(h_d) red.
%   Console: health (NaN, min h, NIS2), descent / hold / point criteria, near-wall |e_abar|, paired mean difference in the
%   near-wall motion window (same seeds), with its SEM.
function plot_conly_y2nodelay()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_plaw_walls');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    TR = {'_ramp2',                    'ramp: hold 0.5 s, descent T_d 2 s, hold 0.5 s', 'ramp',   [0.5 2.5];
          '_rtri_f1',                  'ramp\_tri 1.1-2 R, 1 Hz, T_d 1 s',              'narrow', [0 1.0];
          '_rtri_f2p5_td0p5_top1000',  'wide 1.1-10 R, 2.5 Hz, T_d 0.5 s',              'wide',   [0 0.5]};
    ARM = {'_sig010_rho15', 'y_2 walks back d steps (current)';  '_sig010_rho15_y2nodelay', 'y_2 delay ignored'};
    C = {[0 0.2 0.9], [0.9 0.5 0]};  cT = [0.8 0 0];  FS = 15;  LFS = 11;  AXLW = 1.8;  W = {'plane', 'cell'};
    for it = 1:3
        f = figure('Position', [40 40 1600 1250], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
        tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
        for iw = 1:2
            ax = gobjects(3, 1);  for r = 1:3; ax(r) = nexttile(tl, (r - 1) * 2 + iw); hold(ax(r), 'on'); box(ax(r), 'on'); end
            h1 = gobjects(0);  E = cell(1, 2);
            for ia = 1:2
                S = load(fullfile(id, sprintf('f1p2_%s_PI%s%s.mat', W{iw}, ARM{ia, 1}, TR{it, 1})));  t = S.t(:);
                if ia == 1; h1(end + 1) = plot(ax(1), t, mean(S.a_true, 2), '-', 'Color', cT, 'LineWidth', 1.5, 'DisplayName', 'a_z/a_{nom} true'); end %#ok<AGROW>
                h1(end + 1) = plot(ax(1), t, mean(S.a_hat, 2), '-', 'Color', C{ia}, 'LineWidth', 1.2, 'DisplayName', ARM{ia, 2}); %#ok<AGROW>
                e = S.a_true - S.a_hat;  E{ia} = e;  m = mean(e, 2);  s = std(e, 0, 2);
                fill(ax(2), [t; flipud(t)], [m - s; flipud(m + s)], C{ia}, 'FaceAlpha', 0.18, 'EdgeColor', 'none');  plot(ax(2), t, m, '-', 'Color', C{ia}, 'LineWidth', 1.4);
                if ia == 1; ce = S.c_eff_d(:); ce(S.wd(:) < 1) = NaN; plot(ax(3), t, ce, '-', 'Color', cT, 'LineWidth', 1.4); end
                plot(ax(3), t, mean(S.c_hat, 2), '-', 'Color', C{ia}, 'LineWidth', 1.7);
                seg = TR{it, 4};  mot = t > 0.3 & t <= seg(2) + 0.02;  nw = S.wd(:) < 2 & t > 0.1;  nis2 = mean(S.innov2(t > 0.1, :).^2 ./ S.S2(t > 0.1, :), 'all', 'omitnan');
                if strcmp(TR{it, 3}, 'ramp'); hold_w = t > seg(2) + 0.1; extra = sprintf('hold mean %+.4f', mean(m(hold_w))); else; extra = sprintf('tri |e_a| %.4f', mean(abs(e(t > seg(2), :)), 'all')); end
                fprintf('%-6s %-5s %-34s | NaN %d min h %.3f NIS2 %.2f | descent max|mean| %.3f | near-wall |e_a| %.4f mean %+.4f | %s\n', ...
                    TR{it, 3}, W{iw}, ARM{ia, 2}, nnz(isnan(S.a_hat)), min(S.h_true(:)), nis2, max(abs(m(mot))), mean(abs(e(nw, :)), 'all'), mean(e(nw, :), 'all'), extra);
            end
            d = mean(E{2}(nw, :) - E{1}(nw, :), 1);   % paired difference per seed, near-wall window
            fprintf('%-6s %-5s paired (ignored - current) near-wall e_a: %+.4f +- %.4f (SEM)\n', TR{it, 3}, W{iw}, mean(d), std(d) / sqrt(numel(d)));
            plot(ax(2), [0 t(end)], [0 0], ':', 'Color', [0.3 0.3 0.3]);
            ylim(ax(1), [0 1.1]);  ylim(ax(2), [-0.1 0.1]);  ylim(ax(3), [-0.4 1.0]);
            for r = 1:3; xlim(ax(r), [0 t(end)]); set(ax(r), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW); grid(ax(r), 'off'); end
            lg = legend(ax(1), h1, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'AutoUpdate', 'off');
            lg.Title.String = sprintf('%s wall, plane init; %s (red in row 3 = c_{eff})', W{iw}, TR{it, 2});
            xlabel(ax(3), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
            if iw == 1
                ylabel(ax(1), 'a_z / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax(2), 'e_a  (true - est)', 'FontSize', FS, 'FontWeight', 'bold');
                ylabel(ax(3), 'c', 'FontSize', FS, 'FontWeight', 'bold');
            end
        end
        out = fullfile(fd, sprintf('fig_conly_y2nodelay_%s.png', TR{it, 3}));  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
    end
end
