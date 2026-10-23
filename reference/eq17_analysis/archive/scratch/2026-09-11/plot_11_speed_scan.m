% PURPOSE (2026-09-18): figures of the speed scan (run_11_speed_scan.m), ladder style, one column per descent speed:
%     speed11_<plant>_<ver>.png   4 rows x 3 columns (T_d fast | mid | slow): w_d; a_true vs a_hat +- sigma_seed;
%                                 e_a = a_true - a_hat; c0_hat (green), d1_hat (blue) +- sigma_seed, +- sqrt(P) dashed,
%                                 true values (red / orange). Each column has its own time axis.
%     speed11_<plant>_<ver>_cd.png  the (c0, d1) plane: the two boundary lines (h_init, h_bottom), 10-seed end points per
%                                 speed (colour), the seed (square) and the band-fit truth (star).
%   Output reference/eq17_analysis/derivation/figures/ | EXPIRES: with the [1/1] two-wall line
function plot_11_speed_scan(res_dir)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    if nargin < 1 || isempty(res_dir); res_dir = fullfile(root, 'test_results', 'temp_11_speed_scan'); end
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; BANDC = [0.45 0.55 0.95]; COL_C0 = [0 0.5 0]; COL_C0T = [0.8 0.3 0];
    FS = 15; LFS = 11; AXLW = 2.0;
    for pl = {'cell', 'plane'}
        for ver = {'A', 'B'}
            files = dir(fullfile(res_dir, sprintf('%s_%s_Td*.mat', pl{1}, ver{1})));
            if isempty(files); continue; end
            RR = {};  Tds = [];
            for k = 1:numel(files); RR{end+1} = load(fullfile(res_dir, files(k).name)); Tds(end+1) = RR{end}.Td; end %#ok<AGROW>
            [Tds, ord] = sort(Tds);  RR = RR(ord);  nc = numel(RR);
            f = figure('Position', [40 40 1700 1150], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
            tiledlayout(4, nc, 'TileSpacing', 'compact', 'Padding', 'compact');  ax = gobjects(4, nc);
            for c = 1:nc
                r = RR{c};  t = r.t;  T_END = t(end);  ea = r.a_true - r.a_hat;
                t1 = r.t_hold; t2 = t1 + r.Td;
                % row 1: commanded height
                ax(1, c) = nexttile(c); hold on; box on;
                h0 = plot(NaN, NaN, 'w.', 'DisplayName', sprintf('%s plant, version %s, T_d = %g s (peak %.1f R/s): seed (c_0, d_1) = (%.3f, %.3f), prior %.2f', ...
                          upper(pl{1}), r.version, r.Td, pi/2 * (r.h_init - r.h_bottom) / r.Td, r.seed_cd(1), r.seed_cd(2), r.sig(1)));
                hw = plot(t, r.wd, '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'w_d / R');
                for tt = [t1 t2]; xline(tt, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
                xlim([0 T_END]); ylim([0 6]);
                if c == 1; ylabel('w / R', 'FontSize', FS, 'FontWeight', 'bold'); end
                legend([h0 hw], 'Location', 'northoutside', 'Orientation', 'vertical', 'FontSize', LFS - 1, 'FontWeight', 'bold', 'Box', 'on');
                local_ax(FS, AXLW);
                % row 2: gain
                ax(2, c) = nexttile(nc + c); hold on; box on;
                am = mean(r.a_hat, 2); as = std(r.a_hat, 0, 2);
                hb = fill([t; flipud(t)], [am - as; flipud(am + as)], BANDC, 'FaceAlpha', 0.30, 'EdgeColor', 'none', 'DisplayName', 'a_{hat} \pm \sigma_{seed}');
                ht = plot(t, mean(r.a_true, 2), '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'a_{true} / a_{nom}');
                hh = plot(t, am, '-', 'Color', COL_HAT, 'LineWidth', 2.0, 'DisplayName', 'a_{hat} / a_{nom}');
                for tt = [t1 t2]; xline(tt, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
                xlim([0 T_END]); ylim([0 1]);
                if c == 1; ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold'); end
                legend([ht hh hb], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
                local_ax(FS, AXLW);
                % row 3: gain error
                ax(3, c) = nexttile(2 * nc + c); hold on; box on;
                em = mean(ea, 2); es = std(ea, 0, 2);
                hb = fill([t; flipud(t)], [em - es; flipud(em + es)], BANDC, 'FaceAlpha', 0.30, 'EdgeColor', 'none', 'DisplayName', '\pm \sigma_{seed}');
                yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
                he = plot(t, em, '-', 'Color', COL_HAT, 'LineWidth', 2.0, 'DisplayName', 'e_a = a_{true} - a_{hat}  (seed mean)');
                for tt = [t1 t2]; xline(tt, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
                xlim([0 T_END]); ylim([-0.1 0.1]);
                if c == 1; ylabel('e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold'); end
                legend([he hb], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
                local_ax(FS, AXLW);
                % row 4: constants
                ax(4, c) = nexttile(3 * nc + c); hold on; box on;
                dm = mean(r.d1_hat, 2); ds = std(r.d1_hat, 0, 2); dp = mean(r.sP_d1, 2);
                cm = mean(r.c0_hat, 2); cs = std(r.c0_hat, 0, 2); cp = mean(r.sP_c0, 2);
                hb = fill([t; flipud(t)], [dm - ds; flipud(dm + ds)], BANDC, 'FaceAlpha', 0.30, 'EdgeColor', 'none', 'DisplayName', '\pm \sigma_{seed}');
                hr = plot(t, r.true_cd(2) * ones(size(t)), '-', 'Color', COL_TRUE, 'LineWidth', 1.6, 'DisplayName', 'd_1 true (band fit)');
                hd = plot(t, dm, '-', 'Color', COL_HAT, 'LineWidth', 2.0, 'DisplayName', 'd_{1,hat}');
                plot(t, dm + dp, '--', 'Color', COL_HAT, 'LineWidth', 1.0, 'HandleVisibility', 'off');
                plot(t, dm - dp, '--', 'Color', COL_HAT, 'LineWidth', 1.0, 'HandleVisibility', 'off');
                fill([t; flipud(t)], [cm - cs; flipud(cm + cs)], [0.55 0.85 0.55], 'FaceAlpha', 0.30, 'EdgeColor', 'none', 'HandleVisibility', 'off');
                hc = plot(t, cm, '-', 'Color', COL_C0, 'LineWidth', 2.0, 'DisplayName', 'c_{0,hat}');
                hct = plot(t, r.true_cd(1) * ones(size(t)), '-', 'Color', COL_C0T, 'LineWidth', 1.4, 'DisplayName', 'c_0 true (band fit)');
                plot(t, cm + cp, '--', 'Color', COL_C0, 'LineWidth', 1.0, 'HandleVisibility', 'off');
                plot(t, cm - cp, '--', 'Color', COL_C0, 'LineWidth', 1.0, 'HandleVisibility', 'off');
                for tt = [t1 t2]; xline(tt, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
                xlim([0 T_END]); ylim([-1.5 2.0]);
                if c == 1; ylabel('c_0,  d_1', 'FontSize', FS, 'FontWeight', 'bold'); end
                xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
                legend([hd hr hc hct hb], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS - 1, 'FontWeight', 'bold', 'Box', 'on');
                local_ax(FS, AXLW);
            end
            fn = fullfile(od, sprintf('speed11_%s_%s.png', pl{1}, ver{1}));
            exportgraphics(f, fn, 'Resolution', 150); close(f);  fprintf('figure: %s\n', fn);
            % ---- (c0, d1) plane ----
            f = figure('Position', [40 40 900 760], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off'); hold on; box on;
            COLS = [0 0.2 0.9; 0 0.55 0.2; 0.55 0 0.6];  H = gobjects(1, 0);  r = RR{1};
            c0g = linspace(-1.5, 1.0, 200);
            HA = [r.h_init, r.a_true_top; r.h_bottom, r.a_true_bot];
            for i = 1:2
                h = HA(i, 1); a = HA(i, 2); g = h - r.W_S;
                H(end+1) = plot(c0g, (c0g - g * (a - 1)) / a, '-', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.4, ...
                                'DisplayName', sprintf('w = %.2f R: c_0 - a d_1 = g(a-1), a = %.3f', h, a)); %#ok<AGROW>
            end
            for c = 1:nc
                rc = RR{c};
                H(end+1) = plot(rc.c0_hat(end, :), rc.d1_hat(end, :), 'o', 'Color', COLS(c, :), 'MarkerFaceColor', COLS(c, :), 'MarkerSize', 7, ...
                                'LineStyle', 'none', 'DisplayName', sprintf('T_d = %g s end points (10 seeds)', rc.Td)); %#ok<AGROW>
            end
            H(end+1) = plot(r.seed_cd(1), r.seed_cd(2), 's', 'Color', [0.3 0.3 0.3], 'MarkerSize', 11, 'LineWidth', 2, 'DisplayName', 'seed');
            H(end+1) = plot(r.true_cd(1), r.true_cd(2), 'p', 'Color', COL_TRUE, 'MarkerFaceColor', COL_TRUE, 'MarkerSize', 16, 'DisplayName', 'true (band fit)');
            xlim([-1.5 1.0]); ylim([-0.5 2.5]);
            xlabel('c_0', 'FontSize', FS, 'FontWeight', 'bold'); ylabel('d_1', 'FontSize', FS, 'FontWeight', 'bold');
            legend(H, 'Location', 'northoutside', 'Orientation', 'vertical', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'NumColumns', 2);
            local_ax(FS, AXLW);
            fn = fullfile(od, sprintf('speed11_%s_%s_cd.png', pl{1}, ver{1}));  exportgraphics(f, fn, 'Resolution', 150); close(f);  fprintf('figure: %s\n', fn);
        end
    end
end

function local_ax(FS, AXLW)
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
end
