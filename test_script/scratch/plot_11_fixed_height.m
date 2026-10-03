% PURPOSE (2026-09-18): figures of the fixed-height test (run_11_fixed_height.m), same style as plot_11_ladder.m
%   minus the trajectory row (the height is a constant, it is written in the identification line):
%     fixed11_<plant>_h<h>.png   3 rows x (A | B): a_true vs a_hat +- sigma_seed; e_a = a_true - a_hat; the constants
%                                c0_hat (green), d1_hat (blue) +- sigma_seed, +- sqrt(P) dashed, true values (red / orange)
%     fixed11_<plant>_cd.png     the (c0, d1) plane: for each height the line c0 - a d1 = g (a - 1) that a single height
%                                can pin, the 10-seed end points of A (dots) and B (triangles), the seed (square) and the
%                                plant's band-fit truth (star). One height = a line; the truth is where the lines cross.
%   Output reference/eq17_analysis/derivation/figures/ | EXPIRES: with the [1/1] two-wall line
function plot_11_fixed_height(res_dir)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    if nargin < 1 || isempty(res_dir); res_dir = fullfile(root, 'test_results', 'temp_11_fixed_height'); end
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; BANDC = [0.45 0.55 0.95]; COL_C0 = [0 0.5 0]; COL_C0T = [0.8 0.3 0];
    FS = 16; LFS = 12; AXLW = 2.0;
    for pl = {'cell', 'plane'}
        files = dir(fullfile(res_dir, sprintf('%s_A_h*.mat', pl{1})));
        if isempty(files); continue; end
        hs = [];  RA = {};  RB = {};
        for k = 1:numel(files)
            fa = fullfile(res_dir, files(k).name);  fb = strrep(fa, '_A_', '_B_');
            if ~exist(fb, 'file'); continue; end
            RA{end+1} = load(fa);  RB{end+1} = load(fb);  hs(end+1) = RA{end}.h;              %#ok<AGROW>
        end
        [hs, ord] = sort(hs, 'descend');  RA = RA(ord);  RB = RB(ord);
        % ---- per-height figures ----
        for k = 1:numel(hs)
            R = {RA{k}, RB{k}};  NAME = {'A  (c_0, d_1) states', 'B  a slot'};
            t = R{1}.t;  T_END = t(end);
            f = figure('Position', [40 40 1500 900], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
            tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');  ax = gobjects(3, 2);
            for v = 1:2
                r = R{v};  ea = r.a_true - r.a_hat;
                ax(1, v) = nexttile(v); hold on; box on;
                am = mean(r.a_hat, 2); as = std(r.a_hat, 0, 2);
                hb = fill([t; flipud(t)], [am - as; flipud(am + as)], BANDC, 'FaceAlpha', 0.30, 'EdgeColor', 'none', 'DisplayName', 'a_{hat} \pm \sigma_{seed}');
                ht = plot(t, mean(r.a_true, 2), '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'a_{true} / a_{nom}');
                hh = plot(t, am, '-', 'Color', COL_HAT, 'LineWidth', 2.0, 'DisplayName', 'a_{hat} / a_{nom}');
                h0 = plot(NaN, NaN, 'w.', 'DisplayName', sprintf('%s plant, hold w = %.2f R, version %s: seed (c_0, d_1) = (%.3f, %.3f), prior %.2f', ...
                          upper(pl{1}), r.h, r.version, r.seed_cd(1), r.seed_cd(2), r.sig(1)));
                xlim([0 T_END]); yl = [min(am - as) - 0.05, max(am + as) + 0.05]; yl = [max(0, yl(1)), min(1, yl(2))]; ylim(yl);
                if v == 1; ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold'); end
                legend([h0 ht hh hb], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS - 1, 'FontWeight', 'bold', 'Box', 'on');
                local_ax(FS, AXLW);
                ax(2, v) = nexttile(2 + v); hold on; box on;
                em = mean(ea, 2); es = std(ea, 0, 2);
                hb = fill([t; flipud(t)], [em - es; flipud(em + es)], BANDC, 'FaceAlpha', 0.30, 'EdgeColor', 'none', 'DisplayName', '\pm \sigma_{seed}');
                yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
                he = plot(t, em, '-', 'Color', COL_HAT, 'LineWidth', 2.0, 'DisplayName', 'e_a = a_{true} - a_{hat}  (seed mean)');
                xlim([0 T_END]); ylim([-0.1 0.1]);
                if v == 1; ylabel('e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold'); end
                legend([he hb], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
                local_ax(FS, AXLW);
                ax(3, v) = nexttile(4 + v); hold on; box on;
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
                xlim([0 T_END]); ylim([-1.5 2.0]);
                if v == 1; ylabel('c_0,  d_1', 'FontSize', FS, 'FontWeight', 'bold'); end
                xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
                legend([hd hr hc hct hb], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS - 1, 'FontWeight', 'bold', 'Box', 'on');
                local_ax(FS, AXLW);
                title(NAME{v}, 'FontSize', LFS, 'FontWeight', 'normal');
            end
            for rr = 1:3; linkaxes(ax(rr, :), 'y'); end
            fn = fullfile(od, sprintf('fixed11_%s_h%s.png', pl{1}, strrep(sprintf('%.2f', hs(k)), '.', 'p')));
            exportgraphics(f, fn, 'Resolution', 150); close(f);  fprintf('figure: %s\n', fn);
        end
        % ---- (c0, d1) plane ----
        f = figure('Position', [40 40 900 760], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off'); hold on; box on;
        COLS = [0 0.2 0.9; 0 0.55 0.2; 0.55 0 0.6];  H = gobjects(1, 0);
        c0g = linspace(-1.5, 1.0, 200);
        for k = 1:numel(hs)
            r = RA{k};  a = r.a_true_h;  g = r.g;
            d1_line = (c0g - g * (a - 1)) / a;
            H(end+1) = plot(c0g, d1_line, '-', 'Color', COLS(k, :), 'LineWidth', 1.6, 'DisplayName', sprintf('w = %.2f R: c_0 - a d_1 = g(a-1), a = %.3f', r.h, a)); %#ok<AGROW>
            plot(r.c0_hat(end, :), r.d1_hat(end, :), 'o', 'Color', COLS(k, :), 'MarkerFaceColor', COLS(k, :), 'MarkerSize', 6, 'HandleVisibility', 'off');
            rb = RB{k};
            plot(rb.c0_hat(end, :), rb.d1_hat(end, :), '^', 'Color', COLS(k, :), 'MarkerSize', 7, 'LineWidth', 1.2, 'HandleVisibility', 'off');
        end
        H(end+1) = plot(RA{1}.seed_cd(1), RA{1}.seed_cd(2), 's', 'Color', [0.3 0.3 0.3], 'MarkerSize', 11, 'LineWidth', 2, 'DisplayName', 'seed');
        H(end+1) = plot(RA{1}.true_cd(1), RA{1}.true_cd(2), 'p', 'Color', COL_TRUE, 'MarkerFaceColor', COL_TRUE, 'MarkerSize', 16, 'DisplayName', 'true (band fit)');
        H(end+1) = plot(NaN, NaN, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6, 'DisplayName', 'A end points (10 seeds)');
        H(end+1) = plot(NaN, NaN, 'k^', 'MarkerSize', 7, 'LineWidth', 1.2, 'DisplayName', 'B end points');
        xlim([-1.5 1.0]); ylim([-0.5 2.5]);
        xlabel('c_0', 'FontSize', FS, 'FontWeight', 'bold'); ylabel('d_1', 'FontSize', FS, 'FontWeight', 'bold');
        legend(H, 'Location', 'northoutside', 'Orientation', 'vertical', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'NumColumns', 2);
        local_ax(FS, AXLW);
        fn = fullfile(od, sprintf('fixed11_%s_cd.png', pl{1}));  exportgraphics(f, fn, 'Resolution', 150); close(f);  fprintf('figure: %s\n', fn);
    end
end

function local_ax(FS, AXLW)
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
end
