% PURPOSE (2026-09-17): the ladder figures of the [1/1] two-wall test (run_11_two_walls.m), one figure per plant x arm,
%   versions A (left) and B (right) side by side, shared y per row, house style (plot_var_ahat_6state.m: True red,
%   Estimate blue, +-sigma_seed light-blue band, FontSize 18 bold, legend northoutside, no title, no grid, 150 dpi):
%     row 1  w_d/R commanded and the true height (seed mean), segment dividers
%     row 2  a_bar_true (red, seed mean) and a_bar_hat (blue, mean +- sigma_seed)
%     row 3  e_a = a_bar_true - a_bar_hat, ABSOLUTE: seed mean (blue) +- sigma_seed band
%     row 4  the constants: c0_hat and d1_hat, mean +- sigma_seed band, seed mean +- sqrt(P) (dashed), and the reference
%            (red): the plant's band-fit value on the cell; on the plane the pointwise inversion d1_eff(w_d) = (g + c0)/a_plane(g) - g
%            with c0 at the seed value (the plane is not in the family, so the constant cannot stay put)
%   Output reference/eq17_analysis/derivation/figures/ladder11_<plant>_<arm>.png  | EXPIRES: with the [1/1] two-wall line
function plot_11_ladder(res_dir)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    if nargin < 1 || isempty(res_dir); res_dir = fullfile(root, 'test_results', 'temp_11_two_walls'); end
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; BANDC = [0.45 0.55 0.95]; FS = 16; LFS = 12; AXLW = 2.0;
    for pl = {'plane', 'cell'}
        for arm = {'T', 'P', 'N'}
            fa = fullfile(res_dir, sprintf('%s_A_%s.mat', pl{1}, arm{1}));  fb = fullfile(res_dir, sprintf('%s_B_%s.mat', pl{1}, arm{1}));
            if ~exist(fa, 'file') || ~exist(fb, 'file'); continue; end
            R = {load(fa), load(fb)};  NAME = {'A  (c_0, d_1) states', 'B  a slot'};
            t = R{1}.t;  seg = R{1}.seg;  T_END = t(end);
            f = figure('Position', [40 40 1500 1150], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
            tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
            ax = gobjects(4, 2);
            for v = 1:2
                r = R{v};  ea = r.a_true - r.a_hat;
                % ---- row 1: heights ----
                ax(1, v) = nexttile(v); hold on; box on;
                h1 = plot(t, r.wd, '-', 'Color', [0.35 0.35 0.35], 'LineWidth', 2.0, 'DisplayName', 'w_d / R');
                h2 = plot(t, mean(r.h_true, 2), '-', 'Color', COL_TRUE, 'LineWidth', 1.4, 'DisplayName', 'w / R (true)');
                local_seg(seg, [0 1.05 * max(r.wd)]);  xlim([0 T_END]); ylim([0 1.05 * max(r.wd)]);
                if v == 1; ylabel('w / R', 'FontSize', FS, 'FontWeight', 'bold'); end
                h0 = plot(NaN, NaN, 'w.', 'DisplayName', sprintf('%s plant, arm %s, version %s: seed (c_0, d_1) = (%.3f, %.3f), prior %.2f', ...
                          upper(pl{1}), arm{1}, R{v}.version, r.seed_cd(1), r.seed_cd(2), r.sig(1)));
                legend(h0, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS - 1, 'FontWeight', 'bold', 'Box', 'on');
                local_ax(FS, AXLW);
                % ---- row 2: gain ----
                ax(2, v) = nexttile(2 + v); hold on; box on;
                am = mean(r.a_hat, 2); as = std(r.a_hat, 0, 2);
                hb = fill([t; flipud(t)], [am - as; flipud(am + as)], BANDC, 'FaceAlpha', 0.30, 'EdgeColor', 'none', 'DisplayName', 'a_{hat} \pm \sigma_{seed}');
                ht = plot(t, mean(r.a_true, 2), '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'a_{true} / a_{nom}');
                hh = plot(t, am, '-', 'Color', COL_HAT, 'LineWidth', 2.0, 'DisplayName', 'a_{hat} / a_{nom}');
                local_seg(seg, [0 1]); xlim([0 T_END]); ylim([0 1]);
                if v == 1; ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold'); end
                legend([ht hh hb], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
                local_ax(FS, AXLW);
                % ---- row 3: absolute gain error ----
                ax(3, v) = nexttile(4 + v); hold on; box on;
                em = mean(ea, 2); es = std(ea, 0, 2);
                hb = fill([t; flipud(t)], [em - es; flipud(em + es)], BANDC, 'FaceAlpha', 0.30, 'EdgeColor', 'none', 'DisplayName', '\pm \sigma_{seed}');
                yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
                he = plot(t, em, '-', 'Color', COL_HAT, 'LineWidth', 2.0, 'DisplayName', 'e_a = a_{true} - a_{hat}  (seed mean)');
                local_seg(seg, [-0.06 0.06]); xlim([0 T_END]); ylim([-0.06 0.06]);
                if v == 1; ylabel('e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold'); end
                legend([he hb], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
                local_ax(FS, AXLW);
                % ---- row 4: the constants ----
                ax(4, v) = nexttile(6 + v); hold on; box on;
                if strcmp(pl{1}, 'plane')       % reference = pointwise inversion of the plane polynomial at the CURRENT c0_hat (seed mean)
                    cm0 = mean(r.c0_hat, 2);  g = max(r.wd - r.W_S, 0.02);  ap = zeros(size(g));
                    for i = 1:numel(g); [~, c] = calc_correction_functions(1 + g(i)); ap(i) = 1 / c; end
                    d1_ref = (g + cm0) ./ ap - g;  c0_ref = -0.984 * ones(size(t));  ref_lab = 'd_{1,eff}(w_d; c_{0,hat})  plane';   % c0 reference = the plane's band fit (w_s = surface)
                else
                    d1_ref = 1.1871 * ones(size(t));  c0_ref = 0.0501 * ones(size(t));  ref_lab = 'd_1  cell band fit';   % cell_cperp_delta (2026-09-20); data-fit plant: 1.197 / 0.056
                end
                dm = mean(r.d1_hat, 2); ds = std(r.d1_hat, 0, 2); dp = mean(r.sP_d1, 2);
                cm = mean(r.c0_hat, 2); cs = std(r.c0_hat, 0, 2); cp = mean(r.sP_c0, 2);
                hb = fill([t; flipud(t)], [dm - ds; flipud(dm + ds)], BANDC, 'FaceAlpha', 0.30, 'EdgeColor', 'none', 'DisplayName', '\pm \sigma_{seed}');
                hr = plot(t, d1_ref, '-', 'Color', COL_TRUE, 'LineWidth', 1.6, 'DisplayName', ref_lab);
                hd = plot(t, dm, '-', 'Color', COL_HAT, 'LineWidth', 2.0, 'DisplayName', 'd_{1,hat}');
                plot(t, dm + dp, '--', 'Color', COL_HAT, 'LineWidth', 1.0, 'HandleVisibility', 'off');
                plot(t, dm - dp, '--', 'Color', COL_HAT, 'LineWidth', 1.0, 'HandleVisibility', 'off');
                fill([t; flipud(t)], [cm - cs; flipud(cm + cs)], [0.55 0.85 0.55], 'FaceAlpha', 0.30, 'EdgeColor', 'none', 'HandleVisibility', 'off');
                hc = plot(t, cm, '-', 'Color', [0 0.5 0], 'LineWidth', 2.0, 'DisplayName', 'c_{0,hat}');
                plot(t, c0_ref, '-', 'Color', [0.8 0.3 0], 'LineWidth', 1.2, 'HandleVisibility', 'off');
                plot(t, cm + cp, '--', 'Color', [0 0.5 0], 'LineWidth', 1.0, 'HandleVisibility', 'off');
                plot(t, cm - cp, '--', 'Color', [0 0.5 0], 'LineWidth', 1.0, 'HandleVisibility', 'off');
                yl = [min([cm - cs; -1.3]), max([dm + ds; 1.5])];  local_seg(seg, yl); xlim([0 T_END]); ylim(yl);
                if v == 1; ylabel('c_0,  d_1', 'FontSize', FS, 'FontWeight', 'bold'); end
                xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
                legend([hd hc hr hb], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
                local_ax(FS, AXLW);
                title(NAME{v}, 'FontSize', LFS, 'FontWeight', 'normal');
            end
            for rr = 1:4; linkaxes(ax(rr, :), 'y'); end
            fn = fullfile(od, sprintf('ladder11_%s_%s.png', pl{1}, arm{1}));  exportgraphics(f, fn, 'Resolution', 150); close(f);
            fprintf('figure: %s\n', fn);
        end
    end
end

function local_seg(seg, yl)
    for i = 1:numel(seg); plot([seg(i) seg(i)], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
end

function local_ax(FS, AXLW)
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
end
