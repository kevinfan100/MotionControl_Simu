% PURPOSE (2026-09-18): ladder figures of the [2/2] two-wall test (run_22_two_walls.m), one figure per plant, one column
%   per arm (cell: T | P; plane: T | N), five rows, house style:
%     row 1  w_d/R commanded and the true height (seed mean), segment dividers
%     row 2  a_bar_true (red) and a_bar_hat (blue, mean +- sigma_seed)
%     row 3  e_a = a_bar_true - a_bar_hat, ABSOLUTE
%     row 4  c0_hat (green) and c1_hat (purple), mean +- sigma_seed band, +- sqrt(P) dashed, true values thin lines
%     row 5  d1_hat (blue) and d2_hat (orange), same
%   True values: the plant's own [2/2] constants (plane two-point, cell band fit) -- both plants are in the family.
%   Output reference/eq17_analysis/derivation/figures/ladder22_<plant>.png | EXPIRES: with the [2/2] line
function plot_22_ladder(res_dir)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    if nargin < 1 || isempty(res_dir); res_dir = fullfile(root, 'test_results', 'temp_22_two_walls'); end
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; BANDC = [0.45 0.55 0.95];
    C = struct('c0', [0 0.5 0], 'c1', [0.55 0 0.6], 'd1', [0 0.2 0.9], 'd2', [0.9 0.5 0]);
    FS = 15; LFS = 11; AXLW = 2.0;
    for pl = {'cell', 'plane'}
        files = dir(fullfile(res_dir, sprintf('%s_A_*.mat', pl{1})));
        if isempty(files); continue; end
        RR = {};  ARMS = {};
        for a = {'T', 'P', 'N'}
            fn = fullfile(res_dir, sprintf('%s_A_%s.mat', pl{1}, a{1}));
            if exist(fn, 'file'); RR{end+1} = load(fn); ARMS{end+1} = a{1}; end %#ok<AGROW>
        end
        nc = numel(RR);
        f = figure('Position', [40 40 700 * nc + 100, 1350], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
        tiledlayout(5, nc, 'TileSpacing', 'compact', 'Padding', 'compact');  ax = gobjects(5, nc);
        for c = 1:nc
            r = RR{c};  t = r.t;  seg = r.seg;  T_END = t(end);  ea = r.a_true - r.a_hat;
            ax(1, c) = nexttile(c); hold on; box on;
            h0 = plot(NaN, NaN, 'w.', 'DisplayName', sprintf('%s plant, arm %s, [2/2] version A: seed (c_0, c_1, d_1, d_2) = (%.3f, %.3f, %.3f, %.3f), prior (%.2f, %.2f, %.2f, %.2f)', ...
                      upper(pl{1}), r.arm, r.seed, r.sig));
            h1 = plot(t, r.wd, '-', 'Color', [0.35 0.35 0.35], 'LineWidth', 2.0, 'DisplayName', 'w_d / R');
            h2 = plot(t, mean(r.h_true, 2), '-', 'Color', COL_TRUE, 'LineWidth', 1.4, 'DisplayName', 'w / R (true)');
            local_seg(seg, [0 1.05 * max(r.wd)]);  xlim([0 T_END]); ylim([0 1.05 * max(r.wd)]);
            if c == 1; ylabel('w / R', 'FontSize', FS, 'FontWeight', 'bold'); end
            legend([h0 h1 h2], 'Location', 'northoutside', 'Orientation', 'vertical', 'FontSize', LFS - 1, 'FontWeight', 'bold', 'Box', 'on');
            local_ax(FS, AXLW);
            ax(2, c) = nexttile(nc + c); hold on; box on;
            am = mean(r.a_hat, 2); as = std(r.a_hat, 0, 2);
            hb = fill([t; flipud(t)], [am - as; flipud(am + as)], BANDC, 'FaceAlpha', 0.30, 'EdgeColor', 'none', 'DisplayName', 'a_{hat} \pm \sigma_{seed}');
            ht = plot(t, mean(r.a_true, 2), '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'a_{true} / a_{nom}');
            hh = plot(t, am, '-', 'Color', COL_HAT, 'LineWidth', 2.0, 'DisplayName', 'a_{hat} / a_{nom}');
            local_seg(seg, [0 1]); xlim([0 T_END]); ylim([0 1]);
            if c == 1; ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold'); end
            legend([ht hh hb], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
            local_ax(FS, AXLW);
            ax(3, c) = nexttile(2 * nc + c); hold on; box on;
            em = mean(ea, 2); es = std(ea, 0, 2);
            hb = fill([t; flipud(t)], [em - es; flipud(em + es)], BANDC, 'FaceAlpha', 0.30, 'EdgeColor', 'none', 'DisplayName', '\pm \sigma_{seed}');
            yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
            he = plot(t, em, '-', 'Color', COL_HAT, 'LineWidth', 2.0, 'DisplayName', 'e_a = a_{true} - a_{hat}  (seed mean)');
            local_seg(seg, [-0.06 0.06]); xlim([0 T_END]); ylim([-0.06 0.06]);
            if c == 1; ylabel('e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold'); end
            legend([he hb], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
            local_ax(FS, AXLW);
            for row = 4:5
                if row == 4; names = {'c0', 'c1'}; lab = 'c_0,  c_1'; else; names = {'d1', 'd2'}; lab = 'd_1,  d_2'; end
                ax(row, c) = nexttile((row - 1) * nc + c); hold on; box on;  H = gobjects(1, 0);
                for j = 1:2
                    nm = names{j};  x = r.([nm '_hat']);  sp = r.(['sP_' nm]);  col = C.(nm);  tv = r.true_th(strcmp(nm, {'c0', 'c1', 'd1', 'd2'}));
                    xm = mean(x, 2); xs = std(x, 0, 2); xp = mean(sp, 2);
                    fill([t; flipud(t)], [xm - xs; flipud(xm + xs)], col, 'FaceAlpha', 0.20, 'EdgeColor', 'none', 'HandleVisibility', 'off');
                    H(end+1) = plot(t, xm, '-', 'Color', col, 'LineWidth', 2.0, 'DisplayName', sprintf('%s_{%s,hat}', nm(1), nm(2))); %#ok<AGROW>
                    plot(t, xm + xp, '--', 'Color', col, 'LineWidth', 1.0, 'HandleVisibility', 'off');
                    plot(t, xm - xp, '--', 'Color', col, 'LineWidth', 1.0, 'HandleVisibility', 'off');
                    H(end+1) = plot(t, tv * ones(size(t)), '-', 'Color', col * 0.6 + [0.4 0 0], 'LineWidth', 1.2, 'DisplayName', sprintf('%s_%s true', nm(1), nm(2))); %#ok<AGROW>
                end
                H(end+1) = fill(NaN, NaN, BANDC, 'FaceAlpha', 0.30, 'EdgeColor', 'none', 'DisplayName', '\pm \sigma_{seed} (fill),  \pm sqrt(P) (dashed)'); %#ok<AGROW>
                yl = [-2.5, 2.5];  local_seg(seg, yl); xlim([0 T_END]); ylim(yl);
                if c == 1; ylabel(lab, 'FontSize', FS, 'FontWeight', 'bold'); end
                if row == 5; xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
                legend(H, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS - 1, 'FontWeight', 'bold', 'Box', 'on');
                local_ax(FS, AXLW);
            end
        end
        for rr = 1:5; linkaxes(ax(rr, :), 'y'); end
        fn = fullfile(od, sprintf('ladder22_%s.png', pl{1}));  exportgraphics(f, fn, 'Resolution', 150); close(f);
        fprintf('figure: %s\n', fn);
    end
end

function local_seg(seg, yl)
    for i = 1:numel(seg); plot([seg(i) seg(i)], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
end

function local_ax(FS, AXLW)
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
end
