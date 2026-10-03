% PURPOSE (2026-09-20): ladder figures of the offset-law two-wall test (run_delta_two_walls.m), one figure per plant, the
%   arms side by side (T | T+ | T- | W), shared y per row, house style (True red, Estimate blue, +-sigma_seed band):
%     row 1  w_d/R commanded and the true height (seed mean), segment dividers
%     row 2  a_bar_true (red, seed mean) and a_bar_hat (blue, mean +- sigma_seed)
%     row 3  e_a = a_bar_true - a_bar_hat, ABSOLUTE: seed mean +- sigma_seed band
%     row 4  delta_hat: mean +- sigma_seed band, seed mean +- sqrt(P) (dashed), target (red): plant delta (+1 R in arm W)
%   Output reference/eq17_analysis/derivation/figures/ladder_delta_<plant>.png  | EXPIRES: with the delta-law line
function plot_delta_ladder(res_dir)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    if nargin < 1 || isempty(res_dir); res_dir = fullfile(root, 'test_results', 'temp_delta_two_walls'); end
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; BANDC = [0.45 0.55 0.95]; FS = 18; LFS = 13; AXLW = 2.0;   % house style (plot_var_ahat_6state.m)
    ARMS = {'T', 'Tp', 'Tm', 'W'};  ARMN = {'T: seed = plant', 'T+: seed + 0.5 R', 'T-: seed - 0.5 R', 'W: w_s mis-set + 1 R'};
    for pl = {'plane', 'cell'}
        files = cellfun(@(a) fullfile(res_dir, sprintf('%s_%s.mat', pl{1}, a)), ARMS, 'UniformOutput', false);
        keep = cellfun(@(f) exist(f, 'file') == 2, files);
        if ~any(keep); continue; end
        R = cellfun(@load, files(keep), 'UniformOutput', false);  names = ARMN(keep);  nA = numel(R);
        t = R{1}.t;  seg = R{1}.seg;  T_END = t(end);
        f = figure('Position', [80 80 560 * nA + 100, 1250], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
        tiledlayout(4, nA, 'TileSpacing', 'compact', 'Padding', 'compact');
        ax = gobjects(4, nA);
        for v = 1:nA
            r = R{v};  ea = r.a_true - r.a_hat;
            ax(1, v) = nexttile(v); hold on; box on;
            plot(t, r.wd, '-', 'Color', [0.35 0.35 0.35], 'LineWidth', 2.0);
            plot(t, mean(r.h_true, 2), '-', 'Color', COL_TRUE, 'LineWidth', 1.4);
            local_seg(seg, [0 1.05 * max(r.wd)]);  xlim([0 T_END]); ylim([0 1.05 * max(r.wd)]);
            if v == 1; ylabel('w / R', 'FontSize', FS, 'FontWeight', 'bold'); end
            h0 = plot(NaN, NaN, 'w.', 'DisplayName', sprintf('%s | %s', upper(pl{1}), names{v}));
            legend(h0, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS - 1, 'FontWeight', 'bold', 'Box', 'on');
            local_ax(FS, AXLW);
            ax(2, v) = nexttile(nA + v); hold on; box on;
            am = mean(r.a_hat, 2); as = std(r.a_hat, 0, 2);
            fill([t; flipud(t)], [am - as; flipud(am + as)], BANDC, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
            h1 = plot(t, mean(r.a_true, 2), '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'a_{true} / a_{nom}');
            h2 = plot(t, am, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', 'a_{hat} / a_{nom}  (mean \pm \sigma_{seed})');
            local_seg(seg, [0 1]);  xlim([0 T_END]); ylim([0 1]);
            if v == 1; ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold'); end
            legend([h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
            local_ax(FS, AXLW);
            ax(3, v) = nexttile(2 * nA + v); hold on; box on;
            em = mean(ea, 2); es = std(ea, 0, 2);
            fill([t; flipud(t)], [em - es; flipud(em + es)], BANDC, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
            h3 = plot(t, em, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', 'e_a = a_{true} - a_{hat}  (mean \pm \sigma_{seed})');
            yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0);
            local_seg(seg, [-0.15 0.15]);  xlim([0 T_END]); ylim([-0.15 0.15]);
            if v == 1; ylabel('e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold'); end
            legend(h3, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
            local_ax(FS, AXLW);
            ax(4, v) = nexttile(3 * nA + v); hold on; box on;
            dm = mean(r.dl_hat, 2); ds = std(r.dl_hat, 0, 2); dp = mean(r.sP_dl, 2);
            fill([t; flipud(t)], [dm - ds; flipud(dm + ds)], BANDC, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
            h4 = plot(t, dm, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', '\delta_{hat}  (mean \pm \sigma_{seed})');
            h5 = plot(t, dm + dp, '--', 'Color', COL_HAT, 'LineWidth', 1.0, 'DisplayName', '\pm sqrt(P)');
            plot(t, dm - dp, '--', 'Color', COL_HAT, 'LineWidth', 1.0);
            h6 = yline(r.dl_target, '-', 'Color', COL_TRUE, 'LineWidth', 1.6, 'DisplayName', '\delta target');
            yl = [min(-0.6, r.dl_target - 1.2), max(1.6, r.dl_target + 1.2)];
            local_seg(seg, yl);  xlim([0 T_END]); ylim(yl);
            if v == 1; ylabel('\delta  [R]', 'FontSize', FS, 'FontWeight', 'bold'); end
            xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
            legend([h4 h5 h6], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
            local_ax(FS, AXLW);
        end
        for rr = 1:4; linkaxes(ax(rr, :), 'y'); end
        fn = fullfile(od, sprintf('ladder_delta_%s.png', pl{1}));
        exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
    end
end

function local_seg(seg, yl)
    for s = seg; plot([s s], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
end

function local_ax(FS, AXLW)
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
end
