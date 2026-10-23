% PURPOSE (2026-09-20, user: "the positioning test at the three heights, init at plant, cell wall"): the offset law held at
%   22.2 / 2.0 / 1.10 R above the reference surface for 3 s (run_delta_fixed_speed.m, sign T, heights [22.2 2.0 1.10]),
%   one figure per plant, columns = heights, rows: a_bar true/hat (mean +- sigma_seed), e_a = a_true - a_hat (absolute),
%   delta_hat (mean +- sigma_seed, +- sqrt(P) dashed, plant delta red). What to expect from the paper gates: no information
%   at 22.2 R (a' 0.002, CRLB 40 R: delta_hat must NOT move and P must NOT shrink), the level alone pins delta at 2.0 and
%   1.10 R with speed ~ a'^2 (plane 1.10 R: a' 0.77; cell 1.10 R: effective 2.15 R, a' 0.21).
%   Output reference/eq17_analysis/derivation/figures/fixed_delta_<plant>.png | EXPIRES: with the delta-law line
function plot_delta_fixed3(sign, res_dir)
    if nargin < 1 || isempty(sign); sign = 'T'; end                          % 'T' seed = plant; 'PI' plane init (delta_hat[0] = 0)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    if nargin < 2 || isempty(res_dir); res_dir = fullfile(root, 'test_results', 'temp_delta_two_walls'); end
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; BANDC = [0.45 0.55 0.95]; FS = 18; LFS = 13; AXLW = 2.0;   % house style (plot_var_ahat_6state.m)
    H = {'22p20', '2p00', '1p10'};  HL = {'22.2 R', '2.0 R', '1.10 R'};
    for pl = {'plane', 'cell'}
        files = cellfun(@(h) fullfile(res_dir, sprintf('fixed_%s_%s_h%s.mat', pl{1}, sign, h)), H, 'UniformOutput', false);
        keep = cellfun(@(f) exist(f, 'file') == 2, files);
        if ~any(keep); continue; end
        R = cellfun(@load, files(keep), 'UniformOutput', false);  names = HL(keep);  nH = numel(R);
        f = figure('Position', [40 40 480 * nH + 100, 950], 'Color', 'w', 'Visible', 'off');
        tiledlayout(3, nH, 'TileSpacing', 'compact', 'Padding', 'compact');  ax = gobjects(3, nH);
        for v = 1:nH
            r = R{v};  t = r.t;  ea = r.a_true - r.a_hat;  T_END = t(end);
            ax(1, v) = nexttile(v); hold on; box on;
            am = mean(r.a_hat, 2); as = std(r.a_hat, 0, 2);  at = mean(r.a_true, 2);
            fill([t; flipud(t)], [am - as; flipud(am + as)], BANDC, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
            h1 = plot(t, at, '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'a_{true} / a_{nom}');
            h2 = plot(t, am, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', 'a_{hat} / a_{nom}');
            yl = [max(0, at(end) - 0.15), min(1, at(end) + 0.15)];  if startsWith(sign, 'PI'); yl = [0 1]; end;  xlim([0 T_END]); ylim(yl);
            lab_seed = '(= plant)'; if startsWith(sign, 'PI'); lab_seed = '(plane init)'; end
            h0 = plot(NaN, NaN, 'w.', 'DisplayName', sprintf('%s, hold %s, seed %.2f', upper(pl{1}), names{v}, r.dl_seed));
            if v == 1; ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold'); end
            legend([h0 h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
            set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
            ax(2, v) = nexttile(nH + v); hold on; box on;
            em = mean(ea, 2); es = std(ea, 0, 2);
            fill([t; flipud(t)], [em - es; flipud(em + es)], BANDC, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
            h3 = plot(t, em, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', 'e_a = a_{true} - a_{hat}  (mean \pm \sigma_{seed})');
            yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
            yl2 = [-0.06 0.06];  if startsWith(sign, 'PI'); yl2 = [-0.05 0.25]; end;  xlim([0 T_END]); ylim(yl2);
            if v == 1; ylabel('e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold'); end
            legend(h3, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
            set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
            ax(3, v) = nexttile(2 * nH + v); hold on; box on;
            dm = mean(r.dl_hat, 2); ds = std(r.dl_hat, 0, 2); dp = mean(r.sP_dl, 2);
            fill([t; flipud(t)], [dm - ds; flipud(dm + ds)], BANDC, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
            h4 = plot(t, dm, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', '\delta_{hat}  (mean \pm \sigma_{seed})');
            h5 = plot(t, dm + dp, '--', 'Color', COL_HAT, 'LineWidth', 1.0, 'DisplayName', '\pm sqrt(P)');
            plot(t, dm - dp, '--', 'Color', COL_HAT, 'LineWidth', 1.0);
            h6 = yline(r.dl_target, '-', 'Color', COL_TRUE, 'LineWidth', 1.6, 'DisplayName', '\delta  (plant)');
            xlim([0 T_END]); ylim([min(r.dl_target, r.dl_seed) - 0.6, max(r.dl_target, r.dl_seed) + 0.6]);
            if v == 1; ylabel('\delta  [R]', 'FontSize', FS, 'FontWeight', 'bold'); end
            xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
            legend([h4 h5 h6], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
            set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
        end
        fn = fullfile(od, sprintf('fixed_delta_%s_%s.png', pl{1}, sign));
        exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
    end
end
