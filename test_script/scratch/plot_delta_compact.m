% PURPOSE (2026-09-20, user: "shrink figures 1-2 to two cases"): the offset-law two-wall test reduced to plane T | cell T
%   (seed = plant, prior 0.5, 10 seeds), three rows: a_bar true/hat, e_a = a_true - a_hat (absolute, mean +- sigma_seed),
%   delta_hat (mean +- sigma_seed, +- sqrt(P) dashed, target red). Same data as ladder_delta_<plant>.png.
%   Output reference/eq17_analysis/derivation/figures/ladder_delta_compact.png | EXPIRES: with the delta-law line
function plot_delta_compact(arm, plants)
    if nargin < 1 || isempty(arm); arm = 'T'; end
    if nargin < 2 || isempty(plants); plants = {'plane', 'cell'}; end        % e.g. plot_delta_compact('PI', {'cell'}): cell wall, plane init, one column
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    rd = fullfile(root, 'test_results', 'temp_delta_two_walls');  od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; BANDC = [0.45 0.55 0.95]; FS = 18; LFS = 13; AXLW = 2.0;   % house style (plot_var_ahat_6state.m)
    PL = plants;  nP = numel(PL);
    f = figure('Position', [80 80 900 * nP, 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(3, nP, 'TileSpacing', 'compact', 'Padding', 'compact');  ax = gobjects(3, nP);
    for ip = 1:nP
        r = load(fullfile(rd, sprintf('%s_%s.mat', PL{ip}, arm)));  t = r.t;  seg = r.seg;  ea = r.a_true - r.a_hat;
        ax(1, ip) = nexttile(ip); hold on; box on;
        am = mean(r.a_hat, 2); as = std(r.a_hat, 0, 2);
        fill([t; flipud(t)], [am - as; flipud(am + as)], BANDC, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
        h1 = plot(t, mean(r.a_true, 2), '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'a_{true} / a_{nom}');
        h2 = plot(t, am, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', 'a_{hat} / a_{nom}  (mean \pm \sigma_{seed})');
        for s = seg; xline(s, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
        xlim([0 t(end)]); ylim([0 1]);
        lab_seed = '(= plant)'; if startsWith(arm, 'PI'); lab_seed = '(plane init)'; end
        if startsWith(arm, 'MID'); lab_seed = '(family midpoint init)'; end
        h0 = plot(NaN, NaN, 'w.', 'DisplayName', sprintf('%s wall, \\delta seed %.2f %s', upper(PL{ip}), r.dl_seed, lab_seed));
        if ip == 1; ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([h0 h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
        ax(2, ip) = nexttile(nP + ip); hold on; box on;
        em = mean(ea, 2); es = std(ea, 0, 2);
        fill([t; flipud(t)], [em - es; flipud(em + es)], BANDC, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
        h3 = plot(t, em, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', 'e_a = a_{true} - a_{hat}  (mean \pm \sigma_{seed})');
        yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
        for s = seg; xline(s, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
        xlim([0 t(end)]); ylim([-0.08 0.08]);
        if ip == 1; ylabel('e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend(h3, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
        ax(3, ip) = nexttile(2 * nP + ip); hold on; box on;
        dm = mean(r.dl_hat, 2); ds = std(r.dl_hat, 0, 2); dp = mean(r.sP_dl, 2);
        fill([t; flipud(t)], [dm - ds; flipud(dm + ds)], BANDC, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
        h4 = plot(t, dm, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', '\delta_{hat}  (mean \pm \sigma_{seed})');
        h5 = plot(t, dm + dp, '--', 'Color', COL_HAT, 'LineWidth', 1.0, 'DisplayName', '\pm sqrt(P)');
        plot(t, dm - dp, '--', 'Color', COL_HAT, 'LineWidth', 1.0);
        h6 = yline(r.dl_target, '-', 'Color', COL_TRUE, 'LineWidth', 1.6, 'DisplayName', '\delta  (plant)');
        for s = seg; xline(s, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
        xlim([0 t(end)]); ylim([min(r.dl_target, r.dl_seed) - 0.6, max(r.dl_target, r.dl_seed) + 0.6]);
        if ip == 1; ylabel('\delta  [R]', 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        legend([h4 h5 h6], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    end
    fn = fullfile(od, sprintf('ladder_delta_compact_%s%s.png', arm, char(join(string(cellfun(@(x) ['_' x], PL, 'UniformOutput', false)), ''))));
    exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
end
