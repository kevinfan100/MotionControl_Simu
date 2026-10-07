% PURPOSE (2026-10-05, user: "match the style of the figure I gave you" = fig_noap_rtri_td0p5_f5_s1, the first-order seed-1 figure;
%   then "update slides 6-8 with the c estimate"): c-only in that layout -- columns plane | cell, rows a_bar (a_true red, a_hat blue,
%   a_m light blue) | e_a | delta_x_m [um] (| c: c_eff(h_d) red, c_hat blue), legend title '<wall> wall, seed 1', the arm label inside
%   the a_hat entry. One seed (column 1 of the file).
%   spec.files = {plane_file, cell_file} (test_results/temp_plaw_walls), spec.labs = {plane_label, cell_label}, spec.name (output tag),
%   spec.crow (true = c row), spec.ylim2 / ylim3 / ylim4 (shared limits per row), spec.wall_note = {'', ''} appended to the title.
%   Output reference/eq17_analysis/derivation/figures/fig_conly_final_<name>_s1.png.   STATUS: SCRATCH
function plot_conly_rtri_final(spec)
    if ~isfield(spec, 'crow'); spec.crow = false; end
    if ~isfield(spec, 'ylim2'); spec.ylim2 = [-0.2 0.2]; end
    if ~isfield(spec, 'ylim3'); spec.ylim3 = [-0.2 0.1]; end
    if ~isfield(spec, 'ylim4'); spec.ylim4 = [-0.5 1]; end
    if ~isfield(spec, 'wall_note'); spec.wall_note = {'', ''}; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_plaw_walls');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    WALLS = {'plane', 'cell'};  nR = 3 + spec.crow;
    AB = char(257);  HAT = char(770);
    COL_TRUE = [0.8 0 0];  C_EST = [0 0.2 0.9];  C_MEAS = [0.55 0.75 1.0];  FS = 18;  LFS = 12;  AXLW = 2.0;
    fig = figure('Position', [40 40 1700 1150 + 380 * spec.crow], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(fig, nR, numel(WALLS), 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = gobjects(nR, numel(WALLS));
    for iw = 1:numel(WALLS)
        r = load(fullfile(id, spec.files{iw}));  t = r.t;
        for q = 1:nR; ax(q, iw) = nexttile(tl, (q - 1) * numel(WALLS) + iw); hold(ax(q, iw), 'on'); end
        hm = plot(ax(1, iw), t, r.a_m(:, 1), '-', 'Color', C_MEAS, 'LineWidth', 0.6, 'DisplayName', [AB '_m']);
        ht = plot(ax(1, iw), t, r.a_true(:, 1), '-', 'Color', COL_TRUE, 'LineWidth', 2.4, 'DisplayName', [AB '_{true}']);
        hh = plot(ax(1, iw), t, r.a_hat(:, 1), '-', 'Color', C_EST, 'LineWidth', 1.8, 'DisplayName', [AB HAT ', ' spec.labs{iw}]);
        ylim(ax(1, iw), [-0.2 1.6]);
        lg = legend(ax(1, iw), [ht hh hm], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', ...
                    'Box', 'on', 'AutoUpdate', 'off');
        lg.Title.String = [sprintf('%s wall, seed 1', WALLS{iw}) spec.wall_note{iw}];
        e = r.a_true(:, 1) - r.a_hat(:, 1);
        plot(ax(2, iw), t, e, '-', 'Color', C_EST, 'LineWidth', 1.8);  yline(ax(2, iw), 0, '-', 'Color', [0.6 0.6 0.6]);
        plot(ax(3, iw), t, r.dxm(:, 1), '-', 'Color', C_MEAS, 'LineWidth', 0.8);  yline(ax(3, iw), 0, '-', 'Color', [0.6 0.6 0.6]);
        if spec.crow
            hc = plot(ax(4, iw), t, r.c_eff_d, '-', 'Color', COL_TRUE, 'LineWidth', 1.0, 'DisplayName', 'c_{eff}(h_d) = 1 - h_d^2 a''_{true}');
            hk = plot(ax(4, iw), t, r.c_hat(:, 1), '-', 'Color', C_EST, 'LineWidth', 1.8, 'DisplayName', ['c' HAT]);
            legend(ax(4, iw), [hc hk], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'AutoUpdate', 'off');
        end
        xlabel(ax(nR, iw), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        Td = r.seg(2);  ko = t >= Td;
        fprintf('%-5s %s | triangle mean %+.4f, mean|e_a| %.4f, worst %+.3f | c_hat end %.3f | dx_m rms triangle %.4f um\n', WALLS{iw}, spec.files{iw}, ...
            mean(e(ko)), mean(abs(e(ko))), e(find(abs(e) == max(abs(e(ko))) & ko, 1)), r.c_hat(end, 1), rms(r.dxm(ko, 1)));
    end
    ylabel(ax(1, 1), AB, 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax(2, 1), ['e_{' AB '}'], 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax(3, 1), '\delta x_m  [\mum]', 'FontSize', FS, 'FontWeight', 'bold');
    if spec.crow; ylabel(ax(4, 1), 'c', 'FontSize', FS, 'FontWeight', 'bold'); end
    for q = 2:nR; linkaxes(ax(q, :), 'y'); end
    ylim(ax(2, 1), spec.ylim2);  ylim(ax(3, 1), spec.ylim3);  if spec.crow; ylim(ax(4, 1), spec.ylim4); end
    for q = 1:nR
        for iw = 1:numel(WALLS)
            xlim(ax(q, iw), [0 t(end)]);  set(ax(q, iw), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');  grid(ax(q, iw), 'off');
        end
    end
    out = fullfile(fd, sprintf('fig_conly_final_%s_s1.png', spec.name));
    exportgraphics(fig, out, 'Resolution', 150);  close(fig);  fprintf('figure: %s\n', out);
end
