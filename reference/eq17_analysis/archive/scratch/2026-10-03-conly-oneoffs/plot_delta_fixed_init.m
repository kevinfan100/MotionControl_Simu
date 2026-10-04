% PURPOSE (2026-09-20 night, C68): the fixed-height 1.10 R cell case with two initialisations side by side --
%   plane init (delta_hat[0] = 0, gain ratio at start 0.17) | wall-family midpoint init (delta_hat[0] = 0.525, ratio 0.6);
%   both prior sqrt(P) = 0.5, 10 seeds. Rows: a_bar true/hat, e_a, delta_hat +- sigma_seed with +- sqrt(P) and the plant
%   delta. House style. Files fixed_cell_PI_h1p10.mat, fixed_cell_MID_h1p10.mat (run_delta_fixed_speed.m).
%   Output reference/eq17_analysis/derivation/figures/fixed_delta_init_cell.png | EXPIRES: with the delta-law line
function plot_delta_fixed_init()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    rd = fullfile(root, 'test_results', 'temp_delta_two_walls');  od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; BANDC = [0.45 0.55 0.95]; FS = 18; LFS = 13; AXLW = 2.0;
    SG = {'PI', 'MID'};  NM = {'plane init  (\delta seed 0)', 'family-midpoint init  (\delta seed 0.525)'};
    f = figure('Position', [80 80 1500 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for c = 1:2
        r = load(fullfile(rd, sprintf('fixed_cell_%s_h1p10.mat', SG{c})));  t = r.t;  ea = r.a_true - r.a_hat;
        nexttile(c); hold on; box on;
        am = mean(r.a_hat, 2); as = std(r.a_hat, 0, 2);
        fill([t; flipud(t)], [am - as; flipud(am + as)], BANDC, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
        h1 = plot(t, mean(r.a_true, 2), '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'a_{true} / a_{nom}');
        h2 = plot(t, am, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', 'a_{hat} / a_{nom}');
        h0 = plot(NaN, NaN, 'w.', 'DisplayName', sprintf('CELL, hold 1.10 R, %s', NM{c}));
        xlim([0 t(end)]); ylim([0 1]);
        if c == 1; ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([h0 h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
        nexttile(2 + c); hold on; box on;
        em = mean(ea, 2); es = std(ea, 0, 2);
        fill([t; flipud(t)], [em - es; flipud(em + es)], BANDC, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
        h3 = plot(t, em, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', 'e_a = a_{true} - a_{hat}  (mean \pm \sigma_{seed})');
        yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
        xlim([0 t(end)]); ylim([-0.05 0.25]);
        if c == 1; ylabel('e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend(h3, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
        nexttile(4 + c); hold on; box on;
        dm = mean(r.dl_hat, 2); ds = std(r.dl_hat, 0, 2); dp = mean(r.sP_dl, 2);
        fill([t; flipud(t)], [dm - ds; flipud(dm + ds)], BANDC, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
        h4 = plot(t, dm, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', '\delta_{hat}  (mean \pm \sigma_{seed})');
        h5 = plot(t, dm + dp, '--', 'Color', COL_HAT, 'LineWidth', 1.0, 'DisplayName', '\pm sqrt(P)');
        plot(t, dm - dp, '--', 'Color', COL_HAT, 'LineWidth', 1.0);
        h6 = yline(r.dl_target, '-', 'Color', COL_TRUE, 'LineWidth', 1.6, 'DisplayName', '\delta  (plant)');
        xlim([0 t(end)]); ylim([-0.6 1.6]);
        if c == 1; ylabel('\delta  [R]', 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        legend([h4 h5 h6], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    end
    fn = fullfile(od, 'fixed_delta_init_cell.png');
    exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
end
