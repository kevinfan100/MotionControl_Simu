% PURPOSE (2026-09-20, explanation figure): what "x -> x + delta" means, walked with the cell data.
%   Left : a_bar against h = w - w_s (centre height above the VISIBLE surface, in R) for the plane polynomial, the cell
%          data (Meng Fig 11) and the test cell plant (= the plane polynomial slid left by delta = 1.05 R, cell_cperp_delta). Horizontal
%          arrows at a_bar = 0.55 and 0.8 show the slide: the cell reaches the same gain delta closer to its visible surface.
%   Right: the same curves against h + delta_wall (centre height above the EFFECTIVE wall: delta_wall = 0 plane,
%          1.05 cell). They collapse onto one curve: the cell IS the plane seen from a wall 1.05 R below the visible surface.
%   Console: for a_bar = 0.55 / 0.8, the x where the plane and the cell reach it, and the difference (= delta).
%   Output reference/eq17_analysis/derivation/figures/cell_shift_walk.png.  EXPIRES: explanation aid for the delta-law (O36).
function plot_cell_shift_walk()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
    D = @(u) 1 - 9/8*u + 1/2*u.^3 - 57/100*u.^4 + 1/5*u.^5 + 7/200*u.^11 - 1/25*u.^12;
    a_plane = @(x) D(1 ./ x);  DEL = 1.05;
    T = readtable(fullfile(root, 'reference', 'eq17_analysis', 'data', 'meng_11072733_fig11_three_surfaces.csv'));
    m = strcmp(T.surface, 'cell') & T.hd_R <= 4.5;  xd = T.hd_R(m);  yd = T.lambda_norm(m);
    a_cell = @(x) 1 ./ cell_cperp_delta(x);   % 2026-09-20: the test cell plant (plane slid 1.05 R, C64); the data-fit plant had a hook below 1.04 R
    xs = linspace(1.0, 8, 800)';
    fprintf('%8s | %14s %14s %10s\n', 'a_bar', 'x plane', 'x cell', 'difference');
    for ab = [0.55 0.8]
        xp = fzero(@(x) a_plane(x) - ab, [1 30]);  xc = fzero(@(x) a_cell(x) - ab, [1.03 30]);
        fprintf('%8.2f | %14.2f %14.2f %10.2f\n', ab, xp, xc, xp - xc);
    end
    FS = 15; LFS = 11; CT = [0.8 0 0]; CC = [0 0.2 0.9]; CD = [0.55 0.75 1]; CS = [0 0.55 0.2]; CG = [0.4 0.4 0.4];
    f = figure('Position', [40 40 2100 620], 'Color', 'w', 'Visible', 'off');
    tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    nexttile; hold on; box on;
    h1 = plot(xs, a_plane(xs), '-', 'Color', CT, 'LineWidth', 2.4, 'DisplayName', 'plane polynomial  a(h)');
    h2 = plot(xd, yd, 'o', 'Color', CD, 'MarkerFaceColor', CD, 'MarkerSize', 5, 'DisplayName', 'cell data (Meng Fig 11)');
    h3 = plot(xs, a_cell(xs), '-', 'Color', CC, 'LineWidth', 2.4, 'DisplayName', sprintf('cell plant = plane polynomial slid left:  a(h + %.2f)   (cell\\_cperp\\_delta)', DEL));
    h4 = gobjects(0);
    for ab = [0.55 0.8]
        xp = fzero(@(x) a_plane(x) - ab, [1 30]);  xc = fzero(@(x) a_cell(x) - ab, [1.03 30]);
        plot([xc xp], [ab ab], '-', 'Color', CG, 'LineWidth', 1.4, 'HandleVisibility', 'off');
        plot(xc, ab, '<', 'Color', CG, 'MarkerFaceColor', CG, 'MarkerSize', 7, 'HandleVisibility', 'off');
        text((xc + xp) / 2, ab + 0.035, sprintf('\\delta = %.2f', xp - xc), 'FontSize', LFS, 'FontWeight', 'bold', 'Color', CG, 'HorizontalAlignment', 'center');
    end
    xline(1, '-', 'Color', CG, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    text(1.03, 0.02, 'visible surface / wall', 'FontSize', LFS - 1, 'FontWeight', 'bold', 'Color', CG);
    xlim([1 8]); ylim([0 1]);
    xlabel('h = w - w_s   (centre height above the visible surface, R)', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
    legend([h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'vertical', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on'); grid off;
    nexttile; hold on; box on;
    g1 = plot(xs, a_plane(xs), '-', 'Color', CT, 'LineWidth', 2.4, 'DisplayName', 'plane:  h + 0');
    g2 = plot(xd + DEL, yd, 'o', 'Color', CD, 'MarkerFaceColor', CD, 'MarkerSize', 5, 'DisplayName', sprintf('cell data:  h + %.2f', DEL));
    g3 = plot(xs + DEL, a_cell(xs), '--', 'Color', CC, 'LineWidth', 2.0, 'DisplayName', sprintf('cell plant:  h + %.2f   (on the plane by construction)', DEL));
    xline(1, '-', 'Color', CG, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    text(1.03, 0.02, 'effective wall', 'FontSize', LFS - 1, 'FontWeight', 'bold', 'Color', CG);
    xlim([1 8]); ylim([0 1]);
    xlabel('h + \delta   (centre height above the effective wall, R)', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
    legend([g1 g2 g3], 'Location', 'northoutside', 'Orientation', 'vertical', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on'); grid off;
    nexttile; hold on; box on;
    r_plant = a_cell(xs) - a_plane(xs + DEL);            % middle panel: blue dashed minus red (same h + delta)
    r_data  = yd - a_plane(xd + DEL);                    % middle panel: data minus red
    d_type  = a_cell(xs) - a_plane(xs);                  % left panel: blue minus red at the same visible height h (positive: the cell gain is higher)
    k1 = plot(xd + DEL, r_data, 'o', 'Color', CD, 'MarkerFaceColor', CD, 'MarkerSize', 4, 'DisplayName', sprintf('cell data(h + %.2f) - plane:  rms %.4f, max %.3f', DEL, rms(r_data), max(abs(r_data))));
    k2 = plot(xs + DEL, r_plant, '--', 'Color', CC, 'LineWidth', 2.2, 'DisplayName', sprintf('cell plant(h + %.2f) - plane:  0 by construction', DEL));
    k3 = plot(xs + DEL, d_type, '-', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.8, 'DisplayName', 'cell plant(h) - plane(h), same visible h (wall-type signal; plotted at h + \delta)');
    yline(0, '-', 'Color', CG, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    xlim([1 8]); ylim([-0.05 0.55]);
    xlabel('h + \delta   (R)', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel('difference  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold');
    legend([k1 k2 k3], 'Location', 'northoutside', 'Orientation', 'vertical', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on'); grid off;
    fprintf('right panel: data - plane rms %.4f max %.4f | plant - plane max %.1e | cell - plane at h = 1.1 / 2 / 5.5: %.3f / %.3f / %.3f\n', ...
            rms(r_data), max(abs(r_data)), max(abs(r_plant)), a_cell(1.1) - a_plane(1.1), a_cell(2) - a_plane(2), a_cell(5.5) - a_plane(5.5));
    fn = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures', 'cell_shift_walk.png');
    exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
end
