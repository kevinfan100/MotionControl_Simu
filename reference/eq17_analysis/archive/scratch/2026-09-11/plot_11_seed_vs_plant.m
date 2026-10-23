% PURPOSE (2026-09-18): the wrong-wall seed of the fixed-height test against the plant it is used on -- the [1/1] law at
%   the seed (c0, d1) versus the plant's a_true/a_nom over 1 <= w/R <= 25, the three hold heights marked. Prints the
%   start-of-run gain error e_a[0] = a_true - a_seed at 22.2 / 2.0 / 1.10 R for both plants.
%   Output reference/eq17_analysis/derivation/figures/fixed11_seed_vs_plant.png | EXPIRES: with the [1/1] two-wall line
function plot_11_seed_vs_plant()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(here);
    FIT = struct('plane', [-0.984, 0.161], 'cell', [0.0501, 1.1871]);    % w_s = wall surface: a = (h + c0)/(h + d1); cell = cell_cperp_delta (2026-09-20)
    law = @(h, cd) (h + cd(1)) ./ (h + cd(2));
    hs = [22.2, 2.0, 1.10];
    fprintf('%-6s %-6s | %8s | %-6s %8s | %8s %7s\n', 'plant', 'h [R]', 'a_true', 'seed', 'a_seed', 'e_a[0]', 'ratio');
    for pl = {'cell', 'plane'}
        if strcmp(pl{1}, 'cell'); seed = FIT.plane; sname = 'plane'; else; seed = FIT.cell; sname = 'cell'; end
        for h = hs
            at = local_true(pl{1}, h);  as = law(h, seed);
            fprintf('%-6s %-6.2f | %8.4f | %-6s %8.4f | %+8.4f %7.2f\n', pl{1}, h, at, sname, as, at - as, at / as);
        end
    end
    hh = [linspace(1.05, 3, 200), linspace(3.01, 25, 300)];  FS = 16;  LFS = 12;
    at_cell = arrayfun(@(x) local_true('cell', x), hh);  at_plane = arrayfun(@(x) local_true('plane', x), hh);
    f = figure('Position', [40 40 1500 560], 'Color', 'w', 'Visible', 'off');
    tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for k = 1:2
        nexttile; hold on; box on;
        if k == 1
            at = at_cell;  seed = FIT.plane;
            n_true = 'a_{true} / a_{nom},  CELL plant';  n_seed = 'a(w - w_s; c_0, d_1) at the plane seed (-0.984, 0.161)';
        else
            at = at_plane; seed = FIT.cell;
            n_true = 'a_{true} / a_{nom},  PLANE plant'; n_seed = 'a(w - w_s; c_0, d_1) at the cell seed (0.050, 1.187)';
        end
        as = law(hh, seed);
        h1 = plot(hh, at, '-', 'Color', [0.8 0 0], 'LineWidth', 2.2, 'DisplayName', n_true);
        h2 = plot(hh, as, '-', 'Color', [0 0.2 0.9], 'LineWidth', 2.2, 'DisplayName', n_seed);
        y_lab = [0.06, 0.14, 0.06];                      % 2.00 R label raised so it does not sit on the 1.10 R one
        for i = 1:numel(hs)
            xline(hs(i), ':', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.4, 'HandleVisibility', 'off');
            text(hs(i) + 0.25, y_lab(i), sprintf('%.2f R', hs(i)), 'FontSize', LFS, 'FontWeight', 'bold', 'Color', [0.3 0.3 0.3]);
        end
        xlim([1 25]); ylim([0 1]);
        xlabel('w / R', 'FontSize', FS, 'FontWeight', 'bold');
        if k == 1; ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([h1 h2], 'Location', 'northoutside', 'Orientation', 'vertical', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on'); grid off;
    end
    fn = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures', 'fixed11_seed_vs_plant.png');
    exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
end

function at = local_true(pl, h)
    if strcmp(pl, 'cell'); at = 1 / cell_cperp_delta(h); else; [~, c] = calc_correction_functions(h); at = 1 / c; end
end
