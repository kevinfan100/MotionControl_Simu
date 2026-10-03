% PURPOSE (2026-09-18, user: "the cell must have a problem -- check case B and the P arm"): where does the cell PLANT
%   (cell_cperp_fig11.m: Fig. 11 data fit on 1.03-5.5 R + a far-field tail 1 - 1.141/(g + 3.23) beyond) differ from the
%   [1/1] band fit (1.056, 2.197) and the [2/2] band fit (0.782, 1.429, 2.484, 1.542) the T arms call "truth"?
%   Prints the mismatch a_plant - a_law over 1.1-25 R and along the canon deep descent (time of the peak), and draws
%   figures/cell_plant_vs_law.png (left: the three curves; right: the two mismatches vs w/R, band edge marked).
%   EXPIRES: with the [1/1]/[2/2] two-wall line
function probe_cell_plant_vs_law()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(here);
    l11 = @(x) (x + 0.056) ./ (x + 1.197);                                          % w_s = wall surface: x = h_bar
    l22 = @(x) (x.^2 - 0.571 * x + 0.353) ./ (x.^2 + 0.484 * x + 0.058);
    hh = [linspace(1.05, 5.5, 300), linspace(5.51, 25, 300)];  g = hh;
    ap = arrayfun(@(x) 1 / cell_cperp_fig11(x), hh);
    d11 = ap - l11(g);  d22 = ap - l22(g);
    band = hh <= 5.5;
    fprintf('cell plant - [1/1]: band max |d| %.4f at %.2f R | beyond 5.5 R max |d| %.4f at %.2f R | at 1.10 %+.4f, 2.0 %+.4f, 5.5 %+.4f, 8 %+.4f, 12 %+.4f, 22.2 %+.4f\n', ...
            max(abs(d11(band))), hh(find(abs(d11) == max(abs(d11(band))), 1)), max(abs(d11(~band))), hh(find(abs(d11) == max(abs(d11(~band))), 1)), ...
            interp1(hh, d11, 1.10), interp1(hh, d11, 2.0), interp1(hh, d11, 5.5), interp1(hh, d11, 8), interp1(hh, d11, 12), interp1(hh, d11, 22.2));
    fprintf('cell plant - [2/2]: band max |d| %.4f | beyond 5.5 R max |d| %.4f at %.2f R | at 1.10 %+.4f, 2.0 %+.4f, 5.5 %+.4f, 8 %+.4f, 12 %+.4f, 22.2 %+.4f\n', ...
            max(abs(d22(band))), max(abs(d22(~band))), hh(find(abs(d22) == max(abs(d22(~band))), 1)), ...
            interp1(hh, d22, 1.10), interp1(hh, d22, 2.0), interp1(hh, d22, 5.5), interp1(hh, d22, 8), interp1(hh, d22, 12), interp1(hh, d22, 22.2));
    % canon deep descent: 22.2 -> 1.10 R, cosine over 0.5-1.5 s
    t = linspace(0.5, 1.5, 201);  wd = 1.10 + (22.2 - 1.10) * (1 + cos(pi * (t - 0.5))) / 2;
    m11 = interp1(hh, d11, wd);  [~, k] = max(abs(m11));
    fprintf('along the canon descent: [1/1] mismatch peaks %+.4f at t = %.2f s (w_d %.2f R)\n', m11(k), t(k), wd(k));
    FS = 16; LFS = 12;
    f = figure('Position', [40 40 1500 560], 'Color', 'w', 'Visible', 'off');
    tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    nexttile; hold on; box on;
    h1 = plot(hh, ap, '-', 'Color', [0.8 0 0], 'LineWidth', 2.2, 'DisplayName', 'a_{true} / a_{nom},  CELL plant (data 1.03-5.5 R + tail)');
    h2 = plot(hh, l11(g), '-', 'Color', [0 0.2 0.9], 'LineWidth', 2.0, 'DisplayName', '[1/1] band fit (0.056, 1.197), w_s = surface');
    h3 = plot(hh, l22(g), '--', 'Color', [0 0.5 0], 'LineWidth', 2.0, 'DisplayName', '[2/2] band fit (0.353, -0.571, 0.484, 0.058)');
    xline(5.5, ':', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.4, 'HandleVisibility', 'off');
    xlim([1 25]); ylim([0.4 1]); xlabel('w / R', 'FontSize', FS, 'FontWeight', 'bold'); ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
    legend([h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'vertical', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on'); grid off;
    nexttile; hold on; box on;
    yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
    h4 = plot(hh, d11, '-', 'Color', [0 0.2 0.9], 'LineWidth', 2.0, 'DisplayName', 'a_{true} - a_{[1/1]}');
    h5 = plot(hh, d22, '--', 'Color', [0 0.5 0], 'LineWidth', 2.0, 'DisplayName', 'a_{true} - a_{[2/2]}');
    xline(5.5, ':', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.4, 'HandleVisibility', 'off');
    text(5.7, -0.025, 'band edge 5.5 R', 'FontSize', LFS, 'FontWeight', 'bold', 'Color', [0.3 0.3 0.3]);
    xlim([1 25]); ylim([-0.03 0.03]); xlabel('w / R', 'FontSize', FS, 'FontWeight', 'bold'); ylabel('a_{true} - a_{law}  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold');
    legend([h4 h5], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on'); grid off;
    fn = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures', 'cell_plant_vs_law.png');
    exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
end
