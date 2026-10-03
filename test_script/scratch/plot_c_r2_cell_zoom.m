% PURPOSE (2026-09-29, user: "let me see the R2 effect: two scalings plus the original"): cell wall, c-only law (f1p2), plane
%   init, prior 0.10, lambda = 1, T_d = 2 s, 10 seeds; arms R2 x 1 (f1p2_cell_PI_sig010_ramp2.mat), R2 / 3 (_r2d3), R2 / 10 (_r2d10)
%   overlaid on the plot_plaw_walls layout (time axis; rows a_true/a_hat, e_a, c_hat with +- sigma_seed band, +- sqrt(P) dashed,
%   band-fit c), zoomed to 2.0 -- 3.0 s (descent ends at 2.5 s, hold after). Console: end values, sqrt(P), honesty (sd/sqrtP), NIS2.
%   House style. Output reference/eq17_analysis/derivation/figures/c_r2_cell_zoom.png   EXPIRES: with the c-only line
function plot_c_r2_cell_zoom(TW, ftag, ARM, LAB)
    if nargin < 1 || isempty(TW); TW = [2.0 3.0]; end                       % time window; [0 3] = the whole run (user 09-29: "not zoomed")
    if nargin < 2; ftag = ''; end                                          % appended to the file name
    if nargin < 3 || isempty(ARM); ARM = {'', '_r2d3', '_r2d10'};  LAB = {'R_2', 'R_2 / 3', 'R_2 / 10'}; end   % arm suffixes (between _sig010 and _ramp2) and legend labels
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    rd = fullfile(root, 'test_results', 'temp_plaw_walls');  od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    keep = cellfun(@(s) exist(fullfile(rd, sprintf('f1p2_cell_PI_sig010%s_ramp2.mat', s)), 'file') == 2, ARM);
    ARM = ARM(keep);  LAB = LAB(keep);  nA = numel(ARM);
    COLS = [0 0.2 0.9; 0.85 0.45 0; 0 0.6 0.2; 0.6 0 0.8; 0.1 0.6 0.7];  BANDS = [0.45 0.55 0.95; 0.95 0.7 0.4; 0.5 0.85 0.55; 0.8 0.6 0.95; 0.6 0.85 0.9];  COL_TRUE = [0.8 0 0];
    COLS = COLS(1:nA, :);  BANDS = BANDS(1:nA, :);
    FS = 18; LFS = 13; AXLW = 2.0; CG = [0.5 0.5 0.5];
    f = figure('Position', [60 60 760 1250], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = gobjects(3, 1);  for rr = 1:3; ax(rr) = nexttile(rr); hold(ax(rr), 'on'); box(ax(rr), 'on'); end
    H = struct('a', gobjects(1, nA), 'e', gobjects(1, nA), 'c', gobjects(1, nA), 'p', gobjects(1, nA));  htrue = gobjects(0);  cfit = NaN;
    fprintf('\n%-8s | %8s %8s | %8s %8s %8s | %6s %6s\n', 'arm', 'e_a_wall', 'e_a_hold', 'c_end', 'sd_seed', 'sqrtP', 'honest', 'NIS2');
    for ia = 1:nA
        r = load(fullfile(rd, sprintf('f1p2_cell_PI_sig010%s_ramp2.mat', ARM{ia})));
        t = r.t;  desc = t >= r.seg(1) & t < r.seg(2);  hold_ = t >= r.seg(3);  ie = find(desc, 1, 'last');  w = t >= TW(1) - 0.05;  cfit = r.c_fit;
        am = mean(r.a_hat, 2);  as = std(r.a_hat, 0, 2);  at = mean(r.a_true, 2);
        ea = r.a_true - r.a_hat;  em = mean(ea, 2);  es = std(ea, 0, 2);
        cm = mean(r.c_hat, 2);  cs = std(r.c_hat, 0, 2);  cp = mean(r.sP_c, 2);
        ok2 = isfinite(r.innov2) & isfinite(r.S2) & r.S2 > 0;  nis2 = r.innov2.^2 ./ r.S2;  nis2(~ok2) = NaN;
        fprintf('%-8s | %+8.3f %+8.3f | %8.3f %8.3f %8.4f | %6.2f %6.2f\n', LAB{ia}, em(ie), mean(em(hold_)), cm(ie), cs(ie), cp(ie), cs(ie) / cp(ie), mean(nis2(desc, :), 'all', 'omitnan'));
        lab = LAB{ia};
        a = ax(1);
        fill(a, [t(w); flipud(t(w))], [am(w) - as(w); flipud(am(w) + as(w))], BANDS(ia, :), 'FaceAlpha', 0.25, 'EdgeColor', 'none');
        if ia == 1; htrue = plot(a, t(w), at(w), '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'a_{true} / a_{nom}'); end
        H.a(ia) = plot(a, t(w), am(w), '-', 'Color', COLS(ia, :), 'LineWidth', 1.8, 'DisplayName', sprintf('a_{hat} / a_{nom}  (mean \\pm \\sigma_{seed}),  %s', lab));
        a = ax(2);
        fill(a, [t(w); flipud(t(w))], [em(w) - es(w); flipud(em(w) + es(w))], BANDS(ia, :), 'FaceAlpha', 0.25, 'EdgeColor', 'none');
        H.e(ia) = plot(a, t(w), em(w), '-', 'Color', COLS(ia, :), 'LineWidth', 1.8, 'DisplayName', sprintf('e_a  (mean \\pm \\sigma_{seed}),  %s', lab));
        a = ax(3);
        fill(a, [t(w); flipud(t(w))], [cm(w) - cs(w); flipud(cm(w) + cs(w))], BANDS(ia, :), 'FaceAlpha', 0.25, 'EdgeColor', 'none');
        H.c(ia) = plot(a, t(w), cm(w), '-', 'Color', COLS(ia, :), 'LineWidth', 1.8, 'DisplayName', sprintf('c_{hat}  (mean \\pm \\sigma_{seed}),  %s', lab));
        H.p(ia) = plot(a, t(w), cm(w) + cp(w), '--', 'Color', COLS(ia, :), 'LineWidth', 1.0, 'DisplayName', sprintf('\\pm sqrt(P),  %s', lab));
        plot(a, t(w), cm(w) - cp(w), '--', 'Color', COLS(ia, :), 'LineWidth', 1.0);
    end
    hfit = yline(ax(3), cfit, '--', 'Color', [0.1 0.1 0.1], 'LineWidth', 2.2, 'DisplayName', sprintf('c band fit of this wall  %+.3f', cfit));
    yline(ax(2), 0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
    YL = {[0.3 0.7], [0 0.2], [0 0.7]};  YLAB = {'a / a_{nom}', 'e_a = a_{true} - a_{hat}  [a / a_{nom}]', 'c'};
    if TW(1) < 1.9; YL = {[0 1], [-0.1 0.2], [-0.3 0.7]}; end               % whole-run view
    for rr = 1:3
        a = ax(rr);  xlim(a, TW);  ylim(a, YL{rr});
        plot(a, [2.5 2.5], YL{rr}, '--', 'Color', CG, 'LineWidth', 1.0, 'HandleVisibility', 'off');
        ylabel(a, YLAB{rr}, 'FontSize', FS, 'FontWeight', 'bold');  if rr == 3; xlabel(a, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
        h0 = plot(a, NaN, NaN, 'w.', 'DisplayName', 'CELL wall, plane init');
        switch rr
            case 1; L = [h0 htrue H.a];
            case 2; L = [h0 H.e];
            case 3; L = [h0 H.c H.p hfit];
        end
        NC = 2;  if nA > 3; NC = 1; end                                         % many arms: one legend column, nothing clipped
        legend(a, L, 'Location', 'northoutside', 'NumColumns', NC, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid(a, 'off');
    end
    fn = fullfile(od, sprintf('c_r2_cell_zoom%s.png', ftag));  exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
end
