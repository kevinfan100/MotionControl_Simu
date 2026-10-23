% PURPOSE (2026-09-28, user: "original plot style, zoomed in, cell only"): cell wall, c-only law (f1p2), plane init, prior 0.10,
%   lambda = 1, T_d = 2 s, 10 seeds; bottoms 1.10 R (f1p2_cell_PI_sig010_ramp2.mat) and 1.05 R (..._hb105.mat) overlaid on the
%   plot_plaw_walls layout (time axis; rows a_true/a_hat, e_a, c_hat with +- sigma_seed band, +- sqrt(P) dashed, band-fit c),
%   zoomed to the end of the descent (2.0 -- 2.6 s; descent ends at 2.5 s). Hold not the subject.
%   House style. Output reference/eq17_analysis/derivation/figures/c_bottom_cell_zoom.png   EXPIRES: with the c-only line
function plot_c_bottom_cell_zoom()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    rd = fullfile(root, 'test_results', 'temp_plaw_walls');  od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    HB = {'', '_hb105', '_hb104'};  HBV = [1.10 1.05 1.04];                    % 1.03 and 1.01 crossed the visible surface (plant refuses h < 1)
    COLS = [0 0.2 0.9; 0.85 0.45 0; 0 0.6 0.2];  BANDS = [0.45 0.55 0.95; 0.95 0.7 0.4; 0.5 0.85 0.55];  COL_TRUE = [0.8 0 0];
    here_rd = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'test_results', 'temp_plaw_walls');
    keep = cellfun(@(s) exist(fullfile(here_rd, sprintf('f1p2_cell_PI_sig010_ramp2%s.mat', s)), 'file') == 2, HB);
    HB = HB(keep);  HBV = HBV(keep);  COLS = COLS(keep, :);  BANDS = BANDS(keep, :);  nB = numel(HB);
    FS = 18; LFS = 13; AXLW = 2.0; CG = [0.5 0.5 0.5];  TW = [2.0 2.6];
    f = figure('Position', [60 60 760 1250], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = gobjects(3, 1);  for rr = 1:3; ax(rr) = nexttile(rr); hold(ax(rr), 'on'); box(ax(rr), 'on'); end
    H = struct('a', gobjects(1, nB), 'e', gobjects(1, nB), 'c', gobjects(1, nB), 'p', gobjects(1, nB));  htrue = gobjects(1, nB);  cfit = NaN;
    TSTY = {'-', '--', ':'};
    fprintf('\n%-5s | %9s %9s %8s | %8s %8s\n', 'h_b', 'a_true_end', 'a_hat_end', 'e_a_end', 'c_hat_end', 'sqrtP_end');
    for ib = 1:nB
        r = load(fullfile(rd, sprintf('f1p2_cell_PI_sig010_ramp2%s.mat', HB{ib})));
        t = r.t;  desc = t >= r.seg(1) & t < r.seg(2);  ie = find(desc, 1, 'last');  w = t >= TW(1) - 0.05 & t <= TW(2) + 0.05;  cfit = r.c_fit;
        am = mean(r.a_hat, 2);  as = std(r.a_hat, 0, 2);  at = mean(r.a_true, 2);
        ea = r.a_true - r.a_hat;  em = mean(ea, 2);  es = std(ea, 0, 2);
        cm = mean(r.c_hat, 2);  cs = std(r.c_hat, 0, 2);  cp = mean(r.sP_c, 2);
        fprintf('%-5.2f | %9.3f %9.3f %+8.3f | %8.3f %8.3f\n', HBV(ib), at(ie), am(ie), em(ie), cm(ie), cp(ie));
        lab = sprintf('h_b = %.2f R', HBV(ib));
        % row 1: gain
        a = ax(1);
        fill(a, [t(w); flipud(t(w))], [am(w) - as(w); flipud(am(w) + as(w))], BANDS(ib, :), 'FaceAlpha', 0.25, 'EdgeColor', 'none');
        htrue(ib) = plot(a, t(w), at(w), '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', sprintf('a_{true} / a_{nom}, h_b = %.2f R', HBV(ib)));
        set(htrue(ib), 'LineStyle', TSTY{ib});
        H.a(ib) = plot(a, t(w), am(w), '-', 'Color', COLS(ib, :), 'LineWidth', 1.8, 'DisplayName', sprintf('a_{hat} / a_{nom}  (mean \\pm \\sigma_{seed}),  %s', lab));
        plot(a, t(ie), am(ie), 'o', 'Color', COLS(ib, :), 'MarkerFaceColor', COLS(ib, :), 'MarkerSize', 7, 'HandleVisibility', 'off');
        % row 2: gain error
        a = ax(2);
        fill(a, [t(w); flipud(t(w))], [em(w) - es(w); flipud(em(w) + es(w))], BANDS(ib, :), 'FaceAlpha', 0.25, 'EdgeColor', 'none');
        H.e(ib) = plot(a, t(w), em(w), '-', 'Color', COLS(ib, :), 'LineWidth', 1.8, 'DisplayName', sprintf('e_a  (mean \\pm \\sigma_{seed}),  %s', lab));
        plot(a, t(ie), em(ie), 'o', 'Color', COLS(ib, :), 'MarkerFaceColor', COLS(ib, :), 'MarkerSize', 7, 'HandleVisibility', 'off');
        % row 3: the constant
        a = ax(3);
        fill(a, [t(w); flipud(t(w))], [cm(w) - cs(w); flipud(cm(w) + cs(w))], BANDS(ib, :), 'FaceAlpha', 0.25, 'EdgeColor', 'none');
        H.c(ib) = plot(a, t(w), cm(w), '-', 'Color', COLS(ib, :), 'LineWidth', 1.8, 'DisplayName', sprintf('c_{hat}  (mean \\pm \\sigma_{seed}),  %s', lab));
        H.p(ib) = plot(a, t(w), cm(w) + cp(w), '--', 'Color', COLS(ib, :), 'LineWidth', 1.0, 'DisplayName', sprintf('\\pm sqrt(P),  %s', lab));
        plot(a, t(w), cm(w) - cp(w), '--', 'Color', COLS(ib, :), 'LineWidth', 1.0);
        plot(a, t(ie), cm(ie), 'o', 'Color', COLS(ib, :), 'MarkerFaceColor', COLS(ib, :), 'MarkerSize', 7, 'HandleVisibility', 'off');
    end
    hfit = yline(ax(3), cfit, '--', 'Color', [0.1 0.1 0.1], 'LineWidth', 2.2, 'DisplayName', sprintf('c band fit of this wall  %+.3f', cfit));
    yline(ax(2), 0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
    YL = {[0.3 0.7], [0 0.2], [0 0.7]};  LAB = {'a / a_{nom}', 'e_a = a_{true} - a_{hat}  [a / a_{nom}]', 'c'};
    for rr = 1:3
        a = ax(rr);  xlim(a, TW);  ylim(a, YL{rr});
        plot(a, [2.5 2.5], YL{rr}, '--', 'Color', CG, 'LineWidth', 1.0, 'HandleVisibility', 'off');
        ylabel(a, LAB{rr}, 'FontSize', FS, 'FontWeight', 'bold');  if rr == 3; xlabel(a, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
        h0 = plot(a, NaN, NaN, 'w.', 'DisplayName', 'CELL wall, plane init');
        switch rr
            case 1; L = [h0 htrue H.a];
            case 2; L = [h0 H.e];
            case 3; L = [h0 H.c H.p hfit];
        end
        legend(a, L, 'Location', 'northoutside', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid(a, 'off');
    end
    fn = fullfile(od, 'c_bottom_cell_zoom.png');  exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
end
