% PURPOSE (2026-09-23 evening, user: "c and p, h notation, pair every observability check with the simulation result"):
%   the (c, p) slope law a' = (1-c)/h^p (plaw_cp_slope, run_plaw_walls law 'f1', kind 'fixed') held at h_d = 22.2 R and 1.10 R
%   for 3 s, plane init, plants plane and cell, 10 seeds. One figure, columns = plane 22.2 | plane 1.10 | cell 22.2 | cell 1.10
%   (same init, y shared per row), rows: a_bar true / hat (mean +- sigma_seed), e_a = a_true - a_hat (absolute),
%   c_hat (mean +- sigma_seed, +- sqrt(P) dashed, band-fit reference of THIS wall), p_hat (same). House style (plot_var_ahat_6state.m).
%   Reads test_results/temp_plaw_walls/f1_fixed_<plant>_PI_h<H>.mat.  Output reference/eq17_analysis/derivation/figures/f1_fixed_PI.png
%   EXPIRES: with the (c, p) fixed-point line
function plot_f1_fixed(arm, law)
    if nargin < 1 || isempty(arm); arm = 'PI'; end
    if nargin < 2 || isempty(law); law = 'f1'; end               % 'f1' (c, p free) | 'f1p2' (p pinned at 2)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    rd = fullfile(root, 'test_results', 'temp_plaw_walls');  od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; BANDC = [0.45 0.55 0.95]; FS = 15; LFS = 11; AXLW = 2.0; CREF = [0.1 0.1 0.1];
    CASES = {'plane', '22p20', '22.2'; 'plane', '1p10', '1.10'; 'cell', '22p20', '22.2'; 'cell', '1p10', '1.10'};
    lab_init = 'plane init';  if strcmp(arm, 'T'); lab_init = 'seed = band fit'; end
    files = cellfun(@(pl, h) fullfile(rd, sprintf('%s_fixed_%s_%s_h%s.mat', law, pl, arm, h)), CASES(:, 1), CASES(:, 2), 'UniformOutput', false);
    keep = cellfun(@(f) exist(f, 'file') == 2, files);
    if ~any(keep); error('no f1_fixed_*.mat found in %s', rd); end
    R = cellfun(@load, files(keep), 'UniformOutput', false);  C = CASES(keep, :);  nC = numel(R);
    has_p = ~startsWith(law, 'f1p2');  nRow = 3 + double(has_p);   % 'f1p2' pins p -> no p row (user 09-23)
    f = figure('Position', [40 40 430 * nC + 100, 310 * nRow + 10], 'Color', 'w', 'Visible', 'off');
    tiledlayout(nRow, nC, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = gobjects(nRow, nC);
    for v = 1:nC
        r = R{v};  t = r.t;  ea = r.a_true - r.a_hat;  T_END = t(end);
        % ---- row 1: gain
        ax(1, v) = nexttile(v); hold on; box on;
        am = mean(r.a_hat, 2); as = std(r.a_hat, 0, 2);  at = mean(r.a_true, 2);
        fill([t; flipud(t)], [am - as; flipud(am + as)], BANDC, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
        h1 = plot(t, at, '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'a_{true} / a_{nom}');
        h2 = plot(t, am, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', 'a_{hat} / a_{nom}  (mean \pm \sigma_{seed})');
        h0 = plot(NaN, NaN, 'w.', 'DisplayName', sprintf('%s wall,  fixed  h_d = %s R,  %s', upper(C{v, 1}), C{v, 3}, lab_init));
        xlim([0 T_END]); ylim([0 1.05]);
        if v == 1; ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([h0 h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'NumColumns', 1);
        % ---- row 2: e_a
        ax(2, v) = nexttile(nC + v); hold on; box on;
        em = mean(ea, 2); es = std(ea, 0, 2);
        fill([t; flipud(t)], [em - es; flipud(em + es)], BANDC, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
        h3 = plot(t, em, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', 'e_a = a_{true} - a_{hat}  (mean \pm \sigma_{seed})');
        yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
        xlim([0 T_END]); ylim([-0.5 0.5]);
        if v == 1; ylabel('e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold'); end
        if v == 1; legend(h3, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on'); end
        % ---- row 3: c_hat
        ax(3, v) = nexttile(2 * nC + v); hold on; box on;
        local_const_row(t, r.c_hat, r.sP_c, r.c_fit, 'c', COL_HAT, BANDC, CREF, v == 1, LFS);
        xlim([0 T_END]); ylim([-0.4 1.0]);
        if v == 1; ylabel('c', 'FontSize', FS, 'FontWeight', 'bold'); end
        if has_p
            % ---- row 4: p_hat
            ax(4, v) = nexttile(3 * nC + v); hold on; box on;
            local_const_row(t, r.d_hat, r.sP_d, r.d_fit, 'p', COL_HAT, BANDC, CREF, v == 1, LFS);
            xlim([0 T_END]); ylim([1.2 2.6]);
            if v == 1; ylabel('p', 'FontSize', FS, 'FontWeight', 'bold'); end
        end
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        for q = 1:nRow; set(ax(q, v), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid(ax(q, v), 'off'); end
    end
    fn = fullfile(od, sprintf('%s_fixed_%s.png', law, arm));  exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
end

function local_const_row(t, x, sp, xfit, lab, COL_HAT, BANDC, CREF, with_legend, LFS)
    cm = mean(x, 2); cs = std(x, 0, 2); cp = mean(sp, 2);
    fill([t; flipud(t)], [cm - cs; flipud(cm + cs)], BANDC, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
    h4 = plot(t, cm, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', [lab '_{hat}  (mean \pm \sigma_{seed})']);
    h5 = plot(t, cm + cp, '--', 'Color', COL_HAT, 'LineWidth', 1.0, 'DisplayName', '\pm sqrt(P)');
    plot(t, cm - cp, '--', 'Color', COL_HAT, 'LineWidth', 1.0);
    h6 = yline(xfit, '--', 'Color', CREF, 'LineWidth', 2.2, 'DisplayName', sprintf('%s band fit of this wall  %+.3f', lab, xfit));
    if with_legend
        legend([h4 h5 h6], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    end
end
