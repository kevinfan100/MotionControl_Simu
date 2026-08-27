% STATUS: ACTIVE (scratch figure) | PURPOSE: the whitened gain measurement
%   y2 that the estimator actually consumes, next to the raw IIR readout it
%   is built from. Meng ramp, production arm, seed 7, z axis, a_bar units.
%
%   Row 1  a_bar_wm[k]   raw readout (paper (13), AR(1) with pole 1-a_cov)
%          a_bar true (red)
%   Row 2  y2[k] = a_bar_wm[k] - (1-a_cov) a_bar_wm[k-1]     whitened (controller :950)
%          y2_hat[k] = y2[k] - e_ah[k]                      what the estimator predicted
%          a_cov * a_bar true (red)                          where y2 should sit
%   Row 3  e_ah[k] = y2[k] - y2_hat[k]                       the innovation fed to l_42
%
%   y2 is not logged; it is rebuilt from a_xm_out / a_disp (rebuild_am_chain
%   proved a_xm_out == a_bar_wm to 2e-16). Sanity: y2_hat / (a_cov a_bar_hat)
%   must sit at ~1 (the back-off term is < 1 %), printed to console.
function out = plot_y2_whitened(mat, seed_idx, ax, tlim)

    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    if nargin < 1 || isempty(mat);      mat = fullfile(od, 'pair_both_arm.mat'); end
    if nargin < 2 || isempty(seed_idx); seed_idx = 1; end
    if nargin < 3 || isempty(ax);       ax = 3; end
    if nargin < 4 || isempty(tlim);     tlim = [0.5 10.5]; end
    S = load(mat); O = S.oBoth.O{1}; r = O.runs{seed_idx};
    a_cov = r.ctrl_const.a_cov;

    t   = r.tout(:);
    ad  = r.a_hat_out(1, ax) / r.a_bar_hat_out(1, ax);      % a_disp [um/pN per unit a_bar]
    awm = r.a_xm_out(:, ax) / ad;                            % a_bar_wm[k]
    at  = r.a_true_out(:, ax) / ad;                          % a_bar true
    ah  = r.a_bar_hat_out(:, ax);
    e2  = r.innov_y2_out(:, ax);
    y2  = [NaN; awm(2:end) - (1 - a_cov) * awm(1:end-1)];    % whitened
    y2h = y2 - e2;                                           % estimator's prediction

    m = t >= tlim(1) & t <= tlim(2);  w = t >= 1 & t <= 4;
    fprintf('seed %d  a_cov %.3g  axis %d\n', O.seeds(seed_idx), a_cov, ax);
    fprintf('1-4 s: mean a_bar_wm %.4f (true %.4f)  | mean y2 %.5f  sd y2 %.5f  | mean a_cov*true %.5f\n', ...
            mean(awm(w)), mean(at(w)), mean(y2(w)), std(y2(w)), a_cov*mean(at(w)));
    fprintf('sanity y2_hat/(a_cov a_bar_hat): median %.4f  [%.4f, %.4f]\n', ...
            median(y2h(w)./(a_cov*ah(w))), min(y2h(w)./(a_cov*ah(w))), max(y2h(w)./(a_cov*ah(w))));
    fprintf('acf of y2 at lag 1..3 (1-4 s): %s   (raw a_bar_wm: %s)\n', ...
            mat2str(local_acf(y2(w), 3), 3), mat2str(local_acf(awm(w), 3), 3));

    C_T = [0.8 0 0]; C_E = [0 0.2 0.9]; C_M = [0.55 0.78 1.0];
    FS = 18; LFS = 13; AXLW = 2.0;
    f = figure('Position', [10 10 1600 1400], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    A = gobjects(3, 1);

    a = nexttile(tl); A(1) = a; hold(a, 'on');
    h1 = plot(a, t(m), awm(m), '-', 'Color', C_M, 'LineWidth', 0.8);
    h2 = plot(a, t(m), at(m),  '-', 'Color', C_T, 'LineWidth', 2.4);
    legend(a, [h1 h2], {'a_{wm}[k]  (IIR readout, eq. 13)', 'a  true'}, 'Location', 'northoutside', ...
           'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    ylabel(a, 'a / a_o', 'FontSize', FS, 'FontWeight', 'bold');

    a = nexttile(tl); A(2) = a; hold(a, 'on');
    h1 = plot(a, t(m), y2(m),  '-', 'Color', C_M, 'LineWidth', 0.8);
    h2 = plot(a, t(m), y2h(m), '-', 'Color', C_E, 'LineWidth', 1.6);
    h3 = plot(a, t(m), a_cov*at(m), '-', 'Color', C_T, 'LineWidth', 2.4);
    legend(a, [h1 h2 h3], {sprintf('y_2[k] = a_{wm}[k] - (1-a_{cov}) a_{wm}[k-1]      a_{cov} = %.3g', a_cov), ...
           '\^y_2[k]  (estimator)', 'a_{cov} \cdot a  true'}, 'Location', 'northoutside', ...
           'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    ylabel(a, 'y_2  (a / a_o)', 'FontSize', FS, 'FontWeight', 'bold');

    a = nexttile(tl); A(3) = a; hold(a, 'on');
    yline(a, 0, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off');
    h1 = plot(a, t(m), e2(m), '-', 'Color', C_E, 'LineWidth', 0.8);
    legend(a, h1, {'e_{ah}[k] = y_2[k] - \^y_2[k]'}, 'Location', 'northoutside', ...
           'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    ylabel(a, 'e_{ah}  (a / a_o)', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(a, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');

    for q = 1:3
        set(A(q), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
        grid(A(q), 'off'); xlim(A(q), tlim);
        if q < 3; set(A(q), 'XTickLabel', []); end
    end
    fn = fullfile(od, 'y2_whitened.png');
    exportgraphics(f, fn, 'Resolution', 150); close(f);
    fprintf('figure -> %s\n', fn);
    out = struct('t', t, 'a_wm', awm, 'y2', y2, 'y2_hat', y2h, 'e_ah', e2, 'a_true', at);
end

function r = local_acf(x, L)
    x = x(isfinite(x)); x = x - mean(x); r = zeros(1, L);
    for l = 1:L; r(l) = sum(x(1+l:end).*x(1:end-l)) / sum(x.^2); end
end
