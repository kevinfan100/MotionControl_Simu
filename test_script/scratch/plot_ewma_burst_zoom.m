% STATUS: ACTIVE (scratch figure) | PURPOSE: make the EWMA recursion visible on
%   ONE burst event: why sigma-hat^2 (a_cov = 0.05) can jump 4x at t ~ 0.97 s
%   on the Meng ramp, production arm, seed 7 (page 1 of readout_chain_record).
%
%   Rows (zoom window, every sample drawn as a stem):
%     1  F_th        thermal force, +-1 sd lines           <- four same-sign kicks
%     2  dh_m, dh_mr tracking error and its HP residual    <- 7-step excursion
%     3  in  = a_cov*(dh_mr)^2   what the EWMA takes IN this step
%        out = a_cov*sigma2[k]   what it LEAKS this step
%        (stationary <=> in ~ out; burst <=> in >> out)
%     4  sigma2[k+1] = sigma2[k] + in - out   staircase, baseline dashed
%   Chain identical to plot_dhm_to_ahm.m eqs (1)-(4), units um / um^2.
function out = plot_ewma_burst_zoom(mat, seed_idx, tzoom)

    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    if nargin < 1 || isempty(mat);      mat = fullfile(od, 'pair_both_arm.mat'); end
    if nargin < 2 || isempty(seed_idx); seed_idx = 1; end
    if nargin < 3 || isempty(tzoom);    tzoom = [0.955 1.000]; end
    S = load(mat); O = S.oBoth.O{1}; r = O.runs{seed_idx};
    a_pd = S.oBoth.ARMS{1}.a_pd;  a_cov = S.oBoth.ARMS{1}.a_cov;  ax = 3;

    t = r.tout(:); dhm = r.dh_m_out(:, ax); Fth = r.F_th_out(:, ax); N = numel(dhm);
    dhmd = zeros(N+1, 1);
    for k = 1:N; dhmd(k+1) = a_pd*dhm(k) + (1-a_pd)*dhmd(k); end
    dhmr = dhm - dhmd(2:end);
    stat = t > 1 & t < 4;                       % far-field stationary stretch
    s2 = zeros(N+1, 1); s2(1) = mean(dhmr(stat).^2);
    for k = 1:N; s2(k+1) = a_cov*dhmr(k)^2 + (1-a_cov)*s2(k); end
    in  = a_cov * dhmr.^2;                      % injected this step
    lk  = a_cov * s2(1:N);                      % leaked this step
    base = median(s2(stat)); sdF = std(Fth(stat)); sdr = std(dhmr(stat));

    m = t >= tzoom(1) & t <= tzoom(2);  tt = t(m);
    fprintf('seed %d  a_pd %.3g  a_cov %.3g  window %.4f-%.4f s (%d samples)\n', ...
            O.seeds(seed_idx), a_pd, a_cov, tzoom, nnz(m));
    fprintf('baseline sigma2 %.3e  peak %.3e (x%.1f)  sd(F_th) %.3f pN  sd(dh_mr) %.4f um\n', ...
            base, max(s2([false; m])), max(s2([false; m]))/base, sdF, sdr);
    fprintf('sum(in) %.3e  sum(out) %.3e  over window; steps with in > 3*out: %d\n', ...
            sum(in(m)), sum(lk(m)), nnz(in(m) > 3*lk(m)));

    C_F = [0.25 0.25 0.25]; C_M = [0.55 0.78 1.0]; C_R = [0 0.2 0.9];
    C_IN = [0.85 0.33 0.10]; C_OUT = [0.6 0.6 0.6]; C_S = [0.85 0.33 0.10];
    FS = 18; LFS = 13; AXLW = 2.0; ms = 4;
    f = figure('Position', [10 10 1700 1500], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    A = gobjects(4, 1);

    a = nexttile(tl); A(1) = a; hold(a, 'on');
    h1 = stem(a, tt, Fth(m), 'filled', 'Color', C_F, 'MarkerSize', ms, 'LineWidth', 1.2);
    h2 = yline(a, sdF, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2); yline(a, -sdF, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2);
    legend(a, [h1 h2], {'F_{th}[k]', sprintf('\\pm1 sd = %.2f pN', sdF)}, 'Location', 'northoutside', ...
           'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    ylabel(a, 'F_{th}  (pN)', 'FontSize', FS, 'FontWeight', 'bold');

    a = nexttile(tl); A(2) = a; hold(a, 'on');
    h1 = stem(a, tt, dhm(m), 'filled', 'Color', C_M, 'MarkerSize', ms, 'LineWidth', 1.0);
    h2 = stem(a, tt, dhmr(m), 'filled', 'Color', C_R, 'MarkerSize', ms, 'LineWidth', 1.2);
    h3 = yline(a, sdr, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2); yline(a, -sdr, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2);
    legend(a, [h1 h2 h3], {'\delta h_m[k]', '\delta h_{mr}[k] = \delta h_m[k] - \delta h_{md}[k+1]', ...
           sprintf('\\pm1 sd(\\delta h_{mr}) = %.3f \\mum', sdr)}, 'Location', 'northoutside', ...
           'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    ylabel(a, '\delta h  (\mum)', 'FontSize', FS, 'FontWeight', 'bold');

    a = nexttile(tl); A(3) = a; hold(a, 'on');
    dt = median(diff(tt)) * 0.22;
    h1 = stem(a, tt - dt, in(m), 'filled', 'Color', C_IN, 'MarkerSize', ms, 'LineWidth', 1.4);
    h2 = stem(a, tt + dt, lk(m), 'filled', 'Color', C_OUT, 'MarkerSize', ms, 'LineWidth', 1.4);
    legend(a, [h1 h2], {sprintf('in[k] = a_{cov} (\\delta h_{mr}[k])^2    a_{cov} = %.3g', a_cov), ...
           'out[k] = a_{cov} \^\sigma^2[k]   (leak)'}, 'Location', 'northoutside', ...
           'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    ylabel(a, {'per-step', '(\mum^2)'}, 'FontSize', FS, 'FontWeight', 'bold');

    a = nexttile(tl); A(4) = a; hold(a, 'on');
    h1 = stairs(a, tt, s2([false; m]), '-', 'Color', C_S, 'LineWidth', 2.4);
    h2 = yline(a, base, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.4);
    legend(a, [h1 h2], {'\^\sigma^2[k+1] = \^\sigma^2[k] + in[k] - out[k]', ...
           sprintf('baseline (median 1-4 s) = %.2e', base)}, 'Location', 'northoutside', ...
           'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    ylabel(a, '\^\sigma^2_{\delta h_{mr}}  (\mum^2)', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(a, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');

    for q = 1:4
        set(A(q), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
        grid(A(q), 'off'); xlim(A(q), tzoom);
        if q < 4; set(A(q), 'XTickLabel', []); end
    end
    fn = fullfile(od, 'ewma_burst_zoom.png');
    exportgraphics(f, fn, 'Resolution', 150); close(f);
    fprintf('figure -> %s\n', fn);
    out = struct('t', tt, 'Fth', Fth(m), 'dhm', dhm(m), 'dhmr', dhmr(m), 'in', in(m), 'out', lk(m), 's2', s2([false; m]));
end
