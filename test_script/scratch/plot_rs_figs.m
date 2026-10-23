% STATUS: SCRATCH -- 2026-10-11, figures of 1011_random_signal_am.tex (random signals, the measured input gain, the KF).
%   Data: A1 = 06-13 gain_compare run (test_results/gain_compare/f1Hz/runs.mat, 6-state Vpersonal, raw y2, lambda_c 0.7,
%   a_cov = a_pd = 0.05, 200 seeds; runs.B = the control law uses the estimate). z axis. Window W = 0.25-0.75 s (after the IIR
%   warm-up, far field: the gain changes < 1% inside it). Whitened y_2 = (dx_r^2 - C_n sigma_n^2)/(C_dpmr 4 k_B T)
%   (variable u below; the logged a_xm is exactly its EWMA (a_cov) after the warm-up). 10-12: no a_m^(1) symbol (user):
%   pages 3-5 show dx_r^2 [nm^2] (affine in u, same rho), pages 6-7 call u 'y_2 (whitened)'. Page 8: both checks are
%   variances over seeds at a fixed time (06: a_m vs K_var IF_eff (a+xi)^2; 08: whitened y_2 vs 2 (a+xi)^2); the difference
%   is whether the correlation of y_2 along time enters r_2. Gains divided by a_nom (a-bar). Notation and terms of motion_ctrl (10-11): thermal motion dx_T, random component
%   dx_r, sigma^2 for variances, r_2 / p_44 lowercase entries. Thermal displacement in one step = a f_T (first term of eps_x;
%   no delta-x_T symbol, user 10-12). Page 3: points seed 1, rho(1) over 200 seeds (same estimator as pages 5, 7).
%   10-13 (user): report reduced to one factor, the autocorrelation of y_2; 'all' = p1, q2 (quadrant lag plots = the
%   rho formula), q3 (estimator belief r_2/N vs actual, with the finite-N formula), q4 (epsilon scan, a_m vs whitened y_2,
%   first-order estimator, cached runs of run_mc_s4_first_order.m tri10). p2-p8 kept for reference.
%   which: 'p1' question | 'p2' white signal | 'p3' lag plots | 'p4' averages | 'p5' chain | 'p6' KF belief | 'p7' whitening |
%   'p8' the two checks | 'all'. 10-12: page 4 (variance of an average) dropped from the report; p4 kept but not in 'all'.
%   Framing (user 10-12): with a_cov (0.05, y_2 = a_m) vs without a_cov (a_cov = 1: a_m from one sample = whitened y_2).
function plot_rs_figs(which)
    if nargin < 1; which = 'all'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    persistent D
    if isempty(D); D = local_load(root); end
    P = {'p1', 'q2', 'q3', 'q4'};                      % 10-13 report: one factor (autocorrelation of y_2), 4 pages
    if ~strcmp(which, 'all'); P = {which}; end
    for i = 1:numel(P); feval(['local_' P{i}], D, fd); end
end

function D = local_load(root)
    S = load(fullfile(root, 'test_results', 'gain_compare', 'f1Hz', 'runs.mat'));  nz = S.runs.B.noisy;
    o = nz(1).simOut;  cc = o.ctrl_const;  t = o.tout(2:end);  an = 0.01471;
    W = t > 0.25 & t < 0.75;  ns = numel(nz);  N = numel(t);
    D.t = t;  D.W = W;  D.an = an;  D.cc = cc;
    [D.kick, D.r, D.u, D.r2, D.am, D.ah, D.at, D.Pa] = deal(zeros(N, ns));
    for s = 1:ns
        so = nz(s).simOut;  d = so.diag;
        D.kick(:, s) = so.a_true_out(2:end, 3) .* so.F_th_out(2:end, 3);
        D.r(:, s) = d.dx_r(2:end, 3);
        D.u(:, s) = (d.dx_r(2:end, 3).^2 - cc.C_n * cc.sigma2_n_s(3)) / (cc.C_dpmr * 4 * cc.kBT) / an;
        D.r2(:, s) = (d.dx_r(2:end, 3) * 1e3).^2;            % dx_r^2 [nm^2]
        D.am(:, s) = d.a_xm(2:end, 3) / an;  D.ah(:, s) = d.a_hat(2:end, 3) / an;  D.at(:, s) = so.a_true_out(2:end, 3) / an;
        D.Pa(:, s) = d.P_a(2:end, 3) / an^2;
    end
    fprintf('window W: a_true %.4f .. %.4f (change %.2f%%)\n', min(mean(D.at(W, :), 2)), max(mean(D.at(W, :), 2)), ...
            100 * (max(mean(D.at(W, :), 2)) / min(mean(D.at(W, :), 2)) - 1));
end

function c = C()                                     % colours and fonts (figure-style.md)
    c = struct('T', [0.85 0 0], 'E', [0 0.25 0.85], 'M', [0.55 0.72 0.98], 'G', [0.45 0.45 0.45], 'FS', 18, 'LW', 1.8);
end

function s = AB(); s = char(257); end               % a-bar (normalized motion gain), house bold font
function s = HAT(); s = char(770); end              % combining hat

function local_fmt(ax, c)
    set(ax, 'FontSize', c.FS, 'FontWeight', 'bold', 'LineWidth', c.LW);  box(ax, 'on');  grid(ax, 'off');
end

function rho = local_acf(X, L)                       % seed-averaged autocorrelation, columns = seeds
    rho = zeros(L + 1, 1);
    for j = 1:size(X, 2); x = X(:, j) - mean(X(:, j)); c = xcorr(x, L, 'coeff'); rho = rho + c(L + 1:end) / size(X, 2); end
end

function ratio = local_blockvar(X, Ns)               % var(mean of N consecutive) / var(one), pooled over seeds
    v1 = var(X(:));  ratio = zeros(size(Ns));
    for i = 1:numel(Ns)
        n = Ns(i);  m = floor(size(X, 1) / n);  B = reshape(X(1:m * n, :), n, m, size(X, 2));  bm = squeeze(mean(B, 1));
        ratio(i) = var(bm(:)) / v1;
    end
end

function local_save(f, fd, name)
    out = fullfile(fd, [name '.png']);  exportgraphics(f, out, 'Resolution', 140);  close(f);  fprintf('figure: %s\n', out);
end

function local_p1(D, fd)                             % the question: smooth, lagging, over-confident estimate
    c = C();  t = D.t;  f = figure('Position', [40 40 1700 1000], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = nexttile(tl);  hold(ax, 'on');
    hm = plot(ax, t, D.am(:, 1), '-', 'Color', c.M, 'LineWidth', 0.8, 'DisplayName', [AB '_m']);
    ht = plot(ax, t, mean(D.at, 2), '-', 'Color', c.T, 'LineWidth', 5, 'DisplayName', AB);
    hh = plot(ax, t, D.ah(:, 1), '-', 'Color', c.E, 'LineWidth', 1.8, 'DisplayName', [AB HAT]);
    ylim(ax, [0 1.3]);  xlim(ax, [0 4]);  ylabel(ax, 'motion gain');
    legend(ax, [ht hh hm], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 15, 'FontWeight', 'bold');  local_fmt(ax, c);
    ax2 = nexttile(tl);  hold(ax2, 'on');  e = D.at(:, 1) - D.ah(:, 1);  sp = 3 * sqrt(D.Pa(:, 1));
    h1 = plot(ax2, t, e, '-', 'Color', c.E, 'LineWidth', 1.4, 'DisplayName', ['e_{' AB '}']);
    h2 = plot(ax2, t, sp, '--', 'Color', c.G, 'LineWidth', 1.8, 'DisplayName', '\pm3 \surdP');  plot(ax2, t, -sp, '--', 'Color', c.G, 'LineWidth', 1.8);
    ylim(ax2, [-0.4 0.4]);  xlim(ax2, [0 4]);  xlabel(ax2, 'Time (sec)');  ylabel(ax2, ['e_{' AB '}']);
    legend(ax2, [h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 15, 'FontWeight', 'bold');  local_fmt(ax2, c);
    w = t > 1.6 & t < 3.4;  fprintf('p1: oscillation, |e| > 3 sqrt(P) in %.0f%% of steps (seed 1)\n', 100 * mean(abs(e(w)) > sp(w)));
    local_save(f, fd, 'fig_rs_p1_question');
end

function local_p2(D, fd)                             % four properties on a white Gaussian signal
    c = C();  x = D.kick(D.W, :) * 1e3;  tt = D.t(D.W) - D.t(find(D.W, 1));  L = 40;
    f = figure('Position', [40 40 1800 560], 'Color', 'w', 'Visible', 'off');  tl = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = nexttile(tl);  plot(ax, tt, x(:, 1), '-', 'Color', c.E, 'LineWidth', 0.8);  xlim(ax, [0 tt(end)]);
    xlabel(ax, 'time (s)');  ylabel(ax, 'a f_T  [nm]');  local_fmt(ax, c);
    ax2 = nexttile(tl);  hold(ax2, 'on');  histogram(ax2, x(:), 80, 'Normalization', 'pdf', 'FaceColor', c.E, 'EdgeColor', 'none');
    xs = linspace(min(x(:)), max(x(:)), 200);  plot(ax2, xs, normpdf(xs, mean(x(:)), std(x(:))), '-', 'Color', c.T, 'LineWidth', 3);
    xlabel(ax2, 'a f_T  [nm]');  ylabel(ax2, 'probability density');  local_fmt(ax2, c);
    ax3 = nexttile(tl);  hold(ax3, 'on');  rho = local_acf(x, L);  stem(ax3, 0:L, rho, 'filled', 'Color', c.E, 'MarkerSize', 4);
    yline(ax3, 0, '-', 'Color', c.G);  ylim(ax3, [-0.2 1.05]);  xlabel(ax3, 'lag \tau  (steps)');  ylabel(ax3, '\rho(\tau)');  local_fmt(ax3, c);
    fprintf('p2: mean %.2f nm, sd %.2f nm, skewness %.2f, rho(1) %.3f\n', mean(x(:)), std(x(:)), skewness(x(:)), rho(2));
    local_save(f, fd, 'fig_rs_p2_white');
end

function local_p3(D, fd)                             % lag plots x[k] vs x[k+1]
    c = C();  S = {D.kick(D.W, 1) * 1e3, D.u(D.W, 1), D.am(D.W, 1)};
    SP = {D.kick(D.W, :), D.u(D.W, :), D.am(D.W, :)};
    NM = {'a f_T  [nm]', [AB '_m,  a_{cov} = 1'], [AB '_m,  a_{cov} = 0.05']};
    f = figure('Position', [40 40 1800 640], 'Color', 'w', 'Visible', 'off');  tl = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    for q = 1:3
        x = S{q};  ax = nexttile(tl);  plot(ax, x(1:end-1), x(2:end), '.', 'Color', c.E, 'MarkerSize', 7);  axis(ax, 'square');
        lim = [min(x) max(x)];  xlim(ax, lim);  ylim(ax, lim);  title(ax, NM{q}, 'FontSize', c.FS - 2);
        xlabel(ax, 'x[k]');  if q == 1; ylabel(ax, 'x[k+1]'); end
        rp = local_acf(SP{q}, 1);                    % rho(1) over 200 seeds, as on the other pages (points: seed 1)
        text(ax, 0.05, 0.92, sprintf('\\rho(1) = %.2f', rp(2)), 'Units', 'normalized', 'FontSize', c.FS, 'FontWeight', 'bold');
        local_fmt(ax, c);
    end
    local_save(f, fd, 'fig_rs_p3_lag');
end

function local_p4(D, fd)                             % variance of an average of N consecutive samples
    c = C();  S = {D.kick(D.W, :), D.r2(D.W, :), D.am(D.W, :)};  NM = {'a f_T', '\deltax_r^2', [AB '_m']};
    Ns = [1 2 5 10 20 50 100 200];  f = figure('Position', [40 40 1800 600], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    for q = 1:3
        rt = local_blockvar(S{q}, Ns);  ax = nexttile(tl);
        h1 = loglog(ax, Ns, rt, 'o-', 'Color', c.E, 'LineWidth', 2.2, 'MarkerFaceColor', c.E, 'DisplayName', 'measured');  hold(ax, 'on');
        h2 = loglog(ax, Ns, 1 ./ Ns, '-', 'Color', c.T, 'LineWidth', 3, 'DisplayName', 'independent: 1/N');
        ylim(ax, [3e-3 1.5]);  xlim(ax, [1 200]);  title(ax, NM{q}, 'FontSize', c.FS - 2);  xlabel(ax, 'N');
        if q == 1; ylabel(ax, 'Var(mean of N) / Var(one)');  legend(ax, [h1 h2], 'Location', 'southwest', 'FontSize', 14); end
        local_fmt(ax, c);
        fprintf('p4: %-22s N=10 %.3f  N=100 %.3f  -> 100 samples worth %.0f\n', NM{q}, rt(Ns == 10), rt(Ns == 100), 1 / rt(Ns == 100));
    end
    local_save(f, fd, 'fig_rs_p4_average');
end

function local_p5(D, fd)                             % the chain: kick -> residual -> single-step reading -> a_m
    c = C();  S = {D.kick(D.W, :) * 1e3, D.r(D.W, :) * 1e3, D.u(D.W, :), D.am(D.W, :)};  L = 60;
    NM = {'a f_T  [nm]', '\deltax_r  [nm]', {[AB '_m'], 'a_{cov} = 1'}, {[AB '_m'], 'a_{cov} = 0.05'}};  tt = D.t(D.W) - D.t(find(D.W, 1));
    f = figure('Position', [40 40 1800 1500], 'Color', 'w', 'Visible', 'off');  tl = tiledlayout(4, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    for q = 1:4
        x = S{q};  rho = local_acf(x, L);
        ax = nexttile(tl);  plot(ax, tt, x(:, 1), '-', 'Color', c.E, 'LineWidth', 0.8);  xlim(ax, [0 tt(end)]);  ylabel(ax, NM{q});
        if q == 4; xlabel(ax, 'time (s)'); end;  local_fmt(ax, c);
        ax2 = nexttile(tl);  hold(ax2, 'on');  histogram(ax2, x(:), 80, 'Normalization', 'pdf', 'FaceColor', c.E, 'EdgeColor', 'none');
        xs = linspace(min(x(:)), max(x(:)), 200);  plot(ax2, xs, normpdf(xs, mean(x(:)), std(x(:))), '-', 'Color', c.T, 'LineWidth', 2.5);
        local_fmt(ax2, c);
        ax3 = nexttile(tl);  hold(ax3, 'on');  stem(ax3, 0:L, rho, 'filled', 'Color', c.E, 'MarkerSize', 3);  yline(ax3, 0, '-', 'Color', c.G);
        ylim(ax3, [-0.3 1.05]);  if q == 4; xlabel(ax3, 'lag \tau (steps)'); end;  local_fmt(ax3, c);
        if iscell(NM{q}); nm = strjoin(NM{q}, ' '); else; nm = NM{q}; end
        fprintf('p5: %-16s mean %+.3g sd %.3g skew %+.2f rho1 %.3f rho5 %.3f rho20 %.3f\n', nm, mean(x(:)), std(x(:)), skewness(x(:)), rho(2), rho(6), rho(21));
    end
    local_save(f, fd, 'fig_rs_p5_chain');
end

function local_p6(D, fd)                             % what the KF believes after N readings vs the actual variance of their mean
    c = C();  Ns = [1 2 5 10 20 50 100 200];  am = D.am(D.W, :);  u = D.u(D.W, :);
    vam = var(am(:));  vu = var(u(:));  IF = local_blockvar(u, 200) * 200;   % N Var(mean of N) / Var(one) = 1 + 2 sum rho
    f = figure('Position', [40 40 1700 650], 'Color', 'w', 'Visible', 'off');  tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = nexttile(tl);  h1 = loglog(ax, Ns, vam ./ Ns, '--', 'Color', c.G, 'LineWidth', 2.5, 'DisplayName', 'p_{44} = r_2 / N');  hold(ax, 'on');
    h2 = loglog(ax, Ns, vam * local_blockvar(am, Ns), 'o-', 'Color', c.E, 'LineWidth', 2.2, 'MarkerFaceColor', c.E, 'DisplayName', 'actual');
    title(ax, 'a_{cov} = 0.05', 'FontSize', c.FS - 2);  xlabel(ax, 'N (readings)');  ylabel(ax, 'variance of the motion gain');
    legend(ax, [h1 h2], 'Location', 'southwest', 'FontSize', 14);  ylim(ax, [5e-4 20]);  local_fmt(ax, c);
    ax2 = nexttile(tl);  g0 = loglog(ax2, Ns, vu ./ Ns, ':', 'Color', c.G, 'LineWidth', 2.5, 'DisplayName', 'r_2 = \sigma^2_{y_2}');  hold(ax2, 'on');
    g1 = loglog(ax2, Ns, IF * vu ./ Ns, '--', 'Color', c.G, 'LineWidth', 2.5, 'DisplayName', 'r_2 = IF \sigma^2_{y_2}');
    g2 = loglog(ax2, Ns, vu * local_blockvar(u, Ns), 'o-', 'Color', c.E, 'LineWidth', 2.2, 'MarkerFaceColor', c.E, 'DisplayName', 'actual');
    title(ax2, 'a_{cov} = 1', 'FontSize', c.FS - 2);  xlabel(ax2, 'N (readings)');
    legend(ax2, [g0 g1 g2], 'Location', 'southwest', 'FontSize', 14);  ylim(ax2, [5e-4 20]);  local_fmt(ax2, c);
    i100 = Ns == 100;  ra = local_blockvar(am, Ns);  ru = local_blockvar(u, Ns);
    fprintf('p6: Var(a_m) %.3f, Var(u) %.3f, IF (measured, N = 200) %.2f\n', vam, vu, IF);
    fprintf('p6: N = 100: a_m belief %.4f actual %.4f (x%.0f) | u belief %.4f, with IF %.4f, actual %.4f\n', vam / 100, vam * ra(i100), ...
            vam * ra(i100) / (vam / 100), vu / 100, IF * vu / 100, vu * ru(i100));
    local_save(f, fd, 'fig_rs_p6_kf');
end

function local_p7(D, fd)                             % whitening: expected a_m after a step in the gain (10-12: rho panel dropped, user)
    c = C();  ac = D.cc.a_cov;
    k = (-20:80)';  a = ones(size(k));  a(k >= 0) = 0.5;          % the gain drops by half at k = 0 (expected values, no noise)
    m = filter(ac, [1 -(1 - ac)], a, (1 - ac) * 1);
    f = figure('Position', [40 40 1100 650], 'Color', 'w', 'Visible', 'off');  ax2 = axes(f);  hold(ax2, 'on');
    g0 = plot(ax2, k, a, '-', 'Color', c.T, 'LineWidth', 5, 'DisplayName', AB);
    g1 = plot(ax2, k, a, '--', 'Color', c.E, 'LineWidth', 2.2, 'DisplayName', ['E ' AB '_m,  a_{cov} = 1']);
    g2 = plot(ax2, k, m, '-', 'Color', c.M, 'LineWidth', 2.5, 'DisplayName', ['E ' AB '_m,  a_{cov} = 0.05']);
    xline(ax2, (1 - ac) / ac, ':', 'Color', c.G, 'LineWidth', 1.8);
    ylim(ax2, [0.4 1.1]);  xlabel(ax2, 'step k');  ylabel(ax2, 'motion gain (expected value)');
    legend(ax2, [g0 g1 g2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 15);  local_fmt(ax2, c);
    fprintf('p7: a_m reaches half of the step at k = %d, mean delay (1-a)/a = %.0f\n', find(m(k >= 0) <= 0.75, 1) - 1, (1 - ac) / ac);
    local_save(f, fd, 'fig_rs_p7_whiten');
end

function local_p8(D, fd)                             % the two checks: variance over seeds at a fixed time, then correlation along time
    c = C();  cc = D.cc;  an = D.an;  L = 60;  k = find(D.W);
    s2n = cc.sigma2_n_s(3);  xi = cc.C_n * s2n / (cc.C_dpmr * 4 * cc.kBT);       % physical, as in the 06 controller
    a = mean(D.at(k, :), 2) * an;                                               % true motion gain (physical)
    sxT = 4 * cc.kBT * a;  A = cc.IF_abc;                                       % IF_eff of motion_control_law_eq17_6state.m
    IFe = 1 + 2 * (sxT.^2 * A(1) + 2 * sxT * s2n * A(2) + s2n^2 * A(3)) ./ (cc.C_dpmr * sxT + cc.C_n * s2n).^2;
    r1 = cc.K_var * IFe .* (a + xi).^2 / an^2;                                  % check 1 formula (06): variance of a_m
    r2w = 2 * (a + xi).^2 / an^2;                                               % check 2 formula (08): variance of whitened y_2
    q1 = var(D.am(k, :), 0, 2) ./ r1;  q2 = var(D.u(k, :), 0, 2) ./ r2w;  tt = D.t(k);
    f = figure('Position', [40 40 1700 650], 'Color', 'w', 'Visible', 'off');  tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = nexttile(tl);  hold(ax, 'on');
    h2 = plot(ax, tt, movmean(q2, 25), '-', 'Color', c.E, 'LineWidth', 2.2, 'DisplayName', 'check 2 (08): a_{cov} = 1');   % 25-step mean of the per-step ratio
    h1 = plot(ax, tt, movmean(q1, 25), '-', 'Color', c.M, 'LineWidth', 2.2, 'DisplayName', 'check 1 (06): a_{cov} = 0.05');
    yline(ax, 1, '-', 'Color', c.T, 'LineWidth', 3);  ylim(ax, [0 2]);  xlim(ax, tt([1 end]));
    xlabel(ax, 'time (s)');  ylabel(ax, 'variance over 200 seeds / formula');
    legend(ax, [h1 h2], 'Location', 'northoutside', 'Orientation', 'vertical', 'FontSize', 14);  local_fmt(ax, c);
    ra = local_acf(D.am(k, :), L);  ru = local_acf(D.u(k, :), L);
    Sa = local_blockvar(D.am(k, :), 200) * 200;  Su = local_blockvar(D.u(k, :), 200) * 200;   % 1 + 2 sum rho, same estimator as page 6
    ax2 = nexttile(tl);  hold(ax2, 'on');
    g1 = plot(ax2, 0:L, ra, 'o-', 'Color', c.M, 'LineWidth', 2, 'MarkerSize', 4, 'DisplayName', sprintf('a_{cov} = 0.05: 1+2\\Sigma\\rho = %.0f, not in r_2', Sa));
    g2 = plot(ax2, 0:L, ru, 'o-', 'Color', c.E, 'LineWidth', 2, 'MarkerSize', 4, 'DisplayName', sprintf('a_{cov} = 1: IF = %.1f, in r_2', Su));
    yline(ax2, 0, '-', 'Color', c.G);  ylim(ax2, [-0.2 1.05]);  xlabel(ax2, 'lag \tau (steps)');  ylabel(ax2, '\rho(\tau) along time');
    legend(ax2, [g1 g2], 'Location', 'northoutside', 'Orientation', 'vertical', 'FontSize', 14);  local_fmt(ax2, c);
    fprintf('p8: var over seeds / formula: check 1 (a_m) mean %.3f [%.3f %.3f] | check 2 (whitened y2) mean %.3f [%.3f %.3f]\n', ...
            mean(q1), min(q1), max(q1), mean(q2), min(q2), max(q2));
    fprintf('p8: along time (N = 200): 1+2 sum rho, a_m %.1f | whitened y2 %.2f; IF_eff (inside the 06 formula) %.2f\n', Sa, Su, mean(IFe));
    local_save(f, fd, 'fig_rs_p8_checks');
end

function local_q2(D, fd)                             % lag plots about the mean: sign of (x[k]-mu)(x[k+1]-mu) = quadrant
    c = C();  S = {D.kick(D.W, 1) * 1e3, D.u(D.W, 1), D.am(D.W, 1)};
    SP = {D.kick(D.W, :), D.u(D.W, :), D.am(D.W, :)};
    NM = {'a f_T  [nm]', [AB '_m,  a_{cov} = 1'], [AB '_m,  a_{cov} = 0.05']};
    f = figure('Position', [40 40 1800 660], 'Color', 'w', 'Visible', 'off');  tl = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    for q = 1:3
        x = S{q} - mean(SP{q}(:));  ax = nexttile(tl);  hold(ax, 'on');
        lo = prctile(x, 0.5);  hi = prctile(x, 99.5);  m = max(abs([lo hi]));  if q == 1; lo = -m; hi = m; end
        patch(ax, [0 hi hi 0], [0 0 hi hi], [1 0.86 0.86], 'EdgeColor', 'none');          % product > 0
        patch(ax, [lo 0 0 lo], [lo lo 0 0], [1 0.86 0.86], 'EdgeColor', 'none');
        patch(ax, [lo 0 0 lo], [0 0 hi hi], [0.92 0.92 0.92], 'EdgeColor', 'none');      % product < 0
        patch(ax, [0 hi hi 0], [lo lo 0 0], [0.92 0.92 0.92], 'EdgeColor', 'none');
        plot(ax, x(1:end-1), x(2:end), '.', 'Color', c.E, 'MarkerSize', 7);
        xline(ax, 0, '-', 'Color', c.G);  yline(ax, 0, '-', 'Color', c.G);
        axis(ax, 'square');  xlim(ax, [lo hi]);  ylim(ax, [lo hi]);  title(ax, NM{q}, 'FontSize', c.FS - 2);
        xlabel(ax, 'x[k] - \mu');  if q == 1; ylabel(ax, 'x[k+1] - \mu'); end
        rp = local_acf(SP{q}, 1);
        text(ax, 0.05, 0.93, sprintf('\\rho(1) = %.2f', rp(2)), 'Units', 'normalized', 'FontSize', c.FS, 'FontWeight', 'bold');
        set(ax, 'Layer', 'top');  local_fmt(ax, c);
        fprintf('q2: %-26s rho(1) %.2f, share of points with positive product %.0f%%\n', NM{q}, rp(2), 100 * mean(x(1:end-1) .* x(2:end) > 0));
    end
    local_save(f, fd, 'fig_rs_q2_quadrant');
end

function local_q3(D, fd)                             % estimator belief r_2/N vs actual variance of the mean of N, and the formula
    c = C();  Ns = [1 2 5 10 20 50 100 200];  X = {D.am(D.W, :), D.u(D.W, :)};  TT = {'a_{cov} = 0.05', 'a_{cov} = 1'};
    f = figure('Position', [40 40 1700 650], 'Color', 'w', 'Visible', 'off');  tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for q = 1:2
        x = X{q};  v = var(x(:));  rho = local_acf_u(x, max(Ns) - 1);  act = v * local_blockvar(x, Ns);
        fm = zeros(size(Ns));
        for i = 1:numel(Ns); n = Ns(i); tau = (1:n - 1)'; fm(i) = v / n * (1 + 2 * sum((1 - tau / n) .* rho(2:n))); end
        ax = nexttile(tl);
        h1 = loglog(ax, Ns, v ./ Ns, '--', 'Color', c.G, 'LineWidth', 2.5, 'DisplayName', 'estimator: r_2 / N');  hold(ax, 'on');
        h2 = loglog(ax, Ns, fm, '-', 'Color', c.T, 'LineWidth', 3, 'DisplayName', '(\sigma^2/N)[1 + 2\Sigma(1-\tau/N)\rho(\tau)]');
        h3 = loglog(ax, Ns, act, 'o', 'Color', c.E, 'MarkerFaceColor', c.E, 'MarkerSize', 8, 'DisplayName', 'actual (200 seeds)');
        title(ax, TT{q}, 'FontSize', c.FS - 2);  xlabel(ax, 'N (readings)');  if q == 1; ylabel(ax, 'variance of the mean of N readings'); end
        ylim(ax, [5e-4 5]);  xlim(ax, [1 200]);
        legend(ax, [h1 h2 h3], 'Location', 'southwest', 'FontSize', 13);  local_fmt(ax, c);
        fprintf('q3: %-16s sigma^2 %.3f | N = 100: estimator %.4f, formula %.4f, actual %.4f, ratio %.1f\n', TT{q}, v, v / 100, fm(Ns == 100), act(Ns == 100), act(Ns == 100) / (v / 100));
    end
    local_save(f, fd, 'fig_rs_q3_belief');
end

function local_q4(~, fd)                             % epsilon scan: y_2 = a_m vs whitened y_2 (first-order estimator, cached runs)
    c = C();  here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    dd = fullfile(root, 'test_results', 'temp_5state_noap');  EE = {{'1e-4', '1e-5'}, {'1e-5'}};   % a_m: eps 1e-3 diverges (printed, not drawn)
    FM = {'mc_s4_plain_eps%s_tri10_plane_s1.mat', 'mc_s4_plain_y2_whiten_eps%s_tri10_plane_s1.mat'};
    TT = {'a_{cov} = 0.05', 'a_{cov} = 1'};   % 10-13 user: name the two cases by a_cov only
    CE = containers.Map({'1e-5', '1e-4'}, {c.E, [0.62 0.75 0.98]});
    f = figure('Position', [40 40 1800 700], 'Color', 'w', 'Visible', 'off');  tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for q = 1:2
        ax = nexttile(tl);  hold(ax, 'on');  hh = gobjects(0);
        E = EE{q};
        for i = 1:numel(E)
            r = load(fullfile(dd, sprintf(FM{q}, E{i})));  e = r.a_true - r.a_hat;
            hh(end + 1) = plot(ax, r.t, e, '-', 'Color', CE(E{i}), 'LineWidth', 1.6, 'DisplayName', ['\epsilon = ' E{i}]); %#ok<AGROW>
            w = r.t >= 1;  fprintf('q4: %-28s eps %-5s |e| mean (t >= 1 s) %.3f, max %.3f\n', TT{q}, E{i}, mean(abs(e(w))), max(abs(e(w))));
        end
        yline(ax, 0, '-', 'Color', c.T, 'LineWidth', 3);  uistack(hh, 'top');
        ylim(ax, [-0.6 0.6]);  xlim(ax, [0 r.t(end)]);  xlabel(ax, 'Time (sec)');  if q == 1; ylabel(ax, ['e_{' AB '}']); end
        title(ax, TT{q}, 'FontSize', c.FS - 2);
        legend(ax, hh, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 14);  local_fmt(ax, c);
    end
    local_save(f, fd, 'fig_rs_q4_eps');
end

function rho = local_acf_u(X, L)                     % the report formula: (1/(n-tau)) sum / (1/n) sum, pooled mean, seed-averaged
    mu = mean(X(:));  n = size(X, 1);  rho = zeros(L + 1, 1);
    for j = 1:size(X, 2)
        x = X(:, j) - mu;  c = xcorr(x, L);  c = c(L + 1:end) ./ (n - (0:L)');  rho = rho + c / mean(x .^ 2) / size(X, 2);
    end
end
