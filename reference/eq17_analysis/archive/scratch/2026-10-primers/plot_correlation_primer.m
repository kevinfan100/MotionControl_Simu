% STATUS: SCRATCH -- 2026-10-11, primer on time correlation (user: explain correlation before moving to a_m and the KF).
%   Same A1 data as plot_random_signal_primer.m (06-13 gain_compare, z, far-field hold 0.05-0.5 s, 50 seeds of runs.B).
%   Signals: thermal kick a f_T (white), dx_r^2 single-step reading, a_m / a_nom (EWMA of the reading).
%   Row 1: lag plot x[k] vs x[k+1] (seed 1). Row 2: variance of the average of N consecutive samples, divided by the
%   single-sample variance, vs N (log-log), with the independent-sample line 1/N (red).
function plot_correlation_primer()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    S = load(fullfile(root, 'test_results', 'gain_compare', 'f1Hz', 'runs.mat'));  nz = S.runs.B.noisy;
    t = nz(1).simOut.tout(2:end);  an = 0.01471;  w = t > 0.05 & t < 0.5;  ns = 50;
    X = cell(1, 3);  for q = 1:3; X{q} = []; end
    for s = 1:ns
        o = nz(s).simOut;  r = o.diag.dx_r(2:end, 3);
        sig = {o.a_true_out(2:end, 3) .* o.F_th_out(2:end, 3), r.^2, o.diag.a_xm(2:end, 3) / an};
        for q = 1:3; X{q} = [X{q}, sig{q}(w)]; end
    end
    NM = {'thermal kick (white)', '\deltax_r^2 single-step reading', 'a_m / a_{nom} (EWMA)'};
    Ns = [1 2 5 10 20 50 100 200];  cE = [0 0.25 0.85];  cT = [0.85 0 0];  FS = 15;
    f = figure('Position', [40 40 1800 1000], 'Color', 'w', 'Visible', 'off');  tl = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    for q = 1:3
        x = X{q};  v1 = var(x(:));  ratio = zeros(size(Ns));
        for i = 1:numel(Ns)
            n = Ns(i);  m = floor(size(x, 1) / n);  B = reshape(x(1:m * n, :), n, m, size(x, 2));  bm = squeeze(mean(B, 1));
            ratio(i) = var(bm(:)) / v1;
        end
        ax = nexttile(tl, q);  plot(ax, x(1:end-1, 1), x(2:end, 1), '.', 'Color', cE, 'MarkerSize', 6);  axis(ax, 'square');
        title(ax, NM{q});  xlabel(ax, 'x[k]');  if q == 1; ylabel(ax, 'x[k+1]'); end
        c = corr(x(1:end-1, 1), x(2:end, 1));  text(ax, 0.05, 0.92, sprintf('corr = %.2f', c), 'Units', 'normalized', 'FontSize', FS, 'FontWeight', 'bold');
        ax2 = nexttile(tl, 3 + q);  loglog(ax2, Ns, ratio, 'o-', 'Color', cE, 'LineWidth', 2, 'MarkerFaceColor', cE);  hold(ax2, 'on');
        loglog(ax2, Ns, 1 ./ Ns, '-', 'Color', cT, 'LineWidth', 2.5);  ylim(ax2, [3e-3 1.5]);  xlim(ax2, [1 200]);
        xlabel(ax2, 'N (consecutive samples averaged)');  if q == 1; ylabel(ax2, 'var(average of N) / var(one)'); end
        if q == 1; legend(ax2, {'measured', 'if independent: 1/N'}, 'Location', 'southwest', 'FontSize', FS - 2); end
        for a = [ax ax2]; set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.6);  box(a, 'on'); end
        fprintf('%-34s lag-1 corr %.2f | var ratio N=10 %.3f (1/N 0.100), N=100 %.3f (1/N 0.010) -> worth %.0f and %.0f independent samples\n', ...
                NM{q}, c, ratio(Ns == 10), ratio(Ns == 100), 1 / ratio(Ns == 10), 1 / ratio(Ns == 100));
    end
    out = fullfile(root, 'test_results', 'figs_unused', 'fig_correlation_primer_A1.png');  exportgraphics(f, out, 'Resolution', 120);  fprintf('figure: %s\n', out);
end
