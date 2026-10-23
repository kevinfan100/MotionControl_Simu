% STATUS: SCRATCH -- 2026-10-11, primer "how to look at a random signal" on the A1 (06-13 gain_compare, 6-state, raw y2) data:
%   three signals of the z axis in the far-field hold (0.05-0.5 s, stationary), 50 seeds of runs.B (law uses a-hat):
%   (1) thermal kick a*f_T [um], (2) tracking-error residual dx_r [um], (3) dx_r^2 (single-step reading), (4) a_m / a_nom (EWMA).
%   Columns: time series (seed 1) | histogram with a Gaussian of the same mean/sd | autocorrelation rho(tau), 0-60 steps (seed-average).
%   Console: mean, sd, skewness, rho(1), rho(5), N_eff/N = 1/(1 + 2 sum rho).
function plot_random_signal_primer()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    S = load(fullfile(root, 'test_results', 'gain_compare', 'f1Hz', 'runs.mat'));  nz = S.runs.B.noisy;
    t = nz(1).simOut.tout(2:end);  an = 0.01471;  w = t > 0.05 & t < 0.5;  ns = 50;  L = 60;
    X = cell(1, 4);  for q = 1:4; X{q} = []; end
    for s = 1:ns
        o = nz(s).simOut;  r = o.diag.dx_r(2:end, 3);
        sig = {o.a_true_out(2:end, 3) .* o.F_th_out(2:end, 3), r, r.^2, o.diag.a_xm(2:end, 3) / an};
        for q = 1:4; X{q} = [X{q}, sig{q}(w)]; end
    end
    NM = {'thermal kick a f_T  [\mum]', 'residual \deltax_r  [\mum]', '\deltax_r^2  (single-step reading)', 'a_m / a_{nom}  (EWMA)'};
    cE = [0 0.25 0.85];  cT = [0.85 0 0];  FS = 15;  tt = t(w) - t(find(w, 1));
    f = figure('Position', [40 40 1800 1500], 'Color', 'w', 'Visible', 'off');  tl = tiledlayout(4, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    for q = 1:4
        x = X{q};  rho = zeros(L + 1, 1);
        for j = 1:size(x, 2); c = xcorr(x(:, j) - mean(x(:, j)), L, 'coeff'); rho = rho + c(L + 1:end) / size(x, 2); end
        ax = nexttile(tl);  plot(ax, tt, x(:, 1), '-', 'Color', cE, 'LineWidth', 0.8);  ylabel(ax, NM{q});  xlim(ax, [0 tt(end)]);
        if q == 4; xlabel(ax, 'time in the hold (s)'); end
        ax2 = nexttile(tl);  hold(ax2, 'on');  histogram(ax2, x(:), 80, 'Normalization', 'pdf', 'FaceColor', cE, 'EdgeColor', 'none');
        xs = linspace(min(x(:)), max(x(:)), 200);  plot(ax2, xs, normpdf(xs, mean(x(:)), std(x(:))), '-', 'Color', cT, 'LineWidth', 2.5);
        if q == 1; title(ax2, 'histogram vs Gaussian (red)'); end
        ax3 = nexttile(tl);  hold(ax3, 'on');  stem(ax3, 0:L, rho, 'filled', 'Color', cE, 'MarkerSize', 3);  yline(ax3, 0, '-', 'Color', [0.5 0.5 0.5]);
        ylim(ax3, [-0.3 1.05]);  if q == 1; title(ax3, 'autocorrelation \rho(\tau)'); end;  if q == 4; xlabel(ax3, 'lag \tau (steps)'); end
        for a = [ax ax2 ax3]; set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.6);  box(a, 'on'); end
        neff = 1 / (1 + 2 * sum(rho(2:end)));
        fprintf('%-36s mean %+.3e | sd %.3e | skew %+.2f | rho1 %.3f | rho5 %.3f | rho20 %.3f | N_eff/N %.3f\n', NM{q}, mean(x(:)), std(x(:)), ...
                skewness(x(:)), rho(2), rho(6), rho(21), neff);
    end
    out = fullfile(root, 'test_results', 'figs_unused', 'fig_random_signal_primer_A1.png');  exportgraphics(f, out, 'Resolution', 120);  fprintf('figure: %s\n', out);
end
