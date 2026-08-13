% STATUS: ACTIVE (scratch) | PURPOSE: Scenario-A figures + the journal's
%   third acceptance (no motion-frequency line in the residual): amplitude
%   spectrum of the x tracking error, arm N vs E98. House style.
%   | EXPIRES: with run_meng_ch4_sA.
function plot_meng_ch4_sA(out, out_dir)

    if nargin < 2 || isempty(out_dir)
        here = fileparts(mfilename('fullpath'));
        root = fileparts(fileparts(here));
        out_dir = fullfile(root, 'test_results', 'meng_ch4_s0');
    end
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end

    t = out.t;  Fs = 1600;
    rN = out.N{1};  rE = out.E98{1};
    C_TRUE = [0.85 0.10 0.10]; C_EST = [0.10 0.25 0.75]; C_MEAS = [0.55 0.75 0.95];
    dxN = out.pd(:,1) - rN.p_true(:,1);  dxE = out.pd(:,1) - rE.p_true(:,1);
    dzN = out.pd(:,3) - rN.p_true(:,3);  dzE = out.pd(:,3) - rE.p_true(:,3);

    f1 = figure('Position', [60 60 980 760], 'Color', 'w');
    tl = tiledlayout(f1, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax = nexttile(tl); hold(ax,'on'); box(ax,'on');
    plot(ax, t, 1e3*dxN, '-', 'Color', C_MEAS, 'LineWidth', 0.7);
    plot(ax, t, 1e3*dxE, '-', 'Color', C_EST,  'LineWidth', 0.7);
    ylabel(ax, '\deltax [nm]');
    legend(ax, {'arm N (\gamma_N fixed)', 'arm E98 (estimated)'}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal');

    ax = nexttile(tl); hold(ax,'on'); box(ax,'on');
    plot(ax, t, 1e3*dzN, '-', 'Color', C_MEAS, 'LineWidth', 0.7);
    plot(ax, t, 1e3*dzE, '-', 'Color', C_EST,  'LineWidth', 0.7);
    ylabel(ax, '\deltaz [nm]');

    ax = nexttile(tl); hold(ax,'on'); box(ax,'on');
    plot(ax, t, rE.a_true(:,3), '-', 'Color', C_TRUE, 'LineWidth', 1.3);
    plot(ax, t, rE.a_hat(:,3),  '-', 'Color', C_EST,  'LineWidth', 0.8);
    plot(ax, t, rN.a_hat(:,3),  ':', 'Color', C_MEAS, 'LineWidth', 1.0);
    ylabel(ax, 'a_z [\mum/pN]');
    legend(ax, {'a true', 'a hat E98', 'a hat N (frozen)'}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal');

    % second half of the run (near wall): amplitude spectrum of delta-x
    ax = nexttile(tl); hold(ax,'on'); box(ax,'on');
    seg = t > 5;  L = nnz(seg);  fr = (0:L-1) * Fs / L;
    AN = abs(fft(detrend(dxN(seg)))) / L * 2;
    AE = abs(fft(detrend(dxE(seg)))) / L * 2;
    plot(ax, fr, 1e3*AN, '-', 'Color', C_MEAS, 'LineWidth', 0.9);
    plot(ax, fr, 1e3*AE, '-', 'Color', C_EST,  'LineWidth', 0.9);
    xlim(ax, [0 5]);
    xlabel(ax, 'f [Hz]'); ylabel(ax, '|\deltax| [nm]');

    fp = fullfile(out_dir, 'meng_ch4_sA_fig5_counterpart.png');
    exportgraphics(f1, fp, 'Resolution', 150);
    fprintf('figure: %s\n', fp);

    % the journal's third acceptance, as numbers
    i1 = find(fr >= 0.95 & fr <= 1.05);
    bg = median(1e3*AE(fr > 1.5 & fr < 4.5));
    fprintf('1 Hz line in dx, t>5s: arm N %.2f nm | E98 %.2f nm | E98 background %.2f nm\n', ...
            max(1e3*AN(i1)), max(1e3*AE(i1)), bg);
end
