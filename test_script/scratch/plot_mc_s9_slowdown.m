% STATUS: SCRATCH -- 2026-10-13, figure for probe_mc_s9_slowdown: plane, seed 1, columns tau_a 0.1 s | 0.3 s.
%   Row 1: a-bar_z true (red), estimate (blue), designed a_d + (a[0]-a_d) lambda_a^k (black dashed). Row 2: kappa = a'/a_hat' and
%   g = (a_d - a_hat)/(a_d - a) (both 20-step moving mean); their product = fraction of the designed rate actually realized.
function plot_mc_s9_slowdown()
    here = fileparts(mfilename('fullpath'));  od = fullfile(fileparts(fileparts(here)), 'test_results', 'temp_1009_ws');
    TM = [100 300];  cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0.95 0.55 0.1];  FS = 20;  LFS = 16;  AB = char(257);  HAT = char(770);
    f = figure('Position', [40 40 1900 1100], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for j = 1:2
        S = load(fullfile(od, sprintf('gainreg_plane_tau%03d.mat', TM(j))));  t = S.t;  k = (0:numel(t) - 1)';
        ad = S.a_d + (S.a_true(2) - S.a_d) * S.lambda_a .^ k;
        a1 = nexttile(tl, j);  hold on; box on;
        plot(a1, t, S.a_true, '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', [AB '_z']);
        plot(a1, t, S.a_hat, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HAT '_z']);
        plot(a1, t, ad, '--', 'Color', 'k', 'LineWidth', 2, 'DisplayName', ['designed, ' char(964) '_a = ' sprintf('%.1f s', S.tau_a)]);
        ylim(a1, [0 1]);  xlim(a1, [0 7 * S.tau_a]);  legend(a1, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        a2 = nexttile(tl, 2 + j);  hold on; box on;
        kap = movmean(S.ap_true ./ S.ap_hat, 20);  g = movmean(S.a_d - S.a_hat, 20) ./ movmean(S.a_d - S.a_true, 20);
        plot(a2, t, kap, '-', 'Color', cE, 'LineWidth', 2.2, 'DisplayName', [char(954) ' = ' AB ''' / ' AB HAT '''']);
        plot(a2, t, g, '-', 'Color', cG, 'LineWidth', 2.2, 'DisplayName', ['g = (' AB '_d - ' AB HAT ') / (' AB '_d - ' AB ')']);
        yline(a2, 1, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.5, 'HandleVisibility', 'off');
        ylim(a2, [0 1.4]);  xlim(a2, [0 7 * S.tau_a]);  xlabel(a2, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        legend(a2, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        for a = [a1 a2]; set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);  grid(a, 'off'); end
    end
    out = fullfile(od, 'fig_mc_s9_slowdown.png');  exportgraphics(f, out, 'Resolution', 130);  fprintf('figure: %s\n', out);
end
