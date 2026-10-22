% STATUS: SCRATCH -- 2026-10-22 (user: "3 um、8 Hz、2 s 具體是怎麼樣和怎麼樣漸增"). The dither command of run_t2_3d_reg (opts.dither = [A f T_ramp]):
%   offset of the desired position in the estimated tangent plane, (u_d, v_d) = A(t) (cos 2 pi f t - 1, sin 2 pi f t), A(t) = A min(t / T_ramp, 1).
%   Left column: u_d(t), v_d(t) [um] with the envelope -2 A(t) / +-A(t) (first 1 s zoom | 0-4 s); right: the path in the (u_d, v_d) plane.
%   Console: per-step motion 2 pi A f T_s. -> test_results/temp_3d_wsonly/fig_dither_shape.png | run: plot_dither_shape(3, 8, 2, 4)
function plot_dither_shape(A, f, Tr, Th)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    Ts = 1 / 1600;  t = (0:Ts:Th)';  At = A * min(t / Tr, 1);
    ud = At .* (cos(2 * pi * f * t) - 1);  vd = At .* sin(2 * pi * f * t);
    cU = [0 0.25 0.85];  cV = [0 0.55 0.2];  cK = [0.4 0.4 0.4];  FS = 18;  LFS = 14;  MU = char(181);
    fg = figure('Position', [40 40 1900 1000], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for row = 1:2
        ax = nexttile(tl, 2 * row - 1);  hold on;  box on;
        h1 = plot(t, ud, '-', 'Color', cU, 'LineWidth', 1.6, 'DisplayName', 'u_d');
        h2 = plot(t, vd, '-', 'Color', cV, 'LineWidth', 1.6, 'DisplayName', 'v_d');
        h3 = plot(t, At, '--', 'Color', cK, 'LineWidth', 2, 'DisplayName', 'radius A(t)');
        plot(t, -At, '--', 'Color', cK, 'LineWidth', 2, 'HandleVisibility', 'off');
        plot(t, -2 * At, ':', 'Color', cK, 'LineWidth', 2, 'HandleVisibility', 'off');
        if row == 1; xlim([0 1]); else; xlim([0 Th]); xline(Tr, '-', 'ramp ends', 'Color', cK, 'LineWidth', 1.5, 'FontSize', LFS, 'FontWeight', 'bold', 'HandleVisibility', 'off'); end
        ylim([-2 * A - 0.5, A + 0.5]);  ylabel(['offset  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
        if row == 2; xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
        if row == 1; legend([h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold'); end
        set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
    end
    ax = nexttile(tl, 2, [2 1]);  hold on;  box on;
    m1 = t <= Tr;  m2 = t >= Th - 1 / f;
    plot(ud(m1), vd(m1), '-', 'Color', [0.6 0.7 0.95], 'LineWidth', 1, 'DisplayName', sprintf('0 - %g s (radius growing)', Tr));
    plot(ud(m2), vd(m2), '-', 'Color', cU, 'LineWidth', 3, 'DisplayName', sprintf('last turn (radius %g %sm)', A, MU));
    plot(0, 0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 10, 'DisplayName', 'start point');
    plot(-A, 0, 'k+', 'MarkerSize', 14, 'LineWidth', 2.5, 'DisplayName', 'centre of the last turn');
    axis equal;  xlim([-2 * A - 0.5, 0.8]);  ylim([-A - 0.5, A + 0.5]);
    xlabel(['u_d  [' MU 'm]  (along u-hat)'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(['v_d  [' MU 'm]  (along v-hat)'], 'FontSize', FS, 'FontWeight', 'bold');
    legend('Location', 'northoutside', 'FontSize', LFS, 'FontWeight', 'bold');
    set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
    fprintf('per-step motion at full radius: 2 pi A f T_s = %.4f um; one turn = %.3f s = %d steps; turns in %g s: %g\n', ...
            2 * pi * A * f * Ts, 1 / f, round(1 / f / Ts), Th, Th * f);
    out = fullfile(od, 'fig_dither_shape.png');
    exportgraphics(fg, out, 'Resolution', 110);  close(fg);  fprintf('figure: %s\n', out);
end
