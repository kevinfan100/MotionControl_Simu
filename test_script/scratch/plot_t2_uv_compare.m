% STATUS: SCRATCH -- 2026-10-21 (user: tau_u one fast, one slow, prior 10 deg). Columns = arms of run_t2_3d_reg (same seed), shared y axes.
%   Rows: theta_y, theta_x (true red, estimates blue / green) | u, v of the probe relative to its start in the TRUE local coordinate [um]
%   (u blue, v green). Console: per arm, max |u|, |v|, rms of the per-step change of u, v (smoothness), end u, v.
%   -> test_results/temp_3d_wsonly/fig_t2_uv_compare_<outtag>.png
%   | run: plot_t2_uv_compare({'ng_uvnl_p10a','tu0p01_a','tu0p2_a'}, {'\tau_u = T_s','\tau_u = 10 ms','\tau_u = 0.2 s'}, 1, 'tau_u')
function plot_t2_uv_compare(TG, LB, seed, outtag)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cGr = [0 0.55 0.2];  FS = 18;  LFS = 14;  TH = char(952);  HT = char(770);  DG = char(176);  MU = char(181);
    nc = numel(TG);
    f = figure('Position', [40 40 760 * nc 1000], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(2, nc, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:nc
        S = load(fullfile(od, ['temp_t2_' TG{ic} '.mat']));  sd = cellfun(@(q) q.seed, S.R.runs);  r = S.R.runs{find(sd == seed, 1)};
        ty = S.R.opts.theta_y;  k = 1:r.kend;  t = r.t(k);
        V = [cosd(ty) 0 sind(ty); 0 1 0; -sind(ty) 0 cosd(ty)];  UV = (V(:, 1:2)' * (r.p(k, :) - r.p(1, :))')';
        ax = nexttile(tl, ic);  hold on;  box on;
        h1 = plot(t, ty + 0 * t, '-', 'Color', cT, 'LineWidth', 3, 'DisplayName', [TH '_y,  ' TH '_x']);
        plot(t, 0 * t, '-', 'Color', cT, 'LineWidth', 3, 'HandleVisibility', 'off');
        h2 = plot(t, r.thy(k), '-', 'Color', cE, 'LineWidth', 1.5, 'DisplayName', [TH HT '_y']);
        h3 = plot(t, r.thx(k), '-', 'Color', cGr, 'LineWidth', 1.5, 'DisplayName', [TH HT '_x']);
        ylim([-30 45]);  xlim([0 t(end)]);  if ic == 1; ylabel([TH '  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        text(0.97, 0.9, LB{ic}, 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', FS, 'FontWeight', 'bold');
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);
        ax = nexttile(tl, nc + ic);  hold on;  box on;
        g1 = plot(t, UV(:, 1), '-', 'Color', cE, 'LineWidth', 1.5, 'DisplayName', 'u');
        g2 = plot(t, UV(:, 2), '-', 'Color', cGr, 'LineWidth', 1.5, 'DisplayName', 'v');
        ylim([-6 6]);  xlim([0 t(end)]);  if ic == 1; ylabel(['u, v  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        legend([g1 g2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);
        dUV = diff(UV);
        fprintf('%s seed %d: max|u| %.2f max|v| %.2f um | rms step du %.4f dv %.4f um | end u %+.2f v %+.2f\n', TG{ic}, seed, ...
                max(abs(UV(:, 1))), max(abs(UV(:, 2))), rms(dUV(:, 1)), rms(dUV(:, 2)), UV(end, 1), UV(end, 2));
    end
    out = fullfile(od, sprintf('fig_t2_uv_compare_%s.png', outtag));
    exportgraphics(f, out, 'Resolution', 110);  close(f);  fprintf('figure: %s\n', out);
end
