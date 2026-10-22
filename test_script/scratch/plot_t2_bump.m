% STATUS: SCRATCH -- 2026-10-22 (user: scan the whole bump, show seed 2). Side view of one run of run_t2_3d_reg with opts.wall = 'bump'
%   (Gaussian bump H, sigma, centre (xc, yc); local tangent plane under the probe): wall profile in the x-z cut at the scan line's y
%   (grey), gap h* R measured along the local normal (grey dashed), probe path p (blue solid). Row 1 = whole run, row 2 = zoom on the
%   scan over the bump. Console: scan h - h* mean / range, lowest h, max angle errors. -> fig_t2_bump_xyz_<tag>.png
%   | run: plot_t2_bump('bump_s2')
function plot_t2_bump(tag)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');  addpath(genpath(fullfile(root, 'model')));  pc = physical_constants();  Rr = pc.R;
    S = load(fullfile(od, ['temp_t2_' tag '.mat']));  o = S.R.opts;  r = S.R.runs{1};  k = 1:r.kend;  p = r.p(k, :);  t = r.t(k);
    H = 3;  sg = 7;  xc = 25;  yc = -3.5;  if isfield(o, 'bump'); H = o.bump(1); sg = o.bump(2); xc = o.bump(3); yc = o.bump(4); end
    ks = find(t >= o.t_scan, 1);  ys = median(p(ks:end, 2));  z0 = 0;
    cW = [0.45 0.45 0.45];  cE = [0 0.25 0.85];  FS = 18;  LFS = 14;  MU = char(181);
    f = figure('Position', [40 40 2000 1400], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for row = 1:2
        ax = nexttile(tl);  hold on;  box on;
        if row == 1; xl = [-10 max(p(:, 1)) + 5]; else; xl = [xc - 22, xc + 22]; end
        xw = linspace(xl(1), xl(2), 800);  zb = H * exp(-((xw - xc).^2 + (ys - yc)^2) / (2 * sg^2));
        zx = -(xw - xc) / sg^2 .* zb;  zy = -(ys - yc) / sg^2 .* zb;  nn = sqrt(1 + zx.^2 + zy.^2);
        fill([xw fliplr(xw)], [zb, -3 + 0 * xw], [0.9 0.9 0.9], 'EdgeColor', 'none', 'HandleVisibility', 'off');
        h1 = plot(xw, zb, '-', 'Color', cW, 'LineWidth', 4, 'DisplayName', sprintf('wall at y = %.1f %sm (scan line)', ys, MU));
        hs = plot(xw - zx ./ nn * r.h_star * Rr, zb + 1 ./ nn * r.h_star * Rr, '--', 'Color', cW, 'LineWidth', 2, 'DisplayName', 'h* R');
        h3 = plot(p(:, 1), p(:, 3), '-', 'Color', cE, 'LineWidth', 2.2, 'DisplayName', 'p');
        plot(p(ks, 1), p(ks, 3), 's', 'Color', cE, 'MarkerFaceColor', cE, 'MarkerSize', 11, 'HandleVisibility', 'off');
        plot(p(end, 1), p(end, 3), '^', 'Color', cE, 'MarkerFaceColor', cE, 'MarkerSize', 11, 'HandleVisibility', 'off');
        if row == 1
            plot(p(1, 1), p(1, 3), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 10, 'HandleVisibility', 'off');
            text(p(1, 1) + 1, p(1, 3) + 1, 'start (t = 0)', 'FontSize', FS - 4, 'FontWeight', 'bold');
            text(p(ks, 1) + 1, p(ks, 3) + 3, sprintf('scan starts (t = %g s)', o.t_scan), 'FontSize', FS - 4, 'FontWeight', 'bold', 'Color', cE);
            ylim([-3 17]);
        else
            ylim([-1 7]);
        end
        axis equal;  xlim(xl);
        xlabel(['x  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(['z  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
        legend([h1 hs h3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);  grid off;
    end
    sc = ks:numel(k);
    fprintf('%s: contact %d | scan h - h* mean %+.3f [%+.3f %+.3f] R, lowest h %.3f | max |e_thy| %.1f |e_thx| %.1f deg | x range %.1f-%.1f um\n', tag, r.contact, ...
            mean(r.h(sc)) - r.h_star, min(r.h(sc)) - r.h_star, max(r.h(sc)) - r.h_star, min(r.h(sc)), max(abs(r.thy(sc) - r.thy_true(sc))), max(abs(r.thx(sc) - r.thx_true(sc))), p(ks, 1), p(end, 1));
    out = fullfile(od, sprintf('fig_t2_bump_xyz_%s.png', tag));
    exportgraphics(f, out, 'Resolution', 110);  close(f);  fprintf('figure: %s\n', out);
end
