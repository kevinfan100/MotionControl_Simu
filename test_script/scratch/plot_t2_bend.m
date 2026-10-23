% STATUS: SCRATCH -- 2026-10-20 (user: bend wall, test q_theta = P_fix^2 I[k] first, compare Q = eps R later). Columns = arms (tags,
%   one seed), shared y axes. Rows: theta_y, theta_x (true red, estimate blue / green, +- sqrt P band) | h (blue) with h* (red) | a_N
%   (true red, estimate blue, a_d black dashed). Time window opts t_win (default the scan, 6.5 s to the end).
%   -> test_results/temp_3d_wsonly/fig_t2_bend_<outtag>.png | run: plot_t2_bend({'bend_pf2','bend_pf5','bend_pf10'}, {'2','5','10'}, 'pfix')
function plot_t2_bend(TG, LB, outtag, t_win)
    if nargin < 4; t_win = [6.5 Inf]; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cGr = [0 0.55 0.2];  cB = [0.75 0.85 1];  FS = 20;  LFS = 15;
    AB = char(257);  HT = char(770);  TH = char(952);  DG = char(176);
    nc = numel(TG);
    f = figure('Position', [40 40 800 * nc 1400], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(3, nc, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:nc
        S = load(fullfile(od, ['temp_t2_' TG{ic} '.mat']));  r = S.R.runs{1};  k = 1:r.kend;  t = r.t(k);
        tw = [t_win(1), min(t_win(2), r.t(end))];
        ax = nexttile(tl, ic);  hold on;  box on;
        fill([t; flipud(t)], [r.thy(k) + r.sP(k, 6); flipud(r.thy(k) - r.sP(k, 6))], cB, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        fill([t; flipud(t)], [r.thx(k) + r.sP(k, 5); flipud(r.thx(k) - r.sP(k, 5))], cB, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        h1 = plot(t, r.thy_true(k), '-', 'Color', cT, 'LineWidth', 2.5, 'DisplayName', [TH '_y,  ' TH '_x']);
        plot(t, r.thx_true(k), '-', 'Color', cT, 'LineWidth', 2.5, 'HandleVisibility', 'off');
        h2 = plot(t, r.thy(k), '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [TH HT '_y']);
        h3 = plot(t, r.thx(k), '-', 'Color', cGr, 'LineWidth', 1.8, 'DisplayName', [TH HT '_x']);
        ylim([-25 25]);  xlim(tw);
        legend([h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        lb = LB{ic};  if all(isstrprop(strrep(lb, '.', ''), 'digit')); lb = ['\surdP_{fix} = ' lb DG]; end   % plain labels pass through (10-21)
        text(0.97, 0.9, lb, 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', FS, 'FontWeight', 'bold');
        if ic == 1; ylabel([TH '  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold'); end
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);
        ax = nexttile(tl, nc + ic);  hold on;  box on;
        h1 = plot(t, r.h_star + 0 * t, '-', 'Color', cT, 'LineWidth', 2.5, 'DisplayName', 'h*');
        h2 = plot(t, r.h(k), '-', 'Color', cE, 'LineWidth', 1.5, 'DisplayName', 'h');
        ylim([0.9 2.6]);  xlim(tw);
        legend([h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        if ic == 1; ylabel('h  [R]', 'FontSize', FS, 'FontWeight', 'bold'); end
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);
        ax = nexttile(tl, 2 * nc + ic);  hold on;  box on;
        h1 = plot(t, r.aN_true(k), '-', 'Color', cT, 'LineWidth', 2.5, 'DisplayName', [AB '_N']);
        h2 = plot(t, r.aN(k), '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HT '_N']);
        h3 = plot(t, S.R.opts.a_d + 0 * t, '--', 'Color', [0 0 0], 'LineWidth', 2, 'DisplayName', sprintf('%s_d = %.1f', AB, S.R.opts.a_d));
        ylim([0 0.3]);  xlim(tw);
        legend([h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        if ic == 1; ylabel(AB + "_N", 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);
        % console: scan-window numbers
        ks = t >= S.R.opts.t_scan;  eth = [r.thy(k) - r.thy_true(k), r.thx(k) - r.thx_true(k)];
        fprintf('%s: contact %d | scan: max|e_thy| %.2f, max|e_thx| %.2f deg; last 1 s sd e_thy %.2f, e_thx %.2f deg; h - h* mean %+.3f [%+.3f %+.3f]; min h %.3f\n', ...
                TG{ic}, r.contact, max(abs(eth(ks, 1))), max(abs(eth(ks, 2))), std(eth(t >= t(end) - 1, 1)), std(eth(t >= t(end) - 1, 2)), ...
                mean(r.h(ks) - r.h_star), min(r.h(ks) - r.h_star), max(r.h(ks) - r.h_star), min(r.h(k)));
    end
    out = fullfile(od, sprintf('fig_t2_bend_%s.png', outtag));
    exportgraphics(f, out, 'Resolution', 150);  close(f);  fprintf('figure: %s\n', out);
end
