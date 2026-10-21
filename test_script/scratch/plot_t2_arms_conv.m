% STATUS: SCRATCH -- 2026-10-21 (user: dither while holding at 15 um, no u, v control). Angle convergence of several arms side by side,
%   same axes: columns = arms (run prefix, seeds <pre>_s<seed>); rows = angle errors theta-hat_y - theta_y (blue), theta-hat_x - theta_x
%   (green) of every seed | sqrt P_theta_y of every seed, over 0 .. t_end. Console: per arm and seed, first time the error stays within
%   5 and 2 deg (both angles) before t_end, and sqrt P_theta_y at t_end.
%   -> test_results/temp_3d_wsonly/fig_t2_conv_<outtag>.png
%   | run: plot_t2_arms_conv({'h15holdnouv','dith03','dith1'}, {'no dither','dither 0.3 um, 2 Hz','dither 1 um, 2 Hz'}, 1:3, 15, 'dither')
function plot_t2_arms_conv(PRE, LB, SD, t_end, outtag)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cGr = [0 0.55 0.2];  FS = 18;  LFS = 14;  TH = char(952);  HT = char(770);  DG = char(176);
    nc = numel(PRE);
    f = figure('Position', [40 40 700 * nc 1000], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(2, nc, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:nc
        ax1 = nexttile(tl, ic);  hold on;  box on;  ax2 = nexttile(tl, nc + ic);  hold on;  box on;
        for s = SD
            S = load(fullfile(od, sprintf('temp_t2_%s_s%d.mat', PRE{ic}, s)));  r = S.R.runs{1};  ty = S.R.opts.theta_y;
            k = find(r.t <= t_end);  k = k(k <= r.kend);  t = r.t(k);  ey = r.thy(k) - ty;  ex = r.thx(k);
            h1 = plot(ax1, t, ey, '-', 'Color', cE, 'LineWidth', 1.2, 'DisplayName', [TH HT '_y - ' TH '_y']);
            h2 = plot(ax1, t, ex, '-', 'Color', cGr, 'LineWidth', 1.2, 'DisplayName', [TH HT '_x - ' TH '_x']);
            g1 = plot(ax2, t, r.sP(k, 6), '-', 'Color', cE, 'LineWidth', 1.5, 'DisplayName', ['\surdP_{' TH 'y}']);
            e = max(abs([ey ex]), [], 2);  tail = flipud(cummax(flipud(e)));
            i5 = find(tail < 5, 1);  i2 = find(tail < 2, 1);
            fprintf('%-14s seed %d: within 5 deg from %5s s, within 2 deg from %5s s | sqrtP_thy at %g s %.2f deg\n', PRE{ic}, s, ...
                    local_t(t, i5), local_t(t, i2), t_end, r.sP(k(end), 6));
        end
        plot(ax1, [0 t_end], [0 0], '-', 'Color', cT, 'LineWidth', 2.5, 'HandleVisibility', 'off');
        ylim(ax1, [-25 15]);  xlim(ax1, [0 t_end]);  ylim(ax2, [0 11]);  xlim(ax2, [0 t_end]);
        text(ax1, 0.97, 0.9, LB{ic}, 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', FS, 'FontWeight', 'bold');
        legend(ax1, [h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        legend(ax2, g1, 'Location', 'northoutside', 'FontSize', LFS, 'FontWeight', 'bold');
        if ic == 1; ylabel(ax1, ['angle error  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold'); ylabel(ax2, ['\surdP  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel(ax2, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        set([ax1 ax2], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);
    end
    out = fullfile(od, sprintf('fig_t2_conv_%s.png', outtag));
    exportgraphics(f, out, 'Resolution', 110);  close(f);  fprintf('figure: %s\n', out);
end

function s = local_t(t, i)
    if isempty(i); s = '-'; else; s = sprintf('%.1f', t(i)); end
end
