% STATUS: SCRATCH -- 2026-10-22 (user: cloud figure not intuitive; look closer to the wall). Wait-only fixed points tm<gap>_A_s<seed> (20 deg
%   plane, probe held still). Row 1: spread of the tracking error along each direction in the x-z plane, std(n_phi' delta p) / mean, against
%   the direction phi (tilt from +z toward +x), from the first 2 s (light) and the whole 20 s (dark) of seed 1, one panel per gap; the dip
%   = the hard direction = the wall normal (red line at 20 deg). Row 2: tilt of the dip (direction of least spread, data only) from [0, t]
%   against time, 3 seeds, red = true. Console: dip depth (1 - min/mean) and the time after which the data-only tilt stays within 5 deg.
%   -> test_results/temp_3d_wsonly/fig_t2_spread.png | run: plot_t2_spread([50 15 10 5], 1:3)
function plot_t2_spread(GAP, SD)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cL = [0.6 0.7 0.95];  FS = 17;  LFS = 13;  DG = char(176);  MU = char(181);  AB = char(257);
    phi = -90:1:90;  nc = numel(GAP);
    f = figure('Position', [40 40 560 * nc 1000], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(2, nc, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ig = 1:nc
        ax1 = nexttile(tl, ig);  hold on;  box on;  ax2 = nexttile(tl, nc + ig);  hold on;  box on;
        for s = SD
            S = load(fullfile(od, sprintf('temp_t2_tm%d_A_s%d.mat', GAP(ig), s)));  r = S.R.runs{1};  k = 1:r.kend;  ty = S.R.opts.theta_y;  t = r.t(k);
            d = (r.pd(k, [1 3]) - r.p(k, [1 3])) * 1e3;  d = d - mean(d);
            n = numel(k);  c11 = cumsum(d(:, 1).^2) ./ (1:n)';  c33 = cumsum(d(:, 2).^2) ./ (1:n)';  c13 = cumsum(d(:, 1) .* d(:, 2)) ./ (1:n)';
            tilt = -0.5 * atan2d(2 * c13, c11 - c33);                   % direction of least spread (tilt from +z toward +x)
            m = t >= 0.05;
            if s == SD(1)
                for win = [2 t(end)]
                    j = find(t <= win, 1, 'last');  C = [c11(j) c13(j); c13(j) c33(j)];
                    sp = arrayfun(@(a) sqrt([sind(a) cosd(a)] * C * [sind(a); cosd(a)]), phi);  sp = sp / mean(sp);
                    if win == 2
                        g1 = plot(ax1, phi, sp, '-', 'Color', cL, 'LineWidth', 2, 'DisplayName', 'first 2 s');
                    else
                        g2 = plot(ax1, phi, sp, '-', 'Color', cE, 'LineWidth', 3, 'DisplayName', 'whole 20 s');
                        fprintf('%2d um: a_N/a_T %.3f | dip depth (1 - min/mean) %.2f %% over 20 s', GAP(ig), mean(r.aN_true(k)) / mean(r.aT_true(k)), 100 * (1 - min(sp)));
                    end
                end
                h1 = plot(ax2, t(m), tilt(m), '-', 'Color', cE, 'LineWidth', 2, 'DisplayName', 'data only, seed 1');
            else
                plot(ax2, t(m), tilt(m), '-', 'Color', cL, 'LineWidth', 1, 'HandleVisibility', 'off');
            end
            e = abs(tilt - ty);  tail = flipud(cummax(flipud(e)));  i5 = find(tail < 5 & t > 0.05, 1);
            if isempty(i5); fprintf(' | seed %d: never within 5 deg', s); else; fprintf(' | seed %d: within 5 deg from %.1f s', s, t(i5)); end
        end
        fprintf('\n');
        xline(ax1, ty, '-', 'Color', cT, 'LineWidth', 2.5, 'HandleVisibility', 'off');
        yl = 0.06;  if GAP(ig) <= 5; yl = 0.2; end;  ylim(ax1, 1 + yl * [-1 1]);  xlim(ax1, [-90 90]);   % 5 um: wider (dip 8.6 %)  xticks(ax1, -90:45:90);
        xlabel(ax1, ['direction  [' DG ' from z]'], 'FontSize', FS, 'FontWeight', 'bold');
        if ig == 1; ylabel(ax1, 'spread / mean', 'FontSize', FS, 'FontWeight', 'bold'); end
        text(ax1, 0.04, 0.92, sprintf('%d %sm   %s_N/%s_T = %.2f', GAP(ig), MU, AB, AB, mean(r.aN_true(k)) / mean(r.aT_true(k))), 'Units', 'normalized', 'FontSize', FS - 3, 'FontWeight', 'bold');
        legend(ax1, [g1 g2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        plot(ax2, [0 t(end)], [ty ty], '-', 'Color', cT, 'LineWidth', 2.5, 'DisplayName', 'true wall normal');
        ylim(ax2, [-90 90]);  yticks(ax2, -90:30:90);  xlim(ax2, [0 t(end)]);
        xlabel(ax2, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        if ig == 1; ylabel(ax2, ['least-spread direction  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold'); end
        legend(ax2, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        set([ax1 ax2], 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
    end
    out = fullfile(od, 'fig_t2_spread.png');
    exportgraphics(f, out, 'Resolution', 110);  close(f);  fprintf('figure: %s\n', out);
end
