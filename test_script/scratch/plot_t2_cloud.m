% STATUS: SCRATCH -- 2026-10-22 (user: "只等時間的情況 ... 直觀的數據來呈現"). Wait-only fixed-point runs tm<gap>_A_s<seed> (20 deg plane,
%   probe held still, no dither). Fig 1 (cloud): tracking error delta p = p_d - p of the whole hold in the x-z plane [nm], one panel per
%   gap, with the true wall normal (red) and the 2-sigma ellipse of the sample covariance (black); title-free, numbers in the console.
%   Fig 2 (data-only angle): the tilt of the short axis of the sample covariance of delta p over [0, t] (no estimator) vs time, next to the
%   filter's theta-hat_y, for every seed; red = true 20 deg.
%   -> test_results/temp_3d_wsonly/fig_t2_cloud.png, fig_t2_cloud_angle.png | run: plot_t2_cloud([50 15], 1:3)
function plot_t2_cloud(GAP, SD)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cL = [0.6 0.7 0.95];  cGr = [0 0.55 0.2];  FS = 18;  LFS = 14;  TH = char(952);  HT = char(770);  DG = char(176);  MU = char(181);
    f1 = figure('Position', [40 40 1500 760], 'Color', 'w', 'Visible', 'off');  t1 = tiledlayout(1, numel(GAP), 'TileSpacing', 'compact', 'Padding', 'compact');
    f2 = figure('Position', [40 40 1500 700], 'Color', 'w', 'Visible', 'off');  t2 = tiledlayout(1, numel(GAP), 'TileSpacing', 'compact', 'Padding', 'compact');
    for ig = 1:numel(GAP)
        for s = SD
            S = load(fullfile(od, sprintf('temp_t2_tm%d_A_s%d.mat', GAP(ig), s)));  r = S.R.runs{1};  k = 1:r.kend;  ty = S.R.opts.theta_y;
            d = (r.pd(k, [1 3]) - r.p(k, [1 3])) * 1e3;                 % x, z tracking error [nm]
            d = d - mean(d);  t = r.t(k);
            % running tilt of the short axis from the sample covariance over [0, t]
            c11 = cumsum(d(:, 1).^2) ./ (1:numel(k))';  c33 = cumsum(d(:, 2).^2) ./ (1:numel(k))';  c13 = cumsum(d(:, 1) .* d(:, 2)) ./ (1:numel(k))';
            ang = 0.5 * atan2d(2 * c13, c11 - c33);                     % long-axis angle from +x
            tilt = -ang;                                                 % short axis = normal; tilt from +z toward +x = minus the long-axis angle from +x
            m = t >= 0.05;
            nexttile(t2, ig);  hold on;  box on;
            if s == SD(1)
                g1 = plot(t(m), tilt(m), '-', 'Color', cE, 'LineWidth', 2, 'DisplayName', 'data only (cloud axis), seed 1');
                g2 = plot(t, r.thy(k), '-', 'Color', cGr, 'LineWidth', 1.5, 'DisplayName', [TH HT '_y (filter), seed 1']);
            else
                plot(t(m), tilt(m), '-', 'Color', cL, 'LineWidth', 1, 'HandleVisibility', 'off');
                plot(t, r.thy(k), '-', 'Color', [0.6 0.85 0.65], 'LineWidth', 1, 'HandleVisibility', 'off');
            end
            if s == SD(1)                                                % cloud figure: seed 1
                ax = nexttile(t1, ig);  hold on;  box on;
                plot(d(1:5:end, 1), d(1:5:end, 2), '.', 'Color', cL, 'MarkerSize', 3, 'HandleVisibility', 'off');
                C = cov(d);  [Ev, Dv] = eig(C);  th = linspace(0, 2 * pi, 200);
                el = 2 * Ev * sqrt(Dv) * [cos(th); sin(th)];
                h1 = plot(el(1, :), el(2, :), '-', 'Color', [0 0 0], 'LineWidth', 2.5, 'DisplayName', '2\sigma ellipse of the cloud');
                Lq = 1.1 * max(abs(el(:)));  nw = [sind(ty) cosd(ty)];
                h2 = plot([-1 1] * Lq * nw(1), [-1 1] * Lq * nw(2), '-', 'Color', cT, 'LineWidth', 2.5, 'DisplayName', 'true wall normal');
                axis equal;  lim = 1.25 * Lq;  xlim([-lim lim]);  ylim([-lim lim]);
                xlabel('\delta x  [nm]', 'FontSize', FS, 'FontWeight', 'bold');  if ig == 1; ylabel('\delta z  [nm]', 'FontSize', FS, 'FontWeight', 'bold'); end
                text(0.03, 0.93, sprintf('%d %sm,  %s_N / %s_T = %.3f', GAP(ig), MU, char(257), char(257), mean(r.aN_true(k)) / mean(r.aT_true(k))), 'Units', 'normalized', 'FontSize', FS - 2, 'FontWeight', 'bold');
                legend([h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
                set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
                ev = sort(diag(Dv));
                fprintf('%d um seed %d: a_N/a_T %.3f | cloud std short %.1f long %.1f nm (ratio %.3f) | cloud tilt at 20 s %.1f deg | filter %.1f deg\n', ...
                        GAP(ig), s, mean(r.aN_true(k)) / mean(r.aT_true(k)), sqrt(ev(1)), sqrt(ev(2)), sqrt(ev(1) / ev(2)), tilt(end), r.thy(k(end)));
            else
                fprintf('%d um seed %d: cloud tilt at 5 / 10 / 20 s %.1f / %.1f / %.1f deg | filter %.1f deg\n', GAP(ig), s, ...
                        tilt(find(t >= 5, 1)), tilt(find(t >= 10, 1)), tilt(end), r.thy(k(end)));
            end
        end
        ax = nexttile(t2, ig);
        plot(ax, [0 t(end)], [ty ty], '-', 'Color', cT, 'LineWidth', 2.5, 'DisplayName', ['true ' TH '_y']);
        ylim(ax, [-90 90]);  xlim(ax, [0 t(end)]);  yticks(ax, -90:30:90);
        xlabel(ax, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');  if ig == 1; ylabel(ax, ['tilt of the normal  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold'); end
        text(ax, 0.97, 0.08, sprintf('%d %sm', GAP(ig), MU), 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', FS, 'FontWeight', 'bold');
        legend(ax, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
    end
    exportgraphics(f1, fullfile(od, 'fig_t2_cloud.png'), 'Resolution', 110);  close(f1);
    exportgraphics(f2, fullfile(od, 'fig_t2_cloud_angle.png'), 'Resolution', 110);  close(f2);
    fprintf('figures: fig_t2_cloud.png, fig_t2_cloud_angle.png\n');
end
