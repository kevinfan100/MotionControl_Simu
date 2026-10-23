% STATUS: SUPERSEDED-BY test_script/scratch/plot_t2_thm_numden.m (closed 10-22: settling value from the measured sigma_T, sigma_N is noise at 50 um, not the readout the estimator uses) -- 2026-10-22 (user did not follow the y_7 = s + n figure twice -> walk the numbers, two panels, nm units).
%   Wait-only fixed points tmg<gap>_s1 (20 deg plane, seed 1). The estimator starts assuming a flat wall (theta-hat = 0), so its angle
%   readout is delta x * delta z (u = x, w = z). Blue: running average of delta x * delta z from t = 0 [nm^2]; red: the value it would
%   settle to = the tilt of the cloud, (sigma_T^2 - sigma_N^2) sin 20 cos 20 with sigma_T, sigma_N measured along / across the true wall;
%   grey band: +- 1 sd of the running average (sd of one product / sqrt(steps / correlation length)).
%   -> test_results/temp_3d_wsonly/fig_t2_thm_walk.png | run: plot_t2_thm_walk([50 5])
function plot_t2_thm_walk(GAP)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cS = [0.85 0.85 0.85];  FS = 18;  LFS = 14;  MU = char(181);  AB = char(257);
    nc = numel(GAP);
    f = figure('Position', [40 40 760 * nc 560], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(1, nc, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:nc
        S = load(fullfile(od, sprintf('temp_t2_tmg%d_s1.mat', GAP(ic))));  r = S.R.runs{1};  k = (1:r.kend)';  t = r.t(k);
        ty = S.R.opts.theta_y;
        d = (r.pd(k, [1 3]) - r.p(k, [1 3])) * 1e3;  d = d - mean(d);                  % delta x, delta z [nm]
        pr = d(:, 1) .* d(:, 2);                                                       % one readout per step [nm^2]
        uw = d * [cosd(ty) sind(ty); -sind(ty) cosd(ty)];                              % along / across the true wall (u, w)
        sT2 = mean(uw(:, 1).^2);  sN2 = mean(uw(:, 2).^2);
        s = -(sT2 - sN2) * sind(ty) * cosd(ty);                                        % value delta x * delta z settles to
        n = (1:numel(k))';  ra = cumsum(pr) ./ n;
        c = xcorr(pr - mean(pr), 50, 'coeff');  lc = max(1, find(c(51:end) < exp(-1), 1) - 1);
        band = std(pr) ./ sqrt(n / lc);
        ax = nexttile(tl, ic);  hold on;  box on;
        fill([t; flipud(t)], [s + band; flipud(s - band)], cS, 'EdgeColor', 'none', 'DisplayName', 'noise left after averaging');
        plot(t, ra, '-', 'Color', cE, 'LineWidth', 2, 'DisplayName', 'running average of  \delta x \cdot \delta z');
        plot([0 t(end)], [s s], '-', 'Color', cT, 'LineWidth', 2.5, 'DisplayName', 'value it settles to (tilt of the cloud)');
        plot([0 t(end)], [0 0], ':', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.5, 'HandleVisibility', 'off');
        yl = max(4 * abs(s), 8 * band(round(numel(k) / 2)));  ylim([-yl yl]);  xlim([0 t(end)]);
        text(0.97, 0.92, sprintf('%d %sm  (%s_N/%s_T = %.2f)', GAP(ic), MU, AB, AB, mean(r.aN_true(k) ./ r.aT_true(k))), 'Units', 'normalized', ...
             'HorizontalAlignment', 'right', 'FontSize', FS - 2, 'FontWeight', 'bold');
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');  if ic == 1; ylabel('[nm^2]', 'FontSize', FS, 'FontWeight', 'bold'); end
        if ic == 1; legend('Location', 'northoutside', 'NumColumns', 3, 'FontSize', LFS, 'FontWeight', 'bold'); end
        set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
        i1 = find(t >= 1, 1);  i20 = numel(k);  nreq = lc * (std(pr) / abs(s))^2;
        fprintf('%2d um: sigma_T %.1f nm, sigma_N %.1f nm | settles to %.1f nm^2 | one product: sd %.0f nm^2 | corr %d steps\n', ...
                GAP(ic), sqrt(sT2), sqrt(sN2), s, std(pr), lc);
        fprintf('       noise left after 1 s %.1f, after 20 s %.1f nm^2 | running avg at 1 s %.1f, at 20 s %.1f | steps for noise = signal %.0f (%.2f s)\n', ...
                band(i1), band(i20), ra(i1), ra(i20), nreq, nreq * (t(2) - t(1)));
    end
    out = fullfile(od, 'fig_t2_thm_walk.png');  exportgraphics(f, out, 'Resolution', 110);  close(f);  fprintf('figure: %s\n', out);
end
