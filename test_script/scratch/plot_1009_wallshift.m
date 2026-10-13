% STATUS: SCRATCH -- 2026-10-12, figures for run_1009_wallshift.m (c-only, wall-position error in the plant only).
%   set = 'neg' : columns DELTA = 0 | -2 | -4 ;  set = 'pos' : columns DELTA = 0 | +2 | +4 (0 on its own trajectory, reference).
%   Rows (motion_ctrl Section 6 style, seed 1): z_d and z [um] (gray dashed: the true contact when on the axis); a-bar_z true and estimate;
%   c-hat and (red, user 10-13) the wall's own c at the true gap g = h_true - DELTA, c(g) = 1 - g^2 a'_plane(g); own y axis per column,
%   dotted = the controller clamp [-3, 1]. (Before 10-13 the red was the estimator-view c(h_d) = 1 - h_d^2 a'_plant(h_d), res.c_need.)
%   delta z_m [um]. Console: 10-seed statistics of e_a = a_true - a_hat. set = 'law': the offline curves (no run).
function plot_1009_wallshift(set_name, seed)
    if nargin < 1; set_name = 'neg'; end
    if nargin < 2; seed = 1; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_1009_ws');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0.5 0.5 0.5];  FS = 22;  LFS = 17;  AXLW = 2.2;  R_UM = 2.25;
    AB = char(257);  HAT = char(770);  az = [AB '_z'];
    if strcmp(set_name, 'law'); local_law(fd, cT, cE, FS); return; end
    switch set_name
        case 'neg'; TG = {'p0', 'm2', 'm4'};
        case 'pos'; TG = {'p0', 'p2', 'p4'};
        case {'neg_best', 'pos_best'}                                         % per-case sqrt(P_fix) chosen by probe_1009_wallshift_pfix.m (reference arm)
            B = load(fullfile(od, 'wallshift_best.mat'));  DLw = [0 -2 -4];  if strcmp(set_name, 'pos_best'); DLw = [0 2 4]; end
            TG = arrayfun(@(d) B.tag{B.delta == d}, DLw, 'UniformOutput', false);
    end
    f = figure('Position', [40 40 2300 2000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(4, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:3
        S = load(fullfile(od, ['wallshift_' TG{ic} '.mat']));  q = find(S.seeds == seed, 1);  t = S.t;  DE = S.delta;
        wd = S.wd;  wd(1) = wd(2);  e = S.a_true - S.a_hat;
        ax = gobjects(4, 1);
        ax(1) = nexttile(tl, ic);  hold on; box on;
        h1 = plot(ax(1), t, wd * R_UM, '-', 'Color', cT, 'LineWidth', 5, 'DisplayName', 'z_d');
        h2 = plot(ax(1), t, S.h_true(:, q) * R_UM, '--', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'z');
        if 1 + DE > 0; yline(ax(1), (1 + DE) * R_UM, '--', 'Color', cG, 'LineWidth', 1.5, 'HandleVisibility', 'off'); end
        ylim(ax(1), [0 55]);  legend(ax(1), [h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        pf = 0.15;  if isfield(S, 'pfix'); pf = S.pfix; end
        text(ax(1), 0.97, 0.92, sprintf('%s = %+g R', ['w' char(772) '_s'], DE), 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', FS, 'FontWeight', 'bold');
        text(ax(1), 0.97, 0.78, sprintf('%sP_{fix} = %.2f', char(8730), pf), 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', FS - 2, 'FontWeight', 'bold');
        ax(2) = nexttile(tl, 3 + ic);  hold on; box on;
        h3 = plot(ax(2), t, S.a_true(:, q), '-', 'Color', cT, 'LineWidth', 5, 'DisplayName', az);
        h4 = plot(ax(2), t, S.a_hat(:, q), '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HAT '_z']);
        ylim(ax(2), [-0.2 1.6]);  legend(ax(2), [h3 h4], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        ax(3) = nexttile(tl, 6 + ic);  hold on; box on;
        g = max(S.h_true(:, q) - DE, 1.0);  dg = 1e-4;                     % true gap of this seed (user 10-13: red = the wall's own c at the true height)
        cn = 1 - g.^2 .* (local_a(g + dg) - local_a(g - dg)) / (2 * dg);   %   c(g) = 1 - g^2 a'_plane(g), independent of what the estimator assumes
        h5 = plot(ax(3), t, cn, '-', 'Color', cT, 'LineWidth', 5, 'DisplayName', 'c');
        h6 = plot(ax(3), t, S.c_hat(:, q), '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', ['c' HAT]);
        yline(ax(3), 1, ':', 'Color', cG, 'LineWidth', 1.8, 'HandleVisibility', 'off');  yline(ax(3), -3, ':', 'Color', cG, 'LineWidth', 1.8, 'HandleVisibility', 'off');
        lo = min([cn(:); S.c_hat(:, q); -0.4]);  hi = max([cn(:); S.c_hat(:, q); 1.1]);
        ylim(ax(3), [lo - 0.1 * (hi - lo), hi + 0.1 * (hi - lo)]);
        legend(ax(3), [h5 h6], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        ax(4) = nexttile(tl, 9 + ic);  hold on; box on;
        plot(ax(4), t, S.dxm(:, q), '-', 'Color', cE, 'LineWidth', 1.0);
        ylim(ax(4), [-0.2 0.2]);  yticks(ax(4), [-0.1 0 0.1]);
        xlabel(ax(4), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        for k = 1:4; xlim(ax(k), [0 1.2]);  set(ax(k), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax(k), 'off'); end
        if ic == 1
            ylabel(ax(1), ['z  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax(2), az, 'FontSize', FS + 2, 'FontWeight', 'bold');
            ylabel(ax(3), 'c', 'FontSize', FS + 2, 'FontWeight', 'bold');  ylabel(ax(4), [char(948) 'z_m  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
        end
        m = mean(e, 2);  dsc = t <= 1.0;  nw = t > 0.9 & t <= 1.0;  hd_ = t > 1.0;
        [~, iw] = max(abs(m .* dsc));
        fprintf('DELTA %+g (%d seeds) | descent: mean |e_a| %.4f, worst mean e_a %+.4f at t %.3f (h_d %.2f) | 0.9-1.0 s: e_a %+.4f +- %.4f | hold: %+.4f +- %.4f | c_hat at clamp %.0f %% of steps | dz_m rms %.4f um\n', ...
                DE, size(e, 2), mean(abs(m(dsc))), m(iw), t(iw), wd(iw), mean(m(nw)), mean(std(e(nw, :), 0, 2)), mean(m(hd_)), mean(std(e(hd_, :), 0, 2)), ...
                100 * mean(S.c_hat(:) <= -3 + 1e-6 | S.c_hat(:) >= 0.99), rms(S.dxm(:)));
    end
    out = fullfile(fd, sprintf('fig_1009_wallshift_%s_s%d.png', set_name, seed));
    exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end

function local_law(fd, cT, cE, FS)
    DL = [-4 -2 0 2 4];  CL = {[0.4 0.6 1], cE, [0 0 0], [1 0.55 0.55], cT};
    f = figure('Position', [40 40 1900 750], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    a1 = nexttile(tl);  hold on; box on;  a2 = nexttile(tl);  hold on; box on;
    for i = 1:numel(DL)
        DE = DL(i);  hd = linspace(1.10 + max(DE, 0), 22.2, 600);  dh = 1e-4;
        A = @(h) local_a(h - DE);  ap = (A(hd + dh) - A(hd - dh)) / (2 * dh);
        plot(a1, hd, A(hd), '-', 'Color', CL{i}, 'LineWidth', 3, 'DisplayName', sprintf('%+g R', DE));
        plot(a2, hd, 1 - hd.^2 .* ap, '-', 'Color', CL{i}, 'LineWidth', 3, 'DisplayName', sprintf('%+g R', DE));
    end
    yline(a2, -3, ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.8, 'HandleVisibility', 'off');  yline(a2, 1, ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.8, 'HandleVisibility', 'off');
    ylabel(a1, [char(257) '_z  (true)'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(a2, 'needed c', 'FontSize', FS, 'FontWeight', 'bold');
    ylim(a2, [-7 1.5]);
    for a = [a1 a2]
        set(a, 'XScale', 'log', 'XTick', [1.1 2 3 5 10 22.2], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.8);  xlim(a, [1.1 22.2]);  grid(a, 'off');
        xlabel(a, 'h_d  [R]  (estimator coordinate)', 'FontSize', FS, 'FontWeight', 'bold');
        legend(a, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 3, 'FontWeight', 'bold');
    end
    out = fullfile(fd, 'fig_1009_wallshift_law.png');
    exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end

function a = local_a(g)
    u = 1 ./ max(g, 1.0);
    a = 1 - (9/8) * u + (1/2) * u.^3 - (57/100) * u.^4 + (1/5) * u.^5 + (7/200) * u.^11 - (1/25) * u.^12;
end
