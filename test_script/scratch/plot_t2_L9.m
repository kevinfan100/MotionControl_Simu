% STATUS: SCRATCH -- 2026-10-22 (user: label state 8 / 9, script l for the gains, top panel = true A-bar + measured position scatter,
%   theta_y only). Wait-only fixed points tmg<gap>_s1 (20 deg plane, seed 1). Columns = gaps (50, 5 um).
%   Row 1: tracking error delta p in the x-z plane [nm] (dots) and the shape of the true gain matrix A-bar (red ellipse, semi-axes ~ sqrt of
%   the gains, scaled to the cloud) with the wall normal (red dashed).
%   Rows 2-4: actual gains l_9j of the estimator row for state 9 = theta_y (theta-hat_y[k+1] = theta-hat_y[k] + sum_j l_9j e_yj, doc Sec. 4),
%   j = measurement: dp_m x, y, z [deg/nm] | a_Tm, a_Nm [deg per unit a-bar] | theta_xm, theta_ym [deg/deg]. Log axes.
%   Optional TAG (cell, one run per column, e.g. {'tmg15_s1', 'tmgB15_s1'} = wait only | wait + dither) and LBL (column labels);
%   output then fig_t2_l9_<out>.png.
%   -> test_results/temp_3d_wsonly/fig_t2_l9.png | run: plot_t2_L9([50 5])  or  plot_t2_L9([15 15], {'tmg15_s1','tmgB15_s1'}, {'wait only','wait + dither'}, '15AB')
function plot_t2_L9(GAP, TAG, LBL, outtag)
    if nargin < 2; TAG = arrayfun(@(g) sprintf('tmg%d_s1', g), GAP, 'UniformOutput', false); end
    if nargin < 3; LBL = repmat({''}, size(GAP)); end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');  addpath(genpath(fullfile(root, 'model')));  pc = physical_constants();  Rnm = pc.R * 1e3;
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0 0.55 0.2];  cO = [0.9 0.5 0];  cL = [0.65 0.75 0.95];  FS = 17;  LFS = 13;
    MU = char(181);  DG = char(176);  TH = char(952);  AB = char(257);  EL = char(8467);  sub = {'x', 'y', 'z'};
    nc = numel(GAP);
    f = figure('Position', [40 40 900 * nc 1700], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(4, nc, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ig = 1:nc
        S = load(fullfile(od, ['temp_t2_' TAG{ig} '.mat']));  r = S.R.runs{1};  k = 1:r.kend;  t = r.t(k);  ty = S.R.opts.theta_y;
        aT = mean(r.aT_true(k));  aN = mean(r.aN_true(k));
        % ---- row 1: cloud + true A-bar shape ----
        d = (r.pd(k, [1 3]) - r.p(k, [1 3])) * 1e3;  d = d - mean(d);
        ax = nexttile(tl, ig);  hold on;  box on;
        plot(d(1:4:end, 1), d(1:4:end, 2), '.', 'Color', cL, 'MarkerSize', 3, 'DisplayName', '\delta p  (measured cloud)');
        V2 = [cosd(ty) sind(ty); -sind(ty) cosd(ty)];  A2 = V2 * diag([aT aN]) * V2';     % columns: u = (cos, -sin), w = (sin, cos) in x-z
        [Ev, Dv] = eig(A2);  ph = linspace(0, 2 * pi, 300);  el = Ev * sqrt(Dv) * [cos(ph); sin(ph)];
        el = el * 2 * sqrt(mean(var(d))) / sqrt(mean(diag(Dv)));          % shape of A-bar, scaled to the 2-sigma size of the cloud
        plot(el(1, :), el(2, :), '-', 'Color', cT, 'LineWidth', 3, 'DisplayName', ['true  ' char(256) '  (shape)']);
        Lq = 1.15 * max(abs(el(:)));  plot([-1 1] * Lq * sind(ty), [-1 1] * Lq * cosd(ty), '--', 'Color', cT, 'LineWidth', 1.5, 'DisplayName', 'wall normal');
        axis equal;  lim = 1.3 * Lq;  xlim([-lim lim]);  ylim([-lim lim]);
        text(0.03, 0.93, sprintf('%d %sm   %s_N/%s_T = %.2f   %s', GAP(ig), MU, AB, AB, aN / aT, LBL{ig}), 'Units', 'normalized', 'FontSize', FS - 1, 'FontWeight', 'bold');
        xlabel('\delta x  [nm]', 'FontSize', FS - 2, 'FontWeight', 'bold');  ylabel('\delta z  [nm]', 'FontSize', FS - 2, 'FontWeight', 'bold');
        legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
        % ---- rows 2-4: l_9j ----
        Lr = r.L9(k, :);
        conv = {Lr(:, 1:3) * 180 / pi / Rnm, Lr(:, 4:5) * 180 / pi, Lr(:, 6:7)};
        nms = {arrayfun(@(i) [EL '_{9,' num2str(i) '}  (\delta p_{m' sub{i} '})'], 1:3, 'UniformOutput', false), ...
               {[EL '_{9,4}  (' AB '_{Tm})'], [EL '_{9,5}  (' AB '_{Nm})']}, {[EL '_{9,6}  (' TH '_{xm})'], [EL '_{9,7}  (' TH '_{ym})']}};
        yl = {['|' EL '_{9j}|  [' DG '/nm]'], ['|' EL '_{9j}|  [' DG ']'], ['|' EL '_{9j}|  [' DG '/' DG ']']};
        cols = {[cE; cG; cO], [cE; cO], [cG; cE]};  YL = {[1e-6 1e0], [1e-5 1e1], [1e-8 1e-2]};
        for q = 1:3
            ax = nexttile(tl, q * nc + ig);  hold on;  box on;  hh = gobjects(0);
            for j = 1:size(conv{q}, 2)
                hh(end + 1) = plot(t, movmedian(abs(conv{q}(:, j)), 81), '-', 'Color', cols{q}(j, :), 'LineWidth', 1.6, 'DisplayName', nms{q}{j});
            end
            set(ax, 'YScale', 'log');  ylim(YL{q});  xlim([0 t(end)]);
            if ig == 1; ylabel(yl{q}, 'FontSize', FS - 2, 'FontWeight', 'bold'); end
            if q == 1; text(0.97, 0.9, ['state 9 = ' TH '_y'], 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', FS - 2, 'FontWeight', 'bold'); end
            if q == 3; xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
            legend(hh, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
        end
    end
    out = fullfile(od, 'fig_t2_l9.png');  if nargin >= 4; out = fullfile(od, ['fig_t2_l9_' outtag '.png']); end
    exportgraphics(f, out, 'Resolution', 100);  close(f);  fprintf('figure: %s\n', out);
end
