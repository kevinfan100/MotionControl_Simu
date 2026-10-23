% STATUS: SCRATCH -- 2026-10-22 (user: compare the Kalman gain with and without the angle measurement). Runs tmgS<gap>_<arm>_s1
%   (seed 1, 20 s fixed point, 20 deg plane): with the angle measurement = A (wait) / B (motion); without = D (wait) / C (motion).
%   Columns = with | without. Row 1: theta-hat_y - theta_y. Rows 2-4: actual gains ell_9j of state 9 = theta_y (1015 doc Sec. 4,
%   rows actually used; angle columns in the theta_m form): dp_m x, y, z [deg/nm] | a_Tm, a_Nm [deg] | theta_xm, theta_ym [deg/deg].
%   -> test_results/temp_3d_wsonly/fig_t2_gain_rows_<gap><mode>.png | run: plot_t2_gain_rows(15, 'motion')
function plot_t2_gain_rows(gap, mode)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');  addpath(genpath(fullfile(root, 'model')));  pc = physical_constants();  Rnm = pc.R * 1e3;
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0 0.55 0.2];  cO = [0.9 0.5 0];  FS = 17;  LFS = 13;
    MU = char(181);  DG = char(176);  TH = char(952);  HT = char(770);  AB = char(257);  EL = char(8467);  sub = {'x', 'y', 'z'};
    if strcmp(mode, 'motion'); ARM = {'B', 'C'}; else; ARM = {'A', 'D'}; end
    CL = {'with angle measurement', 'without angle measurement'};
    f = figure('Position', [40 40 1700 1500], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:2
        S = load(fullfile(od, sprintf('temp_t2_tmgS%d_%s_s1.mat', gap, ARM{ic})));  r = S.R.runs{1};  k = 1:r.kend;  t = r.t(k);
        ax = nexttile(tl, ic);  hold on;  box on;
        plot(t, r.thy(k) - S.R.opts.theta_y, '-', 'Color', cE, 'LineWidth', 2, 'DisplayName', [TH HT '_y - ' TH '_y']);
        plot([0 t(end)], [0 0], '-', 'Color', cT, 'LineWidth', 2.5, 'HandleVisibility', 'off');
        ylim([-30 15]);  xlim([0 t(end)]);
        text(0.97, 0.1, sprintf('%d %sm,  %s,  %s', gap, MU, mode, CL{ic}), 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', FS - 3, 'FontWeight', 'bold');
        if ic == 1; ylabel(['[' DG ']'], 'FontSize', FS, 'FontWeight', 'bold'); end
        legend('Location', 'northoutside', 'FontSize', LFS, 'FontWeight', 'bold');
        set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
        Lr = r.L9(k, :);
        conv = {Lr(:, 1:3) * 180 / pi / Rnm, Lr(:, 4:5) * 180 / pi, Lr(:, 6:7)};
        nms = {arrayfun(@(i) [EL '_{9,' num2str(i) '}  (\delta p_{m' sub{i} '})'], 1:3, 'UniformOutput', false), ...
               {[EL '_{9,4}  (' AB '_{Tm})'], [EL '_{9,5}  (' AB '_{Nm})']}, {[EL '_{9,6}  (' TH '_{xm})'], [EL '_{9,7}  (' TH '_{ym})']}};
        yl = {['|' EL '_{9j}|  [' DG '/nm]'], ['|' EL '_{9j}|  [' DG ']'], ['|' EL '_{9j}|  [' DG '/' DG ']']};
        cols = {[cE; cG; cO], [cE; cO], [cG; cE]};  YL = {[1e-6 1e0], [1e-5 1e1], [1e-8 1e-2]};
        for q = 1:3
            ax = nexttile(tl, q * 2 + ic);  hold on;  box on;  hh = gobjects(0);
            for j = 1:size(conv{q}, 2)
                v = movmedian(abs(conv{q}(:, j)), 81);
                hh(end + 1) = plot(t, v, '-', 'Color', cols{q}(j, :), 'LineWidth', 1.6, 'DisplayName', nms{q}{j});   %#ok<AGROW>
            end
            set(ax, 'YScale', 'log');  ylim(YL{q});  xlim([0 t(end)]);
            if q == 3 && all(isnan(conv{q}(:))); text(0.5, 0.5, 'rows not used', 'Units', 'normalized', 'HorizontalAlignment', 'center', 'FontSize', FS, 'FontWeight', 'bold'); end
            if ic == 1; ylabel(yl{q}, 'FontSize', FS - 2, 'FontWeight', 'bold'); end
            if q == 1; text(0.97, 0.9, ['state 9 = ' TH '_y'], 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', FS - 2, 'FontWeight', 'bold'); end
            if q == 3; xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
            legend(hh, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
        end
        m = t >= 5;
        fprintf('%2d um %s %-26s: median |l_9j| (t > 5 s) dp x/y/z %.2e %.2e %.2e deg/nm | a_Tm a_Nm %.2e %.2e deg | th_xm th_ym %.2e %.2e | err at 20 s %+.1f deg\n', ...
                gap, mode, CL{ic}, median(abs(conv{1}(m, :)), 'omitnan'), median(abs(conv{2}(m, :)), 'omitnan'), median(abs(conv{3}(m, :)), 'omitnan'), r.thy(r.kend) - S.R.opts.theta_y);
    end
    out = fullfile(od, sprintf('fig_t2_gain_rows_%d%s.png', gap, mode));  exportgraphics(f, out, 'Resolution', 100);  close(f);  fprintf('figure: %s\n', out);
end
