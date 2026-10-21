% STATUS: SCRATCH -- 2026-10-21 (user: "我要在起點上看可以多久收斂 和如果牆面的角度縮小會怎麼樣"). Fixed point at the start (22.2 R),
%   prior 10 deg, no gate, q_theta = 0, runs fs_w<theta>_s<seed>. Columns = wall angles. Rows: angle errors theta-hat_y - theta_y (blue) and
%   theta-hat_x - theta_x (green), every seed, with the zero line (red) | sqrt P_theta_y of every seed (blue) and the information estimate
%   sqrt P(t) = (1/P0 + I t / T_s)^(-1/2), I = (a_T - a_N)^2 / (2.5 a_T a_N) from the true gains at that height (black dashed).
%   Console: per wall, I, the time the estimate needs to reach sqrt P = 5 and 2 deg, the run's end sqrt P and end |errors|.
%   -> test_results/temp_3d_wsonly/fig_t2_fixstart.png | run: plot_t2_fixstart([5 10 20], 1:3)
function plot_t2_fixstart(TH, SD, pre)
    if nargin < 3; pre = 'fs'; end                           % run prefix: 'fs' = start 22.2 R, 'f15' = 15 um gap (10-21)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cGr = [0 0.55 0.2];  FS = 18;  LFS = 14;  TH_ = char(952);  DG = char(176);  HT = char(770);
    nc = numel(TH);
    f = figure('Position', [40 40 720 * nc 1000], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(2, nc, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:nc
        ax1 = nexttile(tl, ic);  hold on;  box on;  ax2 = nexttile(tl, nc + ic);  hold on;  box on;
        EE = [];
        for s = SD
            S = load(fullfile(od, sprintf('temp_t2_%s_w%d_s%d.mat', pre, TH(ic), s)));  r = S.R.runs{1};  k = 1:r.kend;  t = r.t(k);
            h1 = plot(ax1, t, r.thy(k) - TH(ic), '-', 'Color', cE, 'LineWidth', 1.2, 'DisplayName', [TH_ HT '_y - ' TH_ '_y']);
            h2 = plot(ax1, t, r.thx(k), '-', 'Color', cGr, 'LineWidth', 1.2, 'DisplayName', [TH_ HT '_x - ' TH_ '_x']);
            g1 = plot(ax2, t, r.sP(k, 6), '-', 'Color', cE, 'LineWidth', 1.5, 'DisplayName', ['\surdP_{' TH_ 'y}']);
            EE = [EE; r.sP(end, 6), abs(r.thy(end) - TH(ic)), abs(r.thx(end))];
            aT = mean(r.aT_true(k));  aN = mean(r.aN_true(k));
        end
        I = (aT - aN)^2 / (2.5 * aT * aN);  P0 = (10 * pi / 180)^2;  Ts = t(2) - t(1);
        tt = linspace(0, t(end), 400);  sPp = (1 / P0 + I * tt / Ts).^(-1/2) * 180 / pi;
        g2 = plot(ax2, tt, sPp, '--', 'Color', [0 0 0], 'LineWidth', 2, 'DisplayName', 'information estimate');
        plot(ax1, t, 0 * t, '-', 'Color', cT, 'LineWidth', 2.5, 'HandleVisibility', 'off');
        ylim(ax1, [-25 25]);  xlim(ax1, [0 t(end)]);  ylim(ax2, [0 11]);  xlim(ax2, [0 t(end)]);
        text(ax1, 0.97, 0.9, sprintf('wall %s_y = %d%s', TH_, TH(ic), DG), 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', FS, 'FontWeight', 'bold');
        legend(ax1, [h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        legend(ax2, [g1 g2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        if ic == 1; ylabel(ax1, ['angle error  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold'); ylabel(ax2, ['\surdP  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel(ax2, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        set([ax1 ax2], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);
        tq = @(sp) (1 / (sp * pi / 180)^2 - 1 / P0) / I * Ts;
        fprintf('wall %2d deg: a_T - a_N %.3f, I %.2e /step | estimate: sqrtP 5 deg at %.0f s, 2 deg at %.0f s | run end (%.0f s): sqrtP %.1f-%.1f deg, |e_thy| %.1f-%.1f, |e_thx| %.1f-%.1f deg\n', ...
                TH(ic), aT - aN, I, tq(5), tq(2), t(end), min(EE(:, 1)), max(EE(:, 1)), min(EE(:, 2)), max(EE(:, 2)), min(EE(:, 3)), max(EE(:, 3)));
    end
    out = fullfile(od, 'fig_t2_fixstart.png');  if ~strcmp(pre, 'fs'); out = fullfile(od, sprintf('fig_t2_fixstart_%s.png', pre)); end
    exportgraphics(f, out, 'Resolution', 110);  close(f);  fprintf('figure: %s\n', out);
end
