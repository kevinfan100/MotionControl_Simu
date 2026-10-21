% STATUS: SCRATCH -- 2026-10-20, single-seed figure for run_t2_3d_reg (house style of plot_mc_s8_b: seed 1, Unicode, bold,
%   true = thick red, estimate = blue (T: green), band = +- sqrt P, no grid / title). Columns: G1 (a_d 0.3) | G2 (a_d 0.1).
%   Rows: gap h [R] (red = h*) | a_N | a_T | a'_N, a'_T (log) | theta-hat_y, theta-hat_x [deg] | |a_N - a_d| with e^(-t/tau_a) (log).
%   -> test_results/temp_3d_wsonly/fig_t2_3d_reg_s<seed>.png | run: plot_t2_3d_reg(seed)
function plot_t2_3d_reg(seed, TG, LB, outtag)
    if nargin < 1; seed = 1; end
    if nargin < 2; TG = {'G1', 'G2'}; end                      % 10-20: columns = any run tags (e.g. the discrimination arms)
    if nargin < 3; LB = {}; end
    if nargin < 4; outtag = ''; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    nc = numel(TG);
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cGr = [0 0.55 0.2];  cB = [0.75 0.85 1];  FS = 20;  LFS = 15;
    AB = char(257);  HT = char(770);  TH = char(952);  DG = char(176);  PR = char(8242);
    f = figure('Position', [40 40 950 * nc 2400], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(6, nc, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:nc
        S = load(fullfile(od, ['temp_t2_' TG{ic} '.mat']));  sd = cellfun(@(q) q.seed, S.R.runs);  r = S.R.runs{find(sd == seed, 1)};   % cell access: failed runs carry an extra field
        k = 1:r.kend;  t = r.t(k);  ax = gobjects(6, 1);
        ax(1) = nexttile(tl, ic);  hold on;  box on;
        h1 = plot(t, r.h_star + 0 * t, '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', 'h*');
        h2 = plot(t, r.h(k), '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'h');
        set(gca, 'YScale', 'log');  ylim([1 25]);  legend([h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        lbl = sprintf('%s:  %s_d = %.1f', TG{ic}, AB, S.R.opts.a_d);  if ~isempty(LB); lbl = LB{ic}; end
        text(0.97, 0.85, lbl, 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', FS, 'FontWeight', 'bold');
        ax(2) = nexttile(tl, nc + ic);  hold on;  box on;
        h1 = plot(t, r.aN_true(k), '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', [AB '_N']);
        h2 = plot(t, r.aN(k), '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HT '_N']);
        ylim([0 1.1]);  legend([h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        ax(3) = nexttile(tl, 2 * nc + ic);  hold on;  box on;
        h1 = plot(t, r.aT_true(k), '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', [AB '_T']);
        h2 = plot(t, r.aT(k), '-', 'Color', cGr, 'LineWidth', 1.8, 'DisplayName', [AB HT '_T']);
        ylim([0 1.1]);  legend([h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        ax(4) = nexttile(tl, 3 * nc + ic);  hold on;  box on;
        h1 = plot(t, r.apN_true(k), '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', [AB PR '_N']);
        h2 = plot(t, r.apT_true(k), '--', 'Color', cT, 'LineWidth', 3, 'DisplayName', [AB PR '_T']);
        h3 = plot(t, max(r.apN(k), 1e-5), '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HT PR '_N']);
        h4 = plot(t, max(r.apT(k), 1e-5), '-', 'Color', cGr, 'LineWidth', 1.8, 'DisplayName', [AB HT PR '_T']);
        set(gca, 'YScale', 'log');  ylim([1e-3 3]);
        legend([h1 h2 h3 h4], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        ax(5) = nexttile(tl, 4 * nc + ic);  hold on;  box on;
        fill([t; flipud(t)], [r.thy(k) + r.sP(k, 6); flipud(r.thy(k) - r.sP(k, 6))], cB, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        h1 = plot(t, S.R.opts.theta_y + 0 * t, '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', [TH '_y,  ' TH '_x']);
        plot(t, 0 * t, '-', 'Color', cT, 'LineWidth', 4, 'HandleVisibility', 'off');
        h2 = plot(t, r.thy(k), '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [TH HT '_y']);
        h3 = plot(t, r.thx(k), '-', 'Color', cGr, 'LineWidth', 1.8, 'DisplayName', [TH HT '_x']);
        ylim([-30 50]);  legend([h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        ax(6) = nexttile(tl, 5 * nc + ic);  hold on;  box on;
        ad = S.R.opts.a_d;  tau = S.R.opts.tau_a;              % 10-20 user: check the -exp response (as 1019 Fig. 7.3)
        h1 = plot(t, abs(r.aN_true(k) - ad), '-', 'Color', cT, 'LineWidth', 3, 'DisplayName', ['|' AB '_N - ' AB '_d|']);
        h2 = plot(t, abs(r.aN(k) - ad), '-', 'Color', cE, 'LineWidth', 1.5, 'DisplayName', ['|' AB HT '_N - ' AB '_d|']);
        h3 = plot(t, abs(r.aN_true(1) - ad) * exp(-t / tau), 'k--', 'LineWidth', 2.5, 'DisplayName', ['e^{-t/' char(964) '_a}']);
        set(gca, 'YScale', 'log');  ylim([1e-3 1]);  xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        legend([h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        for q = 1:6; xlim(ax(q), [0 r.t(end)]);  set(ax(q), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2.2);  grid(ax(q), 'off'); end
        if ic == 1
            ylabel(ax(1), 'h  [R]', 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax(2), [AB '_N'], 'FontSize', FS + 2, 'FontWeight', 'bold');
            ylabel(ax(3), [AB '_T'], 'FontSize', FS + 2, 'FontWeight', 'bold');  ylabel(ax(4), [AB PR], 'FontSize', FS + 2, 'FontWeight', 'bold');
            ylabel(ax(5), [TH '  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax(6), ['|' AB ' - ' AB '_d|'], 'FontSize', FS, 'FontWeight', 'bold');
        end
    end
    out = fullfile(od, sprintf('fig_t2_3d_reg%s_s%d.png', outtag, seed));
    exportgraphics(f, out, 'Resolution', 120);  close(f);  fprintf('figure: %s\n', out);
end
