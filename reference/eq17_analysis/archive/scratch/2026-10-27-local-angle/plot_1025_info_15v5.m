% STATUS: SCRATCH -- 2026-10-25 (user: simplest comparison of 15 and 5 um at a fixed point). 1025 estimated angle, 20 deg wall, theta-hat[0] = 0,
%   runs temp_t3_est{15,5}_s{1,2,3}. Top: 1 / P of theta_y [1/deg^2] vs time (linear axes, the information the estimator has collected;
%   slope = information per second), dashed = 1/5^2 (sqrt P = 5 deg). Bottom: theta-hat_y - theta_y with +- sqrt P, seed 1.
%   15 um blue, 5 um green; seeds 2-3 thin.  -> test_results/temp_3d_wsonly/fig_1025_info_15v5.png | run: plot_1025_info_15v5()
function plot_1025_info_15v5()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    cT = [0.85 0 0];  CO = {[0 0.25 0.85], [0 0.55 0.2]};  CB = {[0.80 0.85 0.97], [0.80 0.93 0.82]};  FS = 17;  LFS = 13;
    TH = char(952);  HT = char(770);  DG = char(176);  MU = char(181);  AB = char(257);  PA = char(8741);  PE = char(8869);
    GAP = [15 5];
    f = figure('Position', [40 40 1300 1000], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax1 = nexttile(tl, 1);  hold on;  box on;  ax2 = nexttile(tl, 2);  hold on;  box on;  h1 = gobjects(0);  h2 = gobjects(0);
    for ig = 1:2
        for s = [2 3 1]
            S = load(fullfile(od, sprintf('temp_t3_est%d_s%d.mat', GAP(ig), s)));  r = S.R.runs{1};  k = (1:r.kend)';  t = r.t(k);
            sp = r.sPth(k, 2);  I = 1 ./ sp.^2;  e = r.thy(k) - 20;  dA = mean(r.aT_true(k) - r.aN_true(k));
            i0 = find(t >= 5, 1);  rate = (I(end) - I(i0)) / (t(end) - t(i0));
            lw = 1;  if s == 1; lw = 2.5; end
            hh = plot(ax1, t, I, '-', 'Color', CO{ig}, 'LineWidth', lw);
            if s == 1
                hh.DisplayName = sprintf('%d %sm  (%s_%s - %s_%s = %.3f):  %.4f /%s^2 per s', GAP(ig), MU, AB, PA, AB, PE, dA, rate, DG);  h1(end + 1) = hh;   %#ok<AGROW>
                hf = fill(ax2, [t; flipud(t)], [sp; -flipud(sp)], CB{ig}, 'EdgeColor', 'none', 'FaceAlpha', 0.7, ...
                          'DisplayName', sprintf('%s\\surdP,  %d %sm', char(177), GAP(ig), MU));  h2(end + 1) = hf;   %#ok<AGROW>
                h2(end + 1) = plot(ax2, t, e, '-', 'Color', CO{ig}, 'LineWidth', 2, 'DisplayName', sprintf('%s%s_y - %s_y,  %d %sm', TH, HT, TH, GAP(ig), MU));   %#ok<AGROW>
                i5 = find(sp < 5, 1);
                fprintf('%2d um: rate %.4f 1/deg^2/s, sqrt P = 5 deg at %.2f s\n', GAP(ig), rate, t(i5));
            else
                hh.HandleVisibility = 'off';
            end
        end
    end
    h1(end + 1) = plot(ax1, [0 20], [1 1] / 25, '--', 'Color', [0.4 0.4 0.4], 'LineWidth', 2, 'DisplayName', ['\surdP = 5' DG '  (1/P = 0.04)']);
    xlim(ax1, [0 20]);  ylim(ax1, [0 0.15]);
    ylabel(ax1, ['1 / P_{' TH 'y}   [1/' DG '^2]'], 'FontSize', FS, 'FontWeight', 'bold');
    legend(ax1, h1, 'Location', 'northoutside', 'NumColumns', 1, 'FontSize', LFS, 'FontWeight', 'bold');
    set(ax1, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
    plot(ax2, [0 20], [0 0], '-', 'Color', cT, 'LineWidth', 2.5, 'HandleVisibility', 'off');
    xlim(ax2, [0 20]);  ylim(ax2, [-25 10]);
    ylabel(ax2, [TH '_y error  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold');  xlabel(ax2, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    legend(ax2, h2, 'Location', 'northoutside', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold');
    set(ax2, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
    out = fullfile(od, 'fig_1025_info_15v5.png');  exportgraphics(f, out, 'Resolution', 150);  close(f);  fprintf('figure: %s\n', out);
end
