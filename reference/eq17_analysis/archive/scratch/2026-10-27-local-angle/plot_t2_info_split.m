% STATUS: SCRATCH -- 2026-10-25 (user: why the angle takes so long at the fixed point; how much information each source gives, which noise).
%   Fixed point, wait only, 20 deg plane, seed 1, 20 s, runs tmgS<gap>_<arm>_s1: A both sources | Y angle measurement only (rows 1-5 may not
%   update theta) | D position path only (angle measurement rows unused).
%   Figure: columns = gaps, 1 / P of theta_y [1/deg^2] vs time (log), three arms; dotted = the target 1/5^2.
%   Console: information rate (slope of 1/P, 5-20 s) per arm, predicted time from sqrt P 10 -> 5 deg, and the angle-measurement prediction
%   from the logs alone: rate = (a_T - a_N)^2 / (IF var([vA_m]_uw)) per sample (IF from the correlation of the samples).
%   -> test_results/temp_3d_wsonly/fig_t2_info_split.png | run: plot_t2_info_split([50 15 5])
function plot_t2_info_split(GAP)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    FS = 18;  LFS = 14;  TH = char(952);  DG = char(176);  MU = char(181);  AB = char(257);
    ARM = {'A', 'Y', 'D'};  LB = {'both', 'angle measurement only', 'position path only'};
    CO = {[0 0 0], [0 0.55 0.2], [0.9 0.5 0]};  LS = {'-', '-', '--'};
    nc = numel(GAP);
    f = figure('Position', [40 40 620 * nc 560], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(1, nc, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:nc
        ax = nexttile(tl, ic);  hold on;  box on;  hh = gobjects(0);
        for ia = 1:3
            S = load(fullfile(od, sprintf('temp_t2_tmgS%d_%s_s1.mat', GAP(ic), ARM{ia})));  r = S.R.runs{1};  k = (1:r.kend)';  t = r.t(k);
            I = 1 ./ r.sP(k, 6) .^ 2;
            hh(end + 1) = plot(t, I, LS{ia}, 'Color', CO{ia}, 'LineWidth', 2.5, 'DisplayName', LB{ia});   %#ok<AGROW>
            m = t >= 5;  i0 = find(m, 1);  rate = (I(end) - I(i0)) / (t(end) - t(i0));
            i5 = find(r.sP(k, 6) < 5, 1);  t5 = NaN;  if ~isempty(i5); t5 = t(i5); end
            fprintf('%2d um %-24s: rate %.2e 1/deg^2/s | sqrt P = 5 deg at %5.1f s (predicted 0.03/rate = %6.1f s) | error at 20 s %+.1f deg\n', ...
                    GAP(ic), LB{ia}, rate, t5, 0.03 / rate, r.thy(r.kend) - S.R.opts.theta_y);
            if ia == 1
                y = r.vAm(k, 7);  dA = mean(r.aT_true(k) - r.aN_true(k));  Ts = t(2) - t(1);
                c = xcorr(y - mean(y), 5, 'coeff');  IF = 1 + 2 * sum(c(7:11) .^ 2);
                pr = dA^2 / (IF * var(y)) / Ts * (pi / 180)^2;
                fprintf('       a_T - a_N %.3f, a_N/a_T %.2f | sd one sample %.2f, IF %.2f -> predicted angle-measurement rate %.2e 1/deg^2/s\n', ...
                        dA, mean(r.aN_true(k) ./ r.aT_true(k)), std(y), IF, pr);
            end
        end
        plot([0 20], [1 1] / 25, ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 2, 'HandleVisibility', 'off');
        set(ax, 'YScale', 'log');  ylim([8e-3 10]);  xlim([0 20]);
        text(0.97, 0.92, sprintf('%d %sm  (%s_N/%s_T = %.2f)', GAP(ic), MU, AB, AB, mean(r.aN_true(k) ./ r.aT_true(k))), 'Units', 'normalized', ...
             'HorizontalAlignment', 'right', 'FontSize', FS - 3, 'FontWeight', 'bold');
        text(0.4, 0.052, ['\surdP = 5' DG], 'FontSize', LFS, 'FontWeight', 'bold', 'Color', [0.4 0.4 0.4]);
        if ic == 1; ylabel(['1 / P_{' TH 'y}  [1/' DG '^2]'], 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        if ic == 1; legend(hh, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold'); end
        set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
    end
    out = fullfile(od, 'fig_t2_info_split.png');  exportgraphics(f, out, 'Resolution', 150);  close(f);  fprintf('figure: %s\n', out);
end
