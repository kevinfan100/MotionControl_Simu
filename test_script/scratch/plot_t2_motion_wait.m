% STATUS: SCRATCH -- 2026-10-22 (user, deck 1022: what adding motion buys). Fixed points held 20 s on the 20 deg plane, runs
%   tm<gap>_A_s<seed> (wait only) and tm<gap>_B_s<seed> (wait + dither 3 um, 8 Hz, 2 s ramp), seeds 1-3.
%   Figure 1 (fig_t2_motion_err.png): theta-hat_y - theta_y, rows = gaps, columns = wait only | wait + dither, every seed (seed 1 bold),
%     same axes as the deck's 50 / 5 um figure.
%   Figure 2 (fig_t2_motion_info.png): information on theta_y collected by the estimator, 1/P_99 [1/deg^2] vs time (log axis), median of
%     the seeds; dashed = wait only, solid = wait + dither; 5 um wait only as the near-wall reference. Slope = information per second.
%   -> test_results/temp_3d_wsonly/ | run: plot_t2_motion_wait([50 15], 1:3)
function plot_t2_motion_wait(GAP, SD)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    cT = [0.85 0 0];  cB = [0 0.25 0.85];  cL = [0.6 0.7 0.95];  FS = 18;  LFS = 14;
    TH = char(952);  HT = char(770);  DG = char(176);  MU = char(181);  AB = char(257);
    ARM = {'A', 'B'};  LB = {'wait only', 'motion'};  nr = numel(GAP);
    % ---- figure 1: theta_y error ----
    f = figure('Position', [40 40 1240 500 * nr], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(nr, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ig = 1:nr
        for ia = 1:2
            ax = nexttile(tl, (ig - 1) * 2 + ia);  hold on;  box on;
            for s = SD
                S = load(fullfile(od, sprintf('temp_t2_tm%d_%s_s%d.mat', GAP(ig), ARM{ia}, s)));  r = S.R.runs{1};  k = 1:r.kend;  t = r.t(k);
                e = r.thy(k) - S.R.opts.theta_y;
                if s == SD(1)
                    hb = plot(t, e, '-', 'Color', cB, 'LineWidth', 2, 'DisplayName', [TH HT '_y - ' TH '_y,  seed ' num2str(s)]);
                    rat = mean(r.aN_true(k)) / mean(r.aT_true(k));
                else
                    plot(t, e, '-', 'Color', cL, 'LineWidth', 1, 'HandleVisibility', 'off');
                end
            end
            plot([0 t(end)], [0 0], '-', 'Color', cT, 'LineWidth', 2.5, 'HandleVisibility', 'off');
            ylim([-30 15]);  xlim([0 t(end)]);
            text(0.97, 0.92, sprintf('%d %sm  (%s_N/%s_T = %.2f),  %s', GAP(ig), MU, AB, AB, rat, LB{ia}), 'Units', 'normalized', ...
                 'HorizontalAlignment', 'right', 'FontSize', FS - 3, 'FontWeight', 'bold');
            if ia == 1; ylabel([TH '_y error  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold'); end
            if ig == nr; xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
            if ig == 1; legend(hb, 'Location', 'northoutside', 'FontSize', LFS, 'FontWeight', 'bold'); end
            set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
        end
    end
    out = fullfile(od, 'fig_t2_motion_err.png');  exportgraphics(f, out, 'Resolution', 110);  close(f);  fprintf('figure: %s\n', out);
    % ---- figure 2: information 1 / P_99 ----
    f = figure('Position', [40 40 1200 760], 'Color', 'w', 'Visible', 'off');  hold on;  box on;
    CG = {[0 0.25 0.85], [0 0.55 0.2]};  hh = gobjects(0);
    for ig = 1:nr
        for ia = 1:2
            [t, I] = local_info(od, GAP(ig), ARM{ia}, SD);
            ls = {'--', '-'};
            hh(end + 1) = plot(t, I, ls{ia}, 'Color', CG{ig}, 'LineWidth', 2.5, 'DisplayName', sprintf('%d %sm, %s', GAP(ig), MU, LB{ia}));
            m = t >= 5;  rate = (I(find(m, 1, 'last')) - I(find(m, 1))) / (t(find(m, 1, 'last')) - t(find(m, 1)));
            fprintf('%2d um %-13s: 1/P at 20 s %.3g 1/deg^2, rate (5-20 s) %.3g 1/deg^2/s, sqrt P at 20 s %.2f deg\n', ...
                    GAP(ig), LB{ia}, I(end), rate, 1 / sqrt(I(end)));
        end
    end
    if exist(fullfile(od, 'temp_t2_tm5_A_s1.mat'), 'file')
        [t, I] = local_info(od, 5, 'A', SD);
        hh(end + 1) = plot(t, I, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 2, 'DisplayName', ['5 ' MU 'm, wait only (reference)']);
        fprintf(' 5 um wait only    : 1/P at 20 s %.3g 1/deg^2\n', I(end));
    end
    set(gca, 'YScale', 'log');  xlim([0 20]);
    xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');  ylabel(['1 / P_{' TH 'y}   [1/' DG '^2]'], 'FontSize', FS, 'FontWeight', 'bold');
    legend(hh, 'Location', 'northoutside', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold');
    set(gca, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
    out = fullfile(od, 'fig_t2_motion_info.png');  exportgraphics(f, out, 'Resolution', 110);  close(f);  fprintf('figure: %s\n', out);
end

function [t, I] = local_info(od, gap, arm, SD)
    % median over seeds of 1/P_99 (state 9 = theta_y, sP column 6, already logged in deg)
    Is = [];
    for s = SD
        S = load(fullfile(od, sprintf('temp_t2_tm%d_%s_s%d.mat', gap, arm, s)));  r = S.R.runs{1};  k = 1:r.kend;
        Is(:, end + 1) = 1 ./ r.sP(k, 6) .^ 2;   %#ok<AGROW>
        t = r.t(k);
    end
    I = median(Is, 2);
end
