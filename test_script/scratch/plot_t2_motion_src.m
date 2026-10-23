% STATUS: SCRATCH -- 2026-10-22 (user, deck 1022: what adding motion buys). Where the change of theta-hat_y comes from, wait only vs
%   motion, runs tmgS<gap>_A_s1 | tmgS<gap>_B_s1 (seed 1, 20 s fixed point, 20 deg plane, start theta-hat_y = 0 => needed change +20 deg).
%   Per panel: theta-hat_y - theta_y (blue), cumulative change of theta-hat_y from the rows dp_m (ell_91..93, orange), a_Tm, a_Nm
%   (ell_94..95, purple) and theta_xm, theta_ym (ell_96..97, green); grey dotted = the change needed (+20 deg).
%   10-22 fix: the first version labelled rows 1-5 as 'y_1 (ell_91..93)' and rows 6-7 as 'ell_94..97'; the code split was 1-5 | 6-7.
%   Rows = gaps, columns = wait only | wait + dither.
%   -> test_results/temp_3d_wsonly/fig_t2_motion_src.png | run: plot_t2_motion_src([50 15])
function plot_t2_motion_src(GAP)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    cT = [0.85 0 0];  cB = [0 0.25 0.85];  cO = [0.9 0.5 0];  cG = [0 0.55 0.2];  cW = [0.5 0.5 0.5];  cP = [0.55 0.2 0.65];  FS = 18;  LFS = 14;
    TH = char(952);  HT = char(770);  DG = char(176);  MU = char(181);  AB = char(257);  EL = char(8467);
    PRE = {'A', 'B'};  LB = {'wait only', 'motion'};  nr = numel(GAP);
    f = figure('Position', [40 40 1240 520 * nr], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(nr, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ig = 1:nr
        for ia = 1:2
            S = load(fullfile(od, sprintf('temp_t2_tmgS%d_%s_s1.mat', GAP(ig), PRE{ia})));  r = S.R.runs{1};  k = 1:r.kend;  t = r.t(k);
            e = r.thy(k) - S.R.opts.theta_y;  c1 = cumsum(r.dth_dp(k, 2));  cg = cumsum(r.dth_ag(k, 2));  cr = cumsum(r.dth_ro(k, 2));
            rat = mean(r.aN_true(k)) / mean(r.aT_true(k));
            ax = nexttile(tl, (ig - 1) * 2 + ia);  hold on;  box on;
            plot([0 t(end)], [0 0], '-', 'Color', cT, 'LineWidth', 2.5, 'HandleVisibility', 'off');
            plot([0 t(end)], -e(1) * [1 1], ':', 'Color', cW, 'LineWidth', 2, 'DisplayName', 'needed change');
            plot(t, e, '-', 'Color', cB, 'LineWidth', 2, 'DisplayName', [TH HT '_y - ' TH '_y']);
            plot(t, c1, '-', 'Color', cO, 'LineWidth', 2, 'DisplayName', ['from \delta p_m  (' EL '_{9,1..3})']);
            plot(t, cg, '-', 'Color', cP, 'LineWidth', 2, 'DisplayName', ['from ' AB '_{Tm}, ' AB '_{Nm}  (' EL '_{9,4..5})']);
            plot(t, cr, '-', 'Color', cG, 'LineWidth', 2, 'DisplayName', ['from ' TH '_{xm}, ' TH '_{ym}  (' EL '_{9,6..7})']);
            ylim([-30 30]);  xlim([0 t(end)]);
            text(0.97, 0.08, sprintf('%d %sm  (%s_N/%s_T = %.2f),  %s', GAP(ig), MU, AB, AB, rat, LB{ia}), 'Units', 'normalized', ...
                 'HorizontalAlignment', 'right', 'FontSize', FS - 3, 'FontWeight', 'bold');
            if ia == 1; ylabel(['[' DG ']'], 'FontSize', FS, 'FontWeight', 'bold'); end
            if ig == nr; xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
            if ig == 1; legend('Location', 'northoutside', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold'); end
            set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
            fprintf('%2d um %-9s: at 20 s error %+.1f deg | from dp_m %+.1f, from a_Tm a_Nm %+.1f, from theta_m %+.1f\n', GAP(ig), LB{ia}, e(end), c1(end), cg(end), cr(end));
        end
    end
    out = fullfile(od, 'fig_t2_motion_src.png');  exportgraphics(f, out, 'Resolution', 110);  close(f);  fprintf('figure: %s\n', out);
end
