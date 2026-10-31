% STATUS: SCRATCH -- 2026-10-25 (user: how to argue that the estimated angle (1025 Sec. 6) converges slowly at 15 um, look at the gains too,
%   and how to tell a setting error from a data limit). Runs temp_t3_est15_s{1,2,3} (run_t3_xyz_fixed, form 'estimated', theta-hat[0] = 0,
%   20 deg wall, 15 um, 20 s). Rows: theta_y error with +- sqrt P (seed 1 band), a-hat_perp - a_perp, a-hat_par - a_par with +- sqrt P.
%   Console: honesty (mean of error^2 / P over 5-20 s, 1 = honest), end bias, information rate d(1/P)/dt (5-20 s) of theta_y and a_perp.
%   -> test_results/temp_3d_wsonly/fig_1025_est15_conv.png | run: analyze_1025_est15()
function analyze_1025_est15()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cL = [0.65 0.75 0.95];  cB = [0.85 0.88 0.97];  FS = 16;  LFS = 12;
    TH = char(952);  HT = char(770);  DG = char(176);  AB = char(257);  PE = char(8869);  PA = char(8741);
    f = figure('Position', [40 40 1300 1100], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    nm = {[TH HT '_y - ' TH '_y  [' DG ']'], [AB HT '_' PE ' - ' AB '_' PE], [AB HT '_' PA ' - ' AB '_' PA]};
    for q = 1:3
        ax = nexttile(tl, q);  hold on;  box on;
        for s = [2 3 1]
            S = load(fullfile(od, sprintf('temp_t3_est15_s%d.mat', s)));  r = S.R.runs{1};  k = (1:r.kend)';  t = r.t(k);
            switch q
                case 1; ty = r.thy_true; if isscalar(ty); ty = ty + 0 * k; else; ty = ty(k); end; e = r.thy(k) - ty;  sp = r.sPth(k, 2);
                case 2; e = r.aN(k) - r.aN_true(k);   sp = r.sPa(k, 2);
                case 3; e = r.aT(k) - r.aT_true(k);   sp = r.sPa(k, 1);
            end
            if s == 1
                fill([t; flipud(t)], [sp; -flipud(sp)], cB, 'EdgeColor', 'none', 'DisplayName', '\pm\surdP  (seed 1)');
                plot(t, e, '-', 'Color', cE, 'LineWidth', 1.6, 'DisplayName', 'seed 1');
            else
                plot(t, e, '-', 'Color', cL, 'LineWidth', 1, 'HandleVisibility', 'off');
            end
            m = t >= 5;
            z2 = mean(e(m).^2 ./ sp(m).^2);  I = 1 ./ sp.^2;  i0 = find(m, 1);  rate = (I(end) - I(i0)) / (t(end) - t(i0));
            fprintf('row %d seed %d: end error %+.3f, sqrt P end %.3f | honesty mean(e^2/P) 5-20 s = %.2f | rate d(1/P)/dt %.3g per s\n', ...
                    q, s, e(end), sp(end), z2, rate);
        end
        plot([0 t(end)], [0 0], '-', 'Color', cT, 'LineWidth', 2.5, 'HandleVisibility', 'off');
        xlim([0 20]);  if q == 1; ylim([-30 15]); else; ylim([-0.12 0.12]); end
        ylabel(nm{q}, 'FontSize', FS, 'FontWeight', 'bold');
        if q == 3; xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
        if q == 1; legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold'); end
        set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
    end
    out = fullfile(od, 'fig_1025_est15_conv.png');  exportgraphics(f, out, 'Resolution', 150);  close(f);  fprintf('figure: %s\n', out);
end
