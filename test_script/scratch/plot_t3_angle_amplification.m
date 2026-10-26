% STATUS: SCRATCH -- 2026-10-24 (user: show why the computed angle jumps at 50 um, rows following the formula):
%   e_theta_y ~ [cos 2theta_y (-2 e_xz) - sin 2theta_y (e_xx - e_zz)] / (2 (a_T - a_N)),  e_ij = A-hat_ij - A-bar_ij.
%   Columns 15 um | 50 um (computed angle, run_t3_pair, 20 deg wall). Rows: (1) numerator from the A-hat errors, (2) a_T - a_N
%   (red = true, blue = estimated = distance of the point (A-hat_xx - A-hat_zz, -2 A-hat_xz) from the origin), (3) theta_y error
%   (solid) with the formula (black dashed, seed 1). 1022 style: seed 1 dark, seeds 2-3 light, red reference line, Time (sec).
%   Output test_results/temp_3d_wsonly/fig_t3_angle_amplification.png | EXPIRES: when the explanation is done
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
S = load(fullfile(fo, 'temp_t3_pair.mat'));
Rr = 2.25;  um = [15 50];  FS = 15;  abar = char(257);  hatc = char(770);
cB = [0 0.25 0.85];  cBl = [0.6 0.72 1];
c2 = cosd(2 * S.thy);  s2 = sind(2 * S.thy);
fig = figure('Color', 'w', 'Position', [80 80 1300 1050]);
tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = gobjects(3, 2);
for ih = 1:2
    sel = find(abs(S.HH - um(ih) / Rr) < 1e-6)';  [~, o] = sort(S.SS(sel), 'descend');  sel = sel(o);
    At = S.xyz{sel(1)}.A_true;  d = S.xyz{sel(1)}.aT_true(1) - S.xyz{sel(1)}.aN_true(1);
    for row = 1:3
        ax(row, ih) = nexttile(tl, (row - 1) * 2 + ih);  hold on;  box on;  hs = gobjects(0);  hp = gobjects(0);
        for i = sel
            a = S.xyz{i};  s1 = (S.SS(i) == 1);  col = cBl;  if s1; col = cB; end
            exx = a.a(:, 1) - At(1,1);  ezz = a.a(:, 3) - At(3,3);  exz = a.a(:, 5) - At(1,3);
            num = c2 * (-2 * exz) - s2 * (exx - ezz);
            if row == 1; y = num;
            elseif row == 2; y = hypot(a.a(:, 1) - a.a(:, 3), -2 * a.a(:, 5));
            else; y = a.thy - S.thy; end
            h = plot(a.t, y, 'Color', col, 'LineWidth', 1 + 0.8 * s1);
            if s1; hs = h; end
            if row == 3 && s1; hp = plot(a.t, num / (2 * d) * 180 / pi, 'k--', 'LineWidth', 1.3); end
        end
        if row == 1; yline(0, 'r', 'LineWidth', 2.5); elseif row == 2; yline(d, 'r', 'LineWidth', 2.5); else; yline(0, 'r', 'LineWidth', 2.5); end
        uistack(hs, 'top');  if ~isempty(hp); uistack(hp, 'top'); end
        xlim([0 S.T]);  set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);
        if row == 1
            ylim([-0.04 0.04]);  if ih == 1; ylabel('numerator'); end
            legend(hs, sprintf('cos2%s_y(-2e_{xz}) - sin2%s_y(e_{xx} - e_{zz}),  seed 1', char(952), char(952)), 'Location', 'northoutside');
            text(0.97, 0.9, sprintf('%d %sm   (%s_N/%s_T = %.2f)', um(ih), char(181), abar, abar, S.xyz{sel(1)}.aN_true(1) / S.xyz{sel(1)}.aT_true(1)), ...
                 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', FS, 'FontWeight', 'bold');
        elseif row == 2
            ylim([0 0.12]);  if ih == 1; ylabel(sprintf('%s_T - %s_N', abar, abar)); end
            legend(hs, sprintf('estimated  (red = true %.3f)', d), 'Location', 'northoutside');
        else
            ylim([-30 30]);  if ih == 1; ylabel('\theta_y error  [\circ]'); end
            legend([hs hp], {sprintf('%s_y - %s_y,  seed 1', ['θ' hatc], char(952)), sprintf('numerator / (2(%s_T - %s_N))', abar, abar)}, ...
                   'Location', 'northoutside', 'Orientation', 'horizontal');
            xlabel('Time (sec)');
        end
    end
end
exportgraphics(fig, fullfile(fo, 'fig_t3_angle_amplification.png'), 'Resolution', 150);
close(fig);
for ih = 1:2   % console: size of the numerator (same at both heights) and of the angle error after 5 s
    sel = find(abs(S.HH - um(ih) / Rr) < 1e-6)';  At = S.xyz{sel(1)}.A_true;  d = S.xyz{sel(1)}.aT_true(1) - S.xyz{sel(1)}.aN_true(1);
    N = [];  E = [];
    for i = sel; a = S.xyz{i}; k = a.t > 5;
        N = [N; c2 * (-2 * (a.a(k,5) - At(1,3))) - s2 * ((a.a(k,1) - At(1,1)) - (a.a(k,3) - At(3,3)))];  E = [E; a.thy(k) - S.thy]; end
    fprintf('%d um: a_T - a_N %.3f | rms numerator %.4f | rms theta_y error %.1f deg (formula %.1f deg)\n', um(ih), d, rms(N), rms(E), rms(N) / (2 * d) * 180 / pi);
end
