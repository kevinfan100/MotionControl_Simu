% STATUS: SCRATCH -- 2026-10-24 (user: both angles' numerator and denominator at 50 um): computed angle, run_t3_pair, 20 deg wall.
%   Columns: theta_y (x-z plane: -2 A_xz, A_xx - A_zz) | theta_x (y-z plane: -2 A_yz, A_yy - A_zz); rows: angle error, numerator,
%   denominator (red = true). 1022 style: seed 1 dark, seeds 2-3 light, Time (sec). Same y range in each row.
%   Output test_results/temp_3d_wsonly/fig_t3_planes_50um.png | EXPIRES: when the question is answered
linerr = @(e) local_break(mod(e + 90, 180) - 90);   % 10-24: line-angle error in (-90, 90], broken where it crosses +-90
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
S = load(fullfile(fo, 'temp_t3_pair.mat'));
FS = 15;  hatc = char(770);  abar = char(257);  Ahat = char(194);  cB = [0 0.25 0.85];  cBl = [0.6 0.72 1];
sel = find(abs(S.HH - 50 / 2.25) < 1e-6)';  [~, o] = sort(S.SS(sel), 'descend');  sel = sel(o);
At = S.xyz{sel(1)}.A_true;  ratio = S.xyz{sel(1)}.aN_true(1) / S.xyz{sel(1)}.aT_true(1);
pl = {'xz', 'yz'};  ang = {'y', 'x'};
tru = {{0, -2 * At(1,3), At(1,1) - At(3,3)}, {0, -2 * At(2,3), At(2,2) - At(3,3)}};
fig = figure('Color', 'w', 'Position', [80 80 1300 1050]);
tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for row = 1:3
    for c = 1:2
        nexttile;  hold on;  box on;  hs = gobjects(0);
        for i = sel
            a = S.xyz{i};  s1 = (S.SS(i) == 1);  col = cBl;  if s1; col = cB; end
            if c == 1; e = linerr(a.thy - S.thy); n = -2 * a.a(:,5); d = a.a(:,1) - a.a(:,3);
            else;      e = linerr(a.thx);          n = -2 * a.a(:,6); d = a.a(:,2) - a.a(:,3); end
            yy = {e, n, d};
            h = plot(a.t, yy{row}, 'Color', col, 'LineWidth', 1 + 0.8 * s1);  if s1; hs = h; end
        end
        yline(tru{c}{row}, 'r', 'LineWidth', 2.5);  uistack(hs, 'top');  xlim([0 S.T]);
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);
        if row == 1
            ylim([-30 30]);  ylabel(sprintf('\\theta_%s error  [\\circ]', ang{c}));
            legend(hs, sprintf('%s_%s - %s_%s,  seed 1   (%s-%s plane)', ['θ' hatc], ang{c}, char(952), ang{c}, pl{c}(1), pl{c}(2)), 'Location', 'northoutside');
            text(0.97, 0.9, sprintf('50 %sm   (%s_N/%s_T = %.2f)', char(181), abar, abar, ratio), 'Units', 'normalized', ...
                 'HorizontalAlignment', 'right', 'FontSize', FS, 'FontWeight', 'bold');
        elseif row == 2
            ylim([-0.06 0.08]);  ylabel('numerator');
            legend(hs, sprintf('-2%s_{%s},  seed 1   (red = true %.3f)', Ahat, pl{c}, abs(tru{c}{2})), 'Location', 'northoutside');
        else
            ylim([-0.06 0.08]);  ylabel('denominator');
            legend(hs, sprintf('%s_{%s%s} - %s_{zz},  seed 1   (red = true %.3f)', Ahat, pl{c}(1), pl{c}(1), Ahat, tru{c}{3}), 'Location', 'northoutside');
            xlabel('Time (sec)');
        end
    end
end
exportgraphics(fig, fullfile(fo, 'fig_t3_planes_50um.png'), 'Resolution', 150);
close(fig);


function e = local_break(e)
    e = e(:);  j = find(abs(diff(e)) > 90);  e(j + 1) = NaN;
end
