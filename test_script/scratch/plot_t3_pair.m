% STATUS: SCRATCH -- 2026-10-24 (user: simple comparison figure): run_t3_pair, wall theta_y = 20 deg, fixed points 50 um and 15 um.
%   2 x 2: top = estimated wall angle theta_y [deg], bottom = estimated normal gain a_N / a_nom; columns = 50 um, 15 um.
%   Red = true, blue = xyz (angle computed from A-hat, plane formula), orange = 1015 (angle as a state); 3 seeds each.
%   Console: theta_y and |theta_x| at 1, 5, 10, 20 s (mean over seeds). | EXPIRES: when the comparison is read
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
S = load(fullfile(fo, 'temp_t3_pair.mat'));
hs = unique(S.HH(:))';  hs = sort(hs, 'descend');  cB = [0 0.3 0.85];  cO = [0.9 0.45 0.1];  lab = {'50 \mum', '15 \mum'};
fig = figure('Color', 'w', 'Position', [80 80 1300 760]);
tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for ih = 1:2
    sel = find(abs(S.HH - hs(ih)) < 1e-9)';
    for row = 1:2
        nexttile((row - 1) * 2 + ih);  hold on;  box on;
        for i = sel
            a = S.xyz{i};  b = S.ref{i};  kb = 1:b.kend;
            if row == 1
                plot(a.t, a.thy, 'Color', cB, 'LineWidth', 1.2);  plot(b.t(kb), b.thy(kb), 'Color', cO, 'LineWidth', 1.2);
            else
                plot(a.t, a.aN, 'Color', cB, 'LineWidth', 1.2);  plot(b.t(kb), b.aN(kb), 'Color', cO, 'LineWidth', 1.2);
            end
        end
        if row == 1
            yline(S.thy, 'r', 'LineWidth', 2.5);  ylim([-20 60]);  ylabel('\theta_y [deg]');
            text(0.5, 1.03, lab{ih}, 'Units', 'normalized', 'HorizontalAlignment', 'center', 'FontSize', 16, 'FontWeight', 'bold');
        else
            yline(S.xyz{sel(1)}.aN_true(1), 'r', 'LineWidth', 2.5);  ylabel('a_N / a_{nom}');  xlabel('t [s]');
            yl = S.xyz{sel(1)}.aN_true(1) + [-0.15 0.15];  ylim(yl);
        end
        set(gca, 'FontSize', 14);
        if row == 1 && ih == 1
            h1 = plot(NaN, NaN, 'Color', cB, 'LineWidth', 2);  h2 = plot(NaN, NaN, 'Color', cO, 'LineWidth', 2);  h3 = plot(NaN, NaN, 'r', 'LineWidth', 2.5);
            legend([h1 h2 h3], {'xyz: angle computed from A-hat', '1015: angle as a state', 'true'}, 'Location', 'northoutside', 'Orientation', 'horizontal');
        end
    end
end
exportgraphics(fig, fullfile(fo, 'fig_t3_pair.png'), 'Resolution', 150);
fprintf('\n height | theta_y-hat at 1 / 5 / 10 / 20 s (mean of 3 seeds)   xyz | 1015      |theta_x-hat| at 20 s  xyz | 1015\n');
for ih = 1:2
    sel = find(abs(S.HH - hs(ih)) < 1e-9)';  ta = [1 5 10 S.T];  ma = zeros(1, 4);  mb = ma;  qa = 0;  qb = 0;
    for i = sel
        a = S.xyz{i};  b = S.ref{i};
        for j = 1:4; ka = find(a.t >= ta(j) - 1e-9, 1);  kb = min(find(b.t >= ta(j) - 1e-9, 1), b.kend);
            ma(j) = ma(j) + a.thy(ka) / 3;  mb(j) = mb(j) + b.thy(kb) / 3; end
        qa = qa + abs(a.thx(end)) / 3;  qb = qb + abs(b.thx(b.kend)) / 3;
    end
    fprintf(' %s | %5.1f %5.1f %5.1f %5.1f | %5.1f %5.1f %5.1f %5.1f |  %4.1f | %4.1f\n', strrep(lab{ih}, '\mu', 'u'), ma, mb, qa, qb);
end
