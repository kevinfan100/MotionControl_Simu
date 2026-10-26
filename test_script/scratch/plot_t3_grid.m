% STATUS: SCRATCH -- 2026-10-23: figures of run_t3_grid (xyz estimator vs 1015, fixed point, wall angle x height).
%   Fig 1: angle error |e_theta| = angle between the true and the estimated wall normal [deg]; Fig 2: e_aN = a_N - a_N-hat.
%   Rows = height h0, columns = wall angle theta_y; blue = xyz (angles from A-hat), orange = 1015 (angles as states); 3 seeds each.
%   Console: health (eigenvalue floor, increment vs eigenvector angle) and |e_theta|, e_aN, e_aT at 1, 2, 5 s (mean over seeds).
%   EXPIRES: when the first xyz test is read
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
S = load(fullfile(fo, 'temp_t3_grid.mat'));
% 10-23 (second run): the xyz arm reads A5 in closed form (trace / sum of squares); the eigen-decomposition is logged as a check.
ths = unique(S.TH(:))';  hs = unique(S.HH(:))';  cB = [0 0.3 0.85];  cO = [0.9 0.45 0.1];
wv = @(tx, ty) [sind(ty) .* cosd(tx), -sind(tx), cosd(ty) .* cosd(tx)];
ang = @(tx, ty, ty0) acosd(min(abs(wv(tx, ty) * wv(0, ty0)'), 1));
for fig_id = 1:2
    fig = figure('Color', 'w', 'Position', [60 60 1500 820]);
    tl = tiledlayout(numel(hs), numel(ths), 'TileSpacing', 'compact', 'Padding', 'compact');
    for ih = 1:numel(hs)
        for it = 1:numel(ths)
            nexttile;  hold on;  box on;
            sel = find(S.TH == ths(it) & S.HH == hs(ih));
            for i = sel'
                a = S.xyz{i};  b = S.ref{i};  kb = 1:b.kend;
                if fig_id == 1
                    plot(a.t, ang(a.thx, a.thy, ths(it)), 'Color', cB, 'LineWidth', 1);
                    plot(b.t(kb), ang(b.thx(kb), b.thy(kb), ths(it)), 'Color', cO, 'LineWidth', 1);
                else
                    plot(a.t, a.aN_true - a.aN, 'Color', cB, 'LineWidth', 1);
                    plot(b.t(kb), b.aN_true(kb) - b.aN(kb), 'Color', cO, 'LineWidth', 1);
                end
            end
            if fig_id == 1; ylim([0 30]); else; ylim([-0.15 0.15]); yline(0, 'k:'); end
            set(gca, 'FontSize', 12);
            if ih == 1; text(0.5, 1.04, sprintf('\\theta_y = %d^\\circ', ths(it)), 'Units', 'normalized', 'HorizontalAlignment', 'center', 'FontSize', 14, 'FontWeight', 'bold'); end
            if it == 1
                if fig_id == 1; ylabel(sprintf('h_0 = %.1f R\n|e_\\theta| [deg]', hs(ih))); else; ylabel(sprintf('h_0 = %.1f R\ne_{aN}', hs(ih))); end
            end
            if ih == numel(hs); xlabel('t [s]'); end
            if ih == 1 && it == 1
                h1 = plot(NaN, NaN, 'Color', cB, 'LineWidth', 2);  h2 = plot(NaN, NaN, 'Color', cO, 'LineWidth', 2);
                legend([h1 h2], {'xyz: angles from A-hat (closed form)', '1015: angles as states'}, 'Location', 'northoutside', 'Orientation', 'horizontal');
            end
        end
    end
    names = {'fig_t3_grid_angle.png', 'fig_t3_grid_aN.png'};
    exportgraphics(fig, fullfile(fo, names{fig_id}), 'Resolution', 150);
end
% ---- console ----
fprintf('health (xyz): eigenvalue floor steps total %d, min eigenvalue %.3f\n', sum(cellfun(@(a) a.n_floor, S.xyz)), min(cellfun(@(a) min(a.mineig), S.xyz)));
dcl = cellfun(@(a) max(acosd(min(abs(sum(wv(a.thx(a.t > 0.5), a.thy(a.t > 0.5)) .* wv(a.thx_eig(a.t > 0.5), a.thy_eig(a.t > 0.5)), 2)), 1))), S.xyz);
fprintf('A5 closed form vs eig(A-hat) after 0.5 s: max over cells %.3f deg, median of the per-cell max %.3f deg\n', max(dcl), median(dcl));
fprintf('ref (1015): runs that stopped early %d\n', sum(cellfun(@(b) b.kend < numel(b.t), S.ref)));
fprintf('\n th_y  h0  | |e_th| xyz 1/2/5 s       1015 1/2/5 s     | e_aN xyz 5 s  1015 5 s | e_aT xyz 5 s  1015 5 s\n');
for ih = 1:numel(hs)
    for it = 1:numel(ths)
        sel = find(S.TH == ths(it) & S.HH == hs(ih));  E = zeros(numel(sel), 10);
        for j = 1:numel(sel)
            a = S.xyz{sel(j)};  b = S.ref{sel(j)};
            ka = @(tt) find(a.t >= tt, 1);  kb = @(tt) min(find(b.t >= tt, 1), b.kend);
            E(j, :) = [ang(a.thx(ka(1)), a.thy(ka(1)), ths(it)), ang(a.thx(ka(2)), a.thy(ka(2)), ths(it)), ang(a.thx(end), a.thy(end), ths(it)), ...
                       ang(b.thx(kb(1)), b.thy(kb(1)), ths(it)), ang(b.thx(kb(2)), b.thy(kb(2)), ths(it)), ang(b.thx(kb(5)), b.thy(kb(5)), ths(it)), ...
                       a.aN_true(end) - a.aN(end), b.aN_true(kb(5)) - b.aN(kb(5)), a.aT_true(end) - a.aT(end), b.aT_true(kb(5)) - b.aT(kb(5))];
        end
        m = mean(E, 1);
        fprintf('%4d %5.2f | %5.1f %5.1f %5.1f   %5.1f %5.1f %5.1f | %+.3f  %+.3f | %+.3f  %+.3f\n', ths(it), hs(ih), m);
    end
end
