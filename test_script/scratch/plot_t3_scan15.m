% STATUS: SCRATCH -- 2026-10-24: figures of run_t3_scan15 (single seed, 20 deg wall, 15 um, hold 5 s / 10 s, then 3 s scan along u-hat,
%   no regulation). One figure per hold; columns = computed angle | estimated angle; rows = theta_y error, theta_x error, gap h [um]
%   (red = 15 um). 1022 style. Output test_results/temp_3d_wsonly/fig_t3_scan15_hold{5,10}.png | EXPIRES: when the scan comparison is read
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
S = load(fullfile(fo, 'temp_t3_scan15.mat'));  out = S.out;
FS = 15;  hatc = char(770);  cB = [0 0.25 0.85];  cG = [0 0.5 0.2];  Rr = 2.25;
arm = {'computed angle', 'estimated angle'};
for j = 1:2
    fig = figure('Color', 'w', 'Position', [80 80 1300 1050]);
    tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for row = 1:3
        for c = 1:2
            nexttile;  hold on;  box on;
            if c == 1; r = out.xyz{j}; k = 1:numel(r.t); else; r = out.ref{j}; k = 1:r.kend; end
            if row == 1; y = r.thy(k) - 20; col = cB; tru = 0;  yl = [-30 30];
            elseif row == 2; y = r.thx(k); col = cG; tru = 0; yl = [-30 30];
            else; y = r.h(k) * Rr; col = cB; tru = 15; yl = [10 30]; end
            yline(tru, 'r', 'LineWidth', 2.5);
            h = plot(r.t(k), y, 'Color', col, 'LineWidth', 1.8);
            xlim([0 r.t(k(end))]);  ylim(yl);  set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);
            if row == 1
                if c == 1; ylabel('\theta_y error  [\circ]'); end
                legend(h, sprintf('%s_y - %s_y,  seed %d   (%s)', ['θ' hatc], char(952), S.seed, arm{c}), 'Location', 'northoutside');
                text(0.97, 0.9, sprintf('15 %sm, hold %d s, scan %d s', char(181), S.holds(j), S.Tsc), 'Units', 'normalized', ...
                     'HorizontalAlignment', 'right', 'FontSize', FS, 'FontWeight', 'bold');
            elseif row == 2
                if c == 1; ylabel('\theta_x error  [\circ]'); end
                legend(h, sprintf('%s_x - %s_x', ['θ' hatc], char(952)), 'Location', 'northoutside');
            else
                if c == 1; ylabel(sprintf('h  [%sm]', char(181))); end
                legend(h, 'gap h  (red = 15 \mum)', 'Location', 'northoutside');  xlabel('Time (sec)');
            end
        end
    end
    exportgraphics(fig, fullfile(fo, sprintf('fig_t3_scan15_hold%d.png', S.holds(j))), 'Resolution', 150);
    close(fig);
end
