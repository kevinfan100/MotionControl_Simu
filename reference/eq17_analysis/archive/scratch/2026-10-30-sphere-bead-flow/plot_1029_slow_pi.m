% STATUS: SCRATCH -- 2026-10-29 (user): figure of run_1029_slow_pi (temp_1029_slow_pi.mat): per arm and seed, (left) minimum gap during the
%   turn with the contact line (1 R = 2.25 um) and h* (2.37 um); (right) true a_33 - a_d averaged over the last 3/4 of the turn.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
S = load(fullfile(fo, 'temp_1029_slow_pi.mat'));  na = size(S.arms, 1);  FS = 14;  cl = {[0 0.25 0.85], [0.85 0.4 0], [0 0.55 0.2]};
f = figure('Color', 'w', 'Position', [40 40 2000 650]);  tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for p = 1:2
    ax = nexttile(tl);  hold on;  box on;
    for id = 1:3
        v = arrayfun(@(ia) S.out{ia, id}(1 + 1 * (p == 1) + 6 * (p == 2)), 1:na);
        if p == 1; v = arrayfun(@(ia) S.out{ia, id}(2), 1:na); else; v = arrayfun(@(ia) S.out{ia, id}(7), 1:na); end
        plot((1:na) + (id - 2) * 0.12, v, 'o', 'MarkerFaceColor', cl{id}, 'MarkerEdgeColor', cl{id}, 'MarkerSize', 9, 'DisplayName', sprintf('seed %d', id));
    end
    if p == 1; yline(2.25, 'r-', 'LineWidth', 2, 'DisplayName', 'contact (1 R)');  yline(2.37, 'k--', 'LineWidth', 1.5, 'DisplayName', 'h*');  ylim([2.2 2.55]);  ylabel('minimum gap during the turn  [\mum]');
    else; yline(0, 'k--', 'LineWidth', 1.5, 'HandleVisibility', 'off');  ylabel('true a_{33} - a_d  (last 3/4 of the turn)');  ylim([-0.06 0.22]); end
    xline(5.5, ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5, 'HandleVisibility', 'off');
    set(ax, 'XTick', 1:na, 'XTickLabel', S.arms(:, 1), 'XLim', [0.5 na + 0.5], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(ax.Toolbar, 'Visible', 'off');
    xtickangle(ax, 30);  legend('Location', 'northwest');
end
fn = fullfile(fo, 'fig_1029_slow_pi.png');  exportgraphics(f, fn, 'Resolution', 140);  close(f);  system(['open ' fn]);
