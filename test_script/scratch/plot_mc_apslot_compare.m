% STATUS: SCRATCH -- 2026-10-14: figure for run_mc_apslot_compare, seed 1. Rows = arms (c-only, S8 w_s, a' as a state), columns plane | cell.
%   a-bar_z true (red, thick), estimate (blue). Output test_results/temp_1009_ws/fig_mc_apslot_compare.png.
function plot_mc_apslot_compare(arms, names, ttag)
    if nargin < 1 || isempty(arms); arms = {'conly', 's8', 'ap05'}; end
    if nargin < 2 || isempty(names); names = {'c-only (1-c)/h^2', 'S8: 1/(h - w_s)^2', 'a'' as a state'}; end
    if nargin < 3; ttag = ''; end   % 10-14: '_desc' = constant-velocity descent runs
    od = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'test_results', 'temp_1009_ws');
    WL = {'plane', 'cell'};  cT = [0.85 0 0];  cE = [0 0.25 0.85];  FS = 18;  LFS = 15;  AB = char(257);  HAT = char(770);
    na = numel(arms);  f = figure('Position', [40 40 2000 520 * na], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(na, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ia = 1:na
        for iw = 1:2
            S = load(fullfile(od, sprintf('apcmp_%s_%s%s.mat', arms{ia}, WL{iw}, ttag)));  t = S.t;
            ax = nexttile(tl);  hold on; box on;
            plot(ax, t, S.a_true(:, 1), '-', 'Color', cT, 'LineWidth', 3.5, 'DisplayName', [AB '_z']);
            plot(ax, t, S.a_hat(:, 1), '-', 'Color', cE, 'LineWidth', 1.4, 'DisplayName', [AB HAT '_z']);
            xlim(ax, [0 t(end)]);  ylim(ax, [0 1.05]);
            text(ax, 0.98, 0.9, sprintf('%s,  %s', names{ia}, WL{iw}), 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', LFS + 1, 'FontWeight', 'bold');
            if ia == 1; legend(ax, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold'); end
            if ia == na; xlabel(ax, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
            if iw == 1; ylabel(ax, [AB '_z'], 'FontSize', FS, 'FontWeight', 'bold'); end
            set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);  grid(ax, 'off');
        end
    end
    out = fullfile(od, sprintf('fig_mc_apslot_compare%s.png', ttag));  exportgraphics(f, out, 'Resolution', 120);  fprintf('figure: %s\n', out);
end
