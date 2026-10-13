% STATUS: SCRATCH -- 2026-10-13, comparison figure for probe_1009_descent_arms.m (conversation, not tex yet).
%   Columns plane | cell. Rows: (1) 10-seed mean e_a = a_true - a_hat, start w_s-hat[0] = 0; (2) same, start -1; (3) 10-seed mean w_s-hat,
%   start 0 solid, start -1 dashed. Arms: c-only (S6, sqrt P_fix 0.15), S8 (numerator 1, prior 1.0), A (prior 0.3), B (numerator 9/8).
%   e_a curves smoothed with a 10 ms moving mean; dotted verticals = h_d 6 R and 2 R (the window edges of the table).
function plot_1009_descent_arms()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  od = fullfile(root, 'test_results', 'temp_1009_ws');
    WL = {'plane', 'cell'};  CO = {'wallshift_p0', 'wallshift_m1.05'};
    AR = {'', '_sw030', '_n98'};  NM = {'S8 (numerator 1)', 'A: prior 0.3', 'B: numerator 9/8'};
    CL = {[0 0.25 0.85], [0.3 0.75 0.95], [0 0.6 0.25]};  FS = 20;  LFS = 15;  WB = ['w' char(772)];  HAT = char(770);
    f = figure('Position', [40 40 2200 1500], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    sm = @(x) movmean(x, 16);
    for iw = 1:2
        C = load(fullfile(od, [CO{iw} '.mat']));  t = C.t;  wd = C.wd;  wd(1) = wd(2);
        t6 = t(find(wd <= 6, 1));  t2 = t(find(wd <= 2, 1));
        for r = 1:2
            ax = nexttile(tl, (r - 1) * 2 + iw);  hold on; box on;  hh = gobjects(0);
            hh(end + 1) = plot(ax, t, sm(mean(C.a_true - C.a_hat, 2)), '-', 'Color', 'k', 'LineWidth', 2.5, 'DisplayName', 'c-only'); %#ok<AGROW>
            st = 'p0';  if r == 2; st = 'm1'; end
            for ia = 1:3
                S = load(fullfile(od, sprintf('wsonly_%s_%s%s.mat', WL{iw}, st, AR{ia})));
                hh(end + 1) = plot(ax, S.t, sm(mean(S.a_true - S.a_hat, 2)), '-', 'Color', CL{ia}, 'LineWidth', 2.2, 'DisplayName', NM{ia}); %#ok<AGROW>
            end
            yline(ax, 0, '-', 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off');
            xline(ax, t6, ':', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.5, 'HandleVisibility', 'off');  xline(ax, t2, ':', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.5, 'HandleVisibility', 'off');
            ylim(ax, [-0.04 0.08]);  xlim(ax, [0 1.2]);
            if r == 1; legend(ax, hh, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
                text(ax, 0.02, 0.92, WL{iw}, 'Units', 'normalized', 'FontSize', FS, 'FontWeight', 'bold'); end
            text(ax, 0.02, 0.78, sprintf('%s%s_s[0] = %s', WB, HAT, strrep(strrep(st, 'p0', '0'), 'm1', '-1')), 'Units', 'normalized', 'FontSize', FS - 2, 'FontWeight', 'bold');
            if iw == 1; ylabel(ax, ['e_{' char(257) '}  (10-seed mean)'], 'FontSize', FS, 'FontWeight', 'bold'); end
            set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.8);  grid(ax, 'off');
        end
        ax = nexttile(tl, 4 + iw);  hold on; box on;
        for ia = 1:3
            for st = {'p0', 'm1'}
                S = load(fullfile(od, sprintf('wsonly_%s_%s%s.mat', WL{iw}, st{1}, AR{ia})));  ls = '-';  if strcmp(st{1}, 'm1'); ls = '--'; end
                plot(ax, S.t, mean(S.w_hat, 2), ls, 'Color', CL{ia}, 'LineWidth', 2.2, 'HandleVisibility', 'off');
            end
        end
        yline(ax, 0, '-', 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off');
        xline(ax, t6, ':', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.5);  xline(ax, t2, ':', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.5);
        ylim(ax, [-1.6 1.4]);  xlim(ax, [0 1.2]);  xlabel(ax, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        text(ax, 0.02, 0.92, 'solid: start 0,  dashed: start -1', 'Units', 'normalized', 'FontSize', FS - 4, 'FontWeight', 'bold');
        if iw == 1; ylabel(ax, [WB HAT '_s  (10-seed mean)  [R]'], 'FontSize', FS, 'FontWeight', 'bold'); end
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.8);  grid(ax, 'off');
    end
    out = fullfile(od, 'fig_1009_descent_arms.png');
    exportgraphics(f, out, 'Resolution', 130);  fprintf('figure: %s\n', out);
end
