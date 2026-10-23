% STATUS: SCRATCH -- 2026-10-15, figures for run_3d_wsonly_flat in the house style of plot_mc_s8_b (seed 1, Unicode a-bar,
%   bold, true = thick red, estimate = blue, no grid / title / guide lines). No lateral motion (lat 0).
%   which = 'main': arm joint, columns plane | cell (plane slid down 1.05 R, both c_par and c_perp), rows
%                   z | a-bar_N | a-bar_T | e_a-bar (N, T) | w-bar_s-hat (+- sqrt P) | delta x_m, delta z_m
%                   -> test_results/temp_3d_wsonly/fig_3d_flat_main_s1.png
%   which = 'cmp' : one figure per wall, columns joint | one-way, rows a-bar_N | a-bar_T | e_a-bar | w-bar_s-hat
%                   -> fig_3d_flat_cmp_<wall>_s1.png
%   which = 'cmpnum' (10-16): arm joint, one figure per wall, columns numerator 9/16 | 1 of the tangential slope law, rows as 'cmp'
%                   -> fig_3d_flat_cmpnum_<wall>_s1.png ; tag (2nd arg) picks the .mat for 'main' / 'cmp' ('_n1' default, '_n916')
function plot_3d_wsonly_flat(which, tag)
    if nargin < 1; which = 'main'; end
    if nargin < 2; tag = '_n1'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    S = load(fullfile(od, ['temp_3d_wsonly_runs' tag '.mat']));  runs = [S.R.runs{:}];
    if strcmp(which, 'cmpnum')
        S2 = load(fullfile(od, 'temp_3d_wsonly_runs_n916.mat'));  r916 = [S2.R.runs{:}];
        S1 = load(fullfile(od, 'temp_3d_wsonly_runs_n1.mat'));    r1 = [S1.R.runs{:}];
    end
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cB = [0.75 0.85 1];  cG = [0 0.55 0.2];  cL = [0.45 0.65 1];
    FS = 22;  LFS = 17;  R_UM = 2.25;
    AB = char(257);  HAT = char(770);  WB = ['w' char(772)];  DL = char(948);  MU = char(181);
    pick = @(wall, arm) runs(strcmp({runs.wall}, wall) & [runs.lat] == 0 & [runs.seed] == 1 & strcmp({runs.arm}, arm));
    WN = {'plane', 'shift'};  WT = {'plane', 'cell (plane - 1.05 R)'};
    if strcmp(which, 'main')
        f = figure('Position', [40 40 1600 2600], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
        tl = tiledlayout(6, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
        for ic = 1:2
            r = pick(WN{ic}, 'joint');  t = r.t;  ax = gobjects(6, 1);
            ax(1) = nexttile(tl, ic);  hold on; box on;
            h1 = plot(ax(1), t, r.pd(:, 3), '-', 'Color', cT, 'LineWidth', 5, 'DisplayName', 'z_d');
            h2 = plot(ax(1), t, r.p(:, 3), '--', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'z');
            ylim(ax(1), [0 55]);  legend(ax(1), [h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            text(ax(1), 0.96, 0.9, WT{ic}, 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', FS, 'FontWeight', 'bold');
            ax(2) = nexttile(tl, 2 + ic);  hold on; box on;
            h3 = plot(ax(2), t, r.aN_true, '-', 'Color', cT, 'LineWidth', 5, 'DisplayName', [AB '_N']);
            h4 = plot(ax(2), t, r.aN, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HAT '_N']);
            ylim(ax(2), [-0.2 1.2]);  legend(ax(2), [h3 h4], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            ax(3) = nexttile(tl, 4 + ic);  hold on; box on;
            h5 = plot(ax(3), t, r.aT_true, '-', 'Color', cT, 'LineWidth', 5, 'DisplayName', [AB '_T']);
            h6 = plot(ax(3), t, r.aT, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HAT '_T']);
            ylim(ax(3), [-0.2 1.2]);  legend(ax(3), [h5 h6], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            ax(4) = nexttile(tl, 6 + ic);  hold on; box on;
            h7 = plot(ax(4), t, r.aN_true - r.aN, '-', 'Color', cE, 'LineWidth', 1.5, 'DisplayName', ['e_{' AB 'N}']);
            h8 = plot(ax(4), t, r.aT_true - r.aT, '-', 'Color', cG, 'LineWidth', 1.5, 'DisplayName', ['e_{' AB 'T}']);
            ylim(ax(4), [-0.1 0.1]);  legend(ax(4), [h7 h8], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            ax(5) = nexttile(tl, 8 + ic);  hold on; box on;
            sp = r.sP(:, 3);  wh = r.ws;
            fill(ax(5), [t; flipud(t)], [wh + sp; flipud(wh - sp)], cB, 'EdgeColor', 'none', 'HandleVisibility', 'off');
            h9 = plot(ax(5), t, r.ws_true + 0 * t, '-', 'Color', cT, 'LineWidth', 5, 'DisplayName', [WB '_s']);
            h10 = plot(ax(5), t, wh, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [WB HAT '_s']);
            ylim(ax(5), [-2 1]);  legend(ax(5), [h9 h10], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            ax(6) = nexttile(tl, 10 + ic);  hold on; box on;
            h11 = plot(ax(6), t, r.dpm(:, 1), '-', 'Color', cL, 'LineWidth', 1.0, 'DisplayName', [DL 'x_m']);
            h12 = plot(ax(6), t, r.dpm(:, 3), '-', 'Color', cE, 'LineWidth', 1.0, 'DisplayName', [DL 'z_m']);
            ylim(ax(6), [-0.2 0.2]);  yticks(ax(6), [-0.1 0 0.1]);  xlabel(ax(6), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
            legend(ax(6), [h11 h12], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            for k = 1:6; xlim(ax(k), [0 3]);  set(ax(k), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2.2);  grid(ax(k), 'off'); end
            if ic == 1
                ylabel(ax(1), ['z  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax(2), [AB '_N'], 'FontSize', FS + 2, 'FontWeight', 'bold');
                ylabel(ax(3), [AB '_T'], 'FontSize', FS + 2, 'FontWeight', 'bold');  ylabel(ax(4), ['e_{' AB '}'], 'FontSize', FS + 2, 'FontWeight', 'bold');
                ylabel(ax(5), [WB '_s  [R]'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax(6), [DL 'p_m  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
            end
        end
        out = fullfile(od, ['fig_3d_flat_main' tag '_s1.png']);
        exportgraphics(f, out, 'Resolution', 140);  fprintf('figure: %s\n', out);  close(f);
    else
        AR = {'joint', 'oneway'};  AT = {'joint', 'one-way'};
        if strcmp(which, 'cmpnum'); AT = {'9/16', '1'}; end
        for iw = 1:2
            f = figure('Position', [40 40 1600 1900], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
            tl = tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
            for ic = 1:2
                if strcmp(which, 'cmpnum')
                    src = {r916, r1};  rr = src{ic};
                    r = rr(strcmp({rr.wall}, WN{iw}) & [rr.lat] == 0 & [rr.seed] == 1 & strcmp({rr.arm}, 'joint'));
                else
                    r = pick(WN{iw}, AR{ic});
                end
                t = r.t;  ax = gobjects(4, 1);
                ax(1) = nexttile(tl, ic);  hold on; box on;
                h1 = plot(ax(1), t, r.aN_true, '-', 'Color', cT, 'LineWidth', 5, 'DisplayName', [AB '_N']);
                h2 = plot(ax(1), t, r.aN, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HAT '_N  (' AT{ic} ')']);
                ylim(ax(1), [-0.2 1.2]);  legend(ax(1), [h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
                text(ax(1), 0.96, 0.9, WT{iw}, 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', FS, 'FontWeight', 'bold');
                ax(2) = nexttile(tl, 2 + ic);  hold on; box on;
                h3 = plot(ax(2), t, r.aT_true, '-', 'Color', cT, 'LineWidth', 5, 'DisplayName', [AB '_T']);
                h4 = plot(ax(2), t, r.aT, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HAT '_T  (' AT{ic} ')']);
                ylim(ax(2), [-0.2 1.2]);  legend(ax(2), [h3 h4], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
                ax(3) = nexttile(tl, 4 + ic);  hold on; box on;
                h5 = plot(ax(3), t, r.aN_true - r.aN, '-', 'Color', cE, 'LineWidth', 1.5, 'DisplayName', ['e_{' AB 'N}']);
                h6 = plot(ax(3), t, r.aT_true - r.aT, '-', 'Color', cG, 'LineWidth', 1.5, 'DisplayName', ['e_{' AB 'T}']);
                ylim(ax(3), [-0.1 0.3]);  legend(ax(3), [h5 h6], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
                ax(4) = nexttile(tl, 6 + ic);  hold on; box on;
                sp = r.sP(:, 3);  wh = r.ws;
                fill(ax(4), [t; flipud(t)], [wh + sp; flipud(wh - sp)], cB, 'EdgeColor', 'none', 'HandleVisibility', 'off');
                h7 = plot(ax(4), t, r.ws_true + 0 * t, '-', 'Color', cT, 'LineWidth', 5, 'DisplayName', [WB '_s']);
                h8 = plot(ax(4), t, wh, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [WB HAT '_s']);
                ylim(ax(4), [-2 1]);  xlabel(ax(4), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
                legend(ax(4), [h7 h8], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
                for k = 1:4; xlim(ax(k), [0 3]);  set(ax(k), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2.2);  grid(ax(k), 'off'); end
                if ic == 1
                    ylabel(ax(1), [AB '_N'], 'FontSize', FS + 2, 'FontWeight', 'bold');  ylabel(ax(2), [AB '_T'], 'FontSize', FS + 2, 'FontWeight', 'bold');
                    ylabel(ax(3), ['e_{' AB '}'], 'FontSize', FS + 2, 'FontWeight', 'bold');  ylabel(ax(4), [WB '_s  [R]'], 'FontSize', FS, 'FontWeight', 'bold');
                end
            end
            out = fullfile(od, sprintf('fig_3d_flat_cmp_%s_s1.png', WN{iw}));
            if strcmp(which, 'cmpnum'); out = fullfile(od, sprintf('fig_3d_flat_cmpnum_%s_s1.png', WN{iw})); end
            exportgraphics(f, out, 'Resolution', 140);  fprintf('figure: %s\n', out);  close(f);
        end
    end
end
