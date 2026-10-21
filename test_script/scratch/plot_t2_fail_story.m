% STATUS: SCRATCH -- 2026-10-21 (user: "失敗的那一組是怎麼樣分析 你可以展開畫給我看"). One failed run of run_t2_3d_reg shown as a chain of
%   phases (light bands, numbered at the top): rows theta | a_T, a_N | a'_N (signed, linear) | single-sample readout a_Nm (|.|, log,
%   dots) | h [R] | regulation step Delta h_d [R/step]. True red, estimates blue / green.
%   -> test_results/temp_3d_wsonly/fig_t2_fail_story_<tag>_s<seed>.png | run: plot_t2_fail_story('ng_uvnl_a', 5, [0 0.02 0.08 0.09 0.243 0.258])
function plot_t2_fail_story(tag, seed, tb)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    S = load(fullfile(od, ['temp_t2_' tag '.mat']));  sd = cellfun(@(q) q.seed, S.R.runs);  r = S.R.runs{find(sd == seed, 1)};
    ty = S.R.opts.theta_y;  k = find(r.t <= tb(end));  k = k(k <= r.kend);  t = r.t(k);
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cGr = [0 0.55 0.2];  cS = [0.93 0.93 0.93];  FS = 18;  LFS = 14;
    AB = char(257);  HT = char(770);  TH = char(952);  DG = char(176);  PR = char(8242);  DL = char(916);
    PH = arrayfun(@(c) char(c), 9312:9320, 'UniformOutput', false);   % circled 1-9
    f = figure('Position', [40 40 1500 1900], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(6, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    YL = {[-60 60], [0.6 1.2], [-0.7 0.5], [1e-3 1e2], [0 25], [-0.3 0.05]};
    for row = 1:6
        ax = nexttile(tl, row);  hold on;  box on;
        for ip = 1:2:numel(tb) - 1                            % shade phases 1 and 3 (2 and 4 stay white)
            patch([tb(ip) tb(ip + 1) tb(ip + 1) tb(ip)], [YL{row}(1) YL{row}(1) YL{row}(2) YL{row}(2)], cS, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        end
        switch row
            case 1
                h1 = plot(t, ty + 0 * t, '-', 'Color', cT, 'LineWidth', 3, 'DisplayName', [TH '_y,  ' TH '_x']);
                plot(t, 0 * t, '-', 'Color', cT, 'LineWidth', 3, 'HandleVisibility', 'off');
                h2 = plot(t, r.thy(k), '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [TH HT '_y']);
                h3 = plot(t, r.thx(k), '-', 'Color', cGr, 'LineWidth', 1.8, 'DisplayName', [TH HT '_x']);
                hh = [h1 h2 h3];  yl = [TH '  [' DG ']'];
                for ip = 1:numel(tb) - 1
                    text((tb(ip) + tb(ip + 1)) / 2, 50, PH{ip}, 'HorizontalAlignment', 'center', 'FontSize', FS + 4, 'FontWeight', 'bold');
                end
            case 2
                h1 = plot(t, r.aT_true(k), '-', 'Color', cT, 'LineWidth', 3, 'DisplayName', [AB '_T,  ' AB '_N']);
                plot(t, r.aN_true(k), '-', 'Color', cT, 'LineWidth', 3, 'HandleVisibility', 'off');
                h2 = plot(t, r.aT(k), '-', 'Color', cGr, 'LineWidth', 1.8, 'DisplayName', [AB HT '_T']);
                h3 = plot(t, r.aN(k), '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HT '_N']);
                hh = [h1 h2 h3];  yl = AB;
            case 3
                h1 = plot(t, r.apN_true(k), '-', 'Color', cT, 'LineWidth', 3, 'DisplayName', [AB PR '_N']);
                h2 = plot(t, r.apN(k), '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HT PR '_N']);
                plot(t, 0 * t, ':', 'Color', [0 0 0], 'LineWidth', 1, 'HandleVisibility', 'off');
                hh = [h1 h2];  yl = [AB PR '_N'];
            case 4
                h1 = plot(t, r.aN_true(k), '-', 'Color', cT, 'LineWidth', 3, 'DisplayName', [AB '_N']);
                h2 = plot(t, abs(r.aNm(k)), '.', 'Color', cE, 'MarkerSize', 7, 'DisplayName', ['|' AB '_{Nm}| (one sample)']);
                set(ax, 'YScale', 'log');  hh = [h1 h2];  yl = ['|' AB '_{Nm}|'];
            case 5
                h1 = plot(t, r.h(k), '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'h');
                hh = h1;  yl = 'h  [R]';
            case 6
                h1 = plot(t, r.dhd(k), '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [DL 'h_d']);
                hh = h1;  yl = [DL 'h_d  [R/step]'];  xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        end
        ylim(YL{row});  xlim([tb(1) tb(end)]);  ylabel(yl, 'FontSize', FS, 'FontWeight', 'bold');
        legend(hh, 'Location', 'eastoutside', 'FontSize', LFS, 'FontWeight', 'bold');
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2, 'Layer', 'top');
    end
    out = fullfile(od, sprintf('fig_t2_fail_story_%s_s%d.png', tag, seed));
    exportgraphics(f, out, 'Resolution', 110);  close(f);  fprintf('figure: %s\n', out);
end
