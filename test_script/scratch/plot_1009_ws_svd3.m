% STATUS: SCRATCH -- 2026-10-11, figure for probe_1009_ws_svd3_track.m (descent 22.2 -> 1.10 R in 1 s, parked 0.2 s; c and w_s; seed 1).
%   Columns plane | cell. Rows: (1) h_d [R], linear axis (constant velocity = straight line); (2) sigma_4, sigma_5, sigma_6 of the 3-step
%   O(k) at every k, log axis; (3)-(5) the a, c, w_s components of v4, v5, v6 (their dx1..3 components are ~0, printed by the probe).
%   Sign of each v_i fixed so that its a / c / w_s component (v4 / v5 / v6) is positive. Optional (SHOW_FORMULA, off by default,
%   user 10-11) row 5 red: v6 from the leading-order O(k), [0, k, 1] / sqrt(1 + k^2), k = 2 (1 - c-hat) / (h_d - w_s-hat).
function plot_1009_ws_svd3(SHOW_FORMULA, prefix, outname, ylim_sig, lab4)
    if nargin < 1 || isempty(SHOW_FORMULA); SHOW_FORMULA = false; end   % user 10-11: no red formula line for now
    if nargin < 2 || isempty(prefix); prefix = 'svd3'; end              % 'svdcum' = window growing from t = 0 (probe_1009_ws_svd_cum.m)
    if nargin < 4 || isempty(ylim_sig); ylim_sig = [1e-10 1e2]; end
    if nargin < 5 || isempty(lab4); lab4 = '4'; end                    % cumulative window: the a-direction is not the 4th sigma -> label 'a'
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_1009_ws');
    fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    WL = {'plane', 'cell'};
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cL = [0.3 0.75 0.95];  cW = [0 0.6 0.25];  FS = 22;  LFS = 17;  AXLW = 2.2;   % cW: w_s component, green (user 10-11)
    S6 = char(963);  AB = char(257);  WB = ['w' char(772)];  HAT = char(770);
    nR = 5;
    f = figure('Position', [40 40 1600 450 * nR], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(nR, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:2
        r = load(fullfile(od, sprintf('%s_%s.mat', prefix, WL{ic})));  t = r.t;  ax = gobjects(nR, 1);
        ax(1) = nexttile(tl, ic);  hold on; box on;
        plot(ax(1), t, r.hd, '-', 'Color', 'k', 'LineWidth', 2.5);
        ylim(ax(1), [0 24]);  yticks(ax(1), [1.1 5 10 15 20]);
        ax(2) = nexttile(tl, 2 + ic);  hold on; box on;
        h4 = plot(ax(2), t, r.sig(:, 4), '-', 'Color', 'k', 'LineWidth', 2.5, 'DisplayName', [S6 '_' lab4]);
        h5 = plot(ax(2), t, r.sig(:, 5), '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [S6 '_5']);
        h6 = plot(ax(2), t, r.sig(:, 6), '-', 'Color', cL, 'LineWidth', 1.8, 'DisplayName', [S6 '_6']);
        set(ax(2), 'YScale', 'log', 'YTick', 10.^(-10:2:4));  ylim(ax(2), ylim_sig);
        legend(ax(2), [h4 h5 h6], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        V = {r.v4, r.v5, r.v6};  VN = {['v_' lab4], 'v_5', 'v_6'};  if ~strcmp(lab4, '4'); VN{1} = ['v (' AB ')']; end
        k = r.alias;  pc = k ./ sqrt(1 + k.^2);  pw = 1 ./ sqrt(1 + k.^2);
        for q = 1:3
            a = nexttile(tl, 2 * (q + 1) + ic);  hold on; box on;  ax(q + 2) = a;  hh = gobjects(0);
            if q == 3 && SHOW_FORMULA
                hh(end + 1) = plot(a, t, pc, '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', 'v_6 formula');   % (0, k, 1)/sqrt(1 + k^2), k = 2(1 - c-hat)/(h_d - w_s-hat)
                plot(a, t, pw, '-', 'Color', cT, 'LineWidth', 5.0, 'HandleVisibility', 'off');
            end
            hh(end + 1) = plot(a, t, V{q}(:, 4), '-', 'Color', 'k', 'LineWidth', 1.8, 'DisplayName', AB);   %#ok<AGROW>
            hh(end + 1) = plot(a, t, V{q}(:, 5), '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'c');   %#ok<AGROW>
            hh(end + 1) = plot(a, t, V{q}(:, 6), '-', 'Color', cW, 'LineWidth', 1.8, 'DisplayName', [WB '_s']);   %#ok<AGROW>
            yline(a, 0, '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 1, 'HandleVisibility', 'off');
            ylim(a, [-1.1 1.1]);  yticks(a, [-1 -0.5 0 0.5 1]);
            legend(a, hh, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            if ic == 1; ylabel(a, VN{q}, 'FontSize', FS + 2, 'FontWeight', 'bold'); end
        end
        xlabel(ax(nR), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        for q = 1:nR; xlim(ax(q), [0 round(t(end), 1)]);  set(ax(q), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax(q), 'off'); end
        if ic == 1
            ylabel(ax(1), 'h_d  [R]', 'FontSize', FS, 'FontWeight', 'bold');
            ylabel(ax(2), S6, 'FontSize', FS + 2, 'FontWeight', 'bold');
        end
    end
    out = fullfile(fd, 'fig_1009_ws_svd3_desc_s1.png');
    if nargin >= 3 && ~isempty(outname); out = outname; end
    exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
