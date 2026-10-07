% PURPOSE (2026-09-21): figure of run_5state_noap_slow.m in house style. Columns = descent time T_d (10 / 20 / 40 s);
%   rows: a_true (red) with a_hat of the three lambda_f arms | e_a = a_true - a_hat per arm (seed mean, band for the
%   pure arm only) | rate delta a_hat per arm with the true increment red. Arms: lambda_f 1 (pure layer 1, blue),
%   0.9995 (orange), 0.998 (green). Output reference/eq17_analysis/derivation/figures/fig_noap_slow.png
%  STATUS: ACTIVE (moved from scratch 2026-09-22; 0920_5state_noaprime.tex depends on it)
function plot_5state_noap_slow(T_d, lf)
    if nargin < 1 || isempty(T_d); T_d = [10, 20, 40]; end
    if nargin < 2 || isempty(lf);  lf  = [1, 0.9995, 0.998]; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    COL_TRUE = [0.8 0 0]; ARM = {[0 0.2 0.9], [0.85 0.45 0], [0.1 0.6 0.2]}; BANDC = [0.45 0.55 0.95]; FS = 18; LFS = 13; AXLW = 2.0;
    nC = numel(T_d);
    f = figure('Position', [40 40 800 * nC, 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(3, nC, 'TileSpacing', 'compact', 'Padding', 'compact');
    for c = 1:nC
        R = cell(numel(lf), 1);
        for il = 1:numel(lf)
            R{il} = load(fullfile(id, sprintf('slow_Td%d_lf%s.mat', T_d(c), strrep(sprintf('%.4f', lf(il)), '.', 'p'))));
        end
        t = R{1}.t;  seg = R{1}.seg;  seg = seg([1 2]);
        % row 1
        nexttile(c); hold on; box on;  hs = gobjects(0);
        h0 = plot(NaN, NaN, 'w.', 'DisplayName', sprintf('ramp over %d s, init a(h_0)', T_d(c)));
        hs(end + 1) = plot(t, mean(R{1}.a_true, 2), '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'a_{true}'); %#ok<AGROW>
        for il = 1:numel(lf)
            hs(end + 1) = plot(t, mean(R{il}.a_hat, 2), '-', 'Color', ARM{il}, 'LineWidth', 1.6, 'DisplayName', local_lab(lf(il))); %#ok<AGROW>
        end
        for s = seg; xline(s, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
        xlim([0 t(end)]); ylim([0 1.1]);
        if c == 1; ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([h0 hs], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
        % row 2
        nexttile(nC + c); hold on; box on;  hs = gobjects(0);
        ea = R{1}.a_true - R{1}.a_hat;  em = mean(ea, 2); es = std(ea, 0, 2);
        fill([t; flipud(t)], [em - es; flipud(em + es)], BANDC, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
        for il = 1:numel(lf)
            hs(end + 1) = plot(t, mean(R{il}.a_true - R{il}.a_hat, 2), '-', 'Color', ARM{il}, 'LineWidth', 1.6, 'DisplayName', ['e_a, ' local_lab(lf(il))]); %#ok<AGROW>
        end
        yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
        for s = seg; xline(s, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
        xlim([0 t(end)]); ylim([-0.25 0.25]);
        if c == 1; ylabel('e_a = a_{true} - a_{hat}  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend(hs, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
        % row 3
        nexttile(2 * nC + c); hold on; box on;  hs = gobjects(0);
        hs(end + 1) = plot(t, movmean(mean(R{1}.da_true, 2), 401) * 1e3, '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'true increment (0.25 s mean)'); %#ok<AGROW>
        for il = 1:numel(lf)
            hs(end + 1) = plot(t, mean(R{il}.da_hat, 2) * 1e3, '-', 'Color', ARM{il}, 'LineWidth', 1.6, 'DisplayName', ['\delta a_{hat}, ' local_lab(lf(il))]); %#ok<AGROW>
        end
        yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
        for s = seg; xline(s, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
        xlim([0 t(end)]); ylim([-0.4 0.4]);
        if c == 1; ylabel('\delta a / a_{nom}  [10^{-3} / step]', 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        legend(hs, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    end
    fn = fullfile(fd, 'fig_noap_slow.png');
    exportgraphics(f, fn, 'Resolution', 150);  copyfile(fn, fullfile(id, 'fig_noap_slow.png'));  close(f);  fprintf('figure: %s\n', fn);
end
function s = local_lab(lf)
    if lf >= 1; s = '\lambda_f = 1 (pure layer 1)'; else; s = sprintf('\\lambda_f = %.4g', lf); end
end
