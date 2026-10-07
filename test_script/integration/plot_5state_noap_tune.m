% PURPOSE (2026-09-21): figure of run_5state_noap_tune.m, house style; one column, three rows: relative error e_a/a per arm
%   (the near-wall reading), rate per arm with the true increment, sqrt(P44)/a per arm (belief) with the seed sd of the
%   first arm dashed. Output reference/eq17_analysis/derivation/figures/fig_noap_tune_Td<T>.png
%  STATUS: ACTIVE (moved from scratch 2026-09-22; 0920_5state_noaprime.tex depends on it)
function plot_5state_noap_tune(Td, arms, labs)
    if nargin < 1 || isempty(Td); Td = 10; end
    if nargin < 2 || isempty(arms); arms = {'ljung', 'ljung_p05', 'ljung_c995', 'ljung_exact', 'q55_p05'}; end
    if nargin < 3 || isempty(labs); labs = {'Ljung (base)', 'Ljung, sqrtP_{44}[0] = 0.05', 'Ljung, clamp 0.995', 'Ljung, exact slope', 'Q_{55} budget, P_{44}[0] = 0.05'}; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    COL_TRUE = [0.8 0 0]; ARM = {[0 0.2 0.9], [0.85 0.45 0], [0.1 0.6 0.2], [0.55 0.1 0.7], [0.1 0.6 0.7]}; FS = 18; LFS = 13; AXLW = 2.0;
    R = cellfun(@(n) load(fullfile(id, sprintf('tune_Td%d_%s.mat', Td, n))), arms, 'UniformOutput', false);
    t = R{1}.t;  seg = R{1}.seg([1 2]);  a = mean(R{1}.a_true, 2);
    f = figure('Position', [60 60 1400 1100], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    nexttile; hold on; box on;  hs = gobjects(0);
    h0 = plot(NaN, NaN, 'w.', 'DisplayName', sprintf('ramp 15 -> 2.5 um over %d s, init a(h_0), 5 seeds', Td));
    for i = 1:numel(arms); hs(end + 1) = plot(t, mean(R{i}.a_true - R{i}.a_hat, 2) ./ a, '-', 'Color', ARM{i}, 'LineWidth', 1.5, 'DisplayName', labs{i}); end %#ok<AGROW>
    yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off'); local_seg(seg);
    xlim([0 t(end)]); ylim([-0.3 0.3]); ylabel('e_a / a  (relative)', 'FontSize', FS, 'FontWeight', 'bold');
    legend([h0 hs], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'NumColumns', 3); local_ax(FS, AXLW);
    nexttile; hold on; box on;  hs = gobjects(0);
    hs(end + 1) = plot(t, movmean(mean(R{1}.da_true, 2), 401) * 1e3, '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'true increment (0.25 s mean)'); %#ok<AGROW>
    for i = 1:numel(arms); hs(end + 1) = plot(t, mean(R{i}.da_hat, 2) * 1e3, '-', 'Color', ARM{i}, 'LineWidth', 1.4, 'DisplayName', ['\delta a_{hat}, ' labs{i}]); end %#ok<AGROW>
    yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off'); local_seg(seg);
    xlim([0 t(end)]); ylim([-0.3 0.3]); ylabel('\delta a / a_{nom}  [10^{-3} / step]', 'FontSize', FS, 'FontWeight', 'bold');
    legend(hs, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'NumColumns', 3); local_ax(FS, AXLW);
    nexttile; hold on; box on;  hs = gobjects(0);
    for i = 1:numel(arms); hs(end + 1) = plot(t, mean(sqrt(R{i}.P_a), 2) ./ a, '-', 'Color', ARM{i}, 'LineWidth', 1.4, 'DisplayName', ['sqrt(P_{44})/a, ' labs{i}]); end %#ok<AGROW>
    for i = 1:numel(arms); plot(t, std(R{i}.a_true - R{i}.a_hat, 0, 2) ./ a, ':', 'Color', ARM{i}, 'LineWidth', 1.2, 'HandleVisibility', 'off'); end
    hs(end + 1) = plot(NaN, NaN, ':', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.2, 'DisplayName', 'dotted: \sigma_{seed}/a (actual)'); %#ok<AGROW>
    local_seg(seg); set(gca, 'YScale', 'log'); xlim([0 t(end)]); ylim([1e-3 1]);
    ylabel('relative uncertainty', 'FontSize', FS, 'FontWeight', 'bold'); xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    legend(hs, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'NumColumns', 3); local_ax(FS, AXLW);
    fn = fullfile(fd, sprintf('fig_noap_tune_Td%d.png', Td));
    exportgraphics(f, fn, 'Resolution', 150);  copyfile(fn, fullfile(id, sprintf('fig_noap_tune_Td%d.png', Td)));  close(f);  fprintf('figure: %s\n', fn);
end
function local_seg(seg)
    for s = seg; xline(s, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
end
function local_ax(FS, AXLW)
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
end
