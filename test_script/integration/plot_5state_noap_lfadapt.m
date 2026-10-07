% PURPOSE (2026-09-21): figure of run_5state_noap_lfadapt.m, house style. One column, four rows: a_true / a_hat of the
%   three arms | e_a = a_true - a_hat per arm (band = pure arm) | rate per arm with the true increment | memory T*(t)
%   of the Ljung arm (log) with the constant arm's 1.25 s. Arms: lambda_f 1 (blue), 0.9995 (orange), Ljung (green).
%   Output reference/eq17_analysis/derivation/figures/fig_noap_lfadapt.png | STATUS: ACTIVE (moved from scratch 2026-09-22; 0920_5state_noaprime.tex depends on it)
function plot_5state_noap_lfadapt(Td)
    if nargin < 1 || isempty(Td); Td = 10; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    COL_TRUE = [0.8 0 0]; ARM = {[0 0.2 0.9], [0.85 0.45 0], [0.1 0.6 0.2]}; BANDC = [0.45 0.55 0.95]; FS = 18; LFS = 13; AXLW = 2.0;
    names = {'lf1', 'lf9995', 'ljung'};  labs = {'\lambda_f = 1 (pure layer 1)', '\lambda_f = 0.9995', 'Ljung schedule (speed x height)'};
    R = cellfun(@(n) load(fullfile(id, sprintf('lfadapt_Td%d_%s.mat', Td, n))), names, 'UniformOutput', false);
    pc = physical_constants();  Ts = pc.Ts;  t = R{1}.t;  seg = R{1}.seg([1 2]);
    f = figure('Position', [60 60 1300 1250], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    nexttile; hold on; box on;  hs = gobjects(0);
    h0 = plot(NaN, NaN, 'w.', 'DisplayName', sprintf('ramp 15 -> 2.5 um over %d s, init a(h_0), 5 seeds', Td));
    hs(end + 1) = plot(t, mean(R{1}.a_true, 2), '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'a_{true}'); %#ok<AGROW>
    for i = 1:3; hs(end + 1) = plot(t, mean(R{i}.a_hat, 2), '-', 'Color', ARM{i}, 'LineWidth', 1.6, 'DisplayName', labs{i}); end %#ok<AGROW>
    local_seg(seg); xlim([0 t(end)]); ylim([0 1.1]); ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
    legend([h0 hs], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on'); local_ax(FS, AXLW);
    nexttile; hold on; box on;  hs = gobjects(0);
    ea = R{1}.a_true - R{1}.a_hat;  em = mean(ea, 2); es = std(ea, 0, 2);
    fill([t; flipud(t)], [em - es; flipud(em + es)], BANDC, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
    for i = 1:3; hs(end + 1) = plot(t, mean(R{i}.a_true - R{i}.a_hat, 2), '-', 'Color', ARM{i}, 'LineWidth', 1.6, 'DisplayName', ['e_a, ' labs{i}]); end %#ok<AGROW>
    yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off'); local_seg(seg);
    xlim([0 t(end)]); ylim([-0.25 0.25]); ylabel('e_a = a_{true} - a_{hat}', 'FontSize', FS, 'FontWeight', 'bold');
    legend(hs, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on'); local_ax(FS, AXLW);
    nexttile; hold on; box on;  hs = gobjects(0);
    hs(end + 1) = plot(t, movmean(mean(R{1}.da_true, 2), 401) * 1e3, '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'true increment (0.25 s mean)'); %#ok<AGROW>
    for i = 1:3; hs(end + 1) = plot(t, mean(R{i}.da_hat, 2) * 1e3, '-', 'Color', ARM{i}, 'LineWidth', 1.6, 'DisplayName', ['\delta a_{hat}, ' labs{i}]); end %#ok<AGROW>
    yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off'); local_seg(seg);
    xlim([0 t(end)]); ylim([-0.4 0.4]); ylabel('\delta a / a_{nom}  [10^{-3} / step]', 'FontSize', FS, 'FontWeight', 'bold');
    legend(hs, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on'); local_ax(FS, AXLW);
    nexttile; hold on; box on;  hs = gobjects(0);
    Tst = Ts ./ max(1 - mean(R{3}.lf, 2), 1e-9);
    hs(end + 1) = plot(t, Tst, '-', 'Color', ARM{3}, 'LineWidth', 1.8, 'DisplayName', 'T* = T_s / (1 - \lambda_f), Ljung'); %#ok<AGROW>
    hs(end + 1) = yline(Ts / (1 - 0.9995), '-', 'Color', ARM{2}, 'LineWidth', 1.6, 'DisplayName', 'T* of \lambda_f = 0.9995'); %#ok<AGROW>
    local_seg(seg); set(gca, 'YScale', 'log'); xlim([0 t(end)]); ylim([0.1 100]);
    ylabel('memory T*  [s]', 'FontSize', FS, 'FontWeight', 'bold'); xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    legend(hs, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on'); local_ax(FS, AXLW);
    fn = fullfile(fd, sprintf('fig_noap_lfadapt_Td%d.png', Td));
    exportgraphics(f, fn, 'Resolution', 150);  copyfile(fn, fullfile(id, sprintf('fig_noap_lfadapt_Td%d.png', Td)));  close(f);  fprintf('figure: %s\n', fn);
end
function local_seg(seg)
    for s = seg; xline(s, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
end
function local_ax(FS, AXLW)
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
end
