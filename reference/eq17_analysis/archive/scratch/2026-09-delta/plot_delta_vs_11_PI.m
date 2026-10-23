% PURPOSE (2026-09-20, user: "same case against [1/1]; add the Meng trajectory; house style; plot delta, c0, d1 separately
%   with their estimates"): cell plant, estimator started as a PLANE (delta law arm PI: delta_hat[0] = 0; [1/1] version A
%   arm P: plane seed (-0.984, 0.161)), family prior 0.5 / 0.52, same 10 seeds. Columns = trajectories (canon deep 4.8 s |
%   Meng ramp 15 -> 2.5 um over 10 s, hold to 12.5 s). House style (plot_var_ahat_6state.m): True red, Estimate blue,
%   +- sigma_seed band, legend northoutside horizontal, no title, no grid, FontSize 18 bold.
%     row 1  a_bar: true (red), delta-law a_hat (blue), [1/1] a_hat (orange), seed means
%     row 2  e_a = a_true - a_hat, seed mean, delta law (blue) and [1/1] (orange), +- sigma_seed bands
%     row 3  delta_hat  (mean +- sigma_seed, +- sqrt(P) dashed) against the plant delta 1.05 (red)
%     row 4  c0_hat     against the cell band fit 0.0501 (red)      [1/1]
%     row 5  d1_hat     against the cell band fit 1.1871 (red)      [1/1]
%   Files: temp_delta_two_walls/cell_PI{,_meng}.mat, temp_11_two_walls/cell_A_P{,_meng}.mat.
%   Output reference/eq17_analysis/derivation/figures/delta_vs_11_PI.png | EXPIRES: with the delta-law line
function plot_delta_vs_11_PI(darm)
    if nargin < 1 || isempty(darm); darm = 'PI'; end                        % 'PI' one-shot y2 update | 'PI_iekf' iterated (O38)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    rd_d = fullfile(root, 'test_results', 'temp_delta_two_walls');  rd_11 = fullfile(root, 'test_results', 'temp_11_two_walls');
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; C_11 = [0.85 0.45 0]; BANDC = [0.45 0.55 0.95]; BAND11 = [0.95 0.75 0.5];
    FS = 18; LFS = 13; AXLW = 2.0;
    TR = {'', '_meng'};  TRN = {'canon deep', 'Meng ramp'};
    C0_T = 0.0501;  D1_T = 1.1871;                                   % cell band fit (cell_cperp_delta, plot_cell_plant_delta.m)
    f = figure('Position', [80 80 1600 1850], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(5, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:2
        fd = fullfile(rd_d, sprintf('cell_%s%s.mat', darm, TR{ic}));  f11 = fullfile(rd_11, sprintf('cell_A_P%s.mat', TR{ic}));
        if ~exist(fd, 'file') || ~exist(f11, 'file'); fprintf('missing %s or %s\n', fd, f11); continue; end
        D = load(fd);  E = load(f11);  t = D.t;  seg = D.seg;  ns = min(D.ns, E.ns);
        ed = D.a_true(:, 1:ns) - D.a_hat(:, 1:ns);  e11 = E.a_true(:, 1:ns) - E.a_hat(:, 1:ns);
        kd = find(t < seg(2), 1, 'last');  hold_m = t >= seg(3);
        fprintf('\n=== CELL wall, plane init, %s (%d seeds)\n', TRN{ic}, ns);
        fprintf('  e_a hold      : delta law %+.4f +- %.4f | [1/1] %+.4f +- %.4f\n', mean(mean(ed(hold_m, :), 1)), std(mean(ed(hold_m, :), 1)), ...
                mean(mean(e11(hold_m, :), 1)), std(mean(e11(hold_m, :), 1)));
        fprintf('  delta_hat     : seed %.2f -> target %.2f | descent end %.3f | run end %.3f +- %.3f (sqrtP %.3f)\n', D.dl_seed, D.dl_target, ...
                mean(D.dl_hat(kd, :)), mean(D.dl_hat(end, :)), std(D.dl_hat(end, :)), mean(D.sP_dl(end, :)));
        fprintf('  c0_hat  [1/1] : seed %.3f -> target %.3f | descent end %.3f | run end %.3f +- %.3f (sqrtP %.3f)\n', E.seed_cd(1), C0_T, ...
                mean(E.c0_hat(kd, :)), mean(E.c0_hat(end, :)), std(E.c0_hat(end, :)), mean(E.sP_c0(end, :)));
        fprintf('  d1_hat  [1/1] : seed %.3f -> target %.3f | descent end %.3f | run end %.3f +- %.3f (sqrtP %.3f)\n', E.seed_cd(2), D1_T, ...
                mean(E.d1_hat(kd, :)), mean(E.d1_hat(end, :)), std(E.d1_hat(end, :)), mean(E.sP_d1(end, :)));
        % row 1: a_bar true / hat
        nexttile(ic); hold on; box on;
        a1 = plot(t, mean(D.a_true(:, 1:ns), 2), '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'a_{true} / a_{nom}');
        a2 = plot(t, mean(D.a_hat(:, 1:ns), 2), '-', 'Color', COL_HAT, 'LineWidth', 1.8, 'DisplayName', 'a_{hat}, \delta law');
        a3 = plot(t, mean(E.a_hat(:, 1:ns), 2), '-', 'Color', C_11, 'LineWidth', 1.8, 'DisplayName', 'a_{hat}, [1/1]');
        a0 = plot(NaN, NaN, 'w.', 'DisplayName', sprintf('cell wall, plane init%s, %s', strrep(strrep(darm, 'PI', ''), '_', ', '), TRN{ic}));
        local_seg(seg);  xlim([0 t(end)]); ylim([0 1]);
        if ic == 1; ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([a0 a1 a2 a3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_ax(FS, AXLW);
        % row 2: e_a
        nexttile(2 + ic); hold on; box on;
        local_band(t, e11, BAND11);  local_band(t, ed, BANDC);
        h1 = plot(t, mean(ed, 2), '-', 'Color', COL_HAT, 'LineWidth', 2.0, 'DisplayName', 'e_a, \delta law');
        h2 = plot(t, mean(e11, 2), '-', 'Color', C_11, 'LineWidth', 2.0, 'DisplayName', 'e_a, [1/1]');
        yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
        local_seg(seg);  xlim([0 t(end)]); ylim([-0.06 0.10]);
        if ic == 1; ylabel('e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_ax(FS, AXLW);
        % rows 2-4: the constants, each against its target
        rows = {D.dl_hat(:, 1:ns), D.sP_dl(:, 1:ns), D.dl_target, '\delta_{hat}', '\delta  (plant)', '\delta  [R]', [-0.6 1.6], COL_HAT, BANDC; ...
                E.c0_hat(:, 1:ns), E.sP_c0(:, 1:ns), C0_T, 'c_{0,hat}  [1/1]', 'c_0  (cell fit)', 'c_0', [-1.3 0.6], C_11, BAND11; ...
                E.d1_hat(:, 1:ns), E.sP_d1(:, 1:ns), D1_T, 'd_{1,hat}  [1/1]', 'd_1  (cell fit)', 'd_1', [-0.2 1.9], C_11, BAND11};
        for r = 1:3
            nexttile(2 * (r + 1) + ic); hold on; box on;
            x = rows{r, 1}; sp = rows{r, 2}; tg = rows{r, 3}; col = rows{r, 8}; bc = rows{r, 9};
            m = mean(x, 2); s = std(x, 0, 2); p = mean(sp, 2);
            fill([t; flipud(t)], [m - s; flipud(m + s)], bc, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
            g1 = plot(t, m, '-', 'Color', col, 'LineWidth', 2.0, 'DisplayName', rows{r, 4});
            g2 = plot(t, m + p, '--', 'Color', col, 'LineWidth', 1.0, 'DisplayName', '\pm sqrt(P)');
            plot(t, m - p, '--', 'Color', col, 'LineWidth', 1.0);
            g3 = yline(tg, '-', 'Color', COL_TRUE, 'LineWidth', 1.6, 'DisplayName', rows{r, 5});
            local_seg(seg);  xlim([0 t(end)]); ylim(rows{r, 7});
            if ic == 1; ylabel(rows{r, 6}, 'FontSize', FS, 'FontWeight', 'bold'); end
            if r == 3; xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
            legend([g1 g2 g3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
            local_ax(FS, AXLW);
        end
    end
    fn = fullfile(od, sprintf('delta_vs_11_%s.png', darm));
    exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
end

function local_band(t, e, col)
    m = mean(e, 2); s = std(e, 0, 2);
    fill([t; flipud(t)], [m - s; flipud(m + s)], col, 'FaceAlpha', 0.20, 'EdgeColor', 'none', 'HandleVisibility', 'off');
end

function local_seg(seg)
    for s = unique(seg); xline(s, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
end

function local_ax(FS, AXLW)
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
end
