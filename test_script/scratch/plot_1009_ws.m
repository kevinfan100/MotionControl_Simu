% STATUS: SCRATCH -- 2026-10-11, figures of 1009_conly_wall_position.tex. FORK OF test_script/scratch/plot_mc_s5.m ('s6_<wall>') @ 52df4c0 |
%   PURPOSE: motion_ctrl Fig. 6.3 / 6.4 layout with a w_s row, one scenario per figure, columns plane | cell | EXPIRES: when the w_s
%   line is merged into motion_ctrl or closed | production changes do not follow automatically.
%   Estimator: c and w_s (run_1009_ws arm 'ws'): q55 = P_fix^2 I, q66 = 0. scen = 'fixtop' | 'fixbot' | 'desc' (run_1009_ws.m).
%   Rows: (1) z_d (red) and z (blue dashed) [um]; (2) a-bar_z true and estimate; (3) c-hat and c(h_d) = 1 - h_d^2 a'(h_d) of the wall
%   (red); (4) w_s-hat and the wall position set in the plant, w_s = 0 (red); (5) delta z_m [um]. Data: test_results/temp_1009_ws.
function plot_1009_ws(seed, scen, arm)
    if nargin < 1; seed = 1; end
    if nargin < 2; scen = 'desc'; end
    if nargin < 3; arm = 'ws'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    id = fullfile(root, 'test_results', 'temp_1009_ws');
    WL = {'plane', 'cell'};
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  FS = 22;  LFS = 17;  AXLW = 2.2;  R_UM = 2.25;
    AB = char(257);  HAT = char(770);  az = [AB '_z'];  WB = ['w' char(772)];
    nR = 5;
    f = figure('Position', [40 40 1600 500 * nR], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(nR, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:2
        r = local_load(id, sprintf('%s_%s_%s', scen, arm, WL{ic}), seed);  t = r.t;  ax = gobjects(nR, 1);
        ax(1) = nexttile(tl, ic);  hold on; box on;
        hd = plot(ax(1), t, r.h_d * R_UM, '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', 'z_d');
        hz = plot(ax(1), t, r.h * R_UM, '--', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'z');
        ylim(ax(1), [0 55]);
        legend(ax(1), [hd hz], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        ax(2) = nexttile(tl, 2 + ic);  hold on; box on;
        ht = plot(ax(2), t, r.a_true, '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', az);
        hh = plot(ax(2), t, r.a_hat, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HAT '_z']);
        ylim(ax(2), [-0.2 1.6]);
        legend(ax(2), [ht hh], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        ax(3) = nexttile(tl, 4 + ic);  hold on; box on;
        ht3 = plot(ax(3), t, r.c_true, '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', 'c');
        hc = plot(ax(3), t, r.c_hat, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', ['c' HAT]);
        ylim(ax(3), [-0.6 1.2]);
        legend(ax(3), [ht3 hc], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        ax(4) = nexttile(tl, 6 + ic);  hold on; box on;
        ht4 = plot(ax(4), t, zeros(size(t)), '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', [WB '_s']);
        hw = plot(ax(4), t, r.w_hat, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [WB HAT '_s']);
        ylim(ax(4), [-1.6 1.0]);
        legend(ax(4), [ht4 hw], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        ax(5) = nexttile(tl, 8 + ic);  hold on; box on;
        plot(ax(5), t, r.dxm, '-', 'Color', cE, 'LineWidth', 1.0);
        ylim(ax(5), [-0.2 0.2]);  yticks(ax(5), [-0.1 0 0.1]);
        xlabel(ax(5), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        for q = 1:nR; xlim(ax(q), [0 round(t(end), 1)]);  set(ax(q), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax(q), 'off'); end
        if ic == 1
            ylabel(ax(1), ['z  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
            ylabel(ax(2), az, 'FontSize', FS + 2, 'FontWeight', 'bold');
            ylabel(ax(3), 'c', 'FontSize', FS + 2, 'FontWeight', 'bold');
            ylabel(ax(4), [WB '_s'], 'FontSize', FS + 2, 'FontWeight', 'bold');
            ylabel(ax(5), [char(948) 'z_m  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
        end
        % console: a-bar error, the two constants and their sqrt(P), the information on c, the slope of the estimated pair vs the wall
        e = r.a_true - r.a_hat;  TK = [0.25 0.5 0.75 0.9 1.0 1.5 t(end)];
        pick = @(x, v) v(find(t >= x - 1e-9, 1));
        Ic = cumsum(r.q55) / r.P_fix^2;                                  % accumulated information on c (q55 = P_fix^2 I)
        fprintf('\n[%s %s %s] seed %d | min h %.4f | w_hat range [%+.2f %+.2f] | c_hat range [%+.2f %+.2f]\n', scen, arm, WL{ic}, seed, ...
                min(r.h), min(r.w_hat), max(r.w_hat), min(r.c_hat), max(r.c_hat));
        fprintf('   t     h_d    e_a     c_hat  c(h_d) sqrtPc  sumI_c | w_hat  sqrtPw | slope pair/true at h_d\n');
        for x = TK
            i = find(t >= x - 1e-9, 1);  if isempty(i); i = numel(t); end
            sr = (1 - r.c_hat(i)) / (r.h_d(i) - r.w_hat(i))^2 / r.ap_d(i);
            fprintf('  %4.2f  %5.2f  %+.3f  %+.3f  %+.3f  %.3f  %7.1f | %+.3f  %.3f  | %.3f\n', t(i), r.h_d(i), e(i), r.c_hat(i), r.c_true(i), ...
                    r.sP_c(i), Ic(i), r.w_hat(i), r.sP_w(i), sr);
        end
        HS = [1.10 1.5 2 3 5 10];  apT = local_ap(HS, WL{ic});
        fprintf('  end pair (c, w_s) = (%+.3f, %+.3f): slope ratio pair/true at h = %s -> %s\n', r.c_hat(end), r.w_hat(end), mat2str(HS), ...
                mat2str((1 - r.c_hat(end)) ./ (HS - r.w_hat(end)).^2 ./ apT, 3));
        fprintf('  motion window: mean |e_a| (t <= 1 s) %.4f | worst %.4f | last 0.5 s mean e_a %+.4f\n', mean(abs(e(t <= 1))), max(abs(e(t <= 1))), mean(e(t >= t(end) - 0.5)));
        unused = pick; %#ok<NASGU>
    end
    out = fullfile(fd, sprintf('fig_1009_ws_%s_s%d.png', scen, seed));
    exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end

function r = local_load(id, name, seed)
    S = load(fullfile(id, [name '.mat']));  q = find(S.seeds == seed, 1);
    wd = S.wd(:);  if wd(1) == 0; wd(1) = wd(2); end
    r = struct('t', S.t(:), 'h_d', wd, 'h', S.h_true(:, q), 'a_true', S.a_true(:, q), 'a_hat', S.a_hat(:, q), 'dxm', S.dxm(:, q), ...
               'c_hat', S.c_hat(:, q), 'c_true', S.c_eff_d(:), 'w_hat', S.w_hat(:, q), 'sP_c', S.sP_c(:, q), 'sP_w', S.sP_w(:, q), ...
               'q55', S.q55(:, q), 'P_fix', S.P_fix, 'ap_d', S.ap_d(:));
    if r.c_true(1) > r.c_true(2) + 0.1 || r.c_true(1) < r.c_true(2) - 0.1; r.c_true(1) = r.c_true(2); r.ap_d(1) = r.ap_d(2); end   % first sample uses h_d = 0
end

function ap = local_ap(h, wall)
    d = 1e-4;  ap = zeros(size(h));
    for i = 1:numel(h)
        if strcmp(wall, 'plane')
            [~, cp] = calc_correction_functions(h(i) + d);  ap1 = 1 / cp;  [~, cp] = calc_correction_functions(h(i) - d);  ap(i) = (ap1 - 1 / cp) / (2 * d);
        else
            ap(i) = (1 / cell_cperp_delta(h(i) + d) - 1 / cell_cperp_delta(h(i) - d)) / (2 * d);
        end
    end
end
