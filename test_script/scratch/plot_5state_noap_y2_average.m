% PURPOSE (2026-09-29, user: "y2 only, Q = 0: explain the lag, the fixed point and T_d, obvious at a glance"):
%   first order, lambda_f = 1, plane, seeds 1:3, hold 0.5 s -> 22.2 -> 1.10 R at constant velocity in T_d -> hold 2 s.
%   Black dashed = the y2-only average of 0929_kf_two_tracks.tex 4.6 with the readings replaced by the true gain:
%     a_avg[k] = ( a_hat[0]/P_aa[0] + sum_j w_j a_true[j] ) / ( 1/P_aa[0] + sum_j w_j ),   w_j = H2(4)^2/R2[j] (diag.w_y2)
%   i.e. the filter's own y2 weights, no y1, no a_m bias.  Rows: gain | e_a = a_true - a_hat around the stop.
%   Columns: T_d = 5 s | T_d = 50 s.  Runs cached in test_results/temp_5state_noap/y2avg_Td{5,50}_s{1..3}.mat.
%   STATUS: SCRATCH
function plot_5state_noap_y2_average(rerun)
    if nargin < 1; rerun = false; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    pc = physical_constants();  R = pc.R;  a_nom = pc.Ts / pc.gamma_N;
    TD = [5 50];  seeds = 1:3;  T_HOLD0 = 0.5;  T_HOLD1 = 2.0;
    COL_TRUE = [0.8 0 0];  C_EST = [0 0.2 0.9];  C_MEAS = [0.55 0.75 1.0];  FS = 18;  LFS = 12;  AXLW = 2.0;
    f = figure('Position', [40 40 1700 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    fprintf('\n%-5s | %-8s %-8s | stop: %-8s %-8s %-8s | hold end: %-8s %-8s\n', 'T_d', 'a start', 'a wall', 'a_hat', 'y2 avg', 'e_a', 'e_a', 'y2 e_a');
    for c = 1:2
        T_d = TD(c);  nS = numel(seeds);  run = cell(1, nS);
        for q = 1:nS
            fn = fullfile(id, sprintf('y2avg_Td%d_s%d.mat', T_d, seeds(q)));
            if ~rerun && exist(fn, 'file'); run{q} = load(fn); continue; end
            cfg = canonical_scenario(0.05, 1.10, 'deep');  cfg.eq17_variant = '5state_noap';
            cfg.trajectory_type = 'ramp_descent';  cfg.h_init = 50;  cfg.h_bottom = 1.10 * R;  cfg.amplitude = 0;
            cfg.t_hold = T_HOLD0;  cfg.t_descend_override = T_d;  cfg.T_sim = T_HOLD0 + T_d + T_HOLD1;  cfg.h_min = 1.10 * R;
            cc = struct('lf_mode', 'const', 'lambda_f', 1, 'gain_order', 1, 'seed_mode', 'true_h0');
            s = run_pure_simulation(cfg, struct('seed', seeds(q), 'collect_diag', true, 'ctrl_const_override', cc));
            r = struct('t', s.tout(:), 'a_true', s.a_true_out(:, 3), 'a_hat', s.diag.a_hat(:, 3), 'a_m', s.diag.a_xm(:, 3), ...
                       'P_a', s.diag.P_a(:, 3), 'w_y2', s.diag.w_y2(:, 3));
            save(fn, '-struct', 'r');  run{q} = r;
        end
        t = run{1}.t - T_HOLD0;  n = numel(t);
        at = zeros(n, nS);  ah = at;  am = at;  av = at;
        for q = 1:nS
            r = run{q};  W0 = 1 / r.P_a(1);  w = r.w_y2;  w(1) = 0;
            at(:, q) = r.a_true;  ah(:, q) = r.a_hat;  am(:, q) = r.a_m;
            av(:, q) = (W0 * r.a_hat(1) + cumsum(w .* r.a_true)) ./ (W0 + cumsum(w));
        end
        at = mean(at, 2) / a_nom;  ah = mean(ah, 2) / a_nom;  am = mean(am, 2) / a_nom;  av = mean(av, 2) / a_nom;
        i0 = find(t >= 0, 1);  i1 = find(t <= T_d, 1, 'last');
        fprintf('%-5g | %-8.3f %-8.3f | stop: %-8.3f %-8.3f %+-8.3f | hold end: %+-8.3f %+-8.3f\n', T_d, at(i0), at(i1), ...
            ah(i1), av(i1), at(i1) - ah(i1), at(end) - ah(end), at(end) - av(end));
        ax1 = nexttile(tl, c);  hold(ax1, 'on');
        h0 = plot(ax1, nan, nan, 'w-', 'DisplayName', sprintf('plane, 22.2 \\rightarrow 1.10 R in T_d = %g s, then hold', T_d));
        ht = plot(ax1, t, at, '-', 'Color', COL_TRUE, 'LineWidth', 2.4, 'DisplayName', 'a_{true}');
        hf = plot(ax1, t, ah, '-', 'Color', C_EST, 'LineWidth', 2.0, 'DisplayName', 'a_{hat} (filter)');
        hv = plot(ax1, t, av, '--', 'Color', 'k', 'LineWidth', 2.0, 'DisplayName', 'y_2-only average of a_{true} (4.6)');
        xlim(ax1, [-T_HOLD0 T_d + T_HOLD1]);  ylim(ax1, [0 1.1]);
        legend(ax1, [h0 ht hf hv], 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'AutoUpdate', 'off');
        ax2 = nexttile(tl, 2 + c);  hold(ax2, 'on');
        plot(ax2, t - T_d, at - ah, '-', 'Color', C_EST, 'LineWidth', 2.0);
        plot(ax2, t - T_d, at - av, '--', 'Color', 'k', 'LineWidth', 2.0);
        xlim(ax2, [-1 T_HOLD1]);  ylim(ax2, [-0.9 0.05]);
        xlabel(ax2, 'time since the stop (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        for ax = [ax1 ax2]
            yl = ylim(ax);  xs = 0;  if ax == ax1; xs = [0 T_d]; end
            for x = xs; plot(ax, [x x], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2); end
            plot(ax, xlim(ax), [0 0], '-', 'Color', [0.6 0.6 0.6]);
            set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
        end
        xlabel(ax1, 'time since the descent start (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        if c == 1
            ylabel(ax1, 'a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
            ylabel(ax2, 'e_a = a_{true} - a_{hat}', 'FontSize', FS, 'FontWeight', 'bold');
        end
    end
    out = fullfile(fd, 'fig_noap_y2_average_td5_td50.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
