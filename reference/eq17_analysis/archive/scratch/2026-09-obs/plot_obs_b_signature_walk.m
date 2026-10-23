function plot_obs_b_signature_walk()
%PLOT_OBS_B_SIGNATURE_WALK  Gate 2 of the observability workflow as a picture (0907_estb_5state_core.tex S7).
%   PURPOSE (2026-09-09, explanation-style rule 9): walk one e_b and one e_{a_w}[k0] along the canon deep command
%   trajectory with the exact law and no noise, and show what each leaves on y2 (e_{a_w}) from a window that starts
%   at the trough. b's signature grows with the distance travelled from k0 and returns to zero at every revisit;
%   a_w's signature is largest at the trough and freezes in the hold. Two panels: the situation (w_bar_d), the two
%   signatures. No filter, no noise, no u coordinate on the figure. Output test_results/apd_acov_meng/obs_b_signature_walk.png
%   EXPIRES: teaching figure, with the 0907 reading copy | 產線改動不會自動跟上
    here = fileparts(mfilename('fullpath')); root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    cfg = canonical_scenario(0.05, 1.1, 'deep');
    params = calc_simulation_params(cfg);  P = params.Value;
    Ts = P.common.Ts;  R = P.common.R;  N = round(cfg.T_sim / Ts) + 1;
    w_hat = P.wall.w_hat(:);  pz = P.wall.pz;
    clear trajectory_generator;
    t = (0:N-1)' * Ts;  wd = zeros(N, 1);  pd_k = P.common.p0;
    for k = 1:N
        [pd_kp1, ~] = trajectory_generator(t(k), P);
        wd(k) = (dot(pd_k, w_hat) - pz) / R;  pd_k = pd_kp1;
    end
    clear trajectory_generator;
    % window start k0 = first step at the trough (end of the descent)
    k0 = find(t >= cfg.t_hold + cfg.t_descend_override, 1);
    [~, cp0] = calc_correction_functions(wd(k0), true);  a0 = 1 / cp0;
    b_ref = 8/9;  e_b = 0.04;  e_a0 = 0.01;
    law = @(u0, b) 1 - 1 ./ (u0 + b * (wd - wd(k0)));          % exact law, u = 1/(1 - a_bar)
    a_ref = law(1/(1 - a0),        b_ref);
    s_b   = law(1/(1 - a0),        b_ref + e_b) - a_ref;         % signature of e_b on y2 (H24 ~ 1)
    s_a   = law(1/(1 - a0 - e_a0), b_ref)       - a_ref;         % signature of e_{a_w}[k0]
    s_b(1:k0-1) = NaN;  s_a(1:k0-1) = NaN;
    % the four numbers of the walk
    [~, k_top] = max(wd(k0:end));  k_top = k_top + k0 - 1;
    k_hold = find(t >= cfg.t_hold + cfg.t_descend_override + cfg.n_cycles / cfg.frequency + 0.5, 1);
    fprintf('window start k0: t = %.2f s, w_bar = %.2f, a_bar = %.3f\n', t(k0), wd(k0), a0);
    fprintf('oscillation top: t = %.2f s, w_bar = %.2f, (1-a)^2 = %.3f | s_b = %+.4f  s_a = %+.4f\n', ...
            t(k_top), wd(k_top), (1 - a_ref(k_top))^2, s_b(k_top), s_a(k_top));
    k_back = k_top + find(abs(wd(k_top:end) - wd(k0)) < 2e-3, 1) - 1;
    fprintf('first revisit of the trough: t = %.2f s | s_b = %+.5f  s_a = %+.4f\n', t(k_back), s_b(k_back), s_a(k_back));
    fprintf('hold: t = %.2f s | s_b = %+.5f  s_a = %+.4f\n', t(k_hold), s_b(k_hold), s_a(k_hold));
    % figure
    tb = [cfg.t_hold, cfg.t_hold + cfg.t_descend_override, cfg.t_hold + cfg.t_descend_override + cfg.n_cycles / cfg.frequency];
    fig = figure('Position', [100 100 1100 760], 'Color', 'w');
    ax1 = subplot(2, 1, 1);
    h1 = plot(t, wd, 'b-', 'LineWidth', 1.8); hold on;
    yl = ylim; hp = gobjects(1, numel(tb));
    for i = 1:numel(tb); hp(i) = plot([tb(i) tb(i)], yl, 'k--', 'LineWidth', 0.8); end
    hk = plot([t(k0) t(k0)], yl, 'r:', 'LineWidth', 1.6);
    ylabel('$\bar{w}_d$', 'Interpreter', 'latex'); box on; set(gca, 'FontSize', 13);
    legend([h1 hp(1) hk], {'$\bar{w}_d$ (command height)', 'phase boundary', 'window start $k_0$'}, 'Interpreter', 'latex', ...
           'Location', 'northoutside', 'Orientation', 'horizontal');
    ax2 = subplot(2, 1, 2);
    plot(t, s_b, 'b-', 'LineWidth', 1.8); hold on;
    plot(t, s_a, '-', 'Color', [0.45 0.70 1.0], 'LineWidth', 1.8);
    plot(t, zeros(size(t)), 'k-', 'LineWidth', 0.5);
    yl = ylim; for i = 1:numel(tb); plot([tb(i) tb(i)], yl, 'k--', 'LineWidth', 0.8); end
    ylabel('$e_{\bar{a}_w}$ on $y_2$', 'Interpreter', 'latex'); xlabel('time [s]'); box on; set(gca, 'FontSize', 13);
    legend({sprintf('signature of $e_b = %+.2f$', e_b), sprintf('signature of $e_{\\bar{a}_w}[k_0] = %+.2f$', e_a0)}, ...
           'Interpreter', 'latex', 'Location', 'northoutside', 'Orientation', 'horizontal');
    linkaxes([ax1 ax2], 'x'); xlim([0 cfg.T_sim]);
    od = fullfile(root, 'test_results', 'apd_acov_meng'); if ~exist(od, 'dir'); mkdir(od); end
    exportgraphics(fig, fullfile(od, 'obs_b_signature_walk.png'), 'Resolution', 150);
    fprintf('saved %s\n', fullfile(od, 'obs_b_signature_walk.png'));
end
