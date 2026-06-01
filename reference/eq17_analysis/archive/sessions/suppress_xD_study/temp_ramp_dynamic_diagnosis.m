function temp_ramp_dynamic_diagnosis()
%TEMP_RAMP_DYNAMIC_DIAGNOSIS  Full time-series diagnostic for ramp 50->5.
%   Records every key dynamic quantity, then verifies hypotheses about
%   the source of the ~5% z-axis bias:
%     H1. IIR EWMA lag in y_2: a_xm[k] should match a_true[k - tau_IIR/2]
%     H2. delta_a_x stuck at 0 because Q77=0 (force_Q77_zero=true)
%     H3. KF cannot compensate deterministic da/dt without delta_a_x action
%
%   Outputs:
%     - .mat with full trajectory arrays
%     - 4-panel diagnostic figure (z axis focus)
%     - Console table: per-phase (early/mid/late ramp) bias attribution

    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(script_dir);
    addpath(fullfile(project_root, 'model'));
    addpath(fullfile(project_root, 'model', 'config'));
    addpath(fullfile(project_root, 'model', 'wall_effect'));
    addpath(fullfile(project_root, 'model', 'thermal_force'));
    addpath(fullfile(project_root, 'model', 'trajectory'));
    addpath(fullfile(project_root, 'model', 'controller'));
    addpath(fullfile(project_root, 'model', 'pure_matlab'));
    addpath(fullfile(project_root, 'model', 'diag'));

    out_dir = fullfile(project_root, 'reference', 'eq17_analysis', ...
                       'figures', 'suppress_xD_ramp_acov005', 'diagnostic');
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end

    % ---------------------------------------------------------------
    % 1. Run baseline ramp (xD on, Q77=0)
    % ---------------------------------------------------------------
    cfg = build_cfg();
    seed = 1;
    opts = struct('seed', seed, 'verbose', false, 'collect_diag', true);

    fprintf('Running ramp 50->5 (a_pd=%.3f, a_cov=%.3f)...\n', cfg.a_pd, cfg.a_cov);
    t0 = tic;
    sim_out = run_pure_simulation(cfg, opts);
    fprintf('  done in %.1fs\n', toc(t0));

    % ---------------------------------------------------------------
    % 2. Extract trajectories
    % ---------------------------------------------------------------
    params = calc_simulation_params(cfg);
    P = params.Value;
    R_radius = P.common.R; w_hat = P.wall.w_hat; pz_wall = P.wall.pz;
    Ts = P.common.Ts; gamma_N = P.common.gamma_N;
    a_freespace = Ts / gamma_N;

    t = sim_out.tout;
    p_d = sim_out.p_d_out;
    p_m = sim_out.p_m_out;
    f_d = sim_out.f_d_out;
    ekf = sim_out.ekf_out;

    a_hat_x = ekf(:, 1);                    % slot 6 axis 1
    a_hat_z = ekf(:, 2);                    % slot 6 axis 3
    h_bar_log = ekf(:, 4);                  % current h_bar from controller

    a_xm_x = sim_out.diag.a_xm(:, 1);
    a_xm_z = sim_out.diag.a_xm(:, 3);
    da_x_x = sim_out.diag.delta_a_hat(:, 1); % slot 7 axis 1 — KF velocity
    da_x_z = sim_out.diag.delta_a_hat(:, 3); % slot 7 axis 3
    xD_x   = sim_out.diag.x_D_hat(:, 1);
    xD_z   = sim_out.diag.x_D_hat(:, 3);
    P77_z  = sim_out.diag.P77(:, 3);
    P_a_z  = sim_out.diag.P_a(:, 3);
    Q77_z  = sim_out.diag.Q77(:, 3);
    sigma2_dxr_z = sim_out.diag.sigma2_dxr_hat(:, 3);
    K_a_y2_z = sim_out.diag.K_kf_a_y2(:, 3);
    gate_z   = sim_out.diag.gate_active(:, 3);            % logical

    % ---------------------------------------------------------------
    % 3. Compute true a from p_m (truth side, using same wall formula)
    % ---------------------------------------------------------------
    N = numel(t);
    a_x_true = zeros(N, 1); a_z_true = zeros(N, 1);
    K_h_para = zeros(N, 1); K_h_perp = zeros(N, 1);
    c_para_log = zeros(N, 1); c_perp_log = zeros(N, 1);
    h_um = zeros(N, 1); h_bar_meas = zeros(N, 1);
    for k = 1:N
        h_um(k) = dot(p_m(k, :)', w_hat) - pz_wall;
        hb = max(h_um(k) / R_radius, 1.001);
        h_bar_meas(k) = hb;
        [c_par, c_per, derivs] = calc_correction_functions(hb, true);
        c_para_log(k) = c_par; c_perp_log(k) = c_per;
        a_x_true(k) = a_freespace / c_par;
        a_z_true(k) = a_freespace / c_per;
        K_h_para(k) = derivs.K_h_para;
        K_h_perp(k) = derivs.K_h_perp;
    end

    % ---------------------------------------------------------------
    % 4. Trajectory dh/dt (numerical) + analytical da/dt
    % ---------------------------------------------------------------
    h_d = dot(p_d, repmat(w_hat', N, 1), 2);     % desired wall-normal
    h_dot_command = [0; diff(h_d)/Ts];           % from p_d trajectory

    % Theoretical da/dt = -(a · K_h / R) · h_dot
    da_dt_z_theory = -(a_z_true .* K_h_perp / R_radius) .* h_dot_command;
    da_dt_x_theory = -(a_x_true .* K_h_para / R_radius) .* h_dot_command;

    % Per-step expected delta_a (Ts step)
    da_step_z_theory = da_dt_z_theory * Ts;       % what delta_a_z should be

    % ---------------------------------------------------------------
    % 5. Lag-shift analysis (find optimal tau that aligns a_xm to a_true)
    % ---------------------------------------------------------------
    % Sweep tau_lag in [0, 0.3] sec, find one minimizing sum sq residual
    tau_grid = (0:5:1600) * Ts;    % 0 to ~1.0s — extended to find true min
    err_z = zeros(numel(tau_grid), 1);
    err_x = zeros(numel(tau_grid), 1);
    idx_eval = (t >= 1.0) & (t <= cfg.T_sim - 0.5);   % skip warmup + endpoints
    for ii = 1:numel(tau_grid)
        n_shift = round(tau_grid(ii)/Ts);
        if n_shift >= N; continue; end
        a_true_shift_z = [zeros(n_shift,1); a_z_true(1:N-n_shift)];
        a_true_shift_x = [zeros(n_shift,1); a_x_true(1:N-n_shift)];
        idx_use = idx_eval & ((1:N)' > n_shift);
        err_z(ii) = mean( (a_xm_z(idx_use) - a_true_shift_z(idx_use)).^2 );
        err_x(ii) = mean( (a_xm_x(idx_use) - a_true_shift_x(idx_use)).^2 );
    end
    [~, imin_z] = min(err_z);
    [~, imin_x] = min(err_x);
    tau_z_opt = tau_grid(imin_z);
    tau_x_opt = tau_grid(imin_x);
    fprintf('Optimal lag (a_xm matching a_true at t-tau):\n');
    fprintf('  z axis: tau = %.4f sec  (= %d steps; %.1f%% of tau_IIR=%.4fs)\n', ...
        tau_z_opt, round(tau_z_opt/Ts), tau_z_opt/(1/cfg.a_cov*Ts)*100, 1/cfg.a_cov*Ts);
    fprintf('  x axis: tau = %.4f sec  (= %d steps)\n', tau_x_opt, round(tau_x_opt/Ts));

    % ---------------------------------------------------------------
    % 6. Per-phase bias attribution
    % ---------------------------------------------------------------
    % Split ramp into 5 equal-duration phases for time-resolved bias
    fprintf('\n========== Per-phase bias (a_z) ==========\n');
    fprintf('Phase    t-window     h-window         a_xm bias  a_hat bias  da_x_z\n');
    n_phases = 5;
    phase_edges = linspace(1.0, cfg.T_sim - 0.5, n_phases + 1);
    phase_table = zeros(n_phases, 6);  % t_mid, h_mid, a_xm_bias, a_hat_bias, da_x_avg, P77_avg
    for ip = 1:n_phases
        m = (t >= phase_edges(ip)) & (t < phase_edges(ip+1));
        a_xm_bias_z = 100 * mean( (a_xm_z(m) - a_z_true(m)) ./ a_z_true(m) );
        a_hat_bias_z = 100 * mean( (a_hat_z(m) - a_z_true(m)) ./ a_z_true(m) );
        da_x_avg = mean(da_x_z(m));
        h_avg = mean(h_um(m));
        t_mid = mean(t(m));
        phase_table(ip, :) = [t_mid, h_avg, a_xm_bias_z, a_hat_bias_z, ...
                              da_x_avg, mean(P77_z(m))];
        fprintf('  %d   [%5.1f-%5.1f]s  [%5.1f-%5.1f]um   %+6.2f%%    %+6.2f%%   %+.2e\n', ...
            ip, phase_edges(ip), phase_edges(ip+1), ...
            mean(h_um(m & t < (phase_edges(ip)+phase_edges(ip+1))/2)), ...
            mean(h_um(m & t >= (phase_edges(ip)+phase_edges(ip+1))/2)), ...
            a_xm_bias_z, a_hat_bias_z, da_x_avg);
    end

    % ---------------------------------------------------------------
    % 7. delta_a_x verification: is it stuck at 0?
    % ---------------------------------------------------------------
    fprintf('\n========== delta_a_x_z statistics ==========\n');
    fprintf('  mean across full sim:  %+.3e\n', mean(da_x_z));
    fprintf('  std:                   %.3e\n', std(da_x_z));
    fprintf('  max abs:               %.3e\n', max(abs(da_x_z)));
    fprintf('  Theoretical da_step (mid-ramp average): %+.3e\n', ...
        mean(da_step_z_theory(idx_eval)));
    fprintf('  Theoretical da_step at end (h~5um):    %+.3e\n', ...
        mean(da_step_z_theory(t > 18 & t < 19.5)));
    ratio_actual_theory = mean(da_x_z(idx_eval)) / max(abs(mean(da_step_z_theory(idx_eval))), 1e-30);
    fprintf('  Ratio actual/theory:   %.3f  (should be ~1.0 if KF tracking velocity)\n', ratio_actual_theory);

    fprintf('\n========== P77_z statistics ==========\n');
    fprintf('  mean P77_z over sim:   %.3e   (Q77=0 -> P77 -> 0)\n', mean(P77_z));
    fprintf('  Q77_z (should be 0):   %.3e\n', mean(Q77_z));
    fprintf('  P_a_z (slot 6) mean:   %.3e\n', mean(P_a_z));

    % ---------------------------------------------------------------
    % 8. Save .mat
    % ---------------------------------------------------------------
    diag_data = struct();
    diag_data.t = t; diag_data.Ts = Ts; diag_data.cfg = cfg;
    diag_data.h_um = h_um; diag_data.h_bar = h_bar_meas;
    diag_data.h_dot_command = h_dot_command;
    diag_data.a_z_true = a_z_true; diag_data.a_x_true = a_x_true;
    diag_data.a_xm_z = a_xm_z; diag_data.a_xm_x = a_xm_x;
    diag_data.a_hat_z = a_hat_z; diag_data.a_hat_x = a_hat_x;
    diag_data.da_x_z = da_x_z; diag_data.da_x_x = da_x_x;
    diag_data.K_h_perp = K_h_perp; diag_data.K_h_para = K_h_para;
    diag_data.da_dt_z_theory = da_dt_z_theory;
    diag_data.da_step_z_theory = da_step_z_theory;
    diag_data.P77_z = P77_z; diag_data.P_a_z = P_a_z; diag_data.Q77_z = Q77_z;
    diag_data.tau_z_opt = tau_z_opt; diag_data.tau_x_opt = tau_x_opt;
    diag_data.phase_table = phase_table;
    diag_data.K_a_y2_z = K_a_y2_z;
    diag_data.gate_z = gate_z;
    save(fullfile(out_dir, 'diagnostic.mat'), 'diag_data');
    fprintf('\nSaved data: %s\n', fullfile(out_dir, 'diagnostic.mat'));

    % ---------------------------------------------------------------
    % 9. Plots
    % ---------------------------------------------------------------
    plot_z_diagnostic(diag_data, out_dir);
    plot_velocity_check(diag_data, out_dir);
    plot_lag_alignment(diag_data, out_dir);
end


% ===================================================================
function plot_z_diagnostic(D, out_dir)
%PLOT_Z_DIAGNOSTIC  4-panel: a_z trajectories, residuals, h(t), velocity check
    f = figure('Visible','off','Position',[100 100 1500 1100]);

    % Panel 1: a_z trajectories
    ax1 = subplot(4,1,1); hold(ax1,'on');
    plot(ax1, D.t, D.a_xm_z, '-', 'Color', [0.6 0.75 0.95], 'LineWidth', 0.6);
    plot(ax1, D.t, D.a_hat_z, '-', 'Color', [0.85 0.10 0.10], 'LineWidth', 1.6);
    plot(ax1, D.t, D.a_z_true, '-', 'Color', [0 0.55 0], 'LineWidth', 2.5);
    set(ax1,'Box','on','FontSize',12,'LineWidth',1.2,'FontWeight','bold');
    ylabel(ax1,'a_z  ({\mu}m/pN)','FontSize',13,'FontWeight','bold','Interpreter','tex');
    legend(ax1, {'a_{xm} (raw IIR)','a_{hat} (KF)','a_{true} (truth)'}, ...
        'Orientation','horizontal','Location','northoutside','Box','off','FontSize',11);
    xlim(ax1,[D.t(1) D.t(end)]); ylim(ax1,[0.005 0.025]);
    title(ax1,'Panel 1: a_z trajectories','FontSize',12,'FontWeight','bold','Interpreter','tex');

    % Panel 2: Residuals (a_xm - a_true), (a_hat - a_true), and theoretical lag bias
    ax2 = subplot(4,1,2); hold(ax2,'on');
    res_xm  = D.a_xm_z - D.a_z_true;
    res_hat = D.a_hat_z - D.a_z_true;
    pred_lag_bias = -D.da_dt_z_theory .* (D.tau_z_opt/2);  % expected lag
    plot(ax2, D.t, res_xm,  '-', 'Color', [0.6 0.75 0.95], 'LineWidth', 0.6);
    plot(ax2, D.t, res_hat, '-', 'Color', [0.85 0.10 0.10], 'LineWidth', 1.4);
    plot(ax2, D.t, pred_lag_bias, '--', 'Color', [0 0 0], 'LineWidth', 1.5);
    set(ax2,'Box','on','FontSize',12,'LineWidth',1.2,'FontWeight','bold');
    ylabel(ax2,'a_z residual','FontSize',13,'FontWeight','bold','Interpreter','tex');
    legend(ax2, {'a_{xm} - a_{true}','a_{hat} - a_{true}','-da/dt {\cdot} \tau/2'}, ...
        'Orientation','horizontal','Location','northoutside','Box','off','FontSize',10);
    xlim(ax2,[D.t(1) D.t(end)]);
    title(ax2,'Panel 2: residuals + theoretical IIR lag bias prediction', ...
        'FontSize',12,'FontWeight','bold','Interpreter','tex');

    % Panel 3: delta_a_x_z (KF velocity state) vs theoretical da/dt * Ts
    ax3 = subplot(4,1,3); hold(ax3,'on');
    plot(ax3, D.t, D.da_x_z, '-', 'Color', [0.85 0.10 0.10], 'LineWidth', 1.4);
    plot(ax3, D.t, D.da_step_z_theory, '--', 'Color', [0 0 0], 'LineWidth', 1.5);
    plot(ax3, D.t, zeros(size(D.t)), ':', 'Color', [0.5 0.5 0.5]);
    set(ax3,'Box','on','FontSize',12,'LineWidth',1.2,'FontWeight','bold');
    ylabel(ax3,'\delta a_x_z  ({\mu}m/pN per step)','FontSize',13,'FontWeight','bold','Interpreter','tex');
    legend(ax3, {'\delta a_x_z (KF state, slot 7)','-(a{\cdot}K_h/R){\cdot}h_{dot}{\cdot}Ts (true motion)','zero'}, ...
        'Orientation','horizontal','Location','northoutside','Box','off','FontSize',10);
    xlim(ax3,[D.t(1) D.t(end)]);
    title(ax3,'Panel 3: KF velocity state vs true da/dt {\cdot} Ts (Q77=0 freezes \delta a_x at 0)', ...
        'FontSize',12,'FontWeight','bold','Interpreter','tex');

    % Panel 4: h(t) and h_bar(t) for context
    ax4 = subplot(4,1,4); hold(ax4,'on');
    yyaxis(ax4,'left');
    plot(ax4, D.t, D.h_um, '-', 'Color', [0 0.4 0.7], 'LineWidth', 1.4);
    ylabel(ax4,'h ({\mu}m)','FontSize',13,'FontWeight','bold','Interpreter','tex');
    yyaxis(ax4,'right');
    plot(ax4, D.t, D.K_h_perp, '-', 'Color', [0.7 0.3 0], 'LineWidth', 1.4);
    ylabel(ax4,'K_{h,perp}','FontSize',13,'FontWeight','bold','Interpreter','tex');
    set(ax4,'Box','on','FontSize',12,'LineWidth',1.2,'FontWeight','bold');
    xlabel(ax4,'Time (sec)','FontSize',13,'FontWeight','bold');
    xlim(ax4,[D.t(1) D.t(end)]);
    title(ax4,'Panel 4: h(t) and K_{h,perp}(t) — note K_h amplification near wall', ...
        'FontSize',12,'FontWeight','bold','Interpreter','tex');

    drawnow;
    p1 = fullfile(out_dir, 'z_diagnostic_4panel.png');
    print(f, p1, '-dpng', '-r150');
    close(f);
    fprintf('Saved: %s\n', p1);
end


function plot_velocity_check(D, out_dir)
%PLOT_VELOCITY_CHECK  Zoom on late ramp: are da_x_z and da_dt_theory consistent?
    f = figure('Visible','off','Position',[100 100 1200 800]);
    mask = D.t >= 12 & D.t <= 20;

    ax1 = subplot(3,1,1); hold(ax1,'on');
    plot(ax1, D.t(mask), D.a_z_true(mask), '-', 'Color', [0 0.55 0], 'LineWidth', 2.5);
    plot(ax1, D.t(mask), D.a_hat_z(mask), '-', 'Color', [0.85 0.10 0.10], 'LineWidth', 1.6);
    plot(ax1, D.t(mask), D.a_xm_z(mask), '-', 'Color', [0.6 0.75 0.95], 'LineWidth', 0.6);
    set(ax1,'Box','on','FontSize',12,'LineWidth',1.2,'FontWeight','bold');
    ylabel(ax1,'a_z','FontSize',13,'FontWeight','bold');
    legend(ax1, {'a_{true}','a_{hat}','a_{xm}'},'Location','northeast','Box','off','FontSize',10);
    title(ax1,'Late ramp (t=12-20s): a_{hat} drift above a_{true}', ...
        'FontSize',12,'FontWeight','bold','Interpreter','tex');
    ylim(ax1,[0.005 0.02]);

    ax2 = subplot(3,1,2); hold(ax2,'on');
    plot(ax2, D.t(mask), D.da_step_z_theory(mask), '--', 'Color', [0 0 0], 'LineWidth', 1.5);
    plot(ax2, D.t(mask), D.da_x_z(mask), '-', 'Color', [0.85 0.10 0.10], 'LineWidth', 1.4);
    plot(ax2, D.t(mask), zeros(sum(mask),1), ':', 'Color', [0.5 0.5 0.5]);
    set(ax2,'Box','on','FontSize',12,'LineWidth',1.2,'FontWeight','bold');
    ylabel(ax2,'\delta a_x_z (per step)','FontSize',13,'FontWeight','bold','Interpreter','tex');
    legend(ax2, {'theory (-aK_h/R{\cdot}h_{dot}{\cdot}Ts)','KF state','zero'}, ...
        'Location','southwest','Box','off','FontSize',10);
    title(ax2,'KF velocity state lags theoretical da/dt severely', ...
        'FontSize',12,'FontWeight','bold','Interpreter','tex');

    ax3 = subplot(3,1,3); hold(ax3,'on');
    plot(ax3, D.t(mask), D.K_h_perp(mask), '-', 'Color', [0.7 0.3 0], 'LineWidth', 1.5);
    yyaxis(ax3,'right');
    plot(ax3, D.t(mask), D.h_um(mask), '-', 'Color', [0 0.4 0.7], 'LineWidth', 1.5);
    ylabel(ax3,'h ({\mu}m)','FontSize',13,'FontWeight','bold','Interpreter','tex');
    yyaxis(ax3,'left');
    ylabel(ax3,'K_{h,perp}','FontSize',13,'FontWeight','bold','Interpreter','tex');
    set(ax3,'Box','on','FontSize',12,'LineWidth',1.2,'FontWeight','bold');
    xlabel(ax3,'Time (sec)','FontSize',13,'FontWeight','bold');
    title(ax3,'K_{h,perp} amplifies near wall (h\rightarrow 5{\mu}m)', ...
        'FontSize',12,'FontWeight','bold','Interpreter','tex');

    drawnow;
    p1 = fullfile(out_dir, 'late_ramp_zoom.png');
    print(f, p1, '-dpng', '-r150');
    close(f);
    fprintf('Saved: %s\n', p1);
end


function plot_lag_alignment(D, out_dir)
%PLOT_LAG_ALIGNMENT  Test H1: a_xm[k] vs a_true[k - tau_opt] should align well
    f = figure('Visible','off','Position',[100 100 1200 800]);
    n_shift = round(D.tau_z_opt / D.Ts);
    a_true_shift = [zeros(n_shift,1); D.a_z_true(1:end-n_shift)];

    ax1 = subplot(2,1,1); hold(ax1,'on');
    plot(ax1, D.t, D.a_xm_z, '-', 'Color', [0.6 0.75 0.95], 'LineWidth', 0.6);
    plot(ax1, D.t, D.a_z_true, '-', 'Color', [0 0.55 0], 'LineWidth', 2.5);
    plot(ax1, D.t, a_true_shift, '--', 'Color', [0 0.3 0.7], 'LineWidth', 2.0);
    set(ax1,'Box','on','FontSize',12,'LineWidth',1.2,'FontWeight','bold');
    ylabel(ax1,'a_z','FontSize',13,'FontWeight','bold');
    legend(ax1, {'a_{xm}', 'a_{true}(t)', sprintf('a_{true}(t - %.0fms) lag-shifted', D.tau_z_opt*1000)}, ...
        'Location','best','Box','off','FontSize',10);
    title(ax1,sprintf('H1 verification: a_{xm} aligns with a_{true} delayed by %.0f ms', ...
        D.tau_z_opt*1000),'FontSize',12,'FontWeight','bold','Interpreter','tex');
    xlim(ax1,[D.t(1) D.t(end)]); ylim(ax1,[0.005 0.025]);

    ax2 = subplot(2,1,2); hold(ax2,'on');
    res_unshifted = D.a_xm_z - D.a_z_true;
    res_shifted   = D.a_xm_z - a_true_shift;
    plot(ax2, D.t, res_unshifted, '-', 'Color', [0.6 0.75 0.95], 'LineWidth', 0.6);
    plot(ax2, D.t, res_shifted,   '-', 'Color', [0 0.3 0.7], 'LineWidth', 1.0);
    plot(ax2, D.t, zeros(size(D.t)), ':', 'Color', [0.5 0.5 0.5]);
    set(ax2,'Box','on','FontSize',12,'LineWidth',1.2,'FontWeight','bold');
    xlabel(ax2,'Time (sec)','FontSize',13,'FontWeight','bold');
    ylabel(ax2,'residual','FontSize',13,'FontWeight','bold');
    legend(ax2, {'a_{xm} - a_{true}(t)', 'a_{xm} - a_{true}(t - \tau)'}, ...
        'Location','best','Box','off','FontSize',10);
    title(ax2,'Lag-shifted residual collapses around 0 if H1 correct (only chi-sq noise remains)', ...
        'FontSize',12,'FontWeight','bold','Interpreter','tex');
    xlim(ax2,[D.t(1) D.t(end)]);

    drawnow;
    p1 = fullfile(out_dir, 'lag_alignment.png');
    print(f, p1, '-dpng', '-r150');
    close(f);
    fprintf('Saved: %s\n', p1);
end


function cfg = build_cfg()
    cfg = user_config();
    cfg.theta = 0; cfg.phi = 0; cfg.pz = 0;
    cfg.enable_wall_effect = true;
    cfg.h_min = 1.001 * 2.25;

    cfg.trajectory_type = 'ramp_descent';
    cfg.h_init    = 50;
    cfg.h_bottom  = 5;
    cfg.t_hold    = 0;
    cfg.amplitude = 0; cfg.frequency = 0; cfg.n_cycles = 0;
    cfg.T_sim = 20;
    cfg.h_dot_max_override  = 2.25;
    cfg.h_ddot_max_override = 0;

    cfg.controller_type = 7;
    cfg.ctrl_enable = true;
    cfg.lambda_c = 0.7;

    cfg.iir_warmup_mode    = 'prefill';
    cfg.t_warmup_kf        = 0;
    cfg.cdpmr_method       = 'closed_form';
    cfg.a_pd               = 0.05;
    cfg.a_cov              = 0.005;
    cfg.sigma2_w_fA        = 0;
    cfg.force_Q77_zero     = true;
    cfg.suppress_xD        = false;       % xD on (production)

    cfg.meas_noise_enable = true;
    cfg.meas_noise_std    = [0.00062; 0.000057; 0.00331];
    cfg.thermal_enable    = true;

    cfg.Q66_OL_mode        = true;
    cfg.sigma2_w_a_direct  = 0;
    cfg.Q66_physical_mode  = false;
    cfg.R22_include_Q66    = true;
    cfg.Q36_cross_term     = false;
end
