function temp_ramp_diag_compare()
%TEMP_RAMP_DIAG_COMPARE  Compare Q77=0 vs Q77 enabled in ramp scenario.
%   Tests whether enabling delta_a_x state adaptation (via non-zero Q77)
%   reduces the late-ramp bias by allowing KF to predict-forward the
%   deterministic da/dt from h_dot * K_h.

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

    % Test: does shrinking IIR window (larger a_cov) reduce lag bias?
    cases = struct( ...
        'tag',         {'acov0p005_baseline', 'acov0p05_short_window', 'acov0p1_shorter'}, ...
        'force_zero',  {true,                 true,                     true}, ...
        'sigma2_w_fA', {0,                    0,                        0}, ...
        'a_cov',       {0.005,                0.05,                     0.1}, ...
        'desc',        {'a_{cov}=0.005 baseline (\\tau_{IIR}=125ms)', ...
                        'a_{cov}=0.05 (\\tau_{IIR}=12.5ms)', ...
                        'a_{cov}=0.1 (\\tau_{IIR}=6.25ms)'} ...
    );

    seed = 1;
    results = cell(numel(cases), 1);

    for ic = 1:numel(cases)
        c = cases(ic);
        fprintf('\n[%d/%d] Running %s\n', ic, numel(cases), c.tag);
        cfg = build_cfg(c.force_zero, c.sigma2_w_fA);
        cfg.a_cov = c.a_cov;
        opts = struct('seed', seed, 'verbose', false, 'collect_diag', true);
        t0 = tic;
        sim_out = run_pure_simulation(cfg, opts);
        fprintf('  done in %.1fs\n', toc(t0));

        D = analyze_run(sim_out, cfg);
        D.tag = c.tag;
        D.desc = c.desc;
        D.cfg = cfg;
        results{ic} = D;

        fprintf('  z bias  a_xm = %+.2f%%  a_hat = %+.2f%%   z_std a_hat = %.2f%%\n', ...
            D.bias_xm_z_pct, D.bias_hat_z_pct, D.std_hat_z_pct);
        fprintf('  delta_a_x_z mean = %+.2e  (theory mid-ramp = %+.2e, end = %+.2e)\n', ...
            mean(D.da_x_z(D.idx_eval)), ...
            mean(D.da_step_z_theory(D.idx_eval)), ...
            mean(D.da_step_z_theory(D.t > 18 & D.t < 19.5)));
        fprintf('  P77_z mean = %.2e\n', mean(D.P77_z));
        fprintf('  zErr (3D track std) = %.2f nm\n', D.zErr_nm);
    end

    save(fullfile(out_dir, 'acov_compare.mat'), 'results');

    plot_acov_comparison(results, out_dir);
end


function plot_acov_comparison(results, out_dir)
    nC = numel(results);
    f = figure('Visible','off','Position',[100 100 1500 900]);
    colors = lines(nC);

    % Panel 1: a_z trajectories
    ax1 = subplot(3,1,1); hold(ax1,'on');
    plot(ax1, results{1}.t, results{1}.a_z_true, '-', 'Color', [0 0.55 0], 'LineWidth', 2.5, ...
        'DisplayName', 'a_{true}');
    for ic = 1:nC
        plot(ax1, results{ic}.t, results{ic}.a_hat_z, '-', 'Color', colors(ic,:), ...
            'LineWidth', 1.4, 'DisplayName', strrep(results{ic}.tag,'_','\_'));
    end
    set(ax1,'Box','on','FontSize',12,'LineWidth',1.2,'FontWeight','bold');
    ylabel(ax1,'a_{hat,z}','FontSize',13,'FontWeight','bold');
    legend(ax1, 'show', 'Location', 'best', 'Box', 'off', 'FontSize', 10);
    title(ax1,'a_{hat,z} comparison: shrinking IIR window via larger a_{cov}', ...
        'FontSize',12,'FontWeight','bold','Interpreter','tex');
    xlim(ax1,[results{1}.t(1) results{1}.t(end)]);
    ylim(ax1,[0.005 0.025]);

    % Panel 2: residual
    ax2 = subplot(3,1,2); hold(ax2,'on');
    for ic = 1:nC
        plot(ax2, results{ic}.t, results{ic}.a_hat_z - results{ic}.a_z_true, ...
            '-', 'Color', colors(ic,:), 'LineWidth', 1.0, ...
            'DisplayName', strrep(results{ic}.tag,'_','\_'));
    end
    plot(ax2, results{1}.t, zeros(size(results{1}.t)), ':', 'Color', [0 0 0]);
    set(ax2,'Box','on','FontSize',12,'LineWidth',1.2,'FontWeight','bold');
    ylabel(ax2,'a_{hat} - a_{true}','FontSize',13,'FontWeight','bold');
    legend(ax2, 'show', 'Location', 'best', 'Box', 'off', 'FontSize', 10);
    title(ax2,'Residual: should drop with larger a_{cov} if lag-bias is the cause', ...
        'FontSize',12,'FontWeight','bold','Interpreter','tex');
    xlim(ax2,[results{1}.t(1) results{1}.t(end)]);

    % Panel 3: a_xm raw (should also show smaller lag with larger a_cov)
    ax3 = subplot(3,1,3); hold(ax3,'on');
    plot(ax3, results{1}.t, results{1}.a_z_true, '-', 'Color', [0 0.55 0], 'LineWidth', 2.5, ...
        'DisplayName','a_{true}');
    for ic = 1:nC
        plot(ax3, results{ic}.t, results{ic}.a_xm_z, '-', 'Color', colors(ic,:), ...
            'LineWidth', 0.5, 'DisplayName', strrep(results{ic}.tag,'_','\_'));
    end
    set(ax3,'Box','on','FontSize',12,'LineWidth',1.2,'FontWeight','bold');
    xlabel(ax3,'Time (sec)','FontSize',13,'FontWeight','bold');
    ylabel(ax3,'a_{xm,z}','FontSize',13,'FontWeight','bold');
    legend(ax3, 'show', 'Location', 'best', 'Box', 'off', 'FontSize', 10);
    title(ax3,'Raw a_{xm} trajectories — sampling noise grows with a_{cov}', ...
        'FontSize',12,'FontWeight','bold','Interpreter','tex');
    xlim(ax3,[results{1}.t(1) results{1}.t(end)]);

    drawnow;
    p1 = fullfile(out_dir, 'acov_compare.png');
    print(f, p1, '-dpng', '-r150');
    close(f);
    fprintf('Saved: %s\n', p1);
end


function D = analyze_run(sim_out, cfg)
    [~, ~, ~] = fileparts(mfilename('fullpath'));
    params = calc_simulation_params(cfg);
    P = params.Value;
    R_radius = P.common.R; w_hat = P.wall.w_hat; pz_wall = P.wall.pz;
    Ts = P.common.Ts; gamma_N = P.common.gamma_N;
    a_freespace = Ts / gamma_N;

    t = sim_out.tout;
    p_d = sim_out.p_d_out;
    p_m = sim_out.p_m_out;
    ekf = sim_out.ekf_out;

    a_hat_z = ekf(:, 2);
    a_xm_z = sim_out.diag.a_xm(:, 3);
    da_x_z = sim_out.diag.delta_a_hat(:, 3);
    P77_z  = sim_out.diag.P77(:, 3);

    N = numel(t);
    a_z_true = zeros(N, 1); K_h_perp = zeros(N, 1); h_um = zeros(N, 1);
    for k = 1:N
        h_um(k) = dot(p_m(k, :)', w_hat) - pz_wall;
        hb = max(h_um(k) / R_radius, 1.001);
        [~, c_per, derivs] = calc_correction_functions(hb, true);
        a_z_true(k) = a_freespace / c_per;
        K_h_perp(k) = derivs.K_h_perp;
    end

    h_d_z = p_d * w_hat;
    h_dot_command = [0; diff(h_d_z)/Ts];
    da_dt_z_theory = -(a_z_true .* K_h_perp / R_radius) .* h_dot_command;
    da_step_z_theory = da_dt_z_theory * Ts;

    idx_eval = (t >= 0.5) & (t <= cfg.T_sim - 0.2);
    rel_err_z_hat = (a_hat_z(idx_eval) - a_z_true(idx_eval)) ./ a_z_true(idx_eval);
    rel_err_z_xm  = (a_xm_z(idx_eval)  - a_z_true(idx_eval)) ./ a_z_true(idx_eval);

    D.t = t; D.Ts = Ts;
    D.a_z_true = a_z_true;
    D.a_xm_z = a_xm_z; D.a_hat_z = a_hat_z;
    D.da_x_z = da_x_z; D.P77_z = P77_z;
    D.K_h_perp = K_h_perp; D.h_um = h_um;
    D.h_dot_command = h_dot_command;
    D.da_step_z_theory = da_step_z_theory;
    D.idx_eval = idx_eval;
    D.bias_xm_z_pct  = 100 * mean(rel_err_z_xm);
    D.bias_hat_z_pct = 100 * mean(rel_err_z_hat);
    D.std_hat_z_pct  = 100 * std(rel_err_z_hat);
    D.zErr_nm = std((p_m(idx_eval, 3) - p_d(idx_eval, 3)) * 1e3);
end


function plot_comparison(results, out_dir)
    nC = numel(results);
    f = figure('Visible','off','Position',[100 100 1400 900]);

    colors = lines(nC);

    % Panel 1: a_z_true, all a_hat overlaid
    ax1 = subplot(3,1,1); hold(ax1,'on');
    plot(ax1, results{1}.t, results{1}.a_z_true, '-', 'Color', [0 0.55 0], 'LineWidth', 2.5, ...
        'DisplayName', 'a_{true}');
    for ic = 1:nC
        plot(ax1, results{ic}.t, results{ic}.a_hat_z, '-', 'Color', colors(ic,:), ...
            'LineWidth', 1.4, 'DisplayName', results{ic}.tag);
    end
    set(ax1,'Box','on','FontSize',12,'LineWidth',1.2,'FontWeight','bold');
    ylabel(ax1,'a_z (KF)','FontSize',13,'FontWeight','bold');
    legend(ax1, 'show', 'Location', 'best', 'Box', 'off', 'FontSize', 10);
    title(ax1,'a_{hat,z} comparison across Q77 settings', ...
        'FontSize',12,'FontWeight','bold','Interpreter','tex');
    xlim(ax1,[results{1}.t(1) results{1}.t(end)]);
    ylim(ax1,[0.005 0.022]);

    % Panel 2: residuals (a_hat - a_true)
    ax2 = subplot(3,1,2); hold(ax2,'on');
    for ic = 1:nC
        plot(ax2, results{ic}.t, results{ic}.a_hat_z - results{ic}.a_z_true, ...
            '-', 'Color', colors(ic,:), 'LineWidth', 1.0, ...
            'DisplayName', results{ic}.tag);
    end
    plot(ax2, results{1}.t, zeros(size(results{1}.t)), ':', 'Color', [0 0 0]);
    set(ax2,'Box','on','FontSize',12,'LineWidth',1.2,'FontWeight','bold');
    ylabel(ax2,'a_{hat} - a_{true}','FontSize',13,'FontWeight','bold');
    legend(ax2, 'show', 'Location', 'best', 'Box', 'off', 'FontSize', 10);
    title(ax2,'Residual (a_{hat,z} - a_{true,z}) — does Q77 reduce late-ramp drift?', ...
        'FontSize',12,'FontWeight','bold','Interpreter','tex');
    xlim(ax2,[results{1}.t(1) results{1}.t(end)]);

    % Panel 3: delta_a_x_z trajectories
    ax3 = subplot(3,1,3); hold(ax3,'on');
    plot(ax3, results{1}.t, results{1}.da_step_z_theory, '--', 'Color', [0 0 0], 'LineWidth', 1.5, ...
        'DisplayName', 'theory: -(aK_h/R)h_{dot}Ts');
    for ic = 1:nC
        plot(ax3, results{ic}.t, results{ic}.da_x_z, '-', 'Color', colors(ic,:), ...
            'LineWidth', 1.2, 'DisplayName', results{ic}.tag);
    end
    plot(ax3, results{1}.t, zeros(size(results{1}.t)), ':', 'Color', [0.5 0.5 0.5]);
    set(ax3,'Box','on','FontSize',12,'LineWidth',1.2,'FontWeight','bold');
    xlabel(ax3,'Time (sec)','FontSize',13,'FontWeight','bold');
    ylabel(ax3,'\delta a_{x,z} (slot 7)','FontSize',13,'FontWeight','bold','Interpreter','tex');
    legend(ax3, 'show', 'Location', 'best', 'Box', 'off', 'FontSize', 10);
    title(ax3,'KF velocity state \delta a_{x,z}: does enabling Q77 unfreeze it?', ...
        'FontSize',12,'FontWeight','bold','Interpreter','tex');
    xlim(ax3,[results{1}.t(1) results{1}.t(end)]);

    drawnow;
    p1 = fullfile(out_dir, 'Q77_compare.png');
    print(f, p1, '-dpng', '-r150');
    close(f);
    fprintf('Saved: %s\n', p1);
end


function plot_velocity_comparison(results, out_dir)
    nC = numel(results);
    f = figure('Visible','off','Position',[100 100 1300 700]);
    mask = results{1}.t >= 12;
    colors = lines(nC);

    ax = gca; hold(ax,'on');
    plot(ax, results{1}.t(mask), results{1}.da_step_z_theory(mask), '--', ...
        'Color', [0 0 0], 'LineWidth', 2.0, 'DisplayName', 'theory');
    for ic = 1:nC
        plot(ax, results{ic}.t(mask), results{ic}.da_x_z(mask), '-', ...
            'Color', colors(ic,:), 'LineWidth', 1.3, 'DisplayName', results{ic}.tag);
    end
    set(ax,'Box','on','FontSize',13,'LineWidth',1.3,'FontWeight','bold');
    xlabel(ax,'Time (sec)','FontSize',14,'FontWeight','bold');
    ylabel(ax,'\delta a_{x,z}','FontSize',14,'FontWeight','bold','Interpreter','tex');
    legend(ax,'show','Location','northeast','Box','off','FontSize',11);
    title(ax,'Late-ramp \delta a_{x,z}: does Q77 enable velocity tracking?', ...
        'FontSize',13,'FontWeight','bold','Interpreter','tex');

    drawnow;
    p1 = fullfile(out_dir, 'Q77_velocity_zoom.png');
    print(f, p1, '-dpng', '-r150');
    close(f);
    fprintf('Saved: %s\n', p1);
end


function cfg = build_cfg(force_zero, sigma2_w_fA)
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
    cfg.sigma2_w_fA        = sigma2_w_fA;
    cfg.force_Q77_zero     = force_zero;
    cfg.suppress_xD        = false;

    cfg.meas_noise_enable = true;
    cfg.meas_noise_std    = [0.00062; 0.000057; 0.00331];
    cfg.thermal_enable    = true;

    cfg.Q66_OL_mode        = true;
    cfg.sigma2_w_a_direct  = 0;
    cfg.Q66_physical_mode  = false;
    cfg.R22_include_Q66    = true;
    cfg.Q36_cross_term     = false;
end
