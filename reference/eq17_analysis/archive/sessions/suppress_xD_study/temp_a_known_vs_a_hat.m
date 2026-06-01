function temp_a_known_vs_a_hat()
%TEMP_A_KNOWN_VS_A_HAT
%   Architecture comparison: 3 paths to estimate 'a' (mobility coefficient)
%     P1. a_xm        : raw IIR EWMA from closed-loop tracking residuals
%     P2. a_hat       : KF posterior using a_xm as measurement (current production)
%     P3. a_known_pm  : direct compute a_free / c_perp(h_bar(p_m))  (model-based)
%
%   Reference: a_z_truth from re-run with meas_noise=false + thermal=false

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

    cfg_noisy = build_cfg();
    cfg_clean = cfg_noisy;
    cfg_clean.meas_noise_enable = false;
    cfg_clean.thermal_enable    = false;

    seed = 1;
    opts = struct('seed', seed, 'verbose', false, 'collect_diag', true);

    fprintf('Run A (production: thermal+meas noise on)...\n');
    t0 = tic;
    sim_noisy = run_pure_simulation(cfg_noisy, opts);
    fprintf('  done in %.1fs\n', toc(t0));

    fprintf('Run B (no thermal, no meas noise -> clean reference)...\n');
    t0 = tic;
    sim_clean = run_pure_simulation(cfg_clean, opts);
    fprintf('  done in %.1fs\n', toc(t0));

    params = calc_simulation_params(cfg_noisy);
    P = params.Value;
    R_radius = P.common.R; w_hat = P.wall.w_hat; pz_wall = P.wall.pz;
    Ts = P.common.Ts; gamma_N = P.common.gamma_N;
    a_freespace = Ts / gamma_N;

    [~, a_known_noisy_z] = compute_a_known(sim_noisy.p_m_out, w_hat, pz_wall, R_radius, a_freespace);
    [~, a_z_truth]       = compute_a_known(sim_clean.p_m_out, w_hat, pz_wall, R_radius, a_freespace);

    t      = sim_noisy.tout;
    a_xm_z = sim_noisy.diag.a_xm(:, 3);
    a_hat_z = sim_noisy.ekf_out(:, 2);

    win = (t >= 1.0) & (t <= cfg_noisy.T_sim - 0.5);

    rel_xm    = (a_xm_z          - a_z_truth) ./ a_z_truth * 100;
    rel_hat   = (a_hat_z         - a_z_truth) ./ a_z_truth * 100;
    rel_known = (a_known_noisy_z - a_z_truth) ./ a_z_truth * 100;

    fprintf('\n========== a_z estimator comparison (window 1.0-%.1fs) ==========\n', cfg_noisy.T_sim - 0.5);
    fprintf('%-15s  %-12s  %-12s  %-12s\n', 'path', 'bias (%)', 'std (%)', 'RMSE (%)');
    print_row('a_xm (raw)',    rel_xm(win));
    print_row('a_hat (KF)',    rel_hat(win));
    print_row('a_known_pm',    rel_known(win));
    fprintf('\nChi-sq noise floor from IIR: sqrt(2*a_cov/(2-a_cov)) = %.2f%%\n', ...
        sqrt(2*cfg_noisy.a_cov/(2-cfg_noisy.a_cov))*100);

    plot_overlay(t, a_z_truth, a_xm_z, a_hat_z, a_known_noisy_z, out_dir);
    plot_residuals(t, rel_xm, rel_hat, rel_known, out_dir);

    cmp = struct();
    cmp.t = t; cmp.a_z_truth = a_z_truth;
    cmp.a_xm_z = a_xm_z; cmp.a_hat_z = a_hat_z; cmp.a_known_noisy_z = a_known_noisy_z;
    cmp.rel_xm = rel_xm; cmp.rel_hat = rel_hat; cmp.rel_known = rel_known;
    cmp.win = win; cmp.cfg = cfg_noisy;
    save(fullfile(out_dir, 'a_known_vs_a_hat.mat'), 'cmp');
    fprintf('\nSaved: %s\n', fullfile(out_dir, 'a_known_vs_a_hat.mat'));
end


function [h_um, a_z] = compute_a_known(p_m, w_hat, pz_wall, R_radius, a_freespace)
    N = size(p_m, 1);
    h_um = zeros(N, 1); a_z = zeros(N, 1);
    for k = 1:N
        h_um(k) = dot(p_m(k, :)', w_hat) - pz_wall;
        hb = max(h_um(k) / R_radius, 1.001);
        [~, c_per] = calc_correction_functions(hb, false);
        a_z(k) = a_freespace / c_per;
    end
end


function print_row(name, x)
    fprintf('%-15s  %+9.3f   %9.3f   %9.3f\n', name, mean(x), std(x), sqrt(mean(x.^2)));
end


function plot_overlay(t, a_truth, a_xm, a_hat, a_known, out_dir)
    f = figure('Visible','off','Position',[100 100 1500 700]);
    ax = axes(f); hold(ax,'on');
    plot(ax, t, a_xm,    '-', 'Color', [0.6 0.75 0.95], 'LineWidth', 0.6);
    plot(ax, t, a_hat,   '-', 'Color', [0.85 0.10 0.10], 'LineWidth', 1.6);
    plot(ax, t, a_known, '-', 'Color', [0.95 0.55 0.0], 'LineWidth', 1.6);
    plot(ax, t, a_truth, '-', 'Color', [0 0.55 0], 'LineWidth', 2.5);
    set(ax,'Box','on','FontSize',12,'LineWidth',1.2,'FontWeight','bold');
    xlabel(ax,'Time (sec)','FontSize',13,'FontWeight','bold');
    ylabel(ax,'a_z  ({\mu}m/pN)','FontSize',13,'FontWeight','bold','Interpreter','tex');
    legend(ax, {'a_{xm} (raw, variance-based)', ...
                'a_{hat} (KF posterior, production)', ...
                'a_{known} = a_{free}/c_{perp}(h_{bar}(p_m))  [model-based, w/ sensor noise]', ...
                'truth (no sensor / thermal noise)'}, ...
        'Orientation','vertical','Location','northoutside','Box','off','FontSize',11);
    xlim(ax,[t(1) t(end)]); ylim(ax,[0.005 0.025]);
    drawnow;
    p = fullfile(out_dir, 'a_known_vs_a_hat_overlay.png');
    print(f, p, '-dpng', '-r150'); close(f);
    fprintf('Saved: %s\n', p);
end


function plot_residuals(t, rel_xm, rel_hat, rel_known, out_dir)
    f = figure('Visible','off','Position',[100 100 1500 900]);

    ax1 = subplot(2,1,1); hold(ax1,'on');
    plot(ax1, t, rel_xm,    '-', 'Color', [0.6 0.75 0.95], 'LineWidth', 0.6);
    plot(ax1, t, rel_hat,   '-', 'Color', [0.85 0.10 0.10], 'LineWidth', 1.4);
    plot(ax1, t, zeros(size(t)), ':', 'Color', [0.4 0.4 0.4]);
    set(ax1,'Box','on','FontSize',12,'LineWidth',1.2,'FontWeight','bold');
    ylabel(ax1,'residual (%)','FontSize',13,'FontWeight','bold');
    legend(ax1, {'a_{xm} (raw)', 'a_{hat} (KF)'}, ...
        'Orientation','horizontal','Location','northoutside','Box','off','FontSize',11);
    xlim(ax1,[t(1) t(end)]); ylim(ax1,[-50 50]);

    ax2 = subplot(2,1,2); hold(ax2,'on');
    plot(ax2, t, rel_known, '-', 'Color', [0.95 0.55 0.0], 'LineWidth', 1.2);
    plot(ax2, t, zeros(size(t)), ':', 'Color', [0.4 0.4 0.4]);
    set(ax2,'Box','on','FontSize',12,'LineWidth',1.2,'FontWeight','bold');
    xlabel(ax2,'Time (sec)','FontSize',13,'FontWeight','bold');
    ylabel(ax2,'residual (%)','FontSize',13,'FontWeight','bold');
    legend(ax2, {'a_{known} = a_{free}/c_{perp}(h_{bar}(p_m))'}, ...
        'Orientation','horizontal','Location','northoutside','Box','off','FontSize',11);
    xlim(ax2,[t(1) t(end)]); ylim(ax2,[-2 2]);

    drawnow;
    p = fullfile(out_dir, 'a_known_vs_a_hat_residual.png');
    print(f, p, '-dpng', '-r150'); close(f);
    fprintf('Saved: %s\n', p);
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
