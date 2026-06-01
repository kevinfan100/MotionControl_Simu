function temp_suppress_xD_ramp_apd05_acov005()
%TEMP_SUPPRESS_XD_RAMP_APD05_ACOV005  Compare suppress_xD on vs off.
%   ramp 50->5, a_pd=0.05, a_cov=0.005 (Wave 2D default), seed=1.
%   Tests "EKF still estimates x_D, but control law does not use it"
%   (xD_hat_for_ctrl forced to 0 in Eq.17 force law).

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

    fig_dir = fullfile(project_root, 'reference', 'eq17_analysis', ...
                       'figures', 'suppress_xD_ramp_acov005');
    if ~exist(fig_dir, 'dir'); mkdir(fig_dir); end

    s_cfg = struct('type','ramp_descent','h_init',50,'h_bottom',5, ...
                   't_hold',0,'T_sim',20, ...
                   'h_dot_max_override',2.25,'h_ddot_max_override',0);

    seed = 1;
    y_lim = [0.005, 0.025];          % unified, no negative values

    cases = struct( ...
        'tag',  {'xD_on',  'xD_off'}, ...
        'flag', {false,    true   }, ...
        'desc', {'x_D used in ctrl law', 'x_D suppressed in ctrl law'} ...
    );

    for ic = 1:numel(cases)
        c = cases(ic);
        fig_path = fullfile(fig_dir, [c.tag '.png']);
        fprintf('[%d/%d] suppress_xD=%d  -> %s\n', ic, numel(cases), c.flag, c.tag);
        run_one(s_cfg, seed, c.flag, c.desc, y_lim, fig_path);
    end
end


function run_one(s_cfg, seed, suppress_xD_flag, desc, y_lim, fig_path)
    cfg = build_cfg(s_cfg, suppress_xD_flag);
    opts = struct('seed', seed, 'verbose', false, 'collect_diag', true);
    t0 = tic;
    sim_out = run_pure_simulation(cfg, opts);
    fprintf('  sim done in %.1fs\n', toc(t0));

    params = calc_simulation_params(cfg);
    P = params.Value;
    R_radius = P.common.R; w_hat = P.wall.w_hat; pz_wall = P.wall.pz;
    Ts = P.common.Ts; gamma_N = P.common.gamma_N;
    a_freespace = Ts / gamma_N;

    t = sim_out.tout;
    T_sim = cfg.T_sim;
    p_d = sim_out.p_d_out;
    p_m = sim_out.p_m_out;
    ekf = sim_out.ekf_out;

    N = length(t);
    a_x_true = zeros(N, 1); a_z_true = zeros(N, 1);
    for k = 1:N
        h_um = dot(p_m(k, :)', w_hat) - pz_wall;
        hb = max(h_um / R_radius, 1.001);
        [c_par_k, c_per_k] = calc_correction_functions(hb);
        a_x_true(k) = a_freespace / c_par_k;
        a_z_true(k) = a_freespace / c_per_k;
    end

    a_hat_x = ekf(:, 1);
    a_hat_z = ekf(:, 2);
    a_xm_x  = sim_out.diag.a_xm(:, 1);
    a_xm_z  = sim_out.diag.a_xm(:, 3);

    t_start = 0.5;
    t_end = T_sim - 0.2;
    idx = (t >= t_start) & (t <= t_end);

    rel_err_x_hat = (a_hat_x(idx) - a_x_true(idx)) ./ a_x_true(idx);
    rel_err_z_hat = (a_hat_z(idx) - a_z_true(idx)) ./ a_z_true(idx);
    rel_err_x_xm  = (a_xm_x(idx)  - a_x_true(idx)) ./ a_x_true(idx);
    rel_err_z_xm  = (a_xm_z(idx)  - a_z_true(idx)) ./ a_z_true(idx);

    bias_x_hat = 100 * mean(rel_err_x_hat); std_x_hat = 100 * std(rel_err_x_hat);
    bias_z_hat = 100 * mean(rel_err_z_hat); std_z_hat = 100 * std(rel_err_z_hat);
    bias_x_xm  = 100 * mean(rel_err_x_xm);  std_x_xm  = 100 * std(rel_err_x_xm);
    bias_z_xm  = 100 * mean(rel_err_z_xm);  std_z_xm  = 100 * std(rel_err_z_xm);
    z_err_nm = std((p_m(idx, 3) - p_d(idx, 3)) * 1e3);

    fprintf('  a_hat: x[%+.2f/%.2f] z[%+.2f/%.2f]  zErr=%.2fnm\n', ...
        bias_x_hat, std_x_hat, bias_z_hat, std_z_hat, z_err_nm);
    fprintf('  a_xm : x[%+.2f/%.2f] z[%+.2f/%.2f]\n', ...
        bias_x_xm, std_x_xm, bias_z_xm, std_z_xm);

    plot_one(t, a_xm_x, a_xm_z, a_x_true, a_z_true, a_hat_x, a_hat_z, ...
             desc, ...
             bias_x_hat, std_x_hat, bias_x_xm, std_x_xm, ...
             bias_z_hat, std_z_hat, bias_z_xm, std_z_xm, ...
             z_err_nm, y_lim, fig_path);
end


function plot_one(t, a_xm_x, a_xm_z, a_x_true, a_z_true, a_hat_x, a_hat_z, ...
                  desc, ...
                  bias_x_hat, std_x_hat, bias_x_xm, std_x_xm, ...
                  bias_z_hat, std_z_hat, bias_z_xm, std_z_xm, ...
                  z_err_nm, y_lim, fig_path)

    light_blue = [0.45 0.70 0.95];
    green_bold = [0    0.55 0   ];
    red_bold   = [0.85 0.10 0.10];

    f = figure('Visible','off','Position',[100 100 1400 800]);

    ax1 = subplot(2,1,1); hold(ax1,'on');
    h_meas = plot(ax1, t, a_xm_x, '-', 'Color', light_blue, 'LineWidth', 0.8);
    h_true = plot(ax1, t, a_x_true, '-', 'Color', green_bold, 'LineWidth', 3.0);
    h_est  = plot(ax1, t, a_hat_x, '-', 'Color', red_bold, 'LineWidth', 2.0);
    set(ax1,'Box','on','XGrid','off','YGrid','off', ...
        'FontSize', 14, 'LineWidth', 1.4, 'FontWeight','bold');
    ylabel(ax1, 'a_x  ({\mu}m/pN)', 'FontSize', 16, 'FontWeight','bold','Interpreter','tex');
    title_x = sprintf('a_x   (a_{pd}=0.05  a_{cov}=0.005, %s)', desc);
    title(ax1, title_x, 'FontSize', 13, 'FontWeight','bold','Interpreter','tex');
    legend(ax1, [h_meas h_true h_est], {'Measured','True','Estimated'}, ...
        'Orientation','horizontal','Location','northoutside','Box','off', ...
        'FontSize', 13, 'FontWeight','bold');
    xlim(ax1, [t(1) t(end)]); ylim(ax1, y_lim);

    ax2 = subplot(2,1,2); hold(ax2,'on');
    plot(ax2, t, a_xm_z, '-', 'Color', light_blue, 'LineWidth', 0.8);
    plot(ax2, t, a_z_true, '-', 'Color', green_bold, 'LineWidth', 3.0);
    plot(ax2, t, a_hat_z, '-', 'Color', red_bold, 'LineWidth', 2.0);
    set(ax2,'Box','on','XGrid','off','YGrid','off', ...
        'FontSize', 14, 'LineWidth', 1.4, 'FontWeight','bold');
    xlabel(ax2, 'Time (sec)', 'FontSize', 16, 'FontWeight','bold');
    ylabel(ax2, 'a_z  ({\mu}m/pN)', 'FontSize', 16, 'FontWeight','bold','Interpreter','tex');
    title(ax2, 'a_z', 'FontSize', 13, 'FontWeight','bold','Interpreter','tex');
    xlim(ax2, [t(1) t(end)]); ylim(ax2, y_lim);

    drawnow;
    print(f, fig_path, '-dpng', '-r150');
    fprintf('  saved: %s\n', fig_path);
    close(f);
end


function cfg = build_cfg(s_cfg, suppress_xD_flag)
    cfg = user_config();
    cfg.theta = 0; cfg.phi = 0; cfg.pz = 0;
    cfg.enable_wall_effect = true;
    cfg.h_min = 1.001 * 2.25;

    cfg.trajectory_type = s_cfg.type;
    cfg.h_init    = s_cfg.h_init;
    cfg.h_bottom  = s_cfg.h_bottom;
    cfg.t_hold    = s_cfg.t_hold;
    cfg.amplitude = 0; cfg.frequency = 0; cfg.n_cycles = 0;
    if isfield(s_cfg,'T_sim'); cfg.T_sim = s_cfg.T_sim; end
    if isfield(s_cfg,'h_dot_max_override')
        cfg.h_dot_max_override = s_cfg.h_dot_max_override;
    end
    if isfield(s_cfg,'h_ddot_max_override')
        cfg.h_ddot_max_override = s_cfg.h_ddot_max_override;
    end

    cfg.controller_type = 7;
    cfg.ctrl_enable = true;
    cfg.lambda_c = 0.7;

    cfg.iir_warmup_mode    = 'prefill';
    cfg.t_warmup_kf        = 0;
    cfg.cdpmr_method       = 'closed_form';
    cfg.a_pd               = 0.05;       % Wave 2D default
    cfg.a_cov              = 0.005;      % Wave 2D default
    cfg.sigma2_w_fA        = 0;
    cfg.force_Q77_zero     = true;
    cfg.suppress_xD        = suppress_xD_flag;

    cfg.meas_noise_enable = true;
    cfg.meas_noise_std    = [0.00062; 0.000057; 0.00331];
    cfg.thermal_enable    = true;

    cfg.Q66_OL_mode        = true;
    cfg.sigma2_w_a_direct  = 0;
    cfg.Q66_physical_mode  = false;
    cfg.R22_include_Q66    = true;
    cfg.Q36_cross_term     = false;
end
