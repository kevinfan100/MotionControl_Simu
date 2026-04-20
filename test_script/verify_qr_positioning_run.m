function verify_qr_positioning_run(start_idx, end_idx, output_file, T_sim, t_warmup, variant_filter)
%VERIFY_QR_POSITIONING_RUN Worker: runs subset of (scenario, variant, seed) matrix
%
%   Usage from Bash:
%     matlab -batch "verify_qr_positioning_run(1, 13, 'test_results/verify/qr_pos_b1.mat')"
%
%   Optional args (Session 7 P1 verification):
%     T_sim          - simulation time [s], default 30
%     t_warmup       - steady-state window start [s], default 10
%     variant_filter - vector of variant indices to run, default [1 2 3 4]
%                      (1=empirical, 2=emp_acov005, 3=frozen_correct, 4=frozen_smartPf)
%
%   Matrix iteration order (n_sc * n_var * n_seed total):
%     for is = 1:n_sc, for iv = 1:n_var, for ie = 1:n_seed -> idx=1..total
%
%   Defaults (Session 6 reproduction):
%     T_sim=30, t_warmup=10, all 4 variants
%
%   P1 test (Session 7):
%     T_sim=15, t_warmup=2, variant_filter=[1 3]

    if nargin < 4 || isempty(T_sim),          T_sim = 30; end
    if nargin < 5 || isempty(t_warmup),       t_warmup = 10; end
    if nargin < 6 || isempty(variant_filter), variant_filter = [1 2 3 4]; end

    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(script_dir);
    addpath(fullfile(project_root, 'model'));
    addpath(fullfile(project_root, 'model', 'config'));
    addpath(fullfile(project_root, 'model', 'wall_effect'));
    addpath(fullfile(project_root, 'model', 'thermal_force'));
    addpath(fullfile(project_root, 'model', 'trajectory'));
    addpath(fullfile(project_root, 'model', 'controller'));
    addpath(script_dir);
    cd(project_root);

    % Define matrix
    scenarios(1) = struct('name', 'near_wall_h25',  'h_init', 2.5);
    scenarios(2) = struct('name', 'free_space_h50', 'h_init', 50);

    % 2026-04-18 Session 5: a_hat quality discovery via a_cov + Pf_init tuning
    Pf_default = [0; 0; 1e-4; 1e-4; 0; 10*(0.0147)^2; 0];
    Pf_small   = [0; 0; 1e-4; 1e-4; 0; 1e-5; 0];
    Pf_smart   = [0; 0; 1e-4; 1e-4; 0; 1e-3; 0];
    variants_all(1) = struct('name','empirical',       'Qz',[0;0;1e4;0.1;0;1e-4;0],   'Rz',[0.01;1.0],   'Pf', Pf_default, 'a_cov', 0.05);
    variants_all(2) = struct('name','emp_acov005',     'Qz',[0;0;1e4;0.1;0;1e-4;0],   'Rz',[0.01;1.0],   'Pf', Pf_default, 'a_cov', 0.005);
    variants_all(3) = struct('name','frozen_correct',  'Qz',[0;0;1;0;0;1e-8;1e-8],    'Rz',[0.397;1.0],  'Pf', Pf_small,   'a_cov', 0.005);
    variants_all(4) = struct('name','frozen_smartPf',  'Qz',[0;0;1;0;0;1e-8;1e-8],    'Rz',[0.397;1.0],  'Pf', Pf_smart,   'a_cov', 0.005);
    variants = variants_all(variant_filter);

    seeds = [12345, 67890, 11111];

    n_sc = numel(scenarios); n_var = numel(variants); n_seed = numel(seeds);
    n_total = n_sc * n_var * n_seed;

    triples = zeros(n_total, 3);
    k = 0;
    for is = 1:n_sc
        for iv = 1:n_var
            for ie = 1:n_seed
                k = k+1;
                triples(k, :) = [is iv ie];
            end
        end
    end

    start_idx = max(1, start_idx);
    end_idx   = min(n_total, end_idx);
    n_run = end_idx - start_idx + 1;

    fprintf('===== verify_qr_positioning_run batch [%d - %d] =====\n', start_idx, end_idx);
    fprintf('Workers: %d runs of 50 total\n', n_run);

    [out_dir, ~, ~] = fileparts(output_file);
    if ~isempty(out_dir) && ~exist(out_dir, 'dir'), mkdir(out_dir); end

    results = cell(n_run, 1);
    t_batch = tic;
    for j = 1:n_run
        idx = start_idx + j - 1;
        is = triples(idx, 1); iv = triples(idx, 2); ie = triples(idx, 3);
        results{j} = run_one(scenarios(is), variants(iv), seeds(ie), idx, n_total, T_sim, t_warmup);
        % Incremental checkpoint after every run so we never lose progress
        t_batch_total = toc(t_batch);
        save(output_file, 'results', 'start_idx', 'end_idx', 'triples', ...
                          'scenarios', 'variants', 'seeds', 't_batch_total', 'j', ...
                          'T_sim', 't_warmup');
    end
    t_batch_total = toc(t_batch);
    fprintf('\nBatch DONE in %.1f s. Saved: %s\n', t_batch_total, output_file);
end

function r = run_one(scenario, variant, seed, idx, n_total, T_sim, t_warmup)
    config = user_config();
    config.theta = 0; config.phi = 0; config.pz = 0;
    config.h_min = 1.1 * 2.25;
    config.enable_wall_effect = true;
    config.h_init = scenario.h_init;
    config.h_bottom = scenario.h_init;   % static positioning
    config.amplitude = 0;
    config.t_hold = 0;
    config.frequency = 1;
    config.n_cycles = 1;
    config.trajectory_type = 'positioning';
    config.ctrl_enable = true;
    config.controller_type = 7;
    config.lambda_c = 0.7;
    % a_cov per-variant (overrides default)
    if isfield(variant, 'a_cov')
        config.a_cov = variant.a_cov;
    else
        config.a_cov = 0.05;
    end
    config.a_pd = 0.05; config.a_prd = 0.05;
    config.epsilon = 0.01;
    config.meas_noise_enable = true;
    % Use main-project sensor noise spec to align with baseline (2026-04-18 fix)
    config.meas_noise_std = [0.00062; 0.000057; 0.00331];
    config.thermal_enable = true;
    config.thermal_seed = seed;
    % Pf_init per-variant (overrides default)
    if isfield(variant, 'Pf')
        config.Pf_init_diag = variant.Pf;
    else
        config.Pf_init_diag = [0; 0; 1e-4; 1e-4; 0; 10*(0.0147)^2; 0];
    end
    config.beta = 0; config.lamdaF = 1.0;
    config.T_sim = T_sim;   % Session 6 default 30s; Session 7 P1 test: 15s
    config.Qz_diag_scaling = variant.Qz;
    config.Rz_diag_scaling = variant.Rz;

    clear motion_control_law_7state trajectory_generator;
    params = calc_simulation_params(config);
    assignin('base', 'params', params);
    assignin('base', 'p0', params.Value.common.p0);
    assignin('base', 'Ts', params.Value.common.Ts);

    fprintf('  [%2d/%d] %-15s + %-13s + seed %5d: ', ...
            idx, n_total, scenario.name, variant.name, seed);
    t_start = tic;
    simOut = sim('model/system_model', 'StopTime', num2str(config.T_sim));
    sim_time = toc(t_start);

    % --- Extract ---
    Ts = params.Value.common.Ts;
    p_d = simOut.p_d_out';
    p_m = simOut.p_m_out';
    ekf = simOut.ekf_out';
    a_hat_x = ekf(1, :);
    a_hat_z = ekf(2, :);
    N = size(p_d, 2);
    t = (0:N-1) * Ts;

    err_x = (p_m(1,:) - p_d(1,:)) * 1000;   % nm
    err_y = (p_m(2,:) - p_d(2,:)) * 1000;
    err_z = (p_m(3,:) - p_d(3,:)) * 1000;
    err_3d = vecnorm(p_m - p_d, 2, 1) * 1000;

    w_hat = params.Value.wall.w_hat;
    pz = params.Value.wall.pz;
    R_p = params.Value.common.R;
    a_nom = Ts / params.Value.common.gamma_N;

    a_true_x = zeros(1, N);
    a_true_z = zeros(1, N);
    for k = 1:N
        h_val = (p_m(:, k)' * w_hat - pz) / R_p;
        if h_val > 1.001
            [c_para, c_perp] = calc_correction_functions(h_val);
        else
            c_para = 15; c_perp = 15;
        end
        a_true_x(k) = a_nom / c_para;
        a_true_z(k) = a_nom / c_perp;
    end

    % --- Steady-state window (configurable via t_warmup arg) ---
    idx_ss = t >= t_warmup;

    % --- Tracking error stats (per-axis nm) ---
    r = struct();
    r.scenario = scenario.name;
    r.variant  = variant.name;
    r.seed     = seed;
    r.h_init   = scenario.h_init;
    r.sim_time_s = sim_time;
    r.idx      = idx;

    r.tracking_mean_x = mean(err_x(idx_ss));
    r.tracking_mean_y = mean(err_y(idx_ss));
    r.tracking_mean_z = mean(err_z(idx_ss));
    r.tracking_std_x  = std(err_x(idx_ss));
    r.tracking_std_y  = std(err_y(idx_ss));
    r.tracking_std_z  = std(err_z(idx_ss));
    r.tracking_rmse_3d = rms(err_3d(idx_ss));

    % --- Theoretical std (closed-loop, includes noise contribution) ---
    % theory_axis = sqrt(C_dpmr_eff * 4kBT * a_axis / a_nom + C_n * sigma2_n)
    % Load lookups
    try
        cdpmr_lookup_path = fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
                                     'test_results', 'verify', 'cdpmr_eff_lookup.mat');
        L = load(cdpmr_lookup_path);
        lc_q = max(min(config.lambda_c, L.lc_grid(end)), L.lc_grid(1));
        % a_x at this h
        h_init_bar = (params.Value.common.p0' * w_hat - pz) / R_p;
        if h_init_bar < 1.001, h_init_bar = 1.001; end
        [c_para_h, c_perp_h] = calc_correction_functions(h_init_bar);
        a_x_h = a_nom / c_para_h;
        a_z_h = a_nom / c_perp_h;
        ar_x = a_x_h / a_nom;
        ar_z = a_z_h / a_nom;
        ar_x_q = max(min(ar_x, L.aratio_grid(end)), L.aratio_grid(1));
        ar_z_q = max(min(ar_z, L.aratio_grid(end)), L.aratio_grid(1));
        Cdpmr_x = interp2(L.aratio_grid, L.lc_grid, L.Cdpmr_tab, ar_x_q, lc_q, 'linear');
        Cdpmr_z = interp2(L.aratio_grid, L.lc_grid, L.Cdpmr_tab, ar_z_q, lc_q, 'linear');
        Cnp_x   = interp2(L.aratio_grid, L.lc_grid, L.Cnp_tab,   ar_x_q, lc_q, 'linear');
        Cnp_z   = interp2(L.aratio_grid, L.lc_grid, L.Cnp_tab,   ar_z_q, lc_q, 'linear');
        sigma2_dXT = 4 * params.Value.ctrl.k_B * params.Value.ctrl.T * a_nom;
        sigma2_n_x_axis = config.meas_noise_std(1)^2;
        sigma2_n_z_axis = config.meas_noise_std(3)^2;
        % del_pmr theory variance (lookup-based, legacy)
        var_dpmr_x = Cdpmr_x * sigma2_dXT * ar_x + Cnp_x * sigma2_n_x_axis;
        var_dpmr_z = Cdpmr_z * sigma2_dXT * ar_z + Cnp_z * sigma2_n_z_axis;
        r.theory_std_dpmr_x_nm = sqrt(var_dpmr_x) * 1000;
        r.theory_std_dpmr_z_nm = sqrt(var_dpmr_z) * 1000;
        r.Cdpmr_x = Cdpmr_x; r.Cdpmr_z = Cdpmr_z;
        r.Cnp_x = Cnp_x; r.Cnp_z = Cnp_z;
    catch ME
        warning('Theory lookup failed: %s', ME.message);
        r.theory_std_dpmr_x_nm = NaN;
        r.theory_std_dpmr_z_nm = NaN;
    end

    % --- Sigma-based theory tracking std + a_hat std (after bug fix + Level 1) ---
    % Direct extraction of Var(delta_x) and Var(a_hat - a_true) per axis from
    % 11-dim Lyapunov. Level 1: rigorous rho_a via compute_rho_a_rigorous
    % (was constant 4; now per-(lc,ar,Q) computed from autocorrelation).
    try
        sigma2_dXT_full = 4 * params.Value.ctrl.k_B * params.Value.ctrl.T * a_nom;
        Q_kf_scale = config.Qz_diag_scaling;
        R_kf_scale = config.Rz_diag_scaling;
        a_cov_local = config.a_cov;
        chi_sq = 2 * a_cov_local / (2 - a_cov_local);
        opts_sigma = struct('f0', 0, 'verbose', false);
        for ax_label = {'x', 'z'}
            if strcmp(ax_label{1}, 'x')
                a_phys_use = a_x_h; sigma2_n_use = sigma2_n_x_axis;
            else
                a_phys_use = a_z_h; sigma2_n_use = sigma2_n_z_axis;
            end
            % Pass 1: compute Sigma without Sigma_na (so rho_a only uses state autocorrelation)
            opts_sigma.physical_scaling = struct( ...
                'sigma2_dXT',     sigma2_dXT_full, ...
                'a_phys',         a_phys_use, ...
                'a_nom',          a_nom, ...
                'sigma2_n',       sigma2_n_use, ...
                'actual_a_m_var', 0);
            [~, ~, ~, A_aug, dgn] = compute_7state_cdpmr_eff(config.lambda_c, 0, ...
                config.a_pd, Q_kf_scale, R_kf_scale, opts_sigma);

            % Compute rigorous rho_a
            rho_a_rig = compute_rho_a_rigorous(A_aug, dgn, config.a_pd, ...
                a_cov_local, sigma2_n_use, 200);

            % Pass 2: include Sigma_na (a_m noise) and Sigma_mult (f_d coupling, Level 3)
            actual_a_m_var = chi_sq * rho_a_rig * a_phys_use^2;
            S_phys_lv1 = dgn.Sigma_aug_phys ...
                       + actual_a_m_var * dgn.Sigma_na;
            % Level 3: multiplicative f_d coupling (-e6 * f_d_nom enters delta_x).
            % Compute Var(f_d_nom), Cov(e6, f_d_nom) from Sigma_phys_lv1.
            % f_d_nom = ((1-lc)*del_p3_hat - d_hat) / a_phys
            %   del_p3_hat = delta_x[k-2] + e3 = state(idx_dx_d2) + state(idx_e(3))
            %   d_hat = e4 = state(idx_e(4))
            i_dx2 = dgn.idx_dx_d2;  i_e3 = dgn.idx_e(3);
            i_e4  = dgn.idx_e(4);   i_e6 = dgn.idx_e(6);
            lc = config.lambda_c;
            % Selector vector for f_d_nom: c_fd' * x_aug = ((1-lc)*(state(i_dx2)+state(i_e3)) - state(i_e4)) / a_phys
            c_fd = zeros(size(S_phys_lv1, 1), 1);
            c_fd(i_dx2) = (1-lc) / a_phys_use;
            c_fd(i_e3)  = (1-lc) / a_phys_use;
            c_fd(i_e4)  = -1 / a_phys_use;
            Var_fd_nom = c_fd' * S_phys_lv1 * c_fd;
            % Selector for e6
            c_e6 = zeros(size(S_phys_lv1, 1), 1);
            c_e6(i_e6) = 1;
            Var_e6_phys = c_e6' * S_phys_lv1 * c_e6;
            Cov_e6_fd   = c_e6' * S_phys_lv1 * c_fd;
            % Isserlis: Var(-e6*f_d_nom) = Var(e6)*Var(f_d) + Cov^2
            Var_input_extra = Var_e6_phys * Var_fd_nom + Cov_e6_fd^2;
            % Add Sigma_mult contribution scaled by Var_input_extra
            S_phys_lv3 = S_phys_lv1 + Var_input_extra * dgn.Sigma_mult;

            Var_dx_lv3 = S_phys_lv3(dgn.idx_dx, dgn.idx_dx);
            Var_e6_lv3 = S_phys_lv3(dgn.idx_e(6), dgn.idx_e(6));

            % Level 5: a_true fluctuation via p_m drift × wall sensitivity.
            % For positioning, a_true_axis = a_nom/c_axis(h_bar). p_m_z drift
            % -> h_bar drift -> c changes -> a_true fluctuates. Measured
            % e_a = (a_hat - a_true)/a_true includes this a_true fluctuation.
            % For z axis (wall-normal): c = c_perp; sensitivity is large near wall.
            % For x axis (tangential): c = c_para; sensitivity mild.
            dh = 0.001;
            [c_para_p, c_perp_p] = calc_correction_functions(h_init_bar + dh);
            [c_para_m, c_perp_m] = calc_correction_functions(max(h_init_bar - dh, 1.001));
            dcperp_dhbar = (c_perp_p - c_perp_m) / (2*dh);
            dcpara_dhbar = (c_para_p - c_para_m) / (2*dh);
            if strcmp(ax_label{1}, 'x')
                sens = abs(dcpara_dhbar) / c_para_h;     % 1/c_para · |dc_para/dh_bar|
            else
                sens = abs(dcperp_dhbar) / c_perp_h;     % 1/c_perp · |dc_perp/dh_bar|
            end
            % Tracking std (p_m) in um from Var(delta_x) which equals Var(p_m-pd)
            std_pm_um = sqrt(max(Var_dx_lv3, 0));
            % Relative std of a_true: sensitivity · std(h_bar) = sensitivity · std(p_m_z)/R
            % NOTE: only p_m_z affects h_bar; tangential axes use SAME p_m_z std (projected)
            % but for simplicity use axis's own tracking std (valid for w_hat=[0;0;1])
            R_p_local = params.Value.common.R;
            rel_std_a_true_wall = sens * std_pm_um / R_p_local;

            % Combine (assume independent): total Var(e_a) = Var(e_a_ekf) + Var(a_true_wall)
            Var_a_rel_total = (sqrt(max(Var_e6_lv3, 0)) / max(a_phys_use, eps))^2 ...
                            + rel_std_a_true_wall^2;
            std_a_rel_pct = 100 * sqrt(Var_a_rel_total);

            std_dx_nm = sqrt(max(Var_dx_lv3, 0)) * 1000;
            if strcmp(ax_label{1}, 'x')
                r.tracking_std_theory_x_nm = std_dx_nm;
                r.ahat_std_theory_x_pct = std_a_rel_pct;
                r.rho_a_x = rho_a_rig;
            else
                r.tracking_std_theory_z_nm = std_dx_nm;
                r.ahat_std_theory_z_pct = std_a_rel_pct;
                r.rho_a_z = rho_a_rig;
            end
        end
    catch ME
        warning('Sigma-based theory failed: %s', ME.message);
        r.tracking_std_theory_x_nm = NaN;
        r.tracking_std_theory_z_nm = NaN;
        r.ahat_std_theory_x_pct = NaN;
        r.ahat_std_theory_z_pct = NaN;
    end

    % --- a_hat error stats (per-axis %) ---
    e_a_x = (a_hat_x - a_true_x) ./ max(a_true_x, eps);
    e_a_z = (a_hat_z - a_true_z) ./ max(a_true_z, eps);
    r.ahat_bias_x_pct = 100 * mean(e_a_x(idx_ss));
    r.ahat_bias_z_pct = 100 * mean(e_a_z(idx_ss));
    r.ahat_std_x_pct  = 100 * std(e_a_x(idx_ss));
    r.ahat_std_z_pct  = 100 * std(e_a_z(idx_ss));
    r.ahat_max_x_pct  = 100 * max(abs(e_a_x(idx_ss)));
    r.ahat_max_z_pct  = 100 * max(abs(e_a_z(idx_ss)));
    r.ahat_rms_x_pct  = 100 * sqrt(mean(e_a_x(idx_ss).^2));
    r.ahat_rms_z_pct  = 100 * sqrt(mean(e_a_z(idx_ss).^2));

    % --- Spike (within first 1000 samples = 0.625 s) ---
    n_first = min(1000, N);
    [mx_x, kx] = max(abs(e_a_x(1:n_first)));
    [mx_z, kz] = max(abs(e_a_z(1:n_first)));
    r.spike_x_pct = 100 * mx_x;
    r.spike_x_t   = t(kx);
    r.spike_z_pct = 100 * mx_z;
    r.spike_z_t   = t(kz);

    % NaN sentinel
    r.nan_count = sum(isnan(p_m(:))) + sum(isnan(ekf(:)));

    fprintf('done %.1fs | 3D RMSE %.1fnm | a_hat_z bias %+.1f%% std %.1f%% spike %.0f%%\n', ...
        sim_time, r.tracking_rmse_3d, r.ahat_bias_z_pct, r.ahat_std_z_pct, r.spike_z_pct);
end
