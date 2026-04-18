function verify_qr_positioning_run(start_idx, end_idx, output_file)
%VERIFY_QR_POSITIONING_RUN Worker: runs subset of (scenario, variant, seed) matrix
%
%   Usage from Bash:
%     matlab -batch "verify_qr_positioning_run(1, 13, 'test_results/verify/qr_pos_b1.mat')"
%
%   Matrix iteration order (50 total, [scenario, variant, seed] varying fastest in seed):
%     for is = 1:n_sc, for iv = 1:n_var, for ie = 1:n_seed -> idx=1..50
%
%   Hardcoded:
%     scenarios = {h=2.5, h=50}
%     variants  = {empirical, beta, Bprime, Bprime_Remp, Qemp_Rderived}
%     seeds     = [12345 67890 11111 22222 33333]

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
    variants(1) = struct('name','empirical',       'Qz',[0;0;1e4;0.1;0;1e-4;0],   'Rz',[0.01;1.0],   'Pf', Pf_default, 'a_cov', 0.05);
    variants(2) = struct('name','emp_acov005',     'Qz',[0;0;1e4;0.1;0;1e-4;0],   'Rz',[0.01;1.0],   'Pf', Pf_default, 'a_cov', 0.005);
    variants(3) = struct('name','frozen_correct',  'Qz',[0;0;1;0;0;1e-8;1e-8],    'Rz',[0.397;1.0],  'Pf', Pf_small,   'a_cov', 0.005);
    variants(4) = struct('name','frozen_smartPf',  'Qz',[0;0;1;0;0;1e-8;1e-8],    'Rz',[0.397;1.0],  'Pf', Pf_smart,   'a_cov', 0.005);

    seeds = [12345, 67890, 11111];

    n_sc = numel(scenarios); n_var = numel(variants); n_seed = numel(seeds);
    n_total = n_sc * n_var * n_seed;  % 50

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
        results{j} = run_one(scenarios(is), variants(iv), seeds(ie), idx, n_total);
        % Incremental checkpoint after every run so we never lose progress
        t_batch_total = toc(t_batch);
        save(output_file, 'results', 'start_idx', 'end_idx', 'triples', ...
                          'scenarios', 'variants', 'seeds', 't_batch_total', 'j');
    end
    t_batch_total = toc(t_batch);
    fprintf('\nBatch DONE in %.1f s. Saved: %s\n', t_batch_total, output_file);
end

function r = run_one(scenario, variant, seed, idx, n_total)
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
    config.T_sim = 15;
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

    % --- Steady-state window (reduced from P2 standard 10s for speed) ---
    t_warmup = 5;
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
        sigma2_n = config.meas_noise_std(3)^2;
        % del_pmr theory variance
        var_dpmr_x = Cdpmr_x * sigma2_dXT * ar_x + Cnp_x * sigma2_n;
        var_dpmr_z = Cdpmr_z * sigma2_dXT * ar_z + Cnp_z * sigma2_n;
        r.theory_std_dpmr_x_nm = sqrt(var_dpmr_x) * 1000;
        r.theory_std_dpmr_z_nm = sqrt(var_dpmr_z) * 1000;
        r.Cdpmr_x = Cdpmr_x; r.Cdpmr_z = Cdpmr_z;
        r.Cnp_x = Cnp_x; r.Cnp_z = Cnp_z;
    catch ME
        warning('Theory lookup failed: %s', ME.message);
        r.theory_std_dpmr_x_nm = NaN;
        r.theory_std_dpmr_z_nm = NaN;
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
