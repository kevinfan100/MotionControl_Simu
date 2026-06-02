% verify_acov_and_tau.m
% Two complementary tests:
%   (1) a_cov sweep: verify formula's dependence on a_cov is correct
%       a_cov ∈ {0.005, 0.05, 0.5}, fixed a_pd=0.05, h=20um, 3 seeds
%   (2) tau_max sensitivity: test if extending τ_max reduces residual
%       Use existing cached grid sims (no new simulation cost)
%       Compare:
%         - closed-form with tau_max=30 (numerical sum)
%         - closed-form with tau_max=∞ (geometric tail closed form)
%         - empirical with tau_max ∈ {30, 50, 100, 300}

clear; close all; clc;
clear motion_control_law motion_control_law_eq17_core ...
      motion_control_law_7state trajectory_generator calc_thermal_force;
[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(fileparts(script_dir));
cd(project_root);
addpath(fullfile(project_root, 'model'));
addpath(fullfile(project_root, 'model', 'config'));
addpath(fullfile(project_root, 'model', 'wall_effect'));
addpath(fullfile(project_root, 'model', 'thermal_force'));
addpath(fullfile(project_root, 'model', 'trajectory'));
addpath(fullfile(project_root, 'model', 'controller'));
addpath(fullfile(project_root, 'model', 'dual_track'));

constants = physical_constants();
kBT       = constants.k_B * constants.T;
R_radius  = constants.R;
gamma_N   = constants.gamma_N;
Ts        = 1/1600;
a_nom     = Ts/gamma_N;
sigma_n_axes  = [0.00062; 0.000057; 0.00331];
sigma2_n_axes = sigma_n_axes.^2;
lambda_c  = 0.7;
T_sim     = 20;
h_static  = 20;
ax_names  = {'X', 'Y', 'Z'};
save_dir  = fullfile(project_root, 'test_results', 'learn_variance');

%% ============== TEST 1: a_cov sweep ==============
a_cov_list = [0.005, 0.05, 0.5];
seed_list_1 = [42, 100, 200];
n_seed_1 = length(seed_list_1);
n_acov = length(a_cov_list);

cache1 = fullfile(save_dir, 'acov_sweep_9sims.mat');
if ~exist(cache1, 'file')
    fprintf('=== TEST 1: a_cov sweep (9 sims, ~3 min) ===\n');
    delta_x_m_acov   = cell(n_acov, n_seed_1);
    sigma2_ctrl_acov = cell(n_acov, n_seed_1);
    tout_ref_1 = [];
    for ic = 1:n_acov
        a_cov = a_cov_list(ic);
        for is = 1:n_seed_1
            seed = seed_list_1(is);
            fprintf('  a_cov=%.4f seed=%d ... ', a_cov, seed);
            config = user_config();
            config.trajectory_type   = 'positioning';
            config.h_init            = h_static;
            config.T_sim             = T_sim;
            config.controller_type   = 'eq17_7state';
            config.iir_warmup_mode   = 'prefill';
            config.thermal_enable    = true;
            config.meas_noise_enable = true;
            config.meas_noise_std    = sigma_n_axes;
            config.a_pd              = 0.05;
            config.a_cov             = a_cov;
            opts = struct('seed', seed, 'verbose', false, 'collect_diag', true);
            clear motion_control_law motion_control_law_eq17_core ...
                  motion_control_law_7state trajectory_generator calc_thermal_force;
            ts = tic;
            simOut = run_pure_simulation(config, opts);
            fprintf('%.1fs\n', toc(ts));
            delta_x_m_acov{ic, is}   = simOut.diag.delta_x_m;
            sigma2_ctrl_acov{ic, is} = simOut.diag.sigma2_dxr_hat;
            if isempty(tout_ref_1); tout_ref_1 = simOut.tout; end
        end
    end
    save(cache1, 'delta_x_m_acov', 'sigma2_ctrl_acov', 'tout_ref_1', ...
         'a_cov_list', 'seed_list_1', '-v7.3');
else
    fprintf('Load cache: %s\n', cache1);
    load(cache1);
end

%% Analyze a_cov sweep
N_t = length(tout_ref_1);
warmup_idx_1 = tout_ref_1 >= 2;
a_pd = 0.05;
alpha = 1 - a_pd; beta = lambda_c; c = 1 - beta;
tau_max_test = 30;

% Closed-form ρ_T, ρ_N (a_pd, λc dependent only)
R_T = zeros(tau_max_test+2, 1);
for tau = 0:tau_max_test+1
    T1 = alpha^(tau+2)/(1-alpha^2);
    if tau <= 3
        T2 = alpha^(5-tau)*c/((1-alpha^2)*(1-alpha*beta));
    else
        T2 = (alpha^2*c)/(alpha-beta) * ...
             (alpha^(tau-2)/(1-alpha^2) - beta^(tau-2)/(1-alpha*beta));
    end
    T3 = alpha^(tau+5)*c/((1-alpha^2)*(1-alpha*beta));
    R_g = 1/(alpha-beta)^2 * ( ...
            alpha^(tau+2)/(1-alpha^2) + beta^(tau+2)/(1-beta^2) ...
          - alpha*beta*(alpha^tau+beta^tau)/(1-alpha*beta));
    T4 = alpha^2*c^2*R_g;
    R_T(tau+1) = T1 - T2 - T3 + T4;
end
R_N = zeros(tau_max_test+1, 1);
for tau = 0:tau_max_test
    if tau == 0
        R_N(1) = 2*R_T(1) - 2*R_T(2);
    else
        R_N(tau+1) = 2*R_T(tau+1) - R_T(tau) - R_T(tau+2);
    end
end
R_T = R_T(1:tau_max_test+1);
C_dpmr_cf = R_T(1); C_n_cf = R_N(1);
rho_T_cf = R_T/C_dpmr_cf; rho_N_cf = R_N/C_n_cf;

% Static h=20 weights per axis
h_bar = h_static/R_radius;
[c_para, c_perp] = calc_correction_functions(max(h_bar, 1.001));
a_axes_static = [a_nom/c_para; a_nom/c_para; a_nom/c_perp];
rho_mix = zeros(tau_max_test+1, 3);
for ax = 1:3
    sig_T = C_dpmr_cf*4*kBT*a_axes_static(ax);
    sig_n = C_n_cf*sigma2_n_axes(ax);
    sig_tot = sig_T + sig_n;
    w_T = sig_T/sig_tot; w_n = sig_n/sig_tot;
    rho_mix(:, ax) = w_T*rho_T_cf + w_n*rho_N_cf;
end

% Per a_cov: compute R22 ratio
fprintf('\n=========== TEST 1 results: a_cov sweep, h=20um, a_pd=0.05 ===========\n');
fprintf('a_cov  | R22_prefactor | IF_eff_cf(Z) | ratio_new_X | ratio_new_Y | ratio_new_Z\n');
fprintf('-------|---------------|--------------|-------------|-------------|------------\n');
for ic = 1:n_acov
    a_cov = a_cov_list(ic);
    s_ewma = 1 - a_cov;
    R22_prefactor = 2*a_cov/(2-a_cov);
    IF_eff_cf = zeros(3, 1);
    for ax = 1:3
        IF_eff_cf(ax) = 1 + 2*sum(rho_mix(2:end, ax).^2 .* s_ewma.^(1:tau_max_test)');
    end
    ratios = zeros(n_seed_1, 3);
    for is = 1:n_seed_1
        sigma2_ctrl = sigma2_ctrl_acov{ic, is};
        for ax = 1:3
            y = sigma2_ctrl(warmup_idx_1, ax);
            t = tout_ref_1(warmup_idx_1);
            p = polyfit(t, y, 1);
            y_resid = y - polyval(p, t);
            R22_emp = var(y_resid);
            sig_mean = mean(y);
            R22_th = R22_prefactor * IF_eff_cf(ax) * sig_mean^2;
            ratios(is, ax) = R22_emp / R22_th;
        end
    end
    fprintf(' %.3f | %.4f        | %.4f       | %.3f±%.3f  | %.3f±%.3f  | %.3f±%.3f\n', ...
            a_cov, R22_prefactor, IF_eff_cf(3), ...
            mean(ratios(:,1)), std(ratios(:,1)), ...
            mean(ratios(:,2)), std(ratios(:,2)), ...
            mean(ratios(:,3)), std(ratios(:,3)));
end

%% ============== TEST 2: tau_max sensitivity ==============
% Use cached positioning grid (27 sims)
cache2 = fullfile(save_dir, 'positioning_grid_27sims.mat');
if exist(cache2, 'file')
    fprintf('\n=== TEST 2: tau_max sensitivity (use cached grid) ===\n');
    S = load(cache2);
    delta_x_m_grid    = S.delta_x_m_grid;
    sigma2_ctrl_grid  = S.sigma2_ctrl_grid;
    tout_ref_2 = S.tout_ref;
    h_list_2 = S.h_list;
    a_pd_list_2 = S.a_pd_list;
    seed_list_2 = S.seed_list;
    warmup_idx_2 = tout_ref_2 >= 2;
    N_t_2 = length(tout_ref_2);
    n_h_2 = length(h_list_2); n_apd_2 = length(a_pd_list_2); n_seed_2 = length(seed_list_2);

    a_cov_test = 0.05;
    s_ewma = 1 - a_cov_test;
    R22_prefactor = 2*a_cov_test/(2-a_cov_test);

    tau_max_list = [30, 50, 100, 300];

    fprintf('\n=========== TEST 2: How IF_eff_cf changes with tau_max ===========\n');
    fprintf('(closed-form, h=20um, Z axis, varied a_pd)\n\n');
    fprintf('a_pd   | tau_max=30 | tau_max=50 | tau_max=100 | tau_max=300 | tau_max=∞ (geom)\n');
    fprintf('-------|------------|------------|-------------|-------------|------------------\n');
    h_static_test = 20;
    h_bar = h_static_test/R_radius;
    [c_para, c_perp] = calc_correction_functions(max(h_bar, 1.001));
    a_axes_test = [a_nom/c_para; a_nom/c_para; a_nom/c_perp];

    for ip = 1:n_apd_2
        a_pd_t = a_pd_list_2(ip);
        alpha_t = 1 - a_pd_t;
        c_t = 1 - beta;

        % Compute ρ_mix for this a_pd (need long tau)
        tau_long = 500;
        R_T_long = zeros(tau_long+2, 1);
        for tau = 0:tau_long+1
            T1 = alpha_t^(tau+2)/(1-alpha_t^2);
            if tau <= 3
                T2 = alpha_t^(5-tau)*c_t/((1-alpha_t^2)*(1-alpha_t*beta));
            else
                T2 = (alpha_t^2*c_t)/(alpha_t-beta) * ...
                     (alpha_t^(tau-2)/(1-alpha_t^2) - beta^(tau-2)/(1-alpha_t*beta));
            end
            T3 = alpha_t^(tau+5)*c_t/((1-alpha_t^2)*(1-alpha_t*beta));
            R_g = 1/(alpha_t-beta)^2 * ( ...
                    alpha_t^(tau+2)/(1-alpha_t^2) + beta^(tau+2)/(1-beta^2) ...
                  - alpha_t*beta*(alpha_t^tau+beta^tau)/(1-alpha_t*beta));
            T4 = alpha_t^2*c_t^2*R_g;
            R_T_long(tau+1) = T1 - T2 - T3 + T4;
        end
        R_N_long = zeros(tau_long+1, 1);
        for tau = 0:tau_long
            if tau == 0
                R_N_long(1) = 2*R_T_long(1) - 2*R_T_long(2);
            else
                R_N_long(tau+1) = 2*R_T_long(tau+1) - R_T_long(tau) - R_T_long(tau+2);
            end
        end
        R_T_long = R_T_long(1:tau_long+1);
        rho_T_long = R_T_long/R_T_long(1);
        rho_N_long = R_N_long/R_N_long(1);

        % Z axis weights
        sig_T = R_T_long(1)*4*kBT*a_axes_test(3);
        sig_n = R_N_long(1)*sigma2_n_axes(3);
        sig_tot = sig_T + sig_n;
        w_T = sig_T/sig_tot; w_n = sig_n/sig_tot;
        rho_mix_long = w_T*rho_T_long + w_n*rho_N_long;

        % Compute IF_eff at various tau_max
        if_at_taumax = zeros(length(tau_max_list), 1);
        for k = 1:length(tau_max_list)
            tm = tau_max_list(k);
            if_at_taumax(k) = 1 + 2*sum(rho_mix_long(2:tm+1).^2 .* s_ewma.^(1:tm)');
        end
        % "Infinite" via tau_max=tau_long (already converged)
        if_inf = 1 + 2*sum(rho_mix_long(2:end).^2 .* s_ewma.^(1:tau_long)');

        fprintf(' %.3f |  %.4f   |  %.4f   |   %.4f   |   %.4f   |   %.4f\n', ...
                a_pd_t, if_at_taumax(1), if_at_taumax(2), if_at_taumax(3), ...
                if_at_taumax(4), if_inf);
    end

    % R22 ratio with tau_max=∞ closed form vs empirical (with various tau)
    fprintf('\n=========== TEST 2: R22 ratio under tau_max variation ===========\n');
    fprintf('(h=20um, a_pd=0.05, Z axis, %d seeds pooled, a_cov=0.05)\n', n_seed_2);
    fprintf('Method                           | ratio (Z axis)\n');
    fprintf('---------------------------------|---------------\n');

    ih = 2;          % h=20um
    ip = 2;          % a_pd=0.05
    alpha_2 = 1 - a_pd_list_2(ip);
    tau_long = 500;
    R_T_long = zeros(tau_long+2, 1);
    c_t = 1 - beta;
    for tau = 0:tau_long+1
        T1 = alpha_2^(tau+2)/(1-alpha_2^2);
        if tau <= 3
            T2 = alpha_2^(5-tau)*c_t/((1-alpha_2^2)*(1-alpha_2*beta));
        else
            T2 = (alpha_2^2*c_t)/(alpha_2-beta) * ...
                 (alpha_2^(tau-2)/(1-alpha_2^2) - beta^(tau-2)/(1-alpha_2*beta));
        end
        T3 = alpha_2^(tau+5)*c_t/((1-alpha_2^2)*(1-alpha_2*beta));
        R_g = 1/(alpha_2-beta)^2 * ( ...
                alpha_2^(tau+2)/(1-alpha_2^2) + beta^(tau+2)/(1-beta^2) ...
              - alpha_2*beta*(alpha_2^tau+beta^tau)/(1-alpha_2*beta));
        T4 = alpha_2^2*c_t^2*R_g;
        R_T_long(tau+1) = T1 - T2 - T3 + T4;
    end
    R_N_long = zeros(tau_long+1, 1);
    for tau = 0:tau_long
        if tau == 0
            R_N_long(1) = 2*R_T_long(1) - 2*R_T_long(2);
        else
            R_N_long(tau+1) = 2*R_T_long(tau+1) - R_T_long(tau) - R_T_long(tau+2);
        end
    end
    R_T_long = R_T_long(1:tau_long+1);
    rho_T_long = R_T_long/R_T_long(1);
    rho_N_long = R_N_long/R_N_long(1);

    sig_T = R_T_long(1)*4*kBT*a_axes_test(3);
    sig_n = R_N_long(1)*sigma2_n_axes(3);
    sig_tot = sig_T + sig_n;
    w_T = sig_T/sig_tot; w_n = sig_n/sig_tot;
    rho_mix_long = w_T*rho_T_long + w_n*rho_N_long;

    % IF_eff closed-form at various tau_max
    IF_cf_30  = 1 + 2*sum(rho_mix_long(2:31).^2 .* s_ewma.^(1:30)');
    IF_cf_inf = 1 + 2*sum(rho_mix_long(2:end).^2 .* s_ewma.^(1:tau_long)');

    % Empirical R22 from cells
    ratios_emp_tau = zeros(length(tau_max_list)+2, 1);
    ratios_emp_tau_std = zeros(length(tau_max_list)+2, 1);

    % We compare empirical R22_emp / (R22_prefactor * IF_eff_at_taumax * sigma2_mean^2)
    % where IF_eff is closed-form at each tau_max
    IF_cf_test = [if_at_taumax_main(rho_mix_long, s_ewma, tau_max_list), IF_cf_inf];

    for is = 1:n_seed_2
        sigma2_ctrl = sigma2_ctrl_grid{ih, ip, is};
        y = sigma2_ctrl(warmup_idx_2, 3);
        t = tout_ref_2(warmup_idx_2);
        p = polyfit(t, y, 1);
        y_resid = y - polyval(p, t);
        R22_emp_z = var(y_resid);
        sig_mean = mean(y);
        for kk = 1:length(tau_max_list)+1
            R22_th = R22_prefactor * IF_cf_test(kk) * sig_mean^2;
            ratios_emp_tau(kk) = ratios_emp_tau(kk) + R22_emp_z / R22_th / n_seed_2;
        end
    end

    for kk = 1:length(tau_max_list)
        fprintf(' tau_max=%-3d (closed-form sum)    |   %.4f\n', tau_max_list(kk), ratios_emp_tau(kk));
    end
    fprintf(' tau_max=∞ (geom tail extrapolation)|   %.4f\n', ratios_emp_tau(length(tau_max_list)+1));

else
    fprintf('Cache for grid not found, skipping TEST 2\n');
end

%% Helper to compute IF_eff at multiple tau_max values
% Inline helper at end of file
fprintf('\n========== SUMMARY ==========\n');
fprintf('(Tests done. See plots for visual.)\n');

function IF_arr = if_at_taumax_main(rho_mix_long, s_ewma, tau_max_list)
    IF_arr = zeros(length(tau_max_list), 1);
    for k = 1:length(tau_max_list)
        tm = tau_max_list(k);
        IF_arr(k) = 1 + 2*sum(rho_mix_long(2:tm+1).^2 .* s_ewma.^(1:tm)');
    end
    IF_arr = IF_arr(:)';
end
