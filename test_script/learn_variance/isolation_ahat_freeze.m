% isolation_ahat_freeze.m
% Isolation test #1: Lock a_hat to true mobility (a_hat_freeze) and remeasure
% R22 ratio. If EKF nonlinear feedback (1/a_hat in control law) is the main
% residual source, freezing a_hat should make ratio → 1.0.
%
% Test: static h=20um, a_pd=0.05, a_cov=0.05, 5 seeds.
%       Compare baseline (no freeze) vs freeze=a_z_true.

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

%% Config
seed_list   = [42, 100, 200, 300, 400];
n_seed      = length(seed_list);
h_static    = 20;
T_sim       = 20;
sigma_n_axes  = [0.00062; 0.000057; 0.00331];
sigma2_n_axes = sigma_n_axes.^2;
ax_names    = {'X', 'Y', 'Z'};
lambda_c    = 0.7;
a_pd        = 0.05;
a_cov       = 0.05;
tau_max     = 30;

constants = physical_constants();
kBT       = constants.k_B * constants.T;
R_radius  = constants.R;
gamma_N   = constants.gamma_N;
Ts        = 1/1600;
a_nom     = Ts/gamma_N;

% Compute true a_axes at h=20
h_bar = h_static / R_radius;
[c_para, c_perp] = calc_correction_functions(max(h_bar, 1.001));
a_axes_true = [a_nom/c_para; a_nom/c_para; a_nom/c_perp];
fprintf('h=%dum: h_bar=%.3f, c_para=%.4f, c_perp=%.4f\n', ...
        h_static, h_bar, c_para, c_perp);
fprintf('  a_true: X=%.4e Y=%.4e Z=%.4e [um/pN]\n\n', ...
        a_axes_true(1), a_axes_true(2), a_axes_true(3));

save_dir   = fullfile(project_root, 'test_results', 'learn_variance');
cache_path = fullfile(save_dir, 'isolation_ahat_freeze_5seeds.mat');

%% Run sims (cached): baseline + freeze
if ~exist(cache_path, 'file')
    fprintf('=== Run %d sims for baseline + freeze test (~%.1f min) ===\n', ...
            2*n_seed, 2*n_seed*18/60);
    sigma2_ctrl_base   = cell(n_seed, 1);
    sigma2_ctrl_freeze = cell(n_seed, 1);
    delta_x_m_base     = cell(n_seed, 1);
    delta_x_m_freeze   = cell(n_seed, 1);
    a_hat_base         = cell(n_seed, 1);
    a_hat_freeze_sav   = cell(n_seed, 1);
    tout_ref = [];

    for is = 1:n_seed
        seed = seed_list(is);
        config = user_config();
        config.trajectory_type   = 'positioning';
        config.h_init            = h_static;
        config.T_sim             = T_sim;
        config.controller_type   = 'eq17_7state';
        config.iir_warmup_mode   = 'prefill';
        config.thermal_enable    = true;
        config.meas_noise_enable = true;
        config.meas_noise_std    = sigma_n_axes;
        config.a_pd              = a_pd;
        config.a_cov             = a_cov;

        % --- Baseline: standard EKF (a_hat estimated) ---
        opts_b = struct('seed', seed, 'verbose', false, 'collect_diag', true);
        fprintf('  seed=%d baseline ... ', seed);
        clear motion_control_law motion_control_law_eq17_core ...
              motion_control_law_7state trajectory_generator calc_thermal_force;
        ts = tic;
        simOut_b = run_pure_simulation(config, opts_b);
        fprintf('%.1fs\n', toc(ts));
        sigma2_ctrl_base{is}   = simOut_b.diag.sigma2_dxr_hat;
        delta_x_m_base{is}     = simOut_b.diag.delta_x_m;
        a_hat_base{is}         = simOut_b.diag.a_hat;

        % --- Freeze: lock a_hat = a_axes_true ---
        opts_f = struct('seed', seed, 'verbose', false, 'collect_diag', true, ...
                        'a_hat_freeze', a_axes_true);
        fprintf('  seed=%d freeze   ... ', seed);
        clear motion_control_law motion_control_law_eq17_core ...
              motion_control_law_7state trajectory_generator calc_thermal_force;
        ts = tic;
        simOut_f = run_pure_simulation(config, opts_f);
        fprintf('%.1fs\n', toc(ts));
        sigma2_ctrl_freeze{is} = simOut_f.diag.sigma2_dxr_hat;
        delta_x_m_freeze{is}   = simOut_f.diag.delta_x_m;
        a_hat_freeze_sav{is}   = simOut_f.diag.a_hat;

        if isempty(tout_ref); tout_ref = simOut_b.tout; end
    end
    save(cache_path, 'sigma2_ctrl_base', 'sigma2_ctrl_freeze', ...
         'delta_x_m_base', 'delta_x_m_freeze', ...
         'a_hat_base', 'a_hat_freeze_sav', 'tout_ref', ...
         'a_axes_true', 'h_static', 'seed_list', '-v7.3');
    fprintf('Cached → %s\n', cache_path);
else
    fprintf('Load cache → %s\n', cache_path);
    load(cache_path);
end

N_t = length(tout_ref);
warmup_idx = tout_ref >= 2;

%% Sanity: verify freeze actually held a_hat constant
fprintf('\n=== Sanity: a_hat range across trace (seed 1) ===\n');
fprintf(' axis | baseline (min, max, mean)            | freeze (min, max, mean)\n');
for ax = 1:3
    a_b = a_hat_base{1}(:, ax);
    a_f = a_hat_freeze_sav{1}(:, ax);
    fprintf('  %s   | [%.4e, %.4e, mean=%.4e]   | [%.4e, %.4e, mean=%.4e]\n', ...
            ax_names{ax}, min(a_b), max(a_b), mean(a_b), ...
            min(a_f), max(a_f), mean(a_f));
end
fprintf('  → freeze should show min=max=mean=a_true ✓\n');

%% Compute closed-form IF_eff (same as before, for theory comparison)
alpha = 1 - a_pd; beta = lambda_c; c = 1 - beta;
R_T = zeros(tau_max+2, 1);
for tau = 0:tau_max+1
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
R_N = zeros(tau_max+1, 1);
for tau = 0:tau_max
    if tau == 0
        R_N(1) = 2*R_T(1) - 2*R_T(2);
    else
        R_N(tau+1) = 2*R_T(tau+1) - R_T(tau) - R_T(tau+2);
    end
end
R_T = R_T(1:tau_max+1);
C_dpmr_cf = R_T(1); C_n_cf = R_N(1);
rho_T_cf = R_T/C_dpmr_cf; rho_N_cf = R_N/C_n_cf;

s_ewma = 1 - a_cov;
R22_prefactor = 2*a_cov/(2-a_cov);

IF_eff_cf = zeros(3, 1);
for ax = 1:3
    sig_T = C_dpmr_cf * 4*kBT*a_axes_true(ax);
    sig_n = C_n_cf * sigma2_n_axes(ax);
    sig_tot = sig_T + sig_n;
    w_T = sig_T/sig_tot; w_n = sig_n/sig_tot;
    rho_mix = w_T*rho_T_cf + w_n*rho_N_cf;
    IF_eff_cf(ax) = 1 + 2*sum(rho_mix(2:end).^2 .* s_ewma.^(1:tau_max)');
end

%% Compute R22 ratio for baseline and freeze
ratios_base   = zeros(n_seed, 3);
ratios_freeze = zeros(n_seed, 3);
sigma2_mean_base   = zeros(n_seed, 3);
sigma2_mean_freeze = zeros(n_seed, 3);

for is = 1:n_seed
    sig_b = sigma2_ctrl_base{is};
    sig_f = sigma2_ctrl_freeze{is};
    for ax = 1:3
        % baseline
        y_b = sig_b(warmup_idx, ax);
        t_b = tout_ref(warmup_idx);
        p_b = polyfit(t_b, y_b, 1);
        R22_emp_b = var(y_b - polyval(p_b, t_b));
        sig2_mean_b = mean(y_b);
        R22_th_b = R22_prefactor * IF_eff_cf(ax) * sig2_mean_b^2;
        ratios_base(is, ax) = R22_emp_b / R22_th_b;
        sigma2_mean_base(is, ax) = sig2_mean_b;

        % freeze
        y_f = sig_f(warmup_idx, ax);
        t_f = tout_ref(warmup_idx);
        p_f = polyfit(t_f, y_f, 1);
        R22_emp_f = var(y_f - polyval(p_f, t_f));
        sig2_mean_f = mean(y_f);
        R22_th_f = R22_prefactor * IF_eff_cf(ax) * sig2_mean_f^2;
        ratios_freeze(is, ax) = R22_emp_f / R22_th_f;
        sigma2_mean_freeze(is, ax) = sig2_mean_f;
    end
end

%% Also compute empirical IF_eff for baseline vs freeze
IF_eff_emp_base   = zeros(n_seed, 3);
IF_eff_emp_freeze = zeros(n_seed, 3);
for is = 1:n_seed
    for ax = 1:3
        % baseline
        dx_b = delta_x_m_base{is}(:, ax);
        dpmd_prev = 0; dpmr_b = zeros(N_t, 1);
        for k = 1:N_t
            dpmd = (1-a_pd)*dpmd_prev + a_pd*dx_b(k);
            dpmr_b(k) = dx_b(k) - dpmd;
            dpmd_prev = dpmd;
        end
        y = dpmr_b(warmup_idx);
        N_y = length(y);
        rho_hat = zeros(tau_max+1, 1);
        for tau = 0:tau_max
            rho_hat(tau+1) = mean(y(1:N_y-tau) .* y(1+tau:N_y));
        end
        rho_hat = rho_hat / rho_hat(1);
        IF_eff_emp_base(is, ax) = 1 + 2*sum(rho_hat(2:end).^2 .* s_ewma.^(1:tau_max)');

        % freeze
        dx_f = delta_x_m_freeze{is}(:, ax);
        dpmd_prev = 0; dpmr_f = zeros(N_t, 1);
        for k = 1:N_t
            dpmd = (1-a_pd)*dpmd_prev + a_pd*dx_f(k);
            dpmr_f(k) = dx_f(k) - dpmd;
            dpmd_prev = dpmd;
        end
        y = dpmr_f(warmup_idx);
        N_y = length(y);
        rho_hat = zeros(tau_max+1, 1);
        for tau = 0:tau_max
            rho_hat(tau+1) = mean(y(1:N_y-tau) .* y(1+tau:N_y));
        end
        rho_hat = rho_hat / rho_hat(1);
        IF_eff_emp_freeze(is, ax) = 1 + 2*sum(rho_hat(2:end).^2 .* s_ewma.^(1:tau_max)');
    end
end

%% Print results
fprintf('\n========== R22 ratio: baseline vs freeze ==========\n');
fprintf(' axis | baseline (5 seeds)   | freeze (5 seeds)     | change\n');
fprintf('------|----------------------|----------------------|--------\n');
for ax = 1:3
    rb = ratios_base(:, ax);
    rf = ratios_freeze(:, ax);
    fprintf('  %s   | %.4f ± %.4f       | %.4f ± %.4f       | %+.4f\n', ...
            ax_names{ax}, mean(rb), std(rb), mean(rf), std(rf), ...
            mean(rf) - mean(rb));
end

fprintf('\n========== IF_eff: closed-form vs empirical (base/freeze) ==========\n');
fprintf(' axis | IF_eff_cf | IF_eff_emp base    | IF_eff_emp freeze | freeze-base\n');
fprintf('------|-----------|--------------------|--------------------|-----------\n');
for ax = 1:3
    cf = IF_eff_cf(ax);
    eb = mean(IF_eff_emp_base(:, ax));
    ef = mean(IF_eff_emp_freeze(:, ax));
    fprintf('  %s   |  %.4f   | %.4f ± %.4f     | %.4f ± %.4f    | %+.4f\n', ...
            ax_names{ax}, cf, eb, std(IF_eff_emp_base(:, ax)), ...
            ef, std(IF_eff_emp_freeze(:, ax)), ef - eb);
end

%% Interpretation
fprintf('\n========== Interpretation ==========\n');
delta_z = mean(ratios_freeze(:,3)) - mean(ratios_base(:,3));
fprintf('  Z axis R22 ratio change (freeze − baseline) = %+.4f\n', delta_z);
if delta_z > 0.03
    fprintf('  → EKF feedback IS a significant cause (≥3%% of residual)\n');
elseif delta_z > 0.005
    fprintf('  → EKF feedback contributes a small amount (~1-3%% of residual)\n');
else
    fprintf('  → EKF feedback is NOT the main residual source (<1%% effect)\n');
end

%% Plot
set(groot, 'defaultTextInterpreter','tex');
set(groot, 'defaultLegendInterpreter','tex');
set(groot, 'defaultAxesTickLabelInterpreter','tex');

COL_BASE   = [0.85 0.00 0.00];
COL_FREEZE = [0.00 0.55 0.00];

fig = figure('Position', [50 50 1500 700], 'Color', 'w');
tl  = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% R22 ratio
nexttile;
hold on;
for ax = 1:3
    rb = ratios_base(:, ax);
    rf = ratios_freeze(:, ax);
    errorbar(ax-0.15, mean(rb), std(rb), 'o', 'Color', COL_BASE, ...
             'MarkerFaceColor', COL_BASE, 'MarkerSize', 12, 'LineWidth', 2);
    errorbar(ax+0.15, mean(rf), std(rf), 's', 'Color', COL_FREEZE, ...
             'MarkerFaceColor', COL_FREEZE, 'MarkerSize', 12, 'LineWidth', 2);
end
yline(1.0, 'k--', 'LineWidth', 2);
yline(1.05, 'k:', 'LineWidth', 1); yline(0.95, 'k:', 'LineWidth', 1);
plot(NaN, NaN, 'o', 'Color', COL_BASE, 'MarkerFaceColor', COL_BASE, ...
     'MarkerSize', 10, 'DisplayName', 'baseline (a\_hat est)');
plot(NaN, NaN, 's', 'Color', COL_FREEZE, 'MarkerFaceColor', COL_FREEZE, ...
     'MarkerSize', 10, 'DisplayName', 'freeze (a\_hat=a\_true)');
set(gca, 'XTick', 1:3, 'XTickLabel', ax_names, 'FontSize', 16, ...
    'FontWeight', 'bold', 'LineWidth', 2);
ylabel('R_{22} ratio', 'FontSize', 18);
title('(1) R_{22} ratio: baseline vs a\_hat freeze (static h=20)', 'FontSize', 16);
legend('show', 'FontSize', 13, 'Location', 'best');
grid on; box on; ylim([0.7 1.15]);

% IF_eff comparison
nexttile;
hold on;
for ax = 1:3
    plot(ax-0.2, IF_eff_cf(ax), 'k_', 'MarkerSize', 30, 'LineWidth', 3, ...
         'HandleVisibility', 'off');
    errorbar(ax, mean(IF_eff_emp_base(:,ax)), std(IF_eff_emp_base(:,ax)), 'o', ...
             'Color', COL_BASE, 'MarkerFaceColor', COL_BASE, 'MarkerSize', 12, ...
             'LineWidth', 2);
    errorbar(ax+0.2, mean(IF_eff_emp_freeze(:,ax)), std(IF_eff_emp_freeze(:,ax)), 's', ...
             'Color', COL_FREEZE, 'MarkerFaceColor', COL_FREEZE, 'MarkerSize', 12, ...
             'LineWidth', 2);
end
plot(NaN, NaN, 'k_', 'MarkerSize', 20, 'LineWidth', 3, 'DisplayName', 'closed-form');
plot(NaN, NaN, 'o', 'Color', COL_BASE, 'MarkerFaceColor', COL_BASE, ...
     'MarkerSize', 10, 'DisplayName', 'emp baseline');
plot(NaN, NaN, 's', 'Color', COL_FREEZE, 'MarkerFaceColor', COL_FREEZE, ...
     'MarkerSize', 10, 'DisplayName', 'emp freeze');
set(gca, 'XTick', 1:3, 'XTickLabel', ax_names, 'FontSize', 16, ...
    'FontWeight', 'bold', 'LineWidth', 2);
ylabel('IF_{eff}', 'FontSize', 18);
title('(2) IF_{eff}: closed-form vs empirical', 'FontSize', 16);
legend('show', 'FontSize', 13, 'Location', 'best');
grid on; box on;

sgtitle('Isolation test: a\_hat freeze (lock a\_hat = a\_true)', ...
        'FontSize', 17, 'FontWeight', 'bold');

out_path = fullfile(save_dir, 'isolation_ahat_freeze.png');
exportgraphics(fig, out_path, 'Resolution', 150);
fprintf('\nSaved: %s\n', out_path);

set(groot, 'defaultTextInterpreter','remove');
set(groot, 'defaultLegendInterpreter','remove');
set(groot, 'defaultAxesTickLabelInterpreter','remove');
