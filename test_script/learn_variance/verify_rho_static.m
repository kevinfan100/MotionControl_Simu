% verify_rho_static.m
% Phase 2.5: Verify closed-form ρ_δp_mr against empirical at STATIC positioning.
%
% Motivation: previous empirical was from ramp 50→5 where a varies, so ρ̂ has
% non-stationary bias. Run static h=const sims to get clean ρ̂.
%
% Plan:
%   - 5 seeds × positioning h=20 um × T=20s × thermal+noise ON
%   - Process δp_m through sandbox IIR (a_pd=0.05)
%   - Compute ρ̂(τ) per axis, average across seeds
%   - Compare to closed-form prediction at h=20

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
seed_list      = [42, 100, 200, 300, 400];
n_seed         = length(seed_list);
h_static       = 20;                  % positioning at h=20 um
T_sim          = 20;
sigma_n_axes   = [0.00062; 0.000057; 0.00331];
sigma2_n_axes  = sigma_n_axes.^2;
axis_names     = {'X', 'Y', 'Z'};
lambda_c       = 0.7;
a_pd           = 0.05;
a_cov          = 0.05;
alpha          = 1 - a_pd;
beta           = lambda_c;
c              = 1 - beta;
s_ewma         = 1 - a_cov;
tau_max        = 30;

constants  = physical_constants();
kBT        = constants.k_B * constants.T;
R_radius   = constants.R;
gamma_N    = constants.gamma_N;
Ts         = 1/1600;
a_nom      = Ts/gamma_N;

save_dir   = fullfile(project_root, 'test_results', 'learn_variance');
cache_path = fullfile(save_dir, 'baseline_static_h20_5seeds.mat');

%% PHASE 1: Run static positioning sims (cached)
if ~exist(cache_path, 'file')
    fprintf('=== Run %d static positioning sims (h=%dum) ===\n', n_seed, h_static);
    delta_x_m_all = cell(n_seed, 1);
    sigma2_ctrl_all = cell(n_seed, 1);
    tout_ref = [];
    for is = 1:n_seed
        seed = seed_list(is);
        fprintf('  seed=%d ... ', seed);
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
        opts = struct('seed', seed, 'verbose', false, 'collect_diag', true);
        clear motion_control_law motion_control_law_eq17_core ...
              motion_control_law_7state trajectory_generator calc_thermal_force;
        ts = tic;
        simOut = run_pure_simulation(config, opts);
        fprintf('%.1fs\n', toc(ts));
        delta_x_m_all{is}   = simOut.diag.delta_x_m;
        sigma2_ctrl_all{is} = simOut.diag.sigma2_dxr_hat;
        if isempty(tout_ref)
            tout_ref = simOut.tout;
        end
    end
    save(cache_path, 'delta_x_m_all', 'sigma2_ctrl_all', 'tout_ref', ...
         'h_static', 'sigma_n_axes', 'sigma2_n_axes', 'seed_list');
    fprintf('Cached → %s\n', cache_path);
else
    fprintf('Loading cache → %s\n', cache_path);
    load(cache_path);
end

N_t = length(tout_ref);

%% PHASE 2: Compute ρ̂(τ) per axis, pool over seeds, drop warmup
warmup_idx = tout_ref >= 2;
rho_hat_static = zeros(tau_max+1, n_seed, 3);
for is = 1:n_seed
    delta_x_m = delta_x_m_all{is};
    for ax = 1:3
        dx = delta_x_m(:, ax);
        dpmd_prev = 0; dpmr = zeros(N_t, 1);
        for k = 1:N_t
            dpmd = (1-a_pd)*dpmd_prev + a_pd*dx(k);
            dpmr(k) = dx(k) - dpmd;
            dpmd_prev = dpmd;
        end
        y = dpmr(warmup_idx);
        N_y = length(y);
        for tau = 0:tau_max
            R_tau = mean(y(1:N_y-tau) .* y(1+tau:N_y));
            rho_hat_static(tau+1, is, ax) = R_tau;
        end
        rho_hat_static(:, is, ax) = rho_hat_static(:, is, ax) / rho_hat_static(1, is, ax);
    end
end
rho_static_mean = squeeze(mean(rho_hat_static, 2));   % [tau x 3]
rho_static_std  = squeeze(std(rho_hat_static, [], 2));

%% PHASE 3: Closed-form ρ at static h=20
h_bar_static = h_static / R_radius;
[c_para_st, c_perp_st] = calc_correction_functions(max(h_bar_static, 1.001));
a_axes_static = [a_nom/c_para_st; a_nom/c_para_st; a_nom/c_perp_st];

% Compute R_T(τ), R_N(τ), ρ_T, ρ_N (only depend on α, β, c — independent of axis)
R_T_cf = zeros(tau_max+2, 1);
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
    R_T_cf(tau+1) = T1 - T2 - T3 + T4;
end
R_N_cf = zeros(tau_max+1, 1);
for tau = 0:tau_max
    if tau == 0
        R_N_cf(1) = 2*R_T_cf(1) - 2*R_T_cf(2);
    else
        R_N_cf(tau+1) = 2*R_T_cf(tau+1) - R_T_cf(tau) - R_T_cf(tau+2);
    end
end
R_T_cf = R_T_cf(1:tau_max+1);
C_dpmr_cf = R_T_cf(1);
C_n_cf    = R_N_cf(1);
rho_T_cf  = R_T_cf / C_dpmr_cf;
rho_N_cf  = R_N_cf / C_n_cf;

% Per-axis weights at static h=20
rho_cf = zeros(tau_max+1, 3);
IF_eff_cf = zeros(3, 1);
w_T_axes = zeros(3, 1); w_n_axes = zeros(3, 1);
for ax = 1:3
    sig_T = C_dpmr_cf * 4*kBT*a_axes_static(ax);
    sig_n = C_n_cf * sigma2_n_axes(ax);
    sig_tot = sig_T + sig_n;
    w_T_axes(ax) = sig_T/sig_tot;
    w_n_axes(ax) = sig_n/sig_tot;
    rho_cf(:, ax) = w_T_axes(ax)*rho_T_cf + w_n_axes(ax)*rho_N_cf;
    IF_eff_cf(ax) = 1 + 2*sum(rho_cf(2:end, ax).^2 .* s_ewma.^(1:tau_max)');
end

%% Empirical IF_eff_emp from static ρ̂
IF_eff_emp_static = zeros(3, 1);
for ax = 1:3
    IF_eff_emp_static(ax) = 1 + 2*sum(rho_static_mean(2:end, ax).^2 .* s_ewma.^(1:tau_max)');
end

%% Print
fprintf('\n=== STATIC h=%dum: ρ(τ) closed-form vs empirical ===\n', h_static);
fprintf(' τ  | rho_cf_Z  | rho_emp_Z (mean±std)  | diff   | rho_cf_X | rho_emp_X\n');
fprintf('----|-----------|------------------------|--------|----------|----------\n');
for tau = 0:min(tau_max, 12)
    fprintf(' %2d | %+.4f   | %+.4f ± %.4f         | %+.4f | %+.4f  | %+.4f\n', ...
            tau, rho_cf(tau+1, 3), rho_static_mean(tau+1, 3), rho_static_std(tau+1, 3), ...
            rho_cf(tau+1, 3) - rho_static_mean(tau+1, 3), ...
            rho_cf(tau+1, 1), rho_static_mean(tau+1, 1));
end

fprintf('\n=== IF_eff: closed-form vs empirical (STATIC) ===\n');
fprintf(' axis | closed-form | empirical (static) | diff%%\n');
fprintf('------|-------------|--------------------|---------\n');
for ax = 1:3
    fprintf('  %s   |   %.4f    |   %.4f             | %+.2f%%\n', ...
            axis_names{ax}, IF_eff_cf(ax), IF_eff_emp_static(ax), ...
            100*(IF_eff_cf(ax)/IF_eff_emp_static(ax) - 1));
end

% Production OptA reference
denom_e = 1 + 2*(1-lambda_c)^2;
rho_e_1 = (1-lambda_c)*(2-lambda_c)/denom_e;
rho_e_2 = (1-lambda_c)/denom_e;
Var_dx_over_sig_e = (1 + 2*lambda_c*rho_e_1 + 2*lambda_c^2*rho_e_2)/(1-lambda_c^2);
inv_var_ratio = 1/Var_dx_over_sig_e;
rho_dx_1_OA = lambda_c + inv_var_ratio*(rho_e_1 + lambda_c*rho_e_2);
rho_dx_2_OA = lambda_c*rho_dx_1_OA + inv_var_ratio*rho_e_2;
IF_eff_OptA = 1 + 2*(rho_dx_1_OA^2*s_ewma + rho_dx_2_OA^2*s_ewma^2/(1-lambda_c^2*s_ewma));
fprintf('\n(reference: production OptA IF_eff = %.4f, ramp empirical = 3.22)\n', IF_eff_OptA);

%% R22 verification at static h=20
fprintf('\n=== R22 ratio at static h=%dum ===\n', h_static);
R22_prefactor = 2*a_cov/(2-a_cov);
sig_per_seed = zeros(n_seed, 3);
R22_emp_per_seed = zeros(n_seed, 3);
for is = 1:n_seed
    sigma2_ctrl = sigma2_ctrl_all{is};
    for ax = 1:3
        y = sigma2_ctrl(warmup_idx, ax);
        % linear detrend for static? sigma2 should be stationary, but
        % use linear-detrended var to match other analyses
        t = tout_ref(warmup_idx);
        p = polyfit(t, y, 1);
        y_resid = y - polyval(p, t);
        R22_emp_per_seed(is, ax) = var(y_resid);
        sig_per_seed(is, ax) = mean(y);
    end
end
R22_emp_mean = mean(R22_emp_per_seed, 1);
sig_mean     = mean(sig_per_seed, 1);

fprintf(' axis | σ²̂ mean      | R22 emp       | R22 th(OptA)   | R22 th(new)   | ratio(OptA) | ratio(new)\n');
fprintf('------|--------------|---------------|----------------|---------------|-------------|------------\n');
for ax = 1:3
    R22_th_OptA = R22_prefactor * IF_eff_OptA * sig_mean(ax)^2;
    R22_th_new  = R22_prefactor * IF_eff_cf(ax) * sig_mean(ax)^2;
    fprintf('  %s   | %.4e   | %.4e    | %.4e     | %.4e    |   %.4f    |  %.4f\n', ...
            axis_names{ax}, sig_mean(ax), R22_emp_mean(ax), R22_th_OptA, R22_th_new, ...
            R22_emp_mean(ax)/R22_th_OptA, R22_emp_mean(ax)/R22_th_new);
end

%% Plot
set(groot, 'defaultTextInterpreter','tex');
set(groot, 'defaultLegendInterpreter','tex');
set(groot, 'defaultAxesTickLabelInterpreter','tex');

COL_AX = [0.85 0.20 0.20; 0.20 0.65 0.30; 0.20 0.30 0.85];

fig = figure('Position', [50 50 1500 800], 'Color', 'w');
tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% (1) ρ(τ) for static
nexttile;
tau_axis = 0:tau_max;
for ax = 1:3
    errorbar(tau_axis, rho_static_mean(:, ax), rho_static_std(:, ax), 'o-', ...
             'Color', COL_AX(ax,:), 'LineWidth', 1.5, 'MarkerSize', 5, ...
             'DisplayName', sprintf('%s emp (static)', axis_names{ax})); hold on;
end
for ax = 1:3
    plot(tau_axis, rho_cf(:, ax), 'x--', 'Color', COL_AX(ax,:), ...
         'LineWidth', 2, 'MarkerSize', 8, ...
         'DisplayName', sprintf('%s closed-form', axis_names{ax}));
end
yline(0, 'k:', 'LineWidth', 1);
set(gca, 'FontSize', 15, 'FontWeight', 'bold', 'LineWidth', 2);
xlabel('lag \tau', 'FontSize', 17);
ylabel('\rho_{\delta p_{mr}}(\tau)', 'FontSize', 17);
title(sprintf('(1) ρ(τ) static h=%dum: closed-form vs empirical', h_static), ...
      'FontSize', 16);
legend('show', 'FontSize', 11, 'Location', 'northeast', 'NumColumns', 2);
grid on; box on;

% (2) bar plot of IF_eff comparison
nexttile;
bar_data = [repmat(IF_eff_OptA, 3, 1), IF_eff_cf, IF_eff_emp_static];
b = bar(1:3, bar_data, 0.85);
b(1).FaceColor = [0.8 0.4 0.0];
b(2).FaceColor = [0.0 0.55 0.0];
b(3).FaceColor = [0.2 0.3 0.85];
set(gca, 'XTick', 1:3, 'XTickLabel', axis_names, 'FontSize', 15, ...
    'FontWeight', 'bold', 'LineWidth', 2);
ylabel('IF_{eff}', 'FontSize', 17);
title('(2) IF_{eff} comparison (static h=20)', 'FontSize', 16);
legend({'OptA (production)', 'closed-form (new)', 'empirical (static)'}, ...
       'FontSize', 12, 'Location', 'best');
grid on; box on;

sgtitle(sprintf('Static positioning ρ̂ verification (5 seeds, h=%dum)', h_static), ...
        'FontSize', 17, 'FontWeight', 'bold');

out_path = fullfile(save_dir, 'verify_rho_static.png');
exportgraphics(fig, out_path, 'Resolution', 150);
fprintf('\nSaved: %s\n', out_path);

set(groot, 'defaultTextInterpreter','remove');
set(groot, 'defaultLegendInterpreter','remove');
set(groot, 'defaultAxesTickLabelInterpreter','remove');
