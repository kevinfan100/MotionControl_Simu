% derive_rho_dpmr_step2.m
% Phase 2: Combine R_T(τ) and R_N(τ) with thermal/noise weights to get
% ρ_δp_mr(τ), then compute IF_eff. Compare to empirical ρ̂(τ) and IF_eff.
%
% Operating point: ramp 50→5 um, ramp-average a for Z axis.
%
% ρ_δp_mr(τ) = w_T · ρ_T(τ) + w_n · ρ_N(τ)
%   w_T = C_dpmr · 4kBT·a / σ²_δp_mr
%   w_n = C_n · σ²_n / σ²_δp_mr
%   σ²_δp_mr = C_dpmr·4kBT·a + C_n·σ²_n
%
% IF_eff = 1 + 2·Σ ρ_δp_mr²(τ)·s^τ      (s = 1 - a_cov)

clear; close all; clc;
[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(fileparts(script_dir));
cd(project_root);
addpath(fullfile(project_root, 'model'));
addpath(fullfile(project_root, 'model', 'config'));
addpath(fullfile(project_root, 'model', 'wall_effect'));

%% Setup
lambda_c = 0.7;
a_pd     = 0.05;
a_cov    = 0.05;
alpha    = 1 - a_pd;
beta     = lambda_c;
c        = 1 - beta;
s_ewma   = 1 - a_cov;
tau_max  = 50;

constants  = physical_constants();
kBT        = constants.k_B * constants.T;
R_radius   = constants.R;
gamma_N    = constants.gamma_N;
Ts         = 1/1600;
a_nom      = Ts/gamma_N;

% Per-axis sensor noise (production)
sigma_n_axes  = [0.00062; 0.000057; 0.00331];
sigma2_n_axes = sigma_n_axes.^2;

%% Compute R_T(τ) and R_N(τ) closed-form (re-using derive_rho_dpmr_step1.m logic)
R_T = zeros(tau_max+2, 1);   % allow tau=-1 access via shift if needed
R_N = zeros(tau_max+1, 1);

for tau = 0:tau_max+1
    T1 = alpha^(tau+2) / (1 - alpha^2);
    if tau <= 3
        T2 = alpha^(5-tau) * c / ((1-alpha^2)*(1-alpha*beta));
    else
        T2 = (alpha^2 * c)/(alpha-beta) * ...
             (alpha^(tau-2)/(1-alpha^2) - beta^(tau-2)/(1-alpha*beta));
    end
    T3 = alpha^(tau+5) * c / ((1-alpha^2)*(1-alpha*beta));
    R_g = 1/(alpha-beta)^2 * ( ...
            alpha^(tau+2)/(1-alpha^2) + beta^(tau+2)/(1-beta^2) ...
          - alpha*beta*(alpha^tau + beta^tau)/(1-alpha*beta));
    T4 = alpha^2 * c^2 * R_g;
    R_T(tau+1) = T1 - T2 - T3 + T4;
end

for tau = 0:tau_max
    if tau == 0
        R_N(1) = 2*R_T(1) - 2*R_T(2);
    else
        R_N(tau+1) = 2*R_T(tau+1) - R_T(tau) - R_T(tau+2);
    end
end

R_T = R_T(1:tau_max+1);
C_dpmr = R_T(1);
C_n    = R_N(1);

rho_T = R_T / C_dpmr;
rho_N = R_N / C_n;

fprintf('=== Closed-form R_T(0)=C_dpmr=%.4f, R_N(0)=C_n=%.4f ===\n', C_dpmr, C_n);

%% Compute ramp-average weights (Z axis, h ramping 50→5)
% Use the same ramp-average a as in the empirical dataset
T_sim = 20;
N_t = 32001;
tout = linspace(0, T_sim, N_t)';
h_init = 50; h_bottom = 5;
rate = (h_init - h_bottom)/T_sim;
h_t = max(h_init - rate*tout, h_bottom);
h_bar_t = h_t / R_radius;

a_z_t = zeros(N_t, 1);
for k = 1:N_t
    [~, c_perp] = calc_correction_functions(max(h_bar_t(k), 1.001));
    a_z_t(k) = a_nom / c_perp;
end

% Per-axis weights at ramp-averaged a
fprintf('\n=== Ramp-averaged weights per axis ===\n');
fprintf(' axis | a_mean      | thermal var      | noise var      | w_T     | w_n\n');
fprintf('------|-------------|------------------|----------------|---------|--------\n');
ax_names = {'X', 'Y', 'Z'};
a_para_t = zeros(N_t, 1);
for k = 1:N_t
    [c_para, ~] = calc_correction_functions(max(h_bar_t(k), 1.001));
    a_para_t(k) = a_nom / c_para;
end
a_per_axis = {a_para_t, a_para_t, a_z_t};
w_T_axes = zeros(3,1); w_n_axes = zeros(3,1);
for ax = 1:3
    a_ax = mean(a_per_axis{ax});
    sig_T = C_dpmr * 4*kBT * a_ax;
    sig_n = C_n * sigma2_n_axes(ax);
    sig_tot = sig_T + sig_n;
    w_T_axes(ax) = sig_T / sig_tot;
    w_n_axes(ax) = sig_n / sig_tot;
    fprintf('  %s   | %.4e  |   %.4e    |  %.4e    | %.4f  | %.4f\n', ...
            ax_names{ax}, a_ax, sig_T, sig_n, w_T_axes(ax), w_n_axes(ax));
end

%% Compute ρ_δp_mr(τ) per axis using closed-form
rho_mix = zeros(tau_max+1, 3);   % [tau, axis]
for ax = 1:3
    rho_mix(:, ax) = w_T_axes(ax)*rho_T + w_n_axes(ax)*rho_N;
end

%% Compute IF_eff per axis (closed-form)
% IF_eff = 1 + 2·Σ_{τ=1..tau_max} ρ²(τ) · s^τ
IF_eff_closed = zeros(3, 1);
for ax = 1:3
    rho_ax = rho_mix(:, ax);
    IF_eff_closed(ax) = 1 + 2*sum(rho_ax(2:end).^2 .* s_ewma.^(1:tau_max)');
end

% Also compute IF_eff for thermal-only and noise-only (sanity)
IF_eff_T_only = 1 + 2*sum(rho_T(2:end).^2 .* s_ewma.^(1:tau_max)');
IF_eff_N_only = 1 + 2*sum(rho_N(2:end).^2 .* s_ewma.^(1:tau_max)');

% Current production IF_eff (Option A)
denom_e = 1 + 2*(1-lambda_c)^2;
rho_e_1 = (1-lambda_c)*(2-lambda_c)/denom_e;
rho_e_2 = (1-lambda_c)/denom_e;
Var_dx_over_sig_e = (1 + 2*lambda_c*rho_e_1 + 2*lambda_c^2*rho_e_2)/(1-lambda_c^2);
inv_var_ratio = 1/Var_dx_over_sig_e;
rho_dx_1 = lambda_c + inv_var_ratio*(rho_e_1 + lambda_c*rho_e_2);
rho_dx_2 = lambda_c*rho_dx_1 + inv_var_ratio*rho_e_2;
IF_eff_OptA = 1 + 2*(rho_dx_1^2*s_ewma + rho_dx_2^2*s_ewma^2/(1-lambda_c^2*s_ewma));

fprintf('\n=== IF_eff comparison ===\n');
fprintf('  IF_eff_OptA (production code)       = %.4f\n', IF_eff_OptA);
fprintf('  IF_eff thermal-only (pure ρ_T)      = %.4f\n', IF_eff_T_only);
fprintf('  IF_eff noise-only (pure ρ_N)        = %.4f\n', IF_eff_N_only);
for ax = 1:3
    fprintf('  IF_eff_closed (%s axis, ramp-avg)    = %.4f\n', ...
            ax_names{ax}, IF_eff_closed(ax));
end
fprintf('  → from prior empirical IF_eff_emp Z = 3.22 (target)\n');

%% Compare to empirical ρ̂(τ) from cache
fprintf('\n=== ρ_δp_mr(τ) closed-form vs empirical (Z axis) ===\n');
fprintf(' τ  | closed (Z) | empirical (Z, mean across 5 seeds) | diff\n');
fprintf('----|------------|------------------------------------|-----\n');

% Recompute empirical ρ̂ from baseline_sims_5seeds.mat
cache_path = fullfile(project_root, 'test_results', 'learn_variance', ...
                     'baseline_sims_5seeds.mat');
S = load(cache_path);
N_emp = length(S.tout_ref);
warmup_idx = S.tout_ref >= 2;
rho_hat = zeros(tau_max+1, 5, 3);
for is = 1:5
    delta_x_m = S.delta_x_m_all{is};
    for ax = 1:3
        dx = delta_x_m(:, ax);
        dpmd_prev = 0; dpmr = zeros(N_emp, 1);
        for k = 1:N_emp
            dpmd = (1-a_pd)*dpmd_prev + a_pd*dx(k);
            dpmr(k) = dx(k) - dpmd;
            dpmd_prev = dpmd;
        end
        y = dpmr(warmup_idx);
        N_y = length(y);
        for tau = 0:tau_max
            R_tau = mean(y(1:N_y-tau) .* y(1+tau:N_y));
            rho_hat(tau+1, is, ax) = R_tau;
        end
        rho_hat(:, is, ax) = rho_hat(:, is, ax) / rho_hat(1, is, ax);
    end
end
rho_hat_mean = squeeze(mean(rho_hat, 2));   % [tau x 3]

for tau = 0:min(tau_max, 10)
    fprintf(' %2d | %+.4f    |  %+.4f                            | %+.4f\n', ...
            tau, rho_mix(tau+1, 3), rho_hat_mean(tau+1, 3), ...
            rho_mix(tau+1, 3) - rho_hat_mean(tau+1, 3));
end

%% R22 ratio recomputation with new IF_eff
mat_path = fullfile(project_root, 'test_results', 'learn_variance', 'compare_am.mat');
M = load(mat_path);
sigma2_ctrl_z = M.sigma2_ctrl_z;
tout_ramp     = M.tout;
T_sim_ramp    = tout_ramp(end);
a_z_true      = M.a_z_true;
sigma2_n_z    = M.sigma2_n_s(3);
C_dpmr_v      = M.C_dpmr;
C_n_v         = M.C_n;

R22_prefactor = 2*a_cov/(2-a_cov);
sigma2_dxr_theory_t = C_dpmr_v*4*kBT*a_z_true + C_n_v*sigma2_n_z;

n_seg = 10;
seg_edges = linspace(0, T_sim_ramp, n_seg+1);
ratio_OptA = zeros(n_seg, 1);
ratio_new  = zeros(n_seg, 1);
for sg = 1:n_seg
    if sg < n_seg
        idx = (tout_ramp >= seg_edges(sg)) & (tout_ramp < seg_edges(sg+1));
    else
        idx = (tout_ramp >= seg_edges(sg)) & (tout_ramp <= seg_edges(sg+1));
    end
    t_seg = tout_ramp(idx);
    sig_seg = sigma2_ctrl_z(idx);
    p_sig = polyfit(t_seg, sig_seg, 1);
    sig_residual = sig_seg - polyval(p_sig, t_seg);
    R22_emp = var(sig_residual);
    sigma2_mid = mean(sigma2_dxr_theory_t(idx));
    R22_th_OptA = R22_prefactor * IF_eff_OptA * sigma2_mid^2;
    R22_th_new  = R22_prefactor * IF_eff_closed(3) * sigma2_mid^2;
    ratio_OptA(sg) = R22_emp / R22_th_OptA;
    ratio_new(sg)  = R22_emp / R22_th_new;
end
fprintf('\n=== R22 ratio comparison ===\n');
fprintf(' seg | ratio (IF_eff_OptA=%.2f) | ratio (IF_eff_new=%.2f)\n', ...
        IF_eff_OptA, IF_eff_closed(3));
fprintf('-----|-------------------------|-------------------------\n');
for sg = 1:n_seg
    fprintf(' %2d  |        %.4f           |        %.4f\n', sg, ratio_OptA(sg), ratio_new(sg));
end
fprintf('mean |        %.4f           |        %.4f\n', ...
        mean(ratio_OptA), mean(ratio_new));

%% Plot
set(groot, 'defaultTextInterpreter','tex');
set(groot, 'defaultLegendInterpreter','tex');
set(groot, 'defaultAxesTickLabelInterpreter','tex');

COL_AX = [0.85 0.20 0.20; 0.20 0.65 0.30; 0.20 0.30 0.85];
COL_TH = [0 0 0];

fig = figure('Position', [50 50 1500 800], 'Color', 'w');
tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% ρ(τ) closed-form vs empirical
nexttile;
tau_axis = 0:tau_max;
% empirical (3 axes)
for ax = 1:3
    plot(tau_axis, rho_hat_mean(:, ax), 'o-', 'LineWidth', 1.5, ...
         'Color', COL_AX(ax,:), 'MarkerSize', 5, ...
         'DisplayName', sprintf('%s emp', ax_names{ax})); hold on;
end
% closed-form (3 axes)
for ax = 1:3
    plot(tau_axis, rho_mix(:, ax), 'x--', 'LineWidth', 2, ...
         'Color', COL_AX(ax,:), 'MarkerSize', 8, ...
         'DisplayName', sprintf('%s closed', ax_names{ax}));
end
yline(0, 'k:', 'LineWidth', 1);
set(gca, 'FontSize', 15, 'FontWeight', 'bold', 'LineWidth', 2);
xlabel('lag \tau', 'FontSize', 17);
ylabel('\rho_{\delta p_{mr}}(\tau)', 'FontSize', 17);
title('(1) ρ(τ): closed-form vs empirical', 'FontSize', 16);
legend('show', 'FontSize', 11, 'Location', 'northeast', 'NumColumns', 2);
grid on; box on;
xlim([0 tau_max]);

% R22 ratio improvement
nexttile;
plot(1:n_seg, ratio_OptA, 'o-', 'Color', [0.8 0 0], 'LineWidth', 2, ...
     'MarkerFaceColor', [0.8 0 0], 'MarkerSize', 9, ...
     'DisplayName', sprintf('OptA (%.2f)', IF_eff_OptA)); hold on;
plot(1:n_seg, ratio_new, 's-', 'Color', [0 0.55 0], 'LineWidth', 2, ...
     'MarkerFaceColor', [0 0.55 0], 'MarkerSize', 9, ...
     'DisplayName', sprintf('new closed-form (%.2f)', IF_eff_closed(3)));
yline(1.0, 'k--', 'LineWidth', 2);
yline(1.05, 'k:', 'LineWidth', 1);
yline(0.95, 'k:', 'LineWidth', 1);
set(gca, 'FontSize', 15, 'FontWeight', 'bold', 'LineWidth', 2);
xlabel('segment', 'FontSize', 17);
ylabel('R_{22}^{emp} / R_{22}^{theory}', 'FontSize', 17);
title(sprintf('(2) R_{22} ratio: %.3f → %.3f', mean(ratio_OptA), mean(ratio_new)), ...
      'FontSize', 16);
legend('show', 'FontSize', 12, 'Location', 'best');
grid on; box on;
ylim([0.4 1.4]);

sgtitle('Phase 2 verification: ρ_{δp_{mr}} from closed-form + R22 ratio', ...
        'FontSize', 17, 'FontWeight', 'bold');

save_dir = fullfile(project_root, 'test_results', 'learn_variance');
out_path = fullfile(save_dir, 'derive_rho_dpmr_step2.png');
exportgraphics(fig, out_path, 'Resolution', 150);
fprintf('\nSaved: %s\n', out_path);

set(groot, 'defaultTextInterpreter','remove');
set(groot, 'defaultLegendInterpreter','remove');
set(groot, 'defaultAxesTickLabelInterpreter','remove');
