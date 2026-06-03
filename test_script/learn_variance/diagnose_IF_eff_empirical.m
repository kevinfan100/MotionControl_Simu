% diagnose_IF_eff_empirical.m
% Empirical ρ_δpmr autocorrelation to diagnose the IF_eff formula.
%
% Background:
%   R22_theory = R22_prefactor · IF_eff · σ⁴_dxr
%   Current IF_eff (Option A_MA2_full, closed-form):
%     ρ_δx(1) = 0.852, ρ_δx(2) = 0.672, ρ(τ≥3) = λc·ρ(τ-1)
%     IF_eff = 1 + 2·[ρ²(1)·s + ρ²(2)·s²/(1-λc²·s)] = 3.90
%
% Observed: R22_emp / R22_theory ≈ 0.742, i.e. theory over by ~1.34×
% Hypothesis: ρ̂_δpmr from production sims decays faster than closed-form,
%             so IF_eff_emp < 3.90 → R22_theory comes down → ratio → 1.0
%
% Plan:
%   1. Load cached 5-seed baseline sims (ramp 50→5, thermal ON)
%   2. For each seed × axis: compute δp_mr via sandbox IIR (a_pd=0.05)
%   3. Estimate ρ̂(τ) for τ=0..30, pool across seeds
%   4. Compare ρ̂(τ) vs Option A closed-form ρ(τ)
%   5. Compute IF_eff_emp = 1 + 2·Σ ρ̂²(τ)·s^τ
%   6. Test: R22_theory_corrected = R22_prefactor · IF_eff_emp · σ⁴_dxr
%            Does R22_emp / R22_theory_corrected → 1.0?

clear; close all; clc;

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(fileparts(script_dir));
cd(project_root);
addpath(fullfile(project_root, 'model'));
addpath(fullfile(project_root, 'model', 'config'));
addpath(fullfile(project_root, 'model', 'wall_effect'));
addpath(fullfile(project_root, 'model', 'controller'));

%% Load baseline sims
cache_path = fullfile(project_root, 'test_results', 'learn_variance', ...
                     'baseline_sims_5seeds.mat');
if ~exist(cache_path, 'file')
    error(['baseline_sims_5seeds.mat not found. ' ...
           'Run verify_Cdpmr_offline.m first to populate cache.']);
end
S = load(cache_path);
delta_x_m_all   = S.delta_x_m_all;
sigma2_ctrl_all = S.sigma2_ctrl_baseline;
tout_ref        = S.tout_ref;
lambda_c        = S.lambda_c;
T_sim           = S.T_sim;
Ts              = S.Ts;
sigma2_n_axes   = S.sigma2_n_axes;
seed_list       = S.seed_list;
N               = length(tout_ref);
n_seed          = length(seed_list);
axis_names      = {'X', 'Y', 'Z'};

a_pd  = 0.05;
a_cov = 0.05;

%% Compute δp_mr (HP residual) for each seed × axis via sandbox IIR
fprintf('=== Compute δp_mr trace via sandbox IIR ===\n');
dpmr_all = cell(n_seed, 3);   % [seed, axis] → [N x 1] trace
for is = 1:n_seed
    delta_x_m = delta_x_m_all{is};
    for ax = 1:3
        dx = delta_x_m(:, ax);
        dpmd = zeros(N, 1);
        dpmr = zeros(N, 1);
        dpmd_prev = 0;
        for k = 1:N
            dpmd(k) = (1-a_pd)*dpmd_prev + a_pd*dx(k);
            dpmr(k) = dx(k) - dpmd(k);
            dpmd_prev = dpmd(k);
        end
        dpmr_all{is, ax} = dpmr;
    end
    fprintf('  seed=%d done\n', seed_list(is));
end

%% Estimate empirical ρ̂(τ) for τ=0..30
% Use entire ramp trace (variance is mildly time-varying but ρ should be stable).
% Drop first 2 sec for IIR settling.
tau_max = 30;
rho_hat = zeros(tau_max+1, n_seed, 3);
warmup_idx = tout_ref >= 2;

for is = 1:n_seed
    for ax = 1:3
        y = dpmr_all{is, ax}(warmup_idx);
        N_y = length(y);
        % Direct autocorrelation estimator: R̂(τ) = (1/N) Σ y[n]·y[n+τ]
        for tau = 0:tau_max
            valid = 1:(N_y - tau);
            R_tau = mean(y(valid) .* y(valid + tau));
            rho_hat(tau+1, is, ax) = R_tau;
        end
        % Normalize by R̂(0)
        rho_hat(:, is, ax) = rho_hat(:, is, ax) / rho_hat(1, is, ax);
    end
end

% Average across seeds (per axis), and across all (seeds × axes)
rho_hat_per_axis = mean(rho_hat, 2);  % [tau, 1, axis]
rho_hat_per_axis = squeeze(rho_hat_per_axis);  % [tau x 3]
rho_hat_pool = mean(rho_hat_per_axis, 2);  % [tau x 1], pooled across all 3 axes

% Std across seeds (per axis), for error bars
rho_hat_std_per_axis = std(rho_hat, [], 2);
rho_hat_std_per_axis = squeeze(rho_hat_std_per_axis);

%% Closed-form Option A predictions
denom_e = 1 + 2 * (1 - lambda_c)^2;
rho_e_1 = (1 - lambda_c) * (2 - lambda_c) / denom_e;
rho_e_2 = (1 - lambda_c) / denom_e;
Var_dx_over_sig_e = (1 + 2*lambda_c*rho_e_1 + 2*lambda_c^2*rho_e_2) / (1 - lambda_c^2);
inv_var_ratio = 1 / Var_dx_over_sig_e;
rho_OptA = zeros(tau_max+1, 1);
rho_OptA(1) = 1;  % ρ(0) = 1
rho_OptA(2) = lambda_c + inv_var_ratio * (rho_e_1 + lambda_c*rho_e_2);
rho_OptA(3) = lambda_c * rho_OptA(2) + inv_var_ratio * rho_e_2;
for tau = 3:tau_max
    rho_OptA(tau+1) = lambda_c * rho_OptA(tau);  % geometric tail
end

%% Print comparison
fprintf('\n=== ρ(τ) comparison (axis Z, pooled across 5 seeds) ===\n');
fprintf(' τ  | ρ̂(τ) Z empirical (mean±std) | ρ(τ) Option A | diff\n');
fprintf('----|------------------------------|---------------|--------\n');
for tau = 0:8
    fprintf(' %2d | %.4f ± %.4f             |  %.4f        | %+.4f\n', ...
            tau, rho_hat_per_axis(tau+1, 3), rho_hat_std_per_axis(tau+1, 3), ...
            rho_OptA(tau+1), ...
            rho_hat_per_axis(tau+1, 3) - rho_OptA(tau+1));
end

%% Compute IF_eff_emp from ρ̂ and IF_eff_closed-form
s_ewma = 1 - a_cov;
% IF_eff = 1 + 2·Σ_{τ≥1} ρ²(τ)·s^τ

% (a) Empirical IF_eff per axis
IF_eff_emp_per_axis = zeros(3, 1);
for ax = 1:3
    rho_ax = rho_hat_per_axis(:, ax);
    IF_eff_emp_per_axis(ax) = 1 + 2 * sum(rho_ax(2:end).^2 .* s_ewma.^(1:tau_max)');
end

% (b) Pooled empirical IF_eff
IF_eff_emp_pool = 1 + 2 * sum(rho_hat_pool(2:end).^2 .* s_ewma.^(1:tau_max)');

% (c) Closed-form Option A IF_eff (production code uses this)
IF_eff_closed = 1 + 2 * sum(rho_OptA(2:end).^2 .* s_ewma.^(1:tau_max)');
% sanity: closed-form analytic
rho_dx_1 = rho_OptA(2);
rho_dx_2 = rho_OptA(3);
IF_eff_closed_analytic = 1 + 2*(rho_dx_1^2 * s_ewma ...
                              + rho_dx_2^2 * s_ewma^2 / (1 - lambda_c^2 * s_ewma));

fprintf('\n=== IF_eff comparison ===\n');
fprintf('  IF_eff Option A (analytic closed-form) = %.4f  ← production code\n', ...
        IF_eff_closed_analytic);
fprintf('  IF_eff Option A (numerical sum)        = %.4f\n', IF_eff_closed);
fprintf('  IF_eff_emp axis X                       = %.4f\n', IF_eff_emp_per_axis(1));
fprintf('  IF_eff_emp axis Y                       = %.4f\n', IF_eff_emp_per_axis(2));
fprintf('  IF_eff_emp axis Z                       = %.4f\n', IF_eff_emp_per_axis(3));
fprintf('  IF_eff_emp pooled (X+Y+Z)               = %.4f\n', IF_eff_emp_pool);
fprintf('  Closed / Empirical (Z)                  = %.3f\n', ...
        IF_eff_closed_analytic / IF_eff_emp_per_axis(3));

%% Apply to R22 verification — does ratio approach 1.0 with IF_eff_emp?
fprintf('\n=== R22 ratio recomputation using IF_eff_emp ===\n');

% Recreate verify_R22 data and recompute with IF_eff_emp.
mat_path = fullfile(project_root, 'test_results', 'learn_variance', 'compare_am.mat');
M = load(mat_path);
sigma2_ctrl_z = M.sigma2_ctrl_z;
tout          = M.tout;
T_sim_v       = tout(end);
a_z_true      = M.a_z_true;
sigma2_n_z    = M.sigma2_n_s(3);
C_dpmr_v      = M.C_dpmr;
C_n_v         = M.C_n;
N_v           = length(tout);

R22_prefactor = 2*a_cov/(2-a_cov);
sigma2_dxr_theory_t = C_dpmr_v * 4*1.71284e-2*0.250 .* a_z_true + C_n_v * sigma2_n_z;  % placeholder
% recompute using exact kBT
constants = physical_constants();
kBT = constants.k_B * constants.T;
sigma2_dxr_theory_t = C_dpmr_v * 4 * kBT * a_z_true + C_n_v * sigma2_n_z;

n_seg = 10;
seg_edges = linspace(0, T_sim_v, n_seg+1);
ratio_old = zeros(n_seg, 1);
ratio_new = zeros(n_seg, 1);
for sg = 1:n_seg
    if sg < n_seg
        idx = (tout >= seg_edges(sg)) & (tout < seg_edges(sg+1));
    else
        idx = (tout >= seg_edges(sg)) & (tout <= seg_edges(sg+1));
    end
    t_seg = tout(idx);
    sig_seg = sigma2_ctrl_z(idx);
    p_sig = polyfit(t_seg, sig_seg, 1);
    sig_residual = sig_seg - polyval(p_sig, t_seg);
    R22_emp = var(sig_residual);

    sigma2_mid = mean(sigma2_dxr_theory_t(idx));
    R22_th_old = R22_prefactor * IF_eff_closed_analytic * sigma2_mid^2;
    R22_th_new = R22_prefactor * IF_eff_emp_per_axis(3) * sigma2_mid^2;

    ratio_old(sg) = R22_emp / R22_th_old;
    ratio_new(sg) = R22_emp / R22_th_new;
end
fprintf(' seg | ratio (IF_eff_closed) | ratio (IF_eff_emp_Z)\n');
fprintf('-----|----------------------|----------------------\n');
for sg = 1:n_seg
    fprintf(' %2d  |      %.4f          |      %.4f\n', sg, ratio_old(sg), ratio_new(sg));
end
fprintf('mean |      %.4f          |      %.4f\n', mean(ratio_old), mean(ratio_new));
fprintf('std  |      %.4f          |      %.4f\n', std(ratio_old), std(ratio_new));

%% Plot
set(groot, 'defaultTextInterpreter','tex');
set(groot, 'defaultLegendInterpreter','tex');
set(groot, 'defaultAxesTickLabelInterpreter','tex');

COL_X = [0.85 0.20 0.20];
COL_Y = [0.20 0.65 0.30];
COL_Z = [0.20 0.30 0.85];
COL_AX = [COL_X; COL_Y; COL_Z];
COL_TH = [0.0 0.0 0.0];

fig = figure('Position', [50 50 1700 1000], 'Color', 'w');
tl  = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% (1) ρ(τ) vs τ, all 3 axes + Option A
nexttile;
tau_axis = 0:tau_max;
for ax = 1:3
    errorbar(tau_axis, rho_hat_per_axis(:, ax), rho_hat_std_per_axis(:, ax), ...
             'o-', 'Color', COL_AX(ax, :), 'MarkerFaceColor', COL_AX(ax, :), ...
             'LineWidth', 1.8, 'MarkerSize', 6, ...
             'DisplayName', sprintf('axis %s emp', axis_names{ax})); hold on;
end
plot(tau_axis, rho_OptA, 's--', 'Color', COL_TH, 'LineWidth', 2.5, ...
     'MarkerFaceColor', COL_TH, 'MarkerSize', 8, ...
     'DisplayName', 'Option A closed-form');
set(gca, 'FontSize', 15, 'FontWeight', 'bold', 'LineWidth', 2);
xlabel('lag \tau', 'FontSize', 17);
ylabel('\rho_{\delta p_{mr}}(\tau)', 'FontSize', 17);
title('(1) Autocorrelation ρ(τ): empirical vs Option A', 'FontSize', 16);
legend('show', 'FontSize', 12, 'Location', 'northeast');
grid on; box on; xlim([0 tau_max]);

% (2) ρ²(τ)·s^τ contribution per τ (the IF_eff integrand)
nexttile;
contrib_emp = rho_hat_per_axis(:, 3).^2 .* s_ewma.^(0:tau_max)';
contrib_OptA = rho_OptA.^2 .* s_ewma.^(0:tau_max)';
plot(tau_axis(2:end), contrib_emp(2:end), 'o-', 'Color', COL_Z, ...
     'MarkerFaceColor', COL_Z, 'LineWidth', 2, 'MarkerSize', 8, ...
     'DisplayName', 'empirical (Z)'); hold on;
plot(tau_axis(2:end), contrib_OptA(2:end), 's--', 'Color', COL_TH, ...
     'LineWidth', 2, 'MarkerSize', 8, ...
     'DisplayName', 'Option A closed-form');
set(gca, 'FontSize', 15, 'FontWeight', 'bold', 'LineWidth', 2);
xlabel('lag \tau', 'FontSize', 17);
ylabel('\rho^2(\tau) \cdot s^\tau', 'FontSize', 17);
title('(2) IF_{eff} per-τ contribution', 'FontSize', 16);
legend('show', 'FontSize', 12, 'Location', 'northeast');
grid on; box on; xlim([1 tau_max]);

% (3) Cumulative IF_eff up to τ
nexttile;
IF_cum_emp = zeros(tau_max+1, 1);
IF_cum_OptA = zeros(tau_max+1, 1);
for tau = 0:tau_max
    IF_cum_emp(tau+1) = 1 + 2 * sum(rho_hat_per_axis(2:tau+1, 3).^2 .* s_ewma.^(1:tau)');
    IF_cum_OptA(tau+1) = 1 + 2 * sum(rho_OptA(2:tau+1).^2 .* s_ewma.^(1:tau)');
end
plot(tau_axis, IF_cum_emp, 'o-', 'Color', COL_Z, ...
     'MarkerFaceColor', COL_Z, 'LineWidth', 2, 'MarkerSize', 8, ...
     'DisplayName', 'IF_{eff,emp}(Z) cumulative'); hold on;
plot(tau_axis, IF_cum_OptA, 's--', 'Color', COL_TH, ...
     'LineWidth', 2, 'MarkerSize', 8, ...
     'DisplayName', 'IF_{eff,closed} cumulative');
yline(IF_eff_closed_analytic, 'k:', 'LineWidth', 1.5);
set(gca, 'FontSize', 15, 'FontWeight', 'bold', 'LineWidth', 2);
xlabel('upper lag \tau', 'FontSize', 17);
ylabel('cumulative IF_{eff}', 'FontSize', 17);
title('(3) Cumulative IF_{eff} convergence', 'FontSize', 16);
legend('show', 'FontSize', 12, 'Location', 'southeast');
grid on; box on; xlim([0 tau_max]);

% (4) R22 ratio per segment: old vs corrected
nexttile;
plot(1:n_seg, ratio_old, 'o-', 'Color', [0.85 0.0 0.0], 'LineWidth', 2, ...
     'MarkerFaceColor', [0.85 0.0 0.0], 'MarkerSize', 9, ...
     'DisplayName', sprintf('IF_{eff,closed}=%.2f', IF_eff_closed_analytic)); hold on;
plot(1:n_seg, ratio_new, 's-', 'Color', [0.0 0.55 0.0], 'LineWidth', 2, ...
     'MarkerFaceColor', [0.0 0.55 0.0], 'MarkerSize', 9, ...
     'DisplayName', sprintf('IF_{eff,emp}=%.2f', IF_eff_emp_per_axis(3)));
yline(1.0, 'k--', 'LineWidth', 2);
yline(1.1, 'k:', 'LineWidth', 1);
yline(0.9, 'k:', 'LineWidth', 1);
set(gca, 'FontSize', 15, 'FontWeight', 'bold', 'LineWidth', 2);
xlabel('segment', 'FontSize', 17);
ylabel('R_{22}^{emp} / R_{22}^{theory}', 'FontSize', 17);
title(sprintf('(4) R_{22} ratio: closed vs emp IF_{eff}  (mean %.3f → %.3f)', ...
              mean(ratio_old), mean(ratio_new)), 'FontSize', 15);
legend('show', 'FontSize', 12, 'Location', 'best');
grid on; box on; ylim([0.4 1.4]); xlim([0.5 n_seg+0.5]);

sgtitle('IF_{eff} empirical diagnosis (5 seeds × 3 axes, ramp 50-5)', ...
        'FontSize', 17, 'FontWeight', 'bold');

save_dir = fullfile(project_root, 'test_results', 'learn_variance');
out_path = fullfile(save_dir, 'diagnose_IF_eff_empirical.png');
exportgraphics(fig, out_path, 'Resolution', 150);
fprintf('\nSaved: %s\n', out_path);

set(groot, 'defaultTextInterpreter','remove');
set(groot, 'defaultLegendInterpreter','remove');
set(groot, 'defaultAxesTickLabelInterpreter','remove');
