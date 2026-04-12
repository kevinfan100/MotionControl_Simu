%% analyze_task1b_iir_bias.m
% Task 1b: Verify IIR finite-sample bias hypothesis (offline, no Simulink).
%
% Hypothesis: The ~8% a_m negative bias observed in Phase 2A is explained by
% the IIR EMA variance estimator's finite-sample bias amplified by del_pmr
% autocorrelation (inherited from closed-loop dynamics).
%
% Method:
%   Step 1 - Reapply IIR variance estimator to the Phase 2A del_pmr time series
%            with an a_prd sweep and measure mean(V_IIR)/Var_sample.
%   Step 2 - Compute the analytic autocorrelation rho(L) from A_aug + Sigma_aug
%            (via compute_7state_cdpmr_eff). Predict bias using the closed-form
%            E[V_IIR]/Var(x) = 1 - (a/(2-a)) * (1 + 2 * sum_{L>=1} rho(L) * (1-a)^L).
%   Step 3 - Compare theory vs empirical and decide next step.

clear; clc;

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(script_dir);
addpath(script_dir);
addpath(fullfile(project_root, 'model'));
addpath(fullfile(project_root, 'model', 'config'));

fprintf('===== Task 1b: IIR finite-sample bias analysis =====\n\n');

%% Load Phase 2A data
data_file = fullfile(project_root, 'test_results', 'verify', 'phase2_chisquared_mc.mat');
if ~isfile(data_file)
    error('Missing %s. Regenerate via phase2_chisquared_mc.m', data_file);
end
data = load(data_file);
res = data.results;

del_pmr  = res.del_pmr;             % 3 x N
ss_start = res.ss_start;
N        = size(del_pmr, 2);
ss       = ss_start:N;
Ts       = 1/1600;

lc                   = res.lc;
a_nom                = res.a_nom;
C_dpmr_eff_phase2    = res.C_dpmr_eff;

fprintf('Loaded del_pmr %dx%d, ss [%d..%d] (%.1f sec of steady state).\n', ...
        size(del_pmr,1), N, ss(1), ss(end), length(ss)*Ts);
fprintf('lc = %.2f, a_nom = %.4e, C_dpmr_eff (phase2) = %.4f\n\n', ...
        lc, a_nom, C_dpmr_eff_phase2);

%% Physical constants and V_theory
constants  = physical_constants();
k_B        = constants.k_B;
T_temp     = constants.T;
gamma_N    = constants.gamma_N;
sigma2_dXT = 4 * k_B * T_temp * Ts / gamma_N;
V_theory_z = C_dpmr_eff_phase2 * sigma2_dXT;
fprintf('sigma2_dXT = %.4e um^2\n', sigma2_dXT);
fprintf('V_theory(z) = C_dpmr_eff * sigma2_dXT = %.4e um^2\n\n', V_theory_z);

%% Sample ground truth per axis
axname         = {'x', 'y', 'z'};
V_sample_xyz   = zeros(1, 3);
mean_xyz       = zeros(1, 3);
for ax_i = 1:3
    x_ss = del_pmr(ax_i, ss);
    mean_xyz(ax_i)     = mean(x_ss);
    V_sample_xyz(ax_i) = mean((x_ss - mean_xyz(ax_i)).^2);
    fprintf('  %s-axis: mean=%+.3e  V_sample=%.4e  V_sample/V_theory=%.4f\n', ...
            axname{ax_i}, mean_xyz(ax_i), V_sample_xyz(ax_i), ...
            V_sample_xyz(ax_i)/V_theory_z);
end
fprintf('\n');

% Focus on z-axis (trajectory / wall-normal axis matches Phase 2 analysis)
del_pmr_z = del_pmr(3, :);
mean_z    = mean_xyz(3);
V_sample  = V_sample_xyz(3);

%% Sample autocorrelation rho_L^sample
L_max      = 100;
rho_sample = zeros(L_max+1, 1);
x_center   = del_pmr_z(ss) - mean_z;
for L = 0:L_max
    n_pairs = length(ss) - L;
    rho_sample(L+1) = sum(x_center(1:n_pairs) .* x_center(1+L:end)) / (n_pairs * V_sample);
end
fprintf('Sample rho(1..5)        = [%s]\n', sprintf('%.4f ', rho_sample(2:6)));
fprintf('Sample rho(10,20,50,100) = [%.4f %.4f %.4f %.4f]\n\n', ...
        rho_sample(11), rho_sample(21), rho_sample(51), rho_sample(101));

%% Step 1: Empirical IIR bias sweep
fprintf('--- Step 1: Empirical IIR bias sweep (z-axis) ---\n');
a_prd_list   = [0.005, 0.01, 0.02, 0.05, 0.1];
a_cov_fixed  = 0.05;

V_IIR_afix   = zeros(size(a_prd_list));   % a_cov = 0.05 fixed
V_IIR_match  = zeros(size(a_prd_list));   % a_cov = a_prd (controller default)

for i = 1:length(a_prd_list)
    a_prd_i = a_prd_list(i);
    V1 = compute_IIR_var_series(del_pmr_z, a_prd_i, a_cov_fixed);
    V2 = compute_IIR_var_series(del_pmr_z, a_prd_i, a_prd_i);
    V_IIR_afix(i)  = mean(V1(ss));
    V_IIR_match(i) = mean(V2(ss));
end

fprintf('\n%-8s %-14s %-14s %-14s %-14s\n', ...
        'a_prd', 'V_IIR(a_cov=5e-2)', 'V_IIR(matched)', 'V_IIR/V_sample', 'V_IIR/V_theory');
for i = 1:length(a_prd_list)
    e1 = V_IIR_afix(i) / V_sample;
    e2 = V_IIR_afix(i) / V_theory_z;
    fprintf('%-8.4f %-14.4e %-14.4e %-14.4f %-14.4f\n', ...
            a_prd_list(i), V_IIR_afix(i), V_IIR_match(i), e1, e2);
end

%% Step 2: Analytic autocorrelation from Sigma_aug
fprintf('\n--- Step 2: Analytic autocorrelation via compute_7state_cdpmr_eff ---\n');

uc         = user_config();
a_pd       = uc.a_pd;
Q_kf_scale = uc.Qz_diag_scaling;
R_kf_scale = uc.Rz_diag_scaling;

opts = struct('f0', 0, 'verbose', false);
[C_dpmr_eff_th, C_np_eff_th, L_ss, A_aug, diag_th] = ...
    compute_7state_cdpmr_eff(lc, 0, a_pd, Q_kf_scale, R_kf_scale, opts);
Sigma_aug = diag_th.Sigma_th;

fprintf('C_dpmr_eff (analytic) = %.4f  (phase2 stored: %.4f)\n', ...
        C_dpmr_eff_th, C_dpmr_eff_phase2);
fprintf('A_aug max|eig|        = %.6f\n', diag_th.max_eig_A_aug);

% Observable: del_pmr = (1-a_pd) * (x(idx_dx_d2) - x(idx_pmd_prev))
n_aug        = 11;
idx_dx_d2    = 3;
idx_pmd_prev = 11;
c_s          = zeros(n_aug, 1);
c_s(idx_dx_d2)    =  1;
c_s(idx_pmd_prev) = -1;

% gamma_theory(L) [physical units, thermal only at a=1 unit, free space]
%   gamma_th(L) = (1-a_pd)^2 * c_s' * A_aug^L * Sigma_aug * c_s * sigma2_dXT
% For rho_L the sigma2_dXT scale cancels.
gamma_th_uni = zeros(L_max+1, 1);
A_L = eye(n_aug);
for L = 0:L_max
    gamma_th_uni(L+1) = (1-a_pd)^2 * (c_s' * A_L * Sigma_aug * c_s);
    A_L = A_L * A_aug;
end
rho_th = gamma_th_uni / gamma_th_uni(1);

fprintf('\nTheory rho(1..5)        = [%s]\n', sprintf('%.4f ', rho_th(2:6)));
fprintf('Theory rho(10,20,50,100) = [%.4f %.4f %.4f %.4f]\n\n', ...
        rho_th(11), rho_th(21), rho_th(51), rho_th(101));

% Cross-check gamma_th(0) in physical units equals V_theory (Phase 1 consistency)
V_theory_via_Sigma = gamma_th_uni(1) * sigma2_dXT;
fprintf('V_theory from Sigma_aug : %.4e  (V_theory from C_dpmr_eff: %.4e)\n', ...
        V_theory_via_Sigma, V_theory_z);

%% Predicted theoretical bias (closed-form EMA on autocorrelated process)
predicted_bias_white     = 1 - a_prd_list ./ (2 - a_prd_list);
predicted_bias_rho_th    = zeros(size(a_prd_list));
predicted_bias_rho_smp   = zeros(size(a_prd_list));

for i = 1:length(a_prd_list)
    a = a_prd_list(i);

    powers = ((1-a).^(1:L_max)).';    % column
    S_th   = sum(rho_th(2:L_max+1) .* powers);
    S_smp  = sum(rho_sample(2:L_max+1) .* powers);

    predicted_bias_rho_th(i)  = 1 - (a/(2-a)) * (1 + 2*S_th);
    predicted_bias_rho_smp(i) = 1 - (a/(2-a)) * (1 + 2*S_smp);
end

%% Step 3: Compare theory vs empirical
fprintf('\n--- Step 3: Theory vs Empirical (bias = E[V_IIR]/V_sample) ---\n');
fprintf('%-8s %-12s %-12s %-14s %-14s\n', ...
        'a_prd', 'empirical', 'thy(white)', 'thy(rho_smp)', 'thy(rho_theory)');
for i = 1:length(a_prd_list)
    e = V_IIR_afix(i) / V_sample;
    fprintf('%-8.4f %-12.4f %-12.4f %-14.4f %-14.4f\n', ...
            a_prd_list(i), e, predicted_bias_white(i), ...
            predicted_bias_rho_smp(i), predicted_bias_rho_th(i));
end

% Phase 2A observed ratio (a_m_z / a_nom = V_IIR / V_theory)
i05 = find(a_prd_list == 0.05, 1);
phase2_bias_pct_obs = (1 - 0.925) * 100;
theory_bias_pct_at5 = (1 - predicted_bias_rho_th(i05)) * 100;
empir_bias_pct_at5  = (1 - V_IIR_afix(i05)/V_sample) * 100;
empir_over_theory_at5 = (1 - V_IIR_afix(i05)/V_theory_z) * 100;

explained_pct_theory = theory_bias_pct_at5 / phase2_bias_pct_obs * 100;
explained_pct_offline = empir_over_theory_at5 / phase2_bias_pct_obs * 100;

fprintf('\n--- Decision ---\n');
fprintf('Phase 2A observed a_m_z bias                 : %5.2f%% low\n', phase2_bias_pct_obs);
fprintf('Phase 2A Var_sample / V_theory               : %6.4f\n', V_sample/V_theory_z);
fprintf('Offline empirical V_IIR/V_sample (a_prd=0.05) : %6.4f (%.2f%% low)\n', ...
        V_IIR_afix(i05)/V_sample, empir_bias_pct_at5);
fprintf('Offline empirical V_IIR/V_theory (a_prd=0.05) : %6.4f (%.2f%% low)\n', ...
        V_IIR_afix(i05)/V_theory_z, empir_over_theory_at5);
fprintf('Analytic theory (rho from Sigma_aug)          : %.2f%% low\n', theory_bias_pct_at5);
fprintf('Theory explains (vs V_IIR/V_theory)           : %.0f%% of observed\n', ...
        explained_pct_offline);
fprintf('Theory (pure analytic) explains               : %.0f%% of observed\n', ...
        explained_pct_theory);

if explained_pct_offline >= 70
    verdict = 'CONFIRMED';
    next_step = 'Task 1c: design bias correction factor, re-run Phase 2 benchmark.';
elseif explained_pct_offline >= 30
    verdict = 'PARTIAL';
    next_step = 'Task 1c (partial correction) + investigate residual bias sources.';
else
    verdict = 'REJECTED';
    next_step = 'Re-check L_ss consistency with Simulink runtime; find other bias source.';
end
fprintf('\nHypothesis verdict: %s\n', verdict);
fprintf('Next: %s\n\n', next_step);

%% Save results
out = struct();
out.a_prd_list               = a_prd_list;
out.a_cov_fixed              = a_cov_fixed;
out.V_IIR_afix               = V_IIR_afix;
out.V_IIR_match              = V_IIR_match;
out.V_sample_xyz             = V_sample_xyz;
out.V_theory_z               = V_theory_z;
out.V_theory_via_Sigma       = V_theory_via_Sigma;
out.rho_sample               = rho_sample;
out.rho_theory               = rho_th;
out.gamma_theory_unit        = gamma_th_uni;
out.predicted_bias_white     = predicted_bias_white;
out.predicted_bias_rho_th    = predicted_bias_rho_th;
out.predicted_bias_rho_smp   = predicted_bias_rho_smp;
out.lc                       = lc;
out.a_pd                     = a_pd;
out.a_nom                    = a_nom;
out.C_dpmr_eff_phase2        = C_dpmr_eff_phase2;
out.C_dpmr_eff_theory        = C_dpmr_eff_th;
out.phase2_bias_pct_obs      = phase2_bias_pct_obs;
out.theory_bias_pct_at5      = theory_bias_pct_at5;
out.empir_bias_pct_at5       = empir_bias_pct_at5;
out.empir_over_theory_at5    = empir_over_theory_at5;
out.explained_pct_theory     = explained_pct_theory;
out.explained_pct_offline    = explained_pct_offline;
out.verdict                  = verdict;
out.A_aug_max_eig            = diag_th.max_eig_A_aug;
out.L_max                    = L_max;
out.script                   = 'analyze_task1b_iir_bias.m';
out.data_source              = 'phase2_chisquared_mc.mat';

out_file = fullfile(project_root, 'test_results', 'verify', 'task1b_iir_bias_analysis.mat');
save(out_file, 'out');
fprintf('Saved: %s\n', out_file);

%% Figure
FS     = 16;
LW_th  = 2.5;
LW_sim = 1.8;

fig = figure('Position', [100, 100, 1400, 650], 'Color', 'w', 'Visible', 'off');

% Panel 1: Autocorrelation rho(L)
subplot(1, 2, 1);
L_plot   = 0:50;
p_smp = stem(L_plot, rho_sample(1:51), 'filled', ...
    'MarkerSize', 6, 'Color', [0 0.2 0.8], 'LineStyle', 'none');
hold on;
p_th  = plot(L_plot, rho_th(1:51), '-', 'LineWidth', LW_th, 'Color', [0.8 0 0]);
yline(0, 'k-', 'Alpha', 0.4);
xlabel('lag L [samples]', 'FontSize', FS, 'FontWeight', 'bold');
ylabel('\rho(L)', 'FontSize', FS, 'FontWeight', 'bold');
legend([p_smp, p_th], {'sample', 'theory (\Sigma_{aug})'}, ...
    'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS-2);
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2.0, 'Box', 'on');
ylim([-0.5, 1.05]);
xlim([0, 50]);

% Panel 2: IIR bias vs a_prd
subplot(1, 2, 2);
p_e = plot(a_prd_list, V_IIR_afix/V_sample, 'o', ...
    'MarkerSize', 10, 'MarkerFaceColor', [0 0.2 0.8], 'MarkerEdgeColor', [0 0.2 0.8]);
hold on;
p_th_ac = plot(a_prd_list, predicted_bias_rho_th, '-', ...
    'LineWidth', LW_th, 'Color', [0.8 0 0]);
p_th_w  = plot(a_prd_list, predicted_bias_white, '--', ...
    'LineWidth', LW_th, 'Color', [0 0.6 0]);
yline(1, 'k-', 'Alpha', 0.4);
yline(V_IIR_afix(i05)/V_theory_z, ':', 'Alpha', 0.6, 'Color', [0.3 0.3 0.3]);
xlabel('a_{prd}', 'FontSize', FS, 'FontWeight', 'bold');
ylabel('E[V_{IIR}] / V_{sample}', 'FontSize', FS, 'FontWeight', 'bold');
legend([p_e, p_th_ac, p_th_w], ...
    {'empirical', 'theory (autocorr)', 'theory (white)'}, ...
    'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS-2);
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2.0, ...
    'XScale', 'log', 'Box', 'on');
xlim([0.003, 0.2]);
ylim([0.85, 1.02]);

fig_path = fullfile(project_root, 'reference', 'for_test', 'fig_task1b_iir_bias.png');
exportgraphics(fig, fig_path, 'Resolution', 150);
close(fig);
fprintf('Figure saved: %s\n', fig_path);

fprintf('\nDone.\n');

%% Local functions
function V_IIR = compute_IIR_var_series(x, a_prd, a_cov)
    N = length(x);
    dpmrd = 0;
    dpmr2 = 0;
    V_IIR = zeros(N, 1);
    for k = 2:N
        dpmrd = (1 - a_prd) * dpmrd + a_prd * x(k);
        dpmr2 = (1 - a_cov) * dpmr2 + a_cov * x(k)^2;
        V_IIR(k) = max(dpmr2 - dpmrd^2, 0);
    end
end
