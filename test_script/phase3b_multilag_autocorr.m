%% phase3b_multilag_autocorr.m — Multi-lag autocorrelation for a estimation
%
% Phase 2A showed that IIR variance estimation has ~43% relative std, limited
% by chi-squared noise + temporal autocorrelation. This script explores
% multi-lag autocorrelation as an alternative.
%
% Theory:
%   gamma_L = E[del_pmr[k] * del_pmr[k-L]]
%           = c' * A_aug^L * Sigma_steady * c    (for L >= 0, thermal-only)
%           = alpha_L * a   (linear in a, since Sigma_steady_thermal = a * Sigma_th_unit)
%
% Using multiple lags gives an over-determined LS estimate:
%   a_hat = argmin_a sum_L w_L * (gamma_L_empirical - alpha_L * a)^2
%
% If the effective sample size scales with L, we'd expect ~sqrt(L) precision
% improvement. Verify via Phase 2A data and windowed estimates.

clear; close all; clc;

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(script_dir);
cd(project_root);

addpath(fullfile(project_root, 'model'));
addpath(fullfile(project_root, 'model', 'config'));
addpath(script_dir);

fprintf('===== phase3b_multilag_autocorr =====\n\n');

%% Load Phase 2A data
p2a = load(fullfile(project_root, 'test_results', 'verify', 'phase2_chisquared_mc.mat'));
results_2a = p2a.results;
del_pmr = results_2a.del_pmr;
a_nom = results_2a.a_nom;
C_dpmr_eff = results_2a.C_dpmr_eff;
ss_start = results_2a.ss_start;
N = size(del_pmr, 2);
ss = ss_start:N;

fprintf('Loaded Phase 2A data: %d steady-state samples\n', length(ss));

%% Recompute A_aug and Sigma_th_unit for the same (lc, a_pd, Q, R)
constants = physical_constants();
k_B = constants.k_B; T_temp = constants.T;
Ts = constants.Ts; gamma_N = constants.gamma_N;
sigma2_dXT = 4 * k_B * T_temp * Ts / gamma_N;

lc = results_2a.lc;
a_pd = 0.05;  % same as Phase 2A
Q_kf_scale = [0; 0; 1e4; 1e-1; 0; 1e-4; 0];
R_kf_scale = [1e-2; 1e0];

fprintf('Recomputing augmented Lyapunov at lc=%.2f, a_pd=%.3f...\n', lc, a_pd);
[~, ~, L, A_aug, diag_out] = compute_7state_cdpmr_eff( ...
    lc, 0.05, a_pd, Q_kf_scale, R_kf_scale);
fprintf('  DARE iters: %d, max|eig(A_aug)| = %.6f\n', diag_out.dare_iters, diag_out.max_eig_A_aug);

Sigma_th_unit = diag_out.Sigma_th;  % 11x11, per unit thermal variance

%% Extraction vector: del_pmr[k] = (1-a_pd) * (x[3] - x[11]) + (1-a_pd)*n_p[k]
% For thermal-only: del_pmr[k] = (1-a_pd) * (dx_d2[k] - pmd_prev[k])
%                              = c' * x_aug[k]
c_vec = zeros(11, 1);
c_vec(3)  = (1 - a_pd);
c_vec(11) = -(1 - a_pd);

% Verify 0-lag matches C_dpmr_eff (should match Phase 1 result)
gamma_0_theory = c_vec' * Sigma_th_unit * c_vec;
fprintf('  c'' * Sigma_th_unit * c = %.6f (should equal C_dpmr_eff = %.6f)\n', ...
        gamma_0_theory, C_dpmr_eff);

%% Compute theoretical alpha_L for L = 0, 1, ..., L_max
% gamma_L_thermal = c' * A_aug^L * Sigma_th_unit * c * a
L_max = 30;
alpha_L = zeros(L_max+1, 1);   % gamma_L / a
An = Sigma_th_unit;  % Sigma_steady at L=0
alpha_L(1) = c_vec' * An * c_vec;  % L=0
A_power = eye(11);
for L = 1:L_max
    A_power = A_power * A_aug;
    % gamma_L = c' * A^L * Sigma_th_unit * c
    alpha_L(L+1) = c_vec' * A_power * Sigma_th_unit * c_vec;
end

fprintf('\nTheoretical alpha_L (first 11 lags):\n');
for L = 0:min(10, L_max)
    fprintf('  L=%2d: alpha = %+.5f, ratio to alpha_0 = %+.4f\n', ...
            L, alpha_L(L+1), alpha_L(L+1)/alpha_L(1));
end

%% Empirical autocorrelation of del_pmr (single long window)
fprintf('\n--- Empirical autocorrelation of del_pmr (z-axis) ---\n');

x = del_pmr(3, ss);  % z-axis del_pmr
N_ss = length(x);
x = x - mean(x);  % center

gamma_L_emp = zeros(L_max+1, 1);
for L = 0:L_max
    if L == 0
        gamma_L_emp(1) = mean(x.^2);
    else
        gamma_L_emp(L+1) = mean(x(1:end-L) .* x(L+1:end));
    end
end

fprintf('Empirical gamma_L (z-axis, whole window):\n');
for L = 0:min(10, L_max)
    theory = alpha_L(L+1) * sigma2_dXT;  % at a = a_nom
    if theory ~= 0
        rel_err = 100*(gamma_L_emp(L+1) - theory)/theory;
        fprintf('  L=%2d: emp=%.4e, theory(a=a_nom)=%.4e, rel_err=%+5.1f%%\n', ...
                L, gamma_L_emp(L+1), theory, rel_err);
    else
        fprintf('  L=%2d: emp=%.4e, theory=0\n', L, gamma_L_emp(L+1));
    end
end

%% LS estimate: a from multiple lags
fprintf('\n--- LS estimate using multiple lags ---\n');

% Convert empirical gamma_L to a estimate via LS: a = sum(alpha_L * gamma_L) / sum(alpha_L^2)
% Per-sample-variance noise weighting: assume var(gamma_L_emp) roughly equal across lags
% for large N; use unweighted LS.

for L_use = [1, 2, 5, 10, 20]   % use first L_use+1 lags (0..L_use)
    aL = alpha_L(1:L_use+1) * sigma2_dXT;   % expected gamma_L per unit a
    gL = gamma_L_emp(1:L_use+1);
    if norm(aL) > 0
        a_LS = (aL' * gL) / (aL' * aL) * a_nom;  % a scaled back
    else
        a_LS = 0;
    end
    err_pct = 100*(a_LS - a_nom)/a_nom;
    fprintf('  using L=0..%2d: a_LS = %.4e, err = %+5.2f%%\n', L_use, a_LS, err_pct);
end

%% Compare window-based statistical noise: single-lag vs multi-lag
fprintf('\n--- Statistical noise comparison (window-based) ---\n');

window_length = 800;    % 0.5 seconds
n_windows = floor(N_ss / window_length);
fprintf('Window length %d (%.2fs), %d windows\n', window_length, window_length*Ts, n_windows);

a_single_wins = zeros(n_windows, 1);
a_multi_wins = cell(length([1, 5, 10, 20]), 1);
L_list = [1, 5, 10, 20];
for i = 1:length(L_list)
    a_multi_wins{i} = zeros(n_windows, 1);
end

for w = 1:n_windows
    idx = (w-1)*window_length+1 : w*window_length;
    xw = x(idx);
    xw = xw - mean(xw);

    % Empirical gamma_L for this window
    g_win = zeros(L_max+1, 1);
    for L = 0:L_max
        if L == 0
            g_win(1) = mean(xw.^2);
        else
            g_win(L+1) = mean(xw(1:end-L) .* xw(L+1:end));
        end
    end

    % Single-lag (L=0) estimate
    a_single_wins(w) = g_win(1) / (alpha_L(1) * sigma2_dXT) * a_nom;

    % Multi-lag estimates
    for i = 1:length(L_list)
        L_use = L_list(i);
        aL = alpha_L(1:L_use+1) * sigma2_dXT;
        gL = g_win(1:L_use+1);
        if norm(aL) > 0
            a_multi_wins{i}(w) = (aL' * gL) / (aL' * aL) * a_nom;
        end
    end
end

fprintf('\nWindow statistics (mean, std, rel std):\n');
fprintf('  Method           mean(a_hat)   std        rel_std    bias\n');
m = mean(a_single_wins);
s = std(a_single_wins);
fprintf('  Single-lag (L=0) %.4e   %.4e   %5.2f%%   %+5.2f%%\n', ...
        m, s, 100*s/m, 100*(m-a_nom)/a_nom);
for i = 1:length(L_list)
    mw = mean(a_multi_wins{i});
    sw = std(a_multi_wins{i});
    fprintf('  Multi-lag L=%2d:  %.4e   %.4e   %5.2f%%   %+5.2f%%\n', ...
            L_list(i), mw, sw, 100*sw/mw, 100*(mw-a_nom)/a_nom);
end

% Expected sqrt(L) improvement
fprintf('\nExpected improvement from L lags (ideal sqrt(L)):\n');
for L = [1, 5, 10, 20]
    fprintf('  L=%2d: sqrt(L) = %.3f (i.e. %.1f%% of single-lag std)\n', ...
            L, sqrt(L), 100/sqrt(L));
end

%% Save results
phase3b_results = struct();
phase3b_results.alpha_L = alpha_L;
phase3b_results.gamma_L_emp = gamma_L_emp;
phase3b_results.a_single_wins = a_single_wins;
phase3b_results.a_multi_wins = a_multi_wins;
phase3b_results.L_list = L_list;
phase3b_results.window_length = window_length;
phase3b_results.lc = lc;
phase3b_results.a_pd = a_pd;
phase3b_results.sigma2_dXT = sigma2_dXT;
phase3b_results.a_nom = a_nom;

out_dir = fullfile(project_root, 'test_results', 'verify');
save(fullfile(out_dir, 'phase3b_multilag_autocorr.mat'), 'phase3b_results');
fprintf('\nSaved: phase3b_multilag_autocorr.mat\n');

%% Figures
fig = figure('Position', [100 100 1400 700]);

subplot(1, 2, 1);
plot(0:L_max, alpha_L, 'b-o', 'LineWidth', 1.5); hold on;
plot(0:L_max, gamma_L_emp/sigma2_dXT, 'r--s', 'LineWidth', 1.5);
xlabel('Lag L');
ylabel('gamma_L / sigma2_{dXT}');
legend({'Theory (alpha_L)', 'Empirical (a=a_{nom})'}, 'Location', 'best');
title('Multi-lag autocorrelation of del_{pmr}');
grid on;
set(gca, 'FontSize', 12);

subplot(1, 2, 2);
rel_stds = [std(a_single_wins)/mean(a_single_wins) * 100];
labels = {'L=0'};
for i = 1:length(L_list)
    rel_stds(end+1) = std(a_multi_wins{i})/mean(a_multi_wins{i}) * 100;
    labels{end+1} = sprintf('L<=%d', L_list(i));
end
bar(rel_stds); set(gca, 'XTickLabel', labels);
ylabel('Relative std of a_{hat} [%]');
title(sprintf('Statistical precision (window %.2fs)', window_length*Ts));
grid on;
set(gca, 'FontSize', 12);

fig_dir = fullfile(project_root, 'reference', 'for_test');
saveas(fig, fullfile(fig_dir, 'fig_phase3b_multilag.png'));
fprintf('Figure saved: fig_phase3b_multilag.png\n');

fprintf('\nDone.\n');
