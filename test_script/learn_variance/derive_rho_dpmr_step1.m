% derive_rho_dpmr_step1.m
% Phase 1 verification: closed-form R_T(τ) and R_N(τ) for τ=0..30
% vs brute-force impulse-response autocorrelation.
%
% See derivation_IF_eff.md §1-§3.
%
% Strategy:
%   (A) Brute-force: compute impulse responses f_T[n], f_N[n] from
%       F_T(z), F_N(z) (without z⁻³ delay, which doesn't affect autocorrelation)
%       Then R(τ) = Σ_n h[n]·h[n+τ] truncated at large N.
%   (B) Closed-form: use the analytical formulas for T₁..T₄ and R_T(τ),
%       then derive R_N(τ) = 2·R_T(τ) − R_T(τ−1) − R_T(τ+1).
%   (C) Verify (A) == (B) to machine precision.

clear; close all; clc;
[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(fileparts(script_dir));
cd(project_root);

%% Setup
lambda_c = 0.7;
a_pd     = 0.05;
alpha    = 1 - a_pd;
beta     = lambda_c;
c        = 1 - beta;
tau_max  = 30;

%% (A) Brute-force impulse responses
N_imp = 5000;   % truncation horizon (large enough for AR decay)

% F_T_eff(z) = α/(1-αz⁻¹) - αc·z⁻³/[(1-αz⁻¹)(1-βz⁻¹)]
% Time domain:
%   p_1[n] = α^(n+1)              for n≥0
%   p_2[n] = αc·g[n-3]            for n≥3
%   g[m]   = (α^(m+1) - β^(m+1))/(α-β)

p1 = zeros(N_imp, 1);
p2 = zeros(N_imp, 1);
for n = 0:N_imp-1
    p1(n+1) = alpha^(n+1);
    if n >= 3
        m = n - 3;
        g_m = (alpha^(m+1) - beta^(m+1))/(alpha - beta);
        p2(n+1) = alpha * c * g_m;
    end
end
f_T = p1 - p2;

% F_N(z) = (1-z⁻¹)·F_T(z) → f_N[n] = f_T[n] - f_T[n-1]
f_N = f_T - [0; f_T(1:end-1)];

% Brute-force autocorrelations
R_T_brute = zeros(tau_max+1, 1);
R_N_brute = zeros(tau_max+1, 1);
for tau = 0:tau_max
    R_T_brute(tau+1) = sum(f_T(1:end-tau) .* f_T(1+tau:end));
    R_N_brute(tau+1) = sum(f_N(1:end-tau) .* f_N(1+tau:end));
end

%% (B) Closed-form using derived T₁..T₄ split
R_T_closed = zeros(tau_max+1, 1);
for tau = 0:tau_max
    % T_1(τ) = α^(τ+2) / (1-α²)   for all τ ≥ 0
    T1 = alpha^(tau+2) / (1 - alpha^2);

    % T_2(τ): two regimes
    if tau <= 3
        T2 = alpha^(5-tau) * c / ((1 - alpha^2)*(1 - alpha*beta));
    else
        T2 = (alpha^2 * c)/(alpha - beta) * ...
             (alpha^(tau-2)/(1-alpha^2) - beta^(tau-2)/(1-alpha*beta));
    end

    % T_3(τ) = α^(τ+5)·c / [(1-α²)(1-αβ)]    for all τ ≥ 0
    T3 = alpha^(tau+5) * c / ((1 - alpha^2)*(1 - alpha*beta));

    % T_4(τ) = α²c²·R_g[τ]
    %   R_g[τ] = 1/(α-β)² · [α^(τ+2)/(1-α²) + β^(τ+2)/(1-β²)
    %                       - αβ(α^τ+β^τ)/(1-αβ)]
    R_g = 1/(alpha-beta)^2 * ( ...
            alpha^(tau+2)/(1-alpha^2) + beta^(tau+2)/(1-beta^2) ...
          - alpha*beta*(alpha^tau + beta^tau)/(1-alpha*beta));
    T4 = alpha^2 * c^2 * R_g;

    R_T_closed(tau+1) = T1 - T2 - T3 + T4;
end

% R_N(τ) from R_T using (1-z⁻¹) relationship:
% R_N(τ) = 2·R_T(τ) - R_T(τ-1) - R_T(τ+1)
% For τ=0: R_N(0) = 2 R_T(0) - 2 R_T(1)  (using R_T(-1)=R_T(1) by symmetry)
R_N_closed = zeros(tau_max+1, 1);
% Need R_T(τ+1), so loop only up to tau_max-1; for tau_max use brute approximation
for tau = 0:tau_max-1
    if tau == 0
        % R_N(0) = 2 R_T(0) - 2 R_T(1)  (symmetric)
        R_N_closed(1) = 2*R_T_closed(1) - 2*R_T_closed(2);
    else
        R_N_closed(tau+1) = 2*R_T_closed(tau+1) - R_T_closed(tau) - R_T_closed(tau+2);
    end
end
% For tau_max, use brute-force value (boundary)
R_N_closed(tau_max+1) = R_N_brute(tau_max+1);

%% Print comparison
fprintf('=== R_T(τ) verification: closed-form vs brute-force ===\n');
fprintf(' τ  | R_T_brute      | R_T_closed     | |diff|\n');
fprintf('----|----------------|----------------|-----------\n');
for tau = 0:min(tau_max, 8)
    fprintf(' %2d | %.10e | %.10e | %.2e\n', tau, R_T_brute(tau+1), R_T_closed(tau+1), ...
            abs(R_T_brute(tau+1) - R_T_closed(tau+1)));
end

fprintf('\n=== R_N(τ) verification: closed-form vs brute-force ===\n');
fprintf(' τ  | R_N_brute      | R_N_closed     | |diff|\n');
fprintf('----|----------------|----------------|-----------\n');
for tau = 0:min(tau_max-1, 8)
    fprintf(' %2d | %.10e | %.10e | %.2e\n', tau, R_N_brute(tau+1), R_N_closed(tau+1), ...
            abs(R_N_brute(tau+1) - R_N_closed(tau+1)));
end

%% Sanity check: R_T(0) = C_dpmr, R_N(0) = C_n
denom_common = 1 - (1-a_pd)*lambda_c;
C_dpmr_formula = (1-a_pd)^2 * ( 2*(1-a_pd)*(1-lambda_c)/denom_common ...
                              + (2/(2-a_pd))/((1+lambda_c)*denom_common) );
Cn_t1 = 2/(2-a_pd);
Cn_t2 = 2*(1-a_pd)^2*a_pd*(1-lambda_c)/((2-a_pd)*denom_common);
Cn_t3 = 2*(1-lambda_c)^2/((2-a_pd)*(1+lambda_c)*denom_common);
C_n_formula = (1-a_pd)^2 * (Cn_t1 + Cn_t2 + Cn_t3);

fprintf('\n=== R(0) sanity ===\n');
fprintf('  R_T(0) brute      = %.8f\n', R_T_brute(1));
fprintf('  R_T(0) closed     = %.8f\n', R_T_closed(1));
fprintf('  C_dpmr formula    = %.8f  (should match R_T(0))\n', C_dpmr_formula);
fprintf('  R_N(0) brute      = %.8f\n', R_N_brute(1));
fprintf('  R_N(0) closed     = %.8f\n', R_N_closed(1));
fprintf('  C_n formula       = %.8f  (should match R_N(0))\n', C_n_formula);

%% Sanity: large τ asymptotic form  R_T(τ) ~ A_T·α^τ + B_T·β^τ
% Fit A_T and B_T from R_T(τ=10) and R_T(τ=11) (asymptotic regime)
% System: A_T·α^10 + B_T·β^10 = R_T(10)
%         A_T·α^11 + B_T·β^11 = R_T(11)
M = [alpha^10, beta^10; alpha^11, beta^11];
sol = M \ [R_T_closed(11); R_T_closed(12)];
A_T_fit = sol(1);
B_T_fit = sol(2);
fprintf('\n=== Asymptotic R_T(τ) ≈ A_T·α^τ + B_T·β^τ ===\n');
fprintf('  Fitted from τ=10,11: A_T = %.4f, B_T = %.4f\n', A_T_fit, B_T_fit);
% Check at τ=20: prediction
pred_20 = A_T_fit*alpha^20 + B_T_fit*beta^20;
fprintf('  Prediction at τ=20: %.4e, brute: %.4e  (should match)\n', pred_20, R_T_brute(21));

%% Plot
set(groot, 'defaultTextInterpreter','tex');
set(groot, 'defaultLegendInterpreter','tex');
set(groot, 'defaultAxesTickLabelInterpreter','tex');

fig = figure('Position', [50 50 1500 700], 'Color', 'w');
tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% R_T comparison
nexttile;
tau_axis = 0:tau_max;
plot(tau_axis, R_T_brute, 'ko-', 'LineWidth', 2, 'MarkerSize', 7, ...
     'MarkerFaceColor', 'k', 'DisplayName', 'R_T brute'); hold on;
plot(tau_axis, R_T_closed, 'rx', 'LineWidth', 2, 'MarkerSize', 12, ...
     'DisplayName', 'R_T closed-form');
% asymptotic tail
plot(tau_axis(11:end), A_T_fit*alpha.^tau_axis(11:end) + B_T_fit*beta.^tau_axis(11:end), ...
     '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5, ...
     'DisplayName', 'A_T·α^τ + B_T·β^τ');
set(gca, 'FontSize', 16, 'FontWeight', 'bold', 'LineWidth', 2);
xlabel('τ', 'FontSize', 18);
ylabel('R_T(τ)', 'FontSize', 18);
title('R_T(τ): brute-force vs closed-form', 'FontSize', 16);
legend('show', 'FontSize', 13);
grid on; box on;

% R_N comparison
nexttile;
plot(tau_axis, R_N_brute, 'ko-', 'LineWidth', 2, 'MarkerSize', 7, ...
     'MarkerFaceColor', 'k', 'DisplayName', 'R_N brute'); hold on;
plot(tau_axis, R_N_closed, 'rx', 'LineWidth', 2, 'MarkerSize', 12, ...
     'DisplayName', 'R_N closed-form (via R_T)');
set(gca, 'FontSize', 16, 'FontWeight', 'bold', 'LineWidth', 2);
xlabel('τ', 'FontSize', 18);
ylabel('R_N(τ)', 'FontSize', 18);
title('R_N(τ): brute-force vs closed-form', 'FontSize', 16);
legend('show', 'FontSize', 13);
grid on; box on;

sgtitle(sprintf('Phase 1 verification (α=%.3f, β=%.2f, c=%.2f)', alpha, beta, c), ...
        'FontSize', 17, 'FontWeight', 'bold');

save_dir = fullfile(project_root, 'test_results', 'learn_variance');
out_path = fullfile(save_dir, 'derive_rho_dpmr_step1.png');
exportgraphics(fig, out_path, 'Resolution', 150);
fprintf('\nSaved: %s\n', out_path);

set(groot, 'defaultTextInterpreter','remove');
set(groot, 'defaultLegendInterpreter','remove');
set(groot, 'defaultAxesTickLabelInterpreter','remove');
