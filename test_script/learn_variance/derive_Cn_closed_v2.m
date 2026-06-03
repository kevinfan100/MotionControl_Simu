% derive_Cn_closed_v2.m
% Closed-form C_n derivation via direct sum of squared impulse response.
% Same method that produces the existing C_dpmr closed form.
%
% Setup (parallel to existing C_dpmr derivation):
%   F_T(z) = α/(1-α·z⁻¹) − α·c·z⁻³ · G(z)            ... for thermal
%   F_N(z) = (1-z⁻¹) · F_T(z) · z³                    ... for noise
%
%   where G(z) = 1/[(1-α·z⁻¹)(1-β·z⁻¹)]
%         α = 1-a_pd, β = λc, c = 1-β
%
% Time-domain impulse responses:
%   f_T[n] = α^(n+1) − α·c·g[n-3]
%   f_N[n] = α^(n+1) − α^n − α·c·g[n-3] + α·c·g[n-4]    (= f_T[n] − f_T[n-1])
%
%   g[n] = (α^(n+1) − β^(n+1))/(α−β)   for n≥0, else 0
%
% Variance:
%   C_dpmr = Σ f_T[n]²              (existing code)
%   C_n    = Σ f_N[n]² = 2 C_dpmr − 2 R_FT[1]
%
%   R_FT[1] = Σ f_T[n] · f_T[n+1]   (lag-1 autocorrelation of f_T)

clear; close all; clc;
[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(fileparts(script_dir));
cd(project_root);

lambda_c = 0.7;

%% Closed-form C_n via spectral relation C_n = 2 C_dpmr - 2 R_FT[1]
Cn_closed = @(a_pd, lc) compute_Cn_via_RFT(a_pd, lc);
Cn_num    = @(a_pd, lc) compute_Cn_numerical(a_pd, lc);
Cdpmr_closed = @(a_pd, lc) compute_Cdpmr_existing(a_pd, lc);

%% Verify
fprintf('=== Verify: closed-form C_n vs numerical integration ===\n');
fprintf(' a_pd  | C_n_closed     | C_n_numerical  | C_dpmr (existing) | R_FT[1]\n');
fprintf('-------|----------------|----------------|-------------------|--------\n');
for a = [0.001, 0.005, 0.01, 0.05, 0.1, 0.3, 0.5, 0.7]
    Cn_cl = Cn_closed(a, lambda_c);
    Cn_nm = Cn_num(a, lambda_c);
    Cdp   = Cdpmr_closed(a, lambda_c);
    Rft1  = (2*Cdp - Cn_cl) / 2;
    fprintf(' %.4f|  %.8f   |  %.8f   | %.6f         | %.6f\n', ...
            a, Cn_cl, Cn_nm, Cdp, Rft1);
end

%% Limit check
fprintf('\n=== Limit a_pd → 0:  C_n → 2/(1+λc) = %.6f ===\n', 2/(1+lambda_c));
for a = [1e-2, 1e-4, 1e-6, 1e-8]
    fprintf('  a_pd=%.0e:  C_n = %.6f\n', a, Cn_closed(a, lambda_c));
end

%% Apply to a_pd=0.5 Z axis
constants = physical_constants();
kBT       = constants.k_B * constants.T;
R_radius  = constants.R;
gamma_N   = constants.gamma_N;
sigma2_n_z = 0.00331^2;
Ts        = 1/1600;
a_nom     = Ts / gamma_N;
h_bar     = 30/R_radius;
[~, c_perp] = calc_correction_functions(max(h_bar, 1.001));
a_z = a_nom / c_perp;

a = 0.5;
Cdp = Cdpmr_closed(a, lambda_c);
Cn_new  = Cn_closed(a, lambda_c);
Cn_code = 2/(1+lambda_c);

theory_code = Cdp*4*kBT*a_z + Cn_code*sigma2_n_z;
theory_new  = Cdp*4*kBT*a_z + Cn_new *sigma2_n_z;

fprintf('\n=== a_pd=0.5 Z-axis evaluation ===\n');
fprintf('  C_dpmr = %.4f\n', Cdp);
fprintf('  C_n_code = %.4f,  C_n_new = %.4f  (code over by %.0fx)\n', ...
        Cn_code, Cn_new, Cn_code/Cn_new);
fprintf('  theory_code  = %.4e\n', theory_code);
fprintf('  theory_new   = %.4e\n', theory_new);
fprintf('  emp ratio vs code  = 0.932\n');
fprintf('  emp ratio vs new   = %.4f  (target=1.0)\n', 0.932 * theory_code/theory_new);

%% Plot
set(groot, 'defaultTextInterpreter','tex');
set(groot, 'defaultLegendInterpreter','tex');
set(groot, 'defaultAxesTickLabelInterpreter','tex');

a_dense = logspace(log10(0.001), log10(0.8), 200);
Cn_cl_arr = arrayfun(@(a) Cn_closed(a, lambda_c), a_dense);
Cn_nm_arr = arrayfun(@(a) Cn_num(a, lambda_c), a_dense);

fig = figure('Position', [50 50 1500 600], 'Color', 'w');
tl  = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
semilogx(a_dense, Cn_cl_arr, '-', 'Color', [0 0.45 0], 'LineWidth', 3, ...
         'DisplayName', 'C_n closed-form'); hold on;
semilogx(a_dense, Cn_nm_arr, '--', 'Color', [0.8 0.0 0.0], 'LineWidth', 2, ...
         'DisplayName', 'C_n numerical');
semilogx(a_dense, (2/(1+lambda_c))*ones(size(a_dense)), ':', ...
         'Color', [0.8 0.4 0.0], 'LineWidth', 2.5, ...
         'DisplayName', 'code = 2/(1+λ_c)');
set(gca, 'FontSize', 16, 'FontWeight', 'bold', 'LineWidth', 2);
xlabel('a\_pd', 'FontSize', 18);
ylabel('C_n', 'FontSize', 18);
title('C_n: closed-form via Σf²', 'FontSize', 17);
legend('show', 'FontSize', 13, 'Location', 'southwest');
grid on; box on;

nexttile;
err = abs(Cn_cl_arr - Cn_nm_arr);
semilogx(a_dense, err, '-', 'Color', [0.5 0.0 0.5], 'LineWidth', 2);
set(gca, 'FontSize', 16, 'FontWeight', 'bold', 'LineWidth', 2);
xlabel('a\_pd', 'FontSize', 18);
ylabel('|closed − numerical|', 'FontSize', 18);
title('Closed-form vs numerical (should be ~10^{-12})', 'FontSize', 17);
grid on; box on;

sgtitle('C_n closed-form via direct impulse-response sum (d=2)', ...
        'FontSize', 17, 'FontWeight', 'bold');

save_dir = fullfile(project_root, 'test_results', 'learn_variance');
out_path = fullfile(save_dir, 'derive_Cn_closed_v2.png');
exportgraphics(fig, out_path, 'Resolution', 150);
fprintf('\nSaved: %s\n', out_path);

set(groot, 'defaultTextInterpreter','remove');
set(groot, 'defaultLegendInterpreter','remove');
set(groot, 'defaultAxesTickLabelInterpreter','remove');

%% Helper: existing C_dpmr formula
function Cdp = compute_Cdpmr_existing(a_pd, lambda_c)
    a = a_pd; lc = lambda_c;
    bracket = 2*(1-a)*(1-lc)/(1-(1-a)*lc) ...
            + (2/(2-a))/((1+lc)*(1-(1-a)*lc));
    Cdp = (1-a)^2 * bracket;
end

%% Closed-form C_n via spectral identity C_n = 2·C_dpmr - 2·R_FT[1]
function Cn = compute_Cn_via_RFT(a_pd, lambda_c)
    alpha = 1-a_pd;  beta = lambda_c;  c = 1-beta;

    % Existing C_dpmr (Σf_T²)
    Cdp = compute_Cdpmr_existing(a_pd, lambda_c);

    % R_FT[1] = Σ f_T[n]·f_T[n+1]
    % Decomposition: f_T[n] = p1[n] - p2[n]
    %   p1[n] = α^(n+1)                       for n≥0
    %   p2[n] = α·c·(α^(n-2)-β^(n-2))/(α-β)    for n≥3, else 0

    % T1 = Σ_{n≥0} p1[n]·p1[n+1] = Σ α^(2n+3) = α³/(1-α²)
    T1 = alpha^3/(1-alpha^2);

    % T2 = Σ p1[n]·p2[n+1], requires n+1≥3 → n≥2
    %    = Σ_{n≥2} α^(n+1) · αc·(α^(n-1)-β^(n-1))/(α-β)
    %    = αc/(α-β) · [Σ_{n≥2} α^(2n)  −  Σ_{n≥2} α^(n+1)β^(n-1) ]
    %    = αc/(α-β) · [ α⁴/(1-α²) − α³β · (αβ)^0/(1-αβ) ]
    %      (Σ_{n≥2} α^(n+1)β^(n-1) = α³β·Σ_{m≥0}(αβ)^m = α³β/(1-αβ))
    T2_termA = alpha^4/(1-alpha^2);
    T2_termB = alpha^3 * beta / (1 - alpha*beta);
    T2 = alpha*c/(alpha-beta) * (T2_termA - T2_termB);

    % T3 = Σ p2[n]·p1[n+1], requires n≥3
    %    = Σ_{n≥3} αc·(α^(n-2)-β^(n-2))/(α-β) · α^(n+2)
    %    = αc/(α-β) · [Σ_{n≥3} α^(2n) − Σ_{n≥3} α^(n+2)β^(n-2)]
    %    = αc/(α-β) · [α⁶/(1-α²) − α⁵β·(αβ)^0/(1-αβ)]  ... let me recompute carefully
    % Σ_{n≥3} α^(2n) = α⁶/(1-α²)
    % Σ_{n≥3} α^(n+2)β^(n-2): substitute m=n-3, n=m+3 → α^(m+5)β^(m+1) = α⁵β·(αβ)^m
    %                       = α⁵β/(1-αβ)
    T3_termA = alpha^6/(1-alpha^2);
    T3_termB = alpha^5 * beta / (1 - alpha*beta);
    T3 = alpha*c/(alpha-beta) * (T3_termA - T3_termB);

    % T4 = Σ p2[n]·p2[n+1], requires n≥3
    %    = α²c²/(α-β)² · Σ_{n≥3} (α^(n-2)-β^(n-2))(α^(n-1)-β^(n-1))
    %    = α²c²/(α-β)² · Σ_{m≥0} (α^(m+1)-β^(m+1))(α^(m+2)-β^(m+2))
    %      (m=n-3)
    %    = α²c²/(α-β)² · [α³/(1-α²) − α²β/(1-αβ) − αβ²/(1-αβ) + β³/(1-β²)]
    %    = α²c²/(α-β)² · [α³/(1-α²) − αβ(α+β)/(1-αβ) + β³/(1-β²)]
    A4 = alpha^3/(1-alpha^2);
    B4 = alpha*beta*(alpha+beta)/(1-alpha*beta);
    C4 = beta^3/(1-beta^2);
    T4 = alpha^2*c^2/(alpha-beta)^2 * (A4 - B4 + C4);

    R_FT_1 = T1 - T2 - T3 + T4;

    Cn = 2*Cdp - 2*R_FT_1;
end

function Cn = compute_Cn_numerical(a_pd, lambda_c)
    % FIX: S_n_dpm for d=2 should use z⁻³ (not z⁻²), because
    %      δp_m_from_n = 1 - z⁻ᵈ·H_n  with H_n = z⁻¹·(1-λc)/(1-λc·z⁻¹)
    %      → 1 - z⁻(d+1)·(1-λc)/(1-λc·z⁻¹)
    omega = linspace(-pi, pi, 16384);
    z_inv = exp(-1i*omega);
    H_HP  = (1-a_pd)*(1-z_inv) ./ (1 - (1-a_pd)*z_inv);
    S_n   = 1 - z_inv.^3 .* (1-lambda_c) ./ (1 - lambda_c*z_inv);  % ★ z⁻³
    Cn    = trapz(omega, abs(H_HP.*S_n).^2) / (2*pi);
end
