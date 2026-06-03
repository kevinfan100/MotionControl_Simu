% verify_R22_intrinsic.m
% Verify R22_intrinsic absolute value and scaling law on segmented ramp.
%
%   R22_intrinsic = R22_prefactor · IF_eff · (a + xi)^2         [a_xm space]
%                 = R22_prefactor · IF_eff · sigma2_dxr^2 / (C_dpmr·4kBT)^2
%
% Per-segment:
%   - empirical Var(sigma2_dxr_hat) within segment (linear-detrended)
%   - theory  = R22_prefactor · IF_eff · sigma2_dxr_theory(h_mid)^2
%
% Data source: test_results/learn_variance/compare_am.mat  (run compare_am.m first)

clear; close all; clc;

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(fileparts(script_dir));
cd(project_root);
addpath(fullfile(project_root, 'model'));
addpath(fullfile(project_root, 'model', 'config'));
addpath(fullfile(project_root, 'model', 'wall_effect'));
addpath(fullfile(project_root, 'model', 'controller'));

%% Load compare_am.mat
mat_path = fullfile(project_root, 'test_results', 'learn_variance', 'compare_am.mat');
if ~exist(mat_path, 'file')
    error('compare_am.mat not found. Run compare_am.m first.');
end
S = load(mat_path);

tout            = S.tout;
sigma2_ctrl_z   = S.sigma2_ctrl_z;
a_xm_ctrl_z     = S.a_xm_ctrl_z;
a_hat_ctrl_z    = S.a_hat_ctrl_z;
a_z_true        = S.a_z_true;
C_dpmr          = S.C_dpmr;
C_n             = S.C_n;
a_pd            = S.a_pd;
a_cov           = S.a_cov;
lambda_c        = S.lambda_c;

constants  = physical_constants();
kBT        = constants.k_B * constants.T;
R_radius   = constants.R;
if isfield(S, 'sigma2_n_s')
    sigma2_n_z = S.sigma2_n_s(3);          % use ACTUAL sim noise
else
    error(['compare_am.mat is stale (missing sigma2_n_s). ' ...
           'Re-run compare_am.m to refresh.']);
end

N     = length(tout);
T_sim = tout(end);

%% Build EXACTLY the same R22 constants the controller uses
% Match build_eq17_constants.m Option A_MA2_full path
denom_e = 1 + 2 * (1 - lambda_c)^2;
rho_e_1 = (1 - lambda_c) * (2 - lambda_c) / denom_e;
rho_e_2 = (1 - lambda_c) / denom_e;
Var_dx_over_sig_e = (1 + 2*lambda_c*rho_e_1 + 2*lambda_c^2*rho_e_2) / (1 - lambda_c^2);
inv_var_ratio = 1 / Var_dx_over_sig_e;
rho_dx_1 = lambda_c + inv_var_ratio * (rho_e_1 + lambda_c*rho_e_2);
rho_dx_2 = lambda_c * rho_dx_1 + inv_var_ratio * rho_e_2;
s_ewma   = 1 - a_cov;
IF_eff   = 1 + 2 * (rho_dx_1^2 * s_ewma ...
                  + rho_dx_2^2 * s_ewma^2 / (1 - lambda_c^2 * s_ewma));
R22_prefactor = 2 * a_cov / (2 - a_cov);
xi_z          = (C_n / C_dpmr) * sigma2_n_z / (4 * kBT);

fprintf('=== R22 constants (closed-form, Option A_MA2_full) ===\n');
fprintf('  lambda_c = %.3f, a_cov = %.3f\n', lambda_c, a_cov);
fprintf('  R22_prefactor = 2·a_cov/(2-a_cov) = %.6f\n', R22_prefactor);
fprintf('  IF_eff(s=%.2f)               = %.4f\n', s_ewma, IF_eff);
fprintf('  rho_delta_x(1) = %.4f, rho(2) = %.4f\n', rho_dx_1, rho_dx_2);
fprintf('  xi_z (sensor-eq mobility)    = %.4e [um/pN]\n', xi_z);
fprintf('  C_dpmr = %.4f, C_n = %.4f\n\n', C_dpmr, C_n);

%% Theory mean trajectory (for plotting only — detrend uses linear fit)
sigma2_dxr_theory_t = C_dpmr * 4 * kBT * a_z_true + C_n * sigma2_n_z;
R22_theory_t_sig2   = R22_prefactor * IF_eff * sigma2_dxr_theory_t .^ 2;
R22_theory_t_a_az   = R22_prefactor * IF_eff * (a_z_true     + xi_z).^2;
R22_theory_t_a_ah   = R22_prefactor * IF_eff * (a_hat_ctrl_z + xi_z).^2;

%% Per-segment empirical vs theory
n_seg = 10;
seg_edges = linspace(0, T_sim, n_seg+1);

seg_t_center      = zeros(n_seg, 1);
seg_a_z_mid       = zeros(n_seg, 1);
seg_a_hat_mid     = zeros(n_seg, 1);
seg_R22_emp_sig2  = zeros(n_seg, 1);   % var(sigma2_ctrl_z), linear-detrended  [um^4]
seg_R22_th_sig2   = zeros(n_seg, 1);   % R22_prefactor·IF_eff·sigma2_dxr^2     [um^4]
seg_R22_emp_a     = zeros(n_seg, 1);   % var(a_xm_ctrl_z), linear-detrended    [(um/pN)^2]
seg_R22_th_a_az   = zeros(n_seg, 1);   % R22_prefactor·IF_eff·(a_z+xi)^2       [(um/pN)^2]
seg_R22_th_a_ah   = zeros(n_seg, 1);   % R22_prefactor·IF_eff·(a_hat+xi)^2     [(um/pN)^2]
seg_N_eff         = zeros(n_seg, 1);

for s = 1:n_seg
    if s < n_seg
        idx = (tout >= seg_edges(s)) & (tout < seg_edges(s+1));
    else
        idx = (tout >= seg_edges(s)) & (tout <= seg_edges(s+1));
    end

    t_seg     = tout(idx);
    sig_seg   = sigma2_ctrl_z(idx);
    axm_seg   = a_xm_ctrl_z(idx);

    % Linear detrend (slope due to ramp's a_z drift within window)
    p_sig          = polyfit(t_seg, sig_seg, 1);
    sig_residual   = sig_seg - polyval(p_sig, t_seg);
    p_axm          = polyfit(t_seg, axm_seg, 1);
    axm_residual   = axm_seg - polyval(p_axm, t_seg);

    seg_t_center(s)     = mean(t_seg);
    seg_R22_emp_sig2(s) = var(sig_residual);
    seg_R22_emp_a(s)    = var(axm_residual);

    a_z_mid             = mean(a_z_true(idx));
    a_hat_mid           = mean(a_hat_ctrl_z(idx));
    sigma2_mid          = mean(sigma2_dxr_theory_t(idx));
    seg_a_z_mid(s)      = a_z_mid;
    seg_a_hat_mid(s)    = a_hat_mid;
    seg_R22_th_sig2(s)  = R22_prefactor * IF_eff * sigma2_mid^2;
    seg_R22_th_a_az(s)  = R22_prefactor * IF_eff * (a_z_mid   + xi_z)^2;
    seg_R22_th_a_ah(s)  = R22_prefactor * IF_eff * (a_hat_mid + xi_z)^2;
    % effective independent samples for an EWMA with smoothing a_cov:
    %   N_eff ≈ N_samples · a_cov / (2 - a_cov)
    seg_N_eff(s) = sum(idx) * a_cov / (2 - a_cov);
end

%% Print tables
fprintf('=== Per-segment R22 verification  (sigma^2 space, units [um^4]) ===\n');
fprintf(' seg | t_ctr | a_z_mid    a_hat_mid  | R22_emp        R22_theory     emp/th\n');
fprintf('-----|-------|-----------|------------|---------------|---------------|------\n');
for s = 1:n_seg
    fprintf(' %2d  | %5.2fs| %.3e %.3e | %.3e     %.3e     %.3f\n', ...
            s, seg_t_center(s), seg_a_z_mid(s), seg_a_hat_mid(s), ...
            seg_R22_emp_sig2(s), seg_R22_th_sig2(s), ...
            seg_R22_emp_sig2(s) / seg_R22_th_sig2(s));
end

fprintf('\n=== Per-segment R22 verification  (a_xm space, units [(um/pN)^2]) ===\n');
fprintf(' seg | t_ctr | R22_emp(a_xm)  R22_th(a_z)    R22_th(a_hat) | r(az)  r(ah)\n');
fprintf('-----|-------|---------------|---------------|--------------|------|------\n');
for s = 1:n_seg
    fprintf(' %2d  | %5.2fs| %.3e     %.3e     %.3e    | %.3f  %.3f\n', ...
            s, seg_t_center(s), ...
            seg_R22_emp_a(s), seg_R22_th_a_az(s), seg_R22_th_a_ah(s), ...
            seg_R22_emp_a(s) / seg_R22_th_a_az(s), ...
            seg_R22_emp_a(s) / seg_R22_th_a_ah(s));
end

fprintf('\n=== Summary ratios ===\n');
fprintf('  sigma^2-space: mean(emp/theory) = %.3f, std = %.3f\n', ...
        mean(seg_R22_emp_sig2./seg_R22_th_sig2), ...
        std(seg_R22_emp_sig2./seg_R22_th_sig2));
fprintf('  a_xm-space  : mean(emp/theory_az)  = %.3f, std = %.3f\n', ...
        mean(seg_R22_emp_a./seg_R22_th_a_az), ...
        std(seg_R22_emp_a./seg_R22_th_a_az));
fprintf('  a_xm-space  : mean(emp/theory_ah)  = %.3f, std = %.3f\n', ...
        mean(seg_R22_emp_a./seg_R22_th_a_ah), ...
        std(seg_R22_emp_a./seg_R22_th_a_ah));

%% Plot
set(groot, 'defaultTextInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex');

COL_EMP_SIG = [0.85 0.00 0.00];   % red
COL_EMP_A   = [0.00 0.20 0.80];   % blue
COL_TH      = [0.00 0.55 0.00];   % green
COL_AUX     = [0.45 0.45 0.45];   % gray

fig = figure('Position', [50 50 1600 1000], 'Color', 'w');
tl  = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% ------ (1) sigma^2-space: R22 emp vs theory per segment ------
nexttile;
plot(seg_t_center, seg_R22_emp_sig2, 'o-', 'Color', COL_EMP_SIG, 'LineWidth', 2.2, ...
     'MarkerFaceColor', COL_EMP_SIG, 'MarkerSize', 8); hold on;
plot(seg_t_center, seg_R22_th_sig2,  '--', 'Color', COL_TH, 'LineWidth', 3);
set(gca, 'FontSize', 18, 'FontWeight', 'bold', 'LineWidth', 2);
xlabel('Time [sec]', 'FontSize', 20);
ylabel('$R_{22}$ [um$^4$]', 'FontSize', 20);
title('(1) $\sigma^2$-space:  empirical vs theory', 'FontSize', 18);
legend({'emp $\mathrm{Var}(\widehat{\sigma^2_{\delta x_r}})$', ...
        'theory $\Pi \cdot \mathrm{IF}_{\rm eff} \cdot \sigma_{\delta x_r}^4$'}, ...
       'FontSize', 14, 'Location', 'best');
grid on; box on; xlim([0 T_sim]);

% ------ (2) a_xm-space: R22 emp vs theory ------
nexttile;
plot(seg_t_center, seg_R22_emp_a,    'o-', 'Color', COL_EMP_A, 'LineWidth', 2.2, ...
     'MarkerFaceColor', COL_EMP_A, 'MarkerSize', 8); hold on;
plot(seg_t_center, seg_R22_th_a_az,  '--', 'Color', COL_TH, 'LineWidth', 3);
plot(seg_t_center, seg_R22_th_a_ah,  ':',  'Color', COL_AUX, 'LineWidth', 2);
set(gca, 'FontSize', 18, 'FontWeight', 'bold', 'LineWidth', 2);
xlabel('Time [sec]', 'FontSize', 20);
ylabel('$R_{22}^{(a)}$ [(um/pN)$^2$]', 'FontSize', 20);
title('(2) $a_{xm}$-space:  empirical vs theory', 'FontSize', 18);
legend({'emp $\mathrm{Var}(a_{xm})$', ...
        'theory ($a_z$ true)', ...
        'theory ($\hat{a}$)'}, ...
       'FontSize', 14, 'Location', 'best');
grid on; box on; xlim([0 T_sim]);

% ------ (3) Scaling law: emp vs theory scatter (log-log) ------
nexttile;
loglog(seg_R22_th_sig2, seg_R22_emp_sig2, 'o', 'Color', COL_EMP_SIG, ...
       'MarkerFaceColor', COL_EMP_SIG, 'MarkerSize', 10); hold on;
lo = min([seg_R22_th_sig2; seg_R22_emp_sig2]) * 0.7;
hi = max([seg_R22_th_sig2; seg_R22_emp_sig2]) * 1.3;
loglog([lo hi], [lo hi], 'k--', 'LineWidth', 2);
loglog([lo hi], [lo hi]*1.2, 'k:', 'LineWidth', 1);
loglog([lo hi], [lo hi]/1.2, 'k:', 'LineWidth', 1);
set(gca, 'FontSize', 18, 'FontWeight', 'bold', 'LineWidth', 2);
xlabel('$R_{22}$ theory [um$^4$]', 'FontSize', 20);
ylabel('$R_{22}$ empirical [um$^4$]', 'FontSize', 20);
title('(3) Scaling law (log-log, $\pm$20\% band)', 'FontSize', 18);
legend({'segments', '$y=x$', '$\pm 20\%$'}, 'FontSize', 14, 'Location', 'best');
grid on; box on; axis equal; xlim([lo hi]); ylim([lo hi]);

% ------ (4) Ratio over time ------
nexttile;
plot(seg_t_center, seg_R22_emp_sig2 ./ seg_R22_th_sig2, 's-', ...
     'Color', COL_EMP_SIG, 'LineWidth', 2, 'MarkerFaceColor', COL_EMP_SIG, ...
     'MarkerSize', 8); hold on;
plot(seg_t_center, seg_R22_emp_a ./ seg_R22_th_a_az, 'd-', ...
     'Color', COL_EMP_A, 'LineWidth', 2, 'MarkerFaceColor', COL_EMP_A, ...
     'MarkerSize', 8);
yline(1.0,  'k--', 'LineWidth', 2);
yline(1.2,  'k:',  'LineWidth', 1);
yline(0.8,  'k:',  'LineWidth', 1);
set(gca, 'FontSize', 18, 'FontWeight', 'bold', 'LineWidth', 2);
xlabel('Time [sec]', 'FontSize', 20);
ylabel('$R_{22}^{\rm emp} / R_{22}^{\rm theory}$', 'FontSize', 20);
title('(4) Ratio over ramp  (target $=1$, $\pm 20\%$ band)', 'FontSize', 18);
legend({'$\sigma^2$-space', '$a_{xm}$-space (vs $a_z$)'}, ...
       'FontSize', 14, 'Location', 'best');
grid on; box on; xlim([0 T_sim]); ylim([0 2]);

sgtitle('R22 intrinsic verification (ramp 50-5 um, T=20 s, lc=0.7, a_{cov}=0.05)', ...
        'FontSize', 18, 'FontWeight', 'bold', 'Interpreter', 'tex');

save_dir = fullfile(project_root, 'test_results', 'learn_variance');
out_path = fullfile(save_dir, 'verify_R22_intrinsic.png');
exportgraphics(fig, out_path, 'Resolution', 150);
fprintf('\nSaved: %s\n', out_path);

%% Sanity figure: sigma2_ctrl(t) + theory mean ± sqrt(R22) band
fig2 = figure('Position', [80 80 1500 700], 'Color', 'w');

t_plot   = tout;
mu_plot  = sigma2_dxr_theory_t;
sd_plot  = sqrt(R22_theory_t_sig2);

% ±1σ band
fill([t_plot; flipud(t_plot)], ...
     [mu_plot + sd_plot; flipud(mu_plot - sd_plot)], ...
     COL_TH, 'FaceAlpha', 0.20, 'EdgeColor', 'none', ...
     'DisplayName', 'theory mean $\pm \sqrt{R_{22}}$');
hold on;
p_ctrl = plot(t_plot, sigma2_ctrl_z, '-', 'Color', COL_EMP_SIG, 'LineWidth', 1.0, ...
              'DisplayName', '$\widehat{\sigma^2_{\delta x_r}}$ controller');
p_ctrl.Color(4) = 0.55;
plot(t_plot, mu_plot, '-', 'Color', COL_TH, 'LineWidth', 3, ...
     'DisplayName', 'theory mean');

set(gca, 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2);
xlabel('Time [sec]', 'FontSize', 24);
ylabel('$\sigma^2$ [um$^2$]', 'FontSize', 24);
title('Controller $\widehat{\sigma^2_{\delta x_r}}$ vs theory mean $\pm$ 1$\sigma$ band', ...
      'FontSize', 20);
legend('Location', 'northoutside', 'Orientation', 'horizontal', ...
       'NumColumns', 3, 'FontSize', 16);
grid on; box on; xlim([0 T_sim]);

% report 68% containment
in_band = (sigma2_ctrl_z >= mu_plot - sd_plot) & (sigma2_ctrl_z <= mu_plot + sd_plot);
in_2sig = (sigma2_ctrl_z >= mu_plot - 2*sd_plot) & (sigma2_ctrl_z <= mu_plot + 2*sd_plot);
fprintf('\n=== Containment within theory mean ± k·sqrt(R22) ===\n');
fprintf('  ±1·sqrt(R22) : %.1f%% of samples (target ~68%% if Gaussian)\n', 100*mean(in_band));
fprintf('  ±2·sqrt(R22) : %.1f%% of samples (target ~95%% if Gaussian)\n', 100*mean(in_2sig));

out_path2 = fullfile(save_dir, 'verify_R22_band.png');
exportgraphics(fig2, out_path2, 'Resolution', 150);
fprintf('\nSaved: %s\n', out_path2);

set(groot, 'defaultTextInterpreter','remove');
set(groot, 'defaultLegendInterpreter','remove');
set(groot, 'defaultAxesTickLabelInterpreter','remove');
