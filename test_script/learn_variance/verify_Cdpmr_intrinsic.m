% verify_Cdpmr_intrinsic.m
% Quantitative segment-by-segment verification of the C_dpmr mean formula.
%
%   E[sigma2_dxr_hat]_theory = C_dpmr · 4kBT · a_z + C_n · sigma2_n
%
% Per-segment:
%   - empirical mean(sigma2_dxr_hat) within window
%   - theory using a_z_true at segment midpoint
%   - ratio emp/theory  (target = 1.00)
%
% Parallels verify_R22_intrinsic.m structure. Reads compare_am.mat (run
% compare_am.m first).

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
C_dpmr          = S.C_dpmr;          % already includes (1-a_pd)^2 prefactor
C_n             = S.C_n;
a_pd            = S.a_pd;
a_cov           = S.a_cov;
lambda_c        = S.lambda_c;

constants  = physical_constants();
kBT        = constants.k_B * constants.T;
if isfield(S, 'sigma2_n_s')
    sigma2_n_z = S.sigma2_n_s(3);          % use ACTUAL sim noise
else
    error(['compare_am.mat is stale (missing sigma2_n_s). ' ...
           'Re-run compare_am.m to refresh.']);
end

N     = length(tout);
T_sim = tout(end);

fprintf('=== Constants ===\n');
fprintf('  lambda_c = %.3f, a_pd = %.3f, a_cov = %.3f\n', lambda_c, a_pd, a_cov);
fprintf('  C_dpmr   = %.4f  (includes (1-a_pd)^2 = %.4f)\n', C_dpmr, (1-a_pd)^2);
fprintf('  C_n      = %.4f\n', C_n);
fprintf('  4kBT     = %.4e [pN*um]\n', 4*kBT);
fprintf('  sigma2_n_z = %.4e [um^2]\n\n', sigma2_n_z);

%% Theory mean trajectory (time-resolved)
sigma2_dxr_theory_t = C_dpmr * 4 * kBT * a_z_true + C_n * sigma2_n_z;

%% Per-segment empirical vs theory mean
n_seg = 10;
seg_edges = linspace(0, T_sim, n_seg+1);

seg_t_center      = zeros(n_seg, 1);
seg_h_proxy       = zeros(n_seg, 1);  % a_z_mid as proxy for h
seg_a_z_mid       = zeros(n_seg, 1);
seg_mean_emp      = zeros(n_seg, 1);    % mean(sigma2_ctrl_z within segment)
seg_mean_th       = zeros(n_seg, 1);    % C_dpmr·4kBT·a_z_mid + C_n·σ²_n
seg_mean_th_int   = zeros(n_seg, 1);    % integrate theory_t within segment (better for ramp)
seg_std_emp       = zeros(n_seg, 1);    % std(sigma2_ctrl_z within segment) — for SE
seg_N             = zeros(n_seg, 1);
seg_N_eff         = zeros(n_seg, 1);    % effective sample count (EWMA-corrected)
seg_SE_mean       = zeros(n_seg, 1);

for s = 1:n_seg
    if s < n_seg
        idx = (tout >= seg_edges(s)) & (tout < seg_edges(s+1));
    else
        idx = (tout >= seg_edges(s)) & (tout <= seg_edges(s+1));
    end

    seg_t_center(s)    = mean(tout(idx));
    seg_a_z_mid(s)     = mean(a_z_true(idx));
    seg_mean_emp(s)    = mean(sigma2_ctrl_z(idx));
    seg_std_emp(s)     = std(sigma2_ctrl_z(idx));
    seg_N(s)           = sum(idx);
    % EWMA correlation time ~ 1/a_cov samples → effective independent samples
    seg_N_eff(s)       = seg_N(s) * a_cov / (2 - a_cov);
    seg_SE_mean(s)     = seg_std_emp(s) / sqrt(seg_N_eff(s));

    % Two theory formulations:
    %   (1) Point-evaluated at segment midpoint
    seg_mean_th(s)     = C_dpmr * 4 * kBT * seg_a_z_mid(s) + C_n * sigma2_n_z;
    %   (2) Trajectory-averaged within window (handles ramp curvature)
    seg_mean_th_int(s) = mean(sigma2_dxr_theory_t(idx));
end

ratio_pt  = seg_mean_emp ./ seg_mean_th;
ratio_int = seg_mean_emp ./ seg_mean_th_int;

%% Print table
fprintf('=== Per-segment C_dpmr verification (sigma^2 mean, units [um^2]) ===\n');
fprintf(' seg | t_ctr | a_z_mid    | mean_emp     mean_th(pt) mean_th(int) | r_pt   r_int   | SE/emp(%%)\n');
fprintf('-----|-------|------------|--------------|------------|-------------|--------|--------|----------\n');
for s = 1:n_seg
    fprintf(' %2d  | %5.2fs| %.4e | %.4e   %.4e   %.4e    | %.4f  %.4f  |  %5.2f%%\n', ...
            s, seg_t_center(s), seg_a_z_mid(s), ...
            seg_mean_emp(s), seg_mean_th(s), seg_mean_th_int(s), ...
            ratio_pt(s), ratio_int(s), 100*seg_SE_mean(s)/seg_mean_emp(s));
end

fprintf('\n=== Summary ratios (target = 1.000) ===\n');
fprintf('  mean(r_pt)  = %.4f, std = %.4f, max|deviation| = %.4f\n', ...
        mean(ratio_pt), std(ratio_pt), max(abs(ratio_pt-1)));
fprintf('  mean(r_int) = %.4f, std = %.4f, max|deviation| = %.4f\n', ...
        mean(ratio_int), std(ratio_int), max(abs(ratio_int-1)));

fprintf('\n=== Global mean check ===\n');
fprintf('  Global mean(sigma2_ctrl_z)              = %.4e\n', mean(sigma2_ctrl_z));
fprintf('  Global mean(theory trajectory)          = %.4e\n', mean(sigma2_dxr_theory_t));
fprintf('  Global ratio emp/theory                 = %.4f\n', mean(sigma2_ctrl_z)/mean(sigma2_dxr_theory_t));

%% Plot
set(groot, 'defaultTextInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex');

COL_EMP    = [0.85 0.00 0.00];   % red
COL_TH     = [0.00 0.55 0.00];   % green
COL_RESID  = [0.40 0.20 0.70];   % purple
COL_AUX    = [0.45 0.45 0.45];   % gray

fig = figure('Position', [50 50 1600 1000], 'Color', 'w');
tl  = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% ---- (1) per-segment mean: emp vs theory ----
nexttile;
plot(seg_t_center, seg_mean_emp, 'o-', 'Color', COL_EMP, 'LineWidth', 2.2, ...
     'MarkerFaceColor', COL_EMP, 'MarkerSize', 8, ...
     'DisplayName', 'emp mean($\widehat{\sigma^2_{\delta p_{mr}}}$)'); hold on;
plot(seg_t_center, seg_mean_th, 's--', 'Color', COL_TH, 'LineWidth', 2.5, ...
     'MarkerFaceColor', COL_TH, 'MarkerSize', 8, ...
     'DisplayName', 'theory (pt: $a_z$@mid)');
plot(seg_t_center, seg_mean_th_int, 'd:', 'Color', COL_AUX, 'LineWidth', 2, ...
     'MarkerSize', 7, 'DisplayName', 'theory (int: $\overline{\sigma^2_{th}(t)}$)');
% error bars on emp
errorbar(seg_t_center, seg_mean_emp, seg_SE_mean, '.', 'Color', COL_EMP, ...
         'LineWidth', 1.5, 'HandleVisibility', 'off');
set(gca, 'FontSize', 18, 'FontWeight', 'bold', 'LineWidth', 2);
xlabel('Time [sec]', 'FontSize', 20);
ylabel('$\overline{\sigma^2}$ [um$^2$]', 'FontSize', 20);
title('(1) Segment mean: empirical vs theory', 'FontSize', 18);
legend('FontSize', 13, 'Location', 'best');
grid on; box on; xlim([0 T_sim]);

% ---- (2) ratio over time ----
nexttile;
plot(seg_t_center, ratio_pt,  's-', 'Color', COL_EMP, 'LineWidth', 2, ...
     'MarkerFaceColor', COL_EMP, 'MarkerSize', 9, ...
     'DisplayName', 'ratio (pt theory)'); hold on;
plot(seg_t_center, ratio_int, 'd-', 'Color', COL_RESID, 'LineWidth', 2, ...
     'MarkerFaceColor', COL_RESID, 'MarkerSize', 8, ...
     'DisplayName', 'ratio (int theory)');
yline(1.0,  'k--', 'LineWidth', 2);
yline(1.05, 'k:',  'LineWidth', 1);
yline(0.95, 'k:',  'LineWidth', 1);
set(gca, 'FontSize', 18, 'FontWeight', 'bold', 'LineWidth', 2);
xlabel('Time [sec]', 'FontSize', 20);
ylabel('emp / theory', 'FontSize', 20);
title('(2) Ratio over ramp  (target $=1$, $\pm 5\%$ band)', 'FontSize', 18);
legend('FontSize', 13, 'Location', 'best');
grid on; box on; xlim([0 T_sim]); ylim([0.85 1.15]);

% ---- (3) scatter emp vs theory (linear, 45-deg ref) ----
nexttile;
plot(seg_mean_th, seg_mean_emp, 'o', 'Color', COL_EMP, ...
     'MarkerFaceColor', COL_EMP, 'MarkerSize', 12); hold on;
lo = 0.9 * min([seg_mean_emp; seg_mean_th]);
hi = 1.1 * max([seg_mean_emp; seg_mean_th]);
plot([lo hi], [lo hi], 'k--', 'LineWidth', 2);
plot([lo hi], [lo hi]*1.05, 'k:', 'LineWidth', 1);
plot([lo hi], [lo hi]/1.05, 'k:', 'LineWidth', 1);
set(gca, 'FontSize', 18, 'FontWeight', 'bold', 'LineWidth', 2);
xlabel('mean theory [um$^2$]', 'FontSize', 20);
ylabel('mean empirical [um$^2$]', 'FontSize', 20);
title('(3) Scatter (emp vs theory, $\pm 5\%$ band)', 'FontSize', 18);
grid on; box on; axis equal; xlim([lo hi]); ylim([lo hi]);

% ---- (4) trace overlay: full IIR vs theory ----
nexttile;
p_iir = plot(tout, sigma2_ctrl_z, '-', 'Color', COL_EMP, 'LineWidth', 0.8, ...
             'DisplayName', '$\widehat{\sigma^2}$ controller (IIR)'); hold on;
p_iir.Color(4) = 0.40;
plot(tout, sigma2_dxr_theory_t, '-', 'Color', COL_TH, 'LineWidth', 3, ...
     'DisplayName', 'theory mean $\sigma^2_{th}(t)$');
plot(seg_t_center, seg_mean_emp, 'ko', 'MarkerFaceColor', 'k', ...
     'MarkerSize', 9, 'DisplayName', 'segment emp mean');
set(gca, 'FontSize', 18, 'FontWeight', 'bold', 'LineWidth', 2);
xlabel('Time [sec]', 'FontSize', 20);
ylabel('$\sigma^2$ [um$^2$]', 'FontSize', 20);
title('(4) Full IIR trace vs theory mean', 'FontSize', 18);
legend('FontSize', 13, 'Location', 'northeast');
grid on; box on; xlim([0 T_sim]);

sgtitle('C_{dpmr} intrinsic verification (ramp 50-5 um, T=20 s, lc=0.7, a_{pd}=0.05)', ...
        'FontSize', 18, 'FontWeight', 'bold', 'Interpreter', 'tex');

save_dir = fullfile(project_root, 'test_results', 'learn_variance');
out_path = fullfile(save_dir, 'verify_Cdpmr_intrinsic.png');
exportgraphics(fig, out_path, 'Resolution', 150);
fprintf('\nSaved: %s\n', out_path);

set(groot, 'defaultTextInterpreter','remove');
set(groot, 'defaultLegendInterpreter','remove');
set(groot, 'defaultAxesTickLabelInterpreter','remove');
