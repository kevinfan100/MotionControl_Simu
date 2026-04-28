%% build_cdpmr_eff_lookup.m — Build 2-D lookup table for C_dpmr_eff and C_np_eff
%
% Sweeps (lc, a/a_nom) grid at fixed a_pd = 0.05, computes C_dpmr_eff and
% C_np_eff via augmented Lyapunov for 7-state EKF.
%
% a/a_nom (= 1/c(h_bar)) is the wall-proximity indicator: 1.0 in free space,
% ~0.1 near wall (z-axis). At each grid point, Q and R are set self-consistently:
%   Q(3,3) = 4*k_B*T*a = sigma2_dXT * aratio     (thermal process noise)
%   R(1,1) = sigma2_n                              (sensor noise, fixed)
%   R(2,2) = chi_sq_R * aratio^2 * sigma2_dXT      (a_m estimator variance)
%
% This ensures the DARE solution matches the physical noise levels at each
% wall proximity, and C_dpmr_eff adapts consistently.
%
% Output: test_results/verify/cdpmr_eff_lookup.mat
%         reference/qr_analysis/fig_cdpmr_eff_lookup.png

clear; close all; clc;

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(script_dir);
addpath(script_dir);
addpath(fullfile(project_root, 'model'));
addpath(fullfile(project_root, 'model', 'config'));

fprintf('===== build_cdpmr_eff_lookup (2-D lc x a/a_nom) =====\n\n');

%% Grid definition
lc_grid = [0.4, 0.5, 0.6, 0.7, 0.8, 0.9];                   % 6 values
aratio_grid = [0.03, 0.05, 0.1, 0.2, 0.3, 0.5, 0.7, 1.0];   % 8 values (a/a_nom)
a_pd = 0.05;

% Q(6,6) and Q(7,7) from trajectory derivation (beta: both = Var(d2a)/sigma2_dXT).
% Loaded from q77_trajectory.mat produced by compute_q77_from_trajectory.
% If not found, falls back to 0 (reverts to pre-derivation behavior).
q77_path = fullfile(project_root, 'test_results', 'verify', 'q77_trajectory.mat');
if exist(q77_path, 'file')
    q77_data = load(q77_path, 'Q77_scaling');
    Q66_fixed = q77_data.Q77_scaling;   % beta: Q66 = Q77
    Q77_fixed = q77_data.Q77_scaling;
    fprintf('Loaded Q77_scaling from q77_trajectory.mat: %.4e (deployed to slots 6 & 7)\n', ...
            Q77_fixed);
else
    Q66_fixed = 0;
    Q77_fixed = 0;
    fprintf('q77_trajectory.mat not found; using Q66=Q77=0 (pre-derivation default)\n');
end

% Physical constants for self-consistent Q/R
phys = physical_constants();
a_nom = phys.Ts / phys.gamma_N;
sigma2_dXT = 4 * phys.k_B * phys.T * a_nom;
sigma2_n = 0.01^2;   % sensor noise variance [um^2], noise ON

% chi-squared factor for R(2,2) = chi_sq_R * aratio^2 * sigma2_dXT
% chi_sq_R = (2*a_cov/(2-a_cov)) * autocorr_amp * a_nom / (4*k_B*T)
a_cov = 0.05;
white_chi_sq = 2 * a_cov / (2 - a_cov);   % = 0.0513
autocorr_amp = 4.0;                         % from Task 1d Layer 2
chi_sq_R = white_chi_sq * autocorr_amp * a_nom / (4 * phys.k_B * phys.T);
% chi_sq_R ≈ 0.176 — at aratio=1, R(2,2)_scaling = 0.176

rho_dummy = 0.05;

n_lc = length(lc_grid);
n_ar = length(aratio_grid);
total = n_lc * n_ar;

Cdpmr_tab = nan(n_lc, n_ar);
Cnp_tab   = nan(n_lc, n_ar);
max_eig_tab = nan(n_lc, n_ar);
dare_iters_tab = nan(n_lc, n_ar);
dare_err_tab = nan(n_lc, n_ar);

%% Loop
fprintf('Grid: %d lc x %d a/a_nom = %d points\n', n_lc, n_ar, total);
fprintf('a_pd = %.4f, chi_sq_R = %.4f, sigma2_n = %.1e\n', a_pd, chi_sq_R, sigma2_n);
fprintf('R(1,1)_scaling = %.4f (fixed)\n\n', sigma2_n / sigma2_dXT);

t_start = tic;
for i = 1:n_lc
    for j = 1:n_ar
        lc = lc_grid(i);
        ar = aratio_grid(j);

        % Self-consistent Q/R at this wall proximity
        % Q(6,6), Q(7,7) from beta-derivation trajectory; Q(3,3) scales with aratio
        Q_kf_scale = [0; 0; ar; 0; 0; Q66_fixed; Q77_fixed];
        R_kf_scale = [sigma2_n / sigma2_dXT; chi_sq_R * ar^2];

        opts = struct('f0', 0);
        try
            [C_dpmr_eff, C_np_eff, L, A_aug, diag_out] = ...
                compute_7state_cdpmr_eff(lc, rho_dummy, a_pd, ...
                                          Q_kf_scale, R_kf_scale, opts);
            Cdpmr_tab(i, j) = C_dpmr_eff;
            Cnp_tab(i, j)   = C_np_eff;
            max_eig_tab(i, j) = diag_out.max_eig_A_aug;
            dare_iters_tab(i, j) = diag_out.dare_iters;
            dare_err_tab(i, j) = diag_out.dare_err;
        catch ME
            fprintf('  ERROR at lc=%.2f ar=%.3f: %s\n', lc, ar, ME.message);
            Cdpmr_tab(i, j) = NaN;
            Cnp_tab(i, j)   = NaN;
        end
    end
    fprintf('  row %d/%d (lc=%.2f) done  [%.1f sec elapsed]\n', ...
            i, n_lc, lc_grid(i), toc(t_start));
end
t_total = toc(t_start);
fprintf('\nTotal time: %.1f sec\n', t_total);

%% Summary
fprintf('\nC_dpmr_eff table (rows = lc, cols = a/a_nom):\n');
fprintf('  a/a_nom: '); fprintf('%9.3f ', aratio_grid); fprintf('\n');
for i = 1:n_lc
    fprintf('  lc=%.2f: ', lc_grid(i));
    fprintf('%9.4f ', Cdpmr_tab(i, :));
    fprintf('\n');
end

fprintf('\nC_np_eff table:\n');
fprintf('  a/a_nom: '); fprintf('%9.3f ', aratio_grid); fprintf('\n');
for i = 1:n_lc
    fprintf('  lc=%.2f: ', lc_grid(i));
    fprintf('%9.4f ', Cnp_tab(i, :));
    fprintf('\n');
end

%% Reference point check (free-space: a/a_nom = 1)
[~, ar_ref_idx] = min(abs(aratio_grid - 1.0));
[~, lc_ref_idx] = min(abs(lc_grid - 0.7));
fprintf('\nReference point (free-space, a/a_nom=1):\n');
fprintf('  lc=%.2f ar=%.3f  ->  C_dpmr_eff=%.4f, C_np_eff=%.4f\n', ...
        lc_grid(lc_ref_idx), aratio_grid(ar_ref_idx), ...
        Cdpmr_tab(lc_ref_idx, ar_ref_idx), Cnp_tab(lc_ref_idx, ar_ref_idx));

%% Monotonicity check: C_dpmr_eff should increase as a/a_nom decreases (near wall)
fprintf('\nMonotonicity check (lc=0.70 slice, expect increasing as ar decreases):\n');
slice_ref = Cdpmr_tab(lc_ref_idx, :);
for j = 1:n_ar
    fprintf('  ar=%5.3f: C_dpmr_eff=%.4f', aratio_grid(j), slice_ref(j));
    if j > 1
        d = slice_ref(j) - slice_ref(j-1);
        fprintf('   (delta: %+.4f)', d);
    end
    fprintf('\n');
end
% C_dpmr should DECREASE as ar increases (decreasing toward wall normal)
mono_ok = all(diff(slice_ref) <= 1e-3);
if mono_ok
    fprintf('  MONOTONIC (decreasing with ar): PASS\n');
else
    fprintf('  MONOTONIC: WARN — not strictly decreasing\n');
end

%% Sanity checks for lookup table
finite_count = sum(~isnan(Cdpmr_tab(:)));
fprintf('\nLookup table sanity:\n');
fprintf('  Finite points: %d / %d\n', finite_count, total);
fprintf('  Cdpmr_tab min=%.4f max=%.4f\n', min(Cdpmr_tab(:)), max(Cdpmr_tab(:)));
fprintf('  Cnp_tab   min=%.4f max=%.4f\n', min(Cnp_tab(:)), max(Cnp_tab(:)));
fprintf('  Max eigenvalue across grid: %.6f\n', max(max_eig_tab(:)));

%% Save lookup
out_dir = fullfile(project_root, 'test_results', 'verify');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

lookup = struct();
lookup.lc_grid = lc_grid;
lookup.aratio_grid = aratio_grid;
lookup.a_pd = a_pd;
lookup.chi_sq_R = chi_sq_R;
lookup.sigma2_n = sigma2_n;
lookup.Cdpmr_tab = Cdpmr_tab;    % [n_lc x n_ar]
lookup.Cnp_tab = Cnp_tab;        % [n_lc x n_ar]
lookup.max_eig_tab = max_eig_tab;
lookup.dare_iters_tab = dare_iters_tab;
lookup.dare_err_tab = dare_err_tab;
lookup.Q66_fixed = Q66_fixed;
lookup.Q77_fixed = Q77_fixed;
lookup.build_timestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');
lookup.note = ['2-D augmented Lyapunov lookup for 7-state EKF + IIR HP filter. ', ...
               '(lc, a/a_nom) grid with self-consistent Q/R at each point: ', ...
               'Q(3,3)=ar*sigma2_dXT, R(2,2)=chi_sq*ar^2. Q(6,6)=Q(7,7) loaded ', ...
               'from q77_trajectory.mat (beta: Var(d2a)/sigma2_dXT).'];

save(fullfile(out_dir, 'cdpmr_eff_lookup.mat'), '-struct', 'lookup');
fprintf('\nSaved: %s\n', fullfile(out_dir, 'cdpmr_eff_lookup.mat'));

%% Figure: 2D heatmaps
fig_dir = fullfile(project_root, 'reference', 'qr_analysis');
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end

fig = figure('Position', [100, 100, 1400, 600]);

subplot(1, 2, 1);
imagesc(aratio_grid, lc_grid, Cdpmr_tab);
set(gca, 'YDir', 'normal');
colorbar;
xlabel('a / a_{nom}');
ylabel('\lambda_c');
set(gca, 'FontSize', 14, 'LineWidth', 1.5);

subplot(1, 2, 2);
imagesc(aratio_grid, lc_grid, Cnp_tab);
set(gca, 'YDir', 'normal');
colorbar;
xlabel('a / a_{nom}');
ylabel('\lambda_c');
set(gca, 'FontSize', 14, 'LineWidth', 1.5);

saveas(fig, fullfile(fig_dir, 'fig_cdpmr_eff_lookup.png'));
fprintf('Figure saved: fig_cdpmr_eff_lookup.png\n');

%% Gate: free-space reference
ref_val = Cdpmr_tab(lc_ref_idx, ar_ref_idx);
if ref_val >= 3.95 && ref_val <= 4.10
    fprintf('\nGATE G1 (free-space ref): PASS (C_dpmr_eff[0.7, 1.0] = %.4f in [3.95, 4.10])\n', ref_val);
else
    fprintf('\nGATE G1: FAIL (C_dpmr_eff[0.7, 1.0] = %.4f NOT in [3.95, 4.10])\n', ref_val);
    error('build_cdpmr_eff_lookup:reference_fail', ...
          'Free-space reference out of expected band');
end

if mono_ok
    fprintf('GATE G2 (monotonicity): PASS\n');
else
    fprintf('GATE G2 (monotonicity): WARN\n');
end

fprintf('\nDone.\n');
