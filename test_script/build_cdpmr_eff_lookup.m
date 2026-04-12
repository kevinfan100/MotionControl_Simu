%% build_cdpmr_eff_lookup.m — Build 2-D lookup table for C_dpmr_eff and C_np_eff
%
% Sweeps (lc, rho) grid at fixed a_pd = 0.05, computes C_dpmr_eff and C_np_eff
% via augmented Lyapunov for 7-state EKF.
%
% Output: test_results/verify/cdpmr_eff_lookup.mat
%         reference/for_test/fig_cdpmr_eff_lookup.png

clear; close all; clc;

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(script_dir);
addpath(script_dir);
addpath(fullfile(project_root, 'model'));
addpath(fullfile(project_root, 'model', 'config'));

fprintf('===== build_cdpmr_eff_lookup =====\n\n');

%% Grid definition
lc_grid = [0.4, 0.5, 0.6, 0.7, 0.8, 0.9];              % 6 values
rho_grid = logspace(-3, 2, 21);                         % 21 values from 1e-3 to 1e2
a_pd = 0.05;

Q_kf_scale = [0; 0; 1e4; 1e-1; 0; 1e-4; 0];
R_kf_scale = [1e-2; 1e0];

n_lc = length(lc_grid);
n_rho = length(rho_grid);
total = n_lc * n_rho;

Cdpmr_tab = nan(n_lc, n_rho);
Cnp_tab   = nan(n_lc, n_rho);
max_eig_tab = nan(n_lc, n_rho);
dare_iters_tab = nan(n_lc, n_rho);
dare_err_tab = nan(n_lc, n_rho);

%% Loop
fprintf('Grid: %d lc x %d rho = %d points\n', n_lc, n_rho, total);
fprintf('a_pd = %.4f, Q_kf_scale = [%s], R_kf_scale = [%s]\n\n', ...
        a_pd, sprintf('%.1e ', Q_kf_scale), sprintf('%.1e ', R_kf_scale));

t_start = tic;
for i = 1:n_lc
    for j = 1:n_rho
        lc = lc_grid(i);
        rho = rho_grid(j);
        try
            [C_dpmr_eff, C_np_eff, L, A_aug, diag_out] = ...
                compute_7state_cdpmr_eff(lc, rho, a_pd, Q_kf_scale, R_kf_scale);
            Cdpmr_tab(i, j) = C_dpmr_eff;
            Cnp_tab(i, j)   = C_np_eff;
            max_eig_tab(i, j) = diag_out.max_eig_A_aug;
            dare_iters_tab(i, j) = diag_out.dare_iters;
            dare_err_tab(i, j) = diag_out.dare_err;
        catch ME
            fprintf('  ERROR at lc=%.2f rho=%.3g: %s\n', lc, rho, ME.message);
            Cdpmr_tab(i, j) = NaN;
            Cnp_tab(i, j)   = NaN;
        end
    end
    fprintf('  row %d/%d (lc=%.2f) done\n', i, n_lc, lc_grid(i));
end
t_total = toc(t_start);
fprintf('\nTotal time: %.1f sec\n', t_total);

%% Summary
fprintf('\nC_dpmr_eff table (rows = lc, cols = rho_grid index):\n');
fprintf('      rho: '); fprintf('%9.2e ', rho_grid); fprintf('\n');
for i = 1:n_lc
    fprintf('  lc=%.2f: ', lc_grid(i));
    fprintf('%9.4f ', Cdpmr_tab(i, :));
    fprintf('\n');
end

fprintf('\nC_np_eff table:\n');
fprintf('      rho: '); fprintf('%9.2e ', rho_grid); fprintf('\n');
for i = 1:n_lc
    fprintf('  lc=%.2f: ', lc_grid(i));
    fprintf('%9.4f ', Cnp_tab(i, :));
    fprintf('\n');
end

%% Reference point check
[~, rho_ref_idx] = min(abs(rho_grid - 0.05));
[~, lc_ref_idx] = min(abs(lc_grid - 0.7));
fprintf('\nReference point:\n');
fprintf('  lc=%.2f rho=%.3f  ->  C_dpmr_eff=%.4f, C_np_eff=%.4f\n', ...
        lc_grid(lc_ref_idx), rho_grid(rho_ref_idx), ...
        Cdpmr_tab(lc_ref_idx, rho_ref_idx), Cnp_tab(lc_ref_idx, rho_ref_idx));
fprintf('  K=2 baseline (lc=0.7, a_pd=0.05): 3.161\n');
fprintf('  Expected in wide band: [3.5, 6.0]\n');

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
lookup.rho_grid = rho_grid;
lookup.a_pd = a_pd;
lookup.Cdpmr_tab = Cdpmr_tab;
lookup.Cnp_tab = Cnp_tab;
lookup.Q_kf_scale = Q_kf_scale;
lookup.R_kf_scale = R_kf_scale;
lookup.f0 = 0;
lookup.max_eig_tab = max_eig_tab;
lookup.dare_iters_tab = dare_iters_tab;
lookup.dare_err_tab = dare_err_tab;
lookup.build_timestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');
lookup.note = ['Augmented Lyapunov lookup for 7-state EKF + IIR HP filter. ', ...
               'Phase 1 steady-state f_0 = 0 (free space). Only valid at a_pd=0.05.'];

save(fullfile(out_dir, 'cdpmr_eff_lookup.mat'), '-struct', 'lookup');
fprintf('\nSaved: %s\n', fullfile(out_dir, 'cdpmr_eff_lookup.mat'));

%% Figure: heatmaps
fig_dir = fullfile(project_root, 'reference', 'for_test');
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end

fig = figure('Position', [100, 100, 1400, 600]);

subplot(1, 2, 1);
imagesc(log10(rho_grid), lc_grid, Cdpmr_tab);
set(gca, 'YDir', 'normal');
colorbar;
xlabel('log_{10}(rho)');
ylabel('lc');
title('C_{dpmr,eff}(lc, rho)');
set(gca, 'FontSize', 14, 'LineWidth', 1.5);

subplot(1, 2, 2);
imagesc(log10(rho_grid), lc_grid, Cnp_tab);
set(gca, 'YDir', 'normal');
colorbar;
xlabel('log_{10}(rho)');
ylabel('lc');
title('C_{np,eff}(lc, rho)');
set(gca, 'FontSize', 14, 'LineWidth', 1.5);

saveas(fig, fullfile(fig_dir, 'fig_cdpmr_eff_lookup.png'));
fprintf('Figure saved: fig_cdpmr_eff_lookup.png\n');

%% Gate: reference point in expected range
ref_val = Cdpmr_tab(lc_ref_idx, rho_ref_idx);
if ref_val >= 3.5 && ref_val <= 6.0
    fprintf('\nGATE: PASS (C_dpmr_eff[0.7, 0.05] = %.4f in [3.5, 6.0])\n', ref_val);
else
    fprintf('\nGATE: FAIL (C_dpmr_eff[0.7, 0.05] = %.4f NOT in [3.5, 6.0])\n', ref_val);
    error('build_cdpmr_eff_lookup:reference_fail', ...
          'Reference point out of expected band');
end

fprintf('\nDone.\n');
