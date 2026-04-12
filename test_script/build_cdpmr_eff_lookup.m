%% build_cdpmr_eff_lookup.m — Build 2-D lookup table for C_dpmr_eff and C_np_eff
%
% Sweeps (lc, f_0) grid at fixed a_pd = 0.05, computes C_dpmr_eff and C_np_eff
% via augmented Lyapunov for 7-state EKF.
%
% f_0 is the linearization point for Fe(3,6) = -f_dx[k], i.e., the control
% force magnitude. At f_0 = 0, lookup matches original Phase 1 (free-space).
% At f_0 > 0, lookup captures how closed-loop dynamics change with |f_d|,
% addressing the ~8% persistent bias observed in Phase 2.
%
% Output: test_results/verify/cdpmr_eff_lookup.mat
%         reference/for_test/fig_cdpmr_eff_lookup.png

clear; close all; clc;

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(script_dir);
addpath(script_dir);
addpath(fullfile(project_root, 'model'));
addpath(fullfile(project_root, 'model', 'config'));

fprintf('===== build_cdpmr_eff_lookup (2-D lc x f_0) =====\n\n');

%% Grid definition
lc_grid = [0.4, 0.5, 0.6, 0.7, 0.8, 0.9];        % 6 values
f0_grid = [0, 0.5, 1, 2, 5, 10, 20];              % 7 values [pN]
a_pd = 0.05;

% rho for compute_7state_cdpmr_eff is a cosmetic parameter (not used in
% the computation, stored only in diagnostics). Use any value.
rho_dummy = 0.05;

Q_kf_scale = [0; 0; 1e4; 1e-1; 0; 1e-4; 0];
R_kf_scale = [1e-2; 1e0];

n_lc = length(lc_grid);
n_f0 = length(f0_grid);
total = n_lc * n_f0;

Cdpmr_tab = nan(n_lc, n_f0);
Cnp_tab   = nan(n_lc, n_f0);
max_eig_tab = nan(n_lc, n_f0);
dare_iters_tab = nan(n_lc, n_f0);
dare_err_tab = nan(n_lc, n_f0);

%% Loop
fprintf('Grid: %d lc x %d f_0 = %d points\n', n_lc, n_f0, total);
fprintf('a_pd = %.4f, Q_kf_scale = [%s], R_kf_scale = [%s]\n\n', ...
        a_pd, sprintf('%.1e ', Q_kf_scale), sprintf('%.1e ', R_kf_scale));

t_start = tic;
for i = 1:n_lc
    for j = 1:n_f0
        lc = lc_grid(i);
        f0 = f0_grid(j);
        opts = struct('f0', f0);
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
            fprintf('  ERROR at lc=%.2f f_0=%.2f: %s\n', lc, f0, ME.message);
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
fprintf('\nC_dpmr_eff table (rows = lc, cols = f_0 [pN]):\n');
fprintf('      f_0: '); fprintf('%9.2f ', f0_grid); fprintf('\n');
for i = 1:n_lc
    fprintf('  lc=%.2f: ', lc_grid(i));
    fprintf('%9.4f ', Cdpmr_tab(i, :));
    fprintf('\n');
end

fprintf('\nC_np_eff table:\n');
fprintf('      f_0: '); fprintf('%9.2f ', f0_grid); fprintf('\n');
for i = 1:n_lc
    fprintf('  lc=%.2f: ', lc_grid(i));
    fprintf('%9.4f ', Cnp_tab(i, :));
    fprintf('\n');
end

%% Reference point check (matches original Phase 1 at f_0 = 0)
[~, f0_ref_idx] = min(abs(f0_grid - 0));   % f_0 = 0
[~, lc_ref_idx] = min(abs(lc_grid - 0.7));
fprintf('\nReference point (matches Phase 1 at f_0=0):\n');
fprintf('  lc=%.2f f_0=%.2f  ->  C_dpmr_eff=%.4f, C_np_eff=%.4f\n', ...
        lc_grid(lc_ref_idx), f0_grid(f0_ref_idx), ...
        Cdpmr_tab(lc_ref_idx, f0_ref_idx), Cnp_tab(lc_ref_idx, f0_ref_idx));
fprintf('  Phase 1 value (expected): 3.9242\n');

%% Monotonicity check: C_dpmr_eff should increase with |f_0|
fprintf('\nMonotonicity check (lc=0.70 slice):\n');
slice_ref = Cdpmr_tab(lc_ref_idx, :);
for j = 1:n_f0
    fprintf('  f_0=%5.2f: C_dpmr_eff=%.4f', f0_grid(j), slice_ref(j));
    if j > 1
        d = slice_ref(j) - slice_ref(1);
        fprintf('   (delta vs f_0=0: %+.4f)', d);
    end
    fprintf('\n');
end
mono_ok = all(diff(slice_ref) >= -1e-3);  % allow tiny numerical noise
if mono_ok
    fprintf('  MONOTONIC: PASS\n');
else
    fprintf('  MONOTONIC: WARN — not strictly monotonic\n');
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
lookup.f0_grid = f0_grid;
lookup.a_pd = a_pd;
lookup.Cdpmr_tab = Cdpmr_tab;    % [n_lc x n_f0]
lookup.Cnp_tab = Cnp_tab;        % [n_lc x n_f0]
lookup.Q_kf_scale = Q_kf_scale;
lookup.R_kf_scale = R_kf_scale;
lookup.max_eig_tab = max_eig_tab;
lookup.dare_iters_tab = dare_iters_tab;
lookup.dare_err_tab = dare_err_tab;
lookup.build_timestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');
lookup.note = ['2-D augmented Lyapunov lookup for 7-state EKF + IIR HP filter. ', ...
               '(lc, f_0) grid. f_0 axis captures dependence on |f_d| via Fe(3,6). ', ...
               'a_pd = 0.05 fixed.'];

save(fullfile(out_dir, 'cdpmr_eff_lookup.mat'), '-struct', 'lookup');
fprintf('\nSaved: %s\n', fullfile(out_dir, 'cdpmr_eff_lookup.mat'));

%% Figure: 2D heatmaps
fig_dir = fullfile(project_root, 'reference', 'for_test');
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end

fig = figure('Position', [100, 100, 1400, 600]);

subplot(1, 2, 1);
imagesc(f0_grid, lc_grid, Cdpmr_tab);
set(gca, 'YDir', 'normal');
colorbar;
xlabel('f_0 [pN]');
ylabel('lc');
title('C_{dpmr,eff}(lc, f_0)');
set(gca, 'FontSize', 14, 'LineWidth', 1.5);

subplot(1, 2, 2);
imagesc(f0_grid, lc_grid, Cnp_tab);
set(gca, 'YDir', 'normal');
colorbar;
xlabel('f_0 [pN]');
ylabel('lc');
title('C_{np,eff}(lc, f_0)');
set(gca, 'FontSize', 14, 'LineWidth', 1.5);

saveas(fig, fullfile(fig_dir, 'fig_cdpmr_eff_lookup.png'));
fprintf('Figure saved: fig_cdpmr_eff_lookup.png\n');

%% Gate: reference point within expected band (matches Phase 1)
ref_val = Cdpmr_tab(lc_ref_idx, f0_ref_idx);
if ref_val >= 3.90 && ref_val <= 3.95
    fprintf('\nGATE G1 (reference match): PASS (C_dpmr_eff[0.7, 0] = %.4f in [3.90, 3.95])\n', ref_val);
else
    fprintf('\nGATE G1: FAIL (C_dpmr_eff[0.7, 0] = %.4f NOT in [3.90, 3.95])\n', ref_val);
    error('build_cdpmr_eff_lookup:reference_fail', ...
          'Reference point out of expected band');
end

if mono_ok
    fprintf('GATE G2 (monotonicity): PASS\n');
else
    fprintf('GATE G2 (monotonicity): WARN\n');
end

fprintf('\nDone.\n');
