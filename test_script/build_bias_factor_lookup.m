%% build_bias_factor_lookup.m - Build 1-D IIR finite-sample bias factor lookup
%
% Task 1c: For each lc in the grid, compute the IIR_bias_factor from the
% closed-form derived in Task 1b:
%
%   IIR_bias_factor(lc) = E[V_IIR] / gamma(0)
%                       = 1 - (a_prd/(2-a_prd)) * (1 + 2*sum_L rho(L)*(1-a_prd)^L)
%
% where rho(L) is the autocorrelation of del_pmr computed from A_aug / Sigma_aug
% via compute_7state_cdpmr_eff (thermal-only, f_0 = 0, free space).
%
% The controller applies this as: a_m = (V_IIR / IIR_bias_factor - noise_corr) / den
% to undo the finite-sample + autocorrelation bias of the EMA variance estimator.
%
% Output: test_results/verify/bias_factor_lookup.mat
%         reference/qr_analysis/fig_bias_factor_lookup.png

clear; close all; clc;

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(script_dir);
addpath(script_dir);
addpath(fullfile(project_root, 'model'));
addpath(fullfile(project_root, 'model', 'config'));

fprintf('===== build_bias_factor_lookup (1-D lc sweep, Task 1c) =====\n\n');

%% Grid (aligned with cdpmr_eff_lookup)
lc_grid = [0.4, 0.5, 0.6, 0.7, 0.8, 0.9];
a_pd    = 0.05;
a_prd   = 0.05;
L_max   = 100;

Q_kf_scale = [0; 0; 1e4; 1e-1; 0; 1e-4; 0];
R_kf_scale = [1e-2; 1e0];

n_lc = length(lc_grid);
bias_factor_tab = nan(1, n_lc);
white_bias_tab  = nan(1, n_lc);
rho_tab         = nan(n_lc, L_max+1);
max_eig_tab     = nan(1, n_lc);

fprintf('Grid: %d lc points\n', n_lc);
fprintf('a_pd = %.4f, a_prd = %.4f, L_max = %d\n', a_pd, a_prd, L_max);
fprintf('Q_kf_scale = [%s]\n', sprintf('%.1e ', Q_kf_scale));
fprintf('R_kf_scale = [%s]\n\n', sprintf('%.1e ', R_kf_scale));

%% Loop
opts = struct('f0', 0, 'verbose', false);
t0 = tic;
for i = 1:n_lc
    lc = lc_grid(i);
    [~, ~, ~, A_aug, diag_out] = compute_7state_cdpmr_eff( ...
        lc, 0, a_pd, Q_kf_scale, R_kf_scale, opts);
    Sigma_aug = diag_out.Sigma_th;

    % Observable del_pmr / (1 - a_pd) : c_s(3)=1, c_s(11)=-1
    n_aug = 11;
    c_s = zeros(n_aug, 1);
    c_s(3)  =  1;
    c_s(11) = -1;

    % Autocorrelation via rho_L = c'*A^L*Sigma*c / (c'*Sigma*c)
    gamma_L = zeros(L_max+1, 1);
    A_L = eye(n_aug);
    for L = 0:L_max
        gamma_L(L+1) = c_s' * A_L * Sigma_aug * c_s;
        A_L = A_L * A_aug;
    end
    rho = gamma_L / gamma_L(1);

    % Closed-form bias factor (autocorrelation amplified)
    powers = ((1 - a_prd).^(1:L_max)).';
    S      = sum(rho(2:L_max+1) .* powers);
    bias_factor_tab(i) = 1 - (a_prd/(2-a_prd)) * (1 + 2*S);
    white_bias_tab(i)  = 1 - (a_prd/(2-a_prd));   % no-autocorr baseline

    rho_tab(i, :)   = rho.';
    max_eig_tab(i)  = diag_out.max_eig_A_aug;

    fprintf('  lc=%.2f  bias_factor=%.4f  (white=%.4f, correction=%.4fx)\n', ...
            lc, bias_factor_tab(i), white_bias_tab(i), 1/bias_factor_tab(i));
end
fprintf('\nTotal time: %.1f sec\n', toc(t0));

%% Reference point check vs Task 1b (lc=0.7 a_prd=0.05)
[~, lc_ref_idx] = min(abs(lc_grid - 0.7));
ref_val = bias_factor_tab(lc_ref_idx);
fprintf('\nReference point (Task 1b verified):\n');
fprintf('  lc=%.2f a_prd=%.3f -> bias_factor = %.4f  (Task 1b report: 0.9069)\n', ...
        lc_grid(lc_ref_idx), a_prd, ref_val);

if abs(ref_val - 0.9069) < 1e-3
    fprintf('  GATE G1 (Task 1b match): PASS\n');
else
    warning('build_bias_factor_lookup:ref_mismatch', ...
            'Reference point %.4f differs from Task 1b 0.9069 by %.4f', ...
            ref_val, ref_val - 0.9069);
end

%% Save lookup
out_dir = fullfile(project_root, 'test_results', 'verify');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

lookup = struct();
lookup.lc_grid         = lc_grid;
lookup.bias_factor_tab = bias_factor_tab;
lookup.white_bias_tab  = white_bias_tab;
lookup.rho_tab         = rho_tab;
lookup.max_eig_tab     = max_eig_tab;
lookup.a_pd            = a_pd;
lookup.a_prd           = a_prd;
lookup.L_max           = L_max;
lookup.Q_kf_scale      = Q_kf_scale;
lookup.R_kf_scale      = R_kf_scale;
lookup.build_timestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');
lookup.note = ['IIR finite-sample bias factor for Task 1c correction. ', ...
               'Closed-form from Task 1b: E[V_IIR]/gamma(0) = 1 - (a_prd/(2-a_prd)) ', ...
               '* (1 + 2*sum_L rho(L)*(1-a_prd)^L). rho(L) from Sigma_aug (thermal, ', ...
               'f_0=0, free space). a_prd fixed at user_config default.'];

save(fullfile(out_dir, 'bias_factor_lookup.mat'), '-struct', 'lookup');
fprintf('\nSaved: %s\n', fullfile(out_dir, 'bias_factor_lookup.mat'));

%% Figure: bias_factor vs lc + first few rho lags
FS = 16;
fig = figure('Position', [100, 100, 1200, 500], 'Color', 'w', 'Visible', 'off');

subplot(1, 2, 1);
plot(lc_grid, bias_factor_tab, 'o-', 'LineWidth', 2.5, 'MarkerSize', 10, ...
    'Color', [0.8 0 0], 'MarkerFaceColor', [0.8 0 0]);
hold on;
plot(lc_grid, white_bias_tab, '--', 'LineWidth', 2.0, 'Color', [0 0.6 0]);
xlabel('lc', 'FontSize', FS, 'FontWeight', 'bold');
ylabel('IIR bias factor', 'FontSize', FS, 'FontWeight', 'bold');
legend({'autocorr (used)', 'white noise only'}, ...
    'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS-2);
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2.0, 'Box', 'on');
ylim([0.85, 1.0]);

subplot(1, 2, 2);
colors_lc = [0 0.2 0.8; 0 0.5 0.5; 0 0.6 0; 0.7 0.5 0; 0.8 0 0; 0.5 0 0.5];
for i = 1:n_lc
    plot(0:20, rho_tab(i, 1:21), '-', 'LineWidth', 1.8, ...
        'Color', colors_lc(i, :), 'DisplayName', sprintf('lc=%.1f', lc_grid(i)));
    hold on;
end
yline(0, 'k-', 'Alpha', 0.4);
xlabel('lag L [samples]', 'FontSize', FS, 'FontWeight', 'bold');
ylabel('\rho(L)', 'FontSize', FS, 'FontWeight', 'bold');
legend('Location', 'northoutside', 'Orientation', 'horizontal', ...
    'NumColumns', 3, 'FontSize', FS-4);
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2.0, 'Box', 'on');
xlim([0, 20]);

fig_dir = fullfile(project_root, 'reference', 'qr_analysis');
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end
fig_path = fullfile(fig_dir, 'fig_bias_factor_lookup.png');
exportgraphics(fig, fig_path, 'Resolution', 150);
close(fig);
fprintf('Figure saved: %s\n', fig_path);

fprintf('\nDone.\n');
