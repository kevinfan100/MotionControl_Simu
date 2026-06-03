% verify_Cn_thermal_off.m
% Isolate C_n by turning thermal OFF:  E[σ²̂_δpmr] = C_n · σ²_n   (no thermal term)
% Compare empirical C_n vs three candidate closed-forms:
%   (A) white noise through HP1:
%         C_n_A = 2(1-a_pd)²/(2-a_pd)
%   (B) λc-coloured noise through HP1 (numerical integral):
%         C_n_B = (1/2π) ∫ |H_HP|² · S_n_coloured(ω) dω
%   (C) current code (small-a_pd limit of B):
%         C_n_C = 2/(1+λc)
%
% Path: (B-style) baseline-sims + offline sandbox sweep to minimize CPU.
%   - Run 3 thermal-OFF sims at a_pd=0.05, positioning h=20 um
%   - Offline sandbox sweep over a_pd ∈ {0.005, 0.05, 0.5}, all 3 axes

clear; close all; clc;
clear motion_control_law motion_control_law_eq17_core ...
      motion_control_law_7state trajectory_generator calc_thermal_force;

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(fileparts(script_dir));
cd(project_root);
addpath(fullfile(project_root, 'model'));
addpath(fullfile(project_root, 'model', 'config'));
addpath(fullfile(project_root, 'model', 'wall_effect'));
addpath(fullfile(project_root, 'model', 'thermal_force'));
addpath(fullfile(project_root, 'model', 'trajectory'));
addpath(fullfile(project_root, 'model', 'controller'));
addpath(fullfile(project_root, 'model', 'dual_track'));

a_pd_list      = [0.005, 0.05, 0.5];
seed_list      = [42, 100, 200];
axis_names     = {'X', 'Y', 'Z'};
sigma_n_axes   = [0.00062; 0.000057; 0.00331];
sigma2_n_axes  = sigma_n_axes.^2;

n_apd  = length(a_pd_list);
n_seed = length(seed_list);

constants = physical_constants();
R_radius  = constants.R;

save_dir   = fullfile(project_root, 'test_results', 'learn_variance');
cache_path = fullfile(save_dir, 'baseline_thermal_off_3seeds.mat');

%% PHASE 1: Collect thermal-OFF baseline sims (cached)
if ~exist(cache_path, 'file')
    fprintf('===== PHASE 1: thermal-OFF baseline sims =====\n');
    delta_x_m_all = cell(n_seed, 1);
    sigma2_ctrl_all = cell(n_seed, 1);
    tout_ref = [];
    t0 = tic;
    for is = 1:n_seed
        seed = seed_list(is);
        fprintf('  seed=%d (%d/%d) ... ', seed, is, n_seed);
        config = user_config();
        config.trajectory_type   = 'positioning';
        config.h_init            = 20;            % away from wall
        config.T_sim             = 20;
        config.controller_type   = 'eq17_7state';
        config.iir_warmup_mode   = 'prefill';
        config.thermal_enable    = false;          % <-- thermal OFF
        config.meas_noise_enable = true;
        config.meas_noise_std    = sigma_n_axes;
        config.a_pd              = 0.05;
        config.a_cov             = 0.05;
        opts = struct('seed', seed, 'verbose', false, 'collect_diag', true);
        clear motion_control_law motion_control_law_eq17_core ...
              motion_control_law_7state trajectory_generator calc_thermal_force;
        ts = tic;
        simOut = run_pure_simulation(config, opts);
        fprintf('%.1fs\n', toc(ts));
        delta_x_m_all{is}   = simOut.diag.delta_x_m;
        sigma2_ctrl_all{is} = simOut.diag.sigma2_dxr_hat;
        if isempty(tout_ref)
            tout_ref = simOut.tout;
            lambda_c = config.lambda_c;
            T_sim    = simOut.tout(end);
            Ts       = simOut.tout(2) - simOut.tout(1);
        end
    end
    fprintf('Phase 1 total: %.1fs\n', toc(t0));
    save(cache_path, 'delta_x_m_all', 'sigma2_ctrl_all', 'tout_ref', ...
         'lambda_c', 'T_sim', 'Ts', 'sigma2_n_axes', 'seed_list');
    fprintf('Cached → %s\n\n', cache_path);
else
    fprintf('Loading cache → %s\n\n', cache_path);
    load(cache_path);
end

N = length(tout_ref);

%% Sanity: production C_n with the actual sigma2_n
fprintf('=== Production a_pd=0.05 controller sanity ===\n');
% drop first 2 sec for IIR warm-up settling
warmup_idx = tout_ref >= 2;
for is = 1:n_seed
    for ax = 1:3
        m = mean(sigma2_ctrl_all{is}(warmup_idx, ax));
        Cn_emp = m / sigma2_n_axes(ax);
        fprintf('  seed=%-4d  axis %s: σ²̂_mean=%.4e  C_n_emp = %.4f\n', ...
                seed_list(is), axis_names{ax}, m, Cn_emp);
    end
end
fprintf('  (production code C_n = 2/(1+λc) = %.4f)\n\n', 2/(1+lambda_c));

%% PHASE 2: Sandbox sweep
fprintf('===== PHASE 2: offline sandbox a_pd sweep =====\n');
a_cov = 0.05;

Cn_emp_all = zeros(n_apd, n_seed, 3);   % [apd, seed, axis]
for ip = 1:n_apd
    a_pd = a_pd_list(ip);
    fprintf('--- a_pd=%.4f ---\n', a_pd);
    for is = 1:n_seed
        delta_x_m = delta_x_m_all{is};
        sigma2_sb = zeros(N, 3);
        for ax = 1:3
            dx = delta_x_m(:, ax);
            dpmd_prev = 0;
            sig_prev  = sigma2_n_axes(ax);    % rough prefill (~C_n·σ²_n scale)
            for k = 1:N
                dpmd = (1-a_pd)*dpmd_prev + a_pd*dx(k);
                dpmr = dx(k) - dpmd;
                sig_new = (1-a_cov)*sig_prev + a_cov*dpmr^2;
                sigma2_sb(k, ax) = sig_new;
                dpmd_prev = dpmd;
                sig_prev  = sig_new;
            end
        end
        for ax = 1:3
            % drop first 2 sec for warmup
            m = mean(sigma2_sb(warmup_idx, ax));
            Cn_emp_all(ip, is, ax) = m / sigma2_n_axes(ax);
        end
        fprintf('  seed=%-4d  C_n_emp:  X=%.4f  Y=%.4f  Z=%.4f\n', ...
                seed_list(is), Cn_emp_all(ip, is, 1), ...
                Cn_emp_all(ip, is, 2), Cn_emp_all(ip, is, 3));
    end
end

%% Candidate closed-form C_n values
omega   = linspace(-pi, pi, 4096);
z_inv   = exp(-1i*omega);

Cn_A = zeros(n_apd, 1);   % white through HP1
Cn_B = zeros(n_apd, 1);   % λc-coloured (variance=σ²_n) through HP1
Cn_C = 2/(1+lambda_c);    % current production formula

% λc-coloured spectrum (normalized so ∫S/2π = 1, i.e. var = σ²_n)
S_lc = (1-lambda_c^2) ./ abs(1 - lambda_c*z_inv).^2;

for ip = 1:n_apd
    a = a_pd_list(ip);
    H_HP_sq = abs((1-a)*(1 - z_inv) ./ (1 - (1-a)*z_inv)).^2;
    Cn_A(ip) = 2*(1-a)^2/(2-a);
    Cn_B(ip) = trapz(omega, H_HP_sq .* S_lc) / (2*pi);
end

%% Aggregate
fprintf('\n===== Aggregate: empirical C_n vs three candidates =====\n');
fprintf(' a_pd    | axis | C_n_emp(mean±std, 3 seeds)  | (A) white  (B) λc-col  (C) code\n');
fprintf('---------|------|-----------------------------|-----------------------------\n');
for ip = 1:n_apd
    for ax = 1:3
        r = squeeze(Cn_emp_all(ip, :, ax));
        fprintf(' %.4f  |  %s   | %.4f ± %.4f               | %.4f      %.4f      %.4f\n', ...
                a_pd_list(ip), axis_names{ax}, ...
                mean(r), std(r), Cn_A(ip), Cn_B(ip), Cn_C);
    end
end

% Pool across axes & seeds — but X/Y likely too noisy (sensor too small)
% so pool only meaningful: use Z (largest sensor noise, cleanest C_n signal)
fprintf('\n=== Z-axis pooled (3 seeds, cleanest signal) ===\n');
fprintf(' a_pd    | C_n_emp_Z (mean ± std) | best match\n');
fprintf('---------|------------------------|------------\n');
for ip = 1:n_apd
    r = squeeze(Cn_emp_all(ip, :, 3));
    diffA = abs(mean(r) - Cn_A(ip));
    diffB = abs(mean(r) - Cn_B(ip));
    diffC = abs(mean(r) - Cn_C);
    [~, best] = min([diffA, diffB, diffC]);
    bestlbl = {'A (white)', 'B (λc-col)', 'C (code)'};
    fprintf(' %.4f  | %.4f ± %.4f         | %s\n', ...
            a_pd_list(ip), mean(r), std(r), bestlbl{best});
end

save(fullfile(save_dir, 'verify_Cn_thermal_off.mat'), ...
     'a_pd_list', 'seed_list', 'axis_names', 'sigma2_n_axes', ...
     'Cn_emp_all', 'Cn_A', 'Cn_B', 'Cn_C', 'lambda_c');

%% Plot
set(groot, 'defaultTextInterpreter','tex');
set(groot, 'defaultLegendInterpreter','tex');
set(groot, 'defaultAxesTickLabelInterpreter','tex');

COL_X    = [0.85 0.20 0.20];
COL_Y    = [0.20 0.65 0.30];
COL_Z    = [0.20 0.30 0.85];
COL_AX   = [COL_X; COL_Y; COL_Z];
COL_A    = [0.50 0.50 0.50];   % white-noise theory
COL_B    = [0.00 0.55 0.00];   % λc-coloured theory
COL_C    = [0.80 0.40 0.00];   % current code

fig = figure('Position', [50 50 1500 800], 'Color', 'w');
tl  = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% (1) C_n vs a_pd — Z axis (cleanest), with 3 theory candidates
nexttile;
a_dense = logspace(log10(0.001), log10(0.8), 100);
Cn_A_dense = 2*(1-a_dense).^2./(2-a_dense);
Cn_B_dense = zeros(size(a_dense));
for i = 1:length(a_dense)
    a = a_dense(i);
    H_HP_sq = abs((1-a)*(1 - z_inv) ./ (1 - (1-a)*z_inv)).^2;
    Cn_B_dense(i) = trapz(omega, H_HP_sq .* S_lc) / (2*pi);
end

semilogx(a_dense, Cn_A_dense, '-', 'Color', COL_A, 'LineWidth', 2, ...
         'DisplayName', '(A) white through HP1'); hold on;
semilogx(a_dense, Cn_B_dense, '-', 'Color', COL_B, 'LineWidth', 2, ...
         'DisplayName', '(B) λ_c-coloured through HP1');
semilogx(a_dense, Cn_C*ones(size(a_dense)), '-', 'Color', COL_C, ...
         'LineWidth', 2, 'DisplayName', '(C) 2/(1+λ_c) [code]');
% Empirical Z axis: scatter
for ip = 1:n_apd
    r = squeeze(Cn_emp_all(ip, :, 3));
    errorbar(a_pd_list(ip), mean(r), std(r), 'o', 'Color', COL_Z, ...
             'MarkerFaceColor', COL_Z, 'MarkerSize', 12, 'LineWidth', 2, ...
             'HandleVisibility', 'off');
end
plot(NaN, NaN, 'o', 'Color', COL_Z, 'MarkerFaceColor', COL_Z, ...
     'MarkerSize', 12, 'DisplayName', 'empirical (Z axis, 3 seeds)');
set(gca, 'FontSize', 16, 'FontWeight', 'bold', 'LineWidth', 2);
xlabel('a\_pd', 'FontSize', 18);
ylabel('C_n', 'FontSize', 18);
title('C_n vs a\_pd  (Z axis pooled, σ_n=0.00331)', 'FontSize', 16);
legend('show', 'FontSize', 13, 'Location', 'southwest');
grid on; box on;
ylim([0 1.5]);

% (2) Per-axis empirical at each a_pd
nexttile;
hold on;
for ax = 1:3
    means = zeros(n_apd, 1);
    stds  = zeros(n_apd, 1);
    for ip = 1:n_apd
        r = squeeze(Cn_emp_all(ip, :, ax));
        means(ip) = mean(r);
        stds(ip)  = std(r);
    end
    errorbar(a_pd_list, means, stds, 'o-', 'Color', COL_AX(ax,:), ...
             'MarkerFaceColor', COL_AX(ax,:), 'MarkerSize', 10, ...
             'LineWidth', 2, 'DisplayName', sprintf('axis %s (σ_n=%g)', ...
             axis_names{ax}, sigma_n_axes(ax)));
end
% Overlay theory candidates
semilogx(a_dense, Cn_A_dense, '-',  'Color', COL_A, 'LineWidth', 1.5, ...
         'DisplayName', '(A) white');
semilogx(a_dense, Cn_B_dense, '-',  'Color', COL_B, 'LineWidth', 1.5, ...
         'DisplayName', '(B) λ_c-col');
semilogx(a_dense, Cn_C*ones(size(a_dense)), '-', 'Color', COL_C, ...
         'LineWidth', 1.5, 'DisplayName', '(C) code');
set(gca, 'XScale', 'log', 'FontSize', 16, 'FontWeight', 'bold', 'LineWidth', 2);
xlabel('a\_pd', 'FontSize', 18);
ylabel('C_n empirical', 'FontSize', 18);
title('C_n per axis (X/Y σ_n tiny → high noise)', 'FontSize', 16);
legend('show', 'FontSize', 12, 'Location', 'southwest', 'NumColumns', 2);
grid on; box on;

sgtitle('C_n thermal-OFF verification: empirical vs (A) white / (B) λ_c-col / (C) code', ...
        'FontSize', 17, 'FontWeight', 'bold');

out_path = fullfile(save_dir, 'verify_Cn_thermal_off.png');
exportgraphics(fig, out_path, 'Resolution', 150);
fprintf('\nSaved: %s\n', out_path);

set(groot, 'defaultTextInterpreter','remove');
set(groot, 'defaultLegendInterpreter','remove');
set(groot, 'defaultAxesTickLabelInterpreter','remove');
