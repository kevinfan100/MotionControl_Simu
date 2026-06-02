% verify_Cdpmr_offline.m
% Phase B: Pure-sandbox comprehensive verification (CPU-light alternative).
%
% Plan:
%   PHASE 1: Run 5 baseline sims at a_pd=0.05 (production), save delta_x_m
%            for all 3 axes. Cache to disk; re-use if cache exists.
%            (5 × ~17 s = ~85 s only on first run.)
%
%   PHASE 2: Offline sandbox sweep — apply LP1(a_pd) + EWMA(a_cov=0.05) to the
%            same delta_x_m for each a_pd ∈ {0.005, 0.05, 0.5}, all 3 axes.
%            Per-segment ratio = mean(σ²̂_sandbox) / mean(σ²_theory).
%
%   PHASE 3: Aggregate stats + plots.
%
% Caveat (B's limitation): δp_m was generated at a_pd=0.05 closed loop, so
% off-design a_pd values process a slightly off-design input. The feedback
% chain a_pd → a_hat → f_d → δp_m is weak for slow trajectories, expected
% impact < 1% — verified by comparing seed=42, a_pd=0.05 case against
% compare_am.mat (same setup) at end.

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

%% Sweep grid
a_pd_list   = [0.005, 0.05, 0.5];
seed_list   = [42, 100, 200, 300, 400];
axis_names  = {'X', 'Y', 'Z'};
sigma_n_axes  = [0.00062; 0.000057; 0.00331];
sigma2_n_axes = sigma_n_axes.^2;

n_apd  = length(a_pd_list);
n_seed = length(seed_list);
n_seg  = 10;

constants = physical_constants();
kBT       = constants.k_B * constants.T;
R_radius  = constants.R;
gamma_N   = constants.gamma_N;

save_dir   = fullfile(project_root, 'test_results', 'learn_variance');
if ~exist(save_dir, 'dir'); mkdir(save_dir); end
cache_path = fullfile(save_dir, 'baseline_sims_5seeds.mat');

%% PHASE 1: Collect baseline (a_pd=0.05) sims for 5 seeds — CACHED
if ~exist(cache_path, 'file')
    fprintf('===== PHASE 1: Collecting %d baseline sims (~%.0fs total) =====\n', ...
            n_seed, n_seed*17);
    delta_x_m_all = cell(n_seed, 1);
    sigma2_ctrl_baseline = cell(n_seed, 1);   % keep controller's σ²̂ for sanity
    tout_ref = [];
    t0 = tic;
    for is = 1:n_seed
        seed = seed_list(is);
        fprintf('  seed=%d (%d/%d) ... ', seed, is, n_seed);
        config = user_config();
        config.trajectory_type   = 'ramp_descent';
        config.h_init            = 50;
        config.h_bottom          = 5;
        config.T_sim             = 20;
        config.controller_type   = 'eq17_7state';
        config.iir_warmup_mode   = 'prefill';
        config.thermal_enable    = true;
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
        delta_x_m_all{is}        = simOut.diag.delta_x_m;       % [N x 3]
        sigma2_ctrl_baseline{is} = simOut.diag.sigma2_dxr_hat;  % [N x 3]
        if isempty(tout_ref)
            tout_ref = simOut.tout;
            lambda_c = config.lambda_c;
            T_sim    = simOut.tout(end);
            Ts       = simOut.tout(2) - simOut.tout(1);
        end
    end
    fprintf('Phase 1 total: %.1fs\n', toc(t0));
    save(cache_path, 'delta_x_m_all', 'sigma2_ctrl_baseline', 'tout_ref', ...
         'lambda_c', 'T_sim', 'Ts', 'sigma2_n_axes', 'seed_list');
    fprintf('Cached → %s\n\n', cache_path);
else
    fprintf('Loading cached baseline → %s\n\n', cache_path);
    load(cache_path);
end

N = length(tout_ref);

%% Per-axis a_z(t) (deterministic, same across seeds)
h_init = 50; h_bottom = 5;
rate    = (h_init - h_bottom) / T_sim;
h_t     = max(h_init - rate * tout_ref, h_bottom);
h_bar_t = h_t / R_radius;
a_nom   = Ts / gamma_N;
a_axes_t = zeros(N, 3);
for k = 1:N
    [c_para, c_perp] = calc_correction_functions(max(h_bar_t(k), 1.001));
    a_axes_t(k, 1) = a_nom / c_para;
    a_axes_t(k, 2) = a_nom / c_para;
    a_axes_t(k, 3) = a_nom / c_perp;
end

%% PHASE 2: Sandbox sweep (no MATLAB sim, all in-memory)
fprintf('===== PHASE 2: Sandbox sweep =====\n');
ratios_all = zeros(n_apd, n_seed, 3, n_seg);
Cdpmr_used = zeros(n_apd, 1);
C_n        = 2 / (1 + lambda_c);
a_cov      = 0.05;
seg_edges  = linspace(0, T_sim, n_seg+1);

% storage for sanity check at (seed=42, a_pd=0.05, axis=Z)
sanity_sigma2_sb = [];

t0_p2 = tic;
for ip = 1:n_apd
    a_pd = a_pd_list(ip);
    C_dpmr_bracket = 2*(1-a_pd)*(1-lambda_c)/(1-(1-a_pd)*lambda_c) ...
                   + (2/(2-a_pd))/((1+lambda_c)*(1-(1-a_pd)*lambda_c));
    C_dpmr = (1-a_pd)^2 * C_dpmr_bracket;
    Cdpmr_used(ip) = C_dpmr;
    fprintf('--- a_pd=%.4f  C_dpmr=%.4f ---\n', a_pd, C_dpmr);

    % Theory mean trajectory per axis (independent of seed)
    sigma2_theory_t = zeros(N, 3);
    for ax = 1:3
        sigma2_theory_t(:, ax) = C_dpmr * 4 * kBT * a_axes_t(:, ax) ...
                               + C_n * sigma2_n_axes(ax);
    end

    for is = 1:n_seed
        delta_x_m = delta_x_m_all{is};   % [N x 3]
        sigma2_sb = zeros(N, 3);
        for ax = 1:3
            dx = delta_x_m(:, ax);
            dpmd_prev = 0;
            sig_prev  = C_dpmr * 4 * kBT * a_axes_t(1, ax) ...
                      + C_n * sigma2_n_axes(ax);
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
            for s = 1:n_seg
                if s < n_seg
                    idx = (tout_ref >= seg_edges(s)) & (tout_ref < seg_edges(s+1));
                else
                    idx = (tout_ref >= seg_edges(s)) & (tout_ref <= seg_edges(s+1));
                end
                ratios_all(ip, is, ax, s) = mean(sigma2_sb(idx, ax)) ...
                                          / mean(sigma2_theory_t(idx, ax));
            end
        end

        % Save sanity-check signal: seed=42 (is=1), a_pd=0.05 (ip=2), Z (ax=3)
        if is == 1 && abs(a_pd - 0.05) < 1e-9
            sanity_sigma2_sb = sigma2_sb(:, 3);
        end

        % Per-(a_pd, seed) summary
        r_xyz = zeros(3, 1);
        for ax = 1:3
            r = squeeze(ratios_all(ip, is, ax, :));
            r_xyz(ax) = mean(r);
        end
        fprintf('   seed=%-4d  X=%.4f  Y=%.4f  Z=%.4f\n', ...
                seed_list(is), r_xyz(1), r_xyz(2), r_xyz(3));
    end
end
fprintf('Phase 2 total: %.2fs\n\n', toc(t0_p2));

%% PHASE 2.5: Sanity — sandbox seed=42, a_pd=0.05, Z should match controller
fprintf('===== Sanity (B''s assumption check) =====\n');
sig_ctrl_z = sigma2_ctrl_baseline{1}(:, 3);   % seed=42, Z
fprintf('  ctrl    σ²̂_Z  mean = %.4e, std = %.4e\n', ...
        mean(sig_ctrl_z), std(sig_ctrl_z));
fprintf('  sandbox σ²̂_Z  mean = %.4e, std = %.4e\n', ...
        mean(sanity_sigma2_sb), std(sanity_sigma2_sb));
fprintf('  max abs diff (sandbox vs ctrl, σ²) = %.4e\n', ...
        max(abs(sanity_sigma2_sb - sig_ctrl_z)));
fprintf('  → if max diff < 1e-4, sandbox-only path is faithful enough.\n\n');

%% PHASE 3: Aggregate stats
fprintf('===== PHASE 3: Aggregate per (a_pd, axis), N=%d ratios each =====\n', ...
        n_seed*n_seg);
fprintf(' a_pd    | C_dpmr | axis | mean    std     max|dev|  | within 5%%?\n');
fprintf('---------|--------|------|---------------------------|-----------\n');
agg_mean = zeros(n_apd, 3);
agg_std  = zeros(n_apd, 3);
agg_maxd = zeros(n_apd, 3);
for ip = 1:n_apd
    for ax = 1:3
        r = reshape(squeeze(ratios_all(ip, :, ax, :)), [], 1);
        agg_mean(ip, ax) = mean(r);
        agg_std(ip, ax)  = std(r);
        agg_maxd(ip, ax) = max(abs(r - 1));
        within5 = (abs(agg_mean(ip,ax) - 1) <= 0.05);
        fprintf(' %.4f  | %.4f |  %s   | %.4f  %.4f  %.4f   |  %s\n', ...
                a_pd_list(ip), Cdpmr_used(ip), axis_names{ax}, ...
                agg_mean(ip, ax), agg_std(ip, ax), agg_maxd(ip, ax), ...
                tern(within5, 'YES', 'no '));
    end
end

fprintf('\n=== Global pooled (N=%d ratios) ===\n', numel(ratios_all));
all_r = ratios_all(:);
fprintf('  mean = %.4f, std = %.4f, median = %.4f\n', ...
        mean(all_r), std(all_r), median(all_r));
fprintf('  fraction |dev| ≤ 5%%  = %.1f%%\n', 100*mean(abs(all_r-1) <= 0.05));
fprintf('  fraction |dev| ≤ 10%% = %.1f%%\n', 100*mean(abs(all_r-1) <= 0.10));

save(fullfile(save_dir, 'verify_Cdpmr_offline.mat'), ...
     'a_pd_list', 'seed_list', 'axis_names', 'sigma2_n_axes', ...
     'ratios_all', 'agg_mean', 'agg_std', 'agg_maxd', 'Cdpmr_used');

%% Plots
set(groot, 'defaultTextInterpreter','tex');
set(groot, 'defaultLegendInterpreter','tex');
set(groot, 'defaultAxesTickLabelInterpreter','tex');

COL_X = [0.85 0.20 0.20];
COL_Y = [0.20 0.65 0.30];
COL_Z = [0.20 0.30 0.85];
COL_AX = [COL_X; COL_Y; COL_Z];

fig = figure('Position', [50 50 1700 1000], 'Color', 'w');
tl  = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% ---- (1) box plot per (a_pd, axis) ----
nexttile;
hold on;
positions = [];
data_box  = [];
group_idx = [];
counter = 0;
for ip = 1:n_apd
    for ax = 1:3
        counter = counter + 1;
        r = reshape(squeeze(ratios_all(ip, :, ax, :)), [], 1);
        positions = [positions; ip + (ax-2)*0.25];
        data_box  = [data_box; r];
        group_idx = [group_idx; counter*ones(numel(r),1)];
    end
end
boxplot(data_box, group_idx, 'positions', positions, ...
        'colors', repmat(COL_AX, n_apd, 1), 'symbol', 'k+', 'widths', 0.18);
yline(1.0,  'k--', 'LineWidth', 2);
yline(1.05, 'k:',  'LineWidth', 1);
yline(0.95, 'k:',  'LineWidth', 1);
set(gca, 'XTick', 1:n_apd, ...
         'XTickLabel', arrayfun(@(x) sprintf('a\\_pd=%.3f', x), a_pd_list, ...
                                'UniformOutput', false), ...
         'FontSize', 15, 'FontWeight', 'bold', 'LineWidth', 2);
ylabel('emp / theory', 'FontSize', 17);
title('(1) Ratio distribution per (a\_pd, axis), 50 ratios per box', 'FontSize', 16);
for ax = 1:3
    plot(NaN, NaN, 's', 'Color', COL_AX(ax,:), 'MarkerFaceColor', COL_AX(ax,:), ...
         'MarkerSize', 12, 'DisplayName', sprintf('axis %s', axis_names{ax}));
end
legend('show', 'FontSize', 13, 'Location', 'best');
grid on; box on;

% ---- (2) per-axis mean ± std vs a_pd ----
nexttile;
hold on;
for ax = 1:3
    errorbar(a_pd_list, agg_mean(:, ax), agg_std(:, ax), 'o-', ...
             'Color', COL_AX(ax, :), 'MarkerFaceColor', COL_AX(ax, :), ...
             'LineWidth', 2.2, 'MarkerSize', 10, ...
             'DisplayName', sprintf('axis %s', axis_names{ax}));
end
set(gca, 'XScale', 'log', 'FontSize', 15, 'FontWeight', 'bold', 'LineWidth', 2);
yline(1.0,  'k--', 'LineWidth', 2);
yline(1.05, 'k:',  'LineWidth', 1);
yline(0.95, 'k:',  'LineWidth', 1);
xlabel('a\_pd', 'FontSize', 17);
ylabel('mean ratio  \pm  std', 'FontSize', 17);
title('(2) Mean ratio vs a\_pd', 'FontSize', 16);
legend('show', 'FontSize', 13, 'Location', 'best');
grid on; box on;

% ---- (3) global histogram ----
nexttile;
histogram(all_r, 'BinWidth', 0.02, 'FaceColor', [0.4 0.4 0.7], ...
          'EdgeColor', 'k', 'FaceAlpha', 0.7);
hold on;
xline(1.0, 'k--', 'LineWidth', 2.5);
xline(mean(all_r), 'r-', 'LineWidth', 2.5);
set(gca, 'FontSize', 15, 'FontWeight', 'bold', 'LineWidth', 2);
xlabel('emp / theory', 'FontSize', 17);
ylabel('count', 'FontSize', 17);
title(sprintf('(3) Global distribution (N=%d, mean=%.3f, std=%.3f)', ...
              length(all_r), mean(all_r), std(all_r)), 'FontSize', 15);
legend({'distribution', 'target=1', sprintf('mean=%.3f', mean(all_r))}, ...
       'FontSize', 13, 'Location', 'best');
grid on; box on;

% ---- (4) seed consistency ----
nexttile;
hold on;
for ip = 1:n_apd
    for ax = 1:3
        seed_means = squeeze(mean(ratios_all(ip, :, ax, :), 4));
        scatter(repmat(ip + (ax-2)*0.25, n_seed, 1), seed_means, ...
                90, COL_AX(ax, :), 'filled', 'MarkerEdgeColor', 'k', ...
                'MarkerFaceAlpha', 0.7);
    end
end
yline(1.0, 'k--', 'LineWidth', 2);
yline(1.05, 'k:', 'LineWidth', 1);
yline(0.95, 'k:', 'LineWidth', 1);
set(gca, 'XTick', 1:n_apd, ...
         'XTickLabel', arrayfun(@(x) sprintf('a\\_pd=%.3f', x), a_pd_list, ...
                                'UniformOutput', false), ...
         'FontSize', 15, 'FontWeight', 'bold', 'LineWidth', 2);
ylabel('seed-mean ratio (over 10 segments)', 'FontSize', 16);
title('(4) Seed-by-seed consistency (5 dots/axis/a\_pd)', 'FontSize', 16);
grid on; box on;

sgtitle('C_{dpmr} offline comprehensive: 3 a_{pd} × 5 seeds × 3 axes × 10 segs = 450 ratios', ...
        'FontSize', 17, 'FontWeight', 'bold');

out_path = fullfile(save_dir, 'verify_Cdpmr_offline.png');
exportgraphics(fig, out_path, 'Resolution', 150);
fprintf('\nSaved: %s\n', out_path);

set(groot, 'defaultTextInterpreter','remove');
set(groot, 'defaultLegendInterpreter','remove');
set(groot, 'defaultAxesTickLabelInterpreter','remove');

function s = tern(cond, a, b)
    if cond, s = a; else, s = b; end
end
