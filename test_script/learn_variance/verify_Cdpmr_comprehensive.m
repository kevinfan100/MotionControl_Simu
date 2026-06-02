% verify_Cdpmr_comprehensive.m
% Broad quantitative C_dpmr verification.
%   - a_pd ∈ {0.005, 0.05, 0.5}        (100× span; (1-a_pd)² from 0.99 to 0.25)
%   - seeds ∈ {42, 100, 200, 300, 400}  (5 seeds, statistical confidence)
%   - all 3 axes (X, Y, Z)              (c_para for X/Y, c_perp for Z)
%   - trajectory: ramp_descent 50→5 um, T_sim=20s
% Total: 3 × 5 = 15 sims × ~17 s = ~4 min
%
% Per (a_pd, axis) cell: 5 seeds × 10 segments = 50 ratios → aggregate
%   target ratio = 1.000
%   acceptance band: |dev| ≤ 0.05 (5%)

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
a_pd_list = [0.005, 0.05, 0.5];
seed_list = [42, 100, 200, 300, 400];
n_apd  = length(a_pd_list);
n_seed = length(seed_list);
axis_names = {'X', 'Y', 'Z'};

constants  = physical_constants();
kBT        = constants.k_B * constants.T;
R_radius   = constants.R;
gamma_N    = constants.gamma_N;

sigma_n_axes  = [0.00062; 0.000057; 0.00331];   % production noise levels
sigma2_n_axes = sigma_n_axes.^2;

n_seg = 10;
ratios_all = zeros(n_apd, n_seed, 3, n_seg);    % [a_pd, seed, axis, seg]
Cdpmr_used = zeros(n_apd, 1);

%% Run sweep
fprintf('===== Comprehensive C_dpmr verification =====\n');
fprintf('  a_pd:  '); fprintf('%.3f ', a_pd_list); fprintf('\n');
fprintf('  seeds: '); fprintf('%d ', seed_list); fprintf('\n');
fprintf('  axes:  X Y Z\n');
fprintf('  total sims = %d × ~17s = ~%.1f min\n\n', ...
        n_apd*n_seed, n_apd*n_seed*17/60);

t0_total = tic;
for ip = 1:n_apd
    a_pd = a_pd_list(ip);
    for is = 1:n_seed
        seed = seed_list(is);
        fprintf('--- a_pd=%.4f, seed=%-4d ---\n', a_pd, seed);

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
        config.a_pd              = a_pd;
        config.a_cov             = 0.05;

        opts = struct('seed', seed, 'verbose', false, 'collect_diag', true);
        clear motion_control_law motion_control_law_eq17_core ...
              motion_control_law_7state trajectory_generator calc_thermal_force;
        simOut = run_pure_simulation(config, opts);

        tout         = simOut.tout;
        N            = length(tout);
        T_sim        = tout(end);
        sigma2_diag  = simOut.diag.sigma2_dxr_hat;   % [N x 3]
        lambda_c     = config.lambda_c;
        Ts           = tout(2) - tout(1);

        % Theory C_dpmr (closed-form with (1-a_pd)^2 prefactor)
        C_dpmr_bracket = 2*(1-a_pd)*(1-lambda_c)/(1-(1-a_pd)*lambda_c) ...
                       + (2/(2-a_pd))/((1+lambda_c)*(1-(1-a_pd)*lambda_c));
        C_dpmr = (1-a_pd)^2 * C_dpmr_bracket;
        C_n    = 2 / (1 + lambda_c);
        Cdpmr_used(ip) = C_dpmr;

        % h(t) ramp + per-axis a(t) (X/Y use c_para, Z uses c_perp)
        rate    = (config.h_init - config.h_bottom) / config.T_sim;
        h_t     = max(config.h_init - rate * tout, config.h_bottom);
        h_bar_t = h_t / R_radius;
        a_nom   = Ts / gamma_N;
        a_axes_t = zeros(N, 3);
        for k = 1:N
            [c_para, c_perp] = calc_correction_functions(max(h_bar_t(k), 1.001));
            a_axes_t(k, 1) = a_nom / c_para;
            a_axes_t(k, 2) = a_nom / c_para;
            a_axes_t(k, 3) = a_nom / c_perp;
        end

        % Per-axis theory mean trajectory
        sigma2_theory_t = zeros(N, 3);
        for ax = 1:3
            sigma2_theory_t(:, ax) = C_dpmr * 4 * kBT * a_axes_t(:, ax) ...
                                   + C_n * sigma2_n_axes(ax);
        end

        % Per-segment empirical vs theory ratio
        seg_edges = linspace(0, T_sim, n_seg+1);
        for ax = 1:3
            for s = 1:n_seg
                if s < n_seg
                    idx = (tout >= seg_edges(s)) & (tout < seg_edges(s+1));
                else
                    idx = (tout >= seg_edges(s)) & (tout <= seg_edges(s+1));
                end
                ratios_all(ip, is, ax, s) = mean(sigma2_diag(idx, ax)) ...
                                          / mean(sigma2_theory_t(idx, ax));
            end
            r = squeeze(ratios_all(ip, is, ax, :));
            fprintf('   axis %s : mean=%.4f  std=%.4f  max|dev|=%.4f\n', ...
                    axis_names{ax}, mean(r), std(r), max(abs(r-1)));
        end
    end
end
fprintf('\nTotal sim time: %.1f s\n', toc(t0_total));

%% Aggregate per (a_pd, axis)
fprintf('\n===== Aggregate per (a_pd, axis): N=%d ratios each =====\n', ...
        n_seed*n_seg);
fprintf(' a_pd    | C_dpmr  | axis | mean    std     max|dev|  | within 5%%?\n');
fprintf('---------|---------|------|---------------------------|------------\n');
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
        fprintf(' %.4f  | %.4f  |  %s   | %.4f  %.4f  %.4f   |  %s\n', ...
                a_pd_list(ip), Cdpmr_used(ip), axis_names{ax}, ...
                agg_mean(ip, ax), agg_std(ip, ax), agg_maxd(ip, ax), ...
                tern(within5, 'YES', 'no '));
    end
end

fprintf('\n=== Global pooled (N=%d ratios) ===\n', numel(ratios_all));
all_r = ratios_all(:);
fprintf('  mean = %.4f, std = %.4f, median = %.4f\n', ...
        mean(all_r), std(all_r), median(all_r));
fprintf('  fraction within 5%%  = %.1f%%\n', 100*mean(abs(all_r-1) <= 0.05));
fprintf('  fraction within 10%% = %.1f%%\n', 100*mean(abs(all_r-1) <= 0.10));

%% Save
save_dir = fullfile(project_root, 'test_results', 'learn_variance');
save(fullfile(save_dir, 'verify_Cdpmr_comprehensive.mat'), ...
     'a_pd_list', 'seed_list', 'axis_names', 'sigma2_n_axes', ...
     'ratios_all', 'agg_mean', 'agg_std', 'agg_maxd', 'Cdpmr_used');

%% Plot
set(groot, 'defaultTextInterpreter','tex');
set(groot, 'defaultLegendInterpreter','tex');
set(groot, 'defaultAxesTickLabelInterpreter','tex');

COL_X = [0.85 0.20 0.20];
COL_Y = [0.20 0.65 0.30];
COL_Z = [0.20 0.30 0.85];
COL_AX = [COL_X; COL_Y; COL_Z];

fig = figure('Position', [50 50 1700 1000], 'Color', 'w');
tl  = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% ---- (1) box plot of ratios per (a_pd, axis) ----
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
% legend markers
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
title('(2) Mean ratio vs a\_pd (axes overlaid)', 'FontSize', 16);
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
title(sprintf('(3) Global ratio distribution (N=%d, mean=%.3f, std=%.3f)', ...
              length(all_r), mean(all_r), std(all_r)), 'FontSize', 15);
legend({'distribution', 'target=1', sprintf('mean=%.3f', mean(all_r))}, ...
       'FontSize', 13, 'Location', 'best');
grid on; box on;

% ---- (4) seed-to-seed consistency ----
nexttile;
hold on;
for ip = 1:n_apd
    for ax = 1:3
        seed_means = squeeze(mean(ratios_all(ip, :, ax, :), 4));  % [5x1]
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
title('(4) Seed-by-seed consistency  (5 dots/axis/a\_pd)', 'FontSize', 16);
grid on; box on;

sgtitle('C_{dpmr} comprehensive verification: 3 a_{pd} × 5 seeds × 3 axes × 10 segs = 450 ratios', ...
        'FontSize', 17, 'FontWeight', 'bold');

out_path = fullfile(save_dir, 'verify_Cdpmr_comprehensive.png');
exportgraphics(fig, out_path, 'Resolution', 150);
fprintf('\nSaved: %s\n', out_path);

set(groot, 'defaultTextInterpreter','remove');
set(groot, 'defaultLegendInterpreter','remove');
set(groot, 'defaultAxesTickLabelInterpreter','remove');

function s = tern(cond, a, b)
    if cond, s = a; else, s = b; end
end
