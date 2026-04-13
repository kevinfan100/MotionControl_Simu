%% verify_task1d_paper_benchmark_mc.m - Task 1d A2: dynamic near-wall MC
%
% Wraps the logic of phase2_paper_benchmark.m in a Monte Carlo loop over seeds
% to measure median a_hat error in the dynamic near-wall benchmark scenario,
% with Task 1c correction enabled.
%
% Scenario (matches phase2_paper_benchmark.m):
%   - lc = 0.4 (paper's value)
%   - 1 Hz oscillation, 2.5 um amplitude, 10 cycles
%   - h_init = 50, h_bottom = 2.5 (descending near wall)
%   - thermal_enable = true, meas_noise_enable = false
%   - T_sim = 12 s
%
% Output per seed: a_hat_x / a_hat_z median error vs a_true, cross-correlation
% lag with a_true, per-seed stats. Aggregates pooled across all seeds.
%
% Output files:
%   test_results/verify/task1d_paper_benchmark_mc.mat
%   reference/for_test/fig_task1d_paper_benchmark.png

clear; clc;
clear motion_control_law motion_control_law_7state ...
      motion_control_law_23state trajectory_generator calc_thermal_force;

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(script_dir);
cd(project_root);

addpath(fullfile(project_root, 'model'));
addpath(fullfile(project_root, 'model', 'config'));
addpath(fullfile(project_root, 'model', 'wall_effect'));
addpath(fullfile(project_root, 'model', 'thermal_force'));
addpath(fullfile(project_root, 'model', 'trajectory'));
addpath(fullfile(project_root, 'model', 'controller'));
addpath(script_dir);

fprintf('===== Task 1d A2: dynamic near-wall Monte Carlo =====\n\n');

constants = physical_constants();
k_B     = constants.k_B;
T_temp  = constants.T;
Ts      = constants.Ts;
gamma_N = constants.gamma_N;
a_nom   = Ts / gamma_N;
R_probe = constants.R;

%% Scenario config (paper-like, matches phase2_paper_benchmark.m)
n_seeds = 10;
base_seed = 20260412;

config = user_config();
config.h_init             = 50;
config.h_bottom           = 2.5;
config.amplitude          = 2.5;
config.frequency          = 1;
config.n_cycles           = 10;
config.t_hold             = 0.5;
config.enable_wall_effect = true;
config.trajectory_type    = 'osc';
config.ctrl_enable        = true;
config.controller_type    = 7;
config.lambda_c           = 0.4;         % paper's value
config.thermal_enable     = true;
config.meas_noise_enable  = false;
config.meas_noise_std     = [0; 0; 0];
config.T_sim              = 12;

% Build params once just to display config info (will be rebuilt per seed)
params_init = calc_simulation_params(config);
fprintf('Config: lc=%.2f, freq=%.1f Hz, amp=%.1f um, n_cycles=%d, T_sim=%.1f s\n', ...
        config.lambda_c, config.frequency, config.amplitude, ...
        config.n_cycles, config.T_sim);
fprintf('Task 1c: IIR_bias_factor = %.4f, C_dpmr_eff = %.4f\n', ...
        params_init.Value.ctrl.IIR_bias_factor, params_init.Value.ctrl.C_dpmr_eff);
fprintf('MC: %d seeds, base seed = %d\n\n', n_seeds, base_seed);

%% Constants for a_true computation (same across all seeds)
w_hat = params_init.Value.wall.w_hat;
pz    = params_init.Value.wall.pz;

%% MC loop
model_path = fullfile(project_root, 'model', 'system_model');

per_seed = struct();
per_seed.seed               = zeros(1, n_seeds);
per_seed.median_err_x       = zeros(1, n_seeds);
per_seed.median_err_z       = zeros(1, n_seeds);
per_seed.mean_err_x         = zeros(1, n_seeds);
per_seed.mean_err_z         = zeros(1, n_seeds);
per_seed.max_err_x          = zeros(1, n_seeds);
per_seed.max_err_z          = zeros(1, n_seeds);
per_seed.rmse_3d            = zeros(1, n_seeds);
per_seed.lag_x_samples      = zeros(1, n_seeds);
per_seed.lag_z_samples      = zeros(1, n_seeds);
per_seed.ok                 = false(1, n_seeds);

% Collect pooled error arrays (will grow)
pool_err_z = [];
pool_err_x = [];

% For figure: keep seed 1 time series
sample_seed_data = struct();

t_mc0 = tic;
for i = 1:n_seeds
    seed = base_seed + i;
    per_seed.seed(i) = seed;

    clear motion_control_law motion_control_law_7state ...
          motion_control_law_23state trajectory_generator calc_thermal_force;

    % Rebuild params so calc_thermal_params.randi(...) gives a fresh thermal seed
    rng(seed);
    params = calc_simulation_params(config);
    assignin('base', 'params', params);
    assignin('base', 'p0', params.Value.common.p0);
    assignin('base', 'Ts', Ts);
    thermal_seed_i = params.Value.thermal.seed;

    fprintf('[%2d/%2d] seed=%d th_seed=%d ...', i, n_seeds, seed, thermal_seed_i);
    t0 = tic;
    try
        simOut = sim(model_path, 'StopTime', num2str(config.T_sim), ...
            'SaveTime', 'on', 'TimeSaveName', 'tout', ...
            'SaveOutput', 'on', 'OutputSaveName', 'yout');
    catch ME
        fprintf(' FAIL: %s\n', ME.message);
        continue;
    end
    fprintf(' ran (%.1f sec)', toc(t0));

    p_d = simOut.p_d_out';
    p_m = simOut.p_m_out';
    ekf = simOut.ekf_out';
    N   = size(p_d, 2);
    t   = (0:N-1) * Ts;

    a_hat_x = ekf(1, :);
    a_hat_z = ekf(2, :);

    % Ground truth a_true from h
    h_k = p_m' * w_hat - pz;     % N x 1
    a_true_traj = zeros(N, 1);
    for k = 1:N
        h_bar = max(h_k(k) / R_probe, 1.001);
        [~, cperp] = calc_correction_functions(h_bar);
        a_true_traj(k) = Ts / (gamma_N * cperp);
    end

    % Steady state: skip first 2 s (EKF convergence)
    ss = round(2/Ts):N;

    err_x = (a_hat_x(ss).' - a_true_traj(ss)) ./ a_true_traj(ss);
    err_z = (a_hat_z(ss).' - a_true_traj(ss)) ./ a_true_traj(ss);
    abs_err_x = abs(err_x);
    abs_err_z = abs(err_z);

    per_seed.median_err_x(i) = median(abs_err_x);
    per_seed.median_err_z(i) = median(abs_err_z);
    per_seed.mean_err_x(i)   = mean(abs_err_x);
    per_seed.mean_err_z(i)   = mean(abs_err_z);
    per_seed.max_err_x(i)    = max(abs_err_x);
    per_seed.max_err_z(i)    = max(abs_err_z);
    err_3d = sqrt(sum((p_m(:, ss) - p_d(:, ss)).^2, 1));
    per_seed.rmse_3d(i)      = mean(err_3d);

    % Cross-correlation for lag (z-axis)
    a_hat_z_centered = a_hat_z(ss) - mean(a_hat_z(ss));
    a_true_centered  = a_true_traj(ss).' - mean(a_true_traj(ss));
    max_lag = 100;
    if std(a_true_centered) > 1e-12
        [xc_z, lags_z] = xcorr(a_hat_z_centered, a_true_centered, max_lag, 'coeff');
        [~, idx_peak] = max(xc_z);
        per_seed.lag_z_samples(i) = lags_z(idx_peak);
    else
        per_seed.lag_z_samples(i) = 0;
    end

    a_hat_x_centered = a_hat_x(ss) - mean(a_hat_x(ss));
    if std(a_true_centered) > 1e-12
        [xc_x, lags_x] = xcorr(a_hat_x_centered, a_true_centered, max_lag, 'coeff');
        [~, idx_peak] = max(xc_x);
        per_seed.lag_x_samples(i) = lags_x(idx_peak);
    else
        per_seed.lag_x_samples(i) = 0;
    end

    pool_err_z = [pool_err_z; err_z(:)];
    pool_err_x = [pool_err_x; err_x(:)];

    per_seed.ok(i) = true;

    if i == 1
        sample_seed_data.t           = t;
        sample_seed_data.ss          = ss;
        sample_seed_data.a_hat_x     = a_hat_x;
        sample_seed_data.a_hat_z     = a_hat_z;
        sample_seed_data.a_true_traj = a_true_traj;
        sample_seed_data.h_k         = h_k;
        sample_seed_data.p_d         = p_d;
        sample_seed_data.p_m         = p_m;
        sample_seed_data.xc_z        = xc_z;
        sample_seed_data.lags_z      = lags_z;
    end

    fprintf('  med_err: x=%5.2f%% z=%5.2f%%  lag_z=%+3d\n', ...
        per_seed.median_err_x(i)*100, per_seed.median_err_z(i)*100, ...
        per_seed.lag_z_samples(i));
end
fprintf('\nTotal MC time: %.1f sec\n', toc(t_mc0));

%% Aggregate
ok = per_seed.ok;
n_ok = sum(ok);
fprintf('\n--- Aggregate stats (%d / %d ok seeds) ---\n', n_ok, n_seeds);

pool_abs_z = abs(pool_err_z);
pool_abs_x = abs(pool_err_x);

agg.pool_median_x = median(pool_abs_x);
agg.pool_median_z = median(pool_abs_z);
agg.pool_mean_x   = mean(pool_abs_x);
agg.pool_mean_z   = mean(pool_abs_z);
agg.pool_p90_x    = prctile(pool_abs_x, 90);
agg.pool_p90_z    = prctile(pool_abs_z, 90);
agg.pool_max_x    = max(pool_abs_x);
agg.pool_max_z    = max(pool_abs_z);

agg.seed_median_median_x = median(per_seed.median_err_x(ok));
agg.seed_median_median_z = median(per_seed.median_err_z(ok));
agg.seed_median_spread_x = std(per_seed.median_err_x(ok));
agg.seed_median_spread_z = std(per_seed.median_err_z(ok));

agg.mean_lag_z = mean(per_seed.lag_z_samples(ok));
agg.mean_lag_x = mean(per_seed.lag_x_samples(ok));
agg.mean_rmse_3d = mean(per_seed.rmse_3d(ok));

fprintf('Pooled median |a_hat-a_true|/a_true:  x=%5.2f%%  z=%5.2f%%\n', ...
    agg.pool_median_x*100, agg.pool_median_z*100);
fprintf('Pooled mean   |a_hat-a_true|/a_true:  x=%5.2f%%  z=%5.2f%%\n', ...
    agg.pool_mean_x*100, agg.pool_mean_z*100);
fprintf('Pooled 90-pct:                        x=%5.2f%%  z=%5.2f%%\n', ...
    agg.pool_p90_x*100, agg.pool_p90_z*100);
fprintf('Per-seed median(median):              x=%5.2f%%  z=%5.2f%%\n', ...
    agg.seed_median_median_x*100, agg.seed_median_median_z*100);
fprintf('Lag (cross-correlation peak):         x=%+.1f  z=%+.1f samples (%.2f ms)\n', ...
    agg.mean_lag_x, agg.mean_lag_z, agg.mean_lag_z*Ts*1e3);
fprintf('3D tracking RMSE:                     %.1f nm\n', agg.mean_rmse_3d*1e3);

%% Save
out = struct();
out.config    = config;
out.per_seed  = per_seed;
out.agg       = agg;
out.pool_err_x = pool_err_x;
out.pool_err_z = pool_err_z;
out.sample_seed_data = sample_seed_data;
out.a_nom     = a_nom;
out.IIR_bias_factor = params_init.Value.ctrl.IIR_bias_factor;
out.C_dpmr_eff      = params_init.Value.ctrl.C_dpmr_eff;
out.base_seed       = base_seed;
out.n_seeds         = n_seeds;

out_dir  = fullfile(project_root, 'test_results', 'verify');
out_file = fullfile(out_dir, 'task1d_paper_benchmark_mc.mat');
save(out_file, 'out');
fprintf('\nSaved: %s\n', out_file);

%% Figure: 4 panels
FS    = 14;
LW_r  = 2.5;
LW_o  = 1.5;
C_ref = [0 0.6 0];
C_out = [0.8 0 0];
C_err = [0 0.2 0.8];

fig = figure('Position', [100, 100, 1500, 900], 'Color', 'w', 'Visible', 'off');

% Panel 1: sample seed 1 — a_hat_z vs a_true (time series)
subplot(2, 2, 1);
t1 = sample_seed_data.t;
ss1 = sample_seed_data.ss;
plot(t1(ss1), sample_seed_data.a_true_traj(ss1)/a_nom, '-', 'Color', C_ref, 'LineWidth', LW_r);
hold on;
plot(t1(ss1), sample_seed_data.a_hat_z(ss1)/a_nom, '-', 'Color', C_out, 'LineWidth', LW_o);
hold off;
xlabel('Time [s]', 'FontSize', FS, 'FontWeight', 'bold');
ylabel('a (z) / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
legend({'a_{true}', 'a_{hat}'}, 'Location', 'northoutside', ...
    'Orientation', 'horizontal', 'FontSize', FS-2);
title(sprintf('Seed %d: z-axis (dynamic)', per_seed.seed(1)), 'FontSize', FS);
set(gca, 'FontSize', FS, 'LineWidth', 1.8, 'Box', 'on');

% Panel 2: pooled error histogram
subplot(2, 2, 2);
edges = linspace(-0.5, 0.5, 80);
histogram(pool_err_z, edges, 'Normalization', 'pdf', ...
    'FaceColor', C_err, 'FaceAlpha', 0.7, 'EdgeColor', 'none');
hold on;
xline(0, 'k-', 'LineWidth', 1.5);
xline(agg.pool_median_z, '--', 'Color', C_out, 'LineWidth', 2.0, ...
    'Label', sprintf('med |err|=%.1f%%', agg.pool_median_z*100));
hold off;
xlabel('(a_{hat} - a_{true}) / a_{true}', 'FontSize', FS, 'FontWeight', 'bold');
ylabel('PDF', 'FontSize', FS, 'FontWeight', 'bold');
title(sprintf('z-axis error distribution (pooled %d seeds)', n_ok), 'FontSize', FS);
set(gca, 'FontSize', FS, 'LineWidth', 1.8, 'Box', 'on');
xlim([-0.5, 0.5]);

% Panel 3: cross-correlation (seed 1)
subplot(2, 2, 3);
plot(sample_seed_data.lags_z, sample_seed_data.xc_z, '-', 'Color', C_err, 'LineWidth', LW_r);
hold on;
xline(0, 'k-', 'Alpha', 0.4);
[~, idx_peak] = max(sample_seed_data.xc_z);
lag_peak = sample_seed_data.lags_z(idx_peak);
xline(lag_peak, '--', 'Color', C_out, 'LineWidth', 2.0, ...
    'Label', sprintf('lag=%d', lag_peak));
hold off;
xlabel('Lag [samples]', 'FontSize', FS, 'FontWeight', 'bold');
ylabel('corr(a_{hat}(t+L), a_{true}(t))', 'FontSize', FS, 'FontWeight', 'bold');
title(sprintf('Cross-corr z-axis (peak %+d = %.1f ms)', ...
    lag_peak, lag_peak*Ts*1e3), 'FontSize', FS);
set(gca, 'FontSize', FS, 'LineWidth', 1.8, 'Box', 'on');

% Panel 4: per-seed median error bar
subplot(2, 2, 4);
seeds_ok = per_seed.seed(ok);
mm_x = per_seed.median_err_x(ok) * 100;
mm_z = per_seed.median_err_z(ok) * 100;
bar_data = [mm_x(:), mm_z(:)];
b = bar(1:n_ok, bar_data);
b(1).FaceColor = [0.2 0.5 0.8];
b(2).FaceColor = [0.8 0.3 0.2];
hold on;
yline(mean(mm_x), '--', 'Color', [0.2 0.5 0.8], 'LineWidth', 1.5);
yline(mean(mm_z), '--', 'Color', [0.8 0.3 0.2], 'LineWidth', 1.5);
hold off;
xlabel('Seed index', 'FontSize', FS, 'FontWeight', 'bold');
ylabel('median |err| [%]', 'FontSize', FS, 'FontWeight', 'bold');
legend({'x-axis', 'z-axis'}, 'Location', 'northoutside', ...
    'Orientation', 'horizontal', 'FontSize', FS-2);
title(sprintf('Per-seed median err (mean x=%.1f%%, z=%.1f%%)', ...
    mean(mm_x), mean(mm_z)), 'FontSize', FS);
set(gca, 'FontSize', FS, 'LineWidth', 1.8, 'Box', 'on');

fig_dir  = fullfile(project_root, 'reference', 'for_test');
fig_path = fullfile(fig_dir, 'fig_task1d_paper_benchmark.png');
exportgraphics(fig, fig_path, 'Resolution', 150);
close(fig);
fprintf('Figure saved: %s\n', fig_path);

fprintf('\nDone.\n');
