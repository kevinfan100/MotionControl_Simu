%% phase2_paper_benchmark.m — Run paper-like scenario for precision comparison
%
% Paper Fig 5 setup:
%   - 1 Hz sinusoidal tracking in 3D
%   - z: ramp from h_init down to h_min ≈ 2.5 um (near wall)
%   - x, y: 1 Hz sine, amplitude 9 um (paper says 9 um)
%   - T_sim ~ 10 seconds
%
% Metrics to extract:
%   - tracking error RMSE per axis (mean, std)
%   - std of del_pmr per axis (for comparison with paper Fig 6)
%   - a_hat convergence (via EKF)
%
% Gate: if 3D RMSE much larger than paper's "close to random error with zero
% mean", Phase 3 is needed.

clear; close all; clc;
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

fprintf('===== phase2_paper_benchmark =====\n\n');

constants = physical_constants();
k_B = constants.k_B; T_temp = constants.T;
Ts = constants.Ts; gamma_N = constants.gamma_N;

% Paper setup: 1 Hz oscillation, z descends to wall
config = user_config();
config.h_init    = 50;            % start far from wall
config.h_bottom  = 2.5;           % near-wall trough
config.amplitude = 2.5;           % small oscillation
config.frequency = 1;             % 1 Hz
config.n_cycles  = 10;
config.t_hold    = 0.5;
config.enable_wall_effect = true;
config.trajectory_type = 'osc';
config.ctrl_enable = true;
config.controller_type = 7;
config.lambda_c = 0.4;            % paper uses λc = 0.4
config.thermal_enable = true;
config.meas_noise_enable = false; % noiseless first
config.meas_noise_std = [0; 0; 0];
config.T_sim = 12;

params = calc_simulation_params(config);
C_dpmr_used = params.Value.ctrl.C_dpmr_eff;
fprintf('lc = %.2f, C_dpmr_eff = %.4f\n', config.lambda_c, C_dpmr_used);

assignin('base', 'params', params);
assignin('base', 'p0', params.Value.common.p0);
assignin('base', 'Ts', Ts);

model_path = fullfile(project_root, 'model', 'system_model');
rng(77);
fprintf('Running Simulink (T_sim=%.1f)...', config.T_sim);
t0 = tic;
simOut = sim(model_path, 'StopTime', num2str(config.T_sim), ...
    'SaveTime', 'on', 'TimeSaveName', 'tout', ...
    'SaveOutput', 'on', 'OutputSaveName', 'yout');
fprintf(' done (%.1f sec)\n\n', toc(t0));

p_d = simOut.p_d_out';
p_m = simOut.p_m_out';
ekf = simOut.ekf_out';
N = size(p_d, 2);
t = (0:N-1) * Ts;

% Tracking error (no delay): p_m - p_d
err = p_m - p_d;

% Controller's delayed tracking error
del_pm = zeros(3, N);
for k = 3:N, del_pm(:, k) = p_d(:, k-2) - p_m(:, k); end

% Skip initial transient (first 2s for EKF to converge)
ss = round(2/Ts):N;
fprintf('Steady-state analysis window: t = %.1f to %.1f s (%d samples)\n\n', ...
        t(ss(1)), t(ss(end)), length(ss));

%% Tracking error statistics
fprintf('--- Tracking error statistics (paper Fig 5 comparison) ---\n');
for ax = 1:3
    ax_name = {'x', 'y', 'z'};
    err_ax = err(ax, ss);
    fprintf('  %s: mean = %+.4e um, std = %.4e um (%.1f nm), max|err| = %.4e um\n', ...
        ax_name{ax}, mean(err_ax), std(err_ax), std(err_ax)*1e3, max(abs(err_ax)));
end

% 3D RMSE
err_3d = sqrt(sum(err(:, ss).^2, 1));
fprintf('\n  3D RMSE = %.4e um = %.1f nm\n', mean(err_3d), mean(err_3d)*1e3);
fprintf('  3D max  = %.4e um = %.1f nm\n', max(err_3d), max(err_3d)*1e3);

%% del_pm and del_pmr statistics (for paper Fig 6)
fprintf('\n--- del_pm / del_pmr statistics (for paper Fig 6 comparison) ---\n');
a_pd = config.a_pd;
del_pmd = zeros(3, N); del_pmr = zeros(3, N);
for k = 2:N
    del_pmd(:, k) = (1-a_pd)*del_pmd(:, k-1) + a_pd*del_pm(:, k);
    del_pmr(:, k) = del_pm(:, k) - del_pmd(:, k);
end
for ax = 1:3
    ax_name = {'x', 'y', 'z'};
    fprintf('  %s: std(del_pm) = %.4e um (%.1f nm)   std(del_pmr) = %.4e um (%.1f nm)\n', ...
        ax_name{ax}, std(del_pm(ax,ss)), std(del_pm(ax,ss))*1e3, ...
        std(del_pmr(ax,ss)), std(del_pmr(ax,ss))*1e3);
end

%% EKF gain tracking
fprintf('\n--- EKF a_hat tracking (vs true time-varying a from wall model) ---\n');
addpath(fullfile(project_root, 'model', 'wall_effect'));

% Compute true a per step from p_m position (wall distance)
w_hat = params.Value.wall.w_hat;
R_probe = constants.R;
pz = params.Value.wall.pz;
h_k = p_m' * w_hat - pz;  % N x 1
a_true_traj = zeros(N, 1);
for k = 1:N
    h_bar = max(h_k(k) / R_probe, 1.001);
    [cpara, cperp] = calc_correction_functions(h_bar);
    gamma_k = gamma_N * cperp;    % assume normal direction ~ z
    a_true_traj(k) = Ts / gamma_k;
end

% a_hat tracking error
a_hat_x_err = abs(ekf(1, ss)' - a_true_traj(ss)) ./ a_true_traj(ss);
a_hat_z_err = abs(ekf(2, ss)' - a_true_traj(ss)) ./ a_true_traj(ss);

fprintf('  |a_hat_x - a_true| / a_true:  median = %.2f%%, mean = %.2f%%, max = %.2f%%\n', ...
    100*median(a_hat_x_err), 100*mean(a_hat_x_err), 100*max(a_hat_x_err));
fprintf('  |a_hat_z - a_true| / a_true:  median = %.2f%%, mean = %.2f%%, max = %.2f%%\n', ...
    100*median(a_hat_z_err), 100*mean(a_hat_z_err), 100*max(a_hat_z_err));

%% Save results
results = struct();
results.err       = err;
results.del_pm    = del_pm;
results.del_pmr   = del_pmr;
results.ekf       = ekf;
results.a_true    = a_true_traj;
results.h         = h_k;
results.t         = t;
results.ss        = ss;
results.config    = config;
results.C_dpmr    = C_dpmr_used;
for ax = 1:3
    results.err_mean(ax)    = mean(err(ax, ss));
    results.err_std(ax)     = std(err(ax, ss));
    results.err_max(ax)     = max(abs(err(ax, ss)));
    results.del_pm_std(ax)  = std(del_pm(ax, ss));
    results.del_pmr_std(ax) = std(del_pmr(ax, ss));
end
results.rmse_3d_mean = mean(err_3d);
results.a_hat_x_err_median = median(a_hat_x_err);
results.a_hat_z_err_median = median(a_hat_z_err);

out_dir = fullfile(project_root, 'test_results', 'verify');
save(fullfile(out_dir, 'phase2_paper_benchmark.mat'), 'results');
fprintf('\nSaved: phase2_paper_benchmark.mat\n');

%% Figure: tracking error time series
fig = figure('Position', [100 100 1400 900]);
ax_names = {'x', 'y', 'z'};
for ax = 1:3
    subplot(3, 2, 2*ax-1);
    plot(t, err(ax, :)*1e3, 'b-', 'LineWidth', 1);
    hold on;
    yline(0, 'k:', 'LineWidth', 1);
    hold off;
    xlabel('Time [s]'); ylabel([ax_names{ax} ' error [nm]']);
    title(sprintf('%s tracking error (std=%.1f nm)', ax_names{ax}, results.err_std(ax)*1e3));
    set(gca, 'FontSize', 11);

    subplot(3, 2, 2*ax);
    histogram(err(ax, ss)*1e3, 40, 'Normalization', 'pdf');
    xlabel([ax_names{ax} ' error [nm]']); ylabel('PDF');
    title(sprintf('%s distribution (mean=%.1f nm)', ax_names{ax}, results.err_mean(ax)*1e3));
    set(gca, 'FontSize', 11);
end
fig_dir = fullfile(project_root, 'reference', 'for_test');
saveas(fig, fullfile(fig_dir, 'fig_phase2_paper_benchmark.png'));
fprintf('Figure saved: fig_phase2_paper_benchmark.png\n');

fprintf('\nDone.\n');
