function plot_qr_h50_figures()
%PLOT_QR_H50_FIGURES Generate fig1 (gain est) + fig2 (tracking error) at h=50.
%   Uses production frozen_correct_peraxisR11_R22_Q770 + per-axis on-the-fly
%   compute of C_dpmr/Cn/beta in calc_ctrl_params (post per-axis Bus refactor).
%   Single-seed (12345). No 5-seed +/- annotations. No +/-1sigma reference lines.
%   fig2 includes delta x_md (IIR LP) overlay; equal subplot sizes via tiledlayout.
%   Style follows test_script/run_simulation.m thesis style.

cd('C:/Users/PME406_01/Desktop/code/MotionControl_Simu_qr');
addpath('test_script');
addpath('model');
addpath('model/config');
addpath('model/controller');
addpath('model/wall_effect');
addpath('model/thermal_force');
addpath('model/trajectory');

%% Setup config (matches verify_qr_positioning_run variants_all(7))
config = user_config();
config.theta = 0; config.phi = 0; config.pz = 0;
config.h_min = 1.1 * 2.25;
config.enable_wall_effect = true;
config.h_init = 50;
config.h_bottom = 50;
config.amplitude = 0;
config.t_hold = 0;
config.frequency = 1;
config.n_cycles = 1;
config.trajectory_type = 'positioning';
config.ctrl_enable = true;
config.controller_type = 7;
config.lambda_c = 0.7;
config.a_cov = 0.005;
config.a_pd = 0.05; config.a_prd = 0.05;
config.epsilon = 0.01;
config.meas_noise_enable = true;
config.meas_noise_std = [0.00062; 0.000057; 0.00331];
config.thermal_enable = true;
config.thermal_seed = 12345;
config.Pf_init_diag = [0; 0; 1e-4; 1e-4; 0; 1e-5; 0];
config.beta = 0; config.lamdaF = 1.0;
config.T_sim = 15;
config.Qz_diag_scaling = [0; 0; 1; 0; 0; 1e-8; 0; 0; 0];
config.Rz_diag_scaling = [1.53e-3; 1.29e-5; 4.35e-2; 0.01881; 0.01881; 0.01784];

%% Run simulation
clear motion_control_law_7state trajectory_generator;
params = calc_simulation_params(config);
assignin('base', 'params', params);
assignin('base', 'p0', params.Value.common.p0);
assignin('base', 'Ts', params.Value.common.Ts);

fprintf('Running simulation (h=50, seed 12345, T_sim=15s, per-axis Cdpmr)...\n');
fprintf('Per-axis C_dpmr_eff = [%.4f, %.4f, %.4f]\n', params.Value.ctrl.C_dpmr_eff);
fprintf('Per-axis IIR_bias_factor = [%.4f, %.4f, %.4f]\n', params.Value.ctrl.IIR_bias_factor);
t_start = tic;
simOut = sim('model/system_model', 'StopTime', num2str(config.T_sim));
fprintf('Simulation done in %.1f sec\n', toc(t_start));

%% Extract data
Ts = params.Value.common.Ts;
p_d = simOut.p_d_out';
p_m = simOut.p_m_out';
ekf = simOut.ekf_out';
N = size(p_d, 2);
t = (0:N-1) * Ts;

a_hat_x = ekf(1, :);
a_hat_z = ekf(2, :);
a_m_x   = ekf(3, :);
a_m_z   = ekf(4, :);

% True a per step
w_hat = params.Value.wall.w_hat;
pz = params.Value.wall.pz;
R_p = params.Value.common.R;
a_nom = Ts / params.Value.common.gamma_N;
a_true_x = zeros(1, N);
a_true_z = zeros(1, N);
for k = 1:N
    h_val = (p_m(:, k)' * w_hat - pz) / R_p;
    if h_val > 1.001
        [c_para, c_perp] = calc_correction_functions(h_val);
    else
        c_para = 15; c_perp = 15;
    end
    a_true_x(k) = a_nom / c_para;
    a_true_z(k) = a_nom / c_perp;
end

% Tracking error per axis [nm]
err_x = (p_m(1,:) - p_d(1,:)) * 1000;
err_y = (p_m(2,:) - p_d(2,:)) * 1000;
err_z = (p_m(3,:) - p_d(3,:)) * 1000;

% Compute delta_pm and delta_pmd per writeup §2.1
a_pd = config.a_pd;
del_pm_x = zeros(1, N); del_pm_y = zeros(1, N); del_pm_z = zeros(1, N);
for k = 3:N
    del_pm_x(k) = p_d(1, k-2) - p_m(1, k);
    del_pm_y(k) = p_d(2, k-2) - p_m(2, k);
    del_pm_z(k) = p_d(3, k-2) - p_m(3, k);
end
del_pmd_x = zeros(1, N); del_pmd_y = zeros(1, N); del_pmd_z = zeros(1, N);
for k = 2:N
    del_pmd_x(k) = (1-a_pd)*del_pmd_x(k-1) + a_pd*del_pm_x(k);
    del_pmd_y(k) = (1-a_pd)*del_pmd_y(k-1) + a_pd*del_pm_y(k);
    del_pmd_z(k) = (1-a_pd)*del_pmd_z(k-1) + a_pd*del_pm_z(k);
end
% Sign: err = p_m - p_d, del_pm = p_d - p_m  =>  -del_pmd matches err sign
del_pmd_x_plot = -del_pmd_x * 1000;
del_pmd_y_plot = -del_pmd_y * 1000;
del_pmd_z_plot = -del_pmd_z * 1000;

%% Steady-state window
t_warmup = 2;
idx_ss = t >= t_warmup;

fig_dir = 'reference/for_test/fig_qr_verification_h50';
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end

%% Style (from run_simulation.m EXP style)
COL_REF = [0.0 0.6 0.0];     % green: True / Reference
COL_OUT = [0.8 0.0 0.0];     % red:   Estimated / Output
COL_ERR = [0.0 0.2 0.8];     % blue:  Error
COL_MEAS = [0.45 0.55 0.95]; % light blue: Measured (a_m raw)
COL_LP   = [0.0 0.0 0.0];    % black: del_xmd LP overlay
EXP_FS  = 18;
EXP_LFS = 14;
EXP_LW  = 2.0;
EXP_LR  = 3;
EXP_LO  = 2;
EXP_LM  = 0.7;   % thin for measured

%% FIG 1 - gain estimation (a_x, a_z) - 2x1
fig1 = figure('Position', [100, 100, 1100, 700], 'Color', 'w');
tl1 = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

% a_x
nexttile;
hold on;
plot(t(idx_ss), a_m_x(idx_ss),    '-', 'Color', COL_MEAS, 'LineWidth', EXP_LM, 'DisplayName', 'Measured');
plot(t(idx_ss), a_true_x(idx_ss), '-', 'Color', COL_REF,  'LineWidth', EXP_LR, 'DisplayName', 'True');
plot(t(idx_ss), a_hat_x(idx_ss),  '-', 'Color', COL_OUT,  'LineWidth', EXP_LO, 'DisplayName', 'Estimated');
xlim([t_warmup, max(t)]);
ylabel('a_x  (\mum/pN)', 'FontSize', EXP_FS, 'FontWeight', 'bold');
bias_x_pct = 100 * mean( (a_hat_x(idx_ss) - a_true_x(idx_ss)) ./ max(a_true_x(idx_ss), eps) );
std_x_pct  = 100 * std ( (a_hat_x(idx_ss) - a_true_x(idx_ss)) ./ max(a_true_x(idx_ss), eps) );
title(sprintf('a_x:   bias  %+.2f%%     std  %.2f%%', bias_x_pct, std_x_pct), ...
      'FontSize', EXP_FS, 'FontWeight', 'bold');
legend('Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', EXP_LFS, 'FontWeight', 'bold', 'Box', 'off');
set(gca, 'FontSize', EXP_FS, 'FontWeight', 'bold', 'LineWidth', EXP_LW, 'Box', 'on');
grid off;

% a_z
nexttile;
hold on;
plot(t(idx_ss), a_m_z(idx_ss),    '-', 'Color', COL_MEAS, 'LineWidth', EXP_LM);
plot(t(idx_ss), a_true_z(idx_ss), '-', 'Color', COL_REF,  'LineWidth', EXP_LR);
plot(t(idx_ss), a_hat_z(idx_ss),  '-', 'Color', COL_OUT,  'LineWidth', EXP_LO);
xlim([t_warmup, max(t)]);
xlabel('Time (sec)', 'FontSize', EXP_FS, 'FontWeight', 'bold');
ylabel('a_z  (\mum/pN)', 'FontSize', EXP_FS, 'FontWeight', 'bold');
bias_z_pct = 100 * mean( (a_hat_z(idx_ss) - a_true_z(idx_ss)) ./ max(a_true_z(idx_ss), eps) );
std_z_pct  = 100 * std ( (a_hat_z(idx_ss) - a_true_z(idx_ss)) ./ max(a_true_z(idx_ss), eps) );
title(sprintf('a_z:   bias  %+.2f%%     std  %.2f%%', bias_z_pct, std_z_pct), ...
      'FontSize', EXP_FS, 'FontWeight', 'bold');
set(gca, 'FontSize', EXP_FS, 'FontWeight', 'bold', 'LineWidth', EXP_LW, 'Box', 'on');
grid off;

exportgraphics(fig1, fullfile(fig_dir, 'fig1_gain_estimation.png'), 'Resolution', 150);
close(fig1);

%% FIG 2 - tracking error (3x1) with delta_x_md overlay; equal subplots
fig2 = figure('Position', [100, 100, 1100, 900], 'Color', 'w');
tl2 = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

y_max_abs = max([max(abs(err_x(idx_ss))), max(abs(err_y(idx_ss))), max(abs(err_z(idx_ss)))]);
ylim_common = [-y_max_abs, y_max_abs] * 1.05;

err_data    = {err_x,    err_y,    err_z};
del_pmd_dat = {del_pmd_x_plot, del_pmd_y_plot, del_pmd_z_plot};
labels      = {'x', 'y', 'z'};

for k = 1:3
    nexttile;
    hold on;
    plot(t(idx_ss), err_data{k}(idx_ss),    '-', 'Color', COL_ERR, 'LineWidth', 0.6, 'DisplayName', 'error');
    plot(t(idx_ss), del_pmd_dat{k}(idx_ss), '-', 'Color', COL_LP,  'LineWidth', EXP_LO, 'DisplayName', '\delta x_{md} (LP)');

    mean_v = mean(err_data{k}(idx_ss));
    std_v  = std (err_data{k}(idx_ss));
    title(sprintf('%s error:   mean  %+.2f nm     std  %.2f nm', labels{k}, mean_v, std_v), ...
          'FontSize', EXP_FS, 'FontWeight', 'bold');
    ylabel(sprintf('%s error  (nm)', labels{k}), 'FontSize', EXP_FS, 'FontWeight', 'bold');
    if k == 3
        xlabel('Time (sec)', 'FontSize', EXP_FS, 'FontWeight', 'bold');
    end
    if k == 1
        legend('Location', 'northoutside', 'Orientation', 'horizontal', ...
               'FontSize', EXP_LFS, 'FontWeight', 'bold', 'Box', 'off');
    end
    xlim([t_warmup, max(t)]);
    ylim(ylim_common);
    set(gca, 'FontSize', EXP_FS, 'FontWeight', 'bold', 'LineWidth', EXP_LW, 'Box', 'on');
    grid off;
end

exportgraphics(fig2, fullfile(fig_dir, 'fig2_tracking_error.png'), 'Resolution', 150);
close(fig2);

fprintf('\nSaved: %s/fig1_gain_estimation.png\n', fig_dir);
fprintf('Saved: %s/fig2_tracking_error.png\n', fig_dir);
fprintf('\n=== DONE plot_qr_h50_figures (seed 12345, per-axis Cdpmr) ===\n');
fprintf('Single-seed stats:\n');
fprintf('  a_hat_x: bias %+.2f%%, std %.2f%%\n', bias_x_pct, std_x_pct);
fprintf('  a_hat_z: bias %+.2f%%, std %.2f%%\n', bias_z_pct, std_z_pct);
fprintf('  err_x:   mean %+.2f nm, std %.2f nm\n', mean(err_x(idx_ss)), std(err_x(idx_ss)));
fprintf('  err_y:   mean %+.2f nm, std %.2f nm\n', mean(err_y(idx_ss)), std(err_y(idx_ss)));
fprintf('  err_z:   mean %+.2f nm, std %.2f nm\n', mean(err_z(idx_ss)), std(err_z(idx_ss)));

end
