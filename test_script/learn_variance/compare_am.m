% compare_am.m
% Compare controller a_m vs sandbox-derived a_m on the SAME simulation run.
%
% Controller path:  delta_x_m -> (LP1 + EWMA, eq17 alg) -> sigma2_dxr_hat
%                   -> invert via C_dpmr formula -> a_xm
%                   -> EKF posterior -> a_hat
% Sandbox path:     same delta_x_m -> (LP1 + EWMA, same eq17 alg)
%                   -> invert -> a_xm_sandbox  (no EKF)
%
% Reference truth:  a_z_true(t) = a_nom / c_perp(h(t)/R)

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

%% Run production simulation (ramp_descent 50->5, T=20s)
config = user_config();
config.trajectory_type   = 'ramp_descent';
config.h_init            = 50;
config.h_bottom          = 5;
config.T_sim             = 20;
config.controller_type   = 'eq17_7state';
config.iir_warmup_mode   = 'prefill';
config.thermal_enable    = true;
config.meas_noise_enable = true;
config.meas_noise_std    = [0.00062; 0.000057; 0.00331];

opts = struct('seed', 42, 'verbose', true, 'collect_diag', true);

fprintf('===== Running controller eq17 simulation =====\n');
t0 = tic;
simOut = run_pure_simulation(config, opts);
fprintf('Elapsed: %.2f s\n\n', toc(t0));

%% Extract controller signals (Z axis)
tout            = simOut.tout;
N               = length(tout);
Ts              = tout(2) - tout(1);
delta_z         = simOut.diag.delta_x_m(:, 3);
sigma2_ctrl_z   = simOut.diag.sigma2_dxr_hat(:, 3);
a_xm_ctrl_z     = simOut.diag.a_xm(:, 3);
a_hat_ctrl_z    = simOut.diag.a_hat(:, 3);

%% Constants
constants  = physical_constants();
kBT        = constants.k_B * constants.T;
gamma_N    = constants.gamma_N;
R_radius   = constants.R;
a_nom      = Ts / gamma_N;
lambda_c   = config.lambda_c;
sigma2_n_z = config.meas_noise_std(3)^2;
a_pd       = config.a_pd;     % 0.05
a_cov      = config.a_cov;    % 0.05

% Paper-aligned closed-form C_dpmr and C_n (d=2)
denom_common = 1 - (1-a_pd)*lambda_c;
% C_dpmr (existing, unchanged)
C_dpmr_bracket = 2*(1-a_pd)*(1-lambda_c)/denom_common ...
              + (2/(2-a_pd))/((1+lambda_c)*denom_common);
C_dpmr = (1-a_pd)^2 * C_dpmr_bracket;
% C_n (new 3-term closed-form, see derive_Cn_closed_v2.m)
Cn_t1 = 2 / (2 - a_pd);
Cn_t2 = 2*(1-a_pd)^2 * a_pd * (1-lambda_c) / ((2-a_pd)*denom_common);
Cn_t3 = 2*(1-lambda_c)^2 / ((2-a_pd)*(1+lambda_c)*denom_common);
C_n   = (1-a_pd)^2 * (Cn_t1 + Cn_t2 + Cn_t3);

fprintf('a_pd=%.3f, a_cov=%.3f, lambda_c=%.3f\n', a_pd, a_cov, lambda_c);
fprintf('C_dpmr=%.4f, C_n=%.4f\n\n', C_dpmr, C_n);

%% True a_z(t) — ground truth from wall correction model
h_init   = config.h_init;
h_bottom = config.h_bottom;
rate     = (h_init - h_bottom) / config.T_sim;
h_t      = max(h_init - rate * tout, h_bottom);
h_bar_t  = h_t / R_radius;
a_z_true = zeros(N, 1);
for k = 1:N
    [~, c_perp] = calc_correction_functions(max(h_bar_t(k), 1.001));
    a_z_true(k) = a_nom / c_perp;
end

%% Sandbox IIR — match eq17 algorithm exactly (LP1 + EWMA, NO LP2 subtraction)
sigma2_sandbox = zeros(N, 1);
dpmd_prev = 0;
sigma2_prev = C_dpmr * 4 * kBT * a_z_true(1) + C_n * sigma2_n_z;  % prefill
for k = 1:N
    dpmd = (1-a_pd)*dpmd_prev + a_pd*delta_z(k);
    dpmr = delta_z(k) - dpmd;
    sigma2_sandbox(k) = (1-a_cov)*sigma2_prev + a_cov*dpmr^2;
    dpmd_prev = dpmd;
    sigma2_prev = sigma2_sandbox(k);
end

% Invert sandbox sigma2 to a_xm
a_xm_sandbox = (sigma2_sandbox - C_n * sigma2_n_z) / (C_dpmr * 4 * kBT);

%% Print diff stats
fprintf('=== sigma2_dxr_hat: controller vs sandbox ===\n');
fprintf('  ctrl    mean = %.6e, std = %.6e\n', mean(sigma2_ctrl_z), std(sigma2_ctrl_z));
fprintf('  sandbox mean = %.6e, std = %.6e\n', mean(sigma2_sandbox), std(sigma2_sandbox));
fprintf('  max abs diff = %.6e\n\n', max(abs(sigma2_ctrl_z - sigma2_sandbox)));

fprintf('=== a_m: ctrl a_xm vs sandbox a_xm vs ctrl a_hat (EKF) ===\n');
fprintf('  ctrl    a_xm  mean = %+.6e  std = %.6e\n', mean(a_xm_ctrl_z),   std(a_xm_ctrl_z));
fprintf('  sandbox a_xm  mean = %+.6e  std = %.6e\n', mean(a_xm_sandbox),  std(a_xm_sandbox));
fprintf('  ctrl    a_hat mean = %+.6e  std = %.6e\n', mean(a_hat_ctrl_z),  std(a_hat_ctrl_z));
fprintf('  true    a_z   mean = %+.6e  std = %.6e\n', mean(a_z_true),      std(a_z_true));

%% Save .mat for offline inspection
save_dir = fullfile(project_root, 'test_results', 'learn_variance');
sigma2_n_s = config.meas_noise_std(:).^2;     % 3x1 [um^2]  (actual sim noise)
save(fullfile(save_dir, 'compare_am.mat'), ...
     'tout', 'delta_z', 'a_z_true', ...
     'sigma2_ctrl_z', 'sigma2_sandbox', ...
     'a_xm_ctrl_z', 'a_hat_ctrl_z', 'a_xm_sandbox', ...
     'C_dpmr', 'C_n', 'a_pd', 'a_cov', 'lambda_c', 'sigma2_n_s');

%% Plot — four lines on same axes
set(groot, 'defaultTextInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex');

COL_TRUE   = [0.0 0.6 0.0];           % green -- ground truth
COL_CTRL   = [0.85 0.0 0.0];          % red   -- controller a_xm (raw inversion)
COL_SAND   = [0.0 0.2 0.8];           % blue  -- sandbox a_xm

fig = figure('Position', [50 50 1500 900], 'Color', 'w');

% Plot controller first (red, solid). Sandbox dashed on top — dashes leave
% gaps where the red shows through, so both lines remain visible.
p_ctrl = plot(tout, a_xm_ctrl_z,   '-',  'Color', COL_CTRL, 'LineWidth', 2.2, ...
              'DisplayName', '$a_{xm}$ controller'); hold on;

p_sand = plot(tout, a_xm_sandbox,  '--', 'Color', COL_SAND, 'LineWidth', 1.6, ...
              'DisplayName', '$a_{xm}$ sandbox (offline)');

p_true = plot(tout, a_z_true,      '-',  'Color', COL_TRUE, 'LineWidth', 4, ...
              'DisplayName', '$a_z$ true (wall model)');

set(gca, 'FontSize', 24, 'FontWeight', 'bold', 'LineWidth', 2.5);
xlabel('Time [sec]', 'FontSize', 28, 'FontWeight', 'bold');
ylabel('$a_z$ [um/pN]', 'FontSize', 28, 'FontWeight', 'bold');
grid on; box on;
xlim([0 config.T_sim]);

legend([p_ctrl p_sand p_true], 'Location', 'northoutside', ...
       'Orientation', 'horizontal', 'NumColumns', 3, ...
       'FontSize', 20, 'FontWeight', 'bold', ...
       'Interpreter', 'latex', 'Box', 'on');

out_path = fullfile(save_dir, 'compare_am_z.png');
exportgraphics(fig, out_path, 'Resolution', 150);
fprintf('\nSaved: %s\n', out_path);

set(groot, 'defaultTextInterpreter','remove');
set(groot, 'defaultLegendInterpreter','remove');
set(groot, 'defaultAxesTickLabelInterpreter','remove');
