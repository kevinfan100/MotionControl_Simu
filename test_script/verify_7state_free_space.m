%% verify_7state_free_space.m
%
% Free-space verification of 7-state EKF: Eq.12/13 + gain convergence
%
% Sweeps lambda_c = [0.4, 0.5, 0.6, 0.7, 0.8, 0.9] in free space
% (h_init=50 um, wall effect < 1%) with stationary target (amp=0).
%
% Checks:
%   1. Eq.12: Var(e) vs theory C(lc)*sigma2_deltaXT
%   2. Eq.13: a_meas = Var(e)/(C(lc)*4*kB*T) should ~ a_nom
%   3. EKF:   a_hat should converge to a_nom = Ts/gamma_N
%
% Setup: thermal=ON, measurement noise=OFF, T_sim=5 s, steady state=last 50%

clear; close all; clc;
clear motion_control_law motion_control_law_23state ...
      motion_control_law_7state trajectory_generator;

%% Setup paths

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(script_dir);
cd(project_root);

addpath(fullfile(project_root, 'model'));
addpath(fullfile(project_root, 'model', 'config'));
addpath(fullfile(project_root, 'model', 'wall_effect'));
addpath(fullfile(project_root, 'model', 'thermal_force'));
addpath(fullfile(project_root, 'model', 'trajectory'));
addpath(fullfile(project_root, 'model', 'controller'));

%% Physical constants

constants = physical_constants();
k_B     = constants.k_B;       % [pN*um/K]
T_temp  = constants.T;         % [K]
Ts      = constants.Ts;        % [sec]
gamma_N = constants.gamma_N;   % [pN*sec/um]

a_nom = Ts / gamma_N;                              % [um/pN]
sigma2_deltaXT = 4 * k_B * T_temp * Ts / gamma_N;  % [um^2]

fprintf('a_nom = %.4e um/pN\n', a_nom);
fprintf('sigma2_deltaXT = %.4e um^2\n', sigma2_deltaXT);

%% Sweep parameters

lc_list  = [0.4, 0.5, 0.6, 0.7, 0.8, 0.9];
T_sim    = 5;                       % [sec]
N_total  = round(T_sim / Ts);       % 8000 steps
ss_start = round(N_total / 2);      % steady-state from 50%

n_lc = length(lc_list);

%% Pre-allocate results

C_th    = zeros(1, n_lc);
C_dx    = zeros(1, n_lc);
Var_th  = zeros(1, n_lc);
Var_dx  = zeros(1, n_lc);
Var_x   = zeros(1, n_lc);
Var_y   = zeros(1, n_lc);
Var_z   = zeros(1, n_lc);
Var_hp_x = zeros(1, n_lc);
Var_hp_z = zeros(1, n_lc);
ahat_x  = zeros(1, n_lc);
ahat_z  = zeros(1, n_lc);

%% Load Simulink model once

model_path = fullfile(project_root, 'model', 'system_model');
load_system(model_path);
fprintf('Model loaded.\n');

%% Sweep loop

for idx = 1:n_lc
    lc = lc_list(idx);
    fprintf('\n--- [%d/%d] lambda_c = %.1f ---\n', idx, n_lc, lc);

    % Clear persistent variables in controller and trajectory
    clear motion_control_law motion_control_law_23state ...
          motion_control_law_7state trajectory_generator;

    % Build config: free space, stationary, thermal only
    config = user_config();
    config.theta     = 0;
    config.phi       = 0;
    config.pz        = 0;
    config.h_min     = 1.1 * 2.25;

    config.h_init    = 50;           % far from wall (h_bar ~ 22)
    config.amplitude = 0;            % stationary target
    config.frequency = 1;
    config.n_cycles  = 3;

    config.ctrl_enable     = true;
    config.controller_type = 7;
    config.lambda_c        = lc;

    config.thermal_enable    = true;
    config.meas_noise_enable = false;
    config.meas_noise_std    = [0; 0; 0];

    config.T_sim = T_sim;

    % Calculate parameters and assign to base workspace
    params = calc_simulation_params(config);
    assignin('base', 'params', params);
    assignin('base', 'p0', params.Value.common.p0);
    assignin('base', 'Ts', Ts);

    % Run Simulink
    simOut = sim(model_path, ...
        'StopTime', num2str(T_sim), ...
        'SaveTime', 'on', 'TimeSaveName', 'tout', ...
        'SaveOutput', 'on', 'OutputSaveName', 'yout');

    % Extract signals
    N_disc  = size(simOut.p_d_out, 1);
    p_d_log = simOut.p_d_out';      % [3 x N]
    p_m_log = simOut.p_m_out';      % [3 x N]
    ekf_log = simOut.ekf_out';      % [4 x N]

    % Tracking error [um]
    err = p_m_log - p_d_log;        % [3 x N]
    ss_idx = ss_start:N_disc;

    % Raw variance from simulation [um^2]
    Var_x(idx) = var(err(1, ss_idx));
    Var_y(idx) = var(err(2, ss_idx));
    Var_z(idx) = var(err(3, ss_idx));

    % Offline HP variance (matches controller Step[3] IIR structure)
    %   del_pmd[k] = (1-a_pd)*del_pmd[k-1] + a_pd*err[k]
    %   del_pmr[k] = err[k] - del_pmd[k]   (HP residual)
    a_pd_val = config.a_pd;
    for ax = [1, 3]   % x and z axes
        err_ax = err(ax, :);
        del_pmd_off = zeros(1, N_disc);
        for k = 2:N_disc
            del_pmd_off(k) = (1-a_pd_val)*del_pmd_off(k-1) + a_pd_val*err_ax(k);
        end
        del_pmr_off = err_ax - del_pmd_off;
        if ax == 1
            Var_hp_x(idx) = var(del_pmr_off(ss_idx));
        else
            Var_hp_z(idx) = var(del_pmr_off(ss_idx));
        end
    end

    % EKF gain estimate: steady-state mean [um/pN]
    ahat_x(idx) = mean(ekf_log(1, ss_idx));   % x-axis
    ahat_z(idx) = mean(ekf_log(2, ss_idx));   % z-axis

    % Theory: C(lc) and C_dx(lc, a_var)
    a_var = config.a_pd;
    C_th(idx)   = 2 + 1 / (1 - lc^2);
    C_dx(idx)   = calc_C_dx(lc, a_var);
    Var_th(idx) = C_th(idx) * sigma2_deltaXT;
    Var_dx(idx) = C_dx(idx) * sigma2_deltaXT;

    fprintf('  C(lc) = %.3f,  Var_theory = %.4e um^2\n', C_th(idx), Var_th(idx));
    fprintf('  Var(e): x=%.4e  y=%.4e  z=%.4e um^2\n', ...
            Var_x(idx), Var_y(idx), Var_z(idx));
    fprintf('  a_hat:  x=%.4e  z=%.4e  a_nom=%.4e um/pN\n', ...
            ahat_x(idx), ahat_z(idx), a_nom);
end

close_system(model_path, 0);

%% Derived quantities

% Eq.13: recover a from variance (using C and C_dx)
a_meas      = Var_z ./ (C_th * 4 * k_B * T_temp);     % [um/pN] (original C)
a_meas_dx   = Var_z ./ (C_dx * 4 * k_B * T_temp);     % [um/pN] (C_dx corrected)

% Percentage errors: HP variance vs C_dx theory
Var_hp_dx_err_pct = (Var_hp_z - Var_dx) ./ Var_dx * 100;
Var_raw_C_err_pct = (Var_z - Var_th) ./ Var_th * 100;
ahat_z_pct        = (ahat_z - a_nom) / a_nom * 100;

%% Print summary table

fprintf('\n');
fprintf('=============================================================================================\n');
fprintf('  7-State Free-Space Verification: Eq.12 + C_dx + Offline HP\n');
fprintf('  h_init=50 um, amp=0, thermal=ON, noise=OFF, T_sim=%g s, a_pd=%.2f\n', T_sim, a_var);
fprintf('=============================================================================================\n');
fprintf('  lc   C(lc)  C_dx   Var_th     Var_dx     Var_raw_z  Var_hp_z   raw_C%%  hp_Cdx%%  a_hat%%\n');
fprintf('---------------------------------------------------------------------------------------------\n');
for i = 1:n_lc
    fprintf('  %.1f  %5.2f  %5.2f  %.3e  %.3e  %.3e  %.3e  %+6.1f  %+6.1f  %+6.1f\n', ...
        lc_list(i), C_th(i), C_dx(i), Var_th(i), Var_dx(i), Var_z(i), Var_hp_z(i), ...
        Var_raw_C_err_pct(i), Var_hp_dx_err_pct(i), ahat_z_pct(i));
end
fprintf('=============================================================================================\n');
fprintf('  a_nom = %.4e um/pN    sigma2_deltaXT = %.4e um^2\n', a_nom, sigma2_deltaXT);
fprintf('======================================================================================\n');

% 3-axis comparison (should be similar for stationary target)
fprintf('\n  3-Axis Variance Comparison:\n');
fprintf('  lc   Var_x[um2]   Var_y[um2]   Var_z[um2]   spread\n');
fprintf('  ---------------------------------------------------\n');
for i = 1:n_lc
    v = [Var_x(i), Var_y(i), Var_z(i)];
    spread = (max(v) - min(v)) / mean(v) * 100;
    fprintf('  %.1f  %.3e  %.3e  %.3e  %.1f%%\n', ...
        lc_list(i), v(1), v(2), v(3), spread);
end

%% Figures

% Figure 1: Variance vs lambda_c (raw + HP + theory)
fig1 = figure('Position', [100 400 800 500]);
plot(lc_list, Var_th, 'k--o', 'LineWidth', 2, 'MarkerSize', 8, ...
    'DisplayName', 'C(lc) theory');
hold on;
plot(lc_list, Var_dx, 'g--d', 'LineWidth', 2, 'MarkerSize', 8, ...
    'DisplayName', sprintf('C_{dx}(lc, %.2f) theory', a_var));
plot(lc_list, Var_hp_z, 'r-s', 'LineWidth', 2, 'MarkerSize', 8, ...
    'DisplayName', 'HP Var(\delta_{pmr,z})');
hold off;
xlabel('\lambda_c');
ylabel('Variance [um^2]');
title('Eq.12: Raw vs HP Residual Variance vs \lambda_c');
legend('Location', 'northwest');
grid on;

% Figure 2: a_hat / a_nom vs lambda_c
fig2 = figure('Position', [850 400 700 500]);
plot(lc_list, ahat_x / a_nom, 'b-^', 'LineWidth', 2, 'MarkerSize', 8, ...
    'DisplayName', 'a_{hat,x} / a_{nom}');
hold on;
plot(lc_list, ahat_z / a_nom, 'r-s', 'LineWidth', 2, 'MarkerSize', 8, ...
    'DisplayName', 'a_{hat,z} / a_{nom}');
yline(1, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Ideal (a_{hat} = a_{nom})');
hold off;
xlabel('\lambda_c');
ylabel('a_{hat} / a_{nom}');
title('EKF Gain Estimation: Convergence to a_{nom}');
legend('Location', 'best');
grid on;
ylim([0 2]);

%% Save figures

out_dir = fullfile(project_root, 'test_results');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

exportgraphics(fig1, fullfile(out_dir, 'verify_7state_free_var.png'), 'Resolution', 150);
exportgraphics(fig2, fullfile(out_dir, 'verify_7state_free_ahat.png'), 'Resolution', 150);
fprintf('\nFigures saved to %s\n', out_dir);

%% ==================== Local Function ====================

function c = calc_C_dx(lc, av)
%CALC_C_DX IIR-corrected variance factor C_dx(lambda_c, a_var)
%   Accounts for IIR LP filter effect on tracking error variance.
%   When av=0: C_dx = C(lc) = 2 + 1/(1-lc^2)
    c = 2*(1-av)*(1-lc) / (1-(1-av)*lc) ...
      + (2/(2-av)) / ((1+lc)*(1-(1-av)*lc));
end
