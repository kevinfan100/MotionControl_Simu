%% verify_sigma_recursion.m - Verify Time-Varying Variance Recursion
%
% Verifies the 4x4 Sigma recursion (R2) for Controller 4 (3-state KF):
%   Sigma[k+1] = A_cl[k] * Sigma[k] * A_cl[k]' + sigma2_deltaXT[k] * b*b'
%
% Phase 1: Stationary (no wall effect, positioning) — sanity check
%          V_theory should converge to C_dpm * sigma2_deltaXT
%
% Phase 2: Time-varying (wall effect ON, oscillation trajectory)
%          V_theory[k] vs cycle-averaged empirical variance
%
% Math reference: reference/for_test/temp_variance_recursion.tex

clear; close all; clc;
clear motion_control_law_4 trajectory_generator calc_thermal_force;

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(script_dir);
cd(project_root);

addpath(fullfile(project_root, 'model'));
addpath(fullfile(project_root, 'model', 'config'));
addpath(fullfile(project_root, 'model', 'wall_effect'));
addpath(fullfile(project_root, 'model', 'thermal_force'));
addpath(fullfile(project_root, 'model', 'trajectory'));
addpath(fullfile(project_root, 'model', 'controller'));

%% ===== Configuration =====

lc       = 0.7;
kf_R_val = 0.05;    % R / sigma2_deltaXT (rho for KF design)
rerun    = true;

% Output directory
out_dir = fullfile(project_root, 'test_results', 'verify');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

% Physical constants
c = physical_constants();
gamma_N        = c.gamma_N;
Ts             = c.Ts;
k_B            = c.k_B;
T_temp         = c.T;
R_probe        = c.R;
sigma2_dXT_nom = 4 * k_B * T_temp * Ts / gamma_N;

% KF design parameter
R_kf = kf_R_val * sigma2_dXT_nom;
rho  = kf_R_val;

% Figure style
AXIS_LW   = 3.0;
FONT_SIZE = 32;
LEGEND_FS = 24;
LINE_REF  = 4;
LINE_OUT  = 3;

%% ===== Helper: Run Sigma Recursion =====

function [V_theory, Sigma_all] = run_sigma_recursion(L_history, sigma2_dXT_k, lc_val)
%RUN_SIGMA_RECURSION Compute 4x4 variance recursion offline
%   L_history: [N x 3] Kalman gains per step
%   sigma2_dXT_k: [N x 1] time-varying thermal displacement variance
%   lc_val: closed-loop pole

    Fe = [0, 1, 0; 0, 0, 1; 0, 0, 1];
    H_obs = [1, 0, 0];
    b = [-1; 0; 0; -1];

    N = size(L_history, 1);
    V_theory = zeros(N, 1);
    Sigma_all = zeros(4, 4, N);
    Sigma = zeros(4, 4);

    for k = 1:N
        L_k = L_history(k, :)';
        A_cl = [lc_val, 0, 0, 1-lc_val;
                zeros(3, 1), Fe - L_k * H_obs];
        Sigma = A_cl * Sigma * A_cl' + sigma2_dXT_k(k) * (b * b');
        Sigma = 0.5 * (Sigma + Sigma');
        V_theory(k) = Sigma(1, 1);
        Sigma_all(:, :, k) = Sigma;
    end
end

%% ===== Helper: Reconstruct del_pm =====

function del_pm = reconstruct_del_pm(p_d, p_m)
    N = size(p_d, 1);
    del_pm = zeros(N, 3);
    for k = 3:N
        del_pm(k, :) = p_d(k-2, :) - p_m(k, :);
    end
end

%% ################################################################
%  PHASE 1: Stationary Verification
%  ################################################################

fprintf('===== Phase 1: Stationary Verification =====\n');

T_sim_p1 = 20;
mat_p1 = fullfile(out_dir, 'sigma_recursion_phase1.mat');

%% Phase 1: Run simulation

if rerun
    clear motion_control_law_4 trajectory_generator calc_thermal_force;
    rng(42);

    config = user_config();
    config.trajectory_type    = 'positioning';
    config.h_init             = 50;
    config.t_hold             = 0;
    config.enable_wall_effect = false;
    config.ctrl_enable        = true;
    config.controller_type    = 4;
    config.lambda_c           = lc;
    config.kf_R               = R_kf;
    config.meas_noise_enable  = false;
    config.meas_noise_std     = [0; 0; 0];
    config.thermal_enable     = true;
    config.T_sim              = T_sim_p1;

    params = calc_simulation_params(config);
    p0 = params.Value.common.p0;

    assignin('base', 'params', params);
    assignin('base', 'p0', p0);
    assignin('base', 'Ts', Ts);

    fprintf('  Running stationary simulation (T=%ds)...', T_sim_p1);
    simOut = sim(fullfile(project_root, 'model', 'system_model'), ...
        'StopTime', num2str(T_sim_p1), ...
        'SaveTime', 'on', 'TimeSaveName', 'tout', ...
        'SaveOutput', 'on', 'OutputSaveName', 'yout');
    fprintf(' done.\n');

    p1.p_d = simOut.p_d_out;
    p1.p_m = simOut.p_m_out;
    p1.ekf = simOut.ekf_out;
    p1.tout = simOut.tout;
    save(mat_p1, '-struct', 'p1');
else
    p1 = load(mat_p1);
end

%% Phase 1: Sigma recursion

N_p1 = size(p1.p_m, 1);
L_hist_p1 = p1.ekf(:, 1:3);     % [L1, L2, L3]
sigma2_dXT_p1 = sigma2_dXT_nom * ones(N_p1, 1);  % constant (no wall effect)

[V_theory_p1, ~] = run_sigma_recursion(L_hist_p1, sigma2_dXT_p1, lc);

%% Phase 1: Empirical variance (time average, steady state)

del_pm_p1 = reconstruct_del_pm(p1.p_d, p1.p_m);
ss_start = round(N_p1 / 2);
V_empirical_p1 = var(del_pm_p1(ss_start:end, 3));

%% Phase 1: Theory (analytic C_dpm for Controller 4)

% rho -> le -> C_dpm via Lyapunov
a_norm = (1 + sqrt(1 + 4*rho)) / 2;
L_ss = 1 / a_norm;
le = 1 - L_ss;

% 4-state Lyapunov for Controller 4 (KF observer)
Fe = [0, 1, 0; 0, 0, 1; 0, 0, 1];
H_obs = [1, 0, 0];
L_ss_vec = L_ss * [1; 1; 1];
A_cl_ss = [lc, 0, 0, 1-lc;
           zeros(3, 1), Fe - L_ss_vec * H_obs];
b_vec = [-1; 0; 0; -1];
Sigma_lyap = dlyap(A_cl_ss, b_vec * b_vec');
C_dpm_analytic = Sigma_lyap(1, 1);   % already normalized (B = b, not b*sqrt(sigma2))

%% Phase 1: Report

V_theory_converged = mean(V_theory_p1(ss_start:end));
C_dpm_recursion = V_theory_converged / sigma2_dXT_nom;
C_dpm_empirical = V_empirical_p1 / sigma2_dXT_nom;

fprintf('\n  Phase 1 Results (lc=%.1f, rho=%.3f, le=%.4f):\n', lc, rho, le);
fprintf('    C_dpm (analytic Lyapunov):  %.6f\n', C_dpm_analytic);
fprintf('    C_dpm (recursion converged): %.6f\n', C_dpm_recursion);
fprintf('    C_dpm (simulation):          %.6f\n', C_dpm_empirical);
fprintf('    Recursion vs analytic:       %.4f%%\n', ...
    100 * abs(C_dpm_recursion - C_dpm_analytic) / C_dpm_analytic);
fprintf('    Simulation vs analytic:      %.4f%%\n', ...
    100 * abs(C_dpm_empirical - C_dpm_analytic) / C_dpm_analytic);

%% Phase 1: Figure

t_p1 = (0:N_p1-1)' * Ts;

fig1 = figure('Position', [100, 100, 1200, 600]);
plot(t_p1, V_theory_p1 / sigma2_dXT_nom, 'b-', 'LineWidth', LINE_OUT);
hold on;
yline(C_dpm_analytic, 'r--', 'LineWidth', LINE_REF);
yline(C_dpm_empirical, 'k:', 'LineWidth', LINE_OUT);
hold off;
xlabel('Time [s]');
ylabel('V / \sigma^2_{\Delta,0}');
legend({'Sigma recursion', ...
        sprintf('Lyapunov C_{dpm} = %.4f', C_dpm_analytic), ...
        sprintf('Simulation = %.4f', C_dpm_empirical)}, ...
       'Location', 'northoutside', 'FontSize', LEGEND_FS);
set(gca, 'FontSize', FONT_SIZE, 'LineWidth', AXIS_LW);
xlim([0, T_sim_p1]);

saveas(fig1, fullfile(project_root, 'reference', 'for_test', ...
    'fig_sigma_recursion_phase1.png'));
fprintf('  Phase 1 figure saved.\n');

%% ################################################################
%  PHASE 2: Time-Varying Verification (Wall Effect + Trajectory)
%  ################################################################

fprintf('\n===== Phase 2: Time-Varying Verification =====\n');

T_sim_p2 = 60;
freq = 1;   % Hz
mat_p2 = fullfile(out_dir, 'sigma_recursion_phase2.mat');

%% Phase 2: Run simulation

if rerun
    clear motion_control_law_4 trajectory_generator calc_thermal_force;
    rng(100);

    config = user_config();
    config.trajectory_type    = 'osc';
    config.h_init             = 20;
    config.h_bottom           = 2.5;
    config.amplitude          = 2.5;
    config.frequency          = freq;
    config.n_cycles           = 55;   % enough for 60s at 1Hz
    config.t_hold             = 0.5;
    config.enable_wall_effect = true;
    config.ctrl_enable        = true;
    config.controller_type    = 4;
    config.lambda_c           = lc;
    config.kf_R               = R_kf;
    config.meas_noise_enable  = false;
    config.meas_noise_std     = [0; 0; 0];
    config.thermal_enable     = true;
    config.T_sim              = T_sim_p2;

    params = calc_simulation_params(config);
    p0 = params.Value.common.p0;
    w_hat = params.Value.wall.w_hat;
    pz    = params.Value.wall.pz;

    assignin('base', 'params', params);
    assignin('base', 'p0', p0);
    assignin('base', 'Ts', Ts);

    fprintf('  Running time-varying simulation (T=%ds)...', T_sim_p2);
    simOut = sim(fullfile(project_root, 'model', 'system_model'), ...
        'StopTime', num2str(T_sim_p2), ...
        'SaveTime', 'on', 'TimeSaveName', 'tout', ...
        'SaveOutput', 'on', 'OutputSaveName', 'yout');
    fprintf(' done.\n');

    p2.p_d   = simOut.p_d_out;
    p2.p_m   = simOut.p_m_out;
    p2.ekf   = simOut.ekf_out;
    p2.tout  = simOut.tout;
    p2.w_hat = w_hat;
    p2.pz    = pz;
    save(mat_p2, '-struct', 'p2');
else
    p2 = load(mat_p2);
    w_hat = p2.w_hat;
    pz    = p2.pz;
end

%% Phase 2: Reconstruct gamma(h[k]) and sigma2_deltaXT[k]

N_p2 = size(p2.p_m, 1);

% h[k] = distance from wall center to probe center along w_hat
h_k = p2.p_m * w_hat - pz;              % [N x 1] um
h_bar_k = h_k / R_probe;                 % dimensionless

% Clamp h_bar to avoid singularity
h_bar_k = max(h_bar_k, 1.001);

c_perp_k = zeros(N_p2, 1);
for ki = 1:N_p2
    [~, c_perp_k(ki)] = calc_correction_functions(h_bar_k(ki));
end
gamma_k = gamma_N * c_perp_k;            % [N x 1]
sigma2_dXT_k = 4 * k_B * T_temp * Ts ./ gamma_k;  % [N x 1]

%% Phase 2: Sigma recursion

L_hist_p2 = p2.ekf(:, 1:3);
[V_theory_p2, ~] = run_sigma_recursion(L_hist_p2, sigma2_dXT_k, lc);

%% Phase 2: Cycle averaging for empirical variance

del_pm_p2 = reconstruct_del_pm(p2.p_d, p2.p_m);
del_pm_z = del_pm_p2(:, 3);  % z-axis (wall normal direction)

samples_per_cycle = round(1 / (freq * Ts));
n_total_cycles = floor(N_p2 / samples_per_cycle);
n_discard = 5;  % discard first 5 cycles (transient)
n_usable = n_total_cycles - n_discard;

fprintf('  Total samples: %d, per cycle: %d, usable cycles: %d\n', ...
    N_p2, samples_per_cycle, n_usable);

% Compute del_pm^2 averaged across usable cycles for each phase
V_cycle_avg = zeros(samples_per_cycle, 1);
for phase = 1:samples_per_cycle
    idx = n_discard * samples_per_cycle + phase + ...
          (0:n_usable-1)' * samples_per_cycle;
    idx = idx(idx <= N_p2);
    V_cycle_avg(phase) = mean(del_pm_z(idx).^2);
end

% Map V_theory to same cycle-averaged basis (use last usable cycle range)
% Take one representative cycle of V_theory after transient
ref_cycle_start = n_discard * samples_per_cycle + 1;
ref_cycle_end   = ref_cycle_start + samples_per_cycle - 1;
if ref_cycle_end <= N_p2
    V_theory_cycle = V_theory_p2(ref_cycle_start:ref_cycle_end);
else
    V_theory_cycle = V_theory_p2(end-samples_per_cycle+1:end);
end

%% Phase 2: Report

rel_err = abs(V_theory_cycle - V_cycle_avg) ./ V_cycle_avg * 100;
fprintf('\n  Phase 2 Results:\n');
fprintf('    Mean relative error: %.2f%%\n', mean(rel_err));
fprintf('    Max relative error:  %.2f%%\n', max(rel_err));
fprintf('    Median relative error: %.2f%%\n', median(rel_err));

%% Phase 2: Figures

t_p2 = (0:N_p2-1)' * Ts;
t_cycle = (0:samples_per_cycle-1)' * Ts;

% --- Figure 2: Time series with h[k] ---
fig2 = figure('Position', [100, 100, 1600, 900]);

subplot(2, 1, 1);
plot(t_p2, h_k, 'k-', 'LineWidth', 1.5);
ylabel('h [um]');
xlabel('Time [s]');
set(gca, 'FontSize', FONT_SIZE, 'LineWidth', AXIS_LW);
xlim([0, T_sim_p2]);

subplot(2, 1, 2);
plot(t_p2, V_theory_p2, 'b-', 'LineWidth', 1.5);
hold on;
plot(t_p2, sigma2_dXT_k, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5);
hold off;
ylabel('Variance [um^2]');
xlabel('Time [s]');
legend({'\Sigma_{11} (recursion)', '\sigma^2_{\Delta}[k]'}, ...
       'Location', 'northoutside', 'FontSize', LEGEND_FS);
set(gca, 'FontSize', FONT_SIZE, 'LineWidth', AXIS_LW);
xlim([0, T_sim_p2]);

saveas(fig2, fullfile(project_root, 'reference', 'for_test', ...
    'fig_sigma_recursion_timeseries.png'));

% --- Figure 3: One cycle comparison ---
fig3 = figure('Position', [100, 100, 1200, 600]);
plot(t_cycle * 1000, V_theory_cycle, 'b-', 'LineWidth', LINE_OUT);
hold on;
plot(t_cycle * 1000, V_cycle_avg, 'r--', 'LineWidth', LINE_OUT);
hold off;
xlabel('Phase within cycle [ms]');
ylabel('Variance [um^2]');
legend({'V_{theory} (recursion)', 'V_{empirical} (cycle avg, N=55)'}, ...
       'Location', 'northoutside', 'FontSize', LEGEND_FS);
set(gca, 'FontSize', FONT_SIZE, 'LineWidth', AXIS_LW);

saveas(fig3, fullfile(project_root, 'reference', 'for_test', ...
    'fig_sigma_recursion_cycle.png'));

% --- Figure 4: Scatter ---
fig4 = figure('Position', [100, 100, 800, 800]);
scatter(V_cycle_avg * 1e6, V_theory_cycle * 1e6, 30, 'b', 'filled');
hold on;
lims = [min([V_cycle_avg; V_theory_cycle]), max([V_cycle_avg; V_theory_cycle])] * 1e6;
plot(lims, lims, 'k--', 'LineWidth', 2);
hold off;
xlabel('V_{empirical} [um^2 x10^{-6}]');
ylabel('V_{theory} [um^2 x10^{-6}]');
set(gca, 'FontSize', FONT_SIZE, 'LineWidth', AXIS_LW);
axis equal;

% R^2
SS_res = sum((V_theory_cycle - V_cycle_avg).^2);
SS_tot = sum((V_cycle_avg - mean(V_cycle_avg)).^2);
R2 = 1 - SS_res / SS_tot;
title(sprintf('R^2 = %.4f', R2), 'FontSize', LEGEND_FS);

saveas(fig4, fullfile(project_root, 'reference', 'for_test', ...
    'fig_sigma_recursion_scatter.png'));

fprintf('  Phase 2 figures saved.\n');

%% ===== Save results =====

results.lc = lc;
results.rho = rho;
results.kf_R = R_kf;
results.sigma2_dXT_nom = sigma2_dXT_nom;

% Phase 1
results.p1.C_dpm_analytic  = C_dpm_analytic;
results.p1.C_dpm_recursion = C_dpm_recursion;
results.p1.C_dpm_empirical = C_dpm_empirical;

% Phase 2
results.p2.V_theory_cycle = V_theory_cycle;
results.p2.V_cycle_avg    = V_cycle_avg;
results.p2.mean_rel_err   = mean(rel_err);
results.p2.R2             = R2;

save(fullfile(out_dir, 'sigma_recursion_results.mat'), 'results');
fprintf('\nResults saved to test_results/verify/sigma_recursion_results.mat\n');
fprintf('Done.\n');
