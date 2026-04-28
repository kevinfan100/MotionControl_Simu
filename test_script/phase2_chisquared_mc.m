%% phase2_chisquared_mc.m — Characterize chi-squared statistical noise floor
%
% Runs ONE long 7-state EKF simulation in free space (fixed a_nom), then
% splits the steady-state portion into non-overlapping windows to estimate
% the statistical distribution of the Eq.13 a_m estimate.
%
% Compares with theoretical chi-squared prediction:
%   std(a_m) / mean(a_m) ~ sqrt(2/N_eff)
% where N_eff is the effective window length of the IIR variance smoother
% (~ 1/a_cov).

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

fprintf('===== phase2_chisquared_mc =====\n\n');

constants = physical_constants();
k_B = constants.k_B; T_temp = constants.T;
Ts = constants.Ts; gamma_N = constants.gamma_N;
a_nom = Ts / gamma_N;
sigma2_dXT = 4 * k_B * T_temp * Ts / gamma_N;

lc = 0.7;
T_sim = 30;    % long run for good statistics
fprintf('lc = %.2f, T_sim = %.1f s\n', lc, T_sim);
fprintf('a_nom = %.4e\n\n', a_nom);

%% Run one long simulation (free space, positioning, fixed a)
clear motion_control_law motion_control_law_7state ...
      motion_control_law_23state trajectory_generator calc_thermal_force;
config = user_config();
config.h_init = 50;
config.amplitude = 0;
config.enable_wall_effect = false;
config.trajectory_type = 'positioning';
config.t_hold = 0;
config.n_cycles = 1;
config.frequency = 1;
config.ctrl_enable = true;
config.controller_type = 7;
config.lambda_c = lc;
config.thermal_enable = true;
config.meas_noise_enable = false;
config.meas_noise_std = [0; 0; 0];
config.T_sim = T_sim;

params = calc_simulation_params(config);
assignin('base', 'params', params);
assignin('base', 'p0', params.Value.common.p0);
assignin('base', 'Ts', Ts);

model_path = fullfile(project_root, 'model', 'system_model');
rng(12345);
fprintf('Running Simulink (T_sim=%.1f)...', T_sim);
t0 = tic;
simOut = sim(model_path, 'StopTime', num2str(T_sim), ...
    'SaveTime', 'on', 'TimeSaveName', 'tout', ...
    'SaveOutput', 'on', 'OutputSaveName', 'yout');
fprintf(' done (%.1f sec)\n\n', toc(t0));

p_d = simOut.p_d_out';
p_m = simOut.p_m_out';
ekf = simOut.ekf_out';
N = size(p_d, 2);
t = (0:N-1) * Ts;

% del_pm reconstruction
del_pm = zeros(3, N);
for k = 3:N, del_pm(:, k) = p_d(:, k-2) - p_m(:, k); end

% Offline IIR (same as controller)
a_pd = config.a_pd;
a_cov = config.a_cov;
a_prd = config.a_prd;
del_pmd = zeros(3, N); del_pmr = zeros(3, N);
del_pmrd = zeros(3, N); del_pmr2_avg = zeros(3, N);
for k = 2:N
    del_pmd(:,k)    = (1-a_pd)*del_pmd(:,k-1) + a_pd*del_pm(:,k);
    del_pmr(:,k)    = del_pm(:,k) - del_pmd(:,k);
    del_pmrd(:,k)   = (1-a_prd)*del_pmrd(:,k-1) + a_prd*del_pmr(:,k);
    del_pmr2_avg(:,k) = (1-a_cov)*del_pmr2_avg(:,k-1) + a_cov*del_pmr(:,k).^2;
end
Var_est_IIR = max(del_pmr2_avg - del_pmrd.^2, 0);  % IIR-smoothed running variance

C_dpmr_eff = params.Value.ctrl.C_dpmr_eff;
den = C_dpmr_eff * 4 * k_B * T_temp;
a_m_IIR = max(Var_est_IIR / den, 0);  % per-step Eq.13 estimate

%% Steady state window: after 10s warmup
ss_start = round(10 / Ts);
ss = ss_start:N;
fprintf('Steady-state window: samples %d..%d (%d samples, %.1f s)\n\n', ...
        ss(1), ss(end), length(ss), length(ss)*Ts);

%% Experiment 2A: chi-squared floor — a_m distribution statistics
fprintf('--- Experiment 2A: a_m statistical distribution ---\n');

for ax = 1:3
    axname = {'x','y','z'};
    m = mean(a_m_IIR(ax, ss));
    s = std(a_m_IIR(ax, ss));
    rel_std = s / m * 100;
    bias = (m - a_nom) / a_nom * 100;
    fprintf('  %s: mean(a_m) = %.4e (%.2f%% of a_nom, bias %+5.2f%%)\n', ...
            axname{ax}, m, 100*m/a_nom, bias);
    fprintf('         std(a_m) = %.4e (%.2f%% rel, chi-sq theory: %.2f%%)\n', ...
            s, rel_std, 100*sqrt(2 * a_cov));
end

%% Theoretical chi-squared prediction
% IIR with coefficient a_cov has effective window N_eff ~ 1/a_cov = 20
% For a chi-squared(N_eff) estimator of variance:
%   std(Var_est) / mean(Var_est) ~ sqrt(2/N_eff)
%   For a_cov = 0.05 -> sqrt(2*0.05) = sqrt(0.1) = 0.316 = 31.6%
N_eff_iir = 1 / a_cov;
chi_sq_rel_std_pct = sqrt(2 / N_eff_iir) * 100;
fprintf('\nTheoretical chi-squared floor (IIR):\n');
fprintf('  a_cov = %.3f -> N_eff = %.0f -> rel std = %.1f%%\n', ...
        a_cov, N_eff_iir, chi_sq_rel_std_pct);

%% Experiment 2B: Sample variance vs true variance over windows
fprintf('\n--- Experiment 2B: Window-based sample variance ---\n');

win_lengths = [100, 500, 1000, 2000, 5000];  % samples per window
for wl = win_lengths
    % Non-overlapping windows in ss
    n_win = floor(length(ss) / wl);
    if n_win < 2, continue; end
    var_samples = zeros(n_win, 3);
    for w = 1:n_win
        idx = ss((w-1)*wl + 1 : w*wl);
        for ax = 1:3
            var_samples(w, ax) = var(del_pmr(ax, idx));
        end
    end
    mean_var = mean(var_samples, 1);
    std_var  = std(var_samples, 0, 1);
    rel_std_z = std_var(3) / mean_var(3) * 100;
    % Expected chi-squared(wl) relative std = sqrt(2/(wl-1))
    expected_rel_std = sqrt(2/(wl-1)) * 100;
    fprintf('  win=%5d samples (%.2fs): rel_std(z) = %5.2f%%, chi-sq(wl) = %5.2f%%, n_win=%d\n', ...
            wl, wl*Ts, rel_std_z, expected_rel_std, n_win);
end

%% Experiment 2C: Paper precision comparison
% Run a near-paper scenario: 1 Hz, amplitude 9 um, close to wall
% (This needs its own simulation — will add as separate script if time allows)
fprintf('\n--- Experiment 2C: paper benchmark (separate script) ---\n');
fprintf('  See phase2_paper_benchmark.m\n');

%% Save results
results = struct();
results.lc = lc;
results.T_sim = T_sim;
results.a_nom = a_nom;
results.C_dpmr_eff = C_dpmr_eff;
results.a_m_IIR = a_m_IIR;
results.del_pmr = del_pmr;
results.Var_est_IIR = Var_est_IIR;
results.ekf_logs = ekf;
results.t = t;
results.ss_start = ss_start;
results.chi_sq_rel_std_pct = chi_sq_rel_std_pct;

for ax = 1:3
    results.a_m_mean(ax) = mean(a_m_IIR(ax, ss));
    results.a_m_std(ax)  = std(a_m_IIR(ax, ss));
    results.a_m_rel_std_pct(ax) = results.a_m_std(ax) / results.a_m_mean(ax) * 100;
    results.a_m_bias_pct(ax) = (results.a_m_mean(ax) - a_nom) / a_nom * 100;
end

out_dir = fullfile(project_root, 'test_results', 'verify');
save(fullfile(out_dir, 'phase2_chisquared_mc.mat'), 'results');
fprintf('\nSaved: phase2_chisquared_mc.mat\n');

%% Plot: a_m time series + histogram
fig = figure('Position', [100, 100, 1400, 700]);

subplot(2,2,1);
plot(t(ss), a_m_IIR(3, ss) / a_nom, 'b-', 'LineWidth', 1);
hold on;
yline(1, 'k--', 'LineWidth', 1.5);
yline(mean(a_m_IIR(3, ss))/a_nom, 'r-', 'LineWidth', 1.5, 'Label', 'mean');
hold off;
xlabel('Time [s]'); ylabel('a_m (z) / a_{nom}');
title(sprintf('a_m time series (rel std = %.1f%%)', results.a_m_rel_std_pct(3)));
set(gca, 'FontSize', 12);

subplot(2,2,2);
histogram(a_m_IIR(3, ss) / a_nom, 50, 'Normalization', 'pdf');
xlabel('a_m (z) / a_{nom}');
ylabel('PDF');
title('a_m distribution');
set(gca, 'FontSize', 12);

subplot(2,2,3);
plot(t(ss), del_pmr(3, ss), 'b-', 'LineWidth', 0.5);
xlabel('Time [s]'); ylabel('del_{pmr} (z) [um]');
title('del_{pmr} time series');
set(gca, 'FontSize', 12);

subplot(2,2,4);
histogram(del_pmr(3, ss), 50, 'Normalization', 'pdf');
xlabel('del_{pmr} (z) [um]');
ylabel('PDF');
title('del_{pmr} distribution');
set(gca, 'FontSize', 12);

fig_dir = fullfile(project_root, 'reference', 'qr_analysis');
saveas(fig, fullfile(fig_dir, 'fig_phase2_chisquared.png'));
fprintf('Figure saved: fig_phase2_chisquared.png\n');

fprintf('\nDone.\n');
