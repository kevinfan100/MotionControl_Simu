%% verify_task1c_correction.m - Verify IIR_bias_factor correction on Phase 2A
%
% Task 1c: Run a fresh 30 sec Simulink with the IIR_bias_factor correction
% enabled, reapply the IIR offline on del_pmr, and report a_m statistics
% with AND without the correction applied post-hoc. Expected outcome:
%   corrected  a_m_z / a_nom >= 0.99
%   uncorrected a_m_z / a_nom == Phase 2A baseline (~0.925)
%
% The Simulink model is now using the corrected controller, so the del_pmr
% time series is already produced under closed-loop feedback with corrected
% a_hat. The correction is applied as a final scaling on Var_IIR.

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

fprintf('===== verify_task1c_correction (30 sec free-space) =====\n\n');

constants = physical_constants();
k_B       = constants.k_B;
T_temp    = constants.T;
Ts        = constants.Ts;
gamma_N   = constants.gamma_N;
a_nom     = Ts / gamma_N;

lc    = 0.7;
T_sim = 30;
fprintf('lc = %.2f, T_sim = %.1f s, a_nom = %.4e\n\n', lc, T_sim, a_nom);

%% Configure simulation (match phase2_chisquared_mc.m)
config = user_config();
config.h_init              = 50;
config.amplitude           = 0;
config.enable_wall_effect  = false;
config.trajectory_type     = 'positioning';
config.t_hold              = 0;
config.n_cycles            = 1;
config.frequency           = 1;
config.ctrl_enable         = true;
config.controller_type     = 7;
config.lambda_c            = lc;
config.thermal_enable      = true;
config.meas_noise_enable   = false;
config.meas_noise_std      = [0; 0; 0];
config.T_sim               = T_sim;

params = calc_simulation_params(config);
assignin('base', 'params', params);
assignin('base', 'p0', params.Value.common.p0);
assignin('base', 'Ts', Ts);

C_dpmr_eff      = params.Value.ctrl.C_dpmr_eff;
C_np_eff        = params.Value.ctrl.C_np_eff;
IIR_bias_factor = params.Value.ctrl.IIR_bias_factor;
fprintf('Loaded params:\n');
fprintf('  C_dpmr_eff       = %.4f\n', C_dpmr_eff);
fprintf('  C_np_eff         = %.4f\n', C_np_eff);
fprintf('  IIR_bias_factor  = %.4f  (expected correction = %.4fx)\n\n', ...
        IIR_bias_factor, 1/IIR_bias_factor);

%% Run Simulink with same RNG seed as Phase 2A
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
N   = size(p_d, 2);
t   = (0:N-1) * Ts;

%% Reconstruct del_pm (matches controller's p_d[k-2] - p_m[k])
del_pm = zeros(3, N);
for k = 3:N
    del_pm(:, k) = p_d(:, k-2) - p_m(:, k);
end

%% Offline IIR (same as controller)
a_pd  = config.a_pd;
a_cov = config.a_cov;
a_prd = config.a_prd;
del_pmd      = zeros(3, N);
del_pmr      = zeros(3, N);
del_pmrd     = zeros(3, N);
del_pmr2_avg = zeros(3, N);
for k = 2:N
    del_pmd(:,k)      = (1-a_pd)  * del_pmd(:,k-1)  + a_pd  * del_pm(:,k);
    del_pmr(:,k)      = del_pm(:,k) - del_pmd(:,k);
    del_pmrd(:,k)     = (1-a_prd) * del_pmrd(:,k-1) + a_prd * del_pmr(:,k);
    del_pmr2_avg(:,k) = (1-a_cov) * del_pmr2_avg(:,k-1) + a_cov * del_pmr(:,k).^2;
end
Var_est_IIR = max(del_pmr2_avg - del_pmrd.^2, 0);

%% Two versions of a_m: uncorrected (phase 2A style) and corrected (task 1c)
den = C_dpmr_eff * 4 * k_B * T_temp;
a_m_uncorr = max(Var_est_IIR / den, 0);
a_m_corr   = max((Var_est_IIR / IIR_bias_factor) / den, 0);

ss_start = round(10 / Ts);
ss       = ss_start:N;
fprintf('Steady state window: %d..%d (%d samples, %.1f s)\n\n', ...
        ss(1), ss(end), length(ss), length(ss)*Ts);

%% a_m statistics
axname = {'x', 'y', 'z'};
fprintf('%-6s %-18s %-18s %-18s %-18s\n', ...
    'axis', 'mean/a_nom (unc)', 'mean/a_nom (cor)', 'bias (unc)', 'bias (cor)');
for ax = 1:3
    m_u = mean(a_m_uncorr(ax, ss));
    m_c = mean(a_m_corr(ax, ss));
    b_u = (m_u - a_nom) / a_nom * 100;
    b_c = (m_c - a_nom) / a_nom * 100;
    fprintf('%-6s %-18.4f %-18.4f %-+18.2f %-+18.2f\n', ...
            axname{ax}, m_u/a_nom, m_c/a_nom, b_u, b_c);
end

%% Gate: z-axis corrected bias within +/- 2% of a_nom
m_u_z = mean(a_m_uncorr(3, ss));
m_c_z = mean(a_m_corr(3, ss));
s_u_z = std(a_m_uncorr(3, ss));
s_c_z = std(a_m_corr(3, ss));
ratio_u = m_u_z / a_nom;
ratio_c = m_c_z / a_nom;

fprintf('\n--- z-axis details ---\n');
fprintf('  uncorrected: mean/a_nom = %.4f   std/mean = %.2f%%\n', ...
        ratio_u, s_u_z/m_u_z*100);
fprintf('  corrected  : mean/a_nom = %.4f   std/mean = %.2f%%\n', ...
        ratio_c, s_c_z/m_c_z*100);
fprintf('  Task 1b predicted (uncorr / V_sample): 0.9162\n');
fprintf('  Phase 2A prior baseline              : 0.9250\n');

if ratio_c >= 0.98 && ratio_c <= 1.02
    fprintf('  GATE G1 (corrected in [0.98, 1.02]): PASS\n');
elseif ratio_c >= 0.97 && ratio_c <= 1.03
    fprintf('  GATE G1: MARGINAL (within [0.97, 1.03])\n');
else
    fprintf('  GATE G1: FAIL (%.4f outside tolerance)\n', ratio_c);
end

%% Save results
results = struct();
results.lc                 = lc;
results.T_sim              = T_sim;
results.a_nom              = a_nom;
results.C_dpmr_eff         = C_dpmr_eff;
results.IIR_bias_factor    = IIR_bias_factor;
results.del_pmr            = del_pmr;
results.Var_est_IIR        = Var_est_IIR;
results.a_m_uncorr         = a_m_uncorr;
results.a_m_corr           = a_m_corr;
results.t                  = t;
results.ss_start           = ss_start;
for ax = 1:3
    results.mean_uncorr(ax) = mean(a_m_uncorr(ax, ss));
    results.mean_corr(ax)   = mean(a_m_corr(ax, ss));
    results.std_uncorr(ax)  = std(a_m_uncorr(ax, ss));
    results.std_corr(ax)    = std(a_m_corr(ax, ss));
end

out_dir  = fullfile(project_root, 'test_results', 'verify');
out_file = fullfile(out_dir, 'task1c_verification.mat');
save(out_file, 'results');
fprintf('\nSaved: %s\n', out_file);

%% Figure: a_m time series + histograms, both uncorrected and corrected
FS = 16;
fig = figure('Position', [100, 100, 1400, 700], 'Color', 'w', 'Visible', 'off');

subplot(2, 2, 1);
plot(t(ss), a_m_uncorr(3, ss)/a_nom, '-', 'LineWidth', 0.8, 'Color', [0 0.2 0.8]);
hold on;
yline(1, 'k-', 'Alpha', 0.4);
yline(ratio_u, '-', 'LineWidth', 2.0, 'Color', [0.8 0 0]);
xlabel('Time [s]', 'FontSize', FS, 'FontWeight', 'bold');
ylabel('a_m(z) / a_{nom}  (uncorrected)', 'FontSize', FS, 'FontWeight', 'bold');
legend({'series', 'a_{nom}', sprintf('mean = %.4f', ratio_u)}, ...
    'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS-4);
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2.0, 'Box', 'on');
ylim([0.2, 1.8]);

subplot(2, 2, 2);
plot(t(ss), a_m_corr(3, ss)/a_nom, '-', 'LineWidth', 0.8, 'Color', [0 0.6 0]);
hold on;
yline(1, 'k-', 'Alpha', 0.4);
yline(ratio_c, '-', 'LineWidth', 2.0, 'Color', [0.8 0 0]);
xlabel('Time [s]', 'FontSize', FS, 'FontWeight', 'bold');
ylabel('a_m(z) / a_{nom}  (corrected)', 'FontSize', FS, 'FontWeight', 'bold');
legend({'series', 'a_{nom}', sprintf('mean = %.4f', ratio_c)}, ...
    'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS-4);
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2.0, 'Box', 'on');
ylim([0.2, 1.8]);

subplot(2, 2, 3);
edges = linspace(0.2, 1.8, 60);
h1 = histogram(a_m_uncorr(3, ss)/a_nom, edges, 'Normalization', 'pdf', ...
    'FaceColor', [0 0.2 0.8], 'FaceAlpha', 0.5, 'EdgeColor', 'none');
hold on;
h2 = histogram(a_m_corr(3, ss)/a_nom, edges, 'Normalization', 'pdf', ...
    'FaceColor', [0 0.6 0], 'FaceAlpha', 0.5, 'EdgeColor', 'none');
xline(1, 'k-', 'LineWidth', 1.5, 'Alpha', 0.6);
xline(ratio_u, ':', 'LineWidth', 2.0, 'Color', [0 0.2 0.8]);
xline(ratio_c, ':', 'LineWidth', 2.0, 'Color', [0 0.6 0]);
xlabel('a_m(z) / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
ylabel('PDF', 'FontSize', FS, 'FontWeight', 'bold');
legend([h1, h2], {'uncorrected', 'corrected'}, ...
    'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS-2);
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2.0, 'Box', 'on');

subplot(2, 2, 4);
bar_data = [ratio_u, ratio_c];
b = bar([1 2], bar_data, 0.6);
b.FaceColor = 'flat';
b.CData(1, :) = [0 0.2 0.8];
b.CData(2, :) = [0 0.6 0];
hold on;
yline(1, 'k-', 'LineWidth', 2.0);
yline(0.9069, ':', 'LineWidth', 1.5, 'Color', [0.5 0.5 0.5]);
ylim([0.85, 1.05]);
xticks([1 2]);
xticklabels({'uncorrected', 'corrected'});
ylabel('mean(a_m) / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
text(1, ratio_u + 0.01, sprintf('%.4f', ratio_u), 'HorizontalAlignment', 'center', ...
    'FontSize', FS, 'FontWeight', 'bold');
text(2, ratio_c + 0.01, sprintf('%.4f', ratio_c), 'HorizontalAlignment', 'center', ...
    'FontSize', FS, 'FontWeight', 'bold');
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2.0, 'Box', 'on');

fig_dir  = fullfile(project_root, 'reference', 'qr_analysis');
fig_path = fullfile(fig_dir, 'fig_task1c_correction.png');
exportgraphics(fig, fig_path, 'Resolution', 150);
close(fig);
fprintf('Figure saved: %s\n', fig_path);

fprintf('\nDone.\n');
