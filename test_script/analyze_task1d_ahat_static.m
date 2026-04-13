%% analyze_task1d_ahat_static.m - Task 1d A1: static a_hat diagnostic
%
% Question: after Task 1c correction, is the EKF-filtered `a_hat` (runtime
% output) actually noisy at ~44% like the raw `a_m` (offline IIR-derived),
% or is it much smoother thanks to the 7-state KF?
%
% Method: run fresh 30 s Phase 2A (free-space positioning, no meas noise,
% thermal on, seed 12345). Extract `a_hat_x` and `a_hat_z` from ekf_out.
% Reconstruct del_pmr and rerun IIR offline to get `a_m_offline` as reference.
% Compare std/mean and autocorrelation for both signals.
%
% Output: task1d_static_ahat.mat + fig_task1d_ahat_vs_am_static.png

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

fprintf('===== Task 1d A1: static a_hat diagnostic =====\n\n');

constants = physical_constants();
k_B     = constants.k_B;
T_temp  = constants.T;
Ts      = constants.Ts;
gamma_N = constants.gamma_N;
a_nom   = Ts / gamma_N;

%% Configure Phase 2A scenario
config = user_config();
config.h_init             = 50;
config.amplitude          = 0;
config.enable_wall_effect = false;
config.trajectory_type    = 'positioning';
config.t_hold             = 0;
config.n_cycles           = 1;
config.frequency          = 1;
config.ctrl_enable        = true;
config.controller_type    = 7;
config.lambda_c           = 0.7;
config.thermal_enable     = true;
config.meas_noise_enable  = false;
config.meas_noise_std     = [0; 0; 0];
config.T_sim              = 30;

params = calc_simulation_params(config);
assignin('base', 'params', params);
assignin('base', 'p0', params.Value.common.p0);
assignin('base', 'Ts', Ts);

fprintf('lc=%.2f  IIR_bias_factor=%.4f  C_dpmr_eff=%.4f\n', ...
        config.lambda_c, params.Value.ctrl.IIR_bias_factor, ...
        params.Value.ctrl.C_dpmr_eff);

%% Run Simulink
rng(12345);
fprintf('Running Simulink (T_sim=%.1f)...', config.T_sim);
t0 = tic;
simOut = sim(fullfile(project_root, 'model', 'system_model'), ...
    'StopTime', num2str(config.T_sim), ...
    'SaveTime', 'on', 'TimeSaveName', 'tout', ...
    'SaveOutput', 'on', 'OutputSaveName', 'yout');
fprintf(' done (%.1f sec)\n\n', toc(t0));

p_d = simOut.p_d_out';
p_m = simOut.p_m_out';
ekf = simOut.ekf_out';          % [4 x N]: rows 1-2 are a_hat_x, a_hat_z
N   = size(p_d, 2);
t   = (0:N-1) * Ts;

%% Extract a_hat (runtime EKF output)
a_hat_x = ekf(1, :);
a_hat_z = ekf(2, :);

%% Reconstruct del_pm and offline IIR -> a_m_offline
del_pm = zeros(3, N);
for k = 3:N
    del_pm(:, k) = p_d(:, k-2) - p_m(:, k);
end

a_pd  = config.a_pd;
a_prd = config.a_prd;
a_cov = config.a_cov;
del_pmd      = zeros(3, N);
del_pmr      = zeros(3, N);
del_pmrd     = zeros(3, N);
del_pmr2_avg = zeros(3, N);
for k = 2:N
    del_pmd(:, k)      = (1 - a_pd) * del_pmd(:, k-1)  + a_pd  * del_pm(:, k);
    del_pmr(:, k)      = del_pm(:, k) - del_pmd(:, k);
    del_pmrd(:, k)     = (1 - a_prd) * del_pmrd(:, k-1) + a_prd * del_pmr(:, k);
    del_pmr2_avg(:, k) = (1 - a_cov) * del_pmr2_avg(:, k-1) + a_cov * del_pmr(:, k).^2;
end
Var_IIR = max(del_pmr2_avg - del_pmrd.^2, 0);

% Apply Task 1c correction and compute offline a_m (corrected)
C_dpmr_eff      = params.Value.ctrl.C_dpmr_eff;
IIR_bias_factor = params.Value.ctrl.IIR_bias_factor;
den             = C_dpmr_eff * 4 * k_B * T_temp;
a_m_offline_raw = max(Var_IIR / den, 0);                     % uncorrected
a_m_offline     = max((Var_IIR / IIR_bias_factor) / den, 0); % with Task 1c

%% Steady state window
ss_start = round(10 / Ts);
ss = ss_start:N;
fprintf('Steady state window: samples %d..%d (%d samples, %.1f s)\n\n', ...
        ss(1), ss(end), length(ss), length(ss)*Ts);

%% Statistics table
fprintf('--- Static statistics (z-axis primary, x-axis check) ---\n');
fprintf('%-24s %-12s %-12s %-14s %-14s\n', ...
    'Signal', 'mean/a_nom', 'std/mean %', 'std [um/pN]', 'autocorr(1)');

signals = struct();
signals.a_hat_x     = a_hat_x(ss);
signals.a_hat_z     = a_hat_z(ss);
signals.a_m_raw_x   = a_m_offline_raw(1, ss);
signals.a_m_raw_z   = a_m_offline_raw(3, ss);
signals.a_m_corr_x  = a_m_offline(1, ss);
signals.a_m_corr_z  = a_m_offline(3, ss);

stats = struct();
fnames = fieldnames(signals);
for i = 1:length(fnames)
    x  = signals.(fnames{i});
    mu = mean(x);
    sd = std(x);
    rs = sd / mu * 100;
    r1 = corr(x(1:end-1).', x(2:end).');
    stats.(fnames{i}).mean_over_nom = mu / a_nom;
    stats.(fnames{i}).rel_std_pct   = rs;
    stats.(fnames{i}).std           = sd;
    stats.(fnames{i}).autocorr1     = r1;
    fprintf('%-24s %-12.4f %-12.2f %-14.4e %-14.4f\n', ...
        fnames{i}, mu/a_nom, rs, sd, r1);
end

%% Autocorrelation rho(L) for L = 0..100 (z-axis only, most important)
L_max = 100;
rho_a_hat_z   = zeros(L_max+1, 1);
rho_a_m_raw_z = zeros(L_max+1, 1);
rho_a_m_cor_z = zeros(L_max+1, 1);

x1 = a_hat_z(ss);      x1 = x1 - mean(x1);
x2 = a_m_offline_raw(3, ss);  x2 = x2 - mean(x2);
x3 = a_m_offline(3, ss);      x3 = x3 - mean(x3);
v1 = mean(x1.^2); v2 = mean(x2.^2); v3 = mean(x3.^2);
for L = 0:L_max
    n = length(x1) - L;
    rho_a_hat_z(L+1)   = mean(x1(1:n) .* x1(1+L:end)) / v1;
    rho_a_m_raw_z(L+1) = mean(x2(1:n) .* x2(1+L:end)) / v2;
    rho_a_m_cor_z(L+1) = mean(x3(1:n) .* x3(1+L:end)) / v3;
end

% Effective time constant: first L where rho drops below 1/e
tc_a_hat_z   = find(rho_a_hat_z   < exp(-1), 1, 'first') - 1;
tc_a_m_cor_z = find(rho_a_m_cor_z < exp(-1), 1, 'first') - 1;

fprintf('\nAutocorrelation time constant (first L where rho < 1/e):\n');
fprintf('  a_hat_z    : %d samples = %.3f sec\n', tc_a_hat_z, tc_a_hat_z*Ts);
fprintf('  a_m_corr_z : %d samples = %.3f sec\n', tc_a_m_cor_z, tc_a_m_cor_z*Ts);

%% PSD comparison (z-axis, log-log)
[pxx_a_hat, f_psd] = pwelch(a_hat_z(ss) - mean(a_hat_z(ss)), [], [], [], 1/Ts);
[pxx_a_m, ~]       = pwelch(a_m_offline(3, ss) - mean(a_m_offline(3, ss)), [], [], [], 1/Ts);

%% Verdict
rs_a_hat_z  = stats.a_hat_z.rel_std_pct;
rs_a_m_cor_z = stats.a_m_corr_z.rel_std_pct;
smooth_factor = rs_a_m_cor_z / rs_a_hat_z;

fprintf('\n--- VERDICT ---\n');
fprintf('a_hat_z    rel std = %.2f %%\n', rs_a_hat_z);
fprintf('a_m_corr_z rel std = %.2f %%\n', rs_a_m_cor_z);
fprintf('EKF smoothing factor: %.2fx\n', smooth_factor);

if rs_a_hat_z <= 10
    verdict = 'PAPER-LEVEL';
    detail  = 'EKF is doing its job. Task 1c was sufficient.';
elseif rs_a_hat_z <= 25
    verdict = 'PARTIAL';
    detail  = 'EKF smooths but not fully. Sigma_e recursion may help.';
else
    verdict = 'EKF PASS-THROUGH';
    detail  = 'EKF barely helping. Architectural change likely required.';
end
fprintf('%s: %s\n', verdict, detail);

%% Save
out = struct();
out.stats         = stats;
out.rho_a_hat_z   = rho_a_hat_z;
out.rho_a_m_raw_z = rho_a_m_raw_z;
out.rho_a_m_cor_z = rho_a_m_cor_z;
out.tc_a_hat_z    = tc_a_hat_z;
out.tc_a_m_cor_z  = tc_a_m_cor_z;
out.pxx_a_hat     = pxx_a_hat;
out.pxx_a_m       = pxx_a_m;
out.f_psd         = f_psd;
out.a_hat_x       = a_hat_x;
out.a_hat_z       = a_hat_z;
out.a_m_offline_raw = a_m_offline_raw;
out.a_m_offline     = a_m_offline;
out.t             = t;
out.ss_start      = ss_start;
out.a_nom         = a_nom;
out.smooth_factor = smooth_factor;
out.verdict       = verdict;
out.IIR_bias_factor = IIR_bias_factor;
out.C_dpmr_eff    = C_dpmr_eff;

out_dir  = fullfile(project_root, 'test_results', 'verify');
out_file = fullfile(out_dir, 'task1d_static_ahat.mat');
save(out_file, 'out');
fprintf('\nSaved: %s\n', out_file);

%% Figure: 4 panels
FS    = 14;
LW_o  = 1.0;
LW_th = 2.2;
C_hat = [0.8 0 0];
C_m   = [0 0.2 0.8];

fig = figure('Position', [100, 100, 1500, 900], 'Color', 'w', 'Visible', 'off');

% Panel 1: a_hat_z vs a_m_corr_z time series (ss)
subplot(2, 2, 1);
plot(t(ss), a_m_offline(3, ss)/a_nom, '-', 'Color', C_m, 'LineWidth', 0.6);
hold on;
plot(t(ss), a_hat_z(ss)/a_nom, '-', 'Color', C_hat, 'LineWidth', 1.2);
yline(1, 'k-', 'Alpha', 0.5);
yline(stats.a_hat_z.mean_over_nom, '--', 'Color', C_hat, 'LineWidth', 1.5);
hold off;
xlabel('Time [s]', 'FontSize', FS, 'FontWeight', 'bold');
ylabel('a / a_{nom} (z)', 'FontSize', FS, 'FontWeight', 'bold');
legend({'a_{m,corr} (offline IIR)', 'a_{hat} (EKF)', 'a_{nom}', 'mean(a_{hat})'}, ...
    'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS-4);
title(sprintf('Time series (rel std: a_{hat}=%.1f%%, a_m=%.1f%%)', ...
    rs_a_hat_z, rs_a_m_cor_z), 'FontSize', FS);
set(gca, 'FontSize', FS, 'LineWidth', 1.8, 'Box', 'on');
ylim([0, 2]);

% Panel 2: std bar chart
subplot(2, 2, 2);
bdat = [stats.a_m_corr_x.rel_std_pct, stats.a_m_corr_z.rel_std_pct; ...
        stats.a_hat_x.rel_std_pct,    stats.a_hat_z.rel_std_pct];
b = bar(bdat);
b(1).FaceColor = C_m; b(2).FaceColor = C_hat;
set(gca, 'XTickLabel', {'a_{m,corr}', 'a_{hat}'});
ylabel('rel std [%]', 'FontSize', FS, 'FontWeight', 'bold');
legend({'x-axis', 'z-axis'}, ...
    'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS-2);
title(sprintf('EKF smoothing: %.2fx', smooth_factor), 'FontSize', FS);
set(gca, 'FontSize', FS, 'LineWidth', 1.8, 'Box', 'on');
for i = 1:2
    for j = 1:2
        text(i + (j-1.5)*0.3, bdat(i,j) + 1, sprintf('%.1f', bdat(i,j)), ...
            'HorizontalAlignment', 'center', 'FontSize', FS-2, 'FontWeight', 'bold');
    end
end

% Panel 3: autocorrelation rho(L)
subplot(2, 2, 3);
plot(0:L_max, rho_a_m_cor_z, '-', 'Color', C_m,   'LineWidth', LW_th);
hold on;
plot(0:L_max, rho_a_hat_z,   '-', 'Color', C_hat, 'LineWidth', LW_th);
yline(exp(-1), 'k:', 'LineWidth', 1.5, 'Alpha', 0.6);
yline(0, 'k-', 'Alpha', 0.4);
hold off;
xlabel('lag L [samples]', 'FontSize', FS, 'FontWeight', 'bold');
ylabel('\rho(L)', 'FontSize', FS, 'FontWeight', 'bold');
legend({'a_{m,corr}', 'a_{hat}', '1/e'}, ...
    'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS-2);
title(sprintf('Autocorr (a_{hat} tc=%d, a_m tc=%d samples)', tc_a_hat_z, tc_a_m_cor_z), ...
    'FontSize', FS);
set(gca, 'FontSize', FS, 'LineWidth', 1.8, 'Box', 'on');
xlim([0, L_max]);

% Panel 4: PSD log-log
subplot(2, 2, 4);
loglog(f_psd, pxx_a_m,   '-', 'Color', C_m,   'LineWidth', LW_th);
hold on;
loglog(f_psd, pxx_a_hat, '-', 'Color', C_hat, 'LineWidth', LW_th);
hold off;
xlabel('Frequency [Hz]', 'FontSize', FS, 'FontWeight', 'bold');
ylabel('PSD', 'FontSize', FS, 'FontWeight', 'bold');
legend({'a_{m,corr}', 'a_{hat}'}, ...
    'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS-2);
title('Power spectral density', 'FontSize', FS);
set(gca, 'FontSize', FS, 'LineWidth', 1.8, 'Box', 'on');
grid off;

fig_dir  = fullfile(project_root, 'reference', 'for_test');
fig_path = fullfile(fig_dir, 'fig_task1d_ahat_vs_am_static.png');
exportgraphics(fig, fig_path, 'Resolution', 150);
close(fig);
fprintf('Figure saved: %s\n', fig_path);

fprintf('\nDone.\n');
