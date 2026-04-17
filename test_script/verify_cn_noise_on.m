% verify_cn_noise_on.m
% Phase 3 Task C: empirical verification of C_n_eff with sigma_n^2 > 0.
%
% Writeup §5.6 predicts:
%   Var(del_pmr)[k] = C_dpmr * 4*k_B*T*a_x(h) + C_n * sigma_n^2
%
% All P2 tests so far ran at sigma_n^2 = 0, so only C_dpmr was verified.
% This script runs a sigma_n > 0 scenario at free-space (where a_x ≈ a_nom
% cleanly) and extracts C_n_empirical:
%
%   C_n_empirical = (Var(del_pmr) - C_dpmr * 4*k_B*T*a_x) / sigma_n^2
%
% Compare to code's C_n_eff = 1.1141.
%
% Scenario:
%   - h_init = 50 um (free-space baseline; wall effect minimal, c_perp ≈ 1.05)
%   - Amplitude = 0 (static hold)
%   - lc = 0.7 (same as lookup build)
%   - T_sim = 30 s (long enough for Var(del_pmr) to be stable)
%   - thermal = on, meas_noise = on
%   - Sweep sigma_n over [0, 0.01, 0.02, 0.03] um

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

fprintf('===== C_n verification with meas_noise ON =====\n\n');

% ---- Constants ----
constants = physical_constants();
k_B     = constants.k_B;
T_temp  = constants.T;
Ts      = constants.Ts;
gamma_N = constants.gamma_N;
a_nom   = Ts / gamma_N;
sigma2_dXT = 4 * k_B * T_temp * Ts / gamma_N;
R_probe = constants.R;

% ---- Operating point ----
h_init = 50;
lc     = 0.7;
T_sim  = 30;

h_bar = h_init / R_probe;
[cpara, cperp] = calc_correction_functions(h_bar);
a_x_th = a_nom / cpara;  % x, y use c_para
a_z_th = a_nom / cperp;  % z uses c_perp

fprintf('h=%.1f, h_bar=%.3f, c_para=%.3f, c_perp=%.3f\n', h_init, h_bar, cpara, cperp);
fprintf('a_x_th=%.4e, a_z_th=%.4e, sigma2_dXT=%.4e\n\n', a_x_th, a_z_th, sigma2_dXT);

% ---- Sigma_n sweep ----
sigma_n_list = [0, 0.01, 0.02, 0.03];   % um
n_cases = length(sigma_n_list);

results = struct();
results.sigma_n_list = sigma_n_list;
results.std_x_pmr_nm = nan(1, n_cases);
results.std_y_pmr_nm = nan(1, n_cases);
results.std_z_pmr_nm = nan(1, n_cases);
results.Var_x_pmr    = nan(1, n_cases);
results.Var_y_pmr    = nan(1, n_cases);
results.Var_z_pmr    = nan(1, n_cases);

for ci = 1:n_cases
    sn = sigma_n_list(ci);
    fprintf('[%d/%d] sigma_n = %.3f um\n', ci, n_cases, sn);

    config = user_config();
    config.h_init             = h_init;
    config.amplitude          = 0;
    config.enable_wall_effect = true;
    config.h_min              = 2.3;
    config.trajectory_type    = 'positioning';
    config.t_hold             = 0;
    config.n_cycles           = 1;
    config.frequency          = 1;
    config.ctrl_enable        = true;
    config.controller_type    = 7;
    config.lambda_c           = lc;
    config.thermal_enable     = true;
    config.meas_noise_enable  = (sn > 0);
    config.meas_noise_std     = [sn; sn; sn];
    config.T_sim              = T_sim;

    params = calc_simulation_params(config);

    % Fixed seed for reproducibility (different per case to avoid
    % correlation but consistent across runs of this script)
    rng(20260417 + ci);

    assignin('base', 'params', params);
    assignin('base', 'p0', params.Value.common.p0);
    assignin('base', 'Ts', Ts);

    model_path = fullfile(project_root, 'model', 'system_model');
    t0 = tic;
    simOut = sim(model_path, 'StopTime', num2str(T_sim), ...
        'SaveTime', 'on', 'TimeSaveName', 'tout', ...
        'SaveOutput', 'on', 'OutputSaveName', 'yout');
    fprintf('    sim done %.1f sec\n', toc(t0));

    p_d = simOut.p_d_out';
    p_m = simOut.p_m_out';
    N = size(p_d, 2);

    % Convention A del_pm, offline IIR del_pmr
    del_pm = zeros(3, N);
    for k = 3:N
        del_pm(:, k) = p_d(:, k-2) - p_m(:, k);
    end
    a_pd  = 0.05;
    del_pmd = zeros(3, N);
    del_pmr = zeros(3, N);
    for k = 2:N
        del_pmd(:,k) = (1-a_pd)*del_pmd(:,k-1) + a_pd*del_pm(:,k);
        del_pmr(:,k) = del_pm(:,k) - del_pmd(:,k);
    end

    ss = round(10/Ts):N;      % skip 10 s transient
    std_x_pmr = std(del_pmr(1, ss), 0);
    std_y_pmr = std(del_pmr(2, ss), 0);
    std_z_pmr = std(del_pmr(3, ss), 0);

    results.std_x_pmr_nm(ci) = 1000 * std_x_pmr;
    results.std_y_pmr_nm(ci) = 1000 * std_y_pmr;
    results.std_z_pmr_nm(ci) = 1000 * std_z_pmr;
    results.Var_x_pmr(ci)    = std_x_pmr^2;
    results.Var_y_pmr(ci)    = std_y_pmr^2;
    results.Var_z_pmr(ci)    = std_z_pmr^2;

    fprintf('    std_pmr (nm): x=%.2f y=%.2f z=%.2f\n', ...
        results.std_x_pmr_nm(ci), results.std_y_pmr_nm(ci), results.std_z_pmr_nm(ci));
end

% ---- Theoretical prediction (from Phase 1 / code lookup) ----
C_dpmr_eff = params.Value.ctrl.C_dpmr_eff;    % 3.9242 at lc=0.7
C_np_eff   = params.Value.ctrl.C_np_eff;      % 1.1141 at lc=0.7

fprintf('\n===== Analysis =====\n');
fprintf('C_dpmr_eff = %.4f, C_np_eff (code) = %.4f\n\n', C_dpmr_eff, C_np_eff);

thermal_var_x = C_dpmr_eff * sigma2_dXT / cpara;
thermal_var_z = C_dpmr_eff * sigma2_dXT / cperp;
fprintf('Thermal Var(del_pmr):  x=%.3e um², z=%.3e um²\n', thermal_var_x, thermal_var_z);
fprintf('  (std_thermal:        x=%.2f nm, z=%.2f nm)\n\n', ...
        1000*sqrt(thermal_var_x), 1000*sqrt(thermal_var_z));

fprintf('%-10s %-12s %-12s %-12s %-12s %-12s\n', ...
    'sigma_n', 'Var_total_x', 'Var_noise_x', 'Cn_emp_x', 'Cn_emp_y', 'Cn_emp_z');
for ci = 1:n_cases
    sn = sigma_n_list(ci);
    sn2 = sn^2;
    Vx = results.Var_x_pmr(ci);
    Vy = results.Var_y_pmr(ci);
    Vz = results.Var_z_pmr(ci);

    % Subtract thermal; what remains should be C_n * sigma_n^2
    Vx_noise = Vx - thermal_var_x;
    Vy_noise = Vy - thermal_var_x;
    Vz_noise = Vz - thermal_var_z;

    if sn > 0
        Cn_x = Vx_noise / sn2;
        Cn_y = Vy_noise / sn2;
        Cn_z = Vz_noise / sn2;
        fprintf('%8.3f   %10.3e   %10.3e   %10.4f   %10.4f   %10.4f\n', ...
            sn, Vx, Vx_noise, Cn_x, Cn_y, Cn_z);
    else
        fprintf('%8.3f   %10.3e   %10.3e   (sigma_n=0: no C_n extraction)\n', ...
            sn, Vx, Vx_noise);
    end
end

% Regression: Var_pmr = thermal + C_n * sigma_n^2  (slope = C_n)
sn2 = sigma_n_list.^2;
Cn_fit_x = (results.Var_x_pmr - thermal_var_x) ./ max(sn2, eps);
Cn_fit_y = (results.Var_y_pmr - thermal_var_x) ./ max(sn2, eps);
Cn_fit_z = (results.Var_z_pmr - thermal_var_z) ./ max(sn2, eps);

% Linear regression slope over nonzero sigma_n
mask = sigma_n_list > 0;
slope_x = (results.Var_x_pmr(mask) - thermal_var_x) * sn2(mask).' / (sn2(mask) * sn2(mask).');
slope_y = (results.Var_y_pmr(mask) - thermal_var_x) * sn2(mask).' / (sn2(mask) * sn2(mask).');
slope_z = (results.Var_z_pmr(mask) - thermal_var_z) * sn2(mask).' / (sn2(mask) * sn2(mask).');

fprintf('\n---- Least-squares slope (C_n empirical) ----\n');
fprintf('  x: %.4f    y: %.4f    z: %.4f    (code prediction: %.4f)\n', ...
    slope_x, slope_y, slope_z, C_np_eff);
fprintf('  Relative error vs code C_np_eff:\n');
fprintf('    x: %+.2f%%   y: %+.2f%%   z: %+.2f%%\n', ...
    100*(slope_x - C_np_eff)/C_np_eff, ...
    100*(slope_y - C_np_eff)/C_np_eff, ...
    100*(slope_z - C_np_eff)/C_np_eff);

% Save
results.thermal_var_x = thermal_var_x;
results.thermal_var_z = thermal_var_z;
results.C_dpmr_eff    = C_dpmr_eff;
results.C_np_eff      = C_np_eff;
results.slope_x       = slope_x;
results.slope_y       = slope_y;
results.slope_z       = slope_z;
results.h_init        = h_init;
results.lc            = lc;
results.cpara         = cpara;
results.cperp         = cperp;

save('test_results/verify/verify_cn_noise_on.mat', '-struct', 'results');
fprintf('\nSaved test_results/verify/verify_cn_noise_on.mat\n');
fprintf('Done.\n');
