% verify_p2_static_h25_mc.m
% Phase 3 Task B: multi-seed P2 static at h=2.5 um to quantify ratio CI.
%
% Single-seed baseline (verify_p2_static_h25.m): ratio = 1.009 / 1.027 / 1.007.
% With only one seed, can't tell how much of the 1.009–1.027 spread is
% chi-squared floor vs systematic. Run 5 seeds to compute per-seed mean±std
% and establish a ±5% gate with confidence.

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

fprintf('===== P2 static h=2.5 multi-seed MC =====\n\n');

constants = physical_constants();
k_B     = constants.k_B;
T_temp  = constants.T;
Ts      = constants.Ts;
gamma_N = constants.gamma_N;
a_nom   = Ts / gamma_N;
sigma2_dXT = 4 * k_B * T_temp * Ts / gamma_N;
R_probe = constants.R;

h_init = 2.5;
lc     = 0.7;
T_sim  = 30;
n_seeds = 5;
base_seed = 20260430;

h_bar = h_init / R_probe;
[cpara, cperp] = calc_correction_functions(h_bar);
fprintf('h=%.2f, h_bar=%.3f, c_para=%.3f, c_perp=%.3f\n', h_init, h_bar, cpara, cperp);
fprintf('N seeds=%d, base=%d, T_sim=%.1f s\n\n', n_seeds, base_seed, T_sim);

ratio_x = nan(1, n_seeds);
ratio_y = nan(1, n_seeds);
ratio_z = nan(1, n_seeds);
std_x   = nan(1, n_seeds);
std_y   = nan(1, n_seeds);
std_z   = nan(1, n_seeds);

for si = 1:n_seeds
    seed = base_seed + si;
    fprintf('[%d/%d] seed=%d ', si, n_seeds, seed);

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
    config.meas_noise_enable  = false;
    config.meas_noise_std     = [0; 0; 0];
    config.T_sim              = T_sim;

    params = calc_simulation_params(config);
    rng(seed);

    assignin('base', 'params', params);
    assignin('base', 'p0', params.Value.common.p0);
    assignin('base', 'Ts', Ts);

    model_path = fullfile(project_root, 'model', 'system_model');
    t0 = tic;
    simOut = sim(model_path, 'StopTime', num2str(T_sim), ...
        'SaveTime', 'on', 'TimeSaveName', 'tout', ...
        'SaveOutput', 'on', 'OutputSaveName', 'yout');
    fprintf('sim=%.0f s ', toc(t0));

    p_d = simOut.p_d_out';
    p_m = simOut.p_m_out';
    N = size(p_d, 2);

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

    ss = round(10/Ts):N;
    std_x(si) = 1000*std(del_pmr(1, ss), 0);
    std_y(si) = 1000*std(del_pmr(2, ss), 0);
    std_z(si) = 1000*std(del_pmr(3, ss), 0);

    C_dpmr_eff = params.Value.ctrl.C_dpmr_eff;
    std_th_x = 1000 * sqrt(C_dpmr_eff * sigma2_dXT / cpara);
    std_th_z = 1000 * sqrt(C_dpmr_eff * sigma2_dXT / cperp);

    ratio_x(si) = std_x(si) / std_th_x;
    ratio_y(si) = std_y(si) / std_th_x;
    ratio_z(si) = std_z(si) / std_th_z;
    fprintf('ratio: x=%.3f y=%.3f z=%.3f\n', ratio_x(si), ratio_y(si), ratio_z(si));
end

fprintf('\n===== Aggregate (%d seeds) =====\n', n_seeds);
fprintf('Theoretical std (c_⊥=%.2f, c_∥=%.2f): x=%.2f nm, z=%.2f nm\n\n', ...
    cperp, cpara, std_th_x, std_th_z);

fprintf('%-6s %-12s %-12s %-12s\n', 'axis', 'mean ratio', 'std ratio', '±95% on mean');
for ax_pair = {'x', 'y', 'z'}
    ax = ax_pair{1};
    r = eval(['ratio_', ax]);
    m = mean(r);
    s = std(r);
    ci95 = 1.96 * s / sqrt(n_seeds);
    fprintf('%-6s  %9.4f    %9.4f    %9.4f\n', ax, m, s, ci95);
end

fprintf('\nPer-seed detail:\n');
fprintf('%-5s', 'seed');
for ax_pair = {'x','y','z'}; fprintf(' %-10s', ['ratio_', ax_pair{1}]); end
fprintf('\n');
for si = 1:n_seeds
    fprintf('%-5d %9.4f  %9.4f  %9.4f\n', base_seed+si, ratio_x(si), ratio_y(si), ratio_z(si));
end

% Verdict
fprintf('\n--- ±5%% gate check ---\n');
for ax_pair = {'x','y','z'}
    ax = ax_pair{1};
    r = eval(['ratio_', ax]);
    m = mean(r);
    within = all(abs(r - 1.0) < 0.05);
    pass_mean = abs(m - 1.0) < 0.05;
    if within && pass_mean
        verdict = 'PASS (all seeds + mean within ±5%)';
    elseif pass_mean
        n_out = sum(abs(r - 1.0) >= 0.05);
        verdict = sprintf('MARGINAL (%d/%d seeds outside ±5%%, mean within)', n_out, n_seeds);
    else
        verdict = 'FAIL (mean outside ±5%)';
    end
    fprintf('  %s: %s\n', ax, verdict);
end

results.ratio_x = ratio_x;
results.ratio_y = ratio_y;
results.ratio_z = ratio_z;
results.std_x = std_x;
results.std_y = std_y;
results.std_z = std_z;
results.h_init = h_init;
results.lc = lc;
results.cpara = cpara;
results.cperp = cperp;
results.C_dpmr_eff = C_dpmr_eff;
results.n_seeds = n_seeds;
results.base_seed = base_seed;
save('test_results/verify/p2_static_h25_mc.mat', '-struct', 'results');
fprintf('\nSaved test_results/verify/p2_static_h25_mc.mat\n');
