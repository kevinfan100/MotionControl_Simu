% temp_p2_static_h25.m
% P2 Step 2: fresh static h=2.5 µm simulation (lc=0.7, no motion, 30 s)
% Isolates architectural near-wall behavior from dynamic lag effects.
%
% Decision criteria (from plan):
%   ratio_z ≤ 1.1  → dynamic 1.29 was pure lag, P2 static passes at h=2.5
%   ratio_z > 1.1  → architectural near-wall issue, independent of dynamics

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

fprintf('===== P2 static h=2.5 simulation =====\n\n');

% ---- Constants ----
constants = physical_constants();
k_B     = constants.k_B;
T_temp  = constants.T;
Ts      = constants.Ts;
gamma_N = constants.gamma_N;
a_nom   = Ts / gamma_N;
sigma2_dXT = 4 * k_B * T_temp * Ts / gamma_N;
R = constants.R;

fprintf('Constants: a_nom = %.4e, sigma2_dXT = %.4e, R = %.2f um\n', a_nom, sigma2_dXT, R);

% ---- Config ----
lc = 0.7;
T_sim = 30;
h_init = 2.5;

config = user_config();
config.h_init = h_init;
config.amplitude = 0;                    % NO motion
config.enable_wall_effect = true;        % wall ON
config.h_min = 2.3;                      % allow h=2.5
config.trajectory_type = 'positioning';  % static hold
config.t_hold = 0;
config.n_cycles = 1;
config.frequency = 1;                    % unused with amplitude=0
config.ctrl_enable = true;
config.controller_type = 7;
config.lambda_c = lc;
config.thermal_enable = true;
config.meas_noise_enable = false;
config.meas_noise_std = [0; 0; 0];
config.T_sim = T_sim;

% h_bar, c_para, c_perp for theoretical
h_bar = h_init / R;
[cpara, cperp] = calc_correction_functions(h_bar);
a_x_theory = a_nom / cpara;
a_z_theory = a_nom / cperp;
fprintf('At h=%.2f um: h_bar=%.3f, c_para=%.3f, c_perp=%.3f\n', h_init, h_bar, cpara, cperp);
fprintf('Theoretical a_x=%.4e, a_z=%.4e (ratio z/nom=%.3f)\n', a_x_theory, a_z_theory, a_z_theory/a_nom);

% Get C_dpmr_eff from params (lookup)
params = calc_simulation_params(config);
C_dpmr_eff = params.Value.ctrl.C_dpmr_eff;
fprintf('C_dpmr_eff (from lookup) = %.4f\n', C_dpmr_eff);

% Theoretical std per axis (at this h)
std_theory_x_nm = 1000 * sqrt(C_dpmr_eff * sigma2_dXT / cpara);
std_theory_z_nm = 1000 * sqrt(C_dpmr_eff * sigma2_dXT / cperp);
fprintf('Theoretical std_x = %.2f nm, std_z = %.2f nm\n\n', std_theory_x_nm, std_theory_z_nm);

% ---- Run ----
assignin('base', 'params', params);
assignin('base', 'p0', params.Value.common.p0);
assignin('base', 'Ts', Ts);

model_path = fullfile(project_root, 'model', 'system_model');
rng(20260415);   % fresh seed
fprintf('Running Simulink (T_sim=%.1f s)...', T_sim);
t0 = tic;
simOut = sim(model_path, 'StopTime', num2str(T_sim), ...
    'SaveTime', 'on', 'TimeSaveName', 'tout', ...
    'SaveOutput', 'on', 'OutputSaveName', 'yout');
fprintf(' done (%.1f sec)\n\n', toc(t0));

p_d = simOut.p_d_out';
p_m = simOut.p_m_out';
ekf_out = simOut.ekf_out';
N = size(p_d, 2);
t = (0:N-1) * Ts;

% ---- Convention A: del_pm[k] = p_d[k-2] - p_m[k] (raw measured tracking error) ----
% NOTE: del_pm is NOT what Section 5 Lyapunov predicts. Section 5 predicts
% Var(del_pmr), where del_pmr is the HP-filtered residual produced by the
% controller's IIR chain. We must reapply the same IIR offline to get del_pmr,
% then compare its variance to theoretical. Using Var(del_pm) directly would
% overestimate by a factor ~Var(del_pm)/Var(del_pmr) ≈ 1.27 (strong closed-loop
% autocorrelation makes the LP del_pmd track the slow variation, inflating
% del_pm relative to del_pmr). This was the source of a false "13% near-wall
% excess" finding prior to 2026-04-15.
del_pm = zeros(3, N);
for k = 3:N
    del_pm(:, k) = p_d(:, k-2) - p_m(:, k);
end

% ---- Reapply controller's IIR to get del_pmr (the Section 5 observable) ----
a_pd  = 0.05;
a_prd = 0.05;
a_cov = 0.05;
del_pmd      = zeros(3, N);
del_pmr      = zeros(3, N);
del_pmrd     = zeros(3, N);
del_pmr2_avg = zeros(3, N);
for k = 2:N
    del_pmd(:,k)      = (1-a_pd)*del_pmd(:,k-1)    + a_pd*del_pm(:,k);
    del_pmr(:,k)      = del_pm(:,k) - del_pmd(:,k);
    del_pmrd(:,k)     = (1-a_prd)*del_pmrd(:,k-1)  + a_prd*del_pmr(:,k);
    del_pmr2_avg(:,k) = (1-a_cov)*del_pmr2_avg(:,k-1) + a_cov*del_pmr(:,k).^2;
end

% Steady-state window: after 10 s warmup
ss_start = round(10 / Ts);
ss = ss_start:N;
fprintf('Steady-state window: samples %d..%d (%.1f s, %d samples)\n', ...
        ss(1), ss(end), length(ss)*Ts, length(ss));

% ---- Empirical std per axis — both signals for comparison ----
labels = {'x', 'y', 'z'};
std_theory = [std_theory_x_nm, std_theory_x_nm, std_theory_z_nm];  % y uses c_para same as x

fprintf('\n=== Per-axis results (del_pmr is the Section 5 observable) ===\n');
fprintf('%-5s %-12s %-14s %-14s %-12s %-10s\n', ...
        'axis', 'mean_pmr [nm]', 'std_pm [nm]', 'std_pmr [nm]', 'theory [nm]', 'ratio_pmr');
ratio_out_pmr = zeros(1, 3);
ratio_out_pm  = zeros(1, 3);
std_pm_nm     = zeros(1, 3);
std_pmr_nm    = zeros(1, 3);
mean_pmr_nm   = zeros(1, 3);
for ax = 1:3
    mn = mean(del_pmr(ax, ss));
    sd_pm  = std(del_pm(ax, ss),  0);
    sd_pmr = std(del_pmr(ax, ss), 0);
    mean_pmr_nm(ax) = 1000 * mn;
    std_pm_nm(ax)   = 1000 * sd_pm;
    std_pmr_nm(ax)  = 1000 * sd_pmr;
    ratio_out_pm(ax)  = std_pm_nm(ax)  / std_theory(ax);
    ratio_out_pmr(ax) = std_pmr_nm(ax) / std_theory(ax);
    fprintf('  %s   %+10.3f   %10.3f     %10.3f     %10.3f   %7.3f\n', ...
            labels{ax}, mean_pmr_nm(ax), std_pm_nm(ax), std_pmr_nm(ax), ...
            std_theory(ax), ratio_out_pmr(ax));
end

fprintf('\nFor reference, std_pm ratios (WRONG signal): %.3f / %.3f / %.3f\n', ...
        ratio_out_pm(1), ratio_out_pm(2), ratio_out_pm(3));

% ---- Verdict (decisive z-axis ratio on CORRECT signal) ----
fprintf('\n=== Verdict (z-axis ratio on del_pmr) ===\n');
if ratio_out_pmr(3) <= 1.05
    fprintf('  RATIO_Z = %.3f  →  PASS  (P2 near-wall matches Section 5 prediction)\n', ratio_out_pmr(3));
elseif ratio_out_pmr(3) <= 1.15
    fprintf('  RATIO_Z = %.3f  →  MARGINAL (small excess, check MC statistical error)\n', ratio_out_pmr(3));
else
    fprintf('  RATIO_Z = %.3f  →  FAIL  (real architectural issue)\n', ratio_out_pmr(3));
end

% ---- Save ----
save('test_results/verify/p2_static_h25.mat', ...
     'del_pm', 'del_pmr', 'del_pmd', 'del_pmrd', 't', 'ss', ...
     'std_pm_nm', 'std_pmr_nm', 'mean_pmr_nm', 'std_theory', ...
     'ratio_out_pm', 'ratio_out_pmr', ...
     'C_dpmr_eff', 'a_nom', 'sigma2_dXT', 'R', 'h_init', 'cpara', 'cperp', 'lc');
fprintf('\nData saved: test_results/verify/p2_static_h25.mat\n');
