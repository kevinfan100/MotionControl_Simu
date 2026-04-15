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

% ---- Convention A: dxm[k] = p_d[k-2] - p_m[k] ----
dxm = zeros(3, N);
for k = 3:N
    dxm(:, k) = p_d(:, k-2) - p_m(:, k);
end

% Steady-state window: after 10 s warmup
ss_start = round(10 / Ts);
ss = ss_start:N;
fprintf('Steady-state window: samples %d..%d (%.1f s, %d samples)\n', ...
        ss(1), ss(end), length(ss)*Ts, length(ss));

% ---- Empirical std per axis ----
labels = {'x', 'y', 'z'};
std_theory = [std_theory_x_nm, std_theory_x_nm, std_theory_z_nm];  % y uses c_para same as x

fprintf('\n=== Per-axis results ===\n');
fprintf('%-5s %-12s %-12s %-12s %-10s\n', 'axis', 'mean [nm]', 'emp std [nm]', 'theory [nm]', 'ratio');
ratio_out = zeros(1, 3);
std_emp_nm = zeros(1, 3);
mean_emp_nm = zeros(1, 3);
for ax = 1:3
    mn = mean(dxm(ax, ss));
    sd = std(dxm(ax, ss), 0);
    mean_emp_nm(ax) = 1000 * mn;
    std_emp_nm(ax) = 1000 * sd;
    ratio_out(ax) = std_emp_nm(ax) / std_theory(ax);
    fprintf('  %s   %+10.3f  %10.3f  %10.3f  %7.3f\n', ...
            labels{ax}, mean_emp_nm(ax), std_emp_nm(ax), std_theory(ax), ratio_out(ax));
end

% ---- Verdict ----
fprintf('\n=== Verdict (z-axis decisive) ===\n');
if ratio_out(3) <= 1.1
    fprintf('  RATIO_Z = %.3f  →  PASS  (static P2 at h=2.5 hits thermal floor)\n', ratio_out(3));
    fprintf('  Implication: dynamic 1.29 seen earlier was PURE dynamic lag\n');
elseif ratio_out(3) <= 1.5
    fprintf('  RATIO_Z = %.3f  →  PARTIAL  (static also has near-wall excess)\n', ratio_out(3));
    fprintf('  Implication: mix of architectural + dynamic residual near wall\n');
else
    fprintf('  RATIO_Z = %.3f  →  FAIL  (architectural near-wall limit)\n', ratio_out(3));
    fprintf('  Implication: need architectural change (Task 1e) or deeper debug\n');
end

% ---- Save ----
save('test_results/verify/p2_static_h25.mat', ...
     'dxm', 't', 'ss', 'std_emp_nm', 'mean_emp_nm', 'std_theory', 'ratio_out', ...
     'C_dpmr_eff', 'a_nom', 'sigma2_dXT', 'R', 'h_init', 'cpara', 'cperp', 'lc');
fprintf('\nData saved: test_results/verify/p2_static_h25.mat\n');
