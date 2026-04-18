% single_AB_run.m
% Per-seed runner for verify_AB_metrics. Workspace inputs (set by -batch):
%   h_init   (scalar, um)
%   seed     (integer)
%   out_tag  (string, used for output filename)

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(script_dir);
cd(project_root);

addpath(fullfile(project_root, 'model'));
addpath(fullfile(project_root, 'model', 'config'));
addpath(fullfile(project_root, 'model', 'wall_effect'));
addpath(fullfile(project_root, 'model', 'thermal_force'));
addpath(fullfile(project_root, 'model', 'trajectory'));
addpath(fullfile(project_root, 'model', 'controller'));

constants = physical_constants();
Ts      = constants.Ts;
gamma_N = constants.gamma_N;
a_nom   = Ts / gamma_N;
R_probe = constants.R;

T_sim = 30;
lc    = 0.7;

config = user_config();
config.h_init             = h_init;
config.amplitude          = 0;
config.enable_wall_effect = true;
config.h_min              = max(h_init - 0.2, 1.5*R_probe);
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

% IMPORTANT: rng(seed) MUST be set before calc_simulation_params, because
% calc_thermal_params.m generates thermal.seed via randi() at params-build
% time. If rng is set after, parallel matlab instances all get the same
% randi result and produce bit-identical simulations.
rng(seed);
params = calc_simulation_params(config);

assignin('base', 'params', params);
assignin('base', 'p0', params.Value.common.p0);
assignin('base', 'Ts', Ts);

model_path = fullfile(project_root, 'model', 'system_model');
t0 = tic;
simOut = sim(model_path, 'StopTime', num2str(T_sim), ...
    'SaveTime', 'on', 'TimeSaveName', 'tout', ...
    'SaveOutput', 'on', 'OutputSaveName', 'yout');
sim_time_s = toc(t0);

p_d = simOut.p_d_out';
p_m = simOut.p_m_out';
ekf = simOut.ekf_out';
N = size(p_d, 2);
ss = round(10/Ts):N;

h_bar = h_init / R_probe;
[cpara, cperp] = calc_correction_functions(h_bar);
a_x_true = a_nom / cpara;
a_z_true = a_nom / cperp;

dx_x = p_d(1, ss) - p_m(1, ss);
dx_z = p_d(3, ss) - p_m(3, ss);
track_x_mean_nm = 1000 * mean(dx_x);
track_x_std_nm  = 1000 * std(dx_x);
track_z_mean_nm = 1000 * mean(dx_z);
track_z_std_nm  = 1000 * std(dx_z);

a_hat_x = ekf(1, ss);
a_hat_z = ekf(2, ss);
ahat_x_mean_pct = 100 * (mean(a_hat_x) - a_x_true) / a_x_true;
ahat_x_std_pct  = 100 * std(a_hat_x) / a_x_true;
ahat_z_mean_pct = 100 * (mean(a_hat_z) - a_z_true) / a_z_true;
ahat_z_std_pct  = 100 * std(a_hat_z) / a_z_true;

out_path = sprintf('test_results/verify/AB_run_%s.mat', out_tag);
save(out_path, 'h_init', 'seed', 'cpara', 'cperp', 'a_x_true', 'a_z_true', ...
     'track_x_mean_nm', 'track_x_std_nm', 'track_z_mean_nm', 'track_z_std_nm', ...
     'ahat_x_mean_pct', 'ahat_x_std_pct', 'ahat_z_mean_pct', 'ahat_z_std_pct', ...
     'sim_time_s');

fprintf('[%s h=%.1f seed=%d] sim=%.0fs trk_x=%+.2f/%.2f trk_z=%+.2f/%.2f ahx=%+.1f/%.1f%% ahz=%+.1f/%.1f%%\n', ...
    out_tag, h_init, seed, sim_time_s, ...
    track_x_mean_nm, track_x_std_nm, track_z_mean_nm, track_z_std_nm, ...
    ahat_x_mean_pct, ahat_x_std_pct, ahat_z_mean_pct, ahat_z_std_pct);
