%MAIN_RUN  Entry point for the 6-state standalone package (edit-and-run).
%
%   This is a SCRIPT, not a function: edit the "SIMULATION SETTINGS" block
%   below and press Run. No command-line arguments. One closed-loop run is
%   executed; the result struct `out` is left in the workspace, a one-line
%   tracking summary is printed, the two verification figures are saved, and
%   `out` is written to a .mat file.
%
%   For the quantitative 3-scenario PASS gate use verify_standalone; for the
%   per-part fidelity gates run test/gates_part1..7.
%
%   See also: verify_standalone, run_simulation, config, controller_6state

% ======================================================================
%  SIMULATION SETTINGS  --  edit these, then press Run
% ----------------------------------------------------------------------
scenario   = 'h50';   % 'h50' | 'h10' | 'ramp2p7'      (scenario geometry)
seed       = [];      % [] = random (printed below); or a fixed integer
T_sim      = [];      % [] = scenario default [sec]; or a number of seconds
lambda_c   = 0.7;     % closed-loop pole          (0 < lambda_c < 1)
a_pd       = 0.05;    % IIR mean-EWMA pole        (measurement chain)
a_cov      = 0.05;    % IIR variance-EWMA pole    (measurement chain)
meas_noise = true;    % sensor measurement noise  (on/off)
thermal    = true;    % thermal (Brownian) force  (on/off)
% ----------------------------------------------------------------------
%  Not exposed here (edit config.m if you must): physical constants
%  (R / gamma_N / Ts / kBT), sensor delay d = 2, wall orientation, t_hold,
%  per-axis measurement-noise std, and the scenario geometry (h_init /
%  h_bottom). Trajectory shape and the output figures are revisited in a
%  later packaging step (PACKAGING_PLAN 9c).
% ======================================================================

% --- resolve a random seed if none was given (reproducible once printed) ---
if isempty(seed)
    rng('shuffle');               % time-based, only to PICK the seed
    seed = randi(2^31 - 1);
end
seed = double(seed);              % accept an integer literal; reject junk early
assert(isscalar(seed) && isreal(seed) && seed >= 0 && seed == floor(seed), ...
       'main_run:badSeed', 'seed must be [] or a nonnegative integer.');

% --- add package folders to the path ---
here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here, 'physics'), fullfile(here, 'sim'), fullfile(here, 'test'));

% --- collect the settings into config overrides (geometry stays in config) ---
overrides = struct('lambda_c', lambda_c, 'a_pd', a_pd, 'a_cov', a_cov, ...
                   'meas_noise', meas_noise, 'thermal', thermal);
if ~isempty(T_sim); overrides.T_sim = T_sim; end

opts = struct('seed', seed, 'overrides', overrides);

fprintf(['[main_run] scenario=%s  seed=%d  lambda_c=%.3g  a_pd=%.3g  ' ...
         'a_cov=%.3g  meas_noise=%d  thermal=%d\n'], ...
        scenario, seed, lambda_c, a_pd, a_cov, meas_noise, thermal);

% --- run the closed loop ---
out = run_simulation(scenario, opts);
fprintf('[main_run] T_sim = %.3g s  (%d steps)\n', out.meta.T_run, numel(out.tout));

% --- physics ground-truth gain from the noise-free probe ---
p = out.meta.params;
a_nom  = p.common.Ts / p.common.gamma_N;
a_true = main_run_a_true(out.p_true_out, p.wall.w_hat, p.wall.pz, ...
                         p.common.R, a_nom);

% --- tracking summary (skip the hold warm-up window) ---
n_warm = round(p.traj.t_hold / p.common.Ts);
idx    = (n_warm + 1):numel(out.tout);
trk_nm = std(out.p_d_out(idx, :) - out.p_m_out(idx, :), 0, 1) * 1e3;
fprintf('[main_run] tracking std [x y z] = [%.1f %.1f %.1f] nm\n', trk_nm);

% --- output folder (provisional name; revisited in PACKAGING_PLAN 9c) ---
out_dir = fullfile(here, 'test_results');
if ~exist(out_dir, 'dir'); mkdir(out_dir); end

% --- save the run FIRST, so a later plotting error cannot lose the result
%     (this is the gap 9a closes: results used to vanish after the run).
%     The .mat name is scenario+seed, so a fixed-seed re-run overwrites it.
settings = struct('scenario', scenario, 'seed', seed, 'T_sim', out.meta.T_run, ...
                  'lambda_c', lambda_c, 'a_pd', a_pd, 'a_cov', a_cov, ...
                  'meas_noise', meas_noise, 'thermal', thermal);
mat_path = fullfile(out_dir, sprintf('run_%s_seed%d.mat', scenario, seed));
save(mat_path, 'out', 'settings');
fprintf('[main_run] result saved to %s\n', mat_path);

% --- figures (current 2; figure set revisited in PACKAGING_PLAN 9c) ---
make_figures(out, a_true, scenario, out_dir);
fprintf('[main_run] figures saved to %s\n', out_dir);


% ----------------------------------------------------------------------
function a_true = main_run_a_true(p_true, w_hat, pz, R, a_nom)
%MAIN_RUN_A_TRUE  Per-step physics ground-truth gain from the noise-free
%   probe. Columns [x y z]; x, y use c_para, z uses c_perp.
    N = size(p_true, 1);
    h_true = (p_true * w_hat - pz) / R;
    a_para = zeros(N, 1); a_perp = zeros(N, 1);
    for k = 1:N
        [cpa, cpe] = wall_corrections(max(h_true(k), 1.001));
        a_para(k) = a_nom / cpa;
        a_perp(k) = a_nom / cpe;
    end
    a_true = [a_para, a_para, a_perp];
end
