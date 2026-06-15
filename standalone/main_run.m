%MAIN_RUN  Entry point for the 6-state standalone package (edit-and-run).
%
%   This is a SCRIPT, not a function: edit the "SIMULATION SETTINGS" block
%   below and press Run. No command-line arguments. One closed-loop run is
%   executed; the result struct `out` is left in the workspace, a one-line
%   tracking summary is printed, the two verification figures are saved, and
%   `out` is written to a .mat file.
%
%   The RNG seed is auto-random each run and printed/saved (out.meta.seed).
%   To reproduce a specific run, call run_simulation directly with its seed:
%       out = run_simulation('h50', struct('seed', 123456789));
%
%   For the quantitative 3-scenario PASS gate use verify_standalone; for the
%   per-part fidelity gates run test/gates_part1..7.
%
%   See also: verify_standalone, run_simulation, config, controller_6state

% ======================================================================
%  SIMULATION SETTINGS  --  edit these, then press Run
% ----------------------------------------------------------------------
scenario   = 'h50';   % 'h50' | 'h10' | 'osc1hz'       (scenario / trajectory)
lambda_c   = 0.7;     % closed-loop pole          (0 < lambda_c < 1)
a_pd       = 0.05;    % IIR mean-EWMA pole        (0 < a_pd <= 1)
a_cov      = 0.05;    % IIR variance-EWMA pole    (0 < a_cov <= 1)
meas_noise = true;    % sensor measurement noise  (on/off)
thermal    = true;    % thermal (Brownian) force  (on/off)
% ----------------------------------------------------------------------
%  Not exposed here (edit config.m if you must): physical constants
%  (R / gamma_N / Ts / kBT), sensor delay d = 2, wall orientation, the
%  scenario geometry (h_init / h_bottom) and timing (T_sim / t_hold), and
%  the per-axis measurement-noise std. The RNG seed is auto-random each run
%  (printed and saved below). Trajectory shape and the output figures are
%  revisited in a later packaging step (PACKAGING_PLAN 9c).
% ======================================================================

% --- validate the user-facing tunables (a clear error beats a silent diverge)
assert(lambda_c > 0 && lambda_c < 1, 'main_run:lambda_c', 'lambda_c must be in (0,1).');
assert(a_pd  > 0 && a_pd  <= 1, 'main_run:a_pd',  'a_pd must be in (0,1].');
assert(a_cov > 0 && a_cov <= 1, 'main_run:a_cov', 'a_cov must be in (0,1].');

% --- add package folders to the path ---
here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here, 'physics'), fullfile(here, 'sim'), fullfile(here, 'test'));

% --- collect the settings into config overrides (geometry/timing stay in config) ---
overrides = struct('lambda_c', lambda_c, 'a_pd', a_pd, 'a_cov', a_cov, ...
                   'meas_noise', meas_noise, 'thermal', thermal);
opts = struct('overrides', overrides);   % no seed -> run_simulation auto-randomizes

fprintf(['[main_run] scenario=%s  lambda_c=%.3g  a_pd=%.3g  a_cov=%.3g  ' ...
         'meas_noise=%d  thermal=%d\n'], ...
        scenario, lambda_c, a_pd, a_cov, meas_noise, thermal);

% --- run the closed loop ---
out  = run_simulation(scenario, opts);
seed = out.meta.seed;                     % the auto-random seed actually used
fprintf('[main_run] seed=%d  T_sim=%.3g s  (%d steps)\n', ...
        seed, out.meta.T_run, numel(out.tout));

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
%     The .mat name is scenario+seed; with the auto-random seed each run
%     leaves a distinct file (overwrite only if run_simulation is called
%     directly with a repeated seed).
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
