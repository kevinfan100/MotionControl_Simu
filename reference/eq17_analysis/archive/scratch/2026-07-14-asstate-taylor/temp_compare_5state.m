% TEMP_COMPARE_5STATE  Four-arm paired-seed study for the 5-state a'-as-state
% variant (aprime_source='state', 5state_taylor_aprime.pdf) on the 4-state EKF.
%
%   Scenario = compare_aprime_4state/build_config (1 Hz osc, hold 0.5 s ->
%   descend 1.0 s -> osc 2 cycles, trough h_bar = 1.2, gate-free).
%
%   Arms (same seeds across arms, paired):
%     REF : control law fed a_true (oracle gain; use_true_gain)
%     A2  : use_taylor_gain, aprime_source='known'  (oracle a')
%     A6a : use_taylor_gain, aprime_source='state'  (a' promoted to state 5, z)
%     A6b : A6a + aprime_learn_t0 = 1.5  (freeze a'-state through the descent)
%
%   Per arm: 1 det run (thermal+meas off, seed 0) + seeds 1:20 noisy.
%   Resumable: caches one .mat per (arm, seed) under
%   test_results/aprime_4state/state5/cache/; at most MAX_NEW new sims per
%   invocation. Re-run until it prints DONE.
%
%   Temp script - delete after use.

seeds   = 1:20;
MAX_NEW = 12;          % new sims per invocation (MCP timeout guard)

script_dir   = fileparts(mfilename('fullpath'));
project_root = fileparts(fileparts(script_dir));
addpath(fullfile(project_root, 'model'), fullfile(project_root, 'model', 'config'), ...
        fullfile(project_root, 'model', 'wall_effect'), ...
        fullfile(project_root, 'model', 'thermal_force'), ...
        fullfile(project_root, 'model', 'trajectory'), ...
        fullfile(project_root, 'model', 'controller'), ...
        fullfile(project_root, 'model', 'dual_track'), script_dir);

out_dir   = fullfile(project_root, 'test_results', 'aprime_4state', 'state5');
cache_dir = fullfile(out_dir, 'cache');
if ~exist(cache_dir, 'dir'); mkdir(cache_dir); end

% --- scenario (identical to compare_aprime_4state/build_config) ---
cfg = user_config();
cfg.eq17_variant    = '4state';
cfg.trajectory_type = 'osc';
cfg.h_init   = 50;
cfg.h_bottom = 2.7;
cfg.amplitude = 2.5;
cfg.frequency = 1;
cfg.n_cycles  = 2;
cfg.t_hold    = 0.5;
cfg.t_descend_override = 1.0;
cfg.T_sim     = 4.0;
pc = physical_constants();
cfg.h_min     = 1.05 * pc.R;
cfg.ctrl_enable = true;
cfg.thermal_enable = true;
cfg.meas_noise_enable = true;
cfg.lambda_c = 0.7;
cfg.a_pd  = 0.05;
cfg.a_cov = 0.05;
cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];
cfg.suppress_xD = true;
cfg.h_bar_safe = 1;

% --- arm table (name, use_true_gain, cfg overrides) ---
arm_names     = {'REF', 'A2', 'A6a', 'A6b'};
arm_truegain  = [true,  false, false, false];
arm_cfgset    = { struct(), ...
                  struct('use_taylor_gain', true, 'aprime_source', 'known'), ...
                  struct('use_taylor_gain', true, 'aprime_source', 'state'), ...
                  struct('use_taylor_gain', true, 'aprime_source', 'state', ...
                         'aprime_learn_t0', 1.5) };

% --- run missing (arm, seed) pairs, up to MAX_NEW this invocation ---
n_new  = 0;
n_left = 0;
for ai = 1:numel(arm_names)
    arm = arm_names{ai};
    cfg_arm = cfg;
    fn = fieldnames(arm_cfgset{ai});
    for i = 1:numel(fn); cfg_arm.(fn{i}) = arm_cfgset{ai}.(fn{i}); end
    use_tg = arm_truegain(ai);

    all_seeds = [0, seeds];           % 0 = det run
    for s = all_seeds
        is_det  = (s == 0);
        cache_f = fullfile(cache_dir, sprintf('%s_s%02d.mat', arm, s));
        if exist(cache_f, 'file'); continue; end
        if n_new >= MAX_NEW; n_left = n_left + 1; continue; end

        cfg_run = cfg_arm;
        if is_det
            cfg_run.thermal_enable = false; cfg_run.meas_noise_enable = false;
        end
        ro = struct('seed', s, 'verbose', false, 'collect_diag', true, ...
                    'use_true_gain', use_tg);

        rec = struct('arm', arm, 'seed', s, 'is_det', is_det, ...
                     'diverged', false, 'diverge_reason', '');
        t_run = tic;
        try
            so = run_pure_simulation(cfg_run, ro);
            e  = so.p_d_out(2:end, :) - so.p_true_out(1:end-1, :);
            if ~all(isfinite(e(:)))
                rec.diverged = true; rec.diverge_reason = 'non-finite trajectory';
            elseif max(abs(e(:))) > 0.5
                rec.diverged = true;
                rec.diverge_reason = sprintf('|e|_max = %.3g um > 0.5 um', max(abs(e(:))));
            end
            rec.dx        = [0; so.p_d_out(2:end, 3) - so.p_true_out(1:end-1, 3)];
            rec.a_hat_z   = so.diag.a_hat(:, 3);
            rec.a_true_z  = so.a_true_out(:, 3);
            rec.aprime_z  = so.diag.a_prime_used(:, 3);
            if isfield(so.diag, 'P_aprime')
                rec.P55_z = so.diag.P_aprime(:, 3);
            else
                rec.P55_z = zeros(numel(so.tout), 1);
            end
            rec.fdz = so.f_d_out(:, 3);
            rec.t   = so.tout;
        catch err
            rec.diverged = true;
            rec.diverge_reason = sprintf('crash: %s', err.message);
            rec.dx = []; rec.a_hat_z = []; rec.a_true_z = []; rec.aprime_z = [];
            rec.P55_z = []; rec.fdz = []; rec.t = [];
        end
        save(cache_f, '-struct', 'rec');
        n_new = n_new + 1;
        fprintf('[compare5] %-3s seed %2d %s done (div=%d, %.1f s)\n', ...
                arm, s, ternary(is_det, 'det', 'noisy'), rec.diverged, toc(t_run));
    end
end

if n_left > 0
    fprintf('PROGRESS: %d new this call, %d still missing - re-run script\n', n_new, n_left);
    return;
end
fprintf('DONE: all %d runs cached under %s\n', numel(arm_names) * (numel(seeds) + 1), cache_dir);

function s = ternary(c, a, b)
    if c; s = a; else; s = b; end
end
