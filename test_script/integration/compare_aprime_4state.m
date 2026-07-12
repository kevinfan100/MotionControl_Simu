function results = compare_aprime_4state(opts)
%COMPARE_APRIME_4STATE Six-arm taylor-gain study on the 4-state EKF (1 Hz).
%
%   results = compare_aprime_4state()
%   results = compare_aprime_4state(opts)
%
%   Runs the standard 1 Hz osc scenario (hold 0.5 s -> descend 1.0 s ->
%   osc 2 cycles, trough h_bar = 1.2, gate-free) through six controller
%   arms, same seeds across arms (paired design):
%
%     REF : a = a_true fed to the control law (oracle gain; EKF still runs)
%     A0  : production AR(1) reverting gain (use_q44_ar1, a_det anchor)
%     A1  : RW + da_x_pred feedforward (all knobs off; taylor baseline)
%     A2  : taylor-gain suite, aprime_source = 'known'  (oracle a')
%     A3  : taylor-gain suite, aprime_source = 'ahat'   (level self-anchored)
%     A4  : taylor-gain suite, aprime_source = 'diff'   (model-free LS slope)
%
%   Derivation: reference/eq17_analysis/derivation/4state_taylor_gain.tex.
%   Per arm: 1 det run (noise off) + numel(seeds) noisy runs, collect_diag.
%   Output: <out_root>/f1Hz[-smoke]/runs.mat  (test_results/ is gitignored).
%
%   opts fields (all optional):
%     seeds    (1:20)   - RNG seeds for noisy runs
%     T_sim    (4.0)    - simulation duration [s]
%     out_root (test_results/aprime_4state) - output root
%     smoke    (false)  - true -> seeds=1 only, saved under -smoke suffix
%     verbose  (true)
%
%   See also: analyze_aprime_4state, run_pure_simulation, compare_gain_6state

    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'seeds');   opts.seeds = 1:20;  end
    if ~isfield(opts, 'T_sim');   opts.T_sim = 4.0;   end
    if ~isfield(opts, 'verbose'); opts.verbose = true; end
    if ~isfield(opts, 'smoke');   opts.smoke = false;  end
    if opts.smoke; opts.seeds = 1; end

    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(fileparts(script_dir));
    addpath(fullfile(project_root, 'model'), fullfile(project_root, 'model', 'config'), ...
            fullfile(project_root, 'model', 'wall_effect'), ...
            fullfile(project_root, 'model', 'thermal_force'), ...
            fullfile(project_root, 'model', 'trajectory'), ...
            fullfile(project_root, 'model', 'controller'), ...
            fullfile(project_root, 'model', 'dual_track'), script_dir);
    if ~isfield(opts, 'out_root')
        opts.out_root = fullfile(project_root, 'test_results', 'aprime_4state');
    end

    cfg = build_config(opts);

    % pre-flight trajectory safety
    params = calc_simulation_params(cfg);
    [is_safe, h_min_actual, t_crit] = check_trajectory_safety(params.Value);
    assert(is_safe, 'Trajectory unsafe: h_min_actual=%.3f um at t=%.2f s', ...
           h_min_actual, t_crit);

    arms = arm_table();
    if opts.verbose
        fprintf('[compare_aprime:1Hz] T_sim=%.1fs seeds=%s arms=%s\n', ...
                cfg.T_sim, mat2str(opts.seeds), strjoin({arms.name}, ','));
    end

    runs = struct();
    for ai = 1:numel(arms)
        arm = arms(ai);
        cfg_arm = apply_arm(cfg, arm);
        cfg_det = cfg_arm;
        cfg_det.thermal_enable = false; cfg_det.meas_noise_enable = false;
        runs.(arm.name).det = run_one(cfg_det, 0, arm.use_true_gain, true);
        for s = 1:numel(opts.seeds)
            runs.(arm.name).noisy(s) = run_one(cfg_arm, opts.seeds(s), arm.use_true_gain, false);
        end
        if opts.verbose
            nd = sum([runs.(arm.name).noisy.diverged]) + runs.(arm.name).det.diverged;
            fprintf('  arm %-3s done (diverged %d/%d)\n', arm.name, nd, numel(opts.seeds) + 1);
        end
    end

    % --- wiring assertions (paired design integrity) ---
    layer0 = layer0_checks(runs, arms);

    if opts.smoke
        out_dir = fullfile(opts.out_root, 'f1Hz-smoke');
    else
        out_dir = fullfile(opts.out_root, 'f1Hz');
    end
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end
    save(fullfile(out_dir, 'runs.mat'), 'runs', 'cfg', 'opts', 'layer0', '-v7.3');
    n_div = count_diverged(runs, arms);
    if opts.verbose
        fprintf('  Layer 0: PASS\n  saved %s  (diverged runs: %d)\n', out_dir, n_div);
    end
    results = struct('out_dir', out_dir, 'layer0', layer0, 'n_diverged', n_div);
end


function arms = arm_table()
    arms = struct('name', {}, 'use_true_gain', {}, 'cfg_set', {});
    arms(1) = struct('name', 'REF', 'use_true_gain', true,  'cfg_set', struct());
    arms(2) = struct('name', 'A0',  'use_true_gain', false, 'cfg_set', struct('use_q44_ar1', true));
    arms(3) = struct('name', 'A1',  'use_true_gain', false, 'cfg_set', struct());
    arms(4) = struct('name', 'A2',  'use_true_gain', false, ...
                     'cfg_set', struct('use_taylor_gain', true, 'aprime_source', 'known'));
    arms(5) = struct('name', 'A3',  'use_true_gain', false, ...
                     'cfg_set', struct('use_taylor_gain', true, 'aprime_source', 'ahat'));
    arms(6) = struct('name', 'A4',  'use_true_gain', false, ...
                     'cfg_set', struct('use_taylor_gain', true, 'aprime_source', 'diff'));
end


function cfg_arm = apply_arm(cfg, arm)
    cfg_arm = cfg;
    fn = fieldnames(arm.cfg_set);
    for i = 1:numel(fn)
        cfg_arm.(fn{i}) = arm.cfg_set.(fn{i});
    end
end


function cfg = build_config(opts)
%BUILD_CONFIG 1 Hz osc_aggr scenario (same as compare_gain_6state f=1).
    cfg = user_config();
    cfg.eq17_variant    = '4state';
    cfg.trajectory_type = 'osc';
    cfg.h_init   = 50;            % [um] h_bar ~ 22
    cfg.h_bottom = 2.7;           % [um] h_bar = 1.2
    cfg.amplitude = 2.5;          % [um]
    cfg.frequency = 1;
    cfg.n_cycles  = 2;            % osc duration = 2 s
    cfg.t_hold    = 0.5;
    cfg.t_descend_override = 1.0;
    cfg.T_sim     = opts.T_sim;
    if ~opts.smoke
        assert(opts.T_sim >= cfg.t_hold + cfg.t_descend_override + cfg.n_cycles / cfg.frequency, ...
               'build_config: T_sim=%.2f s truncates the osc phase', opts.T_sim);
    end
    pc = physical_constants();
    cfg.h_min     = 1.05 * pc.R;
    cfg.ctrl_enable = true;
    cfg.thermal_enable = true;
    cfg.meas_noise_enable = true;
    cfg.lambda_c = 0.7;
    cfg.a_pd  = 0.05;
    cfg.a_cov = 0.05;
    cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];   % [um]
    cfg.suppress_xD = true;
    cfg.h_bar_safe = 1;           % gate-free (trough h_bar = 1.2)
end


function rec = run_one(cfg, seed, use_true_gain, is_det)
%RUN_ONE Single run wrapped in try/catch (crash = diverged).
    if nargin < 4; is_det = false; end
    ro = struct('seed', seed, 'verbose', false, 'collect_diag', true, ...
                'use_true_gain', use_true_gain);
    rec = struct('seed', seed, 'use_true_gain', use_true_gain, 'is_det', is_det, ...
                 'diverged', false, 'diverge_reason', '', 'simOut', []);
    try
        rec.simOut = run_pure_simulation(cfg, ro);
    catch err
        rec.diverged = true;
        rec.diverge_reason = sprintf('crash: %s', err.message);
        return;
    end
    e = rec.simOut.p_d_out(2:end, :) - rec.simOut.p_true_out(1:end-1, :);
    if ~all(isfinite(e(:)))
        % max() ignores NaN, so a finite->NaN blow-up would silently pass the
        % threshold test below (audit-harness finding [1]) — catch it first.
        rec.diverged = true;
        rec.diverge_reason = 'non-finite trajectory (NaN/Inf)';
    elseif max(abs(e(:))) > 0.5
        rec.diverged = true;
        rec.diverge_reason = sprintf('|e|_max = %.3g um > 0.5 um', max(abs(e(:))));
    end
end


function layer0 = layer0_checks(runs, arms)
%LAYER0_CHECKS p_d identity across arms + REF wiring + gate-free noisy runs.
    layer0 = struct('pd_identical', true, 'ref_wiring', true, 'n_checked', 0);
    pd_ref = [];
    for ai = 1:numel(arms)
        nm = arms(ai).name;
        recs = [runs.(nm).det, runs.(nm).noisy];
        for r = recs
            if isempty(r.simOut) || r.diverged; continue; end
            layer0.n_checked = layer0.n_checked + 1;
            if isempty(pd_ref)
                pd_ref = r.simOut.p_d_out;
            else
                assert(isequal(size(r.simOut.p_d_out), size(pd_ref)) && ...
                       max(abs(r.simOut.p_d_out(:) - pd_ref(:))) == 0, ...
                       'Layer0: p_d_out differs (arm %s seed %d)', nm, r.seed);
            end
            if strcmp(nm, 'REF')
                assert(max(abs(r.simOut.diag.a_ctrl_used(:) - r.simOut.a_true_out(:))) == 0, ...
                       'Layer0: REF a_ctrl_used ~= a_true_out (seed %d)', r.seed);
            end
            if ~r.is_det
                assert(~any(r.simOut.diag.gate_active(:)), ...
                       'Layer0: gate fired in expected-gate-free run (arm %s seed %d)', nm, r.seed);
            end
        end
    end
end


function n = count_diverged(runs, arms)
    n = 0;
    for ai = 1:numel(arms)
        nm = arms(ai).name;
        n = n + runs.(nm).det.diverged + sum([runs.(nm).noisy.diverged]);
    end
end
