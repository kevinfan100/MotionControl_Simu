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
%   Derivation: reference/eq17_analysis/derivation/4state_del_hd.tex taylor section.
%   Per arm: 1 det run (noise off) + numel(seeds) noisy runs, collect_diag.
%   Output: <out_root>/f1Hz[-smoke]/runs.mat  (test_results/ is gitignored).
%
%   opts fields (all optional):
%     seeds    (1:20)   - RNG seeds for noisy runs
%     T_sim    (4.0)    - simulation duration [s]
%     out_root (test_results/aprime_4state) - output root
%     smoke    (false)  - true -> seeds=1 only, saved under -smoke suffix
%     arms     (all)    - cellstr subset of {REF,A0,A1,A2,A3,A4} to run
%     y2_noise_model    - 'white' (default) | 'ar1' colored-y2 augmentation,
%                         applied to ALL selected arms (chat 2026-07-12)
%     use_c2_level      - true adds the C2 y_3 = p_md level channel (needs
%                         'ar1'; chat 2026-07-13), applied to selected arms
%     out_tag  ('')     - suffix for the output dir: f1Hz-<tag>
%     verbose  (true)
%
%   See also: analyze_aprime_4state, run_pure_simulation, compare_gain_6state

    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'seeds');   opts.seeds = 1:20;  end
    if ~isfield(opts, 'T_sim');   opts.T_sim = 4.0;   end
    if ~isfield(opts, 'verbose'); opts.verbose = true; end
    if ~isfield(opts, 'smoke');   opts.smoke = false;  end
    if ~isfield(opts, 'out_tag'); opts.out_tag = '';   end
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
    if isfield(opts, 'arms') && ~isempty(opts.arms)
        keep = ismember({arms.name}, opts.arms);
        assert(any(keep), 'compare_aprime_4state:badArms', ...
               'opts.arms matched no arm in {%s}.', strjoin({arms.name}, ','));
        arms = arms(keep);
    end
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
    if ~isempty(opts.out_tag)
        out_dir = [out_dir, '-', opts.out_tag];
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
    if isfield(opts, 'n_cycles') && ~isempty(opts.n_cycles)
        assert(isscalar(opts.n_cycles) && opts.n_cycles > 0 && ...
               opts.n_cycles == round(opts.n_cycles), ...
               'compare_aprime_4state:badNCycles', ...
               'opts.n_cycles must be a positive integer.');
        cfg.n_cycles = opts.n_cycles;   % long-run enabler (chat 2026-07-14)
    else
        cfg.n_cycles = 2;            % osc duration = n_cycles / frequency [s]
    end
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
    if isfield(opts, 'y2_noise_model') && ~isempty(opts.y2_noise_model)
        cfg.y2_noise_model = opts.y2_noise_model;   % colored-y2 knob (chat 2026-07-12)
        % No G1 blanket: the -10% warmup slug traced to the first d = 2
        % buffer-init samples (artifacts, not data); the controller now holds
        % the IIR for those d steps in ar1 mode (init-correct, user directive
        % 2026-07-13). t_warmup_kf stays at the project default (0).
    end
    if isfield(opts, 'use_c2_level') && ~isempty(opts.use_c2_level)
        cfg.use_c2_level = logical(opts.use_c2_level);   % C2 y_3 channel (chat 2026-07-13)
    end
    if isfield(opts, 'use_y2_mirror_h') && ~isempty(opts.use_y2_mirror_h)
        cfg.use_y2_mirror_h = logical(opts.use_y2_mirror_h);   % derived y2 sensitivity H (chat 2026-07-13)
    end
    if isfield(opts, 'r22_eval_adet') && ~isempty(opts.r22_eval_adet)
        cfg.r22_eval_adet = logical(opts.r22_eval_adet);   % PROBE: rectification test (chat 2026-07-13)
    end
    if isfield(opts, 'fe_eval_fdet') && ~isempty(opts.fe_eval_fdet)
        cfg.fe_eval_fdet = logical(opts.fe_eval_fdet);   % PROBE: F_e-from-f_det rectification test (chat 2026-07-13)
    end
    if isfield(opts, 'y2_oracle_meas') && ~isempty(opts.y2_oracle_meas)
        cfg.y2_oracle_meas = logical(opts.y2_oracle_meas);   % PROBE: knife-2 oracle y_2 (chat 2026-07-14)
    end
    if isfield(opts, 'use_gamma_split') && ~isempty(opts.use_gamma_split)
        cfg.use_gamma_split = logical(opts.use_gamma_split);   % z-axis level/shape split (chat 2026-07-14)
        % Freeze window [t0, t1). Standard (default): freeze [0, t_hold+t_descend)
        % -> learn gamma only during oscillation (== the original descent gate).
        cfg.gamma_learn_start = cfg.t_hold + cfg.t_descend_override;
        if isfield(opts, 'gamma_hold_learn') && opts.gamma_hold_learn
            % Discriminating variant: gamma LEARNS during the far-field hold
            % [0, t_hold) (zero curvature, cleanest level info), freezes only
            % through the descent [t_hold, t_hold+t_descend), resumes at osc.
            cfg.gamma_freeze_t0 = cfg.t_hold;
            cfg.gamma_freeze_t1 = cfg.t_hold + cfg.t_descend_override;
        end
    end
    if isfield(opts, 'init_from_anom') && ~isempty(opts.init_from_anom)
        cfg.init_from_anom = logical(opts.init_from_anom);   % a_nom-normalized init (chat 2026-07-14)
    end
    if isfield(opts, 'gamma_hbar_min') && ~isempty(opts.gamma_hbar_min)
        cfg.gamma_hbar_min = opts.gamma_hbar_min;   % near-wall gamma height gate (chat 2026-07-14)
    end
    if isfield(opts, 'use_colored_eps') && ~isempty(opts.use_colored_eps)
        cfg.use_colored_eps = logical(opts.use_colored_eps);   % MA(2) thermal-history augmentation (chat 2026-07-14)
    end
    if isfield(opts, 'gamma_shared') && ~isempty(opts.gamma_shared)
        cfg.gamma_shared = logical(opts.gamma_shared);   % 3-axis shared-gamma federated fusion (chat 2026-07-14)
    end
    if isfield(opts, 'shared_xy_y2_off') && ~isempty(opts.shared_xy_y2_off)
        cfg.shared_xy_y2_off = logical(opts.shared_xy_y2_off);   % DIAGNOSTIC (chat 2026-07-14)
    end
    if isfield(opts, 'sigma_gamma0') && ~isempty(opts.sigma_gamma0)
        cfg.sigma_gamma0 = opts.sigma_gamma0;   % diffuse gamma init std (ln units)
    end
    if isfield(opts, 'gamma_learn_start') && ~isempty(opts.gamma_learn_start)
        cfg.gamma_learn_start = opts.gamma_learn_start;   % explicit override
    end
    if isfield(opts, 'gamma_freeze_t0') && ~isempty(opts.gamma_freeze_t0)
        cfg.gamma_freeze_t0 = opts.gamma_freeze_t0;   % explicit freeze-window override
    end
    if isfield(opts, 'gamma_freeze_t1') && ~isempty(opts.gamma_freeze_t1)
        cfg.gamma_freeze_t1 = opts.gamma_freeze_t1;
    end
    if isfield(opts, 'h_bar_safe') && ~isempty(opts.h_bar_safe)
        cfg.h_bar_safe = opts.h_bar_safe;   % G3 near-wall y_2 gate override
    end
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
                % G1 warmup rows (t < t_warmup_kf) are expected when the ar1
                % colored-y2 mode enables the warmup shield; beyond warmup the
                % run must stay gate-free (h_bar_safe = 1, trough 1.2).
                ga = r.simOut.diag.gate_active;
                tck = r.simOut.tout(:);
                dtk = tck(2) - tck(1);
                ga(tck < r.simOut.ctrl_const.t_warmup_kf + 2*dtk, :) = false;
                if r.simOut.ctrl_const.h_bar_safe <= 1.2
                    % gate-free expectation only holds when G3 sits below the
                    % trough (h_bar_safe override probes near-wall gating).
                    assert(~any(ga(:)), ...
                           'Layer0: gate fired beyond warmup in expected-gate-free run (arm %s seed %d)', nm, r.seed);
                end
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
