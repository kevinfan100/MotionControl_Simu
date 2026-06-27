function out = verify_ahat_ladder(varargin)
%VERIFY_AHAT_LADDER  Attribution "ladder" for the a_x gain-estimation error.
%
%   out = verify_ahat_ladder()                 % run all 5 rungs, default knobs
%   out = verify_ahat_ladder(opts)             % opts struct (see below)
%   out = verify_ahat_ladder('rungs', 0:1, ...)% name-value pairs
%
%   DIAGNOSTIC harness ONLY. Reads the production 4-state controller
%   (motion_control_law_eq17_4state) through run_pure_simulation; it does NOT
%   modify any production code. The motion gain
%       a_x,i = Ts / (gamma_N * c_i(h_bar))
%   is estimated from the a_xm chain (residual variance -> linear inversion).
%   The estimate is imperfect; this harness ATTRIBUTES the residual to layers
%   by adding ONE effect per rung and reporting two metrics per axis (focus on
%   z = wall-normal, worst case; x = wall-parallel reported alongside):
%
%       BIAS   = mean_seeds( estimate / a_true - 1 )      (systematic)
%       SPREAD = across-seed std( estimate / a_true )     (random; the
%                deterministic a_true(t) swing cancels across seeds)
%
%   "estimate" is the RAW diag.a_xm for Rungs 0-2 (oracle control: the loop is
%   open w.r.t. the gain, so a_xm is the bare measurement), and the EKF a_hat
%   for Rungs 3-4 (closed loop: the estimate feeds back into the control law).
%   a_true is taken from simOut.a_true_out (the true gain the driver used at the
%   pre-integration position).
%
%   The 5 rungs:
%     Rung 0  STATIC + FAR   + ORACLE, noise ON, sweep a_cov.
%             Validates KF/chain mechanics + confirms the spread is the Fisher
%             floor sqrt(K_var*IF_eff)*(a+xi)/a, not a tuning artifact.
%             PASS: |bias|<BIAS_PASS_THRESH AND spread within SPREAD_REL_TOL of
%             the floor across the sweep.
%     Rung 1  STATIC + NEAR  + ORACLE, noise ON, same a_cov sweep.
%             Oracle gain -> a_xm stays unbiased even near the wall; SPREAD
%             grows (low SNR). PASS: |bias|<BIAS_PASS_THRESH (bias here would
%             flag a sigma_n / chain bug).
%     Rung 2  DYNAMIC (1 Hz, near-wall trough) + ORACLE.
%             EXPOSES the dynamic-window aliasing bias: mean(a_xm)/a_true vs
%             trajectory phase. (No treatment; measured only.)
%     Rung 3  DYNAMIC + FAR-ish + CLOSED LOOP (real EKF a_hat).
%             EXPOSES the closed-loop coupling bias once a_hat feeds back, away
%             from the near-wall low-SNR confound.
%     Rung 4  DYNAMIC + NEAR wall + CLOSED LOOP. The full case. Bias + spread.
%
%   opts fields (all defaulted):
%     rungs        (0:4)                 subset of rungs to run
%     seeds        (1:20)                RNG seeds (across-seed metric needs >~20)
%     a_cov_sweep  ([0.01 0.02 0.05 0.1])a_cov values for Rungs 0/1
%     a_cov_nom    (0.05)                a_cov for Rungs 2/3/4
%     a_pd         (0.05)                IIR LP weight (held FIXED across the
%                                        a_cov sweep so the sweep isolates a_cov)
%     lambda_c     (0.7)                 closed-loop pole
%     verbose      (true)
%     smoke        (false)               quick check: seeds=1:3, a_cov=[.02 .05]
%     out_root     (<proj>/test_results/ahat_ladder)
%
%   a_cov OVERRIDE PATH: a_cov is a first-class config field threaded through
%   run_pure_simulation -> build_eq17_6state_constants, so the sweep simply sets
%   cfg.a_cov AFTER user_config() (whose apply_qr_preset would otherwise pin
%   a_cov=0.005). No controller edit is needed. a_pd is held fixed at opts.a_pd
%   (NOT defaulted to a_cov) so the sweep varies a_cov alone.
%
%   The floor line is evaluated from each run's own simOut.ctrl_const
%   (K_var, IF_abc, C_dpmr, C_n, xi_per_axis, kBT) at the steady-state true
%   gain, i.e. the controller's own R22_intrinsic model. A measured spread that
%   lands on this line confirms the spread is the Fisher floor.
%
%   Figures (test_results/ahat_ladder/, gitignored):
%     fig_rung0_fisher_floor, fig_rung1_nearwall_spread, fig_rung2_aliasing,
%     fig_rung3_loop_coupling, fig_rung4_full
%   Plus a printed/returned SUMMARY TABLE.
%
%   See also: run_pure_simulation, motion_control_law_eq17_4state,
%             compare_gain_6state, verify_eq17_gscalar_suite,
%             build_eq17_6state_constants

    % ------------------------------------------------------------------
    % Options
    % ------------------------------------------------------------------
    opts = parse_opts(varargin{:});

    % ------------------------------------------------------------------
    % Named constants (no magic numbers downstream)
    % ------------------------------------------------------------------
    K.BIAS_PASS_THRESH = 0.02;     % |bias| pass threshold (Rungs 0/1)        [-]
    K.SPREAD_REL_TOL   = 0.15;     % |spread/floor - 1| tolerance (Rung 0)    [-]
    K.SMOOTH_SEC       = 0.02;     % movmean window for time-series figures    [s]
    K.PHASE_NBINS      = 24;       % phase bins for the Rung 2 aliasing fold   [-]
    K.DIVERGE_UM       = 0.5;      % |aligned error| divergence cutoff        [um]
    K.NEAR_HBAR        = 1.2;      % near-wall static target h_bar             [-]
    K.FAR_HINIT_UM     = 50;       % far / start height                       [um]
    K.NEAR_BOTTOM_UM   = 2.7;      % near-wall osc trough (h_bar ~ 1.2)        [um]
    K.FARISH_BOTTOM_UM = 6.0;      % far-ish osc trough (h_bar ~ 2.67)         [um]
    K.OSC_AMP_UM       = 2.5;      % osc half-amplitude                       [um]
    K.OSC_FREQ_HZ      = 1.0;      % osc frequency                            [Hz]
    K.OSC_NCYC         = 3;        % osc cycles                                [-]
    K.T_HOLD_S         = 0.5;      % initial hold                              [s]
    K.T_DESCEND_S      = 1.0;      % descent duration                          [s]
    K.T_STEADY_STAT_S  = 1.0;      % static steady-state window start          [s]
    K.MEAS_NOISE_STD   = [0.00062; 0.00057; 0.00331];   % per-axis sensor std [um]
    K.AX_X             = 1;        % a_true_out / diag column for x (parallel)
    K.AX_Z             = 3;        % a_true_out / diag column for z (normal)

    pc = physical_constants();
    K.R_UM = pc.R;                 % particle radius [um] (sets near/far h_bar)

    % ------------------------------------------------------------------
    % Path + output dir
    % ------------------------------------------------------------------
    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(fileparts(script_dir));
    addpath(fullfile(project_root, 'model'), ...
            fullfile(project_root, 'model', 'config'), ...
            fullfile(project_root, 'model', 'wall_effect'), ...
            fullfile(project_root, 'model', 'thermal_force'), ...
            fullfile(project_root, 'model', 'trajectory'), ...
            fullfile(project_root, 'model', 'controller'), ...
            fullfile(project_root, 'model', 'dual_track'), script_dir);
    if isempty(opts.out_root)
        opts.out_root = fullfile(project_root, 'test_results', 'ahat_ladder');
    end
    if ~exist(opts.out_root, 'dir'); mkdir(opts.out_root); end

    if opts.verbose
        fprintf('\n=== verify_ahat_ladder ===\n');
        fprintf('  rungs       : %s\n', mat2str(opts.rungs));
        fprintf('  seeds       : %s\n', mat2str(opts.seeds));
        fprintf('  a_cov sweep : %s   (a_pd fixed = %.3g)\n', ...
                mat2str(opts.a_cov_sweep), opts.a_pd);
        fprintf('  a_cov nom   : %.3g\n', opts.a_cov_nom);
        fprintf('  out_root    : %s\n\n', opts.out_root);
    end

    out = struct('opts', opts, 'rung', struct(), 'figs', {{}}, 'summary', []);
    summary = struct('rung', {}, 'regime', {}, 'bias_z', {}, 'spread_z', {}, ...
                     'floor_z', {}, 'pass', {});

    % ==================================================================
    % Rung 0 : STATIC + FAR + ORACLE  (a_cov sweep)
    % ==================================================================
    if any(opts.rungs == 0) || any(opts.rungs == 1)
        % Rungs 0 and 1 share the a_cov sweep; run whichever are requested.
        if any(opts.rungs == 0)
            rg0 = regime_static_far(K);
            S0 = run_sweep_rung(rg0, opts, K);
            out.rung.r0 = S0;
            p0 = all(abs(S0.bias_z) < K.BIAS_PASS_THRESH) && ...
                 max(abs(S0.spread_z ./ S0.floor_z - 1)) < K.SPREAD_REL_TOL;
            f0 = fig_rung0_fisher_floor(S0, opts, K);
            out.figs{end+1} = f0;
            summary(end+1) = mk_row(0, rg0.label, S0, opts, p0); %#ok<AGROW>
            if opts.verbose; report_sweep('Rung 0 (far)', S0, K); end
        end
        if any(opts.rungs == 1)
            rg1 = regime_static_near(K);
            S1 = run_sweep_rung(rg1, opts, K);
            out.rung.r1 = S1;
            p1 = max(abs(S1.bias_z)) < K.BIAS_PASS_THRESH;
            % fig_rung1 overlays far (Rung 0) vs near (Rung 1); fall back to a
            % standalone near plot if Rung 0 was not run this call.
            if exist('S0', 'var')
                f1 = fig_rung1_nearwall_spread(S0, S1, opts, K);
            else
                f1 = fig_rung1_nearwall_spread([], S1, opts, K);
            end
            out.figs{end+1} = f1;
            summary(end+1) = mk_row(1, rg1.label, S1, opts, p1); %#ok<AGROW>
            if opts.verbose; report_sweep('Rung 1 (near)', S1, K); end
        end
    end

    % ==================================================================
    % Rung 2 : DYNAMIC near-wall + ORACLE  (aliasing exposure)
    % ==================================================================
    if any(opts.rungs == 2)
        rg2 = regime_dyn_near_oracle(K);
        S2 = run_single_rung(rg2, opts, K);
        out.rung.r2 = S2;
        f2 = fig_rung2_aliasing(S2, opts, K);
        out.figs{end+1} = f2;
        summary(end+1) = mk_row(2, rg2.label, S2, opts, NaN); %#ok<AGROW>
        if opts.verbose; report_single('Rung 2 (dyn near oracle)', S2, K); end
    end

    % ==================================================================
    % Rung 3 : DYNAMIC far-ish + CLOSED LOOP  (coupling exposure)
    % ==================================================================
    if any(opts.rungs == 3)
        rg3 = regime_dyn_farish_closed(K);
        S3 = run_single_rung(rg3, opts, K);
        out.rung.r3 = S3;
        f3 = fig_rung3_loop_coupling(S3, opts, K);
        out.figs{end+1} = f3;
        summary(end+1) = mk_row(3, rg3.label, S3, opts, NaN); %#ok<AGROW>
        if opts.verbose; report_single('Rung 3 (dyn far-ish closed)', S3, K); end
    end

    % ==================================================================
    % Rung 4 : DYNAMIC near-wall + CLOSED LOOP  (full case)
    % ==================================================================
    if any(opts.rungs == 4)
        rg4 = regime_dyn_near_closed(K);
        S4 = run_single_rung(rg4, opts, K);
        out.rung.r4 = S4;
        f4 = fig_rung4_full(S4, opts, K);
        out.figs{end+1} = f4;
        summary(end+1) = mk_row(4, rg4.label, S4, opts, NaN); %#ok<AGROW>
        if opts.verbose; report_single('Rung 4 (dyn near closed, FULL)', S4, K); end
    end

    out.summary = summary;
    if opts.verbose
        print_summary_table(summary);
        for i = 1:numel(out.figs); fprintf('  [fig] %s\n', out.figs{i}); end
    end
end


% ======================================================================
%  Option parsing
% ======================================================================

function opts = parse_opts(varargin)
    opts = struct('rungs', 0:4, 'seeds', 1:20, ...
                  'a_cov_sweep', [0.01 0.02 0.05 0.1], 'a_cov_nom', 0.05, ...
                  'a_pd', 0.05, 'lambda_c', 0.7, 'verbose', true, ...
                  'smoke', false, 'out_root', '');
    if nargin == 1 && isstruct(varargin{1})
        user = varargin{1};
        fn = fieldnames(user);
        for i = 1:numel(fn); opts.(fn{i}) = user.(fn{i}); end
    elseif nargin >= 1
        if mod(nargin, 2) ~= 0
            error('verify_ahat_ladder:badArgs', ...
                  'Name-value arguments must come in pairs.');
        end
        for i = 1:2:nargin
            name = varargin{i};
            if ~ischar(name) && ~isstring(name)
                error('verify_ahat_ladder:badArgs', 'Argument %d must be a field name.', i);
            end
            opts.(char(name)) = varargin{i+1};
        end
    end
    if opts.smoke
        opts.seeds = 1:3;
        opts.a_cov_sweep = [0.02 0.05];
    end
    opts.rungs = opts.rungs(:).';
    opts.seeds = opts.seeds(:).';
end


% ======================================================================
%  Regime specs  (one struct per rung; drives build_rung_cfg)
% ======================================================================

function rg = regime_static_far(K)
    rg = base_regime();
    rg.name          = 'rung0_far_static_oracle';
    rg.label         = 'static far oracle';
    rg.is_static     = true;
    rg.use_true_gain = true;
    rg.est_field     = 'a_xm';
    rg.h_init        = K.FAR_HINIT_UM;
    rg.h_bottom      = K.FAR_HINIT_UM;
    rg.amplitude     = 0;
    rg.h_min_um      = 1.5 * K.R_UM;
    rg.h_bar_safe    = 1.5;
end

function rg = regime_static_near(K)
    rg = base_regime();
    rg.name          = 'rung1_near_static_oracle';
    rg.label         = 'static near oracle';
    rg.is_static     = true;
    rg.use_true_gain = true;
    rg.est_field     = 'a_xm';
    rg.h_init        = K.NEAR_HBAR * K.R_UM;     % h_bar ~ 1.2
    rg.h_bottom      = K.NEAR_HBAR * K.R_UM;
    rg.amplitude     = 0;
    rg.h_min_um      = 1.05 * K.R_UM;
    rg.h_bar_safe    = 1;                        % gate-free (h_bar ~ 1.2 > 1)
end

function rg = regime_dyn_near_oracle(K)
    rg = base_regime();
    rg.name          = 'rung2_dyn_near_oracle';
    rg.label         = 'dyn near oracle';
    rg.is_static     = false;
    rg.use_true_gain = true;
    rg.est_field     = 'a_xm';
    rg.h_init        = K.FAR_HINIT_UM;
    rg.h_bottom      = K.NEAR_BOTTOM_UM;         % trough h_bar ~ 1.2
    rg.amplitude     = K.OSC_AMP_UM;
    rg.h_min_um      = 1.05 * K.R_UM;
    rg.h_bar_safe    = 1;
end

function rg = regime_dyn_farish_closed(K)
    rg = base_regime();
    rg.name          = 'rung3_dyn_farish_closed';
    rg.label         = 'dyn far-ish closed';
    rg.is_static     = false;
    rg.use_true_gain = false;
    rg.est_field     = 'a_hat';
    rg.h_init        = K.FAR_HINIT_UM;
    rg.h_bottom      = K.FARISH_BOTTOM_UM;       % trough h_bar ~ 2.67 (safe zone)
    rg.amplitude     = K.OSC_AMP_UM;
    rg.h_min_um      = 1.5 * K.R_UM;
    rg.h_bar_safe    = 1.5;
end

function rg = regime_dyn_near_closed(K)
    rg = base_regime();
    rg.name          = 'rung4_dyn_near_closed';
    rg.label         = 'dyn near closed (full)';
    rg.is_static     = false;
    rg.use_true_gain = false;
    rg.est_field     = 'a_hat';
    rg.h_init        = K.FAR_HINIT_UM;
    rg.h_bottom      = K.NEAR_BOTTOM_UM;         % trough h_bar ~ 1.2
    rg.amplitude     = K.OSC_AMP_UM;
    rg.h_min_um      = 1.05 * K.R_UM;
    rg.h_bar_safe    = 1;
end

function rg = base_regime()
    rg = struct('name', '', 'label', '', 'is_static', true, ...
                'use_true_gain', true, 'est_field', 'a_xm', ...
                'h_init', 50, 'h_bottom', 50, 'amplitude', 0, ...
                'h_min_um', 3.375, 'h_bar_safe', 1.5);
end


% ======================================================================
%  Config builder
% ======================================================================

function cfg = build_rung_cfg(rg, a_cov, opts, K)
%BUILD_RUNG_CFG  user_config() + ladder overrides for one rung + a_cov value.
%   a_cov is set AFTER user_config() so it wins over apply_qr_preset's 0.005;
%   a_pd is pinned to opts.a_pd (NOT a_cov) so the sweep isolates a_cov.
    cfg = user_config();
    cfg.eq17_variant      = '4state';
    cfg.ctrl_enable       = true;
    cfg.thermal_enable    = true;
    cfg.meas_noise_enable = true;
    cfg.lambda_c          = opts.lambda_c;
    cfg.a_pd              = opts.a_pd;     % fixed across the a_cov sweep
    cfg.a_cov             = a_cov;         % swept knob (override path)
    cfg.meas_noise_std    = K.MEAS_NOISE_STD;
    cfg.suppress_xD       = true;          % no disturbance state in 4-state anyway
    cfg.use_am_lpf        = false;
    cfg.h_init            = rg.h_init;
    cfg.h_min             = rg.h_min_um;
    cfg.h_bar_safe        = rg.h_bar_safe;

    if rg.is_static
        cfg.trajectory_type = 'positioning';
        cfg.h_bottom        = rg.h_bottom;
        cfg.amplitude       = 0;
        cfg.T_sim           = K.T_STEADY_STAT_S + 3.0;   % steady window + 3 s
    else
        cfg.trajectory_type    = 'osc';
        cfg.h_bottom           = rg.h_bottom;
        cfg.amplitude          = rg.amplitude;
        cfg.frequency          = K.OSC_FREQ_HZ;
        cfg.n_cycles           = K.OSC_NCYC;
        cfg.t_hold             = K.T_HOLD_S;
        cfg.t_descend_override = K.T_DESCEND_S;
        % T_sim covers hold + descend + all cycles.
        cfg.T_sim = K.T_HOLD_S + K.T_DESCEND_S + K.OSC_NCYC / K.OSC_FREQ_HZ;
    end
end


% ======================================================================
%  Sweep + single-config runners
% ======================================================================

function S = run_sweep_rung(rg, opts, K)
%RUN_SWEEP_RUNG  Run a rung over the a_cov sweep; collect bias/spread/floor.
    nsw = numel(opts.a_cov_sweep);
    S = struct();
    S.regime   = rg;
    S.a_cov    = opts.a_cov_sweep(:).';
    S.bias_z   = nan(1, nsw);  S.bias_x   = nan(1, nsw);
    S.spread_z = nan(1, nsw);  S.spread_x = nan(1, nsw);
    S.floor_z  = nan(1, nsw);  S.floor_x  = nan(1, nsw);
    S.a_true_z = nan(1, nsw);  S.a_true_x = nan(1, nsw);
    S.n_div    = zeros(1, nsw);
    for j = 1:nsw
        cfg = build_rung_cfg(rg, opts.a_cov_sweep(j), opts, K);
        R = run_seed_set(cfg, opts.seeds, rg, K);
        m = metrics_from_run(R, K);
        S.bias_z(j)   = m.bias_z;   S.bias_x(j)   = m.bias_x;
        S.spread_z(j) = m.spread_z; S.spread_x(j) = m.spread_x;
        S.a_true_z(j) = m.a_true_z; S.a_true_x(j) = m.a_true_x;
        S.floor_z(j)  = floor_rel(R.ctrl_const, m.a_true_z, K.AX_Z);
        S.floor_x(j)  = floor_rel(R.ctrl_const, m.a_true_x, K.AX_X);
        S.n_div(j)    = R.n_div;
        if opts.verbose
            fprintf('   [%s a_cov=%.3g] bias_z=%+.2f%% spread_z=%.2f%% floor_z=%.2f%% (div=%d)\n', ...
                    rg.label, opts.a_cov_sweep(j), 100*m.bias_z, ...
                    100*m.spread_z, 100*S.floor_z(j), R.n_div);
        end
    end
end


function S = run_single_rung(rg, opts, K)
%RUN_SINGLE_RUNG  Run a rung at the nominal a_cov; keep full per-axis stacks
%   (for the phase / h_bar folds) plus scalar metrics.
    cfg = build_rung_cfg(rg, opts.a_cov_nom, opts, K);
    R = run_seed_set(cfg, opts.seeds, rg, K);
    m = metrics_from_run(R, K);
    S = struct();
    S.regime   = rg;
    S.a_cov    = opts.a_cov_nom;
    S.run      = R;
    S.bias_z   = m.bias_z;   S.bias_x   = m.bias_x;
    S.spread_z = m.spread_z; S.spread_x = m.spread_x;
    S.a_true_z = m.a_true_z; S.a_true_x = m.a_true_x;
    S.floor_z  = floor_rel(R.ctrl_const, m.a_true_z, K.AX_Z);
    S.floor_x  = floor_rel(R.ctrl_const, m.a_true_x, K.AX_X);
    S.n_div    = R.n_div;
end


function R = run_seed_set(cfg, seeds, rg, K)
%RUN_SEED_SET  Run one config across seeds; stack the chosen estimate and the
%   true gain per axis (x,z). Diverged runs are detected (|aligned err| > cutoff)
%   and excluded from the stacks.
    ns = numel(seeds);
    R = struct();
    R.t = []; R.Ts = []; R.hbar_d = []; R.ctrl_const = [];
    R.est_x = []; R.est_z = []; R.at_x = []; R.at_z = [];
    R.t2 = NaN; R.t3 = NaN; R.period = NaN;
    R.n_div = 0; R.n_good = 0;

    est_field = rg.est_field;
    for si = 1:ns
        ro = struct('seed', seeds(si), 'verbose', false, ...
                    'collect_diag', true, 'use_true_gain', rg.use_true_gain);
        try
            s = run_pure_simulation(cfg, ro);
        catch err
            R.n_div = R.n_div + 1;
            if isempty(R.t)   % keep going; only warn once per set head
                warning('verify_ahat_ladder:runCrash', ...
                        '%s seed %d crashed: %s', rg.label, seeds(si), err.message);
            end
            continue;
        end

        if ~isfield(s, 'diag') || ~isfield(s.diag, est_field)
            error('verify_ahat_ladder:missingDiag', ...
                  ['simOut.diag.%s missing — collect_diag must be on and the ' ...
                   'variant must populate it (got eq17_variant=%s).'], ...
                  est_field, getfield_default(cfg, 'eq17_variant', '?'));
        end

        % aligned physical error (same convention as compare_gain_6state)
        e = s.p_d_out(2:end, :) - s.p_true_out(1:end-1, :);
        if any(~isfinite(e(:))) || max(abs(e(:))) > K.DIVERGE_UM
            R.n_div = R.n_div + 1;
            continue;
        end

        a_true = s.a_true_out(2:end, :);          % drop init sample
        est    = s.diag.(est_field)(2:end, :);
        R.est_x = [R.est_x, est(:, K.AX_X)];      %#ok<AGROW>
        R.est_z = [R.est_z, est(:, K.AX_Z)];      %#ok<AGROW>
        R.at_x  = [R.at_x,  a_true(:, K.AX_X)];   %#ok<AGROW>
        R.at_z  = [R.at_z,  a_true(:, K.AX_Z)];   %#ok<AGROW>
        R.n_good = R.n_good + 1;

        if isempty(R.t)
            R.Ts = s.meta.params_value.common.Ts;
            N = size(a_true, 1);
            R.t = (1:N)' * R.Ts;
            P = s.meta.params_value; w = P.wall.w_hat(:);
            R.hbar_d = (s.p_d_out(2:end, :) * w - P.wall.pz) / P.common.R;
            R.ctrl_const = s.ctrl_const;
            if rg.is_static
                R.win = R.t >= K.T_STEADY_STAT_S;
                R.t2 = NaN; R.t3 = NaN; R.period = NaN;
            else
                R.period = 1 / K.OSC_FREQ_HZ;
                R.t2 = K.T_HOLD_S + K.T_DESCEND_S;            % osc start
                R.t3 = R.t2 + K.OSC_NCYC / K.OSC_FREQ_HZ;     % osc end
                % steady window skips the first cycle (let a_hat settle)
                R.win = (R.t >= R.t2 + R.period) & (R.t <= R.t3);
            end
        end
    end

    if R.n_good == 0
        error('verify_ahat_ladder:allDiverged', ...
              '%s: all %d seeds diverged/crashed — no data.', rg.label, ns);
    end
end


function m = metrics_from_run(R, K) %#ok<INUSD>
%METRICS_FROM_RUN  BIAS and across-seed SPREAD over the steady window, per axis.
%   BIAS   = mean_window mean_seeds( est/a_true ) - 1      (uses per-step
%            ensemble means; the deterministic swing is in both num & den).
%   SPREAD = mean_window( std_seed(est) / mean_seed(a_true) ).
    win = R.win;
    m = struct();
    [m.bias_z, m.spread_z, m.a_true_z] = bias_spread(R.est_z, R.at_z, win);
    [m.bias_x, m.spread_x, m.a_true_x] = bias_spread(R.est_x, R.at_x, win);
end


function [bias, spread, a_true_steady] = bias_spread(est_stack, at_stack, win)
%BIAS_SPREAD  est_stack, at_stack are [N x nseed]; win is [N x 1] logical.
    est_ens = mean(est_stack, 2);          % per-step across-seed mean
    at_ens  = mean(at_stack, 2);
    if size(est_stack, 2) >= 2
        est_std = std(est_stack, 0, 2);    % across-seed sigma
    else
        est_std = zeros(size(est_ens));    % single seed -> no spread estimate
    end
    bias   = mean(est_ens(win) ./ at_ens(win)) - 1;
    spread = mean(est_std(win) ./ at_ens(win));
    a_true_steady = mean(at_ens(win));
end


% ======================================================================
%  Floor line  (controller's own R22_intrinsic model, relative)
% ======================================================================

function fr = floor_rel(cc, a_true, ax)
%FLOOR_REL  Predicted across-seed relative std of a_xm at gain a_true on axis ax:
%       std(a_xm)/a_true = sqrt(K_var * IF_eff(a_true)) * (a_true + xi) / a_true
%   from Var(a_xm) = K_var*IF_eff*(a+xi)^2 (R22_intrinsic, amlpf_var_factor=1).
%   Far from the wall xi/a -> 0 so this reduces to sqrt(K_var*IF_eff). cc is the
%   run's simOut.ctrl_const.
    if isempty(cc) || ~isfinite(a_true) || a_true <= 0
        fr = NaN; return;
    end
    snx = cc.sigma2_n_s(ax);
    IF  = if_eff_local(cc.IF_abc, cc.C_dpmr, cc.C_n, cc.kBT, a_true, snx);
    xi  = cc.xi_per_axis(ax);
    fr  = sqrt(cc.K_var * IF) * (a_true + xi) / a_true;
end


function IF = if_eff_local(IF_abc, C_dpmr, C_n, kBT, a, sigma2_nx)
%IF_EFF_LOCAL  Exact color-inflation factor IF_eff (replicates the controller's
%   if_eff_eval; R22_derivation S4-S6). IF_abc = [A;B;C] s-weighted autocorr sums.
    sxT = 4 * kBT * a;
    num = sxT^2 * IF_abc(1) + 2 * sxT * sigma2_nx * IF_abc(2) + sigma2_nx^2 * IF_abc(3);
    den = (C_dpmr * sxT + C_n * sigma2_nx)^2;
    IF  = 1 + 2 * num / den;
end


% ======================================================================
%  Figures   (style mirrors verify_eq17_gscalar_suite: no grid/title,
%             legend northoutside, bold axes)
% ======================================================================

function [FS, LFS, AXLW, COL] = fig_style()
    FS = 18; LFS = 13; AXLW = 2.0;
    COL.meas_z = [0.8 0 0];      % z measured (red)
    COL.meas_x = [0 0.2 0.8];    % x measured (blue)
    COL.floor  = [0 0.6 0];      % floor line (green)
    COL.near   = [0.85 0.3 0];   % near-wall (orange-red)
    COL.far    = [0 0.45 0.74];  % far (blue)
    COL.ref    = [0 0 0];        % reference / unity (black)
end

function finish_axes(FS, AXLW)
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
    grid off;
end

function f = new_fig(w, h)
    f = figure('Position', [80 80 w h], 'Color', 'w', ...
               'NumberTitle', 'off', 'Visible', 'off');
end


function fp = fig_rung0_fisher_floor(S0, opts, K) %#ok<INUSD>
%FIG_RUNG0_FISHER_FLOOR  measured across-seed relative spread vs the Fisher
%   floor sqrt(K_var*IF)*(a+xi)/a, for each a_cov (z and x). Points on line = floor.
    [FS, LFS, AXLW, COL] = fig_style();
    f = new_fig(1000, 660); hold on;
    ac = S0.a_cov;
    hFz = plot(ac, 100*S0.floor_z, '-', 'Color', COL.floor, 'LineWidth', 2.5, ...
               'DisplayName', 'floor z');
    hMz = plot(ac, 100*S0.spread_z, 'o', 'Color', COL.meas_z, 'LineWidth', 2.0, ...
               'MarkerFaceColor', COL.meas_z, 'MarkerSize', 9, 'DisplayName', 'meas z');
    hFx = plot(ac, 100*S0.floor_x, '--', 'Color', COL.floor, 'LineWidth', 2.0, ...
               'DisplayName', 'floor x');
    hMx = plot(ac, 100*S0.spread_x, 's', 'Color', COL.meas_x, 'LineWidth', 2.0, ...
               'MarkerFaceColor', COL.meas_x, 'MarkerSize', 9, 'DisplayName', 'meas x');
    xlabel('a_{cov}', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel('a_{xm} rel. spread  \sigma_{seed}/a_{true} (%)', 'FontSize', FS, 'FontWeight', 'bold');
    legend([hMz hFz hMx hFx], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    finish_axes(FS, AXLW);
    fp = fullfile(opts.out_root, 'fig_rung0_fisher_floor.png');
    exportgraphics(f, fp, 'Resolution', 150); close(f);
end


function fp = fig_rung1_nearwall_spread(S0, S1, opts, K)
%FIG_RUNG1_NEARWALL_SPREAD  top: spread vs a_cov far (Rung0) vs near (Rung1) with
%   floors; bottom: bias vs a_cov for both, with the +/-BIAS_PASS_THRESH band.
    [FS, LFS, AXLW, COL] = fig_style();
    f = new_fig(1000, 940);
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    % --- spread panel ---
    nexttile; hold on;
    ac = S1.a_cov;
    h1 = plot(ac, 100*S1.spread_z, '-o', 'Color', COL.near, 'LineWidth', 2.2, ...
              'MarkerFaceColor', COL.near, 'DisplayName', 'near meas');
    plot(ac, 100*S1.floor_z, ':', 'Color', COL.near, 'LineWidth', 2.0, ...
         'HandleVisibility', 'off');
    hh = h1;
    if ~isempty(S0)
        h0 = plot(S0.a_cov, 100*S0.spread_z, '-o', 'Color', COL.far, 'LineWidth', 2.2, ...
                  'MarkerFaceColor', COL.far, 'DisplayName', 'far meas');
        plot(S0.a_cov, 100*S0.floor_z, ':', 'Color', COL.far, 'LineWidth', 2.0, ...
             'HandleVisibility', 'off');
        hh = [h0 h1];
    end
    ylabel('z spread  \sigma_{seed}/a (%)', 'FontSize', FS, 'FontWeight', 'bold');
    legend(hh, 'Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    finish_axes(FS, AXLW);

    % --- bias panel ---
    nexttile; hold on;
    plot(ac, 100*K.BIAS_PASS_THRESH*ones(size(ac)), 'k--', 'LineWidth', 1.2, ...
         'HandleVisibility', 'off');
    plot(ac, -100*K.BIAS_PASS_THRESH*ones(size(ac)), 'k--', 'LineWidth', 1.2, ...
         'HandleVisibility', 'off');
    hb1 = plot(ac, 100*S1.bias_z, '-o', 'Color', COL.near, 'LineWidth', 2.2, ...
               'MarkerFaceColor', COL.near, 'DisplayName', 'near bias');
    hhb = hb1;
    if ~isempty(S0)
        hb0 = plot(S0.a_cov, 100*S0.bias_z, '-o', 'Color', COL.far, 'LineWidth', 2.2, ...
                   'MarkerFaceColor', COL.far, 'DisplayName', 'far bias');
        hhb = [hb0 hb1];
    end
    xlabel('a_{cov}', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel('z bias  <a_{xm}/a> - 1 (%)', 'FontSize', FS, 'FontWeight', 'bold');
    legend(hhb, 'Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    finish_axes(FS, AXLW);

    fp = fullfile(opts.out_root, 'fig_rung1_nearwall_spread.png');
    exportgraphics(f, fp, 'Resolution', 150); close(f);
end


function fp = fig_rung2_aliasing(S2, opts, K)
%FIG_RUNG2_ALIASING  mean(a_xm)/a_true folded over the oscillation phase (z, x):
%   the systematic deviation from 1 is the dynamic-window aliasing bias.
    [FS, LFS, AXLW, COL] = fig_style();
    R = S2.run;
    [ph_c, rz] = phase_fold(R.t, R.est_z, R.at_z, R.t2, R.t3, R.period, K.PHASE_NBINS);
    [~,    rx] = phase_fold(R.t, R.est_x, R.at_x, R.t2, R.t3, R.period, K.PHASE_NBINS);
    f = new_fig(1000, 600); hold on;
    plot(ph_c, ones(size(ph_c)), '-', 'Color', COL.ref, 'LineWidth', 1.5, ...
         'HandleVisibility', 'off');
    hz = plot(ph_c, rz, '-o', 'Color', COL.meas_z, 'LineWidth', 2.2, ...
              'MarkerFaceColor', COL.meas_z, 'DisplayName', 'z');
    hx = plot(ph_c, rx, '-s', 'Color', COL.meas_x, 'LineWidth', 2.0, ...
              'MarkerFaceColor', COL.meas_x, 'DisplayName', 'x');
    xlabel('oscillation phase  (cycle fraction)', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel('mean(a_{xm}) / mean(a_{true})', 'FontSize', FS, 'FontWeight', 'bold');
    legend([hz hx], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    finish_axes(FS, AXLW);
    fp = fullfile(opts.out_root, 'fig_rung2_aliasing.png');
    exportgraphics(f, fp, 'Resolution', 150); close(f);
end


function fp = fig_rung3_loop_coupling(S3, opts, K)
%FIG_RUNG3_LOOP_COUPLING  closed-loop a_hat/a_true - 1 (%) ensemble vs time (z,x)
%   with the +/-spread band on z: the coupling bias that appears once a_hat feeds
%   back, in the far-ish (safe-zone) regime.
    [FS, LFS, AXLW, COL] = fig_style();
    R = S3.run; t = R.t;
    w_mm = max(3, round(K.SMOOTH_SEC / R.Ts)); sm = @(v) movmean(v, w_mm);
    ez = mean(R.est_z, 2); az = mean(R.at_z, 2);
    ex = mean(R.est_x, 2); axx = mean(R.at_x, 2);
    rz = sm(100 * (ez ./ az - 1));
    rx = sm(100 * (ex ./ axx - 1));
    if size(R.est_z, 2) >= 2
        sd = sm(100 * std(R.est_z, 0, 2) ./ az);
    else
        sd = zeros(size(rz));
    end
    f = new_fig(1100, 600); hold on;
    plot(t, zeros(size(t)), '-', 'Color', COL.ref, 'LineWidth', 1.2, 'HandleVisibility', 'off');
    fill([t; flipud(t)], [rz+sd; flipud(rz-sd)], COL.meas_z, ...
         'FaceAlpha', 0.18, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    hz = plot(t, rz, '-', 'Color', COL.meas_z, 'LineWidth', 2.2, 'DisplayName', 'z bias');
    hx = plot(t, rx, '-', 'Color', COL.meas_x, 'LineWidth', 2.0, 'DisplayName', 'x bias');
    xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel('a_{hat}/a_{true} - 1 (%)', 'FontSize', FS, 'FontWeight', 'bold');
    legend([hz hx], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    finish_axes(FS, AXLW);
    fp = fullfile(opts.out_root, 'fig_rung3_loop_coupling.png');
    exportgraphics(f, fp, 'Resolution', 150); close(f);
end


function fp = fig_rung4_full(S4, opts, K) %#ok<INUSD>
%FIG_RUNG4_FULL  full near-wall closed-loop case: |bias| and spread (%) per axis.
    [FS, LFS, AXLW, COL] = fig_style();
    data = [100*abs(S4.bias_x), 100*S4.spread_x; ...
            100*abs(S4.bias_z), 100*S4.spread_z];     % rows: x, z ; cols: |bias|, spread
    f = new_fig(900, 640); hold on;
    hb = bar(1:2, data, 0.8);
    hb(1).FaceColor = COL.meas_x; hb(2).FaceColor = COL.meas_z;
    set(gca, 'XTick', 1:2, 'XTickLabel', {'x', 'z'});
    ylabel('a_{hat} error (%)', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel('axis', 'FontSize', FS, 'FontWeight', 'bold');
    legend({'|bias|', 'spread \sigma_{seed}/a'}, 'Location', 'northoutside', ...
           'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    finish_axes(FS, AXLW);
    fp = fullfile(opts.out_root, 'fig_rung4_full.png');
    exportgraphics(f, fp, 'Resolution', 150); close(f);
end


function [ph_c, ratio] = phase_fold(t, est_stack, at_stack, t2, t3, period, nbins)
%PHASE_FOLD  Pool (est, a_true) over the oscillation phase in [0,1). Skips the
%   first cycle. Returns bin centers and mean(est)/mean(a_true) per bin.
    in_osc = (t >= t2 + period) & (t <= t3);
    phase = mod(t(in_osc) - t2, period) / period;        % [0,1)
    est = est_stack(in_osc, :); at = at_stack(in_osc, :);
    edges = linspace(0, 1, nbins + 1);
    ph_c = 0.5 * (edges(1:end-1) + edges(2:end));
    ratio = nan(1, nbins);
    for b = 1:nbins
        sel = phase >= edges(b) & phase < edges(b+1);
        if b == nbins; sel = sel | (phase == edges(end)); end
        if any(sel)
            ev = est(sel, :); av = at(sel, :);
            ratio(b) = mean(ev(:)) / mean(av(:));
        end
    end
end


% ======================================================================
%  Reporting
% ======================================================================

function row = mk_row(rung, regime, S, opts, pass)
%MK_ROW  Build a summary row. For sweep rungs, report the nominal-a_cov column.
    [bz, sz, fz] = pick_nominal(S, opts);
    row = struct('rung', rung, 'regime', regime, 'bias_z', bz, ...
                 'spread_z', sz, 'floor_z', fz, 'pass', pass);
end

function [bz, sz, fz] = pick_nominal(S, opts)
%PICK_NOMINAL  z metrics at the nominal a_cov (sweep) or the single value.
    if isfield(S, 'a_cov') && numel(S.a_cov) > 1
        [~, jn] = min(abs(S.a_cov - opts.a_cov_nom));
        bz = S.bias_z(jn); sz = S.spread_z(jn); fz = S.floor_z(jn);
    else
        bz = S.bias_z; sz = S.spread_z; fz = S.floor_z;
    end
end


function report_sweep(name, S, K) %#ok<INUSD>
    fprintf('  [%s] over a_cov sweep:\n', name);
    for j = 1:numel(S.a_cov)
        fprintf('     a_cov=%.3g : bias_z=%+.2f%%  spread_z=%.2f%%  floor_z=%.2f%%  (z spread/floor=%.2f)\n', ...
                S.a_cov(j), 100*S.bias_z(j), 100*S.spread_z(j), 100*S.floor_z(j), ...
                S.spread_z(j)/S.floor_z(j));
    end
end

function report_single(name, S, K) %#ok<INUSD>
    fprintf('  [%s] a_cov=%.3g, good seeds=%d, diverged=%d\n', ...
            name, S.a_cov, S.run.n_good, S.n_div);
    fprintf('     z : bias=%+.2f%%  spread=%.2f%%  floor=%.2f%%\n', ...
            100*S.bias_z, 100*S.spread_z, 100*S.floor_z);
    fprintf('     x : bias=%+.2f%%  spread=%.2f%%  floor=%.2f%%\n', ...
            100*S.bias_x, 100*S.spread_x, 100*S.floor_x);
end


function print_summary_table(summary)
    if isempty(summary); return; end
    fprintf('\n  ===== a_hat ladder SUMMARY (z axis) =====\n');
    fprintf('  %-4s %-26s %9s %9s %9s %-8s\n', ...
            'rung', 'regime', 'bias', 'spread', 'floor', 'pass');
    for i = 1:numel(summary)
        s = summary(i);
        if islogical(s.pass) || (isnumeric(s.pass) && ~isnan(s.pass))
            if s.pass; ptag = 'PASS'; else; ptag = 'FAIL'; end
        else
            ptag = '(report)';
        end
        fprintf('  %-4d %-26s %+8.2f%% %8.2f%% %8.2f%% %-8s\n', ...
                s.rung, s.regime, 100*s.bias_z, 100*s.spread_z, 100*s.floor_z, ptag);
    end
    fprintf('\n');
end


function v = getfield_default(s, name, dflt)
    if isfield(s, name); v = s.(name); else; v = dflt; end
end
