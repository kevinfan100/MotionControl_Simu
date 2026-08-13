% STATUS: ACTIVE (u-coordinate driver) -- spec
%          reference/eq17_analysis/derivation/formC_ugap.tex (+ _ref.tex);
%          controller model/controller/motion_control_law_formC_ugap.m
% FORK OF test_script/integration/run_formC_dist.m @ d0490c3 | PURPOSE: same
%   canonical scenario, seeds and metrics, controller swapped for the GAP
%   coordinate u = 1/(1 - a_bar), where du/d w_bar = 1 + delta a is constant
%   so the one-step quadrature is exact and the predict increment never reads
%   the gain state | EXPIRES: u-coordinate / a_bar-coordinate adjudication |
%   production changes do NOT follow.
%   Arms: 'base' = slot 5 LOCKED at 0  -> derivation (a), 4 states
%         'dist' = slot 5 FREE from 0  -> derivation (b), 5 states, Q_dada = 0
function out = run_formC_ugap(opts, test_opts)
%RUN_FORMC_UGAP  Canonical-scenario driver for the u = 1/(1-a_bar) writing of
%   the state gain law (formC_ugap.tex).
%
%   out = run_formC_ugap()                          % arm 'dist', house seeds
%   out = run_formC_ugap(struct('arm', 'base'))     % the 4-state baseline
%
%   FORK OF run_formC_dist.m: the per-step ordering (RNG seed -> params ->
%   trajectory -> controller -> thermal -> step_dynamics -> sensor-delay
%   buffer), the config / ctrl_const plumbing, the canonical scenario, the
%   seed loop and the console metrics are inherited verbatim, so a run of this
%   driver and a run of run_formC_dist on the same seed are a PAIRED
%   comparison on identical random draws. Four things change:
%       (1) the controller dispatched;
%       (2) delta a is MULTIPLICATIVE on du/d w_bar (the a_bar-coordinate
%           image of this law is formC_state_da.tex, not formC_state_dist.tex),
%           so its prior is the sibling's 0.5, not the additive per-step sup;
%       (3) the shape floor is the LOCAL value at the seed height, not the
%           envelope supremum -- constraint (i) of formC_ugap_ref, which is a
%           WELL-POSEDNESS requirement here rather than a width preference,
%           because the floor reaches P_uu[0] through u_hat[0]^4;
%       (4) the two clamps u >= u_min, v_hat >= u_min are audited: their
%           binding fractions are printed and a non-zero fraction IS the arm
%           failing.
%
%   Authoritative spec:
%       reference/eq17_analysis/derivation/formC_ugap.tex      (equations)
%       reference/eq17_analysis/derivation/formC_ugap_ref.tex  (caveats,
%           constraints, pre-registration)
%   State (5 per axis, mapped onto the inherited 9-slot layout):
%       [dw1 dw2 dw3 u delta_a | (inert) (inert) | m1 m2]
%   Plant truth comes from calc_correction_functions (c_perp on z); the
%   filter is run-time c-free by construction (charter constraint 3): every
%   truth evaluation in this file happens at DRIVER init, offline.
%
%   SCENARIO (canonical, z-axis / perpendicular focus), unchanged:
%       hold 0.5 s @ h = 50 um (h_bar 22.2) -> descend 1.0 s ->
%       1 Hz oscillation, amplitude 2.5 um, trough h = 4.5 um (h_bar 2.0),
%       2 cycles -> final hold at the trough; T = 4.8 s; thermal +
%       measurement noise ON; lambda_c = 0.7.
%
%   THE SHAPE FLOOR (constraint (i)). The seed prior on u is
%       P_uu[0] = (u_0/(1+da))^2 P_da + (1+da)^2 P_w0 + u_0^4 floor_a^2 ,
%   so at u_0 = 22.22 the floor is amplified by 2.4e5. This driver computes
%   BOTH candidates from the published truth curve and passes the local one:
%       local     |a_law(w_seed) - 1/c_perp(w_seed)|      (the error the seed
%                 ACTUALLY makes, at the height the seed is set at)
%       envelope  sup |a_law(w) - 1/c_perp(w)| over the planned envelope
%   with a_law(w) = 1 - 1/((1+da_seed)(w - w0)). Both are printed with the
%   sigma_u each implies and the prior mass they put below u = 1 (the
%   singularity of the output map 1 - 1/u). Nothing here is tuned: the inputs
%   are the seed height, the planned envelope and the published truth curve.
%
%   THE delta a PRIOR. delta a here is a MULTIPLICATIVE correction on
%   du/d w_bar (equivalently b = 1/(1+delta a) in the a_bar writing, anchored
%   at b = 9/8 <=> delta a = -1/9), so its natural scale is O(1) and the
%   default 0.5 is carried over from run_formC_state, the a_bar-coordinate
%   driver of the SAME law. formC_ugap_ref prices P_uu[0] with exactly this
%   value. It is an undetermined number; its contract is INVARIANCE, so
%   opts.Pf_da_std overrides it and sweeping it 10x either way must not move
%   the converged behaviour.
%
%   PARALLEL-AXIS LAW (opts.par_law, default TRUE). The wall-parallel truth is
%   not a member of the one-curve family, so the parallel package is the
%   integration constant alone: a 1-parameter minimax fit of
%   a_law = 1 - 1/(w - w0) to 1/c_para on the planned envelope, with slot 5
%   locked at 0 on x/y and their floor taken locally at the seed height like
%   the z axis. x/y accuracy is DECLARED out of scope: with w_hat = z the
%   mobility matrix is diagonal, so x/y are dynamically decoupled from the z
%   axis under test and cannot contaminate its metrics.
%
%   opts fields (defaults first):
%       .arm         'dist'  'dist' = slot 5 free from 0 (derivation (b));
%                            'base' = slot 5 locked at 0 (derivation (a))
%       .Pf_da_std   0.5     prior on the multiplicative delta a
%       .Pf_w0_std   0.111   wall-position prior, carried over from formB_ws
%       .floor_mode  'local' 'local' = constraint (i) (the ONLY well-posed
%                            choice); 'env' = the envelope sup, provided so
%                            the ill-posedness can be demonstrated rather than
%                            asserted
%       .u_min       1.5     gap clamp on u and v_hat (= h_bar_safe)
%       .par_law     true    x/y run the fitted parallel origin
%       .y2_on       true    false = drop the gain-readout channel
%       .a_cov_scale 1       multiplies the 0.05 base a_cov (invariance sweep)
%       .seeds       [7 11 23 42 101 777]    house 6-seed convention
%       .verbose     false   per-run progress printout
%       .config_override     struct merged into the canonical config
%       .ctrl_const_override struct merged into ctrl_const LAST (wins)
%       .a_ctrl_override     [] | 3x1 [um/pN] | 'true'
%
%   Console metrics per run (z axis, e_a = 100*(a_hat - a_true)/a_true), the
%   sibling's set verbatim so the two drivers' tables can be read side by side:
%       desc pk %   max |e_a| over the descent window
%       osc RMS %   RMS e_a over the oscillation window (settled)
%       hold mean % mean e_a over the final hold (settled)
%       rms all %   RMS e_a over the whole ACTIVE run, t >= t_hold
%       da end      delta_a_hat[end] on z [-]
%       sqrtP55     sqrt(P_dada)[end] on z [-]
%       P55 mono    max single-step INCREASE of P_dada relative to its seed
%                   (Q_dada = 0 predicts monotone decrease)
%       frz         fraction of t >= t_hold over which delta_a_hat is frozen
%       clamp u/v   fraction of steps at each clamp -- MUST be zero
%   Printed only; test_results/ is gitignored, numbers are never committed.
%
%   Output struct: as run_formC_dist, plus out.floor (both floor candidates
%   and their P_uu[0] decomposition) and out.metrics rows for the clamps.
%   Saved to test_results/run_formC_ugap_<arm_tag>.mat (gitignored).
%
%   TEST-LADDER COMPATIBILITY FORM (inherited):
%       simOut = run_formC_ugap(cfg, opts)   % single seed, direct simOut
%
%   See also: motion_control_law_formC_ugap, run_formC_dist, run_formC_state

    % ---- test-ladder compatibility form: run_formC_ugap(cfg, test_opts) ----
    if nargin >= 1 && isstruct(opts) && isfield(opts, 'trajectory_type')
        cfg_t = opts;
        if nargin >= 2 && ~isempty(test_opts); topts = test_opts; else; topts = struct(); end
        if ~isfield(topts, 'seed');                topts.seed = 7;                     end
        if ~isfield(topts, 'verbose');             topts.verbose = false;              end
        if ~isfield(topts, 'ctrl_const_override'); topts.ctrl_const_override = struct(); end
        if ~isfield(topts, 'a_ctrl_override');     topts.a_ctrl_override = [];         end
        if ~isfield(topts, 'log_P_full');          topts.log_P_full = false;           end
        if ~isfield(topts, 'ws_inject');           topts.ws_inject = 0;                end
        if ~isfield(topts, 'plant_cperp');         topts.plant_cperp = [];             end
        out = local_run_once(cfg_t, topts.seed, topts.ctrl_const_override, ...
                             topts.verbose, topts.a_ctrl_override, topts.log_P_full, ...
                             topts.ws_inject, topts.plant_cperp);
        return;
    end

    if nargin < 1 || isempty(opts); opts = struct(); end
    if ~isfield(opts, 'arm');         opts.arm         = 'dist'; end
    if ~isfield(opts, 'Pf_da_std');   opts.Pf_da_std   = 0.5;   end
    if ~isfield(opts, 'Pf_w0_std');   opts.Pf_w0_std   = 0.111; end
    if ~isfield(opts, 'floor_mode');  opts.floor_mode  = 'local'; end
    if ~isfield(opts, 'u_min');       opts.u_min       = 1.5;   end
    if ~isfield(opts, 'par_law');     opts.par_law     = true;  end
    if ~isfield(opts, 'y2_on');       opts.y2_on       = true;  end
    if ~isfield(opts, 'a_cov_scale'); opts.a_cov_scale = 1;     end
    if ~isfield(opts, 'ws_inject');   opts.ws_inject   = 0;     end   % [R] TRUE wall offset, plant side only
    if ~isfield(opts, 'seeds');       opts.seeds       = [];    end
    if ~isfield(opts, 'verbose');     opts.verbose     = false; end
    if ~isfield(opts, 'config_override');     opts.config_override     = struct(); end
    if ~isfield(opts, 'ctrl_const_override'); opts.ctrl_const_override = struct(); end
    if ~isfield(opts, 'a_ctrl_override');     opts.a_ctrl_override     = [];       end

    % ------------------------------------------------------------------
    % Named constants (every number is derived or a house convention)
    % ------------------------------------------------------------------
    AX_Z            = 3;                     % wall-normal axis (perp, focus)
    SEEDS_DEFAULT   = [7 11 23 42 101 777];  % house 6-seed convention
    H_BAR_MIN_PRIOR = 1.1;   % truth-curve validity floor (two-sphere series)
    ENV_LO_MARGIN   = 0.1;   % [-] envelope floor below the commanded trough
    ENV_HI_MARGIN   = 1.0;   % [-] envelope ceiling above the start height
    A_COV_BASE      = 0.05;  % eq17 verified baseline EWMA weight
    OSC_SETTLE_S    = 0.1;   % skip descent->osc transition
    HOLD_SETTLE_S   = 0.3;   % skip osc->hold readout transient
    W0_PLANE        = 0;     % nominal wall at w_bar = 1 (ws0_perp - 1)

    assert(any(strcmpi(opts.arm, {'base', 'dist'})), ...
           'run_formC_ugap:badArm', ...
           'opts.arm must be ''base'' (derivation (a), 4 states) or ''dist'' (derivation (b), 5 states).');
    assert(any(strcmpi(opts.floor_mode, {'local', 'env'})), ...
           'run_formC_ugap:badFloorMode', ...
           'opts.floor_mode must be ''local'' (constraint (i)) or ''env''.');

    seeds = opts.seeds;
    if isempty(seeds); seeds = SEEDS_DEFAULT; end
    seeds = seeds(:).';

    % ------------------------------------------------------------------
    % Canonical scenario config (+ optional overrides, then the h_min gate)
    % ------------------------------------------------------------------
    cfg = local_canonical_config(A_COV_BASE * opts.a_cov_scale, H_BAR_MIN_PRIOR);
    fn = fieldnames(opts.config_override);
    for idx = 1:numel(fn)
        cfg.(fn{idx}) = opts.config_override.(fn{idx});
    end

    pc = physical_constants();
    assert(cfg.h_min / pc.R >= H_BAR_MIN_PRIOR - 1e-12, ...
           'run_formC_ugap:hMinBelowTruthDomain', ...
           'h_min = %.3f um -> h_bar_min = %.3f < %.2f: below the truth-curve validity floor.', ...
           cfg.h_min, cfg.h_min / pc.R, H_BAR_MIN_PRIOR);
    assert(cfg.h_bottom / pc.R >= H_BAR_MIN_PRIOR, ...
           'run_formC_ugap:troughBelowTruthDomain', ...
           'trajectory trough h_bar = %.3f < %.2f (truth-curve validity floor).', ...
           cfg.h_bottom / pc.R, H_BAR_MIN_PRIOR);

    % ------------------------------------------------------------------
    % Envelope, seed height, and the two shape-floor candidates
    % ------------------------------------------------------------------
    env_lo   = cfg.h_bottom / pc.R - ENV_LO_MARGIN;
    env_hi   = cfg.h_init   / pc.R + ENV_HI_MARGIN;
    w_seed   = cfg.h_init / pc.R;          % the controller's seed height
    da_seed  = 0;                          % tex seed section

    flr = local_shape_floors(w_seed, env_lo, env_hi, W0_PLANE, da_seed, false);
    switch lower(opts.floor_mode)
        case 'local'; floor_used = flr.local;  floor_src = 'LOCAL at the seed height (constraint (i))';
        case 'env';   floor_used = flr.env;    floor_src = 'envelope sup (ILL-POSED -- demonstration arm only)';
    end

    ov = struct();
    ov.Pf_a_floor = floor_used;
    ov.Pf_w0_std  = opts.Pf_w0_std;
    ov.Pf_da_std  = opts.Pf_da_std;
    ov.u_min      = opts.u_min;
    ov.ws0_perp   = 1;                % plane
    ov.da_init    = da_seed;

    switch lower(opts.arm)
        case 'base'
            ov.lock_da = true;        % derivation (a): 4 states
        case 'dist'
            ov.lock_da = false;       % derivation (b): 5 states, Q_dada = 0
    end

    if ~opts.y2_on
        ov.y2_off = true;                    % fingerprint arm
    end

    ov.par_law = opts.par_law;
    par_pkg = [];
    if opts.par_law
        par_pkg = local_parallel_law_package_ugap(env_lo, env_hi);
        flr_par = local_shape_floors(w_seed, env_lo, env_hi, par_pkg.w0, da_seed, true);
        par_pkg.floor_local = flr_par.local;
        par_pkg.floor_env   = flr_par.env;
        switch lower(opts.floor_mode)
            case 'local'; ov.Pf_a_floor_par = flr_par.local;
            case 'env';   ov.Pf_a_floor_par = flr_par.env;
        end
        ov.w0_par = par_pkg.w0;
    end

    fn = fieldnames(opts.ctrl_const_override);
    for idx = 1:numel(fn)
        ov.(fn{idx}) = opts.ctrl_const_override.(fn{idx});
    end
    % arm tag (file name + report header)
    tag = lower(opts.arm);
    if opts.y2_on; tag = [tag '_y2on']; else; tag = [tag '_y2off']; end
    if ~opts.par_law;         tag = [tag '_nopar']; end
    if ~strcmpi(opts.floor_mode, 'local'); tag = [tag '_envfloor']; end
    if opts.a_cov_scale ~= 1; tag = [tag sprintf('_acov%g', cfg.a_cov)]; end

    lastwarn('');

    % ------------------------------------------------------------------
    % Seed loop + per-run console metrics
    % ------------------------------------------------------------------
    u0 = w_seed - W0_PLANE;
    free_da = double(strcmpi(opts.arm, 'dist'));
    fprintf('=== Form C GAP-coordinate driver (u = 1/(1-a_bar)) -- arm: %s ===\n', tag);
    fprintf('scenario: hold %.1fs -> descend %.1fs -> %g Hz osc x%d -> hold, T=%.1fs; h_bar_min=%.2f\n', ...
            cfg.t_hold, cfg.t_descend_override, cfg.frequency, cfg.n_cycles, ...
            cfg.T_sim, cfg.h_min / pc.R);
    fprintf('arm=%s (%s)  lock_da=%d  da_init=%+.5g  sqrt_Pda=%.4f  sqrt_Pw0=%.4f  y2_on=%d  a_cov=%.3f  u_min=%.2f\n', ...
            lower(opts.arm), local_arm_words(opts.arm), ov.lock_da, ov.da_init, ...
            ov.Pf_da_std, ov.Pf_w0_std, opts.y2_on, cfg.a_cov, ov.u_min);
    fprintf('SHAPE FLOOR (constraint (i)); seed height w_bar %.4f -> u_hat[0] %.4f, envelope [%.3f, %.3f]\n', ...
            w_seed, u0, env_lo, env_hi);
    fprintf('   %-9s floor_a %.5f -> sigma_u(floor share) %7.3f , P(u<1) %6.3f %%   <- USED: %s\n', ...
            'local', flr.local, u0^2 * flr.local, ...
            100 * local_p_below_one(u0, u0^2 * flr.local), floor_src);
    fprintf('   %-9s floor_a %.5f -> sigma_u(floor share) %7.3f , P(u<1) %6.3f %%   (sup at w_bar %.3f)\n', ...
            'envelope', flr.env, u0^2 * flr.env, ...
            100 * local_p_below_one(u0, u0^2 * flr.env), flr.w_sup);
    P_uu_da   = free_da * (u0 / (1 + da_seed))^2 * ov.Pf_da_std^2;
    P_uu_w0   = (1 + da_seed)^2 * ov.Pf_w0_std^2;
    P_uu_flr  = u0^4 * floor_used^2;
    P_uu_tot  = P_uu_da + P_uu_w0 + P_uu_flr;
    fprintf('   P_uu[0] = %.4g (delta a) + %.4g (wall) + %.4g (floor) = %.4g -> sigma_u %.3f, P(u<1) %.3f %%\n', ...
            P_uu_da, P_uu_w0, P_uu_flr, P_uu_tot, sqrt(P_uu_tot), ...
            100 * local_p_below_one(u0, sqrt(P_uu_tot)));
    if opts.par_law
        fprintf('PAR LAW (x/y): w0 %.4f  fit sup %.4f  floor local %.5f / env %.5f\n', ...
                par_pkg.w0, par_pkg.fit_sup, par_pkg.floor_local, par_pkg.floor_env);
    else
        fprintf('PAR LAW (x/y): OFF -- x/y run the perpendicular origin\n');
    end

    fprintf('%6s | %9s %9s %10s %9s | %11s %10s %6s %6s | %9s %9s | %8s %8s %4s\n', 'seed', ...
            'desc pk %', 'osc RMS%', 'hold mn %', 'rms all%', ...
            'da end', 'sqrtP55end', 'frz', 'da@1.5', ...
            'hold %/s', 'unopp %/s', 'clamp u', 'clamp v', 'NaN');

    plant_cperp = [];   % plant-side boundary arms are not forked (plane only)
    n_seeds = numel(seeds);
    runs  = cell(n_seeds, 1);
    Mrows = zeros(n_seeds, 11);  % desc | osc | hold | rms_all | bud_b | bud_p | nan
                                 % | clamp_u(z) | clamp_v(z) | clamp_u(any ax) | clamp_v(any ax)
    Drows = zeros(n_seeds, 10);
    for q = 1:n_seeds
        s = local_run_once(cfg, seeds(q), ov, opts.verbose, opts.a_ctrl_override, ...
                           false, opts.ws_inject, plant_cperp);
        m = local_run_metrics(s, cfg, AX_Z, OSC_SETTLE_S, HOLD_SETTLE_S);
        runs{q}     = s;
        Mrows(q, :) = [m.desc_peak_pct, m.osc_rms_pct, m.hold_mean_pct, ...
                       m.rms_all_pct, m.budget_b, m.budget_p, m.any_nan, ...
                       m.clamp_u_frac, m.clamp_v_frac, ...
                       m.clamp_u_frac_any, m.clamp_v_frac_any];
        Drows(q, :) = [m.da_end, m.sqrtP55_end, m.sqrtP55_0, m.P55_mono, ...
                       m.frozen_frac, m.t_freeze, m.da_frac_by_descend_end, ...
                       m.hold_drift_pct_per_s, m.hold_delta_pct, ...
                       m.hold_unopposed_pct / max(m.hold_span_s, eps)];
        fprintf('%6d | %9.3f %9.3f %+10.3f %9.3f | %+11.4e %10.3e %6.3f %+6.2f | %+9.3f %+9.1f | %8.4f %8.4f %4d\n', ...
                seeds(q), Mrows(q, 1), Mrows(q, 2), Mrows(q, 3), Mrows(q, 4), ...
                Drows(q, 1), Drows(q, 2), Drows(q, 5), Drows(q, 7), ...
                Drows(q, 8), Drows(q, 10), Mrows(q, 8), Mrows(q, 9), Mrows(q, 7));
    end

    % ------------------------------------------------------------------
    % Aggregates
    % ------------------------------------------------------------------
    mu = mean(Mrows(:, 1:4), 1);
    sd = std(Mrows(:, 1:4), 0, 1);
    fprintf('-------------------------------------------------------------------------------------\n');
    fprintf('mean over %d seeds: desc pk %.3f+-%.3f %%  osc RMS %.3f+-%.3f %%  hold mean %+.3f+-%.3f %%  rms all %.3f+-%.3f %%\n', ...
            n_seeds, mu(1), sd(1), mu(2), sd(2), mu(3), sd(3), mu(4), sd(4));
    RATIO_SEED_FLAG = 6.635;   % chi2(1) 99th percentile
    RATIO_MEAN_PASS = 1 + 1.645 * sqrt(2 / n_seeds);
    flag_b = find(Mrows(:, 5) > RATIO_SEED_FLAG);
    mean_b = mean(Mrows(:, 5));
    if isnan(mean(Mrows(:, 6)))
        fprintf('budget aggregate: delta a mean %.3f (%s <= %.3f), p n/a (locked, zero-width prior)\n', ...
                mean_b, local_verdict(mean_b, RATIO_MEAN_PASS), RATIO_MEAN_PASS);
        fprintf('per-seed traversal ratios (diagnostic; flag > %.3f): delta a max %.3f%s\n', ...
                RATIO_SEED_FLAG, max(Mrows(:, 5)), local_flag_str(flag_b, seeds));
    end
    % --- CONSTRAINT (ii) AUDIT: the clamps must never bind ----------------
    cu = max(Mrows(:, 8)); cv = max(Mrows(:, 9));       % focus axis (z)
    cuA = max(Mrows(:, 10)); cvA = max(Mrows(:, 11));   % worst of the three axes
    n_cnt = zeros(3, 2);
    for q = 1:n_seeds
        n_cnt(:, 1) = n_cnt(:, 1) + runs{q}.clamp_u_count(:);
        n_cnt(:, 2) = n_cnt(:, 2) + runs{q}.clamp_v_count(:);
    end
    fprintf('CLAMP AUDIT (u_min = %.2f, %d steps x %d seeds): z-axis max frac u %.5f / v %.5f ; worst axis %.5f / %.5f\n', ...
            ov.u_min, runs{1}.clamp_steps, n_seeds, cu, cv, cuA, cvA);
    fprintf('   total bindings per axis [x y z]: u [%d %d %d] , v_hat [%d %d %d]\n', ...
            n_cnt(:, 1), n_cnt(:, 2));
    if cu == 0 && cv == 0 && cuA == 0 && cvA == 0
        fprintf('   -> constraint (ii) clean on every axis and every seed\n');
    elseif cu == 0 && cv == 0
        fprintf(['   -> clean on the z axis under test; the bindings are on the PARALLEL axes, which\n', ...
                 '      seed at w0_par and are declared out of scope (diagonal mobility => no z contamination)\n']);
    else
        fprintf(['   -> *** THE CLAMP BINDS ON THE AXIS UNDER TEST *** That is the ARM FAILING, not a\n', ...
                 '      guard working: the posterior reached a_bar < 0, the singularity of the output\n', ...
                 '      map 1 - 1/u (formC_ugap_ref, seed section).\n']);
    end
    fprintf('prior width sqrt(P_dada[0]) (controller-reported, z): %.4e  (declared %.4e)\n', ...
            runs{1}.P_b_out(1, AX_Z), free_da * ov.Pf_da_std);
    fprintf('P_uu[0] (controller-reported, z): %.4g -> sqrt %.4f ; in a_bar units sqrt(P_aa[0]) = %.5f\n', ...
            runs{1}.P_uu_out(1, AX_Z), sqrt(runs{1}.P_uu_out(1, AX_Z)), ...
            runs{1}.P_a_out(1, AX_Z) / runs{1}.a_nom);
    fprintf('u_hat: [0] %.4f -> [end] mean %.4f +- %.4f ; h_bar range (true, seed %d): [%.3f, %.3f]\n', ...
            runs{1}.u_hat_out(1, AX_Z), ...
            mean(cellfun(@(r) r.u_hat_out(end, AX_Z), runs)), ...
            std(cellfun(@(r) r.u_hat_out(end, AX_Z), runs)), seeds(1), ...
            min(runs{1}.h_bar_true_out), max(runs{1}.h_bar_true_out));

    % --- delta a diagnostics (pre-registered in tex S8) -------------------
    fprintf('da_hat[end]  : mean %+.4e +- %.4e   (prior width %.4e)\n', ...
            mean(Drows(:, 1)), std(Drows(:, 1)), free_da * ov.Pf_da_std);
    fprintf('sqrt(P_dada) : [0] %.4e -> [end] mean %.4e +- %.4e  (collapse factor %.3f)\n', ...
            mean(Drows(:, 3)), mean(Drows(:, 2)), std(Drows(:, 2)), ...
            mean(Drows(:, 2)) / max(mean(Drows(:, 3)), eps));
    fprintf('P_dada mono  : max single-step INCREASE / P[0] = %.3e (Q_dada = 0 predicts 0)\n', ...
            max(Drows(:, 4)));
    fprintf('da_hat frozen: fraction %.3f +- %.3f of t >= %.2f s; freeze time mean %.3f s\n', ...
            mean(Drows(:, 5)), std(Drows(:, 5)), cfg.t_hold, mean(Drows(:, 6)));
    fprintf('da_hat acquired on the DESCENT: da_hat(t=%.2f)/da_hat(end) = %.3f +- %.3f\n', ...
            cfg.t_hold + cfg.t_descend_override, mean(Drows(:, 7)), std(Drows(:, 7)));
    fprintf('FINAL-HOLD DRIFT: %+.3f +- %.3f %%/s ; end-start %+.3f +- %.3f %%\n', ...
            mean(Drows(:, 8)), std(Drows(:, 8)), mean(Drows(:, 9)), std(Drows(:, 9)));

    [wmsg, wid] = lastwarn();
    if isempty(wmsg)
        fprintf('WARNINGS: none\n');
    else
        fprintf('WARNINGS: [%s] %s\n', wid, wmsg);
    end

    % ------------------------------------------------------------------
    % Output struct for the plot scripts (test_results/ is gitignored)
    % ------------------------------------------------------------------
    out = struct();
    out.runs    = runs;
    out.metrics = struct('rows', Mrows, ...
                         'row_names', {{'desc_peak_pct', 'osc_rms_pct', ...
                                        'hold_mean_pct', 'rms_all_pct', ...
                                        'budget_b', 'budget_p', 'any_nan', ...
                                        'clamp_u_frac_z', 'clamp_v_frac_z', ...
                                        'clamp_u_frac_any', 'clamp_v_frac_any'}}, ...
                         'da_rows', Drows, ...
                         'da_row_names', {{'da_end', 'sqrtP55_end', 'sqrtP55_0', ...
                                           'P55_mono', 'frozen_frac', 't_freeze', ...
                                           'da_frac_by_descend_end', ...
                                           'hold_drift_pct_per_s', 'hold_delta_pct', ...
                                           'hold_unopposed_pct_per_s'}}, ...
                         'mean_desc_peak_pct', mu(1), 'std_desc_peak_pct', sd(1), ...
                         'mean_osc_rms_pct',   mu(2), 'std_osc_rms_pct',   sd(2), ...
                         'mean_hold_mean_pct', mu(3), 'std_hold_mean_pct', sd(3), ...
                         'mean_rms_all_pct',   mu(4), 'std_rms_all_pct',   sd(4), ...
                         'windows', local_metric_windows(cfg, OSC_SETTLE_S, HOLD_SETTLE_S), ...
                         'axis', AX_Z, 'prior_std_da', ov.Pf_da_std);
    out.floor = struct('local', flr.local, 'env', flr.env, 'w_sup', flr.w_sup, ...
                       'used', floor_used, 'mode', lower(opts.floor_mode), ...
                       'w_seed', w_seed, 'u0', u0, ...
                       'P_uu_parts', [P_uu_da, P_uu_w0, P_uu_flr], ...
                       'env_range', [env_lo, env_hi]);
    out.cfg     = cfg;
    out.opts    = opts;
    out.arm_tag = tag;
    out.seeds   = seeds;
    out.par_pkg = par_pkg;      % [] when par_law is off

    here = fileparts(mfilename('fullpath'));
    out_dir = fullfile(here, '..', '..', 'test_results');
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end
    out_file = fullfile(out_dir, ['run_formC_ugap_' tag '.mat']);
    save(out_file, 'out');
    fprintf('saved: %s\n', out_file);
end


%% =================== Local Helpers ===================

function cfg = local_canonical_config(a_cov, h_bar_min_prior)
%LOCAL_CANONICAL_CONFIG  Canonical hold->descend->osc->hold scenario, verbatim
%   from run_formC_dist so the two drivers are a paired comparison.
    pc = physical_constants();
    cfg = user_config();
    cfg.trajectory_type = 'osc';
    cfg.h_init    = 50;                  % [um] h_bar_0 = 22.2
    cfg.h_bottom  = 4.5;                 % [um] trough h_bar = 2.0
    cfg.amplitude = 2.5;                 % [um] oscillation half-amplitude
    cfg.frequency = 1;                   % [Hz]
    cfg.n_cycles  = 2;
    cfg.t_hold    = 0.5;                 % [s] initial hold
    cfg.t_descend_override = 1.0;        % [s] descent duration
    cfg.T_sim     = 4.8;                 % [s] leaves a 1.3 s final hold
    cfg.h_min     = h_bar_min_prior * pc.R;   % [um] prior-domain clamp
    cfg.ctrl_enable       = true;
    cfg.thermal_enable    = true;
    cfg.meas_noise_enable = true;
    cfg.lambda_c = 0.7;                  % closed-loop pole (canonical)
    cfg.a_pd     = 0.05;                 % LP EWMA weight (canonical)
    cfg.a_cov    = a_cov;                % variance EWMA weight (base 0.05)
    cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];   % [um] per axis
    cfg.h_bar_safe = 1.5;                % near-wall y2 gate (house value)
end


function w = local_metric_windows(cfg, osc_settle_s, hold_settle_s)
%LOCAL_METRIC_WINDOWS  Phase-boundary time windows [s] for the metrics.
    t1 = cfg.t_hold;
    t2 = t1 + cfg.t_descend_override;
    t3 = t2 + cfg.n_cycles / cfg.frequency;
    w = struct('descent', [t1, t2], ...
               'osc',     [t2 + osc_settle_s, t3], ...
               'hold',    [t3 + hold_settle_s, cfg.T_sim]);
end


function s = local_verdict(value, limit)
    if value <= limit; s = 'PASS'; else; s = 'FAIL'; end
end


function s = local_flag_str(idx, seeds)
    if isempty(idx)
        s = '';
    else
        s = [' <- flagged seeds ' mat2str(seeds(idx))];
    end
end


function p = local_p_below_one(u0, sigma_u)
%LOCAL_P_BELOW_ONE  Gaussian prior mass at u < 1 (a_bar < 0), the singularity
%   of the output map 1 - 1/u. This is the number that decides constraint (i):
%   it is a well-posedness statement, not a width preference.
    if sigma_u <= 0; p = 0; return; end
    p = 0.5 * erfc((u0 - 1) / (sigma_u * sqrt(2)));
end


function flr = local_shape_floors(w_seed, w_lo, w_hi, w0, da_seed, use_para)
%LOCAL_SHAPE_FLOORS  The two shape-floor candidates for THIS family.
%   flr = local_shape_floors(w_seed, w_lo, w_hi, w0, da_seed, use_para)
%
%   The integrated law of formC_ugap.tex is
%       a_law(w_bar) = 1 - 1/((1 + delta a)(w_bar - w0)) ,
%   with w0 the integration constant and delta a its seed (0 per the tex seed
%   section), so the residual against the published truth is
%       floor_a(w_bar) = |a_law(w_bar) - 1/c(w_bar)| .
%   Two readings of it:
%       .local  at the SEED HEIGHT -- the error the seed actually makes, and
%               the one constraint (i) of formC_ugap_ref requires, because the
%               floor is amplified by u_hat[0]^4 on its way into P_uu[0];
%       .env    the supremum over the planned envelope -- the width the a_bar
%               writing uses, which here puts 8% of the prior mass at a_bar<0.
%   use_para selects c_para (the x/y package) over c_perp. Offline truth
%   evaluation at driver init only; the controller stays run-time c-free.
    N_SWEEP = 20001;    % dense enough that the sup is grid-independent
    assert(w_hi > w_lo && w_lo - w0 > 0, 'run_formC_ugap:badEnvelope', ...
           'envelope must satisfy w_lo (%.3f) > w0 (%.3f) and w_hi > w_lo.', w_lo, w0);
    g = 1 + da_seed;

    w = linspace(w_lo, w_hi, N_SWEEP).';
    c = zeros(N_SWEEP, 1);
    for i = 1:N_SWEEP
        [c_para_i, c_perp_i] = calc_correction_functions(w(i), true);
        if use_para; c(i) = c_para_i; else; c(i) = c_perp_i; end
    end
    a_law = 1 - 1 ./ (g * (w - w0));
    [flr.env, i_sup] = max(abs(a_law - 1 ./ c));
    flr.w_sup = w(i_sup);

    [c_para_s, c_perp_s] = calc_correction_functions(w_seed, true);
    if use_para; c_seed = c_para_s; else; c_seed = c_perp_s; end
    flr.local = abs(1 - 1 / (g * (w_seed - w0)) - 1 / c_seed);
    flr.w_seed = w_seed;
end


function pkg = local_parallel_law_package_ugap(w_lo, w_hi)
%LOCAL_PARALLEL_LAW_PACKAGE_UGAP  Parallel-axis package (x/y). Verbatim from
%   run_formC_dist: the wall-parallel truth is not a member of this family
%   (Goldman's logarithm at contact), so the whole package is the integration
%   constant -- a 1-parameter minimax fit of a_law = 1 - 1/(w - w0) to
%   1/c_para on the planned envelope, with slot 5 locked at 0 on x/y. Offline
%   truth evaluation; the run-time loop stays c-free. x/y accuracy is DECLARED
%   out of scope -- with w_hat = z the mobility matrix is diagonal, so x/y
%   cannot contaminate the z metrics.
    N_FIT     = 4000;
    GAP_FLOOR = 1e-2;    % law-argument floor, matches the controller guard

    w = linspace(w_lo, w_hi, N_FIT).';
    c = zeros(N_FIT, 1);
    for i = 1:N_FIT
        c(i) = calc_correction_functions(w(i));      % c_para (first output)
    end
    a_target = 1 ./ c;

    obj = @(w0) max(abs(1 - 1 ./ max(w - w0, GAP_FLOOR) - a_target));
    i_mid = round(N_FIT / 2);
    w0_0  = w(i_mid) - 1 / (1 - a_target(i_mid));    % exact match at midpoint
    solver_opts = optimset('TolX', 1e-10, 'TolFun', 1e-12, ...
                           'MaxFunEvals', 2e4, 'MaxIter', 2e4, 'Display', 'off');
    w0 = fminsearch(obj, w0_0, solver_opts);
    w0 = fminsearch(obj, w0,  solver_opts);          % restart: minimax is non-smooth

    pkg = struct();
    pkg.w0      = w0;
    pkg.fit_sup = obj(w0);
end


function s = local_arm_words(arm)
    if strcmpi(arm, 'base')
        s = 'derivation (a): 4 states, delta a locked at 0';
    else
        s = 'derivation (b): 5 states, multiplicative delta a, Q_dada = 0';
    end
end


function fs = local_freeze_stats(t, da, t_start)
%LOCAL_FREEZE_STATS  When does delta_a_hat stop moving? (sibling definition)
    Ts = t(2) - t(1);
    W  = max(2, round(0.05 / Ts));
    rate = zeros(size(da));
    rate(W+1:end) = abs(da(W+1:end) - da(1:end-W)) / (W * Ts);

    m  = t >= t_start;
    r  = rate(m);
    tt = t(m);
    [fs.peak_rate, i_pk] = max(r);
    fs.t_peak  = tt(i_pk);
    fs.window_s = W * Ts;
    if fs.peak_rate <= 0
        fs.frac = 1; fs.t_freeze = tt(1); fs.t_peak = NaN;
        return;
    end
    thr = 0.01 * fs.peak_rate;
    frozen = r < thr;
    fs.frac = mean(frozen);
    i_last_moving = find(~frozen, 1, 'last');
    if isempty(i_last_moving)
        fs.t_freeze = tt(1);
    elseif i_last_moving >= numel(tt)
        fs.t_freeze = NaN;          % still moving at the end
    else
        fs.t_freeze = tt(i_last_moving + 1);
    end
end


function m = local_run_metrics(s, cfg, ax, osc_settle_s, hold_settle_s)
%LOCAL_RUN_METRICS  Per-run console metrics on the focus axis (sibling's set
%   verbatim, plus the constraint (ii) clamp fractions).
    t  = s.tout(:);
    w  = local_metric_windows(cfg, osc_settle_s, hold_settle_s);
    aT = s.a_true_out(:, ax);
    e_pct = 100 * (s.a_hat_out(:, ax) - aT) ./ max(aT, eps);

    w_desc = t >= w.descent(1) & t <= w.descent(2);
    w_osc  = t >  w.osc(1)     & t <= w.osc(2);
    w_hold = t >  w.hold(1);
    assert(any(w_desc) && any(w_osc) && any(w_hold), ...
           'run_formC_ugap:emptyMetricWindow', ...
           'A metric window is empty -- check trajectory timing overrides.');

    m.desc_peak_pct = max(abs(e_pct(w_desc)));
    m.osc_rms_pct   = sqrt(mean(e_pct(w_osc).^2));
    m.hold_mean_pct = mean(e_pct(w_hold));
    w_all = t >= w.descent(1);
    m.rms_all_pct = sqrt(mean(e_pct(w_all).^2));

    % --- delta a diagnostics (slot 5; b_hat/P_b are its aliases) ----------
    da_hat = s.b_hat_out(:, ax);
    P55    = s.P_b_out(:, ax).^2;          % P_*_out store sqrt(P)
    m.da_end      = da_hat(end);
    m.sqrtP55_0   = s.P_b_out(1, ax);
    m.sqrtP55_end = s.P_b_out(end, ax);
    if P55(1) > 0
        m.P55_mono = max([0; diff(P55)]) / P55(1);
    else
        m.P55_mono = 0;                    % locked slot: P55 == 0 throughout
    end
    fs = local_freeze_stats(t, da_hat, w.descent(1));
    m.frozen_frac = fs.frac;
    m.t_freeze    = fs.t_freeze;
    m.freeze      = fs;

    k_desc_end = find(t >= w.descent(2), 1);
    m.da_descend_end = da_hat(k_desc_end);
    if abs(da_hat(end)) > 0
        m.da_frac_by_descend_end = da_hat(k_desc_end) / da_hat(end);
    else
        m.da_frac_by_descend_end = NaN;      % locked arm
    end

    t_h  = t(w_hold);
    e_h  = e_pct(w_hold);
    pfit = polyfit(t_h - t_h(1), e_h, 1);
    m.hold_drift_pct_per_s = pfit(1);
    m.hold_delta_pct       = e_h(end) - e_h(1);
    m.hold_span_s          = t_h(end) - t_h(1);
    n_hold = numel(t_h);
    a_true_hold = mean(aT(w_hold)) / s.a_nom;      % normalized true gain
    m.hold_unopposed_pct = 100 * n_hold * da_hat(end) / a_true_hold;

    trav2_b    = (s.b_hat_out(end, ax) - s.b_hat_out(1, ax))^2;
    budget_b   = s.P_b_out(1, ax)^2 - s.P_b_out(end, ax)^2;
    m.budget_b = trav2_b / max(budget_b, eps);
    m.budget_p = NaN;                     % slot 6 is inert in this writing

    % --- constraint (ii): the clamps must never bind ---------------------
    % Reported BOTH on the focus axis and over all three, because the
    % parallel package seeds x/y at a DIFFERENT origin (w0_par < 0) with a
    % different floor: an x/y binding says nothing about the z arm under
    % test, but hiding it behind a z-only fraction would be dishonest.
    m.clamp_u_frac     = s.clamp_u_count(ax) / max(s.clamp_steps, 1);
    m.clamp_v_frac     = s.clamp_v_count(ax) / max(s.clamp_steps, 1);
    m.clamp_u_frac_any = max(s.clamp_u_count) / max(s.clamp_steps, 1);
    m.clamp_v_frac_any = max(s.clamp_v_count) / max(s.clamp_steps, 1);

    m.any_nan = any(~isfinite(s.p_true_out(:))) || any(~isfinite(s.a_hat_out(:))) ...
                || any(~isfinite(s.b_hat_out(:))) || any(~isfinite(s.u_hat_out(:)));
end


function simOut = local_run_once(config, seed, ctrl_const_override, verbose, ...
                                 a_ctrl_override, log_P_full, ws_inject, plant_cperp)
%LOCAL_RUN_ONCE  One seed of the scenario. Fork of run_formC_dist's body with
%   the controller hard-dispatched to motion_control_law_formC_ugap and the
%   known-disturbance / true-slope diagnostic arms removed (both are
%   a_bar-coordinate objects: the increment here reads no state at all, so
%   there is no integrand to feed).
    if nargin < 7 || isempty(ws_inject);  ws_inject = 0;   end
    if nargin < 8;                        plant_cperp = []; end

    % Fresh persistent state per run (controller, trajectory, thermal).
    clear motion_control_law_formC_ugap trajectory_generator calc_thermal_force;

    % --- RNG seed BEFORE params (thermal/meas seeds drawn from global rng) ---
    rng(seed);

    params = calc_simulation_params(config);
    P = params.Value;

    % --- Offline constants (dimension-agnostic; reuse the 6-state builder) ---
    eq17_opts = struct();
    eq17_opts.lambda_c    = config.lambda_c;
    eq17_opts.option      = 'A_MA2_full';
    eq17_opts.sigma2_n_s  = (config.meas_noise_std(:)).^2;
    eq17_opts.kBT         = P.ctrl.k_B * P.ctrl.T;
    eq17_opts.d           = 2;
    eq17_opts.a_cov       = config.a_cov;
    eq17_opts.a_pd        = config.a_pd;
    eq17_opts.t_warmup_kf = 0;                 % prefill init: no warm-up gate
    eq17_opts.h_bar_safe  = 1.5;
    if isfield(config, 'h_bar_safe') && ~isempty(config.h_bar_safe)
        eq17_opts.h_bar_safe = config.h_bar_safe;
    end
    eq17_opts.iir_warmup_mode = 'prefill';
    ctrl_const = build_eq17_6state_constants(eq17_opts);

    fn = fieldnames(ctrl_const_override);
    for idx = 1:numel(fn)
        ctrl_const.(fn{idx}) = ctrl_const_override.(fn{idx});
    end

    % --- Time grid ---
    Ts = P.common.Ts;
    N = round(config.T_sim / Ts) + 1;
    tout = (0:N-1)' * Ts;

    % --- State init ---
    p0 = P.common.p0;
    p_curr = p0;
    d_delay = ctrl_const.d;
    p_m_buffer = repmat(p0, 1, d_delay + 1);   % d+1 slots -> true d-step delay
    pd_for_ctrl = p0;

    R_drv       = P.common.R;
    a_nom_drv   = P.common.Ts / P.common.gamma_N;      % [um/pN] far-field
    wall_on_drv = isfield(P, 'wall') && P.wall.enable_wall_effect > 0.5;
    pz_plant = 0;
    P_plant  = P;                    % plant-side params (shifted wall)
    if wall_on_drv
        pz_plant = P.wall.pz + ws_inject * R_drv;
        P_plant.wall.pz = pz_plant;
    end
    if wall_on_drv && ~isempty(plant_cperp)
        P_plant.wall.plant_cperp = plant_cperp;
    end
    if wall_on_drv && isfield(P.wall, 'h_bar_min')
        h_bar_floor_drv = P.wall.h_bar_min;
    else
        h_bar_floor_drv = 1.001;
    end

    % --- Logs (sibling schema + the u-coordinate rows) ---
    p_d_out  = zeros(N, 3);
    f_d_out  = zeros(N, 3);
    F_th_out = zeros(N, 3);
    p_m_out  = zeros(N, 3);
    p_true_out = zeros(N, 3);
    a_true_out = zeros(N, 3);
    a_prime_true_out = zeros(N, 3);
    ekf_out  = zeros(N, 4);
    a_hat_out  = zeros(N, 3);
    b_hat_out  = zeros(N, 3);
    p_hat_out  = zeros(N, 3);
    ws_hat_out = zeros(N, 3);
    u_hat_out  = zeros(N, 3);
    v_hat_out  = zeros(N, 3);
    P_uu_out   = zeros(N, 3);
    Sigma_hat_out = zeros(N, 3);
    h_bar_out = zeros(N, 1);
    h_bar_true_out = zeros(N, 1);
    h_bar_d_out = zeros(N, 1);
    gate_out  = false(N, 3);
    a_xm_out  = zeros(N, 3);
    a_prime_out = zeros(N, 3);
    P_a_out     = zeros(N, 3);
    P_b_out     = zeros(N, 3);
    P_p_out     = zeros(N, 3);
    P_ws_out    = zeros(N, 3);
    innov_y2_out = zeros(N, 3);
    innov_y1_out = zeros(N, 3);
    dws_y1_out = zeros(N, 3);
    dws_y2_out = zeros(N, 3);
    K_a_y2_out   = zeros(N, 3);
    K_a_y1_out   = zeros(N, 3);
    P41_out      = zeros(N, 3);
    R2_out       = zeros(N, 3);
    dx_r_out     = zeros(N, 3);
    dh_m_out     = zeros(N, 3);
    a_bar_hat_out = zeros(N, 3);
    Q33_out       = zeros(N, 3);
    a_bar_Q_out   = zeros(N, 3);
    f_bar_out     = zeros(N, 3);
    if log_P_full
        P_full_out = [];   % sized on first step
    end
    clamp_u_count = zeros(3, 1);
    clamp_v_count = zeros(3, 1);
    clamp_steps   = 0;

    for k = 1:N
        t_now = tout(k);

        [pd_kp1, del_pd_k] = trajectory_generator(t_now, P);
        pd_k = pd_for_ctrl;
        p_m_delayed = p_m_buffer(:, 1);

        if wall_on_drv
            h_bar_true_k = max((dot(p_curr, P.wall.w_hat) - pz_plant) / R_drv, h_bar_floor_drv);
            [c_para_k, c_perp_k] = calc_correction_functions(h_bar_true_k);
            if ~isempty(plant_cperp)
                c_perp_k = plant_cperp(h_bar_true_k);
            end
            a_true_k = [a_nom_drv / c_para_k; a_nom_drv / c_para_k; a_nom_drv / c_perp_k];
            a_prime_true_out(k, :) = local_a_prime_true(h_bar_true_k, a_nom_drv, h_bar_floor_drv, plant_cperp).';
        else
            h_bar_true_k = Inf;
            a_true_k = a_nom_drv * ones(3, 1);
        end

        if ischar(a_ctrl_override) || isstring(a_ctrl_override)
            a_ov_k = a_true_k;
        else
            a_ov_k = a_ctrl_override;
        end

        [f_d_k, ekf_k, diag_k] = motion_control_law_formC_ugap(del_pd_k, pd_k, ...
                                     p_m_delayed, P, ctrl_const, a_ov_k);

        if P.thermal.enable > 0.5
            f_th_k = calc_thermal_force(p_curr, P_plant);
        else
            f_th_k = zeros(3, 1);
        end

        F_total = f_d_k + f_th_k;
        p_curr = step_dynamics(p_curr, F_total, P_plant, Ts);

        if config.meas_noise_enable
            n_meas = config.meas_noise_std(:) .* randn(3, 1);
        else
            n_meas = zeros(3, 1);
        end
        p_m_raw = p_curr + n_meas;
        p_m_buffer = [p_m_buffer(:, 2:end), p_m_raw];

        p_d_out(k, :)  = pd_k.';
        f_d_out(k, :)  = f_d_k.';
        F_th_out(k, :) = f_th_k.';
        p_m_out(k, :)  = p_m_raw.';
        p_true_out(k, :) = p_curr.';
        a_true_out(k, :) = a_true_k.';
        ekf_out(k, :)  = ekf_k.';
        a_hat_out(k, :)  = diag_k.a_hat.';
        b_hat_out(k, :)  = diag_k.b_hat.';
        p_hat_out(k, :)  = diag_k.p_hat.';
        ws_hat_out(k, :) = diag_k.ws_hat.';
        u_hat_out(k, :)  = diag_k.u_hat.';
        v_hat_out(k, :)  = diag_k.v_hat.';
        P_uu_out(k, :)   = diag_k.P_uu.';
        Sigma_hat_out(k, :) = diag_k.Sigma_hat.';
        h_bar_out(k)    = diag_k.h_bar;
        h_bar_true_out(k) = h_bar_true_k;
        h_bar_d_out(k)  = diag_k.h_bar_d;
        gate_out(k, :)  = diag_k.gate_active_per_axis(:).';
        a_xm_out(k, :)  = diag_k.a_xm(:).';
        a_prime_out(k, :) = diag_k.a_prime_hat(:).' * R_drv;   % 1/pN -> um/pN
        P_a_out(k, :)     = sqrt(max(diag_k.P_a(:).', 0));
        P_b_out(k, :)     = sqrt(max(diag_k.P_b(:).', 0));
        P_p_out(k, :)     = sqrt(max(diag_k.P_p(:).', 0));
        P_ws_out(k, :)    = sqrt(max(diag_k.P_ws(:).', 0));
        innov_y2_out(k, :) = diag_k.innovation_y2(:).';
        innov_y1_out(k, :) = diag_k.innovation_y1(:).';
        dws_y1_out(k, :) = diag_k.dws_y1(:).';
        dws_y2_out(k, :) = diag_k.dws_y2(:).';
        K_a_y2_out(k, :)   = diag_k.K_kf_a_y2(:).';
        K_a_y1_out(k, :)   = diag_k.K_kf_a_y1(:).';
        P41_out(k, :)      = diag_k.P41(:).';
        R2_out(k, :)       = diag_k.R2(:).';
        dx_r_out(k, :)     = diag_k.dx_r(:).';
        dh_m_out(k, :)     = diag_k.delta_x_m(:).';
        a_bar_hat_out(k, :) = diag_k.a_bar_hat(:).';
        Q33_out(k, :)       = diag_k.Q33(:).';
        a_bar_Q_out(k, :)   = diag_k.a_bar_Q(:).';
        f_bar_out(k, :)     = diag_k.f_bar(:).';
        clamp_u_count = diag_k.clamp_u_count(:);
        clamp_v_count = diag_k.clamp_v_count(:);
        clamp_steps   = diag_k.clamp_steps;
        if log_P_full
            np = size(diag_k.P_full, 1);
            if isempty(P_full_out)
                P_full_out = zeros(N, np, np, 3);
            end
            P_full_out(k, :, :, :) = reshape(diag_k.P_full, [1, np, np, 3]);
        end

        pd_for_ctrl = pd_kp1;

        if verbose && N >= 10 && mod(k, max(1, round(N/10))) == 0
            fprintf('  step %d/%d (t=%.3fs)\n', k, N, t_now);
        end
    end

    simOut.p_d_out    = p_d_out;
    simOut.f_d_out    = f_d_out;
    simOut.F_th_out   = F_th_out;
    simOut.p_m_out    = p_m_out;
    simOut.p_true_out = p_true_out;
    simOut.a_true_out = a_true_out;
    simOut.a_prime_true_out = a_prime_true_out;
    simOut.tout       = tout;
    simOut.ekf_out    = ekf_out;
    simOut.a_hat_out  = a_hat_out;
    simOut.b_hat_out  = b_hat_out;
    simOut.p_hat_out  = p_hat_out;
    simOut.ws_hat_out = ws_hat_out;
    simOut.u_hat_out  = u_hat_out;
    simOut.v_hat_out  = v_hat_out;
    simOut.P_uu_out   = P_uu_out;
    simOut.Sigma_hat_out = Sigma_hat_out;
    simOut.h_bar_out  = h_bar_out;
    simOut.h_bar_true_out = h_bar_true_out;
    simOut.ws_inject = ws_inject;
    simOut.h_bar_d_out = h_bar_d_out;
    simOut.gate_out   = gate_out;
    simOut.a_xm_out   = a_xm_out;
    simOut.a_prime_out = a_prime_out;
    simOut.P_a_out     = P_a_out;
    simOut.P_b_out     = P_b_out;
    simOut.P_p_out     = P_p_out;
    simOut.P_ws_out    = P_ws_out;
    simOut.innov_y2_out = innov_y2_out;
    simOut.innov_y1_out = innov_y1_out;
    simOut.dws_y1_out = dws_y1_out;
    simOut.dws_y2_out = dws_y2_out;
    simOut.K_a_y2_out   = K_a_y2_out;
    simOut.K_a_y1_out   = K_a_y1_out;
    simOut.P41_out      = P41_out;
    simOut.R2_out       = R2_out;
    simOut.dx_r_out     = dx_r_out;
    simOut.dh_m_out     = dh_m_out;
    simOut.a_bar_hat_out = a_bar_hat_out;
    simOut.Q33_out       = Q33_out;
    simOut.a_bar_Q_out   = a_bar_Q_out;
    simOut.f_bar_out     = f_bar_out;
    simOut.clamp_u_count = clamp_u_count;
    simOut.clamp_v_count = clamp_v_count;
    simOut.clamp_steps   = clamp_steps;
    if log_P_full
        simOut.P_full_out = P_full_out;
    end
    simOut.a_nom        = a_nom_drv;
    simOut.R            = R_drv;
    simOut.meta = struct('config', config, 'params_value', P, ...
                         'seed', seed, 'driver', 'run_formC_ugap', ...
                         'ctrl_const_override', ctrl_const_override);
    simOut.ctrl_const = ctrl_const;
end


function ap = local_a_prime_true(h_bar, a_nom, h_floor, plant_cperp)
%LOCAL_A_PRIME_TRUE  d a_h / d h_bar on the EXACT correction curve [um/pN].
%   Central difference; used only as the reference in the analysis, never by
%   the controller.
    if nargin < 4; plant_cperp = []; end
    dh = 1e-4 * max(h_bar, 1);
    hp = h_bar + dh;
    hm = max(h_bar - dh, h_floor);
    [cp_p, ce_p] = calc_correction_functions(hp);
    [cp_m, ce_m] = calc_correction_functions(hm);
    if ~isempty(plant_cperp)
        ce_p = plant_cperp(hp);
        ce_m = plant_cperp(hm);
    end
    den = hp - hm;
    ap_para = a_nom * (1/cp_p - 1/cp_m) / den;
    ap_perp = a_nom * (1/ce_p - 1/ce_m) / den;
    ap = [ap_para; ap_para; ap_perp];
end
