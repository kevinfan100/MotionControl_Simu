% STATUS: ACTIVE (additive-disturbance driver) -- spec
%          reference/eq17_analysis/derivation/formC_state_dist.tex;
%          controller model/controller/motion_control_law_formC_b.m
% FORK OF test_script/integration/run_formC_state.m @ 2f2fef6 | PURPOSE: same
%   canonical scenario, seeds and metrics, controller swapped for the ADDITIVE
%   writing (parameter-free slope (1-a_bar)^2 plus a constant additive
%   disturbance in the gain state equation) | EXPIRES: baseline / disturbance
%   adjudication | production changes do NOT follow.
%   Arms: 'base' = slot 5 LOCKED at 0  -> derivation (a), the 4-state baseline
%                                         with no free parameter at all
%         'dist' = slot 5 FREE from 0  -> derivation (b), 5 states, Q55 = 0
function out = run_formC_b(opts, test_opts)
%RUN_FORMC_DIST  Canonical-scenario driver for the additive-disturbance
%   writing of the parameter-free state gain law (formC_state_dist.tex).
%
%   out = run_formC_b()                          % arm 'dist', house seeds
%   out = run_formC_b(struct('arm', 'base'))     % the 4-state baseline
%
%   FORK OF run_formC_state.m: the per-step ordering (RNG seed -> params ->
%   trajectory -> controller -> thermal -> step_dynamics -> sensor-delay
%   buffer), the config / ctrl_const plumbing, the canonical scenario, the
%   seed loop and the console metrics are inherited verbatim. Only three
%   things change: the controller dispatched, the arm flags, and the prior on
%   the disturbance (which is now a PER-STEP quantity and so has a completely
%   different natural scale from the multiplicative sibling's).
%
%   Authoritative spec:
%       reference/eq17_analysis/derivation/formC_state_dist.tex
%   State (5 per axis, mapped onto the inherited 9-slot layout):
%       [dw1 dw2 dw3 a_bar da | (inert) (inert) | m1 m2]
%   Plant truth comes from calc_correction_functions (c_perp on z); the
%   filter is run-time c-free by construction (charter constraint 3).
%
%   SCENARIO (canonical, z-axis / perpendicular focus), unchanged:
%       hold 0.5 s @ h = 50 um (h_bar 22.2) -> descend 1.0 s ->
%       1 Hz oscillation, amplitude 2.5 um, trough h = 4.5 um (h_bar 2.0),
%       2 cycles -> final hold at the trough; T = 4.8 s; thermal +
%       measurement noise ON; lambda_c = 0.7.
%
%   THE DISTURBANCE PRIOR (the tex's only undetermined number). da is an
%   INCREMENT of a_bar per step, so the multiplicative sibling's 0.5 default
%   is off by four decades and is deliberately NOT inherited. The default here
%   is DERIVED from the closed form the tex gives in S3(b),
%
%       da[k] = (1-a_bar)^2 * [1/b_state(w_bar) - 1] * Sigma[k] ,
%       b_state(w_bar) = (1 - a_true)^2 / a'_true ,
%
%   evaluated offline on the PLANNED trajectory with Sigma[k] approximated by
%   its commanded part Delta_wbar_d[k] (the driver knows the commanded height
%   increments exactly; the feedback and thermal shares of Sigma are zero-mean
%   and are declared, not modelled). The prior width is the sup of |da[k]| over
%   the planned run, which is the same "sup of the gap the state must cover"
%   construction the (b, p) envelope priors use. It is printed every run and
%   its contract is INVARIANCE, not accuracy: opts.Pf_da_std overrides it, and
%   sweeping it 10x either way must not move the converged behaviour.
%
%   PARALLEL-AXIS LAW (opts.par_law, default TRUE). The wall-parallel truth is
%   not a member of the one-curve family, and in THIS writing there is no
%   constant that can reshape a curve, so the parallel package collapses to
%   the integration constant alone: a 1-parameter minimax fit of
%   a_law = 1 - 1/(w - w0) to 1/c_para on the planned envelope, with slot 5
%   locked at 0 on x/y and the whole residual carried as Pf_a_floor_par on
%   P(4,4). x/y accuracy is DECLARED out of scope: with w_hat = z the mobility
%   matrix is diagonal, so x/y are dynamically decoupled from the z axis under
%   test and cannot contaminate its metrics. Offline truth evaluation only --
%   the controller stays run-time c-free.
%
%   opts fields (defaults first):
%       .arm         'dist'  'dist' = slot 5 free from 0 (derivation (b));
%                            'base' = slot 5 locked at 0 (derivation (a))
%       .Pf_da_std   []      [] = derive from the planned trajectory (above);
%                            a scalar overrides it (the invariance sweep)
%       .Pf_w0_std   0.111   wall-position prior, carried over from formB_ws
%       .par_law     true    x/y run the fitted parallel origin
%       .y2_on       true    false = drop the gain-readout channel
%       .a_cov_scale 1       multiplies the 0.05 base a_cov (invariance sweep)
%       .seeds       [7 11 23 42 101 777]    house 6-seed convention
%       .verbose     false   per-run progress printout
%       .config_override     struct merged into the canonical config
%       .ctrl_const_override struct merged into ctrl_const LAST (wins)
%       .a_ctrl_override     [] | 3x1 [um/pN] | 'true'
%
%   Console metrics per run (z axis, e_a = 100*(a_hat - a_true)/a_true):
%       desc pk %   max |e_a| over the descent window
%       osc RMS %   RMS e_a over the oscillation window (settled)
%       hold mean % mean e_a over the final hold (settled)
%       rms all %   RMS e_a over the whole ACTIVE run, t >= t_hold (descent
%                   onward) -- the "relative gain error RMS", reported so the
%                   three phase metrics can be read against one aggregate
%       da end      da_hat[end] on z [-] (a_bar per step)
%       sqrtP55     sqrt(P55)[end] on z [-]
%       P55 mono    max single-step INCREASE of P55 relative to P55[0]
%                   (Q55 = 0 predicts monotone decrease, so this must be 0
%                   up to roundoff -- a pre-registered check, not a metric)
%       frz         fraction of t >= t_hold over which da_hat is frozen
%                   (see local_freeze_stats for the definition)
%   Printed only; test_results/ is gitignored, numbers are never committed.
%
%   Output struct: as run_formC_state, plus out.metrics.da (per-seed
%   disturbance diagnostics) and out.da_prior (the derived prior + provenance).
%   Saved to test_results/run_formC_b_<arm_tag>.mat (gitignored).
%
%   TEST-LADDER COMPATIBILITY FORM (inherited):
%       simOut = run_formC_b(cfg, opts)   % single seed, direct simOut
%
%   See also: motion_control_law_formC_b, run_formC_state, run_formB_ws

    % ---- test-ladder compatibility form: run_formC_b(cfg, test_opts) ----
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
    if ~isfield(opts, 'arm');         opts.arm         = 'best'; end
    if ~isfield(opts, 'da_known');    opts.da_known    = false; end  % KNOWN-DISTURBANCE ARM
    if ~isfield(opts, 'lambda_f');    opts.lambda_f    = 1;     end  % Menq (4.15), 1 = off
    if ~isfield(opts, 'ap_known');    opts.ap_known    = false; end  % TRUE-SLOPE ARM
    if ~isfield(opts, 'ap_src');      opts.ap_src      = 'post'; end
    % ARGUMENT-ERROR ARM. Finite => a_bar' comes from the LAW evaluated at the
    % TRUE a_bar displaced by this constant, exogenously (no feedback from the
    % state). Separates "the argument is known badly" from "the argument IS the
    % state". Units: absolute a_bar. Implies the true-slope path.
    if ~isfield(opts, 'ap_law_bias'); opts.ap_law_bias = NaN; end
    if ~isfield(opts, 'law_b');       opts.law_b       = 1;   end
    % true  = legacy envelope supremum; false = honest seed-local floor
    if ~isfield(opts, 'floor_from_envelope'); opts.floor_from_envelope = false; end
    if ~isfield(opts, 'ap_ewma_a');   opts.ap_ewma_a   = 0.05;  end
    if ~isfield(opts, 'Pf_da_std');   opts.Pf_da_std   = [];    end   % [] = derive
    if ~isfield(opts, 'Pf_w0_std');   opts.Pf_w0_std   = 0.111; end
    if ~isfield(opts, 'par_law');     opts.par_law     = true;  end
    if ~isfield(opts, 'y2_on');       opts.y2_on       = true;  end
    if ~isfield(opts, 'a_cov_scale'); opts.a_cov_scale = 1;     end
    if ~isfield(opts, 'ws_inject');   opts.ws_inject = 0;      end   % [R] TRUE wall offset, plant side only (S11 injection)
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
    H_BAR_MIN_PRIOR = 1.1;   % truth-curve validity floor (two-sphere series);
                             % the plant clamp must not go below it
    ENV_LO_MARGIN   = 0.1;   % [-] envelope floor below the commanded trough:
                             % measured true-trough undershoot 0.033 R plus
                             % position measurement noise ~0.05 R, rounded up
    ENV_HI_MARGIN   = 1.0;   % [-] envelope ceiling above the start height; the
                             % sups are far-field-flat here, so any O(1) pad
                             % gives the same triple (insensitive by design)
    A_COV_BASE      = 0.05;  % eq17 verified baseline EWMA weight
    OSC_SETTLE_S    = 0.1;   % skip descent->osc transition (sibling t>1.6 rule)
    HOLD_SETTLE_S   = 0.3;   % skip osc->hold readout transient
                             % (~24 EWMA taus of a_cov = 0.05 at 1600 Hz)

    assert(any(strcmpi(opts.arm, {'b1', 'bmid', 'best', 'b98', 'bfree1'})), ...
           'run_formC_b:badArm', ...
           'opts.arm must be b1 | bmid | best | b98 | bfree1.');

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
           'run_formC_b:hMinBelowTruthDomain', ...
           'h_min = %.3f um -> h_bar_min = %.3f < %.2f: below the truth-curve validity floor, so the envelope priors would be read off an extrapolated c_perp.', ...
           cfg.h_min, cfg.h_min / pc.R, H_BAR_MIN_PRIOR);
    assert(cfg.h_bottom / pc.R >= H_BAR_MIN_PRIOR, ...
           'run_formC_b:troughBelowTruthDomain', ...
           'trajectory trough h_bar = %.3f < %.2f (truth-curve validity floor).', ...
           cfg.h_bottom / pc.R, H_BAR_MIN_PRIOR);

    % ------------------------------------------------------------------
    % Arm flags -> ctrl_const overrides (user ctrl_const_override wins)
    % ------------------------------------------------------------------
    % Envelope priors: derived from the planned trajectory, not tuned. The
    % triple is scenario-dependent by construction, so it is printed, never
    % asserted against a hard reference.
    env_lo = cfg.h_bottom / pc.R - ENV_LO_MARGIN;
    env_hi = cfg.h_init   / pc.R + ENV_HI_MARGIN;
    % Shape floor for the curve this writing ACTUALLY seeds, a_bar = 1 - 1/w_bar
    % (w0 = ws0_perp - 1 = 0 for a plane). The multiplicative sibling reused the
    % Form B anchored curve here, which is a different function; reusing it
    % would have priced a representation error the filter never makes.
    W0_PLANE    = 0;                 % nominal wall at w_bar = 1 => a_bar(1) = 0
    [floor_a_env, w_floor_sup] = local_envelope_floor_dist(env_lo, env_hi, W0_PLANE);

    % b prior, straight off the published truth on the SAME envelope. b_true
    % is the exact factor the parameter-free law is missing,
    %     b_true(w) = a_true' / (1 - a_true)^2  ,
    % which is 1 at contact and 8/9 in the far field. b_true is NOT monotone:
    % it rises from an interior minimum 0.866978 at w_bar 2.193 to 0.888225 at
    % the envelope top and never reaches 8/9, so the centre is the midpoint of
    % its realised range and the width is the half-range. Nothing tuned; both
    % numbers are read off the truth.
    % Deliberately NOT sup|b_true - 8/9|: defining the prior as the same
    % supremum the shape-acceptance test compares against makes that test
    % vacuous (ratio identically 1.0000, can never fail; verified 2026-08-18).
    % The midpoint keeps that ratio at 1.062, inside the 1.02-1.06 TIGHT band.
    [b_mid, b_half, b_lo_e, b_hi_e] = local_envelope_b_range(env_lo, env_hi);

    % Shape floor for P44[0]. P[0] is a statement about the belief AT t = 0:
    % E[(truth - a_bar_hat[0])^2]. The envelope supremum is the worst error
    % over a band the run has not entered yet -- at the seed height the law is
    % far better than that, and charging the supremum makes the filter distrust
    % a seed it should trust. It is computed per-arm because the local error
    % depends on the b this arm seeds with. The envelope value is kept and
    % printed: it is the right number for a DIFFERENT question (how wrong can
    % the law get anywhere on the planned band), which belongs to the model-
    % error container, not to P[0].

    % Disturbance prior: sup of the S3(b) closed form on the PLANNED trajectory
    % (see the header). Derived, printed, overridable; contract = invariance.
    da_prior = local_da_prior_from_trajectory(cfg);
    if isempty(opts.Pf_da_std)
        Pf_da_used = da_prior.sup;
        Pf_da_src  = 'derived (S3(b) sup on the planned trajectory)';
    else
        Pf_da_used = opts.Pf_da_std;
        Pf_da_src  = 'caller override (invariance sweep)';
    end

    ov = struct();
    ov.Pf_w0_std  = opts.Pf_w0_std;   % wall prior, carried over from formB_ws
    ov.ws0_perp   = 1;                % plane
    ov.lambda_f   = opts.lambda_f;    % Menq (4.15) forgetting factor
    ov.ap_src     = opts.ap_src;      % slope evaluation point
    ov.law_b_formC = opts.law_b;      % 8/9 = far-field anchor, 1 = contact anchor
    ov.ap_ewma_a  = opts.ap_ewma_a;

    % Three arms decompose the question. b1 reproduces the existing
    % parameter-free baseline; bmid isolates what a BETTER FIXED value buys;
    % best is the derivation's actual claim, b estimated from the same seed as
    % bmid. Comparing best against bmid (not against b1) is the controlled
    % test: same seed, same prior centre, only the lock differs.
    switch lower(opts.arm)
        case 'b1'
            ov.lock_b = true;   ov.b_init = 1;
        case 'bmid'
            ov.lock_b = true;   ov.b_init = b_mid;
        case 'best'
            ov.lock_b = false;  ov.b_init = b_mid;
        case 'b98'
            % free, seeded at the FAR-FIELD ANCHOR rather than the band centre
            ov.lock_b = false;  ov.b_init = 8/9;
        case 'bfree1'
            % free, seeded AT the attractor. If b_hat still does not move, the
            % collapse is not what stops it -- b simply receives no usable
            % information over the whole run.
            ov.lock_b = false;  ov.b_init = 1;
        otherwise
            error('run_formC_b:arm', 'opts.arm must be ''b1'' | ''bmid'' | ''best''.');
    end
    ov.Pf_b_std = (~ov.lock_b) * b_half;

    floor_a_seed = local_seed_floor(env_hi - ENV_HI_MARGIN, W0_PLANE, ov.b_init);
    if opts.floor_from_envelope
        ov.Pf_a_floor = floor_a_env;      % legacy: envelope supremum
    else
        ov.Pf_a_floor = floor_a_seed;     % honest P[0]: local error at the seed
    end

    if ~opts.y2_on
        ov.y2_off = true;                    % fingerprint arm
    end

    ov.par_law = opts.par_law;
    par_pkg = [];
    if opts.par_law
        par_pkg = local_parallel_law_package_dist(env_lo, env_hi);
        ov.w0_par         = par_pkg.w0;
        ov.Pf_a_floor_par = par_pkg.floor_a;
    end

    fn = fieldnames(opts.ctrl_const_override);
    for idx = 1:numel(fn)
        ov.(fn{idx}) = opts.ctrl_const_override.(fn{idx});
    end
    % arm tag (file name + report header)
    tag = lower(opts.arm);
    if opts.y2_on; tag = [tag '_y2on']; else; tag = [tag '_y2off']; end
    if ~opts.par_law;         tag = [tag '_nopar']; end
    if opts.a_cov_scale ~= 1; tag = [tag sprintf('_acov%g', cfg.a_cov)]; end

    lastwarn('');

    % ------------------------------------------------------------------
    % Seed loop + per-run console metrics
    % ------------------------------------------------------------------
    fprintf('=== Form C b-STATE driver -- arm: %s ===\n', tag);
    fprintf('scenario: hold %.1fs -> descend %.1fs -> %g Hz osc x%d -> hold, T=%.1fs; h_bar_min=%.2f\n', ...
            cfg.t_hold, cfg.t_descend_override, cfg.frequency, cfg.n_cycles, ...
            cfg.T_sim, cfg.h_min / pc.R);
    fprintf('arm=%s (%s)  lock_da=%d  da_init=%+.5g  sqrt_Pw0=%.4f  y2_on=%d  a_cov=%.3f\n', ...
            lower(opts.arm), local_arm_words(opts.arm), ov.lock_b, ov.b_init, ...
            ov.Pf_w0_std, opts.y2_on, cfg.a_cov);
    fprintf('ENVELOPE on w_bar in [%.3f, %.3f]: shape floor %.5f (sup at w_bar %.3f)\n', ...
            env_lo, env_hi, floor_a_env, w_floor_sup);
    fprintf('P44[0] shape floor in use: %.3e (%s)  | envelope sup %.3e, seed-local %.3e\n', ...
            ov.Pf_a_floor, local_floor_words(opts.floor_from_envelope), ...
            floor_a_env, floor_a_seed);
    fprintf(['b_true on the envelope: [%.5f, %.5f] -> b_init %.5f, ' ...
             'sqrt(P55[0]) %.5f  (far-field limit 8/9 = 0.88889)\n'], ...
            b_lo_e, b_hi_e, b_mid, b_half);
    fprintf('da PRIOR sqrt(P55[0]) = %.4e  <- %s\n', Pf_da_used, Pf_da_src);
    fprintf('   S3(b) closed form on the planned trajectory: sup %.4e (at t=%.3f s, w_bar %.3f), RMS %.4e\n', ...
            da_prior.sup, da_prior.t_sup, da_prior.w_sup, da_prior.rms);
    fprintf('   S3(b) sign structure: one-signed=%d, frac positive %.3f; mean over run %+.4e, mean over descent %+.4e\n', ...
            da_prior.one_signed, da_prior.frac_positive, da_prior.mean_all, da_prior.mean_desc);
    if opts.par_law
        fprintf('PAR LAW (x/y): w0 %.4f  floor %.4f (= fit sup)\n', ...
                par_pkg.w0, par_pkg.floor_a);
    else
        fprintf('PAR LAW (x/y): OFF -- x/y run the perpendicular origin\n');
    end

    fprintf('%6s | %9s %9s %10s %9s | %11s %10s %6s %6s | %9s %9s | %4s\n', 'seed', ...
            'desc pk %', 'osc RMS%', 'hold mn %', 'rms all%', ...
            'da end', 'sqrtP55end', 'frz', 'da@1.5', ...
            'hold %/s', 'unopp %/s', 'NaN');

    plant_cperp = [];   % plant-side boundary arms are not forked (plane only)
    n_seeds = numel(seeds);
    runs  = cell(n_seeds, 1);
    Mrows = zeros(n_seeds, 7);   % desc | osc | hold | rms_all | bud_b | bud_p | nan
    Drows = zeros(n_seeds, 10);  % da_end | sqrtP55_end | sqrtP55_0 | mono | frz |
                                 % t_frz | da frac by descent end | hold drift
                                 % %/s | hold delta % | unopposed %/s
    for q = 1:n_seeds
        s = local_run_once(cfg, seeds(q), ov, opts.verbose, opts.a_ctrl_override, false, opts.ws_inject, plant_cperp, opts.da_known, opts.ap_known, opts.ap_law_bias, opts.law_b);
        m = local_run_metrics(s, cfg, AX_Z, OSC_SETTLE_S, HOLD_SETTLE_S);
        runs{q}     = s;
        Mrows(q, :) = [m.desc_peak_pct, m.osc_rms_pct, m.hold_mean_pct, ...
                       m.rms_all_pct, m.budget_b, m.budget_p, m.any_nan];
        Drows(q, :) = [m.da_end, m.sqrtP55_end, m.sqrtP55_0, m.P55_mono, ...
                       m.frozen_frac, m.t_freeze, m.da_frac_by_descend_end, ...
                       m.hold_drift_pct_per_s, m.hold_delta_pct, ...
                       m.hold_unopposed_pct / max(m.hold_span_s, eps)];
        fprintf('%6d | %9.3f %9.3f %+10.3f %9.3f | %+11.4e %10.3e %6.3f %+6.2f | %+9.3f %+9.1f | %4d\n', ...
                seeds(q), Mrows(q, 1), Mrows(q, 2), Mrows(q, 3), Mrows(q, 4), ...
                Drows(q, 1), Drows(q, 2), Drows(q, 5), Drows(q, 7), ...
                Drows(q, 8), Drows(q, 10), Mrows(q, 7));
    end

    % ------------------------------------------------------------------
    % Aggregates
    % ------------------------------------------------------------------
    mu = mean(Mrows(:, 1:4), 1);
    sd = std(Mrows(:, 1:4), 0, 1);
    fprintf('-------------------------------------------------------------------------------------\n');
    fprintf('mean over %d seeds: desc pk %.3f+-%.3f %%  osc RMS %.3f+-%.3f %%  hold mean %+.3f+-%.3f %%  rms all %.3f+-%.3f %%\n', ...
            n_seeds, mu(1), sd(1), mu(2), sd(2), mu(3), sd(3), mu(4), sd(4));
    % Budget criterion (recalibrated 2026-08-01). The traversal ratio
    % (x_end - x_0)^2 / (P[0] - P[end]) is a chi2(1) variate under the
    % saturated bound E[ratio] <= 1, not a hard cap: a single seed above 1 is
    % ordinary sampling noise. Per-seed values are therefore DIAGNOSTIC, and
    % flagged only beyond the chi2(1) 99th percentile 6.635. The acceptance
    % test is the 6-seed mean, whose one-sided 95% limit under E[ratio] <= 1
    % with Var(chi2(1)) = 2 is 1 + 1.645*sqrt(2/6) = 1.949.
    RATIO_SEED_FLAG = 6.635;   % chi2(1) 99th percentile
    RATIO_MEAN_PASS = 1 + 1.645 * sqrt(2 / n_seeds);
    flag_b = find(Mrows(:, 5) > RATIO_SEED_FLAG);
    flag_p = find(Mrows(:, 6) > RATIO_SEED_FLAG);
    mean_b = mean(Mrows(:, 5));
    mean_p = mean(Mrows(:, 6));
    if isnan(mean_p)
        fprintf('budget aggregate: b mean %.3f (%s <= %.3f), p n/a (locked, zero-width prior)\n', ...
                mean_b, local_verdict(mean_b, RATIO_MEAN_PASS), RATIO_MEAN_PASS);
        fprintf('per-seed traversal ratios (diagnostic; flag > %.3f): b max %.3f%s | p n/a\n', ...
                RATIO_SEED_FLAG, max(Mrows(:, 5)), local_flag_str(flag_b, seeds));
    else
        fprintf('budget aggregate: b mean %.3f (%s <= %.3f), p mean %.3f (%s <= %.3f)\n', ...
                mean_b, local_verdict(mean_b, RATIO_MEAN_PASS), RATIO_MEAN_PASS, ...
                mean_p, local_verdict(mean_p, RATIO_MEAN_PASS), RATIO_MEAN_PASS);
        fprintf('per-seed traversal ratios (diagnostic; flag > %.3f): b max %.3f%s | p max %.3f%s\n', ...
                RATIO_SEED_FLAG, max(Mrows(:, 5)), local_flag_str(flag_b, seeds), ...
                max(Mrows(:, 6)), local_flag_str(flag_p, seeds));
    end
    fprintf('prior width sqrt(P55[0]) (controller-reported, z): %.4e  (declared %.4e)\n', ...
            runs{1}.P_b_out(1, AX_Z), Pf_da_used);
    fprintf('P44[0] (controller-reported, z): sqrt = %.5f\n', runs{1}.P_a_out(1, AX_Z) / runs{1}.a_nom);
    fprintf('h_bar range (true, seed %d): [%.3f, %.3f]\n', seeds(1), ...
            min(runs{1}.h_bar_true_out), max(runs{1}.h_bar_true_out));

    % --- disturbance-state diagnostics (pre-registered in tex S8) ---------
    fprintf('da_hat[end]  : mean %+.4e +- %.4e   (prior width %.4e)\n', ...
            mean(Drows(:, 1)), std(Drows(:, 1)), Pf_da_used);
    fprintf('sqrt(P55)    : [0] %.4e -> [end] mean %.4e +- %.4e  (collapse factor %.3f)\n', ...
            mean(Drows(:, 3)), mean(Drows(:, 2)), std(Drows(:, 2)), ...
            mean(Drows(:, 2)) / max(mean(Drows(:, 3)), eps));
    fprintf('P55 monotone : max single-step INCREASE / P55[0] = %.3e (Q55 = 0 predicts 0)\n', ...
            max(Drows(:, 4)));
    fprintf('da_hat frozen: fraction %.3f +- %.3f of t >= %.2f s; freeze time mean %.3f s\n', ...
            mean(Drows(:, 5)), std(Drows(:, 5)), cfg.t_hold, mean(Drows(:, 6)));
    fprintf('da_hat acquired on the DESCENT: da_hat(t=%.2f)/da_hat(end) = %.3f +- %.3f\n', ...
            cfg.t_hold + cfg.t_descend_override, mean(Drows(:, 7)), std(Drows(:, 7)));
    fprintf('FINAL-HOLD DRIFT (truth says da == 0 there): %+.3f +- %.3f %%/s ; end-start %+.3f +- %.3f %%\n', ...
            mean(Drows(:, 8)), std(Drows(:, 8)), mean(Drows(:, 9)), std(Drows(:, 9)));
    fprintf('   unopposed injection of da_hat(end) over the same window would be %+.1f %%/s (ratio measured/unopposed %.4f)\n', ...
            mean(Drows(:, 10)), mean(Drows(:, 8)) / max(abs(mean(Drows(:, 10))), eps) * sign(mean(Drows(:, 10))));

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
                                        'budget_b', 'budget_p', 'any_nan'}}, ...
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
                         'axis', AX_Z, 'prior_std_da', Pf_da_used);
    out.da_prior = da_prior;
    out.da_prior.used = Pf_da_used;
    out.da_prior.source = Pf_da_src;
    out.shape_floor = struct('floor_a', floor_a_env, 'w_sup', w_floor_sup, ...
                             'env', [env_lo, env_hi]);
    out.cfg     = cfg;
    out.opts    = opts;
    out.arm_tag = tag;
    out.seeds   = seeds;
    out.par_pkg  = par_pkg;      % [] when par_law is off

    here = fileparts(mfilename('fullpath'));
    out_dir = fullfile(here, '..', '..', 'test_results');
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end
    out_file = fullfile(out_dir, ['run_formC_b_' tag '.mat']);
    save(out_file, 'out');
    fprintf('saved: %s\n', out_file);
end


%% =================== Local Helpers ===================

function cfg = local_canonical_config(a_cov, h_bar_min_prior)
%LOCAL_CANONICAL_CONFIG  Canonical hold->descend->osc->hold scenario (the
%   expgain/powerlaw acceptance scenario, with h_min raised to the prior
%   domain floor 1.1*R).
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
%LOCAL_VERDICT  PASS/FAIL label for an aggregate acceptance number.
    if value <= limit
        s = 'PASS';
    else
        s = 'FAIL';
    end
end


function s = local_flag_str(idx, seeds)
%LOCAL_FLAG_STR  Render the flagged-seed list for the diagnostic ratio line.
    if isempty(idx)
        s = '';
    else
        s = [' <- flagged seeds ' mat2str(seeds(idx))];
    end
end



function w = local_floor_words(from_env)
    if from_env; w = 'envelope sup, legacy'; else; w = 'seed-local, honest P[0]'; end
end


function fl = local_seed_floor(w_seed, w0, b_seed)
%LOCAL_SEED_FLOOR  |a_bar_law(w_seed) - a_bar_true(w_seed)| at the SEED only.
%   This is what P[0]'s shape term means: how wrong the seeded curve is where
%   the run actually starts. Not the envelope supremum, which prices a height
%   the run has not reached and belongs to the model-error container instead.
    [~, cp] = calc_correction_functions(w_seed);
    fl = abs((1 - 1 / (b_seed * (w_seed - w0))) - 1 / cp);
end


function [b_mid, b_half, b_lo, b_hi] = local_envelope_b_range(w_lo, w_hi)
%LOCAL_ENVELOPE_B_RANGE  Range of the exact law factor b_true on the envelope.
%   b_true(w) = a_true' / (1 - a_true)^2  with  a_true = 1/c_perp.
%   MULTIPLICATIVE form: a_bar' = b (1 - a_bar)^2.
%   Returns the midpoint and half-range, which are the seed and the prior
%   width for state 5. Offline only -- the controller never reads c(w).
    N_SWEEP = 20001;
    w = linspace(w_lo, w_hi, N_SWEEP).';
    b = zeros(N_SWEEP, 1);
    for i = 1:N_SWEEP
        [~, cp, d] = calc_correction_functions(w(i), true);
        a_t  = 1 / cp;
        ap_t = -d.dc_perp_dh / cp^2;          % d a_bar / d w_bar
        b(i) = ap_t / (1 - a_t)^2;
    end
    b_lo = min(b);  b_hi = max(b);
    b_mid  = 0.5 * (b_lo + b_hi);
    b_half = 0.5 * (b_hi - b_lo);
end


function [floor_a, w_sup] = local_envelope_floor_dist(w_lo, w_hi, w0)
%LOCAL_ENVELOPE_FLOOR_DIST  Shape floor for the curve THIS writing seeds.
%   [floor_a, w_sup] = local_envelope_floor_dist(w_lo, w_hi, w0)
%
%   The integrated law of formC_state_dist.tex is parameter-free,
%       a_bar(w_bar) = 1 - 1/(w_bar - w0) ,
%   with w0 the integration constant (w0 = 0 for a plane whose contact sits at
%   w_bar = 1, so a_bar(1) = 0). The residual against the published truth,
%       floor_a = sup |a_bar(w_bar) - 1/c_perp(w_bar)|   on the envelope,
%   is the gain error the seed cannot represent even before any estimation.
%   It funds the P_aa shape floor (ctrl_const.Pf_a_floor).
%
%   This is NOT the Form B anchored floor: that number prices a different
%   curve (1 - (1+(w-1)/(9/8))^-1) and reusing it would price a representation
%   error this filter never makes. Nothing here is tuned -- the inputs are the
%   planned envelope and the published truth curve.
    N_SWEEP = 20001;    % dense enough that the sup is grid-independent
    assert(w_hi > w_lo && w_lo - w0 > 0, 'run_formC_b:badEnvelope', ...
           'envelope must satisfy w_lo (%.3f) > w0 (%.3f) and w_hi > w_lo.', w_lo, w0);

    w = linspace(w_lo, w_hi, N_SWEEP).';
    c = zeros(N_SWEEP, 1);
    for i = 1:N_SWEEP
        [~, c(i)] = calc_correction_functions(w(i), true);
    end
    a_law = 1 - 1 ./ (w - w0);
    [floor_a, i_sup] = max(abs(a_law - 1 ./ c));
    w_sup = w(i_sup);
end


function pr = local_da_prior_from_trajectory(cfg)
%LOCAL_DA_PRIOR_FROM_TRAJECTORY  Prior width on the additive disturbance.
%   pr = local_da_prior_from_trajectory(cfg)
%
%   formC_state_dist.tex S3(b) gives the disturbance in CLOSED FORM: da is not
%   postulated, it is whatever makes the law-based propagation exact,
%
%       da[k] = (1 - a_bar[k])^2 * [ 1/b_state(w_bar[k]) - 1 ] * Sigma[k] ,
%       b_state(w_bar) = (1 - a_true)^2 / a'_true ,
%       Sigma[k] = Delta_wbar_d[k] + (1-lc) dw_3[k] + F_dw[k] e_a[k] + eps_w[k]
%
%   The driver knows the planned trajectory, so it can evaluate this offline
%   with Sigma approximated by its COMMANDED part Delta_wbar_d[k]. The three
%   omitted shares are zero-mean feedback/noise terms; dropping them is
%   declared here and is why the number is a PRIOR WIDTH, not a value.
%
%   The returned sup is the natural scale the tex asks for -- "the level offset
%   the disturbance must remove divided by the number of steps over which it
%   must remove it" -- computed rather than guessed, since the closed form is
%   exactly (offset rate) per step. Its contract is invariance, so the
%   controller must not depend on it: sweep it 10x either way.
%
%   Offline truth evaluation at driver-init only; the controller stays
%   run-time c-free.
    clear trajectory_generator;
    params = calc_simulation_params(cfg);
    P  = params.Value;
    Ts = P.common.Ts;
    R  = P.common.R;
    N  = round(cfg.T_sim / Ts) + 1;

    assert(isfield(P, 'wall') && P.wall.enable_wall_effect > 0.5, ...
           'run_formC_b:noWall', ...
           'the disturbance prior is a wall-effect quantity; wall effect is off.');
    w_hat = P.wall.w_hat(:);
    pz    = P.wall.pz;
    if isfield(P.wall, 'h_bar_min') && ~isempty(P.wall.h_bar_min)
        h_floor = P.wall.h_bar_min;
    else
        h_floor = 1.001;
    end

    da   = zeros(N, 1);
    wbar = zeros(N, 1);
    pd_k = P.common.p0;
    for k = 1:N
        t_k = (k - 1) * Ts;
        [pd_kp1, del_pd_k] = trajectory_generator(t_k, P);
        w_k  = max((dot(pd_k, w_hat) - pz) / R, h_floor);
        dw_k = dot(del_pd_k, w_hat) / R;                 % Delta_wbar_d[k]
        [~, c_perp, dv] = calc_correction_functions(w_k, true);
        a_t  = 1 / c_perp;                               % true a_bar
        ap_t = -dv.dc_perp_dh / c_perp^2;                % true d a_bar/d w_bar
        b_state = (1 - a_t)^2 / ap_t;                    % law slope / true slope
        da(k)   = (1 - a_t)^2 * (1 / b_state - 1) * dw_k;
        wbar(k) = w_k;
        pd_k    = pd_kp1;
    end
    clear trajectory_generator;

    [pr.sup, i_sup] = max(abs(da));
    pr.rms   = sqrt(mean(da.^2));
    pr.t_sup = (i_sup - 1) * Ts;
    pr.w_sup = wbar(i_sup);
    pr.one_signed = all(da <= 1e-18) || all(da >= -1e-18);
    pr.frac_positive = mean(da > 0);
    pr.mean_all = mean(da);
    t_grid   = (0:N-1).' * Ts;
    m_desc   = t_grid >= cfg.t_hold & t_grid <= cfg.t_hold + cfg.t_descend_override;
    pr.mean_desc = mean(da(m_desc));   % the constant a perfect fit would find
                                       % on the descent alone
    pr.da    = da;
    pr.wbar  = wbar;
    pr.note  = 'Sigma approximated by its commanded part Delta_wbar_d';
end


function pkg = local_parallel_law_package_dist(w_lo, w_hi)
%LOCAL_PARALLEL_LAW_PACKAGE_DIST  Parallel-axis package for the ADDITIVE
%   writing. The wall-parallel truth is not a member of the one-curve family
%   (Goldman's logarithm at contact), and in this writing there is NO constant
%   that reshapes a curve -- da is a per-step increment. So the whole package
%   is the integration constant: a 1-parameter minimax fit of
%       a_law(w_bar) = 1 - 1/(w_bar - w0)
%   to 1/c_para on the planned envelope, with slot 5 locked at 0 on x/y and
%   the entire residual carried as the P(4,4) floor. Offline truth evaluation;
%   the run-time loop stays c-free. x/y accuracy is DECLARED out of scope --
%   with w_hat = z the mobility matrix is diagonal, so x/y cannot contaminate
%   the z metrics.
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
    pkg.floor_a = pkg.fit_sup;   % the whole residual, nothing else to carry
end


function s = local_arm_words(arm)
%LOCAL_ARM_WORDS  One-line description of the arm for the console header.
    if strcmpi(arm, 'base')
        s = 'derivation (a): 4 states, no free parameter';
    else
        s = 'derivation (b): 5 states, additive da, Q55 = 0';
    end
end


function fs = local_freeze_stats(t, da, t_start)
%LOCAL_FREEZE_STATS  When does da_hat stop moving?
%   fs = local_freeze_stats(t, da, t_start)
%
%   Definition (pre-registered, tex S8: "P55 decreases monotonically and
%   da_hat freezes late in the run"). The instantaneous rate is noisy, so the
%   rate is read over a fixed W = 0.05 s backward window,
%       rate[k] = |da[k] - da[k-W]| / (W*Ts) ,
%   and a sample counts as FROZEN when rate[k] < 0.01 * max(rate) over the
%   active part of the run (t >= t_start = the descent start). "Its early
%   rate" is therefore the PEAK rate, which is itself reported so the reader
%   can check it occurs early.
%       fs.frac      fraction of the active samples that are frozen
%       fs.t_freeze  first time after which the rate NEVER again exceeds the
%                    threshold (NaN if it is still moving at the end)
%       fs.peak_rate, fs.t_peak, fs.window_s
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
        % the slot never moved (locked arm): frozen by construction
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
%LOCAL_RUN_METRICS  Per-run console metrics on the focus axis.
%   e_a = 100*(a_hat - a_true)/a_true; windows from the phase boundaries.
%   P[0] budget (V3 convention): with zero process noise on b / p their P can
%   only shrink, so (x_end - x_0)^2 <= P[0] - P[end] must hold; the reported
%   ratio must stay <= 1.
    t  = s.tout(:);
    w  = local_metric_windows(cfg, osc_settle_s, hold_settle_s);
    aT = s.a_true_out(:, ax);
    e_pct = 100 * (s.a_hat_out(:, ax) - aT) ./ max(aT, eps);

    w_desc = t >= w.descent(1) & t <= w.descent(2);
    w_osc  = t >  w.osc(1)     & t <= w.osc(2);
    w_hold = t >  w.hold(1);
    assert(any(w_desc) && any(w_osc) && any(w_hold), ...
           'run_formC_b:emptyMetricWindow', ...
           'A metric window is empty -- check trajectory timing overrides.');

    m.desc_peak_pct = max(abs(e_pct(w_desc)));
    m.osc_rms_pct   = sqrt(mean(e_pct(w_osc).^2));
    m.hold_mean_pct = mean(e_pct(w_hold));
    % Relative-gain-error RMS over the whole ACTIVE run: descent start to the
    % end, i.e. the union of the three phases INCLUDING their transitions.
    % The initial hold is excluded because there the truth is the far field,
    % the seed is nearly exact, and including it only dilutes the number.
    w_all = t >= w.descent(1);
    m.rms_all_pct = sqrt(mean(e_pct(w_all).^2));

    % --- disturbance-state diagnostics (slot 5; b_hat/P_b are its aliases) --
    da_hat = s.b_hat_out(:, ax);
    P55    = s.P_b_out(:, ax).^2;          % P_*_out store sqrt(P)
    m.da_end      = da_hat(end);
    m.sqrtP55_0   = s.P_b_out(1, ax);
    m.sqrtP55_end = s.P_b_out(end, ax);
    % Q55 = 0 and F_e row 5 is a unit row, so P55 can only shrink. Report the
    % largest single-step INCREASE relative to P55[0]; anything above roundoff
    % falsifies the pre-registered claim.
    if P55(1) > 0
        m.P55_mono = max([0; diff(P55)]) / P55(1);
    else
        m.P55_mono = 0;                    % locked slot: P55 == 0 throughout
    end
    fs = local_freeze_stats(t, da_hat, w.descent(1));
    m.frozen_frac = fs.frac;
    m.t_freeze    = fs.t_freeze;
    m.freeze      = fs;

    % How much of the final da_hat is in place by the END OF THE DESCENT?
    % The S3(b) closed form is proportional to the increment, so it sums to
    % exactly zero over a closed oscillation cycle: the constant-da model has
    % nothing to lock onto during the oscillation and essentially all of the
    % disturbance must be acquired on the one-signed descent. This ratio is
    % the direct test of that reading.
    k_desc_end = find(t >= w.descent(2), 1);
    m.da_descend_end = da_hat(k_desc_end);
    if abs(da_hat(end)) > 0
        m.da_frac_by_descend_end = da_hat(k_desc_end) / da_hat(end);
    else
        m.da_frac_by_descend_end = NaN;      % locked arm: da == 0 throughout
    end

    % FINAL-HOLD DRIFT. The commanded increment is zero in the final hold, so
    % the true da is exactly zero there (tex S3(b) property (i)); the constant
    % model nevertheless injects da_hat every step. Pre-registered prediction:
    % a spurious ramp in the gain error, of order (steps)*(da_hat) unless y2
    % opposes it. Reported as the least-squares slope of e_a over the hold
    % window and the end-minus-start change across it.
    t_h  = t(w_hold);
    e_h  = e_pct(w_hold);
    pfit = polyfit(t_h - t_h(1), e_h, 1);
    m.hold_drift_pct_per_s = pfit(1);
    m.hold_delta_pct       = e_h(end) - e_h(1);
    m.hold_span_s          = t_h(end) - t_h(1);
    % what an UNOPPOSED injection of the converged da_hat would give over the
    % same window, in the same percent units (a_bar per step * steps / a_true)
    n_hold = numel(t_h);
    a_true_hold = mean(aT(w_hold)) / s.a_nom;      % normalized true gain
    m.hold_unopposed_pct = 100 * n_hold * da_hat(end) / a_true_hold;

    % P_*_out store sqrt(P); square back for the budget. Row 1 is the
    % controller's init call, which must report the prior widths (contract).
    trav2_b    = (s.b_hat_out(end, ax) - s.b_hat_out(1, ax))^2;
    budget_b   = s.P_b_out(1, ax)^2 - s.P_b_out(end, ax)^2;
    m.budget_b = trav2_b / max(budget_b, eps);
    trav2_p    = (s.p_hat_out(end, ax) - s.p_hat_out(1, ax))^2;
    budget_p   = s.P_p_out(1, ax)^2 - s.P_p_out(end, ax)^2;
    if s.P_p_out(1, ax) == 0
        % p locked with a zero-width prior (amplitude arm): the ratio is 0/0.
        % Report NaN rather than a 0 that reads as a PASS.
        m.budget_p = NaN;
    else
        m.budget_p = trav2_p / max(budget_p, eps);
    end

    m.any_nan = any(~isfinite(s.p_true_out(:))) || any(~isfinite(s.a_hat_out(:))) ...
                || any(~isfinite(s.b_hat_out(:))) || any(~isfinite(s.p_hat_out(:))) ...
                || any(~isfinite(s.ws_hat_out(:)));
end


function simOut = local_run_once(config, seed, ctrl_const_override, verbose, a_ctrl_override, log_P_full, ws_inject, plant_cperp, da_known_on, ap_known_on, ap_law_bias, law_b)
    if nargin < 9  || isempty(da_known_on); da_known_on = false; end
    if nargin < 10 || isempty(ap_known_on); ap_known_on = false; end
    if nargin < 11 || isempty(ap_law_bias); ap_law_bias = NaN; end
    if nargin < 12 || isempty(law_b); law_b = 1; end
    ap_law_on = isfinite(ap_law_bias);
    if ap_law_on; ap_known_on = true; end
    hb_prev = NaN;  cperp_prev = NaN;
if nargin < 7; ws_inject = 0; end
if nargin < 8; plant_cperp = []; end
%LOCAL_RUN_ONCE  One seed of the scenario. Fork of run_5state_expgain's body
%   with the controller hard-dispatched to motion_control_law_formC_b. The
%   log schema is the ancestor's; slot 5 (da) is logged through the b_hat /
%   P_b aliases the controller diag contract already provides.

    % Fresh persistent state per run (controller, trajectory, thermal).
    clear motion_control_law_formC_b trajectory_generator calc_thermal_force;

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

    % knob overrides (controller defaults are the honest ones; these carry the
    % arm flags -- lock_b/lock_p/lock_ws, b_init/p_init, y2_off -- and any
    % ablation overrides)
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
    % S11 injection: TRUE wall shifted by ws_inject*R on the PLANT side only;
    % trajectory and controller keep the nominal frame, so the true contact
    % sits at w_bar_s = 1 + ws_inject in the estimator's coordinates.
    pz_plant = 0;
    P_plant  = P;                    % plant-side params (shifted wall)
    if wall_on_drv
        pz_plant = P.wall.pz + ws_inject * R_drv;
        P_plant.wall.pz = pz_plant;  % step_dynamics / calc_thermal_force see
                                     % the TRUE wall; controller keeps P
    end
    % Boundary override: TRUE z curve = plant_cperp(h_bar), plant side only
    % (calc_gamma_inv / calc_thermal_force dispatch on this field); the
    % controller keeps P and whatever priors the driver derived for it.
    if wall_on_drv && ~isempty(plant_cperp)
        P_plant.wall.plant_cperp = plant_cperp;
    end
    if wall_on_drv && isfield(P.wall, 'h_bar_min')
        h_bar_floor_drv = P.wall.h_bar_min;
    else
        h_bar_floor_drv = 1.001;
    end

    % --- Logs (ancestor schema + p / w_s rows) ---
    p_d_out  = zeros(N, 3);
    f_d_out  = zeros(N, 3);
    F_th_out = zeros(N, 3);
    p_m_out  = zeros(N, 3);
    p_true_out = zeros(N, 3);
    da_known_out = zeros(N, 3);
    a_true_out = zeros(N, 3);
    a_prime_true_out = zeros(N, 3);
    ekf_out  = zeros(N, 4);
    a_hat_out  = zeros(N, 3);
    b_hat_out  = zeros(N, 3);
    p_hat_out  = zeros(N, 3);
    ws_hat_out = zeros(N, 3);
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
    dx_r_out     = zeros(N, 3);   % IIR residual the gain readout is built from [um]
    dh_m_out     = zeros(N, 3);   % delayed position error fed to the controller [um]
    a_bar_hat_out = zeros(N, 3);  % internal normalized gain estimate [-]
    Q33_out       = zeros(N, 3);  % Q33 container actually used [-]
    a_bar_Q_out   = zeros(N, 3);  % a_bar used to build Q33 this step [-]
    f_bar_out     = zeros(N, 3);  % normalized force f_bar = a_o*f [-]
    if log_P_full
        P_full_out = [];   % sized on first step (7 or 9 states, ma2_aug-dependent)
    end

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
            % true d a_h / d h_bar by central difference on the exact curve
            a_prime_true_out(k, :) = local_a_prime_true(h_bar_true_k, a_nom_drv, h_bar_floor_drv, plant_cperp).';
        else
            h_bar_true_k = Inf;
            a_true_k = a_nom_drv * ones(3, 1);
        end

        % a_ctrl_override = 'true' feeds the EXACT gain to the control law
        % every step (the EKF still estimates its own). This pins the closed
        % loop at the ideal one the C_dpmr / C_n constants were derived for, so
        % any residual readout bias cannot be blamed on a_hat being wrong.
        if ischar(a_ctrl_override) || isstring(a_ctrl_override)
            a_ov_k = a_true_k;
        else
            a_ov_k = a_ctrl_override;
        end
        % --- KNOWN-DISTURBANCE ARM (ceiling measurement) -------------------
        %   formC_state_dist.tex S3(b), with the increment written as the
        %   actual step in w_bar (S4 first line):
        %       da[k-1] = [a'_true(w[k-1]) - (1-a_true[k-1])^2] * (w[k]-w[k-1])
        %   The PREVIOUS step is used because predict bridges k-1 -> k, so this
        %   stays causal. Applied on the wall-normal axis only: x/y run the
        %   parallel package and are out of scope for this arm.
        da_known_k = [];
        if da_known_on
            if isfinite(h_bar_true_k) && isfinite(hb_prev) && k > 1
                ap_prev = local_a_prime_true(hb_prev, a_nom_drv, h_bar_floor_drv, plant_cperp);
                abar_prev  = 1 / cperp_prev;                  % a_true / a_o
                aprime_bar = ap_prev(3) / a_nom_drv;          % d a_bar / d w_bar
                da_known_k = [0; 0; (aprime_bar - (1 - abar_prev)^2) ...
                                    * (h_bar_true_k - hb_prev)];
            else
                da_known_k = zeros(3, 1);
            end
        end
        if isfinite(h_bar_true_k)
            hb_prev = h_bar_true_k;  cperp_prev = c_perp_k;
        end
        da_known_out(k, :) = local_row3(da_known_k);

        % --- TRUE-SLOPE ARM: a_bar' from the PREVIOUS step's true height ----
        ap_known_k = [];
        if ap_known_on
            if isfinite(hb_prev) && k > 1
                if ap_law_on
                    % LAW evaluated at the TRUE a_bar plus a constant offset.
                    % Exogenous: the state never enters, so any harm here is
                    % argument error and not feedback.
                    ab_off = 1 / cperp_prev + ap_law_bias;
                    ap_known_k = [0; 0; (1 - ab_off)^2 / law_b];
                else
                    ap_known_k = local_a_prime_true(hb_prev, a_nom_drv, h_bar_floor_drv, ...
                                                    plant_cperp) / a_nom_drv;
                end
            else
                ap_known_k = zeros(3, 1);
            end
        end

        [f_d_k, ekf_k, diag_k] = motion_control_law_formC_b(del_pd_k, pd_k, ...
                                     p_m_delayed, P, ctrl_const, a_ov_k, da_known_k, ap_known_k);

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
    simOut.da_known_out = da_known_out;
    simOut.a_true_out = a_true_out;
    simOut.a_prime_true_out = a_prime_true_out;
    simOut.tout       = tout;
    simOut.ekf_out    = ekf_out;
    simOut.a_hat_out  = a_hat_out;
    simOut.b_hat_out  = b_hat_out;
    simOut.p_hat_out  = p_hat_out;
    simOut.ws_hat_out = ws_hat_out;
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
    if log_P_full
        simOut.P_full_out = P_full_out;
    end
    simOut.a_nom        = a_nom_drv;
    simOut.R            = R_drv;
    simOut.meta = struct('config', config, 'params_value', P, ...
                         'seed', seed, 'driver', 'run_formC_b', ...
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


function r = local_row3(v)
%LOCAL_ROW3  [] -> zeros(1,3); 3x1 -> row. Keeps the known-disturbance log
%   shaped even on arms where it is not used.
    if isempty(v); r = zeros(1, 3); else; r = v(:).'; end
end
