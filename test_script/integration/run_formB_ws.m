% STATUS: ACTIVE (Form B unknown-wall gain-law driver, tier-1 ladder) -- spec formB_ws.tex S1-S8 + formB_ws_ref.tex S9-S11; controller model/controller/motion_control_law_formB_ws.m
function out = run_formB_ws(opts, test_opts)
%RUN_FORMB_WS  Canonical-scenario driver for the 7-state Form B (unknown wall
%   position) gain-law controller (motion_control_law_formB_ws).
%
%   out = run_formB_ws()          % tier-1 production arm, 6 seeds
%   out = run_formB_ws(opts)
%
%   FORK OF run_5state_expgain.m: the per-step ordering (RNG seed -> params ->
%   trajectory -> controller -> thermal -> step_dynamics -> sensor-delay
%   buffer), the config / ctrl_const plumbing, and the ToWorkspace log schema
%   are inherited verbatim. This driver additionally owns the canonical
%   scenario, the 6-seed loop and the console metrics, because the Form B
%   acceptance arms (formB_ws_ref.tex S11) are defined per scenario, not per
%   call.
%
%   Authoritative specs:
%       reference/eq17_analysis/derivation/formB_ws.tex      (S1-S8 equations)
%       reference/eq17_analysis/derivation/formB_ws_ref.tex  (conventions,
%           declared truncations, S9-S11 contracts)
%   State (7 per axis): [dw1 dw2 dw3 a_w b p w_s], parameter order b, p, w_s.
%   Plant truth comes from calc_correction_functions (c_perp on z); the
%   filter is run-time c-free by construction (charter constraint 3).
%
%   SCENARIO (canonical, z-axis / perpendicular focus):
%       hold 0.5 s @ h = 50 um (h_bar 22.2) -> descend 1.0 s ->
%       1 Hz oscillation, amplitude 2.5 um, trough h = 4.5 um (h_bar 2.0),
%       2 cycles -> final hold at the trough; T = 4.8 s; thermal +
%       measurement noise ON; lambda_c = 0.7; 6-seed house convention.
%       h_min = 1.1*R is ENFORCED: below it the published truth curve is
%       extrapolated, and the envelope priors are read off that curve.
%
%   PRIORS (envelope-derived, 2026-08-01 user decision B): the driver knows
%   the heights the run will visit, so it derives (b, p) prior widths and the
%   P_aa shape floor from THAT envelope via local_envelope_priors, and passes
%   them as ctrl_const.Pf_b_std / .Pf_p_std / .Pf_a_floor. The controller
%   defaults (1/8, 1/8, 0.0066) remain the global-sup fallback on [1.1, 10]
%   for callers that do not know their envelope. The derived triple is
%   scenario-dependent by construction and is printed, never asserted.
%   Pf_ws_std stays 0.111 (calibration-based, not a theta_eff sup).
%
%   PARALLEL-AXIS LAW (opts.par_law, default TRUE, 2026-08-02): the x and y
%   filters run their own Form B law instead of the perpendicular one. The
%   parallel truth c_para is NOT representable by the anchored (b, p) = (9/8, 1)
%   law -- its Goldman logarithm at contact pushes the best Form B
%   representative's origin INSIDE the wall -- so the driver derives the
%   parallel package (b_par, p_par, ws0_par) at run time by a 3-parameter
%   minimax fit of the law to 1/c_para on the SAME planned envelope the (b, p)
%   priors come from (local_parallel_law_package). Offline truth evaluation,
%   zero tuning, and the controller stays run-time c-free. The x/y law origin
%   then rides the z-axis w_s estimate (one-way feed), and slots 5-7 of the x/y
%   filters are locked at the fitted constants, so their whole prior mass is
%   the representation floor Pf_a_floor_par on P(4,4). par_law = false is the
%   legacy arm (all three axes on the perpendicular law).
%
%   opts fields (defaults first):
%       .tier        't1'    't1' = lock_ws true (w_s pinned at 1, first-test
%                            ladder decision 2026-07-31); 't2' = w_s released
%       .par_law     true    x/y run the derived parallel law (see above);
%                            false = legacy all-perpendicular arm ('_nopar' tag)
%       .y2_on       true    false = drop the gain-readout channel
%                            (defect-1 fingerprint arm, S11 item 1: adding y2
%                            must IMPROVE a_hat tracking)
%       .oracle_bp   []      [1.0354 0.9500] = lock (b, p) at the minimax
%                            pair -> the shape-floor reference line
%       .wrong_seed  0       1 = offset the b / p seeds by one prior width
%                            (+1/8 each): honest-init recovery arm
%       .a_cov_scale 1       multiplies the 0.05 base a_cov (invariance sweep)
%       .law_form    'len'   'len' = production length writing.
%                            'amp' = AMPLITUDE writing probe (2026-08-09):
%                            the law origin is tied to the length constant,
%                            w_s = b, so at p = 1 the law is a_bar = 1 - b/w
%                            and slot 5 is the ONLY free parameter, with
%                            J_beta = 1/w^2 (no null; the production J_b
%                            vanishes at w_bar = 1 + b_hat, inside the
%                            canonical oscillation band). Forces
%                            lock_p = lock_ws = true, p_init = 1, and swaps
%                            BOTH P0 numbers to the amplitude read-off
%                            (sqrt(P_bb) 0.0708, shape floor 0.0372, against
%                            the length arm's 0.0157 / 0.00284). Tag '_amp'.
%                            Refuses oracle_bp / ctrl_cell_Rc / tier t2.
%                            Derivation: derivation/formB_amp_bonly_probe.tex
%       .seeds       [7 11 23 42 101 777]    house 6-seed convention
%       .verbose     false   per-run progress printout
%       .config_override     struct merged into the canonical config (window
%                            asserts still apply)
%       .ctrl_const_override struct merged into ctrl_const LAST, i.e. it wins
%                            over the arm flags (ablation hook, ancestor style)
%       .a_ctrl_override     [] | 3x1 [um/pN] | 'true' = feed the exact gain
%                            to the CONTROL LAW only; the EKF still estimates
%
%   Console metrics per run (z axis, e_a = 100*(a_hat - a_true)/a_true):
%       descent peak |e_a|, oscillation RMS e_a, final-hold mean e_a, and the
%       P[0]-budget ratios (x_end - x_0)^2 / (P[0] - P[end]) for b and p
%       (Q_bb = Q_pp = 0 and F_e rows 5/6 are unit rows, so P can only shrink
%       and the ratio must stay <= 1). Printed only; test_results/ is
%       gitignored, numbers are never committed.
%
%   CONTROLLER DIAG CONTRACT (driver side; ancestor fields plus Form B rows):
%       diag.a_hat        3x1 [um/pN]  display-layer gain a_o*a_bar_hat*R
%       diag.b_hat        3x1 [-]      Form B length parameter estimate
%       diag.p_hat        3x1 [-]      Form B exponent estimate (REAL state
%                                      here, not the ancestor's alias)
%       diag.ws_hat       3x1 [-]      wall-position estimate w_bar_s
%       diag.P_b/.P_p/.P_ws 3x1 [-]    posterior VARIANCES of slots 5/6/7
%                                      (first call must report the priors)
%       diag.P_a          3x1 [(um/pN)^2], diag.a_prime_hat 3x1 [1/pN]
%       plus the ancestor set: a_xm, h_bar, h_bar_d, gate_active_per_axis,
%       innovation_y2, K_kf_a_y2, R2, dx_r, delta_x_m.
%       Controller knobs used by the arms: lock_b / lock_p / lock_ws,
%       b_init / p_init, y2_off.
%
%   Output struct (per-seed simOut follows the ancestor schema; *_out arrays
%   are N x 3 unless noted; P_*_out store SQRT of the posterior variance):
%       out.runs{q}   full single-seed logs (p_d_out, f_d_out, F_th_out,
%                     p_m_out, p_true_out, a_true_out, a_prime_true_out, tout,
%                     ekf_out, a_hat_out, b_hat_out, p_hat_out, ws_hat_out,
%                     h_bar_out, h_bar_true_out, h_bar_d_out, gate_out,
%                     a_xm_out, a_prime_out, P_a_out, P_b_out, P_p_out,
%                     P_ws_out, innov_y2_out, K_a_y2_out, R2_out, dx_r_out,
%                     dh_m_out, a_nom, R, ctrl_const, meta)
%       out.metrics   per-run table + aggregates + metric windows
%       out.cfg / out.opts / out.arm_tag / out.seeds
%   Saved to test_results/run_formB_ws_<arm_tag>.mat (gitignored).
%
%   TEST-LADDER COMPATIBILITY FORM (smoke_formB_ws / check_formB_* contract):
%       simOut = run_formB_ws(cfg, opts)   % single seed, direct simOut
%   cfg = full scenario config struct (detected by field 'trajectory_type');
%   opts fields: .seed (default 7) .verbose .ctrl_const_override
%   .a_ctrl_override .log_P_full (adds P_full_out, N x n x n x 3;
%   n = 7, or 9 under ma2_aug). The
%   canonical-arm asserts (h_min prior floor) apply only to the arm form.
%
%   See also: motion_control_law_formB_ws, run_5state_expgain

    % ---- test-ladder compatibility form: run_formB_ws(cfg, test_opts) ----
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
    if ~isfield(opts, 'tier');        opts.tier        = 't1';  end
    if ~isfield(opts, 'par_law');     opts.par_law     = true;  end
    if ~isfield(opts, 'y2_on');       opts.y2_on       = true;  end
    if ~isfield(opts, 'oracle_bp');   opts.oracle_bp   = [];    end
    if ~isfield(opts, 'wrong_seed');  opts.wrong_seed  = 0;     end
    if ~isfield(opts, 'a_cov_scale'); opts.a_cov_scale = 1;     end
    if ~isfield(opts, 'ws_inject');   opts.ws_inject = 0;      end   % [R] TRUE wall offset, plant side only (S11 injection)
    if ~isfield(opts, 'plant_gain_law'); opts.plant_gain_law = []; end  % struct(b,p,ws): TRUE z curve = Form B(b,p,ws), plant side only (curve-mismatch arm)
    if ~isfield(opts, 'plant_cell_Rc');  opts.plant_cell_Rc  = []; end  % [um] TRUE z boundary = sphere of this radius (exact J&O), plant side only
    if ~isfield(opts, 'ctrl_cell_Rc');   opts.ctrl_cell_Rc   = []; end  % [um] CONTROLLER anchors/priors re-derived for a cell of this radius
    if ~isfield(opts, 'law_form');    opts.law_form    = 'len'; end  % 'len' = production length writing; 'amp' = amplitude writing probe (single free scalar on the (b, w_s) diagonal)
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
    B_SEED          = 9/8;   % derived anchor: far-field reflection coefficient
    P_SEED          = 1;     % derived anchor: far-field power
    PRIOR_STD_BP    = 1/8;   % GLOBAL-SUP fallback on w_bar in [1.1,10]; only
                             % used for the wrong_seed offset below. The live
                             % priors come from local_envelope_priors on the
                             % envelope this scenario actually visits.
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

    assert(any(strcmpi(opts.tier, {'t1', 't2'})), ...
           'run_formB_ws:badTier', 'opts.tier must be ''t1'' or ''t2''.');
    assert(~(opts.wrong_seed && ~isempty(opts.oracle_bp)), ...
           'run_formB_ws:armConflict', ...
           'wrong_seed and oracle_bp are mutually exclusive arms.');
    assert(any(strcmpi(opts.law_form, {'len', 'amp'})), ...
           'run_formB_ws:badLawForm', 'opts.law_form must be ''len'' or ''amp''.');
    law_amp = strcmpi(opts.law_form, 'amp');
    % The amplitude writing needs the single-free-scalar configuration; every
    % arm below that would break one of its preconditions is refused here
    % rather than silently producing a plausible-looking run.
    assert(~(law_amp && ~isempty(opts.oracle_bp)), 'run_formB_ws:armConflict', ...
           'law_form = ''amp'' and oracle_bp conflict (oracle_bp locks b).');
    assert(~(law_amp && ~isempty(opts.ctrl_cell_Rc)), 'run_formB_ws:armConflict', ...
           'law_form = ''amp'' and ctrl_cell_Rc conflict (cell package sets ws0_perp ~= 1).');
    assert(~(law_amp && strcmpi(opts.tier, 't2')), 'run_formB_ws:armConflict', ...
           'law_form = ''amp'' requires tier t1 (slot 7 must stay locked).');

    seeds = opts.seeds;
    if isempty(seeds); seeds = SEEDS_DEFAULT; end
    seeds = seeds(:).';

    % ------------------------------------------------------------------
    % Canonical scenario config (+ optional overrides, then the h_min gate)
    % ------------------------------------------------------------------
    % Trajectory band. 'deep' (trough w_bar = 1.10) is the standard since
    % 2026-08-19; 'shallow' (2.00) reproduces anything measured before it.
    % Percentages are NOT comparable across the two -- see canonical_scenario.
    if ~isfield(opts, 'scenario'); opts.scenario = 'deep'; end
    cfg = local_canonical_config(A_COV_BASE * opts.a_cov_scale, H_BAR_MIN_PRIOR, ...
                                 opts.scenario);
    fn = fieldnames(opts.config_override);
    for idx = 1:numel(fn)
        cfg.(fn{idx}) = opts.config_override.(fn{idx});
    end

    pc = physical_constants();
    assert(cfg.h_min / pc.R >= H_BAR_MIN_PRIOR - 1e-12, ...
           'run_formB_ws:hMinBelowTruthDomain', ...
           'h_min = %.3f um -> h_bar_min = %.3f < %.2f: below the truth-curve validity floor, so the envelope priors would be read off an extrapolated c_perp.', ...
           cfg.h_min, cfg.h_min / pc.R, H_BAR_MIN_PRIOR);
    assert(cfg.h_bottom / pc.R >= H_BAR_MIN_PRIOR, ...
           'run_formB_ws:troughBelowTruthDomain', ...
           'trajectory trough h_bar = %.3f < %.2f (truth-curve validity floor).', ...
           cfg.h_bottom / pc.R, H_BAR_MIN_PRIOR);

    % ------------------------------------------------------------------
    % Arm flags -> ctrl_const overrides (user ctrl_const_override wins)
    % ------------------------------------------------------------------
    % Envelope priors: derived from the planned trajectory, not tuned. The
    % triple is scenario-dependent by construction, so it is printed, never
    % asserted against a hard reference.
    % Never below the truth-curve validity floor: the envelope is where the
    % priors READ c_perp, and the two-sphere series is not trusted under
    % H_BAR_MIN_PRIOR (c_perp diverges at w_bar = 1). Inert on the shallow
    % band, where the trough is 2.00 and the margin lands at 1.90.
    env_lo = max(cfg.h_bottom / pc.R - ENV_LO_MARGIN, H_BAR_MIN_PRIOR);
    env_hi = cfg.h_init   / pc.R + ENV_HI_MARGIN;
    [sPbb_env, sPpp_env, floor_a_env] = local_envelope_priors(env_lo, env_hi);
    [sPbb_amp, floor_a_amp] = local_envelope_priors_amp(env_lo, env_hi);

    ov = struct();
    ov.Pf_b_std   = sPbb_env;     % sup|b_eff - 9/8| on the envelope
    ov.Pf_p_std   = sPpp_env;     % sup|p_eff - 1|   on the envelope
    ov.Pf_a_floor = floor_a_env;  % sup|a_anchored - 1/c| on the envelope
    ov.lock_ws = strcmpi(opts.tier, 't1');   % t1 = w_s pinned at 1

    % Parallel-axis package on the SAME envelope (see the header block).
    ov.par_law = opts.par_law;
    par_pkg = [];
    if opts.par_law
        par_pkg = local_parallel_law_package(env_lo, env_hi, H_BAR_MIN_PRIOR);
        ov.b_par          = par_pkg.b;
        ov.p_par          = par_pkg.p;
        ov.ws0_par        = par_pkg.ws0;
        ov.Pf_a_floor_par = par_pkg.floor_a;
    end
    % Cell boundary on the WALL-NORMAL axis: the two published asymptotic
    % anchors disagree by 2.3-2.9x for a sphere (vs 1.12x for a plane), so the
    % anchor route would demand a prior width ~30-78x the plane's. The legal
    % construction is the Stage-1a one already used for x/y: fit the law
    % offline to the published exact two-sphere curve on the planned envelope,
    % origin free, and price the width by how much the constants move when the
    % inputs (envelope ends, measured cell radius) move. z only; x/y keep the
    % plane-parallel package, matching the plant (c_para stays the plane curve).
    % Plant-side boundary handle (h_bar -> c_perp); [] = published plane curve.
    % A cell radius wins over a Form B proxy if both are given.
    plant_cperp = [];
    if ~isempty(opts.plant_gain_law)
        plant_cperp = @(hb) calc_formB_cperp(hb, opts.plant_gain_law);
    end
    if ~isempty(opts.plant_cell_Rc)
        plant_lam = opts.plant_cell_Rc / pc.R;
        plant_cperp = @(hb) build_truth_two_sphere(hb, plant_lam);
    end

    cell_pkg = [];
    if ~isempty(opts.ctrl_cell_Rc)
        cell_pkg = local_cell_law_package(env_lo, env_hi, opts.ctrl_cell_Rc / pc.R);
        ov.b_init     = [B_SEED; B_SEED; cell_pkg.b];
        ov.p_init     = [P_SEED; P_SEED; cell_pkg.p];
        ov.ws0_perp   = cell_pkg.ws0;                   % slot 7 stays physical
        ov.Pf_b_std   = [sPbb_env; sPbb_env; cell_pkg.width(1)];
        ov.Pf_p_std   = [sPpp_env; sPpp_env; cell_pkg.width(2)];
        ov.Pf_a_floor = [floor_a_env; floor_a_env; cell_pkg.floor_a];
    end
    if ~opts.y2_on
        ov.y2_off = true;                    % fingerprint arm
    end
    if ~isempty(opts.oracle_bp)
        obp = opts.oracle_bp(:);
        assert(numel(obp) == 2 && all(isfinite(obp)) && all(obp > 0), ...
               'run_formB_ws:badOracle', ...
               'opts.oracle_bp must be [b p], finite and positive.');
        ov.b_init = obp(1);
        ov.p_init = obp(2);
        ov.lock_b = true;                    % shape-floor reference line
        ov.lock_p = true;
    end
    if opts.wrong_seed
        if law_amp
            % "One prior width off" must use THIS writing's width, and p is
            % pinned under the tie, so only b moves.
            ov.b_init = B_SEED + sPbb_amp;
        else
            ov.b_init = B_SEED + PRIOR_STD_BP;   % one prior width off, each
            ov.p_init = P_SEED + PRIOR_STD_BP;
        end
    end
    if law_amp
        % --- Amplitude writing probe (2026-08-09). The law origin is tied to
        %     the length constant (w_s = b), so at p = 1 the law is
        %     a_bar = 1 - b/w_bar and slot 5 is the ONLY free parameter.
        %     Both P0 numbers must come from the amplitude read-off: swapping
        %     only Pf_b_std would leave the gain prior ~13x too tight.
        %     Derivation: derivation/formB_amp_bonly_probe.tex.
        ov.law_form_amp = true;
        ov.lock_b       = false;
        ov.lock_p       = true;              % exponent pinned at the far-field anchor
        ov.lock_ws      = true;              % slot 7 is a frozen reporter here
        ov.p_init       = P_SEED;
        ov.Pf_b_std     = sPbb_amp;          % sup|b_eff,C - 9/8| on the envelope
        ov.Pf_a_floor   = floor_a_amp;       % sup|a_anchored,C - 1/c| on the envelope
        % Under the tie the law's pole sits at w_bar = beta, so the slot-5
        % guard (beta <= w_bar_d - ws_margin) binds whenever the commanded
        % trough is not clear of the seed. A clamped beta looks like a
        % downward traversal and is not one -- refuse the configuration rather
        % than produce that artefact. wrong_seed makes it worse: it starts
        % beta one prior width HIGHER.
        AMP_TROUGH_MARGIN = 0.05;            % [-] readable clearance
        w_trough = cfg.h_bottom / pc.R;
        assert(w_trough > ov.b_init + AMP_TROUGH_MARGIN, ...
               'run_formB_ws:ampTroughClamp', ...
               ['law_form = ''amp'': commanded trough w_bar = %.4f is not clear ', ...
                'of the seed beta = %.4f (needs > seed + %.2f). The slot-5 pole ', ...
                'guard would clamp beta at the trough and fake a downward ', ...
                'traversal.'], w_trough, ov.b_init, AMP_TROUGH_MARGIN);
        % The y2 gate is the ONLY measurement carrying a parameter column, so
        % heights below h_bar_safe contribute NO evidence about beta. If the
        % prior is derived below the gate it prices evidence the filter is
        % forbidden to collect. Report the gap rather than hide it.
        if env_lo < cfg.h_bar_safe - 1e-12
            [sPbb_obs, ~] = local_envelope_priors_amp(cfg.h_bar_safe, env_hi);
            warning('run_formB_ws:ampPriorBelowGate', ...
                    ['amplitude prior derived on [%.3f, %.3f] but the y2 gate ', ...
                     'blinds the parameter channel below h_bar_safe = %.2f. ', ...
                     'Declared sqrt(P[0]) = %.4f; the OBSERVABLE demand saturates ', ...
                     'at %.4f (ratio %.2fx). Travel beyond that is prior, not ', ...
                     'evidence.'], env_lo, env_hi, cfg.h_bar_safe, ...
                    sPbb_amp, sPbb_obs, sPbb_amp / sPbb_obs);
        end
    end
    fn = fieldnames(opts.ctrl_const_override);
    for idx = 1:numel(fn)
        ov.(fn{idx}) = opts.ctrl_const_override.(fn{idx});
    end

    % The fitted law origin carries its own representation width, which adds in
    % quadrature to whatever pre-run knowledge of the surface position the
    % caller declared (applied after the override so it cannot be lost).
    if ~isempty(cell_pkg) && isfield(ov, 'Pf_ws_std') && ~isempty(ov.Pf_ws_std) ...
            && ~(isfield(ov, 'lock_ws') && ov.lock_ws)
        ws_std_v = ov.Pf_ws_std(:);
        if isscalar(ws_std_v); ws_std_v = ws_std_v * ones(3, 1); end
        ws_std_declared = ws_std_v(AX_Z);
        ws_std_v(AX_Z) = sqrt(ws_std_declared^2 + cell_pkg.width(3)^2);
        ov.Pf_ws_std = ws_std_v;
    end

    % arm tag (file name + report header)
    tag = lower(opts.tier);
    if opts.y2_on; tag = [tag '_y2on']; else; tag = [tag '_y2off']; end
    if ~isempty(opts.oracle_bp); tag = [tag '_oraclebp']; end
    if opts.wrong_seed;          tag = [tag '_wrongseed']; end
    if ~opts.par_law;            tag = [tag '_nopar']; end
    if opts.a_cov_scale ~= 1;    tag = [tag sprintf('_acov%g', cfg.a_cov)]; end
    if ~isempty(opts.plant_gain_law); tag = [tag '_plantlaw']; end
    if ~isempty(opts.plant_cell_Rc);  tag = [tag sprintf('_cell%g', opts.plant_cell_Rc)]; end
    if ~isempty(opts.ctrl_cell_Rc);   tag = [tag sprintf('_ctrlcell%g', opts.ctrl_cell_Rc)]; end
    if law_amp;                       tag = [tag '_amp']; end   % NOT optional: the .mat is named by tag

    lastwarn('');

    % ------------------------------------------------------------------
    % Seed loop + per-run console metrics
    % ------------------------------------------------------------------
    fprintf('=== Form B (w_s) driver -- arm: %s ===\n', tag);
    fprintf('scenario: hold %.1fs -> descend %.1fs -> %g Hz osc x%d -> hold, T=%.1fs; h_bar_min=%.2f\n', ...
            cfg.t_hold, cfg.t_descend_override, cfg.frequency, cfg.n_cycles, ...
            cfg.T_sim, cfg.h_min / pc.R);
    fprintf('law_form=%s  lock_ws=%d  y2_on=%d  oracle_bp=%s  wrong_seed=%d  a_cov=%.3f\n', ...
            lower(opts.law_form), ov.lock_ws, opts.y2_on, mat2str(opts.oracle_bp, 5), ...
            opts.wrong_seed, cfg.a_cov);
    fprintf(['ENVELOPE PRIORS on w_bar in [%.3f, %.3f]: sqrt_Pbb %.4f  sqrt_Ppp %.4f  ' ...
             'shape floor %.5f  (derived from the planned trajectory; global-sup fallback %.4f)\n'], ...
            env_lo, env_hi, sPbb_env, sPpp_env, floor_a_env, PRIOR_STD_BP);
    if opts.par_law
        fprintf('PAR LAW (x/y): b %.4f p %.4f ws0 %.4f (+-[%.4f %.4f %.4f]) floor %.4f sup %.3f%%\n', ...
                par_pkg.b, par_pkg.p, par_pkg.ws0, par_pkg.width(1), par_pkg.width(2), ...
                par_pkg.width(3), par_pkg.floor_a, 100 * par_pkg.fit_sup);
    else
        fprintf('PAR LAW (x/y): OFF -- x/y run the perpendicular law (legacy arm)\n');
    end
    if ~isempty(opts.plant_gain_law)
        fprintf('PLANT GAIN LAW (z): Form B b=%g p=%g ws=%g -- controller blind (plane priors kept)\n', ...
                opts.plant_gain_law.b, opts.plant_gain_law.p, opts.plant_gain_law.ws);
    end
    if ~isempty(opts.plant_cell_Rc)
        fprintf('PLANT BOUNDARY (z): CELL Rc = %g um (lam = %.3f), exact two-sphere truth\n', ...
                opts.plant_cell_Rc, opts.plant_cell_Rc / pc.R);
    end
    if ~isempty(cell_pkg)
        fprintf(['CELL LAW (z): b %.4f p %.4f ws0 %.4f (+-[%.4f %.4f %.4f]) floor %.4f ' ...
                 'sup %.3f%% | Rc = %g um\n'], cell_pkg.b, cell_pkg.p, cell_pkg.ws0, ...
                cell_pkg.width(1), cell_pkg.width(2), cell_pkg.width(3), ...
                cell_pkg.floor_a, 100 * cell_pkg.fit_sup, opts.ctrl_cell_Rc);
        if isfield(ov, 'Pf_ws_std') && ~isempty(ov.Pf_ws_std)
            wsv = ov.Pf_ws_std(:);
            fprintf('  w_s prior (z): declared %.4f (+) origin width %.4f -> %.4f\n', ...
                    ws_std_declared, cell_pkg.width(3), wsv(AX_Z));
        end
    end
    fprintf('%6s | %10s %10s %11s | %8s %8s | %4s\n', 'seed', ...
            'desc pk %', 'osc RMS %', 'hold mean %', 'b diag', 'p diag', 'NaN');

    n_seeds = numel(seeds);
    runs  = cell(n_seeds, 1);
    Mrows = zeros(n_seeds, 6);   % desc | osc | hold | bud_b | bud_p | nan
    for q = 1:n_seeds
        s = local_run_once(cfg, seeds(q), ov, opts.verbose, opts.a_ctrl_override, false, opts.ws_inject, plant_cperp);
        m = local_run_metrics(s, cfg, AX_Z, OSC_SETTLE_S, HOLD_SETTLE_S);
        runs{q}     = s;
        Mrows(q, :) = [m.desc_peak_pct, m.osc_rms_pct, m.hold_mean_pct, ...
                       m.budget_b, m.budget_p, m.any_nan];
        fprintf('%6d | %10.3f %10.3f %+11.3f | %8.3f %8.3f | %4d\n', ...
                seeds(q), Mrows(q, 1), Mrows(q, 2), Mrows(q, 3), ...
                Mrows(q, 4), Mrows(q, 5), Mrows(q, 6));
    end

    % ------------------------------------------------------------------
    % Aggregates
    % ------------------------------------------------------------------
    mu = mean(Mrows(:, 1:3), 1);
    sd = std(Mrows(:, 1:3), 0, 1);
    fprintf('-----------------------------------------------------------------\n');
    fprintf('mean over %d seeds: desc pk %.3f+-%.3f %%  osc RMS %.3f+-%.3f %%  hold mean %+.3f+-%.3f %%\n', ...
            n_seeds, mu(1), sd(1), mu(2), sd(2), mu(3), sd(3));
    % Budget criterion (recalibrated 2026-08-01). The traversal ratio
    % (x_end - x_0)^2 / (P[0] - P[end]) is a chi2(1) variate under the
    % saturated bound E[ratio] <= 1, not a hard cap: a single seed above 1 is
    % ordinary sampling noise. Per-seed values are therefore DIAGNOSTIC, and
    % flagged only beyond the chi2(1) 99th percentile 6.635. The acceptance
    % test is the 6-seed mean, whose one-sided 95% limit under E[ratio] <= 1
    % with Var(chi2(1)) = 2 is 1 + 1.645*sqrt(2/6) = 1.949.
    RATIO_SEED_FLAG = 6.635;   % chi2(1) 99th percentile
    RATIO_MEAN_PASS = 1 + 1.645 * sqrt(2 / n_seeds);
    flag_b = find(Mrows(:, 4) > RATIO_SEED_FLAG);
    flag_p = find(Mrows(:, 5) > RATIO_SEED_FLAG);
    mean_b = mean(Mrows(:, 4));
    mean_p = mean(Mrows(:, 5));
    if isnan(mean_p)
        fprintf('budget aggregate: b mean %.3f (%s <= %.3f), p n/a (locked, zero-width prior)\n', ...
                mean_b, local_verdict(mean_b, RATIO_MEAN_PASS), RATIO_MEAN_PASS);
        fprintf('per-seed traversal ratios (diagnostic; flag > %.3f): b max %.3f%s | p n/a\n', ...
                RATIO_SEED_FLAG, max(Mrows(:, 4)), local_flag_str(flag_b, seeds));
    else
        fprintf('budget aggregate: b mean %.3f (%s <= %.3f), p mean %.3f (%s <= %.3f)\n', ...
                mean_b, local_verdict(mean_b, RATIO_MEAN_PASS), RATIO_MEAN_PASS, ...
                mean_p, local_verdict(mean_p, RATIO_MEAN_PASS), RATIO_MEAN_PASS);
        fprintf('per-seed traversal ratios (diagnostic; flag > %.3f): b max %.3f%s | p max %.3f%s\n', ...
                RATIO_SEED_FLAG, max(Mrows(:, 4)), local_flag_str(flag_b, seeds), ...
                max(Mrows(:, 5)), local_flag_str(flag_p, seeds));
    end
    if law_amp
        fprintf(['prior widths sqrt(P[0]) (controller-reported, z): b %.4f  ', ...
                 '(AMPLITUDE envelope %.4f; shape floor %.5f vs length %.5f)\n'], ...
                runs{1}.P_b_out(1, AX_Z), sPbb_amp, floor_a_amp, floor_a_env);
    else
        fprintf('prior widths sqrt(P[0]) (controller-reported, z): b %.4f  p %.4f  (envelope %.4f / %.4f)\n', ...
                runs{1}.P_b_out(1, AX_Z), runs{1}.P_p_out(1, AX_Z), sPbb_env, sPpp_env);
    end
    fprintf('h_bar range (true, seed %d): [%.3f, %.3f]\n', seeds(1), ...
            min(runs{1}.h_bar_true_out), max(runs{1}.h_bar_true_out));

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
                                        'hold_mean_pct', 'budget_b', ...
                                        'budget_p', 'any_nan'}}, ...
                         'mean_desc_peak_pct', mu(1), 'std_desc_peak_pct', sd(1), ...
                         'mean_osc_rms_pct',   mu(2), 'std_osc_rms_pct',   sd(2), ...
                         'mean_hold_mean_pct', mu(3), 'std_hold_mean_pct', sd(3), ...
                         'windows', local_metric_windows(cfg, OSC_SETTLE_S, HOLD_SETTLE_S), ...
                         'axis', AX_Z, 'prior_std_bp', PRIOR_STD_BP);
    out.cfg     = cfg;
    out.opts    = opts;
    out.arm_tag = tag;
    out.seeds   = seeds;
    out.par_pkg  = par_pkg;      % [] when par_law is off
    out.cell_pkg = cell_pkg;     % [] unless the controller is anchored for a cell

    here = fileparts(mfilename('fullpath'));
    out_dir = fullfile(here, '..', '..', 'test_results');
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end
    out_file = fullfile(out_dir, ['run_formB_ws_' tag '.mat']);
    save(out_file, 'out');
    fprintf('saved: %s\n', out_file);
end


%% =================== Local Helpers ===================

function cfg = local_canonical_config(a_cov, h_bar_min_prior, name)
%LOCAL_CANONICAL_CONFIG  Thin wrapper on the shared scenario definition.
%   The five gain-law drivers each carried a byte-identical copy of this
%   scenario until 2026-08-19; they now all read model/config/canonical_scenario
%   so a change to the trajectory cannot reach four drivers and miss the fifth.
%   Default is 'deep' (trough w_bar = 1.10); pass 'shallow' to reproduce a
%   number measured before that date.
    if nargin < 3 || isempty(name); name = 'deep'; end
    cfg = canonical_scenario(a_cov, h_bar_min_prior, name);
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


function [sPbb, sPpp, floor_a] = local_envelope_priors(h_lo, h_hi)
%LOCAL_ENVELOPE_PRIORS  (b, p) prior widths and the shape floor on ONE envelope.
%   [sPbb, sPpp, floor_a] = local_envelope_priors(h_lo, h_hi)
%
%   The anchored pair (b, p) = (9/8, 1) is exact only at the two asymptotic
%   ends; in between the truth demands a slightly different pair. Reading the
%   truth curve c_perp(w_bar) through the gain law gives, at each height, the
%   effective parameters the truth is asking for:
%       b_eff = (c - 1)*(w_bar - 1)                      (level reading)
%       p_eff = -((w_bar - 1) + 9/8)*dc / (c*(c - 1))    (slope reading)
%   Their sup departure from the anchors over the planned envelope IS the
%   prior width -- the same interpolation-gap construction the expgain and
%   powerlaw priors use, evaluated on the heights this run will actually
%   visit instead of the global [1.1, 10] domain. Nothing here is tuned: the
%   only inputs are the trajectory and the published truth curve.
%
%   floor_a is the residual the anchored seed cannot represent even with a
%   perfect (b, p) read-off: sup |a_bar(anchored) - 1/c| over the envelope.
%   It funds the P_aa shape floor (controller ctrl_const.Pf_a_floor).
%
%   Outputs are sqrt-P quantities (standard deviations), [-].
    B_SEED   = 9/8;    % derived far-field reflection coefficient
    P_SEED   = 1;      % derived far-field power
    N_SWEEP  = 20001;  % dense enough that the sup is grid-independent

    assert(h_lo > 1 && h_hi > h_lo, 'run_formB_ws:badEnvelope', ...
           'envelope must satisfy 1 < h_lo (%.3f) < h_hi (%.3f).', h_lo, h_hi);

    h  = linspace(h_lo, h_hi, N_SWEEP).';
    c  = zeros(N_SWEEP, 1);
    dc = zeros(N_SWEEP, 1);
    for i = 1:N_SWEEP
        [~, c_perp_i, dv_i] = calc_correction_functions(h(i), true);
        c(i)  = c_perp_i;
        dc(i) = dv_i.dc_perp_dh;
    end

    b_eff = (c - 1) .* (h - 1);
    p_eff = -((h - 1) + B_SEED) .* dc ./ (c .* (c - 1));
    sPbb  = max(abs(b_eff - B_SEED));
    sPpp  = max(abs(p_eff - P_SEED));

    a_anchored = 1 - (1 + (h - 1) / B_SEED).^(-P_SEED);
    floor_a    = max(abs(a_anchored - 1 ./ c));
end


function [sPbb, floor_a] = local_envelope_priors_amp(h_lo, h_hi)
%LOCAL_ENVELOPE_PRIORS_AMP  b prior width and shape floor, AMPLITUDE writing.
%   [sPbb, floor_a] = local_envelope_priors_amp(h_lo, h_hi)
%
%   Sibling of local_envelope_priors for the tied (law_form_amp) arm, where
%   the law is a_bar = 1 - b/w_bar. Taking logs of the deficit,
%       ln(1 - a_bar) = ln b - p*ln(w_bar)
%   so b is the log-log INTERCEPT and p the slope; with p pinned at 1 the
%   intercept is read straight off the truth deficit (c-1)/c:
%       b_eff = w_bar * (c - 1) / c
%   No p width is returned: the exponent is pinned under the tie, so a p
%   prior would be a claim about a state that cannot move.
%
%   Unlike the length reading, b_eff here is strictly MONOTONE (1 at contact
%   -> 9/8 in the far field), so the sup sits on the envelope FLOOR and the
%   width inherits the floor margin's provenance directly. The length
%   reading's sup is interior and boundary-immune; that difference is
%   declared, with numbers, in derivation/formB_amp_bonly_probe.tex.
%
%   Outputs are sqrt-P quantities (standard deviations), [-].
    B_SEED  = 9/8;    % derived far-field reflection coefficient
    N_SWEEP = 20001;  % same grid density as the length sibling

    % h_lo = trough - ENV_LO_MARGIN, so a trough at the truth-domain floor 1.1
    % lands here at exactly 1.0 and trips this. Name the real cause: the two
    % floors differ by one ENV_LO_MARGIN.
    assert(h_lo > 1 && h_hi > h_lo, 'run_formB_ws:badEnvelope', ...
           ['envelope must satisfy 1 < h_lo (%.3f) < h_hi (%.3f). h_lo is the ', ...
            'commanded trough LESS the tracking margin, so the deepest trough ', ...
            'this admits is 1 + ENV_LO_MARGIN, not the truth-domain floor 1.1.'], ...
           h_lo, h_hi);

    h = linspace(h_lo, h_hi, N_SWEEP).';
    c = zeros(N_SWEEP, 1);
    for i = 1:N_SWEEP
        [~, c_perp_i] = calc_correction_functions(h(i), true);
        c(i) = c_perp_i;
    end

    b_eff = h .* (c - 1) ./ c;
    sPbb  = max(abs(b_eff - B_SEED));

    a_anchored = 1 - B_SEED ./ h;
    floor_a    = max(abs(a_anchored - 1 ./ c));

    % Self-check: pointwise |a_anchored - 1/c| == |b_eff - 9/8| / h exactly,
    % so the two numbers above cannot disagree unless a formula is wrong.
    assert(abs(floor_a - max(abs(b_eff - B_SEED) ./ h)) < 1e-12, ...
           'run_formB_ws:ampPriorIdentity', ...
           'amplitude prior identity violated (%.3e) -- check both formulas.', ...
           abs(floor_a - max(abs(b_eff - B_SEED) ./ h)));
end


function pkg = local_parallel_law_package(w_lo, w_hi, truth_floor)
%LOCAL_PARALLEL_LAW_PACKAGE  Form B constants for the WALL-PARALLEL axes.
%   pkg = local_parallel_law_package(w_lo, w_hi)
%
%   The perpendicular anchors (b, p, w_s) = (9/8, 1, 1) come from two
%   asymptotic limits that both hold for c_perp. For c_para the near-wall
%   limit is Goldman's logarithm, which no power of (1 + g/b) reproduces at
%   contact, so the anchored law fails structurally on x/y (the standing
%   powerlaw/expgain finding). What the parallel truth DOES admit is a Form B
%   representative whose origin sits INSIDE the wall: with the origin free,
%   the three-parameter minimax fit tracks 1/c_para to <0.04% over the whole
%   working envelope. That representative is what the x/y filters run.
%
%   Everything here is offline: the truth curve is evaluated once per driver
%   call on the planned envelope, exactly like local_envelope_priors, and only
%   the resulting constants cross into the controller. Nothing is tuned -- the
%   inputs are the envelope and the published c_para series.
%
%   pkg fields:
%       .b .p .ws0    fitted law constants; ws0 is the law origin when the
%                     physical wall sits at w_bar = 1, so the offset the
%                     controller applies to its w_s estimate is ws0 - 1 [-]
%       .width        1x3 [db dp dws0] envelope-boundary sensitivity: the
%                     largest constant shift over three refits that move the
%                     envelope ends by the same margins the scenario itself is
%                     uncertain to
%       .fit_sup      minimax residual sup|a_law - 1/c_para| on the envelope
%       .floor_a      sqrt P(4,4) seed floor: root-sum of the fit residual and
%                     the three width pushforwards through dA/dtheta -- the
%                     gain error the x/y filters cannot represent away
    N_FIT      = 8000;                 % grid; the sup is grid-independent here
    TH_START   = [0.52, 0.98, 0.58];   % near the fitted optimum (simplex seed)
    GAP_FLOOR  = 0.01;                 % law-argument floor, matches the
                                       % controller's positivity guard
    LO_PROBE   = 0.1;                  % boundary probe = ENV_LO_MARGIN
    HI_PROBE   = 15;                   % far-end probe: where the envelope sups
                                       % have already gone flat

    % The downward probe asks for the truth LO_PROBE below the envelope floor,
    % to price how much the fitted law moves if the scenario floor were lower.
    % On the deep band the floor already sits ON the two-sphere validity limit,
    % so there is nothing valid down there and the probe is clamped to it. The
    % width contributed in that direction is then zero -- which is honest, we
    % cannot price an uncertainty from a series that does not reach it -- but it
    % must be VISIBLE rather than a silent zero, hence the banner line below.
    w_lo_probe = max(w_lo - LO_PROBE, truth_floor);
    pkg.lo_probe_clamped = (w_lo_probe > w_lo - LO_PROBE + 1e-12);
    assert(w_lo_probe > 1, 'run_formB_ws:parProbeBelowContact', ...
           'near-wall boundary probe w_bar = %.3f is not > 1 (c_para domain).', ...
           w_lo_probe);
    if pkg.lo_probe_clamped
        fprintf(['PAR LAW: downward envelope probe clamped %.3f -> %.3f (truth-curve ' ...
                 'floor); its share of the theta width is 0 by construction.\n'], ...
                w_lo - LO_PROBE, w_lo_probe);
    end
    assert(HI_PROBE > w_lo, 'run_formB_ws:parProbeDegenerate', ...
           'far-end probe cap %.3f must exceed the envelope floor %.3f.', HI_PROBE, w_lo);

    [th, sup] = local_fit_par_law(w_lo, w_hi, TH_START, N_FIT, GAP_FLOOR);

    probes = [w_lo + LO_PROBE, w_hi; ...
              w_lo_probe,        w_hi; ...
              w_lo,            min(HI_PROBE, w_hi)];
    dth = zeros(size(probes, 1), 3);
    for j = 1:size(probes, 1)
        dth(j, :) = local_fit_par_law(probes(j, 1), probes(j, 2), th, N_FIT, GAP_FLOOR) - th;
    end
    width = max(abs(dth), [], 1);

    % Width pushforward onto the gain: the level Jacobians dA/dtheta of the
    % SAME law the controller evaluates (local_gain_law_formB), on the grid.
    w    = linspace(w_lo, w_hi, N_FIT).';
    g    = max(w - th(3), GAP_FLOOR);
    u    = 1 + g / th(1);
    upow = u.^(-th(2));
    a_prime = (th(2) / th(1)) * upow ./ u;      % |dA/dws|
    dA_db   = (g / th(1)) .* a_prime;           % |dA/db|
    dA_dp   = upow .* log(u);                   % |dA/dp|

    pkg = struct();
    pkg.b       = th(1);
    pkg.p       = th(2);
    pkg.ws0     = th(3);
    pkg.width   = width;
    pkg.fit_sup = sup;
    pkg.floor_a = sqrt(sup^2 ...
                       + (max(a_prime) * width(3))^2 ...
                       + (max(dA_db)   * width(1))^2 ...
                       + (max(abs(dA_dp)) * width(2))^2);
end


function pkg = local_cell_law_package(w_lo, w_hi, lam)
%LOCAL_CELL_LAW_PACKAGE  Form B constants for a CELL boundary on the z axis.
%   pkg = local_cell_law_package(w_lo, w_hi, lam)
%
%   The plane's anchors (9/8, 1) come from two asymptotic limits that agree to
%   12.5 %. For a sphere of radius ratio lam = Rc/R the same two limits give
%   b_far = (3/2) sqrt(lam) against b_near = 2 [lam/(1+lam)]^2 -- a factor
%   2.3-2.9 apart -- so the anchor route would have to declare a prior width
%   30-78x the plane's, which is the parameter-wander regime the narrow priors
%   exist to avoid. This package takes the OTHER legal route, the one already
%   in production for the wall-parallel axes: fit the law offline to the
%   published exact two-sphere gain 1/c_cell (Jeffrey & Onishi 1984,
%   build_truth_two_sphere) on the planned envelope with the origin free, and
%   price the prior width by how much the fitted constants move when the inputs
%   move -- the envelope ends, and the MEASURED cell radius.
%
%   Everything is offline: the truth curve is evaluated once per driver call
%   and only the resulting constants cross into the controller, which stays
%   run-time c-free. Nothing is tuned; the inputs are the envelope and lam.
%
%   pkg fields (same schema as local_parallel_law_package):
%       .b .p .ws0    fitted constants; ws0 is the law origin when the physical
%                     surface sits at w_bar = 1, so the controller declares the
%                     shift ws0 - 1 through ctrl_const.ws0_perp and slot 7
%                     keeps meaning the physical surface position [-]
%       .width        1x3 [db dp dws0] input sensitivity: the largest constant
%                     shift over four refits (envelope ends +-LO_PROBE, cell
%                     radius +-RC_TOL)
%       .fit_sup      minimax residual sup|a_law - 1/c_cell| on the envelope
%       .floor_a      sqrt P(4,4) seed floor: root-sum of the fit residual and
%                     the three width pushforwards through dA/dtheta
    N_FIT     = 4000;                 % grid; the sup is grid-independent here
    TH_START  = [1.5, 1.7, 0.80];     % near the fitted optimum (simplex seed)
    GAP_FLOOR = 0.01;                 % law-argument floor, matches the
                                      % controller's positivity guard
    LO_PROBE  = 0.1;                  % envelope-end probe = ENV_LO_MARGIN
    RC_TOL    = 0.10;                 % cell-radius measurement tolerance [-]

    assert(lam > 0 && isfinite(lam), 'run_formB_ws:badCellLam', ...
           'cell radius ratio lam = %.4f must be finite and positive.', lam);
    assert(w_lo - LO_PROBE > 1, 'run_formB_ws:cellProbeBelowContact', ...
           'near-wall boundary probe w_bar = %.3f is not > 1.', w_lo - LO_PROBE);

    [th, sup] = local_fit_cell_law(w_lo, w_hi, lam, TH_START, N_FIT, GAP_FLOOR);

    probes = {[w_lo + LO_PROBE, w_hi, lam], [w_lo - LO_PROBE, w_hi, lam], ...
              [w_lo, w_hi, (1 + RC_TOL) * lam], [w_lo, w_hi, (1 - RC_TOL) * lam]};
    dth = zeros(numel(probes), 3);
    for j = 1:numel(probes)
        dth(j, :) = local_fit_cell_law(probes{j}(1), probes{j}(2), probes{j}(3), ...
                                       th, N_FIT, GAP_FLOOR) - th;
    end
    width = max(abs(dth), [], 1);

    w    = linspace(w_lo, w_hi, N_FIT).';
    g    = max(w - th(3), GAP_FLOOR);
    u    = 1 + g / th(1);
    upow = u.^(-th(2));
    a_prime = (th(2) / th(1)) * upow ./ u;      % |dA/dws|
    dA_db   = (g / th(1)) .* a_prime;           % |dA/db|
    dA_dp   = upow .* log(u);                   % |dA/dp|

    pkg = struct();
    pkg.b       = th(1);
    pkg.p       = th(2);
    pkg.ws0     = th(3);
    pkg.lam     = lam;
    pkg.width   = width;
    pkg.fit_sup = sup;
    pkg.floor_a = sqrt(sup^2 ...
                       + (max(a_prime) * width(3))^2 ...
                       + (max(dA_db)   * width(1))^2 ...
                       + (max(abs(dA_dp)) * width(2))^2);
end


function [th, sup] = local_fit_cell_law(w_lo, w_hi, lam, th0, n_fit, gap_floor)
%LOCAL_FIT_CELL_LAW  3-parameter minimax fit of Form B to the cell gain 1/c.
%   Same non-smooth objective and restart discipline as local_fit_par_law.
    w = linspace(w_lo, w_hi, n_fit).';
    a_target = 1 ./ build_truth_two_sphere(w, lam);
    a_target = a_target(:);
    obj = @(t) max(abs(1 - (1 + max(w - t(3), gap_floor) / t(1)).^(-t(2)) - a_target));
    solver_opts = optimset('TolX', 1e-10, 'TolFun', 1e-12, ...
                           'MaxFunEvals', 2e4, 'MaxIter', 2e4, 'Display', 'off');
    th  = fminsearch(obj, th0, solver_opts);
    th  = fminsearch(obj, th,  solver_opts);
    sup = obj(th);
end


function [th, sup] = local_fit_par_law(w_lo, w_hi, th0, n_fit, gap_floor)
%LOCAL_FIT_PAR_LAW  3-parameter minimax fit of the Form B law to 1/c_para.
%   theta = [b, p, ws0]; the objective is the sup residual on [w_lo, w_hi],
%   which is non-smooth, so fminsearch is restarted once from its own answer
%   (a collapsed simplex cannot certify a minimax optimum).
    w = linspace(w_lo, w_hi, n_fit).';
    c = zeros(n_fit, 1);
    for i = 1:n_fit
        c(i) = calc_correction_functions(w(i));      % c_para (first output)
    end
    a_target = 1 ./ c;      % normalized parallel gain a/a_o = 1/c_para
    obj = @(t) max(abs(1 - (1 + max(w - t(3), gap_floor) / t(1)).^(-t(2)) - a_target));
    solver_opts = optimset('TolX', 1e-10, 'TolFun', 1e-12, ...
                           'MaxFunEvals', 2e4, 'MaxIter', 2e4, 'Display', 'off');
    th  = fminsearch(obj, th0, solver_opts);
    th  = fminsearch(obj, th,  solver_opts);
    sup = obj(th);
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
           'run_formB_ws:emptyMetricWindow', ...
           'A metric window is empty -- check trajectory timing overrides.');

    m.desc_peak_pct = max(abs(e_pct(w_desc)));
    m.osc_rms_pct   = sqrt(mean(e_pct(w_osc).^2));
    m.hold_mean_pct = mean(e_pct(w_hold));

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


function simOut = local_run_once(config, seed, ctrl_const_override, verbose, a_ctrl_override, log_P_full, ws_inject, plant_cperp)
if nargin < 7; ws_inject = 0; end
if nargin < 8; plant_cperp = []; end
%LOCAL_RUN_ONCE  One seed of the scenario. Fork of run_5state_expgain's body
%   with the controller hard-dispatched to motion_control_law_formB_ws and the
%   log schema extended by the p / w_s parameter rows.

    % Fresh persistent state per run (controller, trajectory, thermal).
    clear motion_control_law_formB_ws trajectory_generator calc_thermal_force;

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
        [f_d_k, ekf_k, diag_k] = motion_control_law_formB_ws(del_pd_k, pd_k, ...
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
                         'seed', seed, 'driver', 'run_formB_ws', ...
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
