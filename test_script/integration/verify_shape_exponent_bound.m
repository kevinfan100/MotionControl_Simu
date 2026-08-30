function res = verify_shape_exponent_bound(law, opts)
%VERIFY_SHAPE_EXPONENT_BOUND  Shape-acceptance criterion for low-parameter gain laws.
%   STATUS: DRAFT (2026-08-25) | first implementation of the tool named by the
%           charter (CLAUDE.md, constraint 4) and by
%           reference/shared/param_prior_rules.md; the file did not exist before.
%           Every assumption is listed in the ASSUMPTIONS block below. Nothing
%           in production is read or modified by this file; it is offline only.
%
%   res = verify_shape_exponent_bound(law)            law = 'formC'|'formB'|'expgain'|'all'
%   res = verify_shape_exponent_bound(law, opts)
%
%   CRITERION (charter):   sup_E |theta_eff(w) - theta_0|  <~  sqrt(P[0])   <=>  Q_theta = 0 honest
%
%   theta_eff(w) = the parameter value that makes the law reproduce the TRUE
%   correction curve (calc_correction_functions, offline truth) at height w.
%   theta_0      = the seed the driver actually uses (the analytic anchor).
%   sqrt(P[0])   = the prior width. THREE denominators are reported:
%
%     (D) driver   : the width the driver ships. For formC and formB this is
%                    itself sup|theta_eff - theta_0| over the same envelope
%                    (run_formC_b.m local_envelope_b_range; run_formB_ws.m
%                    local_envelope_priors), so ratio (D) is identically 1.0000
%                    whenever theta_0 is the anchor: the circular criterion the
%                    audit flagged. It is printed and flagged CIRCULAR, not used
%                    as the verdict. For expgain the shipped 0.10 is a hand-
%                    rounded copy of the truth-curve sup 0.0942 (tex Stage 2b),
%                    so it is circular one step removed.
%     (A) asymptote-A : |theta_near - theta_far|, the disagreement between the
%                    two PUBLISHED limits (Brenner lubrication near the wall,
%                    Lorentz/Faxen method of reflections in the far field),
%                    read at LEADING order and without touching the truth
%                    curve. One constant must serve the whole traverse, and the
%                    published ends of the traverse already demand two
%                    different constants; their gap is the honest truth-free
%                    width. Degenerates to 0 when the two limits agree (expgain).
%     (B) asymptote-B : the two published expansions taken to their NEXT
%                    published term (Cox-Brenner 3-term lubrication; Faxen
%                    reciprocal 2-term far field), each used on its own side of
%                    the height where they cross, theta_eff read off THAT
%                    truth-free composite, sup over the envelope. Never
%                    degenerate; conservative by construction (each expansion
%                    is worst at the crossover).
%
%   The verdict the tool returns is (A) where (A) is non-degenerate and (B)
%   otherwise; (D) is reported for the record. Thresholds (team-lead spec, the
%   ledger defines none): ratio < 0.9 PASS, 0.9..1.1 TIGHT, > 1.1 FAIL. The
%   ledger's freeze gate (margin = 1/ratio >= 3x) is printed alongside.
%
%   theta_eff PER LAW (perpendicular truth c = c_perp(w), c' = dc/dw, g = w-1):
%     formC    a' = b (1-a)^2, a = 1/c            (formC_state_b.tex S1)
%              b_eff(w)  = a'_true / (1-a_true)^2 = -c' / (c-1)^2        [slope, PRIMARY]
%              b_lev(w)  = c / ((c-1) g)                                  [level, seed curve
%                          a = 1 - 1/(b g), reported only]
%              theta_0 = 8/9 (far-field anchor; contact anchor is 1)
%     formB    a = 1 - (1 + (w - ws)/b)^(-p), theta = (b, p, ws), anchors (9/8, 1, 1)
%              reduction = one parameter at a time, the other two AT THEIR
%              ANCHORS (this is exactly the construction the driver's prior uses):
%              b_eff(w)  = (c-1) g                       [level reading, p=1, ws=1]
%              p_eff(w)  = -(g + 9/8) c' / (c (c-1))     [slope reading, b=9/8, ws=1]
%              ws_eff(w) = w - (9/8)/(c-1)               [level reading, b=9/8, p=1]
%              plus a JOINT (b,p) reading with ws=1 where level AND slope match
%              at once (fzero), reported only. Norm for the 3-vector = max over
%              components of the per-component ratio (prior-scaled inf-norm).
%     expgain  Psi = 1 - a_h/a_o = w^(-b)          (5state_expgain_hd.tex Stage 2b)
%              b_eff(w)  = -dlnPsi/dlnw = -w c' / (c (c-1))               [slope, PRIMARY]
%              b_lev(w)  = -ln((c-1)/c) / ln w                            [level, reported]
%              theta_0 = 1 (both limits)
%
%   ENVELOPE (replicated from the drivers, which hold these as local constants):
%     env_lo = max(h_bottom/R - ENV_LO_MARGIN, H_BAR_MIN_PRIOR), env_hi = h_init/R + ENV_HI_MARGIN
%     deep    -> [1.100, 23.222]     shallow -> [1.900, 23.222]
%
%   opts (all optional):
%     .scenario   'deep' | 'shallow' | 'both'   (default 'both')
%     .env        [lo hi] explicit envelope, overrides .scenario (for ledger
%                 reproduction: [1.1 10] = "all h", [2 10] = "h >= 2")
%     .axis       'perp' (default) | 'para'     truth column; the laws are
%                 anchored for perp, 'para' is only meaningful for the
%                 expgain ledger rows and is reported with the same theta_0
%     .n_grid     20001 (drivers' density)
%     .save_fig   true
%     .out_dir    test_results/shape_bound
%     .verbose    true
%
%   Output res: struct array (one row per law x band) with fields
%     law, band, axis, env, names, theta0, sup, w_sup, profile (w, dtheta),
%     sqrtP_driver, driver_src, ratio_driver, verdict_driver, circular,
%     sqrtP_asymA, ratio_asymA, verdict_asymA,
%     sqrtP_asymB, ratio_asymB, verdict_asymB, crossover,
%     verdict (the returned verdict and which denominator it used), fig
%
%   ASSUMPTIONS / AMBIGUITIES (numbered; referenced from the report)
%    1. theta_eff is the SLOPE reading for formC and expgain (their derivations
%       define b that way) and the driver's own per-parameter readings for
%       formB (level for b and ws, slope for p). Level/joint alternatives are
%       computed and printed but do not enter the verdict.
%    2. theta_0 is the seed the production driver uses: 8/9 (formC, arm 'best'/
%       'b98'), (9/8, 1, 1) (formB), 1 (expgain b). All are one of the two
%       published anchors, which construction (A) requires.
%    3. Driver widths are RE-IMPLEMENTED here (independent code path, different
%       grid) because they live in local functions of the drivers and cannot
%       be called; the formC shallow value is checked against the number the
%       driver/ref document prints (0.021911). Expgain's 0.10 is the controller
%       default (motion_control_law_5state_expgain_alg.m:164), not derived.
%    4. formB's ws width 0.111 is the caller-supplied t2 value quoted in the
%       run_formB_ws.m header ("calibration-based, not a theta_eff sup"); tier
%       t1 (production) locks ws, so its driver width is 0 there.
%    5. Published far-field form = Faxen/Happel-Brenner RECIPROCAL form
%       c = 1/(1 - 9/8 u + 1/2 u^3), u = 1/w. Its leading term is the Lorentz
%       9/8. Consequence for formB: matching 1-a = b/(b+w-ws) to 1-D gives the
%       far-field ws anchor 9/8 (no u^2 term in D), so the ws anchor gap is
%       1/8. Had the ADDITIVE Lorentz form c = 1 + 9/8 u been used instead the
%       far-field ws would be 0 and the gap 1. The reciprocal form is the one
%       the project's truth source is built on (calc_correction_functions).
%    6. Published near-wall form = Brenner lubrication c = 1/eps (K = 1) at
%       leading order (construction A) and Cox & Brenner (1967) three-term
%       c = 1/eps + (1/5) ln(1/eps) + 0.971 for construction B. The 0.971 is a
%       transcribed literature constant, not re-derived here.
%    7. Construction B joins the two expansions at the height where they cross
%       (fzero on c_near - c_far, found near w = 1.95); this is the only
%       branch rule with no free number. Its sup therefore sits at the
%       crossover, where each expansion is least accurate: (B) is a bracket,
%       not an estimate, and reads conservative.
%    8. Envelope margins (0.1 below the trough, 1.0 above the start, floor 1.1)
%       are copied from the drivers; env_hi = 23.222 in both bands.
%    9. Thresholds 0.9 / 1.1 are the team-lead's; shape_ledger.md only fixes
%       the freeze gate margin >= 3x, which is printed too.
%   10. Truth axis: perpendicular (z, c_perp) for all three laws. 'para' rows
%       are ledger reproduction only; formC/formB declare x/y out of scope.
%
%   See also: reference/shared/param_prior_rules.md,
%             reference/eq17_analysis/shape_ledger.md,
%             reference/eq17_analysis/derivation/formC_state_b_ref.tex (open items 3,4)

    if nargin < 1 || isempty(law); law = 'all'; end
    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'scenario'); opts.scenario = 'both';  end
    if ~isfield(opts, 'env');      opts.env      = [];      end
    if ~isfield(opts, 'axis');     opts.axis     = 'perp';  end
    if ~isfield(opts, 'n_grid');   opts.n_grid   = 20001;   end
    if ~isfield(opts, 'save_fig'); opts.save_fig = true;    end
    if ~isfield(opts, 'verbose');  opts.verbose  = true;    end

    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(fileparts(script_dir));
    addpath(fullfile(project_root, 'model', 'wall_effect'));
    addpath(fullfile(project_root, 'model', 'config'));
    if ~isfield(opts, 'out_dir') || isempty(opts.out_dir)
        opts.out_dir = fullfile(project_root, 'test_results', 'shape_bound');
    end
    if opts.save_fig && ~exist(opts.out_dir, 'dir'); mkdir(opts.out_dir); end

    % ------------------------------------------------------------------
    % Named constants (every number is a house convention copied from the
    % drivers, or a published anchor)
    % ------------------------------------------------------------------
    H_BAR_MIN_PRIOR = 1.1;    % truth-curve validity floor (drivers)
    ENV_LO_MARGIN   = 0.1;    % drivers' envelope floor margin
    ENV_HI_MARGIN   = 1.0;    % drivers' envelope ceiling margin
    A_COV_BASE      = 0.05;   % only needed to build the scenario config
    THR_PASS        = 0.9;    % team-lead thresholds (assumption 9)
    THR_TIGHT       = 1.1;
    FREEZE_MARGIN   = 3.0;    % shape_ledger.md freeze gate: margin >= 3x

    laws = lower(string(law));
    if laws == "all"; laws = ["formc", "formb", "expgain"]; end
    if ~isempty(opts.env)
        bands = {struct('name', sprintf('env[%.3g,%.3g]', opts.env), 'env', opts.env(:).')};
    else
        switch lower(opts.scenario)
            case 'both';    names = {'deep', 'shallow'};
            case 'deep';    names = {'deep'};
            case 'shallow'; names = {'shallow'};
            otherwise; error('verify_shape_exponent_bound:scenario', 'scenario must be deep|shallow|both');
        end
        bands = cell(1, numel(names));
        for i = 1:numel(names)
            bands{i} = struct('name', names{i}, ...
                              'env', local_envelope(names{i}, A_COV_BASE, H_BAR_MIN_PRIOR, ...
                                                    ENV_LO_MARGIN, ENV_HI_MARGIN));
        end
    end

    res = [];
    for il = 1:numel(laws)
        for ib = 1:numel(bands)
            r = local_one(char(laws(il)), bands{ib}, opts, THR_PASS, THR_TIGHT, FREEZE_MARGIN);
            if isempty(res); res = r; else; res(end+1) = r; end %#ok<AGROW>
        end
    end

    if opts.verbose; local_print_summary(res, THR_PASS, THR_TIGHT, FREEZE_MARGIN); end
end


% ======================================================================
function env = local_envelope(name, a_cov, h_bar_min_prior, lo_margin, hi_margin)
%LOCAL_ENVELOPE  The drivers' envelope rule on the canonical scenario.
    pc  = physical_constants();
    cfg = canonical_scenario(a_cov, h_bar_min_prior, name);
    env_lo = max(cfg.h_bottom / pc.R - lo_margin, h_bar_min_prior);
    env_hi = cfg.h_init / pc.R + hi_margin;
    env = [env_lo, env_hi];
end


% ======================================================================
function r = local_one(law, band, opts, thr_pass, thr_tight, freeze_margin)
%LOCAL_ONE  Criterion for one law on one envelope.
    N  = opts.n_grid;
    w  = linspace(band.env(1), band.env(2), N).';
    [c, dc] = local_truth(w, opts.axis);
    g  = w - 1;

    r = struct();
    r.law  = law;  r.band = band.name;  r.axis = opts.axis;  r.env = band.env;

    switch law
        case 'formc'
            r.names  = {'b'};
            r.theta0 = 8/9;
            th_eff   = -dc ./ (c - 1).^2;                 % slope reading (S1 b_true)
            r.alt    = struct('name', {'b_level'}, 'val', {c ./ ((c - 1) .* g)});
            % (D) driver: run_formC_b.m local_envelope_b_range -> b_half = max|b_true - 8/9|
            r.sqrtP_driver = max(abs(th_eff - 8/9));
            r.driver_src   = 'run_formC_b.m local_envelope_b_range: sup|b_true-8/9| on the envelope (re-implemented)';
            % (A) anchors: contact b = 1 (a''(1) = 1 exactly), far field 8/9
            r.anchor_near = 1;  r.anchor_far = 8/9;
            th_fun = @(cc, dcc, ww) -dcc ./ (cc - 1).^2;
        case 'formb'
            B0 = 9/8; P0 = 1; WS0 = 1;
            r.names  = {'b', 'p', 'ws'};
            r.theta0 = [B0, P0, WS0];
            b_eff  = (c - 1) .* g;
            p_eff  = -(g + B0) .* dc ./ (c .* (c - 1));
            ws_eff = w - B0 ./ (c - 1);
            th_eff = [b_eff, p_eff, ws_eff];
            [bj, pj] = local_formB_joint(w, c, dc, B0);
            r.alt = struct('name', {'b_joint', 'p_joint'}, 'val', {bj, pj});
            % (D) driver: run_formB_ws.m local_envelope_priors (b, p) + header ws 0.111
            r.sqrtP_driver = [max(abs(b_eff - B0)), max(abs(p_eff - P0)), 0.111];
            r.driver_src   = ['run_formB_ws.m local_envelope_priors: sup|b_eff-9/8|, sup|p_eff-1| ' ...
                              '(re-implemented); ws = 0.111 header value (tier t2; t1 locks ws)'];
            % (A) anchors: contact a = g  => p/b = 1 (b = 1 at p = 1; p = 9/8 at b = 9/8),
            %     ws = 1 at contact; far field (Faxen reciprocal) b = 9/8, p = 1, ws = 9/8
            r.anchor_near = [1, 9/8, 1];  r.anchor_far = [9/8, 1, 9/8];
            th_fun = @(cc, dcc, ww) [(cc - 1) .* (ww - 1), ...
                                     -((ww - 1) + B0) .* dcc ./ (cc .* (cc - 1)), ...
                                     ww - B0 ./ (cc - 1)];
        case 'expgain'
            r.names  = {'b'};
            r.theta0 = 1;
            th_eff   = -w .* dc ./ (c .* (c - 1));          % slope reading (Stage 2b)
            r.alt    = struct('name', {'b_level'}, 'val', {-log((c - 1) ./ c) ./ log(w)});
            r.sqrtP_driver = 0.10;
            r.driver_src   = 'controller default Pf_b_std = 0.10 (motion_control_law_5state_expgain_alg.m:164); tex Stage 2b: rounded from the truth sup 0.0942';
            r.anchor_near = 1;  r.anchor_far = 1;   % 1/K with K = 1, and 1
            th_fun = @(cc, dcc, ww) -ww .* dcc ./ (cc .* (cc - 1));
        otherwise
            error('verify_shape_exponent_bound:law', 'law must be formC|formB|expgain|all');
    end

    % ---- numerator: truth-curve sup, per component --------------------
    n_th   = numel(r.names);
    dtheta = th_eff - r.theta0;                     % signed profile
    r.profile = struct('w', w, 'dtheta', dtheta, 'theta_eff', th_eff);
    r.sup   = zeros(1, n_th);  r.w_sup = zeros(1, n_th);
    r.dtheta_lo = dtheta(1, :);  r.dtheta_hi = dtheta(end, :);
    r.sign_changes = zeros(1, n_th);
    for i = 1:n_th
        [r.sup(i), k] = max(abs(dtheta(:, i)));
        r.w_sup(i) = w(k);
        r.sign_changes(i) = sum(diff(sign(dtheta(:, i))) ~= 0);
    end

    % ---- (D) driver denominator ----------------------------------------
    r.ratio_driver = r.sup ./ r.sqrtP_driver;
    r.circular     = abs(r.sup - r.sqrtP_driver) < 1e-9;   % same supremum => circular
    r.verdict_driver = local_verdicts(r.ratio_driver, thr_pass, thr_tight);

    % ---- (A) two-anchor gap ---------------------------------------------
    r.sqrtP_asymA = abs(r.anchor_near - r.anchor_far);
    assert(all(min(abs(r.theta0 - r.anchor_near), abs(r.theta0 - r.anchor_far)) < 1e-12), ...
           'verify_shape_exponent_bound:seedNotAnchor', ...
           'construction (A) requires theta_0 to be one of the two published anchors.');
    r.ratio_asymA   = r.sup ./ r.sqrtP_asymA;               % Inf when degenerate
    r.verdict_asymA = local_verdicts(r.ratio_asymA, thr_pass, thr_tight);

    % ---- (B) next-order truth-free composite -----------------------------
    [cB, dcB, wx] = local_asymptote_composite(w);
    thB = th_fun(cB, dcB, w);
    r.crossover   = wx;
    r.sqrtP_asymB = max(abs(thB - r.theta0), [], 1);
    r.profile.theta_asymB = thB;
    r.ratio_asymB   = r.sup ./ r.sqrtP_asymB;
    r.verdict_asymB = local_verdicts(r.ratio_asymB, thr_pass, thr_tight);

    % ---- returned verdict: (A) unless degenerate, then (B) ----------------
    use_A = r.sqrtP_asymA > 0;
    r.ratio   = r.ratio_asymA;  r.ratio(~use_A) = r.ratio_asymB(~use_A);
    r.denom   = repmat({'A'}, 1, n_th);  r.denom(~use_A) = {'B'};
    r.verdict = local_verdicts(r.ratio, thr_pass, thr_tight);
    [r.ratio_norm, k] = max(r.ratio);              % prior-scaled inf-norm over components
    r.verdict_norm = r.verdict{k};
    r.margin = 1 ./ r.ratio;                       % ledger "margin" = bound / sup
    r.freeze_ok = all(r.margin >= freeze_margin);

    % ---- figure -----------------------------------------------------------
    r.fig = '';
    if opts.save_fig
        r.fig = local_figure(r, opts.out_dir);
    end

    if opts.verbose; local_print_one(r); end
end


% ======================================================================
function [c, dc] = local_truth(w, ax)
%LOCAL_TRUTH  Published truth curve and its derivative on the grid (offline).
    N = numel(w);  c = zeros(N, 1);  dc = zeros(N, 1);
    for i = 1:N
        [cpa, cpe, d] = calc_correction_functions(w(i), true);
        switch lower(ax)
            case 'perp'; c(i) = cpe; dc(i) = d.dc_perp_dh;
            case 'para'; c(i) = cpa; dc(i) = d.dc_para_dh;
            otherwise; error('verify_shape_exponent_bound:axis', 'axis must be perp|para');
        end
    end
end


% ======================================================================
function [c, dc, wx] = local_asymptote_composite(w)
%LOCAL_ASYMPTOTE_COMPOSITE  Truth-free composite of the two PUBLISHED expansions.
%   near (Cox & Brenner 1967):   c = 1/eps + (1/5) ln(1/eps) + 0.971,  eps = w - 1
%   far  (Faxen / Happel-Brenner 7-4.39, reciprocal): c = 1/(1 - 9/8 u + 1/2 u^3), u = 1/w
%   joined at their crossing wx (no free number). Assumptions 5-7.
    CB_LOG  = 1/5;   CB_CONST = 0.971;                   % Cox-Brenner
    LORENTZ = 9/8;   FAXEN3   = 1/2;                     % Faxen reciprocal form

    c_near  = @(ww) 1 ./ (ww - 1) + CB_LOG * log(1 ./ (ww - 1)) + CB_CONST;
    dc_near = @(ww) -1 ./ (ww - 1).^2 - CB_LOG ./ (ww - 1);
    Df      = @(ww) 1 - LORENTZ ./ ww + FAXEN3 ./ ww.^3;
    dDf     = @(ww) LORENTZ ./ ww.^2 - 3 * FAXEN3 ./ ww.^4;
    c_far   = @(ww) 1 ./ Df(ww);
    dc_far  = @(ww) -dDf(ww) ./ Df(ww).^2;

    wx = fzero(@(ww) c_near(ww) - c_far(ww), [1.3, 4.0]);
    near = w < wx;
    c  = zeros(size(w));  dc = zeros(size(w));
    c(near)   = c_near(w(near));    dc(near)  = dc_near(w(near));
    c(~near)  = c_far(w(~near));    dc(~near) = dc_far(w(~near));
end


% ======================================================================
function [bj, pj] = local_formB_joint(w, c, dc, b0)
%LOCAL_FORMB_JOINT  (b, p) that match level AND slope at each height, ws = 1.
%   level: (1 + g/b)^(-p) = (c-1)/c ;  slope: p = -(b + g) c' / (c (c-1)).
    N = numel(w);  bj = nan(N, 1);  pj = nan(N, 1);
    for i = 1:N
        g = w(i) - 1;  ci = c(i);  dci = dc(i);
        pfun = @(b) -(b + g) * dci / (ci * (ci - 1));
        f    = @(b) -pfun(b) * log(1 + g / b) - log((ci - 1) / ci);
        lo = 0.05; hi = 50;
        try
            if sign(f(lo)) ~= sign(f(hi))
                bj(i) = fzero(f, [lo, hi]);
            else
                bj(i) = fzero(f, b0);
            end
            pj(i) = pfun(bj(i));
        catch
        end
    end
end


% ======================================================================
function v = local_verdicts(ratio, thr_pass, thr_tight)
    v = cell(size(ratio));
    for i = 1:numel(ratio)
        if ~isfinite(ratio(i));      v{i} = 'DEGENERATE';
        elseif ratio(i) < thr_pass;  v{i} = 'PASS';
        elseif ratio(i) <= thr_tight; v{i} = 'TIGHT';
        else;                        v{i} = 'FAIL';
        end
    end
end


% ======================================================================
function local_print_one(r)
    fprintf('\n=== %s | band %s [%.3f, %.3f] | truth axis %s ===\n', ...
            upper(r.law), r.band, r.env(1), r.env(2), r.axis);
    fprintf('  sqrt(P[0]) driver source: %s\n', r.driver_src);
    if ~isnan(r.crossover)
        fprintf('  asymptote-B crossover (near/far expansions meet): w = %.4f\n', r.crossover);
    end
    fprintf('  %-4s %8s %9s %8s | %8s %7s %-10s %s | %8s %7s %-10s | %8s %7s %-10s | %s\n', ...
            'par', 'theta0', 'sup|d|', 'w@sup', 'sqrtP_D', 'ratioD', 'verdictD', 'circ', ...
            'sqrtP_A', 'ratioA', 'verdictA', 'sqrtP_B', 'ratioB', 'verdictB', 'RETURNED');
    for i = 1:numel(r.names)
        fprintf('  %-4s %8.5f %9.5f %8.3f | %8.5f %7.3f %-10s %-4s | %8.5f %7.3f %-10s | %8.5f %7.3f %-10s | %s (%s)\n', ...
                r.names{i}, r.theta0(i), r.sup(i), r.w_sup(i), ...
                r.sqrtP_driver(i), r.ratio_driver(i), r.verdict_driver{i}, local_yn(r.circular(i)), ...
                r.sqrtP_asymA(i), r.ratio_asymA(i), r.verdict_asymA{i}, ...
                r.sqrtP_asymB(i), r.ratio_asymB(i), r.verdict_asymB{i}, ...
                r.verdict{i}, r.denom{i});
        fprintf('       signed profile: d(env_lo) = %+.5f, d(env_hi) = %+.5f, zero crossings = %d\n', ...
                r.dtheta_lo(i), r.dtheta_hi(i), r.sign_changes(i));
    end
    for i = 1:numel(r.alt)
        v = r.alt(i).val;  ok = isfinite(v);
        if any(ok)
            fprintf('       alt reading %-8s : range [%.5f, %.5f]  (reported only)\n', ...
                    r.alt(i).name, min(v(ok)), max(v(ok)));
        end
    end
    fprintf('  RETURNED (prior-scaled inf-norm over components): ratio %.3f -> %s ; margin %.2fx (freeze gate 3x: %s)\n', ...
            r.ratio_norm, r.verdict_norm, 1 / r.ratio_norm, local_yn(r.freeze_ok));
    if ~isempty(r.fig); fprintf('  figure: %s\n', r.fig); end
end


function s = local_yn(tf)
    if tf; s = 'YES'; else; s = 'no'; end
end


% ======================================================================
function local_print_summary(res, thr_pass, thr_tight, freeze_margin)
    fprintf('\n================ SUMMARY (verdict thresholds: PASS < %.1f, TIGHT <= %.1f, FAIL > %.1f; freeze gate margin >= %.0fx) ================\n', ...
            thr_pass, thr_tight, thr_tight, freeze_margin);
    fprintf('%-8s %-8s %-4s %8s %8s %8s | %8s %7s %-10s | %8s %7s %-10s | %8s %7s %-10s | %s\n', ...
            'law', 'band', 'par', 'theta0', 'sup|d|', 'w@sup', 'sqrtP_D', 'ratioD', 'verdictD', ...
            'sqrtP_A', 'ratioA', 'verdictA', 'sqrtP_B', 'ratioB', 'verdictB', 'RETURNED');
    for j = 1:numel(res)
        r = res(j);
        for i = 1:numel(r.names)
            fprintf('%-8s %-8s %-4s %8.5f %8.5f %8.3f | %8.5f %7.3f %-10s | %8.5f %7.3f %-10s | %8.5f %7.3f %-10s | %s(%s)\n', ...
                    r.law, r.band, r.names{i}, r.theta0(i), r.sup(i), r.w_sup(i), ...
                    r.sqrtP_driver(i), r.ratio_driver(i), [r.verdict_driver{i} local_circ_tag(r.circular(i))], ...
                    r.sqrtP_asymA(i), r.ratio_asymA(i), r.verdict_asymA{i}, ...
                    r.sqrtP_asymB(i), r.ratio_asymB(i), r.verdict_asymB{i}, ...
                    r.verdict{i}, r.denom{i});
        end
    end
    fprintf('(D = driver width, circular where flagged *; A = two-anchor gap, truth-free; B = next-order expansions, truth-free bracket)\n');
end


function s = local_circ_tag(tf)
    if tf; s = '*'; else; s = ''; end
end


% ======================================================================
function fpath = local_figure(r, out_dir)
%LOCAL_FIGURE  theta_eff(w) with the anchor and the +/- sqrt(P[0]) bands.
%   Style: no grid, no title, box on, legend northoutside horizontal,
%   True = red, anchor/prior = blue, exportgraphics 150 dpi
%   (template: plot_var_ahat_6state.m).
    FS = 14; LFS = 11; AXLW = 1.2;
    COL_TRUE = [0.85 0.10 0.10];
    COL_HAT  = [0.10 0.30 0.85];
    COL_B    = [0.45 0.45 0.45];
    n = numel(r.names);
    f = figure('Position', [80 80 1000 320 * n + 120], 'Color', 'w', ...
               'NumberTitle', 'off', 'Visible', 'off');
    w = r.profile.w;
    for i = 1:n
        ax = subplot(n, 1, i); hold(ax, 'on');
        th0 = r.theta0(i);
        sA = r.sqrtP_asymA(i);  sD = r.sqrtP_driver(i);  sB = r.sqrtP_asymB(i);
        hD = fill([w; flipud(w)], [th0 + sD * ones(size(w)); flipud(th0 - sD * ones(size(w)))], ...
                  COL_HAT, 'FaceAlpha', 0.15, 'EdgeColor', 'none', ...
                  'DisplayName', sprintf('anchor \\pm \\surdP[0] driver (%.4f)', sD));
        hh = hD;
        if sA > 0
            hA = plot(w, (th0 + sA) * ones(size(w)), '--', 'Color', COL_HAT, 'LineWidth', 1.4, ...
                      'DisplayName', sprintf('\\pm \\surdP[0] asym-A (%.4f)', sA));
            plot(w, (th0 - sA) * ones(size(w)), '--', 'Color', COL_HAT, 'LineWidth', 1.4, ...
                 'HandleVisibility', 'off');
            hh(end+1) = hA; %#ok<AGROW>
        end
        hB = plot(w, r.profile.theta_asymB(:, i), ':', 'Color', COL_B, 'LineWidth', 1.4, ...
                  'DisplayName', sprintf('asym-B composite (\\surdP %.4f)', sB));
        h0 = plot(w, th0 * ones(size(w)), '-', 'Color', COL_HAT, 'LineWidth', 1.6, ...
                  'DisplayName', sprintf('\\theta_0 = %.4f', th0));
        ht = plot(w, r.profile.theta_eff(:, i), '-', 'Color', COL_TRUE, 'LineWidth', 2.0, ...
                  'DisplayName', sprintf('\\theta_{eff} (truth), sup|d| = %.4f', r.sup(i)));
        plot(r.w_sup(i), r.profile.theta_eff(w == r.w_sup(i), i), 'o', 'Color', COL_TRUE, ...
             'MarkerFaceColor', COL_TRUE, 'MarkerSize', 6, 'HandleVisibility', 'off');
        hh = [hh, hB, h0, ht]; %#ok<AGROW>
        set(ax, 'XScale', 'log', 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
        grid(ax, 'off');
        xlim(ax, [r.env(1), r.env(2)]);
        yl = [min(r.profile.theta_eff(:, i)), max(r.profile.theta_eff(:, i))];
        pad = max(sD, sA) * 1.5 + 0.05 * max(1e-3, diff(yl));
        ylim(ax, [min(yl(1), th0 - pad), max(yl(2), th0 + pad)]);
        ylabel(ax, sprintf('%s_{eff}', r.names{i}), 'FontSize', FS, 'FontWeight', 'bold');
        legend(ax, hh, 'Location', 'northoutside', 'Orientation', 'horizontal', ...
               'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'NumColumns', 3);
        if i == n
            xlabel(ax, 'w\_bar = h / R', 'FontSize', FS, 'FontWeight', 'bold');
        end
    end
    fpath = fullfile(out_dir, sprintf('shape_bound_%s_%s_%s.png', r.law, r.band, r.axis));
    exportgraphics(f, fpath, 'Resolution', 150);
    close(f);
end
