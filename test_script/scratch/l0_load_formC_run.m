function r = l0_load_formC_run(src, opts)
%L0_LOAD_FORMC_RUN  Layer-0 loader: one unified view of a formC_b run set.
%
%   r = l0_load_formC_run(src)
%   r = l0_load_formC_run(src, opts)
%
% STATUS: ACTIVE (scratch, L0 instrument) | PURPOSE: read any formC_b run
%   product -- a live per-run struct or one of the known .mat layouts --
%   into ONE field layout, with the init-only row detected and dropped, the
%   physical gain normalised by the run's own a_nom, and the constants taken
%   from the run itself (never from user_config, which this file does not
%   call) | EXPIRES: never while formC_b audits are open.
%
% ACCEPTED src
%   (i)   per-run struct from run_formC_b: out.runs{q}, or the single-seed
%         test-ladder form simOut = run_formC_b(cfg, topts). Fingerprint:
%         has fields a_true_out, a_bar_hat_out, a_nom, tout.
%   (ii)  the driver's aggregate struct out (has .runs) -- all runs stacked.
%   (iii) char path to a .mat file in one of the known layouts:
%         L1 'budget'  : <chan>_out (N x 3 x ns), t, K, a_nom [, P, opts_used]
%                        (baseline_budget_100.mat, fdet_off_budget_100.mat,
%                         stageA_bmid_base_100.mat, stageA_bmid_apknown_100.mat
%                         -- the stageA pair adds b_hat_out, gate_out,
%                         a_prime_true_out and opts_used)
%         L2 'truea'   : A_xm, A_tr, A_ht, D_xr, R2 (N x 3 x ns, PHYSICAL
%                        a_xm/a_true), t, K, P, a_nom   (truea_100.mat)
%         L3 'stack'   : A_wm, A_tr, A_ht, R2_u, gate (N x ns, ONE axis,
%                        already NORMALISED), t, cc, cfg, seeds, a_nom, ax
%                        (stack_deep400.mat -- init row already removed)
%         L4 'driver'  : variable out with out.runs (run_formC_b_*.mat)
%
% OUTPUT r -- every time series is N x 3 x ns (axis in column, seed in page;
%   ns = 1 for a single run). Axes not present in the source (L3 keeps z
%   only) are NaN and listed in r.meta.axes_present. Gain quantities are in
%   NORMALISED units [-] (physical / a_nom) unless stated otherwise.
%   r.t            N x 1 [s]          time after the init row is dropped
%   r.a_true_norm  N x 3 x ns [-]     a_true / a_nom (a_true_out is PHYSICAL
%                                     um/pN in the driver). ALIGNMENT: the
%                                     driver evaluates a_true(k) at p_curr
%                                     BEFORE step_dynamics and writes
%                                     p_true(k) AFTER it, so a_true_norm(k)
%                                     belongs to p_true(k-1). Not re-aligned
%                                     here (recorded in meta.alignment).
%   r.a_bar_hat    N x 3 x ns [-]     posterior normalised gain estimate
%   r.a_xm         N x 3 x ns [-]     raw gain readout a_bar_wm = a_xm/a_nom
%   r.R2           N x 3 x ns [-]     R(2,2) the filter actually used
%   present when the source carries them (else absent -- test with isfield):
%   r.K_a_y1, r.K_a_y2     Kalman gain rows into the gain state from y1 / y2
%   r.K_b_y1, r.K_b_y2     same into slot 5 (b)
%   r.K_dx_y1, r.K_dx_y2   same into the position state
%   r.innov_y1, r.innov_y2 innovations (y1 in [-] = dw units; y2 in [-])
%   r.P41                  cross-covariance P(4,1) [-]
%   r.b_hat                slot-5 state (b / da alias, per the driver)
%   r.P_b                  VARIANCE of slot 5 [-]   (= P_b_out.^2; the
%                          driver logs sqrt(P) -- see meta.P_convention)
%   r.P_a                  VARIANCE of the gain state, NORMALISED [-]
%                          (= (P_a_out / a_nom).^2)
%   r.P_p, r.P_ws          variances of slots 6 / 7 [-]
%   r.gate                 logical, y2 gate active per axis
%   r.p_true               N x 3 x ns [um]  position AFTER step k
%   r.p_d                  N x 3 x ns [um]  command handed to the controller
%                                           at step k (pd_k), i.e. p_d(t_k)
%   r.p_m                  N x 3 x ns [um]  noisy measurement of p_true(k)
%   r.h_bar                N x 1 x ns [-]   controller's measured h_bar
%   r.h_bar_true           N x 1 x ns [-]   true h_bar at the controller call
%   r.h_bar_d              N x 1 x ns [-]   commanded h_bar
%   r.dx_r                 N x 3 x ns [um]  IIR residual the readout squares
%   r.dh_m                 N x 3 x ns [um]  delayed tracking error fed to
%                                           the controller (= delta_x_m)
%   r.a_prime              N x 3 x ns [-]   d a_bar / d h_bar estimate
%   r.a_prime_true         N x 3 x ns [-]   same, exact curve
%   r.f_d, r.F_th          N x 3 x ns [pN]  control / thermal force
%   r.ws_hat, r.p_hat      slots 7 / 6
%   r.lam_b                adaptive forgetting actually applied (1 = off)
%   r.meta
%     .source        'struct' | 'driver-out' | file path
%     .layout        'run' | 'driver' | 'budget' | 'truea' | 'stack'
%     .ctrl_const    the run's own constants (ctrl_const / K / cc)
%     .params_value  params.Value when present (else [])
%     .cfg           scenario config when present (else [])
%     .a_nom         [um/pN] normaliser used  (= Ts/gamma_N = a_o*R)
%     .a_nom_check   relative mismatch vs Ts/gamma_N rebuilt from
%                    params_value when available (NaN otherwise)
%     .band          'deep' | 'shallow' | 'unknown', with .band_source
%     .seeds         seed list; .seeds_source says whether it is stored in
%                    the file or assumed from the producing script
%     .n_seeds, .N, .Ts
%     .axes_present  axes with data (1x3 or 3 for the stack)
%     .init_rows_dropped  0 or 1 (asserted)
%     .init_row      values of the non-zero channels on the dropped row
%                    (a_bar_hat seed, P[0] widths) -- the P[0] contract
%                    lives on that row, so it is kept here
%     .init_zero_channels  channel names used for the all-zero fingerprint
%     .alignment     text, the a_true / p_true off-by-one note
%     .P_convention  text, the sqrt(P) logging note
%     .normalized_in_source  true for the stack layout
%
% INIT-ROW RULE. The controller's first call is init-only: it returns the
%   seeds and prior widths but every RUN-TIME diag channel (a_xm, R2, K_*,
%   innov_*, P41, dx_r, dh_m, h_bar, a_prime) is exactly zero. The row is
%   detected on those channels ONLY -- a_bar_hat, P_a, P_b, b_hat, a_true,
%   p_true are NOT zero there (the init branch writes the seeds, controller
%   ~line 827) and must not be used as the fingerprint. a_xm is REQUIRED to
%   be present and votes first. Exactly 0 or 1 rows are dropped; any
%   all-zero row further down is an error.
%
% NOT ASSERTED: equality of a_true across arms of a paired run. A different
%   applied force realises a different trajectory, so a_true differs at the
%   1e-4 level between arms on the same seed -- expected, not a defect.
%
% opts
%   .verbose   (true)  one-line summary
%   .seeds     ([])    override / supply the seed list (files L1/L2 do not
%                      store it)
%   .keep_init (false) keep the init row (meta.init_rows_dropped = 0 then)

    if nargin < 2 || isempty(opts); opts = struct(); end
    if ~isfield(opts, 'verbose');   opts.verbose   = true;  end
    if ~isfield(opts, 'seeds');     opts.seeds     = [];    end
    if ~isfield(opts, 'keep_init'); opts.keep_init = false; end

    % channels that are exactly zero on the init-only call (and never all
    % zero on a live step). Only these vote.
    INIT_ZERO_CH = {'a_xm', 'R2', 'innov_y1', 'innov_y2', 'K_a_y1', 'K_a_y2', ...
                    'P41', 'dx_r', 'dh_m', 'h_bar', 'a_prime'};

    r = struct();
    meta = struct();
    meta.alignment = ['a_true_norm(k) evaluated BEFORE step k, p_true(k) written AFTER it: ' ...
                      'a_true_norm(k) belongs to p_true(k-1). Not re-aligned here.'];
    meta.P_convention = 'driver logs sqrt(P) in P_*_out; r.P_* are VARIANCES (squared back).';
    meta.init_zero_channels = INIT_ZERO_CH;
    meta.normalized_in_source = false;

    % ------------------------------------------------------------------
    % Dispatch on the source
    % ------------------------------------------------------------------
    if ischar(src) || isstring(src)
        src = char(src);
        assert(exist(src, 'file') == 2, 'l0_load_formC_run:noFile', 'file not found: %s', src);
        L = load(src);
        meta.source = src;
        if isfield(L, 'out') && isstruct(L.out) && isfield(L.out, 'runs')
            [r, meta] = local_from_runs(L.out.runs, meta);
            meta.layout = 'driver';
            meta.seeds = L.out.seeds(:).';  meta.seeds_source = 'stored (out.seeds)';
            if isfield(L.out, 'cfg'); meta.cfg = L.out.cfg; end
        elseif isfield(L, 'A_wm') && isfield(L, 'cc')
            [r, meta] = local_from_stack(L, meta);
            meta.layout = 'stack';
        elseif isfield(L, 'A_xm') && isfield(L, 'A_tr')
            [r, meta] = local_from_truea(L, meta);
            meta.layout = 'truea';
        elseif isfield(L, 'a_true_out') && isfield(L, 'K')
            [r, meta] = local_from_budget(L, meta);
            meta.layout = 'budget';
        else
            error('l0_load_formC_run:unknownLayout', ...
                  'unrecognised .mat layout in %s (vars: %s)', src, strjoin(fieldnames(L).', ' '));
        end
    elseif isstruct(src) && isfield(src, 'runs')
        meta.source = 'driver-out';
        [r, meta] = local_from_runs(src.runs, meta);
        meta.layout = 'driver';
        if isfield(src, 'seeds'); meta.seeds = src.seeds(:).'; meta.seeds_source = 'stored (out.seeds)'; end
        if isfield(src, 'cfg');   meta.cfg = src.cfg; end
    elseif isstruct(src) && isfield(src, 'a_true_out') && isfield(src, 'tout')
        meta.source = 'struct';
        [r, meta] = local_from_runs({src}, meta);
        meta.layout = 'run';
    else
        error('l0_load_formC_run:badSrc', 'src must be a per-run struct, a driver out struct, or a .mat path.');
    end

    if ~isempty(opts.seeds)
        meta.seeds = opts.seeds(:).';  meta.seeds_source = 'caller override';
    end
    if ~isfield(meta, 'seeds'); meta.seeds = []; meta.seeds_source = 'unknown'; end

    % ------------------------------------------------------------------
    % Init-row detection (0 or 1 rows, asserted)
    % ------------------------------------------------------------------
    vote_ch = INIT_ZERO_CH(isfield(r, INIT_ZERO_CH));
    assert(isfield(r, 'a_xm'), 'l0_load_formC_run:noAxm', ...
           'a_xm is required for init-row detection (it is exactly zero on the init call).');
    N0 = numel(r.t);
    zero_row = true(N0, 1);
    any_finite = false;
    for i = 1:numel(vote_ch)
        X = r.(vote_ch{i});
        X = X(:, :);                      % N x (3*ns)
        any_finite = any_finite || any(isfinite(X(:)));
        zero_row = zero_row & all(X == 0 | isnan(X), 2);
    end
    assert(any_finite, 'l0_load_formC_run:allNaNVote', 'fingerprint channels are all NaN.');
    % rows where EVERY vote channel is exactly zero across all axes/seeds
    % (NaN-padded axes of the stack layout count as zero).
    all_zero_rows = find(zero_row);
    n_init = 0;
    meta.init_row = struct();
    if ~isempty(all_zero_rows)
        assert(isequal(all_zero_rows(:).', 1), 'l0_load_formC_run:zeroRows', ...
               'all-zero fingerprint rows at %s -- expected only row 1 (init call) or none.', ...
               mat2str(all_zero_rows(:).'));
        n_init = 1;
    end
    if N0 >= 2
        assert(~zero_row(2), 'l0_load_formC_run:row2zero', 'row 2 is all-zero on the fingerprint channels.');
    end
    if n_init == 1
        % keep what the init row carries (seeds / prior widths)
        keep = {'a_bar_hat', 'P_a', 'P_b', 'P_p', 'P_ws', 'b_hat', 'p_hat', 'ws_hat', 'a_true_norm'};
        for i = 1:numel(keep)
            if isfield(r, keep{i}); meta.init_row.(keep{i}) = squeeze(r.(keep{i})(1, :, :)); end
        end
        meta.init_row.t = r.t(1);
        if ~opts.keep_init
            fn = fieldnames(r);
            for i = 1:numel(fn)
                X = r.(fn{i});
                if (isnumeric(X) || islogical(X)) && size(X, 1) == N0 && N0 > 1
                    r.(fn{i}) = X(2:end, :, :);
                end
            end
            meta.init_rows_dropped = 1;
        else
            meta.init_rows_dropped = 0;
            meta.init_row.note = 'init row detected but KEPT (opts.keep_init)';
        end
    else
        meta.init_rows_dropped = 0;
    end
    meta.init_row_detected = n_init;

    % ------------------------------------------------------------------
    % Band, sizes, a_nom check
    % ------------------------------------------------------------------
    meta.N = numel(r.t);
    meta.n_seeds = size(r.a_true_norm, 3);
    meta.Ts = r.t(2) - r.t(1);
    if ~isfield(meta, 'axes_present')
        meta.axes_present = find(squeeze(any(any(isfinite(r.a_true_norm), 1), 3)).');
    end
    [meta.band, meta.band_source] = local_band(meta, r);
    meta.a_nom_check = NaN;
    if isfield(meta, 'params_value') && ~isempty(meta.params_value) ...
            && isfield(meta.params_value, 'common') ...
            && isfield(meta.params_value.common, 'Ts') && isfield(meta.params_value.common, 'gamma_N')
        a_nom_re = meta.params_value.common.Ts / meta.params_value.common.gamma_N;
        meta.a_nom_check = a_nom_re / meta.a_nom - 1;
    end
    if isempty(meta.seeds); meta.seeds = 1:meta.n_seeds; meta.seeds_source = 'unknown -> 1:n_seeds placeholder'; end
    r.meta = meta;

    if opts.verbose
        fprintf(['[l0_load] %s | layout %s | N %d | seeds %d (%s) | axes %s | band %s (%s) | ' ...
                 'init rows dropped %d | a_nom %.6g um/pN (check %+.1e)\n'], ...
                meta.source, meta.layout, meta.N, meta.n_seeds, meta.seeds_source, ...
                mat2str(meta.axes_present(:).'), meta.band, meta.band_source, ...
                meta.init_rows_dropped, meta.a_nom, meta.a_nom_check);
    end
end

% ======================================================================
function [r, meta] = local_from_runs(runs, meta)
%LOCAL_FROM_RUNS  Stack driver per-run structs (cell) into N x 3 x ns.
    if ~iscell(runs); runs = {runs}; end
    ns = numel(runs);
    s1 = runs{1};
    N  = numel(s1.tout);
    a_nom = s1.a_nom;
    r = struct();
    r.t = s1.tout(:);
    % map: output field <- driver field, scale ('anom' = divide by a_nom,
    %      'sq_anom' = (x/a_nom)^2, 'sq' = x^2, 1 = as is)
    MAP = { ...
        'a_true_norm',  'a_true_out',       'anom'; ...
        'a_bar_hat',    'a_bar_hat_out',    1; ...
        'a_xm',         'a_xm_out',         'anom'; ...
        'R2',           'R2_out',           1; ...
        'K_a_y1',       'K_a_y1_out',       1; ...
        'K_a_y2',       'K_a_y2_out',       1; ...
        'K_b_y1',       'K_b_y1_out',       1; ...
        'K_b_y2',       'K_b_y2_out',       1; ...
        'K_dx_y1',      'K_dx_y1_out',      1; ...
        'K_dx_y2',      'K_dx_y2_out',      1; ...
        'innov_y1',     'innov_y1_out',     1; ...
        'innov_y2',     'innov_y2_out',     1; ...
        'P41',          'P41_out',          1; ...
        'b_hat',        'b_hat_out',        1; ...
        'P_b',          'P_b_out',          'sq'; ...
        'P_a',          'P_a_out',          'sq_anom'; ...
        'P_p',          'P_p_out',          'sq'; ...
        'P_ws',         'P_ws_out',         'sq'; ...
        'gate',         'gate_out',         1; ...
        'p_true',       'p_true_out',       1; ...
        'p_d',          'p_d_out',          1; ...
        'p_m',          'p_m_out',          1; ...
        'h_bar',        'h_bar_out',        1; ...
        'h_bar_true',   'h_bar_true_out',   1; ...
        'h_bar_d',      'h_bar_d_out',      1; ...
        'dx_r',         'dx_r_out',         1; ...
        'dh_m',         'dh_m_out',         1; ...
        'a_prime',      'a_prime_out',      'anom'; ...
        'a_prime_true', 'a_prime_true_out', 'anom'; ...
        'f_d',          'f_d_out',          1; ...
        'F_th',         'F_th_out',         1; ...
        'ws_hat',       'ws_hat_out',       1; ...
        'p_hat',        'p_hat_out',        1; ...
        'lam_b',        'lam_b_out',        1; ...
        'Q44',          'Q44_out',          1; ...
        'Q33',          'Q33_out',          1; ...
        'f_bar',        'f_bar_out',        1; ...
        'dws_y1',       'dws_y1_out',       1; ...
        'dws_y2',       'dws_y2_out',       1};
    for i = 1:size(MAP, 1)
        if ~isfield(s1, MAP{i, 2}); continue; end
        nc = size(s1.(MAP{i, 2}), 2);
        X = zeros(N, nc, ns);
        for q = 1:ns
            assert(runs{q}.a_nom == a_nom, 'l0_load_formC_run:aNomMismatch', 'a_nom differs across runs');
            X(:, :, q) = double(runs{q}.(MAP{i, 2}));
        end
        r.(MAP{i, 1}) = local_scale(X, MAP{i, 3}, a_nom);
    end
    if isfield(r, 'gate'); r.gate = r.gate == 1; end
    meta.a_nom = a_nom;
    meta.ctrl_const = s1.ctrl_const;
    meta.params_value = [];  meta.cfg = [];
    if isfield(s1, 'meta')
        if isfield(s1.meta, 'params_value'); meta.params_value = s1.meta.params_value; end
        if isfield(s1.meta, 'config');       meta.cfg = s1.meta.config; end
        seeds = zeros(1, ns);
        for q = 1:ns; seeds(q) = runs{q}.meta.seed; end
        meta.seeds = seeds;  meta.seeds_source = 'stored (run.meta.seed)';
    end
    if isfield(s1, 'R'); meta.R = s1.R; end
end

% ----------------------------------------------------------------------
function [r, meta] = local_from_budget(L, meta)
%LOCAL_FROM_BUDGET  baseline_budget_100 / fdet_off_budget_100 layout.
    a_nom = L.a_nom;
    r = struct();
    r.t = L.t(:);
    MAP = { ...
        'a_true_norm', 'a_true_out',    'anom'; ...
        'a_bar_hat',   'a_bar_hat_out', 1; ...
        'a_xm',        'a_xm_out',      'anom'; ...
        'R2',          'R2_out',        1; ...
        'K_a_y1',      'K_a_y1_out',    1; ...
        'K_a_y2',      'K_a_y2_out',    1; ...
        'innov_y1',    'innov_y1_out',  1; ...
        'innov_y2',    'innov_y2_out',  1; ...
        'P41',         'P41_out',       1; ...
        'dx_r',        'dx_r_out',      1; ...
        'a_prime',     'a_prime_out',   'anom'; ...
        'a_prime_true', 'a_prime_true_out', 'anom'; ...
        'b_hat',       'b_hat_out',     1; ...
        'gate',        'gate_out',      1};
    for i = 1:size(MAP, 1)
        if isfield(L, MAP{i, 2})
            r.(MAP{i, 1}) = local_scale(double(L.(MAP{i, 2})), MAP{i, 3}, a_nom);
        end
    end
    if isfield(r, 'gate'); r.gate = r.gate == 1; end
    meta.a_nom = a_nom;
    meta.ctrl_const = L.K;
    meta.params_value = [];  meta.cfg = [];
    if isfield(L, 'P'); meta.params_value = L.P; end
    if isfield(L, 'opts_used'); meta.opts_used = L.opts_used; end
    ns = size(r.a_true_norm, 3);
    meta.seeds = 1:ns;
    meta.seeds_source = ['ASSUMED 1:ns from the producing scripts (run_fdet_off_arm_z.m / ' ...
                         'run_stageA_apknown_pair.m convention: seeds 1:100); not stored in file'];
end

% ----------------------------------------------------------------------
function [r, meta] = local_from_truea(L, meta)
%LOCAL_FROM_TRUEA  truea_100 layout (run_formC_b_true_a_arm.m).
    a_nom = L.a_nom;
    r = struct();
    r.t = L.t(:);
    r.a_true_norm = double(L.A_tr) / a_nom;
    r.a_bar_hat   = double(L.A_ht);
    r.a_xm        = double(L.A_xm) / a_nom;
    r.R2          = double(L.R2);
    if isfield(L, 'D_xr'); r.dx_r = double(L.D_xr); end
    meta.a_nom = a_nom;
    meta.ctrl_const = L.K;
    meta.params_value = [];  meta.cfg = [];
    if isfield(L, 'P'); meta.params_value = L.P; end
    ns = size(r.a_true_norm, 3);
    meta.seeds = 1:ns;
    meta.seeds_source = 'ASSUMED 1:ns from the producing script (run_formC_b_true_a_arm.m: seeds 1:100); not stored in file';
    meta.arm_note = 'a_ctrl_override = ''true'' arm (control law fed the exact gain; EKF still estimates)';
end

% ----------------------------------------------------------------------
function [r, meta] = local_from_stack(L, meta)
%LOCAL_FROM_STACK  stack_deep400 layout (verify_formC_am_r22 save_stack):
%   one axis, already normalised, init row already removed by the producer.
    ax = L.ax;
    [N, ns] = size(L.A_wm);
    r = struct();
    r.t = L.t(:);
    r.a_true_norm = local_axpad(double(L.A_tr), ax, N, ns);
    r.a_bar_hat   = local_axpad(double(L.A_ht), ax, N, ns);
    r.a_xm        = local_axpad(double(L.A_wm), ax, N, ns);   % A_wm = a_xm / a_nom
    r.R2          = local_axpad(double(L.R2_u), ax, N, ns);
    if isfield(L, 'gate'); r.gate = local_axpad(double(L.gate), ax, N, ns) == 1; end
    meta.a_nom = L.a_nom;
    meta.ctrl_const = L.cc;
    meta.params_value = [];
    meta.cfg = L.cfg;
    meta.seeds = L.seeds(:).';  meta.seeds_source = 'stored (seeds)';
    meta.axes_present = ax;
    meta.normalized_in_source = true;
    meta.stack_note = 'producer already dropped the init row (N = 7680) and normalised by a_nom';
    if isfield(L, 'kappa_T'); meta.kappa_T = L.kappa_T; end
    if isfield(L, 's2n_nd');  meta.s2n_nd  = L.s2n_nd;  end
    if isfield(L, 'xi_bar');  meta.xi_bar  = L.xi_bar;  end
end

% ----------------------------------------------------------------------
function X = local_scale(X, how, a_nom)
    if ischar(how)
        switch how
            case 'anom';    X = X / a_nom;
            case 'sq';      X = X.^2;
            case 'sq_anom'; X = (X / a_nom).^2;
            otherwise; error('l0_load_formC_run:scale', 'bad scale tag %s', how);
        end
    end
end

function X = local_axpad(A, ax, N, ns)
%LOCAL_AXPAD  N x ns single-axis matrix -> N x 3 x ns with NaN elsewhere.
    X = nan(N, 3, ns);
    X(:, ax, :) = reshape(A, [N, 1, ns]);
end

function [band, src] = local_band(meta, r)
%LOCAL_BAND  'deep' (trough w_bar 1.10) vs 'shallow' (2.00), from the config
%   when stored, else inferred from the minimum true normalised gain on z.
    band = 'unknown'; src = 'none';
    R = NaN;
    if isfield(meta, 'R'); R = meta.R; end
    if isnan(R) && ~isempty(meta.params_value) && isfield(meta.params_value, 'common') ...
            && isfield(meta.params_value.common, 'R')
        R = meta.params_value.common.R;
    end
    hb = NaN;
    if ~isempty(meta.cfg) && isfield(meta.cfg, 'h_bottom')
        hb = meta.cfg.h_bottom;
    elseif ~isempty(meta.params_value) && isfield(meta.params_value, 'traj') ...
            && isfield(meta.params_value.traj, 'h_bottom')
        hb = meta.params_value.traj.h_bottom;
    end
    if isfinite(hb) && isfinite(R)
        w_tr = hb / R;
        if abs(w_tr - 1.10) < 0.05;     band = 'deep';
        elseif abs(w_tr - 2.00) < 0.05; band = 'shallow';
        else;                           band = sprintf('other (trough w_bar %.3f)', w_tr);
        end
        src = 'config h_bottom / R';
        return;
    end
    if isfield(r, 'a_true_norm') && size(r.a_true_norm, 2) >= 3
        az = r.a_true_norm(:, 3, :);
        amin = min(az(isfinite(az)));
        if isempty(amin); return; end
        % deep trough: a_bar(1.10) ~ 0.09 (particle reaches ~1.085);
        % shallow trough: a_bar(2.00) ~ 0.47
        if amin < 0.25;     band = 'deep';
        elseif amin < 0.7;  band = 'shallow';
        else;               band = 'far-only';
        end
        src = sprintf('inferred from min a_true_norm(z) = %.4f', amin);
    end
end
