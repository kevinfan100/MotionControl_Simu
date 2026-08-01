% STATUS: ACTIVE | SSOT derivation: reference/eq17_analysis/derivation/formB_ws.tex
function [f_d, ekf_out, diag] = motion_control_law_formB_ws(del_pd, pd, p_m, params, ctrl_const, a_ctrl_override)
%MOTION_CONTROL_LAW_FORMB_WS  Per-axis 7-state EKF eq17 controller whose motion
%   gain follows the Form B (unknown wall position) gain law
%
%       a_bar(w_bar - w_bar_s, b, p) = 1 - (1 + (w_bar - w_bar_s)/b)^(-p)
%       a_w = a_o * a_bar ,   a_o = Ts/(gamma_N*R)  [1/pN]  (fixed, not a state)
%
%   Authoritative spec: reference/eq17_analysis/derivation/formB_ws.tex (S1-S8)
%   + formB_ws_ref.tex (conventions, declared truncations, S9-S11 contracts).
%   Coding checklist: auto-memory project_formB_ws_coding_checklist_2026-07-31.
%
%   Fork of motion_control_law_5state_expgain (7b structure: gain-as-state,
%   y2 whitening, fdet deterministic mirror, delay buffers). DARE init, Joseph
%   update, sequential scalar updates and the gate/clamp precedents follow
%   motion_control_law_5state_expgain_alg.
%
%   FULLY NORMALIZED INTERNALS (formB_ws_ref Conventions): every internal
%   quantity is dimensionless -- lengths divided by R, gain by a_o, forces
%   multiplied by a_o (f_bar = a_o*f, so a_bar*f_bar = a*f identically).
%   The only dimensionless thermal constant inside the loop is
%
%       kappa_T = 4*kB*T*a_o/R                                   [-]
%
%   a_o crosses the boundary ONLY at: the kappa_T / a_disp definitions ([U0]),
%   the force output conversion f_d = f_bar_d/a_o ([U3]), and the display
%   layer ([U4]). It appears NOWHERE inside the filter loop.
%       [U1] inputs pd / p_m / del_pd [um]          -> /R
%       [U2] sigma2_n_s [um^2]                      -> /R^2
%       [U3] force output f_bar_d [-]               -> /a_o  [pN]
%       [U4] displayed gains                        -> *a_disp (= a_o*R) [um/pN]
%
%   Per-axis state, d = 2 measurement delay, e_theta = true - estimate:
%
%       x = [dw_1; dw_2; dw_3; a_bar_w; b_w; p_w; w_s]
%
%   dw_1 = delta_w[k-2] (measured) ... dw_3 = delta_w[k] (current), in units
%   of R; a_bar_w = normalized gain in [0,1]; b_w (length, units of R),
%   p_w (exponent), w_s (contact position, units of R) are constants
%   (Q55 = Q66 = Q77 = 0).
%
%   Gain rate and parameter Jacobians (S2, log-derivative factorization,
%   evaluated at (w_bar_d - ws_hat, b_hat, p_hat)):
%       a_bar' = (p/b)*(1 + g/b)^(-p-1) ,          g = w_bar_d - ws_hat
%       J_b    = [-1/b + (p+1)*g/(b*(g+b))] * a_bar'
%       J_p    = [1/p - ln(1 + g/b)]        * a_bar'
%       J_ws   = [(p+1)/(g+b)]              * a_bar'
%
%   Estimator PREDICT (S5): dw_hat contracts with lambda_c alone;
%       a_bar[k+1] = a_bar + a_bar' * ( Delta_wbar_d + (1-lc)*dw_3_hat )
%   and the three constants carry over. a_bar is NEVER re-anchored from
%   (b,p,ws) after init (no hybrid 7a behavior).
%
%   F_e (7x7, S6): row 3 = [0 0 lc -F_dw 0 0 0]; row 4 =
%       [0 0 (1-lc)*a_bar'  1+a_bar'*F_dw  J_b*M  J_p*M  J_ws*M],
%   M = Delta_wbar_d + (1-lc)*dw_3_hat; rows 5-7 identity. Unlike the expgain
%   ancestor there is NO M*(da'/da) term in F_e(4,4): Form B's a_bar' does not
%   depend on the state a_bar_w, so F_e(4,4) = 1 + a_bar'*F_dw exactly.
%
%   Measurements (S7). The gain readout is computed in ONE step with kappa_T
%   (never physical-then-divide):
%       a_bar_wm = (sigma2_dwr_hat - C_n*sigma2_nw) / (C_dpmr*kappa_T)
%   a_bar_wm is an exact AR(1) with pole 1-a_cov; the KF is fed the WHITENED
%   increment by default (production behaviour, ctrl_const.y2_whiten = true):
%       y1 = dw_m = dw_1 + n_w
%       y2 = a_bar_wm[k] - (1-a_cov)*a_bar_wm[k-1] = a_cov*u[k]
%       H  = H2_scale * [0 0 0 1  -Grad*J_b  -Grad*J_p  -Grad*J_ws]
%   dy2/da_bar_w is exactly 1 (a_bar' does not depend on a_bar_w -- the
%   ancestor's ~0.4% truncation is structurally absent here). The y2
%   innovation uses the NONLINEAR prediction (S7 innovation line)
%       y2_pred = H2_scale * ( a_bar_hat - a_bar' * Grad_wbar_d )
%   not H2*x: the ancestor's H2*x shortcut is exact only because the expgain
%   slope is homogeneous of degree 1 in b, which Form B's a_bar' is not.
%
%   Q (S8, rank-1 gain block; run time at the current estimate):
%       Q33 = Var(eps_w) in FULL (revision 2026-08-01, D3 precedent of
%             kf_canonical_spec.md S5; supersedes the current-step-only
%             container, which is now only its FIRST TERM):
%               kappa_T*( a_bar_hat[k] + (1-lc)^2*sum_{i=1..d} a_bar_hat[k-i] )
%               + (1-lc)^2*sigma2_nw
%             Only the current-step term still matches the expgain sibling's
%             (4kBT/R)*a_h; the total deliberately does NOT (ref S8 note (i)).
%       Q34 = Q43 = -a_bar'*Q33 ,  Q44 = a_bar'^2*Q33 ,  Q55 = Q66 = Q77 = 0.
%   R = diag(R1, R2), R2 at the ESTIMATE a_bar_hat (never the raw readout):
%       R1 = sigma2_nw  (units of R^2)
%       R2 = K_var*IF_eff*(a_bar + xi_bar)^2 + d*Q44     (spec S8; d = d_delay,
%            the {1,1} delay weighting of kf_canonical_spec Sec.6 -- NOT the
%            ancestor's sum (d-j+1)^2 = 5 factor)
%   with xi_bar = (C_n/C_dpmr)*sigma2_nw/kappa_T (= sibling xi / a_o).
%
%   INIT (S10 contract + checklist; every number derived, none tuned):
%       seeds  b = 9/8 (perp two-sphere reflection coefficient, Jeffrey &
%              Onishi 1984; Tier-1 z-axis anchor, applied on all three axes --
%              the parallel-axis 9/16 anchor is a pending derivation decision),
%              p = 1 (far-field 1/r reflection decay), w_s = 1 (nominal
%              contact); a_bar[0] = gain law at (w_bar_d[0]-ws0, b0, p0).
%       P0     sqrt(P_bb) = sqrt(P_pp) = 1/8 (global sup of b_eff/p_eff,
%              the 12.5% anchor tension); NON-diagonal gain/theta block:
%              P_a_theta = G*P_tt and P_aa = G*P_tt*G' + floor^2 with
%              floor = 0.0066 (anchored-seed shape sup) and G the LEVEL
%              Jacobians dA/dtheta at the start height (the seed a_bar[0] is
%              computed BY the law from the theta seeds, so its error
%              linearizes through dA/dtheta, not the slope Jacobians J_theta):
%                  dA/db = -(g/b)*a_bar', dA/dp = (1-a_bar)*ln(1+g/b),
%                  dA/dws = -a_bar'.
%              Position 3-block from the DARE steady state (7a precedent).
%              P0 is chol-checked positive definite on the FREE-state
%              submatrix (locked rows/cols are exact zeros by construction)
%              and the controller errors out if the check fails.
%
%   LOCK FLAGS (Tier ladder; checklist item 3). ctrl_const.lock_b / .lock_p /
%   .lock_ws (default lock_ws = true: Tier-1 pins w_s at the seed 1, zero
%   model error in sim). Lock semantics: the parameter's Jacobians are zeroed
%   at the source (which zeroes its H column and its F_e row-4 column), its
%   Kalman-gain entries are zeroed in both updates, and its P row/col is
%   pinned at 0 (locked = treated as exactly known; freezing a NONZERO prior
%   with the H column zeroed would secularly pump P44 through the F_e
%   coupling with no measurement to drain it).
%
%   Runtime validity clamps (numerical guards, not tuning; alg precedent):
%   b_hat, p_hat clamped positive; ws_hat <= w_bar_d - ws_margin; a_bar_hat
%   floored before the control inversion. No upper cap on a_bar_hat: the
%   ancestor's 0.9999*a_o cap protected its (a_o - a_h) slope factor, which
%   Form B does not have, and a hard cap at the saturation value 1 would
%   one-sidedly rectify the readout noise.
%
%   Optional ctrl_const knobs:
%       .y2_whiten     whitened y2 increment                 (default true)
%       .fe_row4_full  full row-4 multiplier M               (default true)
%       .use_fdet      deterministic F_e regressor           (default true)
%       .y2_off / .y1_gain_off   channel ablation            (default false)
%       .lock_b / .lock_p / .lock_ws   pin parameter at seed (default
%                      false/false/TRUE -- Tier-1)
%       .b_init / .p_init / .ws_init   seeds                 (default 9/8, 1, 1)
%       .Pf_b_std / .Pf_p_std   sqrt P0 widths               (default 1/8, 1/8)
%       .Pf_ws_std     sqrt P0 width of w_s; REQUIRED if lock_ws = false
%                      (no derived default exists yet -- see
%                      reference/shared/param_prior_rules.md six-step chain)
%       .Pf_a_floor    seed-shape floor on sqrt P_aa         (default 0.0066)
%       .Q_theta_floor tiny floor on free theta Q diag       (default 0)
%       .a_bar_floor   a_bar floor before inversion          (default 0.05)
%       .b_clamp / .p_clamp   [min max] validity clamps      (default [0.05 5])
%       .ws_min / .ws_margin  w_s validity clamps            (default 0, 1e-3)
%
%   a_ctrl_override (3x1, um/pN, optional): feed a chosen gain to the CONTROL
%   LAW only; the EKF still estimates a_bar_w / b / p / w_s.
%
%   See also: motion_control_law_5state_expgain,
%             motion_control_law_5state_expgain_alg, run_formB_ws

    if nargin < 6
        a_ctrl_override = [];
    end
    has_override = ~isempty(a_ctrl_override);
    if has_override
        a_ctrl_override = a_ctrl_override(:);
        assert(numel(a_ctrl_override) == 3 && all(isfinite(a_ctrl_override)) && all(a_ctrl_override > 0), ...
               'motion_control_law_formB_ws:badOverride', ...
               'a_ctrl_override must be a 3x1 finite positive vector [um/pN].');
    end

    % ------------------------------------------------------------------
    % Open-loop bypass
    % ------------------------------------------------------------------
    if params.ctrl.enable < 0.5
        f_d = zeros(3, 1);
        ekf_out = [1; 1; 1; 0];
        if nargout >= 3
            diag = empty_diag_formB();
            diag.f_d = f_d;
        end
        return;
    end

    % ------------------------------------------------------------------
    % Persistent state  (all EKF/IIR quantities FULLY NORMALIZED)
    % ------------------------------------------------------------------
    persistent x_e_per_axis        % 7x3 EKF state (col = axis); slots 4..7 = a_bar, b, p, w_s [-]
    persistent P_per_axis          % cell{3} of 7x7 covariance [-]
    persistent dw_bar_m            % 3x1 IIR LP mean of dw_m [-]
    persistent sigma2_dwr_hat      % 3x1 EWMA variance of dw_r [-]
    persistent a_wm_km1            % 3x1 a_bar_wm[k-1], whitening increment delay [-]
    persistent fbar_det_km1 fbar_det_km2   % noise-free control mirror history, NORMALIZED force [-]
    persistent pd_km1 pd_km2       % trajectory delay buffers [um]
    persistent fbar_d_km1 fbar_d_km2       % past control buffers, NORMALIZED force [-]
    persistent a_ctrl_km1 a_ctrl_km2       % control-law gain history, NORMALIZED [-]
    persistent Delta_wbar_d_km1            % Delta_wbar_d of the PREVIOUS call: the
    persistent F_dw_km1                    %   predict k-1 -> k must use the [k-1]
                                           %   command step and force sum (S4/S5
                                           %   pairing; timing-lead fix 2026-08-01)
    persistent k_step

    persistent initialized
    persistent lambda_c d_delay Ts kappa_T R_radius a_o a_disp
    persistent a_pd a_cov C_dpmr C_n K_var IF_abc xi_bar amlpf_var_factor
    persistent t_warmup_kf h_bar_safe sigma2_n_nd
    persistent enable_wall w_hat_n pz_wall
    persistent Q_theta_floor a_bar_floor b_clamp p_clamp ws_min ws_margin gap_floor
    persistent y2_whiten fe_row4_full use_fdet y2_off y1_gain_off
    persistent lock_mask lock_state_idx

    % ------------------------------------------------------------------
    % [0] Initialization on first call
    % ------------------------------------------------------------------
    if isempty(initialized)
        initialized = true;

        % --- 0A. params constants; the unit boundary is crossed HERE [U0] ---
        Ts        = params.ctrl.Ts;
        R_radius  = params.common.R;                              % [um]
        gamma_N_p = params.ctrl.gamma;                            % [pN*sec/um]
        sigma2_n_nd = params.ctrl.sigma2_noise(:) / R_radius^2;   % [-]  [U2]

        a_o    = Ts / (gamma_N_p * R_radius);                     % [1/pN] far-field gain
        % kappa_T = 4*kB*T*a_o/R: the ONLY thermal constant inside the loop.
        kappa_T = 4 * (params.ctrl.k_B * params.ctrl.T / R_radius) * a_o;   % [-]
        a_disp  = a_o * R_radius;             % [um/pN] display scale per unit a_bar

        % --- 0B. ctrl_const (offline scalars; shared with the 5/6-state family) ---
        lambda_c        = ctrl_const.lambda_c;
        d_delay         = ctrl_const.d;
        C_dpmr          = ctrl_const.C_dpmr;
        C_n             = ctrl_const.C_n;
        K_var           = ctrl_const.K_var;
        IF_abc          = ctrl_const.IF_abc(:);
        t_warmup_kf     = ctrl_const.t_warmup_kf;
        h_bar_safe      = ctrl_const.h_bar_safe;
        a_cov           = ctrl_const.a_cov;
        a_pd            = ctrl_const.a_pd;
        amlpf_var_factor = get_field_default(ctrl_const, 'amlpf_var_factor', 1);
        % xi_bar per the spec definition xi = (C_n/C_dpmr)*sigma2_nw/kappa_T
        % (computed from kappa_T directly, NOT sibling xi / a_o, to keep a_o
        % out of the filter constants).
        xi_bar = (C_n / C_dpmr) * sigma2_n_nd / kappa_T;

        y2_whiten    = logical(get_field_default(ctrl_const, 'y2_whiten', true));
        fe_row4_full = logical(get_field_default(ctrl_const, 'fe_row4_full', true));
        use_fdet     = logical(get_field_default(ctrl_const, 'use_fdet', true));
        y2_off       = logical(get_field_default(ctrl_const, 'y2_off', false));
        y1_gain_off  = logical(get_field_default(ctrl_const, 'y1_gain_off', false));
        Q_theta_floor = get_field_default(ctrl_const, 'Q_theta_floor', 0);

        % --- 0C. Lock flags (Tier ladder; default = Tier-1, w_s pinned) ---
        lock_b  = logical(get_field_default(ctrl_const, 'lock_b',  false));
        lock_p  = logical(get_field_default(ctrl_const, 'lock_p',  false));
        lock_ws = logical(get_field_default(ctrl_const, 'lock_ws', true));
        lock_mask = [lock_b; lock_p; lock_ws];        % param order b, p, w_s
        lock_state_idx = 4 + find(lock_mask);         % state indices 5..7

        % --- 0D. Validity clamps (numerical guards only, not tuning) ---
        a_bar_floor = get_field_default(ctrl_const, 'a_bar_floor', 0.05);
        b_clamp     = get_field_default(ctrl_const, 'b_clamp', [0.05, 5]);   % 7b b-clamp precedent
        p_clamp     = get_field_default(ctrl_const, 'p_clamp', [0.05, 5]);
        ws_min      = get_field_default(ctrl_const, 'ws_min', 0);
        ws_margin   = get_field_default(ctrl_const, 'ws_margin', 1e-3);      % alg h-floor precedent
        gap_floor   = ws_margin;      % keeps w_bar - w_s > 0 in the law

        % --- 0E. Wall geometry ---
        if isfield(params, 'wall')
            w_hat_n     = params.wall.w_hat(:);
            pz_wall     = params.wall.pz;
            enable_wall = params.wall.enable_wall_effect > 0.5;
        else
            w_hat_n     = [0; 0; 1];
            pz_wall     = 0;
            enable_wall = false;
        end

        % --- 0F. Seeds (derived, fixed 2026-07-31; do NOT re-derive) ---
        b_seed_derived  = 9/8;    % perp two-sphere reflection coefficient
        p_seed_derived  = 1;      % far-field 1/r reflection decay
        ws_seed_nominal = 1;      % nominal contact position
        seed_b  = expand3(get_field_default(ctrl_const, 'b_init',  b_seed_derived));
        seed_p  = expand3(get_field_default(ctrl_const, 'p_init',  p_seed_derived));
        seed_ws = expand3(get_field_default(ctrl_const, 'ws_init', ws_seed_nominal));

        % --- 0G. P0 widths (derived, fixed 2026-07-31) ---
        % Defaults below are the GLOBAL-SUP FALLBACKS, valid on w_bar in
        % [1.1, 10]. A driver that knows its planned height envelope should
        % derive the tighter envelope sups and pass them in via Pf_b_std /
        % Pf_p_std / Pf_a_floor (run_formB_ws.local_envelope_priors).
        anchor_tension_std = 1/8;      % global sup of b_eff/p_eff (12.5% tension)
        seed_shape_floor   = 0.0066;   % anchored-seed shape sup on a_bar, [1.1,10]
        Pf_b_std   = expand3(get_field_default(ctrl_const, 'Pf_b_std',   anchor_tension_std));
        Pf_p_std   = expand3(get_field_default(ctrl_const, 'Pf_p_std',   anchor_tension_std));
        Pf_a_floor = expand3(get_field_default(ctrl_const, 'Pf_a_floor', seed_shape_floor));
        Pf_ws_std  = get_field_default(ctrl_const, 'Pf_ws_std', []);
        if lock_mask(3)
            Pf_ws_std = zeros(3, 1);      % pinned = exactly known
        elseif isempty(Pf_ws_std)
            error('motion_control_law_formB_ws:missingWsPrior', ...
                  ['lock_ws = false requires ctrl_const.Pf_ws_std (no derived ', ...
                   'default exists; instantiate it via the P[0] six-step chain ', ...
                   'of reference/shared/param_prior_rules.md before running).']);
        else
            Pf_ws_std = expand3(Pf_ws_std);
        end

        % --- 0H. Seed evaluation height (from p0, exactly the 7a pattern) ---
        w_bar_seed = Inf;
        if enable_wall && isfield(params, 'common') && isfield(params.common, 'p0')
            w_bar_seed = (dot(params.common.p0(:), w_hat_n) - pz_wall) / R_radius;   % [U1]
        end
        enable_seed = enable_wall && isfinite(w_bar_seed);

        % --- 0I. EKF state + covariance init (non-diagonal gain/theta block) ---
        x_e_per_axis = zeros(7, 3);
        P_per_axis   = cell(3, 1);
        a_bar_seed_v = zeros(3, 1);
        P_aa_v       = zeros(3, 1);
        P_bb0_v      = zeros(3, 1);
        P_pp0_v      = zeros(3, 1);
        P_ws0_v      = zeros(3, 1);

        F3 = [0 1 0; 0 0 1; 0 0 lambda_c];
        H3 = [1 0 0];
        free_mask_row = double(~lock_mask.');     % 1x3, param order b, p, w_s

        for ax = 1:3
            gap_seed = max(w_bar_seed - seed_ws(ax), gap_floor);
            [a_bar_seed, ~, ~, ~, ~, dA_db, dA_dp, dA_dws] = ...
                local_gain_law_formB(gap_seed, seed_b(ax), seed_p(ax), enable_seed);
            a_bar_seed = max(a_bar_seed, a_bar_floor);
            a_bar_seed_v(ax) = a_bar_seed;

            % Level Jacobians G = dA/d[b p ws] at the start height; locked
            % parameters carry zero error, so their columns are zeroed.
            G_lvl = [dA_db, dA_dp, dA_dws] .* free_mask_row;
            Ptt   = [Pf_b_std(ax)^2, Pf_p_std(ax)^2, Pf_ws_std(ax)^2] .* free_mask_row;

            P7 = zeros(7);
            % Same Q33 container as run time (S8 revision 2026-08-01),
            % evaluated at the seed where the whole history equals the seed.
            Q3 = zeros(3);
            Q3(3, 3) = kappa_T * a_bar_seed * (1 + d_delay * (1 - lambda_c)^2) ...
                       + (1 - lambda_c)^2 * sigma2_n_nd(ax);
            P7(1:3, 1:3) = solve_dare_kf_local(F3, H3, Q3, sigma2_n_nd(ax));
            P7(4, 4)   = sum(G_lvl.^2 .* Ptt) + Pf_a_floor(ax)^2;     % P_aa
            P7(4, 5:7) = G_lvl .* Ptt;                                % P_a_theta
            P7(5:7, 4) = (G_lvl .* Ptt).';
            P7(5, 5) = Ptt(1);
            P7(6, 6) = Ptt(2);
            P7(7, 7) = Ptt(3);
            P_aa_v(ax) = P7(4, 4);
            P_bb0_v(ax) = P7(5, 5);
            P_pp0_v(ax) = P7(6, 6);
            P_ws0_v(ax) = P7(7, 7);

            % chol PD check on the free-state submatrix (locked rows/cols are
            % exact zeros by construction, so the full matrix is only PSD).
            free_idx = [1:4, 4 + find(~lock_mask).'];
            [~, chol_flag] = chol(P7(free_idx, free_idx));
            if chol_flag ~= 0
                error('motion_control_law_formB_ws:P0NotPD', ...
                      'P0 free-state block is not positive definite (axis %d).', ax);
            end

            P_per_axis{ax} = P7;
            x_e_per_axis(4, ax) = a_bar_seed;
            x_e_per_axis(5, ax) = seed_b(ax);
            x_e_per_axis(6, ax) = seed_p(ax);
            x_e_per_axis(7, ax) = seed_ws(ax);
        end

        % --- 0J. IIR states (prefill to the closed-loop dw_r variance) ---
        dw_bar_m = zeros(3, 1);
        sigma2_dwr_hat = C_dpmr * kappa_T * a_bar_seed_v + C_n * sigma2_n_nd;
        % the prefill is exactly a_bar_wm[0] = a_bar[0]; seed the whitening
        % delay slot with it (first increment is then noise-only, not a step).
        a_wm_km1 = a_bar_seed_v;

        % --- 0K. Delay buffers (force history NORMALIZED, checklist) ---
        pd_km1  = pd;
        pd_km2  = pd;
        fbar_d_km1   = zeros(3, 1);
        fbar_d_km2   = zeros(3, 1);
        fbar_det_km1 = zeros(3, 1);
        fbar_det_km2 = zeros(3, 1);
        a_ctrl_km1 = a_bar_seed_v;
        a_ctrl_km2 = a_bar_seed_v;
        Delta_wbar_d_km1 = 0;
        F_dw_km1 = zeros(3, 1);
        k_step = 1;

        % --- 0L. First call returns zeros (no f_d yet) ---
        f_d = zeros(3, 1);
        a_hat_phys = a_bar_seed_v * a_disp;                       % [U4]
        ekf_out = [a_hat_phys(1); a_hat_phys(3); a_hat_phys(2); 0];
        if nargout >= 3
            diag = empty_diag_formB();
            diag.f_d       = f_d;
            diag.a_hat     = a_hat_phys;
            diag.a_hat_nd  = a_bar_seed_v * a_o;                  % [U4] legacy 1/pN
            diag.a_bar_hat = a_bar_seed_v;
            diag.b_hat     = seed_b;
            diag.p_hat     = seed_p;
            diag.ws_hat    = seed_ws;
            diag.delta_a_hat = seed_b;                            % driver-log alias
            diag.sigma2_dxr_hat = sigma2_dwr_hat;
            diag.P_a  = P_aa_v * a_disp^2;                        % [U4] (um/pN)^2
            diag.P_a_nd = P_aa_v;
            % report the ACTUAL P0 entries (locked slots are exact zeros)
            diag.P_b  = P_bb0_v;
            diag.P_p  = P_pp0_v;
            diag.P_ws = P_ws0_v;
            diag.P77  = P_bb0_v;                                  % legacy alias (b var)
            diag.h_bar_init = w_bar_seed;
            diag.Pf_a_floor = Pf_a_floor;
            diag.Pf_theta_std = [Pf_b_std, Pf_p_std, Pf_ws_std];
            % logging-contract additions (smoke/normalization ladder)
            % Q33 container at the seed (full Var(eps_w), S8 revision):
            % history == seed on the init call.
            diag.Q33     = kappa_T * a_bar_seed_v * (1 + d_delay * (1 - lambda_c)^2) ...
                           + (1 - lambda_c)^2 * sigma2_n_nd;
            diag.a_bar_Q = a_bar_seed_v;             % current-step a_bar in Q33
            diag.f_bar   = zeros(3, 1);              % no force on the init call
            diag.P_full  = cat(3, P_per_axis{1}, P_per_axis{2}, P_per_axis{3});
            if has_override
                diag.a_ctrl_used = a_ctrl_override;
            else
                diag.a_ctrl_used = a_hat_phys;
            end
        end
        return;
    end

    % ------------------------------------------------------------------
    % Per-step: extract per-axis state
    % ------------------------------------------------------------------
    a_bar_hat = x_e_per_axis(4, :).';     % 3x1 [-]

    if has_override
        a_ctrl = a_ctrl_override / a_disp;      % um/pN -> [-]   [U1 interface]
    else
        a_ctrl = a_bar_hat;
    end
    a_ctrl = max(a_ctrl, a_bar_floor);          % floor before inversion (no cap)

    % --- [U1] every length crossing the boundary is divided by R here ---
    if d_delay == 2
        pd_km_d = pd_km2;
    elseif d_delay == 1
        pd_km_d = pd_km1;
    else
        error('motion_control_law_formB_ws:unsupportedDelay', ...
              'Only d=1 or d=2 supported, got d=%g.', d_delay);
    end
    delta_w_m = (pd_km_d - p_m) / R_radius;      % 3x1 dw_m[k]  [-]
    pd_kp1    = pd + del_pd;                     % 3x1 [um]

    if enable_wall
        h_bar = (dot(p_m, w_hat_n) - pz_wall) / R_radius;
    else
        h_bar = Inf;
    end

    one_minus_lc = 1 - lambda_c;

    % ------------------------------------------------------------------
    % Known desired-trajectory height increments (wall-normal, dimensionless).
    %   Delta_wbar_d[k] = (w_d[k+1]-w_d[k])/R          (stored; predict/F_e use [k-1])
    %   Grad_wbar_d[k]  = (w_d[k]-w_d[k-d])/R          (y2 delay back-off)
    %   w_bar_d         = w_d[k]/R                     (law operating point)
    % ------------------------------------------------------------------
    Delta_wbar_d = dot(del_pd, w_hat_n) / R_radius;
    Grad_wbar_d  = dot(pd - pd_km_d, w_hat_n) / R_radius;
    if enable_wall
        w_bar_d = (dot(pd, w_hat_n) - pz_wall) / R_radius;
    else
        w_bar_d = Inf;
    end

    % ------------------------------------------------------------------
    % [1] IIR gain readout a_bar_wm, in ONE step with kappa_T (checklist).
    % ------------------------------------------------------------------
    dw_bar_m_new = (1 - a_pd) * dw_bar_m + a_pd * delta_w_m;
    dw_r = delta_w_m - dw_bar_m_new;
    sigma2_dwr_hat_new = (1 - a_cov) * sigma2_dwr_hat + a_cov * dw_r.^2;
    a_bar_wm = (sigma2_dwr_hat_new - C_n * sigma2_n_nd) / (C_dpmr * kappa_T);   % [-]
    if y2_whiten
        y2 = a_bar_wm - (1 - a_cov) * a_wm_km1;      % = a_cov*u[k]
        H2_scale = a_cov;
    else
        y2 = a_bar_wm;                               % raw AR(1) output (spec body)
        H2_scale = 1;
    end

    % ------------------------------------------------------------------
    % [2] Control law (eq17 implementable; NO disturbance term), NORMALIZED:
    %   fbar_dw = a_bar_ctrl^-1 { Dwbar_d^d[k]
    %             + (1-lc)[dw_m - sum a_bar_ctrl[k-i] fbar_dw[k-i]] }
    % The physical force leaves at [U3] below; history buffers keep fbar.
    % ------------------------------------------------------------------
    if d_delay == 2
        sum_a_fd_past = a_ctrl_km1 .* fbar_d_km1 + a_ctrl_km2 .* fbar_d_km2;
        sum_af_det    = a_ctrl_km1 .* fbar_det_km1 + a_ctrl_km2 .* fbar_det_km2;
    else
        sum_a_fd_past = a_ctrl_km1 .* fbar_d_km1;
        sum_af_det    = a_ctrl_km1 .* fbar_det_km1;
    end
    inv_a_ctrl = 1 ./ a_ctrl;
    traj_term = (pd_kp1 - lambda_c * pd - one_minus_lc * pd_km_d) / R_radius;
    fbar_d = inv_a_ctrl .* (traj_term ...
                            + one_minus_lc * delta_w_m ...
                            - one_minus_lc * sum_a_fd_past);

    % ------------------------------------------------------------------
    % [2b] Deterministic mirror (dw_m := 0, own history) -> exogenous F_dw
    %   regressor for F_e; using the realised force instead rectifies (the
    %   ancestor's measured 24.3% drag-down). Same pattern, normalized force.
    % ------------------------------------------------------------------
    fbar_det = inv_a_ctrl .* (traj_term - one_minus_lc * sum_af_det);
    if d_delay == 2
        F_dw_det = fbar_det + one_minus_lc * (fbar_det_km1 + fbar_det_km2);
        F_dw_raw = fbar_d   + one_minus_lc * (fbar_d_km1 + fbar_d_km2);
    else
        F_dw_det = fbar_det + one_minus_lc * fbar_det_km1;
        F_dw_raw = fbar_d   + one_minus_lc * fbar_d_km1;
    end
    if use_fdet
        F_dw_vec = F_dw_det;
    else
        F_dw_vec = F_dw_raw;      % ablation only: reproduces the rectification
    end

    % --- [U3] force output conversion (the ONLY loop-adjacent a_o use) ---
    f_d = fbar_d / a_o;                          % [pN]

    % ------------------------------------------------------------------
    % [3] Per-axis law evaluation, Q, R, gate; EKF predict + sequential
    %     scalar updates (Joseph form). Split filter form, mirroring the
    %     expgain siblings (kf_canonical_spec Sec.1 skeleton).
    % ------------------------------------------------------------------
    I7 = eye(7);
    t_now = (k_step - 1) * Ts;

    K_a_y2_v   = zeros(3, 1);
    K_dx_y1_v  = zeros(3, 1);
    innov_y2_v = zeros(3, 1);
    innov_y1_v = zeros(3, 1);   % logging only (whiteness diagnostic)
    gate_off   = false(3, 1);
    G_flags    = false(3, 3);
    a_prime_v  = zeros(3, 1);
    Q44_v      = zeros(3, 1);
    Q33_v      = zeros(3, 1);
    a_barQ_v   = zeros(3, 1);
    R2_v       = zeros(3, 1);

    for ax = 1:3
        x_curr = x_e_per_axis(:, ax);
        P_curr = P_per_axis{ax};
        a_bar_i = max(x_curr(4), a_bar_floor);
        b_i  = min(max(x_curr(5), b_clamp(1)), b_clamp(2));
        p_i  = min(max(x_curr(6), p_clamp(1)), p_clamp(2));
        ws_i = x_curr(7);

        % --- Gain rate + parameter Jacobians at (w_bar_d - ws_hat, b, p).
        %     The gain LEVEL is the state; the law supplies only a_bar' and
        %     the J's here (no re-anchoring of a_bar from theta).
        gap_d = max(w_bar_d - ws_i, gap_floor);
        [~, a_prime_i, J_b_i, J_p_i, J_ws_i] = ...
            local_gain_law_formB(gap_d, b_i, p_i, enable_wall && isfinite(w_bar_d));
        % Locked parameters carry zero error: zero their J at the source
        % (this zeroes the H column AND the F_e row-4 column in one move).
        if lock_mask(1); J_b_i  = 0; end
        if lock_mask(2); J_p_i  = 0; end
        if lock_mask(3); J_ws_i = 0; end
        a_prime_v(ax) = a_prime_i;

        % --- Q (7x7): rank-1 gain block + Q33 (spec S8, at the estimate) ---
        % Q33 = Var(eps_w) in FULL (revision 2026-08-01, D3 precedent of
        % kf_canonical_spec.md S5; supersedes the current-step-only
        % container): current thermal step + thermal history weighted
        % (1-lc)^2 + (1-lc)^2 * n_w feedthrough. Only the cross-step
        % correlations of eps_w are dropped (declared, ref S8 note (i)).
        % History comes from the clamped posterior gains a_bar_hat[k-i];
        % the buffers still hold [k-1]/[k-2] here (they roll in [4]).
        if d_delay == 2
            a_bar_hist = a_ctrl_km1(ax) + a_ctrl_km2(ax);
        else
            a_bar_hist = a_ctrl_km1(ax);
        end
        Q33 = kappa_T * (a_bar_i + one_minus_lc^2 * a_bar_hist) ...
              + one_minus_lc^2 * sigma2_n_nd(ax);
        Q_i = zeros(7);
        Q_i(3, 3) = Q33;
        Q_i(3, 4) = -a_prime_i * Q33;
        Q_i(4, 3) = -a_prime_i * Q33;
        Q_i(4, 4) = a_prime_i^2 * Q33;
        for j = 1:3
            if ~lock_mask(j)
                Q_i(4 + j, 4 + j) = Q_theta_floor;   % conditioning only, default 0
            end
        end
        Q44_v(ax) = Q_i(4, 4);
        Q33_v(ax) = Q33;
        a_barQ_v(ax) = a_bar_i;      % the a_bar actually used to build Q33

        % --- R (R2 at the ESTIMATE a_bar_hat, never the raw readout) ---
        R1_i = sigma2_n_nd(ax);
        R2_i = compute_R2_formB(a_bar_i, sigma2_n_nd(ax), IF_abc, C_dpmr, C_n, ...
                                K_var, amlpf_var_factor, xi_bar(ax), kappa_T, ...
                                Q_i(4, 4), d_delay, a_cov, y2_whiten);
        R2_v(ax) = R2_i;

        % --- Gates (OR): warm-up / readout NaN guard / near wall ---
        G1 = (t_now < t_warmup_kf);
        G2 = ((sigma2_dwr_hat_new(ax) - C_n * sigma2_n_nd(ax)) <= 0);
        G3 = (h_bar < h_bar_safe);
        G_flags(:, ax) = [G1; G2; G3];
        gate_off(ax) = G1 || G2 || G3;

        % --- Row-4 multiplier: deterministic part of the step the TRUE
        %     position took over [k-1, k]; the predict bridges k-1 -> k, so
        %     it must use the PREVIOUS call's command step and force sum
        %     (timing-lead fix 2026-08-01; the ancestor family uses the
        %     current-call values, a one-step lead vs the S4/S5 pairing).
        %     Dropping (1-lc)*dw_3_hat zeroes the parameter columns at
        %     every trajectory turning point.
        if fe_row4_full
            M_row4 = Delta_wbar_d_km1 + one_minus_lc * x_curr(3);
        else
            M_row4 = Delta_wbar_d_km1;
        end

        F_dw = F_dw_km1(ax);
        F_e = local_build_F_e_formB(lambda_c, F_dw, a_prime_i, ...
                                    J_b_i, J_p_i, J_ws_i, M_row4);

        % --- EKF predict (S5; constants carry over) ---
        x_pred = [x_curr(2); ...
                  x_curr(3); ...
                  lambda_c * x_curr(3); ...
                  x_curr(4) + a_prime_i * (Delta_wbar_d_km1 + one_minus_lc * x_curr(3)); ...
                  x_curr(5); ...
                  x_curr(6); ...
                  x_curr(7)];
        P_pred = F_e * P_curr * F_e' + Q_i;
        P_pred = 0.5 * (P_pred + P_pred');
        P_pred = freeze_locked_P(P_pred, lock_state_idx);

        % --- Sequential 1-D measurement updates (R diagonal => exact) ---
        freeze_gain = G1;

        % (a) y1 = dw_m observes dw_1
        H1 = [1 0 0 0 0 0 0];
        S1 = H1 * P_pred * H1' + R1_i;
        K1 = (P_pred * H1') / S1;
        if freeze_gain || y1_gain_off; K1(4:7) = 0; end
        K1(lock_state_idx) = 0;
        innov1 = delta_w_m(ax) - H1 * x_pred;
        innov_y1_v(ax) = innov1;          % logging only (whiteness diagnostic)
        x_upd  = x_pred + K1 * innov1;
        ImKH1  = I7 - K1 * H1;
        P_upd  = ImKH1 * P_pred * ImKH1' + K1 * R1_i * K1';   % Joseph form
        P_upd  = 0.5 * (P_upd + P_upd');
        K_dx_y1_v(ax) = K1(3);

        % (b) y2 = gain readout (whitened increment by default).
        %     a_wm_km1 is shifted every step regardless, so the increment
        %     stays valid when the gate reopens.
        if ~gate_off(ax) && ~y2_off
            % H row 2 (spec S7): [0 0 0 1 -Grad*J_b -Grad*J_p -Grad*J_ws];
            % dy2/da_bar is exactly 1.
            H2 = H2_scale * [0, 0, 0, 1, ...
                             -Grad_wbar_d * J_b_i, ...
                             -Grad_wbar_d * J_p_i, ...
                             -Grad_wbar_d * J_ws_i];
            % NONLINEAR predicted measurement (S7 innovation line); H2*x_upd
            % would be wrong here -- see the header.
            y2_pred = H2_scale * (x_upd(4) - a_prime_i * Grad_wbar_d);
            S2  = H2 * P_upd * H2' + R2_i;
            K2  = (P_upd * H2') / S2;
            if freeze_gain; K2(4:7) = 0; end
            K2(lock_state_idx) = 0;
            innov2 = y2(ax) - y2_pred;
            x_upd  = x_upd + K2 * innov2;
            ImKH2  = I7 - K2 * H2;
            P_upd  = ImKH2 * P_upd * ImKH2' + K2 * R2_i * K2';   % Joseph form
            P_upd  = 0.5 * (P_upd + P_upd');
            K_a_y2_v(ax)   = K2(4);
            innov_y2_v(ax) = innov2;
        end

        % --- Validity clamps (numerical guards, not tuning). Locked slots
        %     are never touched (K entries zeroed, predict identity), so
        %     they stay exactly at the seed.
        x_upd(4) = max(x_upd(4), a_bar_floor);
        if ~lock_mask(1)
            x_upd(5) = min(max(x_upd(5), b_clamp(1)), b_clamp(2));
        end
        if ~lock_mask(2)
            x_upd(6) = min(max(x_upd(6), p_clamp(1)), p_clamp(2));
        end
        if ~lock_mask(3)
            x_upd(7) = min(max(x_upd(7), ws_min), w_bar_d - ws_margin);
        end
        P_upd = freeze_locked_P(P_upd, lock_state_idx);

        x_e_per_axis(:, ax) = x_upd;
        P_per_axis{ax} = P_upd;
    end

    % ------------------------------------------------------------------
    % [4] Bookkeeping
    % ------------------------------------------------------------------
    pd_km2 = pd_km1; pd_km1 = pd;
    fbar_d_km2 = fbar_d_km1; fbar_d_km1 = fbar_d;
    fbar_det_km2 = fbar_det_km1; fbar_det_km1 = fbar_det;
    a_ctrl_km2 = a_ctrl_km1; a_ctrl_km1 = a_ctrl;
    Delta_wbar_d_km1 = Delta_wbar_d;
    F_dw_km1 = F_dw_vec;
    dw_bar_m = dw_bar_m_new;
    sigma2_dwr_hat = sigma2_dwr_hat_new;
    a_wm_km1 = a_bar_wm;
    k_step = k_step + 1;

    % ------------------------------------------------------------------
    % [5] Output   ([U4] display layer: the only remaining a_o uses)
    % ------------------------------------------------------------------
    a_bar_post = x_e_per_axis(4, :).';
    a_hat_phys = a_bar_post * a_disp;                  % [um/pN]
    b_post  = x_e_per_axis(5, :).';
    p_post  = x_e_per_axis(6, :).';
    ws_post = x_e_per_axis(7, :).';
    h_bar_now = local_h_bar_out(enable_wall, h_bar);
    ekf_out = [a_hat_phys(1); a_hat_phys(3); a_hat_phys(2); h_bar_now];

    if nargout >= 3
        P_a_v = zeros(3, 1); P_dx_v = zeros(3, 1); P_dx1_v = zeros(3, 1);
        P_b_v = zeros(3, 1); P_p_v  = zeros(3, 1); P_ws_v  = zeros(3, 1);
        for ax = 1:3
            P_a_v(ax)   = P_per_axis{ax}(4, 4);
            P_dx_v(ax)  = P_per_axis{ax}(3, 3);
            P_dx1_v(ax) = P_per_axis{ax}(1, 1);
            P_b_v(ax)   = P_per_axis{ax}(5, 5);
            P_p_v(ax)   = P_per_axis{ax}(6, 6);
            P_ws_v(ax)  = P_per_axis{ax}(7, 7);
        end
        diag = empty_diag_formB();
        diag.sigma2_dxr_hat = sigma2_dwr_hat_new;
        diag.a_xm           = a_bar_wm * a_disp;      % [um/pN]
        diag.a_hm_nd        = a_bar_wm * a_o;         % legacy 1/pN convention
        diag.a_wm_bar       = a_bar_wm;
        diag.y2             = y2;                     % normalized gain units
        diag.delta_x_m      = delta_w_m * R_radius;   % [um]
        diag.innovation_y2  = innov_y2_v;
        diag.innovation_y1  = innov_y1_v;   % [-] normalized; whiteness diagnostic
        diag.K_kf_a_y2      = K_a_y2_v;
        diag.K_kf_dx_y1     = K_dx_y1_v;
        diag.P_a            = P_a_v * a_disp^2;       % (um/pN)^2
        diag.P_a_nd         = P_a_v;                  % [-] (a_bar^2 units)
        diag.P_dx           = P_dx_v * R_radius^2;    % [um^2]
        diag.x_D_hat        = zeros(3, 1);
        diag.b_hat          = b_post;
        diag.p_hat          = p_post;                 % REAL p (not the sibling alias)
        diag.ws_hat         = ws_post;
        diag.delta_a_hat    = b_post;                 % driver-log alias
        diag.a_prime_hat    = a_prime_v * a_o;        % legacy d a_nd/d h_bar [1/pN]
        diag.a_prime_bar    = a_prime_v;              % d a_bar / d w_bar [-]
        diag.gate_active_per_axis = gate_off;
        diag.guards_individual    = G_flags;
        diag.h_bar          = h_bar;
        diag.h_bar_d        = w_bar_d;
        diag.f_d            = f_d;                    % [pN]
        diag.f_det          = fbar_det / a_o;         % [pN]
        diag.F_dh           = F_dw_vec / a_o;         % [pN]
        diag.dx_r           = dw_r * R_radius;        % [um], same convention as delta_x_m
        diag.a_hat          = a_hat_phys;
        diag.a_hat_nd       = a_bar_post * a_o;       % legacy 1/pN convention
        diag.a_bar_hat      = a_bar_post;
        diag.a_ctrl_used    = a_ctrl * a_disp;        % [um/pN]
        diag.P_b            = P_b_v;
        diag.P_p            = P_p_v;
        diag.P_ws           = P_ws_v;
        diag.P77            = P_b_v;                  % legacy alias (exponent-slot var)
        diag.Q77            = Q44_v;                  % legacy alias
        diag.var_da_ram     = Q44_v;
        diag.R2             = R2_v;
        diag.Delta_h_d      = Delta_wbar_d * R_radius;   % [um]
        diag.Delta_H_d      = Grad_wbar_d * R_radius;    % [um]
        diag.delta_x_hat_1  = x_e_per_axis(1, :).' * R_radius;
        diag.delta_x_hat_3  = x_e_per_axis(3, :).' * R_radius;
        diag.P_dx1          = P_dx1_v * R_radius^2;
        % logging-contract additions (smoke/normalization ladder)
        diag.Q33     = Q33_v;
        diag.a_bar_Q = a_barQ_v;
        diag.f_bar   = fbar_d;                        % normalized force [-]
        diag.P_full  = cat(3, P_per_axis{1}, P_per_axis{2}, P_per_axis{3});
    end
end


%% =================== Local Helpers ===================

function [a_bar, a_bar_p, J_b, J_p, J_ws, dA_db, dA_dp, dA_dws] = ...
        local_gain_law_formB(gap, b, p, enable)
%LOCAL_GAIN_LAW_FORMB  Form B gain law, gain rate and Jacobians (S1/S2), all
%   dimensionless. gap = w_bar - w_bar_s > 0 (caller guards positivity).
%       a_bar   = 1 - (1 + gap/b)^(-p)
%       a_bar'  = (p/b)*(1 + gap/b)^(-p-1)              (d a_bar / d w_bar)
%   Slope Jacobians (log-derivative factorization, J = [scalar]*a_bar'):
%       J_b  = [-1/b + (p+1)*gap/(b*(gap+b))] * a_bar'
%       J_p  = [1/p - ln(1+gap/b)]            * a_bar'
%       J_ws = [(p+1)/(gap+b)]                * a_bar'
%   Level Jacobians (P0 cross-block only; dA/dws = -a_bar' is exact):
%       dA_db = -(gap/b)*a_bar' ,  dA_dp = (1-a_bar)*ln(1+gap/b) ,
%       dA_dws = -a_bar'
    if ~enable || ~isfinite(gap)
        a_bar = 1;      % far field: the law saturates at exactly 1
        a_bar_p = 0; J_b = 0; J_p = 0; J_ws = 0;
        dA_db = 0; dA_dp = 0; dA_dws = 0;
        return;
    end
    u    = 1 + gap / b;
    upow = u^(-p);                      % = 1 - a_bar
    a_bar   = 1 - upow;
    a_bar_p = (p / b) * upow / u;       % = (p/b)*u^(-p-1)
    J_b  = (-1 / b + (p + 1) * gap / (b * (gap + b))) * a_bar_p;
    J_p  = (1 / p - log(u)) * a_bar_p;
    J_ws = ((p + 1) / (gap + b)) * a_bar_p;
    dA_db  = -(gap / b) * a_bar_p;
    dA_dp  = upow * log(u);
    dA_dws = -a_bar_p;
end


function F_e = local_build_F_e_formB(lambda_c, F_dw, a_bar_p, J_b, J_p, J_ws, M_row4)
%LOCAL_BUILD_F_E_FORMB  7x7 error-dynamics Jacobian (boxed F_e, spec S6).
%   cols: 1=dw1 2=dw2 3=dw3 4=a_bar 5=b 6=p 7=w_s
%   Row 3 = [0 0 lc            -F_dw            0      0      0     ]
%   Row 4 = [0 0 (1-lc)*a_bar' 1+a_bar'*F_dw   J_b*M  J_p*M  J_ws*M ]
%   Rows 5-7 identity (constants). F_e(4,4) has NO M*(da'/da) term: Form B's
%   a_bar' does not depend on the state a_bar_w (unlike the expgain sibling).
    one_minus_lc = 1 - lambda_c;
    F_e = [0 1 0        0                    0          0          0; ...
           0 0 1        0                    0          0          0; ...
           0 0 lambda_c -F_dw                0          0          0; ...
           0 0 one_minus_lc*a_bar_p  1+a_bar_p*F_dw  J_b*M_row4  J_p*M_row4  J_ws*M_row4; ...
           0 0 0        0                    1          0          0; ...
           0 0 0        0                    0          1          0; ...
           0 0 0        0                    0          0          1];
end


function R2 = compute_R2_formB(a_bar, sigma2_n, IF_abc, C_dpmr, C_n, ...
                               K_var, amlpf_var_factor, xi_bar, kappa_T, Q44, ...
                               delay_steps, a_cov, whitened)
%COMPUTE_R2_FORMB  R(2,2) for the gain readout channel, fully normalized.
%   Same chi-squared chain as the expgain sibling with every physical-gain
%   quantity replaced by its normalized counterpart (a_h -> a_bar,
%   xi -> xi_bar, 4kBT/R*a_h -> kappa_T*a_bar); the overall value is the
%   sibling's divided by a_o^2. IF_eff is homogeneous of degree 0 in
%   (sxT, sigma2_n), and sxT = kappa_T*a_bar equals the sibling's argument
%   numerically, so IF_eff is unchanged.
%       R2_int = amlpf * K_var * IF_eff * (a_bar + xi_bar)^2
%   Delay term = d*Q44 per formB_ws.tex S8 (house back-off channel carried by
%   R; {1,1} weighting per kf_canonical_spec Sec.6) -- NOT the sibling's
%   sum_{j=1}^d (d-j+1)^2 factor.
%   whitened = true: y2 = a_cov*u[k]; per-sample variance a_cov*(2-a_cov)*
%   Var(a_bar_wm), and the d-step Q44 accumulation rides on a_cov*u, picking
%   up a_cov^2 (sibling's derivation, unchanged by the normalization).
    sxT = kappa_T * a_bar;      % numerically == sibling's 4*kBT_over_R*a_h
    num = sxT^2 * IF_abc(1) + 2 * sxT * sigma2_n * IF_abc(2) + sigma2_n^2 * IF_abc(3);
    den = (C_dpmr * sxT + C_n * sigma2_n)^2;
    IF  = 1 + 2 * num / den;
    R2_int = amlpf_var_factor * K_var * IF * (a_bar + xi_bar)^2;
    if whitened
        R2 = a_cov * (2 - a_cov) * R2_int + a_cov^2 * delay_steps * Q44;
    else
        R2 = R2_int + delay_steps * Q44;
    end
end


function P = freeze_locked_P(P, lock_state_idx)
%FREEZE_LOCKED_P  Pin locked-parameter rows/cols of P at 0 (exactly known).
%   They are zero from init and receive no Q/K injections, so this is a
%   defensive re-zeroing against numerical dust, not a projection.
    if ~isempty(lock_state_idx)
        P(lock_state_idx, :) = 0;
        P(:, lock_state_idx) = 0;
    end
end


function v3 = expand3(v)
%EXPAND3  Scalar -> 3x1; anything else -> 3x1 column.
    if isscalar(v)
        v3 = v * ones(3, 1);
    else
        v3 = v(:);
    end
end


function v = get_field_default(s, name, default)
%GET_FIELD_DEFAULT  s.(name) if present and non-empty, else default.
    if isfield(s, name) && ~isempty(s.(name))
        v = s.(name);
    else
        v = default;
    end
end


function P_post = solve_dare_kf_local(F, H, Q, R)
%SOLVE_DARE_KF_LOCAL  Discrete-time KF Riccati steady-state (fixed-point).
    n = size(F, 1);
    P_post = eye(n);
    max_iter = 10000;
    tol = 1e-16;
    for k = 1:max_iter
        P_pred = F * P_post * F' + Q;
        P_pred = 0.5 * (P_pred + P_pred');
        S = H * P_pred * H' + R;
        K = (P_pred * H') / S;
        P_new = (eye(n) - K * H) * P_pred;
        P_new = 0.5 * (P_new + P_new');
        if max(abs(P_new(:) - P_post(:))) < tol * max(1, max(abs(P_post(:))))
            P_post = P_new;
            return;
        end
        P_post = P_new;
    end
end


function h = local_h_bar_out(enable_wall, h_bar)
    if enable_wall
        h = h_bar;
    else
        h = 0;
    end
end


function d = empty_diag_formB()
%EMPTY_DIAG_FORMB  Zeroed diagnostic struct (driver collect_diag compatible;
%   superset of the expgain sibling's fields). b/p/ws placeholders sit at the
%   derived seeds (9/8, 1, 1) so bypass logs stay physically sane.
    d = struct();
    d.sigma2_dxr_hat    = zeros(3, 1);
    d.a_xm              = zeros(3, 1);
    d.a_hm_nd           = zeros(3, 1);
    d.a_wm_bar          = zeros(3, 1);
    d.y2                = zeros(3, 1);
    d.delta_x_m         = zeros(3, 1);
    d.innovation_y2     = zeros(3, 1);
    d.innovation_y1     = zeros(3, 1);
    d.K_kf_a_y2         = zeros(3, 1);
    d.K_kf_dx_y1        = zeros(3, 1);
    d.P_a               = zeros(3, 1);
    d.P_a_nd            = zeros(3, 1);
    d.P_dx              = zeros(3, 1);
    d.x_D_hat           = zeros(3, 1);
    d.b_hat             = (9/8) * ones(3, 1);    % derived seed (see header)
    d.p_hat             = ones(3, 1);
    d.ws_hat            = ones(3, 1);
    d.delta_a_hat       = (9/8) * ones(3, 1);
    d.a_prime_hat       = zeros(3, 1);
    d.a_prime_bar       = zeros(3, 1);
    d.gate_active_per_axis = false(3, 1);
    d.guards_individual    = false(3, 3);
    d.h_bar             = 0;
    d.h_bar_d           = 0;
    d.f_d               = zeros(3, 1);
    d.f_det             = zeros(3, 1);
    d.F_dh              = zeros(3, 1);
    d.dx_r              = zeros(3, 1);
    d.a_hat             = zeros(3, 1);
    d.a_hat_nd          = zeros(3, 1);
    d.a_bar_hat         = zeros(3, 1);
    d.a_ctrl_used       = zeros(3, 1);
    d.P_b               = zeros(3, 1);
    d.P_p               = zeros(3, 1);
    d.P_ws              = zeros(3, 1);
    d.P77               = zeros(3, 1);
    d.Q77               = zeros(3, 1);
    d.var_da_ram        = zeros(3, 1);
    d.R2                = zeros(3, 1);
    d.Delta_h_d         = 0;
    d.Delta_H_d         = 0;
    d.delta_x_hat_1     = zeros(3, 1);
    d.delta_x_hat_3     = zeros(3, 1);
    d.P_dx1             = zeros(3, 1);
    d.h_bar_init        = 0;
    d.Pf_a_floor        = zeros(3, 1);
    d.Pf_theta_std      = zeros(3, 3);
    d.Q33               = zeros(3, 1);
    d.a_bar_Q           = zeros(3, 1);
    d.f_bar             = zeros(3, 1);
    d.P_full            = zeros(7, 7, 3);
end
