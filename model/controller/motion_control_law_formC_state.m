% STATUS: ACTIVE | SSOT derivation: reference/eq17_analysis/derivation/formC_state_da.tex
% FORK OF model/controller/motion_control_law_formB_ws.m @ 73fdc9d | PURPOSE:
%   slope read off the gain STATE (a_bar' = (1-a_bar)^2 (1+da)) with a
%   multiplicative constant disturbance da, instead of off the commanded
%   height with (b, p, w_s) | EXPIRES: when T2/T3/T4 decide adoption |
%   production changes do NOT follow automatically.
%   Slot map: 5 = da (was b); slots 6-7 are PERMANENTLY LOCKED and inert
%   (zero P0, zero Jacobian, zero K) so the 9-slot layout and the whole
%   MA(2) block at [8 9] carry over with no re-indexing. Mathematically the
%   filter is the 5-state of the tex.
function [f_d, ekf_out, diag] = motion_control_law_formC_state(del_pd, pd, p_m, params, ctrl_const, a_ctrl_override)
%MOTION_CONTROL_LAW_FORMC_STATE  Per-axis EKF eq17 controller whose gain slope
%   is a function of the gain state alone (state writing, formC_state_da.tex)
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
%   MA(2) AUGMENTATION (ctrl_const.ma2_aug, 2026-08-01). The Q33 container
%   above models eps_w as WHITE with the full MA(2) variance, which
%   underweights its DC power by (1+2*alpha)^2/(1+2*alpha^2) = 2.17 at
%   lambda_c = 0.7. The exact fix carries the noise memory as states instead
%   of approximating its spectrum:
%       eps_w[k] = w_T[k] + alpha*(w_T[k-1] + w_T[k-2]) - alpha*n_w[k-d],
%       alpha = 1 - lambda_c ,  Var(w_T[j]) = kappa_T*a_bar[j]
%   Two memory states m_1[k] = w_T[k-1], m_2[k] = w_T[k-2] are appended at
%   slots 8, 9, so the state becomes 9-dimensional per axis and the only
%   remaining innovations are w_T[k] (variance kappa_T*a_bar_hat[k], the
%   CURRENT step alone -- no history term) and n_w[k-d]. Then
%       F_e(3,8) = F_e(3,9) = -alpha ,  F_e(4,8) = F_e(4,9) = +a_bar'*alpha
%       row 8 = 0 ,  F_e(9,8) = 1
%       Q = s2T*(g_T g_T') + s2n*(g_n g_n') ,
%           g_T = [0 0 -1 a_bar' 0 0 0 1 0]' , g_n = alpha*[0 0 -1 a_bar' 0 0 0 0 0]'
%   Q is RANK 2 and its off-diagonals are load-bearing: a diagonal-only Q
%   would silently reproduce C(1) = alpha^2*s2T and C(2) = 0 instead of the
%   correct C(1) = alpha*(1+alpha)*s2T and C(2) = alpha*s2T. P[0] correspondingly
%   comes from a 5x5 DARE over [dw_1 dw_2 dw_3 m_1 m_2] mapped into slots
%   [1 2 3 8 9]; the gain/theta block is untouched and starts uncorrelated
%   with the memory states.
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
%   LOCK FLAGS. ctrl_const.lock_da pins the disturbance at its seed (default
%   false = estimate it). Slots 6-7 do not exist in this writing and are held
%   locked at all times. Lock semantics are the ancestor's: the Jacobian is
%   zeroed at the source (which zeroes the H column AND the F_e row-4 column),
%   the Kalman-gain entries are zeroed in both updates, and the P row/col is
%   pinned at 0. Locked slots therefore contribute exactly nothing.
%
%   PARALLEL-AXIS LAW (ctrl_const.par_law). In the state writing the whole
%   parallel package collapses to ONE number: the wall-parallel truth needs a
%   different disturbance, da_par = 1/b_state_para - 1, because its two
%   asymptotic anchors sit further apart than the perpendicular pair. The x/y
%   axes therefore seed at (da_par, w0_par) and lock slot 5. The z axis is the
%   arm under test; x/y accuracy is out of scope for T2-T4 and is DECLARED, not
%   asserted.
%
%   ctrl_const fields specific to this fork (everything else is the ancestor's)
%       .lock_da       pin da at its seed                     (default false)
%       .da_init       da seed                                (default 0 --
%                      deliberately NOT the far-field anchor -1/9, so that
%                      da_hat[inf] -> -1/9 is an ACCEPTANCE, not an input)
%       .Pf_da_std     sqrt P0 on da; uninformative by decision (default 0.5).
%                      Its only contract is invariance: sweep it 10x and
%                      da_hat[inf] must not move.
%       .Pf_w0_std     sqrt P0 on the wall position (default 0.111, carried
%                      over from the height writing). There is no w_s state:
%                      it enters P(4,4) once via d a_bar/d w0 = -a_bar', and
%                      then propagates by the F_e(4,4) dynamics.
%       .Pf_a_floor    shape floor on sqrt P_aa               (default 0.0066)
%       .ws0_perp      nominal wall; w0 = ws0_perp - 1        (default 1)
%       .par_law       x/y run the parallel disturbance       (default false;
%                      REQUIRES .da_par, .w0_par, .Pf_a_floor_par)
%       .a_bar_floor / .a_bar_ceil   a_bar clamps    (default 0.05, 1 - 1e-4)
%       .da_clamp      [min max], must keep 1 + da > 0  (default [-0.9, 5])
%
%   a_ctrl_override (3x1, um/pN, optional): feed a chosen gain to the CONTROL
%   LAW only; the EKF still estimates a_bar_w / da.
%
%   See also: motion_control_law_formB_ws, run_formC_state

    if nargin < 6
        a_ctrl_override = [];
    end
    has_override = ~isempty(a_ctrl_override);
    if has_override
        a_ctrl_override = a_ctrl_override(:);
        assert(numel(a_ctrl_override) == 3 && all(isfinite(a_ctrl_override)) && all(a_ctrl_override > 0), ...
               'motion_control_law_formC_state:badOverride', ...
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
    persistent x_e_per_axis        % n_state x 3 EKF state (col = axis); slots 4..7 = a_bar, b, p, w_s [-]
    persistent P_per_axis          % cell{3} of n_state x n_state covariance [-]
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
    persistent Q_theta_floor a_bar_floor a_bar_ceil da_clamp ws_margin gap_floor
    persistent y2_whiten fe_row4_full use_fdet y2_off y1_gain_off
    persistent q33_dc_match q33_dc_fac
    persistent y2_echo_corr S_echo_T S_echo_n
    persistent ma2_aug alpha_ma2 n_state    % MA(2) noise-memory augmentation
    persistent lock_mask_g lock_mask_ax lock_state_idx_ax
    persistent par_law da_par w0_par w0_nominal

    % Axis roles (world frame; the wall normal is the z axis by convention)
    AX_PAR = [1, 2];    % wall-parallel axes

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
        % DIAGNOSTIC arm (b) of the eps_w MA(2) study (2026-08-01): scale the
        % THERMAL part of Q33 to its DC-matched value S(0)/C(0) =
        % (1+2a)^2/(1+2a^2), a = 1-lambda_c (2.1695 at lambda_c 0.7). The
        % n_w share is genuinely white and stays unscaled. Default OFF; the
        % honest fix is the exact MA(2) augmentation, not this knob.
        q33_dc_match = logical(get_field_default(ctrl_const, 'q33_dc_match', false));
        q33_dc_fac = 1;
        if q33_dc_match
            alpha_dc = 1 - lambda_c;
            q33_dc_fac = (1 + 2*alpha_dc)^2 / (1 + 2*alpha_dc^2);
        end
        % y2 self-echo correction (2026-08-01): the variance readout measures
        % the ACTUAL loop, which runs on the applied gain a_ctrl (= a_bar_hat),
        % so the reading responds to a control-gain error with sensitivity S
        % (loop-pole shift) and the innovation carries only (1-S) of the
        % true-gain deviation. S from the exact 6-state Lyapunov covariance of
        % the mismatched hold loop (states x, x[k-1], x[k-2], u[k-1], u[k-2],
        % EWMA mean tracker; zero tuning; validated far-field S=0.319 vs
        % paired-forcing measurement 0.323 +- 0.043). Per-step blend by the
        % thermal/noise variance shares, S = (S_T*a_bar + S_n*xi)/(a_bar+xi).
        y2_echo_corr = logical(get_field_default(ctrl_const, 'y2_echo_corr', true));
        S_echo_T = 0; S_echo_n = 0;
        if y2_echo_corr
            assert(d_delay == 2, 'motion_control_law_formC_state:echoDelay', ...
                   'y2_echo_corr closed form is derived for d = 2.');
            alE = 1 - lambda_c; epE = 1e-4;
            vE = zeros(2, 3); gE_list = [1, 1/(1+epE), 1/(1-epE)];
            for iN = 1:2
                for iG = 1:3
                    gE = gE_list(iG);
                    AE = zeros(6); BqE = zeros(6,1); BnE = zeros(6,1);
                    AE(1,1)=1; AE(1,3)=-gE*alE; AE(1,4)=-gE*alE; AE(1,5)=-gE*alE;
                    BnE(1)=-gE*alE; BqE(1)=1;
                    AE(2,1)=1; AE(3,2)=1;
                    AE(4,3)=-alE; AE(4,4)=-alE; AE(4,5)=-alE; BnE(4)=-alE;
                    AE(5,4)=1;
                    AE(6,3)=a_pd; AE(6,6)=1-a_pd; BnE(6)=a_pd;
                    if iN == 1; QE = BqE*BqE.'; extraE = 0;
                    else;       QE = BnE*BnE.'; extraE = (1-a_pd)^2; end
                    XE = reshape((eye(36) - kron(AE,AE)) \ QE(:), 6, 6);
                    cE = zeros(1,6); cE(3) = 1-a_pd; cE(6) = -(1-a_pd);
                    vE(iN,iG) = cE*XE*cE.' + extraE;
                end
            end
            S_echo_T = (log(vE(1,2)) - log(vE(1,3))) / (2*epE);
            S_echo_n = (log(vE(2,2)) - log(vE(2,3))) / (2*epE);
        end
        % Arm (c): exact MA(2) augmentation. Two noise-memory states carry
        % w_T[k-1], w_T[k-2] so eps_w becomes white-driven by construction;
        % it REPLACES the white Q33 container rather than rescaling it.
        ma2_aug = logical(get_field_default(ctrl_const, 'ma2_aug', true));
        if ma2_aug && q33_dc_match
            error('motion_control_law_formC_state:armConflict', ...
                  ['ma2_aug and q33_dc_match are mutually exclusive arms of the ', ...
                   'eps_w MA(2) study: the augmentation models the correlation ', ...
                   'exactly, so there is no white container left to DC-match.']);
        end
        alpha_ma2 = 1 - lambda_c;
        if ma2_aug
            n_state = 9;
        else
            n_state = 7;
        end
        y1_gain_off  = logical(get_field_default(ctrl_const, 'y1_gain_off', false));
        Q_theta_floor = get_field_default(ctrl_const, 'Q_theta_floor', 0);

        % --- 0C. Parallel-axis law (x/y); constants are caller-supplied ---
        % Law-origin offset on the WALL-NORMAL axis. The w_s state always means
        % the physical contact position; a boundary whose Form B representative
        % has its origin elsewhere (a cell: origin sits inside the surface, the
        % same phenomenon par_law handles for x/y) declares the shift here, so
        % slot 7 stays interpretable and the x/y one-way feed is untouched.
        % Default 1 = plane = today's behaviour, bit-identical.
        % ws0_perp is read in 0F as the nominal wall (w0 = ws0_perp - 1).

        par_law = logical(get_field_default(ctrl_const, 'par_law', false));
        da_par = 0; w0_par = 1;
        if par_law
            par_fields = {'da_par', 'w0_par', 'Pf_a_floor_par'};
            for ip = 1:numel(par_fields)
                if ~isfield(ctrl_const, par_fields{ip}) || isempty(ctrl_const.(par_fields{ip}))
                    error('motion_control_law_formC_state:missingParLaw', ...
                          ['par_law = true requires ctrl_const.%s. In the state ', ...
                           'writing the whole parallel package is ONE number, ', ...
                           'da_par = 1/b_state_para - 1, plus its seed origin ', ...
                           'w0_par and the representation floor.'], par_fields{ip});
                end
            end
            da_par = ctrl_const.da_par;
            w0_par = ctrl_const.w0_par;
        end

        % --- 0C'. Lock flags (Tier ladder; default = Tier-1, w_s pinned).
        %     The three global flags apply to ALL axes; par_law adds a full
        %     (b, p, w_s) lock on x/y on top, so the mask is PER AXIS.
        % Slot 5 = da. Slots 6-7 do not exist in this writing; they are held
        % locked so the 9-slot layout (and the MA(2) block at [8 9]) carries
        % over unchanged. Locked slots get zero P0, zero J and zero K, so
        % they are provably inert -- the filter IS the 5-state of the tex.
        lock_da = logical(get_field_default(ctrl_const, 'lock_da', false));
        lock_mask_g  = [lock_da; true; true];         % slot order 5, 6, 7
        lock_mask_ax = repmat(lock_mask_g, 1, 3);
        if par_law
            lock_mask_ax(1, AX_PAR) = true;           % x/y ride the fixed da_par
        end
        lock_state_idx_ax = cell(3, 1);
        for ax = 1:3
            lock_state_idx_ax{ax} = 4 + find(lock_mask_ax(:, ax));   % 5..7
        end

        % --- 0D. Validity clamps (numerical guards only, not tuning) ---
        a_bar_floor = get_field_default(ctrl_const, 'a_bar_floor', 0.05);
        % a_bar_ceil keeps 1 - a_bar > 0. The law has an attractor at a_bar = 1
        % (a_bar' -> 0), so the continuous flow never crosses; only a forward
        % Euler overshoot could, and that needs (1-a)(1+da)*dw > 1, i.e. dw of
        % order 2 per step against ~0.0043 in the canonical run (~500x margin).
        % Cheap insurance, never binds.
        a_bar_ceil  = get_field_default(ctrl_const, 'a_bar_ceil', 1 - 1e-4);
        da_clamp    = get_field_default(ctrl_const, 'da_clamp', [-0.9, 5]);  % 1 + da > 0
        ws_margin   = get_field_default(ctrl_const, 'ws_margin', 1e-3);
        gap_floor   = ws_margin;      % keeps w_bar - w0 > 0 at the seed

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

        % --- 0F. Seeds ---
        % da_init = 0 is the LAW-AS-WRITTEN seed, not an anchor: the far-field
        % value da = -1/9 (b = 9/8) is deliberately NOT fed in, so that
        % da_hat[inf] -> -1/9 is an acceptance rather than an input
        % (formC_state_da.tex, seed section; user decision (i) 2026-08-11).
        w0_nominal = get_field_default(ctrl_const, 'ws0_perp', 1) - 1;   % nominal wall
        seed_da = expand3(get_field_default(ctrl_const, 'da_init', 0));

        % --- 0G. P0 widths ---
        % Pf_da_std  : prior on the disturbance. Uninformative by decision;
        %              its only contract is the invariance test (sweep 10x,
        %              da_hat[inf] must not move).
        % Pf_w0_std  : the wall-position prior, carried over unchanged from
        %              the height writing. There is no w_s state to hold it --
        %              it enters P(4,4) once, through (d a_bar / d w0) = -a_bar'.
        Pf_da_std  = expand3(get_field_default(ctrl_const, 'Pf_da_std',  0.5));
        Pf_w0_std  = expand3(get_field_default(ctrl_const, 'Pf_w0_std',  0.111));
        Pf_a_floor = expand3(get_field_default(ctrl_const, 'Pf_a_floor', 0.0066));
        if par_law
            Pf_a_floor(AX_PAR) = ctrl_const.Pf_a_floor_par;
        end

        % --- 0H. Seed evaluation height (from p0, exactly the 7a pattern) ---
        w_bar_seed = Inf;
        if enable_wall && isfield(params, 'common') && isfield(params.common, 'p0')
            w_bar_seed = (dot(params.common.p0(:), w_hat_n) - pz_wall) / R_radius;   % [U1]
        end
        enable_seed = enable_wall && isfinite(w_bar_seed);

        % --- 0I. EKF state + covariance init (non-diagonal gain/theta block) ---
        x_e_per_axis = zeros(n_state, 3);
        P_per_axis   = cell(3, 1);
        a_bar_seed_v = zeros(3, 1);
        P_aa_v       = zeros(3, 1);
        P_bb0_v      = zeros(3, 1);
        P_pp0_v      = zeros(3, 1);
        P_ws0_v      = zeros(3, 1);

        F3 = [0 1 0; 0 0 1; 0 0 lambda_c];
        H3 = [1 0 0];

        for ax = 1:3
            is_par_ax = par_law && any(ax == AX_PAR);
            free_da = double(~lock_mask_ax(1, ax));
            if is_par_ax
                da_law = da_par;   w0_law = w0_par;
            else
                da_law = seed_da(ax);   w0_law = w0_nominal;
            end
            % Integrated law at the start height. w0 (the nominal wall) enters
            % HERE and nowhere else in the whole controller: it sets the
            % integration constant, which is what a_bar_hat[0] IS.
            [a_bar_seed, dA_dda, dA_dw0] = ...
                local_seed_level_formC(w_bar_seed, da_law, w0_law, enable_seed, gap_floor);
            a_bar_seed = min(max(a_bar_seed, a_bar_floor), a_bar_ceil);
            a_bar_seed_v(ax) = a_bar_seed;

            P7 = zeros(n_state);
            if ma2_aug
                % Position + noise-memory block from a 5x5 DARE over
                % [dw_1 dw_2 dw_3 m_1 m_2] with the SAME rank-2 Q the run-time
                % loop uses (thermal + n_w feedthrough), evaluated at the seed
                % gain; mapped into the 9-slot layout [1 2 3 8 9].
                s2T_seed = kappa_T * a_bar_seed;
                g5T = [0; 0; -1; 1; 0];
                g5n = [0; 0; -alpha_ma2; 0; 0];
                F5  = [0 1 0 0 0; ...
                       0 0 1 0 0; ...
                       0 0 lambda_c -alpha_ma2 -alpha_ma2; ...
                       0 0 0 0 0; ...
                       0 0 0 1 0];
                H5  = [1 0 0 0 0];
                Q5  = s2T_seed * (g5T * g5T.') + sigma2_n_nd(ax) * (g5n * g5n.');
                P5  = solve_dare_kf_local(F5, H5, Q5, sigma2_n_nd(ax));
                ma2_idx = [1, 2, 3, 8, 9];
                P7(ma2_idx, ma2_idx) = P5;
            else
                % Same Q33 container as run time (S8 revision 2026-08-01),
                % evaluated at the seed where the whole history equals the seed.
                Q3 = zeros(3);
                Q3(3, 3) = q33_dc_fac * kappa_T * a_bar_seed ...
                               * (1 + d_delay * (1 - lambda_c)^2) ...
                           + (1 - lambda_c)^2 * sigma2_n_nd(ax);
                P7(1:3, 1:3) = solve_dare_kf_local(F3, H3, Q3, sigma2_n_nd(ax));
            end
            % Two exact flow sensitivities, both at fixed w_bar (tex, seed
            % section): d a_bar/d da = (1-a_bar)/(1+da), d a_bar/d w0 = -a_bar'.
            % The wall prior has no state of its own, so it lands in P(4,4)
            % once and then propagates by the F_e(4,4) dynamics.
            v_da = dA_dda * free_da;
            P7(4, 4) = v_da^2 * Pf_da_std(ax)^2 ...
                     + dA_dw0^2 * Pf_w0_std(ax)^2 ...
                     + Pf_a_floor(ax)^2;
            P7(4, 5) = v_da * Pf_da_std(ax)^2;
            P7(5, 4) = P7(4, 5);
            P7(5, 5) = free_da * Pf_da_std(ax)^2;
            % slots 6-7 stay exact zeros (inert)
            P_aa_v(ax)  = P7(4, 4);
            P_bb0_v(ax) = P7(5, 5);
            P_pp0_v(ax) = 0;
            P_ws0_v(ax) = dA_dw0^2 * Pf_w0_std(ax)^2;   % wall share of P_aa

            % chol PD check on the free-state submatrix (locked rows/cols are
            % exact zeros by construction, so the full matrix is only PSD).
            free_idx = [1:4, 5 * ones(1, free_da)];
            if ma2_aug
                free_idx = [free_idx, 8, 9];
            end
            [~, chol_flag] = chol(P7(free_idx, free_idx));
            if chol_flag ~= 0
                error('motion_control_law_formC_state:P0NotPD', ...
                      'P0 free-state block is not positive definite (axis %d).', ax);
            end

            P_per_axis{ax} = P7;
            x_e_per_axis(4, ax) = a_bar_seed;
            x_e_per_axis(5, ax) = da_law;
            x_e_per_axis(6, ax) = 0;      % inert
            x_e_per_axis(7, ax) = w0_law; % inert reporter (seed origin only)
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
            % report the ACTUAL seeded slots (x/y hold the parallel constants
            % under par_law, the requested seeds otherwise)
            diag.b_hat     = x_e_per_axis(5, :).';
            diag.p_hat     = x_e_per_axis(6, :).';
            diag.ws_hat    = x_e_per_axis(7, :).';
            diag.delta_a_hat = diag.b_hat;                        % driver-log alias
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
            diag.Pf_theta_std = [Pf_da_std, Pf_w0_std, zeros(3, 1)];
            % logging-contract additions (smoke/normalization ladder)
            % Q33 container at the seed (full Var(eps_w), S8 revision):
            % history == seed on the init call. With ma2_aug the container is
            % Q(3,3) of the augmented rank-2 Q (current thermal step only, the
            % history having moved into the memory states).
            if ma2_aug
                diag.Q33 = kappa_T * a_bar_seed_v + alpha_ma2^2 * sigma2_n_nd;
            else
                diag.Q33 = q33_dc_fac * kappa_T * a_bar_seed_v ...
                               * (1 + d_delay * (1 - lambda_c)^2) ...
                           + (1 - lambda_c)^2 * sigma2_n_nd;
            end
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
        error('motion_control_law_formC_state:unsupportedDelay', ...
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
    I7 = eye(n_state);
    t_now = (k_step - 1) * Ts;

    K_a_y2_v   = zeros(3, 1);
    K_dx_y1_v  = zeros(3, 1);
    dws_y1_v   = zeros(3, 1);   % logging only: ws update via y1, K1(7)*innov1
    dws_y2_v   = zeros(3, 1);   % logging only: ws update via y2, K2(7)*innov2
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
        a_bar_i = min(max(x_curr(4), a_bar_floor), a_bar_ceil);
        da_i    = min(max(x_curr(5), da_clamp(1)), da_clamp(2));

        % --- Gain rate and Jacobians from the STATE (tex S1/S2):
        %       a_bar'  = (1 - a_bar)^2 (1 + da)
        %       A_a     = d a_bar'/d a_bar = -2 (1 - a_bar)(1 + da)   (< 0)
        %       A_delta = d a_bar'/d da    = (1 - a_bar)^2            (> 0)
        %     No w_bar_d, no w_s_hat: the slope path reads states only, so
        %     "estimate-only" is structural here rather than by audit.
        %     A_delta never vanishes and never changes sign -- unlike the
        %     height writing's J_b, whose exact zero at gap = b sits at
        %     h_bar ~ 2.13, the canonical trough.
        lm = lock_mask_ax(:, ax);
        [a_prime_i, A_a_i, A_d_i] = local_gain_law_formC(a_bar_i, da_i, enable_wall);
        if lm(1); A_d_i = 0; end
        a_prime_v(ax) = a_prime_i;

        % --- Q (7x7): rank-1 gain block + Q33 (spec S8, at the estimate) ---
        % Q33 = Var(eps_w) in FULL (revision 2026-08-01, D3 precedent of
        % kf_canonical_spec.md S5; supersedes the current-step-only
        % container): current thermal step + thermal history weighted
        % (1-lc)^2 + (1-lc)^2 * n_w feedthrough. Only the cross-step
        % correlations of eps_w are dropped (declared, ref S8 note (i)).
        % History comes from the clamped posterior gains a_bar_hat[k-i];
        % the buffers still hold [k-1]/[k-2] here (they roll in [4]).
        if ma2_aug
            % Exact MA(2): the memory states carry w_T[k-1], w_T[k-2], so the
            % only injections left are the CURRENT thermal step and the n_w
            % feedthrough. Rank-2 by construction; the off-diagonals Q(3,8)
            % and Q(4,8) are what reproduce C(1) and C(2) correctly.
            s2T_i = kappa_T * a_bar_i;
            s2n_i = sigma2_n_nd(ax);
            gT = zeros(n_state, 1);
            gT(3) = -1;
            gT(4) = a_prime_i;
            gT(8) = 1;
            gn = zeros(n_state, 1);
            gn(3) = -alpha_ma2;
            gn(4) = a_prime_i * alpha_ma2;
            Q_i = s2T_i * (gT * gT.') + s2n_i * (gn * gn.');
        else
            if d_delay == 2
                a_bar_hist = a_ctrl_km1(ax) + a_ctrl_km2(ax);
            else
                a_bar_hist = a_ctrl_km1(ax);
            end
            Q33 = q33_dc_fac * kappa_T * (a_bar_i + one_minus_lc^2 * a_bar_hist) ...
                  + one_minus_lc^2 * sigma2_n_nd(ax);
            Q_i = zeros(n_state);
            Q_i(3, 3) = Q33;
            Q_i(3, 4) = -a_prime_i * Q33;
            Q_i(4, 3) = -a_prime_i * Q33;
            Q_i(4, 4) = a_prime_i^2 * Q33;
        end
        for j = 1:3
            if ~lm(j)
                Q_i(4 + j, 4 + j) = Q_theta_floor;   % conditioning only, default 0
            end
        end
        Q44_v(ax) = Q_i(4, 4);
        Q33_v(ax) = Q_i(3, 3);
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

        % Row 4's increment must include the MA(2) memory feedthrough, since
        % a_bar' multiplies the WHOLE bracket and a_bar' now depends on the
        % state. (The height writing could drop it from the parameter columns
        % because its a_bar' had no state dependence at all.)
        if ma2_aug
            M_tot = M_row4 + alpha_ma2 * (x_curr(8) + x_curr(9));
        else
            M_tot = M_row4;
        end
        F_dw = F_dw_km1(ax);
        F_e = local_build_F_e_formC(lambda_c, F_dw, a_prime_i, A_a_i, A_d_i, M_tot);

        % --- EKF predict (S5; constants carry over) ---
        x_pred = [x_curr(2); ...
                  x_curr(3); ...
                  lambda_c * x_curr(3); ...
                  x_curr(4) + a_prime_i * (Delta_wbar_d_km1 + one_minus_lc * x_curr(3)); ...
                  x_curr(5); ...
                  x_curr(6); ...
                  x_curr(7)];
        if ma2_aug
            % Deterministic MA(2) memory feedthrough: the -alpha*(m1+m2) share
            % of eps_w is now KNOWN state, so it leaves the noise and enters
            % the prediction. Rows 8/9 shift the memory chain (m1 <- w_T[k],
            % zero mean; m2 <- m1).
            m_sum = x_curr(8) + x_curr(9);
            x_pred(3) = x_pred(3) - alpha_ma2 * m_sum;
            x_pred(4) = x_pred(4) + a_prime_i * alpha_ma2 * m_sum;
            x_pred = [x_pred; 0; x_curr(8)];

            F_aug = zeros(n_state);
            F_aug(1:7, 1:7) = F_e;
            F_aug(3, 8) = -alpha_ma2;
            F_aug(3, 9) = -alpha_ma2;
            F_aug(4, 8) = a_prime_i * alpha_ma2;
            F_aug(4, 9) = a_prime_i * alpha_ma2;
            F_aug(9, 8) = 1;            % row 8 stays all-zero (m1 <- pure noise)
            F_e = F_aug;
        end
        P_pred = F_e * P_curr * F_e' + Q_i;
        P_pred = 0.5 * (P_pred + P_pred');
        P_pred = freeze_locked_P(P_pred, lock_state_idx_ax{ax});

        % --- Sequential 1-D measurement updates (R diagonal => exact) ---
        freeze_gain = G1;

        % (a) y1 = dw_m observes dw_1 (memory states are unobserved: zeros)
        H1 = [1, zeros(1, n_state - 1)];
        S1 = H1 * P_pred * H1' + R1_i;
        K1 = (P_pred * H1') / S1;
        if freeze_gain || y1_gain_off; K1(4:7) = 0; end
        K1(lock_state_idx_ax{ax}) = 0;
        innov1 = delta_w_m(ax) - H1 * x_pred;
        innov_y1_v(ax) = innov1;          % logging only (whiteness diagnostic)
        x_upd  = x_pred + K1 * innov1;
        ImKH1  = I7 - K1 * H1;
        P_upd  = ImKH1 * P_pred * ImKH1' + K1 * R1_i * K1';   % Joseph form
        P_upd  = 0.5 * (P_upd + P_upd');
        K_dx_y1_v(ax) = K1(3);
        dws_y1_v(ax)  = K1(7) * innov1;

        % (b) y2 = gain readout (whitened increment by default).
        %     a_wm_km1 is shifted every step regardless, so the increment
        %     stays valid when the gate reopens.
        if ~gate_off(ax) && ~y2_off
            % H row 2 (tex S7). The height writing had dy2/da_bar EXACTLY 1
            % because a_bar' did not depend on the state; here it does, so the
            % delay back-off carries A_a through:
            %     dy2/da_bar = 1 - A_a * Grad ,   dy2/dda = -A_delta * Grad
            % Numerically this is a ~0.8% correction at canonical speeds; the
            % identification of da runs through F_e(4,5), not through here.
            echo_fac = 1;
            if y2_echo_corr
                S_i = (S_echo_T * a_bar_i + S_echo_n * xi_bar(ax)) ...
                      / (a_bar_i + xi_bar(ax));
                echo_fac = 1 - S_i;
            end
            H2 = H2_scale * echo_fac * [0, 0, 0, ...
                             1 - Grad_wbar_d * A_a_i, ...
                             -Grad_wbar_d * A_d_i, ...
                             0, 0, zeros(1, n_state - 7)];
            % NONLINEAR predicted measurement (S7 innovation line); H2*x_upd
            % would be wrong here -- see the header. The echo share S of the
            % reading tracks the APPLIED gain (= the estimate), so the
            % prediction keeps the full a_bar_hat term and only the true-gain
            % back-off scales by (1-S).
            y2_pred = H2_scale * (x_upd(4) - echo_fac * a_prime_i * Grad_wbar_d);
            S2  = H2 * P_upd * H2' + R2_i;
            K2  = (P_upd * H2') / S2;
            if freeze_gain; K2(4:7) = 0; end
            K2(lock_state_idx_ax{ax}) = 0;
            innov2 = y2(ax) - y2_pred;
            x_upd  = x_upd + K2 * innov2;
            ImKH2  = I7 - K2 * H2;
            P_upd  = ImKH2 * P_upd * ImKH2' + K2 * R2_i * K2';   % Joseph form
            P_upd  = 0.5 * (P_upd + P_upd');
            K_a_y2_v(ax)   = K2(4);
            innov_y2_v(ax) = innov2;
            dws_y2_v(ax)   = K2(7) * innov2;
        end

        % --- Validity clamps (numerical guards, not tuning). Locked slots
        %     are never touched (K entries zeroed, predict identity), so
        %     they stay exactly at the seed.
        x_upd(4) = min(max(x_upd(4), a_bar_floor), a_bar_ceil);
        if ~lm(1)
            x_upd(5) = min(max(x_upd(5), da_clamp(1)), da_clamp(2));
        end
        P_upd = freeze_locked_P(P_upd, lock_state_idx_ax{ax});

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
        diag.dws_y1         = dws_y1_v;    % [-] ws increment from the y1 update
        diag.dws_y2         = dws_y2_v;    % [-] ws increment from the y2 update
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

function [a_bar_p, A_a, A_delta] = local_gain_law_formC(a_bar, da, enable)
%LOCAL_GAIN_LAW_FORMC  Slope read off the gain STATE (tex S1/S2). No height,
%   no wall position, no law parameter -- da is a state.
%       a_bar'  = (1 - a_bar)^2 (1 + da)
%       A_a     = d a_bar'/d a_bar = -2 (1 - a_bar)(1 + da)      (< 0 always)
%       A_delta = d a_bar'/d da    = (1 - a_bar)^2               (> 0 always,
%                 no zero and no sign change -- the height writing's J_b
%                 vanishes at gap = b, i.e. at the canonical trough)
    if ~enable
        a_bar_p = 0; A_a = 0; A_delta = 0;
        return;
    end
    om      = 1 - a_bar;
    one_p   = 1 + da;
    a_bar_p = om^2 * one_p;
    A_a     = -2 * om * one_p;
    A_delta = om^2;
end


function [a_bar, dA_dda, dA_dw0] = ...
        local_seed_level_formC(w_bar, da, w0, enable, gap_floor)
%LOCAL_SEED_LEVEL_FORMC  Integrated law at the seed height (tex, seed section).
%       a_bar(w_bar) = 1 - 1/[(1+da)(w_bar - w0)]
%   w0 is the integration constant, so setting a_bar_hat[0] IS setting the
%   wall position: this is the ONLY place a nominal wall enters the controller.
%   Level Jacobians at fixed w_bar, both exact:
%       d a_bar/d da = (1 - a_bar)/(1 + da) ,   d a_bar/d w0 = -a_bar'
    if ~enable || ~isfinite(w_bar)
        a_bar = 1; dA_dda = 0; dA_dw0 = 0;
        return;
    end
    u      = max(w_bar - w0, gap_floor);
    one_p  = 1 + da;
    a_bar  = 1 - 1 / (one_p * u);
    dA_dda = (1 - a_bar) / one_p;
    dA_dw0 = -1 / (one_p * u^2);          % = -a_bar'
end


function F_e = local_build_F_e_formC(lambda_c, F_dw, a_bar_p, A_a, A_delta, M)
%LOCAL_BUILD_F_E_FORMC  7x7 error-dynamics Jacobian (tex S6).
%   cols: 1=dw1 2=dw2 3=dw3 4=a_bar 5=da 6,7=inert (locked, zero P0/J/K)
%   Row 4 = [0 0 (1-lc)a_bar'  1 + a_bar' F_dw + A_a M   A_delta M   0 0]
%   The A_a*M term has NO counterpart in the height writing, where a_bar' did
%   not depend on a_bar and F_e(4,4) was exactly 1 + a_bar'*F_dw.
%   M = the FULL row-4 increment (command step + (1-lc) dw_3_hat + MA(2)
%   memory feedthrough) -- exactly what a_bar' multiplies in predict.
    one_minus_lc = 1 - lambda_c;
    F_e = zeros(7);
    F_e(1, 2) = 1;
    F_e(2, 3) = 1;
    F_e(3, 3) = lambda_c;   F_e(3, 4) = -F_dw;
    F_e(4, 3) = one_minus_lc * a_bar_p;
    F_e(4, 4) = 1 + a_bar_p * F_dw + A_a * M;
    F_e(4, 5) = A_delta * M;
    F_e(5, 5) = 1;
    F_e(6, 6) = 1;
    F_e(7, 7) = 1;
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
    d.dws_y1            = zeros(3, 1);
    d.dws_y2            = zeros(3, 1);
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
