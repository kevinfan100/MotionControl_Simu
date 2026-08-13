% STATUS: ACTIVE | SSOT derivation: reference/eq17_analysis/derivation/formC_ugap.tex
%          (+ companion formC_ugap_ref.tex for the caveats and the two
%          well-posedness constraints implemented here)
% FORK OF model/controller/motion_control_law_formC_dist.m @ d0490c3 | PURPOSE:
%   the SAME gain law in the straightening coordinate u = 1/(1 - a_bar).
%   The state slot that held a_bar now holds u; du/d w_bar = 1 + delta a is
%   CONSTANT in w_bar, so the one-step quadrature is exact and the predict
%   increment never reads the gain state | EXPIRES: when the u-coordinate /
%   a_bar-coordinate adjudication is settled | production changes do NOT
%   follow.
%   Slot map: 4 = u (gap to the implied wall), 5 = delta a (MULTIPLICATIVE on
%   du/d w_bar); slots 6-7 are PERMANENTLY LOCKED and inert, so the 9-slot
%   layout and the whole MA(2) block at [8 9] carry over with no re-indexing.
%   With lock_da = true it is the 4-state derivation (a); with lock_da = false
%   the 5-state derivation (b) -- same file, one flag.
function [f_d, ekf_out, diag] = motion_control_law_formC_ugap(del_pd, pd, p_m, params, ctrl_const, a_ctrl_override)
%MOTION_CONTROL_LAW_FORMC_UGAP  Per-axis EKF eq17 controller written in the
%   gap coordinate u = 1/(1 - a_bar) (formC_ugap.tex)
%
%       u := 1/(1 - a_bar) ,   a_bar = 1 - 1/u                        (S1)
%       d a_bar / d w_bar = (1 - a_bar)^2 (1 + delta a)
%           <=>  d u / d w_bar = 1 + delta a                          (S1)
%       u(w_bar) = (1 + delta a)(w_bar - w_bar_0)                     (S1)
%       a_w = a_o * a_bar ,   a_o = Ts/(gamma_N*R)  [1/pN]   (fixed, not a state)
%
%   Authoritative spec: reference/eq17_analysis/derivation/formC_ugap.tex
%   (S1-S8 + seed section), which carries BOTH derivations side by side:
%       (a) BASELINE     4 states [dw1 dw2 dw3 u],      delta a locked at 0
%       (b) WITH delta a 5 states [dw1 dw2 dw3 u delta a], delta a[k+1] = delta a[k]
%   This file is both: ctrl_const.lock_da selects (a) (slot 5 pinned at its
%   seed, zero P0, zero Jacobian, zero K -> provably inert) or (b).
%
%   WHAT THE COORDINATE CHANGES, RELATIVE TO THE a_bar WRITING
%   (motion_control_law_formC_dist / _formC_state; ref S "Why this coordinate")
%       predict     u + (1+da)*Sigma_hat        vs  a_bar + a_bar'(a_bar)*Sigma_hat
%                   -- the increment READS NO STATE, so the forward-Euler
%                      quadrature is EXACT, not first-order: the one-signed
%                      ratchet (+1.6% at the end of the descent, +2.1% at the
%                      end of the oscillation on the canonical command) is
%                      removed IDENTICALLY, not reduced.
%       consume     a_bar_hat = 1 - 1/u_hat     (control law and S7 read this)
%       F_e(4,4)    1 + (1+da)F_dw (1-a_bar)^2  vs  1 + a_bar' F_dw + A_a*M
%                   -- the self-amplification term A_a*M is NOT abolished, it
%                      MOVES to the output map d a_bar/du = u^-2, where it
%                      does not compound step to step (ref, "What it does not
%                      buy").
%       F_e(4,5)    Sigma_hat  (accumulated displacement: a delta a error is a
%                   RAMP, a gap error is an OFFSET -- clean separation)
%       H(2,*)      [0 0 0  v_hat^-2  -Grad*v_hat^-2] with the delay back-off
%                   done in u, where it is affine and therefore exact up to
%                   the one DECLARED truncation (the (dw_3 - dw_1) channel).
%       Q           Q_3u = -(1+da)Q33 , Q_uu = (1+da)^2 Q33 , Q_dada = 0
%       R2          delay term d * v_hat^-4 * Q_uu (the a_bar-space image of
%                   the u-space process noise over the back-off)
%   No information is created: (u, delta a) <-> (w_bar_0, delta a) is a known
%   bijection at fixed w_bar and Fisher information is invariant under smooth
%   reparametrisation. What changes is conditioning and the quadrature.
%
%   TWO WELL-POSEDNESS CONSTRAINTS (formC_ugap_ref, seed section). They are
%   NOT tuning; the arm is ill-posed without them:
%     (i)  Pf_a_floor MUST be the LOCAL shape error of THIS family at the seed
%          height, not the envelope supremum. The floor enters P_uu[0] through
%          u_hat[0]^4, so at u_hat[0] = 22.22 the envelope sup 0.0306 gives
%          sigma_u = 15.11 against a mean of 22.22 -- 8% of the prior mass at
%          u < 1, i.e. a_bar < 0, sitting ON the singularity of 1 - 1/u. The
%          local value 0.0056 gives sigma_u = 2.77 and that mass vanishes. The
%          driver computes both from calc_correction_functions and passes the
%          local one; the controller never touches a truth curve.
%     (ii) u >= u_min and v_hat >= u_min (default 1.5 = h_bar_safe, the same
%          house near-wall value). The clamp counts are reported in diag; a
%          non-zero count IS the arm failing, not a guard doing its job.
%
%   Everything below this line is inherited verbatim from the a_bar-coordinate
%   fork and, through it, from motion_control_law_formB_ws.
%
%   FULLY NORMALIZED INTERNALS (formB_ws_ref Conventions): every internal
%   quantity is dimensionless -- lengths divided by R, gain by a_o, forces
%   multiplied by a_o (f_bar = a_o*f, so a_bar*f_bar = a*f identically).
%   The only dimensionless thermal constant inside the loop is
%
%       kappa_T = 4*kB*T*a_o/R                                   [-]
%
%   a_o crosses the boundary ONLY at: the kappa_T definition ([U0]), the force
%   output conversion f_d = f_bar_d/a_o ([U3]), and the display layer ([U4]).
%       [U1] inputs pd / p_m / del_pd [um]          -> /R
%       [U2] sigma2_n_s [um^2]                      -> /R^2
%       [U3] force output f_bar_d [-]               -> /a_o  [pN]
%       [U4] displayed gains                        -> *a_disp (= a_o*R) [um/pN]
%
%   Per-axis state, d = 2 measurement delay, e_theta = true - estimate:
%
%       x = [dw_1; dw_2; dw_3; u; delta a; (inert); (inert) | m_1; m_2]
%
%   dw_1 = delta_w[k-2] (measured) ... dw_3 = delta_w[k] (current), in units
%   of R; u = gap to the implied wall in units of R (u > 1 <=> 0 < a_bar < 1);
%   delta a = multiplicative correction on du/d w_bar, Q_dada = 0.
%
%   Estimator PREDICT (tex S5(b)):
%       Sigma_hat[k] = Delta_wbar_d[k] + (1-lc)*dw_3_hat[k]  (+ the MA(2)
%                      memory feedthrough, which is the KNOWN share of eps_w)
%       u[k+1]       = u[k] + (1 + delta a[k]) * Sigma_hat[k]
%       delta a[k+1] = delta a[k]
%   u is NEVER re-anchored after init.
%
%   Measurements (tex S7). The gain readout is computed in ONE step with
%   kappa_T (never physical-then-divide):
%       a_bar_wm = (sigma2_dwr_hat - C_n*sigma2_nw) / (C_dpmr*kappa_T)
%   a_bar_wm is an exact AR(1) with pole 1-a_cov; the KF is fed the WHITENED
%   increment by default (ctrl_const.y2_whiten = true):
%       y1 = dw_m = dw_1 + n_w
%       y2 = a_bar_wm[k] - (1-a_cov)*a_bar_wm[k-1]
%       v_hat  = u_hat - (1 + delta a_hat) * Grad_d_wbar_d       (S7, affine)
%       y2_pred = H2_scale * [ (1-echo)*(1 - 1/u_hat) + echo*(1 - 1/v_hat) ]
%       H2      = H2_scale * echo * [0 0 0  v_hat^-2  -Grad*v_hat^-2  0 0 ...]
%   echo = 1 - S is the y2 self-echo factor (2026-08-01): the readout measures
%   the ACTUAL loop, which runs on the APPLIED gain a_bar_hat[k], so
%       E[y2] = a_bar_hat[k] + (1-S)*(a_true[k-d] - a_bar_hat[k])
%   which is exactly the convex combination above. In a_bar coordinates the
%   same identity reads a_bar_hat - (1-S)*(back-off), i.e. the sibling's
%   formula; here the back-off is not affine in a_bar, so the combination form
%   is used and the two agree term by term when the law is linearized.
%
%   Q (tex S8; run time at the current estimate):
%       Q33 = Var(eps_w) in FULL (2026-08-01, D3 precedent of
%             kf_canonical_spec.md S5); under ma2_aug the memory states carry
%             the history, so only the current thermal step and the n_w
%             feedthrough are injected and Q is RANK 2.
%       Q_3u = Q_u3 = -(1+da)*Q33 ,  Q_uu = (1+da)^2*Q33 ,  Q_dada = 0
%   R = diag(R1, R2), R2 at the CONSUMED a_bar_hat (never the raw readout):
%       R1 = sigma2_nw  (units of R^2)
%       R2 = K_var*IF_eff*(a_bar + xi_bar)^2 + d * v_hat^-4 * Q_uu
%   with xi_bar = (C_n/C_dpmr)*sigma2_nw/kappa_T. The delay term is the tex's
%   d*Q_uu mapped through the output Jacobian (d a_bar/du)^2 = v_hat^-4, so it
%   sits in the same a_bar units as the readout, exactly as the sibling's
%   d*Q44 did.
%
%   MA(2) AUGMENTATION (ctrl_const.ma2_aug, 2026-08-01), unchanged except that
%   the row-4 image of the noise is (1+da) instead of a_bar':
%       eps_w[k] = w_T[k] + alpha*(w_T[k-1] + w_T[k-2]) - alpha*n_w[k-d],
%       alpha = 1 - lambda_c ,  Var(w_T[j]) = kappa_T*a_bar[j]
%       F_e(3,8) = F_e(3,9) = -alpha ,  F_e(4,8) = F_e(4,9) = +(1+da)*alpha
%       row 8 = 0 ,  F_e(9,8) = 1
%       Q = s2T*(g_T g_T') + s2n*(g_n g_n') ,
%           g_T = [0 0 -1 (1+da) 0 0 0 1 0]' , g_n = alpha*[0 0 -1 (1+da) 0 0 0 0 0]'
%   P[0] correspondingly comes from a 5x5 DARE over [dw_1 dw_2 dw_3 m_1 m_2]
%   mapped into slots [1 2 3 8 9].
%
%   INIT (tex seed section; every number derived or declared, none tuned):
%       seed   u_hat[0]  = w_bar[0] - w0_hat   (= 1/(1 - a_bar_seed): setting
%                          u_hat[0] IS setting the integration constant, so the
%                          nominal wall w0_hat enters HERE and nowhere else)
%              da_hat[0] = 0
%       P0     P_uu[0]    = (u_hat[0]/(1+da_hat))^2 * Pf_da_std^2
%                         + (1+da_hat)^2 * Pf_w0_std^2
%                         + u_hat[0]^4 * Pf_a_floor^2
%              P_u_da[0]  = (u_hat[0]/(1+da_hat)) * Pf_da_std^2   <- NOT zero:
%                          delta a enters the LAW, so its prior enters u at
%                          k = 0 (this is error 1 of the ref's verification
%                          section, inherited from the wrong sibling)
%              P_dada[0]  = Pf_da_std^2
%              Position 3-block (and the MA(2) memory block) from the DARE
%              steady state. P0 is chol-checked positive definite on the
%              FREE-state submatrix and the controller errors out if it fails.
%
%   LOCK FLAGS. ctrl_const.lock_da pins delta a at its seed (true = derivation
%   (a), the 4-state baseline). Slots 6-7 are held locked at all times. Lock
%   semantics are the ancestor's: the Jacobian is zeroed at the source, the
%   Kalman-gain entries are zeroed in both updates, and the P row/col is
%   pinned at 0.
%
%   PARALLEL-AXIS LAW (ctrl_const.par_law). The wall-parallel truth is not a
%   member of this family either, so the parallel package is the integration
%   constant alone: x/y seed u at w_bar[0] - w0_par and lock slot 5 at 0. x/y
%   accuracy is DECLARED out of scope; the z axis is the arm under test and
%   the axes are dynamically decoupled (w_hat = z => Gamma_inv diagonal).
%
%   ctrl_const fields specific to this fork (everything else is the ancestor's)
%       .lock_da       pin delta a at its seed = derivation (a)  (default false)
%       .da_init       delta a seed                              (default 0)
%       .Pf_da_std     sqrt P0 on delta a (default 0.5, carried over from the
%                      a_bar-coordinate sibling run_formC_state; the ref's seed
%                      section prices P_uu[0] with exactly this value). Its
%                      contract is INVARIANCE: sweep it 10x either way and the
%                      converged behaviour must not move.
%       .Pf_w0_std     sqrt P0 on the wall position (default 0.111). There is
%                      no w_0 state: it enters P(4,4) once via du/dw_0 = -(1+da).
%       .Pf_a_floor    LOCAL shape floor at the seed height (default 0.0056;
%                      constraint (i) above -- the driver derives it)
%       .u_min         gap clamp on u and v_hat        (default 1.5)
%       .ws0_perp      nominal wall; w0 = ws0_perp - 1 (default 1)
%       .par_law       x/y run the fitted parallel origin (default false;
%                      REQUIRES .w0_par, .Pf_a_floor_par)
%       .da_clamp      [min max] on delta a; must keep 1 + da > 0
%                      (default [-0.9, 5], the sibling's structural bound)
%
%   a_ctrl_override (3x1, um/pN, optional): feed a chosen gain to the CONTROL
%   LAW only; the EKF still estimates u / delta a.
%
%   See also: motion_control_law_formC_dist, motion_control_law_formC_state,
%             motion_control_law_formB_ws, run_formC_ugap

    if nargin < 6
        a_ctrl_override = [];
    end
    has_override = ~isempty(a_ctrl_override);
    if has_override
        a_ctrl_override = a_ctrl_override(:);
        assert(numel(a_ctrl_override) == 3 && all(isfinite(a_ctrl_override)) && all(a_ctrl_override > 0), ...
               'motion_control_law_formC_ugap:badOverride', ...
               'a_ctrl_override must be a 3x1 finite positive vector [um/pN].');
    end

    % ------------------------------------------------------------------
    % Open-loop bypass
    % ------------------------------------------------------------------
    if params.ctrl.enable < 0.5
        f_d = zeros(3, 1);
        ekf_out = [1; 1; 1; 0];
        if nargout >= 3
            diag = empty_diag_ugap();
            diag.f_d = f_d;
        end
        return;
    end

    % ------------------------------------------------------------------
    % Persistent state  (all EKF/IIR quantities FULLY NORMALIZED)
    % ------------------------------------------------------------------
    persistent x_e_per_axis        % n_state x 3 EKF state (col = axis); slot 4 = u, 5 = delta a
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
    persistent n_clamp_u n_clamp_v n_update_steps   % well-posedness constraint (ii)

    persistent initialized
    persistent lambda_c d_delay Ts kappa_T R_radius a_o a_disp
    persistent a_pd a_cov C_dpmr C_n K_var IF_abc xi_bar amlpf_var_factor
    persistent t_warmup_kf h_bar_safe sigma2_n_nd
    persistent enable_wall w_hat_n pz_wall
    persistent Q_theta_floor a_bar_floor a_bar_ceil da_clamp u_min gap_floor
    persistent y2_whiten fe_row4_full use_fdet y2_off y1_gain_off lambda_f
    persistent y2_echo_corr S_echo_T S_echo_n
    persistent ma2_aug alpha_ma2 n_state    % MA(2) noise-memory augmentation
    persistent lock_mask_g lock_mask_ax lock_state_idx_ax
    persistent par_law w0_par w0_nominal

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
        kappa_T = 4 * (params.ctrl.k_B * params.ctrl.T / R_radius) * a_o;   % [-]
        a_disp  = a_o * R_radius;             % [um/pN] display scale per unit a_bar

        % --- 0B. ctrl_const (offline scalars; shared with the family) ---
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
        xi_bar = (C_n / C_dpmr) * sigma2_n_nd / kappa_T;

        y2_whiten    = logical(get_field_default(ctrl_const, 'y2_whiten', true));
        fe_row4_full = logical(get_field_default(ctrl_const, 'fe_row4_full', true));
        use_fdet     = logical(get_field_default(ctrl_const, 'use_fdet', true));
        y2_off       = logical(get_field_default(ctrl_const, 'y2_off', false));
        % y2 self-echo correction (2026-08-01): the variance readout measures
        % the ACTUAL loop, which runs on the applied gain a_ctrl (= a_bar_hat),
        % so the reading responds to a control-gain error with sensitivity S
        % (loop-pole shift). S from the exact 6-state Lyapunov covariance of
        % the mismatched hold loop; zero tuning (far-field S = 0.319 against a
        % paired-forcing measurement 0.323 +- 0.043).
        y2_echo_corr = logical(get_field_default(ctrl_const, 'y2_echo_corr', true));
        S_echo_T = 0; S_echo_n = 0;
        if y2_echo_corr
            assert(d_delay == 2, 'motion_control_law_formC_ugap:echoDelay', ...
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
        % Exact MA(2) augmentation: two noise-memory states carry w_T[k-1],
        % w_T[k-2] so eps_w becomes white-driven by construction.
        ma2_aug = logical(get_field_default(ctrl_const, 'ma2_aug', true));
        alpha_ma2 = 1 - lambda_c;
        if ma2_aug
            n_state = 9;
        else
            n_state = 7;
        end
        y1_gain_off  = logical(get_field_default(ctrl_const, 'y1_gain_off', false));
        % Menq (4.15) forgetting factor. Falsified 2026-08-12 and NOT adopted;
        % kept only so the invariance/instrument sweeps stay callable.
        % lambda_f = 1 disables it exactly (default).
        lambda_f = get_field_default(ctrl_const, 'lambda_f', 1);
        assert(isscalar(lambda_f) && lambda_f > 0 && lambda_f <= 1, ...
               'motion_control_law_formC_ugap:lambdaF', ...
               'lambda_f must be a scalar in (0, 1]; got %g.', lambda_f);
        Q_theta_floor = get_field_default(ctrl_const, 'Q_theta_floor', 0);

        % --- 0C. Parallel-axis law (x/y); constants are caller-supplied ---
        par_law = logical(get_field_default(ctrl_const, 'par_law', false));
        w0_par = 1;
        if par_law
            par_fields = {'w0_par', 'Pf_a_floor_par'};
            for ip = 1:numel(par_fields)
                if ~isfield(ctrl_const, par_fields{ip}) || isempty(ctrl_const.(par_fields{ip}))
                    error('motion_control_law_formC_ugap:missingParLaw', ...
                          ['par_law = true requires ctrl_const.%s. The parallel ', ...
                           'package in this writing is the seed origin w0_par ', ...
                           'plus the representation floor.'], par_fields{ip});
                end
            end
            w0_par = ctrl_const.w0_par;
        end

        % --- 0C'. Lock flags. lock_da = true IS derivation (a), the 4-state
        %     baseline: slot 5 pinned at its seed with zero P0, zero Jacobian
        %     and zero K, hence provably inert. Slots 6-7 do not exist in this
        %     writing and are held locked at all times.
        lock_da = logical(get_field_default(ctrl_const, 'lock_da', false));
        lock_mask_g  = [lock_da; true; true];         % slot order 5, 6, 7
        lock_mask_ax = repmat(lock_mask_g, 1, 3);
        if par_law
            lock_mask_ax(1, AX_PAR) = true;           % x/y carry no delta a
        end
        lock_state_idx_ax = cell(3, 1);
        for ax = 1:3
            lock_state_idx_ax{ax} = 4 + find(lock_mask_ax(:, ax));   % 5..7
        end

        % --- 0D. Validity clamps ---
        % a_bar_floor / a_bar_ceil are inherited guards on the CONSUMED gain;
        % with u_min = 1.5 the consumed a_bar is in [1/3, 1) by construction,
        % so they never bind. u_min is the load-bearing one (constraint (ii)).
        a_bar_floor = get_field_default(ctrl_const, 'a_bar_floor', 0.05);
        a_bar_ceil  = get_field_default(ctrl_const, 'a_bar_ceil', 1 - 1e-4);
        % delta a multiplies du/d w_bar, so 1 + delta a > 0 is structural.
        da_clamp    = get_field_default(ctrl_const, 'da_clamp', [-0.9, 5]);
        % u >= u_min and v_hat >= u_min. u -> 1 is the singularity of the
        % output map 1 - 1/u; h_bar_safe = 1.5 is the house near-wall value and
        % the natural u_min (ref, seed section). ANY binding is a failure of
        % the arm, so the counts are reported.
        u_min       = get_field_default(ctrl_const, 'u_min', 1.5);
        gap_floor   = get_field_default(ctrl_const, 'ws_margin', 1e-3);

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
        w0_nominal = get_field_default(ctrl_const, 'ws0_perp', 1) - 1;   % nominal wall
        seed_da = expand3(get_field_default(ctrl_const, 'da_init', 0));

        % --- 0G. P0 widths ---
        Pf_da_std  = expand3(get_field_default(ctrl_const, 'Pf_da_std',  0.5));
        Pf_w0_std  = expand3(get_field_default(ctrl_const, 'Pf_w0_std',  0.111));
        Pf_a_floor = expand3(get_field_default(ctrl_const, 'Pf_a_floor', 0.0056));
        if par_law
            Pf_a_floor(AX_PAR) = ctrl_const.Pf_a_floor_par;
        end

        % --- 0H. Seed evaluation height (from p0) ---
        w_bar_seed = Inf;
        if enable_wall && isfield(params, 'common') && isfield(params.common, 'p0')
            w_bar_seed = (dot(params.common.p0(:), w_hat_n) - pz_wall) / R_radius;   % [U1]
        end
        enable_seed = enable_wall && isfinite(w_bar_seed);

        % --- 0I. EKF state + covariance init (non-diagonal u/delta a block) ---
        x_e_per_axis = zeros(n_state, 3);
        P_per_axis   = cell(3, 1);
        u_seed_v     = zeros(3, 1);
        a_bar_seed_v = zeros(3, 1);
        P_uu_v       = zeros(3, 1);
        P_dada0_v    = zeros(3, 1);
        P_w0_share_v = zeros(3, 1);

        F3 = [0 1 0; 0 0 1; 0 0 lambda_c];
        H3 = [1 0 0];

        for ax = 1:3
            is_par_ax = par_law && any(ax == AX_PAR);
            free_da = double(~lock_mask_ax(1, ax));
            da_law = seed_da(ax);
            if is_par_ax
                da_law = 0;   w0_law = w0_par;   % x/y carry no delta a
            else
                w0_law = w0_nominal;
            end
            % Seed the GAP directly (tex seed section): u_hat[0] = w_bar[0] - w0.
            % w0 (the nominal wall) enters HERE and nowhere else in the whole
            % controller: it IS the integration constant.
            u_seed = local_seed_gap_ugap(w_bar_seed, w0_law, enable_seed, gap_floor);
            u_seed = max(u_seed, u_min);
            u_seed_v(ax) = u_seed;
            a_bar_seed = 1 - 1 / u_seed;
            a_bar_seed_v(ax) = min(max(a_bar_seed, a_bar_floor), a_bar_ceil);

            g_seed = 1 + da_law;                 % du/d w_bar at the seed

            P7 = zeros(n_state);
            if ma2_aug
                % Position + noise-memory block from a 5x5 DARE over
                % [dw_1 dw_2 dw_3 m_1 m_2] with the SAME rank-2 Q the run-time
                % loop uses, evaluated at the seed gain; mapped into [1 2 3 8 9].
                s2T_seed = kappa_T * a_bar_seed_v(ax);
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
                Q3 = zeros(3);
                Q3(3, 3) = kappa_T * a_bar_seed_v(ax) ...
                               * (1 + d_delay * (1 - lambda_c)^2) ...
                           + (1 - lambda_c)^2 * sigma2_n_nd(ax);
                P7(1:3, 1:3) = solve_dare_kf_local(F3, H3, Q3, sigma2_n_nd(ax));
            end
            % Seed prior (tex seed section). THREE contributions to P_uu[0]:
            %   (delta a prior) pushed through du/d(delta a) = u/(1+da)
            %   (wall prior)    pushed through du/dw_0     = -(1+da)
            %   (shape floor)   pushed through du/d a_bar  = u^2
            % and the delta a prior ALSO lands off-diagonal: P_u_da[0] =
            % (u/(1+da)) * P_dada is NOT zero, because delta a enters the LAW.
            J_u_da  = u_seed / g_seed;
            J_u_w0  = g_seed;
            P_dada0 = free_da * Pf_da_std(ax)^2;
            P7(4, 4) = J_u_da^2 * P_dada0 ...
                     + J_u_w0^2 * Pf_w0_std(ax)^2 ...
                     + u_seed^4 * Pf_a_floor(ax)^2;
            P7(4, 5) = J_u_da * P_dada0;
            P7(5, 4) = P7(4, 5);
            P7(5, 5) = P_dada0;
            % slots 6-7 stay exact zeros (inert)
            P_uu_v(ax)       = P7(4, 4);
            P_dada0_v(ax)    = P7(5, 5);
            P_w0_share_v(ax) = J_u_w0^2 * Pf_w0_std(ax)^2;   % wall share of P_uu

            % chol PD check on the free-state submatrix (locked rows/cols are
            % exact zeros by construction, so the full matrix is only PSD).
            free_idx = [1:4, 5 * ones(1, free_da)];
            if ma2_aug
                free_idx = [free_idx, 8, 9];
            end
            [~, chol_flag] = chol(P7(free_idx, free_idx));
            if chol_flag ~= 0
                error('motion_control_law_formC_ugap:P0NotPD', ...
                      'P0 free-state block is not positive definite (axis %d).', ax);
            end

            P_per_axis{ax} = P7;
            x_e_per_axis(4, ax) = u_seed;
            x_e_per_axis(5, ax) = da_law;
            x_e_per_axis(6, ax) = 0;      % inert
            x_e_per_axis(7, ax) = w0_law; % inert reporter (seed origin only)
        end

        % --- 0J. IIR states (prefill to the closed-loop dw_r variance) ---
        dw_bar_m = zeros(3, 1);
        sigma2_dwr_hat = C_dpmr * kappa_T * a_bar_seed_v + C_n * sigma2_n_nd;
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
        n_clamp_u = zeros(3, 1);
        n_clamp_v = zeros(3, 1);
        n_update_steps = 0;

        % --- 0L. First call returns zeros (no f_d yet) ---
        f_d = zeros(3, 1);
        a_hat_phys = a_bar_seed_v * a_disp;                       % [U4]
        ekf_out = [a_hat_phys(1); a_hat_phys(3); a_hat_phys(2); 0];
        if nargout >= 3
            diag = empty_diag_ugap();
            diag.f_d       = f_d;
            diag.a_hat     = a_hat_phys;
            diag.a_hat_nd  = a_bar_seed_v * a_o;                  % [U4] legacy 1/pN
            diag.a_bar_hat = a_bar_seed_v;
            diag.u_hat     = u_seed_v;
            diag.v_hat     = u_seed_v;
            diag.b_hat     = x_e_per_axis(5, :).';
            diag.p_hat     = x_e_per_axis(6, :).';
            diag.ws_hat    = x_e_per_axis(7, :).';
            diag.delta_a_hat = diag.b_hat;                        % driver-log alias
            diag.sigma2_dxr_hat = sigma2_dwr_hat;
            % P_a is reported in a_bar units, i.e. P_uu mapped through the
            % output Jacobian (d a_bar/du)^2 = u^-4, so it is directly
            % comparable with the a_bar-coordinate sibling's P44.
            P_aa_v = P_uu_v ./ u_seed_v.^4;
            diag.P_a  = P_aa_v * a_disp^2;                        % [U4] (um/pN)^2
            diag.P_a_nd = P_aa_v;
            diag.P_uu = P_uu_v;
            diag.P_b  = P_dada0_v;
            diag.P_p  = zeros(3, 1);
            diag.P_ws = P_w0_share_v ./ u_seed_v.^4;   % wall share, a_bar units
            diag.P77  = P_dada0_v;                                % legacy alias
            diag.h_bar_init = w_bar_seed;
            diag.Pf_a_floor = Pf_a_floor;
            diag.Pf_theta_std = [Pf_da_std, Pf_w0_std, zeros(3, 1)];
            if ma2_aug
                diag.Q33 = kappa_T * a_bar_seed_v + alpha_ma2^2 * sigma2_n_nd;
            else
                diag.Q33 = kappa_T * a_bar_seed_v ...
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
    % Per-step: extract per-axis state. The CONSUMED gain is the output map
    % of the coordinate, a_bar = 1 - 1/u (tex S5 last line).
    % ------------------------------------------------------------------
    u_state   = x_e_per_axis(4, :).';                 % 3x1 [-]
    u_use     = max(u_state, u_min);
    n_clamp_u = n_clamp_u + double(u_state < u_min);
    a_bar_hat = 1 - 1 ./ u_use;                       % 3x1 [-]

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
        error('motion_control_law_formC_ugap:unsupportedDelay', ...
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
        y2 = a_bar_wm - (1 - a_cov) * a_wm_km1;      % = a_cov*u_ar1[k]
        H2_scale = a_cov;
    else
        y2 = a_bar_wm;                               % raw AR(1) output (spec body)
        H2_scale = 1;
    end

    % ------------------------------------------------------------------
    % [2] Control law (eq17 implementable), NORMALIZED. Unchanged: it reads
    %     the CONSUMED gain a_bar_hat = 1 - 1/u_hat.
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
    %   regressor for F_e.
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
        F_dw_vec = F_dw_raw;      % ablation only
    end

    % --- [U3] force output conversion (the ONLY loop-adjacent a_o use) ---
    f_d = fbar_d / a_o;                          % [pN]

    % ------------------------------------------------------------------
    % [3] Per-axis Q, R, gate; EKF predict + sequential scalar updates
    %     (Joseph form), mirroring the a_bar-coordinate sibling exactly.
    % ------------------------------------------------------------------
    I7 = eye(n_state);
    t_now = (k_step - 1) * Ts;
    n_update_steps = n_update_steps + 1;

    K_a_y2_v   = zeros(3, 1);
    K_dx_y1_v  = zeros(3, 1);
    K_a_y1_v   = zeros(3, 1);   % K1(4): y1's GAIN correction (via P(4,1))
    P41_v      = zeros(3, 1);
    dws_y1_v   = zeros(3, 1);
    dws_y2_v   = zeros(3, 1);
    innov_y2_v = zeros(3, 1);
    innov_y1_v = zeros(3, 1);
    gate_off   = false(3, 1);
    G_flags    = false(3, 3);
    a_prime_v  = zeros(3, 1);
    Quu_v      = zeros(3, 1);
    Q33_v      = zeros(3, 1);
    a_barQ_v   = zeros(3, 1);
    R2_v       = zeros(3, 1);
    v_hat_v    = zeros(3, 1);
    Sigma_v    = zeros(3, 1);

    for ax = 1:3
        x_curr = x_e_per_axis(:, ax);
        P_curr = P_per_axis{ax};
        u_i     = u_use(ax);                 % already clamped and counted
        a_bar_i = min(max(a_bar_hat(ax), a_bar_floor), a_bar_ceil);
        lm = lock_mask_ax(:, ax);
        da_i = min(max(x_curr(5), da_clamp(1)), da_clamp(2));
        g_i  = 1 + da_i;                     % du/d w_bar (tex S1) -- NO state
        J_da_i = double(~lm(1));

        % Effective gain slope, for logging only. In this coordinate it is a
        % DERIVED quantity (output-map Jacobian times the constant increment),
        % never an integrand: a_bar' = (1-a_bar)^2 (1+da) = g/u^2.
        a_prime_v(ax) = g_i / u_i^2;

        % --- Q (spec S8, at the estimate) --------------------------------
        if ma2_aug
            % Exact MA(2): the memory states carry w_T[k-1], w_T[k-2], so the
            % only injections left are the CURRENT thermal step and the n_w
            % feedthrough. Row 4's image of eps_w is (1+da), not a_bar'.
            s2T_i = kappa_T * a_bar_i;
            s2n_i = sigma2_n_nd(ax);
            gT = zeros(n_state, 1);
            gT(3) = -1;
            gT(4) = g_i;
            gT(8) = 1;
            gn = zeros(n_state, 1);
            gn(3) = -alpha_ma2;
            gn(4) = g_i * alpha_ma2;
            Q_i = s2T_i * (gT * gT.') + s2n_i * (gn * gn.');
        else
            if d_delay == 2
                a_bar_hist = a_ctrl_km1(ax) + a_ctrl_km2(ax);
            else
                a_bar_hist = a_ctrl_km1(ax);
            end
            Q33 = kappa_T * (a_bar_i + one_minus_lc^2 * a_bar_hist) ...
                  + one_minus_lc^2 * sigma2_n_nd(ax);
            Q_i = zeros(n_state);
            Q_i(3, 3) = Q33;
            Q_i(3, 4) = -g_i * Q33;
            Q_i(4, 3) = -g_i * Q33;
            Q_i(4, 4) = g_i^2 * Q33;
        end
        for j = 1:3
            if ~lm(j)
                Q_i(4 + j, 4 + j) = Q_theta_floor;   % conditioning only, default 0
            end
        end
        Quu_v(ax) = Q_i(4, 4);
        Q33_v(ax) = Q_i(3, 3);
        a_barQ_v(ax) = a_bar_i;      % the a_bar actually used to build Q33

        % --- R (R2 at the CONSUMED a_bar_hat, never the raw readout) ------
        % The tex's delay term is d*Q_uu in u units; the readout lives in
        % a_bar units, so it is mapped through (d a_bar/du)^2 = v_hat^-4 at
        % the back-off point -- the exact image of the sibling's d*Q44.
        v_R = u_i - g_i * Grad_wbar_d;
        if v_R < u_min; v_R = u_min; end
        R1_i = sigma2_n_nd(ax);
        R2_i = compute_R2_formB(a_bar_i, sigma2_n_nd(ax), IF_abc, C_dpmr, C_n, ...
                                K_var, amlpf_var_factor, xi_bar(ax), kappa_T, ...
                                Q_i(4, 4) / v_R^4, d_delay, a_cov, y2_whiten);
        R2_v(ax) = R2_i;

        % --- Gates (OR): warm-up / readout NaN guard / near wall ---
        G1 = (t_now < t_warmup_kf);
        G2 = ((sigma2_dwr_hat_new(ax) - C_n * sigma2_n_nd(ax)) <= 0);
        G3 = (h_bar < h_bar_safe);
        G_flags(:, ax) = [G1; G2; G3];
        gate_off(ax) = G1 || G2 || G3;

        % --- Sigma_hat: the deterministic part of the step the TRUE position
        %     took over [k-1, k]. The predict bridges k-1 -> k, so it uses the
        %     PREVIOUS call's command step (timing-lead fix 2026-08-01). Under
        %     ma2_aug the KNOWN share of eps_w, alpha*(m_1 + m_2), is state and
        %     therefore belongs in the increment.
        if fe_row4_full
            Sigma_hat = Delta_wbar_d_km1 + one_minus_lc * x_curr(3);
        else
            Sigma_hat = Delta_wbar_d_km1;
        end
        Sigma_pred = Delta_wbar_d_km1 + one_minus_lc * x_curr(3);
        if ma2_aug
            m_sum      = x_curr(8) + x_curr(9);
            Sigma_hat  = Sigma_hat  + alpha_ma2 * m_sum;
            Sigma_pred = Sigma_pred + alpha_ma2 * m_sum;
        end
        Sigma_v(ax) = Sigma_pred;

        F_dw = F_dw_km1(ax);
        F_e = local_build_F_e_ugap(lambda_c, F_dw, u_i, g_i, J_da_i, Sigma_hat);

        % --- EKF predict (tex S5(b)). Row 4's increment READS NO STATE
        %     beyond the constant (1 + delta a): that is the whole point of
        %     the coordinate, and it is why the quadrature is exact.
        x_pred = [x_curr(2); ...
                  x_curr(3); ...
                  lambda_c * x_curr(3); ...
                  x_curr(4) + g_i * Sigma_pred; ...
                  x_curr(5); ...
                  x_curr(6); ...
                  x_curr(7)];
        if ma2_aug
            % The -alpha*(m1+m2) share of eps_w is KNOWN state, so it leaves
            % the noise and enters the prediction (row 3); row 4 already has
            % it through Sigma_pred. Rows 8/9 shift the memory chain.
            x_pred(3) = x_pred(3) - alpha_ma2 * m_sum;
            x_pred = [x_pred; 0; x_curr(8)];

            F_aug = zeros(n_state);
            F_aug(1:7, 1:7) = F_e;
            F_aug(3, 8) = -alpha_ma2;
            F_aug(3, 9) = -alpha_ma2;
            F_aug(4, 8) = g_i * alpha_ma2;
            F_aug(4, 9) = g_i * alpha_ma2;
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
        innov_y1_v(ax) = innov1;
        x_upd  = x_pred + K1 * innov1;
        ImKH1  = I7 - K1 * H1;
        P_upd  = ImKH1 * P_pred * ImKH1' + K1 * R1_i * K1';   % Joseph form
        P_upd  = 0.5 * (P_upd + P_upd');
        K_dx_y1_v(ax) = K1(3);
        K_a_y1_v(ax)  = K1(4);
        P41_v(ax)     = P_pred(4, 1);
        dws_y1_v(ax)  = K1(7) * innov1;

        % (b) y2 = gain readout (whitened increment by default).
        %     The delay back-off is AFFINE in u (tex S7):
        %         u[k-d] = u[k] - (1+da)*Grad + (1+da)*(dw_3 - dw_1)
        %     with the second channel DECLARED as a truncation (ref item 2);
        %     R2's d*v^-4*Q_uu term exists precisely because H omits it.
        u_upd = x_upd(4);
        if u_upd < u_min; u_upd = u_min; n_clamp_u(ax) = n_clamp_u(ax) + 1; end
        da_upd = min(max(x_upd(5), da_clamp(1)), da_clamp(2));
        v_hat  = u_upd - (1 + da_upd) * Grad_wbar_d;
        if v_hat < u_min; v_hat = u_min; n_clamp_v(ax) = n_clamp_v(ax) + 1; end
        v_hat_v(ax) = v_hat;
        if ~gate_off(ax) && ~y2_off
            echo_fac = 1;
            if y2_echo_corr
                S_i = (S_echo_T * a_bar_i + S_echo_n * xi_bar(ax)) ...
                      / (a_bar_i + xi_bar(ax));
                echo_fac = 1 - S_i;
            end
            % E[y2] = a_hat[k] + (1-S)*(a_true[k-d] - a_hat[k]) : the reading
            % follows the APPLIED gain and carries only (1-S) of the deviation.
            % Hence the prediction is the convex combination below and the
            % Jacobian is (1-S) times the back-off Jacobian. In a_bar
            % coordinates this is identically the sibling's
            % a_hat - (1-S)*(back-off).
            H2 = H2_scale * echo_fac * [0, 0, 0, ...
                             1 / v_hat^2, ...
                             -Grad_wbar_d * J_da_i / v_hat^2, ...
                             0, 0, zeros(1, n_state - 7)];
            a_hat_now = 1 - 1 / u_upd;
            a_hat_bko = 1 - 1 / v_hat;
            y2_pred = H2_scale * ((1 - echo_fac) * a_hat_now + echo_fac * a_hat_bko);
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

        % --- Validity clamps (constraint (ii); any binding is reported) ---
        if x_upd(4) < u_min
            x_upd(4) = u_min;
            n_clamp_u(ax) = n_clamp_u(ax) + 1;
        end
        if ~lm(1)
            x_upd(5) = min(max(x_upd(5), da_clamp(1)), da_clamp(2));
        end
        if lambda_f < 1
            P_upd = P_upd / lambda_f;            % Menq (4.15)
            P_upd = 0.5 * (P_upd + P_upd');
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
    u_post     = x_e_per_axis(4, :).';
    u_post_use = max(u_post, u_min);
    a_bar_post = 1 - 1 ./ u_post_use;
    a_hat_phys = a_bar_post * a_disp;                  % [um/pN]
    b_post  = x_e_per_axis(5, :).';
    p_post  = x_e_per_axis(6, :).';
    ws_post = x_e_per_axis(7, :).';
    h_bar_now = local_h_bar_out(enable_wall, h_bar);
    ekf_out = [a_hat_phys(1); a_hat_phys(3); a_hat_phys(2); h_bar_now];

    if nargout >= 3
        P_uu_out = zeros(3, 1); P_dx_v = zeros(3, 1); P_dx1_v = zeros(3, 1);
        P_b_v = zeros(3, 1); P_p_v  = zeros(3, 1); P_ws_v  = zeros(3, 1);
        for ax = 1:3
            P_uu_out(ax) = P_per_axis{ax}(4, 4);
            P_dx_v(ax)   = P_per_axis{ax}(3, 3);
            P_dx1_v(ax)  = P_per_axis{ax}(1, 1);
            P_b_v(ax)    = P_per_axis{ax}(5, 5);
            P_p_v(ax)    = P_per_axis{ax}(6, 6);
            P_ws_v(ax)   = P_per_axis{ax}(7, 7);
        end
        % P in a_bar units: (d a_bar/du)^2 * P_uu = u^-4 * P_uu.
        P_a_v = P_uu_out ./ u_post_use.^4;
        diag = empty_diag_ugap();
        diag.sigma2_dxr_hat = sigma2_dwr_hat_new;
        diag.a_xm           = a_bar_wm * a_disp;      % [um/pN]
        diag.a_hm_nd        = a_bar_wm * a_o;         % legacy 1/pN convention
        diag.a_wm_bar       = a_bar_wm;
        diag.y2             = y2;                     % normalized gain units
        diag.delta_x_m      = delta_w_m * R_radius;   % [um]
        diag.innovation_y2  = innov_y2_v;
        diag.innovation_y1  = innov_y1_v;
        diag.K_kf_a_y2      = K_a_y2_v;
        diag.K_kf_dx_y1     = K_dx_y1_v;
        diag.K_kf_a_y1      = K_a_y1_v;
        diag.P41            = P41_v;
        diag.dws_y1         = dws_y1_v;
        diag.dws_y2         = dws_y2_v;
        diag.P_a            = P_a_v * a_disp^2;       % (um/pN)^2
        diag.P_a_nd         = P_a_v;                  % [-] (a_bar^2 units)
        diag.P_uu           = P_uu_out;               % [-] the NATIVE variance
        diag.P_dx           = P_dx_v * R_radius^2;    % [um^2]
        diag.x_D_hat        = zeros(3, 1);
        diag.b_hat          = b_post;                 % slot 5 = delta a
        diag.p_hat          = p_post;
        diag.ws_hat         = ws_post;
        diag.delta_a_hat    = b_post;                 % driver-log alias
        diag.u_hat          = u_post;
        diag.v_hat          = v_hat_v;
        diag.Sigma_hat      = Sigma_v;
        diag.a_prime_hat    = a_prime_v * a_o;        % legacy d a_nd/d h_bar [1/pN]
        diag.a_prime_bar    = a_prime_v;              % d a_bar / d w_bar [-]
        diag.gate_active_per_axis = gate_off;
        diag.guards_individual    = G_flags;
        diag.h_bar          = h_bar;
        diag.h_bar_d        = w_bar_d;
        diag.f_d            = f_d;                    % [pN]
        diag.f_det          = fbar_det / a_o;         % [pN]
        diag.F_dh           = F_dw_vec / a_o;         % [pN]
        diag.dx_r           = dw_r * R_radius;        % [um]
        diag.a_hat          = a_hat_phys;
        diag.a_hat_nd       = a_bar_post * a_o;       % legacy 1/pN convention
        diag.a_bar_hat      = a_bar_post;
        diag.a_ctrl_used    = a_ctrl * a_disp;        % [um/pN]
        diag.P_b            = P_b_v;
        diag.P_p            = P_p_v;
        diag.P_ws           = P_ws_v;
        diag.P77            = P_b_v;                  % legacy alias
        diag.Q77            = Quu_v;                  % legacy alias (row-4 Q)
        diag.var_da_ram     = Quu_v;
        diag.R2             = R2_v;
        diag.Delta_h_d      = Delta_wbar_d * R_radius;   % [um]
        diag.Delta_H_d      = Grad_wbar_d * R_radius;    % [um]
        diag.delta_x_hat_1  = x_e_per_axis(1, :).' * R_radius;
        diag.delta_x_hat_3  = x_e_per_axis(3, :).' * R_radius;
        diag.P_dx1          = P_dx1_v * R_radius^2;
        diag.Q33     = Q33_v;
        diag.a_bar_Q = a_barQ_v;
        diag.f_bar   = fbar_d;                        % normalized force [-]
        diag.P_full  = cat(3, P_per_axis{1}, P_per_axis{2}, P_per_axis{3});
        diag.clamp_u_count = n_clamp_u;               % constraint (ii) audit
        diag.clamp_v_count = n_clamp_v;
        diag.clamp_steps   = n_update_steps;
    end
end


%% =================== Local Helpers ===================

function u_seed = local_seed_gap_ugap(w_bar, w0, enable, gap_floor)
%LOCAL_SEED_GAP_UGAP  Seed the GAP state directly (tex, seed section).
%       u_hat[0] = w_bar[0] - w_bar_0 = 1/(1 - a_bar_seed)
%   w0 is the integration constant, so setting u_hat[0] IS setting the wall
%   position: this is the ONLY place a nominal wall enters the controller.
%   delta a does not appear -- at k = 0 nothing has been integrated yet, which
%   is why the delta a prior reaches u through the SEED JACOBIAN
%   du/d(delta a) = u/(1+delta a) and not through the seed VALUE.
    if ~enable || ~isfinite(w_bar)
        u_seed = 1 / gap_floor;      % far field: a_bar -> 1
        return;
    end
    u_seed = max(w_bar - w0, gap_floor);
end


function F_e = local_build_F_e_ugap(lambda_c, F_dw, u_hat, g, J_da, Sigma_hat)
%LOCAL_BUILD_F_E_UGAP  7x7 error-dynamics Jacobian (tex S6(b)).
%   cols: 1=dw1 2=dw2 3=dw3 4=u 5=delta a 6,7=inert (locked, zero P0/J/K)
%   Row 3 = [0 0 lc  -F_dw*(1-a_bar)^2  0 0 0]
%   Row 4 = [0 0 g*(1-lc)  1 + g*F_dw*(1-a_bar)^2  J_da*Sigma_hat  0 0]
%   Row 5 = identity (delta a[k+1] = delta a[k])
%   with (1 - a_bar)^2 = u^-2 (the output-map Jacobian d a_bar/du) and
%   g = 1 + delta a.
%   The a_bar writing's self-amplification term A_a*M is ABSENT from F_e(4,4)
%   here: it has moved to the output map u^-2, where it does not compound.
%   F_e(4,5) = Sigma_hat is the accumulated displacement, so a delta a error
%   drives e_u as a RAMP while a gap error is a persistent OFFSET -- the
%   standard position/rate-bias separation. It vanishes in a hold, exactly as
%   the ref declares (delta a is then unobservable through the state path).
    one_minus_lc = 1 - lambda_c;
    j_out = 1 / u_hat^2;                 % (1 - a_bar)^2
    F_e = zeros(7);
    F_e(1, 2) = 1;
    F_e(2, 3) = 1;
    F_e(3, 3) = lambda_c;   F_e(3, 4) = -F_dw * j_out;
    F_e(4, 3) = g * one_minus_lc;
    F_e(4, 4) = 1 + g * F_dw * j_out;
    F_e(4, 5) = J_da * Sigma_hat;
    F_e(5, 5) = 1;
    F_e(6, 6) = 1;
    F_e(7, 7) = 1;
end


function R2 = compute_R2_formB(a_bar, sigma2_n, IF_abc, C_dpmr, C_n, ...
                               K_var, amlpf_var_factor, xi_bar, kappa_T, Q_aa_eff, ...
                               delay_steps, a_cov, whitened)
%COMPUTE_R2_FORMB  R(2,2) for the gain readout channel, fully normalized.
%   Inherited verbatim from the a_bar-coordinate sibling except that the delay
%   term's Q_aa_eff is now the u-space process noise mapped into a_bar units,
%   Q_aa_eff = v_hat^-4 * Q_uu (tex S8: "d * v_hat^-4 * Q_uu").
%       R2_int = amlpf * K_var * IF_eff * (a_bar + xi_bar)^2
%   whitened = true: y2 = a_cov*u_ar1[k]; per-sample variance
%   a_cov*(2-a_cov)*Var(a_bar_wm), and the d-step accumulation picks up a_cov^2.
    sxT = kappa_T * a_bar;
    num = sxT^2 * IF_abc(1) + 2 * sxT * sigma2_n * IF_abc(2) + sigma2_n^2 * IF_abc(3);
    den = (C_dpmr * sxT + C_n * sigma2_n)^2;
    IF  = 1 + 2 * num / den;
    R2_int = amlpf_var_factor * K_var * IF * (a_bar + xi_bar)^2;
    if whitened
        R2 = a_cov * (2 - a_cov) * R2_int + a_cov^2 * delay_steps * Q_aa_eff;
    else
        R2 = R2_int + delay_steps * Q_aa_eff;
    end
end


function P = freeze_locked_P(P, lock_state_idx)
%FREEZE_LOCKED_P  Pin locked-parameter rows/cols of P at 0 (exactly known).
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


function d = empty_diag_ugap()
%EMPTY_DIAG_UGAP  Zeroed diagnostic struct (driver collect_diag compatible;
%   the a_bar-coordinate sibling's set plus the u-coordinate rows u_hat,
%   v_hat, P_uu, Sigma_hat and the clamp counters).
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
    d.K_kf_a_y1         = zeros(3, 1);
    d.P41               = zeros(3, 1);
    d.dws_y1            = zeros(3, 1);
    d.dws_y2            = zeros(3, 1);
    d.P_a               = zeros(3, 1);
    d.P_a_nd            = zeros(3, 1);
    d.P_uu              = zeros(3, 1);
    d.P_dx              = zeros(3, 1);
    d.x_D_hat           = zeros(3, 1);
    d.b_hat             = zeros(3, 1);           % slot 5 = delta a, seed 0
    d.p_hat             = ones(3, 1);
    d.ws_hat            = ones(3, 1);
    d.delta_a_hat       = zeros(3, 1);
    d.u_hat             = ones(3, 1);
    d.v_hat             = ones(3, 1);
    d.Sigma_hat         = zeros(3, 1);
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
    d.clamp_u_count     = zeros(3, 1);
    d.clamp_v_count     = zeros(3, 1);
    d.clamp_steps       = 0;
end
