% STATUS: ACTIVE | SSOT derivation: reference/eq17_analysis/derivation/formC_state_b.tex
% FORK OF model/controller/motion_control_law_formC_dist.m @ 32baf9c | PURPOSE:
%   the STATE writing of the parameter-free gain law with ONE shape state:
%   a_bar' = b (1-a_bar)^2, b a constant per axis, Q55 = 0, seeded at the
%   far-field anchor 8/9 with P55[0] = (b_half)^2 from the planned envelope.
%   SLOT MAP (this file, 2026-08-30 hygiene): 5 = b (state, dimensionless);
%   there is NO additive disturbance da in this writing (has_da_known is
%   hard false; the da_known argument is rejected). Slots 6-7 are
%   PERMANENTLY LOCKED and inert (zero P0, zero Jacobian, zero K) so the
%   9-slot layout and the MA(2) block at [8 9] carry over unchanged.
%   ctrl_const.lock_b = true pins b at its seed (the parameter-free
%   baseline); false estimates it. Passages below that still say "da" are
%   inherited from the dist sibling and describe THAT file, not this one.
%   PRODUCTION RECIPE (default ON since 2026-09-04, derivation
%   reference/eq17_analysis/derivation/0903_aptrue_4state_from_true.tex):
%   law_exact_step (exact law step) + pred_mean2 (second-order mean of the
%   predict) + nw_mcorr (correlated process/measurement noise predict) +
%   pred_mean2_e4 (gain-reading start-point term, e4 and e_b lines).
%   fe44_Aa_scale (kappa, default 1) weights the law's self-sensitivity in
%   the P propagation; 0.5 helps the b_true arm but not the constant-b arms
%   (see its init block), so production stays at 1.
%   law_exact_step = false alone restores the pre-09-04 Euler recipe.
function [f_d, ekf_out, diag] = motion_control_law_formC_b(del_pd, pd, p_m, params, ctrl_const, a_ctrl_override, varargin_da_known_guard, ap_known, b_true, app_known)
%MOTION_CONTROL_LAW_FORMC_DIST  Per-axis EKF eq17 controller whose gain slope
%   is a parameter-free function of the gain state, with an ADDITIVE constant
%   disturbance in the gain state equation (formC_state_dist.tex)
%
%       a_bar'(a_bar) = (1 - a_bar)^2                      (S1; NO parameter)
%       a_w = a_o * a_bar ,   a_o = Ts/(gamma_N*R)  [1/pN]  (fixed, not a state)
%
%   Authoritative spec: reference/eq17_analysis/derivation/formC_state_dist.tex
%   (S1-S8 + seed section), which carries BOTH derivations side by side:
%       (a) BASELINE     4 states [dw1 dw2 dw3 a_bar], no free parameter at all
%       (b) WITH da      5 states [dw1 dw2 dw3 a_bar da], da[k+1] = da[k]
%   This file is both: ctrl_const.lock_da selects (a) (slot 5 pinned at its
%   seed, zero P0, zero Jacobian, zero K -> provably inert) or (b).
%
%   HOW THIS DIFFERS FROM THE MULTIPLICATIVE SIBLING (formC_state_da.tex /
%   motion_control_law_formC_state.m), which is the only fork relationship
%   that matters here:
%       slope        (1-a_bar)^2                 vs  (1-a_bar)^2 * (1+da)
%       da enters    the state equation, additively, ONCE per step
%                                                vs  the slope, multiplicatively
%       F_e(4,5)     1 (constant: never vanishes at a turning point, in a hold,
%                    or in the far field)        vs  A_delta*M (dies with motion)
%       H(2,5)       -d_delay (small but NOT zero; see the S7 note below)
%                                                vs  -Grad*A_delta
%       P(4,5)[0]    0 (independent by construction; the seed level does not
%                    depend on da at all)        vs  dA/dda * P_da
%       Q44          a_bar'^2*Q33 = (1-a_bar)^4*Q33 (FOURTH power, since the
%                    slope no longer carries (1+da))
%   So da is identified mainly through the cross-covariance P(4,5) that F_e
%   builds, i.e. after it has accumulated into a_bar. That is the declared
%   price of the additive placement (tex S7).
%
%   S7 CORRECTION (2026-08-12; this file leads the tex). formC_state_dist.tex
%   S7(b) as first written asserts dy2/d(da) = 0, "da has NO direct
%   measurement column". That contradicts its own S5(b), which applies da once
%   per step. The delay back-off walks the gain back d steps, so it must walk
%   back d applications of the disturbance as well:
%
%       a_bar[k-d] = a_bar[k] - (1-a_bar[k])^2 * Grad_d_wbar_d[k] - d*da[k]
%
%   which is EXACT under da[k+1] = da[k]. Hence dy2/d(da) = -d_delay and the
%   y2 prediction carries a -d_delay*da_hat term. Implemented here; the
%   omitted column was worth ~1% of the retained P(5,4)*H(2,4) contribution,
%   so the qualitative story above is unchanged but the Jacobian value was
%   wrong. The .tex needs the same edit.
%
%   Everything below this line is inherited verbatim from the multiplicative
%   sibling and, through it, from motion_control_law_formB_ws.
%
%   FULLY NORMALIZED INTERNALS (formB_ws_ref Conventions): every internal
%   quantity is dimensionless -- lengths divided by R, gain by a_o, forces
%   multiplied by a_o (f_bar = a_o*f, so a_bar*f_bar = a*f identically).
%   The only dimensionless thermal constant inside the loop is
%
%       kappa_T = 4*kB*T*a_o/R                                   [-]
%
%   a_o = Ts/(gamma_N*R) is the R-NORMALIZED far-field gain [1/pN]; the physical
%   one is a_nom = Ts/gamma_N = a_o*R [um/pN] (driver name). The force scale is
%   f_R = 1/a_o = gamma_N*R/Ts (~153 pN): f_bar = f/f_R. Because a_o already
%   holds one 1/R, kappa_T carries /R and not /R^2. Changing the definition of
%   a_o means changing the power of R in kappa_T AND the division at [U3] in
%   the same edit; changing the a_o line alone runs to completion with every
%   number wrong by R (verified V1-V4, 2026-08-30, ledger L42-L45; rule
%   .claude/rules/normalization-boundary.md).
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
%       x = [dw_1; dw_2; dw_3; a_bar_w; da; (inert); (inert)]
%
%   dw_1 = delta_w[k-2] (measured) ... dw_3 = delta_w[k] (current), in units
%   of R; a_bar_w = normalized gain in (0,1); da = additive disturbance in
%   units of a_bar per step (Q55 = 0). Slots 6-7 do not exist in this writing.
%
%   Gain rate and Jacobians (tex S2, evaluated at the estimate):
%       a_bar'  = (1 - a_bar)^2
%       A_a     = d a_bar'/d a_bar = -2 (1 - a_bar)              (< 0 always)
%       d/d(da) [ a_bar + a_bar'*(...) + da ] = 1                (constant)
%
%   Estimator PREDICT (tex S5(b)): dw_hat contracts with lambda_c alone;
%       a_bar[k+1] = a_bar + (1-a_bar)^2 * ( Delta_wbar_d + (1-lc)*dw_3_hat )
%                    + da
%       da[k+1]    = da
%   a_bar is NEVER re-anchored from the integrated law after init.
%
%   F_e (tex S6(b)); M = the FULL row-4 increment (command step +
%   (1-lc)*dw_3_hat + MA(2) memory feedthrough) -- exactly what a_bar'
%   multiplies in predict:
%       row 3 = [0 0 lc -F_dw 0 0 0]                      (F_e(3,5) = 0: the
%               control law is untouched by da, so the loop-coupling column
%               is identical to the baseline)
%       row 4 = [0 0 (1-lc)*a_bar'  1 + a_bar'*F_dw + A_a*M   1   0 0]
%       row 5 = identity
%   The A_a*M term has no counterpart in the height writing, where a_bar' did
%   not depend on a_bar. The constant 1 in column 5 is what makes a constant
%   da drive e_a as a RAMP while a gain-state error is a persistent OFFSET --
%   the standard position-error / rate-bias separation (tex S6(b)).
%
%   Measurements (tex S7). The gain readout is computed in ONE step with
%   kappa_T (never physical-then-divide):
%       a_bar_wm = (sigma2_dwr_hat - C_n*sigma2_nw) / (C_dpmr*kappa_T)
%   a_bar_wm is an exact AR(1) with pole 1-a_cov; the KF is fed the WHITENED
%   increment by default (production behaviour, ctrl_const.y2_whiten = true):
%       y1 = dw_m = dw_1 + n_w
%       y2 = a_bar_wm[k] - (1-a_cov)*a_bar_wm[k-1] = a_cov*u[k]
%       H  = H2_scale * [0 0 0  1 + 2(1-a_bar)*Grad  -d_delay  0 0]
%   The delay back-off carries A_a through (dy2/da_bar = 1 - A_a*Grad =
%   1 + 2(1-a_bar)*Grad) and, per the S7 correction above, d applications of
%   the disturbance (dy2/d(da) = -d_delay). The y2 innovation uses the
%   NONLINEAR prediction
%       y2_pred = H2_scale * ( a_bar_hat - a_bar'*Grad_wbar_d - d*da_hat )
%   with both back-off terms scaled by the echo factor (1-S).
%
%   Q (tex S8; run time at the current estimate):
%       Q33 = Var(eps_w) in FULL (revision 2026-08-01, D3 precedent of
%             kf_canonical_spec.md S5):
%               kappa_T*( a_bar_hat[k] + (1-lc)^2*sum_{i=1..d} a_bar_hat[k-i] )
%               + (1-lc)^2*sigma2_nw
%       Q34 = Q43 = -a_bar'*Q33 ,  Q44 = a_bar'^2*Q33 = (1-a_bar)^4*Q33 ,
%       Q55 = 0  (FIRST ARM of the tex S8 disturbance-container section: the
%             claim is "no stochastic driving of da", NOT that da is constant,
%             which S3(b) already refutes in closed form. Its pre-registered
%             consequence -- P55 decreases monotonically and da_hat freezes
%             late in the run -- is to be MEASURED.)
%   R = diag(R1, R2), R2 at the ESTIMATE a_bar_hat (never the raw readout):
%       R1 = sigma2_nw  (units of R^2)
%       R2 = K_var*IF_eff*(a_bar + xi_bar)^2 + d*Q44
%   with xi_bar = (C_n/C_dpmr)*sigma2_nw/kappa_T. Under y2_whiten (default)
%   IF_eff is built from IF_abc_white (s = 1 sums, 2026-08-27): the whitened
%   increment is a single sample, so the colour penalty is the long-run
%   factor, and R2 = 2*a_cov^2*IF_white*(a_bar + xi_bar)^2 + a_cov^2*d*Q44.
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
%   CURRENT step alone) and n_w[k-d]. Then
%       F_e(3,8) = F_e(3,9) = -alpha ,  F_e(4,8) = F_e(4,9) = +a_bar'*alpha
%       row 8 = 0 ,  F_e(9,8) = 1
%       Q = s2T*(g_T g_T') + s2n*(g_n g_n') ,
%           g_T = [0 0 -1 a_bar' 0 0 0 1 0]' , g_n = alpha*[0 0 -1 a_bar' 0 0 0 0 0]'
%   Q is RANK 2 and its off-diagonals are load-bearing. P[0] correspondingly
%   comes from a 5x5 DARE over [dw_1 dw_2 dw_3 m_1 m_2] mapped into slots
%   [1 2 3 8 9]; the gain/da block is untouched and starts uncorrelated with
%   the memory states.
%
%   INIT (tex seed section; every number derived or declared, none tuned):
%       seed   a_bar_hat[0] = 1 - 1/(w_bar[0] - w0_hat)   (integrated law;
%              setting a_bar_hat[0] IS setting the integration constant, so
%              the nominal wall w0_hat enters HERE and nowhere else)
%              da_hat[0] = 0  (the far-field mismatch value is deliberately
%              NOT fed in, so a non-zero da_hat[inf] is an ACCEPTANCE)
%       P0     P44[0] = (a_bar'[0])^2 * Pf_w0_std^2 + Pf_a_floor^2
%              P45[0] = 0   (at k = 0 nothing has accumulated: the seed error
%                            and the disturbance are independent BY
%                            CONSTRUCTION -- unlike the multiplicative
%                            sibling, the seed LEVEL does not depend on da
%                            at all, so there is no level Jacobian to carry)
%              P55[0] = Pf_da_std^2
%              Position 3-block (and the MA(2) memory block) from the DARE
%              steady state. P0 is chol-checked positive definite on the
%              FREE-state submatrix and the controller errors out if it fails.
%
%   LOCK FLAGS. ctrl_const.lock_da pins the disturbance at its seed
%   (true = derivation (a), the 4-state BASELINE). Slots 6-7 do not exist in
%   this writing and are held locked at all times. Lock semantics are the
%   ancestor's: the Jacobian is zeroed at the source (which zeroes the F_e
%   row-4 column; there is no H column to zero), the Kalman-gain entries are
%   zeroed in both updates, and the P row/col is pinned at 0.
%
%   PARALLEL-AXIS LAW (ctrl_const.par_law). The wall-parallel truth is not a
%   member of the one-curve family either, and in THIS writing there is no
%   constant that can reshape the curve (da is a per-step increment, not a
%   slope multiplier), so the parallel package collapses to the integration
%   constant alone: x/y seed at w0_par and lock slot 5 at 0. x/y accuracy is
%   DECLARED out of scope, not asserted; the z axis is the arm under test and
%   the axes are dynamically decoupled (w_hat = z => Gamma_inv diagonal).
%
%   ctrl_const fields specific to this fork (everything else is the ancestor's)
%       .lock_da       pin da at its seed = derivation (a)   (default false)
%       .da_init       da seed                               (default 0)
%       .Pf_da_std     sqrt P0 on da. THE ONLY UNDETERMINED NUMBER (tex seed
%                      section). Its natural scale is the level offset the
%                      disturbance must remove divided by the number of steps
%                      over which it must remove it, so the fallback here is
%                      Pf_a_floor * Ts / T_REMOVAL_S with T_REMOVAL_S = 1 s
%                      (order of a manoeuvre; the driver, which knows the
%                      planned trajectory, overrides it with the closed-form
%                      S3(b) sup). Its contract is INVARIANCE: sweep it 10x
%                      either way and the converged behaviour must not move.
%       .Pf_w0_std     sqrt P0 on the wall position (default 0.111, carried
%                      over from the height writing). There is no w_s state:
%                      it enters P(4,4) once via d a_bar/d w0 = -a_bar'.
%       .Pf_a_floor    shape floor on sqrt P_aa              (default 0.0066)
%       .ws0_perp      nominal wall; w0 = ws0_perp - 1       (default 1)
%       .par_law       x/y run the fitted parallel origin    (default false;
%                      REQUIRES .w0_par, .Pf_a_floor_par)
%       .a_bar_floor / .a_bar_ceil   a_bar clamps    (default 0.05, 1 - 1e-4)
%       .b_floor / .b_ceil   numerical guard on slot 5 = b (default 0.5, 2.0).
%                      Guard only: b_true on the envelope lives in
%                      [1.1258, 1.1534], so the guard is ~20x wider than the
%                      physics and must never bind. How far b MAY travel is
%                      P55's job, not the guard's -- P55 records the travel.
%
%   da_known (3x1 or scalar, optional): KNOWN-DISTURBANCE ARM. When supplied,
%   the row-4 disturbance is taken from this exogenous input instead of from
%   slot 5, and slot 5 stops being a state (its Jacobian column is zeroed, so
%   the filter neither estimates it nor carries covariance for it). Use it with
%   lock_da = true, i.e. on top of the 4-state baseline: the arm is then
%   "baseline + the model error removed", which measures the CEILING that any
%   disturbance-estimation scheme could reach. The caller computes it from the
%   plant truth as
%       da[k] = [ a'_true(w_bar[k-1]) - (1 - a_true[k-1])^2 ] * ( w_bar[k] - w_bar[k-1] )
%   which is the closed form of formC_state_dist.tex S3(b) with the increment
%   written as the actual step in w_bar (S4 first line). The previous step is
%   used because predict bridges k-1 -> k.
%
%   a_ctrl_override (3x1, um/pN, optional): feed a chosen gain to the CONTROL
%   LAW only; the EKF still estimates a_bar_w / da.
%
%   See also: motion_control_law_formC_state, motion_control_law_formB_ws,
%             run_formC_dist

    % ap_known (3x1, optional): TRUE-SLOPE ARM. Replaces the law's a_bar'
    % outright, so the integrand carries no state error. This isolates the ONE
    % remaining structural difference from the height writing, whose a_bar' is
    % a deterministic function of the COMMANDED height and therefore identical
    % across seeds, while ours reads the seed-dependent gain state. With it
    % supplied, d a_bar'/d a_bar is 0 by construction, so the A_a*M term leaves
    % F_e(4,4). Diagnostic only.
    if nargin < 8 || isempty(ap_known)
        ap_known = [];
    else
        ap_known = ap_known(:);
        assert(numel(ap_known) == 3 && all(isfinite(ap_known)), ...
               'motion_control_law_formC_b:apKnown', 'ap_known must be a finite 3x1.');
    end
    has_ap_known = ~isempty(ap_known);
    % b_true (3x1, optional): TRUE-b ARM (2026-08-27). Per axis, a positive
    % entry replaces the b the LAW and its Jacobians read (b_hat_i) with the
    % true b(w_bar) of the plant at the particle; zero entries leave that axis
    % alone. Slot 5 is expected to be locked by the driver (ctrl_const.lock_b)
    % so the inert state never drifts. Unlike ap_known this swaps b ONLY --
    % a_bar stays the estimate, so the arm isolates "b known" from "slope
    % known". diag.b_hat reports the b the law used. Diagnostic only; absent
    % or empty = production, bit-identical.
    if nargin < 9 || isempty(b_true)
        b_true = [];
    else
        b_true = b_true(:);
        assert(numel(b_true) == 3 && all(isfinite(b_true)) && all(b_true >= 0), ...
               'motion_control_law_formC_b:bTrue', 'b_true must be a finite non-negative 3x1.');
    end
    % app_known (3x1, optional, 2026-09-02): exogenous law CURVATURE d^2 a_bar/d w_bar^2
    % for the pred_mean2 second-order mean term (0902_formC_aptrue_4state.tex S5,
    % second and third lines). Read at the same height as ap_known. Absent =>
    % pred_mean2 falls back to the constant-b law curvature -2 a'^2/(1-a_bar_hat),
    % which is 20% smaller in magnitude at w_bar = 1.10 (c-free price). Only
    % consulted when ctrl_const.pred_mean2 is true; otherwise ignored.
    if nargin < 10 || isempty(app_known)
        app_known = [];
    else
        app_known = app_known(:);
        assert(numel(app_known) == 3 && all(isfinite(app_known)), ...
               'motion_control_law_formC_b:appKnown', 'app_known must be a finite 3x1.');
    end
    has_app_known = ~isempty(app_known);
    has_b_true = ~isempty(b_true);
    has_da_known = false;   % no additive disturbance in this writing
    % (7th argument) da_known: KNOWN-DISTURBANCE ARM of the dist sibling. This
    % writing has no da state; a caller supplying one would silently get
    % nothing (08-25 audit dead knob), so refuse it.
    if nargin >= 7 && ~isempty(varargin_da_known_guard)
        error('motion_control_law_formC_b:deadKnob', ...
              'da_known is not an arm of this writing (slot 5 = b); pass [].');
    end
    if nargin < 6
        a_ctrl_override = [];
    end
    has_override = ~isempty(a_ctrl_override);
    if has_override
        a_ctrl_override = a_ctrl_override(:);
        assert(numel(a_ctrl_override) == 3 && all(isfinite(a_ctrl_override)) && all(a_ctrl_override > 0), ...
               'motion_control_law_formC_b:badOverride', ...
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
    persistent lambda_c d_delay Ts kappa_T R_radius a_o a_disp r22_delay_scale
    persistent a_inject_step a_inject_frac a_inject_axis   % diagnostic hook, default off
    persistent fe43_off q34_off q44_scale fe44_Aa_off fe44_Aa_scale   % diagnostic flags, default off / scale 1
    persistent jac_exact_step                               % row-4 Jacobian / Q / g_n of the EXACT law step (2026-09-06), default off
    persistent q55_path q55_per_R                           % Q55 = (Delta_b^2/W)|dw_hat| container of the b'_true dw line (2026-09-07), default off
    persistent law_err_q44 law_err_db                       % Q44 law-error term: the filter admits its own law is wrong (2026-09-15), default off
    persistent l51_off                                      % DIAGNOSTIC (2026-09-07): zero the y1 gain on slot 5 (b_hat updated by y2 only), Joseph P update stays consistent
    persistent da_slot Pf_da_std                            % E2 (2026-09-07): slot 6 = additive gain disturbance da (formC_dist writing), default off
    persistent b1_state Pf_b1_std b1_seed_couple            % 2026-09-14: slot 6 = b1 = db/dw (0914_estb_bprime_core.tex), default off
    persistent law_exact_step                               % exact (quadrature-free) law step, default off (port of 2a5dc29)
    persistent pred_mean2                                   % second-order mean term in predict (0902 tex S5), default off
    persistent pred_force_step                              % row-4 known increment = a_hat * fbar_d[k-1] (commanded displacement), default off
    persistent pred_mean2_kr1 K31_km1                       % A2 closed-form term a'' (1-lc)^2 K31 R1 (measurement-noise feedthrough), default off
    persistent pred_mean2_kr1_full                          % complete feedthrough term a'' (1-lc) K31 R1 (all three shares), default off
    persistent a_pd a_cov C_dpmr C_n K_var IF_abc xi_bar amlpf_var_factor
    persistent t_warmup_kf h_bar_safe sigma2_n_nd
    persistent enable_wall w_hat_n pz_wall
    persistent Q_theta_floor a_bar_floor a_bar_ceil ws_margin gap_floor
    persistent b_floor b_ceil
    persistent y2_whiten fe_row4_full use_fdet y2_off y1_gain_off y1_gain_off_from t2_pure_prop lambda_f obs_dump_on
    persistent r2_scale                                       % diagnostic: multiplies the WHOLE R2 (colour chain + delay term) after compute_R2_formB (2026-09-23), default 1 = bit-identical
    persistent mean_knob_from pred_slope_scale y1_gain_leg_scale   % mean-only diagnostic knobs (2026-09-03), default off
    persistent nw_mcorr res1_km1                                  % correlated process/measurement noise correction (2026-09-03), default off
    persistent pred_mean2_e4                                      % gain-reading start-point term of the second-order mean (2026-09-04), default off
    persistent amodel_fe                                          % green variant: e_a -> psi in the covariance model (2026-09-11), default off
    persistent amodel_h                                           % orange variant: the same substitution in the y2 row (2026-09-12), default off
    persistent gain_algebraic                                     % 4-state: the gain is the law value, not a state (2026-09-14), default off
    persistent c0_state c0_seed d1_seed ws_11 Pf_c0_std Pf_d1_std  % [1/1] Pade writing, version A: slots 5 = d1, 7 = c0 (2026-09-17), default off
    persistent pade22 c1_seed d2_seed Pf_c1_std Pf_d2_std pade22_gmin   % [2/2] Pade writing, version A: slots 8 = c1, 9 = d2 (2026-09-18), default off, needs c0_state
    persistent delta_state delta_seed Pf_delta_std                    % offset law, version A: slot 7 = delta (effective-wall depth below w_s), slot 5 locked (2026-09-20), default off
    persistent delta_iekf                                             % offset law: iterated (Gauss-Newton) y2 update, re-linearised at the posterior (2026-09-20, O38), default off
    persistent plaw_c_state plaw_p c_seed Pf_c_std                    % advisor's power law a' = (1-c)/h^p, version A: slot 7 = c, p fixed (plane fit), slot 5 locked (2026-09-21), default off
    persistent plaw_c_slope                                           % version 2 (0921_estb_v2_slope_c.tex): a slot KEPT, slope (1-c)/h^p read at the estimator height, slot 5 = c (2026-09-21), default off
    persistent plaw_dc_slope plaw_g_min plaw_delta_lim                % two-constant slope law (1-c)/(h+delta)^2 (0922_estb_f2_slope_dc.tex): plaw_c_slope + slot 7 = delta (2026-09-22), default off
    persistent plaw_dp_slope plaw_p_lim                               % same family in the other coordinate, 1/(h+delta)^p (0923_estb_f4_slope_dp.tex): slot 5 = p, contact slope pinned at 1 (2026-09-23), default off
    persistent plaw_cp_slope pcp_seed Pf_pcp_std                      % the advisor's slope law (1-c)/h^p with BOTH constants as states, no shift: slot 5 = c, slot 7 = p (2026-09-23 evening), default off
    persistent par_wall_known                                          % 10-07 (Layer-1): wall-parallel axes follow the plane's parallel law driven by the wall-normal step only, default off
    persistent plaw_seed_cross                                        % O40 discriminator (2026-09-23): keep (true, default) or zero the seed cross terms P45[0], P47[0] of the dc/dp laws; P44[0] unchanged
    persistent delta_slin                                             % offset law: Jacobians and predicted reading as Gaussian expectations over the delta uncertainty (2026-09-20), default off
    persistent lambda_f_b lfb_alpha lfb_floor lam_b_spend
    persistent lfb_chan lfb_info lam_b_v                   % 2026-09-24: residual channel for the adaptive slot-5 forgetting ('y2' | 'y1'), information-driven variant (beta), per-axis lambda of this step
    persistent q55a_mode q55a_N q55a_thr q55a_gain q55a_kcu q55a_buf q55a_ptr q55a_g q55a_stat q55a_q q55a_P0 q55a_P0s q55a_rho q55a_Imin q55a_hgate q55a_I q55a_bufI q55a_wI q55a_gfun q55a_dc q55a_cmf q55a_cmD q55a_cmR q55a_Prev q55a_hmin   % 2026-09-29 exploratory: innovation-adaptive Q55 on slot 5 from the y1 residual (default off); 'ratio' (09-30) = Q/R-ratio design, q55a_P0 = P55[0]
    persistent p55_floor_on p55_floor_std                     % 2026-09-09: directional forgetting on slot 5 as a P55 floor (default off)
    persistent b_bins_on b_bin_edges bbin_b bbin_P bbin_cur bbin_switch   % 2026-09-09: b per HEIGHT BIN (default off)
    persistent ap_src ap_ewma_a a_bar_slope_v law_b_formC
    persistent q33_dc_match q33_dc_fac
    persistent y2_echo_corr S_echo_T S_echo_n
    persistent h2_walk_cols_off y2_delay_off   % 2026-10-06 (Layer-1 session, user test): zero the walk-back columns H2(3), H2(5); prediction unchanged (default off)
    persistent ma2_aug alpha_ma2 n_state    % MA(2) noise-memory augmentation
    persistent predict_quad                 % Stage B diagnostic: row-4 predict quadrature rule
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
        % kappa_T = 4*kB*T*a_o/R: the ONLY thermal constant inside the loop.
        kappa_T = 4 * (params.ctrl.k_B * params.ctrl.T / R_radius) * a_o;   % [-]
        a_disp  = a_o * R_radius;             % [um/pN] display scale per unit a_bar

        % --- 0B. ctrl_const (offline scalars; shared with the 5/6-state family) ---
        lambda_c        = ctrl_const.lambda_c;
        d_delay         = ctrl_const.d;
        % Scale on the R2 d-step delay term ONLY. Default 1.0 = the form this
        % file has always used (flat d*Q44). The repo carries three forms of the
        % same term: 1.0 here, ctrl_const.r22_delay_sum_factor = 1.105 in
        % eq17_4state (telescoping correction, build_eq17_6state_constants.m),
        % and sum_j (d-j+1)^2 = 2.5*d*Q44 at d=2 in the expgain sibling. This
        % knob exists to run them against each other; it is not tuning.
        if isfield(ctrl_const, 'r22_delay_scale') && ~isempty(ctrl_const.r22_delay_scale)
            r22_delay_scale = ctrl_const.r22_delay_scale;
        else
            r22_delay_scale = 1.0;
        end
        % DIAGNOSTIC HOOK (2026-08-26): multiply the posterior a_bar of ONE axis
        % by (1 + a_inject_frac) at step a_inject_step, once. Measures the
        % closed-loop response of the y1 -> a_bar path to a KNOWN gain error
        % (injection-response test). a_inject_step = 0 (default) never fires,
        % so production is bit-identical. Not a tuning knob.
        a_inject_step = get_field_default(ctrl_const, 'a_inject_step', 0);
        a_inject_frac = get_field_default(ctrl_const, 'a_inject_frac', 0);
        a_inject_axis = get_field_default(ctrl_const, 'a_inject_axis', 3);
        % DIAGNOSTIC FLAG (2026-08-26): zero the law coupling F_e(4,3) = (1-lc)*a_bar'
        % (tracking-error estimate -> gain, through the law) in the covariance
        % propagation only. Discriminates the sign-of-P(4,1) hypothesis from the
        % injection-response test. Default false => bit-identical.
        fe43_off = get_field_default(ctrl_const, 'fe43_off', false);
        % DIAGNOSTIC FLAG (2026-09-04): drop the law's self-sensitivity A_a*M from F_e(4,4) in the COVARIANCE
        % propagation only (the state predict keeps the exact law step read at a_hat; the mean term keeps A_a).
        % Discriminator for the fast-descent spread of the b_true / production arms: with A_a*M in, P44 grows
        % x16 along the descent, P41 becomes -F_dw P44 dominated, l41 flips sign (+0.036 vs the law-consistent
        % -a' l31 = -0.157 on canon 1.0-1.5 s) and the y1 leg stops cancelling the law-injected position noise
        % (cross-seed sd 0.023 vs 0.0025 in the a'_true arm). 2026-08-13 judged "zeroing A_a = over-confident"
        % on the pre-four-block production arm (+20% bias); the premise differs now (honesty 0.66 = P too LARGE).
        % Default false => bit-identical.
        fe44_Aa_off = get_field_default(ctrl_const, 'fe44_Aa_off', false);
        % fe44_Aa_scale (2026-09-04): kappa in F_e(4,4) = 1 + a' F_dw + kappa * A_a * M for the COVARIANCE propagation
        % (state predict unchanged). kappa = 1 is the EKF Jacobian (current), kappa = 0 == fe44_Aa_off. Measured 09-04:
        % kappa = 1 gives sigma_seed/sqrt(P44) 0.66 / 0.81 in the fast descent (P too large, y1 leg flips sign, spread x6-10),
        % kappa = 0 gives 1.24-1.60 (fast) and 1.75-2.07 (hold): P too small. The value is CALIBRATED against the honesty
        % ratio (its own observable, stacked-fix-audit C.8/C.9) until the split free / law-slaved gain error has a closed form.
        % kappa = 0.5 was CALIBRATED on the b_true arm (b known; honesty ratio sigma_seed/sqrt(P44), seeds 1:10,
        % validated 11:20: worst-instant sd 0.0237 -> 0.0026 canon, 0.0272 -> 0.0082 Meng) and was briefly the
        % production default (2026-09-04 evening). REVERTED to 1 the same night: the verify_formC_b_production_defaults
        % arm smoke showed that with a CONSTANT b (locked 8/9 or estimated) the smaller P44 lets the constant-b model
        % error stand -- locked 8/9 on canon collapsed to the a_bar floor in 10/10 seeds (hold -0.0197 +- 0.0007 vs
        % +0.0006 at kappa = 1); the estimated-b arm kept its hold level but its worst-instant mean did not improve
        % (-0.031 vs -0.034). The calibration does not transfer across arms (stacked-fix-audit C.9). Knob kept for the
        % b_true arm and for the closed-form work (0903 tex S12).
        fe44_Aa_scale = get_field_default(ctrl_const, 'fe44_Aa_scale', 1);
        if fe44_Aa_off; fe44_Aa_scale = 0; end
        % jac_exact_step (2026-09-06, 0903 tex S12 closed form): linearise the EXACT law step consistently.
        %   The predict is a_pred = 1 - 1/(1/(1-a) + b M) (law_exact_step), whose Jacobian row 4 is
        %       d a_pred / d x = (1 - a_pred)^2 * d(1/(1-a) + b M)/d x
        %   so EVERY row-4 entry carries the common factor D = (1-a_pred)^2/(1-a)^2 = 1/(1 + b(1-a)M)^2 = a'(a_pred)/a'(a):
        %       F_e(4,4) = D (1 + a' F_dw),  F_e(4,3) = D (1-lc) a',  F_e(4,8:9) = D alpha a',  F_e(4,5) = D J_b,
        %       F_e(4,1) = -D (1-lc) a' (nw_mcorr),  Q(4,:) and Q(:,4) x D (Q44 x D^2), g_n(4) x D.
        %   The production row keeps the EULER Jacobian (1 + a' F_dw + A_a M on the diagonal, left-endpoint a' elsewhere),
        %   which is exact for the Euler predict but NOT for the exact step: per step the difference is A_a M x (row-4 coupling)
        %   = O(1e-2), and it compounds along the descent like the diagonal does (sum A_a M = O(1)). In the (e3, z) basis,
        %   z = e4 + a' e3, the consistent row gives z+ = D z exactly (the law-slaved part is removed by y1 every step),
        %   the Euler row forces z by A_a M a' (lc e3 + (1-lc) e1 - F_dw e4) per step => P44 grows x16 along the descent,
        %   P41 becomes -F_dw P44 dominated and l41 + a' l31 leaves 0 (measured 09-04/09-06). Replaces the calibrated
        %   kappa (fe44_Aa_scale) by a derived structure; when on, fe44_Aa_scale is ignored (asserted = 1).
        %   Inert on the ap_known arm (exogenous slope, A_a = 0 by construction) and when law_exact_step is off.
        %   Default false => bit-identical.
        % q55_path (2026-09-07, 0903 tex S11 'container of the b'_true dw line'): with b a constant state and Q55 = 0, P55 collapses
        % below |e_b| = |b_true(w) - b_hat| along the descent (sqrt P55 -> 0.0065 on the seed-at-truth-P44[0] arm, hold -0.0106) because
        % the exact law factor b_true(w) changes along the path and the constant model has no term for that change. The container is a
        % random walk in the PATH: Q55 = (Delta_b^2 / W) |dw_hat| per step, Delta_b = range of b_true on the envelope, W = envelope width
        % (both from the driver's envelope sweep, passed as q55_per_R = Delta_b^2/W); one traverse accumulates Delta_b^2, a hold adds
        % nothing, and the b_true arm (slot 5 locked) is untouched. No free number. Default false => bit-identical.
        % l51_off (2026-09-07, E1 diagnostic): the estimated-b arm with the seed-at-truth P44[0] pulls b_hat up by +0.057 in the canon
        % descent, 99% through the y1 leg l51*innov1 (probe_estb_l52_split.m); on Meng the pull comes through the y2 state path P54 H24.
        % Zeroing K1(5) lets b_hat take information from y2 only; the Joseph form keeps P consistent for the modified gain. Default false.
        l51_off = logical(get_field_default(ctrl_const, 'l51_off', false));
        q55_path  = logical(get_field_default(ctrl_const, 'q55_path', false));
        q55_per_R = get_field_default(ctrl_const, 'q55_per_R', 0);
        if q55_path
            assert(q55_per_R > 0, 'motion_control_law_formC_b:q55Path', 'ctrl_const.q55_path needs q55_per_R = Delta_b^2/W > 0 from the driver.');
        end
        % law_err_q44 (2026-09-15, default false => bit-identical; reference/eq17_analysis/derivation/0915_estb_law_error_q44.tex):
        % what the filter admits about its OWN LAW being wrong. The gain slot is propagated by the law and its process
        % noise carries only the thermal share, so the filter integrates the curve as if it were exact: measured on the
        % canon descent, the law moves a_hat by -0.869 while the measurements move it by -0.0002. The derivation gives
        % the error of one step as (1-a)^2 * (b_true(w) - b_hat) * dw, i.e. the SAME direction as F_e(4,5) e_b but a
        % second, independent draw, so
        %     dQ44[k] = ( (1-a)^2 * delta_b * M_law[k] )^2 ,   dQ45 = 0
        % with delta_b = sup |b_true - b_hat| over the envelope. Default delta_b = Pf_b_std, the SAME envelope the b
        % prior already uses (sup |b_true(w) - 8/9|), so no new number enters and the term stays c-free. M_law is the
        % step the law actually integrates, so a hold injects nothing. This is NOT q55_path (that one was on slot 5 and
        % let b_hat wander, R57 family) and NOT law_err_q/consider-b (R4, which claimed to fix the valley bias): the
        % claim here is only about the authority balance between the law and y2, and the acceptance is the y2 share.
        % Colour: the error is deterministic along a monotone path, so the white injection understates the accumulated
        % deviation by sqrt(N) (tex S8); it is meant to open P44 so y2 can correct, not to represent the bias itself.
        law_err_q44 = logical(get_field_default(ctrl_const, 'law_err_q44', false));   % delta_b resolved after Pf_b_std below
        jac_exact_step = logical(get_field_default(ctrl_const, 'jac_exact_step', false));
        if jac_exact_step
            assert(fe44_Aa_scale == 1, 'motion_control_law_formC_b:jacExactStep', ...
                   'ctrl_const.jac_exact_step replaces fe44_Aa_scale; leave fe44_Aa_scale at 1.');
        end                                          % the law_exact_step dependency is asserted where that flag is read
        % DIAGNOSTIC FLAG (2026-08-26): remove the THERMAL position<->gain
        % cross-covariance from P propagation only: Q(3,4), Q(4,8) and the
        % memory->gain rows F_aug(4,8:9). Q44, Q33, the state predict and
        % the measurement models are untouched. Tests whether the sign of
        % P(4,1) (and hence of K1(4)) near the wall comes from this
        % physically real coupling. Default false => bit-identical.
        q34_off = get_field_default(ctrl_const, 'q34_off', false);
        % DIAGNOSTIC KNOB (2026-08-26): multiply Q(4,4) only (the gain state's
        % own process noise), leaving Q(3,4) and everything else as built.
        % Reverse discriminator of the P(4,1) sign competition: does a larger
        % P44 let F_e(3,4)*P44 overtake Q(3,4)? Default 1 => bit-identical.
        q44_scale = get_field_default(ctrl_const, 'q44_scale', 1);
        C_dpmr          = ctrl_const.C_dpmr;
        C_n             = ctrl_const.C_n;
        K_var           = ctrl_const.K_var;
        IF_abc          = ctrl_const.IF_abc(:);
        t_warmup_kf     = ctrl_const.t_warmup_kf;
        h_bar_safe      = ctrl_const.h_bar_safe;
        a_cov           = ctrl_const.a_cov;
        a_pd            = ctrl_const.a_pd;
        amlpf_var_factor = get_field_default(ctrl_const, 'amlpf_var_factor', 1);
        % GUARD (08-19 trap, 08-30 hygiene): this controller has NO a_m LPF
        % (the LPF lives only in eq17_6state), yet the shared builder still
        % honours opts.use_am_lpf and would hand us amlpf_var_factor = 0.089:
        % R2 shrinks 11x while the readout is never filtered. Refuse.
        % (amlpf_var_factor itself stays overridable: sweep_R2_trust_y2.m uses
        % it as an explicit R2 multiplier, which is a declared diagnostic.)
        if logical(get_field_default(ctrl_const, 'use_am_lpf', false))
            error('motion_control_law_formC_b:noAmLpf', ...
                  'use_am_lpf is not supported here: formC_b has no a_m LPF (builder would hand R2 a factor of %g).', amlpf_var_factor);
        end
        % xi_bar per the spec definition xi = (C_n/C_dpmr)*sigma2_nw/kappa_T
        % (computed from kappa_T directly, NOT sibling xi / a_o, to keep a_o
        % out of the filter constants).
        xi_bar = (C_n / C_dpmr) * sigma2_n_nd / kappa_T;

        y2_whiten    = logical(get_field_default(ctrl_const, 'y2_whiten', true));
        r2_scale     = get_field_default(ctrl_const, 'r2_scale', 1);   % diagnostic y2-weight sweep (user 09-23: "R2 halved as a whole"); NOT a derived number -- report as a sweep
        assert(isscalar(r2_scale) && r2_scale > 0, 'motion_control_law_formC_b:r2Scale', 'ctrl_const.r2_scale must be a positive scalar.');
        fe_row4_full = logical(get_field_default(ctrl_const, 'fe_row4_full', true));
        % Colour factor matches the readout object (2026-08-27): the whitened
        % increment y2 = a_cov*u[k] is a single sample, so its correlation
        % penalty is the long-run IF_white (s = 1 sums), not the EWMA-output
        % IF_eff (s = 1-a_cov). The raw branch keeps IF_abc. Selected by the
        % existing flag, no new knob; falls back to IF_abc if the builder
        % predates the field.
        if y2_whiten && isfield(ctrl_const, 'IF_abc_white')
            IF_abc = ctrl_const.IF_abc_white(:);
        end

        % Observability capture (model/diag/obs_dump.m). OFF by default and the
        % only cost when off is the persistent logical tested at the call site,
        % so a run with obs_dump = false is bit-identical to one without this
        % code. Consumed by verify_state_observability.m.
        obs_dump_on = logical(get_field_default(ctrl_const, 'obs_dump', false));
        obs_dump('reset', obs_dump_on);

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
        h2_walk_cols_off = logical(get_field_default(ctrl_const, 'h2_walk_cols_off', false));   % 1006_conly_H_simple.tex: H23 = H25 = 0 test
        y2_delay_off = logical(get_field_default(ctrl_const, 'y2_delay_off', false));   % 10-07 test: ignore the y2 delay entirely (H23 = H25 = 0 AND y2_pred without the walk-back)
        S_echo_T = 0; S_echo_n = 0;
        if y2_echo_corr
            assert(d_delay == 2, 'motion_control_law_formC_b:echoDelay', ...
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
            error('motion_control_law_formC_b:armConflict', ...
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
        % pade22 (2026-09-18, default false => bit-identical; reference/eq17_analysis/derivation/0918_estb_22_versionA.tex):
        % the [2/2] law a_bar = (g^2 + c1 g + c0)/(g^2 + d1 g + d2) with all four coefficients as constant states -- slots
        % 5 = d1, 7 = c0 (as c0_state) plus 8 = c1, 9 = d2 in the two slots MA(2) would use. Needs c0_state (asserted
        % where it is read); the state count is set here because everything below is sized by it.
        pade22 = logical(get_field_default(ctrl_const, 'pade22', false));
        if pade22
            assert(~ma2_aug, 'motion_control_law_formC_b:pade22', ...
                   'ctrl_const.pade22 carries c1/d2 in slots 8/9: ma2_aug must be off.');
            n_state = 9;
        end
        % STAGE B DIAGNOSTIC (2026-08-30, test_script/scratch/l3_stageB_prereg.txt):
        % quadrature rule of the ROW-4 PREDICT INTEGRAND only.
        %   'left' (default)  a_pred = a + a'(a) M                    production, arithmetic unchanged
        %   'mid'             a_pred = a + a'(a + a'(a) M / 2) M
        %   'trap'            a_pred = a + [a'(a) + a'(a + a'(a) M)] M / 2
        % with a'(.) = b_hat (1 - .)^2 under the production a_bar clamps.
        % REGISTERED SECOND-ORDER INCONSISTENCY: F_e(4,4) = 1 + a' F_dw + A_a M,
        % F_e(4,3)/(4,5), Q(3:4,3:4), H2 and y2_pred still use a'(a) at the LEFT
        % endpoint, so under 'mid'/'trap' the covariance propagation and the
        % predict differ by |a'(a_half) - a'(a)| / a' ~ (1-a) a' |M| = O(1e-2)
        % per step near the wall. Single factor by design (the flag changes
        % nothing else). On the ap_known arm the slope is exogenous (driver
        % value at the previous true/commanded height; no half/end-point value
        % is plumbed), so that arm keeps 'left' regardless of the flag.
        predict_quad = lower(get_field_default(ctrl_const, 'predict_quad', 'left'));
        assert(any(strcmp(predict_quad, {'left', 'mid', 'trap'})), ...
               'motion_control_law_formC_b:predictQuad', ...
               'ctrl_const.predict_quad must be ''left'' | ''mid'' | ''trap''.');
        % EXACT LAW STEP (port of 2a5dc29 on test/law-error-budget, 2026-09-01):
        % integrate dA/dw = b (1-A)^2 EXACTLY over the step instead of by forward
        % Euler. 1/(1-A) is affine in w, so
        %     1/(1 - A[k+1]) = 1/(1 - A[k]) + b M,   M = the same bracket Euler uses.
        % Removes the left-endpoint quadrature ratchet -1/2 sum a'' dw^2 > 0
        % (09-01, seed-at-truth a'_true arm: open-loop sum +0.00666 on the canonical
        % band vs measured +0.00680). F_e is left linearised (first order identical).
        % EXTENSION over 2a5dc29: the ap_known (exogenous slope) arm is INCLUDED,
        % via b_eff = a'_ext/(1-A)^2 -- see the predict block. (Default false until
        % 2026-09-04.)
        % PRODUCTION DEFAULT since 2026-09-04: law_exact_step, pred_mean2, nw_mcorr and pred_mean2_e4 are ON
        % (0903_aptrue_4state_from_true.tex S1-S11; production hold level +0.0012 +- 0.0016 / +0.0004 +- 0.0023
        % vs +0.019 / +0.010 before, run_prod_ladder.m). The dependent flags DEFAULT TO THEIR UPSTREAM value
        % (pred_mean2 <- law_exact_step, pred_mean2_e4 <- pred_mean2), so law_exact_step = false alone gives the
        % pre-09-04 Euler recipe without tripping the dependency errors; nw_mcorr is independent.
        law_exact_step = logical(get_field_default(ctrl_const, 'law_exact_step', true));
        % pred_mean2 (2026-09-02, default false until 2026-09-04, now ON): add to the row-4
        % predict the second-order MEAN term of 0902_formC_aptrue_4state.tex S5,
        % i.e. the expected drift of e_a that a first-order predict leaves out:
        %     - a'' E[e3 (B - B_hat)]            (stochastic part of the slope-evaluation term)
        %     + 1/2 a'' Var(B - B_hat)            (Jensen: noisy step through the concave law)
        %     + 1/2 (a'' - a''_law) B_hat^2        (true curvature vs the constant-b curvature of the exact step)
        % with B - B_hat = (1-lc) e3 [+ alpha (e8+e9)] + F_dw e4 + w_T [- alpha n_w], all
        % moments from P_curr / the Q containers; a'' = app_known if fed, else a''_law.
        % The DETERMINISTIC slope-evaluation group -a'' dw3_hat B_hat is NOT added here:
        % it is removed by reading the exogenous slope at the estimated height
        % (driver ap_known_at = 'est'), never by the controller. Derived on the exact
        % step, so it requires law_exact_step. Diagnostic arm only.
        pred_mean2 = logical(get_field_default(ctrl_const, 'pred_mean2', law_exact_step));   % default follows law_exact_step (production default ON)
        if pred_mean2 && ~law_exact_step
            error('motion_control_law_formC_b:predMean2', ...
                  'ctrl_const.pred_mean2 is derived on the exact law step; set law_exact_step = true as well.');
        if jac_exact_step
            assert(law_exact_step, 'motion_control_law_formC_b:jacExactStep', ...
                   'ctrl_const.jac_exact_step linearises the exact law step; set law_exact_step = true as well.');
        end
        end
        % pred_force_step (2026-09-02, default false => bit-identical): the row-4
        % predict's KNOWN increment becomes the commanded displacement
        %     a_ctrl[k-1] * fbar_d[k-1]      (a_ctrl = the gain the control law used)
        % instead of the closed-loop proxy Delta_w_d + (1-lc) dw3_hat [+ alpha(m1+m2)].
        % Probe (canon deep, 8 seeds, hold): the proxy differs from the commanded
        % displacement by sd 3e-4 (13% of the step) and that difference is
        % correlated with dw3_hat, E = -4.5e-7 -- exactly the A2 violation that
        % left +0.0005 (canon) / +0.002 (meng) in the compensated a'_true arm.
        % With the commanded displacement the unknown increment is exactly
        % w_T + fbar_d e_a (var 4.319e-6 vs 4.321e-6 expected) and orthogonal to
        % the estimate (E[dw3_hat u] = 0.04e-6). pred_mean2 then uses
        %     cov(e3,u) = fbar_d P34 ,  Var(u) = fbar_d^2 P44 + kappa_T a_hat
        % (no P33, no memory covariances). F_e is left as is (~1% effect on P).
        % Requires law_exact_step (the increment feeds the exact step).
        % STATUS: FALSIFIED as a fix (2026-09-02, a'_true@est + exact + pred_mean2,
        % same 10 seeds as the proxy arm). Variant x_curr(4)*fbar_d[k-1] (posterior of
        % k-1 x force): canon hold -0.0007 with the seed spread x2.4 (0.0026 -> 0.0061),
        % meng hold -0.031. Variant a_ctrl[k-1]*fbar_d[k-1] (this line): canon hold
        % +0.0011/+0.0038/+0.0037 on seeds 1-3 vs proxy +0.0013/-0.0003/+0.0012, i.e.
        % worse than the proxy it was meant to replace. Mechanism: the commanded
        % displacement carries the gain ESTIMATE as a factor (fbar_d = intended/a_ctrl),
        % so gain-estimate noise becomes height-increment noise and walks the gain
        % along the law; the closed-loop proxy is anchored to command + tracking error
        % (y1) and is immune. Kept as a recorded negative result; do not enable.
        % pred_mean2_kr1 (2026-09-02, default false => bit-identical): the fourth mean
        % term, the A2 residual in closed form. The controller reacts to the y1 noise
        % n_w (a real displacement, -(1-lc) n_w inside eps_w) and the posterior dw3_hat
        % moved with the SAME n_w through K31, so E[dw3_hat (B - B_hat)] = (1-lc) K31 R1
        % (probe: 4.25e-7 closed form vs 4.93e-7 measured at hold, 5.61 vs 5.46 in the
        % oscillation). The cross term a'' B_hat (B - B_hat) of the expansion then has
        % mean a'' (1-lc)^2 K31 R1, which is added here; K31 is the y1 gain of the
        % previous call (buffer), R1 the y1 noise variance. No new state, c-free.
        pred_mean2_kr1 = logical(get_field_default(ctrl_const, 'pred_mean2_kr1', false));
        % STATUS (2026-09-03 evening): kr1 AND kr1_full are RETRACTED as compensation.
        % The first-order slope covariance E[a' u] = -a'' (1-lc) K31 R1 (a' is read at
        % the filter's height, which shares n_w with u) cancels the three shares below
        % exactly (probe: Cov(a',u) +0.736 vs shares -0.694, 1e-6/step, canon hold);
        % the truth's net feedthrough is ~0 and either flag drifts the hold estimate
        % down (long hold: -0.22e-6/step with kr1_full, +0.24 without). Keep off.
        % pred_mean2_kr1_full (2026-09-03, default false => bit-identical): the SAME
        % n_w also sits in the posterior error e3 = dw3 - dw3_hat (coefficient -K31),
        % so the two P-based pieces of pred_mean2 miss it as well:
        %     E[dw3_hat u] = +(1-lc) K31 R1        share  +a'' (1-lc)^2 K31 R1   (kr1)
        %     E[e3 u]      = cov_code - (1-lc) K31 R1   share  +a'' (1-lc)   K31 R1
        %     E[u^2]       = var_code - 2 (1-lc)^2 K31 R1   share  -a'' (1-lc)^2 K31 R1
        % The first and third cancel; the net is a'' (1-lc) K31 R1, i.e. kr1 / (1-lc).
        % Probe (analyze_aptrue_est_final.m, 8 seeds, 1e-6/step): remainder after kr1
        % meng near -0.229 / hold -0.472, canon hold -0.513; the missing share
        % a'' (1-lc) lc K31 R1 = -0.200 / -0.473 / -0.486. Supersedes kr1 when on.
        pred_mean2_kr1_full = logical(get_field_default(ctrl_const, 'pred_mean2_kr1_full', false));
        K31_km1 = zeros(3, 1);
        pred_force_step = logical(get_field_default(ctrl_const, 'pred_force_step', false));
        if pred_force_step && ~law_exact_step
            error('motion_control_law_formC_b:predForceStep', ...
                  'ctrl_const.pred_force_step feeds the exact law step; set law_exact_step = true as well.');
        end
        y1_gain_off  = logical(get_field_default(ctrl_const, 'y1_gain_off', false));
        % y1_gain_off_from (2026-09-03, diagnostic, default 0 = never): same cut as
        % y1_gain_off but only from controller step k_step >= this value, so the
        % y1 -> gain leg can be removed in a hold WITHOUT touching the motion
        % segments (paired against the untouched run it measures the mean y1
        % innovation offset through the gain row: d(a_hat) = -l41 * iota * N).
        y1_gain_off_from = get_field_default(ctrl_const, 'y1_gain_off_from', 0);
        % mean-only diagnostic knobs (2026-09-03, default off => bit-identical). From
        % k_step >= mean_knob_from: (i) the slope used in the row-4 KNOWN-STEP increment
        % (exact law step, ap_known branch) is multiplied by pred_slope_scale -- P, F_e
        % and the y2 back-off untouched; (ii) the y1 -> gain STATE correction
        % K1(4)*innov1 is multiplied by y1_gain_leg_scale -- P (Joseph, unscaled K1)
        % untouched. Paired against the unscaled run they give d(hold drift)/d(flow)
        % for the two first-order mean flows the level-mode equations leave open.
        % nw_mcorr (2026-09-03, default false until 2026-09-04, now ON): the controller reacts to
        % the same y1[k] the update uses, so the process noise w[k] = gn n_w[k] + w_T-part
        % is correlated with the measurement noise: M = E[w n_w] = R1 gn, gn = (1-lc)[0 0 -1 a' ...].
        % The KF assumed M = 0; that assumption is the root of the whole n_w feedthrough
        % family measured on 2026-09-03 (Cov(dw3_hat,u), Cov(a',u), Cov(dw3_hat,innov1)).
        % Standard correlated-noise form (derivation 0903_formC_aptrue_update_half.tex S6):
        %   w = (M/R1) n_w + w~,  n_w = y1 - H1 x  =>  x+ = (F - gn H1) x + B u + gn y1 + w~
        %   predict: x_pred = F x_upd + B u + gn (y1 - x_upd(1))   [posterior residual of the previous call]
        %            F_e <- F_e - gn e1',  Q <- Q - R1 gn gn'  (the n_w share leaves Q)
        %   pred_mean2: u = (1-lc)(e3 - e1) + ..., so cov/var use P13, P11, P18, P19, P14.
        % c-free: needs y1, x_upd(1), R1, lc, a' only.
        nw_mcorr = logical(get_field_default(ctrl_const, 'nw_mcorr', true));   % production default ON since 2026-09-04
        res1_km1 = zeros(3, 1);
        % pred_mean2_e4 (2026-09-04, default OFF until the evening of 2026-09-04, now follows pred_mean2): when the slope reads the
        % GAIN STATE (a_bar' = b (1 - a_hat)^2: production and the b_true arm; NOT the ap_known
        % arm, whose slope reads a height), the start-point term of the second-order mean is
        %     (d a_bar'/d a_hat) Cov(e4, u) = A_a [ (1-lc) P34 + alpha (P48 + P49) + F_dw P44 ]   [nw_mcorr: - (1-lc) P14]
        % in place of pred_mean2's height-reading -a'' Cov(e3, u). The two differ by
        %     A_a Cov(e4 + a' e3, u)
        % which is ~0 in a hold (P34 = -a' P33 there) and -1.2e-6 (canon) / -0.2e-6 (Meng) per step
        % in the near-wall descent (probe_btrue_e4_line.m, 10 seeds). Derivation:
        % 0903_aptrue_4state_from_true.tex S10. Requires pred_mean2. With slot 5 FREE (production) the
        % same flag adds the e_b line (S11): (d a'/d b_hat) Cov(e_b, u) = (a'/b_hat) [ (1-lc) P35 + alpha (P58 + P59)
        % + F_dw P45 ] [nw_mcorr: - (1-lc) P15], the filter's own posterior cross-covariances (constant-b model).
        pred_mean2_e4 = logical(get_field_default(ctrl_const, 'pred_mean2_e4', pred_mean2));   % default follows pred_mean2 (production default ON)
        if pred_mean2_e4 && ~pred_mean2
            error('motion_control_law_formC_b:predMean2E4', ...
                  'ctrl_const.pred_mean2_e4 replaces a term of pred_mean2; set pred_mean2 = true as well.');
        end
        mean_knob_from    = get_field_default(ctrl_const, 'mean_knob_from', 0);
        pred_slope_scale  = get_field_default(ctrl_const, 'pred_slope_scale', 1);
        y1_gain_leg_scale = get_field_default(ctrl_const, 'y1_gain_leg_scale', 1);
        % T2 hook (TEST ONLY, default false): zero the loop-coupling column
        % F_dw and the process noise Q, so P(4,4) propagates by the row-4
        % diagonal alone. Then the tex S6 flow identity is EXACT
        %     sqrt(P44[k]) / a_bar'[k] = const
        % and any error in the A_a*M term shows up as a drift in that ratio.
        % Use with y2_on = false and y1_gain_off = true.
        t2_pure_prop = logical(get_field_default(ctrl_const, 't2_pure_prop', false));
        % Forgetting factor, Menq thesis eq (4.15):
        %     P[k] = (1/lambda_f) * [I - K[k] H] P^f[k]
        % i.e. a MULTIPLICATIVE inflation of the POSTERIOR, applied once after
        % the measurement update. Our update is sequential 1-D (y1 then y2), so
        % it is applied once after BOTH, which is the same object as his single
        % joint update. Multiplicative means locked slots (P == 0 exactly) stay
        % locked: 0/lambda = 0. Note it inflates the whole P, including the
        % dw_bar block, not just the gain -- that is faithful to the method.
        % lambda_f = 1 disables it exactly (default). NO derivation exists for
        % its value; it is a knob, so results are reported as a sweep.
        % STATUS: DIAGNOSTIC ONLY, NOT AN ADOPTED FIX. Falsified 2026-08-12 on
        % 12/12 paired comparisons (2 trajectories x 2 arms x 3 lambda values):
        % monotonically harmful, nothing improved. It does repair the honesty
        % ratio late in a run (rho 1.29 -> 0.79 on canonical) but costs more
        % than it buys, because rho(t) here is NOT uniformly biased -- it is
        % over-pessimistic early (0.18) and over-confident late, and a
        % uniform-in-time inflation cannot help a non-uniform deficit.
        % The derivation deliberately does NOT carry it: its S8 discipline is
        % to NAME what the model omits and price it into Q; a forgetting
        % factor inflates P without naming anything, which is the opposite.
        % Its legitimate use here is as an INSTRUMENT: the lambda that restores
        % rho -> 1 measures the size of the unmodelled variance.
        % Where the SLOPE is evaluated. The true-slope arm showed the whole
        % gap to the height writing is the integrand carrying the state's own
        % error; these break that feedback WITHOUT using the wall:
        %   'post' (default)  a_bar_hat as it stands = today's behaviour
        %   'pred'            the PRE-update value, so this step's measurement
        %                     noise never reaches the slope
        %   'ewma'            an EWMA of a_bar_hat, weight ap_ewma_a
        ap_src    = lower(get_field_default(ctrl_const, 'ap_src', 'post'));
        law_b_formC = get_field_default(ctrl_const, 'law_b_formC', 1);
        % DEAD KNOB (08-25 audit): the law's b comes from slot 5 (b_hat_i);
        % law_b_formC is read nowhere else. Refuse a non-default value so a
        % caller cannot believe it is tuning the law.
        if ~isequal(law_b_formC, 1)
            error('motion_control_law_formC_b:deadKnob', ...
                  'ctrl_const.law_b_formC has no effect in this writing (b is slot 5); got %g.', law_b_formC);
        end
        ap_ewma_a = get_field_default(ctrl_const, 'ap_ewma_a', 0.05);
        assert(any(strcmp(ap_src, {'post','pred','ewma','cmd','act'})), ...
               'motion_control_law_formC_b:apSrc', 'ap_src must be post|pred|ewma|cmd|act.');
        a_bar_slope_v = [];
        lambda_f = get_field_default(ctrl_const, 'lambda_f', 1);
        assert(isscalar(lambda_f) && lambda_f > 0 && lambda_f <= 1, ...
               'motion_control_law_formC_b:lambdaF', ...
               'lambda_f must be a scalar in (0, 1]; got %g.', lambda_f);
        % SLOT-5-ONLY forgetting. The Menq factor above inflates the whole
        % posterior; its 2026-08-12 falsification (12/12 harmful) rested on
        % rho(t) being NON-UNIFORMLY biased -- pessimistic early, confident late
        % -- so a uniform-in-time, uniform-in-state inflation cannot help.
        %
        % For slot 5 alone the deficit IS one-sided and late. Measured
        % 2026-08-20 on the deep band: 77-90 % of b_hat's total travel happens
        % inside the 1 s descent, the 1.9 s oscillation contributes -0.7 to
        % +18.8 %, and both holds contribute nothing -- yet P55 keeps shrinking
        % all the way to 4.8 s, by up to 512x. The information stops arriving at
        % t = 1.5 s and the covariance does not notice. That is a different
        % object from the one that was falsified, which is why this exists.
        %
        % Applied as a CONGRUENCE transform, D P D with D = diag(1,1,1,1,
        % 1/sqrt(lambda),1,1): P(5,5) gains 1/lambda and every cross term
        % P(i,5) gains 1/sqrt(lambda). Scaling P(5,5) alone would inflate the
        % variance without the covariances and can leave P indefinite; a
        % congruence cannot, whatever lambda is.
        %
        % STATUS: DIAGNOSTIC. lambda_f_b has NO derivation -- it is a knob, and
        % the charter forbids tuned numbers in production. Report it as a sweep,
        % never as a chosen value. 1 disables it exactly.
        lambda_f_b = get_field_default(ctrl_const, 'lambda_f_b', 1);
        assert(isscalar(lambda_f_b) && lambda_f_b > 0 && lambda_f_b <= 1, ...
               'motion_control_law_formC_b:lambdaFb', ...
               'lambda_f_b must be a scalar in (0, 1]; got %g.', lambda_f_b);
        % ADAPTIVE slot-5 forgetting, after Meng's dissertation Ch.6 p.100:
        %     lambda[k+1] = exp(-alpha * residual^2)
        % (the 2025 TIE version deleted the forgetting factor entirely, so our
        % lineage inherited the constant-lambda form from the paper, not this
        % one). Large residual forgets hard, small residual does not forget --
        % the loop is self-limiting, which is exactly the property a CONSTANT
        % lambda lacks and the reason its 2026-08-12 falsification does not
        % transfer.
        %
        % Two deliberate departures from his form, both to make alpha a
        % BANDWIDTH rather than a scenario-specific value:
        %
        %  1. the residual is NORMALISED, innov2^2/S2 instead of innov2^2. The
        %     raw square carries units, so alpha would have to be re-tuned for
        %     every gain scale and every R. Normalised it is a chi-square(1)
        %     statistic and dimensionless.
        %  2. the EXCESS over unity is used, max(0, innov2^2/S2 - 1). Under a
        %     correct model E[innov2^2/S2] = 1, so feeding the raw ratio would
        %     forget CONSTANTLY even when nothing is wrong. Subtracting one
        %     makes lambda = 1 the resting state and forgetting a response to
        %     the filter being surprised BY ITS OWN standard.
        %
        % Only y2 matters: H(1,5) = 0, so y1 carries no information about b.
        %
        % The floor is a SAFETY clamp only, not the operating constraint. The
        % real constraint is a BUDGET on the total forgetting over the run:
        %
        %     sum_k (1 - lambda[k])  <=  1
        %
        % because a constant lambda at the information floor gives exactly
        % N*(1-lambda) = 1, i.e. one run's worth of data discarded. Applying
        % that steady-state number POINTWISE was measured (2026-08-20) to be
        % ~20x too strict: the adaptive law sits at lambda = 1 for 94.8 % of
        % steps, so its total spend was 0.052 of the budget, and every alpha
        % from 0.5 to 10 gave bit-identical output because each one saturated
        % the pointwise clamp on the first surprised step. A clamp that makes
        % the tuning parameter irrelevant is measuring the clamp, not the law.
        %
        % So the floor is set well below the steady-state value and the spend
        % is REPORTED (diag.lam_b_spend) for the caller to check after the run.
        % alpha = 0 disables the whole thing exactly.
        lfb_alpha = get_field_default(ctrl_const, 'lambda_f_b_alpha', 0);
        % Safety clamp only -- see the budget note above. 0.99 is far below
        % the steady-state information floor on purpose, so that alpha, not
        % the clamp, decides how hard a surprised step forgets.
        lfb_floor = get_field_default(ctrl_const, 'lambda_f_b_floor', 0.99);
        % 2026-09-24 (user: Meng's conditional forgetting as the main line for the c-only slope law): which innovation drives the
        % adaptive lambda. 'y2' = the 08-20/09-09 form (R57: NIS2 sits at its 0.3 design value, the excess never fires); 'y1' = the
        % channel that carries the slope constant's fingerprint (F_e(3,4) F_e(4,5) = F_dw M/h^2 three steps later, NIS1 design 1.00).
        % lambda_f_b_info = beta > 0 is the information-driven alternative (Kulhavy directional forgetting, O28 (1)):
        % lambda_k = exp(-beta * s_k), s_k = 1 - P55_upd/P55_pred = the fraction of P55 this step's data removed; s_k = 0 in hold.
        % Both leave lambda = 1 when nothing is learned, so neither winds up in hold. Reported as sweeps, not chosen values.
        lfb_chan = get_field_default(ctrl_const, 'lambda_f_b_chan', 'y2');
        lfb_info = get_field_default(ctrl_const, 'lambda_f_b_info', 0);
        assert(any(strcmp(lfb_chan, {'y1', 'y2'})), 'motion_control_law_formC_b:lambdaFbChan', 'lambda_f_b_chan must be ''y1'' or ''y2''.');
        assert(isscalar(lfb_info) && lfb_info >= 0, 'motion_control_law_formC_b:lambdaFbInfo', 'lambda_f_b_info must be a non-negative scalar.');
        % q55_adapt (2026-09-29, exploratory, default 'off' => bit-identical): Q55 added at predict from a running statistic of the
        % whitened y1 innovation nu1 = innov1/sqrt(S1) of the PREVIOUS steps (window q55_adapt_N):
        %   'wmean'  |mean nu1| > thr        => Q55 += gain |M|        'cusum'  max(g+, g-) > thr => Q55 += gain |M|  (allowance kcu)
        %   'prop'   Q55 += gain mean(nu1)^2 |M|                       'nis'    Q55 += gain max(0, mean(nu1^2) - 1) |M|
        % Path-proportional (|M|) so a hold adds nothing. Tuned sweeps, not derived values (user 09-29: find the mechanism first).
        q55a_mode = get_field_default(ctrl_const, 'q55_adapt', 'off');
        assert(any(strcmp(q55a_mode, {'off', 'wmean', 'cusum', 'prop', 'nis', 'ratio', 'rhoconst', 'r2ratio', 'ratiogate', 'ratioinnov', 'ratiodrift', 'covmatch', 'rationew', 'drift', 'driftsq'})), 'motion_control_law_formC_b:q55Adapt', 'q55_adapt must be off | wmean | cusum | prop | nis | ratio | rhoconst | r2ratio | ratiogate | ratioinnov | ratiodrift | covmatch | rationew | drift | driftsq.');
        % 'drift' / 'driftsq' (2026-10-06, Layer-1 session, user-approved test of O50): Q55 from how much c may change, no P_fix, no I.
        %   'drift'   Q55 = dc^2 |Delta w_d| / h_hat^2       (random walk in 1/h: one approach accumulates ~dc^2, every revisit adds it again)
        %   'driftsq' Q55 = (dc |Delta w_d| / h_hat^2)^2     (the step's deterministic change squared: right size per step, total over a pass ~0)
        %   dc = q55_adapt_dc (wall-family change of c_eff over one approach). Needs plaw_c_slope (h_v2).
        % 'rationew' (2026-10-04, Layer-1 session, user-approved): memory follows HEIGHT visited, not time. Q55 = P_fix^2 I_k with P_fix = P_new
        % (q55_adapt_P0) while the commanded gap h_d = w_d - w_s goes below the lowest h_d reached so far (new ground: c may be new), else
        % P_rev (q55_adapt_Prev; revisit: c already learned there, long memory). Ratio form, so no windup; P relaxes to P_rev only with evidence.
        % 'covmatch' (2026-10-04, Layer-1 session; covariance matching, Mehra 1970 / Mohamed & Schwarz 1999): the data's own push on c, dc_k = c_upd - c_pred
        % (both channels). If the model is right, E[dc_k^2] = P55_pred - P55_upd. EWMA (alpha = 1/q55_adapt_N) of D = dc^2 and Rm = P55_pred - P55_upd.
        % q55_adapt_cmf 'pure': Q55 = D (no P_fix)  |  'ratio': Q55 = P_fix^2 I_k * D/Rm (ratio scaled by observed / model-predicted correction power).
        % 'ratiodrift' (2026-10-04, Layer-1 session, user-approved; = O50 'placement C'): Q55 = P_fix^2 I_k + dc^2 |Delta w_d| / h_hat^2. Second term = c drifts with
        % HEIGHT: |Delta w_d|/h_hat^2 = the commanded step in 1/h, summing to ~0.86 from 22.2 R to contact, so one approach adds dc^2; dc = change of c_eff over one approach
        % (wall family: plane ~0.2, cell ~0.76). Additive, not multiplied on I: c_eff depends on height, I on speed^2. Needs plaw_c_slope (h_v2).
        % 'ratioinnov' (2026-10-04, Layer-1 session, user: "P_fix as a function of the innovation, no threshold"): Q55 = P_0^2 I_k (mean nu1 / sigma)^2,
        % sigma = 1/sqrt(q55_adapt_N) = the noise size of the windowed mean when c is right (so the factor has mean 1 there). I_k = the model says
        % c can be learned this step (P track); (mean nu1/sigma)^2 = the data says how wrong c is (data track). Notation 1004_q55_ratio_from_kf.tex.
        % 'ratio' (2026-09-30, Q/R-ratio design): R_k = 1/I_k, I_k = 1/P55_upd - 1/P55_pred (this step's information on slot 5, both
        % channels); rho_k = (P55[0]/R_k)^2; Q55 at the next predict = R_k * rho_k = P55[0]^2 * I_k. Holds the steady-state P55 at the
        % prior P55[0] wherever data flows and adds nothing where it does not (hold, far field). One constant, the prior itself.
        q55a_N    = max(1, round(get_field_default(ctrl_const, 'q55_adapt_N', 100)));
        q55a_thr  = get_field_default(ctrl_const, 'q55_adapt_thr', 0.3);
        q55a_gain = get_field_default(ctrl_const, 'q55_adapt_gain', 0);
        q55a_kcu  = get_field_default(ctrl_const, 'q55_adapt_kcu', 0.5);
        q55a_rho  = get_field_default(ctrl_const, 'q55_adapt_rho', 0);    % 'rhoconst' (09-30 user question): the textbook constant Q/R ratio, Q55 = rho / max(I_k, I_min)
        q55a_Imin = get_field_default(ctrl_const, 'q55_adapt_Imin', 1e-4); % 'rhoconst': floor on I_k so that Q stays finite where no information flows
        q55a_hgate = get_field_default(ctrl_const, 'q55_adapt_hgate', Inf);   % 10-02 (oscillation test): any q55_adapt mode adds Q only while the estimator height h_hat < hgate [R] (Inf = no gate)
        q55a_P0s  = get_field_default(ctrl_const, 'q55_adapt_P0', 0);   % 'ratio': sqrt(P_0) override (0 = the slot-5 prior std, the design default); the ONLY constant of the ratio design
        assert(isscalar(q55a_P0s) && q55a_P0s >= 0, 'motion_control_law_formC_b:q55AdaptP0', 'q55_adapt_P0 must be a non-negative scalar (sqrt of P_0).');
        q55a_wI   = logical(get_field_default(ctrl_const, 'q55_adapt_wI', false));   % 'ratioinnov' (10-04): I-weighted window, statistic = sum(I nu1)/sqrt(sum I^2) (= mean/sigma, weighted by learnability)
        q55a_gfun = get_field_default(ctrl_const, 'q55_adapt_gfun', 'lin');        % 'ratioinnov' (10-04): factor g(x), x = (mean nu1/sigma)^2: 'lin' g = x | 'max1' g = max(1, x) (P_low is the floor)
        assert(any(strcmp(q55a_gfun, {'lin', 'max1'})), 'motion_control_law_formC_b:q55AdaptGfun', 'q55_adapt_gfun must be lin | max1.');
        q55a_cmf  = get_field_default(ctrl_const, 'q55_adapt_cmf', 'ratio');       % 'covmatch' form, see above
        assert(any(strcmp(q55a_cmf, {'pure', 'ratio'})), 'motion_control_law_formC_b:q55AdaptCmf', 'q55_adapt_cmf must be pure | ratio.');
        q55a_Prev = get_field_default(ctrl_const, 'q55_adapt_Prev', 0);            % 'rationew': sqrt(P_rev) (0 = same as P_new, i.e. plain ratio)
        assert(isscalar(q55a_Prev) && q55a_Prev >= 0, 'motion_control_law_formC_b:q55AdaptPrev', 'q55_adapt_Prev must be a non-negative scalar (sqrt of P_rev).');
        q55a_dc   = get_field_default(ctrl_const, 'q55_adapt_dc', 0);              % 'ratiodrift': dc, change of c_eff over one approach [-]
        assert(isscalar(q55a_dc) && q55a_dc >= 0, 'motion_control_law_formC_b:q55AdaptDc', 'q55_adapt_dc must be a non-negative scalar.');
        assert(~(lfb_info > 0 && (lfb_alpha > 0 || lambda_f_b < 1)), 'motion_control_law_formC_b:lambdaFbInfo', ...
               'lambda_f_b_info is an alternative to lambda_f_b_alpha and a fixed lambda_f_b, not a stack.');
        assert(isscalar(lfb_alpha) && lfb_alpha >= 0, ...
               'motion_control_law_formC_b:lambdaFbAlpha', ...
               'lambda_f_b_alpha must be a non-negative scalar; got %g.', lfb_alpha);
        assert(isscalar(lfb_floor) && lfb_floor > 0 && lfb_floor <= 1, ...
               'motion_control_law_formC_b:lambdaFbFloor', ...
               'lambda_f_b_floor must be in (0, 1]; got %g.', lfb_floor);
        assert(~(lfb_alpha > 0 && lambda_f_b < 1), ...
               'motion_control_law_formC_b:lambdaFbBoth', ...
               'lambda_f_b_alpha and a fixed lambda_f_b < 1 are alternatives, not a stack.');
        Q_theta_floor = get_field_default(ctrl_const, 'Q_theta_floor', 0);

        % --- 0C. Parallel-axis law (x/y); constants are caller-supplied ---
        % In the ADDITIVE writing there is no constant that reshapes the curve
        % (da is a per-step increment, not a slope multiplier), so the whole
        % parallel package is the INTEGRATION CONSTANT w0_par alone, fitted
        % offline by the driver; x/y then lock slot 5 at 0 and carry their
        % representation error entirely in Pf_a_floor_par on P(4,4).
        % ws0_perp is read in 0F as the nominal wall (w0 = ws0_perp - 1).

        par_law = logical(get_field_default(ctrl_const, 'par_law', false));
        w0_par = 1;
        if par_law
            par_fields = {'w0_par', 'Pf_a_floor_par'};
            for ip = 1:numel(par_fields)
                if ~isfield(ctrl_const, par_fields{ip}) || isempty(ctrl_const.(par_fields{ip}))
                    error('motion_control_law_formC_b:missingParLaw', ...
                          ['par_law = true requires ctrl_const.%s. In the additive ', ...
                           'writing the parallel package is the seed origin w0_par ', ...
                           'plus the representation floor -- there is no parallel ', ...
                           'disturbance constant, because da is a per-step ', ...
                           'increment and cannot reshape a curve.'], par_fields{ip});
                end
            end
            w0_par = ctrl_const.w0_par;
        end

        % --- 0C'. Lock flags. lock_da = true IS derivation (a), the 4-state
        %     baseline: slot 5 pinned at its seed with zero P0, zero Jacobian
        %     and zero K, hence provably inert.
        % Slot 5 = da (additive). Slots 6-7 do not exist in this writing; they
        % are held locked at all times so the 9-slot layout (and the MA(2)
        % block at [8 9]) carries over unchanged. The filter IS the 5-state of
        % the tex when slot 5 is free, and the 4-state when it is locked.
        lock_da = logical(get_field_default(ctrl_const, 'lock_b', false));
        % E2 (2026-09-07): ctrl_const.da_slot frees slot 6 as the ADDITIVE gain disturbance da of formC_dist ('gain' placement):
        % row 4 gains + da per step outside the a_bar' bracket, F_e(4,6) = 1, H(2,6) = -d, P66[0] = Pf_da_std^2 (the driver's S3(b)
        % closed-form sup), Q66 = 0, seed 0. Purpose: give the innovations that the estimated-b arm otherwise charges to b_hat
        % (probe_estb_l52_split.m: canon +0.057 through y1) a level-type home. Paper gate 3 (0903 tex S11): da and b alias when
        % a' M is constant over the window (uniform far-field descent); they separate where a' or M changes. Default false.
        da_slot   = logical(get_field_default(ctrl_const, 'da_slot', false));
        Pf_da_std = expand3(get_field_default(ctrl_const, 'Pf_da_std', 0));
        if da_slot
            assert(all(Pf_da_std > 0), 'motion_control_law_formC_b:daSlot', 'ctrl_const.da_slot needs Pf_da_std > 0 from the driver.');
        end
        % b1_state (2026-09-14, default false => bit-identical; reference/eq17_analysis/derivation/0914_estb_bprime_core.tex):
        % slot 6 = b1 = db/dw, the law constant's HEIGHT slope, constant in the model (Q66 = 0). b moves with the step,
        %     b[k+1] = b[k] + b1[k] dw[k]  =>  F_e(5,:) = e5' + b1 c' + dw_hat e6',  g(5) = b1,  mean term Cov(e_6, u);
        % the exact law step gains + b1 dw^2/2 and the curvature a'' gains + b1 (1-a)^2. Seed b1[0] = 0, P66[0] = Pf_b1_std^2
        % (driver: sup |db_true/dw| on the envelope). b1_seed_couple (default true) adds the seed-line coupling of the tex:
        % P56[0] = (L/2) P66[0], P55[0] += (L^2/4) P66[0], L = start height - the seed line's zero-gain height.
        b1_state       = logical(get_field_default(ctrl_const, 'b1_state', false));
        Pf_b1_std      = expand3(get_field_default(ctrl_const, 'Pf_b1_std', 0));
        b1_seed_couple = logical(get_field_default(ctrl_const, 'b1_seed_couple', true));
        % amodel_fe (2026-09-11, default false => bit-identical; reference/eq17_analysis/derivation/0911_estb_green_variant.tex):
        % the "green" variant. The COVARIANCE model reads the gain error through the a model with w_s known,
        %     e_a -> psi = -a' e_3 + ((1-a)/b) e_b ,
        % wherever the step error u carries it (F_e rows 3-4: the F_dw e_4 share) and in the second-order mean term
        % (Cov(e_4,u) -> Cov(psi,u), u built with the green step-error vector). H, Q, R, P[0], the mean predict and the
        % control law are untouched. Acceptance: test_script/integration/verify_formC_b_amodel_fe.m.
        amodel_fe = logical(get_field_default(ctrl_const, 'amodel_fe', false));
        if amodel_fe
            assert(~jac_exact_step && ~pred_force_step && ~da_slot, 'motion_control_law_formC_b:amodelFe', ...
                   'ctrl_const.amodel_fe is derived on the Euler row-4 Jacobian with the production step error; jac_exact_step, pred_force_step and da_slot must be off.');
            assert(~pred_mean2 || pred_mean2_e4, 'motion_control_law_formC_b:amodelFe', ...
                   'ctrl_const.amodel_fe replaces Cov(e_4,u) in the pred_mean2_e4 term; set pred_mean2_e4 = true when pred_mean2 is on.');
        end
        % amodel_h (2026-09-12, default false => bit-identical; reference/eq17_analysis/derivation/0912_estb_orange_variant.tex):
        % the "orange" variant = green PLUS the same substitution in the y2 row, so the gain error is read through the
        % a model everywhere in the error model: H2(4) -> 0, its share moves onto e_3 and e_b. The predicted measurement
        % y2_pred, Q, R, P[0], the mean predict and the control law are untouched. The a slot then has a zero column in
        % BOTH rows and F keeps it invariant, so it is formally unobservable (gate 4 rank 6 of 7) while K_4 stays nonzero
        % through P_41, P_43, P_45. Acceptance: test_script/integration/verify_formC_b_amodel_fe.m (checks B).
        amodel_h = logical(get_field_default(ctrl_const, 'amodel_h', false));
        if amodel_h
            assert(amodel_fe, 'motion_control_law_formC_b:amodelH', ...
                   'ctrl_const.amodel_h is the measurement half of the same substitution; set amodel_fe = true as well.');
        end
        % gain_algebraic (2026-09-14, default false => bit-identical; reference/eq17_analysis/derivation/0914_estb_4state_core.tex):
        % the self-consistent 4-state that the a-model substitution points at. Declaring e_a = -a' e_3 + ((1-a)/b) e_b
        % removes the gain slot's own degree of freedom, so here the slot stops being a state: it is OVERWRITTEN every
        % step with the law value at the estimator's own height, w_hat = w_bar_d - dw3_hat, and slot 4 joins the locked
        % list (K(4) = 0, P row/col 4 pinned at 0). The filter is x = [dw1 dw2 dw3 b] with a derived gain output; the
        % start gain uncertainty now lives entirely in Pf_b_std through (1-a)/b. Needs amodel_fe and amodel_h so that
        % F_e and H carry no a_w column. Acceptance: verify_formC_b_amodel_fe.m (checks C).
        gain_algebraic = logical(get_field_default(ctrl_const, 'gain_algebraic', false));
        if gain_algebraic
            assert(amodel_fe && amodel_h, 'motion_control_law_formC_b:gainAlgebraic', ...
                   'ctrl_const.gain_algebraic is the 4-state form of the same substitution; set amodel_fe and amodel_h as well.');
        end
        % c0_state (2026-09-17, default false => bit-identical; reference/eq17_analysis/derivation/0916_estb_11_two_versions.tex,
        % version A): the [1/1] law in its Pade coefficients, a_bar = (g + c0)/(g + d1), g = w_hat - w_s, with BOTH
        % coefficients as constant states -- slot 5 carries d1 (NOT b), slot 7 carries c0 (unlocked) -- and the gain slot 4
        % is the law value at the estimator's own height (needs gain_algebraic, hence amodel_fe and amodel_h). w_s =
        % ctrl_const.ws0_perp is the ACTUAL wall, known. Pure 5-state only: MA(2), b1_state, da_slot, b_true and ap_known
        % must be off (b_bins / p55_floor are checked where they are read). Seeds c0_init / d1_init, priors Pf_c0_std /
        % Pf_d1_std. Reported through the existing slots: diag.b_hat = d1_hat, diag.ws_hat = c0_hat, P_b = P55, P_ws = P77.
        c0_state = logical(get_field_default(ctrl_const, 'c0_state', false));
        if c0_state
            assert(gain_algebraic, 'motion_control_law_formC_b:c0State', ...
                   'ctrl_const.c0_state is version A of 0916_estb_11_two_versions.tex: set gain_algebraic (and amodel_fe, amodel_h) as well.');
            assert(~b1_state && ~da_slot && ~ma2_aug && ~has_b_true && ~has_ap_known, 'motion_control_law_formC_b:c0State', ...
                   'ctrl_const.c0_state is the pure 5-state form: b1_state, da_slot, ma2_aug, b_true and ap_known must be off.');
            c0_seed   = expand3(get_field_default(ctrl_const, 'c0_init', 0));
            d1_seed   = expand3(get_field_default(ctrl_const, 'd1_init', 9/8));
            Pf_c0_std = expand3(get_field_default(ctrl_const, 'Pf_c0_std', 0));
            Pf_d1_std = expand3(get_field_default(ctrl_const, 'Pf_d1_std', 0));
            ws_11     = get_field_default(ctrl_const, 'ws0_perp', 1);
        end
        if pade22                                    % [2/2] (0918 tex): seeds c1_init / d2_init, priors Pf_c1_std / Pf_d2_std
            assert(c0_state, 'motion_control_law_formC_b:pade22', ...
                   'ctrl_const.pade22 is the [2/2] extension of c0_state: set c0_state (and gain_algebraic, amodel_fe, amodel_h) as well.');
            c1_seed   = expand3(get_field_default(ctrl_const, 'c1_init', 0));
            d2_seed   = expand3(get_field_default(ctrl_const, 'd2_init', 0));
            Pf_c1_std = expand3(get_field_default(ctrl_const, 'Pf_c1_std', 0));
            Pf_d2_std = expand3(get_field_default(ctrl_const, 'Pf_d2_std', 0));
            % smallest gap g = w_hat - w_s at which the [2/2] denominator must stay positive (the band floor in the
            % caller's coordinate: 0.1 when w_s is the contact height, 1.1 when w_s is the wall surface). Default 0 keeps
            % the original guard d2 >= 2 gap_floor.
            pade22_gmin = get_field_default(ctrl_const, 'pade22_gmin', 0);
        end
        % delta_state (2026-09-20, default false => bit-identical; reference/eq17_analysis/derivation/0920_estb_delta_law.tex,
        % version A): the offset law a_bar = D_perp(1/g), g = (w_hat - w_s) + delta, D_perp = the exact plane polynomial
        % (calc_correction_functions), with ONE constant state: slot 7 = delta, the depth of the effective (hydrodynamic) wall
        % below the known reference surface w_s = ctrl_const.ws0_perp (plane 0, cell 1.05 R; ledger C62). Zero shape constants:
        % slot 5 is LOCKED (inert), slot 4 is the law value at the estimator's own height (needs gain_algebraic, hence amodel_fe
        % and amodel_h). d a/d delta = a', d a'/d delta = a'' (sympy: check_estb_delta_jacobians.py, ALL PASS). Seed delta_init,
        % prior Pf_delta_std (two-sided). Reported through the existing slot: diag.ws_hat = delta_hat, P_ws = P77.
        delta_state = logical(get_field_default(ctrl_const, 'delta_state', false));
        if delta_state
            assert(gain_algebraic, 'motion_control_law_formC_b:deltaState', ...
                   'ctrl_const.delta_state is version A of 0920_estb_delta_law.tex: set gain_algebraic (and amodel_fe, amodel_h) as well.');
            assert(~c0_state && ~pade22 && ~b1_state && ~da_slot && ~ma2_aug && ~has_b_true && ~has_ap_known, 'motion_control_law_formC_b:deltaState', ...
                   'ctrl_const.delta_state is the pure 4-state offset law: c0_state, pade22, b1_state, da_slot, ma2_aug, b_true and ap_known must be off.');
            delta_seed   = expand3(get_field_default(ctrl_const, 'delta_init', 0));
            Pf_delta_std = expand3(get_field_default(ctrl_const, 'Pf_delta_std', 0));
            ws_11        = get_field_default(ctrl_const, 'ws0_perp', 0);
        end
        % delta_iekf (2026-09-20, default false => bit-identical; ledger O38): the y2 row is a strongly nonlinear function
        % of delta (a_bar = D_perp(1/(h + delta)), slope 0.77 -> 0.21 across a 1 R correction at h = 1.1), so ONE first-order
        % update converts a large innovation with the wrong slope, moves half way and collapses P as if it had converged.
        % Iterate the update at the posterior (Gauss-Newton on the y2 row: x_i+1 = x- + K_i (y - h(x_i) - H_i (x- - x_i)))
        % until delta stops moving; P from the last linearisation. Pure numerical convergence, no tuning knob.
        delta_iekf = logical(get_field_default(ctrl_const, 'delta_iekf', false));
        if delta_iekf
            assert(delta_state, 'motion_control_law_formC_b:deltaIekf', 'ctrl_const.delta_iekf needs delta_state.');
        end
        % delta_slin (2026-09-20 night, default false => bit-identical): statistical linearisation of the offset law over
        % the delta uncertainty. With a 1 R initial error (2 sigma of the family prior) the point slope a'(g_hat) is 0.77
        % where the slope averaged over the error range is 0.40: every Jacobian that carries a' (F_e(3,7), H2(3), H2(7))
        % overstates the information by up to 4x, P collapses and the mean stalls (fixed 1.10 R, plane init on the cell:
        % delta_hat 0.77 after 3 s, 4.7 sigma off, while +-0.5 R seeds converge within sqrt(P)). Replace a', a'' and the
        % predicted reading by their Gaussian expectations over delta ~ N(delta_hat, P77) with the 3-point Gauss-Hermite
        % rule (nodes delta_hat, delta_hat +- sqrt(3 P77); weights 2/3, 1/6, 1/6; exact to 5th order). No tuning knob:
        % the width is the filter's own P77 and vanishes as it converges (point EKF recovered).
        delta_slin = logical(get_field_default(ctrl_const, 'delta_slin', false));
        if delta_slin
            assert(delta_state, 'motion_control_law_formC_b:deltaSlin', 'ctrl_const.delta_slin needs delta_state.');
        end
        % plaw_c_state (2026-09-21, default false => bit-identical; the advisor's slope form a' = (1 - c)/h^p tested once under the
        % wall-test protocol, version A): a_bar = 1 - (1 - c)/((p - 1) h^(p-1)), h = w_hat - w_s (w_s = ctrl_const.ws0_perp, the
        % visible surface), integrated from the far field (a(inf) = 1 is the definition of a_nom). p is FIXED at the plane's band
        % fit (ctrl_const.plaw_p, fit_advisor_law_walls.m: 1.9666 on [1.10, 22.2]); ONE constant state: slot 7 = c (the gain at
        % the visible contact when p = 2; a shape-type constant, ledger C53). Slot 5 is LOCKED (inert), slot 4 is the law value at
        % the estimator's own height (needs gain_algebraic, hence amodel_fe and amodel_h). Jacobians (sympy: check_estb_plaw_jacobians.py):
        % d a/d c = (1 - a)/(1 - c), d a'/d c = -1/h^p, d a'/d dw3 = -a''. Seed c_init (plane 0), prior Pf_c_std (two-sided).
        % Reported through the existing slot: diag.ws_hat = c_hat, P_ws = sqrt(P77).
        plaw_c_state = logical(get_field_default(ctrl_const, 'plaw_c_state', false));
        if plaw_c_state
            assert(gain_algebraic, 'motion_control_law_formC_b:plawCState', ...
                   'ctrl_const.plaw_c_state is version A of the advisor power law: set gain_algebraic (and amodel_fe, amodel_h) as well.');
            assert(~c0_state && ~pade22 && ~delta_state && ~b1_state && ~da_slot && ~ma2_aug && ~has_b_true && ~has_ap_known, 'motion_control_law_formC_b:plawCState', ...
                   'ctrl_const.plaw_c_state is the pure 4-state power law: c0_state, pade22, delta_state, b1_state, da_slot, ma2_aug, b_true and ap_known must be off.');
            plaw_p   = get_field_default(ctrl_const, 'plaw_p', 2);
            assert(plaw_p > 1, 'motion_control_law_formC_b:plawCState', 'ctrl_const.plaw_p must exceed 1 (a(inf) = 1 needs the integral of 1/h^p to converge).');
            c_seed   = expand3(get_field_default(ctrl_const, 'c_init', 0));
            Pf_c_std = expand3(get_field_default(ctrl_const, 'Pf_c_std', 0));
            ws_11    = get_field_default(ctrl_const, 'ws0_perp', 0);
        end
        % plaw_c_slope (2026-09-21, default false => bit-identical; reference/eq17_analysis/derivation/0921_estb_v2_slope_c.tex,
        % "version 2", user 09-21): the a slot is KEPT as a state (production structure) and the advisor slope form
        % a' = (1 - c)/h^p, read at the estimator's own height h = w_hat - w_s (w_s = ctrl_const.ws0_perp, the visible surface),
        % serves ONLY its propagation, a[k+1] = a[k] + int_h^{h+M} a' dh (exact integral of the height law over the step,
        % replacing the Euler step; the Euler ratchet of 08-26 has no closed-form exact step for a general law, this one has).
        % Slot 5 = c, the one constant (free unless lock_b); p = ctrl_const.plaw_p fixed by the plane fit (2.042). Jacobians
        % (sympy check_estb_v2_slope_c.py 15/15): d a'/d c = -1/h^p, d a'/d dw3_hat = +p a'/h, d a'/d a_hat = 0 => F_e(4,4) =
        % 1 + a' F_dw (no A_a M term), F_e(4,5) = -M/h^p, F_e(4,3) = a'(1-lc) + M p a'/h; H2 = [0 0 -Grad p a'/h 1 +Grad/h^p].
        % Seed: a_hat[0] = plane polynomial at the start height (the only value law used), c_init (plane fit +0.037), Pf_c_std.
        % Pure a-slot form: gain_algebraic / amodel_fe / amodel_h / c0_state / pade22 / delta_state / plaw_c_state / b1_state /
        % da_slot / b_true / ap_known / law_exact_step / pred_mean2 must be off (their law-specific terms are the b law's);
        % ma2_aug and nw_mcorr are law-independent and allowed. Reported through the existing slot: diag.b_hat = c_hat, P_b = sqrt(P55).
        plaw_c_slope = logical(get_field_default(ctrl_const, 'plaw_c_slope', false));
        % plaw_dc_slope (2026-09-22, default false => bit-identical; reference/eq17_analysis/derivation/0922_estb_f2_slope_dc.tex):
        % the two-constant slope law a' = (1 - c)/(h + delta)^2 read at the estimator height, g_hat = w_d - dw3_hat - w_s + delta_hat.
        % Same a-slot structure as plaw_c_slope (which it switches on), p = 2 pinned by the far-field anchor (9/8)/h^2; slot 5 = c
        % (level type), slot 7 = delta (position type, delta > 0 = effective wall below the visible surface, same sign as delta_state).
        % Jacobians (sympy check_estb_f2_slope_dc.py 33/33): d a'/d delta = d a'/d h = -2 a'/g, so F_e(4,7) = -(2 a'/g) M and
        % H2(7) = +Grad 2 a'/g = -H2(3): the slope sees e_delta - e_dw3 only. Seed: a_hat[0] = plane polynomial (protocol),
        % delta_init / c_init = the plane's band pair (+0.104, -0.085), priors Pf_delta_std / Pf_c_std; P44[0] carries the family's
        % value sensitivities a'^2 Pf_delta^2 + Pf_c^2/g^2 with the cross terms P47 = a' Pf_delta^2, P45 = Pf_c^2/g (tex, Seed).
        % Guards: g_hat >= plaw_g_min (numerical, default 0.5), delta_hat in plaw_delta_lim (default [-0.9, 3]), 1 - c >= gap floor.
        % Reported through the existing slots: diag.b_hat = c_hat, P_b = sqrt(P55); diag.ws_hat = delta_hat, P_ws = sqrt(P77).
        % plaw_dp_slope (2026-09-23, default false => bit-identical; 0923_estb_f4_slope_dp.tex): the SAME family as plaw_dc_slope in the
        % other coordinate, a' = 1/(h + delta)^p -- the contact slope is pinned at 1 (rigid contact: plane 1.000, shifted plane 1.000,
        % sphere 1.024) and the far-field level is carried by the exponent p in slot 5 instead of the level constant c. Chosen because
        % the exponent's column carries (1 + ln g) more information than the level's (probe_ident_map_f2.m: descent CRLB/prior p
        % 0.019/0.038 vs c 0.076/0.19 on plane/cell) at equal family residual. Jacobians (sympy check_estb_f4_slope_dp.py 30/30):
        % d a'/d p = -a' ln g, d a'/d delta = d a'/d h = -p a'/g; exact step (g^(1-p) - (g+M)^(1-p))/(p-1). Seed p_init (plane band
        % fit 1.950), prior Pf_p_std; guard p_hat in plaw_p_lim (default [1.2, 4]). Everything else (delta slot, priors' cross terms,
        % guards, reporting) is plaw_dc_slope's with the value sensitivities of this law: d a/d delta = a', d a/d p =
        % (1/(p-1) + ln g)/((p-1) g^(p-1)). diag.b_hat = p_hat here.
        plaw_dp_slope = logical(get_field_default(ctrl_const, 'plaw_dp_slope', false));
        if plaw_dp_slope
            plaw_p_lim = get_field_default(ctrl_const, 'plaw_p_lim', [1.2, 4]);
            assert(numel(plaw_p_lim) == 2 && plaw_p_lim(1) > 1 && plaw_p_lim(1) < plaw_p_lim(2), 'motion_control_law_formC_b:plawDpSlope', ...
                   'ctrl_const.plaw_p_lim must be an increasing pair above 1 (the exact step needs p > 1).');
        end
        plaw_dc_slope = plaw_dp_slope || logical(get_field_default(ctrl_const, 'plaw_dc_slope', false));
        if plaw_dc_slope
            plaw_c_slope = true;                                       % all plaw_c_slope branches apply, with g_hat in place of h_hat
            delta_seed    = expand3(get_field_default(ctrl_const, 'delta_init', 0));
            Pf_delta_std  = expand3(get_field_default(ctrl_const, 'Pf_delta_std', 0));
            plaw_g_min    = get_field_default(ctrl_const, 'plaw_g_min', 0.5);
            plaw_delta_lim = get_field_default(ctrl_const, 'plaw_delta_lim', [-0.9, 3]);
            plaw_seed_cross = logical(get_field_default(ctrl_const, 'plaw_seed_cross', true));   % O40: false zeroes P45[0], P47[0] (P44[0] kept) -- the far-field wander discriminator
            assert(plaw_g_min > 0 && numel(plaw_delta_lim) == 2 && plaw_delta_lim(1) < plaw_delta_lim(2), 'motion_control_law_formC_b:plawDcSlope', ...
                   'ctrl_const.plaw_g_min must be positive and plaw_delta_lim an increasing pair.');
        end
        % plaw_cp_slope (2026-09-23 evening, default false => bit-identical): the advisor's slope law a' = (1 - c)/h^p with BOTH
        % constants as states and no shift -- slot 5 = c (plaw_c_slope's level constant), slot 7 = p (the exponent, in delta's slot),
        % h_hat = w_d - dw3_hat - w_s. Jacobians: d a'/d c = -1/h^p, d a'/d p = -a' ln h, d a'/d dw3_hat = +p a'/h; exact step
        % (1-c)(h0^(1-p) - h1^(1-p))/(p-1). Seed sensitivities from the value law a = 1 - (1-c) h^(1-p)/(p-1): d a/d c = h^(1-p)/(p-1),
        % d a/d p = (1-c) h^(1-p)(1/(p-1) + ln h)/(p-1) (sympy check_estb_f1_slope_cp.py). Seeds c_init / p_init = the plane's band pair
        % (fit_two_param_height_walls.m F1: +0.096, 1.894), priors Pf_c_std / Pf_p_std; guard p_hat in plaw_p_lim (default [1.2, 4]).
        % Reported through the existing slots: diag.b_hat = c_hat (P_b = sqrt(P55)), diag.ws_hat = p_hat (P_ws = sqrt(P77)).
        plaw_cp_slope = logical(get_field_default(ctrl_const, 'plaw_cp_slope', false));
        if plaw_cp_slope
            assert(~plaw_dc_slope && ~plaw_dp_slope, 'motion_control_law_formC_b:plawCpSlope', ...
                   'ctrl_const.plaw_cp_slope excludes plaw_dc_slope / plaw_dp_slope (slot 7 is p here, not delta).');
            plaw_c_slope = true;                                       % all plaw_c_slope branches apply (slot 5 = c, plain gap h_hat)
            plaw_p_lim   = get_field_default(ctrl_const, 'plaw_p_lim', [1.2, 4]);
            assert(numel(plaw_p_lim) == 2 && plaw_p_lim(1) > 1 && plaw_p_lim(1) < plaw_p_lim(2), 'motion_control_law_formC_b:plawCpSlope', ...
                   'ctrl_const.plaw_p_lim must be an increasing pair above 1 (the exact step needs p > 1).');
            pcp_seed   = expand3(get_field_default(ctrl_const, 'p_init', 2));
            Pf_pcp_std = expand3(get_field_default(ctrl_const, 'Pf_p_std', 0));
            plaw_seed_cross = logical(get_field_default(ctrl_const, 'plaw_seed_cross', true));
            assert(all(pcp_seed > plaw_p_lim(1) & pcp_seed < plaw_p_lim(2)), 'motion_control_law_formC_b:plawCpSlope', 'ctrl_const.p_init must lie inside plaw_p_lim.');
        end
        % par_wall_known (10-07, Layer-1; default false => bit-identical): on the wall-PARALLEL axes (AX_PAR) the slope law and c
        % are perpendicular-axis objects. Moving parallel to the wall does not change the gain, and the axis's own tracking error is
        % not a wall-normal step. With the flag, axes 1-2 propagate the gain by the plane's parallel law a'_par(h) times the
        % commanded wall-normal step only (h = w_d - w_s, no own dw3), carry no c column, no thermal gain noise (Q row/col 4 = 0),
        % and seed a_hat[0] = a_par,plane(h0). Found by the raster scan (x 320 um/s at 1.10 R): without it a_hat_x hits the
        % floor/ceiling because the x tracking error (~1 um) times the perpendicular slope (~0.8) is fed as a gain step.
        par_wall_known = logical(get_field_default(ctrl_const, 'par_wall_known', false));
        if par_wall_known
            assert(plaw_c_slope, 'motion_control_law_formC_b:parWallKnown', 'ctrl_const.par_wall_known is wired for the plaw_c_slope family only.');
        end
        if plaw_c_slope
            assert(~gain_algebraic && ~amodel_fe && ~amodel_h, 'motion_control_law_formC_b:plawCSlope', ...
                   'ctrl_const.plaw_c_slope keeps the a slot: gain_algebraic, amodel_fe and amodel_h must be off.');
            assert(~c0_state && ~pade22 && ~delta_state && ~plaw_c_state && ~b1_state && ~da_slot && ~has_b_true && ~has_ap_known, 'motion_control_law_formC_b:plawCSlope', ...
                   'ctrl_const.plaw_c_slope is the pure a-slot form: c0_state, pade22, delta_state, plaw_c_state, b1_state, da_slot, b_true and ap_known must be off.');
            assert(~law_exact_step && ~pred_mean2, 'motion_control_law_formC_b:plawCSlope', ...
                   'ctrl_const.plaw_c_slope has its own exact height step: law_exact_step and pred_mean2 (b-law terms) must be off.');
            assert(strcmp(ap_src, 'post'), 'motion_control_law_formC_b:plawCSlope', 'ctrl_const.plaw_c_slope sets the slope itself: leave ap_src at its default.');
            plaw_p   = get_field_default(ctrl_const, 'plaw_p', 2);
            assert(plaw_p > 1, 'motion_control_law_formC_b:plawCSlope', 'ctrl_const.plaw_p must exceed 1.');
            if plaw_dc_slope && ~plaw_dp_slope
                assert(plaw_p == 2, 'motion_control_law_formC_b:plawDcSlope', 'ctrl_const.plaw_dc_slope pins p = 2 (far-field anchor (9/8)/h^2).');
            end
            c_seed   = expand3(get_field_default(ctrl_const, 'c_init', 0));
            Pf_c_std = expand3(get_field_default(ctrl_const, 'Pf_c_std', 0));
            ws_11    = get_field_default(ctrl_const, 'ws0_perp', 0);
            if plaw_dp_slope                                           % slot 5 = p: its seed and prior replace c's (the variables are the slot-5 seed / prior)
                c_seed   = expand3(get_field_default(ctrl_const, 'p_init', 2));
                Pf_c_std = expand3(get_field_default(ctrl_const, 'Pf_p_std', 0));
                assert(all(c_seed > plaw_p_lim(1) & c_seed < plaw_p_lim(2)), 'motion_control_law_formC_b:plawDpSlope', 'ctrl_const.p_init must lie inside plaw_p_lim.');
            end
        end
        lock_mask_g  = [lock_da || delta_state || plaw_c_state; ~(da_slot || b1_state); ~(c0_state || delta_state || plaw_c_state || plaw_dc_slope || plaw_cp_slope)];   % slot order 5, 6, 7 (7 = p when plaw_cp_slope) (6 = da when da_slot, b1 when b1_state; 7 = c0 when c0_state, delta when delta_state, c when plaw_c_state, delta when plaw_dc_slope; 5 locked under delta_state / plaw_c_state; 5 = c FREE under plaw_c_slope)
        lock_mask_ax = repmat(lock_mask_g, 1, 3);
        if par_law
            lock_mask_ax(1, AX_PAR) = true;           % x/y carry no disturbance
        end
        if b1_state; lock_mask_ax(2, :) = lock_mask_ax(1, :); end   % b1 is free exactly where b is
        lock_state_idx_ax = cell(3, 1);
        for ax = 1:3
            lock_state_idx_ax{ax} = 4 + find(lock_mask_ax(:, ax));   % 5..7
            if gain_algebraic                                        % the gain slot is a derived output, not a state
                lock_state_idx_ax{ax} = [4; lock_state_idx_ax{ax}(:)];
            end
        end

        % --- 0D. Validity clamps (numerical guards only, not tuning) ---
        a_bar_floor = get_field_default(ctrl_const, 'a_bar_floor', 0.05);
        % a_bar_ceil keeps 1 - a_bar > 0, which the law REQUIRES (a_bar' =
        % (1-a_bar)^2 and the whole family lives on a_bar < 1). The law has an
        % attractor at a_bar = 1 (a_bar' -> 0), so the continuous flow never
        % crosses; a forward-Euler overshoot needs (1-a)*dw > 1, i.e. dw of
        % order 2 per step against ~0.0043 in the canonical run (~500x
        % margin), and the additive da would have to be O(1) per step against
        % a prior width of order 1e-4. Cheap insurance, never binds.
        a_bar_ceil  = get_field_default(ctrl_const, 'a_bar_ceil', 1 - 1e-4);
        % da is an INCREMENT of a_bar per step and a_bar lives in (0,1), so no
        % admissible value can exceed 1 in magnitude. That structural bound is
        % the clamp; it is not a tuning knob and never binds (the run-time
        % values are O(1e-4)).
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
        % da_init = 0 is the tex seed (S9): the closed-form S3(b) mismatch is
        % deliberately NOT fed in, so any non-zero da_hat[inf] is an
        % ACCEPTANCE rather than an input.
        w0_nominal = get_field_default(ctrl_const, 'ws0_perp', 1) - 1;   % nominal wall
        % b seed and prior. Both are the MIDPOINT and HALF-RANGE of b_true
        % over the planned envelope, computed OFFLINE by the driver -- the
        % controller never reads c(w). Midpoint rather than the far-field 9/8
        % because b_true does not straddle 9/8 on this envelope: it sits above
        % it everywhere, so 9/8 is the lower edge, not a centre. Using the
        % half-range also keeps the shape-acceptance test non-vacuous, which a
        % prior defined AS that same sup could never be.
        seed_b   = expand3(get_field_default(ctrl_const, 'b_init', 9/8));
        Pf_b_std = expand3(get_field_default(ctrl_const, 'Pf_b_std', 0));
        law_err_db = expand3(get_field_default(ctrl_const, 'law_err_db', Pf_b_std));   % see law_err_q44 in 0D
        if law_err_q44
            assert(all(law_err_db > 0), 'motion_control_law_formC_b:lawErrQ44', ...
                   'ctrl_const.law_err_q44 needs delta_b > 0 (it defaults to Pf_b_std, the b prior envelope).');
        end
        b_floor  = get_field_default(ctrl_const, 'b_floor', 0.60);
        b_ceil   = get_field_default(ctrl_const, 'b_ceil',  1.05);
        % --- b PER HEIGHT BIN (2026-09-09, user-approved; observability C51: 4 bins PASS on canon) ---
        % b is a property of the WALL AT THIS HEIGHT, not of the run: b_true runs 8/9 far field -> ~0.93 near the
        % wall -> 1 at contact (the two published anchors already say b is not constant). One constant state
        % averages all of that into a single number, so it is wrong at both ends. Here slot 5 carries the b OF THE
        % BIN THE PROBE IS IN: on a bin change the current (b_hat, P55) is stored back and the new bin's pair is
        % loaded, with the cross-covariances zeroed (a priori the new bin's b is uncorrelated with the current
        % dw/a_w states -- the standard map approximation; it DISCARDS information rather than inventing it).
        % The bin index comes from the COMMANDED height, which is deterministic and known: no self-loop, no gate.
        % Q55 stays 0 inside a bin: within one bin b IS a constant, so nothing here re-opens the forgetting family
        % (R56/R57/R58). Edges are supplied by the caller (driver default: the seed line's own contact belief plus
        % 0.5 R steps); there is no house default, and b_bins_on without edges is an error.
        b_bins_on   = logical(get_field_default(ctrl_const, 'b_bins_on', false));
        b_bin_edges = get_field_default(ctrl_const, 'b_bin_edges', []);
        if b_bins_on
            assert(~isempty(b_bin_edges) && isvector(b_bin_edges) && issorted(b_bin_edges(:)), ...
                   'motion_control_law_formC_b:bBins', ...
                   'b_bins_on requires ctrl_const.b_bin_edges: an ascending vector of inner bin edges in R (no house default).');
            b_bin_edges = b_bin_edges(:).';
        end
        % --- P55 FLOOR (2026-09-09, directional forgetting in its steady-state form; literature O28) ---
        % A uniform forgetting factor inflates P55 every step, including the hold and far-field steps that carry no
        % information about b, and winds up (Meng: sqrt P55 0.039 -> 0.29). The directional form (Kulhavy-Karny 1984,
        % Cao-Schwartz 2000) inflates only along the excited direction; for the scalar slot 5 its steady state is a
        % floor on P55 reached only where b is being learned. Written as the floor directly: after the update, if
        % P55 < p55_floor_std^2, scale row/column 5 back up to the floor (congruence, P stays PSD). Zero new numbers:
        % the default floor is the family prior Pf_b_std, i.e. the estimator is never allowed to be more certain about b
        % than the within-band variation of b_true (rule 4: sup|theta_eff - theta_0| <~ sqrt P, applied continuously).
        % Does nothing when no information arrives (P55 does not shrink there), so no windup. Default OFF.
        p55_floor_on  = logical(get_field_default(ctrl_const, 'p55_floor_on', false));
        p55_floor_std = expand3(get_field_default(ctrl_const, 'p55_floor_std', Pf_b_std));
        assert(~(p55_floor_on && (lambda_f_b < 1 || lfb_alpha > 0)), 'motion_control_law_formC_b:p55Floor', ...
               'p55_floor_on cannot be combined with lambda_f_b / lambda_f_b_alpha (two forgetting devices on one slot).');

        % --- 0G. P0 widths ---
        % Pf_da_std  : prior on the additive disturbance, the tex's ONLY
        %              undetermined number. Fallback scale = (level offset it
        %              must remove) / (steps over which it removes it) =
        %              Pf_a_floor * Ts / T_REMOVAL_S; the driver, which knows
        %              the planned trajectory, overrides it with the closed-form
        %              S3(b) sup. Contract = invariance (sweep 10x, converged
        %              behaviour must not move).
        % Pf_w0_std  : the wall-position prior, carried over unchanged from
        %              the height writing. There is no w_s state to hold it --
        %              it enters P(4,4) once, through (d a_bar / d w0) = -a_bar'.
        T_REMOVAL_S = 1;      % [s] order of a manoeuvre; fallback only
        Pf_a_floor_pre = get_field_default(ctrl_const, 'Pf_a_floor', 0.0066);
        % (08-25 audit had a DEAD-KNOB error here refusing ctrl_const.Pf_da_std. Since 2026-09-07 the driver always passes it
        %  -- the S3(b) sup -- and it acts only when ctrl_const.da_slot frees slot 6 as da; without da_slot it is ignored.)
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
            b_law = seed_b(ax);
            % b_law must be set on EVERY axis, and the SEED LEVEL must use the
            % SAME b as the state and the slope. Splitting them is the defect
            % found on 2026-08-13 in the sibling: the level is built with one b
            % while slot 5 carries another, and the first y2 update simply
            % reconciles b to whatever the level actually used -- a 10-sigma
            % jump in one step that looks like convergence.
            b_law = seed_b(ax);
            if is_par_ax
                w0_law = w0_par;
            else
                w0_law = w0_nominal;
            end
            % Integrated law at the start height. w0 (the nominal wall) enters
            % HERE and nowhere else in the whole controller: it sets the
            % integration constant, which is what a_bar_hat[0] IS. Note the
            % integrated law does NOT involve da: the disturbance is a per-step
            % increment of the state equation, not a reshaping of the curve.
            [a_bar_seed, dA_dw0] = ...
                local_seed_level_formC(w_bar_seed, w0_law, enable_seed, gap_floor, b_law);
            if c0_state                                   % [1/1] at the start height (0916 two-versions tex, Seed and prior)
                g_seed = w_bar_seed - ws_11;
                a_bar_seed = (g_seed + c0_seed(ax)) / max(g_seed + d1_seed(ax), gap_floor);
                if pade22                                 % [2/2] at the start height (0918 tex, Seed and prior)
                    a_bar_seed = local_gain_22(w_bar_seed, ws_11, 0, c0_seed(ax), c1_seed(ax), d1_seed(ax), d2_seed(ax), gap_floor);
                end
                dA_dw0 = 0;                               % no wall-position container in P44: slot 4 is a derived output
            end
            if delta_state                                % offset law at the start height (0920 tex, Seed and prior)
                a_bar_seed = local_gain_delta(w_bar_seed, ws_11, 0, delta_seed(ax));
                dA_dw0 = 0;                               % the position-type container is slot 7 itself
            end
            if plaw_c_state                               % advisor power law at the start height (plane init: c = 0)
                a_bar_seed = local_gain_plaw(w_bar_seed, ws_11, 0, c_seed(ax), plaw_p);
                dA_dw0 = 0;                               % no wall-position container: slot 4 is a derived output, slot 7 = c is shape-type
            end
            if plaw_c_slope                               % version 2: the plane polynomial at the start height is the only value law used (0921 tex, Seed)
                [a_bar_seed, ap_seed_v2] = local_law_delta(w_bar_seed - ws_11);
                dA_dw0 = -ap_seed_v2;                     % d a/d w_s = -a': the production wall-position container stays available (Pf_w0_std)
                if par_wall_known && any(ax == AX_PAR)    % parallel axes: the plane's PARALLEL gain at the start height
                    a_bar_seed = local_par_plane(w_bar_seed - ws_11);  dA_dw0 = 0;
                end
            end
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
            % Seed prior (tex seed section). The only level sensitivity is
            % the wall position, d a_bar/d w0 = -a_bar' (exact); the wall prior
            % has no state of its own, so it lands in P(4,4) once and then
            % propagates by the F_e(4,4) dynamics.
            %     P44[0] = (a_bar'[0])^2 * Pf_w0_std^2 + Pf_a_floor^2
            %     P45[0] = 0     <- structural, not a modelling choice: at
            %                       k = 0 nothing has accumulated, and the
            %                       integrated law that sets a_bar_hat[0] does
            %                       not contain da at all, so the seed error
            %                       and the disturbance are independent BY
            %                       CONSTRUCTION (contrast the multiplicative
            %                       sibling, where dA/dda is nonzero).
            %     P55[0] = Pf_da_std^2
            % b enters the LAW, so dA/db = -1/(w-w_s) = -(1-a_bar)/b is
            % NONZERO and the seed carries a genuine cross term. (Contrast the
            % additive sibling, where P45[0] = 0 was structural because da did
            % not appear in the integrated law.)
            dA_db = +(1 - a_bar_seed) / seed_b(ax);
            P7(4, 4) = dA_dw0^2 * Pf_w0_std(ax)^2 ...
                     + free_da * dA_db^2 * Pf_b_std(ax)^2 ...
                     + Pf_a_floor(ax)^2;
            P7(4, 5) = free_da * dA_db * Pf_b_std(ax)^2;
            P7(5, 4) = P7(4, 5);
            P7(5, 5) = free_da * Pf_b_std(ax)^2;
            if c0_state                                   % version A priors: diag (c0, d1), slot 4 locked at 0 (0916 two-versions tex)
                assert(~b_bins_on && ~p55_floor_on, 'motion_control_law_formC_b:c0State', ...
                       'ctrl_const.c0_state is the pure 5-state form: b_bins_on and p55_floor_on must be off.');
                P7(4, 4) = 0;  P7(4, 5) = 0;  P7(5, 4) = 0;
                P7(5, 5) = free_da * Pf_d1_std(ax)^2;
                P7(7, 7) = Pf_c0_std(ax)^2;
                if pade22; P7(8, 8) = Pf_c1_std(ax)^2;  P7(9, 9) = Pf_d2_std(ax)^2; end   % [2/2]: diag priors on c1, d2
            end
            if delta_state                                % offset law priors: slot 7 = delta only; slots 4 (derived) and 5 (locked) at 0 (0920 tex)
                assert(~b_bins_on && ~p55_floor_on, 'motion_control_law_formC_b:deltaState', ...
                       'ctrl_const.delta_state is the pure 4-state offset law: b_bins_on and p55_floor_on must be off.');
                P7(4, 4) = 0;  P7(4, 5) = 0;  P7(5, 4) = 0;  P7(5, 5) = 0;
                P7(7, 7) = Pf_delta_std(ax)^2;
            end
            if plaw_c_state                               % advisor power law priors: slot 7 = c only; slots 4 (derived) and 5 (locked) at 0
                assert(~b_bins_on && ~p55_floor_on, 'motion_control_law_formC_b:plawCState', ...
                       'ctrl_const.plaw_c_state is the pure 4-state power law: b_bins_on and p55_floor_on must be off.');
                P7(4, 4) = 0;  P7(4, 5) = 0;  P7(5, 4) = 0;  P7(5, 5) = 0;
                P7(7, 7) = Pf_c_std(ax)^2;
            end
            if plaw_c_slope                               % version 2 priors: slot 5 = c, no seed cross term (c is not in the value), P44 as production
                assert(~b_bins_on && ~p55_floor_on, 'motion_control_law_formC_b:plawCSlope', ...
                       'ctrl_const.plaw_c_slope: b_bins_on and p55_floor_on (slot-5 = b containers) must be off.');
                P7(4, 4) = dA_dw0^2 * Pf_w0_std(ax)^2 + Pf_a_floor(ax)^2;
                P7(4, 5) = 0;  P7(5, 4) = 0;
                P7(5, 5) = double(~lock_mask_ax(1, ax)) * Pf_c_std(ax)^2;
                if plaw_dc_slope                          % 0922 tex, Seed: the start value was read from the plane; the wall's (delta, c) move it by
                    g0_dc  = max(w_bar_seed - ws_11 + delta_seed(ax), plaw_g_min);   % d a/d delta = a'(g0), d a/d c = 1/g0 (value law, used once here)
                    ap0_dc = (1 - c_seed(ax)) / g0_dc^2;  da5_dc = 1 / g0_dc;
                    if plaw_dp_slope                      % 0923 tex, Seed: slot 5 = p; d a/d delta = a' = 1/g0^p, d a/d p = (1/(p-1) + ln g0)/((p-1) g0^(p-1))
                        p0 = c_seed(ax);  ap0_dc = 1 / g0_dc^p0;  da5_dc = (1 / (p0 - 1) + log(g0_dc)) / ((p0 - 1) * g0_dc^(p0 - 1));
                    end
                    P7(4, 4) = P7(4, 4) + ap0_dc^2 * Pf_delta_std(ax)^2 + da5_dc^2 * P7(5, 5);
                    P7(4, 5) = da5_dc * P7(5, 5);  P7(5, 4) = P7(4, 5);
                    P7(7, 7) = Pf_delta_std(ax)^2;
                    P7(4, 7) = ap0_dc * P7(7, 7);  P7(7, 4) = P7(4, 7);
                    if ~plaw_seed_cross; P7(4, 5) = 0;  P7(5, 4) = 0;  P7(4, 7) = 0;  P7(7, 4) = 0; end   % O40 discriminator: level uncertainty kept, its attribution to the constants removed
                end
                if plaw_cp_slope                          % (c, p) both states, no shift: value law a = 1 - (1-c) h^(1-p)/(p-1) at the seed height (used once here)
                    h0_cp = max(w_bar_seed - ws_11, 1);  p0_cp = pcp_seed(ax);  c0_cp = c_seed(ax);
                    da5_cp = h0_cp^(1 - p0_cp) / (p0_cp - 1);                                                   % d a/d c
                    da7_cp = (1 - c0_cp) * h0_cp^(1 - p0_cp) * (1 / (p0_cp - 1) + log(h0_cp)) / (p0_cp - 1);     % d a/d p
                    P7(4, 4) = P7(4, 4) + da5_cp^2 * P7(5, 5) + da7_cp^2 * Pf_pcp_std(ax)^2;
                    P7(4, 5) = da5_cp * P7(5, 5);  P7(5, 4) = P7(4, 5);
                    P7(7, 7) = Pf_pcp_std(ax)^2;
                    P7(4, 7) = da7_cp * P7(7, 7);  P7(7, 4) = P7(4, 7);
                    if ~plaw_seed_cross; P7(4, 5) = 0;  P7(5, 4) = 0;  P7(4, 7) = 0;  P7(7, 4) = 0; end
                end
                if par_wall_known && any(ax == AX_PAR)    % parallel axes: the seed does not depend on c or p (known parallel law)
                    P7(4, 4) = Pf_a_floor(ax)^2;  P7(4, [5 7]) = 0;  P7([5 7], 4) = 0;
                end
            end
            if b_bins_on && ax == 1
                nb = numel(b_bin_edges) + 1;                       % every bin starts at the SAME seed pair (b_init, Pf_b_std)
                bbin_b = repmat(seed_b(:), 1, nb);  bbin_P = repmat(Pf_b_std(:).^2, 1, nb);
                bbin_cur = nan(3, 1);  bbin_switch = zeros(3, 1);
            end
            P7(6, 6) = double(da_slot) * Pf_da_std(ax)^2;   % slot 6 = da when da_slot, else exact zero (inert); slot 7 inert
            if b1_state && ~lock_mask_ax(2, ax)             % slot 6 = b1 (0914 tex, Seed and prior): b1[0] = 0
                P7(6, 6) = Pf_b1_std(ax)^2;
                L_seed = w_bar_seed - (w0_law + 1 / b_law); % start height - the seed line's zero-gain height
                if b1_seed_couple && isfinite(L_seed)
                    P7(5, 6) = 0.5 * L_seed * P7(6, 6);  P7(6, 5) = P7(5, 6);
                    P7(5, 5) = P7(5, 5) + 0.25 * L_seed^2 * P7(6, 6);
                end
            end
            P_aa_v(ax)  = P7(4, 4);
            P_bb0_v(ax) = P7(5, 5);
            P_pp0_v(ax) = P7(6, 6);
            P_ws0_v(ax) = dA_dw0^2 * Pf_w0_std(ax)^2;   % wall share of P_aa
            if c0_state || delta_state || plaw_c_state || plaw_dc_slope || plaw_cp_slope; P_ws0_v(ax) = P7(7, 7); end     % c0_state / delta_state / plaw_c_state / plaw_dc_slope: slot 7 is c0 / delta / c / delta, report its own prior in the init row

            % chol PD check on the free-state submatrix (locked rows/cols are
            % exact zeros by construction, so the full matrix is only PSD).
            free_idx = [1:4, 5 * ones(1, free_da), 6 * ones(1, double(da_slot || P7(6, 6) > 0))];
            if c0_state; free_idx = [1:3, 5 * ones(1, free_da), 7]; end   % slot 4 is locked (derived output), slot 7 = c0 is free
            if delta_state || plaw_c_state; free_idx = [1:3, 7]; end        % offset / power law: slots 4 (derived) and 5 (locked) out, slot 7 = delta / c free
            if plaw_dc_slope; free_idx = [free_idx, 7]; end                 % two-constant slope law: slot 7 = delta free as well
            if plaw_cp_slope; free_idx = [free_idx, 7]; end                 % (c, p) slope law: slot 7 = p free as well
            if ma2_aug || pade22
                free_idx = [free_idx, 8, 9];
            end
            [~, chol_flag] = chol(P7(free_idx, free_idx));
            if chol_flag ~= 0
                error('motion_control_law_formC_b:P0NotPD', ...
                      'P0 free-state block is not positive definite (axis %d).', ax);
            end

            P_per_axis{ax} = P7;
            x_e_per_axis(4, ax) = a_bar_seed;
            x_e_per_axis(5, ax) = b_law;
            x_e_per_axis(6, ax) = 0;      % inert
            x_e_per_axis(7, ax) = w0_law; % inert reporter (seed origin only)
            if c0_state                   % version A: slot 5 = d1, slot 7 = c0, both estimated
                x_e_per_axis(5, ax) = d1_seed(ax);
                x_e_per_axis(7, ax) = c0_seed(ax);
                if pade22; x_e_per_axis(8, ax) = c1_seed(ax);  x_e_per_axis(9, ax) = d2_seed(ax); end   % [2/2]: slots 8 = c1, 9 = d2
            end
            if delta_state; x_e_per_axis(7, ax) = delta_seed(ax); end   % offset law: slot 7 = delta (slot 5 keeps b_law, locked and inert)
            if plaw_c_state; x_e_per_axis(7, ax) = c_seed(ax); end      % power law: slot 7 = c (slot 5 keeps b_law, locked and inert)
            if plaw_c_slope; x_e_per_axis(5, ax) = c_seed(ax); end      % version 2: slot 5 = c
            if plaw_dc_slope; x_e_per_axis(7, ax) = delta_seed(ax); end % two-constant slope law: slot 7 = delta
            if plaw_cp_slope; x_e_per_axis(7, ax) = pcp_seed(ax); end   % (c, p) slope law: slot 7 = p
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
        lam_b_v = ones(3, 1);        % this step's slot-5 forgetting factor per axis (1 = none), logged as diag.lam_b_k
        q55a_buf = zeros(q55a_N, 3);  q55a_bufI = zeros(q55a_N, 3);  q55a_cmD = zeros(3, 1);  q55a_cmR = zeros(3, 1);  q55a_hmin = inf(3, 1);  q55a_ptr = 0;  q55a_g = zeros(2, 3);  q55a_stat = zeros(3, 1);  q55a_q = zeros(3, 1);   % q55_adapt buffers, see init
        q55a_I = zeros(3, 1);         % q55_adapt ratio modes: I_{k-1}, this step's information on slot 5 (10-02: kept apart from the innovation statistic)
        q55a_P0  = P_bb0_v;           % q55_adapt 'ratio': the slot-5 prior variance P55[0] per axis
        if q55a_P0s > 0; q55a_P0 = q55a_P0s^2 * ones(3, 1); end   % override: P_0 = q55_adapt_P0^2
        a_ctrl_km1 = a_bar_seed_v;
        a_ctrl_km2 = a_bar_seed_v;
        Delta_wbar_d_km1 = 0;
        lam_b_spend = zeros(3, 1);   % running total of (1 - lambda), the budget
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
            diag.a_hat_nd  = a_bar_seed_v * a_o;                  % [U4] legacy 1/pN (do NOT mix with a_disp)
            diag.a_bar_hat = a_bar_seed_v;
            % report the ACTUAL seeded slots (x/y hold the parallel constants
            % under par_law, the requested seeds otherwise)
            diag.b_hat     = x_e_per_axis(5, :).';
            diag.p_hat     = x_e_per_axis(6, :).';
            diag.ws_hat    = x_e_per_axis(7, :).';
            if pade22; diag.c1_hat = x_e_per_axis(8, :).';  diag.d2_hat = x_e_per_axis(9, :).'; end   % [2/2] reporters
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
            diag.Pf_theta_std = [Pf_b_std, Pf_w0_std, zeros(3, 1)];
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
            diag.predict_quad_used = predict_quad;      % Stage B: logged once
            if has_ap_known && ~strcmp(predict_quad, 'left')
                diag.predict_quad_used = [predict_quad ' (ap_known arm: left fallback)'];
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
        error('motion_control_law_formC_b:unsupportedDelay', ...
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
    if b1_state
        assert(~da_slot && ~b_bins_on && ~p55_floor_on && ~amodel_fe && ~jac_exact_step && ~pred_force_step && ~q55_path ...
               && ~has_ap_known && ~has_b_true && strcmp(predict_quad, 'left') && strcmp(ap_src, 'post') && lambda_f >= 1, ...
               'motion_control_law_formC_b:b1State', ...
               ['ctrl_const.b1_state is derived on the production recipe with the slope read at the posterior gain; ' ...
                'da_slot, b_bins_on, p55_floor_on, amodel_*, jac_exact_step, pred_force_step, q55_path, ap_known, b_true, ' ...
                'predict_quad ~= left, ap_src ~= post and lambda_f < 1 are not supported with it.']);
    end
    I7 = eye(n_state);
    t_now = (k_step - 1) * Ts;

    K_a_y2_v   = zeros(3, 1);
    K_dx_y1_v  = zeros(3, 1);
    K_dx_y2_v  = zeros(3, 1);   % L32: y2's correction to the CURRENT tracking error slot
    K_b_y1_v   = zeros(3, 1);   % L51: b's correction from y1
    K_b_y2_v   = zeros(3, 1);   % L52: b's correction from y2
    K_a_y1_v   = zeros(3, 1);   % K1(4): y1's GAIN correction (via P(4,1))
    P41_v      = zeros(3, 1);   % the cross-covariance that creates it
    dws_y1_v   = zeros(3, 1);   % logging only: ws update via y1, K1(7)*innov1
    dws_y2_v   = zeros(3, 1);   % logging only: ws update via y2, K2(7)*innov2
    innov_y2_v = zeros(3, 1);
    innov_y1_v = zeros(3, 1);   % logging only (whiteness diagnostic)
    S1_v       = nan(3, 1);     % logging only: innovation covariance of y1 (2026-09-08, wall-hypothesis likelihood)
    S2_v       = nan(3, 1);     % logging only: innovation covariance of y2 (NaN when the y2 gate is closed)
    gate_off   = false(3, 1);
    G_flags    = false(3, 3);
    a_prime_v  = zeros(3, 1);
    pred_mean2_v = zeros(3, 1);
    lam_b_v    = ones(3, 1);    % adaptive slot-5 forgetting, per axis (diag)
    Q44_v      = zeros(3, 1);
    Q33_v      = zeros(3, 1);
    a_barQ_v   = zeros(3, 1);
    R2_v       = zeros(3, 1);

    for ax = 1:3
        % Reset every step: if the y2 gate closes, these must NOT carry the
        % previous step's values into the adaptive forgetting law.
        innov2 = NaN;  S2 = NaN;
        x_curr = x_e_per_axis(:, ax);
        P_curr = P_per_axis{ax};
        if b_bins_on && ~lock_mask_ax(1, ax) && isfinite(w_bar_d)
            j_new = 1 + sum(w_bar_d >= b_bin_edges);               % bin from the COMMANDED height (deterministic)
            if isnan(bbin_cur(ax))
                bbin_cur(ax) = j_new;                              % first step: the seed belongs to this bin
            elseif j_new ~= bbin_cur(ax)
                bbin_b(ax, bbin_cur(ax)) = x_curr(5);  bbin_P(ax, bbin_cur(ax)) = P_curr(5, 5);   % store what this bin learnt
                x_curr(5) = bbin_b(ax, j_new);
                P_curr(5, :) = 0;  P_curr(:, 5) = 0;               % new bin's b: a priori uncorrelated with the other states
                P_curr(5, 5) = bbin_P(ax, j_new);
                bbin_cur(ax) = j_new;  bbin_switch(ax) = bbin_switch(ax) + 1;
            end
        end
        a_bar_i = min(max(x_curr(4), a_bar_floor), a_bar_ceil);
        % SLOT 5 IS b. Read the CURRENT state -- reading x_e_per_axis here
        % would take the seed every step and the law would never see the
        % estimate move, which is a silent failure: the run completes and the
        % numbers look plausible.
        b_hat_i = min(max(x_curr(5), b_floor), b_ceil);
        kap_i = NaN;
        if c0_state                                    % slot 5 = d1, slot 7 = c0: the law's slope scale is 1/(d1 - c0), no b clamp
            kap_i   = max(x_curr(5) - x_curr(7), gap_floor);
            b_hat_i = 1 / kap_i;
        end
        if has_b_true && b_true(ax) > 0
            b_hat_i = b_true(ax);          % TRUE-b ARM: the law reads the plant's b
        end
        b1_free = b1_state && ~lock_mask_ax(2, ax);   % b1_state: slot 6 = b1 (0914 tex)
        b1_hat_i = 0;
        if b1_free; b1_hat_i = x_curr(6); end

        % --- Gain rate and Jacobians from the STATE (tex S1/S2):
        %       a_bar'      = b (1 - a_bar)^2
        %       d a_bar'/da = -2 b (1 - a_bar)                         (< 0)
        %       d a_bar'/db = +a_bar' / b = (1 - a_bar)^2              (> 0)
        %     Both b Jacobians (F_e(4,5) and H(2,5)) carry a displacement
        %     factor, so they vanish together in a hold.
        lm = lock_mask_ax(:, ax);
        if isempty(a_bar_slope_v); a_bar_slope_v = x_e_per_axis(4, :).'; end
        switch ap_src
            case 'post';  a_bar_sl = a_bar_i;
            case 'pred';  a_bar_sl = min(max(a_bar_slope_v(ax), a_bar_floor), a_bar_ceil);
            case 'ewma'
                a_bar_slope_v(ax) = (1 - ap_ewma_a) * a_bar_slope_v(ax) + ap_ewma_a * a_bar_i;
                a_bar_sl = min(max(a_bar_slope_v(ax), a_bar_floor), a_bar_ceil);
            case {'cmd', 'act'}
                % HEIGHT WRITING (formC_height_b.tex). The slope is read at a
                % HEIGHT, not at the gain state:
                %     a_bar(u) = 1 - 1/(b u) ,   a_bar'(u) = 1/(b u^2)
                % 'cmd' takes u at the commanded height; 'act' also subtracts the
                % tracking-error estimate, so u is where the filter believes the
                % particle ACTUALLY is. Both are realizable only because the wall
                % origin w0 is known -- that premise is what these cases test.
                %
                % Neither reads the gain state, so the self-loop the three
                % state-reading cases carry (gain wrong -> slope wrong -> gain
                % more wrong; 2.24 pp even on a plant that IS the estimator's own
                % law) is absent, and A_a = 0 is the derivative, not a
                % simplification. 'act' does read a state, dw_3, but that is the
                % best-observed one in the filter -- y1 measures it directly at
                % unit gain -- rather than the worst.
                w0_k = w0_nominal;
                if par_law && any(ax == AX_PAR); w0_k = w0_par; end
                u_k = w_bar_d - w0_k;
                if strcmp(ap_src, 'act'); u_k = u_k - x_curr(3); end
                u_k   = max(u_k, gap_floor / max(b_hat_i, eps));
                gap_k = max(b_hat_i * u_k, gap_floor);
                a_bar_sl = min(max(1 - 1/gap_k, a_bar_floor), a_bar_ceil);
        end
        % Jacobian is taken AT the evaluation point. Zeroing A_a instead was a
        % defect (2026-08-13): A_a*M is what grows P44 along the descent, so
        % removing it made the filter over-confident and confounded the test
        % with a covariance-propagation change. The lag is one step (or a short
        % EWMA) against a 1 Hz trajectory, so treating it as negligible for the
        % Jacobian is the standard approximation and keeps P intact.
        % b is STATE 5 here, not a constant: a_bar' = (1-a_bar)^2 / b_hat.
        [a_prime_i, A_a_i] = local_gain_law_formC(a_bar_sl, enable_wall, b_hat_i);
        % d(a_bar')/db and d(a_bar')/d(dw_3), SIGNED per writing. Both feed the
        % columns below through one convention -- F_e(4,5) = dap_db*M and
        % H(2,5) = -Grad*dap_db, likewise for the dw_3 pair -- so carrying the
        % signed derivative rather than a magnitude is what lets the two writings
        % share this code instead of keeping a second set of columns in step.
        %
        %   state  writing   a_bar' = b (1-a_bar)^2   =>   d/db = +a_bar'/b
        %   height writing   a_bar' = 1/(b u^2)       =>   d/db = -a_bar'/b
        %
        % The sign genuinely flips. It is a reparameterisation: b multiplies the
        % slope in one writing and divides it in the other (formC_height_b.tex).
        dap_db_i  = +a_prime_i / b_hat_i;
        dap_dd3_i = 0;
        dap_ddl_i = 0;                                 % d a'/d delta (slot 7), nonzero only under plaw_dc_slope
        if any(strcmp(ap_src, {'cmd','act'}))
            dap_db_i = -a_prime_i / b_hat_i;
            if strcmp(ap_src, 'act')
                % d(a_bar')/d(dw_3) = +2 a_bar'/u. u = w_d - w0 - dw_3, so a
                % LARGER tracking error is a LOWER height and a STEEPER slope.
                dap_dd3_i = 2 * a_prime_i / u_k;
            end
            A_a_i = 0;   % the derivative, not an approximation
        end
        if has_ap_known
            a_prime_i = ap_known(ax);   % exogenous integrand
            A_a_i     = 0;              % no state dependence left
        end
        if plaw_c_slope                 % version 2 (0921 tex S1): the slope is read at the estimator's own height, slot 5 = c
            h_v2 = max(w_bar_d - x_curr(3) - ws_11, 1);
            if plaw_dc_slope            % 0922 tex S1: the argument is the shifted gap g = h + delta (slot 7), floored numerically
                h_v2 = max(w_bar_d - x_curr(3) - ws_11 + x_curr(7), plaw_g_min);
            end
            a_prime_i = (1 - x_curr(5)) / h_v2^plaw_p;
            A_a_i     = 0;                              % d a'/d a_hat = 0: the slope does not read the gain slot
            dap_db_i  = -1 / h_v2^plaw_p;               % d a'/d c  (slot 5 is c here)
            dap_dd3_i = plaw_p * a_prime_i / h_v2;      % d a'/d dw3_hat = +p a'/h (h = w_d - dw3_hat - w_s)
            if plaw_dc_slope; dap_ddl_i = -plaw_p * a_prime_i / h_v2; end   % d a'/d delta = d a'/d g = -p a'/g = -(d a'/d dw3_hat)
            if plaw_dp_slope                            % 0923 tex S1: slot 5 = p, contact slope pinned at 1
                p_v2      = x_curr(5);
                a_prime_i = 1 / h_v2^p_v2;
                dap_db_i  = -a_prime_i * log(h_v2);     % d a'/d p = -a' ln g
                dap_dd3_i = p_v2 * a_prime_i / h_v2;    % d a'/d dw3_hat = +p a'/g
                dap_ddl_i = -p_v2 * a_prime_i / h_v2;   % d a'/d delta   = -p a'/g
            end
            if plaw_cp_slope                            % (c, p) both states, plain gap h_hat: slot 5 = c, slot 7 = p
                p_v2      = x_curr(7);
                a_prime_i = (1 - x_curr(5)) / h_v2^p_v2;
                dap_db_i  = -1 / h_v2^p_v2;             % d a'/d c
                dap_dd3_i = p_v2 * a_prime_i / h_v2;    % d a'/d dw3_hat = +p a'/h
                dap_ddl_i = -a_prime_i * log(h_v2);     % d a'/d p (slot 7) = -a' ln h
            end
            is_par_wk = par_wall_known && any(ax == AX_PAR);
            if is_par_wk                                % parallel axis: plane parallel slope at the commanded height, no c, no own dw3
                h_v2 = max(w_bar_d - ws_11, 1);
                [~, a_prime_i] = local_par_plane(h_v2);
                dap_db_i = 0;  dap_dd3_i = 0;  dap_ddl_i = 0;
            end
        end
        % F_e(4,5). d(a_bar')/db = -a_bar'/b exactly, so the column is
        % -a_bar' * M / b_hat, where M is the SAME row-4 increment a_bar'
        % multiplies in predict. It is PROPORTIONAL TO DISPLACEMENT, so b is
        % unobservable through this path in a hold -- the structural cost of
        % replacing the additive disturbance (whose column was a constant 1).
        % Built after M is known; the scalar factor is stored here.
        dap_dc0_i = 0;
        if c0_state                                    % 0916 two-versions tex section 1: d a'/d d1 = -(1-a)^2/(d1-c0)^2, d a'/d c0 = +(1-a)^2/(d1-c0)^2
            dap_db_i  = -a_prime_i / kap_i;            % slot 5 is d1 here
            dap_dc0_i = +a_prime_i / kap_i;
        end
        dap_dc1_i = 0;  dap_dd2_i = 0;  app22_i = 0;  dA22_i = zeros(1, 4);
        if pade22                                      % [2/2] (0918 tex S1): slope, curvature and ALL Jacobians at the estimator's own gap.
            % The four d a'/d theta are TOTAL derivatives at fixed g (sympy-checked, check_estb_22_jacobians.py), so the
            % A_a * (d a/d theta) share the [1/1] path adds through H2(4) = 1 - Grad A_a must not be added again: A_a = 0
            % here and the delayed-height curvature term -Grad a'' enters H2(3) explicitly below.
            g22 = w_bar_d - x_curr(3) - ws_11;
            [a_prime_i, app22_i, dA22_i, dAp22_i] = local_law_22(g22, x_curr(7), x_curr(8), x_curr(5), x_curr(9), gap_floor);
            A_a_i = 0;
            dap_dc0_i = dAp22_i(1);  dap_dc1_i = dAp22_i(2);  dap_db_i = dAp22_i(3);  dap_dd2_i = dAp22_i(4);
        end
        app_dl_i = 0;
        if delta_state                                 % offset law (0920 tex S1): slope and curvature at the estimator's own effective gap
            % g = (w_hat - w_s) + delta_hat; a' = -u^2 D'(u), a'' = u^4 D'' + 2 u^3 D', u = 1/g. d a'/d delta = a'' exactly, so
            % A_a = 0 is the derivative (not an approximation) and the delayed-height curvature enters H2(3) explicitly below.
            g_dl = w_bar_d - x_curr(3) - ws_11 + x_curr(7);
            sig_dl = 0;  if delta_slin; sig_dl = sqrt(max(P_curr(7, 7), 0)); end
            [~, a_prime_i, app_dl_i] = local_law_delta_exp(g_dl, sig_dl);   % point values when sig_dl = 0
            A_a_i = 0;  dap_db_i = 0;  dap_dc0_i = app_dl_i;   % slot 5 locked; slot 7 = delta
        end
        app_pl_i = 0;  dac_pl_i = 0;
        if plaw_c_state                                % advisor power law: slope, curvature and the c Jacobians at the estimator's own height
            % h = w_hat - w_s; a' = (1-c)/h^p, a'' = -p(1-c)/h^(p+1), d a/d c = (1-a)/(1-c), d a'/d c = -1/h^p. A_a = 0 is the derivative
            % (the law reads the height, not the gain slot); the delayed-height curvature enters H2(3) explicitly below, as for delta_state.
            h_pl = w_bar_d - x_curr(3) - ws_11;
            [~, a_prime_i, app_pl_i, dac_pl_i, dapc_pl_i] = local_law_plaw(h_pl, x_curr(7), plaw_p);
            A_a_i = 0;  dap_db_i = 0;  dap_dc0_i = dapc_pl_i;   % slot 5 locked; slot 7 = c
        end
        J_b_fac_i = dap_db_i * double(~lm(1));
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
            if b1_free; gT(5) = b1_hat_i; gn(5) = b1_hat_i * alpha_ma2; end   % b1_state: b moves with the step
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
            if b1_free                                % b1_state: g = [-1, a', b1] on slots 3-5
                Q_i(3, 5) = -b1_hat_i * Q33;              Q_i(5, 3) = Q_i(3, 5);
                Q_i(4, 5) = a_prime_i * b1_hat_i * Q33;   Q_i(5, 4) = Q_i(4, 5);
                Q_i(5, 5) = b1_hat_i^2 * Q33;
            end
        end
        for j = 1:3
            if ~lm(j)
                Q_i(4 + j, 4 + j) = Q_i(4 + j, 4 + j) + Q_theta_floor;   % conditioning only, default 0 (added: b1_state puts b1^2 Q33 in slot 5)
            end
        end
        if par_wall_known && any(ax == AX_PAR); Q_i(4, :) = 0;  Q_i(:, 4) = 0; end   % parallel axis: the thermal kick does not move the gain
        Q44_v(ax) = Q_i(4, 4);
        Q33_v(ax) = Q_i(3, 3);
        a_barQ_v(ax) = a_bar_i;      % the a_bar actually used to build Q33

        % --- R (R2 at the ESTIMATE a_bar_hat, never the raw readout) ---
        R1_i = sigma2_n_nd(ax);
        R2_i = compute_R2_formB(a_bar_i, sigma2_n_nd(ax), IF_abc, C_dpmr, C_n, ...
                                K_var, amlpf_var_factor, xi_bar(ax), kappa_T, ...
                                Q_i(4, 4), d_delay, a_cov, y2_whiten, r22_delay_scale);
        R2_i = R2_i * r2_scale;                  % diagnostic whole-R2 scale (default 1)
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
        if par_wall_known && any(ax == AX_PAR); M_tot = Delta_wbar_d_km1; end   % parallel axis: only the wall-normal commanded step changes the gain
        F_dw = F_dw_km1(ax);
        if t2_pure_prop; F_dw = 0; Q_i = zeros(n_state); end
        if q55_path && ~lm(1); Q_i(5, 5) = Q_i(5, 5) + q55_per_R * abs(M_tot); end   % path-proportional container of the b'_true dw line, see init
        q55a_q(ax) = 0;
        if ~strcmp(q55a_mode, 'off') && ~lm(1)                                % q55_adapt (see init): statistic of the PREVIOUS steps' y1 residual
            st = q55a_stat(ax);  qadd = 0;
            switch q55a_mode
                case 'wmean'; if abs(st) > q55a_thr; qadd = q55a_gain * abs(M_tot); end
                case 'cusum'; if st > q55a_thr;      qadd = q55a_gain * abs(M_tot); end
                case 'prop';  qadd = q55a_gain * st^2 * abs(M_tot);
                case 'nis';   qadd = q55a_gain * max(0, st - 1) * abs(M_tot);
                case 'ratio'; qadd = q55a_P0(ax)^2 * q55a_I(ax);               % Q55 = R_k * rho_k with rho_k = (P_0 / R_k)^2, R_k = 1 / I_k (I_{k-1} from the last update)
                case 'rhoconst'; qadd = q55a_rho / max(q55a_I(ax), q55a_Imin); % constant ratio: Q55 = R_k * rho = rho / I_{k-1} (floored)
                case 'ratiogate'; qadd = q55a_P0(ax)^2 * q55a_I(ax) * double(abs(st) > q55a_thr);   % 10-02: ratio, opened only while the windowed mean of the whitened y1 innovation exceeds thr (model-error evidence)
                case 'r2ratio';  qadd = q55a_rho * R2_i;  q55a_stat(ax) = R2_i;  % literal R2 ratio (09-30 user question): Q55 = rho * R2[k], no H (R2 in a-bar^2 units)
                case 'ratioinnov'                                                  % 10-04: ratio scaled by g(x), x = (mean nu1 / sigma)^2, see init
                    if q55a_wI; xr = st^2; else; xr = st^2 * q55a_N; end           %   wI: st is already mean/sigma (I-weighted); else sigma^2 = 1/N
                    if strcmp(q55a_gfun, 'max1'); xr = max(1, xr); end
                    qadd = q55a_P0(ax)^2 * q55a_I(ax) * xr;
                case 'rationew'                                                    % 10-04: P_fix by new ground / revisit, see init
                    assert(plaw_c_slope, 'motion_control_law_formC_b:q55AdaptNew', 'q55_adapt rationew needs a plaw slope law (ws_11).');
                    hd_new = w_bar_d - ws_11;  Pn = q55a_P0(ax);
                    if hd_new < q55a_hmin(ax) - 1e-4; q55a_hmin(ax) = hd_new;          % new ground: short memory
                    elseif q55a_Prev > 0; Pn = q55a_Prev^2; end                       % revisit: long memory
                    qadd = Pn^2 * q55a_I(ax);
                case 'covmatch'                                                    % 10-04: covariance matching, see init
                    if strcmp(q55a_cmf, 'pure'); qadd = q55a_cmD(ax);
                    else; rho_cm = 1; if q55a_cmR(ax) > 0; rho_cm = q55a_cmD(ax) / q55a_cmR(ax); end
                        qadd = q55a_P0(ax)^2 * q55a_I(ax) * rho_cm; end
                case 'ratiodrift'                                                  % 10-04: ratio + height drift dc^2 |M| / h_hat^2, see init
                    assert(plaw_c_slope, 'motion_control_law_formC_b:q55AdaptDrift', 'q55_adapt ratiodrift needs a plaw slope law (h_v2).');
                    qadd = q55a_P0(ax)^2 * q55a_I(ax) + q55a_dc^2 * abs(Delta_wbar_d_km1) / h_v2^2;   % commanded step only: deterministic, 0 in a hold (M_tot's dw3 jitter would not cancel under |.|)
                case 'drift'                                                       % 10-06: O50 alone, see init
                    assert(plaw_c_slope, 'motion_control_law_formC_b:q55AdaptDrift', 'q55_adapt drift needs a plaw slope law (h_v2).');
                    qadd = q55a_dc^2 * abs(Delta_wbar_d_km1) / h_v2^2;
                case 'driftsq'                                                     % 10-06: deterministic step change squared, see init
                    assert(plaw_c_slope, 'motion_control_law_formC_b:q55AdaptDrift', 'q55_adapt driftsq needs a plaw slope law (h_v2).');
                    qadd = (q55a_dc * Delta_wbar_d_km1 / h_v2^2)^2;
            end
            if ~isinf(q55a_hgate) && plaw_c_slope && h_v2 > q55a_hgate; qadd = 0; end   % height gate (oscillation test): no Q above hgate
            Q_i(5, 5) = Q_i(5, 5) + qadd;  q55a_q(ax) = qadd;
        end
        F_e = local_build_F_e_formC(lambda_c, F_dw, a_prime_i, A_a_i, ...
                                    J_b_fac_i * M_tot, M_tot, dap_dd3_i);
        if plaw_dc_slope || plaw_cp_slope; F_e(4, 7) = dap_ddl_i * M_tot; end   % 0922 tex S5: F_e(4,7) = -(2 a'/g) M = -(height share of F_e(4,3)); (c, p) law: -(a' ln h) M
        if fe43_off; F_e(4, 3) = 0; end          % diagnostic, see init
        if fe44_Aa_scale ~= 1; F_e(4, 4) = F_e(4, 4) - (1 - fe44_Aa_scale) * A_a_i * M_tot; end   % see init: kappa-weighted self-sensitivity in P (kappa = 0 == fe44_Aa_off)
        if da_slot; F_e(4, 6) = 1; end                 % E2: additive disturbance enters a_bar at unit gain every step
        if b1_free                                     % b1_state (0914 tex, production blocks): F_e(5,:) = e5' + b1 c' + dw_hat e6'
            F_e(5, 3) = one_minus_lc * b1_hat_i;
            F_e(5, 4) = F_dw * b1_hat_i;
            F_e(5, 6) = M_tot;
        end
        cc1_i = 0;  cd2_i = 0;                         % [2/2] level Jacobians of c1, d2 (zero unless pade22)
        if amodel_fe                                   % green variant, see init (0911 tex S4): F_dw e_4 -> F_dw psi in rows 3-4
            assert(~has_ap_known && ~has_b_true, 'motion_control_law_formC_b:amodelFe', ...
                   'ctrl_const.amodel_fe needs the law slope with b as the estimated constant (no ap_known / b_true arm).');
            cb_i = (1 - a_bar_sl) / b_hat_i * double(~lm(1));   % (d a/d b) at fixed w_s; e_b = 0 when slot 5 is locked
            cc_i = 0;
            if c0_state                                    % (c0, d1) writing (0916 two-versions tex S4): d a/d d1 = -a(1-a)/(d1-c0), d a/d c0 = (1-a)/(d1-c0)
                cb_i = -a_bar_sl * (1 - a_bar_sl) / kap_i * double(~lm(1));
                cc_i = (1 - a_bar_sl) / kap_i;
            end
            if pade22                                      % [2/2] (0918 tex S4): d a/d (c0, c1, d1, d2) = (1, g, -a g, -a)/D
                cc_i = dA22_i(1);  cc1_i = dA22_i(2);  cb_i = dA22_i(3) * double(~lm(1));  cd2_i = dA22_i(4);
            end
            if delta_state                                 % offset law (0920 tex S4): d a/d delta = a' (slot 7), slot 5 locked
                cc_i = a_prime_i;  cb_i = 0;
            end
            if plaw_c_state                                % power law: d a/d c = (1-a)/(1-c) (slot 7), slot 5 locked
                cc_i = dac_pl_i;  cb_i = 0;
            end
            F_e(3, 3) = F_e(3, 3) + F_dw * a_prime_i;
            F_e(3, 4) = F_e(3, 4) + F_dw;                  % -F_dw -> 0: the step error no longer carries e_4
            F_e(3, 5) = F_e(3, 5) - F_dw * cb_i;
            F_e(3, 7) = F_e(3, 7) - F_dw * cc_i;           % c0 column (zero unless c0_state)
            F_e(4, 3) = F_e(4, 3) - F_dw * a_prime_i^2;
            F_e(4, 4) = F_e(4, 4) - a_prime_i * F_dw;
            F_e(4, 5) = F_e(4, 5) + F_dw * a_prime_i * cb_i;
        end

        % --- EKF predict (tex S5(b)). Row 4 carries the ADDITIVE disturbance
        %     x_curr(5) outside the a_bar' bracket; row 5 is an integrator of
        %     nothing, da[k+1] = da[k].
        % Row-4 increment under ctrl_const.predict_quad (Stage B, see init).
        % 'left' keeps the production expression VERBATIM -- the M_row4 product
        % here and the MA(2) share added below are two separate products, so
        % the default stays bit-identical. 'mid'/'trap' form the bracket the
        % predict actually multiplies (M_pred; fe_row4_full does not enter the
        % predict, only F_e) and re-read the law at the half/end point with the
        % same a_bar clamps as the production slope call. The MA(2) share is
        % then already inside the increment and is NOT added again below.
        quad_left = strcmp(predict_quad, 'left') || has_ap_known;
        if quad_left
            x4_pred = x_curr(4) + a_prime_i * (Delta_wbar_d_km1 + one_minus_lc * x_curr(3));
        else
            M_pred = Delta_wbar_d_km1 + one_minus_lc * x_curr(3);
            if ma2_aug; M_pred = M_pred + alpha_ma2 * (x_curr(8) + x_curr(9)); end
            switch predict_quad
                case 'mid'
                    a_half = min(max(a_bar_sl + 0.5 * a_prime_i * M_pred, a_bar_floor), a_bar_ceil);
                    ap_q   = local_gain_law_formC(a_half, enable_wall, b_hat_i);
                case 'trap'
                    a_end  = min(max(a_bar_sl + a_prime_i * M_pred, a_bar_floor), a_bar_ceil);
                    ap_end = local_gain_law_formC(a_end, enable_wall, b_hat_i);
                    ap_q   = 0.5 * (a_prime_i + ap_end);
            end
            x4_pred = x_curr(4) + ap_q * M_pred;
        end
        if da_slot; x4_pred = x4_pred + x_curr(6); end   % E2: + da per step, outside the a_bar' bracket
        x_pred = [x_curr(2); ...
                  x_curr(3); ...
                  lambda_c * x_curr(3); ...
                  x4_pred; ...
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
            if quad_left; x_pred(4) = x_pred(4) + a_prime_i * alpha_ma2 * m_sum; end
            x_pred = [x_pred; 0; x_curr(8)];

            F_aug = zeros(n_state);
            F_aug(1:7, 1:7) = F_e;
            F_aug(3, 8) = -alpha_ma2;
            F_aug(3, 9) = -alpha_ma2;
            F_aug(4, 8) = a_prime_i * alpha_ma2;
            F_aug(4, 9) = a_prime_i * alpha_ma2;
            F_aug(9, 8) = 1;            % row 8 stays all-zero (m1 <- pure noise)
            if b1_free; F_aug(5, 8) = b1_hat_i * alpha_ma2; F_aug(5, 9) = b1_hat_i * alpha_ma2; end   % b1_state: row 5 = e5' + b1 c'
            F_e = F_aug;
        elseif pade22                                % [2/2]: slots 8 = c1, 9 = d2 are constants; row 3 gets their level columns (0918 tex S4)
            x_pred = [x_pred; x_curr(8); x_curr(9)];
            F_aug = zeros(n_state);
            F_aug(1:7, 1:7) = F_e;
            F_aug(8, 8) = 1;  F_aug(9, 9) = 1;
            F_aug(3, 8) = -F_dw * cc1_i;
            F_aug(3, 9) = -F_dw * cd2_i;
            F_e = F_aug;
        end
        if nw_mcorr                                  % correlated-noise input gn (y1 - x_upd(1)) of the previous call, see init
            x_pred(3) = x_pred(3) - one_minus_lc * res1_km1(ax);
            if quad_left; x_pred(4) = x_pred(4) + a_prime_i * one_minus_lc * res1_km1(ax); end
        end
        if b1_free                                   % b1_state: b moves with the full step (the bracket the gain row multiplies)
            M_b1 = Delta_wbar_d_km1 + one_minus_lc * x_curr(3);
            if ma2_aug; M_b1 = M_b1 + alpha_ma2 * (x_curr(8) + x_curr(9)); end
            if nw_mcorr; M_b1 = M_b1 + one_minus_lc * res1_km1(ax); end
            x_pred(5) = x_curr(5) + b1_hat_i * M_b1;
        end
        if law_exact_step                            % exact law step, see init
            M_pred = Delta_wbar_d_km1 + one_minus_lc * x_curr(3);
            if ma2_aug; M_pred = M_pred + alpha_ma2 * (x_curr(8) + x_curr(9)); end
            if nw_mcorr; M_pred = M_pred + one_minus_lc * res1_km1(ax); end   % the estimated noise reaction is a known part of the step
            if pred_force_step; M_pred = a_ctrl_km1(ax) * fbar_d_km1(ax); end   % commanded displacement = gain the control law USED x its force (a_ctrl at k-1 = posterior of k-2; x_curr(4) would carry the k-1 innovation, endogenous)
            if has_ap_known
                % Exogenous slope: the SAME exact step, with the local law
                % constant recovered from the fed slope,
                %     b_eff = a'_ext/(1-A)^2
                %     => A[k+1] = A + a'_ext M / (1 + a'_ext M/(1-A)).
                % No new information enters (b_eff is a reparameterisation of
                % the slope already supplied); reduces identically to the
                % production branch when a'_ext = b_hat (1-A)^2. Closes the
                % 2a5dc29 exclusion that left the oracle arm on forward Euler.
                ap_state = a_prime_i;                % mean-only knob, see init
                if mean_knob_from > 0 && k_step >= mean_knob_from; ap_state = a_prime_i * pred_slope_scale; end
                den = 1 + ap_state * M_pred / (1 - x_curr(4));
                if den > 0
                    x_pred(4) = x_curr(4) + ap_state * M_pred / den;
                end                                  % else: keep the Euler step (edge only)
            else
                u_inv = 1 / (1 - x_curr(4)) + b_hat_i * M_pred;
                if b1_free; u_inv = u_inv + 0.5 * b1_hat_i * M_pred^2; end   % b1_state: 1/(1-a) = integral of b(w) dw
                if u_inv > 0
                    x_pred(4) = 1 - 1 / u_inv;
                end                                  % else: keep the Euler step (A -> 1 edge only)
            end
        end
        if plaw_c_slope                              % version 2: exact integral of the height law over the step (0921 tex S1, S4)
            M_pred = Delta_wbar_d_km1 + one_minus_lc * x_curr(3);
            if ma2_aug; M_pred = M_pred + alpha_ma2 * (x_curr(8) + x_curr(9)); end
            if nw_mcorr; M_pred = M_pred + one_minus_lc * res1_km1(ax); end
            h0_v2 = max(w_bar_d - x_curr(3) - ws_11, 1);  h1_v2 = max(h0_v2 + M_pred, 1);
            if plaw_dc_slope                         % 0922 tex S4: the same integral over the shifted gap g = h + delta, p = 2: (1-c)(1/g0 - 1/g1)
                h0_v2 = max(w_bar_d - x_curr(3) - ws_11 + x_curr(7), plaw_g_min);  h1_v2 = max(h0_v2 + M_pred, plaw_g_min);
            end
            x_pred(4) = x_curr(4) + (1 - x_curr(5)) * (h0_v2^(1 - plaw_p) - h1_v2^(1 - plaw_p)) / (plaw_p - 1);   % = a + int a' dh; first order a + a' M
            if plaw_dp_slope                         % 0923 tex S4: exponent is the state, level 1: (g0^(1-p) - g1^(1-p))/(p-1)
                p_v2 = x_curr(5);
                x_pred(4) = x_curr(4) + (h0_v2^(1 - p_v2) - h1_v2^(1 - p_v2)) / (p_v2 - 1);
            end
            if plaw_cp_slope                         % (c, p) law: both from the state, plain gap: (1-c)(h0^(1-p) - h1^(1-p))/(p-1)
                p_v2 = x_curr(7);
                x_pred(4) = x_curr(4) + (1 - x_curr(5)) * (h0_v2^(1 - p_v2) - h1_v2^(1 - p_v2)) / (p_v2 - 1);
            end
        end
        if pred_mean2                                % second-order mean term, see init
            app_law = -2 * a_prime_i^2 / (1 - x_curr(4));
            if b1_free; app_law = app_law + b1_hat_i * (1 - a_bar_sl)^2; end   % b1_state: a'' = b1 (1-a)^2 - 2 b^2 (1-a)^3
            if has_app_known; app_i = app_known(ax); else; app_i = app_law; end
            Pc = P_curr;
            if ma2_aug
                cov_e3u = one_minus_lc * Pc(3, 3) + alpha_ma2 * (Pc(3, 8) + Pc(3, 9)) + F_dw * Pc(3, 4);
                var_u   = one_minus_lc^2 * Pc(3, 3) ...
                        + alpha_ma2^2 * (Pc(8, 8) + Pc(9, 9) + 2 * Pc(8, 9)) ...
                        + 2 * one_minus_lc * alpha_ma2 * (Pc(3, 8) + Pc(3, 9)) ...
                        + F_dw^2 * Pc(4, 4) ...
                        + 2 * F_dw * (one_minus_lc * Pc(3, 4) + alpha_ma2 * (Pc(4, 8) + Pc(4, 9))) ...
                        + s2T_i + alpha_ma2^2 * s2n_i;
            else
                cov_e3u = one_minus_lc * Pc(3, 3) + F_dw * Pc(3, 4);
                var_u   = one_minus_lc^2 * Pc(3, 3) + F_dw^2 * Pc(4, 4) + 2 * one_minus_lc * F_dw * Pc(3, 4) + Q33;
            end
            cov_e4u = one_minus_lc * Pc(3, 4) + F_dw * Pc(4, 4);            % Cov(e4, u), pred_mean2_e4 only
            cov_e5u = one_minus_lc * Pc(3, 5) + F_dw * Pc(4, 5);            % Cov(e_b, u) under the filter's constant-b model, pred_mean2_e4 with slot 5 free (S11)
            if ma2_aug
                cov_e4u = cov_e4u + alpha_ma2 * (Pc(4, 8) + Pc(4, 9));
                cov_e5u = cov_e5u + alpha_ma2 * (Pc(5, 8) + Pc(5, 9));
            end
            if pred_force_step                       % unknown increment = w_T + fbar_d e_a, see init
                cov_e3u = fbar_d_km1(ax) * Pc(3, 4);
                cov_e4u = fbar_d_km1(ax) * Pc(4, 4);
                cov_e5u = fbar_d_km1(ax) * Pc(4, 5);
                var_u   = fbar_d_km1(ax)^2 * Pc(4, 4) + kappa_T * a_bar_i;
            end
            if nw_mcorr                              % u = (1-lc)(e3 + m-errors - e1) + F_dw e4 + w_T: the n_w share is replaced by -(1-lc) e1
                if ma2_aug
                    cov_e3u = cov_e3u - one_minus_lc * Pc(3, 1);
                    var_u   = var_u - alpha_ma2^2 * s2n_i ...
                            + one_minus_lc^2 * Pc(1, 1) - 2 * one_minus_lc^2 * (Pc(1, 3) + Pc(1, 8) + Pc(1, 9)) ...
                            - 2 * F_dw * one_minus_lc * Pc(1, 4);
                else
                    cov_e3u = cov_e3u - one_minus_lc * Pc(3, 1);
                    var_u   = var_u - one_minus_lc^2 * sigma2_n_nd(ax) ...
                            + one_minus_lc^2 * Pc(1, 1) - 2 * one_minus_lc^2 * Pc(1, 3) - 2 * F_dw * one_minus_lc * Pc(1, 4);
                end
                cov_e4u = cov_e4u - one_minus_lc * Pc(1, 4);
                cov_e5u = cov_e5u - one_minus_lc * Pc(1, 5);
            end
            if amodel_fe                                 % green step error u = c'e + w_T, see init (0911 tex S5-S7)
                cvec = zeros(n_state, 1);
                cvec(3) = one_minus_lc - F_dw * a_prime_i;
                cvec(5) = F_dw * cb_i;
                if nw_mcorr; cvec(1) = -one_minus_lc; end
                if ma2_aug
                    cvec(8) = alpha_ma2;  cvec(9) = alpha_ma2;
                    wvar = s2T_i + alpha_ma2^2 * s2n_i * double(~nw_mcorr);
                else
                    wvar = Q33 - one_minus_lc^2 * sigma2_n_nd(ax) * double(nw_mcorr);
                end
                Pcv     = Pc * cvec;
                cov_e3u = Pcv(3);
                cov_e5u = Pcv(5);
                var_u   = cvec.' * Pcv + wvar;
                cov_e4u = -a_prime_i * cov_e3u + cb_i * cov_e5u;   % Cov(psi, u): e_4 -> psi in the start-point term
            end
            if pred_mean2_e4 && ~has_ap_known            % gain-reading start-point term A_a Cov(e4,u) [+ (da'/db) Cov(e_b,u), slot 5 free], see init
                mean2_i = A_a_i * cov_e4u + 0.5 * app_i * var_u + 0.5 * (app_i - app_law) * M_pred^2;
                if ~lm(1) && ~has_b_true; mean2_i = mean2_i + dap_db_i * cov_e5u; end   % S11 e_b line; P(:,5) = 0 when slot 5 is locked anyway
            else
                mean2_i = -app_i * cov_e3u + 0.5 * app_i * var_u + 0.5 * (app_i - app_law) * M_pred^2;
            end
            if pred_mean2_kr1_full                  % complete feedthrough term, see init
                mean2_i = mean2_i + app_i * one_minus_lc * K31_km1(ax) * sigma2_n_nd(ax);
            elseif pred_mean2_kr1                   % A2 closed form (dw3_hat share only), see init
                mean2_i = mean2_i + app_i * one_minus_lc^2 * K31_km1(ax) * sigma2_n_nd(ax);
            end
            x_pred(4) = x_pred(4) + mean2_i;
            pred_mean2_v(ax) = mean2_i;
            if b1_free                               % b1_state: second-order mean of the b row, Cov(e_6, u) (0914 tex, production blocks)
                cov_e6u = one_minus_lc * Pc(3, 6) + F_dw * Pc(4, 6);
                if ma2_aug; cov_e6u = cov_e6u + alpha_ma2 * (Pc(6, 8) + Pc(6, 9)); end
                if nw_mcorr; cov_e6u = cov_e6u - one_minus_lc * Pc(1, 6); end
                x_pred(5) = x_pred(5) + cov_e6u;
            end
        end
        if q44_scale ~= 1; Q_i(4, 4) = q44_scale * Q_i(4, 4); end   % diagnostic, see init
        if q34_off                                   % diagnostic, see init
            Q_i(3, 4) = 0;  Q_i(4, 3) = 0;
            if n_state > 7
                Q_i(4, 8) = 0;  Q_i(8, 4) = 0;
                F_e(4, 8) = 0;  F_e(4, 9) = 0;
            end
        end
        if nw_mcorr                                  % F - (M/R1) H1 and Q - M M'/R1, see init
            gn_c = zeros(n_state, 1);  gn_c(3) = -one_minus_lc;  gn_c(4) = a_prime_i * one_minus_lc;
            if b1_free; gn_c(5) = b1_hat_i * one_minus_lc; end   % b1_state
            F_e(:, 1) = F_e(:, 1) - gn_c;
            Q_i = Q_i - sigma2_n_nd(ax) * (gn_c * gn_c.');
        end
        if law_err_q44                               % the law's own error, see init (0915 tex S7): same direction as
            M_law = M_tot;                           % F_e(4,5) e_b but an independent draw, so slot 4 only
            if nw_mcorr; M_law = M_law + one_minus_lc * res1_km1(ax); end
            Q_i(4, 4) = Q_i(4, 4) + (dap_db_i * law_err_db(ax) * M_law)^2;
        end
        if jac_exact_step && ~has_ap_known               % consistent row-4 Jacobian of the exact law step, see init
            M_jac = M_tot;
            if nw_mcorr; M_jac = M_jac + one_minus_lc * res1_km1(ax); end
            if pred_force_step; M_jac = a_ctrl_km1(ax) * fbar_d_km1(ax); end
            den_jac = 1 + b_hat_i * (1 - x_curr(4)) * M_jac;
            if den_jac > 0
                D_jac = 1 / den_jac^2;                   % = (1 - a_pred)^2 / (1 - a)^2 for the exact step actually taken
            else
                D_jac = 1 + A_a_i * M_jac;               % the predict fell back to Euler (edge only): keep its Jacobian
            end
            F_e(4, :) = D_jac * F_e(4, :);               % every off-diagonal row-4 entry is proportional to a'(a)
            F_e(4, 4) = D_jac * (1 + a_prime_i * F_dw);  % diagonal: D (1 + a' F_dw) replaces 1 + a' F_dw + A_a M
            Q_i(4, :) = D_jac * Q_i(4, :);               % noise enters the gain row through a'(a_pred) = D a'(a)
            Q_i(:, 4) = D_jac * Q_i(:, 4);
            if da_slot; F_e(4, 6) = 1; end               % da enters outside the law: not scaled
        end
        if gain_algebraic                            % 4-state: the gain slot is the law at the PREDICTED height, see init
            w0_ga = w0_nominal;
            if par_law && any(ax == AX_PAR); w0_ga = w0_par; end
            if pade22                                % [2/2] at the predicted height (0918 tex S3)
                x_pred(4) = min(max(local_gain_22(w_bar_d, ws_11, x_pred(3), x_pred(7), x_pred(8), x_pred(5), x_pred(9), gap_floor), a_bar_floor), a_bar_ceil);
            elseif c0_state                          % version A: [1/1] in (c0, d1) at the predicted height
                x_pred(4) = min(max(local_gain_11(w_bar_d, ws_11, x_pred(3), x_pred(7), x_pred(5), gap_floor), a_bar_floor), a_bar_ceil);
            elseif delta_state                       % offset law at the predicted height (0920 tex S3)
                x_pred(4) = min(max(local_gain_delta(w_bar_d, ws_11, x_pred(3), x_pred(7)), a_bar_floor), a_bar_ceil);
            elseif plaw_c_state                      % advisor power law at the predicted height
                x_pred(4) = min(max(local_gain_plaw(w_bar_d, ws_11, x_pred(3), x_pred(7), plaw_p), a_bar_floor), a_bar_ceil);
            else
                x_pred(4) = min(max(local_gain_from_law(w_bar_d, w0_ga, x_pred(3), x_pred(5), gap_floor), a_bar_floor), a_bar_ceil);
            end
        end
        if par_wall_known && any(ax == AX_PAR)        % parallel axis: gain row = itself only (no own-dw3, no c, no memory columns)
            F_e(4, :) = 0;  F_e(4, 4) = 1;
            x_pred(4) = x_curr(4) + a_prime_i * Delta_wbar_d_km1;   % known parallel law times the wall-normal commanded step
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
        K31_km1(ax) = K1(3);                         % kept for the next call's pred_mean2_kr1 term
        if freeze_gain || y1_gain_off || (y1_gain_off_from > 0 && k_step >= y1_gain_off_from); K1(4:7) = 0; end
        K1(lock_state_idx_ax{ax}) = 0;
        if l51_off; K1(5) = 0; end                     % E1 diagnostic, see init
        innov1 = delta_w_m(ax) - H1 * x_pred;
        innov_y1_v(ax) = innov1;          % logging only (whiteness diagnostic)
        S1_v(ax)       = S1;              % logging only
        if ~any(strcmp(q55a_mode, {'off', 'ratio', 'rhoconst', 'r2ratio'})) && isfinite(S1) && S1 > 0   % ('ratiogate' keeps the wmean statistic)   % q55_adapt: running y1 statistic, consumed at the NEXT predict (see init); 'ratio' reads P instead (after the update)
            nu1 = innov1 / sqrt(S1);
            if ax == 1; q55a_ptr = mod(q55a_ptr, q55a_N) + 1; end
            q55a_buf(q55a_ptr, ax) = nu1;
            switch q55a_mode
                case {'wmean', 'prop', 'ratiogate', 'ratioinnov'}; q55a_stat(ax) = mean(q55a_buf(:, ax));
                case 'nis';             q55a_stat(ax) = mean(q55a_buf(:, ax).^2);
                case 'cusum'
                    q55a_g(1, ax) = max(0, q55a_g(1, ax) + nu1 - q55a_kcu);
                    q55a_g(2, ax) = max(0, q55a_g(2, ax) - nu1 - q55a_kcu);
                    q55a_stat(ax) = max(q55a_g(:, ax));
            end
        end
        x_upd  = x_pred + K1 * innov1;
        if mean_knob_from > 0 && k_step >= mean_knob_from && y1_gain_leg_scale ~= 1
            x_upd(4) = x_upd(4) + (y1_gain_leg_scale - 1) * K1(4) * innov1;   % state only; P below uses the unscaled K1
        end
        ImKH1  = I7 - K1 * H1;
        P_upd  = ImKH1 * P_pred * ImKH1' + K1 * R1_i * K1';   % Joseph form
        P_upd  = 0.5 * (P_upd + P_upd');
        K_dx_y1_v(ax) = K1(3);
        % y1 corrects the GAIN too, through K1 = P*H1'/S1 with H1 = e_1':
        % K1(4) = P(4,1)/S1. The derivation never remarks on this path, but
        % it is the standard KF update -- P(4,1) is built by F_e(3,4) = -F_dw
        % (gain error drives tracking error) and by Q(3,4) = -a_bar'*Q33.
        K_a_y1_v(ax) = K1(4);
        K_b_y1_v(ax)  = K1(5);
        P41_v(ax)    = P_pred(4, 1);
        dws_y1_v(ax)  = K1(7) * innov1;

        % (b) y2 = gain readout (whitened increment by default).
        %     a_wm_km1 is shifted every step regardless, so the increment
        %     stays valid when the gate reopens.
        H2_log = [];
        if ~gate_off(ax) && ~y2_off
            % H row 2 (tex S7, CORRECTED 2026-08-12 -- see the header note).
            % The height writing had dy2/da_bar EXACTLY 1 because a_bar' did
            % not depend on the state; here it does, so the delay back-off
            % carries A_a through. And because S5(b) applies da ONCE PER STEP,
            % walking the gain back d steps must walk back d applications of
            % the disturbance too:
            %     a_bar[k-d] = a_bar[k] - a_bar'*Grad - d*da      (EXACT under
            %                                              da[k+1] = da[k])
            %     dy2/da_bar = 1 - A_a*Grad = 1 + 2(1 - a_bar)*Grad
            %     dy2/d(da)  = -d_delay                (NOT zero; the tex as
            %                  first written said 0, which contradicted its
            %                  own S5(b))
            % The column is small but not absent: |(P H')_5| runs ~1% of the
            % retained P(5,4)*H(2,4) term, so the identification of da is
            % still dominated by the cross-covariance P(4,5) that F_e builds.
            echo_fac = 1;
            if y2_echo_corr
                S_i = (S_echo_T * a_bar_i + S_echo_n * xi_bar(ax)) ...
                      / (a_bar_i + xi_bar(ax));
                echo_fac = 1 - S_i;
            end
            % y2 = a_bar_w - a_bar'*Grad, so every column is -Grad times that
            % state's share of d(a_bar'). Column 3 is nonzero only in 'act',
            % where the slope depends on the tracking-error estimate.
            H2 = H2_scale * echo_fac * [0, 0, ...
                             -Grad_wbar_d * dap_dd3_i, ...
                             1 - Grad_wbar_d * A_a_i, ...
                             -Grad_wbar_d * dap_db_i * double(~lm(1)), ...
                             -d_delay * double(da_slot), 0, zeros(1, n_state - 7)];   % E2: y2 backs off d applications of da
            if plaw_dc_slope || plaw_cp_slope; H2(7) = -H2_scale * echo_fac * Grad_wbar_d * dap_ddl_i; end   % 0922 tex S7: H2(7) = +Grad 2 a'/g = -H2(3); (c, p) law: +Grad a' ln h
            if amodel_h                                  % orange variant, see init (0912 tex S5): e_4 -> psi in the y2 row
                H2(3) = H2(3) - H2(4) * a_prime_i;
                H2(5) = H2(5) + H2(4) * cb_i;
                if c0_state || delta_state || plaw_c_state   % c0 / delta / c column (0916 two-versions tex S7; 0920 tex S7): substitution share + slope walk-back share
                    H2(7) = H2(7) + H2(4) * cc_i - H2_scale * echo_fac * Grad_wbar_d * dap_dc0_i;
                end
                if delta_state                           % offset law: delayed-height curvature -Grad a'' in the dw3 column (A_a = 0 above), so H2(3) = -a'(g_d) to first order
                    H2(3) = H2(3) + H2_scale * echo_fac * Grad_wbar_d * app_dl_i;
                end
                if plaw_c_state                          % power law: the same delayed-height curvature term (A_a = 0 above)
                    H2(3) = H2(3) + H2_scale * echo_fac * Grad_wbar_d * app_pl_i;
                end
                if pade22                                % [2/2] (0918 tex S5): c1, d2 columns and the delayed-height curvature -Grad a'' in the dw3 column
                    H2(8) = H2(4) * cc1_i - H2_scale * echo_fac * Grad_wbar_d * dap_dc1_i;
                    H2(9) = H2(4) * cd2_i - H2_scale * echo_fac * Grad_wbar_d * dap_dd2_i;
                    H2(3) = H2(3) + H2_scale * echo_fac * Grad_wbar_d * app22_i;
                end
                H2(4) = 0;
            end
            if h2_walk_cols_off || y2_delay_off; H2(3) = 0; H2(5) = 0; end   % 10-06 test: drop the walk-back Jacobian columns (dw3, slot 5); y2_pred keeps the walk-back term
            % NONLINEAR predicted measurement (S7 innovation line); H2*x_upd
            % would be wrong here -- see the header. The echo share S of the
            % reading tracks the APPLIED gain (= the estimate), so the
            % prediction keeps the full a_bar_hat term and BOTH back-off
            % terms -- the slope walk-back a_bar'*Grad and the d applications
            % of the disturbance d*da_hat -- scale by (1-S), exactly as their
            % Jacobians do in H2 above.
            if delta_state && delta_iekf
                % Iterated y2 update (O38). Exact reading model at iterate x_i: the law at the estimator's own effective
                % gap g_i = w_d - dw3_i - w_s + delta_i and at the delayed gap g_i - Grad; the echo share (1 - S) scales the
                % walk-back and the sensitivity, exactly as the one-shot branch below does to first order.
                x_minus = x_upd;  P_minus = P_upd;  x_i = x_minus;  H2 = zeros(1, n_state);  K2 = zeros(n_state, 1);
                innov2 = 0;  y2_pred = 0;
                a_app = x_minus(4);                                 % the gain the loop APPLIED this step: fixed during the iterations
                sig_i = 0;  if delta_slin; sig_i = sqrt(max(P_minus(7, 7), 0)); end
                for it = 1:10
                    g_i  = w_bar_d - x_i(3) - ws_11 + x_i(7);
                    [a_di, ap_di]  = local_law_delta_exp(g_i - Grad_wbar_d, sig_i);
                    % reading model: (1 - S) x law at the delayed gap of the iterate + S x the applied gain (echo share
                    % tracks what was applied, not the iterate) -- the first version used the iterate in the echo term and
                    % therefore solved the one-shot's own fixed point (R65).
                    y2_pred = H2_scale * (a_app - echo_fac * (a_app - a_di));
                    h_dl = H2_scale * echo_fac * ap_di;                % d y2_pred / d delta to the same order as the one-shot row
                    H2 = zeros(1, n_state);  H2(3) = -h_dl;  H2(7) = h_dl;
                    S2  = H2 * P_minus * H2' + R2_i;
                    K2  = (P_minus * H2') / S2;
                    if freeze_gain; K2(4:7) = 0; end
                    K2(lock_state_idx_ax{ax}) = 0;
                    innov2 = y2(ax) - y2_pred;
                    x_new  = x_minus + K2 * (innov2 - H2 * (x_minus - x_i));
                    dstep  = abs(x_new(7) - x_i(7));
                    x_i    = x_new;
                    if dstep < 1e-6; break; end
                end
                x_upd  = x_i;
                H2_log = H2;
                ImKH2  = I7 - K2 * H2;
                P_upd  = ImKH2 * P_minus * ImKH2' + K2 * R2_i * K2';   % Joseph form at the last linearisation
                P_upd  = 0.5 * (P_upd + P_upd');
            else
            y2_pred = H2_scale * (x_upd(4) ...
                        - double(~y2_delay_off) * echo_fac * a_prime_i * Grad_wbar_d ...
                        - echo_fac * d_delay * x_upd(6) * double(da_slot));   % E2: d applications of da_hat, scaled like its Jacobian
            H2_log = H2;
            S2  = H2 * P_upd * H2' + R2_i;
            K2  = (P_upd * H2') / S2;
            if freeze_gain; K2(4:7) = 0; end
            K2(lock_state_idx_ax{ax}) = 0;
            innov2 = y2(ax) - y2_pred;
            x_upd  = x_upd + K2 * innov2;
            ImKH2  = I7 - K2 * H2;
            P_upd  = ImKH2 * P_upd * ImKH2' + K2 * R2_i * K2';   % Joseph form
            P_upd  = 0.5 * (P_upd + P_upd');
            end
            K_a_y2_v(ax)   = K2(4);
            K_dx_y2_v(ax)  = K2(3);
            K_b_y2_v(ax)   = K2(5);
            innov_y2_v(ax) = innov2;
            S2_v(ax)       = S2;              % logging only
            dws_y2_v(ax)   = K2(7) * innov2;
        end

        % --- Injection-response hook (diagnostic, default off; see init) ---
        if a_inject_step > 0 && k_step == a_inject_step && ax == a_inject_axis
            x_upd(4) = x_upd(4) * (1 + a_inject_frac);
        end

        if gain_algebraic                            % 4-state: re-evaluate the law at the UPDATED height and b, see init
            w0_ga = w0_nominal;
            if par_law && any(ax == AX_PAR); w0_ga = w0_par; end
            if pade22                                % [2/2] at the updated height; keep the denominator positive at the band floor g = pade22_gmin
                D_min = pade22_gmin^2 + x_upd(5) * pade22_gmin + x_upd(9);
                if D_min < 2 * gap_floor; x_upd(9) = x_upd(9) + (2 * gap_floor - D_min); end
                x_upd(4) = local_gain_22(w_bar_d, ws_11, x_upd(3), x_upd(7), x_upd(8), x_upd(5), x_upd(9), gap_floor);
            elseif c0_state                          % version A: [1/1] in (c0, d1) at the updated height; keep d1 - c0 > 0 first
                x_upd(5) = max(x_upd(5), x_upd(7) + 2 * gap_floor);
                x_upd(4) = local_gain_11(w_bar_d, ws_11, x_upd(3), x_upd(7), x_upd(5), gap_floor);
            elseif delta_state                       % offset law at the updated height; numerical guard g_hat >= 1 + gap_floor (contact of the polynomial)
                x_upd(7) = max(x_upd(7), 1 + gap_floor - (w_bar_d - x_upd(3) - ws_11));
                x_upd(4) = local_gain_delta(w_bar_d, ws_11, x_upd(3), x_upd(7));
            elseif plaw_c_state                      % advisor power law at the updated height; numerical guard 1 - c >= gap_floor (a flat law at c = 1)
                x_upd(7) = min(x_upd(7), 1 - gap_floor);
                x_upd(4) = local_gain_plaw(w_bar_d, ws_11, x_upd(3), x_upd(7), plaw_p);
            else
                x_upd(4) = local_gain_from_law(w_bar_d, w0_ga, x_upd(3), x_upd(5), gap_floor);
            end
        end
        % --- Validity clamps (numerical guards, not tuning). Locked slots
        %     are never touched (K entries zeroed, predict identity), so
        %     they stay exactly at the seed.
        x_upd(4) = min(max(x_upd(4), a_bar_floor), a_bar_ceil);
        if ~lm(1)
            % Slot 5 is b in THIS writing, so the guard is b's own numerical
            % range [b_floor, b_ceil] -- not the sibling's da_clamp, which is
            % the additive disturbance's range. The sibling's [-1, 1] excluded
            % b's own seed (b_init 1.1396 > 1), so b was pinned at the upper
            % bound on step 1 and stayed there for the whole run; the giveaway
            % was x_upd(5) moving while P55 did not, which no Kalman update can
            % do. A guard is a numerical guard only: how far b may travel is
            % P55's job, because P55 records the travel and a clamp does not.
            if plaw_c_slope                           % slot 5 is c: 1 - c >= gap floor (a flat law at c = 1); no physical lower bound, wide numerical one
                if plaw_dp_slope                      % slot 5 = p: the offline search range, p > 1 (the c clamp above would pin p at 1 - gap_floor: bug caught 09-23, p_hat sat at the lower limit from step 1)
                    x_upd(5) = min(max(x_upd(5), plaw_p_lim(1)), plaw_p_lim(2));
                else
                    x_upd(5) = min(max(x_upd(5), -3), 1 - gap_floor);
                end
                if plaw_dc_slope; x_upd(7) = min(max(x_upd(7), plaw_delta_lim(1)), plaw_delta_lim(2)); end   % slot 7 = delta: numerical range of the offline search
                if plaw_cp_slope; x_upd(7) = min(max(x_upd(7), plaw_p_lim(1)), plaw_p_lim(2)); end           % slot 7 = p: the offline search range, p > 1
            elseif ~c0_state                          % slot 5 is d1 under c0_state: its only guard is d1 - c0 > 0 (applied above)
                x_upd(5) = min(max(x_upd(5), b_floor), b_ceil);
            end
        end
        if lambda_f < 1
            P_upd = P_upd / lambda_f;            % Menq (4.15)
            P_upd = 0.5 * (P_upd + P_upd');
        end
        lam_b_k = 1;
        if lambda_f_b < 1
            lam_b_k = lambda_f_b;                       % fixed
        elseif lfb_alpha > 0
            % Adaptive (Meng diss. (6.11) with the NIS excess in place of the raw residual). No measurement on the chosen
            % channel this step means no evidence, so no forgetting -- NOT "forget by default".
            if strcmp(lfb_chan, 'y1'); iv = innov1;  Sv = S1; else; iv = innov2;  Sv = S2; end
            if isfinite(iv) && isfinite(Sv) && Sv > 0
                excess  = max(0, iv^2 / Sv - 1);
                lam_b_k = max(exp(-lfb_alpha * excess), lfb_floor);
                lam_b_spend(ax) = lam_b_spend(ax) + (1 - lam_b_k);
            end
        elseif lfb_info > 0 && ~lm(1) && P_pred(5, 5) > 0
            % Information-driven (directional) forgetting: forget in proportion to what this step's data removed from P55.
            s_k     = max(0, 1 - P_upd(5, 5) / P_pred(5, 5));
            lam_b_k = max(exp(-lfb_info * s_k), lfb_floor);
            lam_b_spend(ax) = lam_b_spend(ax) + (1 - lam_b_k);
        end
        lam_b_v(ax) = lam_b_k;
        if any(strcmp(q55a_mode, {'ratio', 'rhoconst', 'ratiogate', 'ratioinnov', 'ratiodrift', 'covmatch', 'rationew', 'drift', 'driftsq'})) && ~lm(1) && P_pred(5, 5) > 0 && P_upd(5, 5) > 0
            q55a_I(ax) = max(0, 1 / P_upd(5, 5) - 1 / P_pred(5, 5));      % I_k = this step's information on slot 5 (both channels), consumed at the next predict
            if ~any(strcmp(q55a_mode, {'ratiogate', 'ratioinnov'})); q55a_stat(ax) = q55a_I(ax); end
            if strcmp(q55a_mode, 'ratioinnov') && q55a_wI && q55a_ptr > 0         % I-weighted statistic over the same window as nu1 (overrides the plain mean)
                q55a_bufI(q55a_ptr, ax) = q55a_I(ax);  sI2 = sum(q55a_bufI(:, ax).^2);
                if sI2 > 0; q55a_stat(ax) = sum(q55a_bufI(:, ax) .* q55a_buf(:, ax)) / sqrt(sI2); else; q55a_stat(ax) = 0; end
            end   % logged as q55a_stat for ratio / rhoconst (plot_rho_diag); ratiogate logs the gate statistic instead
            if strcmp(q55a_mode, 'covmatch')                                       % EWMA of the observed and the model-predicted correction power on c
                al_cm = 1 / q55a_N;  dc_k = x_upd(5) - x_pred(5);
                q55a_cmD(ax) = (1 - al_cm) * q55a_cmD(ax) + al_cm * dc_k^2;
                q55a_cmR(ax) = (1 - al_cm) * q55a_cmR(ax) + al_cm * max(0, P_pred(5, 5) - P_upd(5, 5));
                if q55a_cmR(ax) > 0; q55a_stat(ax) = q55a_cmD(ax) / q55a_cmR(ax); end   % logged: observed / predicted (1 when the model is right)
            end
        end
        if p55_floor_on && ~lm(1) && P_upd(5, 5) < p55_floor_std(ax)^2 && P_upd(5, 5) > 0
            lam_b_k = P_upd(5, 5) / p55_floor_std(ax)^2;   % < 1: the congruence below lifts P55 exactly to the floor
            lam_b_spend(ax) = lam_b_spend(ax) + (1 - lam_b_k);
        end
        if lam_b_k < 1
            % Slot-5-only forgetting, as a congruence so P stays PSD.
            dv = ones(n_state, 1);  dv(5) = 1 / sqrt(lam_b_k);
            P_upd = (dv * dv.') .* P_upd;
            P_upd = 0.5 * (P_upd + P_upd');
        end
        lam_b_v(ax) = lam_b_k;
        P_upd = freeze_locked_P(P_upd, lock_state_idx_ax{ax});

        if strcmp(ap_src, 'pred'); a_bar_slope_v(ax) = x_pred(4); end
        if obs_dump_on
            % One record per axis per step. H2_log is [] when the y2 gate was
            % closed or y2 is off -- the empty slot is meaningful (that channel
            % did not update), so it is kept rather than dropped.
            obs_dump('append', struct('ax', ax, 'k', k_step, 'F', F_e, ...
                'H', {{H1, H2_log}}, 'R', [R1_i, R2_i], ...
                'x_pred', x_pred, 'x_upd', x_upd, 'P_pred', P_pred, ...
                'P_upd', P_upd, 'gate', gate_off(ax)));
        end

        res1_km1(ax) = delta_w_m(ax) - x_upd(1);    % posterior y1 residual = E[n_w[k] | data], used by nw_mcorr in the next call
        if b_bins_on && ~lm(1) && ~isnan(bbin_cur(ax))
            bbin_b(ax, bbin_cur(ax)) = x_upd(5);  bbin_P(ax, bbin_cur(ax)) = P_upd(5, 5);   % keep the map current
        end
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
        diag.a_hm_nd        = a_bar_wm * a_o;         % legacy 1/pN convention (do NOT mix with a_disp)
        diag.a_wm_bar       = a_bar_wm;
        diag.y2             = y2;                     % normalized gain units
        diag.delta_x_m      = delta_w_m * R_radius;   % [um]
        diag.innovation_y2  = innov_y2_v;
        diag.innovation_y1  = innov_y1_v;   % [-] normalized; whiteness diagnostic
        diag.S1             = S1_v;         % innovation covariance of y1 (logging only)
        diag.S2             = S2_v;         % innovation covariance of y2 (NaN when gated off)
        diag.K_kf_a_y2      = K_a_y2_v;
        diag.K_kf_dx_y2     = K_dx_y2_v;   % L32
        diag.K_kf_b_y1      = K_b_y1_v;      % L51
        diag.K_kf_b_y2      = K_b_y2_v;      % L52
        diag.K_kf_dx_y1     = K_dx_y1_v;
        diag.K_kf_a_y1      = K_a_y1_v;
        diag.P41            = P41_v;
        diag.dws_y1         = dws_y1_v;    % [-] ws increment from the y1 update
        diag.dws_y2         = dws_y2_v;    % [-] ws increment from the y2 update
        diag.P_a            = P_a_v * a_disp^2;       % (um/pN)^2
        diag.P_a_nd         = P_a_v;                  % [-] (a_bar^2 units)
        diag.P_dx           = P_dx_v * R_radius^2;    % [um^2]
        diag.x_D_hat        = zeros(3, 1);
        if has_b_true
            b_post(b_true > 0) = b_true(b_true > 0);   % report the b the law used
        end
        diag.b_hat          = b_post;
        if b_bins_on; diag.b_bin = bbin_cur; else; diag.b_bin = zeros(3, 1); end   % which height bin slot 5 is carrying
        diag.p_hat          = p_post;                 % REAL p (not the sibling alias)
        diag.ws_hat         = ws_post;
        if pade22; diag.c1_hat = x_e_per_axis(8, :).';  diag.d2_hat = x_e_per_axis(9, :).'; end   % [2/2] reporters (slots 8, 9)
        diag.delta_a_hat    = b_post;                 % driver-log alias
        diag.a_prime_hat    = a_prime_v * a_o;        % legacy d a_nd/d h_bar [1/pN]
        diag.a_prime_bar    = a_prime_v;              % d a_bar / d w_bar [-]
        diag.lam_b_k        = lam_b_v;                % this step's slot-5 forgetting factor per axis (1 = none)
        diag.q55a_stat      = q55a_stat;              % q55_adapt: y1 statistic used at this step's predict (0 when off)
        diag.q55a_q         = q55a_q;                 % q55_adapt: Q55 added at this step's predict (0 when off)
        diag.pred_mean2     = pred_mean2_v;           % second-order mean term added in predict [a_bar], 0 when off
        diag.gate_active_per_axis = gate_off;
        diag.guards_individual    = G_flags;
        diag.h_bar          = h_bar;
        diag.lam_b          = lam_b_v;
    diag.lam_b_spend    = lam_b_spend;
    diag.h_bar_d        = w_bar_d;
        diag.f_d            = f_d;                    % [pN]
        diag.f_det          = fbar_det / a_o;         % [pN]
        diag.F_dh           = F_dw_vec / a_o;         % [pN]
        diag.dx_r           = dw_r * R_radius;        % [um], same convention as delta_x_m
        diag.a_hat          = a_hat_phys;
        diag.a_hat_nd       = a_bar_post * a_o;       % legacy 1/pN convention (do NOT mix with a_disp)
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

function [a_bar_p, A_a] = local_gain_law_formC(a_bar, enable, law_b)
%LOCAL_GAIN_LAW_FORMC  Slope read off the gain STATE (tex S1/S2). No height,
%   no wall position, NO parameter of any kind -- this is the whole law.
%       a_bar' = (1 - a_bar)^2
%       A_a    = d a_bar'/d a_bar = -2 (1 - a_bar)               (< 0 always)
%   The additive disturbance does NOT enter here (contrast the multiplicative
%   sibling's (1+da) factor): it enters the state equation once per step.
%   law_b (optional, default 1) MULTIPLIES the slope. The far-field limit of
%   b_true is 8/9 EXACTLY (c_perp -> 1 + 9/(8 h_bar)), so law_b = 8/9 is the
%   asymptotic anchor, not a fit. law_b = 1 is the CONTACT anchor, where
%   a_true(1) = 0 and a_true'(1) = 1 give b_true = 1 exactly.
    if nargin < 3 || isempty(law_b); law_b = 1; end
    if ~enable
        a_bar_p = 0; A_a = 0;
        return;
    end
    om      = 1 - a_bar;
    a_bar_p = law_b * om^2;
    A_a     = -2 * law_b * om;
end


function [a_bar, dA_dw0] = local_seed_level_formC(w_bar, w0, enable, gap_floor, law_b)
%LOCAL_SEED_LEVEL_FORMC  Integrated law at the seed height (tex, seed section).
%       a_bar(w_bar) = 1 - 1/[law_b (w_bar - w0)]
%   This IS the integral of a_bar' = (1-a_bar)^2/law_b: with u = 1/(1-a_bar),
%   du/dw_bar = law_b. The slope and the seed MUST carry the same law_b or
%   the arm is estimating one family while seeded on another (defect found
%   2026-08-13). At law_b = 9/8 and w0 = 0 this is the EXACT far-field truth,
%   since c_perp -> 1 + (9/8)/h_bar gives a_bar -> 1 - (9/8)/h_bar, i.e.
%   law_b = 8/9 with w0 = 0.
%   w0 is the integration constant, so setting a_bar_hat[0] IS setting the
%   wall position: this is the ONLY place a nominal wall enters the controller.
%   da does not appear -- the integrated curve is parameter-free, which is
%   exactly why P45[0] = 0 is structural rather than a modelling choice.
%   Level Jacobian at fixed w_bar, exact:  d a_bar/d w0 = -a_bar'
    if nargin < 5 || isempty(law_b); law_b = 1; end
    if ~enable || ~isfinite(w_bar)
        a_bar = 1; dA_dw0 = 0;
        return;
    end
    u      = max(w_bar - w0, gap_floor);
    a_bar  = 1 - 1 / (law_b * u);
    dA_dw0 = -1 / (law_b * u^2);          % = -a_bar'
end


function F_e = local_build_F_e_formC(lambda_c, F_dw, a_bar_p, A_a, J_da, M, J_d3)
%LOCAL_BUILD_F_E_FORMC  7x7 error-dynamics Jacobian (tex S6(b)).
%   cols: 1=dw1 2=dw2 3=dw3 4=a_bar 5=da 6,7=inert (locked, zero P0/J/K)
%   Row 3 = [0 0 lc -F_dw 0 0 0]      <- F_e(3,5) = 0: the control law is
%           untouched by da, so the loop-coupling column is IDENTICAL to the
%           baseline (tex S6(b)).
%   Row 4 = [0 0 (1-lc)a_bar'  1 + a_bar' F_dw + A_a M   J_b   0 0]
%   J_b = -a_bar' * M / b_hat, supplied already multiplied by M.
%   Row 5 = identity (da[k+1] = da[k]).
%   J_b is PROPORTIONAL TO DISPLACEMENT, so b is unobservable in a hold
%   through this path -- and through H(2,5) as well, which carries Grad. The
%   two paths vanish together. That is the structural price of estimating a
%   law constant instead of an additive disturbance.
%   The A_a*M term has NO counterpart in the height writing, where a_bar' did
%   not depend on a_bar and F_e(4,4) was exactly 1 + a_bar'*F_dw.
%   M = the FULL row-4 increment (command step + (1-lc) dw_3_hat + MA(2)
%   memory feedthrough) -- exactly what a_bar' multiplies in predict.
    if nargin < 7 || isempty(J_d3); J_d3 = 0; end
    one_minus_lc = 1 - lambda_c;
    F_e = zeros(7);
    F_e(1, 2) = 1;
    F_e(2, 3) = 1;
    F_e(3, 3) = lambda_c;   F_e(3, 4) = -F_dw;
    F_e(4, 3) = one_minus_lc * a_bar_p + J_d3 * M;
    F_e(4, 4) = 1 + a_bar_p * F_dw + A_a * M;
    F_e(4, 5) = J_da;
    F_e(5, 5) = 1;
    F_e(6, 6) = 1;
    F_e(7, 7) = 1;
end

function R2 = compute_R2_formB(a_bar, sigma2_n, IF_abc, C_dpmr, C_n, ...
                               K_var, amlpf_var_factor, xi_bar, kappa_T, Q44, ...
                               delay_steps, a_cov, whitened, dscale)
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
    if nargin < 14 || isempty(dscale); dscale = 1.0; end
    if whitened
        R2 = a_cov * (2 - a_cov) * R2_int + dscale * a_cov^2 * delay_steps * Q44;
    else
        R2 = R2_int + dscale * delay_steps * Q44;
    end
end


function a_bar = local_gain_from_law(w_bar_d, w0, dw3_hat, b_hat, gap_floor)
%LOCAL_GAIN_FROM_LAW  a_bar = 1 - 1/(b (w_hat - w_s)) at the estimator's own height w_hat = w_bar_d - dw3_hat.
%   Same guards as the 'act' slope branch: the gap is floored so the inversion stays finite near contact.
    u = max(w_bar_d - w0 - dw3_hat, gap_floor / max(b_hat, eps));
    a_bar = 1 - 1 / max(b_hat * u, gap_floor);
end

function a_bar = local_gain_11(w_bar_d, w_s, dw3_hat, c0_hat, d1_hat, gap_floor)
%LOCAL_GAIN_11  a_bar = (g + c0)/(g + d1), g = w_hat - w_s, w_hat = w_bar_d - dw3_hat (0916_estb_11_two_versions.tex, section 3).
%   The [1/1] Pade writing with the ACTUAL wall w_s known; the denominator is floored so the value stays finite.
    g = w_bar_d - dw3_hat - w_s;
    a_bar = (g + c0_hat) / max(g + d1_hat, gap_floor);
end

function a_bar = local_gain_22(w_bar_d, w_s, dw3_hat, c0_hat, c1_hat, d1_hat, d2_hat, gap_floor)
%LOCAL_GAIN_22  a_bar = (g^2 + c1 g + c0)/(g^2 + d1 g + d2), g = w_bar_d - dw3_hat - w_s (0918_estb_22_versionA.tex, section 3).
    g = w_bar_d - dw3_hat - w_s;
    a_bar = (g^2 + c1_hat * g + c0_hat) / max(g^2 + d1_hat * g + d2_hat, gap_floor);
end

function [ap, app, dA, dAp] = local_law_22(g, c0, c1, d1, d2, gap_floor)
%LOCAL_LAW_22  Slope, curvature and Jacobians of the [2/2] law at the gap g (0918_estb_22_versionA.tex, section 1).
%   dA  = d a_bar /d (c0, c1, d1, d2) at fixed g;  dAp = d a_bar'/d (c0, c1, d1, d2), TOTAL derivatives at fixed g
%   (sympy check: test_script/scratch/check_estb_22_jacobians.py, ALL PASS).
    N  = g^2 + c1 * g + c0;
    D  = max(g^2 + d1 * g + d2, gap_floor);
    Np = 2 * g + c1;
    Dp = 2 * g + d1;
    a  = N / D;
    ap  = (Np - a * Dp) / D;
    app = (2 * (1 - a) - 2 * ap * Dp) / D;
    dA  = [1 / D, g / D, -a * g / D, -a / D];
    dAp = [-Dp / D^2, (D - g * Dp) / D^2, a * g * Dp / D^2 - a / D - ap * g / D, a * Dp / D^2 - ap / D];
end

function a_bar = local_gain_delta(w_bar_d, w_s, dw3_hat, delta_hat)
%LOCAL_GAIN_DELTA  a_bar = D_perp(1/g), g = (w_hat - w_s) + delta, w_hat = w_bar_d - dw3_hat (0920_estb_delta_law.tex, section 3).
%   The offset law: the exact plane polynomial at the distance to the EFFECTIVE wall (w_s - delta). g is floored at the
%   polynomial's contact 1 (a_bar = 0 there), the same clamp the plants use.
    a_bar = local_law_delta(w_bar_d - dw3_hat - w_s + delta_hat);
end

function [a, ap, app] = local_law_delta_exp(g, sig)
%LOCAL_LAW_DELTA_EXP  Gaussian expectations of the offset law's value, slope and curvature over g ~ N(g, sig^2)
%   (statistical linearisation, flag delta_slin). 3-point Gauss-Hermite: nodes g, g +- sqrt(3) sig, weights 2/3, 1/6, 1/6
%   (exact for polynomials to degree 5). sig = 0 returns the point values of local_law_delta.
    if sig <= 0
        [a, ap, app] = local_law_delta(g);
        return
    end
    d = sqrt(3) * sig;
    [a0, ap0, app0] = local_law_delta(g);
    [a1, ap1, app1] = local_law_delta(g + d);
    [a2, ap2, app2] = local_law_delta(g - d);
    a   = (2/3) * a0   + (1/6) * (a1   + a2);
    ap  = (2/3) * ap0  + (1/6) * (ap1  + ap2);
    app = (2/3) * app0 + (1/6) * (app1 + app2);
end

function [a, ap, app] = local_law_delta(g)
%LOCAL_LAW_DELTA  Value, slope and curvature of the plane polynomial at the effective gap g (0920_estb_delta_law.tex, section 1).
%   a = D(u), a' = -u^2 D'(u), a'' = u^4 D''(u) + 2 u^3 D'(u), u = 1/max(g, 1); D = calc_correction_functions.m:81-82
%   (sympy check: test_script/scratch/check_estb_delta_jacobians.py, ALL PASS). d a/d delta = a', d a'/d delta = a''.
    u   = 1 / max(g, 1);
    u2  = u * u;  u3 = u2 * u;  u4 = u3 * u;  u5 = u4 * u;  u9 = u5 * u4;  u10 = u9 * u;  u11 = u10 * u;  u12 = u11 * u;
    a   = 1 - (9/8) * u + (1/2) * u3 - (57/100) * u4 + (1/5) * u5 + (7/200) * u11 - (1/25) * u12;
    Dp  = -(9/8) + (3/2) * u2 - (228/100) * u3 + u4 + (77/200) * u10 - (12/25) * u11;
    Dpp = 3 * u - (684/100) * u2 + 4 * u3 + (770/200) * u9 - (132/25) * u10;
    ap  = -u2 * Dp;
    app = u4 * Dpp + 2 * u3 * Dp;
end

function a_bar = local_gain_plaw(w_bar_d, w_s, dw3_hat, c_hat, p)
%LOCAL_GAIN_PLAW  Advisor power law at the estimator's own height: a_bar = 1 - (1 - c)/((p - 1) h^(p-1)), h = w_bar_d - dw3_hat - w_s.
    a_bar = local_law_plaw(w_bar_d - dw3_hat - w_s, c_hat, p);
end

function [a, ap, app, dac, dapc] = local_law_plaw(h, c, p)
%LOCAL_LAW_PLAW  Value, slope, curvature and the c Jacobians of the advisor power law a' = (1 - c)/h^p integrated from the far
%   field: a = 1 - (1 - c) phi, phi = 1/((p - 1) h^(p-1)); a' = (1 - c)/h^p; a'' = -p (1 - c)/h^(p+1); d a/d c = phi = (1 - a)/(1 - c);
%   d a'/d c = -1/h^p (sympy check: test_script/scratch/check_estb_plaw_jacobians.py, ALL PASS). h floored at the visible contact 1,
%   the same clamp the plants use (the law itself has no contact: a(1) = 1 - (1 - c)/(p - 1)).
    h    = max(h, 1);
    hp1  = h^(p - 1);
    phi  = 1 / ((p - 1) * hp1);
    a    = 1 - (1 - c) * phi;
    ap   = (1 - c) / (hp1 * h);
    app  = -p * ap / h;
    dac  = phi;
    dapc = -1 / (hp1 * h);
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
%   The slot-5 placeholder is 0 (the da seed of this writing); slots 6-7 are
%   inert in this writing and their placeholders are cosmetic.
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
    d.K_kf_dx_y2        = zeros(3, 1);
    d.K_kf_b_y1         = zeros(3, 1);
    d.K_kf_b_y2         = zeros(3, 1);
    d.K_kf_dx_y1        = zeros(3, 1);
    d.K_kf_a_y1         = zeros(3, 1);
    d.P41               = zeros(3, 1);
    d.dws_y1            = zeros(3, 1);
    d.dws_y2            = zeros(3, 1);
    d.P_a               = zeros(3, 1);
    d.P_a_nd            = zeros(3, 1);
    d.P_dx              = zeros(3, 1);
    d.x_D_hat           = zeros(3, 1);
    d.b_hat             = zeros(3, 1);           % slot 5 = b (placeholder; init fills the seed)
    d.p_hat             = ones(3, 1);
    d.ws_hat            = ones(3, 1);
    d.c1_hat            = zeros(3, 1);           % [2/2] pade22: slot 8 (zero otherwise)
    d.d2_hat            = zeros(3, 1);           % [2/2] pade22: slot 9 (zero otherwise)
    d.delta_a_hat       = zeros(3, 1);
    d.a_prime_hat       = zeros(3, 1);
    d.a_prime_bar       = zeros(3, 1);
    d.pred_mean2        = zeros(3, 1);
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


function v = local_da_inject(has_known, J_da, da)
%LOCAL_DA_INJECT  Row-4 disturbance term. In the estimated arm it is gated by
%   the lock mask through J_da; in the known-disturbance arm the value is
%   exogenous and always applied (J_da is zeroed there only so that no
%   Jacobian / covariance is built for it).
    if has_known
        v = da;
    else
        v = J_da * da;
    end
end


function [a, ap] = local_par_plane(h)
%LOCAL_PAR_PLANE  Plane wall-PARALLEL gain a_par = 1/c_par(h) and its slope d a_par/dh at height h [R] (par_wall_known, 10-07).
%   The plane is the known wall (wall-test protocol: estimator starts from the plane); central difference, step 1e-5 R.
    h = max(h, 1.0);  dh = 1e-5;
    [cp0, ~] = calc_correction_functions(h);  [cpp, ~] = calc_correction_functions(h + dh);  [cpm, ~] = calc_correction_functions(max(h - dh, 1.0));
    a = 1 / cp0;  ap = (1 / cpp - 1 / cpm) / (h + dh - max(h - dh, 1.0));
end
