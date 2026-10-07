% STATUS: ACTIVE | SSOT = reference/eq17_analysis/derivation/0920_5state_noaprime.tex (full) / 0921_5state_noap_simple.tex (teacher chain)
% LAYER-1 CLOSED 2026-09-22. Final defaults: gain_order 2, lf_mode 'ljung' (clamp 0.98, gain block [4 5 8]), P44[0] by seed_mode
% (0.5 far / 0.05 cheat), sqrtP55[0] = a_nom/40000, rate_damp off, y2_mirror off (breaks the ramp), no noise state for the readout.
function [f_d, ekf_out, diag] = motion_control_law_eq17_5state_noap(del_pd, pd, p_m, params, ctrl_const, a_ctrl_override, a_true_diag, aprime_true_diag)
%MOTION_CONTROL_LAW_EQ17_5STATE_NOAP  5-state EKF controller WITHOUT a' anywhere.
%
%   [f_d, ekf_out, diag] = motion_control_law_eq17_5state_noap(del_pd, pd, p_m, params, ctrl_const)
%   [f_d, ekf_out, diag] = motion_control_law_eq17_5state_noap(..., ctrl_const, a_ctrl_override)
%
%   Layer-1 estimator of the development ladder (2026-09-20): the gain a_x is
%   an unknown parameter with a KINEMATIC model only (constant rate, no process
%   noise on either gain slot). No wall function c(h_bar), no slope a', no
%   step-size feed-forward enters the gain model. Every a'-FREE defect found
%   later on the expgain / formB / Meng lines is repaired here (derivation
%   sections in brackets):
%
%     A  y2_whiten        whitened readout y2' = a_cov*u[k], H2' = a_cov*H2,
%                         R22' = 2 a_cov^2 IF_white (a+xi)^2            [S7]
%     B  fe_km1           predict k-1 -> k uses f_d[k-1..k-3]           [S1,S8]
%     C  upd_before_law   law consumes the fresh posterior; exact past-
%        corr_input       error bookkeeping u_c (no A3 approximation)   [S4]
%     D  nw_mcorr         n_x double path as correlated-noise KF:
%                         F(3,1) = alpha, known input -alpha*res1        [S6]
%     E  ma2_aug          thermal MA(2) memory slots m1, m2, rank-1 Q    [S5]
%     F  prior P[0]       gain block from a prior, position block DARE   [S9]
%     G  IF_white         colour factor s = 1 for the whitened readout   [S7]
%
%   Every flag defaults to TRUE (this is the production form of the layer);
%   each can be switched off for the leave-one-out audit (.claude/rules/
%   stacked-fix-audit.md). Constraint: fe_km1 = false forces
%   upd_before_law = false (the old ordering needs f_d[k] before the update).
%
%   State (7 slots, slots 6-7 locked when ma2_aug = false):
%       x = [dx_1; dx_2; dx_3; a_x; da_x; m_1; m_2]
%       dx_1 = dx[k-2], dx_2 = dx[k-1], dx_3 = dx[k]; a_x gain [um/pN];
%       da_x per-step gain rate; m_1 = t[k-1], m_2 = t[k-2] thermal kicks.
%
%   ctrl_const (build_eq17_6state_constants output + optional fields):
%       .y2_whiten .fe_km1 .upd_before_law .corr_input .nw_mcorr .ma2_aug
%                                   logical, default true
%       .gain_order                 1 | 2 (default) | 3: local-polynomial order of the gain model. 1 locks the rate slot
%                                   (a random-walk level under the schedule); 3 adds slot 8 = the per-step change of the
%                                   rate (a[k-d] = a - d*da + d(d+1)/2*dda, so H2 gets +3 on slot 8 for d = 2)
%       .y2_mirror                  true (default, aligned with the mainline) | false: the a_xm gauge inverts the residual variance assuming
%                                   g = a/a_hat = 1; at g != 1 the loop suppresses the fluctuations being read, so
%                                   dE[a_xm]/da |_{g=1} = s < 1 (~0.67 at lc = 0.7, d = 2). H2 row x s_i with the
%                                   DERIVED per-axis s_i = y2_mirror_sT + y2_mirror_cN sigma2_n/(4 kBT a_hat)
%                                   (build_eq17_6state_constants compute_y2_mirror, exact Lyapunov; 4state 07-13,
%                                   formB echo S = 0.32). Without it y2 is over-credited by 1/s: level learns slower
%                                   than booked and P44 is overconfident by 1/s. Measured 2026-09-23: 1.10 R hold
%                                   honesty 1.54 -> 1.32 and 22.2 R 1.17 -> 1.05 with a seed at the truth; with the
%                                   far-field seed a_nom at 1.10 R (loop gain 11x off at start) the g = 1 linearisation
%                                   is invalid during the transient and the hold scatter doubles -> use seed_mode
%                                   'true_h0' (mainline convention) or switch it off for far-field-seed demos.
%       .rate_damp                  false (default) | true: the rate slot reverts to zero over the schedule's memory,
%                                   F(5,5) = rho = 1 - 1/T*, Q55 = (1 - rho^2) (r a_hat)^2 (stationary rate variance =
%                                   the command-implied rate squared; zero free numbers). r = 0 => rho = 1, Q55 = 0.
%       .seed_value                 3x1 [um/pN]: explicit a_hat[0] (overrides seed_mode's value; used for the cheat
%                                   init on a non-plane plant, where the truth is the plant's own curve)
%       .seed_mode                  'anom' (default; a_hat[0] = a_nom, far-
%                                   field init) | 'true_h0' (a_nom/c(h_bar_0),
%                                   cheat init for trajectory runs)
%       .Pf_a_std_rel               sqrt(P44[0])/a_nom; default 0.5 for seed_mode 'anom' (must cover a_nom/c_perp
%                                   down to h_bar ~ 1.2), 0.05 for 'true_h0' (the seed is the truth; 0.5 only buys
%                                   the +-0.4 start transient, measured 2026-09-21)
%       .Pf_da_std_rel              sqrt(P55[0])/a_nom per step, default 1/40000 (runs start in a hold, so the rate
%                                   is ~0 at t = 0; 1/400 let the first correlated samples be read as a slope and
%                                   produced the +-0.09 start transient; with lf_mode ljung the schedule re-opens
%                                   P55 when the command moves, so the tight prior costs nothing on the ramp)
%       .lambda_f                   forgetting factor on the GAIN block (slots 4-5): P(4:5,:) and P(:,4:5)
%                                   scaled by 1/sqrt(lambda_f) before each predict (block / lambda_f, cross terms
%                                   / sqrt). Default 1 = off (pure layer 1). Any value < 1 is a memory knob: the
%                                   arm is labelled layer 1.5 in the derivation, not layer 1.
%       .lf_mode                    'ljung' (default since 2026-09-23): per-step tracking-optimal memory | 'const': lambda_f as given (1 = pure layer 1)
%                                   (Ljung cube-root law, Meng ledger 31) T* = (2 J r^2)^(-1/3) steps with
%                                   r = |d ln a / dt|*Ts from the COMMAND: r = |h_dot_d| / (h_bar_d (h_bar_d - 1)) * Ts
%                                   (published far-field log-slope, a shape assumption => layer 2 knowledge) and
%                                   J = a_hat^2 H2'^2 / R22' the filter's own relative Fisher rate per step;
%                                   lambda_f = 1 - 1/T*, clamped to [lf_min, 1]. Zero free numbers besides the clamp.
%       .lf_min                     clamp for 'ljung' (default 0.98, T* >= 31 ms; 0.998 was the slow-ramp value)
%       .lf_slots                   which slots the forgetting acts on (default [4 5 8] = the gain block; [5] = rate only)
%       .lf_slope                   'far' (default): log-slope 1/(h(h-1)) | 'exact': |K_h_perp(h_d)| of the plane
%                                   polynomial (full wall-law knowledge, layer 2)
%       .lf_mode = 'q55'            rate-change budget instead of forgetting: Q55[k] = (r * a_hat)^2 / T* per step
%                                   (a random walk that may move the rate by its own size r*a over the optimal memory
%                                   T* steps: sqrt-N accumulation), no P scaling;
%                                   r = 0 in a hold => Q55 = 0 and the rate slot is pulled back by y2
%       .y2_oracle                  DIAGNOSTIC (default false): replace u[k] by a white Gaussian sample with
%                                   mean a_true[k-d] and the Isserlis marginal 2(a+xi)^2, IF = 1 (needs a_true_diag)
%       .q_gain_true                DIAGNOSTIC (default false): thermal Q evaluated at a_true instead of a_hat
%       .q_aprime_true              DIAGNOSTIC (default false, 2026-09-28): the mainline (formC_b) Q with the true
%                                   slope: the thermal step also moves the gain through the law, so
%                                   Q34 = Q43 = -a'_true*Q33, Q44 = a'_true^2*Q33 (a'_true = da_true/dx along the
%                                   axis, at the true height of step k-1, from the driver's 8th argument). Only Q
%                                   changes; the state prediction and F_e are untouched. With ma2_aug the rank-1
%                                   noise vector gets g(4) = a'_true, which also sets the memory cross term Q46
%                                   (needed for Q to stay positive semidefinite).
%       .q_aprime_cross             true (default) | false: with q_aprime_true, false keeps only Q44 = a'_true^2*Q33
%                                   and drops the cross terms (discriminator arm, 2026-09-28)
%       .q44_r2_frac                0 (default) | rho > 0 (2026-09-30): Q44 += rho * R2[k-1], R2 in gain units
%                                   (R22 / H2(4)^2 of the previous y2 update). Q44/R2 = rho is fixed, so the gain
%                                   memory is 1/sqrt(rho) steps at every height; rho = (1 - lambda_f)^2 equivalent.
%       .q44_aprime_cmd             DIAGNOSTIC (default false, 2026-09-30): Q44 += (a'_true[k-1] * del_pd[k-1])^2, the
%                                   squared gain change the commanded step causes (upper bound: reads the true slope).
%       .q44_lag_budget             DIAGNOSTIC eps > 0 (default 0, 2026-09-30): Q44 += R2[k-1] * rho_k,
%                                   rho_k = (a'_true[k-1] del_pd[k-1] / (eps a_hat))^2: memory = eps a_hat / |gain change per step|.
%       .q44_mse_opt                DIAGNOSTIC kappa > 0 (default 0, 2026-09-30): Q44 += R2[k-1] * l*^2,
%                                   l* = kappa (4 v^2 / R2)^(1/3), v = |a'_true del_pd|: the steady gain that minimises
%                                   (v/l)^2 + l R2/2 (lag^2 + noise^2). kappa = 1 is the optimum; 0.5, 2 bracket it.
%       .lf_info_beta               beta > 0 (default 0, 2026-09-30): slot-4 congruence forgetting P44 / lambda_k, cross / sqrt,
%                                   lambda_k = max(exp(-beta s_k), lf_info_floor = 0.99), s_k = 1 - P44_post/P44_pred (formC_b
%                                   lambda_f_b_info, c-only line C78, ported to the gain slot).
%       .q44_ratio_std              sqrt(P0)/a_nom > 0 (default 0, 2026-09-30): Q/R-ratio design ported from formC_b q55_adapt
%                                   = 'ratio' (ap model): I_k = 1/P44_post - 1/P44_pred, next predict Q44 += P0^2 I_k.
%       .q44_ratio_cap              false (default) | true: Q44 += P0^2 min(I_k, 1/P0), so one step never puts back more than P0
%                                   (R2 collapses when a_hat -> 0, I_k -> 1e4, the uncapped rule then forgets everything).
%       .q44_ratio_rel              sigma > 0 (default 0, 2026-09-30): as q44_ratio_std but sqrt(P0) = sigma * a_hat[k-1],
%                                   so the held uncertainty shrinks with the gain toward the wall.
%
%   Outputs as the siblings: ekf_out = [a_hat_x; a_hat_z; a_hat_y; h_bar];
%   diag adds P_da (P55), S_y1 (believed prior innovation variance of y1),
%   innovation_y1, y2_fed (value fed to the EKF).
%
%   See also: motion_control_law_eq17_5state, build_eq17_6state_constants

    if nargin < 6; a_ctrl_override = []; end
    if nargin < 7; a_true_diag = []; end        % driver ground truth; consumed only by y2_oracle / q_gain_true (diagnostic arms)
    if nargin < 8; aprime_true_diag = []; end   % driver ground-truth slope da/dx per axis; consumed only by q_aprime_true
    has_override = ~isempty(a_ctrl_override);
    if has_override
        a_ctrl_override = a_ctrl_override(:);
        assert(numel(a_ctrl_override) == 3 && all(isfinite(a_ctrl_override)) && all(a_ctrl_override > 0), ...
               'motion_control_law_eq17_5state_noap:badOverride', ...
               'a_ctrl_override must be a 3x1 finite positive vector.');
    end

    N_STATE = 8;      % slot 8 = second-order gain term, used only when gain_order = 3 (else locked)
    if params.ctrl.enable < 0.5
        f_d = zeros(3, 1);  ekf_out = [1; 1; 1; 0];
        if nargout >= 3; diag = empty_diag_noap(); diag.f_d = f_d; end
        return;
    end

    % ------------------------------------------------------------------
    % Persistent state
    % ------------------------------------------------------------------
    persistent x_e_per_axis P_per_axis
    persistent dx_bar_m sigma2_dxr_hat a_xm_km1
    persistent pd_km1 pd_km2
    persistent f_d_km1 f_d_km2 f_d_km3          % forces f_d[k-1], f_d[k-2], f_d[k-3]
    persistent a_ctrl_km1 a_ctrl_km2            % gains the law used at k-1, k-2
    persistent a_post_km1 a_post_km2            % older posteriors a_hat[k-2|k-2], a_hat[k-3|k-3] (x_curr(4) is k-1)
    persistent res1_km1 u_c_km1                 % posterior y1 residual and past-error input of the previous call
    persistent k_step initialized
    persistent lambda_c alpha d_delay Ts kBT R_radius sigma2_n_s a_nom_p
    persistent a_pd a_cov C_dpmr C_n K_var IF_abc_raw IF_abc_w xi_per_axis
    persistent t_warmup_kf h_bar_safe
    persistent enable_wall w_hat_n pz_wall
    persistent y2_whiten fe_km1 upd_before_law corr_input nw_mcorr ma2_aug
    persistent y2_oracle q_gain_true a_true_km1 a_true_km2              % diagnostic arms + ground-truth delay line
    persistent q_aprime_true q_aprime_cross aprime_true_km1              % mainline Q with the true slope (diagnostic)
    persistent lambda_f lf_scale lf_mode lf_min lf_used lf_slope q55_used  % gain-block forgetting (layer 1.5 knob), default off
    persistent gain_order Pf_dda_std_rel                                    % local-polynomial order of the gain model
    persistent rate_damp rho_used q55_damp                                  % Singer/OU rate variant (2026-09-21)
    persistent lf_slots                                                      % slots the forgetting acts on
    persistent y2_mirror y2m_sT y2m_cN                                       % y2 gauge sensitivity (echo) correction
    persistent q44_r2_frac r2a_km1                                           % Q44 = rho * R2 (in gain units, previous step)
    persistent q44_aprime_cmd del_pd_km1                                     % Q44 = (a'_true * commanded step)^2 (upper-bound arm)
    persistent q44_lag_budget                                                % rho = (a'_true * step / (eps * a_hat))^2 (upper-bound arm)
    persistent q44_mse_opt                                                   % l* = kappa (4 v^2 / R2)^(1/3), v = |a'_true step| (upper-bound arm)
    persistent lf_info_beta lf_info_floor                                    % information-driven forgetting of slot 4 (c-only line, C78)
    persistent q44_ratio_std q44r_I                                          % Q/R-ratio design (ap model 09-30): Q44[k+1] = P0^2 I_k
    persistent q44_ratio_cap                                                 % cap the put-back at P0 per step: Q44 = P0^2 min(I, 1/P0)
    persistent q44_ratio_rel                                                 % relative P0: sqrt(P0) = sigma * a_hat

    if isempty(initialized)
        initialized = true;
        Ts        = params.ctrl.Ts;
        kBT       = params.ctrl.k_B * params.ctrl.T;
        R_radius  = params.common.R;
        sigma2_n_s = params.ctrl.sigma2_noise(:);
        a_nom_p   = Ts / params.ctrl.gamma;

        lambda_c    = ctrl_const.lambda_c;
        alpha       = 1 - lambda_c;
        d_delay     = ctrl_const.d;
        assert(d_delay == 2, 'motion_control_law_eq17_5state_noap:delay', 'Only d = 2 is implemented.');
        C_dpmr      = ctrl_const.C_dpmr;
        C_n         = ctrl_const.C_n;
        K_var       = ctrl_const.K_var;
        IF_abc_raw  = ctrl_const.IF_abc(:);
        IF_abc_w    = ctrl_const.IF_abc_white(:);
        xi_per_axis = ctrl_const.xi_per_axis(:);
        t_warmup_kf = ctrl_const.t_warmup_kf;
        h_bar_safe  = ctrl_const.h_bar_safe;
        a_cov       = ctrl_const.a_cov;
        a_pd        = ctrl_const.a_pd;
        iir_mode = get_field_default(ctrl_const, 'iir_warmup_mode', 'prefill');
        assert(strcmpi(iir_mode, 'prefill'), 'motion_control_law_eq17_5state_noap:warmup', 'Only iir_warmup_mode = prefill is supported.');

        y2_whiten      = logical(get_field_default(ctrl_const, 'y2_whiten', true));
        fe_km1         = logical(get_field_default(ctrl_const, 'fe_km1', true));
        upd_before_law = logical(get_field_default(ctrl_const, 'upd_before_law', true));
        corr_input     = logical(get_field_default(ctrl_const, 'corr_input', true));
        nw_mcorr       = logical(get_field_default(ctrl_const, 'nw_mcorr', true));
        ma2_aug        = logical(get_field_default(ctrl_const, 'ma2_aug', true));
        if ~fe_km1 && upd_before_law
            error('motion_control_law_eq17_5state_noap:flagCombo', ...
                  'upd_before_law requires fe_km1 (the predict must not depend on f_d[k]).');
        end
        y2_oracle   = logical(get_field_default(ctrl_const, 'y2_oracle', false));
        lambda_f    = get_field_default(ctrl_const, 'lambda_f', 1);
        assert(lambda_f > 0 && lambda_f <= 1, 'motion_control_law_eq17_5state_noap:lambdaF', 'lambda_f must be in (0, 1].');
        lf_slots = get_field_default(ctrl_const, 'lf_slots', [4 5 8]);
        y2_mirror = logical(get_field_default(ctrl_const, 'y2_mirror', true));   % default ON since 2026-09-23 (mainline formC_b y2_echo_corr = true); needs a seed near the truth, see header
        y2m_sT = get_field_default(ctrl_const, 'y2_mirror_sT', 1);  y2m_cN = get_field_default(ctrl_const, 'y2_mirror_cN', 0);
        lf_scale = ones(N_STATE, 1);  lf_scale(lf_slots) = 1 / sqrt(lambda_f);  lf_scale = lf_scale * lf_scale.';
        lf_mode = lower(get_field_default(ctrl_const, 'lf_mode', 'ljung'));   % default 'ljung' since 2026-09-23 (the closed layer-1.5 form); 'const' with lambda_f = 1 = pure layer 1
        lf_min  = get_field_default(ctrl_const, 'lf_min', 0.98);    % T* >= 31 ms: on the 1 Hz canon the law itself asks for ~30 ms (clamp scan 2026-09-21: osc rms 0.185 -> 0.069 a_nom); the slow ramp pays 5.0 -> 5.3% rel rms
        lf_used = lambda_f * ones(3, 1);  q55_used = zeros(3, 1);
        lf_slope = lower(get_field_default(ctrl_const, 'lf_slope', 'far'));
        q_gain_true = logical(get_field_default(ctrl_const, 'q_gain_true', false));
        q_aprime_true = logical(get_field_default(ctrl_const, 'q_aprime_true', false));
        q_aprime_cross = logical(get_field_default(ctrl_const, 'q_aprime_cross', true));   % false = Q44 only (discriminator)
        q44_r2_frac = get_field_default(ctrl_const, 'q44_r2_frac', 0);  r2a_km1 = nan(3, 1);
        q44_aprime_cmd = logical(get_field_default(ctrl_const, 'q44_aprime_cmd', false));  del_pd_km1 = zeros(3, 1);
        q44_lag_budget = get_field_default(ctrl_const, 'q44_lag_budget', 0);
        q44_mse_opt = get_field_default(ctrl_const, 'q44_mse_opt', 0);
        lf_info_beta = get_field_default(ctrl_const, 'lf_info_beta', 0);  lf_info_floor = get_field_default(ctrl_const, 'lf_info_floor', 0.99);
        q44_ratio_std = get_field_default(ctrl_const, 'q44_ratio_std', 0);  q44r_I = zeros(3, 1);   % sqrt(P0) in units of a_nom
        q44_ratio_cap = logical(get_field_default(ctrl_const, 'q44_ratio_cap', false));
        q44_ratio_rel = get_field_default(ctrl_const, 'q44_ratio_rel', 0);
        seed_mode     = lower(get_field_default(ctrl_const, 'seed_mode', 'anom'));
        if strcmp(seed_mode, 'true_h0'); pf_a_default = 0.05; else; pf_a_default = 0.5; end
        Pf_a_std_rel  = get_field_default(ctrl_const, 'Pf_a_std_rel', pf_a_default);
        Pf_da_std_rel = get_field_default(ctrl_const, 'Pf_da_std_rel', 1 / 40000);
        gain_order = get_field_default(ctrl_const, 'gain_order', 2);
        rate_damp = logical(get_field_default(ctrl_const, 'rate_damp', false));  rho_used = ones(3, 1);  q55_damp = zeros(3, 1);
        assert(any(gain_order == [1 2 3]), 'motion_control_law_eq17_5state_noap:gainOrder', 'gain_order must be 1|2|3.');
        Pf_dda_std_rel = get_field_default(ctrl_const, 'Pf_dda_std_rel', 1 / 40000^2 * 40000 / 400);   % per step^2: rate may change by (a_nom/400)/400 steps   % runs start in a hold (r = 0): the rate prior is ~0; a_nom/400 read the first samples as a slope (start transient, measured 2026-09-21)

        if isfield(params, 'wall')
            w_hat_n = params.wall.w_hat;  pz_wall = params.wall.pz;
            enable_wall = params.wall.enable_wall_effect > 0.5;
        else
            w_hat_n = [0; 0; 1];  pz_wall = 0;  enable_wall = false;
        end

        % --- seed: far-field physics (a_nom) or cheat init at the true start height ---
        switch seed_mode
            case 'anom'
                a_x_init = a_nom_p * ones(3, 1);
            case 'true_h0'
                p0_init = params.common.p0(:);
                h_bar_init = max((dot(p0_init, w_hat_n) - pz_wall) / R_radius, 1.001);
                [c_para0, c_perp0] = calc_correction_functions(h_bar_init);
                a_x_init = [a_nom_p / c_para0; a_nom_p / c_para0; a_nom_p / c_perp0];
            otherwise
                error('motion_control_law_eq17_5state_noap:seedMode', 'seed_mode must be anom|true_h0.');
        end
        sv = get_field_default(ctrl_const, 'seed_value', []);
        if ~isempty(sv); a_x_init = sv(:); end

        % --- P[0]: position/memory block by DARE (driven, has a fixed point), gain block by prior ---
        x_e_per_axis = zeros(N_STATE, 3);
        x_e_per_axis(4, :) = a_x_init.';
        P_per_axis = cell(3, 1);
        for ax = 1:3
            s2T = 4 * kBT * a_x_init(ax);
            P0 = zeros(N_STATE);
            if ma2_aug
                F5 = [0 1 0 0 0; 0 0 1 0 0; alpha * nw_mcorr, 0, lambda_c, -alpha, -alpha; 0 0 0 0 0; 0 0 0 1 0];
                g5 = [0; 0; -1; 1; 0];
                Q5 = s2T * (g5 * g5.') + (~nw_mcorr) * alpha^2 * sigma2_n_s(ax) * ([0;0;1;0;0] * [0 0 1 0 0]);
                P5 = solve_dare_kf_local(F5, [1 0 0 0 0], Q5, sigma2_n_s(ax));
                idx = [1 2 3 6 7];
                P0(idx, idx) = P5;
            else
                F3 = [0 1 0; 0 0 1; alpha * nw_mcorr, 0, lambda_c];
                Q3 = zeros(3);
                Q3(3, 3) = s2T * (1 + 2 * alpha^2) + (~nw_mcorr) * alpha^2 * sigma2_n_s(ax);
                P0(1:3, 1:3) = solve_dare_kf_local(F3, [1 0 0], Q3, sigma2_n_s(ax));
            end
            P0(4, 4) = (Pf_a_std_rel * a_nom_p)^2;
            if gain_order >= 2; P0(5, 5) = (Pf_da_std_rel * a_nom_p)^2; end
            if gain_order >= 3; P0(8, 8) = (Pf_dda_std_rel * a_nom_p)^2; end
            P_per_axis{ax} = P0;
        end

        % --- IIR states (prefill), whitening delay, buffers ---
        dx_bar_m = zeros(3, 1);
        sigma2_dxr_hat = 4 * kBT * a_x_init * C_dpmr + C_n * sigma2_n_s;
        a_xm_km1 = a_x_init;
        pd_km1 = pd;  pd_km2 = pd;
        f_d_km1 = zeros(3, 1);  f_d_km2 = zeros(3, 1);  f_d_km3 = zeros(3, 1);
        a_ctrl_km1 = a_x_init;  a_ctrl_km2 = a_x_init;
        a_post_km1 = a_x_init;  a_post_km2 = a_x_init;
        res1_km1 = zeros(3, 1);  u_c_km1 = zeros(3, 1);
        a_true_km1 = a_x_init;  a_true_km2 = a_x_init;
        aprime_true_km1 = zeros(3, 1);
        k_step = 1;

        f_d = zeros(3, 1);
        ekf_out = [a_x_init(1); a_x_init(3); a_x_init(2); 0];
        if nargout >= 3
            diag = empty_diag_noap();
            diag.f_d = f_d;  diag.a_hat = a_x_init;  diag.sigma2_dxr_hat = sigma2_dxr_hat;
            diag.a_ctrl_used = a_x_init;
            for ax = 1:3; diag.P_a(ax) = P_per_axis{ax}(4, 4); diag.P_da(ax) = P_per_axis{ax}(5, 5); end
        end
        return;
    end

    % ------------------------------------------------------------------
    % Per-step inputs
    % ------------------------------------------------------------------
    delta_x_m = pd_km2 - p_m;                % y1[k] = pd[k-d] - p_m[k]
    pd_kp1    = pd + del_pd;
    if enable_wall
        h_bar = (dot(p_m, w_hat_n) - pz_wall) / R_radius;
    else
        h_bar = Inf;
    end
    t_now = (k_step - 1) * Ts;

    % --- variance-channel readout (paper 2025 Eq.9-13) and whitening ---
    dx_bar_m_new = (1 - a_pd) * dx_bar_m + a_pd * delta_x_m;
    dx_r = delta_x_m - dx_bar_m_new;
    sigma2_dxr_hat_new = (1 - a_cov) * sigma2_dxr_hat + a_cov * dx_r.^2;
    a_xm = (sigma2_dxr_hat_new - C_n * sigma2_n_s) / (C_dpmr * 4 * kBT);
    if y2_oracle
        assert(~isempty(a_true_diag), 'motion_control_law_eq17_5state_noap:oracle', 'y2_oracle needs a_true_diag (7th arg).');
        u_or = a_true_km2 + sqrt(2) * (a_true_km2 + xi_per_axis) .* randn(3, 1);   % white, mean a_true[k-d], Isserlis marginal
        y2_fed = a_cov * u_or;  h2_scale = a_cov;
    elseif y2_whiten
        y2_fed = a_xm - (1 - a_cov) * a_xm_km1;        % = a_cov * u[k]
        h2_scale = a_cov;
    else
        y2_fed = a_xm;
        h2_scale = 1;
    end

    % --- gates (as the siblings) ---
    G1 = (t_now < t_warmup_kf);
    G2v = ((sigma2_dxr_hat_new - C_n * sigma2_n_s) <= 0);
    G3 = (h_bar < h_bar_safe);
    G_flags = [repmat(G1, 1, 3); G2v.'; repmat(G3, 1, 3)];
    gate_off = G1 | G2v.' | G3;

    a_prior_law = x_e_per_axis(4, :).';      % a_hat[k-1|k-1]
    if any(strcmp(lf_mode, {'ljung', 'q55'}))
        % relative drift rate of the gain from the COMMAND (height + speed), per step
        h_d   = max((dot(pd, w_hat_n) - pz_wall) / R_radius, 1.001);
        hd_dot = abs(dot(del_pd, w_hat_n) / R_radius) / Ts;               % [R/s]
        if strcmp(lf_slope, 'exact')
            [~, ~, dv] = calc_correction_functions(h_d, true);
            slope = abs(dv.K_h_perp);                                      % |d ln a / d h_bar| of the plane law
        else
            slope = 1 / (h_d * max(h_d - 1, 1e-3));                        % far-field log-slope
        end
        r_rel = hd_dot * slope * Ts;                                       % |d ln a / dt| * Ts
        for ax = 1:3
            a_i = max(a_prior_law(ax), 1e-3 * a_nom_p);
            IF_i = if_eff_eval(IF_abc_w, C_dpmr, C_n, kBT, a_i, sigma2_n_s(ax));
            J_i = a_i^2 / (2 * IF_i * (a_i + xi_per_axis(ax))^2);       % relative Fisher rate per whitened sample
            T_star = (2 * J_i * r_rel^2)^(-1/3);                           % steps
            if rate_damp
                rho_used(ax) = 1 - 1 / max(T_star, 1);
                q55_damp(ax) = (1 - rho_used(ax)^2) * (r_rel * a_i)^2;
            end
            if strcmp(lf_mode, 'ljung')
                lf_used(ax) = min(1, max(lf_min, 1 - 1 / T_star));
            else
                lf_used(ax) = 1;
                q55_used(ax) = (r_rel * a_i)^2 / T_star;                   % rate-change budget per step (RW over T* steps)
            end
        end
    end
    H_full = [1 0 0 0 0 0 0 0; h2_scale * [0 0 0 1 -d_delay 0 0 0]];
    if gain_order >= 3; H_full(2, 8) = h2_scale * d_delay * (d_delay + 1) / 2; end   % a[k-d] = a - d*da + d(d+1)/2*dda
    if gain_order == 1; H_full(2, 5) = 0; end
    H_y1 = H_full(1, :);

    % ------------------------------------------------------------------
    % Old ordering (upd_before_law = false): law first, with the k-1 posterior
    % ------------------------------------------------------------------
    if ~upd_before_law
        [f_d, a_ctrl] = local_law(a_prior_law, a_ctrl_override, has_override, pd_kp1, pd, pd_km2, delta_x_m, ...
                                  a_ctrl_km1, a_ctrl_km2, f_d_km1, f_d_km2, lambda_c, alpha);
    end

    % ------------------------------------------------------------------
    % Predict (k-1 -> k) and update with y[k], per axis
    % ------------------------------------------------------------------
    x_post_all = zeros(N_STATE, 3);  P_post_all = cell(3, 1);
    innov1_v = zeros(3, 1);  innov2_v = zeros(3, 1);  S11_v = zeros(3, 1);  S22_v = zeros(3, 1);
    K_a_y2_v = zeros(3, 1);  K_dx_y1_v = zeros(3, 1);  K_a_y1_v = zeros(3, 1);  w_y2_v = zeros(3, 1);
    for ax = 1:3
        x_curr = x_e_per_axis(:, ax);  P_curr = P_per_axis{ax};
        if strcmp(lf_mode, 'ljung')
            sc = ones(N_STATE, 1);  sc(lf_slots) = 1 / sqrt(lf_used(ax));  P_curr = (sc * sc.') .* P_curr;
        elseif lambda_f < 1
            P_curr = lf_scale .* P_curr;                                  % selective forgetting on the gain block
        end
        a_km1 = x_curr(4);

        if fe_km1
            F_dx  = f_d_km1(ax) + alpha * (f_d_km2(ax) + f_d_km3(ax));
            dF_dx = alpha * (f_d_km2(ax) + 2 * f_d_km3(ax));
        else
            F_dx  = f_d(ax) + alpha * (f_d_km1(ax) + f_d_km2(ax));       % legacy one-step lead
            dF_dx = alpha * (f_d_km1(ax) + 2 * f_d_km2(ax));
        end
        F_e = zeros(N_STATE);
        F_e(1, 2) = 1;  F_e(2, 3) = 1;
        F_e(3, 3) = lambda_c;  F_e(3, 4) = -F_dx;  F_e(3, 5) = dF_dx;
        F_e(4, 4) = 1;  F_e(5, 5) = 1;
        if gain_order >= 2; F_e(4, 5) = 1; end
        if rate_damp; F_e(5, 5) = rho_used(ax); end
        if gain_order >= 3; F_e(5, 8) = 1;  F_e(8, 8) = 1; end
        if nw_mcorr; F_e(3, 1) = alpha; end
        s2T = 4 * kBT * a_km1;
        if q_gain_true; s2T = 4 * kBT * a_true_km1(ax); end
        if ma2_aug
            F_e(3, 6) = -alpha;  F_e(3, 7) = -alpha;  F_e(7, 6) = 1;
            g = [0; 0; -1; 0; 0; 1; 0; 0];
            if q_aprime_true && q_aprime_cross; g(4) = aprime_true_km1(ax); end   % Q44 = a'^2 Q33, Q34 = -a' Q33 (+ Q46 = a' Q33)
            Q = s2T * (g * g.');
        else
            Q = zeros(N_STATE);
            Q(3, 3) = 4 * kBT * (a_km1 + alpha^2 * (a_post_km1(ax) + a_post_km2(ax)));
        end
        if ~nw_mcorr; Q(3, 3) = Q(3, 3) + alpha^2 * sigma2_n_s(ax); end
        if q_aprime_true && ~ma2_aug && q_aprime_cross
            ap = aprime_true_km1(ax);
            Q(3, 4) = -ap * Q(3, 3);  Q(4, 3) = Q(3, 4);  Q(4, 4) = ap^2 * Q(3, 3);
        end
        if q_aprime_true && ~q_aprime_cross                                 % discriminator arm: Q44 only, no cross terms
            Q(4, 4) = Q(4, 4) + aprime_true_km1(ax)^2 * Q(3, 3);
        end
        if q44_r2_frac > 0 && isfinite(r2a_km1(ax)); Q(4, 4) = Q(4, 4) + q44_r2_frac * r2a_km1(ax); end   % gain random walk, memory 1/sqrt(rho) steps
        if q44_aprime_cmd; Q(4, 4) = Q(4, 4) + (aprime_true_km1(ax) * del_pd_km1(ax))^2; end   % the gain change the commanded step causes
        if q44_lag_budget > 0 && isfinite(r2a_km1(ax))                 % memory = lag budget / gain change per step
            Q(4, 4) = Q(4, 4) + r2a_km1(ax) * (aprime_true_km1(ax) * del_pd_km1(ax) / (q44_lag_budget * max(a_km1, 1e-3 * a_nom_p)))^2;
        end
        if q44_mse_opt > 0 && isfinite(r2a_km1(ax))                    % memory that minimises lag^2 + noise^2
            v_a = abs(aprime_true_km1(ax) * del_pd_km1(ax));
            l_opt = min(q44_mse_opt * (4 * v_a^2 / r2a_km1(ax))^(1/3), 0.5);
            Q(4, 4) = Q(4, 4) + r2a_km1(ax) * l_opt^2;
        end
        if q44_ratio_rel > 0                                            % Q/R ratio with P0 shrinking with the gain
            P0r = (q44_ratio_rel * max(a_km1, 1e-3 * a_nom_p))^2;
            Q(4, 4) = Q(4, 4) + P0r^2 * q44r_I(ax);
        end
        if q44_ratio_std > 0
            P0r = (q44_ratio_std * a_nom_p)^2;  Ir = q44r_I(ax);  if q44_ratio_cap; Ir = min(Ir, 1 / P0r); end
            Q(4, 4) = Q(4, 4) + P0r^2 * Ir;                           % puts back what the last update removed: P44 held at P0
        end   % puts back what the last update removed: P44 held at P0
        if strcmp(lf_mode, 'q55'); Q(5, 5) = Q(5, 5) + q55_used(ax); end
        if rate_damp; Q(5, 5) = Q(5, 5) + q55_damp(ax); end

        % deterministic-map predict with the known inputs
        x_pred = zeros(N_STATE, 1);
        x_pred(1) = x_curr(2);  x_pred(2) = x_curr(3);
        x_pred(3) = lambda_c * x_curr(3);
        if ma2_aug;    x_pred(3) = x_pred(3) - alpha * (x_curr(6) + x_curr(7));  x_pred(7) = x_curr(6); end
        if nw_mcorr;   x_pred(3) = x_pred(3) - alpha * res1_km1(ax); end
        if corr_input; x_pred(3) = x_pred(3) - u_c_km1(ax); end
        x_pred(4) = x_curr(4);  x_pred(5) = x_curr(5);
        if gain_order >= 2; x_pred(4) = x_pred(4) + x_curr(5); end
        if rate_damp; x_pred(5) = rho_used(ax) * x_curr(5); end
        if gain_order >= 3; x_pred(5) = x_pred(5) + x_curr(8);  x_pred(8) = x_curr(8); end
        P_pred = F_e * P_curr * F_e.' + Q;  P_pred = 0.5 * (P_pred + P_pred.');

        % measurement covariance
        a_for_R = max(x_pred(4), 1e-3 * a_nom_p);
        if y2_oracle
            R22_i = 2 * a_cov^2 * (a_for_R + xi_per_axis(ax))^2;                  % white oracle: IF = 1
        elseif y2_whiten
            IF_i = if_eff_eval(IF_abc_w, C_dpmr, C_n, kBT, a_for_R, sigma2_n_s(ax));
            R22_i = 2 * a_cov^2 * IF_i * (a_for_R + xi_per_axis(ax))^2;
        else
            IF_i = if_eff_eval(IF_abc_raw, C_dpmr, C_n, kBT, a_for_R, sigma2_n_s(ax));
            R22_i = K_var * IF_i * (a_for_R + xi_per_axis(ax))^2;
        end
        % y2 gauge sensitivity (mirror): the measurement map is h2(x) = a_cov*E[u | a = a_hat] = a_cov*(a - d*da), so the
        % PREDICTED measurement keeps the full map; only the Jacobian row is scaled by s_i = dE[u]/da |_{g=1} < 1
        % (2026-09-23: scaling the prediction too was a bug -- it left (1-s)*a_hat in the innovation at the truth).
        H_jac = H_full;
        if y2_mirror
            s_i = y2m_sT + y2m_cN * sigma2_n_s(ax) / (4 * kBT * a_for_R);   % gauge sensitivity dE[a_xm]/da at g = 1
            H_jac(2, :) = s_i * H_jac(2, :);
        end
        if gate_off(ax)
            H_use = H_y1;  H_jac_use = H_y1;  y_use = delta_x_m(ax);  R_use = sigma2_n_s(ax);
        else
            H_use = H_full;  H_jac_use = H_jac;  y_use = [delta_x_m(ax); y2_fed(ax)];  R_use = [sigma2_n_s(ax), 0; 0, R22_i];
        end
        y_pred = H_use * x_pred;                       % full map (level term unscaled)
        if y2_mirror && ~gate_off(ax)                  % mainline convention: the delayed-rate term -d*da is scaled like its Jacobian
            y_pred(2) = y_pred(2) + h2_scale * (1 - s_i) * d_delay * x_pred(5);
        end
        innov  = y_use - y_pred;
        S_inn  = H_jac_use * P_pred * H_jac_use.' + R_use;  S_inn = 0.5 * (S_inn + S_inn.');
        K_kf   = (P_pred * H_jac_use.') / S_inn;       % Jacobian
        if G1; K_kf(4, :) = 0; K_kf(5, :) = 0; end
        x_post = x_pred + K_kf * innov;
        ImKH = eye(N_STATE) - K_kf * H_jac_use;
        P_post = ImKH * P_pred * ImKH.' + K_kf * R_use * K_kf.';  P_post = 0.5 * (P_post + P_post.');
        if ~ma2_aug; P_post(6:7, :) = 0; P_post(:, 6:7) = 0; x_post(6:7) = 0; end
        if gain_order < 3; P_post(8, :) = 0; P_post(:, 8) = 0; x_post(8) = 0; end
        if gain_order < 2; P_post(5, :) = 0; P_post(:, 5) = 0; x_post(5) = 0; end
        if (q44_ratio_std > 0 || q44_ratio_rel > 0) && P_pred(4, 4) > 0 && P_post(4, 4) > 0; q44r_I(ax) = max(0, 1 / P_post(4, 4) - 1 / P_pred(4, 4)); end   % this step's information on a, both channels
        if lf_info_beta > 0 && P_pred(4, 4) > 0                       % lambda_k = max(exp(-beta s_k), floor), s_k = share of P44 this step removed
            s_k = max(0, 1 - P_post(4, 4) / P_pred(4, 4));
            lam_k = max(exp(-lf_info_beta * s_k), lf_info_floor);
            dv = ones(N_STATE, 1);  dv(4) = 1 / sqrt(lam_k);  P_post = (dv * dv.') .* P_post;  P_post = 0.5 * (P_post + P_post.');
        end

        x_post_all(:, ax) = x_post;  P_post_all{ax} = P_post;
        innov1_v(ax) = innov(1);  S11_v(ax) = S_inn(1, 1);
        K_dx_y1_v(ax) = K_kf(3, 1);  K_a_y1_v(ax) = K_kf(4, 1);
        if ~gate_off(ax); innov2_v(ax) = innov(2); K_a_y2_v(ax) = K_kf(4, 2); S22_v(ax) = S_inn(2, 2);  w_y2_v(ax) = H_jac_use(2, 4)^2 / R22_i;  r2a_km1(ax) = R22_i / H_jac_use(2, 4)^2; end
    end
    x_e_per_axis = x_post_all;  P_per_axis = P_post_all;
    a_post = x_e_per_axis(4, :).';  da_post = x_e_per_axis(5, :).';

    % ------------------------------------------------------------------
    % New ordering: law with the fresh posterior a_hat[k|k]
    % ------------------------------------------------------------------
    if upd_before_law
        [f_d, a_ctrl] = local_law(a_post, a_ctrl_override, has_override, pd_kp1, pd, pd_km2, delta_x_m, ...
                                  a_ctrl_km1, a_ctrl_km2, f_d_km1, f_d_km2, lambda_c, alpha);
    end

    % --- exact past-error input for the NEXT predict (S4):
    %     u_c[k] = f[k] (a_hat[k|k] - a_c[k]) + alpha * sum_i f[k-i] [(a_hat[k|k] - a_c[k-i]) - i da_hat[k|k]]
    if corr_input
        u_c_new = f_d .* (a_post - a_ctrl) ...
                + alpha * (f_d_km1 .* ((a_post - a_ctrl_km1) - 1 * da_post) ...
                         + f_d_km2 .* ((a_post - a_ctrl_km2) - 2 * da_post));
    else
        u_c_new = zeros(3, 1);
    end
    res1_new = delta_x_m - x_e_per_axis(1, :).';      % posterior y1 residual = E[n_x[k] | data]

    % ------------------------------------------------------------------
    % Bookkeeping
    % ------------------------------------------------------------------
    pd_km2 = pd_km1;  pd_km1 = pd;
    f_d_km3 = f_d_km2;  f_d_km2 = f_d_km1;  f_d_km1 = f_d;
    a_ctrl_km2 = a_ctrl_km1;  a_ctrl_km1 = a_ctrl;
    a_post_km2 = a_post_km1;  a_post_km1 = a_prior_law;   % a_hat[k-1|k-1] becomes the "k-2" buffer next call
    res1_km1 = res1_new;  u_c_km1 = u_c_new;
    if ~isempty(a_true_diag); a_true_km2 = a_true_km1;  a_true_km1 = a_true_diag(:); end
    if ~isempty(aprime_true_diag); aprime_true_km1 = aprime_true_diag(:); end
    del_pd_km1 = del_pd(:);
    dx_bar_m = dx_bar_m_new;  sigma2_dxr_hat = sigma2_dxr_hat_new;  a_xm_km1 = a_xm;
    k_step = k_step + 1;

    % ------------------------------------------------------------------
    % Output
    % ------------------------------------------------------------------
    h_bar_now = h_bar;  if ~enable_wall; h_bar_now = 0; end
    ekf_out = [a_post(1); a_post(3); a_post(2); h_bar_now];
    if nargout >= 3
        diag = empty_diag_noap();
        for ax = 1:3
            diag.P_a(ax)   = P_per_axis{ax}(4, 4);
            diag.P_da(ax)  = P_per_axis{ax}(5, 5);
            diag.P_dx(ax)  = P_per_axis{ax}(3, 3);
            diag.P_dx1(ax) = P_per_axis{ax}(1, 1);
        end
        diag.sigma2_dxr_hat = sigma2_dxr_hat_new;  diag.a_xm = a_xm;  diag.y2_fed = y2_fed;
        diag.delta_x_m = delta_x_m;  diag.dx_r = dx_r;
        diag.innovation_y1 = innov1_v;  diag.innovation_y2 = innov2_v;  diag.S_y1 = S11_v;  diag.S_y2 = S22_v;  diag.w_y2 = w_y2_v;
        diag.K_kf_a_y2 = K_a_y2_v;  diag.K_kf_dx_y1 = K_dx_y1_v;  diag.K_kf_a_y1 = K_a_y1_v;
        diag.delta_a_hat = da_post;  diag.a_hat = a_post;  diag.a_ctrl_used = a_ctrl;
        diag.delta_x_hat_1 = x_e_per_axis(1, :).';  diag.delta_x_hat_3 = x_e_per_axis(3, :).';
        diag.gate_active_per_axis = gate_off(:);  diag.guards_individual = G_flags;
        diag.h_bar = h_bar;  diag.f_d = f_d;  diag.lambda_f_used = lf_used;
    end
end

%% =================== Local Helpers ===================
function [f_d, a_ctrl] = local_law(a_use, a_ctrl_override, has_override, pd_kp1, pd, pd_km_d, delta_x_m, ...
                                   a_ctrl_km1, a_ctrl_km2, f_d_km1, f_d_km2, lambda_c, alpha)
%LOCAL_LAW  Vpersonal active law (d = 2): f = a^-1 { Dx_d^d + alpha [dx_m - sum a_c[k-i] f[k-i]] }.
    if has_override; a_ctrl = a_ctrl_override; else; a_ctrl = a_use; end
    sum_a_fd_past = a_ctrl_km1 .* f_d_km1 + a_ctrl_km2 .* f_d_km2;
    f_d = (1 ./ a_ctrl) .* (pd_kp1 - lambda_c * pd - alpha * pd_km_d + alpha * delta_x_m - alpha * sum_a_fd_past);
end

function IF = if_eff_eval(IF_abc, C_dpmr, C_n, kBT, a, sigma2_nx)
%IF_EFF_EVAL  Colour-inflation factor (R22_derivation S4-S6); IF_abc at s = 1-a_cov (raw) or s = 1 (whitened).
    sxT = 4 * kBT * a;
    num = sxT^2 * IF_abc(1) + 2 * sxT * sigma2_nx * IF_abc(2) + sigma2_nx^2 * IF_abc(3);
    den = (C_dpmr * sxT + C_n * sigma2_nx)^2;
    IF  = 1 + 2 * num / den;
end

function P_post = solve_dare_kf_local(F, H, Q, R)
%SOLVE_DARE_KF_LOCAL  Discrete-time KF Riccati steady state (fixed-point iteration, driven block only).
    n = size(F, 1);  P_post = eye(n) * max(Q(:));  max_iter = 20000;  tol = 1e-14 * max(1, max(Q(:)));
    for it = 1:max_iter
        P_pred = F * P_post * F.' + Q;  P_pred = 0.5 * (P_pred + P_pred.');
        S = H * P_pred * H.' + R;  K = (P_pred * H.') / S;
        P_new = (eye(n) - K * H) * P_pred;  P_new = 0.5 * (P_new + P_new.');
        if max(abs(P_new(:) - P_post(:))) < tol; P_post = P_new; return; end
        P_post = P_new;
    end
    warning('motion_control_law_eq17_5state_noap:dare', 'P[0] position-block DARE did not converge in %d iterations.', max_iter);
end

function v = get_field_default(s, name, default)
    if isfield(s, name) && ~isempty(s.(name)); v = s.(name); else; v = default; end
end

function d = empty_diag_noap()
    z = zeros(3, 1);
    d = struct('sigma2_dxr_hat', z, 'a_xm', z, 'y2_fed', z, 'delta_x_m', z, 'dx_r', z, ...
               'innovation_y1', z, 'innovation_y2', z, 'S_y1', z, 'S_y2', z, 'w_y2', z, ...
               'K_kf_a_y2', z, 'K_kf_dx_y1', z, 'K_kf_a_y1', z, ...
               'P_a', z, 'P_da', z, 'P_dx', z, 'P_dx1', z, 'P77', z, 'Q77', z, ...
               'x_D_hat', z, 'delta_a_hat', z, 'a_hat', z, 'a_ctrl_used', z, ...
               'delta_x_hat_1', z, 'delta_x_hat_3', z, ...
               'gate_active_per_axis', false(3, 1), 'guards_individual', false(3, 3), ...
               'h_bar', 0, 'f_d', z, 'lambda_f_used', ones(3, 1));
end
