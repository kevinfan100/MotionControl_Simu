function ctrl_const = build_eq17_6state_constants(opts)
%BUILD_EQ17_6STATE_CONSTANTS Offline scalar-constants builder for the
%   RevisedControl_Vpersonal 6-state EKF controller.
%
%   ctrl_const = build_eq17_6state_constants(opts)
%
%   Companion to motion_control_law_eq17_6state.m. Computes the
%   time-invariant scalar constants once at simulation init; the per-step
%   controller pulls them from ctrl_const.
%
%   Differs from build_eq17_constants (7-state) in two ways:
%     1. C_dpmr / C_n use the FULL a_pd-dependent closed form
%        (reference/eq17_analysis/derivation/Cdpmr_Cn_derivation.pdf), not
%        the a_pd->0 simplification (2 + 1/(1-lc^2)) / (2/(1+lc)).
%     2. iir_warmup_mode defaults to 'prefill' and t_warmup_kf to 0 (the
%        three-pillar equilibrium init: prefill + Riccati Pf + G1-off).
%
%   --------- Inputs (struct opts) ---------
%       opts.lambda_c     - closed-loop pole, in (0,1).        [scalar, required]
%       opts.sigma2_n_s   - per-axis sensor noise variance     [3x1, um^2, required]
%       opts.kBT          - thermal energy (consistent units)  [scalar, required]
%       opts.a_cov        - EWMA variance estimator weight      (default 0.05)
%       opts.a_pd         - IIR LP weight for dx_bar_m mean     (default = a_cov)
%       opts.d            - measurement delay (steps)           (default 2)
%       opts.t_warmup_kf  - KF warmup time [sec]                (default 0, prefill)
%       opts.h_bar_safe   - safe h_bar threshold (Guard 3)      (default 1.5)
%       opts.iir_warmup_mode - 'prefill' (default) | 'legacy'
%       opts.IF_eff_calibrated - optional per-axis IF_eff       [3x1, override]
%
%   --------- Outputs (struct ctrl_const) ---------
%       lambda_c, d, a_cov, a_pd, kBT, sigma2_n_s
%       C_dpmr, C_n        - FULL a_pd closed form (scalars)
%       K_var (R22_prefactor) = 2*a_cov/(2-a_cov)
%       IF_eff, IF_eff_per_axis - color-inflation factor (s-weighted)
%       xi_per_axis       - (C_n/C_dpmr)*sigma2_n_s/(4*kBT)     [3x1]
%       t_warmup_kf, h_bar_safe, iir_warmup_mode
%
%   Validation reference (lambda_c=0.7, a_pd=0.05):
%       C_dpmr_full ~ 3.93  (vs a_pd->0 simplified 3.9608)
%       C_n_full    ~ 1.15  (vs a_pd->0 simplified 1.1765)
%
%   See also: motion_control_law_eq17_6state, build_eq17_constants

    if nargin < 1 || ~isstruct(opts)
        error('build_eq17_6state_constants:invalidInput', 'opts must be a struct.');
    end

    % ---------- Required-field checks ----------
    required = {'lambda_c', 'sigma2_n_s', 'kBT'};
    for k = 1:numel(required)
        f = required{k};
        if ~isfield(opts, f) || isempty(opts.(f))
            error('build_eq17_6state_constants:missingInput', 'opts.%s is required.', f);
        end
    end

    lambda_c = opts.lambda_c;
    if ~isscalar(lambda_c) || ~isfinite(lambda_c) || lambda_c <= 0 || lambda_c >= 1
        error('build_eq17_6state_constants:invalidLambda', ...
              'lambda_c must be a finite scalar in (0,1); got %g.', lambda_c);
    end

    sigma2_n_s = opts.sigma2_n_s(:);
    if numel(sigma2_n_s) ~= 3 || any(~isfinite(sigma2_n_s)) || any(sigma2_n_s <= 0)
        error('build_eq17_6state_constants:invalidSigma2', ...
              'sigma2_n_s must be a 3-element positive finite vector.');
    end

    kBT = opts.kBT;
    if ~isscalar(kBT) || ~isfinite(kBT) || kBT <= 0
        error('build_eq17_6state_constants:invalidKBT', ...
              'kBT must be a positive finite scalar; got %g.', kBT);
    end

    % ---------- Default-fill optional fields ----------
    a_cov = get_opt(opts, 'a_cov', 0.05);
    a_pd  = get_opt(opts, 'a_pd', a_cov);
    d     = get_opt(opts, 'd', 2);
    t_warmup_kf = get_opt(opts, 't_warmup_kf', 0);     % prefill default: no G1
    h_bar_safe  = get_opt(opts, 'h_bar_safe', 1.5);
    iir_warmup_mode = get_opt(opts, 'iir_warmup_mode', 'prefill');

    if d ~= round(d) || d < 0
        error('build_eq17_6state_constants:invalidDelay', ...
              'opts.d must be a non-negative integer; got %g.', d);
    end

    % ------------------------------------------------------------
    % C_dpmr / C_n : FULL a_pd-dependent closed form
    %   (Cdpmr_Cn_derivation.pdf S3/S4; verified a_pd->0 reduces to
    %    2+1/(1-lc^2) and 2/(1+lc).)
    % ------------------------------------------------------------
    lc   = lambda_c;
    apd  = a_pd;
    one_m_apd = 1 - apd;
    denom_pole = 1 - one_m_apd * lc;          % 1 - (1-a_pd)*lc, shared pole product

    C_dpmr = one_m_apd^2 * ( ...
               2 * one_m_apd * (1 - lc) / denom_pole ...
             + (2 / (2 - apd)) * 1 / ((1 + lc) * denom_pole) );

    C_n = (2 * one_m_apd^2 / (2 - apd)) * ( ...
               1 ...
             + one_m_apd^2 * apd * (1 - lc) / denom_pole ...
             + (1 - lc)^2 / ((1 + lc) * denom_pole) );

    % ------------------------------------------------------------
    % K_var (R22 prefactor) = g_gauss * g_E = 2 * a_cov/(2-a_cov)
    %   (Isserlis Var(X^2)=2sigma^4 times EWMA noise gain a_cov/(2-a_cov),
    %    R22_derivation.pdf S3.)
    % ------------------------------------------------------------
    K_var = 2 * a_cov / (2 - a_cov);

    % ------------------------------------------------------------
    % IF_eff : self-correlation inflation factor (s-weighted, s=1-a_cov).
    %   Simplified single-pole (lambda_c) closed form (matches
    %   build_eq17_constants Option A 'A_MA2_full'); per-axis override
    %   accepted via opts.IF_eff_calibrated for X2a-style calibration.
    % ------------------------------------------------------------
    denom_e = 1 + 2 * (1 - lc)^2;
    rho_e_1 = (1 - lc) * (2 - lc) / denom_e;
    rho_e_2 = (1 - lc) / denom_e;
    Var_dx_over_sig_e = (1 + 2*lc*rho_e_1 + 2*lc^2*rho_e_2) / (1 - lc^2);
    inv_var_ratio = 1 / Var_dx_over_sig_e;
    rho_dx_1 = lc + inv_var_ratio * (rho_e_1 + lc*rho_e_2);
    rho_dx_2 = lc * rho_dx_1 + inv_var_ratio * rho_e_2;
    s_ewma = 1 - a_cov;
    IF_eff = 1 + 2 * (rho_dx_1^2 * s_ewma ...
                    + rho_dx_2^2 * s_ewma^2 / (1 - lc^2 * s_ewma));

    if isfield(opts, 'IF_eff_calibrated') && ~isempty(opts.IF_eff_calibrated)
        IF_eff_per_axis = opts.IF_eff_calibrated(:);
        if numel(IF_eff_per_axis) ~= 3 || any(~isfinite(IF_eff_per_axis)) || any(IF_eff_per_axis <= 0)
            error('build_eq17_6state_constants:invalidIFeff', ...
                  'IF_eff_calibrated must be a 3-element positive finite vector.');
        end
    else
        IF_eff_per_axis = IF_eff * ones(3, 1);
    end

    % ------------------------------------------------------------
    % xi : sensor-noise offset, (C_n/C_dpmr)*sigma2_n_s/(4*kBT)   [3x1]
    % ------------------------------------------------------------
    xi_per_axis = (C_n / C_dpmr) * sigma2_n_s / (4 * kBT);

    % ------------------------------------------------------------
    % Pack outputs
    % ------------------------------------------------------------
    ctrl_const = struct();
    ctrl_const.lambda_c        = lambda_c;
    ctrl_const.d               = d;
    ctrl_const.a_cov           = a_cov;
    ctrl_const.a_pd            = a_pd;
    ctrl_const.kBT             = kBT;
    ctrl_const.sigma2_n_s      = sigma2_n_s;
    ctrl_const.C_dpmr          = C_dpmr;
    ctrl_const.C_n             = C_n;
    ctrl_const.K_var           = K_var;
    ctrl_const.R22_prefactor   = K_var;             % alias (matches 7-state naming)
    ctrl_const.IF_eff          = IF_eff;
    ctrl_const.IF_eff_per_axis = IF_eff_per_axis;
    ctrl_const.xi_per_axis     = xi_per_axis;
    ctrl_const.t_warmup_kf     = t_warmup_kf;
    ctrl_const.h_bar_safe      = h_bar_safe;
    ctrl_const.iir_warmup_mode = iir_warmup_mode;
    ctrl_const.meta = struct( ...
        'derivation', 'RevisedControl_Vpersonal + Cdpmr_Cn_derivation.pdf (full a_pd)', ...
        'description', 'Offline scalar constants for 6-state Vpersonal EKF');
end


function v = get_opt(opts, name, default)
%GET_OPT Return opts.(name) if present and non-empty, else default.
    if isfield(opts, name) && ~isempty(opts.(name))
        v = opts.(name);
    else
        v = default;
    end
end
