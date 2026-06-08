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
%       (no IF_eff_calibrated override: IF_eff is computed exactly per-step in
%        the controller from IF_abc below.)
%
%   --------- Outputs (struct ctrl_const) ---------
%       lambda_c, d, a_cov, a_pd, kBT, sigma2_n_s
%       C_dpmr, C_n        - FULL a_pd closed form (scalars)
%       K_var (R22_prefactor) = 2*a_cov/(2-a_cov)
%       IF_abc = [A;B;C]   - s-weighted autocorr sums for exact per-step IF_eff
%                            (R22_derivation S4-S6); IF_eff = 1 + 2*(sxT^2 A +
%                            2 sxT snx B + snx^2 C)/(C_dpmr sxT + C_n snx)^2
%       xi_per_axis       - (C_n/C_dpmr)*sigma2_n_s/(4*kBT)     [3x1]
%       t_warmup_kf, h_bar_safe, iir_warmup_mode
%
%   Validation reference (lambda_c=0.7, a_pd=0.05), verified by MCP eval:
%       C_dpmr_full = 3.1610  (vs a_pd->0 simplified 3.9608)
%       C_n_full    = 1.1093  (vs a_pd->0 simplified 1.1765)
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
    % IF color-inflation: EXACT closed form (R22_derivation.tex S4-S6).
    %   IF_eff = 1 + 2*sum_{tau>=1} rho_dpmr^2(tau)*s^tau, s=1-a_cov, depends on
    %   the per-axis ratio r = sigma2_dxT/sigma2_nx (= 4kBT*a/sigma2_nx). Since a
    %   is time-varying, the controller evaluates IF_eff per-step from a_hat. Here
    %   we precompute the three s-weighted autocorrelation sums (depend only on
    %   lambda_c, a_pd, a_cov) so the controller's per-step IF is O(1):
    %       A = sum R_fT(tau)^2 s^tau, B = sum R_fT R_fN s^tau, C = sum R_fN^2 s^tau
    %       IF_eff = 1 + 2*(sxT^2*A + 2*sxT*snx*B + snx^2*C)/(C_dpmr*sxT+C_n*snx)^2
    %   Replaces the old single-pole approximation + IF_eff_calibrated override.
    %   Numerically exact: this IS the brute force R22_derivation validates to
    %   <8e-15 against its analytic 3-term geometric closed form (S6).
    % ------------------------------------------------------------
    [IF_abc_A, IF_abc_B, IF_abc_C, RfT0, RfN0] = compute_if_abc(lc, apd, a_cov);
    % Self-check: R_fT(0)=C_dpmr, R_fN(0)=C_n (independent route to the same consts).
    if abs(RfT0 - C_dpmr) > 1e-5 * C_dpmr || abs(RfN0 - C_n) > 1e-5 * C_n
        warning('build_eq17_6state_constants:IFselfcheck', ...
                ['IF autocorr self-check mismatch: R_fT(0)=%.6f vs C_dpmr=%.6f ; ' ...
                 'R_fN(0)=%.6f vs C_n=%.6f'], RfT0, C_dpmr, RfN0, C_n);
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
    ctrl_const.IF_abc          = [IF_abc_A; IF_abc_B; IF_abc_C];  % s-weighted autocorr sums for exact per-step IF_eff
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


function [A, B, C, RfT0, RfN0] = compute_if_abc(lc, apd, a_cov)
%COMPUTE_IF_ABC  s-weighted autocorrelation sums for the exact IF_eff
%   (R22_derivation.tex S4-S6), via F_T/F_N impulse responses (toolbox-free).
%
%   F_T = (1-apd)(1-q)q^3[1+(1-lc)q+(1-lc)q^2] / [(1-(1-apd)q)(1-lc q)],  q=z^-1
%   F_N = (1-apd)(1-q)^2[1+(1-lc)q+(1-lc)q^2] / [same den]   (z^3 cancels q^3)
%
%   Returns  A = sum_{tau>=1} R_fT(tau)^2 s^tau,  B = sum R_fT(tau) R_fN(tau) s^tau,
%   C = sum R_fN(tau)^2 s^tau  (s = 1-a_cov), plus RfT0=R_fT(0)=C_dpmr and
%   RfN0=R_fN(0)=C_n for an independent self-check. Poles (1-apd, lc) < 1, so the
%   impulse responses decay geometrically; N/Tmax are set for machine precision.
    s = 1 - a_cov;
    q1 = [1, -1]; q3 = [0, 0, 0, 1]; thnum = [1, (1 - lc), (1 - lc)];
    numFT = (1 - apd) * conv(conv(q1, q3), thnum);
    numFN = (1 - apd) * conv(conv(q1, q1), thnum);
    den   = conv([1, -(1 - apd)], [1, -lc]);
    N = 8000;
    imp = [1; zeros(N - 1, 1)];
    hFT = filter(numFT, den, imp);
    hFN = filter(numFN, den, imp);
    Tmax = 600;
    RfT0 = sum(hFT .^ 2);
    RfN0 = sum(hFN .^ 2);
    A = 0; B = 0; C = 0;
    for t = 1:Tmax
        RfT = sum(hFT(1:end - t) .* hFT(1 + t:end));
        RfN = sum(hFN(1:end - t) .* hFN(1 + t:end));
        st  = s ^ t;
        A = A + RfT ^ 2 * st;
        B = B + RfT * RfN * st;
        C = C + RfN ^ 2 * st;
    end
end
