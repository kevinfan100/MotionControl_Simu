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
%       opts.use_am_lpf   - post-LPF on a_xm -> a_m_det before KF  (default false)
%       opts.a_det        - EWMA weight for the a_m_det LPF        (default 0.005)
%       (no IF_eff_calibrated override: IF_eff is computed exactly per-step in
%        the controller from IF_abc below.)
%
%   --------- Outputs (struct ctrl_const) ---------
%       lambda_c, d, a_cov, a_pd, kBT, sigma2_n_s
%       C_dpmr, C_n        - FULL a_pd closed form (scalars)
%       K_var (R22_prefactor) = 2*a_cov/(2-a_cov)
%       var_da_increment_factor = 2/(1+lambda_c)  - closed-form increment factor
%                            for Var(delta_a_ram) (replaces the i.i.d. 2x)
%       IF_abc = [A;B;C]   - s-weighted autocorr sums for exact per-step IF_eff
%                            (R22_derivation S4-S6); IF_eff = 1 + 2*(sxT^2 A +
%                            2 sxT snx B + snx^2 C)/(C_dpmr sxT + C_n snx)^2
%                            s = 1-a_cov: Var of the EWMA OUTPUT (raw readout)
%       IF_abc_white = same sums at s = 1 (long-run / DC-power factor) for a
%                            WHITENED readout y2 = a_cov*u[k] (2026-08-27)
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
    use_am_lpf = get_opt(opts, 'use_am_lpf', false);   % post-LPF on a_xm -> a_m_det
    a_det      = get_opt(opts, 'a_det', 0.005);        % EWMA weight for a_m_det

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
    % a_m_det LPF: post-EWMA(a_det) on a_xm before the KF
    %   (am_lpf_r22_design.md). a_m_det[k]=(1-a_det)a_m_det[k-1]+a_det a_xm[k].
    %   R22_intrinsic scales by amlpf_var_factor = [a_det/(2-a_det)]*IF2, with
    %   IF2 = 1 + 2 s2/(1-s2), s2 = (1-a_det)(1-a_cov) -- the AR(1) approx of
    %   rho_axm(tau) ~ (1-a_cov)^tau (P3 V-ii validates; a-dependent fallback in
    %   design.md S3.3). use_am_lpf=false -> factor 1 (baseline, bit-identical).
    % ------------------------------------------------------------
    if use_am_lpf
        K_var2 = a_det / (2 - a_det);
        s2     = (1 - a_det) * (1 - a_cov);
        IF2    = 1 + 2 * s2 / (1 - s2);
        amlpf_var_factor = K_var2 * IF2;
    else
        amlpf_var_factor = 1;
    end

    % ------------------------------------------------------------
    % var_da_increment_factor: closed-form factor for Var(delta_a_ram).
    %   delta_a_ram[k] = a_xram[k+1]-a_xram[k] is the one-step increment of the
    %   closed-loop gain fluctuation a_xram = (a*K_h/R)*x_ram. For a stationary
    %   signal Var(increment) = 2(1-rho1)*Var(a_xram). Here x_ram is the
    %   closed-loop wall-normal tracking error: Var(x_ram) = C_dx*sigma2_dh with
    %   C_dx = 2+1/(1-lc^2), and lag-1 autocorr rho1 = lc + 2(1-lc)/C_dx, so
    %   1-rho1 = 1/((1+lc)*C_dx). The C_dx amplification and the (1-rho1) smoothing
    %   CANCEL: 2(1-rho1)*C_dx = 2/(1+lc). Hence
    %       Var(delta_a_ram) = [2/(1+lc)] * (a*K_h/R)^2 * sigma2_dh.
    %   (The earlier i.i.d. reading used factor 2, over-estimating by (1+lc)=1.7x
    %    at lc=0.7; direct measurement of the true gain increment gives
    %    emp/closed = 0.998. See eq17_6state_review_findings.md.)
    % ------------------------------------------------------------
    var_da_increment_factor = 2 / (1 + lc);

    % ------------------------------------------------------------
    % r22_delay_sum_factor: correction for the R22 d-step delay term.
    %   The code sums the two increment variances var(delta_a_ram[k-1]) +
    %   var(delta_a_ram[k-2]) as if independent. But the residual delay part is
    %   r2_delay = delta_a_ram[k-1] + delta_a_ram[k-2], which TELESCOPES:
    %       delta_a_ram[k-1] + delta_a_ram[k-2] = a_ram[k] - a_ram[k-2].
    %   Hence Var(r2_delay) = 2*V_a*(1-rho2) (rho2 = lag-2 autocorr of a_ram = of
    %   delta_x), not 2*[2*V_a*(1-rho1)] = independent-sum. Correction factor is
    %       (1-rho2) / (2*(1-rho1))  ~ +10.4% at lc=0.7   (audit 2026-07-10).
    %   For d=1 the single increment term is exact -> factor 1.
    %   rho_dx_1/rho_dx_2 use the same thermal-dominated epsilon MA(2) closed form
    %   as build_eq17_constants Option A (independent of a_pd); verified
    %   rho_dx_1=0.8515, rho_dx_2=0.6718 at lc=0.7.
    % ------------------------------------------------------------
    one_m_lc  = 1 - lc;
    denom_e   = 1 + 2 * one_m_lc^2;
    rho_e_1   = one_m_lc * (2 - lc) / denom_e;
    rho_e_2   = one_m_lc / denom_e;
    Var_dx_over_sig_e = (1 + 2*lc*rho_e_1 + 2*lc^2*rho_e_2) / (1 - lc^2);
    inv_var_ratio = 1 / Var_dx_over_sig_e;
    rho_dx_1  = lc + inv_var_ratio * (rho_e_1 + lc*rho_e_2);
    rho_dx_2  = lc * rho_dx_1 + inv_var_ratio * rho_e_2;
    if d == 2
        r22_delay_sum_factor = (1 - rho_dx_2) / (2 * (1 - rho_dx_1));
    else
        r22_delay_sum_factor = 1;                  % d=1 single term is exact
    end

    % ------------------------------------------------------------
    % IF color-inflation: EXACT closed form (R22_derivation.tex S4-S6).
    %   IF_eff = 1 + 2*sum_{tau>=1} rho_dpmr^2(tau)*s^tau, s=1-a_cov, depends on
    %   the per-axis ratio r = sigma2_dxT/sigma2_nx (= 4kBT*a/sigma2_nx). This
    %   is the variance of the EWMA OUTPUT sigma2_hat (raw readout). Since a
    %   is time-varying, the controller evaluates IF_eff per-step from a_hat. Here
    %   we precompute the three s-weighted autocorrelation sums (depend only on
    %   lambda_c, a_pd, a_cov) so the controller's per-step IF is O(1):
    %       A = sum R_fT(tau)^2 s^tau, B = sum R_fT R_fN s^tau, C = sum R_fN^2 s^tau
    %       IF_eff = 1 + 2*(sxT^2*A + 2*sxT*snx*B + snx^2*C)/(C_dpmr*sxT+C_n*snx)^2
    %   Replaces the old single-pole approximation + IF_eff_calibrated override.
    %   Numerically exact: this IS the brute force R22_derivation validates to
    %   <8e-15 against its analytic 3-term geometric closed form (S6).
    % ------------------------------------------------------------
    [IF_abc_A, IF_abc_B, IF_abc_C, RfT0, RfN0] = compute_if_abc(lc, apd, 1 - a_cov);
    % WHITENED counterpart (2026-08-27). The s^tau weight above is the overlap
    % of two EWMA kernel taps tau apart: IF_eff answers "Var(sigma2_hat)", the
    % variance of the EWMA OUTPUT (R22_derivation.tex, boxed eq:IF-def). A
    % controller that whitens the readout (y2_whiten = true) feeds the KF the
    % single increment y2 = a_cov*u[k] instead; the EWMA window is undone and
    % the averaging is now the KF's own (flat kernel for a constant parameter),
    % so the colour penalty is the long-run factor
    %     IF_white = 1 + 2*sum_{tau>=1} rho_u^2(tau)      (s = 1, DC-power ratio)
    % = same A/B/C with unit weights. It depends on (lambda_c, a_pd) only, so
    % a whitened R2 carries a_cov solely through its explicit a_cov^2 factor
    % (rule: H2 ~ a_cov, R2 ~ a_cov^2, K2*innov independent of a_cov).
    % Consumers select by their own y2_whiten flag; raw-readout controllers
    % (eq17_{4,5,6}state) keep IF_abc.
    [IF_abcw_A, IF_abcw_B, IF_abcw_C] = compute_if_abc(lc, apd, 1);
    % Self-check: R_fT(0)=C_dpmr, R_fN(0)=C_n (independent route to the same consts).
    if abs(RfT0 - C_dpmr) > 1e-5 * C_dpmr || abs(RfN0 - C_n) > 1e-5 * C_n
        warning('build_eq17_6state_constants:IFselfcheck', ...
                ['IF autocorr self-check mismatch: R_fT(0)=%.6f vs C_dpmr=%.6f ; ' ...
                 'R_fN(0)=%.6f vs C_n=%.6f'], RfT0, C_dpmr, RfN0, C_n);
    end

    % ------------------------------------------------------------
    % y2_ar1_phi: lag-1 autocorrelation of the a_xm estimator noise v
    %   (colored-y2 modeling, chat 2026-07-12). v is the deviation of the
    %   variance EWMA sigma2_dxr_hat from its mean: an AR(1) with pole
    %   (1-a_cov) driven by the COLORED squared residuals dx_r^2 (Wick:
    %   rho_s = rho_dxr^2), so rho_v(1) > 1-a_cov. Yule-Walker lag-1 AR(1)
    %   fit; regime spread over sigma2_dxT/sigma2_nx is < 0.003 so the
    %   thermal-dominated limit serves all axes. 0.9864 at lc=0.7,
    %   a_pd=a_cov=0.05. Consumed by motion_control_law_eq17_4state when
    %   ctrl_const.y2_noise_model = 'ar1' (5th state v).
    % ------------------------------------------------------------
    y2_ar1_phi = compute_y2_ar1_phi(lc, apd, a_cov);

    % ------------------------------------------------------------
    % y2_mirror_sT / y2_mirror_cN: sensitivity of the a_xm gauge at g=1
    %   (g-mismatch modeling). a_xm inverts the residual variance
    %   sigma2_dxr assuming the loop-gain ratio g = a/a_hat equals 1. At
    %   g != 1 the loop suppression changes the read residual variance, so
    %   E[a_xm|a,a_hat] scales as C_T(g)*s2T + C_N(g)*s2n over the g=1
    %   normalization. C_T(g)/C_N(g) are the closed-loop variance
    %   coefficients of the thermal (w) and sensor (n) noise sources, each
    %   an exact discrete-Lyapunov solve of the hold-phase loop (no Monte
    %   Carlo). The EKF needs the sensitivity dE[a_xm]/da at g=1 (NOT 1: the
    %   loop suppresses the fluctuations being read). Central difference at
    %   g=1 gives sT (thermal-dominated) and cN (sensor-mix correction).
    %   Self-check: C_T(1)=C_dpmr, C_N(1)=C_n (independent route to the same
    %   consts, matched < 1e-6 below).
    % ------------------------------------------------------------
    [y2_mirror_sT, y2_mirror_cN, CT1, CN1] = compute_y2_mirror(lc, apd, d);
    if abs(CT1 - C_dpmr) > 1e-5 * C_dpmr || abs(CN1 - C_n) > 1e-5 * C_n
        warning('build_eq17_6state_constants:mirrorSelfcheck', ...
                ['y2_mirror self-check mismatch: C_T(1)=%.6f vs C_dpmr=%.6f ; ' ...
                 'C_N(1)=%.6f vs C_n=%.6f'], CT1, C_dpmr, CN1, C_n);
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
    ctrl_const.use_am_lpf      = use_am_lpf;
    ctrl_const.a_det           = a_det;
    ctrl_const.amlpf_var_factor = amlpf_var_factor;
    ctrl_const.var_da_increment_factor = var_da_increment_factor;  % 2/(1+lc) closed form
    ctrl_const.r22_delay_sum_factor    = r22_delay_sum_factor;     % (1-rho2)/(2(1-rho1)) at d=2, 1 at d=1 (telescoped delay-sum)
    ctrl_const.IF_abc          = [IF_abc_A; IF_abc_B; IF_abc_C];  % s-weighted autocorr sums for exact per-step IF_eff (raw readout)
    ctrl_const.IF_abc_white    = [IF_abcw_A; IF_abcw_B; IF_abcw_C];  % s = 1 sums for the whitened readout (y2_whiten = true)
    ctrl_const.y2_ar1_phi      = y2_ar1_phi;    % lag-1 rho of a_xm noise (colored-y2 AR(1) fit)
    ctrl_const.y2_mirror_sT    = y2_mirror_sT;  % thermal-dominated sensitivity dE[a_xm]/da at g=1
    ctrl_const.y2_mirror_cN    = y2_mirror_cN;  % sensor-mix correction coefficient
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


function phi = compute_y2_ar1_phi(lc, apd, a_cov)
%COMPUTE_Y2_AR1_PHI  Lag-1 autocorr of the a_xm EWMA estimator noise v.
%   Same impulse-response machinery as compute_if_abc: dx_r ACF from the
%   thermal-path transfer F_T, squared via Wick (rho_s = rho_dxr^2), then the
%   EWMA output ACF is the two-sided geometric sum
%       gamma_v(tau) ~ sum_m (1-a_cov)^|m| * rho_s(|tau+m|)
%   and phi = gamma_v(1)/gamma_v(0). Asymptotic pole is (1-a_cov); the colored
%   driver lifts the lag-1 value above it (0.9864 vs 0.95 at lc=0.7,
%   a_pd=a_cov=0.05; matches the measured a_xm noise ACF on f1Hz A2 data).
    q1 = [1, -1]; q3 = [0, 0, 0, 1]; thnum = [1, (1 - lc), (1 - lc)];
    numFT = (1 - apd) * conv(conv(q1, q3), thnum);
    den   = conv([1, -(1 - apd)], [1, -lc]);
    N = 8000;
    imp = [1; zeros(N - 1, 1)];
    hFT = filter(numFT, den, imp);
    Tmax = 800;
    rho = zeros(Tmax + 1, 1);
    for t = 0:Tmax
        rho(t + 1) = sum(hFT(1:end - t) .* hFT(1 + t:end));
    end
    rho_s = (rho / rho(1)).^2;                 % Wick: ACF of dx_r^2
    alpha = 1 - a_cov;
    g01 = zeros(2, 1);
    for tau = 0:1
        ssum = 0;
        for m = -Tmax:Tmax
            idx = abs(tau + m);
            if idx <= Tmax
                ssum = ssum + alpha^abs(m) * rho_s(idx + 1);
            end
        end
        g01(tau + 1) = ssum;
    end
    phi = g01(2) / g01(1);
end


function [A, B, C, RfT0, RfN0] = compute_if_abc(lc, apd, s)
%COMPUTE_IF_ABC  s-weighted autocorrelation sums for the exact IF_eff
%   (R22_derivation.tex S4-S6), via F_T/F_N impulse responses (toolbox-free).
%   s = 1-a_cov reproduces the EWMA-output IF_eff (raw readout); s = 1 gives
%   the whitened long-run factor IF_white (2026-08-27). Both use the SAME
%   R_fT/R_fN autocorrelations; only the lag weights differ.
%
%   F_T = (1-apd)(1-q)q^3[1+(1-lc)q+(1-lc)q^2] / [(1-(1-apd)q)(1-lc q)],  q=z^-1
%   F_N = (1-apd)(1-q)^2[1+(1-lc)q+(1-lc)q^2] / [same den]   (z^3 cancels q^3)
%
%   Returns  A = sum_{tau>=1} R_fT(tau)^2 s^tau,  B = sum R_fT(tau) R_fN(tau) s^tau,
%   C = sum R_fN(tau)^2 s^tau, plus RfT0=R_fT(0)=C_dpmr and RfN0=R_fN(0)=C_n
%   for an independent self-check. Poles (1-apd, lc) < 1, so the impulse
%   responses decay geometrically (rho^2 ~ (1-apd)^(2 tau) at s = 1); N/Tmax
%   are set for machine precision.
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


function [sT, cN, CT1, CN1, CTp1, CNp1] = compute_y2_mirror(lc, apd, d)
%COMPUTE_Y2_MIRROR  Closed-loop sensitivity of the a_xm gauge at g=1.
%   C_T(g)/C_N(g) are the closed-loop variance coefficients of the thermal
%   (w) and sensor (n) noise, read out through the residual dx_r (the signal
%   a_xm inverts). Each is an exact stationary discrete-Lyapunov solve of the
%   hold-phase loop (cl_var_dxr below). Central difference (dg=1e-4) gives the
%   g-derivatives at g=1; sT=(C_T(1)+C_T'(1))/C_T(1) is dE[a_xm]/da in the
%   thermal-dominated limit, cN=C_N'(1)/C_T(1) the sensor-mix correction.
    dg = 1e-4;
    CT1  = cl_var_dxr(1,      lc, apd, d, 'T');
    CN1  = cl_var_dxr(1,      lc, apd, d, 'N');
    CTp  = cl_var_dxr(1 + dg, lc, apd, d, 'T');
    CTm  = cl_var_dxr(1 - dg, lc, apd, d, 'T');
    CNp  = cl_var_dxr(1 + dg, lc, apd, d, 'N');
    CNm  = cl_var_dxr(1 - dg, lc, apd, d, 'N');
    CTp1 = (CTp - CTm) / (2 * dg);
    CNp1 = (CNp - CNm) / (2 * dg);
    sT = (CT1 + CTp1) / CT1;
    cN = CNp1 / CT1;
end


function V = cl_var_dxr(g, lc, apd, d, src)
%CL_VAR_DXR  Stationary Var(dx_r) driven by one unit-variance noise source.
%   src='T' thermal (w), src='N' sensor (n). Builds the hold-phase loop
%   state-space via mirror_ss, solves the Stein equation P = A*P*A' + Q, and
%   reads Var(dx_r) = c'*P*c (plus the direct dn^2 term for the sensor source;
%   z[k] is built from inputs up to k-1 so z[k] and n[k] are independent and
%   the cross term vanishes).
    [A, Bn, Bw, c, dn] = mirror_ss(g, lc, apd, d);
    if src == 'T'
        P = dlyap_local(A, Bw * Bw');
        V = c' * P * c;
    else
        P = dlyap_local(A, Bn * Bn');
        V = c' * P * c + dn ^ 2;
    end
end


function [A, Bn, Bw, c, dn] = mirror_ss(g, lc, apd, d)
%MIRROR_SS  Hold-phase closed-loop state space of the a_xm read-out chain.
%   State z[k] (before step k), inputs n[k] (sensor), w[k] (thermal):
%     dxm = dx[k-d] + n ; u = g*beta*dxm - beta*sum(u[k-1..k-d]) ;
%     xbn = (1-apd)*xb[k-1] + apd*dxm ; dx[k+1] = dx[k] - u - w ;
%     dxr = dxm - xbn = (1-apd)*(dx[k-d] + n - xb[k-1])   (OUTPUT).
%   d=2: z=[dx;dx(k-1);dx(k-2);u(k-1);u(k-2);xb(k-1)] (6 states).
%   d=1: z=[dx;dx(k-1);u(k-1);xb(k-1)]                (4 states).
    beta = 1 - lc;
    om   = 1 - apd;                 % 1 - a_pd
    if d == 2
        A = [ 1, 0, -g*beta,  beta,  beta, 0; ...
              1, 0,  0,       0,     0,    0; ...
              0, 1,  0,       0,     0,    0; ...
              0, 0,  g*beta, -beta, -beta, 0; ...
              0, 0,  0,       1,     0,    0; ...
              0, 0,  apd,     0,     0,    om ];
        Bn = [-g*beta; 0; 0; g*beta; 0; apd];
        Bw = [-1; 0; 0; 0; 0; 0];
        c  = om * [0; 0; 1; 0; 0; -1];
    elseif d == 1
        A = [ 1, -g*beta,  beta, 0; ...
              1,  0,       0,    0; ...
              0,  g*beta, -beta, 0; ...
              0,  apd,     0,    om ];
        Bn = [-g*beta; 0; g*beta; apd];
        Bw = [-1; 0; 0; 0];
        c  = om * [0; 1; 0; -1];
    else
        error('build_eq17_6state_constants:mirrorDelay', ...
              'compute_y2_mirror supports d=1 or d=2 only; got %g.', d);
    end
    dn = om;
end


function X = dlyap_local(A, Q)
%DLYAP_LOCAL  Solve the Stein equation X = A*X*A' + Q. Uses dlyap (Control
%   Toolbox) when present, else a toolbox-free squaring iteration
%   X = sum_k A^k Q (A')^k (converges for spectral radius < 1; the hold-phase
%   loop is stable so this holds). Fallback mirrors solve_dare_kf_local in
%   motion_control_law_eq17_4state.m.
    if exist('dlyap', 'file') ~= 0
        try
            X = dlyap(A, Q);
            X = (X + X') / 2;
            return;
        catch
            % Control Toolbox absent or dlyap failed: fall through to iterate.
        end
    end
    X = Q;
    T = A;
    for it = 1:200000
        dX = T * X * T';
        X  = X + dX;
        T  = T * T;
        if max(abs(dX(:))) < 1e-14
            break;
        end
    end
    X = (X + X') / 2;
end
