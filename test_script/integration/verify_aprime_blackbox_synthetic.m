function out = verify_aprime_blackbox_synthetic(opts)
%VERIFY_APRIME_BLACKBOX_SYNTHETIC  Pure-algebra synthetic test of the
%   black-box regression estimator for a' (see reference/eq17_analysis/
%   derivation/aprime_blackbox_regression.tex), fully decoupled from
%   run_pure_simulation / the real controller.
%
%   Purpose: verify_aprime_blackbox_v1_ensemble found a statistically
%   significant, WRONG-SIGN discrepancy (t-stat up to -7.6) when comparing
%   the estimator against the real controller's a_xm/dx_r. This script
%   isolates whether the CORE REGRESSION ALGEBRA itself is correct, by
%   testing it on synthetic data built to satisfy the derivation's assumed
%   model EXACTLY -- decoupled from any real-controller confound (e.g. the
%   documented near-wall a_xm bias).
%
%   Construction (matches aprime_blackbox_regression.tex term-for-term):
%     1. dx_r[k]: AR(1) Gaussian, steady-state Var = C_dpmr*4kBT*a_level_true
%        + C_n*sigma2_n_s (the correct fluctuation-dissipation variance AT
%        the level value -- frozen-coefficient, same approximation used
%        throughout the project's own Q/R derivations).
%     2. a_xm_raw[k]: built via the SAME chi-squared EWMA mechanism as the
%        real a_xm channel (sigma2_hat = EWMA(dx_r^2), then linear
%        inversion) -- so a_xm_raw's own noise IS a genuine quadratic
%        functional of dx_r, faithfully testing the Isserlis "a_xm is built
%        from dx_r^2, is correlating it with dx_r circular" argument.
%     3. a_xm[k] = a_xm_raw[k] - a_prime_true*dx_r[k]: the TRUE a' signal is
%        injected ADDITIVELY, exactly matching the derivation's linear model
%        a_xm[k] ~= a_level[k] + a'[k]*Delta_h[k] + n_a[k] (Delta_h = -dx_r).
%     4. a_m_det[k]: LP filter of a_xm (matches the real a_m_det mechanism).
%     5. Estimator: same batch formula as verify_aprime_blackbox_v1_ensemble.
%
%   If this recovers the KNOWN a_prime_true (small |t-stat|), the pure
%   algebra + Isserlis argument are confirmed correct, and the real-system
%   discrepancy must come from something that VIOLATES an assumption here
%   (e.g. dx_r ~= -Delta_h being only approximate, or the documented
%   near-wall a_xm bias not captured by this additive model). If this test
%   itself fails, the algebra/derivation has a real bug.
%
%   out = verify_aprime_blackbox_synthetic()
%   out = verify_aprime_blackbox_synthetic(opts)  opts.n_trials (default 60),
%       opts.N (default 20000), opts.rho_list (default [0 0.85])
%
%   See also: verify_aprime_blackbox_v1_ensemble,
%             reference/eq17_analysis/derivation/aprime_blackbox_regression.tex

    if nargin < 1 || isempty(opts); opts = struct(); end
    if ~isfield(opts, 'n_trials');      opts.n_trials      = 60;      end
    if ~isfield(opts, 'N');             opts.N             = 20000;   end
    if ~isfield(opts, 'rho_list');      opts.rho_list      = [0, 0.85]; end
    if ~isfield(opts, 'a_cov');         opts.a_cov         = 0.05;    end
    if ~isfield(opts, 'a_det_lp');      opts.a_det_lp      = 0.0002;  end
    if ~isfield(opts, 't_steady_frac'); opts.t_steady_frac = 0.2;     end
    if ~isfield(opts, 'verbose');       opts.verbose       = true;    end

    here = fileparts(mfilename('fullpath'));
    proj = fullfile(here, '..', '..');
    addpath(genpath(proj));

    % --- Known ground truth + physical constants (matching real system scale) ---
    pc  = physical_constants();
    kBT = pc.k_B * pc.T;                 % [pN*um]
    lambda_c = 0.7;
    C_dpmr = 2 + 1/(1 - lambda_c^2);     % 3.96 (matches build_eq17_6state_constants)
    C_n    = 2/(1 + lambda_c);           % 1.18
    sigma2_n_s   = 0.001^2;              % [um^2], representative sensor noise
    a_level_true = 0.01;                 % [um/pN], representative level
    a_prime_true = 0.0015;               % [1/pN], KNOWN true slope (matches oracle order-of-magnitude)

    sigma2_dxr0 = C_dpmr * 4 * kBT * a_level_true + C_n * sigma2_n_s;   % steady-state Var(dx_r)

    N   = opts.N;
    nT  = opts.n_trials;
    ac  = opts.a_cov;
    adl = opts.a_det_lp;
    t_steady_idx = round(opts.t_steady_frac * N);

    nR = numel(opts.rho_list);
    ap_mean = nan(nR,1); ap_se = nan(nR,1); t_stat = nan(nR,1);

    for ri = 1:nR
        rho = opts.rho_list(ri);
        ap_trial = nan(nT,1);
        for ti = 1:nT
            rng(1000*ri + ti);   % independent, reproducible per trial

            % --- 1. dx_r: AR(1) Gaussian, steady-state Var = sigma2_dxr0 ---
            w   = randn(N,1);
            dxr = zeros(N,1);
            dxr(1) = sqrt(sigma2_dxr0) * w(1);
            for k = 2:N
                dxr(k) = rho*dxr(k-1) + sqrt(1-rho^2)*sqrt(sigma2_dxr0)*w(k);
            end

            % --- 2. a_xm_raw via the SAME chi-squared EWMA mechanism ---
            sigma2_hat = zeros(N,1);
            sigma2_hat(1) = sigma2_dxr0;   % prefill (matches iir_warmup_mode='prefill')
            for k = 2:N
                sigma2_hat(k) = (1-ac)*sigma2_hat(k-1) + ac*dxr(k)^2;
            end
            a_xm_raw = (sigma2_hat - C_n*sigma2_n_s) / (C_dpmr*4*kBT);   % E[.] = a_level_true

            % --- 3. inject the TRUE a' signal additively ---
            a_xm = a_xm_raw - a_prime_true * dxr;

            % --- 4. a_level reference: LP filter of a_xm ---
            a_mdet = zeros(N,1);
            a_mdet(1) = a_level_true;
            for k = 2:N
                a_mdet(k) = (1-adl)*a_mdet(k-1) + adl*a_xm(k);
            end

            % --- 5. estimator (same batch formula as verify_aprime_blackbox_v1_ensemble) ---
            win = t_steady_idx:N;
            Cov_batch = mean((a_xm(win) - a_mdet(win)) .* dxr(win));
            Var_batch = mean(dxr(win).^2);
            ap_trial(ti) = -Cov_batch / Var_batch;
        end
        ap_mean(ri) = mean(ap_trial);
        ap_se(ri)   = std(ap_trial) / sqrt(nT);
        t_stat(ri)  = (ap_mean(ri) - a_prime_true) / ap_se(ri);
    end

    out.rho_list     = opts.rho_list;
    out.a_prime_true = a_prime_true;
    out.ap_mean      = ap_mean;
    out.ap_se        = ap_se;
    out.t_stat       = t_stat;

    if opts.verbose
        fprintf('\n[Synthetic algebra test: KNOWN a_prime_true = %.4g 1/pN, %d trials, N=%d]\n', a_prime_true, nT, N);
        fprintf('  %-10s %14s %14s %10s\n', 'rho(dx_r)', 'mean(a_hat'')', 'SE', 't-stat');
        for ri = 1:nR
            fprintf('  %-10.2f %14.6g %14.6g %10.2f\n', opts.rho_list(ri), ap_mean(ri), ap_se(ri), t_stat(ri));
        end
        fprintf('  (|t-stat| ~< 2 is consistent with a_hat_prime recovering the TRUE, KNOWN a''.)\n');
    end
end
