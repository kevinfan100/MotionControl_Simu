function out = verify_aprime_blackbox_synthetic_delay(opts)
%VERIFY_APRIME_BLACKBOX_SYNTHETIC_DELAY  Tests whether the d-step a_xm
%   measurement delay (a_xm[k] = a_x[k-d] + n_a[k], per
%   4state_del_hd_ar1.tex's Output equation) explains the wrong-sign
%   discrepancy found in verify_aprime_blackbox_v1_ensemble, by comparing:
%     (a) naive regression: correlate a_xm[k] against dx_r[k] (current) --
%         what verify_aprime_blackbox_v1_ensemble actually did (mismatched
%         alignment, since a_xm[k] reflects dx_r[k-d], not dx_r[k]).
%     (b) delay-corrected regression: correlate a_xm[k] against dx_r[k-d].
%
%   Same synthetic construction as verify_aprime_blackbox_synthetic, but the
%   true a' signal is injected via the DELAYED dx_r[k-d] (matching
%   a_x[k-d] = a_level - a_prime_true*dx_r[k-d]):
%       a_xm[k] = a_xm_raw[k] - a_prime_true*dx_r[k-d]
%
%   out = verify_aprime_blackbox_synthetic_delay()
%   out = verify_aprime_blackbox_synthetic_delay(opts)  opts.d_delay (default 2)
%
%   See also: verify_aprime_blackbox_synthetic, verify_aprime_blackbox_v1_ensemble

    if nargin < 1 || isempty(opts); opts = struct(); end
    if ~isfield(opts, 'n_trials');      opts.n_trials      = 60;      end
    if ~isfield(opts, 'N');             opts.N             = 20000;   end
    if ~isfield(opts, 'rho');           opts.rho           = 0.85;    end
    if ~isfield(opts, 'd_delay');       opts.d_delay       = 2;       end
    if ~isfield(opts, 'a_cov');         opts.a_cov         = 0.05;    end
    if ~isfield(opts, 'a_det_lp');      opts.a_det_lp      = 0.0002;  end
    if ~isfield(opts, 't_steady_frac'); opts.t_steady_frac = 0.2;     end
    if ~isfield(opts, 'verbose');       opts.verbose       = true;    end

    here = fileparts(mfilename('fullpath'));
    proj = fullfile(here, '..', '..');
    addpath(genpath(proj));

    pc  = physical_constants();
    kBT = pc.k_B * pc.T;
    lambda_c = 0.7;
    C_dpmr = 2 + 1/(1 - lambda_c^2);
    C_n    = 2/(1 + lambda_c);
    sigma2_n_s   = 0.001^2;
    a_level_true = 0.01;
    a_prime_true = 0.0015;

    sigma2_dxr0 = C_dpmr * 4 * kBT * a_level_true + C_n * sigma2_n_s;

    N   = opts.N;
    nT  = opts.n_trials;
    ac  = opts.a_cov;
    adl = opts.a_det_lp;
    rho = opts.rho;
    d   = opts.d_delay;
    t_steady_idx = round(opts.t_steady_frac * N);

    ap_naive = nan(nT,1);
    ap_fixed = nan(nT,1);

    for ti = 1:nT
        rng(2000 + ti);

        w = randn(N,1);
        dxr = zeros(N,1);
        dxr(1) = sqrt(sigma2_dxr0) * w(1);
        for k = 2:N
            dxr(k) = rho*dxr(k-1) + sqrt(1-rho^2)*sqrt(sigma2_dxr0)*w(k);
        end

        sigma2_hat = zeros(N,1);
        sigma2_hat(1) = sigma2_dxr0;
        for k = 2:N
            sigma2_hat(k) = (1-ac)*sigma2_hat(k-1) + ac*dxr(k)^2;
        end
        a_xm_raw = (sigma2_hat - C_n*sigma2_n_s) / (C_dpmr*4*kBT);

        % --- inject the TRUE a' signal via the DELAYED dx_r[k-d] ---
        % (matches a_xm[k] = a_x[k-d] + n_a[k], a_x[k-d] = a_level - a'*dx_r[k-d])
        dxr_delayed = [zeros(d,1); dxr(1:end-d)];   % dxr_delayed(k) = dxr(k-d)
        a_xm = a_xm_raw - a_prime_true * dxr_delayed;

        a_mdet = zeros(N,1);
        a_mdet(1) = a_level_true;
        for k = 2:N
            a_mdet(k) = (1-adl)*a_mdet(k-1) + adl*a_xm(k);
        end

        win = t_steady_idx:N;

        % (a) naive: correlate against dx_r[k] (current, mismatched)
        Cov_naive = mean((a_xm(win) - a_mdet(win)) .* dxr(win));
        Var_naive = mean(dxr(win).^2);
        ap_naive(ti) = -Cov_naive / Var_naive;

        % (b) corrected: correlate against dx_r[k-d]
        Cov_fixed = mean((a_xm(win) - a_mdet(win)) .* dxr_delayed(win));
        Var_fixed = mean(dxr_delayed(win).^2);
        ap_fixed(ti) = -Cov_fixed / Var_fixed;
    end

    out.a_prime_true  = a_prime_true;
    out.ap_naive_mean = mean(ap_naive); out.ap_naive_se = std(ap_naive)/sqrt(nT);
    out.ap_fixed_mean = mean(ap_fixed); out.ap_fixed_se = std(ap_fixed)/sqrt(nT);
    out.t_naive = (out.ap_naive_mean - a_prime_true) / out.ap_naive_se;
    out.t_fixed = (out.ap_fixed_mean - a_prime_true) / out.ap_fixed_se;

    if opts.verbose
        fprintf('\n[Synthetic delay test: d=%d, rho=%.2f, KNOWN a_prime_true=%.4g 1/pN, %d trials]\n', d, rho, a_prime_true, nT);
        fprintf('  %-30s %14s %14s %10s\n', 'regression', 'mean(a_hat'')', 'SE', 't-stat');
        fprintf('  %-30s %14.6g %14.6g %10.2f\n', 'naive (vs dx_r[k])', out.ap_naive_mean, out.ap_naive_se, out.t_naive);
        fprintf('  %-30s %14.6g %14.6g %10.2f\n', 'delay-corrected (vs dx_r[k-d])', out.ap_fixed_mean, out.ap_fixed_se, out.t_fixed);
    end
end
