function rho_a = compute_rho_a_rigorous(A_aug, d, a_pd, a_cov, sigma2_n, Lmax)
%COMPUTE_RHO_A_RIGOROUS Autocorrelation amplification for IIR variance estimator
%
%   rho_a = compute_rho_a_rigorous(A_aug, d, a_pd, a_cov, sigma2_n, Lmax)
%
%   Returns rho_a = 1 + 2 * sum_{L=1}^{Lmax} rho(L)^2 * (1 - a_cov)^L
%   where rho(L) = normalized autocorrelation of del_pmr at lag L.
%
%   Only state-contribution part is used (exogenous noise is white so
%   contributes no correlation at L>=1; its variance only dilutes rho_state).
%
%   Inputs:
%       A_aug    - 11-dim closed-loop augmented transition matrix
%       d        - diagnostics struct from compute_7state_cdpmr_eff containing:
%                    .idx_dx_d2, .idx_pmd_prev (state indices)
%                    .Sigma_aug_phys  (preferred, physical-units Sigma)
%                    OR .Sigma_th     (fallback, unit-variance)
%       a_pd     - IIR LP smoothing factor
%       a_cov    - IIR variance EMA smoothing factor
%       sigma2_n - per-axis sensor noise variance (um^2); scalar
%       Lmax     - max lag for autocorrelation sum (e.g. 200)
%
%   Output:
%       rho_a    - autocorrelation amplification (>= 1, no amp = 1)
%
%   Extracted from compute_r22_self_consistent.m (2026-04-20).

    n_aug = size(A_aug, 1);
    gain = (1 - a_pd);

    % Output selector c such that state part of del_pmr = c' * x_aug
    % del_pmr[k] = gain * (dx_d2[k] - pmd_prev[k] + n_p[k])
    c = zeros(n_aug, 1);
    c(d.idx_dx_d2)    =  gain;
    c(d.idx_pmd_prev) = -gain;

    % Build physical Sigma_aug if provided, else use unit-variance combination
    if isfield(d, 'Sigma_aug_phys')
        Sigma_full = d.Sigma_aug_phys;
    else
        % fallback: unit-variance combination (thermal only)
        Sigma_full = d.Sigma_th;
    end

    % State-contribution variance and autocorrelation
    V_state = c' * Sigma_full * c;
    if V_state <= 0
        rho_a = 1;   % degenerate; no correlation effect
        return;
    end

    % Noise contribution to del_pmr variance (exogenous, adds to L=0 only)
    V_noise = gain^2 * sigma2_n;
    V_total = V_state + V_noise;

    % Autocorrelation at lag L: state part propagates via A_aug
    rho_sum = 0;
    cA = c';           % 1 x n_aug
    damp = 1 - a_cov;
    damp_L = damp;     % (1-a_cov)^L accumulating

    for L = 1:Lmax
        cA = cA * A_aug;                  % c' * A^L
        corr_L = (cA * Sigma_full * c);   % scalar
        rho_L = corr_L / V_total;          % normalized by total (state+noise)
        rho_sum = rho_sum + (rho_L^2) * damp_L;
        damp_L = damp_L * damp;
        if damp_L < 1e-20, break; end
    end

    rho_a = 1 + 2 * rho_sum;
end
