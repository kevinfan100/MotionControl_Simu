function IF2 = compute_if2_rigorous(a_vec, sigma2_nx, lambda_c, a_pd, a_cov, a_det, kBT)
%COMPUTE_IF2_RIGOROUS  a-dependent 2nd-stage color-inflation IF2 for a_m_det R22.
%   (am_lpf_r22_design.md §3.3 — rigorous L1 replacement for the AR(1) approx
%    rho_axm(tau) ~ (1-a_cov)^tau used in build_eq17_6state_constants.)
%
%   IF2 = compute_if2_rigorous(a_vec, sigma2_nx, lambda_c, a_pd, a_cov, a_det, kBT)
%
%   The post-LPF satisfies Var(a_m_det) = [a_det/(2-a_det)] * IF2(a) * Var(a_xm),
%   IF2(a) = 1 + 2*sum_{tau>=1} (1-a_det)^tau * rho_axm(tau; a), where rho_axm is
%   the autocorrelation of sigma2_dxr_hat (the EWMA(a_cov) output that a_xm is an
%   affine map of). Rigorously:
%       gsig(tau)  ∝  sum_d phi^|d| * gamma_u(|d-tau|),     phi = 1-a_cov
%       gamma_u(l) ∝  [sxT*RfT(l) + snx*RfN(l)]^2  (Isserlis on Gaussian dx_r)
%       sxT = 4*kBT*a,  snx = sigma2_nx
%       rho_axm(tau) = gsig(tau)/gsig(0)        (overall scale cancels in ratio)
%   RfT(l)/RfN(l) = autocovariances of the F_T/F_N impulse responses (the SAME
%   filters as compute_if_abc in build_eq17_6state_constants). The AR(1) approx
%   is recovered if gamma_u(l) were a pure geometric phi^l; the sxT:snx mix bends
%   it away from that, and the bend is a-dependent -> IF2 grows near the wall
%   (small a, sensor-dominated), exactly where the AR(1) version under-predicts.
%
%   Inputs:
%     a_vec      [Mx1] motion-gain values to evaluate          [um/pN]
%     sigma2_nx  scalar per-axis sensor-noise variance         [um^2]
%     lambda_c, a_pd, a_cov, a_det  controller scalars
%     kBT        thermal energy (sxT = 4*kBT*a)                 [pN*um]
%   Output:
%     IF2        [Mx1] per-a second-stage inflation factor
%
%   See also: build_eq17_6state_constants (compute_if_abc), verify_r22_amlpf_6state

    lc = lambda_c; apd = a_pd; phi = 1 - a_cov;
    a_vec = a_vec(:);

    % --- F_T / F_N impulse responses (mirror compute_if_abc) ---
    q1 = [1, -1]; q3 = [0, 0, 0, 1]; thnum = [1, (1 - lc), (1 - lc)];
    numFT = (1 - apd) * conv(conv(q1, q3), thnum);
    numFN = (1 - apd) * conv(conv(q1, q1), thnum);
    den   = conv([1, -(1 - apd)], [1, -lc]);
    Nimp  = 8000; imp = [1; zeros(Nimp - 1, 1)];
    hFT = filter(numFT, den, imp);
    hFN = filter(numFN, den, imp);

    % --- autocovariances RfT(l), RfN(l), l = 0..Lmax (poles<1 -> geometric decay) ---
    Lmax = 600;
    RfT = zeros(Lmax + 1, 1); RfN = zeros(Lmax + 1, 1);
    for L = 0:Lmax
        RfT(L + 1) = sum(hFT(1:end - L) .* hFT(1 + L:end));
        RfN(L + 1) = sum(hFN(1:end - L) .* hFN(1 + L:end));
    end

    % --- IF2 per a-value ---
    T2   = 300;                         % tau range for the (1-a_det)^tau sum
    Dmax = Lmax + T2;                   % d range (gamma_u=0 beyond |.|>Lmax)
    d    = (-Dmax:Dmax).';
    wphi = phi .^ abs(d);               % phi^|d|
    w_det = (1 - a_det) .^ (1:T2).';    % geometric LPF weights
    IF2  = zeros(numel(a_vec), 1);

    for ia = 1:numel(a_vec)
        sxT = 4 * kBT * a_vec(ia); snx = sigma2_nx;
        g  = sxT * RfT + snx * RfN;     % [Lmax+1 x 1], index l+1
        gu = g .^ 2;                    % gamma_u(l) up to a constant (cancels in rho)
        gsig = zeros(T2 + 1, 1);
        for t = 0:T2
            l  = abs(d - t);
            ok = l <= Lmax;
            gsig(t + 1) = sum(wphi(ok) .* gu(l(ok) + 1));
        end
        rho = gsig / gsig(1);           % rho_axm(tau) = gsig(tau)/gsig(0)
        IF2(ia) = 1 + 2 * sum(w_det .* rho(2:end));
    end
end
