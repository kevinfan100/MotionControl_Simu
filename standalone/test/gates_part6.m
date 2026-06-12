function gates_part6()
%GATES_PART6 Theory gates for packaging part 6 (Q/R CONTENT verification).
%
%   Part 6 changes NO code (the Q/R formulas landed with the EKF in
%   pack(6)); it verifies the Q/R FORMULAS are correct CONSTRUCTIONS --
%   the "self-construction review" goal. The controller has no diag
%   output, so each formula is re-derived and checked against a reference.
%
%   The INDEPENDENT verification weight rests on F2 and F5 (Monte-Carlo
%   ground truth). F1/F3/F4 are consistency / drift guards -- correct, but
%   they cannot fail without a transcription typo; they are labelled as
%   such below, NOT oversold as independent checks (per part-6 review).
%
%   F1  Q55 increment-factor algebra (cancellation DOCUMENTED, not proven):
%       2*(1-rho1)*C_dx == 2/(1+lc) with C_dx=2+1/(1-lc^2),
%       rho1=lc+2*(1-lc)/C_dx. Reduces to C_dx-2=1/(1-lc^2) -- a
%       definitional identity. The PHYSICAL content (that C_dx/rho1 are
%       the real closed-loop variance/autocorr) is supplied by F2.
%   F2  Q55 GROUND TRUTH (synthetic ideal closed loop, the load-bearing
%       check): simulate the z-axis ideal tracking error per paper Eq.19
%           dx[k+1] = lc*dx[k] - ( w[k] + (1-lc)*(w[k-1]+w[k-2]) )
%       w white, var = sigma2_dh. Confirms from scratch Var(dx) =
%       C_dx*sigma2_dh, Var(increment) = [2/(1+lc)]*sigma2_dh, lag-1
%       autocorr = rho1 -- the full Q55 derivation chain (mother
%       emp/closed ~ 0.998).
%   F3  IF self-consistency (NOT independent): "Method A == Method B" is
%       an algebraic regrouping of one expression (transcription guard);
%       the impulse-sum self-check R_fT(0)==C_dpmr / R_fN(0)==C_n ties the
%       closed form to the transfer-function MODEL (same model the
%       controller uses). It proves consistency, not ground truth.
%   F4  C_dpmr / C_n: the a_pd -> 0 limit reducing to 2+1/(1-lc^2) /
%       2/(1+lc) is a genuine analytic generalization check; the
%       a_pd=0.05 -> 3.1610/1.1093 assertion is a drift guard.
%   F5  C_dpmr / C_n GROUND TRUTH (real closed loop): reconstruct the IIR
%       residual dx_r externally from the actual h50 run logs (verified
%       log-row mapping from part-3 C2) and confirm its variance matches
%       the closed form  C_dpmr*4kBT*a_z + C_n*sigma2_nx_z  -- the genuine
%       physical anchor for the two constants F3 only self-checks.

    here = fileparts(mfilename('fullpath'));
    sa_root = fileparts(here);

    orig_path = path; orig_dir = pwd;
    cleanup = onCleanup(@() local_restore(orig_path, orig_dir));

    addpath(sa_root, fullfile(sa_root, 'physics'), fullfile(sa_root, 'sim'));

    lc = 0.7; apd = 0.05; acov = 0.05;

    % ================= F1: Q55 increment-factor exact algebra =============
    C_dx = 2 + 1/(1 - lc^2);
    rho1 = lc + 2*(1 - lc)/C_dx;
    cancellation = 2*(1 - rho1)*C_dx;          % should equal 2/(1+lc)
    target = 2/(1 + lc);
    assert(abs(cancellation - target) < 1e-14, ...
           'F1 increment factor: 2(1-rho1)C_dx = %.12f vs 2/(1+lc) = %.12f', ...
           cancellation, target);
    fprintf('F1 PASS  Q55 increment factor 2(1-rho1)*C_dx == 2/(1+lc) = %.6f (exact, dev %.1e)\n', ...
            target, abs(cancellation - target));

    % ================= F2: Q55 emp/closed (synthetic ideal closed loop) ===
    rng(61);
    N = 2e5; burn = 2e3;
    sigma2_dh = 1.0;                            % normalized (ratios are dimensionless)
    w = sqrt(sigma2_dh) * randn(N, 1);
    dx = zeros(N, 1);
    for k = 3:N-1
        eps_k = w(k) + (1 - lc) * (w(k-1) + w(k-2));
        dx(k+1) = lc * dx(k) - eps_k;
    end
    dxs = dx(burn+1:end);
    var_level = var(dxs);
    var_incr  = var(diff(dxs));
    ac = xcorr(dxs - mean(dxs), 1, 'normalized');
    rho1_emp = ac(end);

    r_level = var_level / (C_dx * sigma2_dh);
    r_incr  = var_incr  / (target * sigma2_dh);
    assert(abs(r_level - 1) < 0.03, 'F2 Var(dx)/[C_dx*s2dh] = %.4f', r_level);
    assert(abs(r_incr  - 1) < 0.03, 'F2 Var(incr)/[2/(1+lc)*s2dh] = %.4f', r_incr);
    assert(abs(rho1_emp - rho1) < 0.02, 'F2 rho1 emp %.4f vs closed %.4f', rho1_emp, rho1);
    fprintf(['F2 PASS  synthetic ideal loop: Var(dx)/C_dx-form = %.4f, ' ...
             'Var(incr)/[2/(1+lc)]-form = %.4f, rho1 %.4f (closed %.4f)\n'], ...
            r_level, r_incr, rho1_emp, rho1);

    % ================= F3: IF dual-computation + self-check ===============
    s = 1 - acov;
    q1 = [1, -1]; q3 = [0, 0, 0, 1]; thnum = [1, (1 - lc), (1 - lc)];
    numFT = (1 - apd) * conv(conv(q1, q3), thnum);
    numFN = (1 - apd) * conv(conv(q1, q1), thnum);
    den   = conv([1, -(1 - apd)], [1, -lc]);
    Nimp = 8000; imp = [1; zeros(Nimp - 1, 1)];
    hFT = filter(numFT, den, imp);
    hFN = filter(numFN, den, imp);
    Tmax = 600;

    % algebraic C_dpmr / C_n (closed form)
    one_m_apd = 1 - apd; denom_pole = 1 - one_m_apd * lc;
    C_dpmr = one_m_apd^2 * (2*one_m_apd*(1-lc)/denom_pole ...
             + (2/(2-apd)) * 1/((1+lc)*denom_pole));
    C_n = (2*one_m_apd^2/(2-apd)) * (1 + one_m_apd^2*apd*(1-lc)/denom_pole ...
             + (1-lc)^2/((1+lc)*denom_pole));

    % self-check: impulse-response sum-of-squares == algebraic constants
    RfT0 = sum(hFT.^2); RfN0 = sum(hFN.^2);
    assert(abs(RfT0 - C_dpmr) < 1e-5 * C_dpmr && abs(RfN0 - C_n) < 1e-5 * C_n, ...
           'F3 self-check: R_fT(0)=%.6f vs C_dpmr=%.6f; R_fN(0)=%.6f vs C_n=%.6f', ...
           RfT0, C_dpmr, RfN0, C_n);

    % choose a representative operating point (z axis at h50)
    a_nom = (1/1600) / 0.0425; sxT = 4 * (1.3806503e-5 * 310.15) * a_nom; snx = 0.00331^2;

    % Method A: s-weighted autocorr sums A,B,C
    A = 0; B = 0; C = 0;
    for t = 1:Tmax
        RfT = sum(hFT(1:end-t) .* hFT(1+t:end));
        RfN = sum(hFN(1:end-t) .* hFN(1+t:end));
        st = s^t;
        A = A + RfT^2 * st;
        B = B + RfT * RfN * st;
        C = C + RfN^2 * st;
    end
    IF_A = 1 + 2*(sxT^2*A + 2*sxT*snx*B + snx^2*C) / (C_dpmr*sxT + C_n*snx)^2;

    % Method B: direct per-tau squared residual autocovariance
    %   gamma(tau) = sxT*R_fT(tau) + snx*R_fN(tau),  gamma(0) = sxT*C_dpmr + snx*C_n
    g0 = sxT * C_dpmr + snx * C_n;
    IF_B_acc = 0;
    for t = 1:Tmax
        RfT = sum(hFT(1:end-t) .* hFT(1+t:end));
        RfN = sum(hFN(1:end-t) .* hFN(1+t:end));
        g_t = sxT * RfT + snx * RfN;
        IF_B_acc = IF_B_acc + (g_t / g0)^2 * s^t;
    end
    IF_B = 1 + 2 * IF_B_acc;

    assert(abs(IF_A - IF_B) < 1e-10 * IF_A, 'F3 IF_A %.10f vs IF_B %.10f', IF_A, IF_B);
    assert(IF_A > 1 && isfinite(IF_A), 'F3 IF_eff out of range %.4f', IF_A);
    fprintf(['F3 PASS  IF self-consistency: Method A (A,B,C packed) == Method B ' ...
             '(regrouped), IF_eff = %.4f; self-check R_fT(0)=C_dpmr / R_fN(0)=C_n ' ...
             '(closed form vs transfer-function model)\n'], IF_A);

    % ================= F4: C_dpmr / C_n construction ======================
    cd_full = C_dpmr; cn_full = C_n;
    assert(abs(cd_full - 3.1610) < 5e-4 && abs(cn_full - 1.1093) < 5e-4, ...
           'F4 a_pd=0.05 values drifted: %.4f / %.4f', cd_full, cn_full);
    % a_pd -> 0 limit: full form -> 2+1/(1-lc^2) / 2/(1+lc)
    apd0 = 1e-8; omp0 = 1 - apd0; dp0 = 1 - omp0*lc;
    cd_lim = omp0^2 * (2*omp0*(1-lc)/dp0 + (2/(2-apd0)) * 1/((1+lc)*dp0));
    cn_lim = (2*omp0^2/(2-apd0)) * (1 + omp0^2*apd0*(1-lc)/dp0 + (1-lc)^2/((1+lc)*dp0));
    cd_simp = 2 + 1/(1 - lc^2);                % 3.9608
    cn_simp = 2/(1 + lc);                      % 1.1765
    assert(abs(cd_lim - cd_simp) < 1e-6 && abs(cn_lim - cn_simp) < 1e-6, ...
           'F4 a_pd->0 limit: C_dpmr %.6f vs %.6f, C_n %.6f vs %.6f', ...
           cd_lim, cd_simp, cn_lim, cn_simp);
    fprintf(['F4 PASS  C_dpmr/C_n construction: a_pd=0.05 -> %.4f/%.4f; ' ...
             'a_pd->0 limit -> %.4f/%.4f (= 2+1/(1-lc^2) / 2/(1+lc))\n'], ...
            cd_full, cn_full, cd_lim, cn_lim);

    % ================= F5: C_dpmr/C_n REAL closed-loop ground truth =======
    % Reconstruct the controller's IIR residual dx_r externally from the
    % actual h50 run, using the part-3 C2 verified log-row mapping
    %   delta_x_m_z[k] = p_d_out(k-2, 3) - p_m_out(k-3, 3)
    % (z = wall normal; both p_d and p_m are logged). Replay the same IIR
    % (a_pd EWMA, dx_bar init 0 per controller [0H]) and check
    %   Var(dx_r) ~ C_dpmr*4kBT*a_z + C_n*sigma2_nx_z   (the constants'
    % physical meaning -- closed-loop residual variance coefficients).
    rng(7); params = config('h50');
    out = run_simulation('h50', struct('seed', 7));
    pdz = out.p_d_out(:, 3); pmz = out.p_m_out(:, 3);
    Nz = numel(pdz);
    dxm = nan(Nz, 1);
    for k = 4:Nz, dxm(k) = pdz(k-2) - pmz(k-3); end     % verified mapping
    apd_c = params.ctrl.a_pd;
    dxbar = 0; dxr = nan(Nz, 1);
    for k = 4:Nz
        dxbar = (1 - apd_c) * dxbar + apd_c * dxm(k);
        dxr(k) = dxm(k) - dxbar;
    end
    burn5 = round(0.5 / params.common.Ts);              % drop 0.5 s warm-up
    var_dxr_emp = var(dxr(burn5+1:end), 'omitnan');

    kBT5 = params.ctrl.k_B * params.ctrl.T;
    a_nom5 = params.common.Ts / params.common.gamma_N;
    [~, cpe5] = wall_corrections(params.traj.h_init / params.common.R);
    a_z = a_nom5 / cpe5;
    var_dxr_closed = C_dpmr * 4 * kBT5 * a_z + C_n * params.ctrl.sigma2_noise(3);
    r5 = var_dxr_emp / var_dxr_closed;
    assert(abs(r5 - 1) < 0.10, ...
           'F5 Var(dx_r) emp/closed = %.4f (emp %.4e, closed %.4e)', ...
           r5, var_dxr_emp, var_dxr_closed);
    fprintf(['F5 PASS  real closed-loop residual: Var(dx_r) emp/closed = %.4f ' ...
             '(C_dpmr*4kBT*a_z + C_n*sigma2_nx_z ground truth)\n'], r5);

    fprintf(['\n=== gates_part6: ALL PASS (F1 Q55 algebra, F2 Q55 ground truth, ' ...
             'F3 IF self-consistency, F4 C limit, F5 C ground truth) ===\n']);
end


function local_restore(orig_path, orig_dir)
%LOCAL_RESTORE  Undo path/cwd mutations on exit (incl. on error).
    path(orig_path);
    cd(orig_dir);
end
