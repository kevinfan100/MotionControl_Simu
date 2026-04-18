function [C_dpmr_eff, C_np_eff, L, A_aug, diagnostics] = ...
    compute_7state_cdpmr_eff(lc, rho, a_pd, Q_kf_scale, R_kf_scale, opts)
%COMPUTE_7STATE_CDPMR_EFF Augmented Lyapunov for 7-state EKF + IIR HP filter
%
%   Derives the effective C_dpmr and C_np coefficients for the relationship
%   Var(del_pmr) = C_dpmr_eff * (4*k_B*T*a) + C_np_eff * sigma2_n
%
%   applied in Eq.13 gain recovery:
%       a_m = (Var(del_pmr) - C_np_eff * sigma2_n) / (C_dpmr_eff * 4*k_B*T)
%
%   Uses a steady-state augmented Lyapunov formulation over an 11-dim state:
%       x_aug = [delta_x, dx_d1, dx_d2, e1..e7, pmd_prev]
%
%   where:
%       delta_x  = current true tracking error (=delta_x3)
%       dx_d1    = delta_x[k-1]
%       dx_d2    = delta_x[k-2]     (what del_pm sees)
%       e1..e7   = 7-state EKF estimation errors (paper Eq.18)
%       pmd_prev = del_pmd[k-1]     (IIR LP previous state)
%
%   Phase 1 assumption: steady-state f_d = 0 (free space, no trajectory coupling
%   via Fe(3,6)). Extension to non-zero f_d is a Phase 2 task.
%
%   Inputs:
%       lc         scalar closed-loop pole (0 < lc < 1)
%       rho        scalar = sigma2_n / sigma2_dXT (noise-to-thermal ratio)
%       a_pd       scalar IIR LP coefficient (0 < a_pd < 1)
%       Q_kf_scale 7x1 Q scaling (multiplied by sigma2_dXT = 1 for unit solve)
%       R_kf_scale 2x1 R scaling (same)
%       opts       struct: .f0 (default 0), .dare_tol (1e-15), .dare_maxit (5000),
%                          .verbose (false)
%
%   Outputs:
%       C_dpmr_eff      Var(del_pmr)/(4*k_B*T*a) from thermal (dimensionless)
%       C_np_eff        Var(del_pmr)/sigma2_n from pos noise (dimensionless)
%       L               7x2 steady-state KF gain
%       A_aug           11x11 augmented transition
%       diagnostics     struct with max_eig, Sigma_th, Sigma_np, dare_iters, etc.

    if nargin < 6 || isempty(opts), opts = struct; end
    if ~isfield(opts, 'f0'),         opts.f0 = 0;     end
    if ~isfield(opts, 'dare_tol'),   opts.dare_tol = 1e-10; end
    if ~isfield(opts, 'dare_maxit'), opts.dare_maxit = 30000; end
    if ~isfield(opts, 'verbose'),    opts.verbose = false; end
    % Optional: provide physical scaling to get Sigma_aug_phys in physical units.
    % opts.physical_scaling = struct with fields:
    %   .sigma2_dXT  : natural thermal unit (e.g. 2.5190e-4 um^2)
    %   .a_phys      : physical motion gain (e.g. a_nom for free-space)
    %   .a_nom       : free-space a_nom = Ts/gamma_N
    %   .sigma2_n    : sensor noise variance (um^2) - e.g. 1e-4 noise ON, 0 OFF
    %   .Q66_abs     : physical Q(6,6) absolute value (default 0)
    %   .Q77_abs     : physical Q(7,7) absolute value (default 0)
    % When supplied, diagnostics.Sigma_aug_phys is populated.
    if ~isfield(opts, 'physical_scaling'), opts.physical_scaling = []; end

    % ---------------------------------------------------------------
    % 1. Build Fe and H for the 7-state EKF (paper Eq.18)
    % ---------------------------------------------------------------
    % Fe(3,6) is -f_dx[k]; we freeze it at opts.f0 (= 0 for Phase 1)
    Fe = [0 1 0  0  0  0       0;
          0 0 1  0  0  0       0;
          0 0 1 -1  0 -opts.f0 0;
          0 0 0  1  1  0       0;
          0 0 0  0  1  0       0;
          0 0 0  0  0  1       1;
          0 0 0  0  0  0       1];

    H = [1 0 0 0 0 0 0;
         0 0 0 0 0 1 0];

    % ---------------------------------------------------------------
    % 2. Build Q_kf and R_kf for DARE (using unit sigma2_dXT)
    %    In the Lyapunov the physical noise scalings are handled separately
    %    in Sigma_th, Sigma_np.  The DARE only uses these to compute L_ss.
    % ---------------------------------------------------------------
    sigma2_dXT_unit = 1;   % unit for DARE; actual physical value scales linearly
    Q_kf = sigma2_dXT_unit * diag(Q_kf_scale(:));
    R_kf = sigma2_dXT_unit * diag(R_kf_scale(:));

    % Note: rho = sigma2_n / sigma2_dXT is used later in position-noise input
    % (not in DARE, which uses the designer's R_kf regardless of actual rho).

    % ---------------------------------------------------------------
    % 3. Iterative DARE for 7-state EKF steady-state gain L
    %    The 7-state DARE is ill-conditioned (marginal eigenvalues at 1)
    %    so full machine-precision convergence is infeasible. Use a large
    %    initial Pf = Q + I for numerical stability and ~30000 iters.
    % ---------------------------------------------------------------
    [L, dare_iters, dare_err] = iterative_dare_7state(Fe, H, Q_kf, R_kf, ...
                                                       opts.dare_tol, opts.dare_maxit);

    if any(~isfinite(L(:)))
        error('compute_7state_cdpmr_eff:dare_failed', ...
              'DARE failed to converge: dare_err=%.3e after %d iters', ...
              dare_err, dare_iters);
    end

    % Closed-loop error dynamics:  A_e = Fe*(I - L*H) = Fe - Fe*L*H
    A_e = Fe - Fe * L * H;

    % ---------------------------------------------------------------
    % 4. Build 11x11 augmented A_aug
    %    Index map:
    %       idx_dx       = 1          (delta_x[k])
    %       idx_dx_d1    = 2          (delta_x[k-1])
    %       idx_dx_d2    = 3          (delta_x[k-2], what del_pm sees)
    %       idx_e(1..7)  = 4..10      (7 EKF error states)
    %       idx_pmd_prev = 11         (del_pmd[k-1], IIR LP prev state)
    % ---------------------------------------------------------------
    n_aug = 11;
    A_aug = zeros(n_aug);

    idx_dx       = 1;
    idx_dx_d1    = 2;
    idx_dx_d2    = 3;
    idx_e        = 4:10;           % e1..e7
    idx_pmd_prev = 11;

    % Row 1: delta_x[k+1] = lc*delta_x + (1-lc)*e3 - e4 - f0*e6 - a*f_T
    %   e3 index = idx_e(3) = 6
    %   e4 index = idx_e(4) = 7
    %   e6 index = idx_e(6) = 9
    A_aug(idx_dx, idx_dx)       = lc;
    A_aug(idx_dx, idx_e(3))     = 1 - lc;
    A_aug(idx_dx, idx_e(4))     = -1;
    A_aug(idx_dx, idx_e(6))     = -opts.f0;

    % Row 2: dx_d1[k+1] = delta_x[k]
    A_aug(idx_dx_d1, idx_dx)    = 1;

    % Row 3: dx_d2[k+1] = dx_d1[k]
    A_aug(idx_dx_d2, idx_dx_d1) = 1;

    % Rows 4-10: 7-state EKF closed-loop error dynamics
    A_aug(idx_e, idx_e)         = A_e;

    % Row 11: pmd_prev[k+1] = del_pmd[k] = (1-a_pd)*pmd_prev[k] + a_pd*del_pm[k]
    %   del_pm[k] = dx_d2[k] + n_p[k]
    %   The deterministic part of del_pm couples via dx_d2
    A_aug(idx_pmd_prev, idx_pmd_prev) = 1 - a_pd;
    A_aug(idx_pmd_prev, idx_dx_d2)    = a_pd;

    % Stability check
    eigs_A_aug = eig(A_aug);
    max_eig = max(abs(eigs_A_aug));
    if max_eig >= 1 - 1e-10
        warning('compute_7state_cdpmr_eff:unstable', ...
                'A_aug eigenvalue %.6f too close to 1 (a_pd=%.4g, lc=%.3g)', ...
                max_eig, a_pd, lc);
    end

    % ---------------------------------------------------------------
    % 5. Build noise input vectors (all 11x1)
    % ---------------------------------------------------------------
    % Thermal: a*f_T enters both delta_x (row 1) and e3 (row 6), same source
    B_th = zeros(n_aug, 1);
    B_th(idx_dx)    = -1;
    B_th(idx_e(3))  = -1;

    % Position measurement noise n_p: enters 7 error states via -Fe*L(:,1)
    %   and IIR pmd_prev via a_pd (direct coupling)
    B_np = zeros(n_aug, 1);
    B_np(idx_e)         = -(Fe * L(:, 1));
    B_np(idx_pmd_prev)  = a_pd;

    % Gain measurement noise n_a: enters 7 error states via -Fe*L(:,2)
    B_na = zeros(n_aug, 1);
    B_na(idx_e)         = -(Fe * L(:, 2));

    % Process noise on state 6 (a): enters idx_e(6) only
    B_q66 = zeros(n_aug, 1);
    B_q66(idx_e(6))     = 1;

    % Process noise on state 7 (delta_a): enters idx_e(7) only
    B_q77 = zeros(n_aug, 1);
    B_q77(idx_e(7))     = 1;

    % ---------------------------------------------------------------
    % 6. Solve Lyapunov equations (unit-variance inputs)
    %    Use dlyapchol for numerical stability; dlyap fails when A_aug
    %    has eigenvalues very close to unit circle (common for this 7-state
    %    KF due to marginal eigenvalues).
    % ---------------------------------------------------------------
    % Thermal unit solve
    Sigma_th = solve_dlyap_robust(A_aug, B_th);

    % Position noise unit solve
    Sigma_np = solve_dlyap_robust(A_aug, B_np);

    % Process-noise unit solves for Q(6,6) and Q(7,7) drivers
    Sigma_q66 = solve_dlyap_robust(A_aug, B_q66);
    Sigma_q77 = solve_dlyap_robust(A_aug, B_q77);

    % ---------------------------------------------------------------
    % 7. Extract C_dpmr_eff and C_np_eff
    %    del_pmr[k] = (1-a_pd) * (dx_d2[k] - pmd_prev[k] + n_p[k])
    %    Var(del_pmr) = (1-a_pd)^2 * (S(3,3) + S(11,11) - 2*S(3,11) + sigma2_n)
    % ---------------------------------------------------------------
    gain_sq = (1 - a_pd)^2;

    var_state_th = Sigma_th(idx_dx_d2, idx_dx_d2) ...
                 + Sigma_th(idx_pmd_prev, idx_pmd_prev) ...
                 - 2 * Sigma_th(idx_dx_d2, idx_pmd_prev);
    C_dpmr_eff = gain_sq * var_state_th;

    var_state_np = Sigma_np(idx_dx_d2, idx_dx_d2) ...
                 + Sigma_np(idx_pmd_prev, idx_pmd_prev) ...
                 - 2 * Sigma_np(idx_dx_d2, idx_pmd_prev);
    % Note: add the direct +sigma2_n contribution (sigma2_n = 1 in unit solve)
    C_np_eff = gain_sq * (var_state_np + 1);

    % ---------------------------------------------------------------
    % 8. Diagnostics
    % ---------------------------------------------------------------
    diagnostics = struct();
    diagnostics.max_eig_A_aug = max_eig;
    diagnostics.eigs_A_aug    = eigs_A_aug;
    diagnostics.dare_iters    = dare_iters;
    diagnostics.dare_err      = dare_err;
    diagnostics.Sigma_th      = Sigma_th;
    diagnostics.Sigma_np      = Sigma_np;
    diagnostics.Sigma_q66     = Sigma_q66;
    diagnostics.Sigma_q77     = Sigma_q77;
    diagnostics.A_e           = A_e;
    diagnostics.Fe            = Fe;
    diagnostics.H             = H;
    diagnostics.B_th          = B_th;
    diagnostics.B_np          = B_np;
    diagnostics.B_na          = B_na;
    diagnostics.B_q66         = B_q66;
    diagnostics.B_q77         = B_q77;
    diagnostics.Var_dpmr_th_state = var_state_th;
    diagnostics.Var_dpmr_np_state = var_state_np;
    diagnostics.rho           = rho;
    diagnostics.idx_dx        = idx_dx;
    diagnostics.idx_dx_d2     = idx_dx_d2;
    diagnostics.idx_e         = idx_e;
    diagnostics.idx_pmd_prev  = idx_pmd_prev;

    % ---------------------------------------------------------------
    % 8b. Optional physical-scaling combination
    %     Sigma_aug_phys = (a_phys/a_nom)^2 * sigma2_dXT * Sigma_th
    %                    + sigma2_n * Sigma_np
    %                    + Q66_abs * Sigma_q66
    %                    + Q77_abs * Sigma_q77
    %     Rationale: each Sigma_X is solved for unit-variance driver, so
    %     physical variance = (physical driver variance) * Sigma_X.
    % ---------------------------------------------------------------
    if ~isempty(opts.physical_scaling)
        ps = opts.physical_scaling;
        if ~isfield(ps, 'Q66_abs'), ps.Q66_abs = 0; end
        if ~isfield(ps, 'Q77_abs'), ps.Q77_abs = 0; end
        if ~isfield(ps, 'sigma2_n'), ps.sigma2_n = 0; end
        if ~isfield(ps, 'a_nom') || ~isfield(ps, 'a_phys') || ~isfield(ps, 'sigma2_dXT')
            error('compute_7state_cdpmr_eff:physical_scaling_missing', ...
                  'opts.physical_scaling requires sigma2_dXT, a_phys, a_nom');
        end
        a_ratio = ps.a_phys / ps.a_nom;
        thermal_variance = (a_ratio^2) * ps.sigma2_dXT;

        Sigma_aug_phys =   thermal_variance * Sigma_th ...
                         + ps.sigma2_n      * Sigma_np ...
                         + ps.Q66_abs       * Sigma_q66 ...
                         + ps.Q77_abs       * Sigma_q77;
        Sigma_aug_phys = 0.5 * (Sigma_aug_phys + Sigma_aug_phys');

        diagnostics.Sigma_aug_phys = Sigma_aug_phys;
        diagnostics.Sigma_e_dx_phys     = Sigma_aug_phys(idx_dx,    idx_dx);     % Var(delta_x[k])
        diagnostics.Sigma_e_dx_d2_phys  = Sigma_aug_phys(idx_dx_d2, idx_dx_d2);  % Var(delta_x[k-2])
        diagnostics.physical_scaling_used = ps;
    end

    if opts.verbose
        fprintf('  [compute_7state_cdpmr_eff] lc=%.3f a_pd=%.4f f0=%.3f\n', ...
                lc, a_pd, opts.f0);
        fprintf('    DARE: %d iters, err=%.3e\n', dare_iters, dare_err);
        fprintf('    max|eig(A_aug)| = %.6f\n', max_eig);
        fprintf('    C_dpmr_eff = %.6f, C_np_eff = %.6f\n', C_dpmr_eff, C_np_eff);
    end
end

% =========================================================================
% =========================================================================
function Sigma = solve_dlyap_robust(A, B)
%SOLVE_DLYAP_ROBUST  Solve X = A*X*A' + B*B' with fallback for ill-conditioned A.
%   Tries dlyapchol first (more stable), falls back to dlyap, then to a
%   direct fixed-point iteration.

    try
        % dlyapchol: returns Cholesky factor S such that Sigma = S'*S
        S = dlyapchol(A, B);
        Sigma = S' * S;
        Sigma = 0.5 * (Sigma + Sigma');
        return;
    catch
        % dlyapchol failed, try dlyap
    end

    try
        Q = B * B';
        Q = 0.5 * (Q + Q');
        Sigma = dlyap(A, Q);
        Sigma = 0.5 * (Sigma + Sigma');
        return;
    catch
        % dlyap failed, use iterative fallback
    end

    % Fixed-point iteration: Sigma[k+1] = A*Sigma[k]*A' + B*B'
    n = size(A, 1);
    Sigma = zeros(n);
    Q = B * B';
    for k = 1:100000
        Sigma_new = A * Sigma * A' + Q;
        Sigma_new = 0.5 * (Sigma_new + Sigma_new');
        err = max(abs(Sigma_new(:) - Sigma(:)));
        Sigma = Sigma_new;
        if err < 1e-12 && k > 100
            break;
        end
    end
    if err >= 1e-6
        warning('solve_dlyap_robust:slow_convergence', ...
                'Iterative Lyapunov did not fully converge: err=%.3e after %d iters', err, k);
    end
end

% =========================================================================
function [L, iters, err] = iterative_dare_7state(Fe, H, Q_kf, R_kf, tol, maxit)
%ITERATIVE_DARE_7STATE  Solve steady-state DARE via Riccati recursion.
%   Returns L so that e[k+1] = (Fe - Fe*L*H)*e[k] - Fe*L*v[k] + w[k]
%
%   Note: the 7-state DARE is ill-conditioned due to marginal eigenvalues
%   at 1 in Fe. Uses Pf_init = Q + I for numerical stability (empirically
%   converges to the same L as 1e6*eye but faster).

    n = size(Fe, 1);
    Pf = Q_kf + eye(n);

    L = zeros(n, size(H, 1));
    err = inf;
    iters = 0;
    for k = 1:maxit
        S = H * Pf * H' + R_kf;
        S = 0.5 * (S + S');
        L = (Pf * H') / S;
        P_post = (eye(n) - L * H) * Pf;
        P_post = 0.5 * (P_post + P_post');
        Pf_new = Fe * P_post * Fe' + Q_kf;
        Pf_new = 0.5 * (Pf_new + Pf_new');

        err = max(abs(Pf_new(:) - Pf(:)));
        iters = k;
        if err < tol
            Pf = Pf_new;
            break;
        end
        Pf = Pf_new;
    end

    % Final L from converged Pf
    S = H * Pf * H' + R_kf;
    S = 0.5 * (S + S');
    L = (Pf * H') / S;
end
