function [C_dpmr_eff, C_np_eff, L, A_aug, diagnostics] = ...
    compute_7state_cdpmr_eff_v2(lc, d_delay, a_pd, Q_kf_scale, R_kf_scale, opts)
%COMPUTE_7STATE_CDPMR_EFF_V2 Augmented Lyapunov for v2 7-state EKF (Eq.19) + IIR HP filter
%
%   v2 port of compute_7state_cdpmr_eff (qr branch / sigma-ratio-filter).
%   Adapted to v2 paper Eq.17 + Σf_d → Eq.19 form F_e (Phase 1 §6, §10):
%       Row 3 = [0, 0, λ_c, -(1+d·(1-λ_c)), 0, -f0, 0]
%   For d=2, λ_c=0.7 → F_e(3,3)=0.7, F_e(3,4)=-1.6 (vs v1: 1.0, -1.0)
%
%   Computes the effective C_dpmr per axis for the relationship
%       Var(dx_r) = C_dpmr_eff * (4 k_B T a) + C_np_eff * sigma2_n
%   used in Eq.13 gain recovery:
%       a_xm = (Var(dx_r) - C_np_eff * sigma2_n) / (C_dpmr_eff * 4 k_B T)
%
%   Uses an 11-dim augmented Lyapunov over:
%       x_aug = [delta_x, dx_d1, dx_d2, e1..e7, pmd_prev]
%
%   Phase 1 assumption: f0 = 0 (positioning baseline). Extension to non-zero
%   f_d is a future task (analogous to qr Phase 2 multiplicative coupling).
%
%   Inputs:
%       lc          scalar closed-loop pole (0 < lc < 1)         e.g. 0.7
%       d_delay     scalar sensor delay in steps                  default 2
%       a_pd        scalar IIR LP coefficient (0 < a_pd < 1)     e.g. 0.05
%       Q_kf_scale  7x1 Q diag scaling for DARE (unit sigma2)
%       R_kf_scale  2x1 R diag scaling for DARE (unit sigma2)
%       opts        struct: .f0 (default 0), .dare_tol (1e-15), .dare_maxit (5000),
%                           .verbose (false), .Fe_form ('eq19' default | 'eq18')
%
%   Outputs:
%       C_dpmr_eff  Var(dx_r) / (4 k_B T a) from thermal driver (dimensionless)
%       C_np_eff    Var(dx_r) / sigma2_n from sensor noise driver (dimensionless)
%       L           7x2 steady-state KF gain
%       A_aug       11x11 augmented transition matrix
%       diagnostics struct with max_eig, Sigma_th, Sigma_np, dare_iters, etc.
%
%   See also: build_eq17_constants, motion_control_law_eq17_7state,
%             compute_7state_cdpmr_eff (v1 / qr branch)

    if nargin < 6 || isempty(opts), opts = struct; end
    if ~isfield(opts, 'f0'),         opts.f0 = 0;     end
    if ~isfield(opts, 'dare_tol'),   opts.dare_tol = 1e-10; end
    if ~isfield(opts, 'dare_maxit'), opts.dare_maxit = 30000; end
    if ~isfield(opts, 'verbose'),    opts.verbose = false; end
    if ~isfield(opts, 'Fe_form'),    opts.Fe_form = 'eq19'; end

    if nargin < 2 || isempty(d_delay), d_delay = 2; end

    % ---------------------------------------------------------------
    % 1. Build F_e (v2 Eq.19 form) and H for the 7-state EKF
    %    Row 3 (Eq.19, Option I, slowly-varying x_D):
    %       F_e(3,:) = [0, 0, λ_c, -(1+d·(1-λ_c)), 0, -f0, 0]
    %    (v1 / qr used Eq.18 form Row 3 = [0,0,1,-1,0,-f0,0])
    % ---------------------------------------------------------------
    use_eq18 = strcmpi(opts.Fe_form, 'eq18');
    if use_eq18
        % v1-style direct partition: F_e(3,1)=-(1-lc), (3,3)=1, (3,4)=-1
        Fe = [0           1 0  0  0  0       0;
              0           0 1  0  0  0       0;
             -(1-lc)      0 1 -1  0 -opts.f0 0;
              0           0 0  1  1  0       0;
              0           0 0  0  1  0       0;
              0           0 0  0  0  1       1;
              0           0 0  0  0  0       1];
    else
        % v2 Eq.19 form (default): F_e(3,3)=lc, F_e(3,4)=-(1 + d·(1-lc))
        Fe34 = -(1 + d_delay * (1 - lc));
        Fe = [0 1 0   0     0   0       0;
              0 0 1   0     0   0       0;
              0 0 lc  Fe34  0  -opts.f0 0;
              0 0 0   1     1   0       0;
              0 0 0   0     1   0       0;
              0 0 0   0     0   1       1;
              0 0 0   0     0   0       1];
    end

    H = [1 0 0 0 0 0 0;
         0 0 0 0 0 1 0];

    % ---------------------------------------------------------------
    % 2. Build Q_kf and R_kf for DARE (using unit sigma2_dXT)
    %    DARE solve uses unit-scaled (Q,R); physical scaling applies later
    %    via separate Lyapunov drivers (Sigma_th, Sigma_np, ...).
    % ---------------------------------------------------------------
    sigma2_dXT_unit = 1;
    Q_kf = sigma2_dXT_unit * diag(Q_kf_scale(:));
    R_kf = sigma2_dXT_unit * diag(R_kf_scale(:));

    % ---------------------------------------------------------------
    % 3. Iterative DARE for steady-state gain L (7-state DARE is
    %    ill-conditioned due to Jordan blocks at eig=1, so we use
    %    Riccati recursion with Pf_init = Q + I).
    % ---------------------------------------------------------------
    [L, dare_iters, dare_err] = iterative_dare_7state(Fe, H, Q_kf, R_kf, ...
                                                       opts.dare_tol, opts.dare_maxit);

    if any(~isfinite(L(:)))
        error('compute_7state_cdpmr_eff_v2:dare_failed', ...
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
    %
    %   Row 1 (delta_x dynamics): from v2 Eq.19 closed-loop tracking,
    %     delta_x[k+1] = lc * delta_x[k] - e3-error - e4-error - f0*e6 - thermal
    %   In Eq.19, the closed-loop pole appears explicitly. The row coefficient
    %   on delta_x is exactly lc (matching qr Phase 1 derivation).
    %   The Σf_d substitution under slowly-varying x_D adds no extra rows here.
    % ---------------------------------------------------------------
    n_aug = 11;
    A_aug = zeros(n_aug);

    idx_dx       = 1;
    idx_dx_d1    = 2;
    idx_dx_d2    = 3;
    idx_e        = 4:10;           % e1..e7
    idx_pmd_prev = 11;

    % Row 1: delta_x[k+1] = lc*delta_x + (1-lc)*e3 - e4 - f0*e6 - a*f_T
    %   (same closed-loop tracking error dynamics as v1, since the
    %   scenario assumption is steady-state f_d ≈ 0 / slowly-varying.)
    A_aug(idx_dx, idx_dx)       = lc;
    A_aug(idx_dx, idx_e(3))     = 1 - lc;
    A_aug(idx_dx, idx_e(4))     = -1;
    A_aug(idx_dx, idx_e(6))     = -opts.f0;

    % Row 2: dx_d1[k+1] = delta_x[k]
    A_aug(idx_dx_d1, idx_dx)    = 1;

    % Row 3: dx_d2[k+1] = dx_d1[k]
    A_aug(idx_dx_d2, idx_dx_d1) = 1;

    % Rows 4-10: 7-state EKF closed-loop error dynamics (uses v2 F_e)
    A_aug(idx_e, idx_e)         = A_e;

    % Row 11: pmd_prev[k+1] = del_pmd[k] = (1-a_pd)*pmd_prev[k] + a_pd*del_pm[k]
    %   del_pm[k] = dx_d2[k] + n_p[k]; deterministic part couples via dx_d2.
    A_aug(idx_pmd_prev, idx_pmd_prev) = 1 - a_pd;
    A_aug(idx_pmd_prev, idx_dx_d2)    = a_pd;

    % Stability check
    eigs_A_aug = eig(A_aug);
    max_eig = max(abs(eigs_A_aug));
    if max_eig >= 1 - 1e-10
        warning('compute_7state_cdpmr_eff_v2:unstable', ...
                'A_aug eigenvalue %.6f too close to 1 (a_pd=%.4g, lc=%.3g)', ...
                max_eig, a_pd, lc);
    end

    % ---------------------------------------------------------------
    % 5. Build noise input vectors (all 11x1)
    % ---------------------------------------------------------------
    % Thermal: a*f_T enters both delta_x (row 1) and e3 (row 6 = idx_e(3)),
    % same source.
    B_th = zeros(n_aug, 1);
    B_th(idx_dx)    = -1;
    B_th(idx_e(3))  = -1;

    % Position measurement noise n_p: enters 7 error states via -Fe*L(:,1)
    % and IIR pmd_prev via a_pd (direct coupling).
    B_np = zeros(n_aug, 1);
    B_np(idx_e)         = -(Fe * L(:, 1));
    B_np(idx_pmd_prev)  = a_pd;

    % Gain measurement noise n_a: enters 7 error states via -Fe*L(:,2)
    B_na = zeros(n_aug, 1);
    B_na(idx_e)         = -(Fe * L(:, 2));

    % ---------------------------------------------------------------
    % 6. Solve unit-variance Lyapunov equations
    % ---------------------------------------------------------------
    Sigma_th = solve_dlyap_robust(A_aug, B_th);
    Sigma_np = solve_dlyap_robust(A_aug, B_np);
    Sigma_na = solve_dlyap_robust(A_aug, B_na);

    % ---------------------------------------------------------------
    % 7. Extract C_dpmr_eff and C_np_eff
    %    dx_r[k] = (1-a_pd) * (dx_d2[k] - pmd_prev[k] + n_p[k])
    %    Var(dx_r) = (1-a_pd)^2 * (S(3,3) + S(11,11) - 2*S(3,11) + sigma2_n)
    %
    %   v2 measurement chain matches v1: HP-by-subtraction LP filter, where
    %   dx_bar_m_new = (1-a_pd)*dx_bar_m_old + a_pd*delta_x_m
    %   dx_r       = delta_x_m - dx_bar_m_new
    %              = (1-a_pd) * (delta_x_m - dx_bar_m_old)
    % ---------------------------------------------------------------
    gain_sq = (1 - a_pd)^2;

    var_state_th = Sigma_th(idx_dx_d2, idx_dx_d2) ...
                 + Sigma_th(idx_pmd_prev, idx_pmd_prev) ...
                 - 2 * Sigma_th(idx_dx_d2, idx_pmd_prev);
    C_dpmr_eff = gain_sq * var_state_th;

    var_state_np = Sigma_np(idx_dx_d2, idx_dx_d2) ...
                 + Sigma_np(idx_pmd_prev, idx_pmd_prev) ...
                 - 2 * Sigma_np(idx_dx_d2, idx_pmd_prev);
    C_np_eff = gain_sq * (var_state_np + 1);   % +1 for direct sigma2_n contribution

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
    diagnostics.Sigma_na      = Sigma_na;
    diagnostics.A_e           = A_e;
    diagnostics.Fe            = Fe;
    diagnostics.H             = H;
    diagnostics.B_th          = B_th;
    diagnostics.B_np          = B_np;
    diagnostics.B_na          = B_na;
    diagnostics.Var_dxr_th_state = var_state_th;
    diagnostics.Var_dxr_np_state = var_state_np;
    diagnostics.idx_dx        = idx_dx;
    diagnostics.idx_dx_d2     = idx_dx_d2;
    diagnostics.idx_e         = idx_e;
    diagnostics.idx_pmd_prev  = idx_pmd_prev;
    diagnostics.Fe_form       = opts.Fe_form;

    if opts.verbose
        fprintf('  [compute_7state_cdpmr_eff_v2] lc=%.3f a_pd=%.4f f0=%.3f form=%s\n', ...
                lc, a_pd, opts.f0, opts.Fe_form);
        fprintf('    DARE: %d iters, err=%.3e\n', dare_iters, dare_err);
        fprintf('    max|eig(A_aug)| = %.6f\n', max_eig);
        fprintf('    C_dpmr_eff = %.6f, C_np_eff = %.6f\n', C_dpmr_eff, C_np_eff);
    end
end

% =========================================================================
function Sigma = solve_dlyap_robust(A, B)
%SOLVE_DLYAP_ROBUST  Solve X = A*X*A' + B*B' with fallback for ill-conditioned A.

    try
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
    err = inf;
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
        warning('compute_7state_cdpmr_eff_v2:slow_lyap_convergence', ...
                'Iterative Lyapunov did not fully converge: err=%.3e after %d iters', err, k);
    end
end

% =========================================================================
function [L, iters, err] = iterative_dare_7state(Fe, H, Q_kf, R_kf, tol, maxit)
%ITERATIVE_DARE_7STATE  Solve steady-state DARE via Riccati recursion.

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
