function out = verify_aprime_blackbox_v0(opts)
%VERIFY_APRIME_BLACKBOX_V0  V0 null/bias check for the black-box regression
%   estimator of a' (see reference/eq17_analysis/derivation/
%   aprime_blackbox_regression.tex).
%
%   Free-space run (wall disabled -> a' == 0 everywhere, exactly). Recomputes
%   a_hat_prime[k] = -Cov_hat[k] / sigma2_dxr_hat[k] OFFLINE from the logged
%   diag.a_xm, diag.a_m_det, diag.dx_r, diag.sigma2_dxr_hat -- all already
%   produced by motion_control_law_eq17_4state.m. No controller change.
%
%   Pass criterion (per the derivation doc): mean(a_hat_prime) over a
%   steady-state window should be consistent with zero at the sampling-noise
%   level (|mean| / cross-seed SE ~ O(1)) -- confirms the Isserlis
%   cancellation (Cov(a_level,dx_r)=0, Cov(n_a,dx_r)=0) holds in the actual
%   simulated signal, not just the idealized-Gaussian argument.
%
%   out = verify_aprime_blackbox_v0()
%   out = verify_aprime_blackbox_v0(opts)  opts.seeds (default 1:5),
%                                          opts.T_sim (default 8 sec)
%
%   See also: run_pure_simulation, motion_control_law_eq17_4state,
%             reference/eq17_analysis/derivation/aprime_blackbox_regression.tex

    if nargin < 1 || isempty(opts); opts = struct(); end
    if ~isfield(opts, 'seeds');   opts.seeds   = 1:5;  end
    if ~isfield(opts, 'T_sim');   opts.T_sim   = 8;    end
    if ~isfield(opts, 'verbose'); opts.verbose = true; end

    here = fileparts(mfilename('fullpath'));
    proj = fullfile(here, '..', '..');
    addpath(genpath(proj));

    % ------------------------------------------------------------------
    % Config: free-space (wall disabled) so a_x == a_nom everywhere and
    % a' == 0 exactly -- the H0 null case the derivation's Isserlis
    % argument must survive.
    % ------------------------------------------------------------------
    cfg = user_config();
    cfg.eq17_variant       = '4state';
    cfg.enable_wall_effect = false;        % <-- V0: a' == 0 everywhere
    cfg.trajectory_type    = 'positioning';
    cfg.h_init             = 20;
    cfg.h_bottom           = 20;
    cfg.amplitude          = 0;
    cfg.ctrl_enable        = true;
    cfg.thermal_enable     = true;
    cfg.meas_noise_enable  = true;
    cfg.meas_noise_std     = [0.00062; 0.00057; 0.00331];   % [um]
    cfg.lambda_c           = 0.7;
    cfg.a_pd               = 0.05;
    cfg.a_cov              = 0.05;
    cfg.suppress_xD        = true;
    cfg.use_am_lpf         = false;
    cfg.a_det              = 0.005;
    cfg.T_sim              = opts.T_sim;

    a_cov = cfg.a_cov;
    t_steady = 1.0;    % skip IIR warmup transient

    ns = numel(opts.seeds);
    ap_mean = nan(ns, 3);
    ap_std  = nan(ns, 3);

    for si = 1:ns
        ro = struct('seed', opts.seeds(si), 'verbose', false, 'collect_diag', true);
        s  = run_pure_simulation(cfg, ro);
        Ts = s.meta.params_value.common.Ts;
        N  = size(s.diag.a_xm, 1);
        t  = (0:N-1)' * Ts;

        a_xm   = s.diag.a_xm;             % [N x 3] um/pN
        a_mdet = s.diag.a_m_det;          % [N x 3] um/pN (a_level)
        dxr    = s.diag.dx_r;             % [N x 3] um
        svar   = s.diag.sigma2_dxr_hat;   % [N x 3] um^2

        % --- Recursive Cov_hat and a_hat_prime (aprime_blackbox_regression.tex) ---
        Cov_hat = zeros(N, 3);
        for k = 2:N
            Cov_hat(k,:) = (1 - a_cov) * Cov_hat(k-1,:) ...
                           + a_cov * (a_xm(k,:) - a_mdet(k,:)) .* dxr(k,:);
        end
        svar_safe = max(svar, eps);
        a_prime_hat = -Cov_hat ./ svar_safe;   % [N x 3], units 1/pN

        win = t >= t_steady;
        ap_mean(si,:) = mean(a_prime_hat(win,:), 1);
        ap_std(si,:)  = std(a_prime_hat(win,:), 0, 1);

        if si == 1
            out.t_vec = t;
            out.a_prime_hat_seed1 = a_prime_hat;
        end
    end

    out.ap_mean = ap_mean;
    out.ap_std  = ap_std;
    out.ap_mean_over_seeds = mean(ap_mean, 1);
    out.ap_se_over_seeds   = std(ap_mean, 0, 1) / sqrt(ns);   % cross-seed SE of the mean

    if opts.verbose
        ax = {'x','y','z'};
        fprintf('\n[V0 null/bias check: free-space, a''==0 everywhere, %d seeds, T_sim=%.1fs]\n', ns, opts.T_sim);
        fprintf('  %-28s %10s %10s %10s\n', 'metric / axis', ax{:});
        fprintf('  %-28s %10.4g %10.4g %10.4g\n', 'mean(a_hat_prime) [1/pN]', out.ap_mean_over_seeds);
        fprintf('  %-28s %10.4g %10.4g %10.4g\n', 'cross-seed SE of mean', out.ap_se_over_seeds);
        fprintf('  %-28s %10.4g %10.4g %10.4g\n', 'within-run std(a_hat_prime)', mean(ap_std,1));
        for ax_i = 1:3
            ratio = abs(out.ap_mean_over_seeds(ax_i)) / max(out.ap_se_over_seeds(ax_i), eps);
            fprintf('  axis %s: |mean|/SE = %6.2f  (expect O(1) if consistent with zero)\n', ax{ax_i}, ratio);
        end
    end
end
