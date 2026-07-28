function out = verify_aprime_blackbox_v1_ensemble(opts)
%VERIFY_APRIME_BLACKBOX_V1_ENSEMBLE  Cross-seed ensemble unbiasedness check
%   for the black-box regression estimator of a' (see reference/
%   eq17_analysis/derivation/aprime_blackbox_regression.tex).
%
%   Same static near-wall hold scenario as verify_aprime_blackbox_v1_hold,
%   but instead of a single-trajectory real-time EWMA recursion (which has
%   few effective INDEPENDENT samples due to autocorrelation -- diagnosed
%   in verify_aprime_blackbox_v1_hold), this computes ONE BATCH regression
%   estimate per seed (pooling the whole steady window as one sample), then
%   averages across many genuinely independent seeds. Mirrors
%   aram_variance.tex's N=100 cross-seed methodology: separates "is the
%   regression formula unbiased" (high statistical power via independent
%   seeds) from "how fast does a single real-time deployment converge"
%   (already answered pessimistically by the CRLB dwell-time analysis and
%   by verify_aprime_blackbox_v1_hold's single-trajectory EWMA test).
%
%   Per seed, per axis (steady window t >= t_steady):
%       a_hat_prime_seed = -mean((a_xm - a_m_det) .* dx_r_wn) / mean(dx_r_wn.^2)
%   Cross-seed mean/SE then compared to the (near-constant) oracle
%   a'_true(h_bar) -- never fed to the estimator.
%
%   out = verify_aprime_blackbox_v1_ensemble()
%   out = verify_aprime_blackbox_v1_ensemble(opts)  opts.seeds (default 1:60),
%       opts.T_sim (default 8 sec), opts.t_steady (default 1.5 sec)
%
%   See also: verify_aprime_blackbox_v0, verify_aprime_blackbox_v1_hold,
%             reference/eq17_analysis/derivation/aprime_blackbox_regression.tex

    if nargin < 1 || isempty(opts); opts = struct(); end
    if ~isfield(opts, 'seeds');    opts.seeds    = 1:60;  end
    if ~isfield(opts, 'T_sim');    opts.T_sim    = 8;     end
    if ~isfield(opts, 't_steady'); opts.t_steady = 1.5;   end
    if ~isfield(opts, 'verbose');  opts.verbose  = true;  end

    here = fileparts(mfilename('fullpath'));
    proj = fullfile(here, '..', '..');
    addpath(genpath(proj));

    % ------------------------------------------------------------------
    % Config: same static near-wall hold as verify_aprime_blackbox_v1_hold
    % (h_bar ~ 1.78, gate-free, a_det_lp slow).
    % ------------------------------------------------------------------
    pc  = physical_constants();
    cfg = user_config();
    cfg.eq17_variant       = '4state';
    cfg.enable_wall_effect = true;
    cfg.trajectory_type    = 'positioning';
    cfg.h_init             = 4;             % h_bar ~= 1.78
    cfg.h_bottom           = 4;
    cfg.amplitude          = 0;
    cfg.h_min              = 1.5 * pc.R;
    cfg.h_bar_safe         = 1.2;
    cfg.ctrl_enable        = true;
    cfg.thermal_enable     = true;
    cfg.meas_noise_enable  = true;
    cfg.meas_noise_std     = [0.00062; 0.00057; 0.00331];
    cfg.lambda_c           = 0.7;
    cfg.a_pd               = 0.05;
    cfg.a_cov              = 0.05;
    cfg.suppress_xD        = true;
    cfg.use_am_lpf         = false;
    cfg.a_det              = 0.0002;
    cfg.T_sim              = opts.T_sim;

    ns       = numel(opts.seeds);
    t_steady = opts.t_steady;
    ap_seed  = nan(ns, 3);
    apt_seed = nan(ns, 3);

    for si = 1:ns
        ro = struct('seed', opts.seeds(si), 'verbose', false, 'collect_diag', true);
        s  = run_pure_simulation(cfg, ro);
        P  = s.meta.params_value;
        Ts = P.common.Ts;
        N  = size(s.diag.a_xm, 1);
        t  = (0:N-1)' * Ts;
        win = t >= t_steady;

        a_xm   = s.diag.a_xm(win,:);      % [Nwin x 3]
        a_mdet = s.diag.a_m_det(win,:);   % [Nwin x 3]
        dxr    = s.diag.dx_r(win,:);      % [Nwin x 3]

        w  = P.wall.w_hat(:); pz = P.wall.pz; R = P.common.R;
        dxr_wn = dxr * w;                 % [Nwin x 1], wall-normal projection

        Cov_batch = mean((a_xm - a_mdet) .* dxr_wn, 1);   % [1 x 3]
        Var_batch = mean(dxr_wn.^2, 1);                    % scalar
        ap_seed(si,:) = -Cov_batch / Var_batch;

        % --- oracle a'_true, averaged over the same window (comparison only) ---
        % NOTE (alignment fix): a_true_out(k,:) is logged at the PRE-integration
        % position (the p_curr the controller/gain acted on at step k), while
        % p_true_out(k,:) is logged AFTER that step's integration -- i.e.
        % p_true_out(k,:) is one sample AHEAD of what a_true_out(k,:) reflects.
        % This is the same one-sample logging-alignment issue already documented
        % in project memory (aram_axm_decomposition: "correct delta_x[k] =
        % x_d[k] - x[k-1]"). Pair a_true_out(k,:) with p_true_out(k-1,:) instead
        % (p0 for k=1) so h_bar_true and a_true refer to the SAME instant.
        p0 = P.common.p0(:).';
        p_true_pre = [p0; s.p_true_out(1:end-1,:)];   % pre-integration position, all k
        h_bar_true = max((p_true_pre(win,:) * w - pz) / R, 1.001);
        a_true = s.a_true_out(win,:);
        Nwin = size(a_true,1);
        K_h_true = zeros(Nwin, 3);
        for k = 1:Nwin
            [~, ~, derivs] = calc_correction_functions(h_bar_true(k), true);
            K_h_true(k,:) = [derivs.K_h_para, derivs.K_h_para, derivs.K_h_perp];
        end
        apt_seed(si,:) = mean(-a_true .* K_h_true / R, 1);
    end

    out.ap_seed  = ap_seed;
    out.apt_seed = apt_seed;
    out.ap_mean  = mean(ap_seed, 1);
    out.ap_se    = std(ap_seed, 0, 1) / sqrt(ns);
    out.apt_mean = mean(apt_seed, 1);
    diff_mean    = out.ap_mean - out.apt_mean;
    out.t_stat   = diff_mean ./ out.ap_se;
    out.rel_bias = diff_mean ./ out.apt_mean;

    if opts.verbose
        ax = {'x','y','z'};
        fprintf('\n[V1-ensemble: static near-wall hold h_bar~1.78, %d independent seeds]\n', ns);
        fprintf('  %-28s %10s %10s %10s\n', 'metric / axis', ax{:});
        fprintf('  %-28s %10.4g %10.4g %10.4g\n', 'mean(a_hat_prime) [1/pN]', out.ap_mean);
        fprintf('  %-28s %10.4g %10.4g %10.4g\n', 'cross-seed SE', out.ap_se);
        fprintf('  %-28s %10.4g %10.4g %10.4g\n', 'oracle a_prime_true [1/pN]', out.apt_mean);
        fprintf('  %-28s %10.1f %10.1f %10.1f\n', 'rel. bias (%%)', 100*out.rel_bias);
        fprintf('  %-28s %10.2f %10.2f %10.2f\n', 't-stat (bias/SE)', out.t_stat);
        fprintf('  (|t-stat| ~< 2 is consistent with zero bias at this sample size)\n');
    end
end
