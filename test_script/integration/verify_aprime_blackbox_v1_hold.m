function out = verify_aprime_blackbox_v1_hold(opts)
%VERIFY_APRIME_BLACKBOX_V1_HOLD  V1' (hold variant) oracle-recovery check
%   for the black-box regression estimator of a' (see reference/
%   eq17_analysis/derivation/aprime_blackbox_regression.tex).
%
%   Static near-wall HOLD scenario (trajectory_type='positioning'), unlike
%   verify_aprime_blackbox_v1's active 1 Hz oscillation. Two issues found in
%   the osc-scenario V1 run are fixed here:
%
%   (1) Locality-vs-SNR tension: the osc scenario sweeps the wall-normal
%       height at 1 Hz (period ~1600 samples), so once the regression's
%       effective window N_eff approaches that period, the "a' is locally
%       constant over the averaging window" assumption breaks -- you are
%       averaging across a stretch of trajectory where the TRUE a' itself
%       is swinging through both signs. A static hold removes this: a_true
%       is (near-)constant in time, so N_eff can be pushed as large as
%       needed for SNR without violating locality.
%   (2) Timescale ordering: a_level (=a_m_det, pole a_det_lp) must be SLOWER
%       than the regression window (pole a_cov) for Cov(a_level,dx_r)~=0 to
%       hold cleanly. The osc run used the default a_det_lp=0.005 (N_eff
%       ~200) while sweeping a_cov down to N_eff=2000, inverting the
%       assumed ordering. Here a_det_lp is set slower than every swept
%       a_cov (N_eff_level ~5000, all sweep points N_eff <= 500... see
%       opts.acov_sweep).
%
%   Ground truth a'(h_bar) = -a_x*K_h/R uses the TRUE noise-free position
%   (comparison only, never fed to the estimator).
%
%   out = verify_aprime_blackbox_v1_hold()
%   out = verify_aprime_blackbox_v1_hold(opts)  opts.seeds (default 1:5),
%       opts.acov_sweep (default [0.05 0.01 0.002 0.0005 0.0002]),
%       opts.T_sim (default 20 sec), opts.t_steady (default 5 sec)
%
%   See also: verify_aprime_blackbox_v0, verify_aprime_blackbox_v1,
%             run_pure_simulation,
%             reference/eq17_analysis/derivation/aprime_blackbox_regression.tex

    if nargin < 1 || isempty(opts); opts = struct(); end
    if ~isfield(opts, 'seeds');      opts.seeds      = 1:5;                                  end
    if ~isfield(opts, 'acov_sweep'); opts.acov_sweep = [0.05, 0.01, 0.002, 0.0005, 0.0002];   end
    if ~isfield(opts, 'T_sim');      opts.T_sim      = 20;                                   end
    if ~isfield(opts, 't_steady');   opts.t_steady   = 5;                                     end
    if ~isfield(opts, 'verbose');    opts.verbose    = true;                                  end

    here = fileparts(mfilename('fullpath'));
    proj = fullfile(here, '..', '..');
    addpath(genpath(proj));

    % ------------------------------------------------------------------
    % Config: static near-wall hold, h_bar ~ 1.78 (sizeable a', gate-free).
    % a_det_lp deliberately set SLOWER than every swept a_cov (fix #2).
    % ------------------------------------------------------------------
    pc  = physical_constants();
    cfg = user_config();
    cfg.eq17_variant       = '4state';
    cfg.enable_wall_effect = true;
    cfg.trajectory_type    = 'positioning';    % static hold, not oscillating
    cfg.h_init             = 4;                % h_bar = 4/2.25 ~= 1.78
    cfg.h_bottom           = 4;                % no descent
    cfg.amplitude          = 0;                % no oscillation
    cfg.h_min              = 1.5 * pc.R;       % 3.375 um < h_init, OK
    cfg.h_bar_safe         = 1.2;               % gate-free (h_bar ~1.78 >> 1.2)
    cfg.ctrl_enable        = true;
    cfg.thermal_enable     = true;
    cfg.meas_noise_enable  = true;
    cfg.meas_noise_std     = [0.00062; 0.00057; 0.00331];   % [um]
    cfg.lambda_c           = 0.7;
    cfg.a_pd               = 0.05;
    cfg.a_cov              = 0.05;             % controller's own internal a_cov (unchanged)
    cfg.suppress_xD        = true;
    cfg.use_am_lpf         = false;
    cfg.a_det              = 0.0002;           % a_level pole: N_eff~5000, slower than every sweep point
    cfg.T_sim              = opts.T_sim;

    acov_sweep = opts.acov_sweep;
    nA = numel(acov_sweep);
    ns = numel(opts.seeds);
    t_steady = opts.t_steady;

    corr_all = nan(ns, 3, nA);
    bias_all = nan(ns, 3, nA);

    for si = 1:ns
        ro = struct('seed', opts.seeds(si), 'verbose', false, 'collect_diag', true);
        s  = run_pure_simulation(cfg, ro);
        P  = s.meta.params_value;
        Ts = P.common.Ts;
        N  = size(s.diag.a_xm, 1);
        t  = (0:N-1)' * Ts;

        a_xm   = s.diag.a_xm;             % [N x 3] um/pN
        a_mdet = s.diag.a_m_det;          % [N x 3] um/pN (a_level, slow pole a_det_lp)
        dxr    = s.diag.dx_r;             % [N x 3] um

        % --- Ground-truth a'(h_bar) = -a_x*K_h/R at the TRUE noise-free position ---
        w  = P.wall.w_hat(:); pz = P.wall.pz; R = P.common.R;
        dxr_wn = dxr * w;                 % [N x 1] um, wall-normal component (shared Delta_h proxy)
        h_bar_true = max((s.p_true_out * w - pz) / R, 1.001);
        a_true = s.a_true_out;            % [N x 3], true gain (pre-integration position)
        K_h_true = zeros(N, 3);
        for k = 1:N
            [~, ~, derivs] = calc_correction_functions(h_bar_true(k), true);
            K_h_true(k,:) = [derivs.K_h_para, derivs.K_h_para, derivs.K_h_perp];
        end
        a_prime_true = -a_true .* K_h_true / R;   % [N x 3], units 1/pN

        win = t >= t_steady;

        for ai = 1:nA
            ac = acov_sweep(ai);
            Cov_hat = zeros(N, 3);
            Var_hat = zeros(N, 1);
            for k = 2:N
                Cov_hat(k,:) = (1 - ac) * Cov_hat(k-1,:) + ac * (a_xm(k,:) - a_mdet(k,:)) * dxr_wn(k);
                Var_hat(k)   = (1 - ac) * Var_hat(k-1)   + ac * dxr_wn(k)^2;
            end
            a_prime_hat = -Cov_hat ./ max(Var_hat, eps);   % [N x 3]

            for ax = 1:3
                c = corrcoef(a_prime_hat(win,ax), a_prime_true(win,ax));
                corr_all(si,ax,ai) = c(1,2);
            end
            bias_all(si,:,ai) = mean(a_prime_hat(win,:) - a_prime_true(win,:), 1) ...
                                 ./ mean(abs(a_prime_true(win,:)), 1);

            if si == 1 && ai == nA
                out.t_vec              = t;
                out.a_prime_hat_seed1  = a_prime_hat;
                out.a_prime_true_seed1 = a_prime_true;
                out.h_bar_true_seed1   = h_bar_true;
            end
        end
    end

    out.acov_sweep = acov_sweep;
    out.corr_all   = corr_all;
    out.bias_all   = bias_all;
    out.corr_mean  = squeeze(mean(corr_all, 1));   % [3 x nA]
    out.bias_mean  = squeeze(mean(bias_all, 1));   % [3 x nA]

    if opts.verbose
        ax = {'x','y','z'};
        fprintf('\n[V1-hold oracle recovery: static near-wall hold h_bar~1.78, %d seeds]\n', ns);
        fprintf('  %-16s %8s | %8s %8s %8s\n', 'a_cov', 'N_eff', ax{:});
        for ai = 1:nA
            fprintf('  corr:  %-9.4f %8d | %8.3f %8.3f %8.3f\n', ...
                acov_sweep(ai), round(1/acov_sweep(ai)), out.corr_mean(1,ai), out.corr_mean(2,ai), out.corr_mean(3,ai));
        end
        fprintf('\n');
        for ai = 1:nA
            fprintf('  rel.bias: %-6.4f %8d | %8.3f %8.3f %8.3f\n', ...
                acov_sweep(ai), round(1/acov_sweep(ai)), out.bias_mean(1,ai), out.bias_mean(2,ai), out.bias_mean(3,ai));
        end
    end
end
