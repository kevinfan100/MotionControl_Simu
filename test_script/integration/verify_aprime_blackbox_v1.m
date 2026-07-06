function out = verify_aprime_blackbox_v1(opts)
%VERIFY_APRIME_BLACKBOX_V1  V1 oracle-recovery check for the black-box
%   regression estimator of a' (see reference/eq17_analysis/derivation/
%   aprime_blackbox_regression.tex).
%
%   Near-wall 1 Hz oscillation scenario (same shape as aram_variance.tex /
%   verify_eq17_4state_suite.m's 'osc_1hz', extended to more cycles for a
%   longer near-wall dwell). Recomputes a_hat_prime[k] OFFLINE from the
%   logged diag.a_xm, diag.a_m_det, diag.dx_r (same recursion as V0), but
%   sweeps the ANALYSIS-TIME EWMA rate (independent of the controller's own
%   internal a_cov, which stays fixed) to show precision vs. effective
%   window N_eff = 1/a_cov -- this is the "dwell time" axis the derivation
%   doc's Precision section predicts (single-sample a_ram buried ~73x,
%   needs N_eff ~ 5300 for O(1) relative precision).
%
%   Ground truth a'(h_bar) = -a_x*K_h/R (oracle, from calc_correction_functions
%   at the TRUE noise-free wall-normal height) is used ONLY for comparison,
%   never fed to the estimator.
%
%   out = verify_aprime_blackbox_v1()
%   out = verify_aprime_blackbox_v1(opts)  opts.seeds (default 1:8),
%       opts.acov_sweep (default [0.05 0.01 0.002 0.0005])
%
%   See also: verify_aprime_blackbox_v0, run_pure_simulation,
%             reference/eq17_analysis/derivation/aprime_blackbox_regression.tex

    if nargin < 1 || isempty(opts); opts = struct(); end
    if ~isfield(opts, 'seeds');      opts.seeds      = 1:8;                        end
    if ~isfield(opts, 'acov_sweep'); opts.acov_sweep = [0.05, 0.01, 0.002, 0.0005]; end
    if ~isfield(opts, 'verbose');    opts.verbose    = true;                       end

    here = fileparts(mfilename('fullpath'));
    proj = fullfile(here, '..', '..');
    addpath(genpath(proj));

    % ------------------------------------------------------------------
    % Config: near-wall 1 Hz oscillation (h_bar trough ~1.2), extended to
    % 8 cycles for a longer near-wall dwell (helps the slower sweep rates
    % accumulate enough effective samples).
    % ------------------------------------------------------------------
    pc  = physical_constants();
    cfg = user_config();
    cfg.eq17_variant       = '4state';
    cfg.enable_wall_effect = true;
    cfg.trajectory_type    = 'osc';
    cfg.h_init             = 50;
    cfg.h_bottom           = 2.7;          % h_bar trough ~ 1.2
    cfg.amplitude          = 2.5;
    cfg.frequency          = 1.0;
    cfg.n_cycles           = 8;
    cfg.t_hold             = 0.5;
    cfg.t_descend_override = 1.0;
    cfg.h_min              = 1.05 * pc.R;
    cfg.h_bar_safe         = 1;            % gate-free even at trough h_bar~1.2
    cfg.ctrl_enable        = true;
    cfg.thermal_enable     = true;
    cfg.meas_noise_enable  = true;
    cfg.meas_noise_std     = [0.00062; 0.00057; 0.00331];   % [um]
    cfg.lambda_c           = 0.7;
    cfg.a_pd               = 0.05;
    cfg.a_cov              = 0.05;         % controller's own internal a_cov (unchanged)
    cfg.suppress_xD        = true;
    cfg.use_am_lpf         = false;
    cfg.a_det              = 0.005;
    cfg.T_sim              = 10;           % 0.5 hold + 1.0 descend + 8 cycles @ 1Hz

    acov_sweep = opts.acov_sweep;
    nA = numel(acov_sweep);
    ns = numel(opts.seeds);
    t_steady = 1.5;   % post hold+descend (matches verify_eq17_4state_suite osc_1hz)

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
        a_mdet = s.diag.a_m_det;          % [N x 3] um/pN
        dxr    = s.diag.dx_r;             % [N x 3] um

        % --- Ground-truth a'(h_bar) = -a_x*K_h/R at the TRUE noise-free position ---
        w  = P.wall.w_hat(:); pz = P.wall.pz; R = P.common.R;
        % Wall-normal projection of dx_r: a_x, a_y, a_z all depend on the SAME
        % scalar height h_bar (via c_para/c_perp) -- the honest Delta_h proxy is
        % dot(dx_r, w_hat), shared across all three gain channels, NOT each
        % axis's own (largely lateral, height-irrelevant) dx_r component.
        dxr_wn = dxr * w;                 % [N x 1] um, wall-normal component
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
            a_prime_hat = -Cov_hat ./ max(Var_hat, eps);   % [N x 3], Var_hat broadcasts over columns

            for ax = 1:3
                c = corrcoef(a_prime_hat(win,ax), a_prime_true(win,ax));
                corr_all(si,ax,ai) = c(1,2);
            end
            bias_all(si,:,ai) = mean(a_prime_hat(win,:) - a_prime_true(win,:), 1) ...
                                 ./ mean(abs(a_prime_true(win,:)), 1);

            if si == 1 && ai == nA   % slowest sweep point: keep the seed-1 time series
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
        fprintf('\n[V1 oracle recovery: near-wall 1Hz osc (%d cycles), %d seeds]\n', cfg.n_cycles, ns);
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
