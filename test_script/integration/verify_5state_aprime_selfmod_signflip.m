function out = verify_5state_aprime_selfmod_signflip(opts)
%VERIFY_5STATE_APRIME_SELFMOD_SIGNFLIP  Does use_selfmod=true recover the
%   correct-sign a'_x on the SAME static-hold scenario that produced a
%   statistically significant wrong-sign result in the external regression
%   test (verify_aprime_blackbox_v1_ensemble.m, 60 seeds, h_bar~1.78)? See
%   reference/eq17_analysis/derivation/5state_aprime_unified.tex Section 6.
%
%   Unlike the external test, this reads diag.delta_a_hat (the driver-logged
%   alias of slot-5 a'_x) DIRECTLY from the real 5-state controller
%   (motion_control_law_eq17_5state_aprime.m with use_selfmod=true) -- no
%   external Cov/Var regression, no dx_r. If Section 6's hypothesis
%   (raw dx_r vs filtered delta_x_hat_3/1) is correct, this should give the
%   RIGHT sign where the external test gave the wrong one.
%
%   out = verify_5state_aprime_selfmod_signflip()
%   out = verify_5state_aprime_selfmod_signflip(opts)  opts.seeds (default 1:60)
%
%   See also: verify_aprime_blackbox_v1_ensemble, motion_control_law_eq17_5state_aprime

    if nargin < 1 || isempty(opts); opts = struct(); end
    if ~isfield(opts, 'seeds');    opts.seeds    = 1:60; end
    if ~isfield(opts, 'T_sim');    opts.T_sim    = 8;    end
    if ~isfield(opts, 't_steady'); opts.t_steady = 1.5;  end
    if ~isfield(opts, 'verbose');  opts.verbose  = true; end

    here = fileparts(mfilename('fullpath'));
    proj = fullfile(here, '..', '..');
    addpath(genpath(proj));

    % Same static near-wall hold as verify_aprime_blackbox_v1_ensemble.m,
    % but eq17_variant='5state_aprime' with use_selfmod=true.
    pc  = physical_constants();
    cfg = user_config();
    cfg.eq17_variant       = '5state_aprime';
    cfg.use_selfmod        = true;
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
        N  = size(s.diag.delta_a_hat, 1);
        t  = (0:N-1)' * Ts;
        win = t >= t_steady;

        % Real controller's own a'_x estimate (slot 5), read directly
        % (diag.delta_a_hat is the driver-logged alias -- run_pure_simulation's
        % diag_log does not carry diag.aprime_hat through).
        ap_seed(si,:) = mean(s.diag.delta_a_hat(win,:), 1);

        % Oracle a'_true(h_bar), same construction as verify_aprime_blackbox_v1_ensemble.m
        w  = P.wall.w_hat(:); pz = P.wall.pz; R = P.common.R;
        p0 = P.common.p0(:).';
        p_true_pre = [p0; s.p_true_out(1:end-1,:)];
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
    out.sign_ok  = sign(out.ap_mean) == sign(out.apt_mean);

    if opts.verbose
        ax = {'x','y','z'};
        fprintf('\n[5-state use_selfmod=true: static near-wall hold h_bar~1.78, %d seeds]\n', ns);
        fprintf('  %-28s %10s %10s %10s\n', 'metric / axis', ax{:});
        fprintf('  %-28s %10.4g %10.4g %10.4g\n', 'mean(diag.delta_a_hat) [1/pN]', out.ap_mean);
        fprintf('  %-28s %10.4g %10.4g %10.4g\n', 'cross-seed SE', out.ap_se);
        fprintf('  %-28s %10.4g %10.4g %10.4g\n', 'oracle a_prime_true [1/pN]', out.apt_mean);
        fprintf('  %-28s %10d %10d %10d\n', 'sign matches oracle?', out.sign_ok);
        fprintf('  %-28s %10.1f %10.1f %10.1f\n', 'rel. bias (%%)', 100*out.rel_bias);
        fprintf('  %-28s %10.2f %10.2f %10.2f\n', 't-stat (bias/SE)', out.t_stat);
        fprintf('  (compare to verify_aprime_blackbox_v1_ensemble.m: t=-3.53,-2.87,-7.65, ALL WRONG SIGN)\n');
    end
end
