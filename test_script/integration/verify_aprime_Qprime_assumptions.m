function out = verify_aprime_Qprime_assumptions(opts)
%VERIFY_APRIME_QPRIME_ASSUMPTIONS  Pure-physics sanity check of the a'/a''
%   magnitude assumptions behind the black-box process-noise design
%   Q_a'[k] = sigma2_aprime2 * (Delta_h_d[k]^2 + sigma2_dh[k]),
%   sigma2_aprime2 ~ (a_nom/R^2)^2 (dimensional anchor -- see
%   reference/eq17_analysis/derivation/aprime_blackbox_regression.tex
%   discussion). Uses ONLY the oracle wall model (calc_correction_functions)
%   -- decoupled from the EKF/controller and from the still-open
%   closed-loop-contamination investigation in
%   verify_aprime_blackbox_v1_ensemble.
%
%   Test 1: sweep h_bar across the project's usual near-wall-to-far-field
%   range; report a_true, a'_true, a''_true and their ratios to the
%   dimensional anchors a_nom/R, a_nom/R^2 -- quantifies how well a SINGLE
%   constant covers the whole range (expect: under-covers near wall,
%   over-covers far field -- already flagged as an architectural limit in
%   prior project work, here quantified with real numbers).
%
%   Test 2: reuse an existing near-wall osc trajectory as a REALISTIC
%   h_bar(t) path (only p_true_out is used -- controller/EKF correctness is
%   irrelevant here), compute the TRUE a'(h_bar[k]) time series, its
%   empirical Var(Delta a'_true) in near-wall vs far-field windows, and
%   compare against the predicted Q_a' formula with the dimensional-anchor
%   sigma.
%
%   out = verify_aprime_Qprime_assumptions()
%
%   See also: reference/eq17_analysis/derivation/aprime_blackbox_regression.tex

    if nargin < 1 || isempty(opts); opts = struct(); end
    if ~isfield(opts, 'verbose'); opts.verbose = true; end

    here = fileparts(mfilename('fullpath'));
    proj = fullfile(here, '..', '..');
    addpath(genpath(proj));

    pc = physical_constants();
    R  = pc.R;
    a_nom = pc.Ts / pc.gamma_N;

    sigma_aprime_dim  = a_nom / R;      % dimensional anchor for |a'|   [1/pN]
    sigma_a2prime_dim = a_nom / R^2;    % dimensional anchor for |a''|  [1/pN/um]

    % ================= Test 1: magnitude sweep across h_bar =================
    h_bar_list = [1.05, 1.1, 1.2, 1.5, 1.78, 2, 3, 5, 10, 22];
    n = numel(h_bar_list);
    a_para = nan(n,1); a_perp = nan(n,1);
    ap_para = nan(n,1); ap_perp = nan(n,1);
    app_para = nan(n,1); app_perp = nan(n,1);

    for i = 1:n
        hb = h_bar_list(i);
        [c_para, c_perp, derivs] = calc_correction_functions(hb, true);
        a_para(i) = a_nom / c_para;
        a_perp(i) = a_nom / c_perp;
        ap_para(i) = -a_para(i) * derivs.K_h_para / R;
        ap_perp(i) = -a_perp(i) * derivs.K_h_perp / R;
        app_para(i) = a_para(i) * (derivs.K_h_para^2 - derivs.K_h_prime_para) / R^2;
        app_perp(i) = a_perp(i) * (derivs.K_h_perp^2 - derivs.K_h_prime_perp) / R^2;
    end

    ratio_ap_para  = ap_para  / sigma_aprime_dim;
    ratio_ap_perp  = ap_perp  / sigma_aprime_dim;
    ratio_app_para = app_para / sigma_a2prime_dim;
    ratio_app_perp = app_perp / sigma_a2prime_dim;

    out.h_bar_list = h_bar_list;
    out.a_para = a_para; out.a_perp = a_perp;
    out.ap_para = ap_para; out.ap_perp = ap_perp;
    out.app_para = app_para; out.app_perp = app_perp;
    out.sigma_aprime_dim  = sigma_aprime_dim;
    out.sigma_a2prime_dim = sigma_a2prime_dim;

    if opts.verbose
        fprintf('\n[Test 1: magnitude sweep -- dimensional anchors a_nom/R=%.4g 1/pN, a_nom/R^2=%.4g 1/pN/um]\n', sigma_aprime_dim, sigma_a2prime_dim);
        fprintf('  %8s | %10s %10s | %14s %14s | %16s %16s\n', 'h_bar', 'a_para', 'a_perp', 'ap_para/anchor', 'ap_perp/anchor', 'app_para/anchor', 'app_perp/anchor');
        for i = 1:n
            fprintf('  %8.2f | %10.4g %10.4g | %14.2f %14.2f | %16.2f %16.2f\n', ...
                h_bar_list(i), a_para(i), a_perp(i), ratio_ap_para(i), ratio_ap_perp(i), ratio_app_para(i), ratio_app_perp(i));
        end
        fprintf('  (ratio ~ O(1) means the dimensional anchor is a reasonable single-constant magnitude; large spread across rows = the known under/over-cover trade-off.)\n');
    end

    % ================= Test 2: Var(Delta a') along a real trajectory =================
    cfg = user_config();
    cfg.eq17_variant       = '4state';
    cfg.enable_wall_effect = true;
    cfg.trajectory_type    = 'osc';
    cfg.h_init             = 50;
    cfg.h_bottom           = 2.7;
    cfg.amplitude          = 2.5;
    cfg.frequency          = 1.0;
    cfg.n_cycles           = 4;
    cfg.t_hold             = 0.5;
    cfg.t_descend_override = 1.0;
    cfg.h_min              = 1.05 * R;
    cfg.h_bar_safe         = 1;
    cfg.ctrl_enable        = true;
    cfg.thermal_enable     = true;
    cfg.meas_noise_enable  = true;
    cfg.meas_noise_std     = [0.00062; 0.00057; 0.00331];
    cfg.lambda_c           = 0.7;
    cfg.a_pd               = 0.05;
    cfg.a_cov              = 0.05;
    cfg.suppress_xD        = true;
    cfg.use_am_lpf         = false;
    cfg.a_det              = 0.005;
    cfg.T_sim              = 5.5;    % 0.5 hold + 1.0 descend + 4 cycles @ 1Hz

    ro = struct('seed', 1, 'verbose', false, 'collect_diag', false);
    s  = run_pure_simulation(cfg, ro);
    P  = s.meta.params_value;
    w  = P.wall.w_hat(:); pz = P.wall.pz;
    N  = size(s.p_true_out,1);

    % pre-integration alignment (same established fix as verify_aprime_blackbox_v1_ensemble)
    p0 = P.common.p0(:).';
    p_true_pre = [p0; s.p_true_out(1:end-1,:)];
    h_bar_true = max((p_true_pre * w - pz) / R, 1.001);

    % TRUE a'(h_bar[k]) time series (z-axis = perp)
    a_prime_true_z = zeros(N,1);
    for k = 1:N
        [~, c_perp, derivs] = calc_correction_functions(h_bar_true(k), true);
        a_prime_true_z(k) = -(a_nom/c_perp) * derivs.K_h_perp / R;
    end

    dApr    = diff(a_prime_true_z);         % Delta a'_true[k]
    dh_true = diff(h_bar_true) * R;         % actual physical height step [um]

    win_near = h_bar_true(1:end-1) < 1.5;
    win_far  = h_bar_true(1:end-1) > 5;

    Var_dApr_near_emp = var(dApr(win_near));
    Var_dApr_far_emp  = var(dApr(win_far));

    sigma2_a2prime     = sigma_a2prime_dim^2;
    Var_dApr_near_pred = sigma2_a2prime * mean(dh_true(win_near).^2);
    Var_dApr_far_pred  = sigma2_a2prime * mean(dh_true(win_far).^2);

    out.Var_dApr_near_emp  = Var_dApr_near_emp;
    out.Var_dApr_far_emp   = Var_dApr_far_emp;
    out.Var_dApr_near_pred = Var_dApr_near_pred;
    out.Var_dApr_far_pred  = Var_dApr_far_pred;

    if opts.verbose
        fprintf('\n[Test 2: Var(Delta a''_true) along a real osc trajectory, single dimensional-anchor sigma2_a'''']\n');
        fprintf('  %-12s %16s %16s %10s\n', 'window', 'empirical Var', 'predicted Var', 'ratio');
        fprintf('  %-12s %16.6g %16.6g %10.2f\n', 'near-wall (h<1.5)', Var_dApr_near_emp, Var_dApr_near_pred, Var_dApr_near_emp/Var_dApr_near_pred);
        fprintf('  %-12s %16.6g %16.6g %10.2f\n', 'far-field (h>5)',   Var_dApr_far_emp,  Var_dApr_far_pred,  Var_dApr_far_emp/Var_dApr_far_pred);
        fprintf('  (ratio far from 1 in opposite directions = the single-constant under/over-cover trade-off, now quantified with real numbers.)\n');
    end
end
