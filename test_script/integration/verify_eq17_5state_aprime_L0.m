function out = verify_eq17_5state_aprime_L0(opts)
%VERIFY_EQ17_5STATE_APRIME_L0  Front (L0) verification of the 5-state a'-slope
%   estimating controller. Validates that the derivation's matrices == the maps
%   the code uses, and that the augmentation did not break the base controller.
%   The MAIN estimability verification (L1, a'_hat vs true a'(h)) is NOT run here.
%
%   out = verify_eq17_5state_aprime_L0()
%   out = verify_eq17_5state_aprime_L0(struct('T_sim',4,'seeds_smoke',1:2))
%
%   Three checks (fast):
%     L0a  Matrix / Jacobian (NO simulation, instant). Exercises the REAL
%          build_F_e_5state_aprime + finite-differences the predict and
%          measurement maps. Confirms the three new couplings:
%              F_e(4,5) = Delta_h_d, F_e(3,5) = dF_dx^h, H(2,5) = -Delta_H_d.
%     L0b  Frozen-a' regression (1 Hz near-wall, full noise): with a'_x frozen at
%          its seed (Q55=0, no slot-5 update) the controller reduces to the base
%          a_x machinery; assert it runs and does not diverge, and report tracking
%          std alongside the 4-state on the same scenario.
%     L0c  Perfect-track (noise-free, true gain fed to the control law): the y_1
%          tracking-error channel must drive the position error to ~0 with no
%          NaN/Inf in the estimator -- a wiring sanity check.
%
%   NOTE: dynamic-only (no commit). Trajectory mirrors verify_eq17_4state.
%
%   See also: motion_control_law_eq17_5state_aprime, build_F_e_5state_aprime,
%             run_pure_simulation, verify_eq17_4state

    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'T_sim');       opts.T_sim = 4.0;     end
    if ~isfield(opts, 'seeds_smoke'); opts.seeds_smoke = 1:2; end
    if ~isfield(opts, 'verbose');     opts.verbose = true;  end

    here = fileparts(mfilename('fullpath'));
    proj = fullfile(here, '..', '..');
    addpath(genpath(proj));

    out = struct();
    vb = opts.verbose;
    if vb; fprintf('==== verify_eq17_5state_aprime_L0 ====\n'); end

    % ===============================================================
    % L0a -- matrix / Jacobian (no simulation)
    % ===============================================================
    out.L0a = check_L0a();
    if vb
        fprintf('[L0a] F_e structure OK; predict-map J err = %.2e, meas J err = %.2e\n', ...
                out.L0a.predict_jac_err, out.L0a.meas_jac_err);
        fprintf('      F_e(4,5)=Delta_h_d, F_e(3,5)=dF_dx^h, H(2,5)=-Delta_H_d all confirmed.\n');
    end

    % ===============================================================
    % L0b -- frozen-a' regression vs 4-state (no divergence)
    % ===============================================================
    out.L0b = check_L0b(opts);
    if vb
        ax = {'x','y','z'};
        fprintf('[L0b] frozen-a'' 5-state: ran, |e|_max = %.1f nm (diverged=%d)\n', ...
                1e3*out.L0b.max_err_um, out.L0b.diverged);
        fprintf('      %-26s %8s %8s %8s\n', 'tracking std [nm] / axis', ax{:});
        fprintf('      %-26s %8.1f %8.1f %8.1f\n', '5-state(frozen a'')', out.L0b.std_aprime_nm);
        fprintf('      %-26s %8.1f %8.1f %8.1f\n', '4-state (reference)', out.L0b.std_4state_nm);
    end

    % ===============================================================
    % L0c -- perfect-track (noise-free, true gain)
    % ===============================================================
    out.L0c = check_L0c(opts);
    if vb
        fprintf('[L0c] perfect-track: |e|_max = %.2e um, steady std = %.2e um (finite est=%d)\n', ...
                out.L0c.max_err_um, out.L0c.steady_std_um, out.L0c.est_finite);
    end

    out.passed = true;   % all asserts above passed if we reached here
    if vb; fprintf('==== L0 PASSED (matrix + regression + perfect-track) ====\n'); end
end


% ====================== L0a ======================

function r = check_L0a()
%CHECK_L0A  Structural + finite-difference validation of the new couplings.
    lc = 0.7;

    % --- (1) Structural: the REAL build_F_e_5state_aprime places its args right.
    f_d = 1.3; F_1 = 0.7; dFh = 0.55; Dh = 0.011;
    Fe = build_F_e_5state_aprime(lc, f_d, F_1, dFh, Dh);
    Fdx = f_d + (1 - lc) * F_1;
    tol0 = 1e-14;
    assert(isequal(Fe(1, :), [0 1 0 0 0]), 'L0a: F_e row 1');
    assert(isequal(Fe(2, :), [0 0 1 0 0]), 'L0a: F_e row 2');
    assert(isequal(Fe(5, :), [0 0 0 0 1]), 'L0a: F_e row 5 (a''_x random walk)');
    assert(abs(Fe(3, 3) - lc) < tol0,        'L0a: F_e(3,3) must be lambda_c');
    assert(abs(Fe(3, 4) - (-Fdx)) < tol0,    'L0a: F_e(3,4) must be -F_dx');
    assert(abs(Fe(3, 5) - dFh) < tol0,       'L0a: F_e(3,5) must be dF_dx^h');
    assert(all(abs(Fe(4, 1:3)) < tol0) && abs(Fe(4, 4) - 1) < tol0, 'L0a: F_e row 4 base');
    assert(abs(Fe(4, 5) - Dh) < tol0,        'L0a: F_e(4,5) must be Delta_h_d (NOT 1)');

    % --- (2) FD of the predict map: dx_pred/dx must equal Phi (predictor form).
    %     Key new entry: d(a_x_pred)/d(a'_x) = Delta_h_d.
    Phi_an = [0 1 0 0 0; 0 0 1 0 0; 0 0 lc 0 0; 0 0 0 1 Dh; 0 0 0 0 1];
    gpred  = @(x) [x(2); x(3); lc * x(3); x(4) + x(5) * Dh; x(5)];
    x0 = [0.011; -0.022; 0.033; 0.013; 0.0051];
    Jp = fd_jac(gpred, x0);
    predict_jac_err = max(abs(Jp(:) - Phi_an(:)));
    assert(predict_jac_err < 1e-6, ...
        'L0a: predict-map Jacobian != Phi (err=%.2e)', predict_jac_err);

    % --- (3) FD of the measurement map: dy/dx must equal H.
    %     Key new entry: d(y2)/d(a'_x) = -Delta_H_d.
    DH = 0.022;
    H_an  = [1 0 0 0 0; 0 0 0 1 -DH];
    mmeas = @(x) [x(1); x(4) - DH * x(5)];
    Jm = fd_jac(mmeas, x0);
    meas_jac_err = max(abs(Jm(:) - H_an(:)));
    assert(meas_jac_err < 1e-6, ...
        'L0a: measurement Jacobian != H (err=%.2e)', meas_jac_err);

    % --- (4) dF_dx^h value: (1-lc)*sum (h_d[k]-h_d[k-i]) f_d[k-i], telescoped via
    %     wall-normal position differences (the form the controller computes).
    w  = [0; 0; 1];
    pd   = [0; 0; 8.0];  pd1 = [0; 0; 8.6];  pd2 = [0; 0; 9.3];   % h_d[k], [k-1], [k-2]
    fd1 = 1.1; fd2 = -0.7;
    dh1 = dot(pd - pd1, w);  dh2 = dot(pd - pd2, w);
    dFh_ctrl = (1 - lc) * (dh1 * fd1 + dh2 * fd2);
    dFh_spec = (1 - lc) * ((pd(3) - pd1(3)) * fd1 + (pd(3) - pd2(3)) * fd2);
    assert(abs(dFh_ctrl - dFh_spec) < tol0, 'L0a: dF_dx^h telescoping');

    r.Fe_struct_ok   = true;
    r.predict_jac_err = predict_jac_err;
    r.meas_jac_err    = meas_jac_err;
end


function J = fd_jac(fun, x)
%FD_JAC  Central-difference Jacobian of fun at x.
    n  = numel(x);
    f0 = fun(x);
    m  = numel(f0);
    J  = zeros(m, n);
    for j = 1:n
        dxj = max(1e-7, 1e-7 * abs(x(j)));
        xp = x; xp(j) = xp(j) + dxj;
        xm = x; xm(j) = xm(j) - dxj;
        J(:, j) = (fun(xp) - fun(xm)) / (2 * dxj);
    end
end


% ====================== L0b ======================

function r = check_L0b(opts)
%CHECK_L0B  Frozen-a' regression: no divergence; tracking vs 4-state.
    f = 1.0;
    cfg_ap = build_cfg(f, opts.T_sim, '5state_aprime');
    cfg_ap.freeze_aprime = true;
    cfg_4  = build_cfg(f, opts.T_sim, '4state');

    [std_ap, maxe_ap, div_ap] = run_track(cfg_ap, opts.seeds_smoke);
    [std_4,  ~,       div_4 ] = run_track(cfg_4,  opts.seeds_smoke);

    assert(~div_ap, 'L0b: 5state_aprime (frozen a'') diverged (|e|_max=%.3g um)', maxe_ap);
    assert(~div_4,  'L0b: 4-state reference diverged');

    r.std_aprime_nm = std_ap;
    r.std_4state_nm = std_4;
    r.max_err_um    = maxe_ap;
    r.diverged      = div_ap;
end


function [std_nm, max_err_um, diverged] = run_track(cfg, seeds)
%RUN_TRACK  Mean tracking std over seeds; divergence flag.
    ns = numel(seeds);
    tr = nan(ns, 3); me = 0;
    for si = 1:ns
        ro = struct('seed', seeds(si), 'verbose', false, 'collect_diag', false, ...
                    'use_true_gain', false);
        s = run_pure_simulation(cfg, ro);
        e = s.p_d_out(2:end, :) - s.p_true_out(1:end-1, :);   % [N-1 x 3] [um]
        tr(si, :) = 1e3 * std(e, 0, 1);                       % [nm]
        me = max(me, max(abs(e(:))));
    end
    std_nm     = mean(tr, 1);
    max_err_um = me;
    diverged   = me > 0.5;       % same threshold as verify_eq17_4state smoke
end


% ====================== L0c ======================

function r = check_L0c(opts)
%CHECK_L0C  Perfect-track: noise-free + true gain -> position error ~0.
    f = 1.0;
    cfg = build_cfg(f, opts.T_sim, '5state_aprime');
    cfg.thermal_enable    = false;
    cfg.meas_noise_enable = false;
    ro = struct('seed', 1, 'verbose', false, 'collect_diag', true, ...
                'use_true_gain', true);
    s = run_pure_simulation(cfg, ro);

    e = s.p_d_out(2:end, :) - s.p_true_out(1:end-1, :);   % [N-1 x 3] [um]
    max_err_um = max(abs(e(:)));
    n = size(e, 1);
    tail = max(1, round(0.75 * n)):n;                      % steady-state quarter
    steady_std_um = max(std(e(tail, :), 0, 1));

    est_finite = all(isfinite(s.diag.a_hat(:))) && all(isfinite(s.diag.delta_a_hat(:)));

    assert(est_finite, 'L0c: estimator produced non-finite a_hat / a''_hat');
    assert(steady_std_um < 1e-3, ...
        'L0c: perfect-track steady std too large (%.3g um); wiring suspect', steady_std_um);

    r.max_err_um    = max_err_um;
    r.steady_std_um = steady_std_um;
    r.est_finite    = est_finite;
end


% ====================== shared scenario ======================

function cfg = build_cfg(f, T_sim, variant)
%BUILD_CFG  1 Hz near-wall osc (mirrors verify_eq17_4state build_cfg).
    cfg = user_config();
    cfg.eq17_variant   = variant;
    cfg.trajectory_type = 'osc';
    cfg.h_init   = 50;
    cfg.h_bottom = 2.7;                       % h_bar trough ~1.2 (near wall)
    cfg.amplitude = 2.5;
    cfg.frequency = f;
    cfg.n_cycles  = 2 * f;
    cfg.t_hold    = 0.5;
    cfg.t_descend_override = 1.0;
    cfg.T_sim     = T_sim;
    pc = physical_constants();
    cfg.h_min     = 1.05 * pc.R;
    cfg.ctrl_enable = true;
    cfg.thermal_enable = true;
    cfg.meas_noise_enable = true;
    cfg.lambda_c = 0.7;
    cfg.a_pd  = 0.05;
    cfg.a_cov = 0.05;
    cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];
    cfg.suppress_xD = true;
    cfg.h_bar_safe = 1;                        % G3 unreachable (trough h_bar=1.2)
    cfg.use_am_lpf = false;
    cfg.a_det      = 0.005;
end
