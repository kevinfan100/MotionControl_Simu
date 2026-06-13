function verify_eq17_unit_gain_override_6state()
%VERIFY_EQ17_UNIT_GAIN_OVERRIDE_6STATE Unit tests for the optional
%   a_ctrl_override (6th arg) + ctrl_const.suppress_xD support in
%   motion_control_law_eq17_6state (true-gain A/B experiment, see
%   reference/eq17_analysis/gain_compare_design.md §4.1).
%
%   T1: backward-compat — 5-arg call vs 6-arg call with [] give
%       bit-identical f_d and a_hat over 40 steps.
%   T2: override wiring — first post-init step with constant override
%       a_ovr and pure measurement offset e0 gives
%       f_d = (1-lc)*e0./a_ovr exactly (law uses a_ovr, not a_hat).
%   T3: suppress_xD + shadow reconstruction — 30 steps with override and
%       suppress_xD=true; f_d matches a shadow re-implementation of the
%       law WITHOUT the x_D term at every step.
%   T4: xD timing — suppress_xD=false: f_d matches shadow law using the
%       PREVIOUS step's diag.x_D_hat (posterior[k-1] enters the law).
%   T5: diag.a_ctrl_used — override mode returns a_ovr; normal mode
%       returns previous step's posterior a_hat (one-step lag).

    fprintf('=== verify_eq17_unit_gain_override_6state ===\n');

    this_dir  = fileparts(mfilename('fullpath'));
    repo_root = fullfile(this_dir, '..', '..');
    addpath(fullfile(repo_root, 'model'));
    addpath(fullfile(repo_root, 'model', 'config'));
    addpath(fullfile(repo_root, 'model', 'controller'));
    addpath(fullfile(repo_root, 'model', 'wall_effect'));

    [P, cc] = build_mocks_6state();
    lc  = P.ctrl.lambda_c;
    p0  = P.common.p0;

    % Wall-aware init gain (mirrors controller section 0E)
    a_nom = P.ctrl.Ts / P.ctrl.gamma;
    [cpa0, cpe0] = calc_correction_functions(p0(3) / P.common.R);
    a_init = [a_nom / cpa0; a_nom / cpa0; a_nom / cpe0];

    n_pass = 0;

    % ---------------- T1: backward compat (5-arg vs 6-arg []) ----------
    rng(7);
    M = 40;
    p_m_seq = repmat(p0, 1, M) + 1e-3 * randn(3, M);   % small walk around p0

    clear motion_control_law_eq17_6state;
    f_a = zeros(3, M); a_a = zeros(3, M);
    for k = 1:M
        [f, ~, dg] = motion_control_law_eq17_6state(zeros(3,1), p0, p_m_seq(:,k), P, cc);
        f_a(:,k) = f; a_a(:,k) = dg.a_hat;
    end
    clear motion_control_law_eq17_6state;
    f_b = zeros(3, M); a_b = zeros(3, M);
    for k = 1:M
        [f, ~, dg] = motion_control_law_eq17_6state(zeros(3,1), p0, p_m_seq(:,k), P, cc, []);
        f_b(:,k) = f; a_b(:,k) = dg.a_hat;
    end
    assert(max(abs(f_a(:) - f_b(:))) == 0, 'T1: f_d not bit-identical');
    assert(max(abs(a_a(:) - a_b(:))) == 0, 'T1: a_hat not bit-identical');
    n_pass = n_pass + 1; fprintf('T1 PASS backward-compat bit-identical\n');

    % ---------------- T2: override used in the law ---------------------
    a_ovr = 2 * a_init;
    e0 = [0.01; 0.01; 0.01];                            % [um] measurement offset
    clear motion_control_law_eq17_6state;
    motion_control_law_eq17_6state(zeros(3,1), p0, p0, P, cc, a_ovr);   % init call
    [f2, ~, dg2] = motion_control_law_eq17_6state(zeros(3,1), p0, p0 - e0, P, cc, a_ovr);
    f2_expect = (1 - lc) * e0 ./ a_ovr;   % pd const -> traj terms 0; hist 0; xD 0
    assert(max(abs(f2 - f2_expect)) < 1e-10 * max(1, max(abs(f2_expect))), ...
           'T2: f_d does not match (1-lc)*e0./a_ovr');
    assert(max(abs(dg2.a_ctrl_used - a_ovr)) == 0, 'T2: a_ctrl_used ~= a_ovr');
    n_pass = n_pass + 1; fprintf('T2 PASS override wiring exact\n');

    % ---------------- T3: shadow law, suppress_xD = true ---------------
    cc_sup = cc; cc_sup.suppress_xD = true;
    rng(11);
    M3 = 30;
    p_m_seq3 = repmat(p0, 1, M3) + 2e-3 * randn(3, M3);
    clear motion_control_law_eq17_6state;
    motion_control_law_eq17_6state(zeros(3,1), p0, p0, P, cc_sup, a_ovr);  % init
    f_km1 = zeros(3,1); f_km2 = zeros(3,1);
    for k = 1:M3
        dxm_k = p0 - p_m_seq3(:,k);                     % pd_km2 stays p0 (positioning)
        sum_past = a_ovr .* f_km1 + a_ovr .* f_km2;     % a_ovr constant; f_km1/km2 track actual controller output
        f_exp = (1 ./ a_ovr) .* ((1-lc)*dxm_k - (1-lc)*sum_past);   % NO xD term
        f_act = motion_control_law_eq17_6state(zeros(3,1), p0, p_m_seq3(:,k), P, cc_sup, a_ovr);
        assert(max(abs(f_act - f_exp)) < 1e-10 * max(1, max(abs(f_exp))), ...
               sprintf('T3: shadow mismatch at k=%d', k));
        f_km2 = f_km1; f_km1 = f_act;
    end
    n_pass = n_pass + 1; fprintf('T3 PASS suppress_xD shadow law (30 steps)\n');

    % ---------------- T4: xD timing (suppress off, prior enters law) ---
    rng(13);
    M4 = 30;
    p_m_seq4 = repmat(p0, 1, M4) + 2e-3 * randn(3, M4);
    clear motion_control_law_eq17_6state;
    [~, ~, dg] = motion_control_law_eq17_6state(zeros(3,1), p0, p0, P, cc, a_ovr); % init
    assert(max(abs(dg.x_D_hat)) == 0, ...
           'T4 setup: x_D_hat after init must be zero (check empty_diag_6state)');
    xD_prev = dg.x_D_hat;                               % posterior after init = 0
    f_km1 = zeros(3,1); f_km2 = zeros(3,1);
    for k = 1:M4
        dxm_k = p0 - p_m_seq4(:,k);
        sum_past = a_ovr .* f_km1 + a_ovr .* f_km2;
        f_exp = (1 ./ a_ovr) .* ((1-lc)*dxm_k - (1-lc)*sum_past - xD_prev);
        [f_act, ~, dg] = motion_control_law_eq17_6state(zeros(3,1), p0, p_m_seq4(:,k), P, cc, a_ovr);
        assert(max(abs(f_act - f_exp)) < 1e-10 * max(1, max(abs(f_exp))), ...
               sprintf('T4: xD-timing shadow mismatch at k=%d', k));
        xD_prev = dg.x_D_hat;                           % this step's posterior -> next law
        f_km2 = f_km1; f_km1 = f_act;
    end
    n_pass = n_pass + 1; fprintf('T4 PASS xD prior-timing shadow law (30 steps)\n');

    % ---------------- T5: a_ctrl_used in normal (EKF) mode -------------
    rng(17);
    M5 = 20;
    p_m_seq5 = repmat(p0, 1, M5) + 2e-3 * randn(3, M5);
    clear motion_control_law_eq17_6state;
    [~, ~, dg] = motion_control_law_eq17_6state(zeros(3,1), p0, p0, P, cc);  % init
    a_hat_prev = dg.a_hat;                              % = a_init
    for k = 1:M5
        [~, ~, dg] = motion_control_law_eq17_6state(zeros(3,1), p0, p_m_seq5(:,k), P, cc);
        assert(max(abs(dg.a_ctrl_used - a_hat_prev)) == 0, ...
               sprintf('T5: a_ctrl_used ~= posterior[k-1] at k=%d', k));
        a_hat_prev = dg.a_hat;
    end
    n_pass = n_pass + 1; fprintf('T5 PASS a_ctrl_used = posterior[k-1] (normal mode)\n');

    fprintf('=== ALL %d/5 PASS ===\n', n_pass);
end


function [P, cc] = build_mocks_6state()
%BUILD_MOCKS_6STATE Minimal params + ctrl_const (mirrors 3guard test mocks).
    R = 2.25; gamma_N = 0.0425; Ts = 1/1600;
    k_B = 1.3806503e-5; T_K = 310.15; kBT = k_B * T_K;
    sigma2_n_s = [0.00062; 0.00057; 0.00331].^2;

    P.ctrl.enable = 1;       P.ctrl.lambda_c = 0.7;
    P.ctrl.gamma = gamma_N;  P.ctrl.Ts = Ts;
    P.ctrl.k_B = k_B;        P.ctrl.T = T_K;
    P.ctrl.sigma2_noise = sigma2_n_s;
    P.common.R = R; P.common.Ts = Ts; P.common.gamma_N = gamma_N;
    P.common.p0 = [0; 0; 50];
    P.wall.w_hat = [0; 0; 1]; P.wall.pz = 0; P.wall.enable_wall_effect = 1;
    P.thermal = struct('enable', 0);

    opts.lambda_c = 0.7;  opts.sigma2_n_s = sigma2_n_s;  opts.kBT = kBT;
    opts.a_cov = 0.05;    opts.a_pd = 0.05;              opts.d = 2;
    opts.t_warmup_kf = 0; opts.h_bar_safe = 1.5;
    cc = build_eq17_6state_constants(opts);
end
