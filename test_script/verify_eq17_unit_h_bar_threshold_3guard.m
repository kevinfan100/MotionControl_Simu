function test_h_bar_threshold_3guard()
%TEST_H_BAR_THRESHOLD_3GUARD Verify 3-guard adaptive R_2 logic
%
%   Priority 6 from deep audit §6.
%
%   Three guards in motion_control_law_eq17_7state.m §[4]:
%     G1: t_now < t_warmup_kf           (warm-up)
%     G2: sigma2_dxr_hat - C_n*sigma2_n_s <= 0   (low SNR)
%     G3: h_bar < h_bar_safe            (near-wall)
%
%   Any guard triggered -> R(2,2) = R_OFF = 1e10 (engineering override).
%   Otherwise R(2,2) = a_cov*IF_var*(a_hat+xi)^2 + delay_R2_factor*Q77.
%
%   Tests use the controller's optional `diag` output (3rd output) which
%   exposes `gate_active_per_axis` (3x1 logical) and `guards_individual`
%   (3x3 logical: rows G1/G2/G3, cols axes).
%
%   T1: G1 boundary — at t=0.1s (< 0.2s warmup), gate active per axis
%   T2: G1 release  — past warmup (~t=0.5s), G1 false
%   T3: G2 trigger  — when sigma2_dxr_hat is small (no signal), G2 active
%   T4: G3 boundary — h_bar=1.4 (< 1.5) gates all axes; h_bar=1.6 releases
%   T5: G3 boundary value — at h_bar=1.5 exactly, G3 should NOT trigger
%       (strict <, not <=)

    fprintf('=== test_h_bar_threshold_3guard ===\n');

    this_dir = fileparts(mfilename('fullpath'));
    repo_root = fullfile(this_dir, '..', '..');
    addpath(fullfile(repo_root, 'model'));
    addpath(fullfile(repo_root, 'model', 'config'));
    addpath(fullfile(repo_root, 'model', 'controller'));
    addpath(fullfile(repo_root, 'model', 'wall_effect'));

    n_pass = 0;

    [params_mock, ctrl_const_mock] = build_mocks();
    Ts = params_mock.ctrl.Ts;
    t_warmup_kf = ctrl_const_mock.t_warmup_kf;
    h_bar_safe  = ctrl_const_mock.h_bar_safe;
    R_radius    = params_mock.common.R;

    p0 = [0; 0; 50];                      % h_bar = 50/2.25 ~ 22 (far field)
    pd = p0;
    del_pd = zeros(3, 1);

    % ------------------------------------------------------------------
    % T1: G1 active during warmup (t < 0.2s)
    % We need to first run past the 2-step warmup_count to get into the
    % EKF path. Then check G1 still active until k_step*Ts >= 0.2s.
    % ------------------------------------------------------------------
    clear motion_control_law_eq17_7state;

    % First two calls = warmup_count steps (NO EKF run, no diag info).
    [~, ~, ~]  = motion_control_law_eq17_7state(del_pd, pd, p0, params_mock, ctrl_const_mock);
    [~, ~, ~]  = motion_control_law_eq17_7state(del_pd, pd, p0, params_mock, ctrl_const_mock);
    [~, ~, ~]  = motion_control_law_eq17_7state(del_pd, pd, p0, params_mock, ctrl_const_mock);

    % Now run more calls until we're at ~t=0.1s (still in G1 t_warmup_kf range)
    % t_warmup_kf is 0.2s, so 0.1s is mid-warmup.
    target_steps_g1 = round(0.1 / Ts);
    last_diag = struct();
    for k = 1:target_steps_g1
        [~, ~, last_diag] = motion_control_law_eq17_7state( ...
            del_pd, pd, p0, params_mock, ctrl_const_mock);
    end

    % All three axes should have G1 = true (warmup not yet finished)
    G1_axis = last_diag.guards_individual(1, :);
    assert(all(G1_axis), ...
        'T1 FAIL: G1 should be active during warmup; got %s', mat2str(G1_axis));
    assert(all(last_diag.gate_active_per_axis), ...
        'T1 FAIL: gate_active should be true during warmup');

    fprintf('[PASS] T1: G1 active during warmup (t=%.3fs, t_warmup_kf=%.2fs)\n', ...
        target_steps_g1*Ts, t_warmup_kf);
    n_pass = n_pass + 1;

    % ------------------------------------------------------------------
    % T2: G1 released past warmup (~t=0.5s)
    %     But G3 may still trigger since at h=50 µm, h_bar=22 > 1.5,
    %     so G3 inactive. G2 depends on sigma2_dxr_hat — at constant p_m
    %     (perfect tracking), sigma2_dxr_hat -> very small near 0 -> G2 may
    %     be active depending on sign of (sigma2_dxr_hat - C_n*sigma2_n_s).
    %     Since p_m=p0 always, dxr=0, sigma2_dxr_hat=0 < C_n*sigma2_n_s,
    %     G2 IS triggered.
    %
    %     So we test that G1 is False past warmup, regardless of other
    %     guards.
    % ------------------------------------------------------------------
    target_steps_g1_off = round(0.5 / Ts);
    additional = target_steps_g1_off - target_steps_g1;
    for k = 1:additional
        [~, ~, last_diag] = motion_control_law_eq17_7state( ...
            del_pd, pd, p0, params_mock, ctrl_const_mock);
    end
    G1_axis_off = last_diag.guards_individual(1, :);
    assert(all(~G1_axis_off), ...
        'T2 FAIL: G1 should be false past warmup; got %s', mat2str(G1_axis_off));

    fprintf('[PASS] T2: G1 released past warmup (t=%.3fs > %.2fs)\n', ...
        target_steps_g1_off*Ts, t_warmup_kf);
    n_pass = n_pass + 1;

    % ------------------------------------------------------------------
    % T3: G2 active when sigma2_dxr_hat <= C_n*sigma2_n_s
    %     With perfect tracking p_m=pd, the IIR variance estimate stays
    %     near 0 — so G2 should be triggered. We continue running past
    %     warmup. We confirmed in T2 that G2 must be triggering (since the
    %     gate is still active).
    % ------------------------------------------------------------------
    G2_axis_off = last_diag.guards_individual(2, :);
    assert(all(G2_axis_off), ...
        'T3 FAIL: G2 should be active for perfect-tracking (sigma2_dxr=0); got %s', ...
        mat2str(G2_axis_off));

    % Show per-axis sigma2_dxr_hat is below threshold
    threshold_per_axis = ctrl_const_mock.C_n * params_mock.ctrl.sigma2_noise;
    sigma2_dxr_below = last_diag.sigma2_dxr_hat <= threshold_per_axis;
    assert(all(sigma2_dxr_below), ...
        'T3 FAIL: sigma2_dxr_hat=%s, threshold=%s', ...
        mat2str(last_diag.sigma2_dxr_hat, 4), mat2str(threshold_per_axis, 4));

    fprintf(['[PASS] T3: G2 active for perfect-tracking (sigma2_dxr=%.3e ', ...
             'all axes <= threshold %s)\n'], last_diag.sigma2_dxr_hat(1), ...
        mat2str(threshold_per_axis(:)', 3));
    n_pass = n_pass + 1;

    % ------------------------------------------------------------------
    % T4: G3 — near wall (h_bar < h_bar_safe = 1.5)
    %     Move particle to h_bar = 1.4 (h = 1.4 * 2.25 = 3.15 um).
    %     But constraint: calc_correction_functions errors if h_bar < 1.
    %     So 1.4 is safe.
    % ------------------------------------------------------------------
    clear motion_control_law_eq17_7state;
    h_near = 1.4 * R_radius;            % 3.15 um
    p_near = [0; 0; h_near];
    pd_near = p_near;

    % Walk past warmup
    [~, ~, ~]  = motion_control_law_eq17_7state(del_pd, pd_near, p_near, params_mock, ctrl_const_mock);
    [~, ~, ~]  = motion_control_law_eq17_7state(del_pd, pd_near, p_near, params_mock, ctrl_const_mock);
    [~, ~, ~]  = motion_control_law_eq17_7state(del_pd, pd_near, p_near, params_mock, ctrl_const_mock);

    target_steps = round(0.5 / Ts);
    for k = 1:target_steps
        [~, ~, last_diag] = motion_control_law_eq17_7state( ...
            del_pd, pd_near, p_near, params_mock, ctrl_const_mock);
    end
    G3_near = last_diag.guards_individual(3, :);
    assert(all(G3_near), ...
        'T4 FAIL: G3 should be active at h_bar=1.4 (< %.1f); got %s', ...
        h_bar_safe, mat2str(G3_near));

    fprintf('[PASS] T4 (near): G3 active at h_bar=%.2f (< %.2f)\n', ...
        last_diag.h_bar, h_bar_safe);

    % ---- Now test h_bar = 1.6 (above threshold) ----
    clear motion_control_law_eq17_7state;
    h_far = 1.6 * R_radius;             % 3.6 um
    p_far = [0; 0; h_far];
    pd_far = p_far;

    [~, ~, ~]  = motion_control_law_eq17_7state(del_pd, pd_far, p_far, params_mock, ctrl_const_mock);
    [~, ~, ~]  = motion_control_law_eq17_7state(del_pd, pd_far, p_far, params_mock, ctrl_const_mock);
    [~, ~, ~]  = motion_control_law_eq17_7state(del_pd, pd_far, p_far, params_mock, ctrl_const_mock);

    for k = 1:target_steps
        [~, ~, last_diag] = motion_control_law_eq17_7state( ...
            del_pd, pd_far, p_far, params_mock, ctrl_const_mock);
    end
    G3_far = last_diag.guards_individual(3, :);
    assert(all(~G3_far), ...
        'T4 FAIL: G3 should be inactive at h_bar=1.6 (> %.1f); got %s', ...
        h_bar_safe, mat2str(G3_far));

    fprintf('[PASS] T4 (far):  G3 inactive at h_bar=%.2f (> %.2f)\n', ...
        last_diag.h_bar, h_bar_safe);
    n_pass = n_pass + 1;

    % ------------------------------------------------------------------
    % T5: G3 strictness — at h_bar = h_bar_safe exactly, NOT triggered
    %     The check is "h_bar < h_bar_safe" (strict <).
    % ------------------------------------------------------------------
    clear motion_control_law_eq17_7state;
    h_exact = h_bar_safe * R_radius;    % h_bar = 1.5 exactly
    p_exact = [0; 0; h_exact];
    pd_exact = p_exact;

    [~, ~, ~]  = motion_control_law_eq17_7state(del_pd, pd_exact, p_exact, params_mock, ctrl_const_mock);
    [~, ~, ~]  = motion_control_law_eq17_7state(del_pd, pd_exact, p_exact, params_mock, ctrl_const_mock);
    [~, ~, ~]  = motion_control_law_eq17_7state(del_pd, pd_exact, p_exact, params_mock, ctrl_const_mock);

    for k = 1:target_steps
        [~, ~, last_diag] = motion_control_law_eq17_7state( ...
            del_pd, pd_exact, p_exact, params_mock, ctrl_const_mock);
    end

    % h_bar should be ~ 1.5 (allowing tiny numerical jitter)
    actual_hbar = last_diag.h_bar;
    G3_exact = last_diag.guards_individual(3, :);
    assert(abs(actual_hbar - h_bar_safe) < 1e-9, ...
        'T5 setup err: h_bar=%.6f, expected %.6f', actual_hbar, h_bar_safe);
    assert(all(~G3_exact), ...
        'T5 FAIL: G3 should be inactive at h_bar=h_bar_safe exactly (strict <)');

    fprintf('[PASS] T5: G3 NOT triggered at h_bar=%.4f (=h_bar_safe, strict <)\n', actual_hbar);
    n_pass = n_pass + 1;

    fprintf('=== ALL %d tests PASS ===\n', n_pass);
end


%% =================== Helpers ===================

function [params_mock, ctrl_const_mock] = build_mocks()
%BUILD_MOCKS Construct minimal struct mocks (mirrors test_motion_control_law_eq17_7state.m)

    R = 2.25;
    gamma_N = 0.0425;
    Ts = 1/1600;
    k_B = 1.3806503e-5;
    T_K = 310.15;
    kBT = k_B * T_K;

    sigma_n_s_um = [0.00062; 0.000057; 0.00331];
    sigma2_n_s = sigma_n_s_um.^2;

    params_mock.ctrl.enable = 1;
    params_mock.ctrl.lambda_c = 0.7;
    params_mock.ctrl.gamma = gamma_N;
    params_mock.ctrl.Ts = Ts;
    params_mock.ctrl.k_B = k_B;
    params_mock.ctrl.T = T_K;
    params_mock.ctrl.sigma2_noise = sigma2_n_s;
    params_mock.ctrl.Pf_init_diag = ...
        [0; 0; 1e-4; 1e-4; 0; 10*(0.0147)^2; 0];
    params_mock.ctrl.controller_type = 17;

    params_mock.common.R = R;
    params_mock.common.Ts = Ts;
    params_mock.common.gamma_N = gamma_N;
    params_mock.common.p0 = [0; 0; 50];

    params_mock.wall.w_hat = [0; 0; 1];
    params_mock.wall.pz = 0;
    params_mock.wall.enable_wall_effect = 1;

    params_mock.traj.amplitude = 10;
    params_mock.traj.frequency = 1;

    opts.lambda_c    = params_mock.ctrl.lambda_c;
    opts.option      = 'A_MA2_full';
    opts.sigma2_n_s  = sigma2_n_s;
    opts.kBT         = kBT;
    opts.t_warmup_kf = 0.2;
    opts.h_bar_safe  = 1.5;
    opts.d           = 2;
    opts.a_cov       = 0.05;
    opts.a_pd        = 0.05;
    opts.sigma2_w_fD = 0;
    ctrl_const_mock = build_eq17_constants(opts);
end
