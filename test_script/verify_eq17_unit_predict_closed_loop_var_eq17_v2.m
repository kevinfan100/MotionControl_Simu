function test_predict_closed_loop_var_eq17_v2()
%TEST_PREDICT_CLOSED_LOOP_VAR_EQ17_V2  Unit tests for v2 closed-form variance prediction
%
%   Tests the modular block-triangular Lyapunov implementation per Phase 7 §6:
%     T1: positioning h=50 baseline, paper-Eq.22-only cross-check vs Phase 2 closed form
%     T2: sigma2_w_fD = 0 baseline, sigma2_e_xD ≈ 0 confirm
%     T3: lambda_c boundary handling (lambda_c=0; lambda_c near 1; reject lambda_c>=1)
%     T4: input validation (missing fields, bad dimensions, bad scenario)
%
%   Notes:
%     * Test 1 verifies that paper Eq.22 dominates positioning baseline
%       (corrections < 0.5%, per Phase 7 §5.4 expectation of < 0.1%).
%     * Test 1 also confirms s2_dx_paper_only matches a Phase 2 hand
%       calculation: (C_dpmr * 4kBT*a_x + C_n * sigma2_n_s) / (1 - lambda_c^2).
%     * No Simulink runtime is invoked; this is a pure-MATLAB unit test.

% Make helper available on path (test sits in test_script/unit_tests/)
this_dir   = fileparts(mfilename('fullpath'));
helper_dir = fullfile(this_dir, '..', '..', 'model', 'diag');
addpath(helper_dir);

fprintf('=== test_predict_closed_loop_var_eq17_v2 ===\n');

% -----------------------------------------------------------------------
% Build a baseline opts struct (positioning h=50)
% -----------------------------------------------------------------------
% Physical constants per project (model/config/physical_constants.m)
k_B    = 1.3806503e-5;        % [pN*um/K]
T_K    = 310.15;              % [K] (37 C)

lambda_c = 0.7;
C_dpmr   = 2 + 1/(1 - lambda_c^2);     % Phase 2 closed form  (~3.961)
C_n      = 2/(1 + lambda_c);           % Phase 2 closed form  (~1.176)
IF_var   = 4.224;                      % Phase 2 IF_var Option A

% Positioning h=50 mock values (representative; actual numbers come from
% calc_simulation_params in Simulink runtime).
% Eq.17 a_x [um/pN] = Ts/gamma_N / c (per axis) — see motion_control_law_eq17_7state §174.
% gamma_N = 0.0425 pN*sec/um, Ts = 1/1600 sec  -> a_nom ~ 1.47e-2 um/pN
gamma_N   = 0.0425;
Ts_const  = 1/1600;
a_nom     = Ts_const / gamma_N;        % ~1.47e-2 um/pN
% At h=50 (h_bar=22.2): c_para ~ 1.0, c_perp ~ 1.5 (rough)
a_par     = a_nom / 1.0;
a_perp50  = a_nom / 1.5;
a_x_axis  = [a_par, a_par, a_perp50];

sigma2_n_s = [0.00062^2, 0.000057^2, 0.00331^2];   % per-axis sensor noise variance [um^2]

% Positioning Q77 = 0 (frozen a, ḣ=ḧ=0 per Phase 8 settings audit §157)
Q77_axis  = [0, 0, 0];
% R(2,2) mock per-axis. Phase 8 §196: chi_sq*rho_a*a_x^2 at h=50.
% chi_sq=2*0.05/(2-0.05)=0.0513, rho_a=4.7. So R(2,2)=0.0513*4.7*a_x^2 ~ 0.241*a_x^2.
R_axis    = 0.241 * a_x_axis.^2;

a_cov = 0.05;
a_pd  = 0.05;
sigma2_w_fD = 0;

opts_base = struct( ...
    'lambda_c',       lambda_c, ...
    'a_x_axis',       a_x_axis, ...
    'h_bar',          50/2.25, ...
    'sigma2_n_s',     sigma2_n_s, ...
    'k_B',            k_B, ...
    'T',              T_K, ...
    'Q77_axis',       Q77_axis, ...
    'R_axis',         R_axis, ...
    'a_cov',          a_cov, ...
    'a_pd',           a_pd, ...
    'C_dpmr',         C_dpmr, ...
    'C_n',            C_n, ...
    'IF_var',         IF_var, ...
    'sigma2_w_fD',    sigma2_w_fD, ...
    'scenario',       'positioning' ...
);

%% --- T1: positioning baseline, paper Eq.22 cross-check --------------------
[s2_dx, s2_e_xD, ~, diag_out] = predict_closed_loop_var_eq17_v2(opts_base);

% Hand-compute paper part for cross-check:
sigma2_dXT_expected = 4 * k_B * T_K * a_x_axis;
s2_dx_paper_expected = (C_dpmr * sigma2_dXT_expected + C_n * sigma2_n_s) / (1 - lambda_c^2);

paper_rel_err = abs(diag_out.s2_dx_paper_only - s2_dx_paper_expected) ./ s2_dx_paper_expected;
assert(all(paper_rel_err < 1e-12), ...
    'T1 FAIL: paper-only mismatch, rel_err = %s', mat2str(paper_rel_err, 3));

% Verify corrections are < 0.5% of paper value (Phase 7 §5.4: expected < 0.1%)
total_correction = s2_dx - diag_out.s2_dx_paper_only;
correction_ratio = total_correction ./ diag_out.s2_dx_paper_only;
assert(all(correction_ratio < 5e-3), ...
    'T1 FAIL: positioning corrections > 0.5%%: ratio=%s', mat2str(correction_ratio, 3));

fprintf('[PASS] T1: paper-Eq.22 part matches hand calc within 1e-12; corrections %.3g%% (< 0.5%%)\n', ...
    100 * max(correction_ratio));
fprintf('         s2_dx_paper_only = [%.4e, %.4e, %.4e] um^2\n', ...
    diag_out.s2_dx_paper_only(1), diag_out.s2_dx_paper_only(2), diag_out.s2_dx_paper_only(3));
fprintf('         sigma_dx (paper)  = [%.3f, %.3f, %.3f] nm\n', ...
    1e3*sqrt(diag_out.s2_dx_paper_only(1)), ...
    1e3*sqrt(diag_out.s2_dx_paper_only(2)), ...
    1e3*sqrt(diag_out.s2_dx_paper_only(3)));

%% --- T2: sigma^2_w_fD = 0 -> sigma^2_e_xD ≈ 0 ------------------------------
% Already covered by opts_base (sigma2_w_fD = 0). Confirm s2_e_xD all zero.
assert(all(s2_e_xD == 0), ...
    'T2 FAIL: sigma2_w_fD=0 should yield s2_e_xD = 0; got %s', mat2str(s2_e_xD, 4));

% Now test sigma2_w_fD > 0 path: should give s2_e_xD ~ sqrt(a_x^2 * sigma2_w_fD * R)
opts_wfD = opts_base;
opts_wfD.sigma2_w_fD = 1e-6;          % small but nonzero
[~, s2_e_xD_pos, ~, ~] = predict_closed_loop_var_eq17_v2(opts_wfD);
expected_xD = sqrt(a_x_axis.^2 * opts_wfD.sigma2_w_fD .* R_axis);
xD_rel_err = abs(s2_e_xD_pos - expected_xD) ./ max(expected_xD, eps);
assert(all(xD_rel_err < 1e-12), ...
    'T2 FAIL: sigma2_w_fD>0 SISO approx mismatch; rel_err=%s', mat2str(xD_rel_err, 3));

fprintf('[PASS] T2: sigma2_w_fD=0 -> s2_e_xD = 0 exactly; sigma2_w_fD=1e-6 matches sqrt(Q55*R)\n');
fprintf('         (sigma2_w_fD=0)   s2_e_xD = [%g, %g, %g]\n', s2_e_xD(1), s2_e_xD(2), s2_e_xD(3));
fprintf('         (sigma2_w_fD=1e-6) s2_e_xD = [%.3e, %.3e, %.3e]\n', ...
    s2_e_xD_pos(1), s2_e_xD_pos(2), s2_e_xD_pos(3));

%% --- T3: lambda_c boundary tests ----------------------------------------
% T3a: lambda_c = 0 should be valid (no AR-1 amplification)
opts_lc0 = opts_base;
opts_lc0.lambda_c = 0;
opts_lc0.C_dpmr   = 2 + 1/(1 - 0^2);    % = 3
opts_lc0.C_n      = 2/(1 + 0);           % = 2
[s2_dx_lc0, ~, ~, ~] = predict_closed_loop_var_eq17_v2(opts_lc0);
assert(all(isfinite(s2_dx_lc0)) && all(s2_dx_lc0 > 0), ...
    'T3a FAIL: lambda_c=0 should give finite positive s2_dx; got %s', mat2str(s2_dx_lc0, 3));

% T3b: lambda_c = 0.95 still in valid range
opts_lcHigh = opts_base;
opts_lcHigh.lambda_c = 0.95;
opts_lcHigh.C_dpmr   = 2 + 1/(1 - 0.95^2);
opts_lcHigh.C_n      = 2/(1 + 0.95);
[s2_dx_lcH, ~, ~, ~] = predict_closed_loop_var_eq17_v2(opts_lcHigh);
assert(all(isfinite(s2_dx_lcH)), ...
    'T3b FAIL: lambda_c=0.95 should give finite s2_dx');

% T3c: lambda_c = 1 should error (boundary)
opts_lc1 = opts_base;
opts_lc1.lambda_c = 1;
try
    predict_closed_loop_var_eq17_v2(opts_lc1);
    error('T3c FAIL: lambda_c=1 should have thrown error');
catch ME
    assert(strcmp(ME.identifier, 'predict_closed_loop_var_eq17_v2:badLambda'), ...
        'T3c FAIL: wrong error id; got %s', ME.identifier);
end

fprintf('[PASS] T3: lambda_c=0 OK; lambda_c=0.95 OK; lambda_c=1 correctly rejected\n');
fprintf('         (lambda_c=0)    s2_dx = [%.4e, %.4e, %.4e]\n', ...
    s2_dx_lc0(1), s2_dx_lc0(2), s2_dx_lc0(3));

%% --- T4: input validation ----------------------------------------------
% T4a: missing required field
opts_bad = rmfield(opts_base, 'C_dpmr');
try
    predict_closed_loop_var_eq17_v2(opts_bad);
    error('T4a FAIL: missing field should have thrown');
catch ME
    assert(strcmp(ME.identifier, 'predict_closed_loop_var_eq17_v2:missingField'), ...
        'T4a FAIL: wrong error id; got %s', ME.identifier);
end

% T4b: bad axis dimension
opts_bad2 = opts_base;
opts_bad2.a_x_axis = [1, 2];   % only 2 entries
try
    predict_closed_loop_var_eq17_v2(opts_bad2);
    error('T4b FAIL: bad axis dim should have thrown');
catch ME
    assert(strcmp(ME.identifier, 'predict_closed_loop_var_eq17_v2:badAxisDim'), ...
        'T4b FAIL: wrong error id; got %s', ME.identifier);
end

% T4c: bad scenario string
opts_bad3 = opts_base;
opts_bad3.scenario = 'bogus';
try
    predict_closed_loop_var_eq17_v2(opts_bad3);
    error('T4c FAIL: bad scenario should have thrown');
catch ME
    assert(strcmp(ME.identifier, 'predict_closed_loop_var_eq17_v2:badScenario'), ...
        'T4c FAIL: wrong error id; got %s', ME.identifier);
end

% T4d: motion scenario without traj fields
opts_bad4 = opts_base;
opts_bad4.scenario = 'motion';
try
    predict_closed_loop_var_eq17_v2(opts_bad4);
    error('T4d FAIL: motion w/o traj fields should have thrown');
catch ME
    assert(strcmp(ME.identifier, 'predict_closed_loop_var_eq17_v2:motionMissingTraj'), ...
        'T4d FAIL: wrong error id; got %s', ME.identifier);
end

fprintf('[PASS] T4: input validation (missing field / bad dim / bad scenario / motion-missing-traj) all rejected\n');

%% --- T5: motion scenario sanity (1 Hz at h=2.5) -------------------------
% Build motion opts; use representative numbers
opts_motion = opts_base;
opts_motion.scenario       = 'motion';
opts_motion.traj_amplitude = 2.5;
opts_motion.traj_freq      = 1.0;
opts_motion.Q77_axis       = [1e-8, 1e-8, 1e-8];   % motion: nonzero Q77
[s2_dx_m, ~, ~, diag_m] = predict_closed_loop_var_eq17_v2(opts_motion);
assert(all(isfinite(s2_dx_m)) && all(s2_dx_m > 0), ...
    'T5 FAIL: motion s2_dx not finite positive');
% E_bracket^2 should equal (A*omega*Ts)^2/2 across all 3 axes
A_h    = 2.5;
omega  = 2*pi*1.0;
Ts     = 1/1600;
E_expected = (A_h * omega * Ts)^2 / 2;
E_err = abs(diag_m.E_bracket2 - E_expected);
assert(all(E_err < 1e-12), ...
    'T5 FAIL: motion E_bracket2 mismatch; got %s expected %g', mat2str(diag_m.E_bracket2, 4), E_expected);

fprintf('[PASS] T5: motion 1 Hz E_bracket2 = (A*omega*Ts)^2/2 = %.4e (matches)\n', E_expected);
fprintf('         s2_dx (motion)  = [%.4e, %.4e, %.4e]\n', ...
    s2_dx_m(1), s2_dx_m(2), s2_dx_m(3));
fprintf('         CV2_a (motion)  = [%.3e, %.3e, %.3e]\n', ...
    diag_m.CV2_a(1), diag_m.CV2_a(2), diag_m.CV2_a(3));

%% --- Summary -----------------------------------------------------------
fprintf('=== All tests passed ===\n');

end
