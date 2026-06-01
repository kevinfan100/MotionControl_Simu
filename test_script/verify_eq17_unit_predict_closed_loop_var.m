function test_predict_closed_loop_var()
%TEST_PREDICT_CLOSED_LOOP_VAR  Unit tests for predict_closed_loop_var helper
%
%   Run this script (no input/output) to validate Lyapunov predictions
%   against design.md analytical formulas at g=1, plus stability and
%   formula-literal checks.
%
%   Tests:
%     T1: g=1 case, C_dpmr_lyap matches C_dpmr_design within 5%
%     T2: Stability sweep g=[0.5, 1.0, 1.5] all stable; g=2.5 unstable
%     T3: C_dpmr_design literal formula 2 + 1/(1-lambda^2) within 1e-12
%     T4: Sensor C_n at lambda=0.7, g=1 matches C_n_design within ~15%

% Make helper available on path (test sits in test_script/unit_tests/)
this_dir   = fileparts(mfilename('fullpath'));
helper_dir = fullfile(this_dir, '..', '..', 'model', 'diag');
addpath(helper_dir);

% Common physical constants for tests
a_true   = 1.0;     % [um/pN]
sigma2_n = 1e-4;    % [um^2]
kBT      = 4.1e-3;  % [pN*um] (room temp)

fprintf('=== test_predict_closed_loop_var ===\n');

%% T1: g=1, lambda_c=0.7 -> Lyapunov C_dpmr should match design within 5%
lambda_c = 0.7;
pred = predict_closed_loop_var(lambda_c, a_true, 1.0, sigma2_n, kBT);
rel_err = abs(pred.C_dpmr_lyap - pred.C_dpmr_design) / pred.C_dpmr_design;
assert(rel_err < 0.05, ...
    'T1 FAIL: C_dpmr_lyap=%.4f vs design=%.4f, rel_err=%.4f exceeds 5%%', ...
    pred.C_dpmr_lyap, pred.C_dpmr_design, rel_err);
fprintf('[PASS] T1: g=1 C_dpmr_lyap=%.4f matches design=%.4f (rel_err=%.2e)\n', ...
    pred.C_dpmr_lyap, pred.C_dpmr_design, rel_err);

%% T2: Stability sweep
lambda_c = 0.7;
g_stable_list = [0.5, 1.0, 1.5];
for g = g_stable_list
    p = predict_closed_loop_var(lambda_c, a_true, g, sigma2_n, kBT);
    assert(p.stable, 'T2 FAIL: g=%.2f expected stable, got unstable', g);
end
% g=2.5 -> effective feedback = 2.5*0.3 = 0.75; check large g where loop blows up
% Need g large enough that companion poly z^3 - z^2 + g*(1-lambda) has |z|>=1
% At lambda=0.7, the sign-flipping condition for instability: g*(1-lambda) >= 1.0 typical
p_unstable = predict_closed_loop_var(0.7, a_true, 5.0, sigma2_n, kBT);
assert(~p_unstable.stable, ...
    'T2 FAIL: g=5.0 expected unstable, got stable (poles=%s)', ...
    mat2str(p_unstable.poles, 4));
fprintf('[PASS] T2: stability sweep g=[0.5,1.0,1.5] stable; g=5.0 unstable (max|p|=%.3f)\n', ...
    max(p_unstable.poles));

%% T3: C_dpmr_design literal formula match within 1e-12
for lam = [0.3, 0.5, 0.7, 0.9]
    p = predict_closed_loop_var(lam, a_true, 1.0, sigma2_n, kBT);
    expected = 2 + 1/(1 - lam^2);
    abs_err = abs(p.C_dpmr_design - expected);
    assert(abs_err < 1e-12, ...
        'T3 FAIL: lambda=%.2f, C_dpmr_design=%.16f vs %.16f, abs_err=%.2e', ...
        lam, p.C_dpmr_design, expected, abs_err);
end
fprintf('[PASS] T3: C_dpmr_design literal formula 2+1/(1-lambda^2) matches within 1e-12\n');

%% T4: Sensor C_n at lambda=0.7, g=1 within ~15% of design
lambda_c = 0.7;
pred = predict_closed_loop_var(lambda_c, a_true, 1.0, sigma2_n, kBT);
rel_err_Cn = abs(pred.C_n_lyap - pred.C_n_design) / pred.C_n_design;
assert(rel_err_Cn < 0.15, ...
    'T4 FAIL: C_n_lyap=%.4f vs design=%.4f, rel_err=%.4f exceeds 15%%', ...
    pred.C_n_lyap, pred.C_n_design, rel_err_Cn);
fprintf('[PASS] T4: lambda=0.7 C_n_lyap=%.4f vs design=%.4f (rel_err=%.2e)\n', ...
    pred.C_n_lyap, pred.C_n_design, rel_err_Cn);

fprintf('=== All tests passed ===\n');
end
