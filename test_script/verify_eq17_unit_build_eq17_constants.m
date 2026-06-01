function test_build_eq17_constants()
%TEST_BUILD_EQ17_CONSTANTS Unit tests for build_eq17_constants
%
%   Verifies:
%       T1: Default-options call returns expected C_dpmr, C_n at λ=0.7
%       T2: Option A IF_var ≈ 4.224 at λ=0.7 (rel-tol 1e-2)
%       T3: Option B IF_var ≈ 2.922 at λ=0.7 (rel-tol 1e-3)
%       T4: ξ_per_axis correctness for given σ²_n_s and kBT
%       T5: delay_R2_factor = 5 for d=2; = 14 for d=3
%       T6: Input validation (lambda_c, sigma2_n_s shape, option string)
%       T7: lambda_c sweep monotone-decay sanity (Option A)
%
%   Usage:
%       Run from MATLAB after addpath('../../model/controller'), or invoke
%       directly from this folder.

    % Add controller path so build_eq17_constants is on the search path
    this_dir = fileparts(mfilename('fullpath'));
    repo_root = fullfile(this_dir, '..', '..');
    addpath(fullfile(repo_root, 'model', 'controller'));

    n_pass  = 0;
    rel_tol = 1e-3;

    % Common opts template
    base_opts.lambda_c   = 0.7;
    base_opts.option     = 'A_MA2_full';
    base_opts.sigma2_n_s = [1.0; 2.0; 3.0];   % μm^2
    base_opts.kBT        = 4.114e-9;          % example value (μm-unit-based)

    % ------------------------------------------------------------
    % T1: default-options call → expected C_dpmr, C_n
    % ------------------------------------------------------------
    cc = build_eq17_constants(base_opts);

    expected_C_dpmr = 2 + 1 / (1 - 0.7^2);    % = 3.96078...
    expected_C_n    = 2 / (1 + 0.7);          % = 1.17647...

    assert(abs(cc.C_dpmr - expected_C_dpmr) / expected_C_dpmr < rel_tol, ...
        'T1 failed: C_dpmr = %.6f, expected %.6f', cc.C_dpmr, expected_C_dpmr);
    assert(abs(cc.C_n - expected_C_n) / expected_C_n < rel_tol, ...
        'T1 failed: C_n = %.6f, expected %.6f', cc.C_n, expected_C_n);
    assert(strcmp(cc.option, 'A_MA2_full'), ...
        'T1 failed: option not propagated correctly');
    assert(isfield(cc, 'meta') && isfield(cc.meta, 'option'), ...
        'T1 failed: meta.option missing');

    fprintf('[PASS] T1: Default opts return expected C_dpmr (%.4f), C_n (%.4f)\n', ...
        cc.C_dpmr, cc.C_n);
    n_pass = n_pass + 1;

    % ------------------------------------------------------------
    % T2: Option A IF_var ≈ 4.224 at λ_c = 0.7  (rel-tol 1e-2)
    % ------------------------------------------------------------
    optsA = base_opts;
    optsA.option = 'A_MA2_full';
    ccA = build_eq17_constants(optsA);

    expected_IF_A = 4.224;
    rel_err_A = abs(ccA.IF_var - expected_IF_A) / expected_IF_A;
    assert(rel_err_A < 1e-2, ...
        'T2 failed: Option A IF_var = %.4f, expected ~%.4f (rel_err=%.3e)', ...
        ccA.IF_var, expected_IF_A, rel_err_A);
    fprintf('[PASS] T2: Option A IF_var = %.4f (~%.3f, rel_err %.2e)\n', ...
        ccA.IF_var, expected_IF_A, rel_err_A);
    n_pass = n_pass + 1;

    % ------------------------------------------------------------
    % T3: Option B IF_var ≈ 2.922 at λ_c = 0.7  (rel-tol 1e-3)
    % ------------------------------------------------------------
    optsB = base_opts;
    optsB.option = 'B_AR1_approx';
    ccB = build_eq17_constants(optsB);

    expected_IF_B = (1 + 0.7^2) / (1 - 0.7^2);   % = 2.92157
    rel_err_B = abs(ccB.IF_var - expected_IF_B) / expected_IF_B;
    assert(rel_err_B < rel_tol, ...
        'T3 failed: Option B IF_var = %.6f, expected %.6f (rel_err=%.3e)', ...
        ccB.IF_var, expected_IF_B, rel_err_B);
    fprintf('[PASS] T3: Option B IF_var = %.4f (closed form %.4f)\n', ...
        ccB.IF_var, expected_IF_B);
    n_pass = n_pass + 1;

    % ------------------------------------------------------------
    % T4: ξ_per_axis correctness
    % ------------------------------------------------------------
    s2 = [0.5; 1.5; 2.0];
    K  = 5e-9;
    optsX = base_opts;
    optsX.sigma2_n_s = s2;
    optsX.kBT        = K;
    ccX = build_eq17_constants(optsX);

    xi_expected = (ccX.C_n / ccX.C_dpmr) * s2 / (4 * K);
    assert(numel(ccX.xi_per_axis) == 3, ...
        'T4 failed: xi_per_axis must have 3 elements');
    rel_err_xi = max(abs(ccX.xi_per_axis - xi_expected) ./ abs(xi_expected));
    assert(rel_err_xi < rel_tol, ...
        'T4 failed: xi_per_axis mismatch, max rel_err = %.3e', rel_err_xi);
    fprintf('[PASS] T4: xi_per_axis matches manual calc (max rel_err %.2e)\n', ...
        rel_err_xi);
    n_pass = n_pass + 1;

    % ------------------------------------------------------------
    % T5: delay_R2_factor = 5 for d=2; = 14 for d=3
    % ------------------------------------------------------------
    optsD2 = base_opts; optsD2.d = 2;
    ccD2 = build_eq17_constants(optsD2);
    assert(ccD2.delay_R2_factor == 5, ...
        'T5 failed: delay_R2_factor for d=2 = %g, expected 5', ...
        ccD2.delay_R2_factor);

    optsD3 = base_opts; optsD3.d = 3;
    ccD3 = build_eq17_constants(optsD3);
    assert(ccD3.delay_R2_factor == 14, ...
        'T5 failed: delay_R2_factor for d=3 = %g, expected 14', ...
        ccD3.delay_R2_factor);

    fprintf('[PASS] T5: delay_R2_factor = 5 (d=2), 14 (d=3)\n');
    n_pass = n_pass + 1;

    % ------------------------------------------------------------
    % T6: Input validation
    % ------------------------------------------------------------
    % T6a: lambda_c = 1.0 should error
    bad = base_opts; bad.lambda_c = 1.0;
    assert_throws(@() build_eq17_constants(bad), ...
        'build_eq17_constants:invalidLambda', 'T6a (lambda_c=1.0)');

    % T6b: lambda_c = 0 should error
    bad = base_opts; bad.lambda_c = 0;
    assert_throws(@() build_eq17_constants(bad), ...
        'build_eq17_constants:invalidLambda', 'T6b (lambda_c=0)');

    % T6c: sigma2_n_s wrong shape
    bad = base_opts; bad.sigma2_n_s = [1; 2];   % only 2 elements
    assert_throws(@() build_eq17_constants(bad), ...
        'build_eq17_constants:invalidSigma2', 'T6c (sigma2_n_s 2-elem)');

    % T6d: sigma2_n_s negative entry
    bad = base_opts; bad.sigma2_n_s = [1; -1; 2];
    assert_throws(@() build_eq17_constants(bad), ...
        'build_eq17_constants:invalidSigma2', 'T6d (sigma2_n_s negative)');

    % T6e: kBT non-positive
    bad = base_opts; bad.kBT = -1e-9;
    assert_throws(@() build_eq17_constants(bad), ...
        'build_eq17_constants:invalidKBT', 'T6e (kBT negative)');

    % T6f: option string invalid
    bad = base_opts; bad.option = 'C_unknown';
    assert_throws(@() build_eq17_constants(bad), ...
        'build_eq17_constants:invalidOption', 'T6f (bad option)');

    fprintf('[PASS] T6: Input validation correctly rejects invalid args\n');
    n_pass = n_pass + 1;

    % ------------------------------------------------------------
    % T7: lambda_c sweep, Option A — IF_var > 1, monotone causal decay
    % ------------------------------------------------------------
    lambdas = [0.3, 0.5, 0.7, 0.9];
    for k = 1:numel(lambdas)
        lc = lambdas(k);
        optsK = base_opts;
        optsK.lambda_c = lc;
        optsK.option   = 'A_MA2_full';
        ccK = build_eq17_constants(optsK);

        assert(ccK.IF_var > 1, ...
            'T7 failed at lambda=%.2f: IF_var = %.4f, expected > 1', lc, ccK.IF_var);

        % Recompute internal rho_dx_1, rho_dx_2 to assert monotone decay
        denom_e = 1 + 2*(1 - lc)^2;
        rho_e_1 = (1 - lc) * (2 - lc) / denom_e;
        rho_e_2 = (1 - lc) / denom_e;
        Var_dx_over_sig_e = (1 + 2*lc*rho_e_1 + 2*lc^2*rho_e_2) / (1 - lc^2);
        inv_var_ratio = 1 / Var_dx_over_sig_e;
        rho_dx_1 = lc + inv_var_ratio * (rho_e_1 + lc*rho_e_2);
        rho_dx_2 = lc * rho_dx_1 + inv_var_ratio * rho_e_2;

        assert(rho_dx_1 > rho_dx_2, ...
            'T7 failed at lambda=%.2f: rho_dx_1=%.4f not > rho_dx_2=%.4f', ...
            lc, rho_dx_1, rho_dx_2);
    end
    fprintf('[PASS] T7: lambda_c sweep — IF_var > 1 and rho_dx monotone for [0.3, 0.5, 0.7, 0.9]\n');
    n_pass = n_pass + 1;

    fprintf('\n=== ALL %d tests PASS ===\n', n_pass);
end

function assert_throws(fn, expected_id, label)
%ASSERT_THROWS Assert that fn() throws an error with given identifier.
    threw = false;
    try
        fn();
    catch ME
        if strcmp(ME.identifier, expected_id)
            threw = true;
        else
            error('%s failed: expected error id ''%s'', got ''%s'' (msg: %s)', ...
                label, expected_id, ME.identifier, ME.message);
        end
    end
    assert(threw, '%s failed: expected an error to be thrown', label);
end
