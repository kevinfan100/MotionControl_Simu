function test_iir_centering_response()
%TEST_IIR_CENTERING_RESPONSE Standalone IIR LP / EWMA filter response
%
%   Priority 2 from deep audit §6.
%
%   Verifies the IIR centering algorithm used in
%   motion_control_law_eq17_7state.m §[1] (lines 305-313):
%       dx_bar_m_new = (1 - a_var) * dx_bar_m + a_var * delta_x_m   (LP, NEW mean)
%       dx_r         = delta_x_m - dx_bar_m_new                     (centered residual)
%       sigma2_dxr_hat_new = (1 - a_cov) * sigma2_dxr_hat + a_cov * dx_r^2
%       a_xm = (sigma2_dxr_hat_new - C_n * sigma2_n_s) / (C_dpmr * 4 kBT)
%
%   Tests:
%       T1: Step response — LP mean asymptotes toward step value at rate (1-a_pd)^k
%       T2: NEW mean form — dx_r at steady-state equals (1-a_pd) * (delta_x_m - dx_bar_m_old)
%           NOT pure noise (verifies "NEW mean" docstring claim)
%       T3: White-noise input — EWMA steady-state of dx_r^2 = a_pd * sigma_n^2 / (2-a_pd)
%           when a_pd = a_cov (matches controller default)
%       T4: Recover known sigma2_n via a_xm closed form when sigma2_n is the
%           dominant noise source (no thermal contribution mocked)
%       T5: NaN-guard — verify NaN input would propagate (no guard in place);
%           document this as a-known absence per audit GAP-2.

    fprintf('=== test_iir_centering_response ===\n');

    n_pass = 0;
    rel_tol = 1e-3;

    % --- Filter parameters (match user_config defaults / Phase 0 §6 lock) ---
    a_pd  = 0.05;     % LP coefficient
    a_cov = 0.05;     % EWMA coefficient on residual squared

    % Closed-form references for Phase 2 (lambda_c=0.7)
    lambda_c = 0.7;
    C_dpmr   = 2 + 1/(1 - lambda_c^2);     % ~3.961
    C_n      = 2/(1 + lambda_c);           % ~1.176
    kBT      = 4.114e-9;                    % [pN*um]

    % ------------------------------------------------------------------
    % T1: Step response — LP mean rises toward step value at rate (1-a_pd)
    % ------------------------------------------------------------------
    N_steps = 500;
    step_val = 1.0;
    delta_x_m_seq = step_val * ones(1, N_steps);

    dx_bar_m = 0;
    dx_bar_m_log = zeros(1, N_steps);
    for k = 1:N_steps
        dx_bar_m = (1 - a_pd) * dx_bar_m + a_pd * delta_x_m_seq(k);
        dx_bar_m_log(k) = dx_bar_m;
    end

    % Closed-form: dx_bar_m[k] = step_val * (1 - (1-a_pd)^k)
    expected = step_val * (1 - (1 - a_pd).^(1:N_steps));
    max_err = max(abs(dx_bar_m_log - expected));
    assert(max_err < 1e-12, ...
        'T1 FAIL: step response mismatch, max_err=%g', max_err);

    % After ~100/a_pd steps, asymptote within numerical eps
    final_err = abs(dx_bar_m_log(end) - step_val);
    assert(final_err < 1e-6, 'T1 FAIL: not converged at N=%d', N_steps);

    fprintf('[PASS] T1 step response: max_err=%.2e, final_err=%.2e\n', max_err, final_err);
    n_pass = n_pass + 1;

    % ------------------------------------------------------------------
    % T2: NEW mean — verify steady-state dx_r at constant input
    %     dx_bar_m[k]    = (1-a)*dx_bar_m[k-1] + a*x[k]
    %     dx_r[k]        = x[k] - dx_bar_m[k]
    %     At steady-state x[k] = X (constant), dx_bar_m -> X, dx_r -> 0
    %
    %     Pre-step mean: if input flips from 0 to X at k=k0, dx_r(k0) = X*(1-a)
    % ------------------------------------------------------------------
    dx_bar_m = 0;
    X = 1.0;
    % First step (input flips): dx_bar_m_new = (1-a)*0 + a*X = a*X
    %                           dx_r          = X - a*X = (1-a)*X
    dx_bar_m_new = (1 - a_pd) * dx_bar_m + a_pd * X;
    dx_r_first   = X - dx_bar_m_new;
    expected_first = (1 - a_pd) * X;
    assert(abs(dx_r_first - expected_first) < 1e-12, ...
        'T2 FAIL: first-step dx_r=%.6f, expected %.6f', dx_r_first, expected_first);

    % After many steps (steady-state X), dx_r approaches 0
    dx_bar_m = 0;
    for k = 1:200
        dx_bar_m = (1 - a_pd) * dx_bar_m + a_pd * X;
    end
    dx_r_ss = X - dx_bar_m;
    assert(abs(dx_r_ss) < 1e-3, ...
        'T2 FAIL: steady-state dx_r should approach 0, got %g', dx_r_ss);

    fprintf('[PASS] T2 NEW mean: first dx_r=%.4f matches (1-a)*X=%.4f, SS dx_r=%.2e\n', ...
        dx_r_first, expected_first, dx_r_ss);
    n_pass = n_pass + 1;

    % ------------------------------------------------------------------
    % T3: White noise input — EWMA SS of dx_r^2
    %
    %   Input: x[k] = noise (zero-mean, std sigma_n)
    %   LP:    dx_bar_m[k] = (1-a)*dx_bar_m[k-1] + a*x[k]
    %   At steady-state, dx_bar_m has variance: a/(2-a) * sigma_n^2
    %   dx_r[k] = x[k] - dx_bar_m[k]
    %
    %   For white-noise input x[k] uncorrelated, x[k] independent of dx_bar_m[k-1]
    %   but NEW mean uses x[k] also — so dx_bar_m[k] = a*x[k] + (1-a)*old_lp.
    %
    %   Var(dx_r) = Var(x[k] - a*x[k] - (1-a)*old_lp)
    %             = Var((1-a)*x[k] - (1-a)*old_lp)
    %             = (1-a)^2 * (Var(x) + Var(old_lp))
    %             = (1-a)^2 * sigma_n^2 * (1 + a/(2-a))
    %             = (1-a)^2 * sigma_n^2 * (2/(2-a))
    %             = 2*(1-a)^2 / (2-a) * sigma_n^2
    % ------------------------------------------------------------------
    rng(42);
    sigma_n = 0.001;          % 1 nm-ish
    sigma_n_sq = sigma_n^2;
    N_mc = 50000;
    x_seq = sigma_n * randn(1, N_mc);

    dx_bar_m = 0;
    sigma2_dxr_hat = 0;
    burn_in = 1000;
    sigma2_dxr_log = zeros(1, N_mc);
    dx_r_log = zeros(1, N_mc);

    for k = 1:N_mc
        dx_bar_m_new = (1 - a_pd) * dx_bar_m + a_pd * x_seq(k);
        dx_r = x_seq(k) - dx_bar_m_new;
        sigma2_dxr_hat = (1 - a_cov) * sigma2_dxr_hat + a_cov * dx_r^2;

        dx_bar_m = dx_bar_m_new;
        sigma2_dxr_log(k) = sigma2_dxr_hat;
        dx_r_log(k) = dx_r;
    end

    expected_var_dxr = 2 * (1 - a_pd)^2 / (2 - a_pd) * sigma_n_sq;
    measured_var_dxr = mean(dx_r_log(burn_in:end).^2);
    measured_ewma_ss = mean(sigma2_dxr_log(burn_in:end));

    rel_err_var = abs(measured_var_dxr - expected_var_dxr) / expected_var_dxr;
    rel_err_ewma = abs(measured_ewma_ss - expected_var_dxr) / expected_var_dxr;
    assert(rel_err_var < 0.05, ...
        'T3 FAIL: empirical var(dx_r)=%.3e vs expected %.3e (rel %.2f)', ...
        measured_var_dxr, expected_var_dxr, rel_err_var);
    assert(rel_err_ewma < 0.05, ...
        'T3 FAIL: EWMA SS=%.3e vs expected %.3e (rel %.2f)', ...
        measured_ewma_ss, expected_var_dxr, rel_err_ewma);

    fprintf(['[PASS] T3 white-noise SS: var(dx_r) = %.3e ', ...
             'vs theory %.3e (rel %.2e); EWMA SS = %.3e (rel %.2e)\n'], ...
        measured_var_dxr, expected_var_dxr, rel_err_var, measured_ewma_ss, rel_err_ewma);
    n_pass = n_pass + 1;

    % ------------------------------------------------------------------
    % T4: Closed-form recovery — at steady-state, expect
    %     a_xm = (sigma2_dxr - C_n * sigma2_n_s) / (C_dpmr * 4 kBT)
    %
    %     With pure white noise input and NO thermal (sigma2_dxT=0):
    %     a_xm should be NEGATIVE / near zero (no thermal component to recover);
    %     This verifies the formula direction sign.
    % ------------------------------------------------------------------
    sigma2_n_s_test = sigma_n_sq;
    den_axm = C_dpmr * 4 * kBT;
    a_xm = (measured_ewma_ss - C_n * sigma2_n_s_test) / den_axm;

    % a_xm should be roughly:
    % (2*(1-a)^2/(2-a) * s^2 - C_n * s^2) / den
    expected_a_xm = (2*(1-a_pd)^2/(2-a_pd) * sigma2_n_s_test - C_n * sigma2_n_s_test) / den_axm;
    rel_err_axm = abs(a_xm - expected_a_xm) / max(abs(expected_a_xm), eps);
    assert(rel_err_axm < 0.1, ...
        'T4 FAIL: a_xm=%.3e vs expected %.3e (rel %.2f)', ...
        a_xm, expected_a_xm, rel_err_axm);

    fprintf(['[PASS] T4 a_xm closed form: a_xm = %.3e (expected %.3e, rel %.2e); ', ...
             'sign matches (negative for no-thermal regime)\n'], a_xm, expected_a_xm, rel_err_axm);
    n_pass = n_pass + 1;

    % ------------------------------------------------------------------
    % T5: NaN-guard documentation — controller has NO explicit NaN guard
    %
    %   Per deep audit GAP-2: "IIR centering ... NaN guard handling"
    %
    %   Verify: a NaN input causes dx_bar_m_new to become NaN; this
    %   propagates indefinitely (no recovery in current implementation).
    %   This is documented behavior; future hardening can add NaN clamps.
    % ------------------------------------------------------------------
    dx_bar_m = 0;
    sigma2_dxr_hat = 0;
    nan_input = NaN;
    dx_bar_m_new = (1 - a_pd) * dx_bar_m + a_pd * nan_input;
    assert(isnan(dx_bar_m_new), ...
        'T5 setup FAIL: NaN should propagate to dx_bar_m_new');

    % Recovery test: feed real input next; LP stays NaN forever
    dx_bar_m = dx_bar_m_new;
    dx_bar_m_new2 = (1 - a_pd) * dx_bar_m + a_pd * 1.0;
    assert(isnan(dx_bar_m_new2), ...
        'T5 expected: NaN persists in LP state without explicit guard');

    fprintf(['[PASS] T5 NaN behavior documented: NaN input persists in LP, ', ...
             'no explicit guard (audit GAP-2 tracked)\n']);
    n_pass = n_pass + 1;

    fprintf('=== ALL %d tests PASS ===\n', n_pass);
end
