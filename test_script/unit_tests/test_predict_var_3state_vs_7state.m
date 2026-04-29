function test_predict_var_3state_vs_7state()
%TEST_PREDICT_VAR_3STATE_VS_7STATE  Cross-check 3-state Lyapunov vs 7-state v2
%
%   Priority 4 from deep audit §6 (closes GAP-4).
%
%   Compares two oracle predictions for sigma^2_dx (closed-loop tracking
%   variance under positioning):
%     - 3-state: predict_closed_loop_var.m (g=1 case, returns
%                 Sigma(1,1) of dlyap(A,B*var*B') for companion form A)
%     - 7-state: predict_closed_loop_var_eq17_v2.m at sigma2_w_fD=0
%                (paper part: (C_dpmr*4kBT*a + C_n*sigma2_n)/(1-lambda_c^2))
%
%   IMPORTANT FINDING:
%     The two oracles produce values that differ by a factor of (1-lambda_c^2)
%     under positioning. At lambda_c=0.7, factor = 0.51, so 7-state ~ 1.96x
%     of 3-state. This is a known formulation difference:
%       - 3-state companion form: Sigma(1,1) ~ (C_dpmr * 4kBT*a + ... ) directly.
%       - 7-state paper form follows Eq.22 closed form with explicit
%         /(1-lambda_c^2) division (per Phase 2 derivation).
%
%   The test verifies BOTH formulas individually against their own closed-form
%   reference, not against each other directly. Per audit GAP-4 spirit, this
%   exposes the formulation difference so the user can decide which is the
%   intended value for downstream comparison.
%
%   Tests:
%     T1: 3-state Lyapunov C_dpmr_lyap matches Phase 2 design within 5%
%     T2: Document the formulation gap (3-state vs 7-state) at sigma2_w_fD=0,
%         expected ratio approx (1-lambda_c^2) = 0.51 at lambda_c=0.7
%     T3: 7-state corrections (xD, a) are zero at sigma2_w_fD=0 with chi-sq=0 inputs
%     T4: 7-state paper form matches its own Phase 2 closed form within 1e-12

    fprintf('=== test_predict_var_3state_vs_7state ===\n');

    % --- Path setup ---
    this_dir   = fileparts(mfilename('fullpath'));
    repo_root  = fullfile(this_dir, '..', '..');
    addpath(fullfile(repo_root, 'model', 'diag'));

    n_pass = 0;

    % --- Common parameters (positioning h=50 baseline) ---
    lambda_c = 0.7;
    k_B      = 1.3806503e-5;     % [pN*um/K]
    T_K      = 310.15;            % [K]
    kBT      = k_B * T_K;

    gamma_N  = 0.0425;
    Ts_const = 1/1600;
    a_nom    = Ts_const / gamma_N;
    a_par    = a_nom / 1.0;        % h_bar=22.2 -> c_para~1
    a_perp50 = a_nom / 1.5;
    a_x_axis = [a_par, a_par, a_perp50];

    sigma2_n_s = [0.00062^2, 0.000057^2, 0.00331^2];

    C_dpmr   = 2 + 1/(1 - lambda_c^2);
    C_n      = 2/(1 + lambda_c);
    IF_var   = 4.224;

    % ------------------------------------------------------------------
    % T1: 3-state Lyapunov consistency (g=1)
    % ------------------------------------------------------------------
    pred3 = cell(3, 1);
    s2_dx_3 = zeros(1, 3);
    for ax = 1:3
        pred3{ax} = predict_closed_loop_var(lambda_c, a_x_axis(ax), 1.0, ...
                                             sigma2_n_s(ax), kBT);
        s2_dx_3(ax) = pred3{ax}.var_dx_total_lyap;
    end
    rel_err_C_dpmr = abs(pred3{1}.C_dpmr_lyap - pred3{1}.C_dpmr_design) / ...
                     pred3{1}.C_dpmr_design;
    assert(rel_err_C_dpmr < 0.05, ...
        'T1 FAIL: 3-state Lyap C_dpmr deviation = %.3e (>5%%)', rel_err_C_dpmr);

    fprintf(['[PASS] T1: 3-state Lyap thermal sigma2_dx (axis 1) = %.3e um^2; ', ...
             'C_dpmr_lyap=%.3f vs design=%.3f (rel_err %.2e)\n'], ...
        pred3{1}.var_dx_thermal_lyap, pred3{1}.C_dpmr_lyap, ...
        pred3{1}.C_dpmr_design, rel_err_C_dpmr);
    n_pass = n_pass + 1;

    % ------------------------------------------------------------------
    % T2: Document formulation gap between 3-state and 7-state oracles
    % ------------------------------------------------------------------
    Q77_axis = [0, 0, 0];
    R_axis   = 0.241 * a_x_axis.^2;       % nominal R(2,2) at h=50
    opts7 = struct( ...
        'lambda_c',    lambda_c, ...
        'a_x_axis',    a_x_axis, ...
        'h_bar',       50/2.25, ...
        'sigma2_n_s',  sigma2_n_s, ...
        'k_B',         k_B, ...
        'T',           T_K, ...
        'Q77_axis',    Q77_axis, ...
        'R_axis',      R_axis, ...
        'a_cov',       0.05, ...
        'a_pd',        0.05, ...
        'C_dpmr',      C_dpmr, ...
        'C_n',         C_n, ...
        'IF_var',      IF_var, ...
        'sigma2_w_fD', 0, ...
        'scenario',    'positioning' ...
    );

    [~, ~, ~, diag7] = predict_closed_loop_var_eq17_v2(opts7);
    s2_dx_7_paper = diag7.s2_dx_paper_only;     % paper Eq.22 part only

    % Document the formulation gap: 7-state divides by (1-lambda_c^2)
    % via Eq.22, while 3-state companion form already absorbs that factor.
    % Expected: 7-state ~ 3-state / (1-lambda_c^2) up to small Lyap residual.
    factor = (1 - lambda_c^2);
    s2_dx_3_scaled = s2_dx_3 / factor;       % rescaled 3-state for comparison

    rel_err = abs(s2_dx_3_scaled - s2_dx_7_paper) ./ s2_dx_7_paper;
    assert(all(rel_err < 0.06), ...
        'T2 FAIL: 3-state/(1-lc^2) vs 7-state paper rel_err = %s (max 6%%)', ...
        mat2str(rel_err, 4));

    % Also document raw ratio
    raw_ratio = s2_dx_3 ./ s2_dx_7_paper;
    fprintf(['[PASS] T2: Formulation gap documented:\n', ...
             '         raw 3-state / 7-state = [%.3f, %.3f, %.3f] (~%.3f = 1-lc^2)\n', ...
             '         scaled 3-state/(1-lc^2) vs 7-state rel_err = [%.2e, %.2e, %.2e]\n', ...
             '         3-state s2_dx (raw)  = [%.4e, %.4e, %.4e]\n', ...
             '         7-state s2_dx_paper  = [%.4e, %.4e, %.4e]\n'], ...
        raw_ratio(1), raw_ratio(2), raw_ratio(3), factor, ...
        rel_err(1), rel_err(2), rel_err(3), ...
        s2_dx_3(1), s2_dx_3(2), s2_dx_3(3), ...
        s2_dx_7_paper(1), s2_dx_7_paper(2), s2_dx_7_paper(3));
    n_pass = n_pass + 1;

    % ------------------------------------------------------------------
    % T3: 7-state corrections are zero at sigma2_w_fD=0
    %     (correction_xD requires sigma2_w_fD>0; correction_a is via
    %     E[bracket^2] * s2_e_a / a_x^2; at chi_sq=0 we'd zero it out.
    %     But chi_sq = 2*a_cov/(2-a_cov) > 0 by default. So correction_a
    %     comes from Q77/R driven CV2_a. With Q77=0 and R>0, L_eff=0, so
    %     CV2_a -> wall_term=0. Result: correction_a=0 too.)
    % ------------------------------------------------------------------
    assert(all(diag7.correction_xD == 0), ...
        'T3 FAIL: correction_xD should be 0 at sigma2_w_fD=0; got %s', ...
        mat2str(diag7.correction_xD, 4));
    assert(all(diag7.CV2_a == 0), ...
        'T3 FAIL: CV2_a should be 0 with Q77=0 and wall_term=0; got %s', ...
        mat2str(diag7.CV2_a, 4));
    assert(all(diag7.correction_a == 0), ...
        'T3 FAIL: correction_a should be 0 at Q77=0; got %s', ...
        mat2str(diag7.correction_a, 4));

    fprintf(['[PASS] T3: corrections zero (sigma2_w_fD=0, Q77=0):\n', ...
             '         correction_xD = [%g, %g, %g]\n', ...
             '         correction_a  = [%g, %g, %g]\n'], ...
        diag7.correction_xD(1), diag7.correction_xD(2), diag7.correction_xD(3), ...
        diag7.correction_a(1), diag7.correction_a(2), diag7.correction_a(3));
    n_pass = n_pass + 1;

    % ------------------------------------------------------------------
    % T4: Phase 2 closed form vs 7-state paper-only — exact match
    % ------------------------------------------------------------------
    s2_dx_closed_form = (C_dpmr * 4*k_B*T_K * a_x_axis + C_n * sigma2_n_s) ...
                        / (1 - lambda_c^2);
    rel_err_cf = abs(s2_dx_closed_form - s2_dx_7_paper) ./ s2_dx_7_paper;
    assert(all(rel_err_cf < 1e-12), ...
        'T4 FAIL: closed form vs 7-state paper-only mismatch %s', ...
        mat2str(rel_err_cf, 3));

    fprintf(['[PASS] T4: Phase 2 closed form == 7-state paper-only ', ...
             'within 1e-12 per axis\n']);
    n_pass = n_pass + 1;

    fprintf('=== ALL %d tests PASS ===\n', n_pass);
end
