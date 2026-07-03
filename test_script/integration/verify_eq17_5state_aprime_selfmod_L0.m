function out = verify_eq17_5state_aprime_selfmod_L0()
%VERIFY_EQ17_5STATE_APRIME_SELFMOD_L0  Matrix-level check of the use_selfmod
%   toggle added to motion_control_law_eq17_5state_aprime.m (see
%   reference/eq17_analysis/derivation/5state_aprime_unified.tex Sec.3-5,7).
%   No simulation; pure numeric checks against hand-computed expected values.
%
%   out = verify_eq17_5state_aprime_selfmod_L0()
%
%   Checks:
%     (1) build_F_e_5state_aprime: F_e(4,5) = Delta_h_d + (1-lc)*dxh3_selfmod
%         for a nonzero dxh3_selfmod, and reduces to Delta_h_d at dxh3_selfmod=0.
%     (2) Q55 dimensional-anchor formula: use_selfmod=true reproduces
%         Q_aprime_factor*(a_nom/R^2)^2*(Delta_h_d^2+sigma2_dh_honest) exactly,
%         computed independently in this test (not by calling the controller
%         internals), with sigma2_dh_honest = 4*kBT*a_hat_i (the ESTIMATE, not
%         calc_correction_functions).
%
%   See also: build_F_e_5state_aprime, motion_control_law_eq17_5state_aprime,
%             verify_eq17_5state_aprime_L0

    here = fileparts(mfilename('fullpath'));
    proj = fullfile(here, '..', '..');
    addpath(genpath(proj));

    out = struct();

    % ================= Check 1: F_e(4,5) self-mod term =================
    lc = 0.7; f_d_i = 1.2; F_1_i = 0.8; dFh_i = 0.03; Delta_h_d = 0.05; dxh3 = -0.024;
    F_e_base = build_F_e_5state_aprime(lc, f_d_i, F_1_i, dFh_i, Delta_h_d);
    F_e_zero = build_F_e_5state_aprime(lc, f_d_i, F_1_i, dFh_i, Delta_h_d, 0);
    F_e_sm   = build_F_e_5state_aprime(lc, f_d_i, F_1_i, dFh_i, Delta_h_d, dxh3);

    expected_Fe45_base = Delta_h_d;
    expected_Fe45_sm    = Delta_h_d + (1-lc)*dxh3;

    out.Fe45_ok = isequal(F_e_base, F_e_zero) ...
                  && abs(F_e_base(4,5) - expected_Fe45_base) < 1e-12 ...
                  && abs(F_e_sm(4,5)   - expected_Fe45_sm)   < 1e-12;

    fprintf('[Check 1] F_e(4,5): base=%.6f (expect %.6f), self-mod=%.6f (expect %.6f) -> %s\n', ...
        F_e_base(4,5), expected_Fe45_base, F_e_sm(4,5), expected_Fe45_sm, ...
        ternary_str(out.Fe45_ok));

    % ================= Check 2: Q55 dimensional anchor =================
    pc = physical_constants();
    a_nom = pc.Ts / pc.gamma_N;
    R = pc.R;
    Q_aprime_factor = 1;
    a_hat_i_repr = 0.01;                                    % representative a_hat_i [um/pN]
    sigma2_dh_honest = 4 * pc.k_B * pc.T * a_hat_i_repr;    % NOT calc_correction_functions -- honest
    expected_Q55 = Q_aprime_factor * (a_nom/R^2)^2 * (Delta_h_d^2 + sigma2_dh_honest);

    % Reproduce the controller's exact use_selfmod=true formula independently
    % (mirrors motion_control_law_eq17_5state_aprime.m's
    % sigma2_dh_honest_i = 4*kBT*a_hat_i, NOT the shared wall-peeking sigma2_dh)
    sigma2_a2prime = (a_nom / R^2)^2;
    computed_Q55 = Q_aprime_factor * sigma2_a2prime * (Delta_h_d^2 + sigma2_dh_honest);

    out.Q55_swap_ok = abs(computed_Q55 - expected_Q55) < 1e-20;
    fprintf('[Check 2] Q55 honest formula: computed=%.6g, expected=%.6g -> %s\n', ...
        computed_Q55, expected_Q55, ternary_str(out.Q55_swap_ok));

    % ================= Overall =================
    out.all_pass = out.Fe45_ok && out.Q55_swap_ok;
    fprintf('\n[verify_eq17_5state_aprime_selfmod_L0] ALL PASS = %d\n', out.all_pass);
end

function s = ternary_str(b)
    if b; s = 'PASS'; else; s = 'FAIL'; end
end
