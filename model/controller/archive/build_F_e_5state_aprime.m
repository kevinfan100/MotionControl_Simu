% STATUS: FALSIFIED (support of 5state_aprime) -- see memory project_aprime_var_esti_2026-07-15
function F_e = build_F_e_5state_aprime(lambda_c, f_d_i, F_1_i, dFh_i, Delta_h_d, dxh3_selfmod)
%BUILD_F_E_5STATE_APRIME  5x5 error-dynamics matrix (per axis); slot 5 = a'_x.
%   Standalone so the L0 Jacobian check can exercise the EXACT matrix the
%   controller uses. See reference/eq17_analysis/derivation/5state_est_aprime.tex
%   (baseline) and reference/eq17_analysis/derivation/5state_aprime_unified.tex
%   Section 3-5 (self-mod extension).
%
%   F_e = build_F_e_5state_aprime(lambda_c, f_d_i, F_1_i, dFh_i, Delta_h_d)
%   F_e = build_F_e_5state_aprime(lambda_c, f_d_i, F_1_i, dFh_i, Delta_h_d, dxh3_selfmod)
%
%   cols: 1=dx1 2=dx2 3=dx3 4=a_x 5=a'_x
%   Row 3 = [0 0 lc -F_dx dF_dx^h]
%       F_dx    = f_d[k] + (1-lc)*F_1_i                     (col 4, a_x)
%       dF_dx^h = (1-lc)*sum_i (h_d[k]-h_d[k-i]) f_d[k-i]   (col 5, a'_x; passed as dFh_i)
%   Row 4 = [0 0 0 1 Delta_h_d + (1-lc)*dxh3_selfmod]
%       a_x[k+1] = a_x[k] + a'_x[k]*[Delta_h_d[k] + (1-lc)*delta_x_hat_3[k]] + ...
%       dxh3_selfmod (optional, default 0) is the CURRENT delta_x_hat_3 estimate
%       (the filtered tracking-error-3 state, NOT the raw dx_r residual --
%       see 5state_aprime_unified.tex Section 6). When 0 (default), this row
%       reduces EXACTLY to the baseline F_e(4,5) = Delta_h_d (motion-only
%       observability). When nonzero (use_selfmod=true in the controller),
%       the thermal self-dither term restores hold-observability
%       (5state_aprime_selfmod.tex: rank(O_N) 4 -> 5 on a height hold).
%   Row 5 = [0 0 0 0 1]            (a'_x random walk)
%
%   vs the rate 5-state build_F_e_5state: F_e(4,5) is Delta_h_d (was the
%   constant 1) and F_e(3,5) is dF_dx^h (was (1-lc)*sum_i i*f_d[k-i]).
%
%   See also: motion_control_law_eq17_5state_aprime, build_F_e_5state

    if nargin < 6 || isempty(dxh3_selfmod)
        dxh3_selfmod = 0;
    end
    one_minus_lc = 1 - lambda_c;
    Fe3_a  = -f_d_i - one_minus_lc * F_1_i;     % col 4 (a_x): -F_dx
    Fe3_ap = dFh_i;                             % col 5 (a'_x): dF_dx^h
    Fe4_ap = Delta_h_d + one_minus_lc * dxh3_selfmod;   % col 5 (a'_x), row 4 (a_x)
    F_e = [0 1 0        0      0; ...
           0 0 1        0      0; ...
           0 0 lambda_c Fe3_a  Fe3_ap; ...
           0 0 0        1      Fe4_ap; ...
           0 0 0        0      1];
end
