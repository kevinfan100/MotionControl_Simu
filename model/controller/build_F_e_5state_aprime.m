function F_e = build_F_e_5state_aprime(lambda_c, f_d_i, F_1_i, dFh_i, Delta_h_d)
%BUILD_F_E_5STATE_APRIME  5x5 error-dynamics matrix (per axis); slot 5 = a'_x.
%   Standalone so the L0 Jacobian check can exercise the EXACT matrix the
%   controller uses. See reference/eq17_analysis/derivation/5state_est_aprime.tex.
%
%   F_e = build_F_e_5state_aprime(lambda_c, f_d_i, F_1_i, dFh_i, Delta_h_d)
%
%   cols: 1=dx1 2=dx2 3=dx3 4=a_x 5=a'_x
%   Row 3 = [0 0 lc -F_dx dF_dx^h]
%       F_dx    = f_d[k] + (1-lc)*F_1_i                     (col 4, a_x)
%       dF_dx^h = (1-lc)*sum_i (h_d[k]-h_d[k-i]) f_d[k-i]   (col 5, a'_x; passed as dFh_i)
%   Row 4 = [0 0 0 1 Delta_h_d]    (a_x[k+1] = a_x[k] + a'_x[k]*Delta_h_d[k] + ...)
%   Row 5 = [0 0 0 0 1]            (a'_x random walk)
%
%   vs the rate 5-state build_F_e_5state: F_e(4,5) is Delta_h_d (was the
%   constant 1) and F_e(3,5) is dF_dx^h (was (1-lc)*sum_i i*f_d[k-i]).
%
%   See also: motion_control_law_eq17_5state_aprime, build_F_e_5state

    one_minus_lc = 1 - lambda_c;
    Fe3_a  = -f_d_i - one_minus_lc * F_1_i;     % col 4 (a_x): -F_dx
    Fe3_ap = dFh_i;                             % col 5 (a'_x): dF_dx^h
    F_e = [0 1 0        0      0; ...
           0 0 1        0      0; ...
           0 0 lambda_c Fe3_a  Fe3_ap; ...
           0 0 0        1      Delta_h_d; ...
           0 0 0        0      1];
end
