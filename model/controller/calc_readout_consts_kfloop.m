% STATUS: ACTIVE | derivation: reference/eq17_analysis/derivation/1007_conly_meng_law.tex S7 (2026-10-07, branch test/conly-meng-law)
function T = calc_readout_consts_kfloop(lc, apd, a_cov, d, K1_grid)
%CALC_READOUT_CONSTS_KFLOOP  Gain-readout constants of the loop closed through the KF estimate (journal law (J6)).
%   Under f_d = (1/a_hat){Delta w_d + (1-lc) dw3_hat} the tracking error is fed back through the position-block Kalman
%   gain K1 = m/(m+r) (1007 tex S7), so every constant of the a_m readout chain -- the residual-variance coefficients
%   C_T (thermal) and C_N (sensor), the s-weighted autocorrelation sums (A, B, C) of the IF colour factor, and the echo
%   sensitivities S_T, S_n -- is a function of K1 instead of a number. This builds them on a K1 grid by exact discrete
%   Lyapunov solves of the 7-state hold loop (d = 2 only):
%       z = [dw1 dw2 dw3 dw1_hat dw2_hat dw3_hat xb],  inputs n[k] (sensor), w[k] (thermal), gain ratio g = a/a_hat,
%       y[k] = dw1 + n ;  x^- = [x2, x3, lc x3] ;  x = x^- + K1 [1 1 1]' (y - x2) ;  u = g (1-lc) x3 ;
%       dw3+ = dw3 - u - w ;  xb+ = (1-apd) xb + apd y ;  OUTPUT dw_r = (1-apd)(y - xb)   (the signal a_m inverts).
%   Self-check (run by check_readout_consts_kfloop.m): at K1 = 1 the loop is identical to the raw-measurement law, so
%   C_T, C_N, A, B, C, S_T, S_n must reproduce build_eq17_6state_constants / calc_ctrl_params at the same (lc, apd, a_cov).
%   Conventions copied from build_eq17_6state_constants.m (compute_if_abc, compute_y2_mirror) and
%   motion_control_law_formC_b.m init (S_echo_T / S_echo_n: log-derivative over g in {1/(1+eps), 1/(1-eps)}).
%   The controller evaluates K1 from (kappa_T a_hat, sigma2_n) with kfloop_K1 below and interpolates the table.
    if nargin < 4 || isempty(d); d = 2; end
    if nargin < 5 || isempty(K1_grid); K1_grid = [0.005, 0.01:0.01:0.99, 0.995, 0.999, 1.0]; end
    assert(d == 2, 'calc_readout_consts_kfloop:delay', 'derived for d = 2 only (got %g)', d);
    n = numel(K1_grid);
    s_ewma = 1 - a_cov;  Tmax = 600;  epE = 1e-4;
    T = struct('K1', K1_grid(:), 'lc', lc, 'apd', apd, 'a_cov', a_cov, 'd', d);
    T.C_T = zeros(n, 1);  T.C_N = zeros(n, 1);
    T.IF_abc = zeros(n, 3);  T.IF_abc_white = zeros(n, 3);
    T.S_T = zeros(n, 1);  T.S_n = zeros(n, 1);
    T.K1_of = @local_K1;                                 % K1(q, r) of 1007 tex S7, for the controller's per-step lookup
    for i = 1:n
        K1 = K1_grid(i);
        [A, Bn, Bw, c, dn] = local_ss(1, lc, apd, K1);
        PT = local_dlyap(A, Bw * Bw');  PN = local_dlyap(A, Bn * Bn');
        T.C_T(i) = c' * PT * c;
        T.C_N(i) = c' * PN * c + dn^2;
        % autocovariances of dw_r per unit-variance source, lags 1..Tmax
        RT = zeros(Tmax, 1);  RN = zeros(Tmax, 1);
        AkT = PT * c;  AkN = PN * c;  Akm1Bn = Bn;        % A^tau P c  and  A^(tau-1) Bn
        for tau = 1:Tmax
            AkT = A * AkT;  AkN = A * AkN;
            RT(tau) = c' * AkT;
            RN(tau) = c' * AkN + c' * Akm1Bn * dn;        % direct feed-through of n[k] into y[k] (tau >= 1)
            Akm1Bn = A * Akm1Bn;
        end
        for ws = 1:2
            s = s_ewma;  if ws == 2; s = 1; end
            st = s .^ (1:Tmax)';
            abc = [sum(RT.^2 .* st), sum(RT .* RN .* st), sum(RN.^2 .* st)];
            if ws == 1; T.IF_abc(i, :) = abc; else; T.IF_abc_white(i, :) = abc; end
        end
        % echo sensitivities, the formC_b init convention
        vE = zeros(2, 2);  gE = [1/(1+epE), 1/(1-epE)];
        for iG = 1:2
            [Ag, Bng, Bwg, cg, dng] = local_ss(gE(iG), lc, apd, K1);
            vE(1, iG) = cg' * local_dlyap(Ag, Bwg * Bwg') * cg;
            vE(2, iG) = cg' * local_dlyap(Ag, Bng * Bng') * cg + dng^2;
        end
        T.S_T(i) = (log(vE(1, 1)) - log(vE(1, 2))) / (2 * epE);
        T.S_n(i) = (log(vE(2, 1)) - log(vE(2, 2))) / (2 * epE);
    end
end

function K1 = local_K1(q, r)
%LOCAL_K1  Steady-state y1 gain of the position block, 1007 tex S7: m = (q + sqrt(q^2 + 4 q r))/2, K1 = m/(m + r).
    m  = (q + sqrt(q.^2 + 4 .* q .* r)) / 2;
    K1 = m ./ (m + r);
end

function [A, Bn, Bw, c, dn] = local_ss(g, lc, apd, K1)
    beta = 1 - lc;  om = 1 - apd;
    A = zeros(7);  Bn = zeros(7, 1);  Bw = zeros(7, 1);
    A(1, 2) = 1;
    A(2, 3) = 1;
    A(3, :) = [-g*beta*K1, 0, 1, 0, g*beta*K1, -g*beta*lc, 0];  Bn(3) = -g*beta*K1;  Bw(3) = -1;
    A(4, :) = [K1, 0, 0, 0, 1 - K1, 0, 0];                      Bn(4) = K1;
    A(5, :) = [K1, 0, 0, 0, -K1, 1, 0];                         Bn(5) = K1;
    A(6, :) = [K1, 0, 0, 0, -K1, lc, 0];                        Bn(6) = K1;
    A(7, :) = [apd, 0, 0, 0, 0, 0, om];                         Bn(7) = apd;
    c  = om * [1; 0; 0; 0; 0; 0; -1];
    dn = om;
end

function X = local_dlyap(A, Q)
%LOCAL_DLYAP  X = A X A' + Q by the Kronecker solve (7 states: 49 x 49, exact).
    nA = size(A, 1);
    X = reshape((eye(nA^2) - kron(A, A)) \ Q(:), nA, nA);
    X = (X + X') / 2;
end
