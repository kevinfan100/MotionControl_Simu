function out = check_second_order_predict_term(seeds)
%CHECK_SECOND_ORDER_PREDICT_TERM  Does the filter's own P predict the law-leg
%   covariance term measured in split_rectification_legs (+0.0068 a_o)?
%
%   Row 4 of predict is bilinear in (a_hat, dx3):  f4 = A + a'(A) [dw_d + (1-lc) x3 + ...].
%   A first-order EKF propagates the mean as f4(x_hat); the mean of f4 over the
%   posterior is larger by the second-order term
%       1/2 sum_ij d2f4/dx_i dx_j P_ij  =  (1-lc) da'/dA P43  +  1/2 d2a'/dA2 M P44
%   with da'/dA = -2 b (1-A),  d2a'/dA2 = 2 b.   Route C = the first term.
%   Registered prediction: sum over the oscillation of (1-lc) da'/dA P43 equals
%   the measured covariance part +0.0068 within 30 %.
%   RESULT 2026-08-30: +0.0098 +- 0.00002 vs +0.0068 (ratio 1.44; sign, order and
%   seed-independence match; the 44 % is the filter's P43 vs the realised
%   covariance -- P43 honesty, not yet measured). First run had P43/a_nom (unit slip).
%   Arm: b_true oracle + exact step, canonical, noisy, log_P_full.
% STATUS: ACTIVE | step 1 of the remaining-bias plan (2026-08-30)

    if nargin < 1 || isempty(seeds); seeds = 1:8; end
    ax = 3;  R = physical_constants().R;  lam = 0.7;
    O = run_formC_b(struct('arm', 'best', 'ap_src', 'post', 'seeds', seeds, 'verbose', false, ...
                           'b_true', true, 'b_true_at', 'cmd', 'log_P_full', true, ...
                           'ctrl_const_override', struct('law_exact_step', true)));
    ns = numel(O.runs);  t = O.runs{1}.tout(:);  a_nom = O.runs{1}.a_nom;
    kk = 2:numel(t);  tt = t(kk);  m = tt >= 1.5 & tt < 3.5;
    T1 = zeros(1, ns);  T2 = zeros(1, ns);  C = zeros(1, ns);  P43m = zeros(1, ns);  P41m = zeros(1, ns);
    for q = 1:ns
        r = O.runs{q};
        ah = r.a_bar_hat_out(:, ax);  bh = r.b_hat_out(:, ax);  dx3 = r.delta_x_hat_3_out(:, ax) / R;
        dwd = [0; diff(r.h_bar_d_out(:))];
        P = r.P_full_out;                                  % N x np x np x 3, posterior, in controller units
        % slot 4 is the DIMENSIONLESS a_bar inside the controller (P_a_out is
        % scaled by a_disp^2 on the way out; P_full is not), slot 3 is in R.
        P43 = squeeze(P(:, 4, 3, ax));                     % a_o x R
        P41 = squeeze(P(:, 4, 1, ax));
        P44 = squeeze(P(:, 4, 4, ax));
        dw = dwd(kk) + (1 - lam) * dx3(kk-1);
        dap = -2 * bh(kk-1) .* (1 - ah(kk-1));             % da'/dA at the posterior of k-1
        t1 = (1 - lam) * dap .* P43(kk-1);                 % route C, per step
        t2 = 0.5 * 2 * bh(kk-1) .* dw .* P44(kk-1);        % a'' M P44 / 2, per step
        T1(q) = sum(t1(m));  T2(q) = sum(t2(m));
        P43m(q) = mean(P43(kk(m)-1));  P41m(q) = mean(P41(kk(m)-1));
        % the measured covariance needs the cross-seed mean; recompute below
        AP(:, q) = bh(kk-1) .* (1 - ah(kk-1)).^2;  DW(:, q) = dw;  %#ok<AGROW>
    end
    apb = mean(AP, 2);  dwb = mean(DW, 2);
    cov_meas = sum(mean((AP(m,:) - apb(m)) .* (DW(m,:) - dwb(m)), 2));
    fprintf('\n=== second-order predict term, b_true + exact, %d seeds, oscillation 1.5-3.5 s (a_o) ===\n', ns);
    fprintf('  measured law-leg covariance part      %+.4f\n', cov_meas);
    fprintf('  (1-lc) da''/dA P43  summed             %+.4f +- %.4f   (route C, from the filter''s own P)\n', mean(T1), std(T1)/sqrt(ns));
    fprintf('  1/2 a'''' M P44      summed             %+.4f +- %.4f\n', mean(T2), std(T2)/sqrt(ns));
    fprintf('  ratio predicted/measured               %.2f\n', mean(T1) / cov_meas);
    fprintf('  mean posterior P43 over the segment    %+.3e   P41 %+.3e   (sign: P43 < 0 => a kick toward the wall raises a_hat)\n', mean(P43m), mean(P41m));
    out = struct('cov_meas', cov_meas, 'T1', T1, 'T2', T2, 'P43m', P43m, 'P41m', P41m);
    fprintf('SECOND ORDER DONE\n');
end
