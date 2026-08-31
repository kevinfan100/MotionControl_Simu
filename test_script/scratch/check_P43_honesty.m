function out = check_P43_honesty(seeds)
%CHECK_P43_HONESTY  Is the filter's cross-covariance P43 (gain x tracking
%   error) the covariance that actually exists between its two errors?
%
%   e_a[k] = a_hat[k] - a_true[k]           (a_bar units, posterior)
%   e_3[k] = x3_hat[k] - dw_true[k]         (R units, posterior)
%   honesty(P43) = < Cov_seeds(e_a, e_3) >_k / < P43 >_k   over the oscillation
%   Also P44 and P33 honesty (the cross term cannot be honest if both diagonals
%   are not).  Instrument check first: the sign/lag convention of x3 is found
%   by correlating x3_hat with +-(h_true - h_d) at lags -1..1; the winner must
%   have |corr| > 0.9 or the script refuses to report honesty.
%   Arm: b_true oracle + exact step, canonical, noisy, log_P_full.
% STATUS: ACTIVE | step 1 of the remaining-bias plan (2026-08-30)

    if nargin < 1 || isempty(seeds); seeds = 1:8; end
    ax = 3;  R = physical_constants().R;
    O = run_formC_b(struct('arm', 'best', 'ap_src', 'post', 'seeds', seeds, 'verbose', false, ...
                           'b_true', true, 'b_true_at', 'cmd', 'log_P_full', true, ...
                           'ctrl_const_override', struct('law_exact_step', true)));
    ns = numel(O.runs);  t = O.runs{1}.tout(:);  a_nom = O.runs{1}.a_nom;  N = numel(t);
    EA = zeros(N, ns);  E3 = zeros(N, ns);  P43 = zeros(N, ns);  P44 = zeros(N, ns);  P33 = zeros(N, ns);
    X3 = zeros(N, ns);  DWT = zeros(N, ns);
    for q = 1:ns
        r = O.runs{q};
        EA(:, q)  = r.a_bar_hat_out(:, ax) - r.a_true_out(:, ax) / a_nom;
        X3(:, q)  = r.delta_x_hat_3_out(:, ax) / R;
        DWT(:, q) = r.h_bar_true_out(:) - r.h_bar_d_out(:);       % true tracking error, R, sign TBD
        P = r.P_full_out;
        P43(:, q) = squeeze(P(:, 4, 3, ax));  P44(:, q) = squeeze(P(:, 4, 4, ax));  P33(:, q) = squeeze(P(:, 3, 3, ax));
    end
    % ---- instrument check: sign and lag of x3 vs the true tracking error --------
    m = t >= 1.5 & t < 3.5;  best = struct('c', 0, 'lag', NaN, 'sgn', NaN);
    for lag = -2:2
        for sgn = [-1 1]
            a = X3(m, :);  b = sgn * circshift(DWT, lag);  b = b(m, :);
            c = corr(a(:), b(:));
            if c > best.c; best = struct('c', c, 'lag', lag, 'sgn', sgn); end
        end
    end
    fprintf('\n[instrument] x3_hat vs %+d*(h_true - h_d) shifted %+d steps: corr %.4f\n', best.sgn, best.lag, best.c);
    assert(best.c > 0.9, 'x3 convention not recovered (corr %.3f): refuse to report honesty', best.c);
    E3 = X3 - best.sgn * circshift(DWT, best.lag);
    % ---- honesty over the oscillation (per-step cross-seed moments, averaged) ----
    ea = EA(m, :) - mean(EA(m, :), 2);  e3 = E3(m, :) - mean(E3(m, :), 2);
    cov43 = mean(sum(ea .* e3, 2) / (ns - 1));   var4 = mean(sum(ea.^2, 2) / (ns - 1));   var3 = mean(sum(e3.^2, 2) / (ns - 1));
    p43 = mean(P43(m, :), 'all');  p44 = mean(P44(m, :), 'all');  p33 = mean(P33(m, :), 'all');
    % seed-group error bar on the cross term (4 groups of 2 are too small for a
    % covariance; use leave-one-seed-out jackknife instead)
    jk = zeros(ns, 1);
    for q = 1:ns
        sel = setdiff(1:ns, q);  a = EA(m, sel) - mean(EA(m, sel), 2);  b = E3(m, sel) - mean(E3(m, sel), 2);
        jk(q) = mean(sum(a .* b, 2) / (ns - 2)) / p43;
    end
    se = sqrt((ns - 1) / ns * sum((jk - mean(jk)).^2));
    fprintf('[P43] realised Cov(e_a, e_3) %+.3e   filter P43 %+.3e   honesty %.2f +- %.2f (jackknife)\n', cov43, p43, cov43 / p43, se);
    fprintf('[P44] realised Var(e_a)      %.3e   filter P44 %.3e   honesty %.2f   (bias of e_a over segment %+.4f)\n', var4, p44, var4 / p44, mean(EA(m, :), 'all'));
    fprintf('[P33] realised Var(e_3)      %.3e   filter P33 %.3e   honesty %.2f\n', var3, p33, var3 / p33);
    fprintf('[corr] realised corr(e_a, e_3) %+.3f   filter P43/sqrt(P33 P44) %+.3f\n', cov43 / sqrt(var3 * var4), p43 / sqrt(p33 * p44));
    % ---- the second-order term with the REALISED covariance instead of P43 -----
    kk = 2:N;  mm = m(kk);
    bh = cell2mat(cellfun(@(r) r.b_hat_out(:, ax), O.runs.', 'UniformOutput', false));
    ah = cell2mat(cellfun(@(r) r.a_bar_hat_out(:, ax), O.runs.', 'UniformOutput', false));
    dap = -2 * bh(kk-1, :) .* (1 - ah(kk-1, :));
    c_k = sum(ea .* e3, 2) / (ns - 1);                       % per-step realised cov (segment only)
    T_real = 0.3 * sum(mean(dap(mm, :), 2) .* c_k);
    T_filt = 0.3 * mean(sum(dap(mm, :) .* P43(kk(mm)-1, :), 1));
    fprintf('[2nd order] (1-lc) da''/dA x Cov summed: with realised cov %+.4f   with filter P43 %+.4f   (measured law-leg cov part +0.0068)\n', T_real, T_filt);
    out = struct('cov43', cov43, 'p43', p43, 'h43', cov43 / p43, 'se43', se, 'h44', var4 / p44, 'h33', var3 / p33, ...
                 'T_real', T_real, 'T_filt', T_filt, 'conv', best);
    fprintf('P43 HONESTY DONE\n');
end
