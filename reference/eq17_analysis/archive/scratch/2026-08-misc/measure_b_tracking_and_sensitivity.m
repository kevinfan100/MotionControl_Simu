% Provenance run for two numbers I quoted without a source (flagged [未核] by
% the audit ledger): "b_hat tracks 26% of b_true" and "trough sensitivity
% ~200% per 1% of b". Neither had a script behind it, so this measures the
% first and derives the second with its assumptions stated, rather than
% defending them.
%
% b_true(w_bar) is DEFINED by inverting the law against the truth:
%     a_true(w) = 1/c_perp(w)          (normalized truth)
%     a_true'(w) = b_true(w) * (1 - a_true(w))^2
%  => b_true(w) = a_true'(w) / (1 - a_true(w))^2
% with a_true' taken by central difference on calc_correction_functions.
cd('/Users/kevin/Code/MotionControl_Simu-motion-test');
addpath(genpath('test_script')); addpath(genpath('model'));

% ---------- b_true(w_bar) from the truth curve -------------------------
wq = linspace(1.05, 24, 20001);
cp = zeros(size(wq));
for i = 1:numel(wq)
    [~, cperp] = calc_correction_functions(wq(i));
    cp(i) = cperp;
end
a_tr = 1 ./ cp;
ap   = gradient(a_tr, wq);
b_tr = ap ./ (1 - a_tr).^2;
b_of_w = @(w) interp1(wq, b_tr, w, 'linear', 'extrap');

fprintf('\n== b_true(w) from the truth curve ==\n');
for w = [1.10, 1.5, 2.0, 3.32, 10, 22.2]
    fprintf('   w_bar %6.2f : b_true = %.4f\n', w, b_of_w(w));
end
fprintf('   far-field anchor 8/9 = %.4f ; contact anchor 1\n', 8/9);

% ---------- (1) does b_hat track b_true? -------------------------------
clear run_formC_b motion_control_law_formC_b;
D = run_formC_b(struct('seeds', 1:10, 'arm', 'best'));   % b FREE = production
AX = 3;  ns = numel(D.runs);
t  = D.runs{1}.tout(2:end);
imov = t >= 0.5 & t < 3.5;          % descent + oscillation (b is unobservable in holds)
slope = zeros(1, ns);  exc_hat = zeros(1, ns);  exc_true = zeros(1, ns);
for s = 1:ns
    r  = D.runs{s};
    bh = r.b_hat_out(2:end, AX);
    bt = b_of_w(r.h_bar_true_out(2:end, 1));
    x  = bt(imov);  y = bh(imov);
    ok = isfinite(x) & isfinite(y);
    p  = polyfit(x(ok), y(ok), 1);
    slope(s)    = p(1);                       % 1.0 = perfect tracking, 0 = frozen
    exc_hat(s)  = max(y(ok)) - min(y(ok));
    exc_true(s) = max(x(ok)) - min(x(ok));
end
fprintf('\n== (1) b_hat vs b_true, arm=best, 10 seeds, descent+oscillation ==\n');
fprintf('   regression slope d(b_hat)/d(b_true) = %.3f +- %.3f  (1 = tracks, 0 = frozen)\n', ...
        mean(slope), std(slope));
fprintf('   excursion b_hat  = %.4f +- %.4f\n', mean(exc_hat), std(exc_hat));
fprintf('   excursion b_true = %.4f\n', mean(exc_true));
fprintf('   excursion ratio  = %.3f\n', mean(exc_hat)/mean(exc_true));

% ---------- (2) trough sensitivity to b, assumptions stated ------------
% The estimator does NOT re-anchor a_bar from the law: it seeds once and then
% integrates a_bar' = b(1-a_bar)^2 forward. Its closed form is
%     1/(1-a_bar(w)) = 1/(1-a_bar_0) + b*(w - w0)
% and the SEED (controller:163) carries NO b:  a_bar_0 = 1 - 1/(w0 - w0_hat).
w0 = 22.2;  wt = 1.10;  b0 = 8/9;
u0 = w0;                                   % 1/(1-a_bar_0) with a_bar_0 = 1-1/w0
lawA = @(b) 1 - 1 ./ (u0 + b*(wt - w0));   % accumulated, seed fixed (what the code does)
lawB = @(b) 1 - 1 ./ (b*wt);               % closed form re-anchored at every w
a_true_trough = 1 / (1 / b_of_w(wt) * 0 + 1);  %#ok<NASGU>
[~, cp_t] = calc_correction_functions(wt);  a_tr_t = 1/cp_t;
fprintf('\n== (2) trough sensitivity to b (w_bar = %.2f, truth a_bar = %.5f) ==\n', wt, a_tr_t);
for nm = {{'accumulated, fixed seed (the code)', lawA}, {'re-anchored closed form', lawB}}
    L = nm{1}{2};
    a0 = L(b0);  a1 = L(b0*1.01);
    fprintf('   %-36s a_bar(b) %+9.5f -> a_bar(1.01b) %+9.5f\n', nm{1}{1}, a0, a1);
    fprintf('   %-36s change %+8.5f = %+8.1f%% of the TRUE a_bar per 1%% of b\n', ...
            '', a1 - a0, 100*(a1 - a0)/a_tr_t);
end
fprintf(['\n   NOTE: the two writings disagree by orders of magnitude, so any\n' ...
         '   "%% per 1%% of b" figure is meaningless without saying which one.\n' ...
         '   The code integrates forward from a b-free seed => the first row.\n\n']);
