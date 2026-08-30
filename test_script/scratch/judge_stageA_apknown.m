% Judge Stage A against test_script/scratch/stageA_prereg.txt.
% Wiring checks W1-W4 are printed BEFORE any bias number, and a failure in
% W1/W2/W3 voids the run (project rule 13).
cd('/Users/kevin/Code/MotionControl_Simu-motion-test');

AX = 3;   % z only (x/y deferred by the user)
B = load('test_results/am_r22_deep/stageA_bmid_base_100.mat');
A = load('test_results/am_r22_deep/stageA_bmid_apknown_100.mat');

drop1 = @(M) M(2:end, :);
sq    = @(S, f) squeeze(S.(f)(:, AX, :));

tB = B.t(2:end);  lc = B.K.lambda_c;
i_hold = tB >= 3.75;                 % trough hold
i_desc = tB >= 0.5 & tB < 1.5;       % descent
i_osc  = tB >= 1.5 & tB < 3.5;       % oscillation

fprintf('\n================ WIRING CHECKS (read these first) ================\n');

% --- W4: init row must be all zeros in the RAW file --------------------
z0B = all(B.a_bar_hat_out(1, :, 1) == 0) && all(B.a_xm_out(1, :, 1) == 0);
z0A = all(A.a_bar_hat_out(1, :, 1) == 0) && all(A.a_xm_out(1, :, 1) == 0);
fprintf('W4 init row all-zero (base/apknown): %d / %d   [both must be 1]\n', z0B, z0A);

% --- W1: b must be pinned at 8/9 on BOTH arms --------------------------
bB = drop1(sq(B, 'b_hat_out'));  bA = drop1(sq(A, 'b_hat_out'));
fprintf('W1 max|b_hat - 8/9|  base %.3e   apknown %.3e   [both must be 0]\n', ...
        max(abs(bB(:) - 8/9)), max(abs(bA(:) - 8/9)));

% --- W3: plant untouched -> a_true identical between arms --------------
atB = drop1(sq(B, 'a_true_out'));  atA = drop1(sq(A, 'a_true_out'));
fprintf('W3 max|a_true_base - a_true_apknown| = %.3e   [must be 0: plant untouched]\n', ...
        max(abs(atB(:) - atA(:))));

% --- W2: did ap_known actually reach the controller? -------------------
% a_prime_out is the slope the controller INTEGRATED. On the apknown arm it
% must follow a_prime_true_out (previous step, driver:1128 uses hb_prev),
% NOT the law's b(1-a_hat)^2. Report both candidate scalings rather than
% assuming one.
apB  = drop1(sq(B, 'a_prime_out'));
apA  = drop1(sq(A, 'a_prime_out'));
aptA = drop1(sq(A, 'a_prime_true_out'));
ahB  = drop1(sq(B, 'a_bar_hat_out'));
ahA  = drop1(sq(A, 'a_bar_hat_out'));
law_pred_A = (8/9) * (1 - ahA).^2;                  % what the LAW would have given
for sc = [1, 1/A.a_nom]
    r = apA(2:end, :) ./ max(aptA(1:end-1, :) * sc, eps);
    fprintf('W2 apknown: median a_prime_out / (a_prime_true[k-1]*%.4g) = %.6f\n', ...
            sc, median(r(isfinite(r))));
end
r_law = apA ./ max(law_pred_A, eps);
fprintf('W2 apknown: median a_prime_out / law_prediction        = %.6f  [must NOT be 1]\n', ...
        median(r_law(isfinite(r_law))));
r_lawB = apB ./ max((8/9) * (1 - ahB).^2, eps);
fprintf('W2 base   : median a_prime_out / law_prediction        = %.6f  [must BE 1]\n', ...
        median(r_lawB(isfinite(r_lawB))));

fprintf('\n================ PRIMARY READOUT ================\n');
% bias = (a_hat - a_true_bar)/a_true_bar over the trough hold, per seed
atbB = atB / B.a_nom;  atbA = atA / A.a_nom;
biasB = mean((ahB(i_hold, :) - atbB(i_hold, :)) ./ atbB(i_hold, :), 1);
biasA = mean((ahA(i_hold, :) - atbA(i_hold, :)) ./ atbA(i_hold, :), 1);
d = biasA(:) - biasB(:);  n = numel(d);
tstat = mean(d) / (std(d) / sqrt(n));
removed = 1 - mean(biasA) / mean(biasB);
fprintf('trough-hold bias   baseline(bmid) %+7.3f%%  +- %.3f%% (sd)\n', ...
        100*mean(biasB), 100*std(biasB));
fprintf('trough-hold bias   ap_known       %+7.3f%%  +- %.3f%% (sd)\n', ...
        100*mean(biasA), 100*std(biasA));
fprintf('paired diff        %+7.4f%%  t = %+.2f  (n = %d)\n', 100*mean(d), tstat, n);
fprintf('FRACTION OF BIAS REMOVED: %.1f%%\n', 100*removed);
if removed >= 2/3
    verdict = 'P1  -> slope/aperture/A_a*M family DOMINATES (needs Stage B to split)';
elseif removed <= 1/3
    verdict = 'P2  -> family EXCLUDED in one run; target moves to P(4,1)/y1 rectification';
else
    verdict = 'P3  -> MIXED; no member attributable from this run';
end
fprintf('VERDICT: %s\n', verdict);

fprintf('\n================ P4: did F_e(4,3) actually move? ================\n');
fe43B = (1 - lc) * apB;  fe43A = (1 - lc) * apA;
for w = {{'far hold', tB < 0.5}, {'descend', i_desc}, {'oscillate', i_osc}, {'trough hold', i_hold}}
    m = w{1}{2};
    fprintf('  %-12s F_e(4,3) base %.5f   apknown %.5f   ratio %.3f\n', w{1}{1}, ...
            mean(mean(fe43B(m, :))), mean(mean(fe43A(m, :))), ...
            mean(mean(fe43A(m, :))) / mean(mean(fe43B(m, :))));
end

fprintf('\n================ P5: y1 leg split (trough hold) ================\n');
for arm = {{'base', B}, {'apknown', A}}
    S = arm{1}{2};
    K1 = drop1(sq(S, 'K_a_y1_out'));  I1 = drop1(sq(S, 'innov_y1_out'));
    k = K1(i_hold, :);  ii = I1(i_hold, :);
    tot = mean(sum(k .* ii, 1));
    sys = mean(sum(mean(k, 1) .* mean(ii, 1) * size(k, 1), 1));   % E[K]*E[i]*n
    sys = size(k, 1) * mean(mean(k, 1) .* mean(ii, 1));
    fprintf('  %-8s total %+.5f  systematic %+.5f  covariance %+.5f  (cov share %.1f%%)\n', ...
            arm{1}{1}, tot, sys, tot - sys, 100*abs(tot - sys)/max(abs(tot), eps));
end

fprintf('\n================ y2 share of a_hat motion ================\n');
for arm = {{'base', B}, {'apknown', A}}
    S = arm{1}{2};
    K1 = drop1(sq(S, 'K_a_y1_out'));  I1 = drop1(sq(S, 'innov_y1_out'));
    K2 = drop1(sq(S, 'K_a_y2_out'));  I2 = drop1(sq(S, 'innov_y2_out'));
    l1 = mean(sum(abs(K1 .* I1), 1));  l2 = mean(sum(abs(K2 .* I2), 1));
    fprintf('  %-8s |y1 leg| %.5f   |y2 leg| %.5f   y2 share %.2f%%\n', ...
            arm{1}{1}, l1, l2, 100*l2/(l1 + l2));
end
fprintf('\n');
