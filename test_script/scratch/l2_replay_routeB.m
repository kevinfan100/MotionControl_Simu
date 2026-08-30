% L2 predict replay -- ROUTE B (rerun with h_bar_d_out, delta_x_hat_3_out, h_bar_true_out,
% a_bar_Q_out, P_a_out, K_b_y1/y2_out), 2 x 100 seeds, z axis.
% M_row4 reconstructed from the logs (division-free, holds included); the closure
% residual r = a'*M_row4 - predict measures the unlogged MA(2) term.
% Registration: test_script/scratch/l2_replay_prereg.txt
cd('/Users/kevin/code/MotionControl_Simu-motion-test');
addpath(genpath('test_script')); addpath(genpath('model'));

AX = 3;  B_LAW = 8/9;  AP_MIN = 1e-3;  PRED_MIN = 1e-12;
W = struct('far', [0, 0.5], 'desc', [0.5, 1.5], 'tr1', [1.5, 1.6], 'osc', [1.6, 3.5], ...
           'descosc', [0.5, 3.5], 'tr2', [3.5, 3.8], 'trough', [3.8, 4.8], 'all', [0, 4.8]);
wn_rules  = {'far', 'desc', 'osc', 'descosc', 'trough'};
wn_budget = {'far', 'desc', 'tr1', 'osc', 'tr2', 'trough', 'all'};
files = {'base',    'l2_bmid_base_100.mat',    'stageA_bmid_base_100.mat'; ...
         'apknown', 'l2_bmid_apknown_100.mat', 'stageA_bmid_apknown_100.mat'};

% true-slope interpolant a'_true(h) = d(1/c_perp)/dh = -dc_perp_dh / c_perp^2 (grid + pchip)
hg = linspace(1.02, 30, 6000).';  apg = zeros(size(hg));
for i = 1:numel(hg)
    [~, cp, dv] = calc_correction_functions(hg(i), true);  apg(i) = -dv.dc_perp_dh / cp^2;
end
ap_true_fun = @(h) interp1(hg, apg, h, 'pchip');

RES = struct();
for a = 1:size(files, 1)
    arm = files{a, 1};
    S = load(fullfile('test_results/am_r22_deep', files{a, 2}));
    t = S.t(:);  N = numel(t);  lc = S.K.lambda_c;  al = 1 - lc;  a_nom = S.a_nom;  R_um = S.R_um;
    ns = size(S.a_bar_hat_out, 3);
    fprintf('\n================ ROUTE B : arm %s (N=%d, seeds=%d, R=%.3f um, lambda_c=%.2f) ================\n', ...
            arm, N, ns, R_um, lc);
    % ---- bit-identity of the rerun vs Stage A, all seeds ----
    mref = matfile(fullfile('test_results/am_r22_deep', files{a, 3}));
    dref = max(abs(S.a_bar_hat_out(:) - reshape(mref.a_bar_hat_out, [], 1)));
    fprintf('  rerun vs Stage A: max|a_bar_hat_out diff| over 100 seeds = %.3e  [must be 0]\n', dref);

    ah  = squeeze(S.a_bar_hat_out(:, AX, :));
    K1  = squeeze(S.K_a_y1_out(:, AX, :));  I1 = squeeze(S.innov_y1_out(:, AX, :));
    K2  = squeeze(S.K_a_y2_out(:, AX, :));  I2 = squeeze(S.innov_y2_out(:, AX, :));
    ap  = squeeze(S.a_prime_out(:, AX, :)) / a_nom;
    apt = squeeze(S.a_prime_true_out(:, AX, :)) / a_nom;
    at  = squeeze(S.a_true_out(:, AX, :)) / a_nom;
    abQ = squeeze(S.a_bar_Q_out(:, AX, :));
    dx3 = squeeze(S.delta_x_hat_3_out(:, AX, :));
    hd  = S.h_bar_d_out;  ht = S.h_bar_true_out;

    pred = [nan(1, ns); ah(2:end, :) - ah(1:end-1, :) - K1(2:end, :) .* I1(2:end, :) - K2(2:end, :) .* I2(2:end, :)];
    corr_leg = K1 .* I1 + K2 .* I2;  corr_leg(1, :) = 0;
    dwd_prev = [nan(1, ns); hd(2:end, :) - hd(1:end-1, :)];   % Delta_wbar_d[k-1]
    dwd_prev(2, :) = 0;                                       % init call: Delta_wbar_d_km1 = 0 (controller 813); h_bar_d_out(1) = 0 is the init-row default
    dw3_prev = [nan(1, ns); dx3(1:end-1, :)] / R_um;          % dw3_hat[k-1]
    M4 = dwd_prev + al * dw3_prev;                            % M_row4[k]
    r  = ap .* M4 - pred;                                     % = -a'*al*(m1+m2)[k-1]
    in_T = t >= W.trough(1) & t <= W.trough(2);
    aT = mean(at(in_T, :), 1);
    bias = mean((ah(in_T, :) - at(in_T, :)) ./ at(in_T, :), 1);
    fprintf('  trough-hold bias: %s %%\n', fmt3(100 * bias));

    % ---- (d) closure residual = unlogged MA(2) term ----
    fprintf('\n  (d) closure residual r = a''*M_row4 - predict  [%% of a_T, cumulative per window]\n');
    fprintf('      window    sum r/aT  mean      sd      SEM   | sum|r|/aT | max|r|/max|pred| | rms(r)/rms(pred)\n');
    for w = 1:numel(wn_budget)
        in = t >= W.(wn_budget{w})(1) & t <= W.(wn_budget{w})(2);  in(1) = false;
        sr = 100 * sum(r(in, :), 1) ./ aT;  sa = 100 * sum(abs(r(in, :)), 1) ./ aT;
        fprintf('      %-8s  %+9.4f  %7.4f  %6.4f  | %8.4f  | %8.2e | %8.2e\n', wn_budget{w}, mean(sr), std(sr), std(sr)/sqrt(ns), ...
                mean(sa), max(abs(r(in, :)), [], 'all') / max(abs(pred(in, :)), [], 'all'), rms(r(in, :), 'all') / rms(pred(in, :), 'all'));
    end
    sr_all = 100 * sum(r(2:end, :), 1) ./ aT;
    fprintf('      VERDICT: |mean cumulative r| over the whole run = %.4f%% of a_T -> %s (threshold 0.2 pp = 1%% of the bias)\n', ...
            abs(mean(sr_all)), tern(abs(mean(sr_all)) < 0.2, 'NEGLIGIBLE', 'NOT negligible'));

    % ---- (f) a' rebuilt from a_bar_Q ----
    apQ = B_LAW * (1 - abQ).^2;
    dQ = abs(apQ(2:end, :) - ap(2:end, :));
    fprintf('\n  (f) a'' from a_bar_Q: (8/9)(1-a_bar_Q[k])^2 vs a_prime_out[k]/a_nom: max|dev| %.3e, median ratio %.6f\n', ...
            max(dQ(:)), median(apQ(2:end, :) ./ ap(2:end, :), 'all'));
    fprintf('      a_bar_Q[k] vs a_bar_hat[k-1]: max|dev| %.3e (clamp events would show here)\n', ...
            max(abs(abQ(2:end, :) - ah(1:end-1, :)), [], 'all'));
    if strcmp(arm, 'apknown')
        for w = {'desc', 'osc', 'trough'}
            in = t >= W.(w{1})(1) & t <= W.(w{1})(2);
            fprintf('      %-7s law-at-estimate / true slope along the path: mean ratio %.4f\n', w{1}, mean(apQ(in, :) ./ ap(in, :), 'all'));
        end
    end

    % ---- three rules on M_row4 (division-free, holds included) ----
    ahL = [nan(1, ns); ah(1:end-1, :)];  apL = [nan(1, ns); ap(1:end-1, :)];  apR = [ap(2:end, :); nan(1, ns)];
    htL = [nan(1, ns); ht(1:end-1, :)];
    left = ap .* M4;
    if strcmp(arm, 'base')
        trap = 0.5 * (ap + apR) .* M4;
        mid  = B_LAW * (1 - (ahL + 0.5 * ap .* M4)).^2 .* M4;
        othr = apR .* M4;      % OTHER endpoint (right) -- negative control requested by team lead
    else
        trap = 0.5 * (apL + ap) .* M4;
        othr = apL .* M4;      % OTHER endpoint (left: true slope one step earlier)
        hmid = 0.5 * (htL + ht);  hmid = max(hmid, 1.02);
        mid  = ap_true_fun(hmid) .* M4;
    end
    % interpolant check vs the driver's central-difference slope at h_true[k]
    chk = ap_true_fun(max(ht(2:end, :), 1.02)) ./ apt(2:end, :) - 1;
    fprintf('\n  a''_true interpolant vs a_prime_true_out at h_true[k]: max|rel dev| %.2e\n', max(abs(chk(:))));

    % exact-M rules on the Route A mask (agreement check)
    msk = abs(ap) >= AP_MIN & abs(pred) >= PRED_MIN;  msk(1, :) = false;  msk(end, :) = false;
    Mx = nan(N, ns);  Mx(msk) = pred(msk) ./ ap(msk);
    leftx = ap .* Mx;
    if strcmp(arm, 'base'); trapx = 0.5 * (ap + apR) .* Mx; else; trapx = 0.5 * (apL + ap) .* Mx; end

    fprintf('\n  window    rule            S/a_T [%%]  mean       sd      SEM\n');
    for w = 1:numel(wn_rules)
        in = t >= W.(wn_rules{w})(1) & t <= W.(wn_rules{w})(2);  in(1) = false;  in(end) = false;   % last row: a'[k+1] undefined
        Sl = sum(left(in, :), 1) ./ aT;  St = sum(trap(in, :), 1) ./ aT;  Sm = sum(mid(in, :), 1) ./ aT;
        Sp = sum(pred(in, :), 1) ./ aT;  So = sum(othr(in, :), 1) ./ aT;
        mm = in & msk;
        Slx = nansum_masked(leftx, mm) ./ aT;  Stx = nansum_masked(trapx, mm) ./ aT;
        rows = {'left(M_row4)', Sl; 'trap(M_row4)', St; 'mid(M_row4)', Sm; 'D_trap=L-T', Sl - St; 'D_mid=L-M', Sl - Sm; ...
                'otherEnd(M_row4)', So; 'D_other=O-T', So - St; ...
                'predict', Sp; 'left(M_exact)', Slx; 'trap(M_exact)', Stx; 'D_trap(M_exact)', Slx - Stx};
        for q = 1:size(rows, 1)
            v = 100 * rows{q, 2};
            fprintf('  %-8s  %-15s  %+10.3f  %8.3f  %7.3f\n', wn_rules{w}, rows{q, 1}, mean(v), std(v), std(v)/sqrt(ns));
        end
        RES.(arm).win.(wn_rules{w}) = struct('S_left', Sl, 'S_trap', St, 'S_mid', Sm, 'S_pred', Sp, 'S_leftx', Slx, 'S_trapx', Stx, 'S_other', So);
    end

    % ---- true-path integrals (the colleague's object) ----
    dht = [nan(1, ns); ht(2:end, :) - ht(1:end-1, :)];
    aptL = [nan(1, ns); apt(1:end-1, :)];
    tp_right = apt .* dht;  tp_left = aptL .* dht;  tp_trap = 0.5 * (aptL + apt) .* dht;
    dat = [nan(1, ns); at(2:end, :) - at(1:end-1, :)];
    fprintf('\n  TRUE PATH (a''_true x Delta h_true), sum over window / a_T [%%]: right-endpoint | left-endpoint | trapezoid | Delta a_true\n');
    for w = {'desc', 'osc', 'descosc'}
        in = t >= W.(w{1})(1) & t <= W.(w{1})(2);  in(1) = false;
        v = [sum(tp_right(in, :), 1); sum(tp_left(in, :), 1); sum(tp_trap(in, :), 1); sum(dat(in, :), 1)] * 100 ./ aT;
        fprintf('    %-8s %+9.3f (sd %.3f) | %+9.3f (sd %.3f) | %+9.3f (sd %.3f) | %+9.3f (sd %.3f)\n', w{1}, ...
                mean(v(1, :)), std(v(1, :)), mean(v(2, :)), std(v(2, :)), mean(v(3, :)), std(v(3, :)), mean(v(4, :)), std(v(4, :)));
    end

    % ---- bias budget per window: Delta a_hat - Delta a_true = Q + (T - Tt) + (Tt - dA_true) + Rm + corr ----
    fprintf('\n  BIAS BUDGET [%% of a_T], per window, mean (sd):  dA_hat-dA_true = Q(left-trap on M) + Smis(trap on M - trap on true path) + (trap_true - dA_true) + Rm(-sum r) + corr\n');
    fprintf('      window   dA_hat-dA_true |    Q      |   Smis    | trapT-dAt |    Rm     |   corr    | check\n');
    off0 = 100 * (ah(1, :) - at(1, :)) ./ aT;
    fprintf('      init offset (seed - a_true[1]) = %+.3f (sd %.3f)\n', mean(off0), std(off0));
    tot = zeros(1, ns);
    for w = 1:numel(wn_budget)
        in = t >= W.(wn_budget{w})(1) & t <= W.(wn_budget{w})(2);  in(1) = false;  in(end) = false;
        k0 = find(in, 1);  k1 = find(in, 1, 'last');
        dAh = (ah(k1, :) - ah(k0 - 1, :)) ./ aT;  dAt = (at(k1, :) - at(k0 - 1, :)) ./ aT;
        Q  = sum(left(in, :) - trap(in, :), 1) ./ aT;
        Sm = sum(trap(in, :) - tp_trap(in, :), 1) ./ aT;
        Tt = sum(tp_trap(in, :), 1) ./ aT - dAt;
        Rm = -sum(r(in, :), 1) ./ aT;
        Cc = sum(corr_leg(in, :), 1) ./ aT;
        chk_b = (dAh - dAt) - (Q + Sm + Tt + Rm + Cc);
        fprintf('      %-7s  %+8.3f (%6.3f) | %+8.3f (%5.3f) | %+8.3f (%5.3f) | %+8.3f (%5.3f) | %+8.3f (%5.3f) | %+8.3f (%6.3f) | %.1e\n', ...
                wn_budget{w}, 100*mean(dAh - dAt), 100*std(dAh - dAt), 100*mean(Q), 100*std(Q), 100*mean(Sm), 100*std(Sm), ...
                100*mean(Tt), 100*std(Tt), 100*mean(Rm), 100*std(Rm), 100*mean(Cc), 100*std(Cc), max(abs(chk_b)));
        RES.(arm).budget.(wn_budget{w}) = struct('dAh_dAt', dAh - dAt, 'Q', Q, 'Smis', Sm, 'Tt', Tt, 'Rm', Rm, 'corr', Cc);
    end
    endbias = 100 * (ah(end, :) - at(end, :)) ./ aT;
    fprintf('      end-of-run (a_hat-a_true)/a_T = %+.3f (sd %.3f) = init offset + ''all'' row; trough-mean bias = %+.3f (sd %.3f)\n', ...
            mean(endbias), std(endbias), 100*mean(bias), 100*std(bias));
    Dall = RES.(arm).win.descosc.S_left - RES.(arm).win.descosc.S_trap;
    resid = bias - Dall;
    fprintf('      RESIDUAL bias - D_trap(desc+osc) = %+.3f (sd %.3f, SEM %.3f) %% of a_T\n', 100*mean(resid), 100*std(resid), 100*std(resid)/sqrt(ns));
    RES.(arm).resid = resid;

    RES.(arm).t = t;  RES.(arm).aT = aT;  RES.(arm).bias = bias;
    RES.(arm).ah = ah;  RES.(arm).at = at;
    RES.(arm).cum_left = cumsum(fillz(pred), 1);
    RES.(arm).cum_trap = cumsum(fillz(pred + (trap - left)), 1);
    RES.(arm).cum_mid  = cumsum(fillz(pred + (mid - left)), 1);
    RES.(arm).step_LmT = fillz(left - trap);
    RES.(arm).bitdiff_vs_stageA = dref;
end
save('test_results/am_r22_deep/l2_routeB_results.mat', '-struct', 'RES', '-v7.3');
fprintf('\nsaved test_results/am_r22_deep/l2_routeB_results.mat\n');

function s = nansum_masked(X, m)
    X(~m) = 0;  s = sum(X, 1);
end
function X = fillz(X)
    X(~isfinite(X)) = 0;
end
function s = fmt3(v)
    s = sprintf('%+8.3f +- %6.3f (SEM %5.3f)', mean(v), std(v), std(v)/sqrt(numel(v)));
end
function s = tern(c, a, b)
    if c; s = a; else; s = b; end
end
