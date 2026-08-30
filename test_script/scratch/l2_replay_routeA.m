% L2 predict replay -- ROUTE A (no rerun). Pre-registration:
% test_script/scratch/l2_replay_prereg.txt (written before this script).
%
% Reconstructs the per-step gain predict from the Kalman identity on the
% Stage A files, divides by the logged slope to recover M_tot, and re-integrates
% the SAME M_tot under left-endpoint (= the code), trapezoid and midpoint rules.
% Holds excluded (0/0). Every mean carries sd and SEM; seed is the resampling unit.
cd('/Users/kevin/code/MotionControl_Simu-motion-test');
addpath(genpath('test_script')); addpath(genpath('model'));

OUT_MAT = 'test_results/am_r22_deep/l2_routeA_results.mat';
files = {'base',    'test_results/am_r22_deep/stageA_bmid_base_100.mat'; ...
         'apknown', 'test_results/am_r22_deep/stageA_bmid_apknown_100.mat'};
B_LAW = 8/9;
AP_MIN = 1e-3;  PRED_MIN = 1e-12;
W = struct('desc', [0.5, 1.5], 'osc', [1.6, 3.5], 'descosc', [0.5, 3.5], 'trough', [3.8, 4.8]);
wn = {'desc', 'osc', 'descosc'};

RES = struct();
for a = 1:size(files, 1)
    arm = files{a, 1};
    S = load(files{a, 2});
    t = S.t(:);  N = numel(t);  lc = S.K.lambda_c;  a_nom = S.a_nom;  ns = size(S.a_bar_hat_out, 3);
    fprintf('\n================ ROUTE A : arm %s  (N=%d, seeds=%d, lambda_c=%.3f, a_nom=%.6g) ================\n', ...
            arm, N, ns, lc, a_nom);
    assert(all(all(S.a_xm_out(1, :, :) == 0)), 'row 1 is not the init call');
    for AX = 3   % z (x/y appended below for the base arm only)
        ah  = squeeze(S.a_bar_hat_out(:, AX, :));
        K1  = squeeze(S.K_a_y1_out(:, AX, :));  I1 = squeeze(S.innov_y1_out(:, AX, :));
        K2  = squeeze(S.K_a_y2_out(:, AX, :));  I2 = squeeze(S.innov_y2_out(:, AX, :));
        ap  = squeeze(S.a_prime_out(:, AX, :)) / a_nom;
        apt = squeeze(S.a_prime_true_out(:, AX, :)) / a_nom;
        at  = squeeze(S.a_true_out(:, AX, :)) / a_nom;
        R = local_replay(t, ah, K1, I1, K2, I2, ap, apt, at, arm, B_LAW, AP_MIN, PRED_MIN, W, wn, a);
        RES.(arm) = R;
    end
    if strcmp(arm, 'base')
        for AX = 1:2
            fprintf('\n---- base arm, axis %d (x/y, left vs trapezoid only) ----\n', AX);
            ah  = squeeze(S.a_bar_hat_out(:, AX, :));
            K1  = squeeze(S.K_a_y1_out(:, AX, :));  I1 = squeeze(S.innov_y1_out(:, AX, :));
            K2  = squeeze(S.K_a_y2_out(:, AX, :));  I2 = squeeze(S.innov_y2_out(:, AX, :));
            ap  = squeeze(S.a_prime_out(:, AX, :)) / a_nom;
            at  = squeeze(S.a_true_out(:, AX, :)) / a_nom;
            pred = ah(2:end, :) - ah(1:end-1, :) - K1(2:end, :) .* I1(2:end, :) - K2(2:end, :) .* I2(2:end, :);
            pred = [nan(1, ns); pred];
            aT = mean(at(t >= W.trough(1) & t <= W.trough(2), :), 1);
            msk = abs(ap) >= AP_MIN & abs(pred) >= PRED_MIN;  msk(1, :) = false;  msk(end, :) = false;
            M = nan(N, ns);  M(msk) = pred(msk) ./ ap(msk);
            apR = [ap(2:end, :); nan(1, ns)];
            left = ap .* M;  trap = 0.5 * (ap + apR) .* M;
            for w = 1:numel(wn)
                in = t >= W.(wn{w})(1) & t <= W.(wn{w})(2);
                sl = nansum_masked(left, in & msk) ./ aT;  st = nansum_masked(trap, in & msk) ./ aT;
                fprintf('  %-8s S_left %s   S_trap %s   D_trap=(L-T) %s   excl %.2f%%\n', wn{w}, ...
                        fmt3(100*sl), fmt3(100*st), fmt3(100*(sl - st)), 100*mean(mean(~msk(in, :))));
            end
        end
    end
end
save(OUT_MAT, '-struct', 'RES');
fprintf('\nsaved %s\n', OUT_MAT);

% ======================================================================
function R = local_replay(t, ah, K1, I1, K2, I2, ap, apt, at, arm, B_LAW, AP_MIN, PRED_MIN, W, wn, a_idx)
    [N, ns] = size(ah);
    % ---- Kalman identity: predict[k] feeds posterior k, k >= 2 ----
    pred = ah(2:end, :) - ah(1:end-1, :) - K1(2:end, :) .* I1(2:end, :) - K2(2:end, :) .* I2(2:end, :);
    pred = [nan(1, ns); pred];
    corr_leg = K1 .* I1 + K2 .* I2;  corr_leg(1, :) = 0;
    ahL = [nan(1, ns); ah(1:end-1, :)];            % a_bar_hat[k-1]
    apL = [nan(1, ns); ap(1:end-1, :)];            % a'[k-1]
    apR = [ap(2:end, :); nan(1, ns)];              % a'[k+1]
    dat = [nan(1, ns); at(2:end, :) - at(1:end-1, :)];   % Delta a_true_n[k]

    % ---- pairing checks (which endpoint does a'[k] sit on?) ----
    fprintf('\n-- pairing checks (arm %s) --\n', arm);
    law_prev = B_LAW * (1 - ahL).^2;
    dev = abs(ap(2:end, :) - law_prev(2:end, :));
    fprintf('  a''[k] vs (8/9)(1-a_hat[k-1])^2 : max|dev| %.3e  median ratio %.6f\n', ...
            max(dev(:)), median(ap(2:end, :) ./ law_prev(2:end, :), 'all'));
    fprintf('  a''[k] vs a''_true[k]           : max|dev| %.3e  ; vs a''_true[k-1]: max|dev| %.3e\n', ...
            max(abs(ap(2:end, :) - apt(2:end, :)), [], 'all'), max(abs(ap(2:end, :) - apt(1:end-1, :)), [], 'all'));
    % lagged correlation of predict[k] with Delta a_true_n[k+l] over descend
    in_d = t >= W.desc(1) & t <= W.desc(2);
    fprintf('  alignment (descend): corr(predict[k], Delta a_true[k+l])  l=-2..2 : ');
    for l = -2:2
        c = zeros(1, ns);
        for s = 1:ns
            kk = find(in_d);  kk = kk(kk + l >= 2 & kk + l <= N);
            cc = corrcoef(pred(kk, s), dat(kk + l, s));  c(s) = cc(1, 2);
        end
        fprintf('%+d:%.4f  ', l, mean(c));
    end
    fprintf('\n');

    % ---- trough truth and the Stage A bias (must reproduce) ----
    in_T = t >= W.trough(1) & t <= W.trough(2);
    aT = mean(at(in_T, :), 1);
    bias = mean((ah(in_T, :) - at(in_T, :)) ./ at(in_T, :), 1);
    fprintf('  trough-hold bias (reproduce Stage A): %s %%\n', fmt3(100*bias));

    % ---- Route A division ----
    msk = abs(ap) >= AP_MIN & abs(pred) >= PRED_MIN;  msk(1, :) = false;  msk(end, :) = false;
    M = nan(N, ns);  M(msk) = pred(msk) ./ ap(msk);
    left = ap .* M;
    if strcmp(arm, 'base')
        trap = 0.5 * (ap + apR) .* M;                          % a'[k] left, a'[k+1] right
        mid  = B_LAW * (1 - (ahL + 0.5 * ap .* M)).^2 .* M;    % law at the half-step
        trap_shift = nan(N, ns);
    else
        trap = 0.5 * (apL + ap) .* M;                          % a'[k-1] left, a'[k] right
        mid  = trap;                                           % time-linear interp == trapezoid (registered)
        trap_shift = 0.5 * (ap + apR) .* M;                    % task's literal pairing (one-step shifted)
    end
    closure = max(abs(left(msk) - pred(msk)));
    fprintf('  closure |a''*M - predict| on mask: max %.3e (division: roundoff by construction)\n', closure);

    fprintf('\n  window    rule        S/a_T [%%]  mean     sd      SEM     | excl%%\n');
    R = struct('t', t, 'pred', pred, 'M', M, 'msk', msk, 'left', left, 'trap', trap, 'mid', mid, ...
               'trap_shift', trap_shift, 'aT', aT, 'bias', bias, 'ah', ah, 'at', at, 'corr', corr_leg, 'ap', ap);
    for w = 1:numel(wn)
        in = t >= W.(wn{w})(1) & t <= W.(wn{w})(2);
        mm = in & msk;
        excl = 100 * mean(mean(~msk(in, :)));
        Sl = nansum_masked(left, mm) ./ aT;   St = nansum_masked(trap, mm) ./ aT;
        Sm = nansum_masked(mid,  mm) ./ aT;   Ss = nansum_masked(trap_shift, mm) ./ aT;
        Sp_all = sum(pred(in, :), 1) ./ aT;   % all samples incl. masked (what the mask drops)
        Sc = sum(corr_leg(in, :), 1) ./ aT;
        dah = (ah(find(in, 1, 'last'), :) - ah(find(in, 1) - 1, :)) ./ aT;
        dat_w = (at(find(in, 1, 'last'), :) - at(find(in, 1) - 1, :)) ./ aT;
        rows = {'left', Sl; 'trap', St; 'mid', Sm; 'trap_shift', Ss; 'D_trap=L-T', Sl - St; 'D_mid=L-M', Sl - Sm; ...
                'pred(all k)', Sp_all; 'corr(K*i)', Sc; 'dA_hat', dah; 'dA_true', dat_w};
        for r = 1:size(rows, 1)
            v = 100 * rows{r, 2};
            if all(isnan(v)); continue; end
            fprintf('  %-8s  %-11s  %+9.3f  %7.3f  %6.3f  | %.2f\n', wn{w}, rows{r, 1}, mean(v), std(v), std(v)/sqrt(numel(v)), excl);
        end
        R.win.(wn{w}) = struct('S_left', Sl, 'S_trap', St, 'S_mid', Sm, 'S_trap_shift', Ss, 'S_corr', Sc, ...
                               'dA_hat', dah, 'dA_true', dat_w, 'excl_pct', excl, 'S_pred_all', Sp_all);
    end
    sgn = sign(mean(R.win.descosc.S_left - R.win.descosc.S_trap));
    fprintf('  SIGN of D_trap over descend+oscillate: %+d ; |D_trap| = %.2f%% of a_T\n', sgn, ...
            100*abs(mean(R.win.descosc.S_left - R.win.descosc.S_trap)));
end

function s = nansum_masked(X, m)
    X(~m) = 0;  s = sum(X, 1);
end

function s = fmt3(v)
    s = sprintf('%+8.3f +- %6.3f (SEM %5.3f)', mean(v), std(v), std(v)/sqrt(numel(v)));
end
