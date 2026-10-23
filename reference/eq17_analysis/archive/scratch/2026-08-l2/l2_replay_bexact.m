% L2 predict replay -- ROUTE B-EXACT (seed 7, z): exact x_pred/x_upd from the
% production obs_dump hook (bit-identical when on, checked in l2_replay_bitcheck.m).
% Verifies (1) the Kalman identity incl. the x_upd(4) clamp, (2) the offline
% reconstruction of M_row4 from h_bar_d_out and delta_x_hat_3_out (units, shift, R),
% (3) the size of the unlogged MA(2) term alpha*(m1+m2).
% Registration: test_script/scratch/l2_replay_prereg.txt
cd('/Users/kevin/code/MotionControl_Simu-motion-test');
addpath(genpath('test_script')); addpath(genpath('model'));
EX = load('/Users/kevin/.claude/jobs/8581427c/tmp/l2r/l2_bexact_seed7.mat');
R_um = physical_constants().R;
AX = 3;
for arm = {'base', 'apknown'}
    E = EX.(arm{1});  lc = E.K.lambda_c;  al = 1 - lc;  a_nom = E.a_nom;
    t = E.tout(:);  N = numel(t);  n = E.n_state;
    fprintf('\n===== B-EXACT arm %s (seed 7, z, n_state=%d, records=%d, R=%.3f um) =====\n', arm{1}, n, numel(E.k), R_um);
    % records i = 1..N-1 <-> log rows k = i+1
    kk = E.k(:);  assert(isequal(kk, (1:N-1).'), 'k_step not 1..N-1');
    Xp = E.x_pred;  Xu = E.x_upd;
    ah  = E.a_bar_hat_out(:, AX);  K1 = E.K_a_y1_out(:, AX);  I1 = E.innov_y1_out(:, AX);
    K2  = E.K_a_y2_out(:, AX);     I2 = E.innov_y2_out(:, AX);
    ap  = E.a_prime_out(:, AX) / a_nom;  hd = E.h_bar_d_out(:);  ht = E.h_bar_true_out(:);
    dx3 = E.delta_x_hat_3_out(:, AX);
    fprintf('  x_upd(4)[i] vs a_bar_hat_out[i+1]        max|diff| %.3e\n', max(abs(Xu(:, 4) - ah(2:end))));
    fprintf('  x_upd(3)[i]*R vs delta_x_hat_3_out[i+1]  max|diff| %.3e um\n', max(abs(Xu(:, 3) * R_um - dx3(2:end))));
    % exact predict (row k = i+1): x_pred(4)[i] - x_upd(4)[i-1], with x_upd(4)[0] = seed = ah(1)
    xu_prev4 = [ah(1); Xu(1:end-1, 4)];
    pred_exact = Xp(:, 4) - xu_prev4;
    pred_ident = ah(2:end) - ah(1:end-1) - K1(2:end) .* I1(2:end) - K2(2:end) .* I2(2:end);
    dI = pred_ident - pred_exact;
    fprintf('  Kalman identity: max|identity - exact| %.3e  (samples > 1e-15: %d -> clamp/lock events)\n', ...
            max(abs(dI)), nnz(abs(dI) > 1e-15));
    % M_tot exact from state slots of the PREVIOUS posterior
    xu_prev3 = [0; Xu(1:end-1, 3)];
    if n >= 9; m_prev = [0; Xu(1:end-1, 8) + Xu(1:end-1, 9)]; else; m_prev = zeros(N-1, 1); end
    dwd_prev = hd(2:end) - hd(1:end-1);                 % Delta_wbar_d[k-1] = w_d[k] - w_d[k-1]
    M_row4_state = dwd_prev + al * xu_prev3;
    M_row4_log   = dwd_prev + al * dx3(1:end-1) / R_um;        % delta_x_hat_3_out[k-1]/R, k-1 = i
    M_tot_exact  = M_row4_state + al * m_prev;
    apk = ap(2:end);
    dcl = abs(pred_exact - apk .* M_tot_exact);
    fprintf('  h_bar_d_out(1:3) = %s ; h_bar_true_out(1:3) = %s\n', mat2str(hd(1:3), 6), mat2str(ht(1:3), 6));
    fprintf('  predict_exact vs a''*M_tot(state slots 3,8,9 + dw_d from h_bar_d_out): max|diff| %.3e (i=1: %.3e ; i>=2: %.3e)\n', ...
            max(dcl), dcl(1), max(dcl(2:end)));
    fprintf('  M_row4 from LOG (delta_x_hat_3_out[k-1]/R) vs from state: max|diff| %.3e\n', ...
            max(abs(M_row4_log - M_row4_state)));
    fprintf('  x_pred(3) check: x_pred(3) - (lc*x3_prev - al*m_prev): max|diff| %.3e\n', ...
            max(abs(Xp(:, 3) - (lc * xu_prev3 - al * m_prev))));
    % size of the unlogged term
    aT = mean(E.a_true_out(t >= 3.8 & t <= 4.8, AX)) / a_nom;
    tk = t(2:end);
    W = {'far', [0 0.5]; 'desc', [0.5 1.5]; 'osc', [1.6 3.5]; 'trough', [3.8 4.8]; 'all', [0 4.8]};
    fprintf('  window   sum a''*M_row4/aT   sum a''*al*(m1+m2)/aT   sum predict/aT   [%% of a_T]   rms(al*m)/rms(M_row4)\n');
    for w = 1:size(W, 1)
        in = tk >= W{w, 2}(1) & tk <= W{w, 2}(2);
        fprintf('  %-7s  %+10.3f          %+10.4f            %+10.3f            %.4f\n', W{w, 1}, ...
                100 * sum(apk(in) .* M_row4_state(in)) / aT, 100 * sum(apk(in) .* al .* m_prev(in)) / aT, ...
                100 * sum(pred_exact(in)) / aT, rms(al * m_prev(in)) / max(rms(M_row4_state(in)), eps));
    end
    % true step vs M_row4
    dht = ht(2:end) - ht(1:end-1);
    in = tk >= 0.5 & tk <= 3.5;
    fprintf('  M_row4 vs true step Delta h_true[k]: corr %.4f, sum(M_row4)/sum(dh_true) = %.5f, sum(a''*M_row4)/sum(a''*dh_true) = %.5f (desc+osc)\n', ...
            corr(M_row4_state(in), dht(in)), sum(M_row4_state(in)) / sum(dht(in)), ...
            sum(apk(in) .* M_row4_state(in)) / sum(apk(in) .* dht(in)));
    for l = -2:2
        ii = find(in);  ii = ii(ii + l >= 1 & ii + l <= N-1);
        fprintf('     corr(M_row4[k], dh_true[k%+d]) = %.4f\n', l, corr(M_row4_state(ii), dht(ii + l)));
    end
end
