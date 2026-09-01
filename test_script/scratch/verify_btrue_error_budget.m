% STATUS: ACTIVE (scratch) | PURPOSE: full log-level verification of the b_true
%   bias mechanism claim (2026-08-31): the EKF second-order blind spot with the
%   dominant cross term A_a(1-lc)P34. Everything offline from
%   arms30_seedtruth_pair.mat (30 seeds, seed-at-truth pair).
%
%   Per-step identity (step k -> k+1), all from logs:
%     dE = [MI - dTrue] + u ,  MI = a'_used * Mhat ,  u = l41 e1 + l42 e2
%     Mhat = dw_cmd + (1-lc) * dw3_hat(prev step, log row k)
%   V1 closure: resid = dE - (MI - dTrue) - u  must be << rms(dE)
%   EXACT split of the predict side (no Taylor):
%     state-error channel  SE = (a'_used - a'_true) * Mhat     <- the law reading
%                               its own estimate; contains the rectifier
%     input channel        IN = a'_true * Mhat - dTrue          <- displacement +
%                               reference-path difference
%   V2 mean budget: cumulative seed means of SE, IN, U1, U2 stack to mean E(t).
%   V3 rectifier check: mean(SE) vs Taylor prediction
%         A_a.*(dWd.*m + (1-lc)*C34) + b.*Mmean.*(v + m.^2)
%     with A_a = -2b(1-a_bar_hat), C34/v/m measured across seeds.
%   V4 negative control: same battery on the a'_true arm (slope exogenous ->
%     SE must collapse and its mean ~ 0).
function out = verify_btrue_error_budget()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    D = load(fullfile(od, 'arms30_seedtruth_pair.mat'));
    R = 2.25; lc = 0.7; b = 8/9;
    out = struct();
    for A = {'btrue','aptrue'}
        d = D.(A{1}); t = d.t; nS = size(d.E, 2); N = numel(t);
        C = d.C;
        dWd = diff(d.hd);                                   % commanded step [R]
        k1 = 2:N;                                           % step k+1 rows
        dE  = d.E(k1,:) - d.E(k1-1,:);
        u1  = C.K_a_y1_out(k1,:) .* C.innov_y1_out(k1,:);
        u2  = C.K_a_y2_out(k1,:) .* C.innov_y2_out(k1,:);
        Mh  = dWd + (1-lc) * C.delta_x_hat_3_out(k1-1,:) / R;   % Mhat used at step k+1
        apU = C.a_prime_out(k1,:) ./ C.ad;   % um/pN -> a_bar' (driver :1229 stores physical)
        MI  = apU .* Mh;
        dT  = C.a_true_norm(k1,:) - C.a_true_norm(k1-1,:);
        res = dE - (MI - dT) - u1 - u2;
        fprintf('\n[%s] V1 closure: rms(resid) %.2e vs rms(dE) %.2e  ratio %.3f\n', ...
            A{1}, rms(res(:)), rms(dE(:)), rms(res(:))/rms(dE(:)));
        % exact split. a_prime_true_out is PHYSICAL (driver: local_a_prime_true
        % without /a_nom); normalize per seed by ad. a_prime_out is already a_bar'.
        apT = C.a_prime_true_out(k1,:) ./ C.ad;
        SE = (apU - apT) .* Mh;
        IN = apT .* Mh - dT;
        % V2 mean budget (cumulative seed means)
        mSE = cumsum(mean(SE,2)); mIN = cumsum(mean(IN,2));
        mU1 = cumsum(mean(u1,2)); mU2 = cumsum(mean(u2,2));
        mEc = mean(d.E(k1,:),2);
        % V3 rectifier: Taylor prediction of mean(SE)
        eps4 = d.E(k1-1,:);                                  % epsilon at step k
        m4 = mean(eps4,2); v4 = var(eps4,0,2);
        abarh = mean(C.a_bar_hat_out(k1-1,:),2);
        A_a = -2*b*(1-abarh);
        % C34 measured across seeds (slot3 error aligned k-1 <-> slot4 error)
        e3 = C.delta_x_hat_3_out(k1,:)/R - C.trk_true(k1-1,:);
        C34 = zeros(numel(k1),1);
        for kk = 1:numel(k1); c = cov(e3(kk,:), eps4(kk,:)); C34(kk) = c(1,2); end
        Mmean = mean(Mh,2);
        predSE = A_a .* (dWd .* m4 + (1-lc)*movmean(C34,801)) + b .* Mmean .* (v4 + m4.^2);
        mPred = cumsum(predSE);
        fprintf('[%s] V2 stack at end: SE %+.5f IN %+.5f U1 %+.5f U2 %+.5f | sum %+.5f vs mean E %+.5f\n', ...
            A{1}, mSE(end), mIN(end), mU1(end), mU2(end), mSE(end)+mIN(end)+mU1(end)+mU2(end), mEc(end));
        fprintf('[%s] V3 rectifier: cum mean(SE) %+.5f vs Taylor pred %+.5f (ratio %.2f) | cross-term share of pred %.2f\n', ...
            A{1}, mSE(end), mPred(end), mSE(end)/max(abs(mPred(end)),1e-9)*sign(mPred(end)), ...
            sum(A_a.*(1-lc).*movmean(C34,801))/max(mPred(end),1e-12));
        out.(A{1}) = struct('t', t(k1), 'mSE', mSE, 'mIN', mIN, 'mU1', mU1, 'mU2', mU2, ...
                            'mE', mEc, 'mPred', mPred, 'resid_ratio', rms(res(:))/rms(dE(:)), ...
                            'C34', C34, 'A_a', A_a);
    end
    save(fullfile(od,'btrue_error_budget.mat'), '-struct', 'out', '-v7.3');
    fprintf('\nsaved btrue_error_budget.mat\n');
end
