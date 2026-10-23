% STATUS: ACTIVE (scratch) | PURPOSE: term-by-term check of the second-order mean
%   line of derivation/0902_formC_aptrue_4state.tex against the PREDICT-stage
%   error increment recorded by probe_aptrue_predict_drift.m (a'_true arm @cmd,
%   law_exact_step, canon deep, 8 seeds, obs_dump).
%   Everything is evaluated from logged quantities, no fit:
%     De    = (a_true[k] - a_pred[k]) - (a_true[k-1] - a_upd[k-1])   True-Est, predict only
%     a' u  = first-order part, u = B_true - B_hat (exact, both known in sim)
%     r     = De - a' u                                  what second order must explain
%     T1..T3 = the three second-order lines of S6 evaluated with TRUE quantities
%     G1..G5 = the five terms of the reduced (filter-computable) closed form
%   Read: mean(r) vs mean(T1+T2+T3) tests the Taylor step; mean(F=sum G) vs mean(r)
%   tests what the assumptions A1-A3 drop. Units printed in 1e-6 a_bar per step.
function out = analyze_aptrue_predict_drift()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    D = load(fullfile(od, 'aptrue_predict_drift_probe.mat'));  S = D.S;
    pc = physical_constants();  lc = 0.7;  al = 1 - lc;
    a_o = pc.Ts / (pc.gamma_N * pc.R);  kappa_T = 4 * (pc.k_B * pc.T / pc.R) * a_o;
    s2nw = (0.00057 / pc.R)^2;                         % meas noise 0.57 nm (house), [R^2]; negligible in Q33
    % numeric law derivatives on the plant curve a_bar(w) = 1/c_perp(w)
    abar = @(w) 1 ./ arrayfun(@(x) local_cp(x), w);
    hh = 1e-3;
    ap_num  = @(w) (abar(w+hh) - abar(w-hh)) / (2*hh);
    app_num = @(w) (abar(w+hh) - 2*abar(w) + abar(w-hh)) / hh^2;

    nS = numel(S);  acc = struct();
    names = {'De','au','r','T1','T2','T3','Tsum','res3','G1','G2','G3','G4','G5','F','Fmr','T1s','S1w','S1m','F2','F2mr','gap','Tsum2','Eex'};
    for f = names; acc.(f{1}) = []; end
    acc.t = []; acc.seed = [];
    for q = 1:nS
        s = S(q);  n = size(s.x_upd, 1);  k = (1:n).' + 1;      % record j <-> driver row k = j+1
        w  = s.h_bar_true;  at = s.a_true;  hd = s.hd;  tr = s.trk_true;  fb = s.f_bar;  apl = s.a_prime;
        x4 = s.x_upd(:,4);  x3 = s.x_upd(:,3);  x8 = s.x_upd(:,8);  x9 = s.x_upd(:,9);  xp4 = s.x_pred(:,4);
        if q == 1
            % --- instrument checks (seed 1 only, identical for all) ---
            fprintf('[check] a_o %.4e  kappa_T %.4e\n', a_o, kappa_T);
            for sh = -1:1     % is h_bar_true logged at the same instant as p_true? (constant diff => same instant + wall offset)
                ii = (2:numel(tr)-1).';  dd = tr(ii) - (hd(ii) - w(ii+sh));
                fprintf('[check] trk_true(k) - (hd(k) - h_bar_true(k%+d)): mean %+.4e  std %.2e\n', sh, mean(dd), std(dd));
            end
            fprintf('[check] a_true vs abar(h_bar_true) same row: max rel %.2e\n', max(abs(at(2:end) - abar(w(2:end))) ./ at(2:end)));
            % alignment by correlation of INCREMENTS (robust to bias): estimator step vs truth step at lag L
            dEst = xp4(2:end) - x4(1:end-1);            % predict increment, records 2..n  (rows k = 3..n+1)
            dBh  = al*x3(1:end-1);                      % rough estimator bracket without command (hold/osc jitter part)
            kk0  = k(2:end);
            fprintf('[check] corr(dEst, dTrue at lag L) / corr(dBh, dW at lag L):');
            for L = -1:2
                idx = kk0 - L; ok = idx >= 2 & idx <= numel(at);
                dT = at(idx(ok)) - at(idx(ok)-1);  dW = w(idx(ok)) - w(idx(ok)-1);
                c1 = corr(dEst(ok), dT);  c2 = corr(dBh(ok), dW);
                fprintf('  L=%d: %.3f / %.3f', L, c1, c2);
            end; fprintf('\n');
            for lagA = 0:2
                idx = k - lagA; ok = idx >= 1;
                e4 = at(idx(ok)) - x4(ok);  fprintf('[check] gain lag %d: var(e4) %.3e  mean(e4) %+.3e\n', lagA, var(e4), mean(e4));
            end
            for lag3 = 0:2
                idx = k - lag3; ok = idx >= 1;
                e3 = tr(idx(ok)) - x3(ok);  P33s = squeeze(s.P_upd(ok,3,3));
                fprintf('[check] slot3 lag %d: var(e3) %.3e  mean P33 %.3e  ratio %.3f | hold-only ratio %.3f\n', lag3, var(e3), mean(P33s), var(e3)/mean(P33s), var(e3(s.t(k(ok))>3.5))/mean(P33s(s.t(k(ok))>3.5)));
            end
            % slope evaluation point: a_prime logged at row k vs numeric slope at hd(k-1) / hd(k)
            jj = 10:n-1;
            fprintf('[check] a_prime(k) vs abar''(hd(k-1)): max rel %.2e | vs abar''(hd(k)): max rel %.2e\n', ...
                max(abs(apl(k(jj)) - ap_num(hd(k(jj)-1))) ./ apl(k(jj))), max(abs(apl(k(jj)) - ap_num(hd(k(jj)))) ./ apl(k(jj))));
            % exact-step reproduction with two Delta_w_d index conventions
            jr = (3:n).';  kr = jr + 1;
            for v = 1:2
                if v == 1; dwd = hd(kr) - hd(kr-1); lab = 'hd(k)-hd(k-1)'; else; dwd = hd(kr-1) - hd(kr-2); lab = 'hd(k-1)-hd(k-2)'; end
                Bh = dwd + al*x3(jr-1) + al*(x8(jr-1) + x9(jr-1));
                x4p = x4(jr-1);  a1 = apl(kr);
                xp = x4p + a1 .* Bh ./ (1 + a1 .* Bh ./ (1 - x4p));
                fprintf('[check] exact step reproduces x_pred(4) with Delta_w_d = %s: max |err| %.2e\n', lab, max(abs(xp - xp4(jr))));
            end
        end
        % --- alignment used below: truth lag LAGA (posterior at record j <-> truth row k-LAGA) ---
        LAGA = 0;   % instrument checks: a_true/h_bar_true(k) are the instant of call k (gain lag 0); p_true_out(k) is post-advance
        j  = (4:n).';  kk = j + 1;                                   % records with 3 predecessors
        dwd = hd(kk) - hd(kk-1);                                    % v1 (checked above)
        Bh  = dwd + al*x3(j-1) + al*(x8(j-1) + x9(j-1));            % estimator bracket (code, incl MA(2))
        Bt  = w(kk-LAGA) - w(kk-1-LAGA);                            % true height increment
        u   = Bt - Bh;
        a1  = apl(kk);                                              % exogenous slope used in call k
        x4p = x4(j-1);
        De  = (at(kk-LAGA) - xp4(j)) - (at(kk-1-LAGA) - x4p);       % predict-stage error increment, True - Est
        au  = a1 .* u;
        r   = De - au;
        app_d = app_num(hd(kk-1));                                  % true curvature at the evaluation height
        d3t = hd(kk-1-LAGA) - w(kk-1-LAGA);                         % true tracking error at the instant before the step (NOT trk_true: p_true_out is post-advance)
        T1  = -app_d .* d3t .* Bt;
        T2  =  app_d .* Bh .* u;
        T3  =  0.5 * app_d .* u.^2;
        Tsum = T1 + T2 + T3;  res3 = r - Tsum;
        gap  = 0.5 * (app_d - (-2 * a1.^2 ./ (1 - x4p))) .* Bh.^2;   % curvature-gap term (true minus constant-b curvature)
        Tsum2 = Tsum + gap;
        Eex  = at(kk-LAGA) - x4(j);                                 % exact-arm error after the update at row k, True - Est
        % filter-computable reduction
        app_f = -2 * a1.^2 ./ (1 - x4p);
        Fdw = fb(kk-1) + al*(fb(kk-2) + fb(kk-3));
        P33 = squeeze(s.P_upd(j-1,3,3));  P34 = squeeze(s.P_upd(j-1,3,4));  P44 = squeeze(s.P_upd(j-1,4,4));
        Q33 = kappa_T*(x4(j-1) + al^2*(x4(j-2) + x4(j-3))) + al^2*s2nw;
        G1 = -app_f .* x3(j-1) .* Bh;
        G2 = -0.5 * app_f .* (1 - lc^2) .* P33;
        G3 = -app_f .* lc .* Fdw .* P34;
        G4 =  0.5 * app_f .* Fdw.^2 .* P44;
        G5 =  0.5 * app_f .* Q33;
        F  = G1 + G2 + G3 + G4 + G5;  Fmr = F - r;
        % --- MA(2)-aware reduction (9-state SSOT): u = al*(e3 + e8 + e9) + Fdw*e4 + w_T - al*n_w, w_T white Var kappa_T*a_hat
        P88 = squeeze(s.P_upd(j-1,8,8));  P99 = squeeze(s.P_upd(j-1,9,9));  P89 = squeeze(s.P_upd(j-1,8,9));
        P38 = squeeze(s.P_upd(j-1,3,8));  P39 = squeeze(s.P_upd(j-1,3,9));  P48 = squeeze(s.P_upd(j-1,4,8));  P49 = squeeze(s.P_upd(j-1,4,9));
        T1s = T1 - G1;                                             % stochastic part of the true T1 (what A2/A3 must predict)
        S1w = -app_f .* al .* P33;                                 % white-container prediction of T1s
        S1m = -app_f .* (al*(P33 + P38 + P39) + Fdw.*P34);         % MA(2)-aware prediction of T1s
        VarU = al^2*(P33 + P88 + P99 + 2*P38 + 2*P39 + 2*P89) + Fdw.^2.*P44 + 2*al*Fdw.*(P34 + P48 + P49) + kappa_T*x4(j-1) + al^2*s2nw;
        F2 = G1 + S1m + 0.5*app_f.*VarU;  F2mr = F2 - r;
        for f = names; acc.(f{1}) = [acc.(f{1}); eval(f{1})]; end
        acc.t = [acc.t; s.t(kk)];  acc.seed = [acc.seed; q*ones(numel(kk),1)];
    end
    % --- report: window means in 1e-6 abar/step, SEM over the 8 seed means ---
    W = {'descend', acc.t>0.5 & acc.t<=1.5; 'osc', acc.t>1.5 & acc.t<=3.5; 'hold', acc.t>3.5; 'ALL', true(size(acc.t))};
    fprintf('\n%-8s', 'window'); for f = names; fprintf('%9s', f{1}); end; fprintf('\n');
    for wi = 1:size(W,1)
        m = W{wi,2};  fprintf('%-8s', W{wi,1});
        for f = names; fprintf('%9.3f', 1e6*mean(acc.(f{1})(m))); end; fprintf('\n');
        if wi == 3
            fprintf('%-8s', ' SEM');
            for f = names
                pm = arrayfun(@(q) mean(acc.(f{1})(m & acc.seed==q)), 1:nS);
                fprintf('%9.3f', 1e6*std(pm)/sqrt(nS));
            end; fprintf('\n');
        end
    end
    % cumulative injected drift over the run (seed mean), for scale against the closed-loop -0.0022
    fprintf('\ncumulative over run (seed mean, abar): sum r %+.5f | sum Tsum %+.5f | sum F %+.5f | sum G1 %+.5f | sum G2..G5 %+.5f\n', ...
        sum(acc.r)/nS, sum(acc.Tsum)/nS, sum(acc.F)/nS, sum(acc.G1)/nS, sum(acc.G2+acc.G3+acc.G4+acc.G5)/nS);
    fprintf('cumulative over hold only: sum r %+.5f | sum F %+.5f\n', sum(acc.r(acc.t>3.5))/nS, sum(acc.F(acc.t>3.5))/nS);
    out = acc;
end
function cp = local_cp(w)
    [~, cp] = calc_correction_functions(w);
end
