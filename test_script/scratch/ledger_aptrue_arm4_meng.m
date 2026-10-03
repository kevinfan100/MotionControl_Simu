% STATUS: ACTIVE (scratch) | PURPOSE: the term-by-term ledger of the a'_true ARM 4
%   (exact step + slope@est + app_known + pred_mean2) on Meng, built with a FIXED
%   first-order reference a'_c = a'_true(w_d[k-1]) (the arm-2 slope) so that it is
%   comparable with arm 2 (2026-09-02). Everything from the obs_dump capture
%   (probe_aptrue_predict_drift_meng{,_arm4}.m); no fit, no Taylor on the estimator
%   side (the exact step and mean2 are read off the logs).
%
%   Predict-stage error increment (True - Est):  De = TI - (EX + mean2)
%     TI    = a_true[k+1] - a_true[k]                          true increment
%     EX    = a'_e B / (1 + a'_e B / (1 - x4))                   the exact step actually taken (a'_e = slope at EST height)
%     mean2 = x_pred(4) - x4 - EX                              the wired second-order term (read off the logs)
%   Truth side, Taylor to 2nd order about the commanded left end (a'_c, a'' there):
%     TI - a'_c Bt = T1 + 1/2 a'' Bt^2,   T1 = -a'' d3t Bt = G1 + E1 + E2 + E3
%        G1 = -a'' x3 B      (deterministic evaluation point; slope@est removes it)
%        E1 = -a'' x3 u      (estimate x increment error)      <- A2 says mean 0
%        E2 = -a'' e3 B      (slot-3 error x estimated increment)<- A2 says mean 0
%        E3 = -a'' e3 u      (error x error)                    <- what cov_e3u models
%   Estimator side:  EX - a'_c B = Sd + 1/2 a''_law B^2 + O(3),  Sd = (a'_e - a'_c) B  (= -G1 + O(3))
%   =>  r4 := De - a'_c u = (T1 - Sd) + gap + T2 + T3 - mean2 + O(3)
%        gap = 1/2 (a'' - a''_law) B^2,  T2 = a'' B u,  T3 = 1/2 a'' u^2
%   mean2 (wired) = -a'' cov_e3u + 1/2 a'' var_u + 1/2 (a'' - a''_law) M^2   (models E3, T3, gap)
%   Five buckets that sum to r4:
%     A2-eval  = E1 + E2 + (G1 - Sd)      pieces the model sets to zero by orthogonality
%     A3-corr  = E3 - (-a'' cov_e3u)      measured error x error vs the P-based model of it
%     A3-var   = T3 - (1/2 a'' var_u)     measured 1/2 a'' u^2 vs the P/Q-based model of it
%     gap-mis  = gap - (1/2 (a''-a''_law) M^2)
%     A2-T2    = T2                        never in the model
%   For the arm-2 file mean2 = 0 and Sd = 0, so r2 = G1 + E1 + E2 + E3 + gap + T2 + T3.
function out = ledger_aptrue_arm4_meng(which)
    if nargin < 1 || isempty(which); which = 'arm4'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    if strcmp(which,'arm4'); fn = 'aptrue_predict_drift_probe_meng_arm4.mat'; else; fn = 'aptrue_predict_drift_probe_meng.mat'; end
    D = load(fullfile(od, fn)); S = D.S;
    pc = physical_constants(); lc = 0.7; al = 1 - lc;
    a_o = pc.Ts / (pc.gamma_N * pc.R); kappa_T = 4 * (pc.k_B * pc.T / pc.R) * a_o; s2nw = (0.00057 / pc.R)^2;
    abar = @(w) 1 ./ arrayfun(@(x) local_cp(x), w); hh = 1e-3;
    ap_num  = @(w) (abar(w+hh) - abar(w-hh)) / (2*hh);
    app_num = @(w) (abar(w+hh) - 2*abar(w) + abar(w-hh)) / hh^2;
    names = {'r','De','au','TI','EX','mean2','Sd','G1','E1','E2','E3','T1','T2','T3','gap','clos','M_E3','M_T3','M_gap','m2chk','A2eval','A3corr','A3var','gapmis','A2T2'};
    acc = struct(); for f = names; acc.(f{1}) = []; end; acc.t = []; acc.seed = [];
    nS = numel(S);
    for q = 1:nS
        s = S(q); n = size(s.x_upd,1); j = (4:n).'; kk = j + 1;
        w = s.h_bar_true; at = s.a_true; hd = s.hd; fb = s.f_bar; apl = s.a_prime;
        x3 = s.x_upd(:,3); x4 = s.x_upd(:,4); x8 = s.x_upd(:,8); x9 = s.x_upd(:,9); xp4 = s.x_pred(:,4);
        dwd = hd(kk) - hd(kk-1);
        B  = dwd + al*x3(j-1) + al*(x8(j-1) + x9(j-1));             % estimator's increment M_pred (incl MA(2))
        Bt = w(kk) - w(kk-1);  u = Bt - B;
        x4p = x4(j-1);  ae = apl(kk);                                % slope actually used (arm4: at est height; arm2: at cmd)
        ac = ap_num(hd(kk-1));                                       % fixed reference slope (cmd height, left end)
        app = app_num(hd(kk-1));                                     % true curvature at the commanded left end
        d3t = hd(kk-1) - w(kk-1);  e3 = d3t - x3(j-1);
        TI = at(kk) - at(kk-1);
        EX = ae .* B ./ (1 + ae .* B ./ (1 - x4p));
        mean2 = xp4(j) - x4p - EX;                                   % wired mean2, read off the logs (0 for arm 2)
        De = TI - (EX + mean2);  au = ac .* u;  r = De - au;
        Sd = (ae - ac) .* B;
        G1 = -app .* x3(j-1) .* B;  E1 = -app .* x3(j-1) .* u;  E2 = -app .* e3 .* B;  E3 = -app .* e3 .* u;
        T1 = -app .* d3t .* Bt;
        app_law = -2 * ae.^2 ./ (1 - x4p);
        gap = 0.5 * (app - app_law) .* B.^2;  T2 = app .* B .* u;  T3 = 0.5 * app .* u.^2;
        clos = r - ((T1 - Sd) + gap + T2 + T3 - mean2);               % O(3) closure
        % --- the wired model of the three pieces it covers (same P as the controller saw)
        Fdw = fb(kk-1) + al*(fb(kk-2) + fb(kk-3));
        P = @(a,b) squeeze(s.P_upd(j-1,a,b));
        cov_e3u = al*P(3,3) + al*(P(3,8) + P(3,9)) + Fdw.*P(3,4);
        var_u   = al^2*(P(3,3) + P(8,8) + P(9,9) + 2*P(3,8) + 2*P(3,9) + 2*P(8,9)) + Fdw.^2.*P(4,4) ...
                + 2*al*Fdw.*(P(3,4) + P(4,8) + P(4,9)) + kappa_T*x4(j-1) + al^2*s2nw;
        app_e = app_num(max(hd(kk-1) - x3(j-1), 1.001));             % curvature at the est height (what app_known feeds)
        M_E3 = -app_e .* cov_e3u;  M_T3 = 0.5 * app_e .* var_u;  M_gap = 0.5 * (app_e - app_law) .* B.^2;
        m2chk = mean2 - (M_E3 + M_T3 + M_gap);                       % reconstruction check of the wired formula (arm 4)
        A2eval = E1 + E2 + (G1 - Sd);  A3corr = E3 - M_E3;  A3var = T3 - M_T3;  gapmis = gap - M_gap;  A2T2 = T2;
        if ~strcmp(which,'arm4'); A3corr = E3; A3var = T3; gapmis = gap; A2eval = E1 + E2 + G1; end   % arm 2: nothing modelled
        for f = names; acc.(f{1}) = [acc.(f{1}); eval(f{1})]; end
        acc.t = [acc.t; s.t(kk)]; acc.seed = [acc.seed; q*ones(numel(kk),1)];
    end
    W = {'far', acc.t>0.5 & acc.t<=6; 'near', acc.t>6 & acc.t<=10.5; 'hold', acc.t>10.6};
    fprintf('\n[%s] Meng, %d seeds, window means in 1e-6 abar/step (True - Est)\n', which, nS);
    fprintf('%-6s', 'win'); for f = names; fprintf('%8s', f{1}); end; fprintf('\n');
    for wi = 1:3
        m = W{wi,2}; fprintf('%-6s', W{wi,1}); for f = names; fprintf('%8.3f', 1e6*mean(acc.(f{1})(m))); end; fprintf('\n');
    end
    m = W{3,2}; fprintf('%-6s', 'SEMh');
    for f = names; pm = arrayfun(@(q) mean(acc.(f{1})(m & acc.seed==q)), 1:nS); fprintf('%8.3f', 1e6*std(pm)/sqrt(nS)); end; fprintf('\n');
    fprintf('\nhold buckets (sum = r): A2-eval %+.3f | A3-corr %+.3f | A3-var %+.3f | gap-mis %+.3f | A2-T2 %+.3f | sum %+.3f | r %+.3f | closure %+.3f\n', ...
        1e6*mean(acc.A2eval(m)), 1e6*mean(acc.A3corr(m)), 1e6*mean(acc.A3var(m)), 1e6*mean(acc.gapmis(m)), 1e6*mean(acc.A2T2(m)), ...
        1e6*mean(acc.A2eval(m)+acc.A3corr(m)+acc.A3var(m)+acc.gapmis(m)+acc.A2T2(m)), 1e6*mean(acc.r(m)), 1e6*mean(acc.clos(m)));
    out = acc;
end
function cp = local_cp(w)
    [~, cp] = calc_correction_functions(w);
end
