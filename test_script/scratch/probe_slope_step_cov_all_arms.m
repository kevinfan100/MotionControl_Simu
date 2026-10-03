% STATUS: ACTIVE (scratch) | PURPOSE: ONE instrument on EVERY oracle-arm variant we
%   have logs for (09-02). Core claim under test: the only predict-side quantity that
%   differs between arms is the slope the law uses, and its bias-relevant part is
%     Cov(a'_used, Mhat)   (slope fluctuation x step-estimate fluctuation, across seeds)
%   Deterministic-slope arms (a'_true @cmd) must give exactly 0; every biased arm must
%   give a nonzero value whose sign matches the bias where the loop is weak.
%   Prints per time-quarter: cum Cov(a'_used,M), cum Cov(a'_true,M), cum SE mean-part,
%   and the cross-seed mean error m at the quarter end.
function probe_slope_step_cov_all_arms()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    R = 2.25; lc = 0.7;
    % ---- full-run files
    F = {'arm_aptrue_evalpoint.mat',   {'a''_true@TRUE  ramp10s Euler seed=anchor', 'a''_true@CMD   ramp10s Euler seed=anchor'}; ...
         'arm_four_evalpoint_meng.mat',{'b_true@TRUE    meng   Euler seed=anchor','b_true@CMD     meng   Euler seed=anchor', ...
                                        'a''_true@TRUE  meng   Euler seed=anchor','a''_true@CMD   meng   Euler seed=anchor'}};
    for i = 1:size(F,1)
        S = load(fullfile(od, F{i,1}));
        for j = 1:numel(S.O)
            runs = S.O{j}.runs; nS = numel(runs); r = runs{1}; N = numel(r.tout);
            E = zeros(N,nS); apU = E; apT = E; d3 = E;
            for q = 1:nS
                r = runs{q}; ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3);
                E(:,q) = r.a_bar_hat_out(:,3) - r.a_true_out(:,3)/ad;
                apU(:,q) = r.a_prime_out(:,3)/ad; apT(:,q) = r.a_prime_true_out(:,3)/ad;
                d3(:,q) = r.delta_x_hat_3_out(:,3)/R;
            end
            hd = r.h_bar_d_out(:,end); t = r.tout(:);
            report(F{i,2}{j}, t, hd, E, apU, apT, d3, lc, nS);
        end
    end
    % ---- compact files (30 seeds, seed=truth)
    G = {'arms30_seedtruth_pair.mat',            'meng  Euler seed=truth'; ...
         'arms30_seedtruth_pair_canon.mat',      'canon Euler seed=truth'; ...
         'arms30_seedtruth_pair_canon_exact.mat','canon EXACT seed=truth'};
    for i = 1:size(G,1)
        S = load(fullfile(od, G{i,1}));
        for A = {'btrue','aptrue'}
            d = S.(A{1}); C = d.C; nS = size(d.E,2);
            if strcmp(A{1},'btrue'); nm = 'b_true@TRUE   '; else; nm = 'a''_true@CMD  '; end
            report([nm ' ' G{i,2}], d.t(:), d.hd(:), d.E, C.a_prime_out./C.ad, C.a_prime_true_out./C.ad, C.delta_x_hat_3_out/R, lc, nS);
        end
    end
end
function report(name, t, hd, E, apU, apT, d3, lc, nS)
    N = numel(t); k1 = 2:N; dWd = diff(hd);
    Mh = dWd + (1-lc)*d3(k1-1,:);
    cv = @(X,Y) mean((X - mean(X,2)).*(Y - mean(Y,2)),2);
    cU = cv(apU(k1,:), Mh); cT = cv(apT(k1,:), Mh);
    mp = (mean(apU(k1,:),2) - mean(apT(k1,:),2)).*mean(Mh,2);
    m  = mean(E,2); T = t(end); edges = [0 .25 .5 .75 1]*T;
    fprintf('\n%-44s (%d seeds)  final m %+8.5f  sd %7.5f\n', name, nS, m(end), std(E(end,:)));
    fprintf('   quarter      cumCov(a''_used,M)  cumCov(a''_true,M)  cum mean-part   m(end of q)\n');
    for q = 1:4
        w = t(k1) >= edges(q) & t(k1) < edges(q+1) + (q==4);
        kq = find(t < edges(q+1) + (q==4), 1, 'last');
        fprintf('   [%4.1f,%4.1f]   %+12.5f      %+12.5f      %+12.5f    %+9.5f\n', edges(q), edges(q+1), sum(cU(w)), sum(cT(w)), sum(mp(w)), m(kq));
    end
    fprintf('   total         %+12.5f      %+12.5f      %+12.5f\n', sum(cU), sum(cT), sum(mp));
end
