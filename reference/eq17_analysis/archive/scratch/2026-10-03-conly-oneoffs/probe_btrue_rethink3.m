% STATUS: ACTIVE (scratch) | PURPOSE: where/when is the positive mean seeded (09-02)?
%   Time profile of cross-seed mean error m(t) and the four cumulative channels,
%   plus the remaining linear amplification from t to the wall (product of the
%   Delta-w_d part of F_e(4,4) only; F_dw part not logged).
function probe_btrue_rethink3()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    B = load(fullfile(od, 'btrue_error_budget.mat'));
    D = load(fullfile(od, 'arms30_seedtruth_pair.mat'));
    d = D.btrue; R = 2.25; lc = 0.7; b = 8/9; C = d.C;
    t = B.btrue.t(:); N = numel(d.t); k1 = 2:N; dWd = diff(d.hd);
    abarh = mean(C.a_bar_hat_out(k1-1,:),2);
    F44lin = 1 - 2*b*(1-abarh).*dWd;                 % Delta-w_d part of F_e(4,4)
    hold_start = find(d.hd(k1) <= d.hd(end)+1e-9, 1);
    amp_rem = flipud(cumprod(flipud(F44lin(1:hold_start)))); % product k..hold_start
    m = B.btrue.mE; sem = std(d.E(k1,:),0,2)/sqrt(30);
    fprintf('   t     w_d   m(t)      SEM      cumSE    cumIN    cumU1    cumU2   | amp->wall\n');
    for tt = [0.5 1 2 3 4 5 6 7 8 9 9.5 10 10.5 11 12.4]
        k = find(t >= tt, 1); a = 1; if k <= hold_start, a = amp_rem(k); end
        fprintf('%5.1f  %5.2f  %+8.5f  %7.5f  %+8.5f %+8.5f %+8.5f %+8.5f  | %6.2f\n', ...
            t(k), d.hd(k+1), m(k), sem(k), B.btrue.mSE(k), B.btrue.mIN(k), B.btrue.mU1(k), B.btrue.mU2(k), a);
    end
    % same for the a'_true arm (negative control)
    fprintf('\n[aptrue]\n');
    for tt = [2 5 8 10 12.4]
        k = find(t >= tt, 1);
        fprintf('%5.1f  m %+8.5f  cumSE %+8.5f cumIN %+8.5f cumU1 %+8.5f cumU2 %+8.5f\n', ...
            t(k), B.aptrue.mE(k), B.aptrue.mSE(k), B.aptrue.mIN(k), B.aptrue.mU1(k), B.aptrue.mU2(k));
    end
end
