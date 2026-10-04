% STATUS: ACTIVE (scratch) | PURPOSE: instrument check for the "cross rectifier"
%   label (09-02). The exact seed-mean of the SE channel in hold/descent contains
%   Cov(e4, dw3_hat) (gain error x ESTIMATED disturbance), not Cov(e4, e3).
%   Split: Cov(e4, dw3_hat) = Cov(e4, dw3_true) + Cov(e4, e3).  Which one carries it?
function probe_btrue_rethink2()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    D = load(fullfile(od, 'arms30_seedtruth_pair.mat'));
    R = 2.25; lc = 0.7; b = 8/9;
    d = D.btrue; t = d.t(:); E = d.E; C = d.C; N = numel(t); k1 = 2:N;
    dWd = diff(d.hd);
    eps4 = E(k1-1,:);                                  % e4 at step k
    dw3h = C.delta_x_hat_3_out(k1-1,:)/R;              % estimated disturbance used in Mhat at step k+1
    dw3t = C.trk_true(k1-1,:);                         % true tracking error, aligned (see 08-31 pairing)
    e3   = dw3h - dw3t;
    Mh   = dWd + (1-lc)*dw3h;
    abarh = mean(C.a_bar_hat_out(k1-1,:),2); A_a = -2*b*(1-abarh);
    cv = @(X,Y) mean((X - mean(X,2)).*(Y - mean(Y,2)),2);
    Ch  = cv(eps4, dw3h); Ct = cv(eps4, dw3t); Ce = cv(eps4, e3);
    % exact seed-mean of SE (a-reference), per step
    SE_exact = b*mean(Mh.*(-2*(1-C.a_true_norm(k1-1,:)).*eps4 + eps4.^2),2);
    SE_log   = mean((C.a_prime_out(k1,:)./C.ad - C.a_prime_true_out(k1,:)./C.ad).*Mh,2);
    T_h = A_a.*(1-lc).*Ch; T_t = A_a.*(1-lc).*Ct; T_e = A_a.*(1-lc).*Ce;
    md = t(k1) <= 10.5; mh = t(k1) > 10.5;
    fprintf('cum SE: logged %+.4f | exact-quadratic %+.4f  (descent %+.4f / hold %+.4f)\n', ...
        sum(SE_log), sum(SE_exact), sum(SE_exact(md)), sum(SE_exact(mh)));
    fprintf('cross via Cov(e4,dw3_hat)  : descent %+.4f  hold %+.4f\n', sum(T_h(md)), sum(T_h(mh)));
    fprintf('  = Cov(e4,dw3_true) part  : descent %+.4f  hold %+.4f\n', sum(T_t(md)), sum(T_t(mh)));
    fprintf('  + Cov(e4,e3)       part  : descent %+.4f  hold %+.4f   <- what the 08-31 script used\n', sum(T_e(md)), sum(T_e(mh)));
    hd = d.hd(k1);
    for band = {[3 7],[2 3],[1.5 2],[1 1.5]}
        mb = md & hd >= band{1}(1) & hd < band{1}(2);
        fprintf('  w_d [%.1f,%.1f): Cov(e4,dw3_hat) %+.2e = true %+.2e + e3 %+.2e\n', band{1}, mean(Ch(mb)), mean(Ct(mb)), mean(Ce(mb)));
    end
    mb = mh; fprintf('  hold          : Cov(e4,dw3_hat) %+.2e = true %+.2e + e3 %+.2e\n', mean(Ch(mb)), mean(Ct(mb)), mean(Ce(mb)));
end
