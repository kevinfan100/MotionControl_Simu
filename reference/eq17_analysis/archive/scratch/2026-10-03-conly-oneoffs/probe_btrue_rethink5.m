% STATUS: ACTIVE (scratch) | PURPOSE: exact (Taylor-free) split of the far-field
%   seed (09-02). Per step, E[SE] = E[a'_used*Mhat] - E[a'_true*Mhat]
%     = (E[a'_used]-E[a'_true])*E[Mhat]      "mean part"
%     + (Cov(a'_used,Mhat)-Cov(a'_true,Mhat)) "covariance part" (Ito-type)
%   Both arms; a'_used in aptrue = a'(cmd) (ap_known_at='cmd'), so the b_true seed
%   relative to the unbiased reference = the b_true row minus the aptrue row.
function probe_btrue_rethink5()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    D = load(fullfile(od, 'arms30_seedtruth_pair.mat'));
    R = 2.25; lc = 0.7;
    fprintf('window      arm     mean-part   cov-part   | cov(a_used,M)  cov(a_true,M)  | E[a_used]-E[a_true]  E[M]\n');
    for W = {[0 0.5],[0.5 1],[1 2],[2 3],[3 5],[5 7],[7 9],[9 10.5],[10.5 12.5]}
        for A = {'btrue','aptrue'}
            d = D.(A{1}); t = d.t(:); C = d.C; N = numel(t); k1 = 2:N; dWd = diff(d.hd);
            Mh  = dWd + (1-lc)*C.delta_x_hat_3_out(k1-1,:)/R;
            apU = C.a_prime_out(k1,:)./C.ad;  apT = C.a_prime_true_out(k1,:)./C.ad;
            cv = @(X,Y) mean((X - mean(X,2)).*(Y - mean(Y,2)),2);
            mp = (mean(apU,2) - mean(apT,2)).*mean(Mh,2);
            cU = cv(apU,Mh); cT = cv(apT,Mh); cp = cU - cT;
            m = t(k1) >= W{1}(1) & t(k1) < W{1}(2);
            fprintf('[%4.1f,%4.1f] %-7s %+9.5f  %+9.5f  | %+10.2e    %+10.2e    | %+9.2e         %+8.2e\n', ...
                W{1}, A{1}, sum(mp(m)), sum(cp(m)), sum(cU(m)), sum(cT(m)), mean(mean(apU(m,:),2)-mean(apT(m,:),2)), mean(mean(Mh(m,:),2)));
        end
    end
end
