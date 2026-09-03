% STATUS: ACTIVE (scratch) | PURPOSE: the UPDATE-half product term on the gain row. The y1 gain of the
%   gain row is l41 = -a'(w_hat) l31 exactly (probe_aptrue_E_from_log), and a' is read at
%   w_hat = w_d - dw3_hat, so l41 fluctuates with dw3_hat (d l41 / d dw3_hat = +a'' l31). The update
%   increment l41 innov1 therefore has a second-order mean a'' l31 Cov(dw3_hat[k-1], innov1[k]) that the
%   linear update model (fixed L) does not contain. Measured from aptrue_est_final_<traj>_full.mat, hold.
%   RESULT (2026-09-03): Cov(dw3_hat_post, innov1) = +1.5e-7 (canon, 4 sigma) / +0.6e-7 (meng, 0.8 sigma);
%   the term = -0.158 +- 0.040 (canon) / -0.058 +- 0.076 (meng) e-6/step on the ESTIMATE -- real on canon
%   but pushes DOWN, opposite to the observed +0.25e-6/step upward hold drift. The linear part
%   l41_0 E[innov1] is unresolved (+-6e-6). The update-half mean needs the full derivation, not this
%   single term.
function probe_aptrue_update_half_term()
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
addpath(genpath(fullfile(root, 'model')));  od = fullfile(root, 'test_results', 'apd_acov_meng');
lc = 0.7; HFLOOR = 1.001; abar = @(w) 1 ./ arrayfun(@(x) local_cp(x), w); hh = 1e-3;
app_num = @(w) (abar(w+hh) - 2*abar(w) + abar(w-hh)) / hh^2;
for tr = {'canon','meng'}
    D = load(fullfile(od, sprintf('aptrue_est_final_%s_full.mat', tr{1}))); S = D.S; nS = numel(S);
    switch tr{1}; case 'canon'; hold_t = @(t) t > 3.5; case 'meng'; hold_t = @(t) t > 10.5; end
    acc = zeros(nS, 6);
    for q = 1:nS
        s = S(q); n = size(s.x_upd,1); j = (4:n).'; kk = j+1; m = hold_t(s.t(kk));
        x3post = s.x_upd(j-1,3);            % posterior dw3_hat of call k-1 (the height a'[k] is read at)
        x3pri  = s.x_pred(j,3);             % prior dw3_hat of call k
        inn1   = s.innov1(kk);              % y1 innovation of call k
        K31    = s.K31(kk);                 % l31 of call k
        heval  = max(s.hd(kk-1) - x3post, HFLOOR); app = app_num(heval); a1 = s.a_prime(kk);
        c_post = mean((x3post(m)-mean(x3post(m))).*(inn1(m)-mean(inn1(m))));
        c_pri  = mean((x3pri(m)-mean(x3pri(m))).*(inn1(m)-mean(inn1(m))));
        % the update-half product term on the gain row, per step: l41 innov1 with l41 = -a'(w_hat) l31
        % -> estimate increment mean = -l31 * E[a1 innov1]; split: -l31 a1_mean E[inn1] (linear) + (-l31) Cov(a1, inn1)
        l41inn = -K31(m) .* a1(m) .* inn1(m);
        lin    = -mean(K31(m)) * mean(a1(m)) * mean(inn1(m));
        cov_a1 = -mean(K31(m)) * mean((a1(m)-mean(a1(m))).*(inn1(m)-mean(inn1(m))));
        acc(q,:) = [c_post, c_pri, mean(app(m))*mean(K31(m))*c_post, mean(l41inn), lin, cov_a1];
    end
    mu = mean(acc,1); se = std(acc,0,1)/sqrt(nS);
    fprintf('[%s hold] Cov(x3_post[k-1], inn1[k]) %+.3e (SEM %.1e) | Cov(x3_pri[k], inn1[k]) %+.3e (SEM %.1e)\n', tr{1}, mu(1), se(1), mu(2), se(2));
    fprintf('           a'''' l31 Cov(x3_post, inn1) = %+.3f e-6/step (SEM %.3f)  [predicted update-half mean push on the estimate]\n', 1e6*mu(3), 1e6*se(3));
    fprintf('           direct: E[l41 inn1] %+.3f e-6/step (SEM %.3f) = linear l41_0 E[inn1] %+.3f (SEM %.3f) + (-l31) Cov(a1, inn1) %+.3f (SEM %.3f)\n', 1e6*mu(4), 1e6*se(4), 1e6*mu(5), 1e6*se(5), 1e6*mu(6), 1e6*se(6));
end
end
function cp = local_cp(w); [~, cp] = calc_correction_functions(w); end
