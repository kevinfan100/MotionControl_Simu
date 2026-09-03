% STATUS: ACTIVE (scratch) | PURPOSE: the term the a'_true probe consumed. r = De - a' u treated
%   a' u as zero-mean, but a' is read at w_bar_d - dw3_hat and dw3_hat shares n_w with u, so
%   Cov(a', u) = -a'' Cov(dw3_hat, u) = -a''(1-lc) K31 R1 -- minus the three second-order
%   feedthrough shares. Reads aptrue_est_final_<traj>_full.mat (probe_aptrue_est_final, kr1_full)
%   and prints per segment: Cov(a1,u), -a''Cov(x3,u), the kr1_full share, E[a1]E[u], E[a1 u].
%   2026-09-03: canon hold +0.736 / +0.736 / -0.694, meng hold +0.746 / +0.747 / -0.674.
function probe_aptrue_cov_a1u()
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
addpath(genpath(fullfile(root, 'model')));  od = fullfile(root, 'test_results', 'apd_acov_meng');
lc = 0.7; al = 1 - lc; HFLOOR = 1.001;
abar = @(w) 1 ./ arrayfun(@(x) local_cp(x), w); hh = 1e-3;
app_num = @(w) (abar(w+hh) - 2*abar(w) + abar(w-hh)) / hh^2;
for tr = {'canon','meng'}
    D = load(fullfile(od, sprintf('aptrue_est_final_%s_full.mat', tr{1}))); S = D.S; nS = numel(S);
    switch tr{1}
        case 'canon'; SEG = {'osc', @(t) t>1.5 & t<=3.5; 'hold', @(t) t>3.5};
        case 'meng';  SEG = {'near', @(t) t>6 & t<=10.5; 'hold', @(t) t>10.5};
    end
    fprintf('[%s] per segment, 1e-6 abar/step (True - Est convention for the drift):\n', tr{1});
    fprintf('%-6s %10s %10s %10s %10s %10s | %10s %10s\n', 'seg', 'cov(a1,u)', '-app*cov(x3,u)', 'app*al*K31R1', 'E[a1]E[u]', 'E[a1 u]', 'kr1_full', 'net(b)+(d)');
    for si = 1:size(SEG,1)
        acc = zeros(nS, 6);
        for q = 1:nS
            s = S(q); n = size(s.x_upd,1); j = (4:n).'; kk = j+1; t = s.t(kk); m = SEG{si,2}(t);
            w = s.h_bar_true; hd = s.hd; x3 = s.x_upd(:,3); x8 = s.x_upd(:,8); x9 = s.x_upd(:,9);
            Dwd = hd(kk) - hd(kk-1); M = Dwd + al*x3(j-1) + al*(x8(j-1)+x9(j-1)); Bt = w(kk) - w(kk-1); u = Bt - M;
            a1 = s.a_prime(kk); heval = max(hd(kk-1) - x3(j-1), HFLOOR); app = app_num(heval);
            R1 = s.R1(j); K31 = s.K31(kk-1); x3p = x3(j-1);
            c_a1u = mean((a1(m)-mean(a1(m))).*(u(m)-mean(u(m))));
            c_x3u = mean((x3p(m)-mean(x3p(m))).*(u(m)-mean(u(m))));
            acc(q,:) = [c_a1u, -mean(app(m))*c_x3u, mean(app(m).*al.*K31(m).*R1(m)), mean(a1(m))*mean(u(m)), mean(a1(m).*u(m)), mean(app(m).*al.*K31(m).*R1(m))];
        end
        mu = 1e6*mean(acc,1); se = 1e6*std(acc,0,1)/sqrt(nS);
        fprintf('%-6s %+10.3f %+10.3f %+10.3f %+10.3f %+10.3f | %+10.3f %+10.3f\n', SEG{si,1}, mu(1), mu(2), mu(3), mu(4), mu(5), mu(6), mu(1)+mu(6));
        fprintf('%-6s %10.3f %10.3f %10.3f %10.3f %10.3f   (SEM)\n', '', se(1), se(2), se(3), se(4), se(5));
    end
end
end
function cp = local_cp(w); [~, cp] = calc_correction_functions(w); end
