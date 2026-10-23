% meng ch4 §34 discriminator, applied to my item (ii).
% Their case: per-step Var(innov)/S = 0.35 looked like R over-inflated 2.8x;
% cutting the inflation made things WORSE; the truth was that the correlation
% penalty is booked into per-step R, so per-step honesty reads 1/IF while the
% TOTAL account is right -- proven by the innovation acf long-run factor
% matching IF exactly.
%
% PRE-REGISTERED, written before running:
%   IF_innov ~ IF_eff used (2.86 at the trough)  => booking correct, item (ii)
%                                                   is withdrawn, do NOT cut R
%   IF_innov ~ 3.5 (what y2 itself gives)        => item (ii) stands
% Naming, per their advice not to let two quantities share one name:
%   IF_var  = 1 + 2*sum rho(tau)*s^tau   (variance route, what the code uses)
%   IF_smpl = 1 + 2*sum rho(tau)         (sample route, unweighted, on y2)
%   IF_inno = 1 + 2*sum rho_innov(tau)   (what the filter actually consumes)
root = '/Users/kevin/Code/MotionControl_Simu-motion-test/';
S = load([root 'test_results/am_r22_deep/stack_deep400.mat']);
L = load([root 'test_results/formC_cdpmr_var_check/raw_seeds.mat']);
iv = squeeze(L.S400.innov_y2_out(:,3,:)); iv = iv(2:end,:); clear L
a_cov = S.cc.a_cov; A = S.A_wm; T = S.A_tr; R2 = S.R2_u; t = S.t;
Y = A(2:end,:) - (1-a_cov)*A(1:end-1,:);
t2 = t(2:end); iv2 = iv(2:end,:); R2b = R2(2:end,:);
am = mean(T,2); am2 = am(2:end);
acf_int = @(X) local_acf(X, 40);
W = {[0.5 1.5],'descent'; [1.75 3.5],'oscillate'; [3.75 4.8],'final hold'};
fprintf('\n window        a/a_o   IF_inno   IF_smpl(y2)   IF_var used   NIS\n');
for i = 1:3
    in = t2 >= W{i,1}(1) & t2 <= W{i,1}(2);
    IFi = acf_int(iv2(in,:));
    IFs = acf_int(Y(in,:));
    IFu = mean(mean(R2b(in,:),2)) / mean(var(Y(in,:),0,2)); % the inflation actually applied
    nis = mean(mean(iv2(in,:).^2 ./ R2b(in,:), 2));
    fprintf(' %-12s %6.3f   %7.3f   %9.3f   %10.3f   %5.3f\n', W{i,2}, mean(am2(in)), IFi, IFs, IFu, nis);
end
function f = local_acf(X, L)
    Xf = X - mean(X, 2); n = size(Xf,1); r = zeros(1,L);
    for k = 1:L
        a = Xf(1:n-k,:); b = Xf(1+k:n,:);
        a = a(:)-mean(a(:)); b = b(:)-mean(b(:));
        r(k) = (a'*b)/sqrt((a'*a)*(b'*b));
    end
    f = 1 + 2*sum(r);
end
