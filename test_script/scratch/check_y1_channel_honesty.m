% y1 channel audit, z axis only (x/y deferred per user instruction).
% Three things nobody has checked today, despite y1 doing 96% of a_hat's
% update work in the error budget:
%   1. E[innov_y1] -- is the position-channel innovation biased?
%   2. whiteness of innov_y1 -- a model error would leave correlation
%   3. honesty: Var(innov_y1) / S1, where S1 is recovered from the filter's
%      own K1(4) = P(4,1)/S1 identity (no need to log P11 separately)
root = '/Users/kevin/Code/MotionControl_Simu-motion-test/';
S = load([root 'test_results/am_r22_deep/baseline_budget_100.mat']);
K = S.K; ax = 3; R = 2.25;   % z axis
R1 = K.sigma2_n_s(ax)/R^2;   % R(1,1), normalized

I1 = squeeze(S.innov_y1_out(:,ax,:)); I1 = I1(2:end,:);
K1 = squeeze(S.K_a_y1_out(:,ax,:));   K1 = K1(2:end,:);
P41= squeeze(S.P41_out(:,ax,:));      P41= P41(2:end,:);
t  = S.t(2:end);

% S1 recovered from the K1(4) = P41/S1 identity. Guard tiny K1.
mask = abs(K1) > 1e-6;
S1 = nan(size(K1));
S1(mask) = P41(mask) ./ K1(mask);

W = {[0.05 0.45],'hold(far)'; [0.5 1.5],'descend'; [1.75 3.5],'oscillate'; [3.75 4.8],'hold(trough)'};
fprintf('\n===== y1 channel audit, axis z, R1 = %.5g =====\n', R1);
fprintf('  window        E[innov1]        t-stat     Var(innov1)/S1(honesty)   median S1\n');
for i = 1:4
    in = t>=W{i,1}(1) & t<=W{i,1}(2);
    v  = I1(in,:);
    m  = mean(v(:));
    se = std(mean(v,2))/sqrt(sum(in));    % rough SEM across time-averaged per-seed? use seed-mean
    % proper: per-seed time-mean, then across-seed SEM
    per_seed = mean(v,1);
    m2 = mean(per_seed); se2 = std(per_seed)/sqrt(numel(per_seed));
    honesty = mean(v(:).^2) / mean(S1(in,:),'all','omitnan');
    fprintf('  %-12s  %+10.6f  t=%+7.2f    %8.3f                  %.4g\n', ...
        W{i,2}, m2, m2/se2, honesty, mean(S1(in,:),'all','omitnan'));
end

% whiteness: acf of innov_y1 in the trough hold (stationary)
ih = t>=3.75;
X = I1(ih,:); Xf = X - mean(X,2); n = size(Xf,1);
rho = zeros(1,10);
for k = 1:10
    a = Xf(1:n-k,:); b = Xf(1+k:n,:); a=a(:)-mean(a(:)); b=b(:)-mean(b(:));
    rho(k) = (a'*b)/sqrt((a'*a)*(b'*b));
end
fprintf('\n  innov_y1 acf (trough hold), tau=1..10:\n  '); fprintf('%+.3f ', rho); fprintf('\n');
fprintf('  R1 (sensor floor, always in R) = %.5g ; typical S1 (trough) = %.5g -> P11 share = %.1f%%\n', ...
        R1, mean(S1(ih,:),'all','omitnan'), 100*(1-R1/mean(S1(ih,:),'all','omitnan')));
