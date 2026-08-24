% If y1's own R1/S1/whiteness are honest (just measured), the +19% push must
% ride the CROSS-COVARIANCE P(4,1), i.e. K1(4), not the innovation itself.
% Decompose the y1 leg per window: E[K1]*E[i1] (systematic gain x systematic
% innovation) vs Cov(K1,i1) (co-movement / rectification), per the sibling's
% method on the descent.
root = '/Users/kevin/Code/MotionControl_Simu-motion-test/';
S = load([root 'test_results/am_r22_deep/baseline_budget_100.mat']);
ax = 3;
I1 = squeeze(S.innov_y1_out(:,ax,:)); I1 = I1(2:end,:);
K1 = squeeze(S.K_a_y1_out(:,ax,:));   K1 = K1(2:end,:);
P41= squeeze(S.P41_out(:,ax,:));      P41= P41(2:end,:);
t  = S.t(2:end);
W = {[0.05 0.45],'hold(far)'; [0.5 1.5],'descend'; [1.75 3.5],'oscillate'; [3.75 4.8],'hold(trough)'};
fprintf('\n  window       y1 leg total    E[K1]*E[i1]   Cov(K1,i1)  Cov share   E[K1]        median P41\n');
for i=1:4
    in = t>=W{i,1}(1) & t<=W{i,1}(2);
    k = K1(in,:); v = I1(in,:); p = P41(in,:);
    leg   = mean(sum(k.*v,1));
    ek = mean(k(:)); ev = mean(v(:));
    term1 = numel(k(:))/size(k,2) * ek*ev;    % E[K]E[i] * n_steps, matched scale
    n = sum(in);
    term1 = n*ek*ev;
    covterm = leg - term1;
    fprintf('  %-12s  %+9.5f      %+9.5f    %+9.5f   %6.1f%%    %+8.5f    %+.4g\n', ...
        W{i,2}, leg, term1, covterm, 100*covterm/leg, ek, median(p(:)));
end
