% F_e(4,3) = (1-lc)*a'(a_hat) + J_d3*M. J_d3 is nonzero only for ap_src='act',
% and this family runs ap_src='post' (confirmed via K.ap_src). So on the runs
% we have, F_e(4,3) = (1-lc)*a_prime EXACTLY -- deterministic, no free knob.
% It is structurally NON-NEGATIVE (a' = b(1-a)^2 >= 0 for a<1, b>0, and
% 1-lc > 0), so it can never itself flip sign; it only PROPAGATES whatever
% correlation already exists in P(3,2) (a pure position/tracking-error
% cross-covariance, no gain-error content of its own) into the gain row.
root = '/Users/kevin/Code/MotionControl_Simu-motion-test/';
S = load([root 'test_results/am_r22_deep/baseline_budget_100.mat']);
ax = 3;
fprintf('ap_src used in this run: %s\n', S.K.ap_src);
ap = squeeze(S.a_prime_out(:,ax,:)) / S.a_nom; ap = ap(2:end,:);   % undo a_o*R scaling (a_prime_hat = a_bar'*a_o, driver x R_drv)
t  = S.t(2:end);
Fe43 = (1-S.K.lambda_c) * ap;
W = {[0.05 0.45],'hold(far)'; [0.5 1.5],'descend'; [1.75 3.5],'oscillate'; [3.75 4.8],'hold(trough)'};
fprintf('\n  window        mean F_e(4,3)   min    max   (all >=0 ? %d)\n', all(Fe43(:)>=-1e-12));
for i=1:4
    in = t>=W{i,1}(1) & t<=W{i,1}(2);
    v = Fe43(in,:);
    fprintf('  %-12s   %8.5f     %8.5f  %8.5f\n', W{i,2}, mean(v(:)), min(v(:)), max(v(:)));
end
