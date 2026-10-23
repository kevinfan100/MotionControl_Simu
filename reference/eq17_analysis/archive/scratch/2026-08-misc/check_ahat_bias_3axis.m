% The effective-pole hypothesis predicts a_hat/a from the residual acf alone.
% Fitted lambda_eff = 0.760 on x/y => a_hat/a = (1-lc)/(1-lambda_eff) = 1.250.
% That is a PREDICTION about a quantity the acf fit never saw. Measure it.
root = '/Users/kevin/Code/MotionControl_Simu-motion-test/';
L = load([root 'test_results/formC_cdpmr_var_check/raw_seeds.mat']);
K = L.S400.K;
AH = L.S400.a_bar_hat_out; AT = L.S400.a_true_out; t = L.S400.t(:);
clear L
axl='xyz';
fprintf('\n  axis   window          a/a_o    a_hat/a - 1\n');
for ax=1:3
    H = squeeze(AH(:,ax,:)); T = squeeze(AT(:,ax,:))/K.a_nom;
    H = H(2:end,:); T = T(2:end,:); tt = t(2:end);
    for w = {[0.05 0.45],'hold(far)'; [3.75 4.8],'hold(trough)'}'
    end
    for i=1:2
        if i==1; win=[0.05 0.45]; nm='hold(far)'; else; win=[3.75 4.8]; nm='hold(trough)'; end
        in = tt>=win(1) & tt<=win(2);
        r = mean(mean(H(in,:),2))/mean(mean(T(in,:),2)) - 1;
        fprintf('   %c     %-13s  %6.4f    %+6.2f %%\n', axl(ax), nm, mean(mean(T(in,:),2)), 100*r);
    end
end
fprintf('\n  PREDICTED by the pole fit (x/y): +25.0 %%   [from lambda_eff = 0.760]\n');
