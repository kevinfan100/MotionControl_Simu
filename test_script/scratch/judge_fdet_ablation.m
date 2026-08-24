% Judge use_fdet=false against the pre-registered baseline numbers, PAIRED
% by seed (both arms use seeds 1:100 on the same trajectory/noise draw --
% RNG seeded per-seed in the driver, so this is a fair pairing).
root = '/Users/kevin/Code/MotionControl_Simu-motion-test/';
BASE = load([root 'test_results/am_r22_deep/baseline_budget_100.mat']);
OFF  = load([root 'test_results/am_r22_deep/fdet_off_budget_100.mat']);
ax = 3;

Wd = [0.5 1.5]; Wh = [3.75 4.8];
[legB,~,cB,shB,ekB,~]      = local_decomp(BASE, ax, Wd);
[legO,~,cO,shO,ekO,~]      = local_decomp(OFF,  ax, Wd);
fprintf('\n===== use_fdet ablation, axis z, paired 100 seeds =====\n');
fprintf('DESCENT window:\n');
fprintf('  baseline (fdet=ON) : leg %+9.5f  Cov share %5.1f%%  E[K1] %+.4f\n', legB, shB, ekB);
fprintf('  fdet=OFF           : leg %+9.5f  Cov share %5.1f%%  E[K1] %+.4f\n', legO, shO, ekO);

[~,~,~,~,~,biasBh] = local_decomp(BASE, ax, Wh);
[~,~,~,~,~,biasOh] = local_decomp(OFF,  ax, Wh);
fprintf('\nFINAL HOLD bias (a_hat - a_true, absolute a_bar units):\n');
fprintf('  baseline (fdet=ON) : %+.5f\n', biasBh);
fprintf('  fdet=OFF           : %+.5f\n', biasOh);

AHb = squeeze(BASE.a_bar_hat_out(:,ax,:)); AHb=AHb(2:end,:);
ATb = squeeze(BASE.a_true_out(:,ax,:))/BASE.a_nom; ATb=ATb(2:end,:);
AHo = squeeze(OFF.a_bar_hat_out(:,ax,:));  AHo=AHo(2:end,:);
ATo = squeeze(OFF.a_true_out(:,ax,:))/OFF.a_nom;   ATo=ATo(2:end,:);
tB = BASE.t(2:end); tO = OFF.t(2:end);
biasB_seed = mean(AHb(tB>=3.75,:) - ATb(tB>=3.75,:), 1);
biasO_seed = mean(AHo(tO>=3.75,:) - ATo(tO>=3.75,:), 1);
d = biasO_seed - biasB_seed;
fprintf('\nPAIRED diff (off - baseline) in final-hold bias: mean %+.5f, SEM %.5f, t = %+.2f\n', ...
    mean(d), std(d)/sqrt(numel(d)), mean(d)/(std(d)/sqrt(numel(d))));
fprintf('as %% of trough a_true: %+.1f pp\n', 100*mean(d)/mean(ATb(tB>=3.75,:),'all'));

function [leg,term1,covterm,share,ek,ehold_bias] = local_decomp(S, ax, W)
    I1 = squeeze(S.innov_y1_out(:,ax,:)); I1 = I1(2:end,:);
    K1 = squeeze(S.K_a_y1_out(:,ax,:));   K1 = K1(2:end,:);
    t  = S.t(2:end);
    AH = squeeze(S.a_bar_hat_out(:,ax,:)); AH = AH(2:end,:);
    AT = squeeze(S.a_true_out(:,ax,:))/S.a_nom; AT = AT(2:end,:);
    in = t>=W(1) & t<=W(2);
    k = K1(in,:); v = I1(in,:);
    leg = mean(sum(k.*v,1));
    n = sum(in); ek = mean(k(:)); ev = mean(v(:));
    term1 = n*ek*ev; covterm = leg-term1; share = 100*covterm/leg;
    ehold_bias = mean(AH(end,:)-AT(end,:));
end
