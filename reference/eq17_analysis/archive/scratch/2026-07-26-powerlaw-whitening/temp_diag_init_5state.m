% temp_diag_init_5state.m (scratch) — decisive bug-vs-setting check on the early
% a_hat transient (z-axis). If: (1) a_xm is unbiased ~chi-sq noise, (2) a_hat is a
% SMOOTHED a_xm, (3) sqrt(P_a) shrinks, (4) K(a<-y2) drops -> pure KF cold-start,
% NOT a derivation/implementation bug.
here = fileparts(mfilename('fullpath'));
S = load(fullfile(here, 'temp_smoke_5state_powerlaw.mat'));
sim = S.sim;
t = sim.tout(:);
ao  = sim.a_true_out(1,3);
azh = sim.a_hat_out(:,3);  axm = sim.a_xm_out(:,3);  Pa = sim.P_a_out(:,3);
Ka  = sim.K_a_y2_out(:,3); azt = sim.a_true_out(:,3);

w = t < 0.5;   % far-field hold: a_true is constant = a_o
fprintf('\n== z-axis, far-field hold (t<0.5, a_true=a_o=%.4g const) ==\n', ao);
fprintf('(1) a_xm unbiased?   mean/a_o=%.3f (->1)   rel std=%.3f (chi-sq floor sqrt(a_cov*IF)=%.2f)\n', ...
        mean(axm(w))/ao, std(axm(w))/ao, sqrt(0.05*4.2));
fprintf('(2) a_hat smoother?  std(a_hat)/a_o=%.3f  vs std(a_xm)/a_o=%.3f  (a_hat << a_xm => smoothing)\n', ...
        std(azh(w))/ao, std(axm(w))/ao);
fprintf('(3) sqrt(P_a) shrinks?  t=0:%.3g  t=0.1:%.3g  t=0.3:%.3g  t=0.5:%.3g  (/a_o)\n', ...
        Pa(1)/ao, Pa(find(t>=0.1,1))/ao, Pa(find(t>=0.3,1))/ao, Pa(find(t>=0.5,1))/ao);
fprintf('(4) K(a<-y2) drops?     t=0:%.3g  t=0.1:%.3g  t=0.3:%.3g  t=0.5:%.3g\n', ...
        Ka(1), Ka(find(t>=0.1,1)), Ka(find(t>=0.3,1)), Ka(find(t>=0.5,1)));

% is the spike within the window where P_a is still large?
[mx, imx] = max(azh(t<0.3));
fprintf('\nspike: max a_hat/a_o=%.3f at t=%.3fs; sqrt(P_a)/a_o there=%.3g, K=%.3g\n', ...
        mx/ao, t(imx), Pa(imx)/ao, Ka(imx));
% predict cannot move a_hat far-field (s_h=0): confirm a_prime_hat ~ 0 in hold
fprintf('a_prime_hat(z) in hold: max|.|=%.3g  (=0 => predict inert; motion is measurement-only)\n', ...
        max(abs(sim.a_prime_out(w,3))));
