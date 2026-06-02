%% confirm_IF_eff_convergence.m
% Worst-case row (a_pd=0.005 -> alpha=0.995, a_cov=0.005 -> s=0.995):
% show that relErr between complete closed-form and brute-force shrinks
% as the brute-force truncation length grows -> proves the 1e-7 seen in
% derive_IF_eff_closed.m is brute-force truncation, not closed-form error.

clear; clc;
a_pd = 0.005; lambda = 0.6; a_cov = 0.005; wT = 0.9997;
alpha = 1-a_pd; beta = lambda; c = 1-beta; s = 1-a_cov; wN = 1-wT;

% --- complete closed-form (same as derive_IF_eff_closed.m) ---
RT_cf = arrayfun(@(t) RT_smalltau(t,alpha,beta,c), 0:3)';
RN_cf = [2*RT_cf(1)-2*RT_cf(2); 2*RT_cf(2)-RT_cf(1)-RT_cf(3); 2*RT_cf(3)-RT_cf(2)-RT_cf(4)];
C_dpmr = RT_cf(1); C_n = RN_cf(1);
A_T = alpha^2/(1-alpha^2) - c/((alpha-beta)*(1-alpha^2)) ...
    - alpha^5*c/((1-alpha^2)*(1-alpha*beta)) ...
    + alpha^3*c^2/((alpha-beta)*(1-alpha^2)*(1-alpha*beta));
B_T = alpha^2*c/(beta^2*(alpha-beta)*(1-alpha*beta)) ...
    - alpha^2*c^2*beta/((alpha-beta)*(1-beta^2)*(1-alpha*beta));
A_N = -(1-alpha)^2/alpha*A_T;  B_N = -(1-beta)^2/beta*B_T;
P = wT*(A_T/C_dpmr) + wN*(A_N/C_n);
Q = wT*(B_T/C_dpmr) + wN*(B_N/C_n);
rho1 = wT*(RT_cf(2)/C_dpmr) + wN*(RN_cf(2)/C_n);
rho2 = wT*(RT_cf(3)/C_dpmr) + wN*(RN_cf(3)/C_n);
a2s=alpha^2*s; abs_=alpha*beta*s; b2s=beta^2*s;
tail = P^2*a2s^3/(1-a2s) + 2*P*Q*abs_^3/(1-abs_) + Q^2*b2s^3/(1-b2s);
IF_cf = 1 + 2*(rho1^2*s + rho2^2*s^2 + tail);

% --- brute force at increasing truncation lengths ---
fprintf('IF_complete (closed-form) = %.10f\n\n', IF_cf);
fprintf('%-8s %-8s | %-14s %-12s\n','N','tau_bf','IF_brute','relErr');
for tb = [800 2000 5000 12000]
  N = tb + 4000;
  p1 = alpha.^(1:N)';  p2 = zeros(N,1);
  for i=4:N, m=i-4; p2(i)=alpha*c*(alpha^(m+1)-beta^(m+1))/(alpha-beta); end
  fT = p1-p2;  fN = fT-[0;fT(1:end-1)];
  RT=zeros(tb+1,1); RN=zeros(tb+1,1);
  for tau=0:tb
    RT(tau+1)=sum(fT(1:end-tau).*fT(1+tau:end));
    RN(tau+1)=sum(fN(1:end-tau).*fN(1+tau:end));
  end
  rho = wT*(RT/RT(1)) + wN*(RN/RN(1));
  IF_bf = 1 + 2*sum(rho(2:tb+1).^2 .* (s.^(1:tb)'));
  fprintf('%-8d %-8d | %-14.10f %-12.2e\n', N, tb, IF_bf, abs(IF_bf-IF_cf)/IF_cf);
end

function R = RT_smalltau(tau,alpha,beta,c)
  T1 = alpha^(tau+2)/(1-alpha^2);
  T2 = alpha^(5-tau)*c/((1-alpha^2)*(1-alpha*beta));
  T3 = alpha^(tau+5)*c/((1-alpha^2)*(1-alpha*beta));
  Rg = (alpha^(tau+2)/(1-alpha^2)+beta^(tau+2)/(1-beta^2) ...
       -alpha*beta*(alpha^tau+beta^tau)/(1-alpha*beta))/(alpha-beta)^2;
  R = T1-T2-T3+alpha^2*c^2*Rg;
end
