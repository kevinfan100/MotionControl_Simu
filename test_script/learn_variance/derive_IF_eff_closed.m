%% derive_IF_eff_closed.m
% Complete geometric closed-form for IF_eff (derivation_IF_eff.md §8, full version).
%
%   IF_eff = 1 + 2*[ rho^2(1)*s + rho^2(2)*s^2
%                  + P^2*(a2s)^3/(1-a2s)
%                  + 2PQ*(abs)^3/(1-abs)
%                  + Q^2*(b2s)^3/(1-b2s) ]
%
% rho(1),rho(2) use EXACT small-tau values; the tail (tau>=3) is the pure
% geometric series of rho(tau)=P*alpha^tau+Q*beta^tau. No truncation, no
% small-tau approximation -> should match brute-force to machine precision.
%
% Also computes the legacy "pure geometric" approx (geometric from tau=1)
% to quantify the small-tau approximation error it carries.

clear; clc;

a_pd_list   = [0.005 0.05 0.2 0.5];
lambda_list = [0.6 0.75 0.9];
a_cov_list  = [0.005 0.05 0.2];
wT_list     = [0.5 0.9 0.9997 1.0];

N      = 6000;   % impulse-response length (brute force)
tau_bf = 800;    % brute-force IF_eff sum length

fprintf('%-7s %-6s %-7s %-8s | %-13s %-13s %-13s | %-10s %-10s\n', ...
  'a_pd','lam','a_cov','w_T','IF_brute','IF_complete','IF_pure_geo', ...
  'relErr_cf','relErr_pg');
fprintf('%s\n', repmat('-',1,100));

maxrel_cf = 0; maxrel_pg = 0;
for a_pd = a_pd_list
 for lambda = lambda_list
  for a_cov = a_cov_list
   for wT = wT_list
     [IF_bf, IF_cf, IF_pg] = compute_all(a_pd, lambda, a_cov, wT, N, tau_bf);
     r_cf = abs(IF_cf - IF_bf)/IF_bf;
     r_pg = abs(IF_pg - IF_bf)/IF_bf;
     maxrel_cf = max(maxrel_cf, r_cf);
     maxrel_pg = max(maxrel_pg, r_pg);
     fprintf('%-7.3f %-6.2f %-7.3f %-8.4f | %-13.6f %-13.6f %-13.6f | %-10.2e %-10.2e\n', ...
       a_pd, lambda, a_cov, wT, IF_bf, IF_cf, IF_pg, r_cf, r_pg);
   end
  end
 end
end
fprintf('%s\n', repmat('-',1,100));
fprintf('MAX rel err  complete closed-form : %.3e\n', maxrel_cf);
fprintf('MAX rel err  pure-geometric (sec8): %.3e\n', maxrel_pg);

%% ===================== local functions =====================
function [IF_bf, IF_cf, IF_pg] = compute_all(a_pd, lambda, a_cov, wT, N, tau_bf)
  alpha = 1 - a_pd;  beta = lambda;  c = 1 - beta;  s = 1 - a_cov;  wN = 1 - wT;

  % ---------- brute force ----------
  p1 = alpha.^(1:N)';                       % p1[n] = alpha^(n+1), index i=n+1
  p2 = zeros(N,1);
  for i = 4:N                               % p2[n] nonzero for n>=3 (i>=4)
    m = i - 4;
    g = (alpha^(m+1) - beta^(m+1)) / (alpha - beta);
    p2(i) = alpha * c * g;
  end
  fT = p1 - p2;
  fN = fT - [0; fT(1:end-1)];

  RT = zeros(tau_bf+2,1);  RN = zeros(tau_bf+2,1);
  for tau = 0:tau_bf+1
    RT(tau+1) = sum(fT(1:end-tau) .* fT(1+tau:end));
    RN(tau+1) = sum(fN(1:end-tau) .* fN(1+tau:end));
  end
  rho_bf = wT*(RT/RT(1)) + wN*(RN/RN(1));
  IF_bf  = 1 + 2*sum( rho_bf(2:tau_bf+1).^2 .* (s.^(1:tau_bf)') );

  % ---------- complete closed-form ----------
  RT_cf = arrayfun(@(t) RT_smalltau(t,alpha,beta,c), 0:3)';   % R_T(0..3)
  RN_cf = zeros(3,1);
  RN_cf(1) = 2*RT_cf(1) - RT_cf(2) - RT_cf(2);   % R_N(0): R_T(-1)=R_T(1)
  RN_cf(2) = 2*RT_cf(2) - RT_cf(1) - RT_cf(3);   % R_N(1)
  RN_cf(3) = 2*RT_cf(3) - RT_cf(2) - RT_cf(4);   % R_N(2)

  C_dpmr = RT_cf(1);  C_n = RN_cf(1);

  A_T = alpha^2/(1-alpha^2) ...
      - c/((alpha-beta)*(1-alpha^2)) ...
      - alpha^5*c/((1-alpha^2)*(1-alpha*beta)) ...
      + alpha^3*c^2/((alpha-beta)*(1-alpha^2)*(1-alpha*beta));
  B_T = alpha^2*c/(beta^2*(alpha-beta)*(1-alpha*beta)) ...
      - alpha^2*c^2*beta/((alpha-beta)*(1-beta^2)*(1-alpha*beta));
  A_N = -(1-alpha)^2/alpha * A_T;
  B_N = -(1-beta)^2 /beta  * B_T;

  a_T = A_T/C_dpmr;  b_T = B_T/C_dpmr;
  a_N = A_N/C_n;     b_N = B_N/C_n;
  P = wT*a_T + wN*a_N;
  Q = wT*b_T + wN*b_N;

  rho1 = wT*(RT_cf(2)/C_dpmr) + wN*(RN_cf(2)/C_n);
  rho2 = wT*(RT_cf(3)/C_dpmr) + wN*(RN_cf(3)/C_n);

  a2s = alpha^2*s;  abs_ = alpha*beta*s;  b2s = beta^2*s;
  tail = P^2*a2s^3/(1-a2s) + 2*P*Q*abs_^3/(1-abs_) + Q^2*b2s^3/(1-b2s);
  IF_cf = 1 + 2*( rho1^2*s + rho2^2*s^2 + tail );

  % ---------- legacy pure-geometric (geometric from tau=1) ----------
  tail_pg = P^2*a2s/(1-a2s) + 2*P*Q*abs_/(1-abs_) + Q^2*b2s/(1-b2s);
  IF_pg = 1 + 2*tail_pg;
end

function R = RT_smalltau(tau, alpha, beta, c)
  % R_T(tau), tau = 0..3, via T1 - T2 - T3 + T4 (small-tau branch of T2)
  T1 = alpha^(tau+2)/(1-alpha^2);
  T2 = alpha^(5-tau)*c/((1-alpha^2)*(1-alpha*beta));        % tau<=3 branch
  T3 = alpha^(tau+5)*c/((1-alpha^2)*(1-alpha*beta));
  Rg = ( alpha^(tau+2)/(1-alpha^2) + beta^(tau+2)/(1-beta^2) ...
       - alpha*beta*(alpha^tau+beta^tau)/(1-alpha*beta) ) / (alpha-beta)^2;
  T4 = alpha^2*c^2*Rg;
  R  = T1 - T2 - T3 + T4;
end
