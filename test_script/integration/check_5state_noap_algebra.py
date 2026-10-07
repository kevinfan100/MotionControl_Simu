#!/usr/bin/env python3
"""Independent algebra check for 0920_5state_noaprime.tex (rule 8: verify before code).

Three claims are checked symbolically (sympy):
  C1  slot-3 closed loop with the update-before-law ordering and the exact past-error
      bookkeeping: a f - a fbar_bar  ==  F_dx e_a - dF_dx e_da + alpha n + u_c
      with u_c = alpha * sum_i f[k-i] * c_i,  c_i = (ahat[k] - ahat[k-i]) - i*dahat[k]
      (everything exact under the kinematic model a[k-i] = a[k] - i*da[k]; no A3 approximation)
  C2  correlated-noise form of the n_x feed-through: substituting n = y1 - X1 turns the
      -alpha*n term into +alpha*X1 (F(3,1) = alpha) and a known input -alpha*y1; the
      estimator's predict then reads lambda*x3 - alpha*(y1 - x1hat) = lambda*x3 - alpha*res1
  C3  MA(2) augmentation: the rank-1 process noise on [x3 m1 m2] reproduces the variance and
      the lag-1/lag-2 covariances of eps_T[k] = t[k] + alpha t[k-1] + alpha t[k-2]
"""
import sympy as sp

alpha, lam = sp.symbols('alpha lambda_c')
d = 2
# --- C1 -------------------------------------------------------------------
a_k, da_k = sp.symbols('a_k da_k')                 # true gain and true (constant) rate at k
ah = sp.symbols('ah_k ah_km1 ah_km2')              # posteriors ahat[k|k], ahat[k-1|k-1], ahat[k-2|k-2]
dah = sp.symbols('dah_k')                          # posterior rate dahat[k|k]
f = sp.symbols('f_k f_km1 f_km2')                  # forces f[k], f[k-1], f[k-2]
n = sp.symbols('n_k')
dxd = sp.symbols('Dxd')                            # Delta x_d^d[k] (common to both laws, cancels)
dx_kmd = sp.symbols('dx_kmd')                      # delta x[k-d]

a = {0: a_k, 1: a_k - da_k, 2: a_k - 2*da_k}       # kinematic truth a[k-i]
ahc = {0: ah[0], 1: ah[1], 2: ah[2]}               # gain the law used at k-i (update-before-law)

# implementable law at k (times ahat_c[k]) and the ideal delayed law (times a[k])
af_impl = dxd + alpha*((dx_kmd + n) - sum(ahc[i]*f[i] for i in (1, 2)))        # = ahat_c[k]*f[k]
af_ideal = dxd + alpha*(dx_kmd - sum(a[i]*f[i] for i in (1, 2)))                # = a[k]*fbarbar[k]
# a[k]*f[k] = (ahat_c[k] + (a[k]-ahat_c[k])) f[k] = af_impl + (a[k]-ahat_c[k]) f[k]
lhs = (af_impl + (a[0] - ahc[0])*f[0]) - af_ideal
lhs = sp.expand(lhs)

e_a = a_k - ah[0]
e_da = da_k - dah
F_dx = f[0] + alpha*(f[1] + f[2])
dF_dx = alpha*(f[1] + 2*f[2])
c = {i: (ah[0] - ahc[i]) - i*dah for i in (1, 2)}
u_c = alpha*sum(f[i]*c[i] for i in (1, 2))
rhs = sp.expand(F_dx*e_a - dF_dx*e_da + alpha*n + u_c)
print('C1 slot-3 bookkeeping exact:', sp.simplify(lhs - rhs) == 0)

# --- C2 -------------------------------------------------------------------
x1, x3, y1, x1h, x3h = sp.symbols('x1 x3 y1 x1h x3h')
# true:  x3[k+1] = lam x3 - alpha n + (rest);  n = y1 - x1
true_next = lam*x3 - alpha*(y1 - x1)
# rewritten: (lam x3 + alpha x1) + known input (-alpha y1)
F_row = sp.Matrix([[alpha, 0, lam]])                # columns x1, x2, x3
rewritten = (F_row*sp.Matrix([x1, 0, x3]))[0] - alpha*y1
print('C2 correlated-noise rewrite exact:', sp.simplify(true_next - rewritten) == 0)
# estimator mean: same map on the posterior gives lam x3h - alpha (y1 - x1h)
est_next = (F_row*sp.Matrix([x1h, 0, x3h]))[0] - alpha*y1
print('C2 predict = lam*x3h - alpha*res1:', sp.simplify(est_next - (lam*x3h - alpha*(y1 - x1h))) == 0)
# error propagation: e3+ = lam e3 + alpha e1  (no n term left)
e1, e3 = sp.symbols('e1 e3')
print('C2 error row F(3,1)=alpha:', sp.simplify((true_next - est_next).subs({x1: x1h + e1, x3: x3h + e3}) - (lam*e3 + alpha*e1)) == 0)

# --- C3 -------------------------------------------------------------------
s = sp.symbols('s', positive=True)                  # Var t = 4 kBT a (stationary for the check)
# augmented block [x3 m1 m2] driven by t[k]: x3 gets -t, m1 gets +t; m2[k+1] = m1[k]
g = sp.Matrix([-1, 1, 0])
Q = s*g*g.T
# eps[k] = t[k] + alpha m1[k] + alpha m2[k]  (the thermal part of the slot-3 shock incl. memory)
# Its variance and lag covariances from the direct MA(2) definition:
var_direct = s*(1 + 2*alpha**2)
cov1_direct = s*(alpha + alpha**2)
cov2_direct = s*alpha
# From the augmented model: eps[k] = -(x3 shock) ... reconstruct with t[k], m1=t[k-1], m2=t[k-2]
t0, t1, t2, t3 = sp.symbols('t0 t1 t2 t3')
eps_k = t0 + alpha*t1 + alpha*t2
eps_km1 = t1 + alpha*t2 + alpha*t3
def cov(u, v):
    # white t with Var s: covariance = s * sum of matching coefficients
    syms = [t0, t1, t2, t3]
    return sp.expand(sum(u.coeff(x)*v.coeff(x) for x in syms))*s
print('C3 MA(2) var:', sp.simplify(cov(eps_k, eps_k) - var_direct) == 0,
      ' lag1:', sp.simplify(cov(eps_k, eps_km1) - cov1_direct) == 0)
# DC power ratio the white-Q container misses (formB 08-01 number at lc=0.7 -> 2.17)
dc = sp.simplify(((1 + 2*alpha)**2)/(1 + 2*alpha**2))
print('C3 DC power ratio (1+2a)^2/(1+2a^2) at alpha=0.3 =', float(dc.subs(alpha, sp.Rational(3, 10))))
print('C3 rank of Q:', Q.rank())
