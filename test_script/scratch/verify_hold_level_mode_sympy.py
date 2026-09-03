# STATUS: ACTIVE (scratch) | PURPOSE: independent sympy check (2026-09-03, research-assistant agent) of the hold level-mode steady state in derivation/0902_formC_aptrue_4state.tex (section Hold level mode): kappa, s, D, E, the two-step-delay denominator and the stationarity collapse. Zero residual. Not part of the MATLAB suite.

import sympy as sp
m3,m4,m2,lc,F,g,r,ap,l21,l31,l32,l41,l42,gam,b2 = sp.symbols('mu3 mu4 mu2 lc F g r ap l21 l31 l32 l41 l42 gam b2')
m3p = lc*m3 - F*m4 - g
m4p = m4 + ap*(1-lc)*m3 + ap*F*m4 + ap*g + r
i2  = gam*m4p + b2

# ---- Task 1: no delay, iota1 = mu3^- ----
i1 = m3p
eqA = sp.Eq(m3, m3p - l31*i1 - l32*i2)
eqB = sp.Eq(m4, m4p - l41*i1 - l42*i2)
sol = sp.solve([eqA,eqB],[m3,m4],dict=True)[0]
mu4 = sp.together(sol[m4])

D = 1-(1-l31)*lc
E = l41 + (1-l42*gam)*ap*l31
kappa_me = l42*gam - E*F/D
s_me = E*g/D + (1-l42*gam)*r - l42*b2 + l32*b2*(l41*lc - (1-l42*gam)*ap*(1-lc))/D
print("Task1 (l32 first order, drop l32*gam):", sp.simplify((mu4 - s_me/kappa_me).subs(l32*gam,0)))
# exact with l32=0
print("Task1 exact l32=0:", sp.simplify(mu4.subs(l32,0) - (s_me/kappa_me).subs(l32,0)))

kappa_claim = l42*gam - ap*F + (l41 - ap*(1-lc))*(1-l31)*F/D
s_claim = ap*g + r - l42*b2 + (l41 - ap*(1-lc))*((1-l31)*g + l32*b2)/D

vals = dict(lc=0.7, ap=0.83, l31=0.66, l41=-0.51, l42=1.5e-3, gam=0.05*0.68, l32=0)
for Fv in (0, 2.6e-4):
    v = dict(vals, F=Fv)
    km = float(kappa_me.subs(v)); kc = float(kappa_claim.subs(v))
    cg_me = float(sp.diff(s_me,g).subs(v)); cg_cl = float(sp.diff(s_claim,g).subs(v))
    print(f"F={Fv}: kappa_me={km:.3e} (tau={1/km:.0f} steps)  kappa_claim={kc:.3e}  g-coef me={cg_me:.4f} claim={cg_cl:.4f}")
print("E =", float(E.subs(vals)), " D =", float(D.subs(vals)))

# ---- Task 3: delay chain, iota1 = mu2 (posterior slot 2), l22=l12=0 ----
i1d = m2
eq2 = sp.Eq(m2, m3 - l21*i1d)
eqA = sp.Eq(m3, m3p - l31*i1d - l32*i2)
eqB = sp.Eq(m4, m4p - l41*i1d - l42*i2)
sold = sp.solve([eq2,eqA,eqB],[m2,m3,m4],dict=True)[0]
mu4d = sp.together(sold[m4])
Dpp = (1+l21)*(1-lc) + l31
kappa_d = l42*gam - E*F/Dpp
s_d = E*g/Dpp + (1-l42*gam)*r - l42*b2
print("Task3 check (l32=0):", sp.simplify(mu4d.subs(l32,0) - s_d/kappa_d))
for l21v in (0.0, 0.66, 0.81, 1.0):
    for Fv in (0, 2.6e-4):
        v = dict(vals, F=Fv, l21=l21v)
        kd = float(kappa_d.subs(v)); cg = float(sp.diff(s_d,g).subs(v)); km=float(kappa_me.subs(v))
        print(f"  l21={l21v}: F={Fv}: kappa_delay={kd:.3e} (vs {km:.3e}, {100*(kd/km-1):+.0f}%)  g-coef={cg:.4f} (vs {float(sp.diff(s_me,g).subs(v)):.4f}, {100*(cg/float(sp.diff(s_me,g).subs(v))-1):+.0f}%)")
