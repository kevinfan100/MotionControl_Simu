# FORK OF nothing (new) | PURPOSE: sympy checks for reference/eq17_analysis/derivation/0914_estb_bprime_core.tex
#   (b1 = db/dw as a state) | EXPIRES: when the tex is closed or superseded
import sympy as sp

w, w0, u0, b0, b1 = sp.symbols('w w0 u0 b0 b1')
lc, F, D, eps = sp.symbols('lambda_c F D eps')
ah, bh, bh1 = sp.symbols('ah bh bh1')
e3, ea, eb, eb1, ew = sp.symbols('e3 ea eb eb1 ew')

# ---- Gain law: integrated law and curvature ----
b_of_w = b0 + b1*(w - w0)
u = u0 + b0*(w - w0) + sp.Rational(1, 2)*b1*(w - w0)**2
a = 1 - 1/u
print('d(1/(1-a))/dw - b(w)          :', sp.simplify(sp.diff(1/(1 - a), w) - b_of_w))
print("da/dw - b(1-a)^2               :", sp.simplify(sp.diff(a, w) - b_of_w*(1 - a)**2))
app = sp.diff(b_of_w*(1 - a)**2, w)
print("a'' - [b1(1-a)^2 - 2b^2(1-a)^3]:", sp.simplify(app - (b1*(1 - a)**2 - 2*b_of_w**2*(1 - a)**3)))

# ---- Error rows 4-5: Euler predicts, first order in (e, eD) ----
eD = (1 - lc)*e3 + F*ea + ew
a_true = (ah + eps*ea) + (bh + eps*eb)*(1 - (ah + eps*ea))**2*(D + eps*eD)
a_est = ah + bh*(1 - ah)**2*D
b_true = (bh + eps*eb) + (bh1 + eps*eb1)*(D + eps*eD)
b_est = bh + bh1*D
row4 = sp.expand(sp.diff(a_true - a_est, eps).subs(eps, 0))
row5 = sp.expand(sp.diff(b_true - b_est, eps).subs(eps, 0))
row4_tex = (1 - lc)*bh*(1 - ah)**2*e3 + (1 + bh*(1 - ah)**2*F - 2*bh*(1 - ah)*D)*ea + (1 - ah)**2*D*eb + bh*(1 - ah)**2*ew
row5_tex = (1 - lc)*bh1*e3 + bh1*F*ea + eb + D*eb1 + bh1*ew
print('e_a row residual               :', sp.simplify(row4 - sp.expand(row4_tex)))
print('e_b row residual               :', sp.simplify(row5 - sp.expand(row5_tex)))
# zeroth order must cancel exactly
print('zeroth order (a, b)            :', sp.simplify((a_true - a_est).subs(eps, 0)), sp.simplify((b_true - b_est).subs(eps, 0)))

# ---- F_e rows 4-5 and g from the rows ----
xs = [sp.Symbol('e1'), sp.Symbol('e2'), e3, ea, eb, eb1]
Fe4 = [sp.simplify(sp.diff(row4.subs(ew, 0), s)) for s in xs]
Fe5 = [sp.simplify(sp.diff(row5.subs(ew, 0), s)) for s in xs]
print('F_e row 4                      :', Fe4)
print('F_e row 5                      :', Fe5)
print('g_4, g_5                       :', sp.simplify(sp.diff(row4, ew)), sp.simplify(sp.diff(row5, ew)))

# ---- Measurement: backward Taylor of the law along Grad ----
G = sp.symbols('G')
u_back = u0 - b0*G + sp.Rational(1, 2)*b1*G**2          # u(w - G) with b, b1 at w (here w0 := w)
a_now = 1 - 1/u0
a_back = sp.series(1 - 1/u_back, G, 0, 3).removeO()
claim = a_now - b0*(1 - a_now)**2*G + sp.Rational(1, 2)*(b1*(1 - a_now)**2 - 2*b0**2*(1 - a_now)**3)*G**2
print('a[k-d] Taylor residual         :', sp.simplify(sp.expand(a_back - claim)))
yhat = ah - bh*(1 - ah)**2*G
print('H24, H25, H26                  :', sp.simplify(sp.diff(yhat, ah)), sp.simplify(sp.diff(yhat, bh)), sp.diff(yhat, bh1))

# ---- Seed line ----
wc, ah0, bh10 = sp.symbols('w_c ah0 bh10')
bseed = sp.solve(sp.Eq(1, 1/(1 - ah0) + b0*(wc - w0) + sp.Rational(1, 2)*bh10*(wc - w0)**2), b0)[0]
print('b_hat[0] residual              :', sp.simplify(bseed - ((1/(1 - ah0) - 1)/(w0 - wc) + sp.Rational(1, 2)*bh10*(w0 - wc))))
mid = bseed + bh10*((w0 + wc)/2 - w0)
print('midpoint identity residual     :', sp.simplify(mid - (1/(1 - ah0) - 1)/(w0 - wc)))
print('d b_hat[0] / d b1[0]           :', sp.simplify(sp.diff(bseed, bh10)))
# P block with e_b[0] = e_c + (L/2) e_b1, E[e_c e_b1] = 0
L, Pc, P66 = sp.symbols('L P_c P66')
J = sp.Matrix([[1, L/2], [0, 1]])
Pseed = sp.diag(Pc, P66)
P0 = J*Pseed*J.T
print('P55[0], P56[0], P66[0]         :', sp.expand(P0[0, 0]), P0[0, 1], P0[1, 1])

# ======================= Production blocks =======================
print('--- production blocks ---')
M, u, e4, e5, e6 = sp.symbols('M u e4 e5 e6')
# exact step with b1, Taylor in M
Gx = lambda A, B, B1, S: 1 - 1/(1/(1 - A) + B*S + sp.Rational(1, 2)*B1*S**2)
ser = sp.series(Gx(ah, bh, bh1, M), M, 0, 3).removeO()
print('exact-step Taylor residual     :', sp.simplify(sp.expand(ser - (ah + bh*(1-ah)**2*M + sp.Rational(1, 2)*(bh1*(1-ah)**2 - 2*bh**2*(1-ah)**3)*M**2))))
# second-order mean of the gain row: joint scaling of (e, u, M)
diffG = Gx(ah + eps*e4, bh + eps*e5, bh1 + eps*e6, eps*M + eps*u) - Gx(ah, bh, bh1, eps*M)
s2 = sp.expand(sp.diff(diffG, eps, 2).subs(eps, 0) / 2)
P44, P45, P46, P55, P56, P66, Pu4, Pu5, Pu6, Puu = sp.symbols('P44 P45 P46 P55 P56 P66 Pu4 Pu5 Pu6 Puu')
poly = sp.Poly(s2, e4, e5, e6, u, M)
Emap = {(2,0,0,0,0): P44, (1,1,0,0,0): P45, (1,0,1,0,0): P46, (0,2,0,0,0): P55, (0,1,1,0,0): P56, (0,0,2,0,0): P66,
        (1,0,0,1,0): Pu4, (0,1,0,1,0): Pu5, (0,0,1,1,0): Pu6, (0,0,0,2,0): Puu}
mean2 = 0
for mon, cf in poly.terms():
    if mon in Emap:
        mean2 += cf*Emap[mon]
    # terms linear in e or u, or pure M^2 (deterministic, already in the exact step): zero mean / not a mean term
mean2 = sp.expand(mean2)
claim = -2*bh*(1-ah)*Pu4 + (1-ah)**2*Pu5 + sp.Rational(1, 2)*(bh1*(1-ah)**2 - 2*bh**2*(1-ah)**3)*Puu
print('gain-row mean residual         :', sp.simplify(mean2 - sp.expand(claim)))
# b row
diffB = (bh + eps*e5) + (bh1 + eps*e6)*(eps*M + eps*u) - (bh + bh1*eps*M)
s2b = sp.expand(sp.diff(diffB, eps, 2).subs(eps, 0) / 2)
print('b-row second-order group       :', s2b, '  (mean = Cov(e6,u))')

# ---- vector form against the controller's cov_e*u / var_u (ma2_aug & nw_mcorr branch) ----
P = sp.Matrix(9, 9, lambda i, j: sp.Symbol('P%d%d' % (min(i, j)+1, max(i, j)+1)))
al = 1 - lc
cvec = sp.Matrix([-(1-lc), 0, (1-lc), F, 0, 0, 0, al, al])
Pc = P*cvec
kT, a_i, s2n = sp.symbols('kappa_T a_i s2n')
Pm = lambda i, j: P[i-1, j-1]
cov_e3u = (1-lc)*Pm(3,3) + al*(Pm(3,8) + Pm(3,9)) + F*Pm(3,4) - (1-lc)*Pm(3,1)
cov_e4u = (1-lc)*Pm(3,4) + F*Pm(4,4) + al*(Pm(4,8) + Pm(4,9)) - (1-lc)*Pm(1,4)
cov_e5u = (1-lc)*Pm(3,5) + F*Pm(4,5) + al*(Pm(5,8) + Pm(5,9)) - (1-lc)*Pm(1,5)
cov_e6u = (1-lc)*Pm(3,6) + F*Pm(4,6) + al*(Pm(6,8) + Pm(6,9)) - (1-lc)*Pm(1,6)
var_u = ((1-lc)**2*Pm(3,3) + al**2*(Pm(8,8) + Pm(9,9) + 2*Pm(8,9)) + 2*(1-lc)*al*(Pm(3,8) + Pm(3,9))
         + F**2*Pm(4,4) + 2*F*((1-lc)*Pm(3,4) + al*(Pm(4,8) + Pm(4,9))) + kT*a_i + al**2*s2n
         - al**2*s2n + (1-lc)**2*Pm(1,1) - 2*(1-lc)**2*(Pm(1,3) + Pm(1,8) + Pm(1,9)) - 2*F*(1-lc)*Pm(1,4))
print('(Pc)_3,4,5,6 vs code           :', [sp.simplify(Pc[i] - v) for i, v in zip([2, 3, 4, 5], [cov_e3u, cov_e4u, cov_e5u, cov_e6u])])
print("c'Pc + kT a vs code var_u      :", sp.simplify((cvec.T*P*cvec)[0] + kT*a_i - var_u))

# ---- F_e rows 3-6 in vector form against the controller's construction ----
ap, Aa = sp.symbols('ap A_a')
Fe = sp.zeros(9, 9)
Fe[0, 1] = 1; Fe[1, 2] = 1
Fe[2, 2] = lc; Fe[2, 3] = -F
Fe[3, 2] = (1-lc)*ap; Fe[3, 3] = 1 + ap*F + Aa*M; Fe[3, 4] = (1-ah)**2*M
Fe[4, 2] = (1-lc)*bh1; Fe[4, 3] = F*bh1; Fe[4, 4] = 1; Fe[4, 5] = M     # new row 5
Fe[5, 5] = 1
Fe[2, 7] = -al; Fe[2, 8] = -al; Fe[3, 7] = ap*al; Fe[3, 8] = ap*al
Fe[4, 7] = bh1*al; Fe[4, 8] = bh1*al                                    # new MA(2) columns on row 5
Fe[8, 7] = 1
gn_c = sp.Matrix([0, 0, -(1-lc), ap*(1-lc), bh1*(1-lc), 0, 0, 0, 0])      # nw_mcorr column (row 5 entry new)
Fe[:, 0] = Fe[:, 0] - gn_c
E = sp.eye(9)
row3 = E[2, :] - cvec.T
row4 = E[3, :] + ap*cvec.T + Aa*M*E[3, :] + (1-ah)**2*M*E[4, :]
row5 = E[4, :] + bh1*cvec.T + M*E[5, :]
print('F_e row 3/4/5 vector-form resid:', sp.simplify(Fe[2, :] - row3), sp.simplify(Fe[3, :] - row4), sp.simplify(Fe[4, :] - row5))
# Q: thermal + n_w shares, minus the nw_mcorr share
gT = sp.Matrix([0, 0, -1, ap, bh1, 0, 0, 1, 0]); gn = sp.Matrix([0, 0, -al, ap*al, bh1*al, 0, 0, 0, 0])
Qcode = kT*a_i*gT*gT.T + s2n*gn*gn.T - s2n*gn_c*gn_c.T
print('Q vs kT a gT gT\' residual       :', sp.simplify(Qcode - kT*a_i*gT*gT.T))
