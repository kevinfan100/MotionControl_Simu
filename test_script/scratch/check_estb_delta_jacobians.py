"""PURPOSE (2026-09-20): independent sympy check of the offset (delta) law closed forms used by 0920_estb_delta_law.tex
before any controller code (derivation-workflow rule 8).
    a(w; delta) = D(u),  u = 1/((w - w_s) + delta),  D = the plane polynomial (calc_correction_functions.m:81-82)
Checks: a', a'', da/ddelta = a', da/dw_s = -a', da'/ddelta = a'', the three endpoint facts, the autonomous form Lambda(a)
being delta-free, dLambda/da = a''/a', the B-row chain rule d a_d / d a = Lambda(a_d)/Lambda(a), the error-dynamics
linearisation e_a = a'(e_delta - e_3), and the two 2-state det O_2 closed forms of the Observability section.
Prints PASS/FAIL per closed form. Run: python3 test_script/scratch/check_estb_delta_jacobians.py
"""
import sympy as sp

w, ws, dl, u, M, nab = sp.symbols('w w_s delta u M nabla', real=True)
lam_c, F_dw = sp.symbols('lambda_c F_dw', real=True)

D = lambda x: 1 - sp.Rational(9, 8)*x + sp.Rational(1, 2)*x**3 - sp.Rational(57, 100)*x**4 + sp.Rational(1, 5)*x**5 \
              + sp.Rational(7, 200)*x**11 - sp.Rational(1, 25)*x**12
Dp = lambda x: sp.diff(D(u), u).subs(u, x)
Dpp = lambda x: sp.diff(D(u), u, 2).subs(u, x)

g = (w - ws) + dl
a = D(1/g)
ap = sp.diff(a, w)
app = sp.diff(a, w, 2)
uu = 1/g

claims = {}
claims["a'   = -u^2 D'(u)"]                       = (ap, -uu**2 * Dp(uu))
claims["a''  = u^4 D''(u) + 2 u^3 D'(u)"]         = (app, uu**4 * Dpp(uu) + 2*uu**3 * Dp(uu))
claims["da/ddelta = a'"]                          = (sp.diff(a, dl), ap)
claims["da/dw_s   = -a'"]                         = (sp.diff(a, ws), -ap)
claims["da'/ddelta = a''"]                        = (sp.diff(ap, dl), app)
claims["D(1) = 0"]                                = (D(sp.Integer(1)), 0)
claims["-D'(1) = 1"]                              = (-Dp(sp.Integer(1)), 1)
claims["lim (1 - D(u))/u = 9/8"]                  = (sp.limit((1 - D(u))/u, u, 0), sp.Rational(9, 8))
# autonomous form: Lambda(a) = -u(a)^2 D'(u(a)) with u(a) = D^{-1}(a); as a function of u it is -u^2 D'(u), no delta
Lam_u = -u**2 * Dp(u)
claims["Lambda(u) has no delta"]                  = (sp.diff(Lam_u, dl), 0)
# dLambda/da = (dLambda/du)/(da/du) = a''/a' (both expressed through u)
dLam_da = sp.diff(Lam_u, u) / sp.diff(D(u), u)
app_u = u**4 * Dpp(u) + 2*u**3 * Dp(u)
claims["dLambda/da = a''/a'"]                     = (dLam_da, app_u / Lam_u)
# B row: a_d = D(u_d), u_d = 1/(1/u - nabla); d a_d / d a = Lambda(u_d)/Lambda(u)
u_d = 1/(1/u - nab)
a_d = D(u_d)
d_ad_da = sp.diff(a_d, u) / sp.diff(D(u), u)
claims["d a_d / d a = Lambda(a_d)/Lambda(a)"]     = (d_ad_da, Lam_u.subs(u, u_d) / Lam_u)
# error dynamics: e_a = a(w, delta) - a(w_hat, delta_hat) to first order = a'(e_delta - e_3), w = w_hat - e_3 (w_hat = w_d - dw3_hat, w = w_d - dw3)
e3, ed = sp.symbols('e_3 e_delta', real=True)
a_true = D(1/((w - e3 - ws) + (dl + ed)))          # true height w - e3 (dw3 = dw3_hat + e3), true delta = delta_hat + e_delta
lin = sp.diff(a_true, e3).subs({e3: 0, ed: 0}) * e3 + sp.diff(a_true, ed).subs({e3: 0, ed: 0}) * ed
claims["e_a = a'(e_delta - e_3)"]                 = (lin, ap * (ed - e3))
# det O_2 closed forms (coefficients frozen)
al, be, p = sp.symbols('alpha beta p', real=True)
A2 = sp.Matrix([[al, -be], [0, 1]])
C1 = sp.Matrix([[1, 0]]);  O1 = sp.Matrix.vstack(C1, C1 * A2)
C2 = sp.Matrix([[-p, p]]); O2 = sp.Matrix.vstack(C2, C2 * A2)
claims["det O_2 (y1 row) = -beta"]                = (O1.det(), -be)
claims["det O_2 (y2 row) = p^2 (alpha - beta - 1)"] = (O2.det(), p**2 * (al - be - 1))
claims["alpha - beta - 1 = -(1 - lambda_c)"]      = ((lam_c + F_dw*p) - F_dw*p - 1, -(1 - lam_c))

ok = True
for name, (lhs, rhs) in claims.items():
    d = sp.simplify(lhs - rhs)
    if d != 0:
        d = sp.simplify(sp.together(d))
    res = (d == 0)
    ok &= res
    print(f"{'PASS' if res else 'FAIL'}  {name}")
print("ALL PASS" if ok else "SOME FAIL")
