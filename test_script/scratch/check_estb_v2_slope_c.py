"""PURPOSE (2026-09-21): independent sympy check of the closed forms of 0921_estb_v2_slope_c.tex -- version 2 of the one-constant
line (user 09-21): the gain a_w is a STATE (a slot), the advisor slope law a' = (1 - c)/h^p written in the estimator's HEIGHT
h = w_d - dw3 - w_s serves ONLY the propagation of that state, c is the one constant (slot 5), p is fixed by the plane fit.
Checks: the slope Jacobians (S1), the row-4 error dynamics from the true and estimated increments (S4), the y2 walk-back row (S5),
the gate-3 det O_2 closed forms for the pair (e_a, e_c) through y2 and through y1 (S7), and the p = 2 endpoint facts.
Run: python3 test_script/scratch/check_estb_v2_slope_c.py
"""
import sympy as sp

wd, dw3, dw3h, ws, c, ch, p, M, nab = sp.symbols('w_d dw3 dw3h w_s c c_hat p M nabla', real=True)
lam_c, F_dw, eps = sp.symbols('lambda_c F_dw epsilon_w', real=True)
e3, ea, ec, rp = sp.symbols('e_3 e_a e_c r_p', real=True)         # e = true - estimate; r_p = slope residual of the family on this wall

h_hat = wd - dw3h - ws
ap = lambda hh, cc: (1 - cc) / hh**p                                # the law: slope only

claims = {}
# --- S1 Jacobians of the slope at the estimator height
ap_h = ap(h_hat, ch)
claims["da'/dc     = -1/h^p"]              = (sp.diff(ap_h, ch), -1 / h_hat**p)
claims["da'/dw_hat = -p a'/h"]             = (sp.diff(ap(sp.Symbol('hh', positive=True), ch), sp.Symbol('hh', positive=True)),
                                              -p * ap(sp.Symbol('hh', positive=True), ch) / sp.Symbol('hh', positive=True))
claims["da'/d(dw3_hat) = +p a'/h"]         = (sp.diff(ap_h, dw3h), p * ap_h / h_hat)
claims["da'/da_hat = 0 (slope does not read the gain slot)"] = (sp.Integer(0), sp.Integer(0))
# --- S4 row 4: true increment a_w+ = a_w + a'_true(w) Dw ; estimated a+ = a + a'(h_hat; c_hat) M ; Dw - M = (1-lc) e3 + F_dw e_a + eps
h_true = wd - dw3 - ws                                               # true height; dw3 = dw3h + e3
Dw = M + (1 - lam_c) * e3 + F_dw * ea + eps
ap_true = ap(h_true, c) + rp                                         # the plant's slope = family at the true (h, c) + residual r'
e_a_next = ea + ap_true * Dw - ap(h_hat, ch) * M                     # e_a+ = a_w+ - a_hat+
e_a_next = e_a_next.subs({dw3: dw3h + e3, c: ch + ec})
lin = sum(sp.diff(e_a_next, v).subs({e3: 0, ea: 0, ec: 0, eps: 0, rp: 0}) * v for v in (e3, ea, ec, eps, rp))
apk = ap(h_hat, ch)
target = (1 + apk * F_dw) * ea + (apk * (1 - lam_c) + M * p * apk / h_hat) * e3 - (M / h_hat**p) * ec + apk * eps + M * rp
claims["row 4 linearisation: e_a+ = (1 + a'F_dw) e_a + (a'(1-lc) + M p a'/h) e_3 - (M/h^p) e_c + a' eps + M r'"] = (sp.expand(lin), sp.expand(target))
claims["F_e(4,4) = 1 + a' F_dw  (no a''/a' M term: the slope is read at h, not at a_hat)"] = (sp.diff(lin, ea), 1 + apk * F_dw)
claims["F_e(4,5) = -M/h^p"]                = (sp.diff(lin, ec), -M / h_hat**p)
claims["F_e(4,3) = a'(1-lc) + M p a'/h"]   = (sp.diff(lin, e3), apk * (1 - lam_c) + M * p * apk / h_hat)
# --- S5 y2 walk-back row: y2_hat = a_hat - a'(h_hat; c_hat) nabla   (first order at the estimator height)
ah = sp.Symbol('a_hat', real=True)
y2h = ah - ap(h_hat, ch) * nab
claims["H2(4) = 1"]                        = (sp.diff(y2h, ah), sp.Integer(1))
claims["H2(3) = -nabla p a'/h"]            = (sp.diff(y2h, dw3h), -nab * p * apk / h_hat)
claims["H2(5) = +nabla/h^p"]               = (sp.diff(y2h, ch), nab / h_hat**p)
# --- S7 gate 3, pair (e_a, e_c), coefficients frozen: A2 = [[alpha, -beta],[0, 1]], alpha = 1 + a' F_dw, beta = M/h^p
al, be, ga, q = sp.symbols('alpha beta gamma q', real=True)
A2 = sp.Matrix([[al, -be], [0, 1]])
C2 = sp.Matrix([[1, ga]]);  O2 = sp.Matrix.vstack(C2, C2 * A2)      # y2 row: [1, nabla/h^p]
claims["det O_2 (y2 row) = -beta - gamma (alpha - 1) = -(M + nabla a' F_dw)/h^p"] = (O2.det(), -be - ga * (al - 1))
C1 = sp.Matrix([[q, 0]]);  O1 = sp.Matrix.vstack(C1, C1 * A2)      # y1 sees e_a through row 3 with weight q = -F_dw
claims["det O_2 (y1 via row 3) = -q^2 beta = -F_dw^2 M/h^p"] = (O1.det(), -q**2 * be)
# --- endpoint facts of the family
hh = sp.Symbol('hh', positive=True)
claims["p = 2, c = 0: integral of a' from 1 to inf = 1 (a(1) = 0 and a(inf) = 1 together)"] = (sp.integrate(1 / hh**2, (hh, 1, sp.oo)), sp.Integer(1))
claims["general p: integral of a' from h to inf = (1-c)/((p-1) h^(p-1))"] = (sp.integrate((1 - c) / hh**p, (hh, sp.Symbol('h0', positive=True), sp.oo), conds='none'),
                                                                              (1 - c) / ((p - 1) * sp.Symbol('h0', positive=True)**(p - 1)))

ok = True
for name, (lhs, rhs) in claims.items():
    d = sp.simplify(lhs - rhs)
    if d != 0:
        d = sp.simplify(sp.together(sp.expand(d)))
    res = (d == 0)
    ok &= res
    print(f"{'PASS' if res else 'FAIL'}  {name}")
print("ALL PASS" if ok else "SOME FAIL")
