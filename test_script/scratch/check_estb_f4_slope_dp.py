# -*- coding: utf-8 -*-
"""PURPOSE (2026-09-23): independent sympy check of the closed forms of 0923_estb_f4_slope_dp.tex -- the two-constant slope law
a' = 1/(h + delta)^p (contact slope pinned at 1) read at the estimator height g_hat = w_d - dw3_hat - w_s + delta_hat, serving
ONLY the propagation of the gain slot; delta (position type) and p (exponent, carries the far level) are the constants
(code slots 7 and 5).  Same claims as check_estb_f2_slope_dc.py in the new coordinate: Jacobians and integrated law (S1),
exact step (S4), row-4 error dynamics and the 19-term expansion (S5/S6), the y2 walk-back row (S7), gate-3 pairs, the
frozen triple (singular) and the two-height determinant of the visible combinations (S9).
Run: python3 test_script/scratch/check_estb_f4_slope_dp.py
"""
import sympy as sp

wd, dw3, dw3h, ws, p, ph, dl, dlh, M, nab = sp.symbols('w_d dw3 dw3h w_s p p_hat delta delta_hat M nabla', real=True)
lam_c, F_dw, eps = sp.symbols('lambda_c F_dw epsilon_w', real=True)
e3, ea, ep, ed, rp = sp.symbols('e_3 e_a e_p e_delta r_p', real=True)
gg = sp.Symbol('g', positive=True)
pp = sp.Symbol('pp', positive=True)                                            # a positive exponent symbol for the integrals

ap = lambda hh, dd, ex: 1 / (hh + dd)**ex
h_hat = wd - dw3h - ws
g_hat = h_hat + dlh
apk = ap(h_hat, dlh, ph)

claims = {}
# --- S1 Jacobians
claims["da'/dp     = -a' ln g"]                      = (sp.diff(ap(gg, 0, ph), ph), -ap(gg, 0, ph) * sp.log(gg))
claims["da'/dh     = -p a'/g"]                       = (sp.diff(ap(gg, 0, ph), gg), -ph * ap(gg, 0, ph) / gg)
claims["da'/ddelta = -p a'/g = da'/dh"]              = (sp.diff(ap(h_hat, dlh, ph), dlh), sp.diff(ap(h_hat, dlh, ph), wd))
claims["da'/d(dw3_hat) = +p a'/g"]                   = (sp.diff(apk, dw3h), ph * apk / g_hat)
claims["contact slope a'(g = 1) = 1 for every p"]    = (ap(sp.Integer(1), 0, ph), sp.Integer(1))
# --- S1 integrated law (seed container only)
h0 = sp.Symbol('h0', positive=True); d0 = sp.Symbol('d0', positive=True)
q = sp.Symbol('q', positive=True)                                              # q = p - 1 > 0
claims["int_h^inf a' = 1/((p-1)(h+delta)^(p-1))"]    = (sp.simplify(sp.integrate(1 / (gg + d0)**(q + 1), (gg, h0, sp.oo))), 1 / (q * (h0 + d0)**q))
abar = 1 - 1 / ((pp - 1) * (gg + d0)**(pp - 1))
claims["da/ddelta = a'"]                              = (sp.simplify(sp.diff(abar, d0) - ap(gg, d0, pp)), sp.Integer(0))
claims["da/dp = (1/(p-1) + ln g)/((p-1) g^(p-1))"]    = (sp.simplify(sp.diff(abar, pp) - (1 / (pp - 1) + sp.log(gg + d0)) / ((pp - 1) * (gg + d0)**(pp - 1))), sp.Integer(0))
# --- S4 exact step
step_exact = (g_hat**(1 - ph) - (g_hat + M)**(1 - ph)) / (ph - 1)
G0 = sp.Symbol('G0', positive=True)
Mp = sp.Symbol('Mp', positive=True)                                            # a positive displacement keeps the integral's branch simple; q = p - 1 > 0
claims["exact step = int_{g}^{g+M} a' dg"]           = (sp.simplify(((G0**(-q) - (G0 + Mp)**(-q)) / q) - sp.integrate(1 / gg**(q + 1), (gg, G0, G0 + Mp), conds='none')), sp.Integer(0))
claims["exact step, first order in M = a' M"]        = (sp.simplify(sp.series(step_exact, M, 0, 2).removeO() - apk * M), sp.Integer(0))
# --- S5 row 4
h_true = wd - dw3 - ws
Dw = M + (1 - lam_c) * e3 + F_dw * ea + eps
ap_true = ap(h_true, dl, p) + rp
e_a_next = (ea + ap_true * Dw - apk * M).subs({dw3: dw3h + e3, dl: dlh + ed, p: ph + ep})
lin = sum(sp.diff(e_a_next, v).subs({e3: 0, ea: 0, ep: 0, ed: 0, eps: 0, rp: 0}) * v for v in (e3, ea, ep, ed, eps, rp))
target = ((1 + apk * F_dw) * ea + (apk * (1 - lam_c) + ph * apk * M / g_hat) * e3 - apk * sp.log(g_hat) * M * ep
          - (ph * apk * M / g_hat) * ed + apk * eps + M * rp)
claims["row 4 linearisation = tex S5"]                = (sp.simplify(sp.expand(lin) - sp.expand(target)), sp.Integer(0))
claims["F_e(4,3) = a'(1-lc) + p a' M/g"]             = (sp.simplify(sp.diff(lin, e3) - (apk * (1 - lam_c) + ph * apk * M / g_hat)), sp.Integer(0))
claims["F_e(4,4) = 1 + a' F_dw"]                     = (sp.simplify(sp.diff(lin, ea) - (1 + apk * F_dw)), sp.Integer(0))
claims["F_e(4,5) = -a' ln g M"]                      = (sp.simplify(sp.diff(lin, ep) + apk * sp.log(g_hat) * M), sp.Integer(0))
claims["F_e(4,6) = -p a' M/g = -(height share of F_e(4,3))"] = (sp.simplify(sp.diff(lin, ed) + (sp.diff(lin, e3) - apk * (1 - lam_c))), sp.Integer(0))
# --- S6 expansion page
dap_lin = ph * apk / g_hat * e3 - ph * apk / g_hat * ed - apk * sp.log(g_hat) * ep + rp
dM = (1 - lam_c) * e3 + F_dw * ea + eps
prod_diff = dap_lin * M + apk * dM + dap_lin * dM
L = sp.log(g_hat)
terms = [ph * apk / g_hat * M * e3, -ph * apk / g_hat * M * ed, -apk * L * M * ep, M * rp,
         apk * (1 - lam_c) * e3, apk * F_dw * ea, apk * eps,
         ph * apk / g_hat * (1 - lam_c) * e3 * e3, ph * apk / g_hat * F_dw * e3 * ea, ph * apk / g_hat * e3 * eps,
         -ph * apk / g_hat * (1 - lam_c) * ed * e3, -ph * apk / g_hat * F_dw * ed * ea, -ph * apk / g_hat * ed * eps,
         -apk * L * (1 - lam_c) * ep * e3, -apk * L * F_dw * ep * ea, -apk * L * ep * eps,
         (1 - lam_c) * rp * e3, F_dw * rp * ea, rp * eps]
claims["expansion page: (1)..(19) sum to the product difference exactly"] = (sp.simplify(sp.expand(prod_diff) - sp.expand(sum(terms))), sp.Integer(0))
claims["expansion page: 7 first-order terms = the linearisation minus e_a"] = (sp.simplify(sp.expand(sum(terms[:7])) - sp.expand(lin - ea)), sp.Integer(0))
# --- S7 y2 walk-back row
ah = sp.Symbol('a_hat', real=True)
y2h = ah - apk * nab
claims["H2(4) = 1"]                                  = (sp.diff(y2h, ah), sp.Integer(1))
claims["H2(3) = -nabla p a'/g"]                      = (sp.simplify(sp.diff(y2h, dw3h) + nab * ph * apk / g_hat), sp.Integer(0))
claims["H2(5) = +nabla a' ln g"]                     = (sp.simplify(sp.diff(y2h, ph) - nab * apk * sp.log(g_hat)), sp.Integer(0))
claims["H2(6) = +nabla p a'/g = -H2(3)"]             = (sp.simplify(sp.diff(y2h, dlh) + sp.diff(y2h, dw3h)), sp.Integer(0))
# --- S9 gate 3
al, qq = sp.symbols('alpha qsym', real=True)
bp, gp, bd, gd = sp.symbols('beta_p gamma_p beta_d gamma_d', real=True)
for nm, be, ga in (("p", bp, gp), ("delta", bd, gd)):
    A2 = sp.Matrix([[al, -be], [0, 1]])
    O2 = sp.Matrix.vstack(sp.Matrix([[1, ga]]), sp.Matrix([[1, ga]]) * A2)
    claims[f"det O_2 (e_a, e_{nm}) y2 row = -beta - gamma (alpha - 1)"] = (O2.det(), -be - ga * (al - 1))
    O1 = sp.Matrix.vstack(sp.Matrix([[qq, 0]]), sp.Matrix([[qq, 0]]) * A2)
    claims[f"det O_2 (e_a, e_{nm}) y1 row = -q^2 beta"] = (O1.det(), -qq**2 * be)
claims["beta_d/beta_p = gamma_d/gamma_p = p/(g ln g)"] = (sp.simplify((ph * apk * M / g_hat) / (apk * L * M) - (ph * apk * nab / g_hat) / (apk * L * nab)), sp.Integer(0))
A3 = sp.Matrix([[al, -bp, -bd], [0, 1, 0], [0, 0, 1]])
C3 = sp.Matrix([[1, gp, gd]])
O3 = sp.Matrix.vstack(C3, C3 * A3, C3 * A3 * A3)
rho = sp.Symbol('rho', real=True)
claims["det O_3 (e_a, e_p, e_delta) frozen = 0 identically"] = (sp.simplify(O3.det().subs({bd: rho * bp, gd: rho * gp})), sp.Integer(0))
g1, g2 = sp.symbols('g1 g2', positive=True)
V = sp.Matrix([[sp.log(g1), ph / g1], [sp.log(g2), ph / g2]])
claims["det[v(g1); v(g2)] = p (g1 ln g1 - g2 ln g2)/(g1 g2)"] = (sp.simplify(V.det() - ph * (g1 * sp.log(g1) - g2 * sp.log(g2)) / (g1 * g2)), sp.Integer(0))
claims["visible combination at one height: F_e(4,5) e_p + F_e(4,6) e_delta = -a' M (ln g e_p + p/g e_delta)"] = (
    sp.simplify(sp.expand(-apk * L * M * ep - ph * apk * M / g_hat * ed) - sp.expand(-apk * M * (L * ep + ph / g_hat * ed))), sp.Integer(0))
claims["level sensitivity ratio (delta,p) over (delta,c) at p = 2: da/dp / (1/g) = 1 + ln g"] = (
    sp.simplify(((1 / (pp - 1) + sp.log(gg)) / ((pp - 1) * gg**(pp - 1))).subs(pp, 2) / (1 / gg) - (1 + sp.log(gg))), sp.Integer(0))

ok = True
for name, (lhs, rhs) in claims.items():
    d = sp.simplify(lhs - rhs)
    if d != 0:
        d = sp.simplify(sp.together(sp.expand(d)))
    res = (d == 0)
    ok &= res
    print(f"{'PASS' if res else 'FAIL'}  {name}")
print(f"{len(claims)} claims;", "ALL PASS" if ok else "SOME FAIL")
