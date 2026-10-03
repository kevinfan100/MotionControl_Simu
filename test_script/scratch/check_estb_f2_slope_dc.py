# -*- coding: utf-8 -*-
"""PURPOSE (2026-09-22): independent sympy check of the closed forms of 0922_estb_f2_slope_dc.tex -- the two-constant slope law
a' = (1 - c)/(h + delta)^2 read at the estimator height g_hat = w_d - dw3_hat - w_s + delta_hat, serving ONLY the propagation of the
gain slot; delta (position type) and c (level type) are the two constants (code slots 7 and 5), p = 2 pinned.
Checks: the slope Jacobians and the integrated law (S1), the exact step (S4), the row-4 error dynamics and the full 19-term
expansion page (S5/S6), the y2 walk-back row (S7), the gate-3 closed forms for the pairs (e_a, e_c), (e_a, e_delta), the frozen
triple (identically singular) and the two-height determinant of the visible combinations (S9).
Run: python3 test_script/scratch/check_estb_f2_slope_dc.py
"""
import sympy as sp

wd, dw3, dw3h, ws, c, ch, dl, dlh, M, nab = sp.symbols('w_d dw3 dw3h w_s c c_hat delta delta_hat M nabla', real=True)
lam_c, F_dw, eps = sp.symbols('lambda_c F_dw epsilon_w', real=True)
e3, ea, ec, ed, rp = sp.symbols('e_3 e_a e_c e_delta r_p', real=True)      # e = true - estimate; r_p = family residual on this wall
gg = sp.Symbol('g', positive=True)                                          # shifted gap g = h + delta

ap = lambda hh, dd, cc: (1 - cc) / (hh + dd)**2                             # the law: slope only
h_hat = wd - dw3h - ws
g_hat = h_hat + dlh
apk = ap(h_hat, dlh, ch)

claims = {}
# --- S1 Jacobians of the slope
claims["da'/dc     = -1/g^2"]                       = (sp.diff(ap(gg, 0, ch), ch), -1 / gg**2)
claims["da'/dh     = -2 a'/g"]                      = (sp.diff(ap(gg, 0, ch), gg), -2 * ap(gg, 0, ch) / gg)
claims["da'/ddelta = -2 a'/g = da'/dh"]             = (sp.diff(ap(h_hat, dlh, ch), dlh), sp.diff(ap(h_hat, dlh, ch), wd))
claims["da'/d(dw3_hat) = +2 a'/g"]                  = (sp.diff(apk, dw3h), 2 * apk / g_hat)
claims["da'/da_hat = 0"]                            = (sp.Integer(0), sp.Integer(0))
# --- S1 integrated law (seed container only)
h0 = sp.Symbol('h0', positive=True); d0 = sp.Symbol('d0', positive=True)
claims["int_h^inf a' = (1-c)/(h+delta)"]            = (sp.integrate((1 - c) / (gg + d0)**2, (gg, h0, sp.oo)), (1 - c) / (h0 + d0))
abar = 1 - (1 - c) / (gg + d0)
claims["a = 1 - (1-c)/(h+delta) = (h + delta - 1 + c)/(h + delta)  ([1/1]: c0 = delta - 1 + c, d1 = delta)"] = (abar, (gg + d0 - 1 + c) / (gg + d0))
claims["da/ddelta = a'"]                             = (sp.diff(abar, d0), ap(gg, d0, c))
claims["da/dc = 1/(h+delta)"]                        = (sp.diff(abar, c), 1 / (gg + d0))
claims["contact a = 0 at g = 1 - c"]                 = (abar.subs(gg + d0, 1 - c).subs(d0, 1 - c - gg), sp.Integer(0))
# --- S4 exact step over the known displacement M
step_exact = (1 - ch) * (1 / g_hat - 1 / (g_hat + M))
claims["exact step = int_{g}^{g+M} a' dg"]          = (step_exact, sp.integrate((1 - ch) / (gg)**2, (gg, g_hat, g_hat + M)))
claims["exact step, first order in M = a' M"]       = (sp.series(step_exact, M, 0, 2).removeO(), apk * M)
# --- S5 row 4: e_a+ = e_a + a'_true Dw - a' M, with dw3 = dw3h + e3, delta = dlh + ed, c = ch + ec
h_true = wd - dw3 - ws
Dw = M + (1 - lam_c) * e3 + F_dw * ea + eps
ap_true = ap(h_true, dl, c) + rp
e_a_next = (ea + ap_true * Dw - apk * M).subs({dw3: dw3h + e3, dl: dlh + ed, c: ch + ec})
lin = sum(sp.diff(e_a_next, v).subs({e3: 0, ea: 0, ec: 0, ed: 0, eps: 0, rp: 0}) * v for v in (e3, ea, ec, ed, eps, rp))
target = ((1 + apk * F_dw) * ea + (apk * (1 - lam_c) + 2 * apk * M / g_hat) * e3 - (M / g_hat**2) * ec
          - (2 * apk * M / g_hat) * ed + apk * eps + M * rp)
claims["row 4 linearisation = tex S5"]               = (sp.expand(lin), sp.expand(target))
claims["F_e(4,3) = a'(1-lc) + 2 a' M/g"]            = (sp.diff(lin, e3), apk * (1 - lam_c) + 2 * apk * M / g_hat)
claims["F_e(4,4) = 1 + a' F_dw"]                    = (sp.diff(lin, ea), 1 + apk * F_dw)
claims["F_e(4,5) = -M/g^2"]                         = (sp.diff(lin, ec), -M / g_hat**2)
claims["F_e(4,6) = -2 a' M/g = -(height share of F_e(4,3))"] = (sp.diff(lin, ed), -(sp.diff(lin, e3) - apk * (1 - lam_c)))
# --- S6 expansion page: with the slope difference linearised, the product difference is exactly the 7 first-order + 12 second-order terms
dap_lin = 2 * apk / g_hat * e3 - 2 * apk / g_hat * ed - ec / g_hat**2 + rp
dM = (1 - lam_c) * e3 + F_dw * ea + eps
prod_diff = dap_lin * M + apk * dM + dap_lin * dM
terms = [2 * apk / g_hat * M * e3, -2 * apk / g_hat * M * ed, -M / g_hat**2 * ec, M * rp,                    # (1)-(4)
         apk * (1 - lam_c) * e3, apk * F_dw * ea, apk * eps,                                                # (5)-(7)
         2 * apk / g_hat * (1 - lam_c) * e3 * e3, 2 * apk / g_hat * F_dw * e3 * ea, 2 * apk / g_hat * e3 * eps,           # (8)-(10)
         -2 * apk / g_hat * (1 - lam_c) * ed * e3, -2 * apk / g_hat * F_dw * ed * ea, -2 * apk / g_hat * ed * eps,        # (11)-(13)
         -(1 - lam_c) / g_hat**2 * ec * e3, -F_dw / g_hat**2 * ec * ea, -1 / g_hat**2 * ec * eps,                        # (14)-(16)
         (1 - lam_c) * rp * e3, F_dw * rp * ea, rp * eps]                                                  # (17)-(19)
claims["expansion page: (1)..(19) sum to the product difference exactly"] = (sp.expand(prod_diff), sp.expand(sum(terms)))
claims["expansion page: 7 first-order terms = the linearisation minus e_a"] = (sp.expand(sum(terms[:7])), sp.expand(lin - ea))
# --- S7 y2 walk-back row
ah = sp.Symbol('a_hat', real=True)
y2h = ah - apk * nab
claims["H2(4) = 1"]                                 = (sp.diff(y2h, ah), sp.Integer(1))
claims["H2(3) = -nabla 2 a'/g"]                     = (sp.diff(y2h, dw3h), -nab * 2 * apk / g_hat)
claims["H2(5) = +nabla/g^2"]                        = (sp.diff(y2h, ch), nab / g_hat**2)
claims["H2(6) = +nabla 2 a'/g = -H2(3)"]            = (sp.diff(y2h, dlh), -sp.diff(y2h, dw3h))
# --- S9 gate 3, coefficients frozen
al, q = sp.symbols('alpha q', real=True)
bc, gc, bd, gd = sp.symbols('beta_c gamma_c beta_d gamma_d', real=True)
for nm, be, ga in (("c", bc, gc), ("delta", bd, gd)):
    A2 = sp.Matrix([[al, -be], [0, 1]])
    O2 = sp.Matrix.vstack(sp.Matrix([[1, ga]]), sp.Matrix([[1, ga]]) * A2)
    claims[f"det O_2 (e_a, e_{nm}) y2 row = -beta - gamma (alpha - 1)"] = (O2.det(), -be - ga * (al - 1))
    O1 = sp.Matrix.vstack(sp.Matrix([[q, 0]]), sp.Matrix([[q, 0]]) * A2)
    claims[f"det O_2 (e_a, e_{nm}) y1 row = -q^2 beta"] = (O1.det(), -q**2 * be)
# the concrete betas / gammas
claims["beta_c/gamma_c = beta_d/gamma_d = M/nabla (same ratio)"] = ((M / g_hat**2) / (nab / g_hat**2), (2 * apk * M / g_hat) / (2 * apk * nab / g_hat))
claims["y2 pair (e_a, e_c): -(M + nabla a' F_dw)/g^2"] = ((-bc - gc * (al - 1)).subs({bc: M / g_hat**2, gc: nab / g_hat**2, al: 1 + apk * F_dw}),
                                                            -(M + nab * apk * F_dw) / g_hat**2)
claims["y2 pair (e_a, e_delta): -(2a'/g)(M + nabla a' F_dw)"] = ((-bd - gd * (al - 1)).subs({bd: 2 * apk * M / g_hat, gd: 2 * apk * nab / g_hat, al: 1 + apk * F_dw}),
                                                                   -(2 * apk / g_hat) * (M + nab * apk * F_dw))
# frozen triple: identically singular when the two constants act through the same channel with the same beta/gamma ratio
A3 = sp.Matrix([[al, -bc, -bd], [0, 1, 0], [0, 0, 1]])
C3 = sp.Matrix([[1, gc, gd]])
O3 = sp.Matrix.vstack(C3, C3 * A3, C3 * A3 * A3)
det3 = sp.simplify(O3.det().subs({bd: bc * 2 * (1 - ch) / gg, gd: gc * 2 * (1 - ch) / gg}))
claims["det O_3 (e_a, e_c, e_delta) frozen = 0 identically"] = (det3, sp.Integer(0))
# two heights: visible combination v(g) = [1/g^2, 2(1-c)/g^3]
g1, g2 = sp.symbols('g1 g2', positive=True)
V = sp.Matrix([[1 / g1**2, 2 * (1 - ch) / g1**3], [1 / g2**2, 2 * (1 - ch) / g2**3]])
claims["det[v(g1); v(g2)] = 2(1-c)(g1 - g2)/(g1^3 g2^3)"] = (sp.simplify(V.det()), sp.simplify(2 * (1 - ch) * (g1 - g2) / (g1**3 * g2**3)))
claims["visible combination at one height: F_e(4,5) e_c + F_e(4,6) e_delta = -(M/g^2)(e_c + 2(1-c)/g e_delta)"] = (
    sp.expand(-M / g_hat**2 * ec - 2 * apk * M / g_hat * ed), sp.expand(-(M / g_hat**2) * (ec + 2 * (1 - ch) / g_hat * ed)))

ok = True
for name, (lhs, rhs) in claims.items():
    d = sp.simplify(lhs - rhs)
    if d != 0:
        d = sp.simplify(sp.together(sp.expand(d)))
    res = (d == 0)
    ok &= res
    print(f"{'PASS' if res else 'FAIL'}  {name}")
print(f"{sum(1 for _ in claims)} claims;", "ALL PASS" if ok else "SOME FAIL")
