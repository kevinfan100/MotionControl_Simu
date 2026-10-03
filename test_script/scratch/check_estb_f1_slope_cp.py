"""Independent check of the (c, p) slope-law Jacobians wired as plaw_cp_slope (2026-09-23 evening).
Law  a'(h) = (1 - c) / h^p,  h = w_d - dw3 - w_s (no shift).  Value law used once for the seed: a(h) = 1 - (1-c) h^(1-p)/(p-1).
Run: python3 check_estb_f1_slope_cp.py  -> N/N PASS."""
import sympy as sp

h, c, p, h0, h1, wd, dw3, ws = sp.symbols('h c p h0 h1 w_d dw3 w_s', positive=True)
ap = (1 - c) / h**p
checks = []
def chk(name, lhs, rhs):
    ok = sp.simplify(lhs - rhs) == 0
    checks.append(ok); print(f"{'PASS' if ok else 'FAIL'}  {name}")

# S1 Jacobians of the slope (code: dap_db_i, dap_ddl_i, dap_dd3_i)
chk("d a'/d c  = -1/h^p",              sp.diff(ap, c), -1 / h**p)
chk("d a'/d p  = -a' ln h",            sp.diff(ap, p), -ap * sp.log(h))
h_of = wd - dw3 - ws
ap_w = ap.subs(h, h_of)
chk("d a'/d dw3 = +p a'/h",            sp.diff(ap_w, dw3), (p * ap / h).subs(h, h_of))
chk("d a'/d w_d = -p a'/h",            sp.diff(ap_w, wd), (-p * ap / h).subs(h, h_of))
# S4 exact step: a[k+1] = a[k] + int_{h0}^{h1} a' dh, h1 = h0 + M (M < 0 when descending, so the increment is negative)
step = (1 - c) * (h0**(1 - p) - h1**(1 - p)) / (p - 1)
chk("exact step  d/dh1 [(1-c)(h0^(1-p) - h1^(1-p))/(p-1)] = a'(h1)  and  step(h1 = h0) = 0",
    sp.diff(step, h1) - ap.subs(h, h1) + step.subs(h1, h0), 0)
# value law consistency and seed sensitivities (code: da5_cp, da7_cp)
a_val = 1 - (1 - c) * h**(1 - p) / (p - 1)
chk("value law: d a/d h = a'",         sp.diff(a_val, h), ap)
chk("value law: a -> 1 as h -> oo (p > 1)", sp.limit(a_val.subs(p, sp.Rational(19, 10)), h, sp.oo), 1)
chk("d a/d c = h^(1-p)/(p-1)",         sp.diff(a_val, c), h**(1 - p) / (p - 1))
chk("d a/d p = (1-c) h^(1-p) (1/(p-1) + ln h)/(p-1)", sp.diff(a_val, p), (1 - c) * h**(1 - p) * (1 / (p - 1) + sp.log(h)) / (p - 1))
# frozen-coefficient one-height combination (gate 3): e_a' = (d a'/d c) e_c + (d a'/d p) e_p  ~  -(1/h^p) [e_c + (1-c) ln h e_p]
comb = sp.diff(ap, c) * sp.Symbol('e_c') + sp.diff(ap, p) * sp.Symbol('e_p')
chk("one height sees  e_c + (1-c) ln h e_p  (up to -1/h^p)", comb, -(sp.Symbol('e_c') + (1 - c) * sp.log(h) * sp.Symbol('e_p')) / h**p)
print(f"\n{sum(checks)}/{len(checks)} PASS")
