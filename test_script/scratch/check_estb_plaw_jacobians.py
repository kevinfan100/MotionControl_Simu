"""PURPOSE (2026-09-21): independent sympy check of the advisor's power-law closed forms before the controller flag
plaw_c_state (derivation-workflow rule 8).  Slope form a' = (1 - c)/h^p integrated from the far field (a(inf) = 1):
    a(h; p, c) = 1 - (1 - c) / ((p - 1) h^(p-1)),   h = w_d - dw3 - w_s,   p fixed (plane fit), c the state (slot 7)
Checks: a' = da/dh, a'' = d2a/dh2, da/dc = (1 - a)/(1 - c), da'/dc = -a'/(1 - c) = -1/h^p, da'/d(dw3) = -a'',
the autonomous form Lambda(a) = (1 - c) ((p-1)(1-a)/(1-c))^(p/(p-1)) (c-dependent: c is shape-type, not position-type),
the plane pins at p = 2: a(1) = c, a'(1) = 1 - c, and the two 2-state det O_2 closed forms (y1 row -F_dw da/dc,
y2 row -(1 - lambda_c) (da'/dc)^2 pattern as in 0920 tex).  Prints PASS/FAIL per closed form.
Run: python3 test_script/scratch/check_estb_plaw_jacobians.py
"""
import sympy as sp

h, p, c, wd, dw3, ws = sp.symbols('h p c w_d dw3 w_s', positive=True)
lam_c, F_dw, nab = sp.symbols('lambda_c F_dw nabla', real=True)

phi = 1 / ((p - 1) * h**(p - 1))
a = 1 - (1 - c) * phi
ap = sp.diff(a, h)
app = sp.diff(a, h, 2)

claims = {}
claims["a'  = (1 - c)/h^p"]                       = (ap, (1 - c) / h**p)
claims["a'' = -p (1 - c)/h^(p+1)"]                = (app, -p * (1 - c) / h**(p + 1))
claims["da/dc = (1 - a)/(1 - c)"]                 = (sp.diff(a, c), (1 - a) / (1 - c))
claims["da/dc = phi"]                             = (sp.diff(a, c), phi)
claims["da'/dc = -1/h^p"]                         = (sp.diff(ap, c), -1 / h**p)
claims["da'/dc = -a'/(1 - c)"]                    = (sp.diff(ap, c), -ap / (1 - c))
# height chain: h = w_d - dw3 - w_s
a_w = a.subs(h, wd - dw3 - ws)
claims["da/d(dw3) = -a'"]                         = (sp.diff(a_w, dw3), -ap.subs(h, wd - dw3 - ws))
claims["da'/d(dw3) = -a''"]                       = (sp.diff(sp.diff(a_w, wd), dw3), -app.subs(h, wd - dw3 - ws))
claims["da/dw_s = -a'"]                           = (sp.diff(a_w, ws), -ap.subs(h, wd - dw3 - ws))
# autonomous form: eliminate h through 1 - a = (1 - c) phi
aa = sp.symbols('a', positive=True)
h_of_a = ((1 - c) / ((p - 1) * (1 - aa)))**(1 / (p - 1))
Lam = ((1 - c) / h**p).subs(h, h_of_a)
Lam_claim = (1 - c) * ((p - 1) * (1 - aa) / (1 - c))**(p / (p - 1))
# symbolic simplification of fractional powers with unknown-sign exponents stalls: check numerically at 5 points to 1e-12
num_ok = all(abs(float((Lam - Lam_claim).subs({p: pv, c: cv, aa: av}))) < 1e-12
             for pv, cv, av in [(1.9666, 0.0, 0.3), (2.0, 0.12, 0.5), (2.3, -0.1, 0.9), (1.5, 0.3, 0.05), (2.7, 0.44, 0.7)])
claims["Lambda(a) = (1 - c) ((p-1)(1-a)/(1-c))^(p/(p-1))  [numeric, 5 pts]"] = (num_ok, True)
claims["Lambda depends on c (shape-type)"]        = (sp.simplify(sp.diff(Lam, c)) == 0, False)
claims["p = 2: a(1) = c"]                         = (a.subs({p: 2, h: 1}), c)
claims["p = 2: a'(1) = 1 - c"]                    = (ap.subs({p: 2, h: 1}), 1 - c)
claims["p = 2: Lambda = (1 - a)^2/(1 - c)"]       = (Lam.subs(p, 2), (1 - aa)**2 / (1 - c))
# det O_2 closed forms (gate 3, coefficients frozen): state pair (dw3, c). Row 3 of F_e under amodel_fe: alpha = lambda_c + F_dw a'
# (self term), beta = -F_dw da/dc (c column); c is a constant (row [0 1]). y1 row C1 = [1 0]; y2 row C2 = [-a', da/dc].
apc, dac = sp.symbols("a_p a_c", real=True)          # a', da/dc at the working point
al = lam_c + F_dw * apc;  be = -F_dw * dac
A2 = sp.Matrix([[al, be], [0, 1]])
C1 = sp.Matrix([[1, 0]]);  O1 = sp.Matrix.vstack(C1, C1 * A2)
C2 = sp.Matrix([[-apc, dac]]); O2 = sp.Matrix.vstack(C2, C2 * A2)
claims["det O_2 (y1 row) = -F_dw da/dc  (zero in a hold: F_dw ~ velocity)"] = (O1.det(), -F_dw * dac)
claims["det O_2 (y2 row) = -(1 - lambda_c) a' da/dc  (zero only as h -> inf)"] = (O2.det(), -(1 - lam_c) * apc * dac)

ok = True
for name, (lhs, rhs) in claims.items():
    if isinstance(lhs, bool):
        res = (lhs == rhs)
    else:
        d = sp.simplify(lhs - rhs)
        if d != 0:
            d = sp.simplify(sp.together(sp.expand_power_base(d, force=True)))
        res = (d == 0)
    ok &= res
    print(f"{'PASS' if res else 'FAIL'}  {name}")
print("ALL PASS" if ok else "SOME FAIL")
