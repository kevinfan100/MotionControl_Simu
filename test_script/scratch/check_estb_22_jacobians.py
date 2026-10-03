"""PURPOSE (2026-09-18): independent sympy check of the [2/2] Pade law Jacobians used by 0918_estb_22_versionA.tex and the
controller flag pade22 (derivation-workflow rule 8: verify before coding).
    a(g) = (g^2 + c1 g + c0) / (g^2 + d1 g + d2),  N' = 2g + c1,  D' = 2g + d1,  a' = (N' - a D') / D
Prints PASS/FAIL per closed form. Run: python3 test_script/scratch/check_estb_22_jacobians.py
"""
import sympy as sp

g, c0, c1, d1, d2 = sp.symbols('g c0 c1 d1 d2', real=True)
N = g**2 + c1*g + c0
D = g**2 + d1*g + d2
a = N / D
Np = sp.diff(N, g)
Dp = sp.diff(D, g)
ap = sp.diff(a, g)

claims = {
    "a'        = (N' - a D')/D":            (ap,                (Np - a*Dp)/D),
    "da/dc0    = 1/D":                      (sp.diff(a, c0),    1/D),
    "da/dc1    = g/D":                      (sp.diff(a, c1),    g/D),
    "da/dd1    = -a g/D":                   (sp.diff(a, d1),    -a*g/D),
    "da/dd2    = -a/D":                     (sp.diff(a, d2),    -a/D),
    "da'/dc0   = -D'/D^2":                  (sp.diff(ap, c0),   -Dp/D**2),
    "da'/dc1   = (D - g D')/D^2":           (sp.diff(ap, c1),   (D - g*Dp)/D**2),
    "da'/dd1   = a g D'/D^2 - a/D - a' g/D": (sp.diff(ap, d1),  a*g*Dp/D**2 - a/D - ap*g/D),
    "da'/dd2   = a D'/D^2 - a'/D":          (sp.diff(ap, d2),   a*Dp/D**2 - ap/D),
    # limits of the two-point form (0916_abar_gap_polynomial.tex S5)
    "a(0)      = c0/d2":                    (a.subs(g, 0),      c0/d2),
    "a'(0)     = (c1 d2 - c0 d1)/d2^2":     (ap.subs(g, 0),     (c1*d2 - c0*d1)/d2**2),
    "far: g(1-a) -> d1 - c1":               (sp.limit(g*(1 - a), g, sp.oo), d1 - c1),
    # the [1/1] limit: c1 = d1 -> a = (g + c0/d1 ... ) is NOT a reduction; instead check that with c1 -> 1/eps-scaled the
    # one-height equation holds:  c0 + c1 g - a d2 - a g d1 = g^2 (a - 1)
    "one height: c0 + c1 g - a d2 - a g d1 = g^2 (a-1)": (c0 + c1*g - a*d2 - a*g*d1, g**2*(a - 1)),
}
ok = True
for name, (lhs, rhs) in claims.items():
    diff = sp.simplify(lhs - rhs)
    res = diff == 0
    ok &= bool(res)
    print(f"{'PASS' if res else 'FAIL'}  {name}" + ("" if res else f"   residual: {diff}"))
print("ALL PASS" if ok else "SOME FAIL")
