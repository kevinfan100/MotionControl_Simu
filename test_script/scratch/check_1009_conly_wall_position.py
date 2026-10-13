# PURPOSE (2026-10-09): independent sympy check of reference/eq17_analysis/derivation/1009_conly_wall_position.tex
#   (c-only slope law (1-c)/h^2 with the wall position w_s as a sixth state, h = x_d - dx3 - w_s, delay-free H as motion_ctrl S5).
#   Checks: (1) first-order expansion of a'(h; c) in (e3, e_c, e_s); (2) row 4 of F_e from the exact gain-error step;
#   (3) leading-order observability matrix rows j = 0..6 against the closed forms written in the tex;
#   (4) frozen coefficients: e_s column = -(2(1-c)/h) x e_c column; (5) constant velocity: the two walked columns and the
#   three-row determinant (Vandermonde in 1/h).
# EXPIRES: with the tex | run: python3 test_script/scratch/check_1009_conly_wall_position.py
import sympy as sp

ok_all = True
def check(name, expr):
    global ok_all
    z = sp.simplify(expr)
    ok = (z == 0)
    ok_all &= ok
    print(f"{'PASS' if ok else 'FAIL'}  {name}" + ("" if ok else f"   residual = {z}"))

# ---------------------------------------------------------------- (1) a' expansion
hh, ch, e3, ec, es, lc, F, Mh, eps, ea = sp.symbols('hh ch e3 ec es lc F Mh eps ea', real=True)
t = sp.symbols('t')
a_p_hat = (1 - ch) / hh**2
a_p_true = (1 - ch - ec) / (hh - e3 - es)**2            # h = hh - e3 - es, c = ch + ec
sub = {e3: t*e3, ec: t*ec, es: t*es, ea: t*ea, eps: t*eps}
lin = sp.diff(a_p_true.subs(sub), t).subs(t, 0)
check("(1) a' first order = a'^ + (2 a'^/h^)(e3 + es) - ec/h^2",
      lin - ((2*a_p_hat/hh)*(e3 + es) - ec/hh**2))

# ---------------------------------------------------------------- (2) row 4 of F_e
M_true = Mh + (1 - lc)*e3
step_err = a_p_true*(M_true + F*ea + eps) - a_p_hat*Mh   # e_a[k+1] - e_a[k]
row4 = sp.expand(sp.diff(step_err.subs(sub), t).subs(t, 0))
check("(2) d e_a coefficient of e3", row4.coeff(e3) - (a_p_hat*(1 - lc) + 2*a_p_hat/hh*Mh))
check("(2) d e_a coefficient of e_a", row4.coeff(ea) - a_p_hat*F)
check("(2) d e_a coefficient of e_c", row4.coeff(ec) - (-Mh/hh**2))
check("(2) d e_a coefficient of e_s", row4.coeff(es) - (2*a_p_hat/hh*Mh))
check("(2) d e_a coefficient of eps", row4.coeff(eps) - a_p_hat)
check("(2) e_s entry = the M-part of the e3 entry", row4.coeff(es) - (row4.coeff(e3) - a_p_hat*(1 - lc)))

# ---------------------------------------------------------------- (3) leading-order O
N = 7
Fk = sp.symbols('F0:%d' % N); bk = sp.symbols('b0:%d' % N); hk = sp.symbols('h0:%d' % N, positive=True)
def Fe(k):
    return sp.Matrix([[0, 1, 0, 0, 0, 0],
                      [0, 0, 1, 0, 0, 0],
                      [0, 0, lc, -Fk[k], 0, 0],
                      [0, 0, 0, 1, -bk[k], 2*(1 - ch)*bk[k]/hk[k]],
                      [0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 1]])
H = sp.Matrix([[1, 0, 0, 0, 0, 0], [0, 0, 0, 1, 0, 0]])
Phi = sp.eye(6)
rows = []
for j in range(N):
    rows.append(H*Phi)
    Phi = Fe(j)*Phi
def B(i): return sum(bk[l] for l in range(i))                              # sum_{l<i} beta_l
def A(i): return sum(2*(1 - ch)*bk[l]/hk[l] for l in range(i))              # sum_{l<i} 2(1-c) beta_l / h_l
for j in range(N):
    y2 = sp.Matrix([[0, 0, 0, 1, -B(j), A(j)]])
    check(f"(3) y2 row j={j}", (rows[j][1, :] - y2).norm())
    if j == 0:
        y1 = sp.Matrix([[1, 0, 0, 0, 0, 0]])
    elif j == 1:
        y1 = sp.Matrix([[0, 1, 0, 0, 0, 0]])
    else:
        S = lambda f: sum(lc**(j - 3 - i)*Fk[i]*f(i) for i in range(j - 2))
        y1 = sp.Matrix([[0, 0, lc**(j - 2), -S(lambda i: 1), S(B), -S(A)]])
    check(f"(3) y1 row j={j}", (rows[j][0, :] - y1).norm())

# ---------------------------------------------------------------- (4) frozen coefficients
b, h0 = sp.symbols('b h0', positive=True)
frz = {**{bk[i]: b for i in range(N)}, **{hk[i]: h0 for i in range(N)}}
for j in range(N):
    r = rows[j].subs(frz)
    check(f"(4) frozen, row pair j={j}: col 6 = -(2(1-c)/h) col 5",
          (r[:, 5] + 2*(1 - ch)/h0*r[:, 4]).norm())

# ---------------------------------------------------------------- (5) constant velocity (continuum)
h, hA, u0, u1, u2, u3 = sp.symbols('h hA u0 u1 u2 u3', positive=True)
# sum_{i<j} beta_i ~ int_{h0}^{hj} dh/h^2 ; sum 2(1-c) beta/h ~ int 2(1-c) dh/h^3
colc = -sp.integrate(1/h**2, (h, h0, hA))
cols = sp.integrate(2*(1 - ch)/h**3, (h, h0, hA))
check("(5) c column  = 1/h_j - 1/h_0", colc - (1/hA - 1/h0))
check("(5) w_s column = (1-c)(1/h_0^2 - 1/h_j^2)", cols - (1 - ch)*(1/h0**2 - 1/hA**2))
# exact law: a(h_j) - a(h_0) = (1-c)(1/h_0 - 1/h_j), h = x - w_s ; sensitivities to c and w_s equal the two columns
c_, ws, x0, xj = sp.symbols('c_ ws x0 xj', real=True)
da = (1 - c_)*(1/(x0 - ws) - 1/(xj - ws))
check("(5) d(a_j - a_0)/dc   = c column  (exact law)", (sp.diff(da, c_) - (1/(xj - ws) - 1/(x0 - ws))))
check("(5) d(a_j - a_0)/dw_s = w_s column (exact law)",
      sp.diff(da, ws) - (1 - c_)*(1/(x0 - ws)**2 - 1/(xj - ws)**2))
# three y2 rows, u = 1/h
row = lambda u: [1, u - u0, (1 - ch)*(u0**2 - u**2)]
D = sp.Matrix([row(u1), row(u2), row(u3)]).det()
check("(5) det = -(1-c)(u2-u1)(u3-u1)(u3-u2)", D + (1 - ch)*(u2 - u1)*(u3 - u1)*(u3 - u2))

print("\nALL PASS" if ok_all else "\nSOME FAILED")
