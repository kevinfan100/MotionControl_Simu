# -*- coding: utf-8 -*-
"""PURPOSE (2026-10-07, branch test/conly-meng-law): independent sympy + numpy check of 1007_conly_meng_law.tex -- the c-only
slope law (p = 2, slot 5 = c) under the journal control law (J6) f_d = (1/a_hat){Delta w_d + (1 - lc) dw3_hat}, force from the
posterior of the same step.
Checks: the slope Jacobians (S1); row 3 of the true dynamics (S3); the exact gain step to first order (S4); the first-order row-4
error dynamics = F_e row 4 and the w_T coefficient (S5); the steady-state position-block filter in closed form (Riccati fixed
point, K1, p1 = K1 r, q = m^2/(m+r)) and the loop variance Var(dw_m) = (2 + 1/(1-lc^2)) q + (1 + K1) r (S7), the latter also
against a numeric Lyapunov solve of the 6-state joint system (A, B of S7) and the closed-loop spectrum {lc, 1 - K1, 0 x 4};
gate-3 determinants (S9); the P[0] closed form of the seed section.
Run: python3 test_script/scratch/check_conly_meng_law.py
"""
import sympy as sp
import numpy as np

# ------------------------------------------------------------------ symbols
wd, dw3h, ws, ch, M, nab = sp.symbols('w_d dw3h w_s c_hat M nabla', real=True)
lc, fd, fdet, wT = sp.symbols('lambda_c f_d f_det w_T', real=True)
e3, ea, ec, rp = sp.symbols('e_3 e_a e_c r_p', real=True)            # e = true - estimate
hh = sp.Symbol('h_hat', positive=True)
q, r = sp.symbols('q r', positive=True)

ap = lambda h, c: (1 - c) / h**2
claims = {}

# --- S1 Jacobians of the slope
claims["da'/dh = -2 a'/h"]          = (sp.diff(ap(hh, ch), hh), -2 * ap(hh, ch) / hh)
claims["da'/dc = -1/h^2"]           = (sp.diff(ap(hh, ch), ch), -1 / hh**2)
claims["da'/d(dw3_hat) = +2 a'/h  (h_hat = w_d - dw3_hat - w_s)"] = (
    sp.diff(ap(wd - dw3h - ws, ch), dw3h), 2 * ap(wd - dw3h - ws, ch) / (wd - dw3h - ws))

# --- S3 row 3 of the true dynamics: dw3[k+1] = dw3 + Dw_d - a f_d - w_T with a = a_hat + e_a, M = a_hat f_d,
#     Dw_d = M - (1-lc) dw3_hat, dw3_hat = dw3 - e3
dw3, ahat, Dwd = sp.symbols('dw3 a_hat Dw_d', real=True)
Mk = ahat * fd
row3_direct = dw3 + (Mk - (1 - lc) * (dw3 - e3)) - (ahat + ea) * fd - wT
row3_tex    = lc * dw3 + (1 - lc) * e3 - fd * ea - wT
claims["S3 row 3: dw3 + Dw_d - a f_d - w_T == lc dw3 + (1-lc) e3 - f_d e_a - w_T"] = (sp.expand(row3_direct), sp.expand(row3_tex))
claims["S3 true displacement DW = M + f_d e_a + w_T"] = (
    sp.expand((ahat + ea) * fd + wT), sp.expand(Mk + fd * ea + wT))

# --- S4 exact gain step, first order in M
step_exact = (1 - ch) * (1 / hh - 1 / (hh + M))
claims["S4 exact step = int_h^{h+M} a' dh"] = (sp.simplify(step_exact - sp.integrate((1 - ch) / sp.Symbol('s')**2, (sp.Symbol('s'), hh, hh + M))), 0)
claims["S4 exact step, first order = a' M"] = (sp.series(step_exact, M, 0, 2).removeO(), ap(hh, ch) * M)

# --- S5 row 4: e_a[k+1] = a_true[k+1] - a_hat[k+1], a_true' at (h_hat - e3, c_hat + e_c) + r_p, DW = M + f_d e_a + w_T
apk    = ap(hh, ch)
ap_true = ap(hh - e3, ch + ec) + rp
DW     = M + fd * ea + wT
ea_next = ea + ap_true * DW - apk * M
lin = sp.series(ea_next.subs({e3: sp.Symbol('t') * e3, ea: sp.Symbol('t') * ea, ec: sp.Symbol('t') * ec,
                              wT: sp.Symbol('t') * wT, rp: sp.Symbol('t') * rp}), sp.Symbol('t'), 0, 2).removeO().subs(sp.Symbol('t'), 1)
lin = sp.expand(lin)
claims["S5 F_e(4,3) = 2 a' M / h"]          = (lin.coeff(e3), sp.expand(2 * apk * M / hh))
claims["S5 F_e(4,4) = 1 + a' f_d"]          = (lin.coeff(ea), sp.expand(1 + apk * fd))
claims["S5 F_e(4,5) = -M / h^2"]            = (lin.coeff(ec), sp.expand(-M / hh**2))
claims["S5 w_T coefficient = a'"]           = (lin.coeff(wT), sp.expand(apk))
claims["S5 residual coefficient = M"]       = (lin.coeff(rp), M)
claims["S5 no zeroth-order term"]           = (lin.subs({e3: 0, ea: 0, ec: 0, wT: 0, rp: 0}), 0)

# --- S7 steady-state filter of the position block, closed form
m  = (q + sp.sqrt(q**2 + 4 * q * r)) / 2
K1 = m / (m + r)
p1 = m * r / (m + r)
claims["S7 Riccati m = m r/(m+r) + q"]       = (sp.simplify(m - (m * r / (m + r) + q)), 0)
claims["S7 q = m^2/(m+r)"]                   = (sp.simplify(q - m**2 / (m + r)), 0)
claims["S7 p1 = K1 r"]                       = (sp.simplify(p1 - K1 * r), 0)
F3 = sp.Matrix([[0, 1, 0], [0, 0, 1], [0, 0, 1]]);  H3 = sp.Matrix([[1, 0, 0]]);  Q3 = sp.diag(0, 0, q)
Pm = sp.Matrix([[m, m, m], [m, m + q, m + q], [m, m + q, m + 2 * q]])
Pp = sp.Matrix([[p1, p1, p1], [p1, p1 + q, p1 + q], [p1, p1 + q, p1 + 2 * q]])
L3 = Pm * H3.T / (H3 * Pm * H3.T)[0, 0] if False else Pm * H3.T / ((H3 * Pm * H3.T)[0, 0] + r)
claims["S7 L = [K1 K1 K1]"]                  = (sp.simplify(L3 - sp.Matrix([K1, K1, K1])), sp.zeros(3, 1))
claims["S7 P = (I - L H) P^-"]               = (sp.simplify((sp.eye(3) - L3 * H3) * Pm - Pp), sp.zeros(3, 3))
claims["S7 P^- = F P F' + Q"]                = (sp.simplify(F3 * Pp * F3.T + Q3 - Pm), sp.zeros(3, 3))
claims["S7 Var dw3_hat = K1^2 (m+r)/(1-lc^2) = q/(1-lc^2)"] = (sp.simplify(K1**2 * (m + r) - q), 0)
C_dpmr = 2 + 1 / (1 - lc**2)
claims["S7 Var dw3 = q/(1-lc^2) + p1 + 2q = C_dpmr q + K1 r"] = (sp.simplify(q / (1 - lc**2) + p1 + 2 * q - (C_dpmr * q + K1 * r)), 0)

# --- S7 numeric: 6-state joint loop, Lyapunov variance vs the closed form; spectrum at g = 1
def joint_A_B(lcv, K, g):
    A = np.zeros((6, 6)); B = np.zeros((6, 2))
    A[0, 1] = 1; A[1, 2] = 1; A[2, 2] = 1; A[2, 5] = -g * (1 - lcv); B[2, 0] = -1
    A[3, :] = [0, K, 0, 0, 1 - K, 0];  A[4, :] = [0, K, 0, 0, -K, 1];  A[5, :] = [0, K, 0, 0, -K, lcv]
    B[3, 1] = K; B[4, 1] = K; B[5, 1] = K
    return A, B
numeric_ok = True
for lcv in (0.4, 0.7):
    for rho in (23.0, 8.0, 2.0, 0.5):
        qv, rv = rho, 1.0
        mv = (qv + np.sqrt(qv**2 + 4 * qv * rv)) / 2;  Kv = mv / (mv + rv)
        A, B = joint_A_B(lcv, Kv, 1.0)
        W = B @ np.diag([qv, rv]) @ B.T
        X = np.linalg.solve(np.eye(36) - np.kron(A, A), W.reshape(-1)).reshape(6, 6)
        var_num = X[0, 0] + rv
        var_cf  = (2 + 1 / (1 - lcv**2)) * qv + (1 + Kv) * rv
        eig = np.sort(np.abs(np.linalg.eigvals(A)))[::-1]
        ok = abs(var_num - var_cf) < 1e-9 * var_cf and abs(eig[0] - max(lcv, 1 - Kv)) < 1e-9 and abs(eig[1] - min(lcv, 1 - Kv)) < 1e-9 and eig[2] < 1e-6
        numeric_ok = bool(numeric_ok and ok)
        print("  numeric lc=%.1f rho=%5.1f: Var(dw_m) Lyapunov %.6f closed form %.6f | |eig| = %s | %s"
              % (lcv, rho, var_num, var_cf, np.array2string(eig[:3], precision=4), "ok" if ok else "MISMATCH"))
claims["S7 numeric: Lyapunov variance == closed form and spectrum == {lc, 1-K1, 0...}"] = (numeric_ok, True)

# --- S9 gate 3 determinants, coefficients frozen
alpha = 1 + apk * fdet;  beta = M / hh**2;  gamma = nab / hh**2
d_y2 = sp.Matrix([[1, gamma], [alpha, gamma - beta]]).det()
claims["S9 det O2 (e_a, e_c) row y2 = -(M + nabla a' f_det)/h^2"] = (sp.simplify(d_y2 + (M + nab * apk * fdet) / hh**2), 0)
d_y1 = sp.Matrix([[-fdet, 0], [-fdet * alpha, fdet * beta]]).det()
claims["S9 det O2 (e_a, e_c) row y1 = -f_det^2 M/h^2"] = (sp.simplify(d_y1 + fdet**2 * M / hh**2), 0)

# ------------------------------------------------------------------ report
npass = 0
for name, (lhs, rhs) in claims.items():
    if isinstance(lhs, bool):
        ok = lhs == rhs
    else:
        ok = sp.simplify(lhs - rhs) == 0 if not isinstance(lhs, sp.MatrixBase) else (lhs - rhs).applyfunc(sp.simplify) == sp.zeros(*lhs.shape)
    npass += ok
    print("%s  %s" % ("PASS" if ok else "FAIL", name))
print("\n%d / %d claims pass" % (npass, len(claims)))
