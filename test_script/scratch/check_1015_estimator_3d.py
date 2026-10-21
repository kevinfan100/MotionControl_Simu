# PURPOSE (2026-10-15): independent check of reference/eq17_analysis/derivation/1015_estimator_3d_wsonly.tex
#   (three-axis estimator: a_T, a_N, one shared w_s, wall angles theta_x, theta_y; tracking-error states in the
#   measurement coordinate). Exact sympy derivatives, compared with the closed forms written in the tex at random points.
#   (1) Sec. 2: V = R_y R_x, A = V diag V' = a_T I + (a_N - a_T) w w', dw/dtheta, dA/dtheta.
#   (2) Sec. 3: tracking-error dynamics from the motion model + control law + d-step delay (d = 2), exact identity.
#   (3) Sec. 6 (a)/(b): w - w^, h - h^, A - A^, normal-displacement difference, first order.
#   (4) Sec. 6 (c): rows e3, e_aT, e_aN of F_e and the q vector, from the exact one-step error.
#   (5) Sec. 7: first-order expectations of S~ (gains and angle entries), sign of theta_m.
#   (6) Sec. 8: R entries from Gaussian fourth moments (Isserlis), cross-covariances at zero angle error.
# EXPIRES: with the tex | run: python3 test_script/scratch/check_1015_estimator_3d.py
import random
import sympy as sp

random.seed(7)
ok_all = True
TOL = 1e-10


def rnd(lo, hi):
    return sp.Float(random.uniform(lo, hi), 30)


def check(name, expr, subs_list):
    """expr: sympy Matrix or scalar that must vanish; evaluated at every point in subs_list."""
    global ok_all
    worst = 0.0
    for s in subs_list:
        v = sp.Matrix([expr]) if not isinstance(expr, sp.MatrixBase) else expr
        val = v.subs(s).evalf(30)
        worst = max(worst, max(abs(float(x)) for x in val))
    ok = worst < TOL
    ok_all &= ok
    print(f"{'PASS' if ok else 'FAIL'}  {name}   (max residual {worst:.1e})")


def Rx(a):
    return sp.Matrix([[1, 0, 0], [0, sp.cos(a), -sp.sin(a)], [0, sp.sin(a), sp.cos(a)]])


def Ry(a):
    return sp.Matrix([[sp.cos(a), 0, sp.sin(a)], [0, 1, 0], [-sp.sin(a), 0, sp.cos(a)]])


def vec(name):
    return sp.Matrix(sp.symbols(f'{name}0:3', real=True))


I3 = sp.eye(3)
t = sp.symbols('t', real=True)

# ----------------------------------------------------------------------------- (1) Sec. 2
tx, ty, aT, aN = sp.symbols('tx ty aT aN', real=True)
V = Ry(ty) * Rx(tx)
u, v, w = V[:, 0], V[:, 1], V[:, 2]
A = V * sp.diag(aT, aT, aN) * V.T
pts1 = [{tx: rnd(-0.6, 0.6), ty: rnd(-0.6, 0.6), aT: rnd(0.2, 1), aN: rnd(0.05, 0.9)} for _ in range(4)]
check("(1) w = (sin ty cos tx, -sin tx, cos ty cos tx)  [math-model.md]",
      w - sp.Matrix([sp.sin(ty) * sp.cos(tx), -sp.sin(tx), sp.cos(ty) * sp.cos(tx)]), pts1)
check("(1) A = a_T I + (a_N - a_T) w w'", A - (aT * I3 + (aN - aT) * w * w.T), pts1)
check("(1) dA/da_T = I - w w' = u u' + v v'", A.diff(aT) - (u * u.T + v * v.T), pts1)
check("(1) dA/da_N = w w'", A.diff(aN) - w * w.T, pts1)
check("(1) dw/dtheta_x = -v", w.diff(tx) + v, pts1)
check("(1) dw/dtheta_y = cos(theta_x) u", w.diff(ty) - sp.cos(tx) * u, pts1)
check("(1) dA/dtheta_x = -(a_N - a_T)(v w' + w v')", A.diff(tx) + (aN - aT) * (v * w.T + w * v.T), pts1)
check("(1) dA/dtheta_y = (a_N - a_T) cos(theta_x)(u w' + w u')", A.diff(ty) - (aN - aT) * sp.cos(tx) * (u * w.T + w * u.T), pts1)

# ----------------------------------------------------------------------------- (2) Sec. 3, exact identity, d = 2
# p[k+1] = p + A0 (f0 + fT0); p[k] = p[k-2] + A1 (f1 + fT1) + A2 (f2 + fT2); index i = k - i.
lc = sp.symbols('lc', real=True)
def rmat(name):
    return sp.Matrix(3, 3, list(sp.symbols(f'{name}0:9', real=True)))
A0, A1, A2, Ah0, Ah1, Ah2 = (rmat(n) for n in ('A0_', 'A1_', 'A2_', 'Ah0_', 'Ah1_', 'Ah2_'))
f1, f2, fT0, fT1, fT2, n_p, pk2, pd0, pd1 = (vec(n) for n in ('f1_', 'f2_', 'fT0_', 'fT1_', 'fT2_', 'np_', 'pk2_', 'pd0_', 'pd1_'))
pk = pk2 + A1 * (f1 + fT1) + A2 * (f2 + fT2)                   # true position at k
pm = pk2 + n_p                                                # measured position at k (delay 2)
ph = pm + Ah1 * f1 + Ah2 * f2                                 # predicted position
f0 = Ah0.inv() * (pd1 - ph - lc * (pd0 - ph))                 # control law
pk1 = pk + A0 * (f0 + fT0)
dp_next = pd1 - pk1
dp_now = pd0 - pk
exact = lc * dp_now - (A0 - Ah0) * f0 - (1 - lc) * ((A1 - Ah1) * f1 + (A2 - Ah2) * f2) \
    - (A0 * fT0 + (1 - lc) * (A1 * fT1 + A2 * fT2) - (1 - lc) * n_p)
allsym = list(set().union(*[m.free_symbols for m in (A0, A1, A2, Ah0, Ah1, Ah2, f1, f2, fT0, fT1, fT2, n_p, pk2, pd0, pd1)]) | {lc})
pts2 = []
for _ in range(2):
    s = {x: rnd(-1, 1) for x in allsym}
    for M_ in (Ah0,):
        for i in range(3):
            s[M_[i, i]] = rnd(2, 3)                           # keep A^ invertible
    s[lc] = rnd(0.2, 0.6)
    pts2.append(s)
check("(2) dp[k+1] = lc dp - (A-A^)[k] f_d[k] - (1-lc) sum (A-A^)[k-i] f_d[k-i] - eps_p  (exact)", dp_next - exact, pts2)
# the tex form: same A - A^ over the last d steps -> -(A-A^) F_dp
same = {**{A1[i]: A0[i] for i in range(9)}, **{A2[i]: A0[i] for i in range(9)},
        **{Ah1[i]: Ah0[i] for i in range(9)}, **{Ah2[i]: Ah0[i] for i in range(9)}}
F_dp = f0 + (1 - lc) * (f1 + f2)
tex3 = lc * dp_now - (A0 - Ah0) * F_dp - (A0 * fT0 + (1 - lc) * (A1 * fT1 + A2 * fT2) - (1 - lc) * n_p)
check("(2) with A-A^ equal over k-2..k: dp[k+1] = lc dp - (A-A^) F_dp - eps_p  (tex Sec. 3)",
      (dp_next - tex3).subs(same), pts2)

# ----------------------------------------------------------------------------- (3)-(4) Sec. 6
# estimates (hat) and errors e; true = hat + t * e
txh, tyh, aTh, aNh, wsh, hcs = sp.symbols('txh tyh aTh aNh wsh hcs', real=True)
etx, ety, eaT, eaN, ews = sp.symbols('etx ety eaT eaN ews', real=True)
pd, dpd, dp3h, e3, F, eps = (vec(n) for n in ('pd_', 'dpd_', 'dp3h_', 'e3_', 'F_', 'eps_'))
cT = sp.Rational(9, 16)
Vh = Ry(tyh) * Rx(txh)
uh, vh, wh = Vh[:, 0], Vh[:, 1], Vh[:, 2]
Ah = aTh * I3 + (aNh - aTh) * wh * wh.T
hh = (wh.T * (pd - dp3h))[0] - wsh
aTph, aNph = cT / hh**2, 1 / hh**2
Mh = dpd + (1 - lc) * dp3h

def true_of(tt):
    Vt = Ry(tyh + tt * ety) * Rx(txh + tt * etx)
    wt = Vt[:, 2]
    aTt, aNt = aTh + tt * eaT, aNh + tt * eaN
    At = aTt * I3 + (aNt - aTt) * wt * wt.T
    dp3 = dp3h + tt * e3
    h = (wt.T * (pd - dp3))[0] - (wsh + tt * ews)
    M = dpd + (1 - lc) * dp3
    disp = M + (At - Ah) * F + tt * eps                       # actual displacement p[k+1] - p[k]
    return dict(w=wt, A=At, h=h, M=M, dn=(wt.T * disp)[0], aT=aTt, aN=aNt, dp3=dp3)

T_ = true_of(t)
first = lambda x: x.diff(t).subs(t, 0)
syms6 = [txh, tyh, aTh, aNh, wsh, lc, etx, ety, eaT, eaN, ews] + list(pd) + list(dpd) + list(dp3h) + list(e3) + list(F) + list(eps)
pts6 = []
for _ in range(4):
    s = {x: rnd(-1, 1) for x in syms6}
    s.update({txh: rnd(-0.5, 0.5), tyh: rnd(-0.5, 0.5), aTh: rnd(0.4, 0.9), aNh: rnd(0.05, 0.35),
              wsh: rnd(-0.2, 0.2), lc: rnd(0.2, 0.6)})
    s.update({pd[2]: rnd(1.5, 3)})                            # h^ positive (z component dominates w^)
    pts6.append(s)

pr = pd - dp3h
cx = sp.cos(txh)
check("(3a) w - w^ = -v^ e_tx + cos tx^ u^ e_ty", first(T_['w']) - (-vh * etx + cx * uh * ety), pts6)
check("(3a) h - h^ = -w^'e3 - e_ws - v^'(p_d - dp3^) e_tx + cos tx^ u^'(p_d - dp3^) e_ty",
      first(T_['h']) - (-(wh.T * e3)[0] - ews - (vh.T * pr)[0] * etx + cx * (uh.T * pr)[0] * ety), pts6)
dA_tex = (uh * uh.T + vh * vh.T) * eaT + wh * wh.T * eaN - (aNh - aTh) * (vh * wh.T + wh * vh.T) * etx \
    + (aNh - aTh) * cx * (uh * wh.T + wh * uh.T) * ety
check("(3a) A - A^ first order", first(T_['A']) - dA_tex, pts6)
dn_tex = (1 - lc) * (wh.T * e3)[0] + (wh.T * F)[0] * eaN + ((aTh - aNh) * (vh.T * F)[0] - (vh.T * Mh)[0]) * etx \
    + cx * ((uh.T * Mh)[0] - (aTh - aNh) * (uh.T * F)[0]) * ety + (wh.T * eps)[0]
check("(3b) normal displacement true - estimated (first order)", first(T_['dn']) - dn_tex, pts6)

# one-step errors: true step minus estimator prediction (no L e_y term)
e3_next = first(lc * T_['dp3'] - (T_['A'] - Ah) * F - t * eps)                    # estimator: lc dp3^
e3_tex = lc * e3 - (uh * uh.T + vh * vh.T) * F * eaT - wh * wh.T * F * eaN \
    + (aNh - aTh) * (vh * wh.T + wh * vh.T) * F * etx - (aNh - aTh) * cx * (uh * wh.T + wh * uh.T) * F * ety - eps
check("(4) row e3 of F_e and q (-eps_p)", e3_next - e3_tex, pts6)

def gain_row(c, eself, self_coef_extra):
    ap_h = c / hh**2
    true_next = (aTh + t * eaT if c == cT else aNh + t * eaN) + c / T_['h']**2 * T_['dn']
    est_next = (aTh if c == cT else aNh) + ap_h * (wh.T * Mh)[0]
    got = first(true_next - est_next)
    wM = (wh.T * Mh)[0]
    tex = (ap_h * (1 - lc) + 2 * ap_h / hh * wM) * (wh.T * e3)[0] + eself * (1 + self_coef_extra) \
        + 2 * ap_h / hh * wM * ews \
        + (ap_h * ((aTh - aNh) * (vh.T * F)[0] - (vh.T * Mh)[0]) + 2 * ap_h / hh * wM * (vh.T * pr)[0]) * etx \
        + cx * (ap_h * ((uh.T * Mh)[0] - (aTh - aNh) * (uh.T * F)[0]) - 2 * ap_h / hh * wM * (uh.T * pr)[0]) * ety \
        + ap_h * (wh.T * eps)[0]
    if c == cT:
        tex = tex + ap_h * (wh.T * F)[0] * eaN                 # e_aN column of the e_aT row
    return got - tex

check("(4) row e_aT of F_e and q (self 1, e_aN column a_T' w^'F_dp)", gain_row(cT, eaT, 0), pts6)
check("(4) row e_aN of F_e and q (self 1 + a_N' w^'F_dp)", gain_row(sp.Integer(1), eaN, (1 / hh**2) * (wh.T * F)[0]), pts6)

# ----------------------------------------------------------------------------- (5) Sec. 7, expectations of S~
# E[S~] = V^' A V^ (noise term subtracted exactly); first order in the angle errors at the true gains
At_ = true_of(t)['A']
S = Vh.T * At_ * Vh
S1 = S.subs(t, 0) + first(S)                                 # zero + first order
ptsS = pts6
check("(5) E[(S_uu + S_vv)/2] = a_T to first order (angle errors only shift it at 2nd order)",
      first((S[0, 0] + S[1, 1]) / 2) - eaT, ptsS)
check("(5) E[S_ww] = a_N to first order", first(S[2, 2]) - eaN, ptsS)
check("(5) E[S_vw] = (a_T - a_N) e_tx to first order", first(S[1, 2]) - (aTh - aNh) * etx, ptsS)
check("(5) E[S_uw] = -(a_T - a_N) cos tx^ e_ty to first order", first(S[0, 2]) + (aTh - aNh) * cx * ety, ptsS)
check("(5) E[S_uv] = 0 to first order (rotation about the normal not seen)", first(S[0, 1]), ptsS)
thx_m = txh + S[1, 2] / (aTh - aNh)
thy_m = tyh - S[0, 2] / ((aTh - aNh) * cx)
check("(5) E[theta_xm] - theta_x^ = e_tx  (sign of the tex line)", first(thx_m) - etx, ptsS)
check("(5) E[theta_ym] - theta_y^ = e_ty  (sign of the tex line)", first(thy_m) - ety, ptsS)

# ----------------------------------------------------------------------------- (6) Sec. 8, R from Gaussian fourth moments
# one zero-mean Gaussian sample r with covariance Sig (local, already / C_T kappa_T): Cov(r_i r_j, r_k r_l) = S_ik S_jl + S_il S_jk
sT, sN, xT, xN = sp.symbols('sT sN xT xN', positive=True)
Sig = sp.diag(sT + xT, sT + xT, sN + xN)                     # zero angle error, noise already rotated to local
def cov4(i, j, k, l):
    return Sig[i, k] * Sig[j, l] + Sig[i, l] * Sig[j, k]
var_T = (cov4(0, 0, 0, 0) + 2 * cov4(0, 0, 1, 1) + cov4(1, 1, 1, 1)) / 4
check("(6) var((S_uu + S_vv)/2) = (a_T + xi_T)^2", sp.simplify(var_T - (sT + xT)**2), [{}])
check("(6) var(S_ww) = 2 (a_N + xi_N)^2", sp.simplify(cov4(2, 2, 2, 2) - 2 * (sN + xN)**2), [{}])
check("(6) var(S_vw) = var(S_uw) = (a_T + xi_T)(a_N + xi_N)", sp.simplify(cov4(1, 2, 1, 2) - (sT + xT) * (sN + xN)), [{}])
cross = sp.Matrix([cov4(0, 0, 2, 2), cov4(0, 0, 1, 2), cov4(0, 0, 0, 2), cov4(2, 2, 1, 2), cov4(1, 2, 0, 2)])
check("(6) cross-covariances of a_Tm, a_Nm, S_vw, S_uw = 0 when the local covariance is diagonal", cross, [{}])
# sensor noise rotated: diagonal only if Sigma_n is isotropic or the angle is zero
sx, sy, sz = sp.symbols('sx sy sz', positive=True)
Sn_loc = Vh.T * sp.diag(sx, sy, sz) * Vh
off = sp.Matrix([Sn_loc[0, 2], Sn_loc[1, 2]])
iso = [{txh: rnd(-0.5, 0.5), tyh: rnd(-0.5, 0.5), sx: 1, sy: 1, sz: 1}]
check("(6) rotated sensor noise V^' Sigma_n V^ diagonal when Sigma_n is isotropic", off, iso)
aniso = [{txh: 0.3, tyh: 0.3, sx: 1, sy: 1, sz: 2}]
val = [float(x) for x in off.subs(aniso[0]).evalf()]
print(f"INFO  anisotropic Sigma_n (1, 1, 2), angles 0.3: off-diagonal (uw, vw) of V^' Sigma_n V^ = {val[0]:+.3f}, {val[1]:+.3f}"
      "  -> the MEAN is removed exactly (C_n Sigma_n subtracted before rotating), but the noise part of the local"
      " covariance is then not diagonal, so R picks up cross-covariances (xi terms) unless Sigma_n is isotropic")

# ----------------------------------------------------------------------------- (7) 10-19 version: a-bar' as a state variable
# a_T[k+1] = a_T + a_T' w'(M + (A - A^) F + eps), a_T' = a_T'^ + e_a'T (a state, no h); estimator a_T^ + a_T'^ w^'M^
aTph, aNph, eaTp, eaNp = sp.symbols('aTph aNph eaTp eaNp', real=True)
pts7 = []
for s in pts6:
    s7 = dict(s);  s7.update({aTph: rnd(0.01, 0.6), aNph: rnd(0.01, 1.0), eaTp: rnd(-1, 1), eaNp: rnd(-1, 1)})
    pts7.append(s7)
wM = (wh.T * Mh)[0]
for name, a_h, ap_h, e_self, e_ap, extra in (('a_T', aTh, aTph, eaT, eaTp, 0), ('a_N', aNh, aNph, eaN, eaNp, 1)):
    got = first((a_h + t * e_self) + (ap_h + t * e_ap) * T_['dn'] - (a_h + ap_h * wM))
    tex = ap_h * (1 - lc) * (wh.T * e3)[0] + e_self + wM * e_ap \
        + ap_h * ((aTh - aNh) * (vh.T * F)[0] - (vh.T * Mh)[0]) * etx \
        + ap_h * cx * ((uh.T * Mh)[0] - (aTh - aNh) * (uh.T * F)[0]) * ety + ap_h * (wh.T * eps)[0] \
        + ap_h * (wh.T * F)[0] * eaN
    check(f"(7) row e_{name} with {name}' as a state (Section 5 (c), 10-19 version)", got - tex, pts7)

print("\nALL PASS" if ok_all else "\nSOME FAILED")
