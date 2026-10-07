# STATUS: SCRATCH -- 2026-10-07, sympy check of motion_ctrl Section 1 (sec1_model.tex): tracking-error dynamics and the law form, d = 2. Expected output: two zeros.
import sympy as sp
lc = sp.symbols('lambda_c')
k = 3
a = {j: sp.Symbol(f'a{j}') for j in range(k-2, k+1)}      # true motion gain
ah = {j: sp.Symbol(f'ah{j}') for j in range(k-2, k+1)}    # gain used in the law
fT = {j: sp.Symbol(f'fT{j}') for j in range(k-2, k+1)}
xd = {j: sp.Symbol(f'xd{j}') for j in range(k-2, k+2)}
n = sp.Symbol('n')
x_km2 = sp.Symbol('x_km2')
fd = {k-2: sp.Symbol('fd1'), k-1: sp.Symbol('fd2')}       # past law outputs (free)
x = {k-2: x_km2}
x[k-1] = x[k-2] + a[k-2]*(fd[k-2] + fT[k-2])
x[k] = x[k-1] + a[k-1]*(fd[k-1] + fT[k-1])
xm = x[k-2] + n                                            # d = 2 delay
xhat = xm + ah[k-1]*fd[k-1] + ah[k-2]*fd[k-2]
fd[k] = (xd[k+1] - xhat - lc*(xd[k] - xhat)) / ah[k]
x[k+1] = x[k] + a[k]*(fd[k] + fT[k])
dx_k = xd[k] - x[k]
dx_kp1 = xd[k+1] - x[k+1]
e = {j: a[j] - ah[j] for j in a}
eps_x = a[k]*fT[k] + (1-lc)*(a[k-1]*fT[k-1] + a[k-2]*fT[k-2]) - (1-lc)*n
eps = e[k]*fd[k] + (1-lc)*(e[k-1]*fd[k-1] + e[k-2]*fd[k-2]) + eps_x
print('dx[k+1] - (lc dx[k] - eps) =', sp.simplify(dx_kp1 - (lc*dx_k - eps)))
# law equals the code form: xd[k+1] - lc xd[k] - (1-lc)(xd[k-d] - dxm) ... with dxm = xd[k-2] - xm
dxm = xd[k-2] - xm
law_code = (xd[k+1] - lc*xd[k] - (1-lc)*xd[k-2] + (1-lc)*dxm - (1-lc)*(ah[k-1]*fd[k-1] + ah[k-2]*fd[k-2])) / ah[k]
print('law (doc) - law (code form) =', sp.simplify(fd[k] - law_code))
