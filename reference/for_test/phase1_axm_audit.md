# Phase 1 — `a_xm` Derivation + Code Cross-Check Audit

**Scope**: writeup_architecture.tex Sections 5, 6, 6.6 — derive from scratch, match to code.
**Out of scope**: simulation runs, EKF Q/R tuning, documentation rewrite.
**Status legend**: `[OK]` match, `[ISSUE]` discrepancy, `[VERIFY]` needs numerical check, `[FLAG]` note for later phase.

---

## 0. Setup — indexing conventions

**Writeup** (Sec 5.1):
```
e[k] = [δx_1, δx_2, δx_3,  e_{x1}, e_{x2}, e_{x3}, e_{xD}, e_{δxD}, e_a, e_{δa},  δx_{md}[k-1]]^T
       ↑oldest  ↑          ↑newest                                                ↑IIR memory
       (sensor sees)       (control uses)
```
Position 1 = `δx[k-2]` (oldest, what sensor sees)
Position 3 = `δx[k]` (newest, feedback to control law with λ_c)
Position 11 = `δx_{md}[k-1]`

**Code** (`compute_7state_cdpmr_eff.m` lines 103–107):
```
idx_dx       = 1    →  delta_x   = δx[k]       (newest)
idx_dx_d1    = 2    →  dx_d1     = δx[k-1]
idx_dx_d2    = 3    →  dx_d2     = δx[k-2]     (oldest, sensor)
idx_e(1..7)  = 4:10 →  7 EKF error states
idx_pmd_prev = 11   →  del_pmd[k-1]            (IIR memory)
```

**Mapping** (code ↔ writeup):
| Code idx | Code meaning | Writeup idx | Writeup meaning |
|---|---|---|---|
| 1 | δx[k] (newest) | 3 | δx_3 (newest) |
| 2 | δx[k-1] | 2 | δx_2 |
| 3 | δx[k-2] (sensor) | 1 | δx_1 (sensor) |
| 4–10 | e1..e7 | 4–10 | e_{x1}..e_{δa} |
| 11 | del_pmd[k-1] | 11 | δx_{md}[k-1] |

Note: positions 1 and 3 are **swapped** between the two conventions. Selector in writeup uses indices (1, 11); code uses (3, 11) — both point to `oldest δx` and `IIR memory`. Physically equivalent.

`[OK]` — both are self-consistent; just a labelling choice. Will flag for Phase 4 docs (writeup or code comment should explicitly note the mapping).

---

## 1. Section 5.2 — closed-loop recursion for δx_3

**Writeup claim** (Eq. 5):
```
δx_3[k+1] = λ_c·δx_3[k] + (1-λ_c)·e_{x3}[k] - e_{xD}[k] - a_x[k]·f_T[k]
```

### Derivation from plant + control law

Plant (Sec 1 Eq. 3):
```
x[k+1] = x[k] + a_x[k]·(f_dx[k] + f_T[k])
```

Control law (Sec 1 Eq. 4):
```
f_dx[k] = (1/â_x[k]) · [(x_d[k+1] - x_d[k]) + (1-λ_c)·δ\hat{x}_3[k] - \hat{x}_D[k]]
```

With §5 scope assumption `â_x ≈ a_x` (simplification #4), so `a_x·f_dx = (x_d[k+1]-x_d[k]) + (1-λ_c)·δ\hat{x}_3 - \hat{x}_D`.

Compute `δx[k+1] = x_d[k+1] - x[k+1]`:
```
δx[k+1] = x_d[k+1] - x[k] - a_x·f_dx[k] - a_x·f_T[k]
        = x_d[k+1] - (x_d[k] - δx[k]) - [(x_d[k+1]-x_d[k]) + (1-λ_c)·δ\hat{x}_3 - \hat{x}_D] - a_x·f_T
        = δx[k] - (1-λ_c)·δ\hat{x}_3 + \hat{x}_D - a_x·f_T
```

Now split estimates via error definitions (code: a = a_hat + (a - a_hat) = a_hat + e_a; similarly x_D = \hat{x}_D + e_{xD}, δx_3 = δ\hat{x}_3 + e_{x3}):
- `\hat{x}_D = x_D - e_{xD}`. Under `x_D = 0` (simplification #2), `\hat{x}_D = -e_{xD}`.
- `δ\hat{x}_3 = δx_3 - e_{x3} = δx[k] - e_{x3}` (since δx_3 = δx[k] in writeup convention).

Substitute:
```
δx[k+1] = δx[k] - (1-λ_c)·(δx[k] - e_{x3}) + (-e_{xD}) - a_x·f_T
        = δx[k] - (1-λ_c)·δx[k] + (1-λ_c)·e_{x3} - e_{xD} - a_x·f_T
        = λ_c·δx[k] + (1-λ_c)·e_{x3}[k] - e_{xD}[k] - a_x[k]·f_T[k]
```

✓ Matches writeup Eq. 5 exactly.

`[OK]` Section 5.2 derivation correct under stated simplifications.

### Code check
`compute_7state_cdpmr_eff.m` lines 113–116:
```matlab
A_aug(idx_dx, idx_dx)   = lc;      % δx[k+1] ← λ_c · δx[k]
A_aug(idx_dx, idx_e(3)) = 1 - lc;  % δx[k+1] ← (1-λ_c) · e_{x3}[k]
A_aug(idx_dx, idx_e(4)) = -1;      % δx[k+1] ← -e_{xD}[k]
A_aug(idx_dx, idx_e(6)) = -opts.f0;% (dropped — #3 simplification)
```

`idx_e(3) = 6` → 3rd EKF error = e_{x3}  ✓
`idx_e(4) = 7` → 4th EKF error = e_{xD}  ✓
Thermal input `-a_x·f_T` enters via `B_th(idx_dx) = -1` (line 147) ✓

`[OK]` Code matches writeup.

---

## 2. Section 5.3 — A matrix block structure

### Row 1 (δx_1[k+1] = δx_2[k])

Writeup: `A(1, 2) = 1`. Code (under index mapping: writeup 1 ↔ code 3, writeup 2 ↔ code 2):
`A_aug(3, 2) = 1`. Line 122: ✓

### Row 2 (δx_2[k+1] = δx_3[k])

Writeup: `A(2, 3) = 1`. Code (writeup 3 ↔ code 1): `A_aug(2, 1) = 1`. Line 119: ✓

### Row 3 (δx_3[k+1] — derived in Sec 5.2)

Writeup: `A(3, 3) = λ_c`, `A(3, 6) = 1 - λ_c`, `A(3, 7) = -1`, rest 0.
Under §5 scope (f_dx → 0), no `-f_dx` term at (3, 6+3) = (3, 9).

Code (writeup 3 ↔ code 1, e_{x3} at writeup 6 ↔ code 6, e_{xD} at writeup 7 ↔ code 7, e_a at writeup 9 ↔ code 9):
- `A_aug(1, 1) = lc` ✓ (line 113)
- `A_aug(1, 6) = 1 - lc` ✓ (line 114, `idx_e(3) = 6`)
- `A_aug(1, 7) = -1` ✓ (line 115, `idx_e(4) = 7`)
- `A_aug(1, 9) = -opts.f0` — code keeps `f_0` as a parameter; set to 0 in Phase 1 build (lookup built with `opts.f0 = 0`). Matches writeup scope note.

`[OK]`

### Rows 4–10 (EKF error block A_e = Fe·(I - L·H))

Writeup block: `[0_{7×3} | A_e | 0_{7×1}]`.
Code: `A_aug(idx_e, idx_e) = A_e` (line 125), with `A_e = Fe - Fe*L*H` (line 89). ✓

Fe under §5 scope has (3, 6) entry set to 0 (f_dx drop). Code uses `opts.f0 = 0`. ✓

`[OK]`

### Row 11 (δx_md recursion)

Physical:
```
δx_md[k+1] = (1-a_pd)·δx_md[k] + a_pd·δx_m[k]
           = (1-a_pd)·δx_md[k] + a_pd·(δx_1[k] - n[k])     (writeup's n convention)
```
where δx_m = δx_1 - n (writeup §5.5).

Writeup `A(11, 1) = a_pd`, `A(11, 11) = 1 - a_pd`, coefficient of n goes into b_n.

Code (writeup 1 ↔ code 3):
- `A_aug(11, 3) = a_pd` ✓ (line 131)
- `A_aug(11, 11) = 1 - a_pd` ✓ (line 130)

`[OK]` A_aug structure correct in both writeup and code.

### b_T (thermal noise vector)

Thermal enters δx_3[k+1] and e_{x3}[k+1] with coefficient `-1` (via `-a_x·f_T` term). Elsewhere 0.

Writeup: `b_T = [0, 0, -1 | 0, 0, -1, 0, 0, 0, 0 | 0]` (positions 3 and 6).
Code: `B_th(idx_dx)=-1, B_th(idx_e(3))=-1` → writeup 3 and 6. ✓

Derivation of b_T(6) = -1: the true δx_3[k+1] gets -a_x·f_T kick; the EKF prediction \hat{δx}_3[k+1|k] doesn't (process noise unknown); measurement correction at step k depends on past. So e_{x3}[k+1] = δx_3[k+1] - \hat{δx}_3[k+1|k] + correction ≈ -a_x·f_T carry-through (plus other terms). Confirmed -1.

`[OK]` b_T matches.

### b_n (sensor noise vector) — **POTENTIAL ISSUE**

**Writeup claim** (§5.3): `b_n = [0, 0, 0 | -Fe·L_ss(:,1) | -a_pd]^T`

**Re-derivation from first principles**:

1. Writeup defines `δx_m[k] = δx_1[k] - n[k]` (§5.5). n is the physical sensor noise (additive to x_m, subtracts from δx_m).

2. Measurement model: `y[k] = H·x[k] + v[k]`. y[1] = δx_m. H·x selects δx_1 (position 1). So `v[1] = y[1] - (H·x)[1] = δx_m - δx_1 = -n`.

3. Writeup's KF state-update (Sec 3, last line):
   `\hat{x}[k+1|k] = Fe·\hat{x}[k|k-1] + Fe·L_ss·(y[k] - H·\hat{x}[k|k-1])`

4. Error dynamics:
   ```
   e[k+1] = x[k+1] - \hat{x}[k+1|k]
          = Fe·x[k] + q[k] - Fe·\hat{x}[k|k-1] - Fe·L_ss·(H·e[k] + v[k])
          = Fe·(I - L_ss·H)·e[k] - Fe·L_ss·v[k] + q[k]
          = A_e·e[k] - Fe·L_ss·v[k] + q[k]
   ```

5. Contribution from n to e[k+1]: `-Fe·L_ss(:,1)·v[1] = -Fe·L_ss(:,1)·(-n) = +Fe·L_ss(:,1)·n`.

6. So in `e[k+1] = ... + b_n·n`: **b_n(err states) should be +Fe·L_ss(:,1)**, not `-Fe·L_ss(:,1)`.

7. b_n(position 11) from δx_md update: coefficient of n is `-a_pd` (from `+a_pd·(δx_1 - n) = +a_pd·δx_1 - a_pd·n`). Writeup `-a_pd` ✓.

**Physically correct b_n** (writeup convention): `[0, 0, 0 | +Fe·L_ss(:,1) | -a_pd]`

**Writeup Sec 5.3 states**: `[0, 0, 0 | -Fe·L_ss(:,1) | -a_pd]`

**Code `compute_7state_cdpmr_eff.m` lines 152–154**:
```matlab
B_np(idx_e)        = -(Fe * L(:, 1));
B_np(idx_pmd_prev) = a_pd;
```
So code b_n = `[0, 0, 0 | -Fe·L(:,1) | +a_pd]`. Code uses **opposite convention for n** (code's "n_p" = -writeup's "n", see line 129 comment `del_pm[k] = dx_d2[k] + n_p[k]`).

**Comparison via Lyapunov invariant (b·b' = (-b)·(-b)')**:
- Physically correct: `b_true = [0, 0, 0 | +g | -a_pd]`, `g = Fe·L(:,1)`
- Code: `b_code = [0, 0, 0 | -g | +a_pd] = -b_true` → `b_code·b_code' = b_true·b_true'` ✓
- Writeup as literally written: `b_wu = [0, 0, 0 | -g | -a_pd]` → **not** equal to ±b_true; b_wu·b_wu' differs in (err, 11) and (11, err) blocks.

**Implication**:
- Code gives the correct Σ_n ✓
- Writeup's literal formula for Σ_n would give wrong cross-covariance Σ_n(1, 11), hence wrong C_n
- C_n has **never been empirically tested** (all P2 runs σ²_n = 0), so this discrepancy has not surfaced in data
- Magnitude of effect: to be computed numerically

`[ISSUE — writeup sign]` b_n(err states) should be +Fe·L_ss(:,1), not -Fe·L_ss(:,1). Code is internally consistent and correct. Phase 4 writeup fix required.

`[VERIFY]` Numerical check: compute Σ_n and C_n under both b_n versions; quantify C_n difference.

### Numerical verification (temp_verify_bn_sign.m)

Ran at lc=0.7, a_pd=0.05, f_0=0, code's lookup Q/R:

| Version | C_n | Σ_n(3,11) | vs physically correct |
|---|---:|---:|---|
| Physically correct (b_true = [0,0,0, +g, -a_pd]) | **1.114115** | −1.999e−02 | ref |
| Code as-is (b_code = [0,0,0, −g, +a_pd] = −b_true) | **1.114115** | −1.999e−02 | ≡ true ✓ |
| Writeup literal (b_wu = [0,0,0, −g, −a_pd]) | **1.041689** | +5.825e−02 | **−6.5%** |

The cross-correlation `Σ_n(3, 11)` flips sign under the writeup formula, changing C_n magnitude.

**Verdict confirmed**:
- Code `compute_7state_cdpmr_eff.m` → correct Σ_n, correct C_n_eff (= 1.1141 at the reference point).
- Writeup §5.3 b_n as literally written is wrong by 6.5% in C_n.
- σ²_n = 0 in all P2 runs hides this; still harmless at runtime (code correct).
- Phase 4 docs fix: flip writeup b_n(err block) from −Fe·L_ss(:,1) to +Fe·L_ss(:,1).

---

## 3. Section 5.4 — Lyapunov decomposition

### Claim
```
Σ[k] = 4 k_B T · a_x[k] · Σ_T  +  σ²_n · Σ_n
Σ_T  = A · Σ_T · A' + b_T · b_T'
Σ_n  = A · Σ_n · A' + b_n · b_n'
```

### Derivation

Independence of thermal (`a_x·f_T`) and sensor noise (`n`) gives linearity:
```
Σ[k] = E[e·e'] = Var(thermal-driven part) + Var(noise-driven part)
     = Var(a_x·f_T input) · Σ_T_unit + Var(n input) · Σ_n_unit
     = 4 k_B T · a_x · Σ_T + σ²_n · Σ_n
```
(treating `a_x·f_T` as having variance 4 k_B T · a_x and `n` as having variance σ²_n, in the stationary limit where `a_x` is locally constant — simplification #6).

Unit-variance Lyapunov equations for Σ_T and Σ_n follow by construction. `[OK]`

### Code check
`solve_dlyap_robust(A_aug, B_th)` → Σ_T (thermal, unit variance) ✓
`solve_dlyap_robust(A_aug, B_np)` → Σ_n (position noise, unit variance) ✓
DARE for L: iterative, tol=1e-10, max 30000 iters ✓
Fallback chain: dlyapchol → dlyap → fixed-point ✓

`[OK]`

### Assumption not in scope note: a_x treated as locally constant

Sec 5.4 writes `Σ[k] = 4 k_B T · a_x[k] · Σ_T`, implicitly assuming `a_x[k]` is constant over the Lyapunov convergence timescale. This is simplification #6 (listed in project_section5_simplifications.md, not in the scope note of the .tex). Task P2 validates this collectively at ±5%.

`[FLAG Phase 4 docs]` Could mention in scope note that a_x is held constant.

### Gain-measurement-noise channel NOT in Lyapunov

Writeup only considers two noise sources: thermal f_T and sensor noise n. The measurement y[2] = a_xm has its own noise (the chi-squared from finite IIR sample on Var(δx_mr)), which enters error states via `L_ss(:,2)` through `-Fe·L·v[2]`. Neither writeup nor code include this in Σ.

Code has `B_na = -(Fe·L(:,2))` (line 158) but it is **unused** for C_dpmr_eff / C_np_eff.

Effect: any closed-loop variance contribution from the a_xm noise channel is omitted from Σ_T, Σ_n. This could add a further term to Var(δx_mr) in principle. Code scale of this channel: R(2,2) = 1 (relatively large, suggests KF designer discounts this measurement).

`[FLAG Phase 3]` Worth a quick numerical estimate — if the a_xm-channel contribution is < 1% of Var(δx_mr), can safely ignore and document in scope note.

---

## 4. Section 5.5 — Variance of δx_mr

### Claim
```
δx_mr[k] = (1 - a_pd) · (δx_1[k] - δx_md[k-1] - n[k])
σ²_{δx_mr}[k] = (1-a_pd)² · [ Σ(1,1) - 2·Σ(1,11) + Σ(11,11) + σ²_n ]
```

### Derivation

From §2 recursions:
```
δx_md[k] = (1-a_pd)·δx_md[k-1] + a_pd·δx_m[k]
δx_mr[k] = δx_m[k] - δx_md[k]
         = δx_m[k] - [(1-a_pd)·δx_md[k-1] + a_pd·δx_m[k]]
         = (1-a_pd)·δx_m[k] - (1-a_pd)·δx_md[k-1]
         = (1-a_pd)·(δx_m[k] - δx_md[k-1])
```
Substitute `δx_m[k] = δx_1[k] - n[k]`:
```
δx_mr[k] = (1-a_pd)·(δx_1[k] - δx_md[k-1] - n[k])      ✓ matches writeup
```

### Variance

`n[k]` is fresh noise at step k, independent of `δx_1[k]` and `δx_md[k-1]` (both contain only past noise values). So:
```
Var(δx_mr) = (1-a_pd)² · [Var(δx_1 - δx_md[k-1]) + Var(n)]
          = (1-a_pd)² · [Var(δx_1) - 2·Cov(δx_1, δx_md[k-1]) + Var(δx_md[k-1]) + σ²_n]
          = (1-a_pd)² · [Σ(1,1) - 2·Σ(1,11) + Σ(11,11) + σ²_n]    ✓
```

where Σ = Σ[k] is the state covariance at step k. Writeup formula ✓.

### Code check

`compute_7state_cdpmr_eff.m` lines 177–188:
```matlab
gain_sq = (1 - a_pd)^2;
var_state_th = Sigma_th(3,3) + Sigma_th(11,11) - 2*Sigma_th(3,11);
C_dpmr_eff   = gain_sq * var_state_th;
var_state_np = Sigma_np(3,3) + Sigma_np(11,11) - 2*Sigma_np(3,11);
C_np_eff     = gain_sq * (var_state_np + 1);
```

Writeup position 1 (δx_1, oldest) ↔ code position 3 (dx_d2). Indexing consistent.

The `+1` inside C_np_eff is the direct σ²_n contribution (not passing through state).

`[OK]` matches writeup.

---

## 5. Section 5.6 — C_dpmr and C_n boxed formulas

### Claim
```
C_dpmr = (1-a_pd)² · [(Σ_T)_{1,1} - 2·(Σ_T)_{1,11} + (Σ_T)_{11,11}]
C_n    = (1-a_pd)² · [(Σ_n)_{1,1} - 2·(Σ_n)_{1,11} + (Σ_n)_{11,11} + 1]
```

Follows from §5.4 decomposition + §5.5 extraction. Matching `σ²_{δx_mr} = C_dpmr·4kBT·a_x + C_n·σ²_n`:

- Coefficient of `4kBT·a_x`: comes from Σ_T only, and the `+σ²_n` direct term vanishes.
  → `C_dpmr = (1-a_pd)²·[Σ_T(1,1) - 2Σ_T(1,11) + Σ_T(11,11)]` ✓
- Coefficient of `σ²_n`: comes from Σ_n AND direct term.
  → `C_n = (1-a_pd)²·[Σ_n(1,1) - 2Σ_n(1,11) + Σ_n(11,11) + 1]` ✓

`[OK]` algebra consistent under correct Σ_n. The Σ_n cross-term is where the §5.3 sign bug surfaces (see §2 above) — writeup-literal numerical answer is off by −6.5% at ref point; code is correct.

### Numerical reference (code output, lc=0.7, a_pd=0.05, f_0=0)

```
C_dpmr_eff = 3.9242   (verified in Phase 1 / Task P2 empirical ±5%)
C_np_eff   = 1.1141   (NOT yet empirically verified — σ²_n = 0 in all P2)
```

`[FLAG Phase 3]` Run σ²_n > 0 scenario and verify C_np_eff = 1.1141 empirically.

---

## 6. Section 6.1 — Bias source

### Claim
```
σ²_{δx_mr}[k]_IIR = (δx_mr)²_avg[k] - (δx_mrd[k])²       (§2 formula)
E[σ²_{δx_mr}_IIR] = Var(δx_mr) - Var(δx_mrd)            (stationary, zero-mean)
β ≡ E[σ²_IIR] / Var(δx_mr) = 1 - Var(δx_mrd)/Var(δx_mr)
```

### Derivation

In stationary steady state, `δx_mr` is zero-mean (its true mean is 0 because it's an HP residual of a stationary signal). Similarly `δx_mrd` is zero-mean (LP of zero-mean signal).

Let `μ_r = E[δx_mr] = 0`, `μ_rd = E[δx_mrd] = 0`. Then:
- `E[(δx_mr)²_avg]` — this is the EMA of `(δx_mr[k])²`. Stationary expectation: `E[(δx_mr)²] = Var(δx_mr) + μ_r² = Var(δx_mr)`. ✓
- `E[(δx_mrd)²]` — stationary: `= Var(δx_mrd) + μ_rd² = Var(δx_mrd)`. ✓

So `E[σ²_IIR] = Var(δx_mr) - Var(δx_mrd)`.

`β = 1 - Var(δx_mrd)/Var(δx_mr)` is the ratio of IIR output to the true `Var(δx_mr)`. Since `δx_mrd` is an LP of a zero-mean signal, Var(δx_mrd) ≥ 0 so β ≤ 1. Typically close to 1 for fast-decaying autocorrelation, further from 1 for heavy autocorrelation.

`[OK]`

### Caveat: finite-sample bias from `(δx_mr)²_avg` ignored

This derivation assumes the EMA `(δx_mr)²_avg` equals its true expectation `E[(δx_mr)²]` exactly. In finite steady-state, the EMA is an unbiased estimator of `E[(δx_mr)²]` (no bias from EMA alone for a stationary input's square), but has variance. The §6.1 formula is about **systematic** bias (=deviation of expectation from `Var(δx_mr)`), not variance of the estimator. So the derivation captures only the mean offset induced by `(δx_mrd)²` subtraction, not the finite-sample variance. ✓ Correct interpretation.

`[OK]`

### Subtle point: β derived from Σ_T only, applied to total σ²

In §6.2 (next), ρ(L) is derived from Σ_T (thermal-only). The resulting β is computed using thermal-only autocorrelation.

The `a_xm` estimator then applies `σ²_IIR / β` as if β were the bias factor for total Var(δx_mr) = Var(thermal part) + Var(noise part). But the noise-driven part has its own autocorrelation structure (from Σ_n, different from Σ_T), so a single β is only correct when one channel dominates.

With σ²_n = 0 (all current tests), only thermal matters → β is exactly right. With σ²_n > 0, there is a second-order inconsistency.

`[FLAG Phase 4 docs]` Either (a) document explicitly that β is thermal-only and assumes σ²_n ≪ thermal, or (b) derive β_n separately and combine.

---

## 7. Section 6.2 — Var(δx_mr) and ρ(L) from Section 5

### Claim (noise-free, σ²_n = 0)
```
Var(δx_mr) = (1-a_pd)² · [Σ_T(1,1) - 2Σ_T(1,11) + Σ_T(11,11)] · 4kBT·a_x
ρ(L) = [ (A^L·Σ_T)(1,1) - 2(A^L·Σ_T)(1,11) + (A^L·Σ_T)(11,11) ]
     / [ Σ_T(1,1) - 2Σ_T(1,11) + Σ_T(11,11) ]
```

### Derivation

From §5.6 with σ²_n = 0: `Var(δx_mr) = (1-a_pd)²·[Σ_T(1,1) - 2Σ_T(1,11) + Σ_T(11,11)]·4kBT·a_x` ✓.

Autocorrelation: define `c_s` as the selector vector such that `δx_mr[k] = (1-a_pd)·c_s^T·e[k] - (1-a_pd)·n[k]` (ignoring noise for thermal-only).

From §5.5 rewrite: `δx_mr[k] = (1-a_pd)·(δx_1[k] - δx_md[k-1])`, so `c_s` has `+1` at position 1 and `−1` at position 11, others 0.

Lag-L covariance (thermal-driven, stationary):
```
Cov(δx_mr[k], δx_mr[k-L]) = (1-a_pd)² · c_s^T · E[e[k]·e[k-L]^T] · c_s
                         = (1-a_pd)² · c_s^T · (A^L · Σ_T) · c_s · 4kBT·a_x
```
(Using `E[e[k]·e[k-L]^T] = A^L·E[e[k-L]·e[k-L]^T] = A^L·Σ_T` for stationary e.)

Dividing by Var(δx_mr):
```
ρ(L) = (c_s^T · A^L · Σ_T · c_s) / (c_s^T · Σ_T · c_s)
```
Expanded: numerator = `(A^L·Σ_T)(1,1) - 2(A^L·Σ_T)(1,11) + (A^L·Σ_T)(11,11)`. Denominator: same with L=0. ✓

(1-a_pd)² and 4kBT·a_x prefactors cancel. ✓

### Code check

`build_bias_factor_lookup.m` lines 57–71:
```matlab
c_s(3)  =  1;            % code pos 3 ↔ writeup pos 1 (oldest δx)
c_s(11) = -1;
A_L = eye(n_aug);
for L = 0:L_max
    gamma_L(L+1) = c_s' * A_L * Sigma_aug * c_s;
    A_L = A_L * A_aug;
end
rho = gamma_L / gamma_L(1);
```

Indexing: code (3, 11) ≡ writeup (1, 11). ✓

`[OK]` matches writeup.

**Subtle note**: bias factor uses only `Sigma_th` (thermal). So the sign bug in b_n does **not** propagate into β. `[OK]` unaffected.

---

## 8. Section 6.3 — Var(δx_mrd) closed form

### Claim
```
δx_mrd[k] = a_prd · Σ_{j≥0} (1-a_prd)^j · δx_mr[k-j]
Var(δx_mrd) = [a_prd/(2-a_prd)] · Var(δx_mr) · [1 + 2·Σ_{m≥1} ρ(m)(1-a_prd)^m]
```

### Derivation

Unroll §2 LP recursion `δx_mrd[k] = (1-a_prd)·δx_mrd[k-1] + a_prd·δx_mr[k]`:
```
δx_mrd[k] = a_prd · Σ_{j=0}^∞ (1-a_prd)^j · δx_mr[k-j]
```
(standard EMA unroll) ✓

Variance:
```
Var(δx_mrd) = a_prd² · Σ_j Σ_l (1-a_prd)^{j+l} · Cov(δx_mr[k-j], δx_mr[k-l])
            = a_prd² · Var(δx_mr) · Σ_j Σ_l (1-a_prd)^{j+l} · ρ(|j-l|)
```

Split into diagonal (j = l) and off-diagonal (j ≠ l, symmetric pairs):
```
Σ_{j,l} (1-a_prd)^{j+l} · ρ(|j-l|)
  = Σ_j (1-a_prd)^{2j} · ρ(0)  +  2·Σ_{j<l} (1-a_prd)^{j+l} · ρ(l-j)
```

Diagonal sum: `Σ_{j=0}^∞ (1-a_prd)^{2j} = 1/[1-(1-a_prd)²] = 1/[a_prd·(2-a_prd)]`, with ρ(0) = 1.

Off-diagonal: let m = l - j ≥ 1. Then j ranges 0..∞, l = j + m.
```
Σ_{j=0}^∞ (1-a_prd)^{2j+m} · ρ(m) = (1-a_prd)^m · ρ(m) / [a_prd·(2-a_prd)]
```
Summed over m ≥ 1: `Σ_{m≥1} ρ(m)·(1-a_prd)^m / [a_prd·(2-a_prd)]`.

Total:
```
Σ_{j,l} ... = (1/[a_prd·(2-a_prd)]) · [1 + 2·Σ_{m≥1} ρ(m)·(1-a_prd)^m]
```

Therefore:
```
Var(δx_mrd) = a_prd² · Var(δx_mr) · (1/[a_prd·(2-a_prd)]) · [1 + 2·Σ_{m≥1} ρ(m)(1-a_prd)^m]
            = [a_prd/(2-a_prd)] · Var(δx_mr) · [1 + 2·Σ_{m≥1} ρ(m)(1-a_prd)^m]
```
✓ matches writeup.

`[OK]`

### Code check

`build_bias_factor_lookup.m` lines 73–76:
```matlab
powers = ((1 - a_prd).^(1:L_max)).';
S      = sum(rho(2:L_max+1) .* powers);
bias_factor_tab(i) = 1 - (a_prd/(2-a_prd)) * (1 + 2*S);
```
S = Σ_{m=1}^{L_max} ρ(m)·(1-a_prd)^m ✓
`[OK]`

### Truncation caveat

Sum truncated at `m_max = 100`. Residual tail: `(1-a_prd)^100 = 0.95^100 ≈ 0.006` → |ρ|·0.006 ≪ numerical precision. ✓ justifies truncation.

---

## 9. Section 6.4 — β boxed formula

### Claim
```
β = 1 - [a_prd/(2-a_prd)] · [1 + 2·Σ_{m≥1} ρ(m)(1-a_prd)^m]
```

Substitute §6.3 into §6.1 `β = 1 - Var(δx_mrd)/Var(δx_mr)`. Var(δx_mr) cancels. ✓

### Bracket interpretation

`[1 + 2·Σ ρ(m)(1-a_prd)^m]`:
- ρ(m) = 0 ∀ m ≥ 1 (white δx_mr): bracket = 1, β_white = 1 - a_prd/(2-a_prd) = (2-2·a_prd)/(2-a_prd) = 0.9744 at a_prd=0.05. ✓ matches writeup.
- Positively correlated (closed-loop): bracket > 1, β < β_white.

`[OK]`

---

## 10. Section 6.5 — numerical β verification

### Claim (lc=0.7, a_prd=0.05)
```
β_white = 0.9744
β       = 0.9069    (bracket ≈ 3.6)
```

### Check against `build_bias_factor_lookup.m` output and `cdpmr_eff_lookup.mat`

From the build script's Gate G1 logic (lines 86–99): at lc=0.7 the computed bias_factor is compared to 0.9069 with tolerance 1e-3. Build passes → bias_factor[lc=0.7] = 0.9069 ± 0.001 ✓.

White reference at a_prd=0.05: β_white = (2 − 2·0.05)/(2 − 0.05) = 1.9/1.95 = 0.9744 ✓.

`[OK]` matches writeup. Will re-verify empirically by running `build_bias_factor_lookup.m` at the end of this audit as a sanity check.

---

## 11. Section 6.6 — bias-corrected a_xm assembly

### Claim
```
E[σ²_{δx_mr}]_IIR = β · Var(δx_mr) = β · (C_dpmr·4kBT·a_x + C_n·σ²_n)
a_xm[k]            = (σ²_{δx_mr}[k]/β − C_n·σ²_n) / (C_dpmr · 4kBT)      [BOXED]
```

### Derivation

Rearrange: `Var(δx_mr) = σ²_IIR/β`. Then `C_dpmr·4kBT·a_x = Var(δx_mr) − C_n·σ²_n = σ²_IIR/β − C_n·σ²_n`. Divide by `C_dpmr·4kBT` to get a_x. ✓

### Code check

`motion_control_law_7state.m` lines 181–186:
```matlab
den        = C_dpmr_eff_const * 4 * k_B * T_temp;       % C_dpmr·4kBT
noise_corr = C_np_eff_const * sigma2_noise;              % C_n·σ²_n
del_pmr_var_unbiased = del_pmr_var / IIR_bias_factor_const;    % σ²/β
a_m        = max((del_pmr_var_unbiased - noise_corr) / den, 0);
```
→ `a_m = max((σ²/β − C_n·σ²_n) / (C_dpmr·4kBT), 0)` ✓ exactly Sec 6.6 boxed.

`[OK]` Code matches writeup Sec 6.6.

### Internal inconsistency in writeup (Sec 2 vs Sec 6.6)

Writeup Sec 2 boxed formula:
```
a_xm[k] = (σ²_{δx_mr}[k] − C_n·σ²_n) / (C_dpmr·4kBT)     [missing /β]
```
Sec 6.6 boxed adds `/β`. Sec 2 was the idealized "infinite sample" form; Sec 6.6 is the finite-sample correction that the code actually uses.

Code uses Sec 6.6. Sec 2 as written is out-of-sync.

`[FLAG Phase 4 docs]` Add cross-reference in Sec 2 ("See §6.6 for finite-sample correction") or replace Sec 2's boxed with the β-corrected form.

### Subtle: β applied to total σ² (already flagged in §6.1)

β is thermal-only but applied to σ²_{δx_mr} (thermal + noise). When σ²_n > 0, a second-order inconsistency. Already flagged in §6.1.

---

## 12. Summary — Phase 1 findings

### Derivation correctness

| Section | Math | Code | Status |
|---|---|---|---|
| 5.1 state vector | OK (just indexing difference writeup↔code) | OK | `[OK]` + doc note |
| 5.2 δx_3 recursion | Derived ✓ matches | OK | `[OK]` |
| 5.3 A matrix | All entries correct | OK | `[OK]` |
| 5.3 b_T | Correct | OK | `[OK]` |
| 5.3 **b_n (err block)** | Writeup has **sign error** (should be +Fe·L, not −Fe·L) | **Code correct** (uses opposite n convention but internally consistent) | **`[ISSUE writeup]`** |
| 5.4 Lyapunov decomp | OK | OK | `[OK]` |
| 5.5 Var(δx_mr) extract | Derived ✓ | OK | `[OK]` |
| 5.6 C_dpmr, C_n | Formula correct (if using correct Σ_n) | OK (uses correct Σ_n) | `[OK]` |
| 6.1 β bias source | Derived ✓ | — | `[OK]` |
| 6.2 ρ(L) extract | Derived ✓ | OK | `[OK]` |
| 6.3 Var(δx_mrd) | Derived ✓ | OK | `[OK]` |
| 6.4 β formula | ✓ | OK | `[OK]` |
| 6.5 β numerical | ✓ (0.9069) | verified via lookup G1 gate | `[OK]` |
| 6.6 a_xm assembly | ✓ | OK (matches boxed) | `[OK]` |

### Issues & flags

**`[ISSUE writeup]`** — Section 5.3 b_n: `-Fe·L_ss(:,1)` should be `+Fe·L_ss(:,1)` (err block). Code is internally correct. Numerical impact on C_n if someone uses writeup literally: **−6.5%** at reference point. Action: Phase 4 writeup fix.

**`[FLAG Phase 4 docs]`**
- Indexing convention mismatch code↔writeup — explicit mapping table in docs
- Sec 2 boxed formula missing β — cross-reference to Sec 6.6
- β derivation uses Σ_T only; applied to total σ² — document scope or extend
- a_x held constant in Σ[k] = 4kBT·a_x·Σ_T assumption — mention in scope note

**`[FLAG Phase 3 verification]`**
- C_np_eff = 1.1141 never verified empirically (σ²_n = 0 in all P2). Run σ²_n > 0 test.
- Gain-measurement-noise channel (L_ss(:,2)) excluded from Σ. Quantify contribution.

### Items NOT issues (checked and consistent)

- A matrix all block entries (verified derivation matches code)
- b_T sign and positions
- Σ_T via dlyap (code's Σ_th equals correct Σ_T up to numerical precision)
- C_dpmr_eff = 3.9242 (empirically verified by Task P2 within ±5%)
- β = 0.9069 (Gate G1 in lookup build script passes)
- a_xm assembly formula in code (matches Sec 6.6 boxed)

### Recommendation

Phase 1 is **complete** for the derivation + code cross-check scope. The derivation chain is sound; the code implements the correct version (code's Σ_n matches physical truth via the Lyapunov invariant b·b' = (−b)·(−b)'). The only real issue is a documentation-level sign bug in writeup §5.3 that has no runtime effect.

**Proceed to Phase 2** (script/config audit) after user review.

---

# Phase 2 — script / config audit

Scope: verify the runtime configuration matches what the lookups were built
with, confirm β=0 is locked, confirm P2 analysis scripts apply the right
signal processing.

## 2.1 β=0 canonical lock

Per user decision (option A): default=0, no code deletion, no assertion added.

- `user_config.m` line 79: `config.beta = 0` ✓
- `verify_p2_static_h25.m`: doesn't touch `config.beta` explicitly — inherits 0 ✓
- `run_simulation.m`: not checked here; standalone P2 scripts confirmed

`motion_control_law_7state.m` lines 268–274 still contain the z-axis Fe_err_z
chart extension and lines 312–315 the `(1+β)·d - β·del_d` updates. Under β=0
these reduce to the same structure as x/y, so runtime behavior is identical.

`[OK]` — β=0 default holds. Code path remains but is a no-op at β=0.

## 2.2 Q/R lookup ↔ user_config consistency

### Baseline values (cross-checked)

| Location | Q_kf_scale | R_kf_scale |
|---|---|---|
| `user_config.m` line 82-83 | `[0, 0, 1e4, 1e-1, 0, 1e-4, 0]` | `[1e-2, 1e0]` |
| `build_cdpmr_eff_lookup.m` line 33-34 | `[0, 0, 1e4, 1e-1, 0, 1e-4, 0]` | `[1e-2, 1e0]` |
| `build_bias_factor_lookup.m` line 34-35 | `[0, 0, 1e4, 1e-1, 0, 1e-4, 0]` | `[1e-2, 1e0]` |

All three match exactly. ✓

### Runtime drift safeguard (added this phase)

Before Phase 2: `calc_ctrl_params.m` only warned on `a_pd`/`a_prd` mismatch
between config and lookups, not on `Qz_diag_scaling`/`Rz_diag_scaling`.
If a user changed Q or R in config but didn't rebuild the lookups, the
stale lookup values would be used silently.

**Added** four warnings in `calc_ctrl_params.m`:
- `calc_ctrl_params:cdpmr_Q_mismatch` — config Qz vs cdpmr_eff_lookup Q_kf_scale
- `calc_ctrl_params:cdpmr_R_mismatch` — config Rz vs cdpmr_eff_lookup R_kf_scale
- `calc_ctrl_params:bf_Q_mismatch` — config Qz vs bias_factor_lookup Q_kf_scale
- `calc_ctrl_params:bf_R_mismatch` — config Rz vs bias_factor_lookup R_kf_scale

Also added `calc_ctrl_params:apd_bf_mismatch` (bias_factor lookup also bakes
in `a_pd`, not just `a_prd`).

### Verification
Ran with default config → no warnings, correct values emitted (C_dpmr_eff=3.9242,
C_np_eff=1.1141, IIR_bias_factor=0.9069). Ran with `Qz(3) = 2e4` (doubled Q33)
→ both cdpmr and bias_factor Q-mismatch warnings fire. ✓

`[OK]` Drift safeguard in place.

## 2.3 P2 script signal-path audit

### `verify_p2_static_h25.m` (fresh sim at h=2.5)
- `simOut.p_d_out`, `simOut.p_m_out` captured via ToWorkspace blocks at Ts=1/1600
- `del_pm[k] = p_d[:,k-2] - p_m[:,k]` (Convention A: sensor-delayed tracking error) ✓
- Reapplied offline IIR: `a_pd=a_prd=a_cov=0.05` (matches controller and user_config) ✓
- Computes `del_pmr_var` on `del_pmr`, not `del_pm` (the post-`8cd6f81` fix) ✓
- Single seed (`rng(20260415)`), 30s, ss window = samples >10s
- `std_theory = sqrt(C_dpmr_eff · sigma2_dXT · a_axis/a_nom)` where C_dpmr_eff
  pulled from params (same lookup code uses at runtime) ✓

`[OK]` signal path correct end-to-end.

### `analyze_p2_h_bin.m` (dynamic bin from pre-existing MC)
- Reads `task1d_paper_benchmark_mc.mat` — data generated under β=0.5 legacy
- Reapplies offline IIR (same coefs) ✓
- Uses `C_dpmr` = `o.C_dpmr_eff` stored in the MC data (lc=0.4 context)
- Uses `IIR_bias_factor = 0.9363` (lc=0.4, also stored in MC data)
- Drops first 2 samples (Convention A: delay alignment) ✓

`[FLAG Phase 3]` β=0.5 legacy data — user wants to redo this bin analysis with
fresh β=0 MC data.

`[OK]` script logic correct; input dataset is what needs refresh.

### Free-space data (`phase2_chisquared_mc.mat`)
- Report §3.1 cites this as source for the 0.989/0.979/1.005 free-space ratios
- Pre-existing MC run with `del_pmr` field already stored
- IIR coefficients of that pre-computed `del_pmr` are not re-documented in the
  file (assumes it was built with the same 0.05 coefs)

`[FLAG Phase 3]` Re-generate free-space data from a fresh run with current
pipeline for parity with h=2.5 static methodology.

## 2.4 Phase 2 summary

| Item | Status |
|---|---|
| β=0 canonical | `[OK]` (per user option A, no code change) |
| Q/R consistency — baseline match | `[OK]` |
| Q/R consistency — runtime drift safeguard | `[OK]` — warnings added |
| a_pd/a_prd consistency — a_pd on bias_factor | `[OK]` — warning added |
| P2 fresh script signal path | `[OK]` |
| P2 dynamic bin script signal path | `[OK]` script; dataset is legacy β=0.5 |
| Free-space data provenance | `[FLAG]` — redo fresh in Phase 3 |

**Code changes**: `calc_ctrl_params.m` — added 5 warnings for drift detection.
No logic change, only diagnostics. Verified via direct test.

Proceed to Phase 3 (simulation re-runs): multi-seed P2 static, fresh
free-space, dynamic bin with β=0, and σ²_n>0 run for C_n verification.




