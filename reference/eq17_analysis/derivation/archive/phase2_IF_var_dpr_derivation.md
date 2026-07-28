# Phase 2 Addendum: IF_var on δp_r (HP-residual) — corrected derivation

**Date**: 2026-04-30
**Status**: Derivation only. NO production code modified.
**Companion**: `phase2_C_dpmr_C_n_derivation.md` (existing IF_var(δx) derivation)
**Helper**: `test_script/temp_w1b_compute_if_var_dpr.m` (verification, can be deleted)

---

## 1. Problem statement

The current `build_eq17_constants.m` (lines 167–192) computes `IF_var` from the autocorrelation of the closed-loop tracking error `δx[k]` (Option A MA(2)-full ⇒ `IF_var(δx) ≈ 4.22` for `λ_c=0.7`). This factor is then used as the EWMA effective-DOF inflation in the construction of `R(2,2)`.

But the IIR EWMA estimator inside the controller (`motion_control_law_eq17_7state.m` lines 280–325) does **not** average `δx²`. It averages `(δp_r[k])²` where:

```
δp_md[k]    = p_d[k-d] - p_m[k]                                  (delayed tracking error)
δp_md_LP[k] = (1-a_pd)·δp_md_LP[k-1] + a_pd·δp_md[k]            (IIR LP, EWMA mean)
δp_r[k]     = δp_md[k] - δp_md_LP[k]                             (HP residual)
σ̂²_δxr[k+1] = (1-a_cov)·σ̂²_δxr[k] + a_cov·δp_r[k]²              (EWMA variance)
```

Because the IIR LP captures the slow components of `δp_md`, the HP residual `δp_r` has a whiter spectrum than `δx`. Therefore:

```
ρ_δp_r(τ) decays faster than ρ_δx(τ)        ⇒        IF_var(δp_r) < IF_var(δx).
```

Goal: derive `IF_var(δp_r) = 1 + 2·Σ_{τ≥1} ρ²_δp_r(τ)` numerically (and symbolically) for the default `(λ_c, a_pd) = (0.7, 0.05)` and confirm.

---

## 2. Derivation (Approach A — spectral / Wiener–Khinchin)

### 2.1 Spectrum of δx

From Phase 2 §3 the closed-loop one-step recursion is `δx[k+1] = λ_c·δx[k] − ε[k]` with ε MA(2)-on-thermal + white-on-sensor:

```
ε[k] = (1-λ_c)·n[k] + a_x·f_T[k] + (1-λ_c)·a_x·f_T[k-1] + (1-λ_c)·a_x·f_T[k-2]
```

Two-sided power spectrum (`ω ∈ (-π, π]`, normalize `a_x = 1`):

```
S_ε(ω) = σ²_dXT · |1 + (1-λ_c)·e^{-iω} + (1-λ_c)·e^{-2iω}|²
       + (1-λ_c)² · σ²_n
```

The transfer from ε to δx is one-step lag inverse:

```
H_δx(z) = -z^{-1} / (1 - λ_c·z^{-1})  ⇒  |H_δx(e^{iω})|² = 1 / |1 - λ_c·e^{-iω}|²
```

```
S_δx(ω) = S_ε(ω) / |1 - λ_c·e^{-iω}|²
```

### 2.2 Spectrum of δp_md

```
δp_md[k] = δx[k-d] + n[k]
```

The pure delay `z^{-d}` does not change the magnitude spectrum. The white sensor noise adds a flat floor (uncorrelated with δx):

```
S_δp_md(ω) = S_δx(ω) + σ²_n
```

### 2.3 Spectrum of δp_r — apply HP filter

The IIR LP transfer function is

```
H_LP(z) = a_pd / (1 - (1-a_pd)·z^{-1})
```

The HP-by-subtract filter is

```
H_HP(z) = 1 - H_LP(z) = (1 - z^{-1}) / (1 - (1-a_pd)·z^{-1})
```

(One zero at `z=1` — DC kill — and one pole at `z=1-a_pd`.)

```
|H_HP(e^{iω})|² = |1 - e^{-iω}|² / |1 - (1-a_pd)·e^{-iω}|²
                = (2 - 2·cos ω) / (1 - 2(1-a_pd)·cos ω + (1-a_pd)²)
```

Therefore

```
S_δp_r(ω) = |H_HP(e^{iω})|² · S_δp_md(ω)
```

### 2.4 Autocorrelation and IF_var

Use Wiener–Khinchin to recover the autocovariance of `δp_r` numerically:

```
γ_δp_r(τ) = (1/(2π)) · ∫_{-π}^{π} S_δp_r(ω) · cos(ωτ) dω
ρ_δp_r(τ) = γ_δp_r(τ) / γ_δp_r(0)
```

In code: sample `S_δp_r(ω)` on `N=2^20` uniform grid, take inverse FFT, normalize.

Final inflation factor:

```
IF_var(δp_r) = 1 + 2 · Σ_{τ=1}^{∞} ρ²_δp_r(τ)
```

---

## 3. Approach B — Augmented state-space Lyapunov

Build the 8-dim augmented state at time `k`:

```
x[k] = [ δx[k];  f_T[k];  f_T[k-1];  f_T[k-2];  δx[k-1];  δx[k-2];  LP[k];  n[k] ]
```

with white inputs `w[k+1] = [f_T[k+1];  n[k+1]]`. The transition `A` and input matrix `B` were assembled exactly per the controller equations (see helper file lines 184–250). For `d=2`:

```
δx[k+1]    = λ_c·δx[k] − f_T[k] − (1-λ_c)·f_T[k-1] − (1-λ_c)·f_T[k-2] − (1-λ_c)·n[k]
LP[k+1]    = (1-a_pd)·LP[k] + a_pd·(δx[k-1] + n[k+1])
δp_r[k]    = δx[k-2] + n[k] − LP[k]   ⇔   c = [0 0 0 0 0 1 -1 1]ᵀ
```

Solve `Σ = A·Σ·Aᵀ + B·W·Bᵀ` (`W = diag(σ²_T, σ²_n)`) via `dlyap`, then evaluate

```
γ_δp_r(τ) = cᵀ · Aᵗ · Σ · c       for τ = 0, 1, 2, …
```

This is a closed-form approach and gives identical results to Approach A.

---

## 4. Numerical results (λ_c = 0.7, a_pd = 0.05, d = 2, thermal-only)

Three independent calculations agree to within 0.03%:

| Method                          | IF_var(δx) | IF_var(δp_r) |
|---------------------------------|-----------:|-------------:|
| Closed-form (existing, Eq.7.2)  | 4.2198     | —            |
| Spectral / FFT                   | 4.2198     | **3.8409**   |
| Augmented Lyapunov (8 state)    | —          | **3.8409**   |
| Monte-Carlo (N = 2 × 10⁶)        | 4.2026     | 3.8398       |

**IF_var(δp_r) ≈ 3.84** for the default parameters, vs `IF_var(δx) = 4.22` — only a 9.0% reduction.

### Sweep IF_var(δp_r) over a_pd (λ_c = 0.7, thermal-only)

| a_pd  | τ_LP = 1/a_pd | IF_var(δp_r) | reduction vs IF_var(δx)=4.22 |
|------:|--------------:|-------------:|----------------------------:|
| 0.010 |          100  |  4.138       | 0.98×                       |
| 0.020 |           50  |  4.059       | 0.96×                       |
| **0.050** |       **20** |  **3.841**  | **0.91×**                   |
| 0.100 |           10  |  3.526       | 0.84×                       |
| 0.200 |            5  |  3.027       | 0.72×                       |
| 0.400 |          2.5  |  2.322       | 0.55×                       |
| 0.700 |          1.4  |  1.611       | 0.38×                       |
| 0.900 |          1.1  |  1.294       | 0.31×                       |

**Interpretation**: the LP corner frequency for `a_pd=0.05` is `≈ a_pd / (2π) ≈ 0.008 rad/sample`. The δx process has correlation length `1/(1-λ_c) ≈ 3.3 steps`, equivalent corner `≈ 0.05`. Since the LP corner is **much lower** than δx's corner, the LP only removes the tiny ultra-low-frequency tail of δx, leaving most of δx's autocorrelation intact in `δp_r`. The expected `IF_var(δp_r) ~ 2-3` would require `a_pd ≈ 0.2-0.4`.

---

## 5. Closed-form structure (no clean scalar)

The combined transfer from white inputs to `δp_r` has the cascaded form

```
H_dpr_ftherm(z) = z^{-d} · [1 + (1-λ_c)z^{-1} + (1-λ_c)z^{-2}] · (1 - z^{-1}) · 1/((1 - λ_c·z^{-1})(1 - (1-a_pd)·z^{-1}))
H_dpr_n(z)      = z^{-d} · (1-λ_c) · (1 - z^{-1}) / ((1 - λ_c z^{-1})(1 - (1-a_pd)·z^{-1}))
                + (1 - z^{-1}) / (1 - (1-a_pd)·z^{-1})    ← direct n[k] path through HP
```

A clean closed scalar like `(1 + λ²)/(1 - λ²)` does **not** exist for general `(λ_c, a_pd)`. Numerical evaluation (FFT or 8-state Lyapunov) is the correct method. This is what the helper script does.

For algorithmic use in production code, the recommended form is the 8-state Lyapunov from Approach B — it is closed-form per parameter pair, runs in milliseconds, and matches FFT to 4 decimals.

---

## 6. Conclusion

```
IF_var(δp_r) = 3.84      (vs IF_var(δx) = 4.22)
```

The reduction is **only 9%**, not the 30-50% the original task expected, because the IIR LP corner (`a_pd=0.05` ⇒ τ=20 steps) is much slower than δx's correlation length (`1/(1-λ_c) ≈ 3.3 steps`). The LP barely whitens δx. To approach `IF_var ~ 2-3`, `a_pd` would need to be ≥ 0.2.

Caveats / simplifications:
1. Sensor-noise is set to zero (`σ²_n = 0`) for cleanest reading. Non-zero σ²_n further reduces `IF_var(δp_r)` because n[k] is white.
2. Result assumes the v2 ε MA(2) structure (`Σf_d` retained in the controller, paper Eq.19 form). If `Σf_d` were dropped (v1 mismatch), ε would have a different MA structure.
3. The 8-state Lyapunov is exact for d=2; generalizing to d≠2 just adds δx history states.
4. The numerical helper uses default `λ_c=0.7`, `a_pd=0.05`; rerun with other parameters as needed.

---

**End of derivation. Ready for review and downstream R(2,2) update if the user decides to switch to IF_var(δp_r).**
