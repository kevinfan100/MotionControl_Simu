# Q66 (Open-Loop Thermal) + R22 (a_xm variance) — Derivation Companion Note

**PDF**: `Q66_OL_R22_derivation.pdf` (4 pages, PDF-only — no `.tex` source in repo;
original authored as `~/Downloads/tin.pdf`, TeX-produced).

This note maps the PDF's formulas to the production code and records the
2026-06-05 numerical verification. The PDF contains two derivations:

- **p.1**: Open-Loop Thermal Q66 (z-axis) — gain process noise from Brownian motion
- **p.1–4**: R22 = Var(a_xm) — EWMA inflation factor for the y_2 measurement noise

---

## Part 1 — Q66 Open-Loop Thermal (PDF §1–3)

### Result (PDF eq. 7)

```
Q66,z(h̄) = Var(δa_z) = ( a_z·K(h̄)/R )² · 4·k_B·T·a_z(h̄)
            with  a_z(h̄) = Δt / (γ_N·C⊥(h̄)),   K(h̄) = (1/C⊥)·dC⊥/dh̄,   h̄ = h/R
```

Physical chain: thermal force → random wall-normal step δh (variance σ²_δh = 4kBT·a_z)
→ because the gain a_z depends on position through the wall correction C⊥(h̄),
δh propagates into a random gain perturbation δa_z = −(a_z·K/R)·δh (chain rule, PDF eq. 6).
"Open-loop" = the noise path bypasses the control loop and the trajectory; it is pure
thermal-motion-driven gain jitter. This is the same physics as the 6-state δa_ram ≈ a_x'·x_ram
(see [[project_revised_control_6state_2026-06-04]]).

### Code map

| PDF symbol | Code | File:location |
|---|---|---|
| C⊥(h̄), C∥(h̄) | `c_perp`, `c_para` | `model/wall_effect/calc_correction_functions.m` |
| K(h̄) = (1/C)·dC/dh̄ | `derivs.K_h_perp` (z), `derivs.K_h_para` (x/y) | same, 3rd output |
| σ²_δh = 4kBT·a_z | `sigma2_dh_thermal = 4*kBT*Ts/(gamma_N_p*c_perp_h)` | `motion_control_law_eq17_core.m` Q66_OL_mode block |
| Q66 eq.7 | `Q66_OL_i = (a_hat_i*K_h_i/R_radius)^2 * sigma2_dh_thermal` | same |

Per-axis: z uses C⊥/K_h_perp (wall-normal), x/y use C∥/K_h_para (wall-parallel).

### Implementation note — the two a_z are DIFFERENT (PDF Implementation note)

The two `a_z` in eq. 7 are not the same quantity:

- `(a_z·K/R)²` sensitivity factor → uses **EKF estimate â_z** (slot 6 of z filter) = `a_hat_i`.
- `σ²_δh = 4kBT·a_z(h̄)` → uses the **deterministic function** of the *measured* position,
  `a_z(h̄_zm)` with `h̄_zm = (p_m·ŵ − p_wall)/R` = `c_perp_h` evaluated at measured h̄.
- `K(h̄)` shape function also uses the measured h̄.

**Why**: grounding σ²_δh and K in the *measured* position (not the estimate) breaks the
bias-amplification loop — only the gain magnitude being estimated uses â. Matches the
architecture rule "use measured h̄[k], not KF â, to avoid bias loop".

### Verification (2026-06-05, MATLAB)

Independent finite-difference of a_z(h̄) vs the analytical chain rule δa_z = −a_z·K/R:

| h̄ | K (wall sens.) | dā/dh analytic vs FD rel_err | Q66 OL | √Q66 [um/pN] |
|---|---|---|---|---|
| 1.20 (near wall) | −4.07 | 9.3e-10 | 7.0e-10 | 2.6e-5 |
| 1.50 | −1.33 | 8.7e-11 | 5.8e-10 | 2.4e-5 |
| 2.00 | −0.52 | 4.4e-10 | 3.0e-10 | 1.7e-5 |
| 5.00 | −0.056 | 1.4e-8 | 1.6e-11 | 4.0e-6 |
| 22.22 (far field) | −0.0024 | 6.9e-8 | 5.3e-14 | 2.3e-7 |

- Chain rule matches FD to machine precision (rel_err < 1e-7) across all h̄.
- Code `sigma2_dh_thermal` ≡ PDF `4kBT·a_z` exactly (diff = 0).
- K spans 1700× (near wall → far field) → Q66 spans 4 orders of magnitude, fully adaptive.
  √Q66 (per-step gain jitter) ≈ 0.18% of a_nom near wall, 0.0016% far field (a_nom = 1.47e-2).

This is the theoretical backing for the empirical finding that **Q66-OL + Q77=0 is optimal**
([[project_q77_zero_optimal]]): the gain noise physically lands on a_x (slot 6, via δh thermal),
not on δa_x (slot 7), so a Q77 random walk is a modeling error (Jordan-block amplification).

---

## Part 2 — R22 = Var(a_xm) (PDF §1–4)

### Result (PDF eq. 19)

```
R22 = Var(a_xm[k]) = 1/(C_d·4kBT)² · [ a_var/(2−a_var) ] · IF_eff(s) · Var(δx_r²[k])
      C_d = 2 + 1/(1−λc²),   s = 1−a_var,   IF_eff(s) = 1 + 2·Σ_{τ≥1} ρ(τ)·s^τ
```

Two factors:
- **finite-window** `a_var/(2−a_var)` (PDF §2): the EWMA noise gain for IID input,
  derived from the filter self-overlap (h⋆h)[0]. → 0 (long window) … → 1 (no averaging).
- **self-correlation inflation** `IF_eff(s)` (PDF §3): autocorrelation of δx_r² reduces the
  effective independent-sample count, inflating variance above the white-noise floor.
  Weighted by the filter's self-overlap (h⋆h)[τ] = (a_var/(2−a_var))·s^|τ|.

### Code map

| PDF symbol | Code | File:location |
|---|---|---|
| a_var | `a_cov` | config / build_eq17_constants |
| a_var/(2−a_var) | `R22_prefactor = 2*a_cov/(2-a_cov)` | `motion_control_law_eq17_core.m` R(2,2) assembly |
| IF_eff(s) | `IF_eff_per_axis` (X2a empirical calibration) | `build_eq17_constants.m` / `test_script/build_helpers/calibrate_IF_eff.m` |
| C_d = 2+1/(1−λc²) | `C_dpmr` | build_eq17_constants |
| R22 (eq.19) | `R22_prefactor * IF_eff_per_axis(ax) * (a_hat + xi)^2 + 5*Q77` | eq17_core R(2,2) assembly |

The `(a_hat + xi)²` form comes from substituting σ²_δxr = C_dpmr·4kBT·a + C_n·σ²_n into
Var(δx_r²) and dividing by (C_d·4kBT)² (linear inversion of a_xm, PDF eq. 9).

### Relationship to existing derivation docs (overlap — read together)

This R22 derivation is the **EWMA self-overlap (h⋆h)** angle. Two other repo docs derive
the same R(2,2) from different angles — all consistent, not contradictory:

- `R22_derivation.tex` (residue/Parseval): exact ρ_dpmr(τ) via pole evaluation at the
  F_T poles (α=1−a_pd, β=λc). Gives the closed-form ρ(τ) this PDF leaves as a generic ρ(τ).
- `reference/shared/writeup_architecture.tex` §7: same three-factor structure (χ_sq, ρ_a).
- `phase6_R_matrix_derivation.md` §4: the production R(2,2) assembly + 3-guard logic.
- `phase2_IF_var_dpr_derivation.md`: IF_var spectral form.

---

## Cross-references

- Q33/Q66/Q77 physics: [[phase5_Q_matrix_derivation]] (this PDF's Q66 OL is the clean
  standalone version of the §6 chain-rule treatment)
- R assembly: [[phase6_R_matrix_derivation]]
- Empirical Q77=0 / Q66-OL optimality: [[project_q77_zero_optimal]]
- 6-state gain-noise physics: [[project_revised_control_6state_2026-06-04]]
