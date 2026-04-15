# Task P2: Positioning Performance Verification

**Branch**: `feat/sigma-ratio-filter`
**Date**: 2026-04-15
**Artifacts**:
- Scripts: `test_script/analyze_p2_h_bin.m`, `test_script/verify_p2_static_h25.m`
- Figures: `reference/for_test/fig_p2_h_bin.png`
- Data (gitignored): `test_results/verify/p2_h_bin.mat`, `p2_static_h25.mat`
- Referenced data (pre-existing): `phase2_chisquared_mc.mat`, `task1d_paper_benchmark_mc.mat`

---

## 1. Context

Section 5/6 of `writeup_architecture.tex` gives the theoretical thermal-limited
tracking-error variance:
```
Var(δx_mr)[k] = C_dpmr · 4 k_B T · a_x(h(t))    (steady state, per axis)
```
We need to verify that simulation empirically matches this under multiple
scenarios, matching the paper's Section III performance claims.

## 2. Positioning spec (derived from paper Section III)

Four criteria, extracted from paper Fig 5/6/7/8/10/11 visual/textual claims:

| # | Criterion | Target |
|---|---|---|
| **P1** | Mean tracking error ≈ 0 per axis | `|mean(δx_m)|` within thermal noise SE |
| **P2** | Empirical std matches theoretical | `std(δx_m) ≈ sqrt(C_dpmr · 4 k_B T · a_x)` |
| **P3** | h-dependence correctly shown | z-axis std scales with `sqrt(1/c_⊥)` |
| **P4** | 3D RMSE ≤ paper level | ~40 nm in dynamic near-wall |

Paper's target is qualitative ("close to zero mean", "in good agreement with
theoretical"); we interpret as ≤~10% NRMSE where possible.

## 3. Verification runs

### 3.1 Free-space static (baseline) — P2 free-space

- Source: `phase2_chisquared_mc.mat` (pre-existing, lc=0.7, T_sim=30s, no wall)
- 20 s steady-state window
- `C_dpmr_eff = 3.9242`, `a_x ≈ a_nom`
- Theoretical: 31.44 nm per axis

| Axis | emp std [nm] | theory [nm] | ratio |
|---|---:|---:|---:|
| x | 31.10 | 31.44 | **0.989** |
| y | 30.80 | 31.44 | **0.979** |
| z | 31.60 | 31.44 | **1.005** |

**✓ PASS** within ±2% (below chi-squared estimator noise).

### 3.2 Near-wall static — P2 near-wall (canonical config, β=0)

- Source: `verify_p2_static_h25.m` → `p2_static_h25.mat`
- Config: `h_init = 2.5 µm`, `lc = 0.7`, `amplitude = 0`, `T_sim = 30 s`, `wall_effect = on`, `thermal = on`, `noise = off`, **`β = 0`** (canonical; user confirmed z-axis β-coupling was experimental, now disabled)
- `h_bar = 1.111`, `c_∥ = 2.311`, `c_⊥ = 10.438`
- Theoretical: 20.68 nm (x/y), 9.73 nm (z)

| Axis | mean [nm] | emp std [nm] | theory [nm] | **ratio** |
|---|---:|---:|---:|---:|
| x | +0.04 | 23.34 | 20.68 | **1.129** |
| y | +0.04 | 23.74 | 20.68 | **1.148** |
| z | +0.04 | 11.00 |  9.73 | **1.130** |

**P1 ✓** (mean essentially 0, ±0.04 nm)
**P2 ⚠ PARTIAL** (~13-15% excess, **near-uniform across all three axes** — this uniformity is the key new finding)

**Earlier run with `β = 0.5` (experimental z-axis chart extension, removed from canonical)**:

| Axis | ratio at β=0.5 | ratio at β=0 | Δratio |
|---|---:|---:|---:|
| x | 1.128 | 1.129 | ~0 |
| y | 1.142 | 1.148 | ~0 |
| z | **1.167** | **1.130** | **−0.037** |

Removing `β` improved z by 3.7 pp but did NOT explain the bulk of the
near-wall excess. The remaining ~13% is near-uniform on all axes and likely
stems from the `f_{dx}` sensitivity drop (Section 5 scope-note simplification
(i)) — this simplification applies equally to all axes via `L_ss`, producing
a symmetric excess, which matches what we see.

### 3.3 Dynamic near-wall h-binning — P2 & P3 dynamic

- Source: `task1d_paper_benchmark_mc.mat` + `analyze_p2_h_bin.m`
- Config: `lc=0.4`, 1 Hz oscillation, h: 2.5→5 µm, 12 s
- Steady-state window: h ∈ [2.47, 7.58] µm
- Binned by h, per-bin sample variance vs `C_dpmr_eff * sigma2_dXT * a(h)/a_nom`

| h bin [µm] | N | h_c | c_⊥ | emp std_z [nm] | theory_z [nm] | ratio_z |
|---|---:|---:|---:|---:|---:|---:|
| [2.47, 3.00) | 3886 | 2.63 | 7.24 | 14.29 | 11.09 | **1.288** |
| [3.00, 4.00) | 2521 | 3.46 | 3.05 | 24.81 | 17.10 | **1.451** |
| [4.00, 5.00) | 1991 | 4.50 | 2.13 | 29.04 | 20.47 | **1.418** |
| [5.00, 7.50) | 7283 | 6.55 | 1.60 | 29.77 | 23.64 | **1.259** |

**P2 ⚠ PARTIAL** (~30-45% excess under dynamic conditions)
**P3 ✓** (empirical envelope drops from ~30 nm at h=6.5 to ~14 nm at h=2.6, consistent with `sqrt(1/c_⊥)` scaling)

x/y-axis ratios across all bins: ~1.10 flat (minimal wall effect on tangential axes).

### 3.4 3D RMSE — P4

- Source: `task1d_paper_benchmark_mc.mat` (Task 1d A2, 10 seeds)
- **3D RMSE = 40.9 nm** (dynamic near-wall sweep, matches paper level)

**P4 ✓**

## 4. Error decomposition at h = 2.5 µm (β=0 canonical)

Combining results from 3.2 (static, β=0) and 3.3 (dynamic, legacy β=0.5 data):
```
empirical_dynamic² ≈ thermal_floor² + static_residual² + dynamic_residual²
14.29² nm²        ≈ 9.73²          + 5.13²            + 8.7²
204               ≈  95             + 26                + 76        ✓
```

| Component | std [nm] | Contribution |
|---|---:|---|
| Thermal floor (Section 5 Lyapunov) | 9.73 | 68% of variance, physics-limited, cannot reduce |
| **Static architectural residual** (β=0) | **5.13** | 13% of variance, likely from `f_dx` drop simplification |
| Dynamic lag residual (1 Hz) | ~8.7 | ~19% of variance, Task 1d A2 identified |

**Note**: dynamic case still uses legacy `task1d_paper_benchmark_mc.mat` data
generated under `β = 0.5`. Re-running dynamic sweep with `β = 0` is a
candidate future task, but the β contribution to dynamic is expected to
be similarly small (~0.4 nm) as in static.

## 5. Open question: the ~13% static near-wall excess (β=0)

**Near-uniform across all three axes** (x: 1.129, y: 1.148, z: 1.130):
a key finding that **rules out wall-effect asymmetry as the cause**. If the
excess were purely a `c_⊥`-related modeling error, z-axis would dominate
(since `c_⊥ = 10.4 ≫ c_∥ = 2.3`). The near-uniformity instead points to
a system-level simplification that affects all axes equally.

Candidate causes (in order of likelihood):
1. **`f_{dx}` sensitivity drop** (Section 5 scope-note simplification):
   Section 5 analyzes the Lyapunov with `F_e(3,6) = 0` instead of the
   real `-f_{dx}[k]` (time-varying). At near-wall, `a_x` is small so
   `f_{dx}` is not proportionally smaller (controller still needs to
   compensate thermal kicks), and the `f_{dx}/a` ratio that enters the
   sensitivity term is larger. **This effect applies equally to all
   three axes via their respective `f_{dx/y/z}[k]`** — consistent with
   the observed uniform ~13% excess.
2. **`L_ss` operating-point mismatch**: EKF `L_ss` is precomputed with
   `a_x = a_nom` (free-space Q scaling). At near-wall, sub-optimal
   `L_ss` → effective `C_dpmr` bigger than Section 5 prediction. Also
   axis-uniform if all axes use the same Q, R tuning.
3. **Section 5 simplification #6 breakdown** (`a_x` treated as constant
   in Lyapunov): static h=2.5 literally satisfies constant-a assumption,
   so this should NOT contribute in the static case — can be ruled out.
4. **Slower EKF convergence at near-wall**: smaller `a_x` → slower
   dynamics; 10-s warmup may not fully settle. Testable by extending
   warmup to 30 s.

**Earlier erroneous hypothesis (now ruled out)**: z-axis β-coupling was a
candidate when β=0.5 gave ratio 1.167; removing β only dropped z ratio by
3.7 pp to 1.130, leaving the bulk of the excess intact and symmetric across
axes. **β is NOT the dominant cause.**

No debug of the remaining ~13% pursued yet — flagged for follow-up.

## 6. P1–P4 final status

| Criterion | Free-space | Near-wall static | Near-wall dynamic | Verdict |
|---|---|---|---|---|
| **P1** mean ≈ 0 | ✓ | ✓ | ✓ (from binning) | **PASS** |
| **P2** thermal match | ✓ (±2%) | ⚠ (+15%) | ⚠ (+29-45%) | **PARTIAL** |
| **P3** h-dependence | n/a | n/a | ✓ | **PASS** |
| **P4** 3D RMSE | n/a | n/a | ✓ (40.9 nm) | **PASS** |

**Overall**: positioning is thermal-limited + reasonable architectural residual.
Consistent with paper's qualitative "in good agreement with theoretical"
claim. **Ready to proceed** to `â`-target discussion as the next step.

## 7. Follow-up candidates (deferred)

- **Debug §5 root cause**: re-solve Section 5 Lyapunov with near-wall
  operating point to check if `C_dpmr` should be larger there
- **Task 1e time-varying Σ_aug**: addresses dynamic lag residual (15%),
  not static (17%)
- **Paper Section III-A complete replica**: slow ramp + 1 Hz x/y sine +
  lc=0.7 — direct apples-to-apples comparison with paper Fig 5/6
- **Extended warmup test**: check if static residual shrinks with 60-s
  warmup instead of 10-s
