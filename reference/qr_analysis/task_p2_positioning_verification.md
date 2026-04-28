# Task P2: Positioning Performance Verification

**Branch**: `feat/sigma-ratio-filter`
**Date**: 2026-04-15 (final after del_pmr signal fix)
**Artifacts**:
- Scripts: `test_script/analyze_p2_h_bin.m`, `test_script/verify_p2_static_h25.m`
- Figures: `reference/for_test/fig_p2_h_bin.png`
- Data (gitignored): `test_results/verify/p2_h_bin.mat`, `p2_static_h25.mat`
- Referenced data (pre-existing): `phase2_chisquared_mc.mat`, `task1d_paper_benchmark_mc.mat`

---

## 1. Context

Section 5/6 of `writeup_architecture.tex` gives the theoretical thermal-limited
tracking-error residual variance:
```
Var(δx_mr)[k] = C_dpmr · 4 k_B T · a_x(h(t))    (steady state, per axis)
```
Note: the theory predicts `Var(δx_mr)` — the HP-filtered residual produced by
the controller's IIR chain — not `Var(δx_m)` (the raw tracking error). This is
a critical distinction (see §5 below).

We verify that simulation empirically matches this theoretical prediction
under free-space, near-wall static, and dynamic near-wall scenarios to
demonstrate paper-level positioning.

## 2. Positioning spec (derived from paper Section III)

Four criteria, extracted from paper Fig 5/6/7/8/10/11 visual/textual claims:

| # | Criterion | Target |
|---|---|---|
| **P1** | Mean tracking error ≈ 0 per axis | `|mean(δx_mr)|` within thermal noise SE |
| **P2** | Empirical std matches theoretical | `std(δx_mr) ≈ sqrt(C_dpmr · 4 k_B T · a_x)` |
| **P3** | h-dependence correctly shown | z-axis std scales with `sqrt(1/c_⊥)` |
| **P4** | 3D RMSE ≤ paper level | ~40 nm in dynamic near-wall |

Paper's target is qualitative ("close to zero mean", "in good agreement with
theoretical"); we require ≤~5% ratio on `del_pmr`.

## 3. Verification runs

All empirical variances below are computed on `del_pmr` (the controller's
HP-filtered residual), reapplied offline with the controller's IIR constants
(`a_pd = a_prd = a_cov = 0.05`) where needed. Using the raw `del_pm` would
overestimate by a factor of about 1.27 in variance (~13% in std) due to
strong closed-loop autocorrelation — see §5.

### 3.1 Free-space static — P2 free-space

- Source: `phase2_chisquared_mc.mat` (pre-existing, lc=0.7, T_sim=30s, no wall)
- 20 s steady-state window
- `C_dpmr_eff = 3.9242`, `a_x ≈ a_nom`
- Theoretical: 31.44 nm per axis
- Signal: `del_pmr` field already stored in the .mat

| Axis | emp std [nm] | theory [nm] | **ratio** |
|---|---:|---:|---:|
| x | 31.10 | 31.44 | **0.989** |
| y | 30.80 | 31.44 | **0.979** |
| z | 31.60 | 31.44 | **1.005** |

**✓ PASS** within ±2%.

### 3.2 Near-wall static — P2 near-wall

- Source: `verify_p2_static_h25.m` → `p2_static_h25.mat`
- Config: `h_init = 2.5 µm`, `lc = 0.7`, `amplitude = 0`, `T_sim = 30 s`, `wall_effect = on`, `thermal = on`, `noise = off`, `β = 0` (canonical)
- `h_bar = 1.111`, `c_∥ = 2.311`, `c_⊥ = 10.438`
- Theoretical: 20.68 nm (x/y), 9.73 nm (z)

| Axis | mean [nm] | emp std (del_pmr) [nm] | theory [nm] | **ratio** |
|---|---:|---:|---:|---:|
| x | −0.013 | 20.86 | 20.68 | **1.009** |
| y | +0.001 | 21.24 | 20.68 | **1.027** |
| z | +0.005 |  9.80 |  9.73 | **1.007** |

**P1 ✓** (mean < 0.02 nm vs std ~10 nm)
**P2 ✓ PASS** within ±3% per axis

For reference: using the raw `del_pm` (wrong signal) gives 1.14 / 1.16 / 1.14,
which would falsely suggest a 13% near-wall excess. See §5.

### 3.3 Dynamic near-wall h-binning — P2 & P3 dynamic

- Source: `task1d_paper_benchmark_mc.mat` + `analyze_p2_h_bin.m`
- Config: `lc=0.4`, 1 Hz oscillation, h: 2.5→5 µm, 12 s, β=0.5 (legacy data — see note)
- Steady-state window: h ∈ [2.47, 7.58] µm after 2-s transient
- Binned by h, per-bin sample variance of `del_pmr` (offline IIR reapplied)
  vs `C_dpmr_eff * sigma2_dXT * a(h)/a_nom`

| h bin [µm] | N | h_c | c_⊥ | emp std_z [nm] | theory_z [nm] | ratio_z |
|---|---:|---:|---:|---:|---:|---:|
| [2.47, 3.00) | 3886 | 2.63 | 7.24 | 11.59 | 11.09 | **1.045** |
| [3.00, 4.00) | 2521 | 3.46 | 3.05 | 17.75 | 17.10 | **1.038** |
| [4.00, 5.00) | 1991 | 4.50 | 2.13 | 20.06 | 20.47 | **0.980** |
| [5.00, 7.50) | 7283 | 6.55 | 1.60 | 24.15 | 23.64 | **1.022** |
| [7.50, 10.00) | 320  | 7.52 | 1.49 | 16.91 | 24.48 | 0.691* |

(* 7.5–10 bin has only 320 samples in a transient tail, ignored)

x-axis ratios across all bins: 0.996 / 1.049 / 1.008 / 1.016 (flat ≈ 1.0).

**Bin-weighted z-axis overall ratio: 1.012** (vs theory 19.86 nm).

**P2 ✓ PASS** within ±5% per bin
**P3 ✓** (empirical envelope drops from ~24 nm at h=6.5 to ~12 nm at h=2.6,
matching `1/sqrt(c_⊥)` scaling)

Legacy-β note: this dataset was generated under the experimental
`β = 0.5` z-axis chart extension, before canonical was set to β=0. The
observed dynamic ratio is essentially the same with or without β (we
verified on the static case that β contributes < 0.4 nm), so the legacy
data is accepted as-is.

### 3.4 3D RMSE — P4

- Source: `task1d_paper_benchmark_mc.mat` (Task 1d A2, 10 seeds)
- **3D RMSE = 40.9 nm** (dynamic near-wall sweep, matches paper level)

**P4 ✓**

## 4. Final P1–P4 status

| Criterion | Free-space | Near-wall static | Near-wall dynamic | Verdict |
|---|---|---|---|---|
| **P1** mean ≈ 0 | ✓ | ✓ (<0.02 nm) | ✓ (from binning) | **PASS** |
| **P2** thermal match | ✓ (±2%) | ✓ (±3%) | ✓ (±5%) | **PASS** |
| **P3** h-dependence | n/a | n/a | ✓ | **PASS** |
| **P4** 3D RMSE | n/a | n/a | ✓ (40.9 nm) | **PASS** |

**Overall**: positioning is thermal-limited at all operating points tested,
Section 5 prediction verified to ±5% in every scenario, 3D RMSE matches
paper. The controller is performing at its physics limit.

## 5. Important methodology note — del_pm vs del_pmr

Early drafts of this report (pre-2026-04-15) contained a measurement bug
that produced a spurious "~13% near-wall static excess" and "~29% dynamic
excess." The issue was that empirical variance was computed from `del_pm`
(raw tracking error) while the theoretical formula predicts `Var(del_pmr)`
(HP-filtered residual).

- **`del_pm[k] = p_d[k-2] - p_m[k]`** — raw measured tracking error
- **`del_pmd[k] = (1-a_pd) del_pmd[k-1] + a_pd del_pm[k]`** — LP of del_pm
- **`del_pmr[k] = del_pm[k] - del_pmd[k]`** — HP residual
- **Section 5 predicts `Var(del_pmr)`**, not `Var(del_pm)`

In closed-loop the tracking error has strong autocorrelation (`ρ(1) ≈ 0.85`),
so the LP `del_pmd` tracks the slow drift, making
```
Var(del_pm) ≈ 1.27 · Var(del_pmr)
```
Taking sqrt gives a ~13% std inflation. This is **entirely independent
of h**, so it's present in free-space too — we just didn't notice because
`phase2_chisquared_mc.mat` already had a pre-computed `del_pmr` field that we
loaded directly. When `verify_p2_static_h25.m` computed `del_pm` from
scratch and compared it directly to the Section 5 theoretical
(Var(del_pmr)), the mismatch manifested as a 13% "excess."

Both analysis scripts (`verify_p2_static_h25.m` and `analyze_p2_h_bin.m`)
now reapply the offline IIR to obtain `del_pmr` before computing empirical
variance. The corrected results are shown above.

**Implication for Section 5**: no architectural residual exists at
near-wall. Section 5's `C_dpmr = 3.9242` is accurate in free-space and
at `h = 2.5 µm`. The `f_dx → 0` linearization simplification (Section 5
scope note) has negligible numerical impact at this operating point.

### 5.1 Direct R-ratio verification (follow-up)

The §5 scope-note claim was originally supported only indirectly (via the
±5% C_dpmr match above). A follow-up task directly measured the ratio
`R = σ²_f · σ²_e / (4·k_B·T·a_x)` per axis and confirmed `R ≪ 1` in both
free-space static and near-wall static (h = 2.5 μm) scenarios:
all six measurements fall in 0.6% – 1.3%.

See `task_p2_linearization_quantitative.md` for the full report.

## 6. Follow-up candidates (none required for positioning)

Because positioning now passes all four criteria cleanly, no debug
follow-up is required. Remaining open items:

- **Task 1e time-varying Σ_aug**: was motivated by an assumed dynamic
  lag residual. Since the clean-signal dynamic ratio is ≤1.045 at the
  worst bin, there is no measurable dynamic residual to address. Task 1e
  may no longer be necessary for positioning purposes.
- **`â` spec**: next session topic, blocked previously on "lock
  positioning spec first." Now unblocked — positioning is locked.
- **Section 7 (numerical results)** writeup integration: optional
  add-on, Task P2 data is ready if/when desired.
