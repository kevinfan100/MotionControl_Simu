# Near-Wall Numerical-Stability Gap — Investigation Log

> Synthesis of 3 reports tracing the discovery and partial fix of catastrophic
> KF divergence at h_bar < 2. Originals preserved in `archive/sessions/`.

## Abstract

When the controller is operated in trajectories that enter the near-wall region
(h_bar < 2 ≈ h < 4.5 um), the EKF Riccati update suffers numerical breakdown
(RCOND ≈ 5e-17, machine eps ≈ 2.2e-16) and state estimates diverge by factors
of 100 to 100,000×. The physical mechanism is identified: K_h(h_bar→1) grows
to −7.88, the Q66 prefactor `(a·K_h/R)²` jumps 60×, P matrix entries become
badly scaled, and `S = H·P·H' + R` becomes ill-conditioned.

Multiple fix paths were proposed; **K_h saturation (cap=0.3)** has been merged
behind a flag and shown to fully fix the descent scenario (T1), partially fix
the near-wall oscillation (T2), with no regression on safe-region scenarios
(T3/T4). Joseph form is also plumbed but does not solve the divergence alone.

**Open**: The fix is a saturation hack, not principled. Square-root KF refactor
remains a future option for full robustness. T2 near-wall dwell still has 28%
bias even with the cap. Paper Fig 6/9 reproduction is partially achieved.

## Source Index

| # | Source (now in archive/sessions/) | Date | Role |
|---|---|---|---|
| 1 | `paper_compare_report.md` | 2026-05-05 | Discovery: 8-scenario sweep finds divergence pattern |
| 2 | `architectural_fix_proposal.md` | 2026-05-06 | Proposal: 4 fix paths (Joseph, K_h cap, hybrid, sqrt-KF) |
| 3 | `Kh_cap_breakthrough.md` | 2026-05-06 | Result: K_h cap=0.3 fixes T1 fully, T2 partially |

---

## 1. Discovery (paper_compare_report.md, 2026-05-05)

### Setup

Tested 4 trajectories × 2 λ_c (=8 sims) to compare against paper Fig 6/9
expectations:

```
controller_type     = eq17_7state
a_pd                = 0.05, a_cov = 0.005
Q66_physical_mode   = true            (Phase 5 §6.9 derivation)
cdpmr_method        = lyapunov
iir_warmup_mode     = prefill
t_warmup_kf         = 0
meas_noise_std      = [0.62, 0.057, 3.31] × 1e-3 um (calibrated)
seed = 10
```

Trajectories:

| Tag | Type | h_init | h_bottom | Notes |
|---|---|---|---|---|
| T1 | ramp_descent | 15 | 2.5 | Paper Fig 4/6 z-descent |
| T2 | osc | 4.5 | 2.5 | Paper Fig 7/9 near-wall (3 Hz, 9 cycles) |
| T3 | ramp_descent | 50 | 5 | Our standard (safe) |
| T4 | positioning | 15 | 15 | Hold |

### 8-condition result summary

```
Trajectory                lc     bz%        sz%       zerr(nm)
T1 paper descent          0.40  +43098    291003    4294   ★ DIVERGE
T1                        0.70  +285      1753      163    ★ DIVERGE
T2 paper nearwall osc     0.40  +2.6e6   3.5e6     4155   ★ DIVERGE
T2                        0.70  -2.9e6   1.2e7     802    ★ DIVERGE
T3 our ramp 50→5          0.40  +31.21    12.26     28.5
T3                        0.70  -0.04     7.68      29.8
T4 positioning h=15       0.40  +31.36    13.69     27.4
T4                        0.70  +3.11     5.83      29.4
```

### Diagnosis

MATLAB warns `RCOND ≈ 5e-17` during T1/T2 sims. KF Riccati `S = H·P·H' + R`
ill-conditioned. Root cause: K_h(h_bar→1) = -7.88 at h_bar=1.11 makes Q66
prefactor `(a·K_h/R)²` jump 60×. P matrix entries become badly scaled.
S^-1 amplifies numerical noise ~1e16× → garbage state.

**This is NOT a Q tuning issue. It is a KF numerical stability issue at the
hydrodynamic singularity h_bar → 1.**

### Verdict

Safe-region (h_bar > 2.2) achieves paper-grade tracking (~30 nm err, ~7% std,
~5% bias). Paper-aligned scenarios diverge by 100-100,000×. Architectural
change required.

## 2. Fix Path Proposal (architectural_fix_proposal.md, 2026-05-06)

### Root-cause code

`model/controller/motion_control_law_eq17_core.m` lines 707-781 use the
standard form:

```matlab
P_pred = F_e * P_curr * F_e' + Q_per_axis{ax};
S = H_use * P_pred * H_use' + R_use;
K_kf = (P_pred * H_use') / S;
P_post = (eye(7) - K_kf * H_use) * P_pred;        % ★ STANDARD form
```

The `(I - K·H)·P_pred` form is mathematically correct but numerically unstable
under ill-conditioned P_pred and near-singular S — catastrophic cancellation.

### Four fix paths proposed

**Fix 1 — Joseph form** (smallest diff, well-known stability technique):
```matlab
I_minus_KH = eye(7) - K_kf * H_use;
P_post = I_minus_KH * P_pred * I_minus_KH' + K_kf * R_use * K_kf';
```
- Mathematically equivalent in exact arithmetic
- Always symmetric (sum of two PSD terms is PSD)
- More robust under floating-point

**Fix 2 — K_h saturation** (hack but simple):
```matlab
K_h_max = 1.0;
K_h_axis = sign(K_h_axis) .* min(abs(K_h_axis), K_h_max);
K_h_pr_axis = sign(K_h_pr_axis) .* min(abs(K_h_pr_axis), K_h_max^2);
```
- Breaks physical accuracy near wall but bounds Q66 prefactor

**Fix 3 — Hybrid**: Joseph + K_h cap. Joseph as primary, cap as safety margin.

**Fix 4 — Square-root KF** (largest refactor): UD factorization `P = U·D·U'`,
never form P directly. Most robust, ~50-line refactor.

### Recommendation

Joseph form (Fix 1) first; if fails, escalate to sqrt-KF (Fix 4).

## 3. K_h Saturation Breakthrough (Kh_cap_breakthrough.md, 2026-05-06)

### What was implemented

Added two new flags to controller:

- `K_h_cap` (default 0 = no cap, legacy behavior preserved)
- `use_joseph_form` (default false)

When `K_h_cap > 0`:

```matlab
K_h_axis    = sign(K_h_axis)    .* min(abs(K_h_axis),    cap);
K_h_pr_axis = sign(K_h_pr_axis) .* min(abs(K_h_pr_axis), cap^2);
```

Joseph form was tested but does NOT solve divergence alone.

### Results with K_h_cap=0.3 (recommended) + physical Q66 + IIR prefill + t_warmup=0

```
                          cap=0 (legacy)   cap=0.3 (new)    paper target
T1 descent 15→2.5
   zerr                   4294 nm DIVERGE  25.36 nm         ✓ achieved
   bias_z                 +43098%          -2.02%           ✓ within 5%
T2 near-wall osc h_bar~1.1 dwell
   zerr                   4155 nm DIVERGE  34 nm            ◐ partial
   bias_z                 +2.6e6%          +28%             ◐ improved 1e5×
T3 safe ramp 50→5
   zerr                   29.86 nm         30.23 nm         ✓ no regression
   bias_z                 +0.14%           +1.85%
T4 positioning h=15
   zerr                   28.48 nm         27.57 nm         ✓ no regression
   bias_z                 -3.56%           -8.30%           ◐ slight degradation
```

### Cap value tuning

```
cap=0.5:   T1 OK (1.23% bias), T2 still bad → too loose
cap=0.3:   T1 great, T2 partial, T3/T4 OK     ← recommended default
cap=0.2:   T2 best zerr 25 nm, but slightly worse on T3/T4 → for near-wall apps
cap=0.1:   T2 over-capped (190% bias)         → too tight
cap=0.05:  particle hits wall                 → unusable
```

### Why it works (analytically)

K_h(h_bar=1.11)=-7.88 → (a·K_h/R)² jumps 60×. P_pred entries get badly scaled.
Capping |K_h| ≤ 0.3 clamps the scaling factor to 0.018. Q66 prefactor stays
bounded. P matrix entries stay similar order of magnitude. S well-conditioned.
KF Riccati clean.

### Trade-off

K_h saturation breaks physical accuracy near wall — assumes c_perp/c_para
don't grow as steeply as they actually do. KF underestimates how fast a is
changing in very-near-wall.

In return: KF stays stable, particle stays in-bounds, paper-grade tracking.

---

## Conclusion / Status

| Issue | Status |
|---|---|
| Divergence in h_bar < 2 region | Identified and characterized (RCOND, K_h blow-up mechanism) |
| Fix-path proposals | 4 paths proposed (Joseph, K_h cap, hybrid, sqrt-KF) |
| K_h cap=0.3 | Implemented behind flag; T1 ✓ T2 ◐ T3 ✓ T4 ◐ |
| Joseph form alone | Tested, does NOT solve divergence |
| Joseph + K_h cap | Tested, no improvement over K_h cap alone |
| sqrt-KF (Fix 4) | NOT implemented; reserved for future if K_h cap proves insufficient |

### Open issues (carried into new-main)

1. **T2 still has 28% bias at cap=0.3**: Dwell at h_bar~1.1 accumulates more
   model error than a single transit. Possible refinements: per-axis cap (z
   stricter), h_bar-dependent cap.
2. **T4 small bias degradation** (-3.56% → -8.3%): Cap perturbs steady-state
   bias slightly. Tolerable but worth understanding.
3. **Principled fix still pending**: K_h cap is a hack. If long-term robustness
   matters, square-root KF refactor is the principled answer.

### Recommended production default

`K_h_cap = 0.3` for production. User can tune to scenario (smaller for dwell
near wall, larger if mostly safe-region).

Joseph form remains as an experimental flag (`use_joseph_form`) with no
demonstrated benefit yet but no demonstrated harm either.
