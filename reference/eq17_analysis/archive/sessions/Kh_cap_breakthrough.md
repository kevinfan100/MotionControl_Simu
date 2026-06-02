# Architectural Fix 2 — K_h Saturation: Breakthrough Result

**Date**: 2026-05-06 (autonomous loop iteration 03 ~01:30)  
**Status**: T1 paper descent fully fixed, T2 paper near-wall osc partially fixed.  
**Code change**: 3 small edits (motion_control_law_eq17_7state.m, build_eq17_constants.m, run_pure_simulation.m)

## What changed

Added `K_h_cap` flag (default 0 = no cap, legacy behavior preserved). When > 0:

```matlab
% After calc_correction_functions in §4:
K_h_axis    = sign(K_h_axis)    .* min(abs(K_h_axis),    cap);
K_h_pr_axis = sign(K_h_pr_axis) .* min(abs(K_h_pr_axis), cap^2);
```

Also added `use_joseph_form` flag (default false). Tested but did NOT solve divergence alone.

## Results (lc=0.7, all flags physical Q66 + IIR prefill + t_warmup_kf=0)

```
              cap=0 (legacy)          cap=0.3 (new)            paper-grade target
T1 descent 15→2.5
   zerr       4294 nm (DIVERGE)        25.36 nm              ✓ achieved
   bias_z     +43098%                  -2.02%                ✓ within 5%
T2 near-wall osc h̄~1.1 dwell
   zerr       4155 nm (DIVERGE)        34 nm                 ◐ partial
   bias_z     +2617109%                +28%                  ◐ improved 100,000×
T3 safe ramp 50→5
   zerr       29.86 nm                 30.23 nm              ✓ no regression
   bias_z     +0.14%                   +1.85%
T4 positioning h=15
   zerr       28.48 nm                 27.57 nm              ✓ no regression
   bias_z     -3.56%                   -8.30%                ◐ slight degradation
```

## Why it works (analytically)

K_h(h̄→1) blows up to -7.88 at h̄=1.11. (a·K_h/R)² jumps 60×. P_pred entries become badly scaled.
Capping |K_h| ≤ 0.3 clamps the scaling factor to 0.018. Q66 prefactor stays bounded.
P matrix entries stay similar order of magnitude. S = H·P·H' + R well-conditioned.
KF Riccati S^-1 numerically clean. State updates clean. No divergence.

## Trade-off

K_h saturation **breaks physical accuracy near wall** — assumes c_perp/c_para don't grow as steeply as they actually do. KF underestimates how fast a is changing in the very-near-wall region.

In return: KF stays numerically stable, particle doesn't get pushed through wall, paper-grade tracking achievable.

## Choosing cap value

```
cap=0.5:   T1 OK (1.23% bias), T2 still bad → too loose
cap=0.3:   T1 great, T2 partial, T3/T4 OK     ← recommended default
cap=0.2:   T2 best zerr 25 nm, but slightly worse on T3/T4 → for near-wall apps
cap=0.1:   T2 over-capped (190% bias)         → too tight
cap=0.05:  particle hits wall                 → unusable
```

**Recommendation**: `K_h_cap = 0.3` as production default. User can tune to scenario.

## Open issues

1. **T2 still has 28% bias at cap=0.3**: dwell time at h̄~1.1 longer than T1 (which only crosses), accumulates more model error. Possible refinements: per-axis cap (z stricter than x/y), or h̄-dependent cap that loosens for transient passes.

2. **Joseph form alone doesn't fix divergence**: tested, KF gave different but still wrong state (caused particle to hit wall). Combined Joseph + K_h cap doesn't help over K_h cap alone. Joseph form retained as flag for completeness.

3. **T4 small bias degradation** (-3.56% → -8.3%): cap perturbs steady-state bias slightly. Tolerable but worth understanding.

## Files

- Code edits: motion_control_law_eq17_7state.m, build_eq17_constants.m, run_pure_simulation.m (default behavior preserved)
- Report: this file
- Previous diagnosis: paper_compare_report.md, architectural_fix_proposal.md
- Reference data: figures/paper_compare/

## Next steps (user to decide)

1. Accept K_h_cap=0.3 as architectural fix → commit + push
2. Refine T2 with per-axis cap or h̄-dependent cap
3. Investigate T4 bias degradation
4. Consider sqrt-KF (Fix 4) for full numerical robustness
