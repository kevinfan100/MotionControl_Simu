# Architectural Fix Proposal — Near-Wall KF Numerical Stability

**Date**: 2026-05-06  
**Status**: Proposal awaiting user direction

## Problem Summary

Per `paper_compare_report.md`, our eq17_7state controller **catastrophically diverges** when h_bar < 2 (= h < 4.5 um). Paper Fig 6/9 scenarios all fail. Numerical mechanism: KF Riccati S^-1 is computed with RCOND ≈ 5e-17 (machine ε = 2.2e-16), producing garbage state updates.

Visual evidence: `figures/paper_compare/T2_paper_nearwall_osc_lc07_fig.png` shows a_z dropping to -1700 um/pN, a_x oscillating ±10^7 around t=1.5s.

## Root Cause Code Location

`model/controller/motion_control_law_eq17_7state.m` lines 707-781:

```matlab
% Predict
P_pred = F_e * P_curr * F_e' + Q_per_axis{ax};
P_pred = 0.5 * (P_pred + P_pred');                % symmetrize

% Update
S = H_use * P_pred * H_use' + R_use;
K_kf = (P_pred * H_use') / S;
P_post = (eye(7) - K_kf * H_use) * P_pred;        % ★ STANDARD form
P_post = 0.5 * (P_post + P_post');                % symmetrize
```

The `(I - K·H)·P_pred` form is mathematically correct but **numerically unstable** when:
- P_pred is ill-conditioned (entries span many orders of magnitude)
- S matrix is near-singular
- Subtraction `I - K·H` involves catastrophic cancellation

This is exactly our scenario when K_h(h_bar) blows up near the wall.

## Recommended Fix Path 1: Joseph Form

**Smallest diff**, well-known KF stability technique:

```matlab
% Replace line 760-761 with:
I_minus_KH = eye(7) - K_kf * H_use;
P_post = I_minus_KH * P_pred * I_minus_KH' + K_kf * R_use * K_kf';
% No need for explicit symmetrization (Joseph form preserves symmetry)
```

### Properties

- Mathematically equivalent to standard form (same in exact arithmetic)
- Always symmetric (`P = A·X·A' + B·Y·B'` is symmetric if X, Y symmetric)
- Always positive semi-definite (sum of two PSD terms)
- More numerically robust under floating-point arithmetic

### Cost
- ~1 extra matrix multiply per axis per step (~3 axes × 1600 steps/sec = 4800 extra mults/sec)
- Negligible CPU impact

### Risk
- Could change steady-state P slightly due to different rounding behavior
- Need re-verify on existing safe-region scenarios (T3, T4) to ensure no regression

## Recommended Fix Path 2: K_h Saturation

**Even simpler hack**, sacrifices physical accuracy for numerical robustness:

```matlab
% In §4 K_h_axis assignment, after calc_correction_functions:
K_h_max = 1.0;   % saturation threshold
K_h_axis = sign(K_h_axis) .* min(abs(K_h_axis), K_h_max);
K_h_pr_axis = sign(K_h_pr_axis) .* min(abs(K_h_pr_axis), K_h_max^2);
```

### Effect at h_bar=1.11

- K_h_perp from -7.88 → -1.0
- (a·K_h/R)² from 1e-5 → 4e-7 (factor 25 reduction)
- Q66 prefactor reduced 25×
- P matrix entries less skewed

### Cons

- Breaks physics for true near-wall behavior
- Underestimates Q66 in legitimate near-wall scenarios
- Not principled

## Recommended Fix Path 3: Hybrid

Apply Joseph form (Fix 1) AND K_h soft saturation. Joseph form gets us most of the way; K_h cap as conservative safety margin.

## Verification Plan

After implementing fix:

1. Re-run all 8 paper-compare conditions (T1-T4 × lc=0.4/0.7)
2. Check T1, T2 no longer diverge (should bound errors to physical magnitude)
3. Verify T3, T4 results unchanged within noise (no regression)
4. Compare to paper Fig 6/9 visually — does our std now match paper?

## Alternative: Square-Root KF (Fix 4, larger refactor)

Use UD factorization: `P = U·D·U'`. Operate on U and D, never form P directly. Avoids condition number squaring entirely. ~50-line refactor of the EKF update. Most robust but most invasive.

Recommended ONLY if Joseph form (Fix 1) doesn't fully solve.

## Decision Needed

User to pick: Fix 1 (Joseph), Fix 2 (K_h cap), Fix 3 (hybrid), or Fix 4 (sqrt KF)?

My recommendation: **Fix 1 (Joseph form) first**, single test against T1 paper scenario. If passes, propagate to all 8 conditions. If still diverges, escalate to Fix 4.

Code change ~3 lines, test cost < 10 min. Low risk, potentially high reward.
