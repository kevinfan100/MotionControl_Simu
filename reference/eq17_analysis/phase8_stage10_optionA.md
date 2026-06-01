# Phase 8 Stage 10 — Option A: G1 KF Slot 6/7 Gating

**Branch**: `test/eq17-5state-ekf`
**Date**: 2026-04-29
**Status**: Smoke test PASS (1 seed, T_sim=1s) — ready for full 5-seed e2e re-run

---

## 1. Background

Wave 4 e2e revealed a structural divergence: â_x diverged through 0
(running away to ±∞) at step ~122 across all 5 seeds, producing NaN.

**Mechanism (root-caused in pre-Stage-10 sanity check)**:
- During G1 (warm-up, t < 0.2s), F_e(3,6) = -f_d accumulates the
  cross-covariance entry P_pred(3,6).
- The shift structure of F_e then leaks this into P_pred(6,1).
- y_1 (= delta_x_m) has measurement noise σ²_n_s; through K_kf(6,1) =
  P_pred(6,1)/S(1,1), this noise propagates into the â_x posterior.
- Bias accumulates step-by-step. â_x (initial wall-aware ~1.43e-02
  um/pN) drifts to negative values, hits 0, then control law
  1/â_x · (...) blows up → numerical NaN.

A standalone diagnostic confirmed: locking â_x = a_nom (the
`ctrl_const.a_hat_freeze` path) makes the run structurally stable,
isolating the bug to the â_x update channel itself.

## 2. Code Change

### Location

File: `model/controller/motion_control_law_eq17_7state.m`
Function: `motion_control_law_eq17_7state` — main predict/update loop
(line 516-588 in modified file).

### Diff (12 added lines, 0 removed)

Inserted between `K_kf` computation and `x_post`/`P_post` update,
inside the per-axis EKF loop:

```matlab
S = H_use * P_pred * H_use' + R_use;
S = 0.5 * (S + S');                     % symmetrize
K_kf = (P_pred * H_use') / S;           % 7x{1,2}

% --- Stage 10 Option A: gate slot 6 + 7 update during G1 (warm-up) ---
% Wave 4 root cause: F_e(3,6) = -f_d accumulates P_pred(3,6) cross-cov
% during warm-up, leaking through K_kf(6,1)·y_1 to drive â_x runaway
% (â_x can be pushed past 0 → control law blowup at step ~122).
% Gate row 6 + 7 of K_kf during G1 only — slot 6/7 evolve via Jordan
% integration only (a_x = const, δa_x ≈ 0). G2/G3 do not gate slot 6/7
% (R(2,2)=R_OFF / 1D y_1-only update already disables y_2 contribution).
if G1
    K_kf(6, :) = 0;     % gate slot 6 (a_x) measurement update
    K_kf(7, :) = 0;     % gate slot 7 (δa_x) — Jordan-pair consistency
end

x_post = x_pred + K_kf * innov;
```

### Notes on existing structure

The pre-existing controller already used a **sequential 1D y_1-only
update** when `gate_y2_off` is true (G1 || G2 || G3):

```matlab
if gate_y2_off
    H_use = H_y1;       % y_1 only
    y_use = delta_x_m(ax);
    R_use = sigma2_n_s(ax);
else
    H_use = H_full;     % both y_1, y_2
    ...
end
```

So the K_kf size adapts:
- **G1/G2/G3 active**: K_kf is 7x1 (y_1 only); `K_kf(6,:) = 0` zeros
  the (6,1) entry.
- **All gates off**: K_kf is 7x2; `K_kf(6,:) = 0` zeros (6,1) and
  (6,2) (the y_2-driven a_x update channel was the original design,
  but is also OK to zero during G1 since G1 ⇒ y_2 already gated).

The conditional uses the per-axis `G1` boolean computed earlier in the
loop (line 536), so it applies to all three axes individually.

### Why G2/G3 are NOT gated

Per task spec: G2 (low SNR) and G3 (near-wall) are
measurement-quality issues. When IIR has matured (post-warmup), the
existing `R(2,2) = R_OFF` / 1D y_1-only fall-back is sufficient
because:
- y_2 cannot contaminate slot 6 (gate already disables it).
- y_1 → slot 6 leakage via P_pred(6,1) is small in steady state
  (P stabilizes to a regime where the cross-coupling is bounded).

The Wave 4 bug was specific to G1 because warm-up is when:
1. P_pred(6,*) entries grow rapidly from 1e-5 init.
2. f_d may be non-zero (controller acts) → F_e(3,6) drives cross-cov.
3. y_1 channel still feeds back through K_kf(6,1).

## 3. Smoke Test Results

### Configuration

- Seed: 1
- T_sim: 1.0 s (covers G1 transient + ~30 G1-release steps)
- h_init = 50 um, positioning trajectory
- meas_noise + thermal: ON
- All other config matches `run_v2_h50_e2e` defaults

### Numerical results

| Phase | Time window | a_hat_x mean | a_hat_x std | min | max |
|-------|------|---|---|---|---|
| Phase 1 (G1 active) | t < 0.2s, n=320 | 1.4334e-02 | 1.56e-17 | 1.4334e-02 | 1.4334e-02 |
| Phase 2 (G1 release, transient) | 0.2 ≤ t < 0.5s, n=480 | 9.3026e-03 | 7.53e-04 | — | — |
| Phase 3 (steady) | t ≥ 0.5s, n=801 | 9.8395e-03 | 1.71e-04 | — | — |

Wall-aware init expected:
- a_x_init = 1.4334e-02 um/pN (c_para = 1.0260, h_bar_init = 26.94)
- a_z_init = 1.3962e-02 um/pN (c_perp = 1.0533)

**Step 122** (= original Wave 4 failure point, t = 76.3 ms):
a_hat_x = 1.4334e-02 (== wall-aware init) — no divergence.

### Tracking std (all seed=1, t ≥ 0.5s window)

| axis | tracking std (nm) |
|------|-------------------|
| x | 28.90 |
| y | 28.55 |
| z | 30.92 |

(Sanity-check baseline ~30 nm cited in brief; all three axes within
±2 nm of that range. Consistent with stable closed-loop operation.)

### NaN check

| signal | has NaN |
|--------|---|
| a_hat_x | 0 |
| a_hat_y | 0 |
| a_hat_z | 0 |
| p_m | 0 |
| f_d | 0 |

### Behavioral verification

1. **G1 freeze working**: Phase 1 std = 1.56e-17 ≈ machine epsilon →
   a_hat truly constant during 0..0.2s, exactly the wall-aware init
   value. Confirms K_kf(6,:) = K_kf(7,:) = 0 is being applied.
2. **G1 release working**: Phase 2 (0.2..0.5s) shows â_x updates
   (mean 1.4334e-02 → 9.30e-03), and Phase 3 settles to 9.84e-03 with
   std 1.7e-04 (≈ 1.7% of mean) — active filtering with bounded
   covariance.
3. **No structural divergence**: Wave 4's step 122 NaN is gone; a_hat
   stays in physical range (positive, bounded) throughout.

### a_hat bias observation (post-handoff)

- Phase 3 a_hat_x = 9.84e-03, vs nominal 1.43e-02 → bias **−31%**.

This bias is NOT a stability issue. It's the same a_hat bias
phenomenon documented in `project_h50_lookup_peraxis_fix.md`
(MEMORY index) — driven by C_dpmr / C_n calibration. Stage 10 only
fixed the warm-up runaway; the bias-correction work is downstream
and not part of this scope.

For comparison, `run_v2_h50_e2e` summary header reports:
- bias x: -28.50%, y: -27.62%, z: -25.10%
- a_hat std x: 1.26e-04, y: 2.33e-04, z: 1.95e-04

## 4. Implementation Questions / Caveats

### None blocking. Two minor notes for the dispatcher:

**Note 1 — F_e(3,6) is still active during G1**

We do NOT gate F_e(3,6) — only K_kf rows 6/7. The cross-covariance
P_pred(3,6) and P_pred(6,1) still grow during G1, but they have no
path to drive â_x because K_kf is forced to 0 at row 6/7. After G1
release, the cross-covariance is "real" (not zero) and KF correctly
incorporates it into the first active update.

This is intentional and matches the task spec ("純靠 Jordan
integration 演化"). The alternative (zeroing P_pred row/col 6,7
during G1) would discard valid cross-coupling information at the
G1 boundary and is not what Option A requires.

**Note 2 — checkcode clean**

`mcp__matlab__check_matlab_code` reports "No issues found" on the
modified file. No new lint warnings introduced.

## 5. Status / Next Step

| Item | Status |
|------|--------|
| Lines changed | +12, -0 |
| Checkcode | clean |
| Smoke seed=1 NaN | NONE (a_hat_x, p_m, f_d all clean) |
| G1 freeze functional | Verified (std 1.56e-17 during t<0.2s) |
| G1 release functional | Verified (â_x updates from 0.2s onward) |
| Tracking std | x=28.9, y=28.5, z=30.9 nm (≈ baseline ~30 nm) |
| Diverge | NO |
| Ready for full 5-seed e2e | YES |

**Hand-off**: dispatcher can re-run Wave 4 e2e with full 5 seeds to
confirm cross-seed CI integrity.
