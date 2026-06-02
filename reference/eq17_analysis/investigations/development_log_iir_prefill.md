# Development Log — IIR Warmup Pre-fill Validation

> Synthesis of 5 iteration reports. Originals preserved in `archive/sessions/`.

## Abstract

The `iir_warmup_mode='prefill'` feature (default since commit `2ea3882`) seeds
the per-axis IIR running variance `sigma2_dxr_hat` to its analytic steady-state
value at `h_init` and sets `warmup_count=0`, replacing the legacy two-step zero-
IC warmup. This avoids the cold-start transient where `a_m≈0` for the first
~1000 samples (driving a_hat low). Five validation rounds were run (single-seed
compare → 5-seed positioning → closed-loop with EKF unfrozen → ramp_descent →
t_warmup_kf sweep) before promoting it to production default.

**Summary verdict**: Pre-fill is safe and preferred in positioning, neutral-to-
positive in ramp_descent, and lets `t_warmup_kf` drop from 0.2 s to 0 without
regression at h=50.

## Source Index

| # | Source (now in archive/sessions/) | Purpose | Conclusion |
|---|---|---|---|
| 1 | `iir_prefill_seed1_compare.md` | Single-seed legacy vs prefill, h=50 positioning, 2 s | Indistinguishable within noise; first 2 ms transient eliminated |
| 2 | `iir_prefill_v1_5seed.md` | 5-seed × 2 modes, R(2,2) self-consistency at h=50 | Both modes: V1 ratio 0.97-1.00 PASS; no regression |
| 3 | `iir_prefill_closedloop_5seed.md` | 5-seed unfrozen-EKF closed-loop regression test | RMSE delta within ±1 pp/axis, tracking std identical |
| 4 | `iir_prefill_ramp_descent.md` | h=50→10 ramp at 4 um/s, seed=1 | Pre-fill SAFE for ramps within tau_IIR (~12.5 ms) vs ramp duration (10 s) |
| 5 | `iir_prefill_twarmup_sweep.md` | t_warmup_kf ∈ {0, 0.025, 0.05, 0.1, 0.2} × 3 seeds | t_warmup_kf=0 safe at h=50; previously 0.2 s margin can drop to 0 |

---

## 1. Seed 1 Compare (Round 1)

Driver: `model/dual_track/run_pure_simulation.m` (then `model/pure_matlab/`).
Scenario: h=50 positioning, T_sim=2.0 s, seed=1, EKF a_hat frozen.

Pre-fill seed at k=2:
- `sigma2_dxr_hat (prefill)` = `[7.38e-4, 7.37e-4, 7.28e-4]` um²
- `sigma2_dxr_hat (legacy)`  = `[0, 0, 0]` um²

Conclusion: First-2-step transient eliminated; downstream EKF behavior matches
legacy after ~5 samples. Bit-identical when `a_hat_freeze` is active (since
f_d doesn't depend on a_hat, which doesn't depend on sigma2_dxr in this
configuration).

## 2. V1 ratio 5-seed validation (Round 2)

5 seeds × 2 modes; metrics over t≥2 s of T_sim=15 s.

V1 variance ratio (`emp_var / R22_pred`, target ~1.0):

| axis | legacy mean ± SE | prefill mean ± SE |
|---|---|---|
| x | 1.001 ± 0.012 | 1.001 ± 0.012 |
| y | 0.970 ± 0.026 | 0.970 ± 0.026 |
| z | 0.979 ± 0.020 | 0.979 ± 0.020 |

Conclusion: Bit-identical within numerical precision (a_hat frozen masks any
prefill propagation). y-axis 1.9% mean DC bias on a_xm warrants separate
follow-up but is orthogonal to the prefill change.

## 3. Closed-loop unfrozen EKF (Round 3)

Same 5-seed × 2 modes but with `a_hat_freeze=[]`, allowing EKF slot 6 to
update normally. This is the actual production regression test.

Bias_pct = 100·(mean(a_hat_steady) - a_true)/a_true, per-axis 5-seed mean±std:

| axis | legacy mean ± std | prefill mean ± std | delta (prefill − legacy) |
|---|---|---|---|
| x | -6.80% ± 2.99 | -6.44% ± 3.08 | +0.36 pp |
| y | -7.48% ± 1.67 | -7.92% ± 2.30 | -0.44 pp |
| z | -7.23% ± 2.08 | -6.15% ± 3.42 | +1.08 pp |

Bias has ~3 pp std across seeds in both modes (matches chi-sq floor on a_hat
spread at h=50, ~19% relative).

Conclusion: Pre-fill helps z RMSE on average (-1.09 pp), slightly hurts y
(+0.41 pp); per-seed sign-flips dominate. tracking_std_nm essentially identical
(~30.5 nm). Pre-fill SAFE as production default for h=50 positioning.

## 4. ramp_descent scenario (Round 4)

50→10 um ramp at 4 um/s over 10 s, seed=1.

| Mode | RMSE ramp window (x, y, z) nm | a_hat bias |
|---|---|---|
| legacy | (33.4, 32.8, 31.6) | larger |
| prefill | (33.7, 34.0, 31.5) | **30-70% lower** |

Conclusion: Pre-fill seed becomes "stale" during the 10 s ramp but does not
cause runaway because the IIR EWMA (tau≈12.5 ms) tracks h(t) far faster than
the ramp. y +1.2 nm RMSE is marginal and at noise floor of the lowest-noise
axis. Pre-fill SAFE for ramp scenarios as far as 50→10 with `h_dot_max=4 um/s`;
caveat: not tested for closer wall (h_bottom ≤ 5 um).

## 5. t_warmup_kf minimum-stable sweep (Round 5)

After pre-fill is established, sweep the orthogonal `t_warmup_kf` parameter
to see if the legacy 0.2 s KF warmup margin can be relaxed.

15 sims (5 t_warmup values × 3 seeds), 7.2 min wall time. Metrics over
t ≥ 5 s of T_sim = 15 s.

Verdict: `t_warmup_kf=0` is stable across all 3 seeds at h=50 positioning,
with tracking std and a_hat std indistinguishable from the 0.2 s baseline.

Recommendation: Default reduced from 0.2 → 0 (or conservative 0.025 s = 40
steps) for positioning scenarios when pre-fill is active. Commit `2ea3882`
adopted t_warmup_kf=0 as the production-wide default.

Caveat: NOT validated for motion / closer wall (h ≤ 5 um, larger Q77 coupling).

---

## Conclusion

Pre-fill mode was promoted from experimental flag to production default after
five rounds of validation. Key takeaways:

1. **No regression**: All 5 validation rounds show statistically indistinguishable
   metrics within 1 pp on bias/RMSE/tracking_std at h=50 positioning.
2. **Bias improvement in ramps**: Pre-fill measurably reduces a_hat bias mid-
   ramp by 30-70% in the tested 50→10 scenario.
3. **Enables t_warmup_kf=0**: Pre-fill makes the KF transient region small enough
   that the prior 0.2 s G1 margin can drop to 0 without regression at h=50.
4. **Open**: Near-wall (h < 5 um) was not exercised; if Q77 wall-coupling shifts
   the IIR/KF dynamics significantly, pre-fill should be re-validated there.

For implementation details and code touch points see commits `e247b2c` (mode
addition) and `2ea3882` (default flip).
