# Development Log — Phase 8 (Wave 1-3 + Stage 10)

> Synthesis of 7 phase8 development reports. Originals preserved in
> `archive/sessions/`. Canonical result lives in
> `phase8_e2e_h50_results_v4.md` (current dir).

## Abstract

Phase 8 was the implementation push that took the Phase 1-7 v2 derivation
(Paper Eq.17 with Σ f_d retained + x_D additive + F_e Eq.19 form) and turned
it into runnable code on `test/eq17-5state-ekf`. The path included two
read-only audits (settings inherited from sigma branch, gap inventory v1 vs
v2), three implementation waves (controller, oracle, driver), a test suite
build-out (Wave 3 unit tests + smoke), an e2e divergence root-cause + fix
(Stage 10 G1 slot 6/7 gating), and converged at the `v4` canonical results
documented separately.

**Outcome**: v2 controller running at h=50 positioning, paper-level
~30 nm tracking and ~5-7% a_hat std, 5-seed CI integrity verified after
Stage 10 fix.

## Source Index

| # | Source (archived) | Phase | Role |
|---|---|---|---|
| 1 | `phase8_settings_audit.md` | Wave 1 audit | What sigma settings can be inherited, what must be re-derived |
| 2 | `phase8_eq17_state_audit.md` | Wave 1 audit | v1 vs v2 controller gap inventory; 3 root-cause bugs identified |
| 3 | `phase8_wave2B_controller_v2.md` | Wave 2 impl | Controller v2 implementation; 564→708 LoC |
| 4 | `phase8_wave2C_oracle.md` | Wave 2 impl | predict_closed_loop_var_eq17_v2 oracle + 5 unit tests |
| 5 | `phase8_wave2D_driver.md` | Wave 2 impl | Driver buffer fix d→d+1; Bus chain extension |
| 6 | `phase8_wave3_test_results.md` | Wave 3 verify | 7 priority unit tests + smoke; all PASS |
| 7 | `phase8_stage10_optionA.md` | Stage 10 fix | â_x divergence at step 122 root-caused + G1 slot 6/7 gate fix |

---

## 1. Settings Audit (Wave 1, 2026-04-29)

Read-only audit of `feat/sigma-ratio-filter` to extract settings worth
inheriting for the v2 eq17 implementation.

**Inheritable from sigma**:
- ParamsBus layout pattern, calc_simulation_params Bus dimensions, run_simulation
  structure, dispatcher pattern, IIR pre-fill mechanism
- Meas noise model + calibration values
- Wall-aware a_hat init concept (compute a_x from c_para/c_perp at h_bar_init)

**Must re-derive** (do NOT borrow from sigma):
- All Q matrix entries (derive per Phase 5 for 7-state v2 structure)
- R(2,2) (derive per Phase 6 time-varying instead of borrowing 0.01881 constant)
- a_cov (lock per design_v2 §5)

**Open questions flagged for reviewer** before Wave 2:
- a_cov target value
- Adaptive R(2,2) keep or replace
- KF runtime gate spec (t_warmup_kf, h_bar_safe)
- Gate-triggered R(2,2) inflation magnitude (1e6 vs 1e10)

## 2. v1 vs v2 State Audit (Wave 1, 2026-04-29)

Identified 3 root-cause bugs in v1 implementation that needed fixing in v2:

**Bug 1 — Σf_d missing in control law**: v1 `f_d[k]` omits the
`−(1−λ_c)·Σ_{i=1..d} f_d[k−i]` term that Paper Eq.17 / Phase 1 §2.3 specifies.
This was the structural source of the 26-44% a_hat bias documented in
`commit 89e6521` and `agent_docs/eq17/eq17-verification.md`. Without Σf_d,
closed-loop F_e does not match what the EKF assumes → systematic bias.

**Bug 2 — Sensor delay buffer off-by-one**: `run_pure_simulation.m` buffer of
length `d_delay=2` delivers `p_m[k−1]` (1-step delay) to the controller instead
of `p_m[k−2]` (2-step). Fix requires joint update of driver buffer length AND
controller `pd_km1`/`pd_km2` synchronization.

**Bug 3 — F_e(3,4) constant -1 vs -1.6**: Phase 1 §10.4 derives
`F_e(3,4) = -(1 + d·(1−λ_c)) = -1.6` for the v2 Eq.19 form (with Σf_d). v1
code uses -1 (simplified). Single-line constant change, medium impact on
Lyapunov / Riccati steady-state.

**Implementation order recommended** for Wave 2:
1. Fix sensor delay (driver + controller sync)
2. Add Σf_d term + persistent f_d history
3. Update F_e(3,4) to -1.6
4. Add sigma2_w_fD knob (default 0)

## 3. Wave 2B — Controller v2 Implementation (2026-04-29)

Implemented `motion_control_law_eq17_7state.m` v2 per Phase 0-7 derivations
and Wave 1 audit findings. 564 → 708 LoC (+144 net).

Key changes:
- `f_d_km1`, `f_d_km2` persistent buffers for Σf_d term
- Σf_d retained outside 1/â bracket (Strategy 1 stability per Phase 1 §4.2)
- x̂_D as separate additive `-x̂_D/â` term
- F_e Row 3 in Eq.19 form (F_e(3,3)=λ_c, F_e(3,4)=-1.6, F_e(3,6)=-f_d[k])
- Wall-aware â_x[0] init via c_para/c_perp at h_init
- Per-axis Pf_init from a_x_init
- a_cov via ctrl_const

**Status at end of Wave 2B**: checkcode clean, NOT committed, NOT simulated.
Ready for Wave 2C oracle + Wave 2D driver.

## 4. Wave 2C — Oracle Implementation (2026-04-29)

Implemented `model/diag/predict_closed_loop_var_eq17_v2.m` (later renamed to
`predict_closed_loop_var_eq17.m` in new-main) per Phase 7 §6 template:

- 234 lines
- Augmented-Lyapunov closed-loop variance predictor for eq17_7state-v2
- Provides `ratio_obs_to_pred` diagnostic for verify scripts
- 5 unit tests all PASS

Implementation deviations from Phase 7 template noted:
- Uses `a_x_axis(i)^2` per template (vs nominal `a_nom_axis`); revisit if motion
  scenarios need refinement
- `E[bracket²]` motion estimate uses `(A·ω·Ts)²/2` rough analytic; flagged for
  potential closed-form refinement against motion verification observations

## 5. Wave 2D — Driver + Bus Chain (2026-04-29)

Implemented driver-side companion to Wave 2B/2C:

| Task | File | Change |
|---|---|---|
| Buffer length fix `d → d+1` for true d-step delay | `run_pure_simulation.m` | +13/-5 |
| Driver pass `eq17_opts.a_pd` + `sigma2_w_fD` | `run_pure_simulation.m` | +7 |
| Bus chain `a_pd` + `sigma2_w_fD` extension | `calc_ctrl_params.m`, `user_config.m`, `calc_simulation_params.m` | (low risk, backward-compat with default-0 fallback) |
| `run_v2_h50_e2e.m` wrapper | `test_script/integration/` | new |

Critical-path note: Driver buffer fix `d → d+1` combined with Agent B's
`pd_km1`/`pd_km2` alignment now implements true 2-step sensor delay matching
Phase 1 §10.4 `F_e(3,4) = -1.6`.

## 6. Wave 3 — Test Suite Build-out (2026-04-29)

Built and verified 7 priority unit tests per deep audit `eq17_simulation_DEEP_audit_2026-04-29.md`:

- `test_delay_R2_factor_d_sweep.m`
- `test_iir_centering_response.m`
- `test_F_e_time_varying_entry.m`
- `test_predict_var_3state_vs_7state.m`
- `test_buffer_alignment_d_sweep.m`
- `test_h_bar_threshold_3guard.m`
- `test_thermal_force_anisotropy.m`

Plus `smoke_wave3_h50_no_noise.m` — Wave 2 integration smoke test.

All tests PASS. Regression on pre-existing tests also clean.

(In new-main these are renamed `verify_eq17_unit_*.m` under
`test_script/unit_tests/`.)

## 7. Stage 10 — G1 KF Slot 6/7 Gating Fix (2026-04-29)

Wave 4 e2e (5-seed) revealed structural divergence: `â_x` diverged through 0
(running to ±∞) at step ~122 across all 5 seeds, producing NaN.

**Root cause**: During G1 warm-up (t < 0.2 s), `F_e(3,6) = -f_d` accumulates
the f_d action through KF gain `K_kf(6,:)` and `K_kf(7,:)` on slot 6 (`â`)
and slot 7 (`δâ`). Even though y_2 is gated OFF, the y_1 update path leaks
into slots 6/7 via the off-diagonal P entries, causing a cross-coupling
amplifier that drives `â_x` runaway.

**Fix (Option A)**: During G1, gate the K_kf rows for slots 6/7 to zero:

```matlab
if guard_G1
    K_kf(6,:) = 0;
    K_kf(7,:) = 0;
end
```

Slot 6/7 stays frozen at init values during G1; releases after t ≥ t_warmup_kf.

**Verification** (seed=1, T_sim=1 s):
- Lines changed: +12, -0
- Checkcode: clean
- Smoke NaN: NONE on a_hat_x, p_m, f_d
- G1 freeze functional: â std = 1.56e-17 during t<0.2 s
- G1 release functional: â updates from 0.2 s onward
- Tracking std: x=28.9, y=28.5, z=30.9 nm (≈ baseline 30 nm)
- Divergence: NO

Stage 10 ready to escalate to full 5-seed e2e.

---

## Conclusion

The Phase 8 development cycle established v2 controller as runnable + paper-grade
on positioning scenarios:

1. **Settings audit + state audit** identified the 3 v1 bugs (Σf_d missing,
   delay buffer off-by-one, F_e(3,4) wrong constant).
2. **Wave 2B/2C/2D** implemented the controller, oracle, and driver in
   parallel, each checkcode-clean and unit-tested.
3. **Wave 3** added 7 priority unit tests + integration smoke, all PASS.
4. **Stage 10** root-caused the cross-coupling divergence in G1 and gated it
   with K_kf(6:7,:)=0.

Canonical 5-seed results are in `phase8_e2e_h50_results_v4.md` (current dir,
not duplicated here). Phase 9 R(2,2) finite-α correction and downstream
investigations build on this foundation.

### Open issues carried into new-main

- Wave 2C oracle `E[bracket²]` motion estimate uses rough analytic; refine
  against motion verification observations
- Wave 2D driver `eq17_opts.a_pd` passthrough was uncommitted at time of write;
  has since been incorporated
- Stage 10 fix is in production but Stage 11 (per-axis effective C_dpmr_eff
  via augmented Lyapunov) was implemented separately
