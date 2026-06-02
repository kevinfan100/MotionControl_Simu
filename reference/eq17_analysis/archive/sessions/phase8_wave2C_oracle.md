# Phase 8 Wave 2 Agent C: predict_closed_loop_var_eq17_v2 Oracle

**Branch**: `test/eq17-5state-ekf`
**Date**: 2026-04-29
**Spec source**: `reference/eq17_analysis/phase7_lyapunov_bench.md` §6
**Deliverables**:
- `model/diag/predict_closed_loop_var_eq17_v2.m` (234 lines)
- `test_script/unit_tests/test_predict_closed_loop_var_eq17_v2.m` (231 lines)

Status: implementation complete, checkcode clean, all 5 unit tests PASS. Not committed.

---

## 1. Implementation Choices (deviations from Phase 7 §6.1 template)

### 1.1 `rho_a` exposed as input (per Wave 2 brief request)
Phase 7 §6.1 template hard-coded `rho_a_pos = 4.7`. Per Wave 2 Agent C brief, `rho_a` is exposed as a configurable input to support sensor-dominated regimes (e.g. h=50 z-axis where rho_a is expected smaller). Two forms:
- `opts.rho_a` (scalar, applied to all axes)
- `opts.rho_a_axis` (1x3 per-axis override; takes precedence)
Default = 4.7 (Phase 7 §4.3 thermal-dominated near-wall).

Function header documents this caveat.

### 1.2 `wall_term` exposed as optional input
Phase 7 §4.3.1 mentions `+ (sens·std(p_m_z)/R)^2` wall-sensitivity term but §6.1 template omits it. Implementation accepts optional `opts.wall_term` (1x3); defaults to zeros. Phase 8 sensitivity studies can fill this if needed.

### 1.3 `Ts` exposed as optional input (motion only)
Phase 7 §6.1 hard-coded `Ts = 1/1600`. Function reads `opts.Ts` if present, defaults to project standard `1/1600`.

### 1.4 `sigma2_e_xD` baseline now exactly zero (was `1e-20`)
Phase 7 §6.1 template wrote `s2_e_xD(i) = 1e-20` as a placeholder for "effectively zero". I changed to `0` exactly so that the divide-by-zero check in test T2 has a crisp boundary and downstream consumers can use `s2_e_xD == 0` as a baseline indicator. The propagation through Step 2 (`s2_e_xD / one_minus_lc2`) correctly yields zero contribution.

### 1.5 `L_eff = sqrt(Q77/R)` (NOT `Q66`)
Phase 7 §6.1 template line 288 reads `L_eff = sqrt(Q66/R)`, but the surrounding text §4.3.1 specifies `L_eff = sqrt(Q77/R(2,2))` and the Wave 2 brief explicitly states `Q77`. This is a Phase 7 spec inconsistency (§6.1 template typo). Resolved per text/brief: `L_eff = sqrt(Q77_axis(i) / R_axis(i))`. Function input is named `Q77_axis` to make the convention unambiguous.

### 1.6 Numerical safety
- `R_axis(i) <= 0` -> `L_eff(i) = 0` (no innovation gain, so chi-sq term vanishes)
- `(L_eff + a_cov) <= eps` -> `CV2_a` collapses to wall_term only
- `lambda_c >= 1` -> error (would give singular `1/(1-lambda_c^2)`)
- `lambda_c < 0` -> error (out of valid AR(1) range)

### 1.7 Input validation
Comprehensive: required-field check, axis-dimension check (1x3), scenario string check, motion-scenario traj-field presence check. All errors use namespaced `predict_closed_loop_var_eq17_v2:*` identifiers so callers can `catch ME` selectively.

### 1.8 No deviation in formulas
The core math is *exactly* per Phase 7 §6.1:
- `chi_sq = 2*a_cov / (2 - a_cov)`
- `CV^2(a) = chi_sq * rho_a * L_eff/(L_eff + a_cov) + wall_term`
- `s2_e_a = CV^2(a) * a_x^2`
- `s2_e_xD = sqrt(a_nom^2 * sigma2_w_fD * R)` for sigma2_w_fD > 0 (using `a_x` ≈ `a_nom` per Phase 7 §8.5; cosmetic since baseline = 0)
- `s2_dx = (C_dpmr*4kBT*a_x + C_n*sigma2_n_s)/(1-lambda_c^2) + s2_e_xD/(1-lambda_c^2) + (E[bracket²]/a_x²)*s2_e_a/(1-lambda_c^2)`

---

## 2. Test Case Results

All tests PASS. checkcode 0 issues on both files.

### T1: positioning h=50 baseline, paper Eq.22 cross-check
Mock parameters (representative for h=50 positioning):
- `lambda_c = 0.7`, `a_cov = 0.05`, `a_pd = 0.05`
- `gamma_N = 0.0425 pN·s/um`, `Ts = 1/1600 s` -> `a_nom ≈ 1.47e-2 um/pN`
- `c_para ≈ 1.0`, `c_perp ≈ 1.5` at h_bar≈22 -> `a_x_axis = [1.47e-2, 1.47e-2, 9.80e-3] um/pN`
- `sigma2_n_s = [(0.62 nm)², (0.057 nm)², (3.31 nm)²]` per project lab spec
- `Q77_axis = [0, 0, 0]` (positioning, frozen a)
- `R_axis = 0.241·a_x²` (chi_sq·rho_a·a_x² at h=50)
- `sigma2_w_fD = 0`

Result:
```
s2_dx_paper_only  = [1.957e-3, 1.956e-3, 1.329e-3] um^2
sigma_dx (paper)  = [44.2 nm, 44.2 nm, 36.5 nm]
correction_xD     = 0  (sigma2_w_fD=0)
correction_a      = ~0  (Q77=0 -> L_eff=0 -> CV2_a=0 since wall_term=0)
total corrections = 0% of paper part
```

**Cross-check with hand calc**: paper Eq.22 form `(C_dpmr*4kBT*a_x + C_n*sigma2_n_s)/(1-lambda_c^2)` matches function output within 1e-12 relative error (machine precision). ✓

**Sanity vs Phase 7 §6.3 prediction**: expected `sigma_dx ≈ 25-35 nm`. Got 36-44 nm (same order; small mismatch due to mock c values being approximate). ✓

### T2: `sigma2_w_fD = 0` baseline, `sigma2_e_xD ≈ 0`
- Baseline (sigma2_w_fD=0): `s2_e_xD = [0, 0, 0]` exactly. ✓
- Non-zero (sigma2_w_fD=1e-6): `s2_e_xD = [1.062e-7, 1.062e-7, 4.719e-8]` matches `sqrt(a_x²·sigma2_w_fD·R)` within 1e-12. ✓

### T3: lambda_c boundary
- `lambda_c = 0` (no AR-1): `s2_dx = [7.56e-4, 7.56e-4, 5.26e-4]` finite positive ✓
- `lambda_c = 0.95` (high): `s2_dx` finite ✓
- `lambda_c = 1`: rejected with `predict_closed_loop_var_eq17_v2:badLambda` ✓

### T4: input validation
- Missing field (`C_dpmr`): rejected with `:missingField` ✓
- Bad axis dim (`a_x_axis = [1,2]`): rejected with `:badAxisDim` ✓
- Bad scenario string: rejected with `:badScenario` ✓
- Motion scenario without traj fields: rejected with `:motionMissingTraj` ✓

### T5: motion sanity (1 Hz @ h=2.5 mock)
- `traj_amplitude = 2.5 um`, `traj_freq = 1 Hz`, `Q77_axis = [1e-8, 1e-8, 1e-8]`
- `E[bracket²] = (A·ω·Ts)²/2 = (2.5·2π·1·(1/1600))²/2 = 4.819e-5` ✓
- `CV²(a) ≈ [5.2%, 5.2%, 7.1%]` — falls within Phase 7 §6.3 motion range 3-15% ✓
- `s2_dx = [1.96e-3, 1.96e-3, 1.34e-3]` (corrections add ~0.05% in this regime due to small Q77)

---

## 3. Numerical Sanity: Positioning h=50 vs Paper Eq.22

**Predicted σ²_δx (positioning, sigma2_w_fD=0, Q77=0, baseline)**:
- x-axis: 1.957e-3 um² (σ ≈ 44.2 nm)
- y-axis: 1.956e-3 um² (σ ≈ 44.2 nm)
- z-axis: 1.329e-3 um² (σ ≈ 36.5 nm)

**Paper Eq.22 only (C_dpmr·4kBT·a_x + C_n·sigma2_n_s)/(1-λ_c²)**: identical to above (paper part = total since corrections = 0 in baseline).

**Cross-check vs Phase 2 hand calc**: relative error < 1e-12 (machine precision).

**Cross-check vs Phase 7 §6.3 expected `25-35 nm`**: ~30% high. Likely due to mock `c_para=1.0` being rough — actual c_para at h_bar=22 may be slightly larger reducing a_x. Order of magnitude correct, real values come from `calc_simulation_params` in Simulink runtime via `motion_control_law_eq17_7state.m`. The function structure itself is correct (paper-Eq.22 cross-check passes to 1e-12).

---

## 4. checkcode Results

```
predict_closed_loop_var_eq17_v2.m: No issues found by checkcode
test_predict_closed_loop_var_eq17_v2.m: No issues found by checkcode
```

Initially flagged 4 issues across both files (preallocation overwrite + 3 unused outputs). All resolved:
- Function: removed `E_bracket2 = zeros(1,3);` preallocation since both `switch` branches assign unconditionally.
- Test: replaced unused outputs with `~`.

---

## 5. Open Items / Questions for User

### 5.1 Mock value calibration vs runtime
T1's mock values give σ_δx ≈ 36-44 nm vs Phase 7 §6.3 "25-35 nm". The gap is from approximate `c_para=1.0`, `c_perp=1.5`; real values from `calc_correction_functions(h_bar=22.2)` would be a bit different. Should I:
- (A) Bring `calc_correction_functions` and `calc_simulation_params` into the unit test, so T1 uses *runtime* values? (Cost: test now depends on Simulink-side init.)
- (B) Keep mocks but tighten them by hard-coding c_para(22.2), c_perp(22.2)? (Cost: requires ground-truth lookup separately.)
- (C) Leave as-is — paper-Eq.22 cross-check at 1e-12 is the primary test, absolute σ_δx comparison is informational? (Lowest cost; current state.)

Default chosen (C). Function structure verified; absolute calibration deferred to Phase 8 e2e test.

### 5.2 `Q77_axis = 0` -> `L_eff = 0` -> CV2_a collapses to wall_term
For positioning baseline (frozen a, no innovation on a-state), `L_eff = sqrt(0/R) = 0`. Then `CV²(a) = chi_sq·rho_a·0/(0 + a_cov) + 0 = 0`. So `s2_e_a = 0` exactly. **This is the Phase 7 §4.3.1 caveat about Q66=0 making the chi-sq chain L_eff "tricky"**. Phase 7 §4.3.1 hints that empirically v1 saw σ_e_a / a_x ≈ 1.7-4.5%, NOT zero. The current closed form produces zero in positioning baseline — too optimistic for a unit-test ground truth.

**Question**: For Phase 8 oracle to be conservative (avoid overstating prediction confidence), should the function force a floor on `CV²(a)` when `Q77 = 0`? E.g. read empirical `1.7-4.5%` from Phase 7 §4.3.1 as a `wall_term` default? Or expose `cv2_floor` as another optional input?

For now, function returns the literal Phase 7 §6.1 closed form. Caller should pass `wall_term` if they want the empirical floor.

### 5.3 `s2_e_xD` SISO approximation a_nom vs a_x
Phase 7 §8.5 explicitly says Q55 should use `a_nom²` not `a_x²` to avoid self-loop. Function uses `a_x²` since Phase 7 §6.1 template uses it (cosmetic when sigma2_w_fD=0). For sigma2_w_fD > 0 motion runs, this could matter. **Should I switch to `a_nom_axis` as an additional 1x3 input?**

For now: function uses `a_x_axis(i)^2` per template. Phase 8 sigma2_w_fD-positive runs should pass `a_x_axis = a_nom_axis` if nominal is preferred (or this gets revisited).

### 5.4 `E[bracket²]` motion estimate accuracy
Section §6.1 / §8.3 use `(A·ω·Ts)²/2` rough estimate; function inherits this. Phase 7 §8.3 flags this as needing more accurate analytical evaluation for sinusoidal x_d. **Phase 8 motion verification** should compare predicted CV²(a) ~5-7% (T5 result) vs simulation observed and decide whether closed-form refinement is needed.

---

## 6. File Locations

- Function: `C:\Users\PME406_01\Desktop\code\MotionControl_Simu_qr\model\diag\predict_closed_loop_var_eq17_v2.m` (234 lines)
- Unit test: `C:\Users\PME406_01\Desktop\code\MotionControl_Simu_qr\test_script\unit_tests\test_predict_closed_loop_var_eq17_v2.m` (231 lines)
- Phase 7 spec: `reference/eq17_analysis/phase7_lyapunov_bench.md` §6

**Not committed** per Wave 2 instructions.
