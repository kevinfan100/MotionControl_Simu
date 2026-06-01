# Task Q/R Reference — Step (a) Results

**Date**: 2026-04-16
**Branch**: `test/qr-paper-reference`

## 1. Experiment setup

**Baseline**: `MotionControl_Simu/test_results/simulation/sim_20260413_133216`
- Scenario: static positioning at h_init = 2.5 um (near-wall, h_bar ~ 1.11)
- 30 sec, thermal ON, meas_noise OFF
- controller_type = 7 (7-state EKF), lambda_c = 0.7, beta = 0.5
- Q/R = code current: Q = [0,0,1e4,1e-1,0,1e-4,0], R = [1e-2, 1.0]

**Paper Q/R** (from `qr_theoretical_values.md`):
- Q = [0, 0, 1, 0, 0, 0, 0] (thermal only, no disturbance/gain slack)
- R(1,1) = 1e-8 (noise OFF, epsilon) or 0.397 (noise ON, = sigma2_n/sigma2_dXT)
- R(2,2) = 0.176 (autocorrelation-corrected, Task 1d Layer 2)

**Lookup tables rebuilt** for paper Q/R:
- C_dpmr_eff(lc=0.7) = 3.8740 (was 3.9242, -1.3%)
- bias_factor(lc=0.7) = 0.9067 (was 0.9069, unchanged)

## 2. Results

| Metric | Baseline (tuning) | Run 1: paper Q/R, noise OFF | Run 2: paper Q/R, noise ON |
|---|---|---|---|
| Q scaling | [0,0,1e4,1e-1,0,1e-4,0] | [0,0,1,0,0,0,0] | [0,0,1,0,0,0,0] |
| R scaling | [1e-2, 1.0] | [1e-8, 0.176] | [0.397, 0.176] |
| meas_noise | OFF | OFF | ON (std=0.01 um) |
| **Stable?** | **Yes** | **Yes** | **Yes** |
| a_hat_x rel std | 22.83% | 21.56% | **2.49%** |
| a_hat_z rel std | 26.32% | 24.54% | **7.64%** |
| a_hat_x mean bias | ~0% | +8.31% | +1.97% |
| a_hat_z mean bias | ~0% | +6.37% | +9.66% |
| 3D RMSE | 31.9 nm | 35.4 nm | 47.4 nm |

## 3. Key findings

### 3.1 Paper Q/R does NOT diverge (contradicts Phase 3 expectation)

Phase 3 MC sweep (documented in `project_sigma_ratio_filter.md`) found
"Physical (Q33=1): diverges". But that test used code R (R22=1.0), not
paper R (R22=0.176).

The combination matters: paper R(2,2) = 0.176 makes the EKF trust
the a_m measurement channel 5.7x more than code R(2,2) = 1.0. This
compensates for the smaller Q(3,3) = 1 by providing continuous gain
information through the measurement channel, preventing the covariance
P from collapsing to zero.

**Implication**: the Phase 3 "physical Q diverges" was a **joint Q+R effect**,
not a Q-alone effect. When R is also set to paper theory, the system
is stable — at least in this static near-wall scenario.

### 3.2 Run 1 (noise OFF): marginal improvement in a_hat spread

a_hat spread is ~2 pp better than baseline (24.5% vs 26.3% for z).
This is because R(2,2) = 0.176 gives the EKF more trust in a_m than
R(2,2) = 1.0, so the EKF can use the gain measurement channel more
effectively.

However, mean bias increases to +6-8% (baseline was ~0%). The bias comes
from Q(4,4) = Q(6,6) = 0: without disturbance and gain random-walk
slack, the EKF cannot correct for model mismatch, so systematic errors
accumulate as positive bias.

3D RMSE is slightly worse (35.4 vs 31.9 nm, +11%). The bias in a_hat
propagates to control force through f_d = (1/a_hat) * (...), causing
a systematic tracking offset.

### 3.3 Run 2 (noise ON): dramatic a_hat improvement

With noise ON and proper R(1,1) = 0.397, a_hat precision improves
dramatically: x-axis 2.49% (was 22.83%), z-axis 7.64% (was 26.32%).
This is a 3-9x improvement.

The mechanism: with R(1,1) = 0.397 (physically correct sensor noise
variance), the EKF properly weights the position channel. Combined
with small R(2,2) = 0.176, both measurement channels contribute
meaningfully to the estimation. The EKF is operating as designed:
R matches the actual noise, Q matches the actual thermal excitation.

However, 3D RMSE degrades to 47.4 nm (+49%) because sensor noise
directly adds to tracking error. The position measurement is noisier,
so the controller tracks less precisely even though it estimates a
more accurately.

### 3.4 Implications for writeup Section 3

The experiment confirms:

1. **Paper Q/R is self-consistent and stable** in this scenario.
   The theoretical values derived in Step (b) are not just mathematically
   correct but operationally viable.

2. **The current code's Q/R inflation trades a_hat precision for
   tracking performance**. The baseline achieves better tracking (31.9 nm)
   at the cost of worse a_hat (26%). Paper Q/R achieves better a_hat
   (7.6%) at the cost of worse tracking (47.4 nm).

3. **R(2,2) matters more than Q(3,3) for stability**. Phase 3's
   "physical Q diverges" was actually "physical Q + code R diverges".
   Paper R(2,2) = 0.176 stabilizes the system by trusting a_m more.

4. **Mean bias (+6-10%) is the cost of Q(4,4) = Q(6,6) = 0**. Without
   disturbance/gain random-walk, the EKF accumulates model mismatch.
   This is the physical reason the code uses nonzero Q(4,4) and Q(6,6).

## 4. Caveats

- Baseline used beta = 0.5 (old value before commit b271cee). Both
  paper Q/R runs also used beta = 0.5 for fair comparison.
- This is a **static** scenario (hold at h_init = 2.5 um). Dynamic
  near-wall scenarios (descent + oscillation) may diverge due to
  Q(6,6) = 0 preventing a_hat from tracking the changing a(h_bar).
- Thermal seed is different between baseline and paper Q/R runs
  (both use random seeds). Single-run comparison, not Monte Carlo.

## 5. Files

- `test_results/verify/paper_qr_paper_qr_noiseOFF.mat` (Run 1)
- `test_results/verify/paper_qr_paper_qr_noiseON.mat` (Run 2)
- `test_results/verify/cdpmr_eff_lookup.mat` (rebuilt for paper Q/R)
- `test_results/verify/bias_factor_lookup.mat` (rebuilt for paper Q/R)
