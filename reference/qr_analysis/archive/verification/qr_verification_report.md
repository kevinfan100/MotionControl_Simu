# Q/R Derivation Simulink Verification Report

**Generated**: 2026-04-17 20:19:35

## Scenario

- Controller: 7-state EKF
- Trajectory: h_init=20.0 -> h_bottom=2.5, amp=2.5 um, f=1.0 Hz, 3 cycles
- Measurement noise: ON (std=[0.01;0.01;0.01] um)
- Thermal: ON
- Duration: 4.80 s (warmup 0.20 s excluded from stats)

## Q/R variants compared

- **Empirical (pre-derivation)**: Q = [0 0 1e+04 0.1 0 0.0001 0 ], R = [0.01 1 ]
- **Derived (backward-diff beta)**: Q = [0 0 1 0 0 1.344e-11 1.344e-11 ], R = [0.397 1.719 ]
- **Derived (B'-2 white-noise equiv)**: Q = [0 0 1 0 0 0.0001302 2.437e-09 ], R = [0.397 1.719 ]
- **B' Q + empirical R**: Q = [0 0 1 0 0 0.0001302 2.437e-09 ], R = [0.01 1 ]
- **Empirical Q + derived R**: Q = [0 0 1e+04 0.1 0 0.0001 0 ], R = [0.397 1.719 ]

## Results

| Metric | empirical | derived_beta | derived_Bprime | Bprime_Remp | Qemp_Rderived |
|---|---|---|---|---|---|
| 3D RMSE (nm) | 57.410 | 65.371 | 62.216 | 53.375 | 58.589 |
| 3D max (nm) | 456.139 | 221.482 | 483.634 | 231.991 | 536.457 |
| X RMSE (nm) | 33.951 | 33.915 | 32.995 | 32.702 | 32.844 |
| Y RMSE (nm) | 31.676 | 33.076 | 37.052 | 31.412 | 32.784 |
| Z RMSE (nm) | 33.762 | 45.046 | 37.541 | 28.155 | 35.765 |
| a_hat_x median rel %% | 15.120 | 15.589 | 16.528 | 15.383 | 11.661 |
| a_hat_z median rel %% | 21.690 | 39.058 | 14.081 | 14.587 | 26.564 |
| a_hat_x track ratio | 1.962 | 1.322 | 2.359 | 2.472 | 2.102 |
| a_hat_z track ratio | 4.699 | 1.431 | 2.168 | 1.081 | 3.736 |
| NaN count | 0 | 0 | 0 | 0 | 0 |

### Acceptance gates (each derived vs empirical)

**Derived (backward-diff beta)**:
- G1 (3D RMSE <= 1.25x empirical): PASS (65.37 <= 71.76 nm)
- G2 (a_hat_z median rel err < 10%): FAIL (39.06%)
- G3 (a_hat_z not frozen, track ratio > 0.5): PASS (1.431)

**Derived (B'-2 white-noise equiv)**:
- G1 (3D RMSE <= 1.25x empirical): PASS (62.22 <= 71.76 nm)
- G2 (a_hat_z median rel err < 10%): FAIL (14.08%)
- G3 (a_hat_z not frozen, track ratio > 0.5): PASS (2.168)

**B' Q + empirical R**:
- G1 (3D RMSE <= 1.25x empirical): PASS (53.37 <= 71.76 nm)
- G2 (a_hat_z median rel err < 10%): FAIL (14.59%)
- G3 (a_hat_z not frozen, track ratio > 0.5): PASS (1.081)

**Empirical Q + derived R**:
- G1 (3D RMSE <= 1.25x empirical): PASS (58.59 <= 71.76 nm)
- G2 (a_hat_z median rel err < 10%): FAIL (26.56%)
- G3 (a_hat_z not frozen, track ratio > 0.5): PASS (3.736)

