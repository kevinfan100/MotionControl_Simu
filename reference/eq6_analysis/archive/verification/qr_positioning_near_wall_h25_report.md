# Q/R Verification — near_wall_h25 (h_init = 2.50 um)

**Generated**: 2026-04-18 15:57:00

## Setup

- Trajectory: positioning (static hold at h_init = 2.50 um)
- T_sim = 30 s, t_warmup = 10 s (steady-state window 20 s)
- Noise ON, thermal ON, lc=0.7, controller_type=7 (7-state EKF)
- Seeds (5): [12345 67890 11111]

## Tracking error per axis (nm) — mean and std across 5 seeds

| variant | mean_x | mean_y | mean_z | std_x | std_y | std_z | 3D RMSE |
|---|---|---|---|---|---|---|---|
| empirical | -0.01 ± 0.04 | -0.01 ± 0.13 | +0.04 ± 0.05 | 25.73 ± 0.45 | 25.99 ± 0.24 | 15.80 ± 0.24 | 39.84 ± 0.30 |
| beta | +0.87 ± 0.20 | -0.38 ± 0.45 | +0.13 ± 0.46 | 25.84 ± 0.19 | 26.42 ± 0.31 | 15.46 ± 0.17 | 40.07 ± 0.25 |
| Bprime | -0.35 ± 0.00 | +0.36 ± 0.00 | +0.81 ± 0.00 | 25.70 ± 0.00 | 26.31 ± 0.00 | 16.21 ± 0.00 | 40.20 ± 0.00 |
| Bprime_Remp | +0.09 ± 0.61 | +0.19 ± 0.43 | +0.34 ± 0.65 | 25.91 ± 0.27 | 26.61 ± 0.31 | 16.04 ± 0.15 | 40.47 ± 0.31 |
| Qemp_Rderived | +0.02 ± 0.02 | -0.06 ± 0.19 | -0.02 ± 0.04 | 25.62 ± 0.46 | 26.03 ± 0.44 | 15.68 ± 0.22 | 39.75 ± 0.35 |

## Tracking std vs theory (del_pmr ratio)

| variant | empirical std_z | theory std_z | ratio_z |
|---|---|---|---|
| empirical | 15.80 ± 0.24 nm | 14.62 nm | 1.081 ± 0.016 |
| beta | 15.46 ± 0.17 nm | 14.62 nm | 1.058 ± 0.012 |
| Bprime | 16.21 ± 0.00 nm | 14.62 nm | 1.109 ± 0.000 |
| Bprime_Remp | 16.04 ± 0.15 nm | 14.62 nm | 1.098 ± 0.010 |
| Qemp_Rderived | 15.68 ± 0.22 nm | 14.62 nm | 1.073 ± 0.015 |

## a_hat error in % (relative to a_true)

| variant | bias_x % | std_x % | bias_z % | std_z % | max_x % | max_z % |
|---|---|---|---|---|---|---|
| empirical | +1.24 ± 3.53 | 19.88 ± 1.54 | +15.12 ± 3.68 | 26.63 ± 1.74 | 77.3 ± 17.2 | 123.6 ± 12.2 |
| beta | -0.76 ± 0.67 | 9.53 ± 0.27 | +4.41 ± 2.78 | 5.72 ± 0.21 | 29.5 ± 2.3 | 34.9 ± 10.7 |
| Bprime | +2.93 ± 0.00 | 19.60 ± 0.00 | +14.69 ± 0.00 | 30.61 ± 0.00 | 69.9 ± 0.0 | 120.3 ± 0.0 |
| Bprime_Remp | +1.75 ± 0.91 | 21.30 ± 0.73 | +15.31 ± 0.97 | 29.50 ± 0.60 | 78.3 ± 0.5 | 118.0 ± 1.6 |
| Qemp_Rderived | +0.06 ± 4.32 | 16.94 ± 2.08 | +12.76 ± 3.53 | 23.70 ± 1.70 | 70.0 ± 14.8 | 109.7 ± 10.4 |

## Spike (max |error| in first 1000 samples)

| variant | spike_x % | spike_z % | spike_z time [s] |
|---|---|---|---|
| empirical | 247 ± 196 | 972 ± 37 | 0.000 ± 0.000 |
| beta | 275 ± 251 | 903 ± 56 | 0.000 ± 0.000 |
| Bprime | 133 ± 0 | 989 ± 0 | 0.000 ± 0.000 |
| Bprime_Remp | 337 ± 288 | 959 ± 42 | 0.000 ± 0.000 |
| Qemp_Rderived | 134 ± 3 | 1134 ± 383 | 0.072 ± 0.124 |

## Per-seed details (a_hat_z bias and 3D RMSE)

| variant | seed→ | 12345 | 67890 | 11111 |
|---|---|---|---|---|
| empirical | bias_z % | +11.4 | +18.7 | +15.3 |
| | RMSE_3D nm | 39.9 | 39.5 | 40.1 |
| beta | bias_z % | +7.6 | +3.1 | +2.5 |
| | RMSE_3D nm | 39.8 | 40.1 | 40.3 |
| Bprime | bias_z % | +14.7 | - | - |
| | RMSE_3D nm | 40.2 | - | - |
| Bprime_Remp | bias_z % | - | +16.0 | +14.6 |
| | RMSE_3D nm | - | 40.7 | 40.2 |
| Qemp_Rderived | bias_z % | +14.7 | +14.9 | +8.7 |
| | RMSE_3D nm | 40.0 | 39.4 | 39.9 |
