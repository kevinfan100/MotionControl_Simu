# Q/R Verification — free_space_h50 (h_init = 50.00 um)

**Generated**: 2026-04-18 15:57:00

## Setup

- Trajectory: positioning (static hold at h_init = 50.00 um)
- T_sim = 30 s, t_warmup = 10 s (steady-state window 20 s)
- Noise ON, thermal ON, lc=0.7, controller_type=7 (7-state EKF)
- Seeds (5): [12345 67890 11111]

## Tracking error per axis (nm) — mean and std across 5 seeds

| variant | mean_x | mean_y | mean_z | std_x | std_y | std_z | 3D RMSE |
|---|---|---|---|---|---|---|---|
| empirical | -0.02 ± 0.05 | -0.01 ± 0.18 | +0.11 ± 0.10 | 36.45 ± 0.73 | 36.98 ± 0.33 | 36.68 ± 0.58 | 63.58 ± 0.50 |
| beta | +1.27 ± 0.30 | -0.58 ± 0.68 | +0.39 ± 1.48 | 36.83 ± 0.45 | 37.80 ± 0.59 | 36.85 ± 0.78 | 64.40 ± 0.19 |
| Bprime | +0.30 ± 0.99 | -0.45 ± 1.04 | +1.07 ± 1.37 | 37.01 ± 0.76 | 37.61 ± 0.16 | 37.49 ± 0.27 | 64.76 ± 0.58 |
| Bprime_Remp | +0.63 ± 1.01 | +0.28 ± 0.61 | +0.50 ± 1.38 | 36.97 ± 0.43 | 38.35 ± 0.51 | 36.86 ± 0.72 | 64.81 ± 0.19 |
| Qemp_Rderived | +0.03 ± 0.02 | -0.09 ± 0.28 | -0.06 ± 0.14 | 36.31 ± 0.69 | 36.98 ± 0.72 | 36.26 ± 0.88 | 63.26 ± 0.67 |

## Tracking std vs theory (del_pmr ratio)

| variant | empirical std_z | theory std_z | ratio_z |
|---|---|---|---|
| empirical | 36.68 ± 0.58 nm | 32.76 nm | 1.119 ± 0.018 |
| beta | 36.85 ± 0.78 nm | 32.76 nm | 1.125 ± 0.024 |
| Bprime | 37.49 ± 0.27 nm | 32.76 nm | 1.144 ± 0.008 |
| Bprime_Remp | 36.86 ± 0.72 nm | 32.76 nm | 1.125 ± 0.022 |
| Qemp_Rderived | 36.26 ± 0.88 nm | 32.76 nm | 1.107 ± 0.027 |

## a_hat error in % (relative to a_true)

| variant | bias_x % | std_x % | bias_z % | std_z % | max_x % | max_z % |
|---|---|---|---|---|---|---|
| empirical | -1.00 ± 3.42 | 19.12 ± 1.39 | +1.36 ± 1.19 | 19.59 ± 1.50 | 70.3 ± 16.2 | 67.8 ± 10.4 |
| beta | -1.95 ± 1.01 | 9.83 ± 0.75 | +4.80 ± 5.56 | 1.69 ± 1.19 | 25.8 ± 4.2 | 8.5 ± 7.8 |
| Bprime | +1.09 ± 3.32 | 20.13 ± 1.57 | +2.76 ± 0.80 | 16.22 ± 0.81 | 75.8 ± 4.4 | 54.3 ± 7.2 |
| Bprime_Remp | -0.47 ± 0.74 | 22.18 ± 0.73 | +1.22 ± 3.05 | 18.97 ± 0.92 | 78.1 ± 3.5 | 74.2 ± 6.2 |
| Qemp_Rderived | -2.01 ± 4.11 | 16.39 ± 1.89 | -0.66 ± 3.55 | 17.24 ± 0.94 | 61.8 ± 15.8 | 56.6 ± 9.8 |

## Spike (max |error| in first 1000 samples)

| variant | spike_x % | spike_z % | spike_z time [s] |
|---|---|---|---|
| empirical | 175 ± 246 | 290 ± 136 | 0.217 ± 0.004 |
| beta | 259 ± 208 | 194 ± 39 | 0.240 ± 0.007 |
| Bprime | 65 ± 59 | 263 ± 151 | 0.224 ± 0.008 |
| Bprime_Remp | 230 ± 230 | 222 ± 105 | 0.234 ± 0.017 |
| Qemp_Rderived | 96 ± 67 | 647 ± 425 | 0.218 ± 0.003 |

## Per-seed details (a_hat_z bias and 3D RMSE)

| variant | seed→ | 12345 | 67890 | 11111 |
|---|---|---|---|---|
| empirical | bias_z % | +0.0 | +2.3 | +1.8 |
| | RMSE_3D nm | 63.5 | 63.1 | 64.1 |
| beta | bias_z % | +11.0 | +3.0 | +0.4 |
| | RMSE_3D nm | 64.3 | 64.3 | 64.6 |
| Bprime | bias_z % | +3.1 | +1.8 | +3.3 |
| | RMSE_3D nm | 64.5 | 65.4 | 64.3 |
| Bprime_Remp | bias_z % | -2.2 | +2.3 | +3.6 |
| | RMSE_3D nm | 64.7 | 65.0 | 64.7 |
| Qemp_Rderived | bias_z % | +1.4 | +1.4 | -4.8 |
| | RMSE_3D nm | 64.0 | 62.7 | 63.1 |
