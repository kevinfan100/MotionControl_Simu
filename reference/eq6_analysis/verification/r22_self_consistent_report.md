# R(2,2) Self-Consistent Fixed-Point Report

**Generated**: 2026-04-17 20:04:18

## Operating point

- lc = 0.700
- a_pd = 0.050
- a_cov = 0.050
- a_phys = a_nom = 1.4706e-02 um/pN (free-space)
- sigma2_n = 1.000e-04 um^2 (noise ON)
- sigma2_dXT = 2.5189e-04 um^2

## Q vector deployed

```
Qz_diag_scaling = [0; 0; 1.0000; 0; 0; 1.3444e-11; 1.3444e-11]
```

## Converged R(2,2)

- **R(2,2) scaling** (on sigma2_dXT): **1.718520**
- R(2,2) absolute: 4.3287e-04 um^2/pN^2
- Sigma_e(3,3) = Var(delta_x[k-2]) = 1.3266e-03 um^2
- V_meas = beta*C_dpmr_eff*Sigma_e = 4.8454e-03 um^2
- Converged: YES (15 iterations)

## Structural constants

- C_dpmr_paper = 2 + 2/(1-lc^2) = 5.9216
- C_dpmr_eff (closed-loop, augmented Lyapunov) = 4.0275
- chi_sq = 2*a_cov/(2-a_cov) = 0.0513
- rho_a (autocorr amplification) = 3.6986
- beta (finite-sample IIR bias) = 0.9069

## Convergence history

| iter | R22_scaling | Sigma_e(3,3) | V_meas | C_dpmr_eff | rel_change | R22_proposed |
|---|---|---|---|---|---|---|
| 1 | 0.176000 | 1.327e-03 | 4.845e-03 | 4.0275 | 8.76e+00 | 1.718520 |
| 2 | 0.947260 | 1.327e-03 | 4.845e-03 | 4.0275 | 8.14e-01 | 1.718520 |
| 3 | 1.332890 | 1.327e-03 | 4.845e-03 | 4.0275 | 2.89e-01 | 1.718520 |
| 4 | 1.525705 | 1.327e-03 | 4.845e-03 | 4.0275 | 1.26e-01 | 1.718520 |
| 5 | 1.622112 | 1.327e-03 | 4.845e-03 | 4.0275 | 5.94e-02 | 1.718520 |
| 6 | 1.670316 | 1.327e-03 | 4.845e-03 | 4.0275 | 2.89e-02 | 1.718520 |
| 7 | 1.694418 | 1.327e-03 | 4.845e-03 | 4.0275 | 1.42e-02 | 1.718520 |
| 8 | 1.706469 | 1.327e-03 | 4.845e-03 | 4.0275 | 7.06e-03 | 1.718520 |
| 9 | 1.712494 | 1.327e-03 | 4.845e-03 | 4.0275 | 3.52e-03 | 1.718520 |
| 10 | 1.715507 | 1.327e-03 | 4.845e-03 | 4.0275 | 1.76e-03 | 1.718520 |
| 11 | 1.717014 | 1.327e-03 | 4.845e-03 | 4.0275 | 8.77e-04 | 1.718520 |
| 12 | 1.717767 | 1.327e-03 | 4.845e-03 | 4.0275 | 4.38e-04 | 1.718520 |
| 13 | 1.718143 | 1.327e-03 | 4.845e-03 | 4.0275 | 2.19e-04 | 1.718520 |
| 14 | 1.718332 | 1.327e-03 | 4.845e-03 | 4.0275 | 1.10e-04 | 1.718520 |
| 15 | 1.718520 | 1.327e-03 | 4.845e-03 | 4.0275 | 5.48e-05 | 1.718520 |

## Sanity gates

- G3 (free-space +-30% of 0.176): **WARN** (ratio 9.764)
- G4 (convergence monotonic):     **PASS** (0 non-mono steps)
