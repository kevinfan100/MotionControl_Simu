# Session 7 — IIR Pre-fill + Warmup Skip (Options P1→P6)

**Date**: 2026-04-19
**Branch**: `test/qr-paper-reference`
**Goal**: Root-cause-fix the IIR transient (Sessions 5/6 finding) so that
`frozen_correct` Q/R can be measured with shorter `t_warmup` and `T_sim`.

**Outcome**: **P6 succeeds**. P5 (theoretical per-axis IIR pre-fill) +
shortened `warmup_count` (320 → 2 steps) fully eliminates the residual
transient. Short window (T=15s, t_warmup=2s) now matches Session 6
long-window baseline (and slightly betters it on bias).

---

## 1. Problem recap

Session 6 confirmed `frozen_correct` Q/R as the optimal positioning
configuration but required `T_sim ≥ 30s` and `t_warmup ≥ 10s` to avoid
measurement artifact. Session 5 short-window had +6.4% bias at h=50.

### Session 6 root-cause trace (`diag_frozen_correct_h50.mat`)

`motion_control_law_7state.m` line 239: when `del_pmr_var(i) < var_threshold`,
`r_gain_i = 1e6` (a_m ignored). With `a_cov=0.005` the IIR EMA fills slowly;
gate opened at k≈1000 (0.625s) with chi-sq spike kicking a_hat. Frozen Q
made recovery slow.

---

## 2. Iteration sequence

| Option | Change | Result on frozen h=50 short-window bias |
|---|---|---|
| (baseline) | unmodified | n/a (Session 6 used long window) |
| **P1** | `del_pmr2_avg = 0.01 * sigma2_dXT` (10x threshold) | +5.7 ± 3.7% |
| **P4** | P1 + `a_m_k1 = a_hat` init + no-update during warmup | +5.6 ± 3.7% (identical to P1, REVERTED) |
| **P5** | per-axis theoretical: `Cdpmr*sigma2_dXT*ar + Cnp*sigma2_n` | +6.2 ± 3.8% (slightly worse than P1) |
| **P6** | P5 + `warmup_count = 2` (skip 320-step IIR-only warmup) | **+0.5 ± 1.6%** ← target met |

### Why each iteration worked / didn't work

**P1**: addresses gate transient (intended). Long window matches Session 6
exactly. But short window still bad because the IIR warmup phase corrupts the
filter state regardless of init.

**P4**: would have fixed "first innovation kick at warmup-end". Identical
results to P1 prove a_m_k1 is NOT the residual mechanism. REVERTED.

**P5**: theoretical per-axis init starts IIR at expected steady-state. But
during the 320-step no-control warmup, IIR EMA decays toward sensor-noise floor
regardless of init (init weight after 320 steps with a_cov=0.005 is
0.995^320 = 20%). So P5 ≈ P1 at warmup end. Slightly worse possibly because
P5's higher init makes the residual decay slightly larger.

**P6**: skips the no-control warmup entirely. Control + EKF run from k=3,
IIR sees closed-loop del_pmr distribution from the start, no warmup decay.
The pre-filled IIR (P5) starts close to true steady-state, and stays there
because the driving signal is correct.

---

## 3. Final P6 implementation

`motion_control_law_7state.m` lines 144-156 (P5 init):
```matlab
% P5: per-axis theoretical pre-fill
ar_init = a_hat / a_nom;   % 3x1, wall-aware
del_pmr2_avg = C_dpmr_eff_const * sigma2_deltaXT * ar_init ...
             + C_np_eff_const * sigma2_noise;
del_pmr2_avg = max(del_pmr2_avg, 0.01 * sigma2_deltaXT);  % floor at 10x threshold
```

`motion_control_law_7state.m` line 162 (P6 warmup skip):
```matlab
% P6: shortened from 0.2s (320 steps) to 2 steps minimum (delay buffer)
warmup_count = 2;
```

No changes to other persistent inits, EKF logic, or IIR formulas.

---

## 4. Verification matrix and final results

3 seeds (12345, 67890, 11111) per cell.

### 4.1 frozen_correct (final P6 result)
| group | scenario | bias % | std % | spike % | 3D RMSE nm |
|---|---|---|---|---|---|
| Session 6 (T=30, w=10, no fix) | h=2.5 | +1.4 ± 0.4 | 4.6 ± 0.1 | n/a | 35 |
| Session 6 | h=50 | +1.4 ± 0.6 | 1.3 ± 0.7 | n/a | 62 |
| **P6 (T=15, w=2)** | h=2.5 | **+0.3 ± 0.6** | **4.5 ± 0.2** | 939-961 | 35.7 |
| **P6 (T=15, w=2)** | h=50 | **+0.5 ± 1.6** | **1.7 ± 1.1** | **15-17** | 61.8 |

### 4.2 empirical (sanity check)
| group | scenario | bias % | std % | spike % |
|---|---|---|---|---|
| Session 6 | h=2.5 | +1.8 ± 1.2 | 20.9 ± 1.0 | n/a |
| Session 6 | h=50 | +0.1 ± 1.2 | 19.9 ± 1.3 | n/a |
| **P6 (T=15, w=2)** | h=2.5 | +0.07 ± 1.5 | 19.7 ± 1.7 | 939-961 |
| **P6 (T=15, w=2)** | h=50 | -1.7 ± 1.1 | 18.4 ± 1.8 | 57-91 |

empirical results within seed variability of Session 6 → P6 doesn't regress.
Spike for empirical h=50 dropped from 415-961% to 57-91% (P6 cleared
the transient even for empirical Q).

### 4.3 Spike at h=2.5 (~939%) is k=0 init artifact
For all variants/options, h=2.5 spike is 939-961%. Mechanism: at k=0
`ekf_out` returns `a_nom` (line 154 init block), but `a_true_z = a_nom/c_perp(1.11) ≈ a_nom/10`,
giving error ratio 10 = 1000%. Algorithm-independent, harmless (excluded by
`idx_ss = t >= t_warmup`). This was confirmed by it being identical across
Sessions 5/6/7 and all P1-P6 variants.

---

## 5. Key insights

1. **The original 320-step IIR warmup was harmful, not helpful.** Its purpose
   was "let IIR fill before EKF uses a_m" — but during this 320-step phase
   there is NO CONTROL, so del_pmr is dominated by sensor noise and slow
   thermal drift, NOT closed-loop residual statistics. The IIR EMA converges
   to a value much smaller than true closed-loop steady state. When EKF
   starts at k=321, a_m is biased low → frozen Q can't recover for seconds.
2. **Init value of `del_pmr2_avg` is largely irrelevant** with the original
   320-step warmup. The IIR EMA "forgets" any init in ~5τ = 1000 samples,
   so init weight at k=320 is only 20%.
3. **Skipping the warmup is safe** for positioning because:
   - Particle starts at p0 = pd[0] (positioning trajectory)
   - del_pm[k] = pd_k2 - p_m starts at 0
   - Control law f_d = (1/a_hat)·(del_pd + (1-lc)·del_p3_hat - d_hat) = 0 at k=3
   - System enters closed-loop smoothly from k=3
4. For motion scenarios (`trajectory_type='osc'`), pd changes per step. del_pd
   is non-zero from k=0. With warmup_count=2 the control engages immediately,
   which is what we want anyway. The IIR P5 pre-fill ensures del_pmr_var is
   sane from k=3.

---

## 6. Decision: keep P6, install as default

P6 = P5 IIR pre-fill + warmup_count=2 is the production fix. It:
- Reproduces Session 6 quality in 1/2 the simulation time (T=15s vs 30s)
- Cuts t_warmup from 10s to 2s
- Eliminates the residual transient at h=50 (spike 137% → 17%)
- Doesn't regress empirical operation
- Should be backward-safe for motion scenarios (untested but reasoning above)

Phase 2: install `frozen_correct` Q/R as positioning preset (next step).

---

## 7. Files modified

- `model/controller/motion_control_law_7state.m` —
  - Lines 144-156: P5 per-axis IIR pre-fill (with floor)
  - Line 162: P6 warmup_count = 2 (was 320)
  - Lines 147-159, 197: P4 changes reverted (no benefit, reverted)
- `test_script/verify_qr_positioning_run.m` — added optional T_sim/t_warmup/variant_filter args.
- Test data (gitignored, regenerable):
  - `qr_p1_b{1..4}.mat` — Group A (P1, T=15, w=2)
  - `qr_p1_long_b{1,2}.mat` — Group B (P1, T=30, w=10) → matches Session 6
  - `qr_p1_w5_b{1,2}.mat` — Group C (P1, T=15, w=5) → bordeline
  - `qr_p4_*.mat` — Group D (P4 ineffective)
  - `qr_p5_*.mat` — Group E (P5 ineffective)
  - `qr_p6_*.mat` — **Group F (P6 SUCCESS)**

---

## 8. Pending

1. **Phase 2**: Install `frozen_correct` preset in `user_config.m` with switch
   `config.qr_preset = 'frozen_correct' | 'empirical' | 'beta'`.
2. **Optional motion verification**: verify P6 doesn't break osc scenarios.
3. **Optional**: rebuild lookup tables with Q(7,7)>0 (Q3 of original plan).
4. Thesis writeup of Sessions 4-7 results.
