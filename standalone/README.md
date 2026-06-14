# 6-state Vpersonal EKF Controller — Standalone Package

> **DRAFT** — the structure, tables, and run instructions are final; the
> explanatory prose (marked ✎) is for the author to finalize in their own
> words before delivery.

A self-contained MATLAB implementation of the 6-state per-axis EKF
controller (RevisedControl_Vpersonal, paper-2023 Eq.17 + a-priori
predictor form) for ultra-precise positioning of a magnetic micro-probe
near a wall in aqueous solution. Extracted from the research repository
as a clean, reviewable unit; verified to reproduce the production
controller to floating-point rounding precision (see §6).

Requires **MATLAB R2025b**. No toolboxes (the DARE solver, IIR, and all
linear algebra are base MATLAB). Reset between runs is automatic.

---

## 1. What this simulates

✎ *One-paragraph plain statement of the physical problem and what the
controller achieves — rewrite in your own words.*

```
The question: can the 6-state EKF controller position a magnetic bead near
a wall (h_bar > 1.2) to ~30 nm precision, while simultaneously estimating
in real time the position-dependent mobility a_x(h)?

+---------------- physical world (continuous, ode4 10 us) -----------+
|  bead dynamics   p_dot = Gamma_inv(p) * (f_d + f_T)                 |
|  Gamma_inv = position-dependent mobility (wall effect c_para/c_perp)|
|  f_T = thermal white noise  (var = 4*kBT*gamma/Ts)                  |
+----------+-------------------------------------^-------------------+
           | p (true position)                   | f_d (control, ZOH)
           v                                     |
   [sensor: + measurement noise n_x, d=2 step delay]                 |
           | p_m                                 |
+----------v--------- controller (discrete 1600 Hz) ----+------------+
|  measurement chain:  delta_x_m -> IIR -> a_xm  (2nd "measurement"  |
|                                                 of the mobility)    |
|  control law:        Eq.17 ACTIVE form (raw delta_x_m feedback     |
|                      + Sigma f_d delay compensation)               |
|  estimator:          6-state EKF [dx1 dx2 dx3 xD_d a_x da_x],      |
|                      paper predictor form + Joseph (D6)            |
+-------------------------------------------------------------------+
```

---

## 2. How to run

`main_run.m` is a **script with a settings block at the top** — edit the
values and press Run (no command-line arguments):

```matlab
% --- in main_run.m, the SIMULATION SETTINGS block ---
scenario   = 'h50';   % 'h50' | 'h10' | 'ramp2p7'
seed       = [];      % [] = random (printed); or a fixed integer
T_sim      = [];      % [] = scenario default [sec]; or a number
lambda_c   = 0.7;     % closed-loop pole
a_pd       = 0.05;    % IIR mean-EWMA pole
a_cov      = 0.05;    % IIR variance-EWMA pole
meas_noise = true;    % sensor noise on/off
thermal    = true;    % thermal force on/off
```

Running it prints a tracking summary, saves the two figures, leaves the
result struct `out` in the workspace, and writes `out` to a `.mat` file.
Scenario geometry (h_init / h_bottom) and the unexposed constants live in
`config.m`.

```matlab
verify_standalone        % all 3 scenarios; h50 carries the quantitative PASS gate
verify_standalone('h50') % single scenario

% per-layer verification gates (test/), two kinds:
addpath test
% (a) SELF-CONTAINED -- run anywhere:
gates_part4   % control-law closed-loop smoke
gates_part5   % EKF recursion (DARE, closed-loop h50, L2 envelope)
gates_part6   % Q/R construction (Q55 + C_dpmr/C_n ground truth)
% (b) EQUIVALENCE vs the production controller -- REQUIRE the research
%     repo on disk (model/...); their PASS is the fidelity record:
gates_part1   % physics layer bit-exact vs source functions
gates_part2   % open-loop full sim bit-exact vs production driver
gates_part3   % constants dual-source vs production builder
gates_part7   % closed-loop h50/h10 rounding-floor equiv vs production
```

`main_run` is "press run and see it work". `verify_standalone` is the
quantitative 3-scenario check. The gates are the per-layer verification
suite (§5–§6). **Gates 1/2/3/7 compare against the research repository
and only run with it present**; when this package is detached as its own
repo they become historical (their recorded PASS is the equivalence
evidence). Gates 4/5/6, `verify_standalone`, and `main_run` are fully
self-contained.

---

## 3. File map (reviewer reading order)

| Order | File | Role |
|---|---|---|
| 1 | `README.md` | this map |
| 2 | `config.m` | every parameter, one place (the only file you edit to change a scenario) |
| 3 | **`controller_6state.m`** | **the controller — one file, numbered steps [0]–[11]** |
| 4 | `sim/run_simulation.m` | time-stepping driver (dual-rate loop, d=2 sensor delay) |
| 5 | `physics/` | plant: `wall_corrections`, `gamma_inv`, `thermal_force`, `trajectory_ref` |
| 6 | `sim/step_dynamics.m` | continuous dynamics (ode4, 10 us inner step) |
| 7 | `test/` | `verify_standalone`, `make_figures`, `gates_part1..7` |

**To review the controller, you open one file: `controller_6state.m`.**
Read it top to bottom against the canonical spec (§4). Steps [0]–[11] are
the execution order; the per-step banners cite the spec section each
implements.

---

## 4. The controller in 11 steps

`controller_6state.m` is a per-axis (x, y, z independent) 6-state EKF in
the paper predictor form (D6). Per step k:

```
[0]  init (first call): offline constants C_dpmr/C_n/K_var/IF_abc/xi,
     wall-aware a_hat seed, DARE -> a-priori P_f0, IIR prefill
[1]  measurement: delta_x_m = p_d[k-2] - p_m;  IIR -> a_xm
[2]  control law: Eq.17 ACTIVE form (uses a-priori a_hat / xD_comb)
---- per-axis loop ----
[3]  innovation  e = y - H*x_hat         (y = [delta_x_m; a_xm])
[4]  R[k]        R11 = sigma2_nx;  R22 = K_var*IF_eff*(a_hat+xi)^2 + delay
[5]  gain        L = P_f*H' / (H*P_f*H' + R)
[6]  state       x+ = Phi_map(x) + L*e            (Phi, NOT F_e -- see warning 1)
[7]  covariance  Joseph form
[8]  F_e[k]      time-varying Row 3 from f_d history
[9]  Q[k]        Q33 three-component + Q55 closed form
[10] forecast    P_f+ = F_e*P*F_e' + Q
---- end loop ----
[11] buffer shifts + ekf_out = [a_hat_x; a_hat_y; a_hat_z; h_bar]
```

State (per axis): `x = [dx1 dx2 dx3 xD_d a_x da_x]` — three delayed
tracking errors (dx1 = delta_x[k-2] is the measured one), the combined
disturbance `delta_x_D^d`, the motion gain `a_x`, and its rate `da_x`.

The single most important structural choice is in step [6] (warning 1).

---

## 5. Three things that look like bugs but are not

A reviewer will likely flag these — they are deliberate and correct:

1. **Step [6] uses `Phi` (Row 3 = lambda_c only), not the full `F_e`.**
   The mean prediction drops the error-coupling terms (−1, −F_dx, dF_dx)
   that `F_e` keeps for the covariance. Reason: those terms multiply
   *estimation errors* (x_D − x̂_D, a − â, …), which are zero-mean — so
   the best mean prediction omits them, while the covariance must carry
   them. Using `F_e` for the mean is exactly what causes the 7-state
   a_hat bias; the deterministic-map predict (Phi) is what fixes it.
   ✎ *(spec §3)*

2. **R22 delay weights `{1,1}` differ from Q33 randgain weights `{4,1}`.**
   Deliberate asymmetry: the R22 delay models a linear measurement term
   `r_2 = n_a − Sigma delta_a_ram[k-i]`, while the Q33 randgain models a
   squared process accumulation. Different objects, different weights.
   ✎ *(spec §6)*

3. **`F_e(3,4) = -1`, not `-1.6`.** The `-1.6 = -(1+d(1-lambda_c))` value
   belongs to the 7-state coordinates (separate x_D, delta_x_D); this
   package pre-combines them into `delta_x_D^d`, in which the entry is
   d-independent `-1`. Do not "fix" it.
   ✎ *(spec §3 / build_F_e comment)*

---

## 6. Verification — what is proven, and the operating envelope

Every layer carries two kinds of gate (see §2 for which need the research repo):
- **equivalence** vs the production controller (fidelity);
- **theory** vs closed-form derivations (construction correctness).

Headline results:
- **Physics layer** bit-exact vs the source functions (`gates_part1`).
- **Open-loop full sim** bit-exact vs production (`gates_part2`, max|diff|=0).
- **Closed-loop h50 / h10** reproduce the production controller to
  **floating-point rounding precision** — ~3 ulps over an 8000-step run,
  9 orders of magnitude below the ~30 nm tracking precision
  (`gates_part7`, rel < 1e-12). It is **not** bit-exact: the package is a
  clean re-implementation (loop fusion, inlined constants, guards removed),
  and mathematically-identical refactors differ by ~1 ulp/step in IEEE-754
  association; the contractive closed loop does not amplify it.
- **Q55 / C_dpmr / C_n** validated against closed-form and ground-truth
  Monte-Carlo (`gates_part6`).

**Operating envelope: h_bar > 1.2, level L2.**

| Quantity | Claim | Where shown |
|---|---|---|
| Position tracking | meets spec for all h_bar > 1.2 (~30 nm) | verify_standalone, gates_part5 E2/E3, gates_part7 |
| a_hat (mobility est) | h50 (h_bar 22) bias ≲ 3% (5-seed mean 2.6%), rel-std ~2.5%; bias rises toward the wall (h10 ~5%). **The near-wall (1.2 < h_bar < 1.5) a_hat is a known structural limitation.** | verify_standalone, gates_part7 H3 |
| Out of scope (L3) | a_hat accurate throughout the near wall — research-level, not this package | — |

✎ *Note: the 2.6% h50 bias is the post-D6 figure (the paper-strict predictor form raised it ~1.8 pt above the pre-D6 ~1%, in exchange for fidelity to the paper — see DECISIONS.md). It is well inside the 5% PASS gate.*

This package removes the production controller's near-wall guards
(G1/G2/G3) for simplicity. The only place its behavior diverges from the
production controller is therefore the near-wall a_hat: `gates_part7` H3
reports it (a_hat_z observed to differ by ~38% only where h_bar < 1.5,
while tracking stays at 30 nm and the filter stays numerically stable;
H3 asserts the safe-segment equivalence, stability, and bounded tracking
— the 38% is the descriptive near-wall figure). Everywhere else the two
agree to rounding precision.

---

## 7. Notation

Symbols follow the canonical spec (`RevisedControl_Vpersonal`). Key terms:

| Concept | Symbol | Code |
|---|---|---|
| closed-loop pole | lambda_c | `lambda_c` |
| sampling interval | Delta t | `Ts` (1/1600 s) |
| sensor delay | d | `d` (= 2, hardcoded) |
| tracking error / measured | delta_x / delta_x_m | `delta_x_m` |
| motion gain / estimate | a_x / a_hat | `a_hat` |
| gain "measurement" | a_xm | `a_xm` |
| combined disturbance | delta_x_D^d | `xD_comb` (state slot 4) |
| residual variance const | C_dpmr (= 3.1610) | `C_dpmr` |
| sensor-noise const | C_n (= 1.1093) | `C_n` |
| tracking variance const | C_dx (= 2 + 1/(1-lambda_c^2) = 3.96) | `C_dx` (comments only; ≠ C_dpmr) |
| wall sensitivity | K_h, K_h' | `K_h_axis`, `derivs.K_h_*` |
| sensor noise variance | sigma2_n_x | `sigma2_nx` |

Note `C_dx` (raw tracking-error variance, 3.96) and `C_dpmr` (IIR-residual
variance, 3.16) are **different constants** — see `gates_part6` F2 vs F5.

---

## 8. Known issues / out of scope

- Near-wall (h_bar < 1.5) a_hat accuracy (L3) — needs the Phase-A
  near-wall model (mean-reverting gain / S cross-covariance), not in scope.
- Tilted wall (theta/phi ≠ 0): the thermal anisotropy formula is exact
  only for the default wall normal [0;0;1].
- d is hardcoded to 2 (the C_dpmr/C_n/IF constants are derived for d=2).
- Oscillation trajectory (paper Fig.10-style) is not in the scenario set.
