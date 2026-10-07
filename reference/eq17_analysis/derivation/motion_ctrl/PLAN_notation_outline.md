# motion_ctrl: notation and outline plan (draft for review, 2026-10-07)

Status: DRAFT, written by the Layer-1 session. User decisions recorded 10-07: notation of Section 2 approved,
lambda_c = 0.4 for every run in the document, Q/R depth B (Section 4). Nothing in the .tex parts has been changed yet.
Source of the paper wording: Meng, Long & Menq, IEEE TIE 72(1), 2025, Sections II-D to II-F (pp. 930-932),
read from the journal PDF in `~/Code/Thesis/`. Equation numbers (5)-(17) below are the journal's.

Writing rule for the document: use the paper's terms and phrasing (left column of Section 1). Do not use the
session's working vocabulary (slot, arm, window, ledger, walk-back, ...).

---

## 1. Terms (paper wording)

| Paper term | Meaning | Paper location |
|---|---|---|
| motion gain a_x[k] = Δt γ_x⁻¹[k] | displacement per unit force per sampling period | (5), p. 931 |
| drag coefficient γ; nominal value γ_N = 6πηR | Stokes' law far from the wall | II-D, p. 930 |
| correction functions Γ⊥(h/R), Γ∥(h/R) | wall effect on the drag coefficient; known for a rigid plane, no analytical formula for curved surfaces or living cells | II-D, p. 930 |
| desired force f_dx[k] | output of the motion control law | (5), (6) |
| thermal force f_T, variance σ²_fT = 4k_BTγ/Δt | random thermal forces | II-E |
| disturbed motion x_D[k] | modeling error and disturbance | (5) |
| tracking error δx[k] = x_d[k] − x[k]; control objective δx[k+1] = λ_c δx[k] | | (6), (7) |
| measured tracking error δx_m[k] | δx with the two-step measurement delay and measurement noise n_x | II-E |
| deterministic / random component of the measured tracking error | separated by a single-pole IIR filter | (9), (10) |
| measured input gain a_xm[k] | second process output, from the variance of the random component | (12), (13) |
| measurement y[k] = [δx_m[k], a_xm[k]]ᵀ | | II-E |
| augmented estimator | Kalman estimator of tracking error, disturbed motion and motion gain | (14)-(17) |
| output matrix H; state transition matrix of the error dynamics F_e[k]; estimator feedback matrix L[k] | | (15)-(17) |
| forecast of the error covariance matrix P^f[k]; process noise q[k], Q[k]; measurement noise R[k] | | (18)-(21) |
| "the estimated motion gain has a delay of two steps" | the paper's own statement about H | p. 932 |

## 2. Notation (approved 10-07)

Normalized variables throughout: lengths divided by the probe radius R, motion gains divided by a_nom, forces
multiplied by a_o. The wall surface is the origin, so no wall-position symbol appears. Position keeps the letter x.

Sampling and delay

| Symbol | Definition |
|---|---|
| k, Δt | sampling index; sampling period (1/1600 s) |
| d | measurement delay in sampling periods (d = 2) |

Position and tracking error

| Symbol | Definition | Paper |
|---|---|---|
| x̄[k] | normalized position x/R; distance of the probe center from the wall surface | x, h/R |
| x̄_d[k], Δx̄_d[k] | desired motion; its increment x̄_d[k+1] − x̄_d[k] | x_d |
| δx̄[k] | tracking error x̄_d − x̄ | δx |
| δx̄₁, δx̄₂, δx̄₃ | δx̄[k−2], δx̄[k−1], δx̄[k] | δx₁, δx₂, δx₃ |
| δx̄_m[k] | measured tracking error | δx_m |
| ĥ[k] | estimated distance to the wall, x̄_d − δx̂̄₃ | — |

Motion gain and force

| Symbol | Definition | Paper |
|---|---|---|
| a_nom | nominal motion gain Δt/γ_N [µm/pN], 1.471e-2 | Δt/γ_N |
| a_o | a_nom/R [1/pN], 6.54e-3; f_R = 1/a_o ≈ 153 pN | — |
| ā[k], â[k] | normalized motion gain a_x/a_nom, and its estimate | a_x, â_x |
| e_ā[k] | estimation error of the motion gain, ā − â | e_a |
| f̄_d[k] | normalized desired force a_o·f_dx | f_dx |
| λ_c | closed-loop pole of the tracking error, 0.4 | λ_c |

Position-dependent motion gain (extension)

| Symbol | Definition |
|---|---|
| ā′(ĥ; c) | slope of the motion gain with respect to x̄, (1 − c)/ĥ² |
| c | slope parameter, estimated as a state |
| M[k] | motion increment used by the estimator, Δx̄_d[k] + (1 − λ_c)·δx̂̄₃[k] |

Measured input gain and noise

| Symbol | Definition | Paper |
|---|---|---|
| ā_m[k] | measured input gain (normalized), from the variance of the random component of δx̄_m | a_xm |
| y₁, y₂ | the two process outputs: δx̄_m and the measured input gain | y = [δx_m, a_xm]ᵀ |
| n_x̄ | measurement noise, variance σ²_n | n_x, σ²_nx |
| ε_x[k] | thermal motion (process noise of the tracking error) | a_x f_T |

Estimator

| Symbol | Definition | Paper |
|---|---|---|
| F_e, H, L, P^f, P, Q, R | bold uppercase; matrix entries lowercase (p^f₅₅, ℓ₅₂, h₂₅, r₂, q₅₅) | same |
| e_y₁, e_y₂ | output errors y − ŷ (renamed from the paper's e_x1, e_ax to keep them apart from the estimation error e_ā) | e_x1, e_ax |
| ε | ratio of the process noise of the motion gain to its measurement noise, Q₄₄ = ε·R₂ (1e-5) | — |
| I[k] | information gain on c at step k, 1/p₅₅ − 1/p^f₅₅ | — |
| P_fix | constant variance of c; q₅₅ = P_fix²·I[k] | — |

With the measurement delay not modeled in H and in the predicted output, ∇_d x̄_d does not appear in the main text.

Units of the two normalizing gains (stated once in Section 1):

    physical      Δx  = a_x · f                 [µm] = [µm/pN]·[pN]
    divide by R   Δx̄ = (a_nom / R) · ā · f = a_o · ā · f
    force scaled  Δx̄ = ā · f̄,   f̄ = a_o f = f / f_R

Wording rule: use the paper's terms (motion gain, desired force, tracking error, measured tracking error, measured
input gain, disturbed motion, augmented estimator, output matrix, state transition matrix of the error dynamics,
estimator feedback matrix, forecast of the error covariance matrix, process noise, measurement noise). Do not use
working terms (slot, arm, walk-back, window, ledger, readout chain, honesty ratio).

## 3. Outline: the paper's logic, then the extension

Common to all sections: z axis (1-D motion, as the paper's (5)); the wall position is known (wall surface = origin);
no disturbed motion; same seeds; trajectories `ramp` (hold, constant-velocity descent, hold) and `ramp_tri`
(descent, then triangle revisits). Plane wall up to Section 5; the unknown wall type (cell) enters in Section 6.

| § | Title (paper wording) | Follows / extends | Content shown | Trajectory, wall |
|---|---|---|---|---|
| 1 | Discrete-time motion model and control law | paper II-D, (5)-(8) | normalized (5), law (6) without x_D, error dynamics (7)-(8): the term e_ā f̄_d is the only deterministic error source | — |
| 2 | Effect of the motion gain on tracking | paper Figs. 5, 8 (nominal) and thesis Ch. 3 (known gain) | â = 1 (nominal) vs â = ā (known): tracking error and its mean near the wall | ramp, plane |
| 3 | Measured input gain | paper II-E, (9)-(13) | chain δx̄_m -> IIR -> σ²_δxr -> ā_m -> y₂, one line each; constants named, not derived | — |
| 4 | Augmented estimator: motion gain as a state | paper II-F, (14)-(17), first-order version | state, F_e, H (paper's 2×n output matrix, delay not modeled), Q₄₄ = ε R₂, KF; result: the estimate lags during the approach | ramp, plane |
| 5 | Position-dependent motion gain | extension (paper p. 935-936: position-dependent drag must be estimated in real time) | ā[k+1] = ā[k] + ā′(ĥ; c)·M[k], c[k+1] = c[k], q₅₅ = 0; result: c is learned on the approach and then no longer updated | ramp, plane |
| 6 | Process noise of c | extension | q₅₅ = P_fix² I[k], derived from the KF lines; meaning of I[k]; result on plane and cell (estimator starts from the plane values) | ramp_tri, plane + cell |
| 7 | Scope of the one-constant model | extension | what the constant c represents; the estimator reaches the limit of one constant on the approach; next: a model of how c changes with height (F2) | both |

## 4. What is shown and what is not (Q/R depth B, approved 10-07)

Shown in the main text: model equations, control law, error dynamics, state/output equations, F_e and H,
the KF three lines, the two design choices (Q₄₄ = ε R₂; q₅₅ = P_fix² I), result figures.

Q and R, depth B (structure plus the two scalings that the argument uses):

    Q[k] = diag(0, 0, Q₃₃[k], Q₄₄[k], q₅₅[k]),   R[k] = diag(R₁, R₂[k])
    Q₃₃[k] ∝ κ_T ā[k]          the thermal motion grows with the motion gain
    R₂[k]  = K_var IF[k] (â[k] + ξ)²   (10-07: equality, not ∝; IF named only, whitening/d·Q₄₄ in comments)
    R₁ = σ²_n                   the variance of the measurement noise

One sentence per entry; the constants (κ_T, ξ) are named, not derived. These two lines support two statements in
the text: near the wall the measured input gain is relatively more accurate (R₂ shrinks with ā²), and the meaning
of the ratio ε = Q₄₄/R₂.

Update 10-07 (user): Section 3 writes C_dpmr, C_n in full (no numbers) plus the a_pd -> 0 limit; Section 4 (first
order) writes the complete estimator out (all state, F_e, H, Q, R entries the code uses), not depth B.

Not in the main text (appendix or code reference): the full Q₃₃/Q₃₄/Q₄₄ expressions, the R₂ chain (K_var, IF,
whitening scale, echo factor), MA(2) and other filter details, observability gates, verification records.

## 5. Open decisions

1. DECIDED 10-07: λ_c = 0.4 for every run in the document (the paper's value; 0.4 vs 0.7 verified equal, ledger C91).
   All figures in the document are to be rerun at 0.4.
2. H in Sections 4-6: delay not modeled (paper's H). Tested 10-07 (ledger C92): ignoring the delay in H and in the
   predicted output changes e_ā by ≤ 0.0004 on ramp and ramp_tri; only the fastest, widest plane run moves (−0.0056).
   So the document can use the paper's H and ŷ₂ = â. Which version the figures use is still the user's call.
3. Reference runs with the true gain / true slope ("known" cases): main text or appendix.
4. Figure style: start from the single-seed style (feedback memory 10-05) and adjust while reviewing.

## 10-07 later decisions (user)

- Notation: only symbols the paper has (no x^f, e^f, n); estimator in the paper's (16)-(17) form; no initial-values block.
- Section 3 title = paper II-E title; Section 4 title names the first-order process.
- y2 = a_m as in the paper, delay ignored; estimator = noap first order with all refinements off ('plain');
  measured: with a_m, epsilon trades far-field noise vs near-wall lag (1e-5: 0.069 / -0.186 ... 1e-3: 0.190 / -0.043);
  the single-sample reading would give 0.074 / -0.093 (not used for now).
- Figures: no a_m line, no e_a row; Fig. 4.2 = three axes, Section 2 trajectory, a_hat = 1 vs estimate (layout of Fig. 2.2).

## 6. Typesetting conventions (checked against the paper's (9)-(21), 10-07)

What the paper does (read at 300 dpi from pages 931-932):

| Kind | Paper | Example in the paper |
|---|---|---|
| matrix | bold upright capital | L[k], P[k], P^f[k], Q[k], R[k], H, F_e[k], I |
| superscript / subscript on a matrix | italic | P^f (f italic), F_e (e italic), transpose ^T italic |
| vector | bold lowercase | e[k], q[k] upright bold; y[k] bold italic (the paper itself is inconsistent here) |
| scalar, scalar entry of a matrix | italic, entries with two indices | l_11[k] (script l), e_{x1}[k], a_x[k], f_dx[k], lambda_c |
| subscript that is a symbol (axis, d, T, m, D, n) | italic | f_dx, f_T, a_xm, x_D, n_x, sigma_{dx_r} |
| subscript that is an abbreviation (a word) | upright | a_var |
| time index | square brackets | x[k], x[k+1] |
| IIR mean | long overline | overline(dx_m)[k] |
| estimate | hat | x_hat_1, a_hat_x |
| physical constants | italic | k_B (k_b in (12)-(13): the paper is inconsistent), T, gamma, Delta t |

Rules applied in motion_ctrl (10-07):

1. Matrices `\mathbf{}` upright bold capital: F_e, H, L, P, P^f, Q, R, I. Entries lowercase italic with two indices:
   l_ij (\ell), q_33, q_44, q_55, r_1, r_2, p_55, p^f_55.
2. Vectors `\mathbf{}` upright bold lowercase: e, q, y, y-hat (y kept upright to match e and q; the paper's bold-italic y is
   treated as a typo).
3. Scalars italic. Subscripts that are symbols stay italic: F_dx, f_d, f_T, kappa_T, a_o, C_n, sigma_n, eps_x, e_{x1},
   e_{a_m}, x_d, h, D_perp, gamma_N.
4. Subscripts that are abbreviations are upright (`\mathrm{}`), as the paper's a_var: a_pd, a_cov, K_var, a_nom,
   P_fix, plane, cell; the named factor IF is upright (\mathrm{IF}). Changed 10-07 (a_pd 14, a_cov 4, C_dpmr 8, K_var 2,
   a_nom 4, IF 1 occurrences).
5. Our additions to the paper (record, do not mix up):
   - bar = normalized (\bar: x-bar = x/R, a-bar = a/a_nom, f-bar = a_o f); long overline = IIR mean (the paper's). On
     dx_m both appear: overline(dx-bar_m).
   - one epsilon glyph (\varepsilon, the paper's; user 10-07 removed the lunate \epsilon): error term eps[k], eps_x[k]
     (time-indexed) vs the ratio eps = q44/r2 (a constant, no index).
   - I[k] (italic, scalar) = information gain on c; bold I = identity matrix.
   - prime = d/dh (a-bar', its estimate a-hat').
   - units upright: \mu\mathrm{m}, \mathrm{pN}.
6. Figures: axis labels in physical units (z [um], dz_m [um]) and normalized gain (a-bar_z); estimate dashed when it
   overlaps the truth; legend only, no title.

7. 10-07 later: C_dpmr renamed C_T in the document (user); pairs with C_n (T = thermal, n = sensor noise, both symbol
   subscripts, italic, like the paper's f_T and n_x). Code keeps C_dpmr. kappa_T = 4 k_B T a_o / R is the Greek kappa
   (one-step thermal variance at a-bar = 1, normalized), not the Latin k of k_B.
8. 10-07 later: Section 3 adds sigma^2_{dx-bar_T}[k] = kappa_T a-bar[k] (ties kappa_T to the paper's (11)); the a_pd -> 0 limit
   line and the second a_m line (with -xi) removed (user). Figures: only the tracking row is dashed (z_d blue dashed on top of
   red z); all other comparisons solid; red drawn first; red width 5.0, blue 1.8 (c(h) figure: two walls, equal width).
