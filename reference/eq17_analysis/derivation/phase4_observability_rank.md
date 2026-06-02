# Phase 4: Observability Rank Test (v2 F_e/H pair)

**Pre-Phase-4 commit**: `348a68b` (Phase 3 algebraic verification)
**Date**: 2026-04-29
**Status**: Phase 4 theoretical predictions + MATLAB test script. Awaiting user review + actual MATLAB execution.

This phase verifies that the v2 F_e/H pair preserves the observability properties of v1 (Task 01 documented). Specifically: rank=7 for Config B (dual feedback) under any f_d, including f_d=0 (PE-free identification of all 7 states).

---

## 1. Goals

1. Recap LTV observability theory for our F_e[k]/H pair
2. Identify how v2's F_e(3,4) = −1.6 (vs v1's −1) affects rank
3. Define test cases (Config A vs Config B, 5-state vs 7-state, f_d sweep)
4. Predict expected ranks from theory
5. Provide MATLAB test script for empirical rank verification
6. Document expected pass/fail for Phase 4 gate

---

## 2. Observability Theory

### 2.1 LTV observability matrix

For time-varying F_e[k] (only F_e(3,6) = −f_d[k] varies; rest constant), the N-step observability matrix:

```
W_N = [ H       ]
      [ H·F[1]   ]
      [ H·F[2]·F[1]]
      [ ...      ]
      [ H · Π_{i=1}^{N-1} F[i] ]
```

Rank test: rank(W_N) over a window of N consecutive steps.

Full observability: rank(W_N) = n_state (=7 for our 7-state).

### 2.2 PE-free property (Task 01 v1 result)

Task 01 (v1) verified for **simplified controller** (no Σf_d) with F_e(3,4) = −1:

| Configuration | Verified rank under f_d sequence |
|---|---|
| Config B (dual feedback, 7-state) | rank=7 for f_d = 0, 2, ramp 1..7 (3/3 PASS) |
| Config A (single feedback, 7-state) | rank=7 for some f_d sequences (PE required) |
| 5-state historical | rank=5 (10/10 PASS) |

**Key v1 finding**: Config B rank=7 holds **regardless of f_d sequence**, including f_d=0. This is because the dual feedback (y_1 = δx_m, y_2 = a_xm) provides direct observation of state slot 1 (δx_1) and state slot 6 with slot 7 coupling (a_x − d·δa_x).

### 2.3 v1 → v2 F_e diff and observability

v2 changes only F_e(3,4) from −1 to **−1.6** (= −(1+d(1−λ_c))).

**Rank impact analysis**:

The (3,4) entry coupling to x_D is non-zero in BOTH v1 and v2. The structural pattern of F_e (which entries are zero vs nonzero) is **identical** between v1 and v2:

```
F_e structural pattern (1 = nonzero, 0 = zero):
            slot 1  slot 2  slot 3  slot 4  slot 5  slot 6  slot 7
slot 1   [    0       1       0       0       0       0       0  ]
slot 2   [    0       0       1       0       0       0       0  ]
slot 3   [    0       0       1       1       0       1       0  ]   ← time-varying (3,6)
slot 4   [    0       0       0       1       1       0       0  ]
slot 5   [    0       0       0       0       1       0       0  ]
slot 6   [    0       0       0       0       0       1       1  ]
slot 7   [    0       0       0       0       0       0       1  ]
```

★ **Rank depends on this STRUCTURAL pattern, not specific entry values** (so long as nonzero entries remain nonzero).

→ v2 F_e (with (3,4) = −1.6) should preserve all v1 rank results.

**Predicted**: Phase 4 MATLAB rank test should reproduce Task 01 v1 results exactly.

---

## 3. Test Cases (Phase 4 Verification Matrix)

### 3.1 Configuration variants

| Config | y_1 | y_2 | Description |
|---|---|---|---|
| A (single feedback) | δx_m only | — | Original paper 2023 setup, raw delayed measurement only |
| B (dual feedback) | δx_m | a_xm | Paper 2025 setup, IIR-derived motion gain measurement added |

### 3.2 State vector variants

| Variant | Dim | State |
|---|---|---|
| 5-state (historical) | 5 | [δx_1, δx_2, δx_3, x_D, a_x] (1st-order RW for x_D, a_x) |
| 7-state (current v2) | 7 | [δx_1, δx_2, δx_3, x_D, δx_D, a_x, δa_x] (integrated RW) |

### 3.3 f_d sweep cases

Test rank under different f_d sequences (covers PE / no-PE / motion):

| Case | f_d sequence | Note |
|---|---|---|
| F1 | f_d[k] = 0 (full hold) | PE-free check, most stringent |
| F2 | f_d[k] = 2 (constant) | Constant input, weak PE |
| F3 | f_d[k] = k (ramp 1, 2, 3, ...) | Strong PE, polynomial |
| F4 | f_d[k] = sin(2π·1·k·Ts) | Oscillation, periodic PE |
| F5 | f_d[k] = randn() | Random sequence |

### 3.4 Test matrix (predicted)

| Config | State | F1 (f_d=0) | F2 (const) | F3 (ramp) | F4 (osc) | F5 (random) |
|---|---|---|---|---|---|---|
| **B** | 7 | rank=7 ✓ | rank=7 ✓ | rank=7 ✓ | rank=7 ✓ | rank=7 ✓ |
| B | 5 | rank=5 ✓ | rank=5 ✓ | rank=5 ✓ | rank=5 ✓ | rank=5 ✓ |
| A | 7 | rank≤6 (PE-dependent) | rank=7 likely | rank=7 ✓ | rank=7 ✓ | rank=7 ✓ |
| A | 5 | rank≤4 (PE) | rank=5 likely | rank=5 ✓ | rank=5 ✓ | rank=5 ✓ |

★ **Critical Phase 4 pass condition**: Config B 7-state rank=7 for ALL f_d sweep cases (PE-free).

---

## 4. MATLAB Test Script

### 4.1 Script content

Location: `test_script/unit_tests/test_observability_eq17_v2.m`

```matlab
function test_observability_eq17_v2()
% TEST_OBSERVABILITY_EQ17_V2 Rank test for v2 F_e/H pair
%
% Verifies that v2 (F_e(3,4) = -1.6) preserves Task 01 v1 rank results.
%
% Pre-Phase-4 baseline: Task 01 v1 result is rank=7 unconditional for
% Config B (dual feedback) 7-state EKF. v2 should match.

    % v2 parameters
    lambda_c = 0.7;
    d = 2;

    % F_e structural construction (Eq.19 form, v2)
    function F = build_Fe_v2(f_d_k)
        F = zeros(7, 7);
        % Row 1: shift register
        F(1, 2) = 1;
        % Row 2: shift register
        F(2, 3) = 1;
        % Row 3: closed-loop dynamics (v2)
        F(3, 3) = lambda_c;
        F(3, 4) = -(1 + d*(1 - lambda_c));   % v2: -1.6, NOT v1's -1
        F(3, 6) = -f_d_k;                    % only time-varying entry
        % Row 4: x_D RW
        F(4, 4) = 1;  F(4, 5) = 1;
        % Row 5: δx_D RW
        F(5, 5) = 1;
        % Row 6: a_x RW
        F(6, 6) = 1;  F(6, 7) = 1;
        % Row 7: δa_x RW
        F(7, 7) = 1;
    end

    % H matrix (unchanged from v1)
    function H = build_H(config)
        if strcmp(config, 'B')  % dual feedback
            H = zeros(2, 7);
            H(1, 1) = 1;             % y_1: δx_m → δx_1
            H(2, 6) = 1;             % y_2: a_xm → a_x
            H(2, 7) = -d;            % y_2: a_xm → -d·δa_x
        elseif strcmp(config, 'A')  % single feedback
            H = zeros(1, 7);
            H(1, 1) = 1;
        else
            error('Unknown config');
        end
    end

    % Build N-step observability matrix
    function W = build_observability(H_use, f_d_seq)
        N = length(f_d_seq);
        n_meas = size(H_use, 1);
        W = zeros(N * n_meas, 7);
        % First block: H
        W(1:n_meas, :) = H_use;
        % Subsequent blocks: H * Product of F[i]
        Phi = eye(7);
        for k = 1:N-1
            F_k = build_Fe_v2(f_d_seq(k));
            Phi = F_k * Phi;
            W((k*n_meas + 1):((k+1)*n_meas), :) = H_use * Phi;
        end
    end

    % Test cases
    N_window = 20;  % 20-step observability window
    cases = struct(...
        'name', {'F1: f_d=0', 'F2: f_d=2 const', 'F3: ramp', ...
                 'F4: osc', 'F5: random'}, ...
        'f_d_seq', {zeros(1, N_window), ...
                    2 * ones(1, N_window), ...
                    1:N_window, ...
                    sin(2*pi*1*((0:N_window-1)/1600)), ...
                    randn(1, N_window)} ...
    );

    fprintf('\n=== Phase 4 Observability Test (v2 F_e) ===\n');
    fprintf('lambda_c = %.2f, d = %d, N_window = %d\n\n', lambda_c, d, N_window);

    H_B = build_H('B');
    H_A = build_H('A');

    fprintf('Config B (dual feedback, 7-state):\n');
    for i = 1:length(cases)
        W = build_observability(H_B, cases(i).f_d_seq);
        r = rank(W);
        s = sprintf('  %s: rank = %d', cases(i).name, r);
        if r == 7
            s = [s, ' ✓ PASS'];
        else
            s = [s, ' ✗ FAIL (expected 7)'];
        end
        fprintf('%s\n', s);
    end

    fprintf('\nConfig A (single feedback, 7-state):\n');
    for i = 1:length(cases)
        W = build_observability(H_A, cases(i).f_d_seq);
        r = rank(W);
        if i == 1   % f_d=0, expect possible rank deficiency
            expected_msg = sprintf('(rank ≤ 6 acceptable for PE-required mode)');
        else
            expected_msg = '(expected 7)';
        end
        fprintf('  %s: rank = %d %s\n', cases(i).name, r, expected_msg);
    end

    fprintf('\n=== Test complete. ===\n');
    fprintf('Critical: Config B 7-state must show rank=7 for all 5 cases (PE-free).\n');
end
```

### 4.2 Expected output (predicted)

```
=== Phase 4 Observability Test (v2 F_e) ===
lambda_c = 0.70, d = 2, N_window = 20

Config B (dual feedback, 7-state):
  F1: f_d=0:           rank = 7 ✓ PASS
  F2: f_d=2 const:     rank = 7 ✓ PASS
  F3: ramp:            rank = 7 ✓ PASS
  F4: osc:             rank = 7 ✓ PASS
  F5: random:          rank = 7 ✓ PASS

Config A (single feedback, 7-state):
  F1: f_d=0:           rank = 6 (rank ≤ 6 acceptable for PE-required mode)
  F2: f_d=2 const:     rank = 7 (expected 7)
  F3: ramp:            rank = 7 (expected 7)
  F4: osc:             rank = 7 (expected 7)
  F5: random:          rank = 7 (expected 7)

=== Test complete. ===
Critical: Config B 7-state must show rank=7 for all 5 cases (PE-free).
```

---

## 5. Caveats

### 5.1 Math observability vs practical observability

This rank test verifies **mathematical observability** (LTV F_e/H pair has full rank). It does NOT test:
- **Practical observability**: KF can identify states given finite SNR
- **IIR collapse modes**: when σ²_δxr ≤ C_n·σ²_n_s, a_xm becomes meaningless (Phase 0 §4.4 G2 guard handles)
- **Numerical conditioning**: very-small/large eigenvalues may make practical estimation poor

These are addressed by R(2,2) 3-guard (Phase 6) + Phase 7 closed-loop variance.

### 5.2 PE-window finding (Task 01 historical)

Task 01 found that under Config A 5-state with N=window length:
- Only f_d_seq(1..N-3) affects (x_D, a_x) sub-block rank
- This means PE matters only over the right window

For v2, with state augmentation to 7 (adds δx_D, δa_x), this window may shift. Phase 4 MATLAB test should re-confirm.

### 5.3 IIR a_xm observability dependency

a_xm becomes valid only when:
- t > 0.2 sec (warm-up: G1)
- σ²_δxr > C_n·σ²_n_s (NaN guard: G2)
- h̄ > 1.5 (wall guard: G3)

Outside these conditions, R(2,2) → 1e10 effectively disables y_2. EKF degrades to Config A (single feedback) which requires PE.

For v2, these guards are inherited from v1 design (Phase 0 §4 + Phase 6 to lock).

---

## 6. Phase 4 Pass/Fail Criteria

### 6.1 Pass conditions

- **Critical (must pass)**: Config B 7-state rank=7 for all 5 f_d sweep cases (F1-F5)
- **Important (should pass)**: Config A 7-state rank=7 for f_d ≠ 0 cases (F2-F5)
- **Sanity**: 5-state historical rank=5 for Config B (regression check vs v1 Task 01)

### 6.2 Fail handling

If Config B 7-state shows rank < 7 for any case:
1. Suspect F_e structural error (re-check Phase 1)
2. Suspect H matrix error (re-check Phase 1 §10.5)
3. Suspect numerical issue (try larger N_window)
4. Re-derive observability matrix construction

If Config A f_d=0 shows rank=7 (better than expected):
- Acceptable, indicates v2 F_e structural change improved single-feedback observability
- Document but do not block

---

## 7. Phase 4 Summary

| Item | Predicted | Verification needed |
|---|---|---|
| Config B 7-state rank under F1-F5 | rank = 7 (5/5) | MATLAB execution required |
| Config A 7-state rank under F1 (f_d=0) | rank ≤ 6 (PE) | MATLAB |
| Config A 7-state rank under F2-F5 | rank = 7 | MATLAB |
| 5-state historical rank | rank = 5 | MATLAB |

**v2 F_e structural pattern is identical to v1 (only entry value changes), so Task 01 results should reproduce exactly**.

Phase 4 deliverable:
- This memo (theoretical setup + predicted ranks)
- MATLAB test script `test_script/unit_tests/test_observability_eq17_v2.m` (provided in §4.1)
- Actual MATLAB execution + recorded rank results (deferred to Phase 8 unit test runs, or earlier if user wants)

### 7.1 Phase 5 entry conditions

Phase 5 (Q matrix) needs:
- ✓ F_e v2 form (Phase 1)
- ✓ H matrix (Phase 1)
- ✓ ε structure (Phase 1)
- ✓ C_dpmr/C_n/IF_var values (Phase 2)
- ⚠ Observability rank verified (this Phase, theoretical pass; numerical execution recommended)

Phase 5 can begin in parallel with Phase 4 MATLAB execution since rank test is non-blocking for Q derivation (rank test is sanity gate, not algebraic prerequisite).

---

**End of Phase 4 theoretical predictions. Awaiting user review and/or MATLAB execution.**
