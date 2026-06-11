# Gain Oracle A/B Round 2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement design doc §12 (`reference/eq17_analysis/gain_oracle_ab_design.md`): gate-free (h̄_safe=1) new scenario at {1,5,10} Hz × 100 seeds, analyzer upgrades (per-cycle discard, SEM, x-direct ram, A1 Q55 dynamic check, A3 desc-window â stats), two new figures (gain compare, motion var), and the locked figure restyle — then sample-render for user approval and run the production batch.

**Architecture:** All changes are additive on branch `test/motion-test`. One driver plumbing line (`h_bar_safe`), runner defaults update (`compare_gain_oracle_6state.m`), and the bulk in `analyze_gain_oracle_6state.m` (windows, stats, two new figs, restyle). The controller is NOT touched this round. Production data goes to `test_results/gain_oracle_ab_nogate/` (gitignored); existing `gain_oracle_ab/` 20-seed data is read-only input for sample renders.

**Tech Stack:** MATLAB R2025b via `Bash -batch` only (`/Applications/MATLAB_R2025b.app/bin/matlab`). Never touch the user's interactive MATLAB/MCP session (it lives on the main checkout). All paths below are relative to the worktree root `/Users/kevin/Code/MotionControl_Simu-motion-test`.

**Hard constraints (from project rules / locked decisions):**
- `fig_det_err` block in `make_figs` is FROZEN — do not modify it in any task.
- Figure style per design §12.6 (FS=18, axes `LineWidth` 2.0, `Box on`, `grid off`, natural ticks, `Time (sec)`, stats-in-title, legend names `Desired` / `a_{true}` / `â`).
- Role colors everywhere: Desired/theory/a_pd = green `[0 0.6 0]`; a_true (arm A) = red `[0.8 0 0]`; â (arm B) = blue `[0 0.2 0.9]`; Measured a_xm = light blue `[0.45 0.55 0.95 0.30]`.
- ram overlay layering: â blue LW 2.5 drawn FIRST (base), a_true red LW 1.0 on top (locked mockup `fig_mockup/fig_traj_ram_mock_v3a_colorswap.png`).
- `checkcode` must report 0 issues on every modified/created .m file.
- Commit after each task (format `<type>(eq17): <subject>`).

---

### Task 1: Driver h̄_safe plumbing + unit test

**Files:**
- Modify: `model/dual_track/run_pure_simulation.m` (line ~115)
- Create: `test_script/unit_tests/verify_eq17_unit_hbar_safe_plumbing.m`

- [ ] **Step 1: Write the failing test**

Create `test_script/unit_tests/verify_eq17_unit_hbar_safe_plumbing.m`:

```matlab
function verify_eq17_unit_hbar_safe_plumbing()
%VERIFY_EQ17_UNIT_HBAR_SAFE_PLUMBING config.h_bar_safe -> ctrl_const plumbing
%   (gain_oracle_ab_design.md §12.1). Positioning hold at h_bar = 1.3
%   (inside the default gate band [1, 1.5)):
%   T1: default (no config field)  -> G3 fires (gate_active true somewhere).
%   T2: config.h_bar_safe = 1      -> G3 never fires on any axis.
%   T3: config.h_bar_safe = 1.5 (explicit default value) -> bit-identical
%       p_m_out/f_d_out vs T1 (plumbing adds no behavior change at default).

    fprintf('=== verify_eq17_unit_hbar_safe_plumbing ===\n');
    this_dir  = fileparts(mfilename('fullpath'));
    repo_root = fullfile(this_dir, '..', '..');
    addpath(fullfile(repo_root, 'model'), ...
            fullfile(repo_root, 'model', 'config'), ...
            fullfile(repo_root, 'model', 'controller'), ...
            fullfile(repo_root, 'model', 'wall_effect'), ...
            fullfile(repo_root, 'model', 'thermal_force'), ...
            fullfile(repo_root, 'model', 'trajectory'), ...
            fullfile(repo_root, 'model', 'dual_track'));

    pc  = physical_constants();
    cfg = user_config();
    cfg.eq17_variant      = '6state';
    cfg.trajectory_type   = 'positioning';
    cfg.h_init            = 1.3 * pc.R;       % h_bar = 1.3, inside gate band
    cfg.h_min             = 1.05 * pc.R;      % allow near-wall hold
    cfg.T_sim             = 0.05;             % 80 steps
    cfg.ctrl_enable       = true;
    cfg.thermal_enable    = true;
    cfg.meas_noise_enable = true;

    ro = struct('seed', 1, 'verbose', false, 'collect_diag', true);

    so1 = run_pure_simulation(cfg, ro);                 % T1 default
    assert(any(so1.diag.gate_active(:)), ...
           'T1: G3 expected active at h_bar=1.3 under default h_bar_safe=1.5');
    fprintf('T1 PASS default gate fires at h_bar=1.3\n');

    cfg2 = cfg; cfg2.h_bar_safe = 1;
    so2 = run_pure_simulation(cfg2, ro);                % T2 gate-free
    assert(~any(so2.diag.gate_active(:)), ...
           'T2: G3 must never fire with h_bar_safe=1 (h_bar=1.3 > 1)');
    fprintf('T2 PASS h_bar_safe=1 -> gate-free\n');

    cfg3 = cfg; cfg3.h_bar_safe = 1.5;
    so3 = run_pure_simulation(cfg3, ro);                % T3 explicit default
    assert(isequal(so1.p_m_out, so3.p_m_out) && isequal(so1.f_d_out, so3.f_d_out), ...
           'T3: explicit h_bar_safe=1.5 must be bit-identical to default');
    fprintf('T3 PASS explicit default bit-identical\n');
    fprintf('ALL 3 PASS\n');
end
```

- [ ] **Step 2: Run test to verify it fails**

```bash
/Applications/MATLAB_R2025b.app/bin/matlab -batch "run('/Users/kevin/Code/MotionControl_Simu-motion-test/test_script/unit_tests/verify_eq17_unit_hbar_safe_plumbing.m')"
```
Expected: T1 PASS, **T2 FAIL** (config field is silently ignored, gate still fires).

- [ ] **Step 3: Implement the plumbing**

In `model/dual_track/run_pure_simulation.m`, replace:

```matlab
    eq17_opts.h_bar_safe  = 1.5;
```

with:

```matlab
    eq17_opts.h_bar_safe  = 1.5;
    if isfield(config, 'h_bar_safe') && ~isempty(config.h_bar_safe)
        eq17_opts.h_bar_safe = config.h_bar_safe;   % Round-2 gate-free override (design §12.1)
    end
```

- [ ] **Step 4: Run test to verify it passes** (same command). Expected: ALL 3 PASS.

- [ ] **Step 5: Regression — existing unit test + checkcode**

```bash
/Applications/MATLAB_R2025b.app/bin/matlab -batch "run('/Users/kevin/Code/MotionControl_Simu-motion-test/test_script/unit_tests/verify_eq17_unit_gain_override_6state.m')"
/Applications/MATLAB_R2025b.app/bin/matlab -batch "checkcode('/Users/kevin/Code/MotionControl_Simu-motion-test/model/dual_track/run_pure_simulation.m'); checkcode('/Users/kevin/Code/MotionControl_Simu-motion-test/test_script/unit_tests/verify_eq17_unit_hbar_safe_plumbing.m')"
```
Expected: 5/5 PASS; checkcode silent.

- [ ] **Step 6: Commit**

```bash
git add model/dual_track/run_pure_simulation.m test_script/unit_tests/verify_eq17_unit_hbar_safe_plumbing.m
git commit -m "feat(eq17): config.h_bar_safe plumbing for gate-free runs + unit test"
```

---

### Task 2: Runner Round-2 defaults + gate-free Layer-0 assertion

**Files:**
- Modify: `test_script/integration/compare_gain_oracle_6state.m`

- [ ] **Step 1: Update defaults and config builder**

In the main function header block, replace:

```matlab
    if nargin < 1 || isempty(freqs); freqs = [1 2 5]; end
    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'seeds');   opts.seeds = 1:5;    end
    if ~isfield(opts, 'T_sim');   opts.T_sim = 7.0;    end
```

with:

```matlab
    if nargin < 1 || isempty(freqs); freqs = [1 5 10]; end
    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'seeds');      opts.seeds = 1:100;    end
    if ~isfield(opts, 'T_sim');      opts.T_sim = 4.0;      end
    if ~isfield(opts, 'n_cyc_per_s') %#ok<ALIGN> % osc duration = 2.0 s (design §12.1)
        opts.n_cyc_per_s = 2;        end
    if ~isfield(opts, 'h_bar_safe'); opts.h_bar_safe = 1;   end   % gate-free (B-prime)
```

Replace the default out_root:

```matlab
        opts.out_root = fullfile(project_root, 'test_results', 'gain_oracle_ab');
```

with:

```matlab
        opts.out_root = fullfile(project_root, 'test_results', 'gain_oracle_ab_nogate');
```

In `build_config`, replace `cfg.n_cycles  = 5 * f;` with:

```matlab
    cfg.n_cycles  = opts.n_cyc_per_s * f;   % osc duration = n_cyc_per_s seconds
```

and append after `cfg.suppress_xD = true;`:

```matlab
    cfg.h_bar_safe = opts.h_bar_safe;   % Round 2: 1 -> G3 unreachable (trough h_bar=1.2)
```

(`build_config(f, opts)` already receives `opts`.) Update the function help header: defaults `freqs = [1 5 10]`, `seeds 1:100`, `T_sim 4.0`, output `gain_oracle_ab_nogate/`; note that Round-1 gated runs are reproduced with `compare_gain_oracle_6state([1 2 5], struct('seeds',1:20,'T_sim',7.0,'n_cyc_per_s',5,'h_bar_safe',1.5,'out_root',<...>/gain_oracle_ab))`.

- [ ] **Step 2: Add gate-free Layer-0 assertion**

Change `layer0_checks(runs, opts)` signature to `layer0_checks(runs, cfg, opts)` (update the call site `layer0 = layer0_checks(runs, S? -> runs, cfg, opts);`). Inside, after the arm-B wiring assertion, add (still inside the per-run loop, non-diverged runs only):

```matlab
            % design §12.7 acceptance #6: expected-gate-free runs must never
            % gate. Applies to NOISY runs only — the no-noise det run latches
            % Guard 2 by design (sigma2_dxr -> 0), which is expected behavior.
            gate_free_expected = isfield(cfg, 'h_bar_safe') && ...
                cfg.h_bar_safe < cfg.h_bottom / 2.25 * (1 - 1e-9);
            if gate_free_expected && ~r.is_det
                assert(~any(r.simOut.diag.gate_active(:)), ...
                       'Layer0: gate fired in expected-gate-free run (arm %c seed %d)', ...
                       arm, r.seed);
            end
```

Do NOT hardcode `2.25`: fetch R once before the loop via `R_phys = runs.A.det.simOut.meta.params_value.common.R;` and use `cfg.h_bottom / R_phys`.

- [ ] **Step 3: Smoke run (isolated output) to validate end-to-end**

```bash
/Applications/MATLAB_R2025b.app/bin/matlab -batch "addpath('/Users/kevin/Code/MotionControl_Simu-motion-test/test_script/integration'); compare_gain_oracle_6state(5, struct('smoke', true))"
```
Expected: `Layer 0: PASS`, saved under `test_results/gain_oracle_ab_nogate/f5Hz-smoke/`. Verify the smoke dir is NOT `f5Hz/`.

- [ ] **Step 4: checkcode + commit**

```bash
/Applications/MATLAB_R2025b.app/bin/matlab -batch "checkcode('/Users/kevin/Code/MotionControl_Simu-motion-test/test_script/integration/compare_gain_oracle_6state.m')"
git add test_script/integration/compare_gain_oracle_6state.m
git commit -m "feat(eq17): round-2 runner defaults (gate-free, 100 seeds, {1,5,10} Hz) + layer0 gate assert"
```

---

### Task 3: Analyzer — data_root, per-cycle discard, SEM, x-direct ram

**Files:**
- Modify: `test_script/integration/analyze_gain_oracle_6state.m`

All edits in this task touch existing functions; later tasks add new ones. Keep `fig_det_err` untouched.

- [ ] **Step 1: opts.data_root + new defaults (main function)**

Replace the defaults block with:

```matlab
    if nargin < 1 || isempty(freqs); freqs = [1 5 10]; end
    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'save_fig'); opts.save_fig = true; end
    if ~isfield(opts, 'verbose');  opts.verbose = true;  end
```

and after `out_root` resolution add:

```matlab
    if ~isfield(opts, 'data_root')
        opts.data_root = opts.out_root;   % default: read where we write
    end
```

with `out_root` default changed to `gain_oracle_ab_nogate`. In the per-frequency loop, load from `data_root`, write to `out_root`:

```matlab
        in_dir  = fullfile(opts.data_root, sprintf('f%gHz', f));
        out_dir = fullfile(opts.out_root,  sprintf('f%gHz', f));
        try
            S = load(fullfile(in_dir, 'runs.mat'));
            if ~exist(out_dir, 'dir'); mkdir(out_dir); end
```

(rest unchanged; summary/analysis/figs go to `out_dir`).

- [ ] **Step 2: per-cycle discard + trough skip + 1-cycle stationarity guard**

In `per_freq_analysis` replace `t_discard = 1.0;` with:

```matlab
    t_discard = 1 / cfg.frequency;   % per-cycle discard: drop 1st osc cycle (design §12.2)
```

In `det_metrics`, the trough loop skip `if tj < t_osc0 + 1.0; continue; end` becomes:

```matlab
            if tj < t_osc0 + 1/f - 1e-9; continue; end   % inside per-cycle discard
```

In `cycle_stationarity`, guard the single-cycle case (1 Hz Round 2): after computing `nc`, add:

```matlab
        if nc < 2
            st.(arm).per_cycle_sd  = nan(max(nc,0), 1);
            st.(arm).half_rel_diff = NaN;
            st.(arm).stationary    = true;   % n/a: cannot split halves
            continue;
        end
```

and in `write_summary_md` print `n/a` when `half_rel_diff` is NaN:

```matlab
    fmt_st = @(v) ternary_str(isnan(v), 'n/a', sprintf('%.1f%%', v*100));
```
Add a tiny helper at file end:

```matlab
function s = ternary_str(cond, a, b)
    if cond; s = a; else; s = b; end
end
```
and use it in the stationarity line.

- [ ] **Step 3: x-direct ram (axis 1 uses e directly, no det subtraction, no deflation)**

Extend `ram_window_stats` with an optional final arg `direct_axes` (default `[]`):

```matlab
function Rarm = ram_window_stats(nz, e_ref, em_ref, get_e, get_em, W, wins, sigma2_n, direct_axes)
    if nargin < 9; direct_axes = []; end
```

Inside the per-seed loop, after computing `ram`/`ramm`, add:

```matlab
        % x-direct (design §12.3): det_x == 0 by mirror symmetry, so the raw
        % aligned error IS the ram on these axes — no self-subtraction.
        for dax = direct_axes
            ram(:, dax)  = get_e(nz(s).simOut)  * sel_col(dax);
            ramm(:, dax) = get_em(nz(s).simOut) * sel_col(dax);
        end
```

Simpler and clearer: compute `e_s = get_e(nz(s).simOut); em_s = get_em(nz(s).simOut);` once at loop top, then `ram = e_s - e_ref; ramm = em_s - em_ref;` followed by `ram(:, direct_axes) = e_s(:, direct_axes); ramm(:, direct_axes) = em_s(:, direct_axes);` (no helper needed). Record which axes are direct: `Rarm.direct_axes = direct_axes;`.

Call sites: the **v2** call becomes `ram_window_stats(nz_a, det_e_v2.(arm), det_em_v2, get_e, get_em, W, wins, sigma2_n, 1)`; the v1 call keeps the old 8-arg form (v1 table is the frozen superseded reference).

In `thermal_theory_check`, x-axis switches to direct e and the deflation correction applies to z only. Replace the per-seed block:

```matlab
        for si = 1:Ns
            e_s = get_e(nz(ok(si)).simOut);
            ram = e_s - det_e_v2.(arm);
            zn  = ram(:, 3) ./ sqrt(s2z);
            for k = 1:numel(th.wx)
                vx(k, si) = var(e_s(W.(th.wx{k}), 1));      % x-direct: no det subtraction
            end
            for k = 1:numel(th.wz)
                vz(k, si) = var(zn(W.(th.wz{k})));
            end
        end
        th.(arm).Ns = Ns;
        th.(arm).x_meas_nm2   = mean(vx, 2).' * 1e6;        % x-direct: no deflation
        th.(arm).x_meas_sem   = std(vx, 0, 2).' / sqrt(Ns) * 1e6;
        th.(arm).x_theory_nm2 = cellfun(@(w) mean(s2x(W.(w))), th.wx) * 1e6;
        th.(arm).x_ratio      = th.(arm).x_meas_nm2 ./ th.(arm).x_theory_nm2;
        th.(arm).z_normvar    = mean(vz, 2).' / corr;       % z keeps ensemble ref + deflation
        th.(arm).z_normvar_sem = std(vz / corr, 0, 2).' / sqrt(Ns);
```

- [ ] **Step 4: A2 — SEM everywhere a cross-seed aggregate is reported**

(a) `paired_ratio_stats`: add SEM:

```matlab
                ratio.mean(w, ax)   = mean(r);
                ratio.sem(w, ax)    = std(r) / sqrt(numel(r));
```
(initialize `ratio.sem = nan(nwins, 3);` next to `ratio.mean`).

(b) `write_ram_table` format becomes `mean ± SEM [min, max]`:

```matlab
    fprintf(fid, '| window | axis | A sd [nm] | B sd [nm] | ratio |\n|---|---|---|---|---|\n');
    ...
            sem = @(v) std(v(~isnan(v))) / sqrt(sum(~isnan(v)));
            fprintf(fid, ['| %s | %c | %.2f ± %.2f [%.2f, %.2f] | %.2f ± %.2f [%.2f, %.2f] ', ...
                          '| %.2f ± %.2f [%.2f, %.2f] |\n'], ...
                    R.wins{w}, ax_name(ax), ...
                    mean(sdA, 'omitnan'), sem(sdA), min(sdA), max(sdA), ...
                    mean(sdB, 'omitnan'), sem(sdB), min(sdB), max(sdB), ...
                    R.ratio.mean(w, ax), R.ratio.sem(w, ax), ...
                    R.ratio.rng(w, ax, 1), R.ratio.rng(w, ax, 2));
```

(c) `ahat_analysis`: per-seed window means for SEM. After `rel = ...` replace the three `rel_err_*` lines with per-seed-first versions:

```matlab
    rel_s = (a_stack - a_true) ./ a_true * 100;          % [N-1 x 3 x Ns]
    ah.rel_err_osc   = squeeze(mean(mean(rel_s(W.osc,  :, :), 1), 3)).';
    ah.rel_err_gon   = squeeze(mean(mean(rel_s(W.gon,  :, :), 1), 3)).';
    ah.rel_err_goff  = squeeze(mean(mean(rel_s(W.goff, :, :), 1), 3)).';
    ah.rel_sem_osc   = squeeze(std(mean(rel_s(W.osc,  :, :), 1), 0, 3)).' / sqrt(numel(ok));
    ah.rel_sem_gon   = squeeze(std(mean(rel_s(W.gon,  :, :), 1), 0, 3)).' / sqrt(numel(ok));
    ah.rel_sem_goff  = squeeze(std(mean(rel_s(W.goff, :, :), 1), 0, 3)).' / sqrt(numel(ok));
```
(keep `ah.ens_mean` as-is; note `mean over seeds of per-seed window mean` equals the old ens-mean number, so values are unchanged — only ± appears. Keep the NaN-fill branch in sync with the new fields.)

(d) `write_summary_md`: thermal tables gain a `± SEM` column from the new `x_meas_sem` / `z_normvar_sem`; the â lines print `value ± SEM` per axis.

- [ ] **Step 5: Re-run analyzer on existing f2Hz data (regression-style check)**

```bash
/Applications/MATLAB_R2025b.app/bin/matlab -batch "addpath('/Users/kevin/Code/MotionControl_Simu-motion-test/test_script/integration'); analyze_gain_oracle_6state(2, struct('data_root', '/Users/kevin/Code/MotionControl_Simu-motion-test/test_results/gain_oracle_ab', 'out_root', '/Users/kevin/Code/MotionControl_Simu-motion-test/test_results/gain_oracle_ab/round2_dev', 'save_fig', false))"
```
Expected: completes; `round2_dev/f2Hz/summary.md` exists; spot-check (read the file): (i) ram v2 table has `± SEM`; (ii) x-direct values within ~3% of the old ensemble-ref values (deflation restored ≈ +2.6% at Ns=20, plus the det-residual leak removal); (iii) z values unchanged vs production summary except window-boundary effects from `t_discard = 0.5` (was 1.0 @2 Hz — osc window now longer, small shifts expected and acceptable in dev output).

- [ ] **Step 6: checkcode + commit**

```bash
/Applications/MATLAB_R2025b.app/bin/matlab -batch "checkcode('/Users/kevin/Code/MotionControl_Simu-motion-test/test_script/integration/analyze_gain_oracle_6state.m')"
git add test_script/integration/analyze_gain_oracle_6state.m
git commit -m "feat(eq17): analyzer round-2 core - data_root, per-cycle discard, SEM, x-direct ram"
```

---

### Task 4: Analyzer — A1 Q55 dynamic check + A3 desc-window â stats

**Files:**
- Modify: `test_script/integration/analyze_gain_oracle_6state.m`

- [ ] **Step 1: shared deterministic gain skeleton (used by A1, fig 1, fig 2)**

In `per_freq_analysis`, after the windows block, add:

```matlab
    % --- deterministic gain skeleton along the DESIRED trajectory (a_pd) ---
    % a_pd,i[k] = a_nom / C_i(h_bar_d[k]) — designer-known, no sim privilege.
    a_nom = P.common.Ts / P.common.gamma_N;
    hbd   = max(h_bar_d, 1.001);
    [c_pa_d, c_pe_d, drv_d] = calc_correction_functions(hbd, true);
    A.gain.a_pd   = [a_nom ./ c_pa_d, a_nom ./ c_pa_d, a_nom ./ c_pe_d];  % [N-1 x 3]
    A.gain.Kh_pd  = [drv_d.K_h_para(:), drv_d.K_h_para(:), drv_d.K_h_perp(:)];
```

(`h_bar_d` already exists for the gate mask; if `calc_correction_functions` is not vectorized over `h_bar`, wrap in a plain for-loop — verify with a 3-element call first.)

- [ ] **Step 2: A1 function**

Add new function (after `ahat_analysis`):

```matlab
function q = q55_dynamic_check(runsA, W, wins, gain, lc, kBT, R_phys)
%Q55_DYNAMIC_CHECK A1: arm-A true-gain ram vs closed forms (design §12.3,
%   findings doc eq17_6state_review_findings.md §8.1 three-layer chain).
%   Level    : Var(a_ram)        vs C_dx   *(a*K_h/R)^2 * 4kBT*a_z
%   Increment: Var(diff(a_ram))  vs 2/(1+lc)*(a*K_h/R)^2 * 4kBT*a_z   (= Q55)
%   Theory is pointwise along the desired trajectory (a_pd, Kh_pd), then
%   window-averaged. Measured variances deflation-corrected by /(1-1/Ns)
%   (ensemble-mean self-subtraction; same correction for the increment —
%   cross-seed independence makes it exact to O(1/Ns^2)).
    nz = runsA.noisy;
    ok = find(~[nz.diverged]);
    Ns = numel(ok);
    q  = struct('Ns', Ns, 'wins', {wins});
    stack = [];
    for s = ok
        stack = cat(3, stack, nz(s).simOut.a_true_out(2:end, :));
    end
    a_ram = stack - mean(stack, 3);                       % [N-1 x 3 x Ns]
    corr  = 1 - 1/Ns;
    C_dx  = 2 + 1/(1 - lc^2);
    s2dh  = 4 * kBT * gain.a_pd(:, 3);                    % per-step kick, h-dir = z
    base  = (gain.a_pd .* gain.Kh_pd / R_phys).^2 .* s2dh;   % [N-1 x 3]
    for w = 1:numel(wins)
        idx = W.(wins{w});
        for ax = [1 3]
            vl = squeeze(var(a_ram(idx, ax, :), 0, 1)) / corr;             % [Ns x 1]
            vi = squeeze(var(diff(a_ram(idx, ax, :), 1, 1), 0, 1)) / corr;
            q.meas_level(w, ax) = mean(vl);  q.sem_level(w, ax) = std(vl)/sqrt(Ns);
            q.meas_incr(w, ax)  = mean(vi);  q.sem_incr(w, ax)  = std(vi)/sqrt(Ns);
            q.th_level(w, ax)   = C_dx        * mean(base(idx, ax));
            q.th_incr(w, ax)    = 2/(1 + lc)  * mean(base(idx, ax));
        end
    end
    q.ratio_level = q.meas_level ./ q.th_level;
    q.ratio_incr  = q.meas_incr  ./ q.th_incr;
end
```

Call from `per_freq_analysis` (after `A.ahat = ...`):

```matlab
    A.q55 = q55_dynamic_check(runs.A, W, {'osc', 'gon', 'goff'}, A.gain, ...
                              cfg.lambda_c, kBT, R_phys);
```

- [ ] **Step 3: A3 — desc-window â stats**

In `ahat_analysis` add (next to the other windows; signature already has `W`):

```matlab
    ah.rel_err_desc = squeeze(mean(mean(rel_s(W.desc, :, :), 1), 3)).';
    ah.rel_sem_desc = squeeze(std(mean(rel_s(W.desc, :, :), 1), 0, 3)).' / sqrt(numel(ok));
    sdw = @(wmask) reshape(std(dev(wmask, :, :), 0, 1), 3, []);
    ah.ram_std_desc = sdw(W.desc);
    ah.ram_std_gon  = sdw(W.gon);
    ah.ram_std_goff = sdw(W.goff);
```
(`dev = a_stack - ah.ens_mean;` already exists; keep the all-diverged NaN branch in sync.)

- [ ] **Step 4: summary.md sections**

In `write_summary_md`:
(a) arm-B gain-estimation block: add desc rel-err line and a per-window ram-std table (seed mean ± SEM, axes x/z).
(b) New section after thermal validation:

```matlab
    fprintf(fid, '\n## A1: a_true gain ram vs Q55 closed forms (arm A, %d seeds)\n\n', A.q55.Ns);
    fprintf(fid, ['Level: Var(a_ram) vs C_dx*(a*K_h/R)^2*4kBT*a_z; ', ...
                  'Increment (= Q55): Var(diff a_ram) vs [2/(1+lc)]*(a*K_h/R)^2*4kBT*a_z. ', ...
                  'Theory pointwise along a_pd, window-averaged; meas /(1-1/Ns).\n\n']);
    fprintf(fid, '| window | axis | level meas/th | incr meas/th |\n|---|---|---|---|\n');
    for w = 1:numel(A.q55.wins)
        for ax = [1 3]
            fprintf(fid, '| %s | %c | %.3f | %.3f |\n', A.q55.wins{w}, 'x z '(2*ax-1), ...
                    A.q55.ratio_level(w, ax), A.q55.ratio_incr(w, ax));
        end
    end
```
(Implementer: use a proper axis-name lookup `ax_name(ax)`, not the inline char trick.)

- [ ] **Step 5: Re-run dev analysis (same command as Task 3 Step 5), checkcode, commit**

Expected: A1 section present; osc-window incr ratio plausibly O(1) far from wall (gon may deviate — that is the experiment's finding, not a bug; do not "fix" it).

```bash
git add test_script/integration/analyze_gain_oracle_6state.m
git commit -m "feat(eq17): A1 Q55 dynamic check + A3 desc-window a_hat stats"
```

---

### Task 5: Restyle fig_traj_det + fig_traj_ram (fig_det_err FROZEN)

**Files:**
- Modify: `test_script/integration/analyze_gain_oracle_6state.m` (`make_figs` only)

- [ ] **Step 1: Replace the style block and the two figure sections**

New shared constants at the top of `make_figs` (replace the existing ones; `fig_det_err` keeps using `COL_A`/`COL_B`/old sizes — leave its lines byte-identical, so KEEP the old constants it references and ADD the new ones):

```matlab
    % Round-2 locked style (design §12.6): role colors + reference fonts
    COL_DES   = [0 0.6 0];                % Desired / a_pd / theory
    COL_TRUE2 = [0.8 0 0];                % a_true (arm A)
    COL_HAT2  = [0 0.2 0.9];              % a_hat  (arm B)
    FS2 = 18; LFS2 = 14; AXLW2 = 2.0;     % fonts + bold box/ticks
    T_END = ceil(t_e(end));
```

`fig_traj_det` (replace plotting/styling, same data):

```matlab
    ft = figure('Position', [80 80 1100 460], 'Color', 'w', 'NumberTitle', 'off', ...
                'Visible', 'off');
    hold on;
    plot(t_e, pd_al(:, 3), '-', 'Color', COL_DES, 'LineWidth', 3, ...
         'DisplayName', 'Desired');
    plot(t_e, A.noisy_det.A.det_traj(:, 3), '-', 'Color', COL_TRUE2, 'LineWidth', 2, ...
         'DisplayName', 'a_{true}');
    plot(t_e, A.noisy_det.B.det_traj(:, 3), '-', 'Color', COL_HAT2, 'LineWidth', 2, ...
         'DisplayName', 'â');
    xlim([0 T_END]);
    title(sprintf('z_{det}:   descent peak  a_{true} %.1f / â %.1f nm     osc A_e  %.2f / %.2f nm', ...
          A.det.A.desc_peak(3)*1e3, A.det.B.desc_peak(3)*1e3, ...
          A.det.A.A_e(3)*1e3, A.det.B.A_e(3)*1e3), 'FontSize', FS2, 'FontWeight', 'bold');
    ylabel('z  (\mum)', 'FontSize', FS2, 'FontWeight', 'bold');
    xlabel('Time (sec)', 'FontSize', FS2, 'FontWeight', 'bold');
    legend('Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', LFS2, 'FontWeight', 'bold', 'Box', 'off');
    set(gca, 'FontSize', FS2, 'FontWeight', 'bold', 'LineWidth', AXLW2, 'Box', 'on');
    grid off;
    exportgraphics(ft, fullfile(out_dir, 'fig_traj_det.png'), 'Resolution', 150);
    close(ft);
```

`fig_traj_ram` (layer order locked; x row uses the DIRECT aligned error per Task 3, so store `keep.e_traj` in the noisy-det keep struct — add `keep.e_traj = get_e-style aligned error of the same traj seed` in `per_freq_analysis` where `keep.ram_traj` is set):

```matlab
    fm = figure('Position', [80 80 1100 720], 'Color', 'w', 'NumberTitle', 'off', ...
                'Visible', 'off');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for r = 1:2
        c = cols(r); nexttile; hold on;
        if c == 1   % x-direct (design §12.3)
            sigA = A.noisy_det.A.e_traj(:, 1) * 1e3;
            sigB = A.noisy_det.B.e_traj(:, 1) * 1e3;
        else
            sigA = A.noisy_det.A.ram_traj(:, c) * 1e3;
            sigB = A.noisy_det.B.ram_traj(:, c) * 1e3;
        end
        hB = plot(t_e, sigB, '-', 'Color', COL_HAT2,  'LineWidth', 2.5, 'DisplayName', 'â');
        hA = plot(t_e, sigA, '-', 'Color', COL_TRUE2, 'LineWidth', 1.0, 'DisplayName', 'a_{true}');
        xlim([0 T_END]); ylim(RAM_YLIM);
        title(sprintf('%c_{ram}:   var  a_{true} %.0f nm^2     â %.0f nm^2', ...
              axl(r), var(sigA), var(sigB)), 'FontSize', FS2, 'FontWeight', 'bold');
        ylabel(sprintf('%c_{ram}  (nm)', axl(r)), 'FontSize', FS2, 'FontWeight', 'bold');
        if r == 1
            legend([hA hB], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
                   'FontSize', LFS2, 'FontWeight', 'bold', 'Box', 'off');
        end
        set(gca, 'FontSize', FS2, 'FontWeight', 'bold', 'LineWidth', AXLW2, 'Box', 'on');
        grid off;
    end
    xlabel('Time (sec)', 'FontSize', FS2, 'FontWeight', 'bold');
    exportgraphics(fm, fullfile(out_dir, 'fig_traj_ram.png'), 'Resolution', 150);
    close(fm);
```

Note: fixed `RAM_YLIM = [-150 150]` is kept (cross-frequency lock); the fixed `YTick`/`XTick` overrides are REMOVED (natural density per §12.6). Delete `XLIM_T`/`XTICK_T`/`DET_Z_YTICK`/`RAM_YTICK` only where no longer referenced — `fig_det_err` still uses `XLIM_T`/`XTICK_T`, keep those definitions.

- [ ] **Step 2: Render from existing f2Hz data, eyeball, checkcode, commit**

```bash
/Applications/MATLAB_R2025b.app/bin/matlab -batch "addpath('/Users/kevin/Code/MotionControl_Simu-motion-test/test_script/integration'); analyze_gain_oracle_6state(2, struct('data_root', '/Users/kevin/Code/MotionControl_Simu-motion-test/test_results/gain_oracle_ab', 'out_root', '/Users/kevin/Code/MotionControl_Simu-motion-test/test_results/gain_oracle_ab/round2_dev'))"
```
Read the produced PNGs (controller agent does this visually): layering = blue thick under red thin; titles carry stats; box/ticks bold; `fig_det_err.png` byte-comparable behavior (regenerated but visually identical to production style of Round 1).

```bash
git add test_script/integration/analyze_gain_oracle_6state.m
git commit -m "feat(eq17): round-2 figure restyle (stats-in-title, role colors, locked ram layering)"
```

---

### Task 6: New figure — fig_gain_compare

**Files:**
- Modify: `test_script/integration/analyze_gain_oracle_6state.m`

- [ ] **Step 1: Data prep in per_freq_analysis**

After the A1 block (Task 4) add:

```matlab
    % --- fig_gain_compare data (design §12.4): arms per user decision ---
    % a_true ensemble = ARM A (gain under near-perfect control);
    % a_hat ensemble  = ARM B (production estimate, = A.ahat.ens_mean);
    % a_xm raw layer  = ARM B, traj-figure seed.
    nzA = runs.A.noisy;  okA = find(~[nzA.diverged]);
    st = [];
    for s = okA
        st = cat(3, st, nzA(s).simOut.a_true_out(2:end, :));
    end
    A.gain.a_true_ens = mean(st, 3);                       % [N-1 x 3]
    if ~isempty(nd_pair_idx)
        A.gain.a_xm_seed = runs.B.noisy(nd_pair_idx).simOut.diag.a_xm(2:end, :);
    else
        A.gain.a_xm_seed = nan(size(A.gain.a_pd));
    end
```

- [ ] **Step 2: Figure block in make_figs**

```matlab
    % ---- fig_gain_compare: a_xm / a_pd / a_true(A) / a_hat(B), x+z ----
    COL_MEAS2 = [0.45 0.55 0.95 0.30];
    fg = figure('Position', [80 80 1100 720], 'Color', 'w', 'NumberTitle', 'off', ...
                'Visible', 'off');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    glbl = {'a_x', 'a_z'};
    for r = 1:2
        c = cols(r); nexttile; hold on;
        hm = plot(t_e, A.gain.a_xm_seed(:, c),  '-', 'Color', COL_MEAS2, ...
                  'LineWidth', 0.5, 'DisplayName', 'Measured');
        hp = plot(t_e, A.gain.a_pd(:, c),       '-', 'Color', COL_DES, ...
                  'LineWidth', 3.0, 'DisplayName', 'a_{pd}');
        ht = plot(t_e, A.gain.a_true_ens(:, c), '-', 'Color', COL_TRUE2, ...
                  'LineWidth', 2.0, 'DisplayName', 'a_{true}');
        hh = plot(t_e, A.ahat.ens_mean(:, c),   '-', 'Color', COL_HAT2, ...
                  'LineWidth', 2.0, 'DisplayName', 'â');
        xlim([0 T_END]);
        ylim([0, 1.25 * max(A.gain.a_pd(:, c))]);
        title(sprintf('%s:   â rel-err  osc %+.1f%%     near-wall %+.1f%%', ...
              glbl{r}, A.ahat.rel_err_osc(c), A.ahat.rel_err_gon(c)), ...
              'FontSize', FS2, 'FontWeight', 'bold');
        ylabel(sprintf('%s  (\\mum/pN)', glbl{r}), 'FontSize', FS2, 'FontWeight', 'bold');
        if r == 1
            legend([hm hp ht hh], 'Location', 'northoutside', 'Orientation', ...
                   'horizontal', 'FontSize', LFS2, 'FontWeight', 'bold', 'Box', 'off');
        end
        set(gca, 'FontSize', FS2, 'FontWeight', 'bold', 'LineWidth', AXLW2, 'Box', 'on');
        grid off;
    end
    xlabel('Time (sec)', 'FontSize', FS2, 'FontWeight', 'bold');
    exportgraphics(fg, fullfile(out_dir, 'fig_gain_compare.png'), 'Resolution', 150);
    close(fg);
```

(`a_xm` can swing negative/huge during warm-up; the `ylim` clamp handles it — do not let autoscale follow a_xm.)

- [ ] **Step 3: Render on f2Hz dev output, eyeball, checkcode, commit**

```bash
git add test_script/integration/analyze_gain_oracle_6state.m
git commit -m "feat(eq17): fig_gain_compare (a_xm/a_pd/a_true/a_hat overlay)"
```

---

### Task 7: New figure — fig_motion_var

**Files:**
- Modify: `test_script/integration/analyze_gain_oracle_6state.m`

- [ ] **Step 1: Pointwise variance data in per_freq_analysis**

The full per-seed ram stacks live in `noisy_det_extract`'s return (currently reduced to `keep`). Where `nd` is still in scope (inside the arm loop), additionally compute the pointwise variance BEFORE reducing (z from ensemble-ref ram with deflation; x direct from aligned e):

```matlab
        % fig_motion_var (design §12.5): pointwise across-seed variance
        Nsn = nd.n_seeds;
        mv_z = var(nd.ram_traj(:, 3, :), 0, 3) / (1 - 1/Nsn);   % deflation-corrected
        ex = [];
        for s2 = nd.seeds_used
            ex = cat(2, ex, get_e(runs.(arm).noisy(s2).simOut) * [1; 0; 0]);
        end
        mv_x = var(ex, 0, 2);                                    % x-direct: no correction
        A.motion_var.(arm) = struct('var_x_um2', mv_x, 'var_z_um2', mv_z(:), 'Ns', Nsn);
```

Theory curve (once, after `A.gain.a_pd` exists):

```matlab
    lc2 = cfg.lambda_c;
    A.motion_var.theory_um2 = (2 + 1/(1-lc2^2)) * 4 * kBT * A.gain.a_pd ...
                              + (1-lc2)/(1+lc2) * repmat(sigma2_n.', size(A.gain.a_pd, 1), 1);
```

- [ ] **Step 2: Figure block in make_figs**

```matlab
    % ---- fig_motion_var: theory(a_pd) vs pointwise ensemble var, x+z ----
    fv = figure('Position', [80 80 1100 720], 'Color', 'w', 'NumberTitle', 'off', ...
                'Visible', 'off');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for r = 1:2
        c = cols(r); nexttile; hold on;
        if c == 1
            vA = A.motion_var.A.var_x_um2 * 1e6;  vB = A.motion_var.B.var_x_um2 * 1e6;
        else
            vA = A.motion_var.A.var_z_um2 * 1e6;  vB = A.motion_var.B.var_z_um2 * 1e6;
        end
        vth = A.motion_var.theory_um2(:, c) * 1e6;
        hB = plot(t_e, vB,  '-', 'Color', COL_HAT2,  'LineWidth', 1.0, 'DisplayName', 'â');
        hA = plot(t_e, vA,  '-', 'Color', COL_TRUE2, 'LineWidth', 1.0, 'DisplayName', 'a_{true}');
        hT = plot(t_e, vth, '-', 'Color', COL_DES,   'LineWidth', 3.0, 'DisplayName', 'Theory');
        xlim([0 T_END]);
        ylim([0, max(vth) * 4]);
        rA = mean(vA(W_osc_fig)) / mean(vth(W_osc_fig));
        rB = mean(vB(W_osc_fig)) / mean(vth(W_osc_fig));
        title(sprintf('var(%c_{ram}):   osc meas/theory  a_{true} %.2f     â %.2f', ...
              axl(r), rA, rB), 'FontSize', FS2, 'FontWeight', 'bold');
        ylabel(sprintf('var(%c_{ram})  (nm^2)', axl(r)), 'FontSize', FS2, 'FontWeight', 'bold');
        if r == 1
            legend([hT hA hB], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
                   'FontSize', LFS2, 'FontWeight', 'bold', 'Box', 'off');
        end
        set(gca, 'FontSize', FS2, 'FontWeight', 'bold', 'LineWidth', AXLW2, 'Box', 'on');
        grid off;
    end
    xlabel('Time (sec)', 'FontSize', FS2, 'FontWeight', 'bold');
    exportgraphics(fv, fullfile(out_dir, 'fig_motion_var.png'), 'Resolution', 150);
    close(fv);
```

`W_osc_fig`: make_figs has no `W`; store the osc mask in the analysis struct in `per_freq_analysis` (`A.W_osc = W.osc;` — logical [N-1 x 1], negligible size) and use `W_osc_fig = A.W_osc;`.

- [ ] **Step 3: Render on f2Hz dev output, eyeball (curves should hug theory for arm A; ~14%-level scatter at Ns=20 is expected and stays in the figure — NO smoothing), checkcode, commit**

```bash
git add test_script/integration/analyze_gain_oracle_6state.m
git commit -m "feat(eq17): fig_motion_var (a_pd theory vs pointwise ensemble variance)"
```

---

### Task 8: Sample gate — full sample set for user review (STOP point)

**Files:** none (run only)

- [ ] **Step 1: Clean dev folder, produce the canonical sample set from existing 20-seed f2Hz data**

```bash
rm -rf /Users/kevin/Code/MotionControl_Simu-motion-test/test_results/gain_oracle_ab/round2_dev
/Applications/MATLAB_R2025b.app/bin/matlab -batch "addpath('/Users/kevin/Code/MotionControl_Simu-motion-test/test_script/integration'); analyze_gain_oracle_6state(2, struct('data_root', '/Users/kevin/Code/MotionControl_Simu-motion-test/test_results/gain_oracle_ab', 'out_root', '/Users/kevin/Code/MotionControl_Simu-motion-test/test_results/gain_oracle_ab/round2_samples'))"
```

- [ ] **Step 2: Controller agent visually inspects all 5 PNGs + summary.md, then STOPS and presents to the user.** Iterate on style details (title contents, line choices) per user feedback by editing `make_figs` and re-rendering. **Do not proceed to Task 9 without explicit user approval** (design §12.7 acceptance #8).

Note: sample data is gated Round-1 data (h̄_safe=1.5, 20 seeds, T=7 s) — gate-duty and A1 gon numbers in the sample summary are NOT Round-2 results; the sample validates style and pipeline only.

---

### Task 9: Production batch + final verification (after user approval)

**Files:** none (run only)

- [ ] **Step 1: Full regression before burning 45 min**

```bash
/Applications/MATLAB_R2025b.app/bin/matlab -batch "run('/Users/kevin/Code/MotionControl_Simu-motion-test/test_script/unit_tests/verify_eq17_unit_gain_override_6state.m')"
/Applications/MATLAB_R2025b.app/bin/matlab -batch "run('/Users/kevin/Code/MotionControl_Simu-motion-test/test_script/unit_tests/verify_eq17_unit_hbar_safe_plumbing.m')"
```
Plus the h50 gate (same invocation as Round 1, `run_eq17_6state_all` h50 scenario): PASS with unchanged numbers.

- [ ] **Step 2: Production run (background, ~45–60 min)**

```bash
/Applications/MATLAB_R2025b.app/bin/matlab -batch "addpath('/Users/kevin/Code/MotionControl_Simu-motion-test/test_script/integration'); compare_gain_oracle_6state([1 5 10], struct()); analyze_gain_oracle_6state([1 5 10], struct())"
```
(defaults now encode the Round-2 matrix; run via Bash `run_in_background`). Watch for: diverged-run reports (legitimate results — near-wall gate-free â may diverge; they are findings), Layer-0 PASS per frequency, three `f*Hz/` folders under `gain_oracle_ab_nogate/` each with runs.mat + analysis.mat + summary.md + 5 figs.

- [ ] **Step 3: Results digest for the user**

Read the three summary.md files; report: diverged counts, A1 ratios, â per-window rel-err (gate-free vs Round-1 gated 20-seed numbers — cross-scenario comparison, flag this caveat), motion-var osc ratios, det descent peaks. No further commits expected from this task (test_results is gitignored).

---

## Self-review notes

- Spec coverage: §12.1 → Tasks 1–2; §12.2–12.3 → Tasks 3–4; §12.4 → Task 6; §12.5 → Task 7; §12.6 → Tasks 5–7; §12.7 → Tasks 8–9. fig_det_err freeze respected (Task 5 explicitly keeps its lines and the constants it uses).
- Type consistency: `A.gain` created in Task 4 Step 1 is consumed by Tasks 6–7; `A.ahat.ens_mean` exists pre-plan; `nd_pair_idx` exists pre-plan in `per_freq_analysis`; `keep.e_traj` added in Task 5 must be set where `keep.ram_traj` is set (aligned error of the SAME seed: `get_e(runs.(arm).noisy(nd_pair_idx).simOut)`).
- Known acceptable deviations in dev/sample renders: window shifts from per-cycle discard on old 2 Hz data; gate-duty nonzero in samples (Round-1 data is gated).
