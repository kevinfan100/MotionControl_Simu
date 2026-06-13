function results = verify_axm_cdpmr_6state(freq_dir, opts)
%VERIFY_AXM_CDPMR_6STATE Verify Var(dx_r) = C_dpmr*4kBT*a + C_n*sigma2_n identity.
%
%   results = verify_axm_cdpmr_6state(freq_dir)
%   results = verify_axm_cdpmr_6state(freq_dir, opts)
%
%   freq_dir : path to a single-frequency dir containing runs.mat
%              (e.g. .../test_results/gain_oracle_ab_nogate/f1Hz)
%   opts     : optional struct
%                opts.n_seg    descent segments for scatter (default 10)
%                opts.save_fig write PNG figures to freq_dir (default true)
%                opts.verbose  print progress (default true)
%
%   Outputs written to freq_dir:
%     fig_dxr_var_time.png    ensemble Var(dx_r) vs time, theory overlay
%     fig_dxr_var_scatter.png Var(dx_r) vs a_true, descent segments + theory line
%     fig_axm_recover.png     a_xm / a_true ensemble, both arms
%     axm_cdpmr_verify.mat    all computed arrays
%     axm_cdpmr_summary.md    per-window ratio/bias table
%
%   Returns results struct: var_dxr, var_theory, a_xm_ens, a_true_ens,
%   scatter, win_stats, C_dpmr, C_n, t_e, a_pd, W.
%
%   Theory identity (plan D2, full a_pd form):
%     Var(dx_r[k]) = C_dpmr * 4kBT * a[k] + C_n * sigma2_n
%   Arm A (a_ctrl = a_true) = verification baseline (theory strict).
%   Arm B (a_ctrl = a_hat)  = diagnosis (self-consistent bias expected).
%
%   Design decisions: see plan sleepy-forging-planet.md.
%   Figure style mirrors analyze_gain_oracle_6state.m make_figs (locked round-2).
%
%   See also: analyze_gain_oracle_6state, compare_gain_oracle_6state

    % ----------------------------------------------------------------
    % Input guards
    % ----------------------------------------------------------------
    if nargin < 1 || isempty(freq_dir)
        error('verify_axm_cdpmr_6state:badInput', 'freq_dir is required');
    end
    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'n_seg');    opts.n_seg    = 10;   end
    if ~isfield(opts, 'save_fig'); opts.save_fig = true; end
    if ~isfield(opts, 'verbose');  opts.verbose  = true; end

    % ----------------------------------------------------------------
    % Addpath: wall_effect functions (calc_correction_functions)
    % ----------------------------------------------------------------
    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(fileparts(script_dir));
    addpath(fullfile(project_root, 'model', 'wall_effect'));

    % ----------------------------------------------------------------
    % Load
    % ----------------------------------------------------------------
    runs_path = fullfile(freq_dir, 'runs.mat');
    if ~exist(runs_path, 'file')
        error('verify_axm_cdpmr_6state:noFile', 'runs.mat not found: %s', runs_path);
    end
    S    = load(runs_path);
    runs = S.runs;
    cfg  = S.cfg;

    % ================================================================
    % 1. PRE-FLIGHT ASSERTS  (rigor point 4)
    % ================================================================
    ok_ref = find(~[runs.A.noisy.diverged], 1);
    assert(~isempty(ok_ref), ...
        'verify_axm_cdpmr_6state: all arm A noisy seeds diverged');
    so_ref = runs.A.noisy(ok_ref).simOut;

    assert(isfield(so_ref, 'diag'), ...
        ['verify_axm_cdpmr_6state: simOut.diag missing. ', ...
         'Re-run compare_gain_oracle_6state with collect_diag=true.']);
    assert(isfield(so_ref.diag, 'dx_r'), ...
        'verify_axm_cdpmr_6state: simOut.diag.dx_r missing');
    assert(isfield(so_ref.diag, 'a_xm'), ...
        'verify_axm_cdpmr_6state: simOut.diag.a_xm missing');
    assert(isfield(so_ref.diag, 'sigma2_dxr_hat'), ...
        'verify_axm_cdpmr_6state: simOut.diag.sigma2_dxr_hat missing');
    assert(isfield(so_ref, 'a_true_out'), ...
        'verify_axm_cdpmr_6state: simOut.a_true_out missing');

    % ================================================================
    % 2. CONSTANTS
    % ================================================================
    P      = runs.A.det.simOut.meta.params_value;
    kBT    = P.ctrl.k_B * P.ctrl.T;
    R_phys = P.common.R;
    w_hat  = P.wall.w_hat(:);      % force [3x1] column
    pz     = P.wall.pz;
    Ts     = P.common.Ts;
    fs     = 1 / Ts;               % [Hz] sample rate

    sig2_n = cfg.meas_noise_std(:).^2;   % [3x1] um^2, per-axis sensor noise var

    % C_dpmr / C_n: full form from ctrl_const (~3.16 / ~1.11 at lambda_c=0.7)
    C_dpmr = so_ref.ctrl_const.C_dpmr;
    C_n    = so_ref.ctrl_const.C_n;

    % Assert full form: simplified (lambda_c-only) gives ~3.96 which is 25%
    % larger; using it in a_xm inversion would produce a systematic ~25% bias.
    assert(C_dpmr < 3.6, ...
        ['verify_axm_cdpmr_6state: C_dpmr=%.4f >= 3.6. This looks like the ', ...
         'lambda_c-only simplified form (3.96). Expected full form ~3.16 ', ...
         '(includes a_pd term). Check run_pure_simulation ctrl_const wiring.'], ...
        C_dpmr);

    if opts.verbose
        fprintf('[axm_verify] C_dpmr=%.4f  C_n=%.4f  kBT=%.4e  R=%.4g um  Ts=%.6g s\n', ...
                C_dpmr, C_n, kBT, R_phys, Ts);
    end

    % ================================================================
    % 3. ALIGNED GRID + WINDOWS  (mirror analyzer L106-128)
    % ================================================================
    t_e   = so_ref.tout(2:end);            % [N x 1] aligned time
    pd_al = so_ref.p_d_out(2:end, :);     % [N x 3] desired trajectory
    N     = numel(t_e);
    T_END = ceil(t_e(end));               % full time span for xlim

    % Deterministic desired h_bar: projection onto wall normal
    h_bar_d = (pd_al * w_hat - pz) / R_phys;   % [N x 1]

    t_osc0    = cfg.t_hold + cfg.t_descend_override;
    t_osc1    = t_osc0 + cfg.n_cycles / cfg.frequency;
    t_discard = 1 / cfg.frequency;    % drop 1st osc cycle (transient)
    H_BAR_GATE = 1.5;                  % = ctrl_const.h_bar_safe

    W.desc = (t_e >= cfg.t_hold       & t_e < t_osc0);
    W.osc  = (t_e >= t_osc0+t_discard & t_e < t_osc1);
    W.gon  = W.osc & (h_bar_d < H_BAR_GATE);
    W.goff = W.osc & (h_bar_d >= H_BAR_GATE);

    if opts.verbose
        fprintf('[axm_verify] windows: desc=%d  osc=%d  gon=%d  goff=%d samples\n', ...
                sum(W.desc), sum(W.osc), sum(W.gon), sum(W.goff));
    end

    % ================================================================
    % 4. a_pd SKELETON  (mirror analyzer L282-295)
    %    a_pd,i[k] = a_nom / C_i(h_bar_d[k]) — designer-known, no sim privilege.
    % ================================================================
    a_nom  = P.common.Ts / P.common.gamma_N;
    hbd    = max(h_bar_d, 1.001);    % clip to valid domain of correction functions
    c_pa_d = zeros(N, 1);
    c_pe_d = zeros(N, 1);
    for ki = 1:N
        [c_pa_d(ki), c_pe_d(ki)] = calc_correction_functions(hbd(ki));
    end
    % [N x 3]: col 1=x(para), col 2=y(para), col 3=z(perp, wall normal)
    a_pd = [a_nom ./ c_pa_d, a_nom ./ c_pa_d, a_nom ./ c_pe_d];

    % ================================================================
    % 5. PER-ARM ENSEMBLE EXTRACTION
    % ================================================================
    % dx_r is zero-mean by construction: the EWMA high-pass residual
    % (delta_x_m minus its moving average) has zero ensemble mean when
    % driven by zero-mean thermal + sensor noise under the closed-loop
    % pole lambda_c.  Therefore var(dx_r, 0, 3) — the standard
    % Bessel-corrected sample variance across seeds — is an unbiased
    % estimator of E[dx_r^2] without any additional deflation factor.
    % Contrast: the analyzer's ram_v2 subtracts a finite-sample det_traj,
    % introducing a 1-1/Ns self-subtraction deflation that must be corrected.
    % Here no such subtraction occurs, so no extra correction is needed.
    ARMS = {'A', 'B'};
    V    = struct();
    dxr_stack_all = struct();    % kept for per-seed scatter computation

    for ai = 1:2
        arm = ARMS{ai};
        nz  = runs.(arm).noisy;
        ok  = find(~[nz.diverged]);
        if isempty(ok)
            error('verify_axm_cdpmr_6state:allDiverged', ...
                  'All arm %s noisy seeds diverged', arm);
        end
        Ns = numel(ok);

        dxr_loc = zeros(N, 3, Ns);
        axm_loc = zeros(N, 3, Ns);
        atr_loc = zeros(N, 3, Ns);
        for si = 1:Ns
            s = ok(si);
            dxr_loc(:, :, si) = nz(s).simOut.diag.dx_r(2:end, :);
            axm_loc(:, :, si) = nz(s).simOut.diag.a_xm(2:end, :);
            atr_loc(:, :, si) = nz(s).simOut.a_true_out(2:end, :);
        end

        % var(X,0,3): Bessel-corrected sample variance across seeds (dim 3)
        V.(arm).var_dxr = var(dxr_loc, 0, 3);   % [N x 3] pointwise ensemble var
        V.(arm).a_xm    = mean(axm_loc, 3);       % [N x 3] ensemble mean a_xm
        V.(arm).a_true  = mean(atr_loc, 3);       % [N x 3] ensemble mean a_true
        V.(arm).Ns      = Ns;
        V.(arm).ok      = ok;

        dxr_stack_all.(arm) = dxr_loc;   % [N x 3 x Ns] retained for scatter

        if opts.verbose
            fprintf('[axm_verify] arm %s: %d / %d non-diverged seeds\n', ...
                    arm, Ns, numel(nz));
        end
    end

    % ================================================================
    % 6. THEORY LINE
    % ================================================================
    % NOTE (rigor point 1, d=2 timing): Var(dx_r[k]) thermal part formally
    % tracks a[k-d] because dx_m = p_d[k-d] - p_m[k] observes dx[k-d].
    % In quasi-static windows (descent, goff): d*Ts = 2/1600 s << EWMA
    % memory (~20 steps = 12.5 ms) and a(t) variation is slow, so
    % a[k-d] ≈ a[k] to < 0.2%.  Theory is built from a_true_ens.A
    % (arm A: a_ctrl = a_true, strict lambda_c pole) without time-shift.
    % A 2-sample forward shift of a_true_ens.A is noted as a refinement
    % for near-wall analysis where a(t) changes rapidly, but is not
    % applied here (effect < 0.5% in descent/goff).
    var_theory = C_dpmr * 4 * kBT .* V.A.a_true + C_n * sig2_n.';  % [N x 3]

    % ================================================================
    % 7. SMOOTHING WINDOW  (mirror fig_motion_var L1259-1260)
    %    Limit to 1/8 osc cycle to preserve within-cycle a(t) shape.
    % ================================================================
    w_mm = max(3, round(min(0.025, 1 / (8 * cfg.frequency)) * fs));

    % ================================================================
    % 8. SCATTER: PER-SEGMENT BLOCK VAR  (rigor point 2)
    % ================================================================
    % Point estimate = segment-mean of the cross-seed pointwise variance
    % V.(arm).var_dxr (SAME estimator as the time-domain figure).  The
    % cross-seed variance removes the seed-common deterministic component
    % (high-passed trajectory residual); a per-seed temporal variance would
    % retain it (matters for arm B near wall).  Error bar = jackknife-over-
    % seeds SEM (seeds are the independent replication units).
    SCALE_NM2 = 1e6;   % um^2 -> nm^2 conversion factor
    sc = struct();

    % --- descent: equal segments (quasi-static a-sweep) ---
    desc_idx    = find(W.desc);
    n_desc      = numel(desc_idx);
    seg_sz      = max(1, floor(n_desc / opts.n_seg));
    n_seg_use   = floor(n_desc / max(seg_sz, 1));
    desc_groups = cell(n_seg_use, 1);
    for seg = 1:n_seg_use
        desc_groups{seg} = desc_idx((seg-1)*seg_sz + 1 : seg*seg_sz);
    end
    sc.desc = scatter_points(desc_groups, V, dxr_stack_all, ARMS, SCALE_NM2);

    % --- osc: per-cycle groups (lighter overlay) ---
    osc_idx    = find(W.osc);
    npc        = round(1 / (cfg.frequency * Ts));   % samples per cycle
    n_full_cyc = floor(numel(osc_idx) / max(npc, 1));
    osc_groups = cell(n_full_cyc, 1);
    for cyc = 1:n_full_cyc
        osc_groups{cyc} = osc_idx((cyc-1)*npc + 1 : cyc*npc);
    end
    sc.osc = scatter_points(osc_groups, V, dxr_stack_all, ARMS, SCALE_NM2);

    % ================================================================
    % 9. PER-WINDOW SUMMARY STATS
    % ================================================================
    % ratio   = mean(Var_meas) / mean(Var_theory)  — target 1.0 for arm A
    %           in descent / goff (quasi-static windows).
    % axm_bias = (mean(a_xm) - mean(a_true)) / mean(a_true) — target ~0 arm A.
    WIN_NAMES = {'desc', 'osc', 'gon', 'goff'};
    win_stats = struct();
    for wi = 1:numel(WIN_NAMES)
        wn  = WIN_NAMES{wi};
        idx = W.(wn);
        if ~any(idx)
            for ai = 1:2
                arm = ARMS{ai};
                win_stats.(arm).(wn).ratio    = nan(1, 3);
                win_stats.(arm).(wn).axm_bias = nan(1, 3);
            end
            continue;
        end
        for ai = 1:2
            arm = ARMS{ai};
            meas  = mean(V.(arm).var_dxr(idx, :), 1);      % [1 x 3]
            th    = mean(var_theory(idx, :),        1);     % [1 x 3]
            axm_m = mean(V.(arm).a_xm(idx, :),     1);     % [1 x 3]
            atr_m = mean(V.(arm).a_true(idx, :),   1);     % [1 x 3]
            win_stats.(arm).(wn).ratio    = meas ./ th;
            win_stats.(arm).(wn).axm_bias = (axm_m - atr_m) ./ atr_m;
        end
    end

    % ================================================================
    % 10. FIGURES  (round-2 locked style)
    % ================================================================
    if opts.save_fig
        % --- Round-2 locked style constants (mirror analyzer L1081-1085) ---
        COL_DES   = [0 0.6 0];                % Desired / a_pd / theory (green)
        COL_TRUE2 = [0.8 0 0];               % a_true / arm A (red)
        COL_HAT2  = [0 0.2 0.9];             % a_hat / arm B (blue)
        COL_MEAS3 = [0.45 0.55 0.95 0.22];  % single-seed a_xm (light blue, transparent)
        FS2   = 18;    % axis label font size
        LFS2  = 14;    % legend font size
        AXLW2 = 2.0;   % axis line width
        COLS  = [1 3]; % display x (col 1) top, z (col 3) bottom
        AXL   = 'xz';  % axis letters for labels

        % nm^2 versions for display
        vdxr_nm2_A = V.A.var_dxr * SCALE_NM2;
        vdxr_nm2_B = V.B.var_dxr * SCALE_NM2;
        vth_nm2    = var_theory   * SCALE_NM2;

        % ---- fig_dxr_var_time (mirrors fig_motion_var L1238-1284) ----
        % Faint raw background (chi-squared scatter) + smoothed main lines
        % + theory green topmost.  Layer convention: â (B) blue thick base,
        % a_true (A) red thin, theory green thickest on top.
        fv = figure('Position', [80 80 1100 720], 'Color', 'w', ...
                    'NumberTitle', 'off', 'Visible', 'off');
        tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
        for r = 1:2
            c = COLS(r); nexttile; hold on;
            vA = vdxr_nm2_A(:, c);
            vB = vdxr_nm2_B(:, c);
            vth = vth_nm2(:, c);

            % faint raw backgrounds (HandleVisibility off: excluded from legend)
            plot(t_e, vB, '-', 'Color', [COL_HAT2  0.25], 'LineWidth', 0.5, ...
                 'HandleVisibility', 'off');
            plot(t_e, vA, '-', 'Color', [COL_TRUE2 0.25], 'LineWidth', 0.5, ...
                 'HandleVisibility', 'off');

            % smoothed main lines
            hB = plot(t_e, movmean(vB,  w_mm), '-', 'Color', COL_HAT2,  ...
                      'LineWidth', 2.5, 'DisplayName', 'a_{hat}-arm (B)');
            hA = plot(t_e, movmean(vA,  w_mm), '-', 'Color', COL_TRUE2, ...
                      'LineWidth', 1.0, 'DisplayName', 'a_{true} (A)');
            hT = plot(t_e, vth,               '-', 'Color', COL_DES,   ...
                      'LineWidth', 3.0, 'DisplayName', 'Theory');

            xlim([0 T_END]);
            ymax_v = max(vth) * 4;
            if ~isfinite(ymax_v) || ymax_v <= 0; ymax_v = 1; end
            ylim([0, ymax_v]);
            ylabel(sprintf('var(dx_{r,%c})  (nm^2)', AXL(r)), ...
                   'FontSize', FS2, 'FontWeight', 'bold');
            if r == 1
                legend([hT hA hB], 'Location', 'northoutside', ...
                       'Orientation', 'horizontal', ...
                       'FontSize', LFS2, 'FontWeight', 'bold', 'Box', 'on');
            end
            if r == 2
                xlabel('Time (sec)', 'FontSize', FS2, 'FontWeight', 'bold');
            end
            set(gca, 'FontSize', FS2, 'FontWeight', 'bold', ...
                     'LineWidth', AXLW2, 'Box', 'on');
            grid off;
        end
        exportgraphics(fv, fullfile(freq_dir, 'fig_dxr_var_time.png'), ...
                       'Resolution', 150);
        close(fv);
        if opts.verbose; fprintf('[axm_verify] wrote fig_dxr_var_time.png\n'); end

        % ---- fig_dxr_var_scatter ----
        % X = a_true (um/pN), Y = Var(dx_r) (nm^2).
        % Descent segments: solid markers with errorbar (arm A red, B blue).
        % Osc per-cycle: lighter semi-transparent markers (no label).
        % Theory line: C_dpmr*4kBT*a + C_n*sig2_n (green LW3).
        fs2 = figure('Position', [80 80 1100 720], 'Color', 'w', ...
                     'NumberTitle', 'off', 'Visible', 'off');
        tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
        for r = 1:2
            c = COLS(r); nexttile; hold on;

            % theory line over the a-range of descent segments + 10% margin
            a_vals = sc.desc.x_a(:, c);
            a_lo   = min(a_vals) * 0.9;
            a_hi   = max(a_vals) * 1.1;
            if a_lo >= a_hi   % guard degenerate range
                a_lo = a_hi * 0.5;
                a_hi = a_hi * 1.5;
            end
            a_line = linspace(a_lo, a_hi, 200);
            y_line = (C_dpmr * 4 * kBT * a_line + C_n * sig2_n(c)) * SCALE_NM2;
            hTh = plot(a_line, y_line, '-', 'Color', COL_DES, ...
                       'LineWidth', 3.0, 'DisplayName', 'Theory');

            % osc per-cycle (lighter, no legend entry)
            if ~isempty(sc.osc.x_a) && size(sc.osc.x_a, 1) > 0
                errorbar(sc.osc.x_a(:, c), sc.osc.y_A(:, c), sc.osc.sem_A(:, c), ...
                         'o', 'Color', [COL_TRUE2 0.4], 'MarkerSize', 4, ...
                         'LineWidth', 0.8, 'HandleVisibility', 'off');
                errorbar(sc.osc.x_a(:, c), sc.osc.y_B(:, c), sc.osc.sem_B(:, c), ...
                         'o', 'Color', [COL_HAT2  0.4], 'MarkerSize', 4, ...
                         'LineWidth', 0.8, 'HandleVisibility', 'off');
            end

            % descent segments (solid filled markers + errorbars)
            hA = errorbar(sc.desc.x_a(:, c), sc.desc.y_A(:, c), sc.desc.sem_A(:, c), ...
                          'o-', 'Color', COL_TRUE2, 'MarkerFaceColor', COL_TRUE2, ...
                          'MarkerSize', 7, 'LineWidth', 1.5, 'DisplayName', 'a_{true} (A)');
            hB = errorbar(sc.desc.x_a(:, c), sc.desc.y_B(:, c), sc.desc.sem_B(:, c), ...
                          's-', 'Color', COL_HAT2,  'MarkerFaceColor', COL_HAT2, ...
                          'MarkerSize', 7, 'LineWidth', 1.5, 'DisplayName', 'a_{hat}-arm (B)');

            ylabel(sprintf('var(dx_{r,%c})  (nm^2)', AXL(r)), ...
                   'FontSize', FS2, 'FontWeight', 'bold');
            if r == 1
                legend([hTh hA hB], 'Location', 'northoutside', ...
                       'Orientation', 'horizontal', ...
                       'FontSize', LFS2, 'FontWeight', 'bold', 'Box', 'on');
            end
            if r == 2
                xlabel('a_{true}  (\mum/pN)', 'FontSize', FS2, 'FontWeight', 'bold');
            end
            set(gca, 'FontSize', FS2, 'FontWeight', 'bold', ...
                     'LineWidth', AXLW2, 'Box', 'on');
            grid off;
        end
        exportgraphics(fs2, fullfile(freq_dir, 'fig_dxr_var_scatter.png'), ...
                       'Resolution', 150);
        close(fs2);
        if opts.verbose; fprintf('[axm_verify] wrote fig_dxr_var_scatter.png\n'); end

        % ---- fig_axm_recover (mirrors fig_gain_compare L1173-1201) ----
        % a_true_ens green LW3 (reference); a_xm_ens arm A red LW2;
        % a_xm_ens arm B blue LW2.  Optional: one seed a_xm background
        % (COL_MEAS3 LW0.9, HandleVisibility off) to show noise scatter.
        ok_pair = find(~[runs.A.noisy.diverged], 1);   % first non-diverged arm A seed
        fg = figure('Position', [80 80 1100 720], 'Color', 'w', ...
                    'NumberTitle', 'off', 'Visible', 'off');
        tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
        glbl = {'a_x', 'a_z'};
        for r = 1:2
            c = COLS(r); nexttile; hold on;

            % optional single-seed a_xm background (arm A, faint light blue)
            if ~isempty(ok_pair)
                axm_seed = runs.A.noisy(ok_pair).simOut.diag.a_xm(2:end, :);
                plot(t_e, axm_seed(:, c), '-', 'Color', COL_MEAS3, ...
                     'LineWidth', 0.9, 'HandleVisibility', 'off');
            end

            % ensemble lines: theory green base, arm A red, arm B blue
            ht = plot(t_e, V.A.a_true(:, c), '-', 'Color', COL_DES,   ...
                      'LineWidth', 3.0, 'DisplayName', 'a_{true}');
            hA = plot(t_e, V.A.a_xm(:, c),   '-', 'Color', COL_TRUE2, ...
                      'LineWidth', 2.0, 'DisplayName', 'a_{xm} (A)');
            hB = plot(t_e, V.B.a_xm(:, c),   '-', 'Color', COL_HAT2,  ...
                      'LineWidth', 2.0, 'DisplayName', 'a_{xm} (B)');

            xlim([0 T_END]);
            ylim([0, 1.25 * max(a_pd(:, c))]);
            ylabel(sprintf('%s  (\\mum/pN)', glbl{r}), ...
                   'FontSize', FS2, 'FontWeight', 'bold');
            if r == 1
                legend([ht hA hB], 'Location', 'northoutside', ...
                       'Orientation', 'horizontal', ...
                       'FontSize', LFS2, 'FontWeight', 'bold', 'Box', 'on');
            end
            if r == 2
                xlabel('Time (sec)', 'FontSize', FS2, 'FontWeight', 'bold');
            end
            set(gca, 'FontSize', FS2, 'FontWeight', 'bold', ...
                     'LineWidth', AXLW2, 'Box', 'on');
            grid off;
        end
        exportgraphics(fg, fullfile(freq_dir, 'fig_axm_recover.png'), ...
                       'Resolution', 150);
        close(fg);
        if opts.verbose; fprintf('[axm_verify] wrote fig_axm_recover.png\n'); end
    end

    % ================================================================
    % 11. SAVE .mat
    % ================================================================
    var_dxr_A    = V.A.var_dxr;
    var_dxr_B    = V.B.var_dxr;
    a_xm_ens_A   = V.A.a_xm;
    a_xm_ens_B   = V.B.a_xm;
    a_true_ens_A = V.A.a_true;
    a_true_ens_B = V.B.a_true;
    scatter_data = sc;
    save(fullfile(freq_dir, 'axm_cdpmr_verify.mat'), ...
         'var_dxr_A', 'var_dxr_B', 'var_theory', ...
         'a_xm_ens_A', 'a_xm_ens_B', ...
         'a_true_ens_A', 'a_true_ens_B', ...
         'scatter_data', 'win_stats', 'C_dpmr', 'C_n', 't_e', 'a_pd');
    if opts.verbose
        fprintf('[axm_verify] wrote axm_cdpmr_verify.mat\n');
    end

    % ================================================================
    % 12. WRITE SUMMARY .md
    % ================================================================
    write_summary_md(fullfile(freq_dir, 'axm_cdpmr_summary.md'), ...
                     cfg, V, win_stats, WIN_NAMES, C_dpmr, C_n);
    if opts.verbose
        fprintf('[axm_verify] wrote axm_cdpmr_summary.md\n');
    end

    % ================================================================
    % 13. PACKAGE RETURN STRUCT
    % ================================================================
    results          = struct();
    results.var_dxr  = V;
    results.var_theory = var_theory;
    results.a_xm_ens = struct('A', V.A.a_xm,   'B', V.B.a_xm);
    results.a_true_ens = struct('A', V.A.a_true, 'B', V.B.a_true);
    results.scatter  = sc;
    results.win_stats = win_stats;
    results.C_dpmr   = C_dpmr;
    results.C_n      = C_n;
    results.t_e      = t_e;
    results.a_pd     = a_pd;
    results.W        = W;

    if opts.verbose
        fprintf('[axm_verify] done -> %s\n', freq_dir);
    end
end   % main function


% ====================================================================
% LOCAL: write_summary_md
% ====================================================================
function write_summary_md(path, cfg, V, win_stats, win_names, C_dpmr, C_n)
%WRITE_SUMMARY_MD Write per-window ratio/bias table to markdown file.
    fid = fopen(path, 'w');
    if fid == -1
        warning('verify_axm_cdpmr_6state:writeErr', ...
                'Cannot open %s for writing; summary skipped.', path);
        return;
    end

    % --- header ---
    n_seeds_A = V.A.Ns;
    n_seeds_B = V.B.Ns;
    if isfield(cfg, 'T_sim')
        t_sim_str = sprintf('%.1f s', cfg.T_sim);
    else
        t_sim_str = '(unknown)';
    end

    fprintf(fid, '# axm / C_dpmr verification : %g Hz\n\n', cfg.frequency);
    fprintf(fid, 'freq=%.4g Hz | arm A = %d seeds | arm B = %d seeds | T_sim = %s\n\n', ...
            cfg.frequency, n_seeds_A, n_seeds_B, t_sim_str);
    fprintf(fid, 'C_dpmr = %.4f (full form, target ~3.16)  |  C_n = %.4f (target ~1.11)\n\n', ...
            C_dpmr, C_n);
    fprintf(fid, ['Identity verified:  Var(dx_r) = C_dpmr * 4kBT * a + C_n * sigma2_n\n', ...
                  'Theory anchor: arm A a_true ensemble (a_ctrl = a_true -> strict lambda_c pole).\n\n']);

    % --- per-window table ---
    fprintf(fid, '## Per-window ratio and a_xm bias\n\n');
    fprintf(fid, '`ratio` = mean(Var_meas) / mean(Var_theory).  Target: arm A desc/goff ≈ 1.0.\n');
    fprintf(fid, '`axm_bias` = (mean(a_xm) - mean(a_true)) / mean(a_true).  Target: arm A desc/goff ≈ 0%%.\n\n');
    fprintf(fid, '| window | arm | ratio_x | ratio_y | ratio_z | bias_x | bias_y | bias_z |\n');
    fprintf(fid, '|--------|-----|---------|---------|---------|--------|--------|--------|\n');
    for wi = 1:numel(win_names)
        wn = win_names{wi};
        for ai = 1:2
            if ai == 1; arm_str = 'A'; else; arm_str = 'B'; end
            r = win_stats.(arm_str).(wn).ratio;
            b = win_stats.(arm_str).(wn).axm_bias;
            fprintf(fid, '| %s | %s | %.3f | %.3f | %.3f | %+.1f%% | %+.1f%% | %+.1f%% |\n', ...
                    wn, arm_str, r(1), r(2), r(3), b(1)*100, b(2)*100, b(3)*100);
        end
    end
    fprintf(fid, '\n');

    % --- interpretation ---
    fprintf(fid, '## Interpretation\n\n');
    fprintf(fid, '### Clean verification windows\n\n');
    fprintf(fid, ['`desc` (descent, quasi-static a-sweep) and `goff` (oscillation, h_bar >= 1.5):\n', ...
                  'C_dpmr quasi-static assumption holds, arm A ratio should be ≈1.0 (±~5%%).\n', ...
                  'These are the primary evidence windows for the identity.\n\n']);
    fprintf(fid, '### Expected deviations (not bugs)\n\n');
    fprintf(fid, ['- **`gon` (gate-on, h_bar < 1.5, near wall)**: rapid a(t) variation and\n', ...
                  '  nonlinear wall correction dynamics invalidate the quasi-static assumption.\n', ...
                  '  Arm A ratio deviating here is expected physics, not a bug.\n', ...
                  '- **`osc` overall**: mixes gon + goff windows; result is dominated by the\n', ...
                  '  gon fraction when the trajectory spends significant time near the wall.\n', ...
                  '- **Arm B, all windows**: systematic ratio deviation and a_xm bias reflect\n', ...
                  '  the self-consistent equilibrium (a_hat != a_true -> closed-loop pole\n', ...
                  '  shifts to ~lambda_c * g -> Var(dx_r) shifts -> a_xm shifts). This is\n', ...
                  '  the expected diagnosis of estimated-arm cost; it is not a failure.\n\n']);
    fprintf(fid, '### Sanity check\n\n');
    fprintf(fid, ['C_dpmr = %.4f (full form used here). If the simplified form (3.96,\n', ...
                  'lambda_c-only) were used instead, arm A a_xm would be systematically\n', ...
                  'high-biased by ~25%% (ratio_xm / ratio_cdpmr = 3.96/3.16 ≈ 1.25).\n'], C_dpmr);
    fclose(fid);
end


% ====================================================================
% LOCAL: scatter_points
% ====================================================================
function out = scatter_points(groups, V, dxr_stack_all, arms, scale)
%SCATTER_POINTS Per-group (a_true, Var(dx_r), SEM) for both arms.
%   groups : cell array of row-index vectors (one per scatter point).
%   Point estimate = group-mean of the cross-seed pointwise variance
%   V.(arm).var_dxr (consistent with the time-domain figure; removes the
%   seed-common deterministic component).  Error bar = jackknife-over-seeds
%   SEM.  x-axis = a_true (arm A ensemble) at the group.
    ng  = numel(groups);
    out = struct('x_a',  zeros(ng, 3), 'y_A',  zeros(ng, 3), 'y_B', zeros(ng, 3), ...
                 'sem_A', zeros(ng, 3), 'sem_B', zeros(ng, 3));
    for g = 1:ng
        rows = groups{g};
        out.x_a(g, :) = mean(V.A.a_true(rows, :), 1);
        for ai = 1:2
            arm = arms{ai};
            for ax = 1:3
                pt  = mean(V.(arm).var_dxr(rows, ax)) * scale;
                stk = reshape(dxr_stack_all.(arm)(rows, ax, :), numel(rows), []);  % [W x Ns]
                se  = jackknife_var_sem(stk) * scale;
                if ai == 1
                    out.y_A(g, ax)   = pt;
                    out.sem_A(g, ax) = se;
                else
                    out.y_B(g, ax)   = pt;
                    out.sem_B(g, ax) = se;
                end
            end
        end
    end
end


% ====================================================================
% LOCAL: jackknife_var_sem
% ====================================================================
function sem = jackknife_var_sem(stk)
%JACKKNIFE_VAR_SEM SEM of the group-mean cross-seed variance, jackknifed
%   over seeds (the independent replication units).
%   stk : [W x Ns] dx_r over a group, one axis, all seeds.
    [W, Ns] = size(stk);
    if Ns < 2 || W < 2
        sem = 0;
        return;
    end
    jk = zeros(1, Ns);
    for si = 1:Ns
        keep   = true(1, Ns);
        keep(si) = false;
        vk     = var(stk(:, keep), 0, 2);   % cross-seed var (leave seed si out), per time
        jk(si) = mean(vk);
    end
    sem = sqrt((Ns - 1) / Ns * sum((jk - mean(jk)).^2));
end
