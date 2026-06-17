function results = verify_axm_cdpmr_6state(freq_dir, opts)
%VERIFY_AXM_CDPMR_6STATE Verify Var(dx_r) = C_dpmr*4kBT*a + C_n*sigma2_n identity.
%
%   results = verify_axm_cdpmr_6state(freq_dir)
%   results = verify_axm_cdpmr_6state(freq_dir, opts)
%
%   freq_dir : path to a single-frequency dir containing runs.mat
%              (e.g. .../test_results/gain_compare/f1Hz)
%   opts     : optional struct
%                opts.save_fig write PNG figures to freq_dir (default true)
%                opts.verbose  print progress (default true)
%
%   Outputs written to test_results/axm_verify/<freq> (or opts.out_dir):
%     fig_dxr_var_time.png    ensemble Var(dx_r) vs time, theory overlay
%     fig_dxr_var_scatter.png Var(dx_r) vs a_true, per-sample cloud + theory line
%     fig_axm_recover.png     a_xm / a_true ensemble, both arms
%     fig_dx_var_time.png     ensemble Var(delta x) vs time, C_dx theory overlay
%     fig_dx_var_scatter.png  Var(delta x) vs a_true, per-sample cloud + theory line
%     axm_cdpmr_verify.mat    all computed arrays
%     axm_cdpmr_summary.md    per-window ratio/bias table (C_dpmr + C_dx)
%
%   Returns results struct: var_dxr, var_dx, var_theory, var_dx_theory,
%   a_xm_ens, a_true_ens, scatter, win_stats, C_dpmr, C_n, C_dx, C_n_fb,
%   t_e, a_pd, W.
%
%   Theory identity (plan D2, full a_pd form):
%     Var(dx_r[k]) = C_dpmr * 4kBT * a[k] + C_n * sigma2_n
%   a=a_true arm (a_ctrl = a_true) = verification baseline (theory strict).
%   a=â arm (a_ctrl = a_hat)  = diagnosis (self-consistent bias expected).
%
%   Design decisions: see plan sleepy-forging-planet.md.
%   Figure style mirrors analyze_gain_6state.m make_figs (locked round-2).
%
%   See also: analyze_gain_6state, compare_gain_6state

    % ----------------------------------------------------------------
    % Input guards
    % ----------------------------------------------------------------
    if nargin < 1 || isempty(freq_dir)
        error('verify_axm_cdpmr_6state:badInput', 'freq_dir is required');
    end
    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'save_fig'); opts.save_fig = true; end
    if ~isfield(opts, 'verbose');  opts.verbose  = true; end
    if ~isfield(opts, 'n_bin');    opts.n_bin    = 28;   end  % a-axis bins for scatter figs

    % ----------------------------------------------------------------
    % Output dir: dedicated test_results/axm_verify/<freq>, kept separate
    % from the gain_compare runs.mat clutter. Override with opts.out_dir.
    % ----------------------------------------------------------------
    if isfield(opts, 'out_dir') && ~isempty(opts.out_dir)
        out_dir = opts.out_dir;
    else
        fd_clean = regexprep(freq_dir, '[/\\]+$', '');      % strip trailing sep
        % NB: fileparts splits on the last dot, so 'f0.5Hz' -> name='f0', ext='.5Hz';
        % concatenate name+ext to recover the true folder name 'f0.5Hz'.
        [src_parent, nm_fp, ex_fp] = fileparts(fd_clean);
        freq_name = [nm_fp, ex_fp];                         % e.g. 'f1Hz' or 'f0.5Hz'
        out_dir = fullfile(fileparts(src_parent), 'axm_verify', freq_name);
    end
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end

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
        'verify_axm_cdpmr_6state: all a=a_true arm noisy seeds diverged');
    so_ref = runs.A.noisy(ok_ref).simOut;

    assert(isfield(so_ref, 'diag'), ...
        ['verify_axm_cdpmr_6state: simOut.diag missing. ', ...
         'Re-run compare_gain_6state with collect_diag=true.']);
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

    % R22 constants: R2_intrinsic = K_var * IF_eff(a,sig2_n) * (a + xi)^2
    %   K_var  = 2*a_cov/(2-a_cov)          (EWMA gain x Isserlis 2sigma^4)
    %   IF_abc = [A;B;C]                    (s-weighted autocorr sums -> per-step IF_eff)
    %   xi     = (C_n/C_dpmr)*sig2_n/(4kBT) (per-axis sensor-noise floor)
    % Pulled from the SAME stored ctrl_const the controller used (self-consistent).
    cc = so_ref.ctrl_const;
    assert(all(isfield(cc, {'K_var', 'IF_abc', 'xi_per_axis'})), ...
        ['verify_axm_cdpmr_6state: ctrl_const missing R22 fields ', ...
         '(K_var / IF_abc / xi_per_axis). Re-run sim with build_eq17_6state_constants.']);
    K_var  = cc.K_var;
    IF_abc = cc.IF_abc(:);             % [A;B;C]
    xi_ax  = cc.xi_per_axis(:).';      % [1x3] per-axis sensor floor

    if opts.verbose
        fprintf('[axm_verify] C_dpmr=%.4f  C_n=%.4f  kBT=%.4e  R=%.4g um  Ts=%.6g s\n', ...
                C_dpmr, C_n, kBT, R_phys, Ts);
        fprintf('[axm_verify] R22: K_var=%.5f  IF_abc=[%.3g %.3g %.3g]  xi=[%.3g %.3g %.3g]\n', ...
                K_var, IF_abc(1), IF_abc(2), IF_abc(3), xi_ax(1), xi_ax(2), xi_ax(3));
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
    osc_dur   = cfg.n_cycles / cfg.frequency;
    t_osc1    = t_osc0 + osc_dur;
    t_discard = min(1 / cfg.frequency, 0.5 * osc_dur);  % drop 1st cycle, cap at half osc (single-cycle support; mirrors analyzer)
    H_BAR_GATE = 1.5;                  % = ctrl_const.h_bar_safe

    W.desc = (t_e >= cfg.t_hold       & t_e < t_osc0);
    W.osc  = (t_e >= t_osc0+t_discard & t_e < t_osc1);
    W.near  = W.osc & (h_bar_d < H_BAR_GATE);
    W.far = W.osc & (h_bar_d >= H_BAR_GATE);

    if opts.verbose
        fprintf('[axm_verify] windows: desc=%d  osc=%d  near=%d  far=%d samples\n', ...
                sum(W.desc), sum(W.osc), sum(W.near), sum(W.far));
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
    % Contrast: the analyzer's ram subtracts a finite-sample det_traj,
    % introducing a 1-1/Ns self-subtraction deflation that must be corrected.
    % Here no such subtraction occurs, so no extra correction is needed.
    ARMS   = {'A', 'B'};
    V      = struct();
    stacks = struct();   % per-seed raw stacks [N x 3 x Ns], needed for jackknife error bars

    for ai = 1:2
        arm = ARMS{ai};
        nz  = runs.(arm).noisy;
        ok  = find(~[nz.diverged]);
        if isempty(ok)
            error('verify_axm_cdpmr_6state:allDiverged', ...
                  'All arm %s noisy seeds diverged', arm);
        end
        Ns = numel(ok);

        dxr_loc   = zeros(N, 3, Ns);
        axm_loc   = zeros(N, 3, Ns);
        atr_loc   = zeros(N, 3, Ns);
        ptrue_loc = zeros(N, 3, Ns);
        for si = 1:Ns
            s = ok(si);
            dxr_loc(:, :, si)   = nz(s).simOut.diag.dx_r(2:end, :);
            axm_loc(:, :, si)   = nz(s).simOut.diag.a_xm(2:end, :);
            atr_loc(:, :, si)   = nz(s).simOut.a_true_out(2:end, :);
            ptrue_loc(:, :, si) = nz(s).simOut.p_true_out(1:end-1, :);
        end

        % var(X,0,3): Bessel-corrected sample variance across seeds (dim 3)
        V.(arm).var_dxr = var(dxr_loc,   0, 3);   % [N x 3] pointwise ensemble var
        V.(arm).var_dx  = var(ptrue_loc, 0, 3);   % [N x 3] true-motion var (p_d cancels)
        V.(arm).var_axm = var(axm_loc,   0, 3);   % [N x 3] cross-seed Var(a_xm) = R2_intrinsic
        V.(arm).a_xm    = mean(axm_loc,  3);       % [N x 3] ensemble mean a_xm
        V.(arm).a_true  = mean(atr_loc,  3);       % [N x 3] ensemble mean a_true
        V.(arm).Ns      = Ns;
        V.(arm).ok      = ok;

        % per-seed raw stacks for scatter-plot jackknife error bars
        stacks.(arm).dxr   = dxr_loc;    % [N x 3 x Ns] per-seed dx_r signal
        stacks.(arm).ptrue = ptrue_loc;  % [N x 3 x Ns] per-seed p_true signal
        stacks.(arm).axm   = axm_loc;    % [N x 3 x Ns] per-seed a_xm signal (R22 jackknife)
        stacks.(arm).Ns    = Ns;

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
    % In quasi-static windows (descent, far): d*Ts = 2/1600 s << EWMA
    % memory (~20 steps = 12.5 ms) and a(t) variation is slow, so
    % a[k-d] ≈ a[k] to < 0.2%.  Theory is built from a_true_ens.A
    % (a=a_true arm: a_ctrl = a_true, strict lambda_c pole) without time-shift.
    % A 2-sample forward shift of a_true_ens.A is noted as a refinement
    % for near-wall analysis where a(t) changes rapidly, but is not
    % applied here (effect < 0.5% in descent/far).
    var_theory = C_dpmr * 4 * kBT .* V.A.a_true + C_n * sig2_n.';  % [N x 3]

    % C_dx / C_n_fb: true-motion tracking error (delta x = p_d - p_true) theory.
    % C_dx  = 2 + 1/(1-lc^2)      ~ 3.96 for lambda_c = 0.7
    % C_n_fb = (1-lc)/(1+lc)      ~ 0.176 for lambda_c = 0.7
    lc     = cfg.lambda_c;
    C_dx   = 2 + 1 / (1 - lc^2);
    C_n_fb = (1 - lc) / (1 + lc);
    var_dx_theory = C_dx * 4 * kBT .* V.A.a_true + C_n_fb * sig2_n.';  % [N x 3]

    if opts.verbose
        fprintf('[axm_verify] C_dx=%.4f  C_n_fb=%.4f\n', C_dx, C_n_fb);
    end

    % R22 theory: R2_intrinsic = K_var * IF_eff(a, sig2_n) * (a + xi)^2.
    % IF_eff is the per-step colored-noise inflation (exact, via IF_abc); it is
    % time-varying through a. Anchored to the a=a_true arm ensemble a_true (same
    % anchor as var_theory / var_dx_theory). This is the RAW a_xm-signal variance
    % (the +sum-of-past-Q55 delay term in the controller's R2_eff is an effective-R
    % correction for the d-step measurement lag and is NOT present in Var(a_xm)).
    IF_mat = zeros(N, 3);
    for c = 1:3
        IF_mat(:, c) = if_eff_eval(IF_abc, C_dpmr, C_n, kBT, V.A.a_true(:, c), sig2_n(c));
    end
    var_axm_theory = K_var .* IF_mat .* (V.A.a_true + xi_ax).^2;   % [N x 3]

    % ================================================================
    % 7. SMOOTHING WINDOW  (mirror fig_motion_var L1259-1260)
    %    Limit to 1/8 osc cycle to preserve within-cycle a(t) shape.
    % ================================================================
    w_mm = max(3, round(min(0.025, 1 / (8 * cfg.frequency)) * fs));

    % ================================================================
    % 8. SCATTER MASK
    % ================================================================
    SCALE_NM2 = 1e6;   % um^2 -> nm^2 conversion factor
    sc_mask = W.desc | W.osc;   % a-varying region for per-sample scatter

    % ================================================================
    % 9. PER-WINDOW SUMMARY STATS
    % ================================================================
    % ratio   = mean(Var_meas) / mean(Var_theory)  — target 1.0 for a=a_true arm
    %           in descent / far (quasi-static windows).
    % axm_bias = (mean(a_xm) - mean(a_true)) / mean(a_true) — target ~0 for a=a_true arm.
    WIN_NAMES = {'desc', 'osc', 'near', 'far'};
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
        COL_TRUE2 = [0.8 0 0];               % a_true / a=a_true arm (red)
        COL_HAT2  = [0 0.2 0.9];             % a_hat / a=â arm (blue)
        COL_MEAS3 = [0.45 0.55 0.95 0.22];  % single-seed a_xm (light blue, transparent)
        FS2   = 18;    % axis label font size
        LFS2  = 14;    % legend font size
        AXLW2 = 2.0;   % axis line width
        COLS  = [1 3]; % display x (col 1) top, z (col 3) bottom
        AXL   = 'xz';  % axis letters for labels

        % Unified scatter x-axis: one shared a-range across the x (col 1) and
        % z (col 3) panels, both arms, restricted to the scatter mask. Every
        % scatter figure then uses identical, aligned, plain-decimal x ticks.
        ascat_all = [V.A.a_true(sc_mask, [1 3]); V.B.a_true(sc_mask, [1 3])];
        if isempty(ascat_all) || ~(min(ascat_all(:)) < max(ascat_all(:)))
            a_lo_g = 0; a_hi_g = 1;
        else
            a_lo_g = min(ascat_all(:)) * 0.95;
            a_hi_g = max(ascat_all(:)) * 1.05;
        end
        a_line_g = linspace(a_lo_g, a_hi_g, 200);

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
                      'LineWidth', 2.5, 'DisplayName', 'â');
            hA = plot(t_e, movmean(vA,  w_mm), '-', 'Color', COL_TRUE2, ...
                      'LineWidth', 1.0, 'DisplayName', 'a_{true}');
            hT = plot(t_e, vth,               '-', 'Color', COL_DES,   ...
                      'LineWidth', 3.0, 'DisplayName', 'Theory');

            xlim([0 T_END]);
            ymax_v = 1.15 * max([max(movmean(vA, w_mm)), max(movmean(vB, w_mm)), max(vth)]);
            if ~isfinite(ymax_v) || ymax_v <= 0; ymax_v = 1; end
            ylim([0, ymax_v]);
            ylabel(sprintf('var(d_{%cr})  (nm^2)', AXL(r)), ...
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
        exportgraphics(fv, fullfile(out_dir, 'fig_dxr_var_time.png'), ...
                       'Resolution', 150);
        close(fv);
        if opts.verbose; fprintf('[axm_verify] wrote fig_dxr_var_time.png\n'); end

        % ---- fig_dxr_var_scatter ----
        % Binned points (opts.n_bin a-value bins per arm) + jackknife SEM error bars.
        % X = a_true (um/pN), Y = Var(dx_r) (nm^2).
        % Theory line: C_dpmr*4kBT*a + C_n*sig2_n (green LW3).
        % a=a_true arm: red circles ('o'); a=â arm: blue squares ('s').
        fs2 = figure('Position', [80 80 1100 720], 'Color', 'w', ...
                     'NumberTitle', 'off', 'Visible', 'off');
        tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
        for r = 1:2
            c = COLS(r); nexttile; hold on;

            % binned statistics for each arm on its own a-axis
            B_A = a_bin_points(V.A.a_true(:,c), V.A.var_dxr(:,c), ...
                               reshape(stacks.A.dxr(:,c,:), N, []), ...
                               sc_mask, opts.n_bin, SCALE_NM2);
            B_B = a_bin_points(V.B.a_true(:,c), V.B.var_dxr(:,c), ...
                               reshape(stacks.B.dxr(:,c,:), N, []), ...
                               sc_mask, opts.n_bin, SCALE_NM2);

            % theory drawn across the shared global a-range (aligned x-axes)
            a_line = a_line_g;   % shared global a-range -> theory across full aligned axis
            y_line = (C_dpmr * 4 * kBT * a_line + C_n * sig2_n(c)) * SCALE_NM2;
            hTh = plot(a_line, y_line, '-', 'Color', COL_DES, ...
                       'LineWidth', 3.0, 'DisplayName', 'Theory');

            % a=a_true arm: red circles
            hA = errorbar(B_A.x, B_A.y, B_A.sem, 'o', ...
                          'Color', COL_TRUE2, 'MarkerFaceColor', COL_TRUE2, ...
                          'MarkerSize', 6, 'LineWidth', 1.0, 'CapSize', 3, ...
                          'LineStyle', 'none', 'DisplayName', 'a_{true}');

            % a=â arm: blue squares
            hB = errorbar(B_B.x, B_B.y, B_B.sem, 's', ...
                          'Color', COL_HAT2, 'MarkerFaceColor', COL_HAT2, ...
                          'MarkerSize', 6, 'LineWidth', 1.0, 'CapSize', 3, ...
                          'LineStyle', 'none', 'DisplayName', 'â');

            ylabel(sprintf('var(d_{%cr})  (nm^2)', AXL(r)), ...
                   'FontSize', FS2, 'FontWeight', 'bold');
            if r == 1
                legend([hTh hA hB], 'Location', 'northoutside', ...
                       'Orientation', 'horizontal', ...
                       'FontSize', LFS2, 'FontWeight', 'bold', 'Box', 'on');
            end
            if r == 2
                xlabel('a_{true}  (\mum/pN)', 'FontSize', FS2, 'FontWeight', 'bold');
            end
            xlim([a_lo_g a_hi_g]);             % shared, aligned x-range (both panels)
            ax = gca;
            ax.XAxis.Exponent = 0;             % plain decimals, no x10^-3 offset
            xtickformat('%.3f');               % identical decimal format on both panels
            set(gca, 'FontSize', FS2, 'FontWeight', 'bold', ...
                     'LineWidth', AXLW2, 'Box', 'on');
            grid off;
        end
        exportgraphics(fs2, fullfile(out_dir, 'fig_dxr_var_scatter.png'), ...
                       'Resolution', 150);
        close(fs2);
        if opts.verbose; fprintf('[axm_verify] wrote fig_dxr_var_scatter.png\n'); end

        % ---- fig_axm_recover (mirrors fig_gain_compare L1173-1201) ----
        % a_true_ens green LW3 (reference); a_xm_ens a=a_true arm red LW2;
        % a_xm_ens a=â arm blue LW2.  Optional: one seed a_xm background
        % (COL_MEAS3 LW0.9, HandleVisibility off) to show noise scatter.
        ok_pair = find(~[runs.B.noisy.diverged], 1);   % first non-diverged a=â arm seed (background single-seed)
        fg = figure('Position', [80 80 1100 720], 'Color', 'w', ...
                    'NumberTitle', 'off', 'Visible', 'off');
        tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
        glbl = {'a_{mx}', 'a_{mz}'};
        for r = 1:2
            c = COLS(r); nexttile; hold on;

            % optional single-seed a_xm background (a=â arm, faint light blue -> matches blue ensemble)
            if ~isempty(ok_pair)
                axm_seed = runs.B.noisy(ok_pair).simOut.diag.a_xm(2:end, :);
                plot(t_e, axm_seed(:, c), '-', 'Color', COL_MEAS3, ...
                     'LineWidth', 0.9, 'HandleVisibility', 'off');
            end

            % ensemble lines: theory green base, a=a_true arm red, a=â arm blue
            ht = plot(t_e, V.A.a_true(:, c), '-', 'Color', COL_DES,   ...
                      'LineWidth', 3.0, 'DisplayName', 'a_{true}');
            hA = plot(t_e, V.A.a_xm(:, c),   '-', 'Color', COL_TRUE2, ...
                      'LineWidth', 2.0, 'DisplayName', 'a_m(a_{true})');
            hB = plot(t_e, V.B.a_xm(:, c),   '-', 'Color', COL_HAT2,  ...
                      'LineWidth', 2.0, 'DisplayName', 'a_m(â)');

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
        exportgraphics(fg, fullfile(out_dir, 'fig_axm_recover.png'), ...
                       'Resolution', 150);
        close(fg);
        if opts.verbose; fprintf('[axm_verify] wrote fig_axm_recover.png\n'); end

        % ---- fig_dx_var_time ----
        % True-motion tracking error Var(delta x) vs time.
        % Same layer convention as fig_dxr_var_time: faint raw + smoothed + theory.
        % Theory: C_dx*4kBT*a + C_n_fb*sigma2_n (green LW3).
        dx_ylab   = {'\delta x', '\delta z'};
        vdx_nm2_A = V.A.var_dx * SCALE_NM2;
        vdx_nm2_B = V.B.var_dx * SCALE_NM2;
        vdxth_nm2 = var_dx_theory * SCALE_NM2;

        fdx = figure('Position', [80 80 1100 720], 'Color', 'w', ...
                     'NumberTitle', 'off', 'Visible', 'off');
        tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
        for r = 1:2
            c = COLS(r); nexttile; hold on;
            vA  = vdx_nm2_A(:, c);
            vB  = vdx_nm2_B(:, c);
            vth = vdxth_nm2(:, c);

            % faint raw backgrounds
            plot(t_e, vB, '-', 'Color', [COL_HAT2  0.25], 'LineWidth', 0.5, ...
                 'HandleVisibility', 'off');
            plot(t_e, vA, '-', 'Color', [COL_TRUE2 0.25], 'LineWidth', 0.5, ...
                 'HandleVisibility', 'off');

            % smoothed main lines
            hB = plot(t_e, movmean(vB, w_mm), '-', 'Color', COL_HAT2,  ...
                      'LineWidth', 2.5, 'DisplayName', 'â');
            hA = plot(t_e, movmean(vA, w_mm), '-', 'Color', COL_TRUE2, ...
                      'LineWidth', 1.0, 'DisplayName', 'a_{true}');
            hT = plot(t_e, vth,               '-', 'Color', COL_DES,   ...
                      'LineWidth', 3.0, 'DisplayName', 'Theory');

            xlim([0 T_END]);
            ymax_v = 1.15 * max([max(movmean(vA, w_mm)), max(movmean(vB, w_mm)), max(vth)]);
            if ~isfinite(ymax_v) || ymax_v <= 0; ymax_v = 1; end
            ylim([0, ymax_v]);
            ylabel(sprintf('var(%s)  (nm^2)', dx_ylab{r}), ...
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
        exportgraphics(fdx, fullfile(out_dir, 'fig_dx_var_time.png'), ...
                       'Resolution', 150);
        close(fdx);
        if opts.verbose; fprintf('[axm_verify] wrote fig_dx_var_time.png\n'); end

        % ---- fig_dx_var_scatter ----
        % Binned points (opts.n_bin a-value bins per arm) + jackknife SEM error bars.
        % X = a_true (um/pN), Y = Var(delta x) (nm^2).
        % Theory: C_dx*4kBT*a + C_n_fb*sigma2_n (green LW3).
        % a=a_true arm: red circles ('o'); a=â arm: blue squares ('s').
        fdxs = figure('Position', [80 80 1100 720], 'Color', 'w', ...
                      'NumberTitle', 'off', 'Visible', 'off');
        tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
        for r = 1:2
            c = COLS(r); nexttile; hold on;

            % binned statistics for each arm on its own a-axis
            B_A = a_bin_points(V.A.a_true(:,c), V.A.var_dx(:,c), ...
                               reshape(stacks.A.ptrue(:,c,:), N, []), ...
                               sc_mask, opts.n_bin, SCALE_NM2);
            B_B = a_bin_points(V.B.a_true(:,c), V.B.var_dx(:,c), ...
                               reshape(stacks.B.ptrue(:,c,:), N, []), ...
                               sc_mask, opts.n_bin, SCALE_NM2);

            % theory drawn across the shared global a-range (aligned x-axes)
            a_line = a_line_g;   % shared global a-range -> theory across full aligned axis
            y_line = (C_dx * 4 * kBT * a_line + C_n_fb * sig2_n(c)) * SCALE_NM2;
            hTh = plot(a_line, y_line, '-', 'Color', COL_DES, ...
                       'LineWidth', 3.0, 'DisplayName', 'Theory');

            % a=a_true arm: red circles
            hA = errorbar(B_A.x, B_A.y, B_A.sem, 'o', ...
                          'Color', COL_TRUE2, 'MarkerFaceColor', COL_TRUE2, ...
                          'MarkerSize', 6, 'LineWidth', 1.0, 'CapSize', 3, ...
                          'LineStyle', 'none', 'DisplayName', 'a_{true}');

            % a=â arm: blue squares
            hB = errorbar(B_B.x, B_B.y, B_B.sem, 's', ...
                          'Color', COL_HAT2, 'MarkerFaceColor', COL_HAT2, ...
                          'MarkerSize', 6, 'LineWidth', 1.0, 'CapSize', 3, ...
                          'LineStyle', 'none', 'DisplayName', 'â');

            ylabel(sprintf('var(%s)', dx_ylab{r}), ...
                   'FontSize', FS2, 'FontWeight', 'bold');
            if r == 1
                legend([hTh hA hB], 'Location', 'northoutside', ...
                       'Orientation', 'horizontal', ...
                       'FontSize', LFS2, 'FontWeight', 'bold', 'Box', 'on');
            end
            if r == 2
                xlabel('a_{true}  (\mum/pN)', 'FontSize', FS2, 'FontWeight', 'bold');
            end
            xlim([a_lo_g a_hi_g]);             % shared, aligned x-range (both panels)
            ax = gca;
            ax.XAxis.Exponent = 0;             % plain decimals, no x10^-3 offset
            xtickformat('%.3f');               % identical decimal format on both panels
            set(gca, 'FontSize', FS2, 'FontWeight', 'bold', ...
                     'LineWidth', AXLW2, 'Box', 'on');
            grid off;
        end
        exportgraphics(fdxs, fullfile(out_dir, 'fig_dx_var_scatter.png'), ...
                       'Resolution', 150);
        close(fdxs);
        if opts.verbose; fprintf('[axm_verify] wrote fig_dx_var_scatter.png\n'); end

        % ---- fig_axm_var_time ----
        % R22: cross-seed Var(a_xm) vs time. Same layer convention as the
        % var_dxr/var_dx figs (faint raw + smoothed main + theory green top).
        % Theory: K_var * IF_eff(a,sig2_n) * (a + xi)^2 (= R2_intrinsic, green LW3).
        axm_ylab   = {'a_{mx}', 'a_{mz}'};
        vaxm_nm2_A = V.A.var_axm    * SCALE_NM2;   % (nm/pN)^2
        vaxm_nm2_B = V.B.var_axm    * SCALE_NM2;
        vaxmth_nm2 = var_axm_theory * SCALE_NM2;

        fav = figure('Position', [80 80 1100 720], 'Color', 'w', ...
                     'NumberTitle', 'off', 'Visible', 'off');
        tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
        for r = 1:2
            c = COLS(r); nexttile; hold on;
            vA  = vaxm_nm2_A(:, c);
            vB  = vaxm_nm2_B(:, c);
            vth = vaxmth_nm2(:, c);

            % faint raw backgrounds
            plot(t_e, vB, '-', 'Color', [COL_HAT2  0.25], 'LineWidth', 0.5, ...
                 'HandleVisibility', 'off');
            plot(t_e, vA, '-', 'Color', [COL_TRUE2 0.25], 'LineWidth', 0.5, ...
                 'HandleVisibility', 'off');

            % smoothed main lines
            hB = plot(t_e, movmean(vB, w_mm), '-', 'Color', COL_HAT2,  ...
                      'LineWidth', 2.5, 'DisplayName', 'â');
            hA = plot(t_e, movmean(vA, w_mm), '-', 'Color', COL_TRUE2, ...
                      'LineWidth', 1.0, 'DisplayName', 'a_{true}');
            hT = plot(t_e, vth,               '-', 'Color', COL_DES,   ...
                      'LineWidth', 3.0, 'DisplayName', 'Theory');

            xlim([0 T_END]);
            ymax_v = 1.15 * max([max(movmean(vA, w_mm)), max(movmean(vB, w_mm)), max(vth)]);
            if ~isfinite(ymax_v) || ymax_v <= 0; ymax_v = 1; end
            ylim([0, ymax_v]);
            ylabel(sprintf('var(%s)  ((nm/pN)^2)', axm_ylab{r}), ...
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
        exportgraphics(fav, fullfile(out_dir, 'fig_axm_var_time.png'), ...
                       'Resolution', 150);
        close(fav);
        if opts.verbose; fprintf('[axm_verify] wrote fig_axm_var_time.png\n'); end

        % ---- fig_axm_var_scatter ----
        % R22: Var(a_xm) vs a, a-binned + jackknife SEM. Theory line:
        % K_var * IF_eff(a,sig2_n) * (a + xi)^2 (green LW3). a=a_true red 'o',
        % a=â blue 's'. log-log shape: flat sensor floor (small a) -> a^2 thermal.
        favs = figure('Position', [80 80 1100 720], 'Color', 'w', ...
                      'NumberTitle', 'off', 'Visible', 'off');
        tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
        for r = 1:2
            c = COLS(r); nexttile; hold on;

            B_A = a_bin_points(V.A.a_true(:,c), V.A.var_axm(:,c), ...
                               reshape(stacks.A.axm(:,c,:), N, []), ...
                               sc_mask, opts.n_bin, SCALE_NM2);
            B_B = a_bin_points(V.B.a_true(:,c), V.B.var_axm(:,c), ...
                               reshape(stacks.B.axm(:,c,:), N, []), ...
                               sc_mask, opts.n_bin, SCALE_NM2);

            a_line = a_line_g;   % shared global a-range -> theory across full aligned axis
            IF_line = if_eff_eval(IF_abc, C_dpmr, C_n, kBT, a_line, sig2_n(c));
            y_line  = (K_var * IF_line .* (a_line + xi_ax(c)).^2) * SCALE_NM2;
            hTh = plot(a_line, y_line, '-', 'Color', COL_DES, ...
                       'LineWidth', 3.0, 'DisplayName', 'Theory');

            hA = errorbar(B_A.x, B_A.y, B_A.sem, 'o', ...
                          'Color', COL_TRUE2, 'MarkerFaceColor', COL_TRUE2, ...
                          'MarkerSize', 6, 'LineWidth', 1.0, 'CapSize', 3, ...
                          'LineStyle', 'none', 'DisplayName', 'a_{true}');
            hB = errorbar(B_B.x, B_B.y, B_B.sem, 's', ...
                          'Color', COL_HAT2, 'MarkerFaceColor', COL_HAT2, ...
                          'MarkerSize', 6, 'LineWidth', 1.0, 'CapSize', 3, ...
                          'LineStyle', 'none', 'DisplayName', 'â');

            ylabel(sprintf('var(%s)  ((nm/pN)^2)', axm_ylab{r}), ...
                   'FontSize', FS2, 'FontWeight', 'bold');
            if r == 1
                legend([hTh hA hB], 'Location', 'northoutside', ...
                       'Orientation', 'horizontal', ...
                       'FontSize', LFS2, 'FontWeight', 'bold', 'Box', 'on');
            end
            if r == 2
                xlabel('a_{true}  (\mum/pN)', 'FontSize', FS2, 'FontWeight', 'bold');
            end
            xlim([a_lo_g a_hi_g]);             % shared, aligned x-range (both panels)
            ax = gca;
            ax.XAxis.Exponent = 0;             % plain decimals, no x10^-3 offset
            xtickformat('%.3f');               % identical decimal format on both panels
            set(gca, 'FontSize', FS2, 'FontWeight', 'bold', ...
                     'LineWidth', AXLW2, 'Box', 'on');
            grid off;
        end
        exportgraphics(favs, fullfile(out_dir, 'fig_axm_var_scatter.png'), ...
                       'Resolution', 150);
        close(favs);
        if opts.verbose; fprintf('[axm_verify] wrote fig_axm_var_scatter.png\n'); end
    end

    % ================================================================
    % 11. SAVE .mat
    % ================================================================
    var_dxr_A    = V.A.var_dxr;
    var_dxr_B    = V.B.var_dxr;
    var_dx_A     = V.A.var_dx;
    var_dx_B     = V.B.var_dx;
    var_axm_A    = V.A.var_axm;
    var_axm_B    = V.B.var_axm;
    a_xm_ens_A   = V.A.a_xm;
    a_xm_ens_B   = V.B.a_xm;
    a_true_ens_A = V.A.a_true;
    a_true_ens_B = V.B.a_true;
    save(fullfile(out_dir, 'axm_cdpmr_verify.mat'), ...
         'var_dxr_A', 'var_dxr_B', 'var_theory', ...
         'var_dx_A',  'var_dx_B',  'var_dx_theory', ...
         'var_axm_A', 'var_axm_B', 'var_axm_theory', ...
         'a_xm_ens_A', 'a_xm_ens_B', ...
         'a_true_ens_A', 'a_true_ens_B', ...
         'win_stats', 'C_dpmr', 'C_n', 'C_dx', 'C_n_fb', ...
         'K_var', 'IF_abc', 'xi_ax', 't_e', 'a_pd');
    if opts.verbose
        fprintf('[axm_verify] wrote axm_cdpmr_verify.mat\n');
    end

    % ================================================================
    % 12. WRITE SUMMARY .md
    % ================================================================
    % C_dx ratio: mean(Var(delta x)_meas) / mean(Var(delta x)_theory), a=a_true arm.
    CDX_WINS = {'desc', 'osc'};
    cdx_ratio = struct();
    for wi = 1:numel(CDX_WINS)
        wn  = CDX_WINS{wi};
        idx = W.(wn);
        if any(idx)
            cdx_ratio.(wn) = mean(V.A.var_dx(idx, :), 1) ./ ...
                             mean(var_dx_theory(idx, :), 1);   % [1 x 3]
        else
            cdx_ratio.(wn) = nan(1, 3);
        end
    end
    % R22 ratio: mean(Var(a_xm)_meas) / mean(R2_intrinsic_theory), a=a_true arm,
    % over all 4 windows (near-wall is where the chi-sq floor / IF_eff regime matters).
    r22 = struct('K_var', K_var, 'xi', xi_ax, 'ratio', struct());
    for wi = 1:numel(WIN_NAMES)
        wn  = WIN_NAMES{wi};
        idx = W.(wn);
        if any(idx)
            r22.ratio.(wn) = mean(V.A.var_axm(idx, :), 1) ./ ...
                             mean(var_axm_theory(idx, :), 1);   % [1 x 3]
        else
            r22.ratio.(wn) = nan(1, 3);
        end
    end

    write_summary_md(fullfile(out_dir, 'axm_cdpmr_summary.md'), ...
                     cfg, V, win_stats, WIN_NAMES, C_dpmr, C_n, C_dx, C_n_fb, ...
                     cdx_ratio, r22);
    if opts.verbose
        fprintf('[axm_verify] wrote axm_cdpmr_summary.md\n');
    end

    % ================================================================
    % 13. PACKAGE RETURN STRUCT
    % ================================================================
    results              = struct();
    results.var_dxr      = V;
    results.var_theory   = var_theory;
    results.var_dx_theory = var_dx_theory;
    results.var_axm_theory = var_axm_theory;
    results.a_xm_ens     = struct('A', V.A.a_xm,   'B', V.B.a_xm);
    results.a_true_ens   = struct('A', V.A.a_true,  'B', V.B.a_true);
    results.scatter      = struct('a_true',   V.A.a_true, ...
                                  'var_dxr_A', V.A.var_dxr, ...
                                  'var_dxr_B', V.B.var_dxr, ...
                                  'var_dx_A',  V.A.var_dx, ...
                                  'var_dx_B',  V.B.var_dx, ...
                                  'var_axm_A', V.A.var_axm, ...
                                  'var_axm_B', V.B.var_axm, ...
                                  'mask',      sc_mask);
    results.r22          = r22;
    results.win_stats    = win_stats;
    results.C_dpmr       = C_dpmr;
    results.C_n          = C_n;
    results.C_dx         = C_dx;
    results.C_n_fb       = C_n_fb;
    results.t_e          = t_e;
    results.a_pd         = a_pd;
    results.W            = W;

    if opts.verbose
        fprintf('[axm_verify] done -> %s\n', out_dir);
    end
end   % main function


% ====================================================================
% LOCAL: write_summary_md
% ====================================================================
function write_summary_md(path, cfg, V, win_stats, win_names, C_dpmr, C_n, C_dx, C_n_fb, cdx_ratio, r22)
%WRITE_SUMMARY_MD Write per-window ratio/bias table to markdown file.
%   cdx_ratio : struct with fields .desc [1x3] and .osc [1x3] giving
%               mean(Var(delta x)) / mean(Var(delta x)_theory) per axis.
%   r22       : struct with .K_var, .xi [1x3], .ratio.<win> [1x3] giving
%               mean(Var(a_xm)) / mean(R2_intrinsic_theory) per axis per window.
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
    fprintf(fid, 'freq=%.4g Hz | a=a_true = %d seeds | a=â = %d seeds | T_sim = %s\n\n', ...
            cfg.frequency, n_seeds_A, n_seeds_B, t_sim_str);
    fprintf(fid, 'C_dpmr = %.4f (full form, target ~3.16)  |  C_n = %.4f (target ~1.11)\n', ...
            C_dpmr, C_n);
    fprintf(fid, 'C_dx = %.4f (tracking-error, target ~3.96)  |  C_n_fb = %.4f (target ~0.176)\n\n', ...
            C_dx, C_n_fb);
    fprintf(fid, ['Identity verified:  Var(dx_r) = C_dpmr * 4kBT * a + C_n * sigma2_n\n', ...
                  'Theory anchor: a=a_true arm ensemble (a_ctrl = a_true -> strict lambda_c pole).\n\n']);

    % --- headline: C_dpmr — does Var(dx_r) match? ---
    hd_d = win_stats.A.desc;
    hd_o = win_stats.A.osc;
    fprintf(fid, '## Headline - does Var(dx_r) match the formula?\n\n');
    fprintf(fid, 'Verification arm **a=a_true**, per axis [x y z]  (target: ratio 1.0, bias 0%%):\n\n');
    fprintf(fid, '| window | ratio_x | ratio_y | ratio_z | bias_x | bias_y | bias_z |\n');
    fprintf(fid, '|--------|---------|---------|---------|--------|--------|--------|\n');
    fprintf(fid, '| desc (clean quasi-static) | %.3f | %.3f | %.3f | %+.1f%% | %+.1f%% | %+.1f%% |\n', ...
            hd_d.ratio(1), hd_d.ratio(2), hd_d.ratio(3), ...
            hd_d.axm_bias(1)*100, hd_d.axm_bias(2)*100, hd_d.axm_bias(3)*100);
    fprintf(fid, '| osc (full, dynamic)       | %.3f | %.3f | %.3f | %+.1f%% | %+.1f%% | %+.1f%% |\n\n', ...
            hd_o.ratio(1), hd_o.ratio(2), hd_o.ratio(3), ...
            hd_o.axm_bias(1)*100, hd_o.axm_bias(2)*100, hd_o.axm_bias(3)*100);
    fprintf(fid, ['Read: if both rows sit at ratio~1.0 and bias~0%%, the formula matches throughout.\n', ...
                  'Any osc deviation is localized to the near-wall trough -> see Details (near vs far).\n\n']);

    % --- headline: C_dx — does true-motion Var(delta x) match? ---
    fprintf(fid, '## Headline - C_dx (true-motion var, a=a_true arm)\n\n');
    fprintf(fid, ['Theory: Var(delta x) = C_dx * 4kBT * a + C_n_fb * sigma2_n  ', ...
                  '(C_dx=%.4f, C_n_fb=%.4f)\n\n'], C_dx, C_n_fb);
    fprintf(fid, '| window | ratio_x | ratio_y | ratio_z |\n');
    fprintf(fid, '|--------|---------|---------|----------|\n');
    fprintf(fid, '| desc   | %.3f | %.3f | %.3f |\n', ...
            cdx_ratio.desc(1), cdx_ratio.desc(2), cdx_ratio.desc(3));
    fprintf(fid, '| osc    | %.3f | %.3f | %.3f |\n\n', ...
            cdx_ratio.osc(1), cdx_ratio.osc(2), cdx_ratio.osc(3));
    fprintf(fid, ['ratio = mean(Var(delta x)_meas) / mean(Var(delta x)_theory).  ', ...
                  'Target: ~1.0 for desc and far windows.\n\n']);

    % --- headline: R22 — does Var(a_xm) match R2_intrinsic? ---
    fprintf(fid, '## Headline - R22 (a_xm measurement-noise var, a=a_true arm)\n\n');
    fprintf(fid, ['Theory: Var(a_xm) = K_var * IF_eff(a,sig2_n) * (a + xi)^2  ', ...
                  '(R2_intrinsic; K_var=%.5f, xi=[%.3g %.3g %.3g])\n\n'], ...
            r22.K_var, r22.xi(1), r22.xi(2), r22.xi(3));
    fprintf(fid, '| window | ratio_x | ratio_y | ratio_z |\n');
    fprintf(fid, '|--------|---------|---------|----------|\n');
    for wi = 1:numel(win_names)
        wn = win_names{wi};
        rr = r22.ratio.(wn);
        fprintf(fid, '| %-4s | %.3f | %.3f | %.3f |\n', wn, rr(1), rr(2), rr(3));
    end
    fprintf(fid, ['\nratio = mean(Var(a_xm)_meas) / mean(R2_intrinsic_theory), a=a_true arm.\n', ...
                  'Var(a_xm) is the variance OF the EWMA variance estimate (4th moment), so\n', ...
                  'expect a looser match (~5-15%%) than the C_dpmr first-moment identity.\n', ...
                  'The R2_eff delay term (sum of d past Var(delta a_ram)=Q55) is an\n', ...
                  'effective-R correction for the d-step lag; it is NOT in the raw Var(a_xm)\n', ...
                  'and is verified compositionally via the already-checked Q55.\n\n']);

    % --- details: per-window table (desc / osc / near / far x both arms) ---
    fprintf(fid, '## Details - per-window ratio and a_xm bias\n\n');
    fprintf(fid, '`ratio` = mean(Var_meas) / mean(Var_theory).  Target: a=a_true arm desc/far ≈ 1.0.\n');
    fprintf(fid, '`axm_bias` = (mean(a_xm) - mean(a_true)) / mean(a_true).  Target: a=a_true arm desc/far ≈ 0%%.\n\n');
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
    fprintf(fid, ['`desc` (descent, quasi-static a-sweep) and `far` (oscillation, h_bar >= 1.5):\n', ...
                  'C_dpmr quasi-static assumption holds, a=a_true arm ratio should be ≈1.0 (±~5%%).\n', ...
                  'These are the primary evidence windows for the identity.\n\n']);
    fprintf(fid, '### Expected deviations (not bugs)\n\n');
    fprintf(fid, ['- **`near` (gate-on, h_bar < 1.5, near wall)**: rapid a(t) variation and\n', ...
                  '  nonlinear wall correction dynamics invalidate the quasi-static assumption.\n', ...
                  '  a=a_true arm ratio deviating here is expected physics, not a bug.\n', ...
                  '- **`osc` overall**: mixes near + far windows; result is dominated by the\n', ...
                  '  near fraction when the trajectory spends significant time near the wall.\n', ...
                  '- **a=â arm, all windows**: systematic ratio deviation and a_xm bias reflect\n', ...
                  '  the self-consistent equilibrium (a_hat != a_true -> closed-loop pole\n', ...
                  '  shifts to ~lambda_c * g -> Var(dx_r) shifts -> a_xm shifts). This is\n', ...
                  '  the expected diagnosis of estimated-arm cost; it is not a failure.\n\n']);
    fprintf(fid, '### Sanity check\n\n');
    fprintf(fid, ['C_dpmr = %.4f (full form used here). If the simplified form (3.96,\n', ...
                  'lambda_c-only) were used instead, a=a_true arm a_xm would be systematically\n', ...
                  'high-biased by ~25%% (ratio_xm / ratio_cdpmr = 3.96/3.16 ≈ 1.25).\n'], C_dpmr);
    fclose(fid);
end


% ====================================================================
% LOCAL: a_bin_points
% ====================================================================
function B = a_bin_points(a_axis, vps, stk_full, mask, n_bin, scale)
%A_BIN_POINTS Bin per-sample variance by a-value; return binned mean + jackknife SEM.
%
%   a_axis   [N x 1]    a-value at each time sample (um/pN)
%   vps      [N x 1]    per-sample cross-seed variance (um^2)
%   stk_full [N x Ns]   per-seed raw signal values (um), for jackknife
%   mask     [N x 1]    logical, restricts samples to include (e.g. sc_mask)
%   n_bin    scalar     number of equal-width a-axis bins
%   scale    scalar     multiply output y and sem (e.g. SCALE_NM2 = 1e6)
%
%   Returns B with fields x [nb x 1], y [nb x 1], sem [nb x 1] after removing
%   any bins with fewer than MIN_BIN_COUNT samples.
    MIN_BIN_COUNT = 3;   % minimum samples per bin to form a valid estimate
    idx = find(mask(:));
    a   = a_axis(idx);
    if isempty(a)
        B.x = zeros(0, 1); B.y = zeros(0, 1); B.sem = zeros(0, 1);
        return;
    end
    edges = linspace(min(a), max(a), n_bin + 1);
    B.x   = nan(n_bin, 1);
    B.y   = nan(n_bin, 1);
    B.sem = nan(n_bin, 1);
    for b = 1:n_bin
        if b < n_bin
            in_bin = a >= edges(b) & a <  edges(b+1);
        else
            in_bin = a >= edges(b) & a <= edges(b+1);   % include right edge in last bin
        end
        sel = idx(in_bin);
        if numel(sel) < MIN_BIN_COUNT; continue; end
        B.x(b)   = mean(a_axis(sel));
        B.y(b)   = mean(vps(sel)) * scale;
        B.sem(b) = jackknife_var_sem(stk_full(sel, :)) * scale;
    end
    keep  = ~isnan(B.x);
    B.x   = B.x(keep);
    B.y   = B.y(keep);
    B.sem = B.sem(keep);
end


% ====================================================================
% LOCAL: jackknife_var_sem
% ====================================================================
function sem = jackknife_var_sem(stk)
%JACKKNIFE_VAR_SEM Jackknife standard error of the mean cross-seed variance.
%
%   stk  [W x Ns]   per-seed signal values for W samples in one a-value bin
%
%   For each leave-one-out iteration, computes the Bessel-corrected cross-seed
%   variance at each of the W rows, then averages over rows.  Returns the
%   jackknife SE across those Ns leave-one-out estimates.
    [W, Ns] = size(stk);
    if Ns < 2 || W < 1
        sem = 0;
        return;
    end
    jk = zeros(1, Ns);
    for si = 1:Ns
        keep     = true(1, Ns);
        keep(si) = false;
        vk       = var(stk(:, keep), 0, 2);   % [W x 1] cross-seed var, seed si left out
        jk(si)   = mean(vk);
    end
    sem = sqrt((Ns - 1) / Ns * sum((jk - mean(jk)).^2));
end


% ====================================================================
% LOCAL: if_eff_eval  (mirror motion_control_law_eq17_6state.m L587-596)
% ====================================================================
function IF = if_eff_eval(IF_abc, C_dpmr, C_n, kBT, a, sigma2_nx)
%IF_EFF_EVAL Exact color-inflation factor IF_eff for R22 (R22_derivation S4-S6).
%   IF = 1 + 2*(sxT^2*A + 2*sxT*snx*B + snx^2*C) / (C_dpmr*sxT + C_n*snx)^2,
%   sxT = 4*kBT*a (thermal residual var), snx = sigma2_nx (sensor), IF_abc=[A;B;C]
%   the offline s-weighted autocorrelation sums. Element-wise in a (snx scalar),
%   so it serves both the [N x 1] theory column and the a_line scatter sweep.
    sxT = 4 * kBT * a;
    num = sxT.^2 * IF_abc(1) + 2 * sxT * sigma2_nx * IF_abc(2) + sigma2_nx^2 * IF_abc(3);
    den = (C_dpmr * sxT + C_n * sigma2_nx).^2;
    IF  = 1 + 2 * num ./ den;
end


