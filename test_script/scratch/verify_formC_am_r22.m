% STATUS: ACTIVE (scratch) | PURPOSE: verify the formC_b gain readout (a_m) and
%   R(2,2) chain -- the raw AR(1) readout, the whitened increment the KF is
%   actually fed, and the a_m LPF cascade applied OFFLINE to the same readout
%   | EXPIRES: when the R22 audit closes, or when the LPF is wired into the
%   loop (which this script deliberately does NOT do)
%
% WHAT IS AND IS NOT TESTED
%   Tested : the closed forms for Var(a_bar_wm), Var(y2) and Var(a_m_det), all
%            measured as CROSS-SEED pointwise variance (a_true is common-mode
%            across seeds, so it cancels without any deflation model).
%   Not    : what an in-loop LPF would do to a_hat / tracking. The LPF here is
%            post-processing of the logged readout; wiring it in also collides
%            with the whitening pole and is a separate decision.
%
% PRE-REGISTERED FAILURE SIGNATURES (read these BEFORE any ratio)
%   * whitening not wired        -> lag-1 autocorr of y2 stays near 1-a_cov = 0.95
%   * whitening factor dropped   -> whitened ratio off by 1/(a_cov(2-a_cov)) = 10.3x
%   * echo (1-S) acts on noise   -> whitened ratio a CONSTANT near (1-S)^2 = 0.46
%   * IF_eff counted twice       -> whitened per-sample ratio near IF = 3.4, not 1
%
% DECLARED UP FRONT: this is the a = a_hat arm (the deployed estimator). The
%   6-state validation that reached +-3% used the a = a_true arm; that number
%   does NOT transfer. Expect a few percent near the wall, and expect the
%   squared-residual identity Var(dw_r^2) = 2 sigma^4 to run high wherever the
%   HP filter leaves a deterministic remnant (Var((m+n)^2) = 4 m^2 sigma^2 +
%   2 sigma^4). Judge the shape and the constant, not the third digit.
%
%   out = verify_formC_am_r22()                       % 40 seeds, z axis
%   out = verify_formC_am_r22(struct('seeds', 1:6))   % smoke
function out = verify_formC_am_r22(opts)

    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'seeds');    opts.seeds    = 1:40;         end
    if ~isfield(opts, 'ax');       opts.ax       = 3;            end   % z
    if ~isfield(opts, 'arm');      opts.arm      = 'best';       end
    if ~isfield(opts, 'betas');    opts.betas    = [0.005 0.05]; end   % a_det values
    if ~isfield(opts, 'seed_fig'); opts.seed_fig = [];           end   % [] = first seed
    if ~isfield(opts, 'save_fig'); opts.save_fig = true;         end
    if ~isfield(opts, 'data');     opts.data     = [];           end   % preloaded run set
    % Stack cache: the per-seed run set is ~5 MB per seed, so it is NOT kept.
    % What the analysis needs is four N x n_seed matrices plus the constants;
    % that is what gets written, and reloading it skips the simulation.
    if ~isfield(opts, 'save_stack'); opts.save_stack = '';        end
    if ~isfield(opts, 'stack');      opts.stack      = '';        end

    script_dir   = fileparts(mfilename('fullpath'));
    project_root = fileparts(fileparts(script_dir));
    addpath(fullfile(project_root, 'test_script', 'integration'));
    out_dir = fullfile(project_root, 'test_results', 'am_r22');
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end

    ax    = opts.ax;
    axl   = 'xyz';

    if ~isempty(opts.stack)
        % ---- reload a previously stacked run set (no simulation) ----
        SK = load(opts.stack);
        assert(SK.ax == ax, 'verify_formC_am_r22:stackAxis', ...
               'the cached stack is axis %d, opts.ax is %d', SK.ax, ax);
        A_wm = SK.A_wm; A_tr = SK.A_tr; R2_u = SK.R2_u; A_ht = SK.A_ht;
        gate = SK.gate;
        t = SK.t; N = numel(t); n_seed = size(A_wm, 2);
        cc = SK.cc; a_cov = cc.a_cov; C_dpmr = cc.C_dpmr; C_n = cc.C_n;
        K_var = cc.K_var; IF_abc = cc.IF_abc(:); d_delay = cc.d;
        kappa_T = SK.kappa_T; s2n_nd = SK.s2n_nd; xi_bar = SK.xi_bar;
        D = struct('cfg', SK.cfg);   % a_nom is not needed: A_wm is already normalized
        opts.seeds = SK.seeds;
        fprintf('\nstack reloaded: %s  (%d seeds, axis %c)\n', opts.stack, n_seed, axl(ax));
        local_print_constants(cc, IF_abc, kappa_T, xi_bar, ax, axl, d_delay);
    else
    % ------------------------------------------------------------------
    % [1] Run (or accept) the formC_b seed set
    % ------------------------------------------------------------------
    if isempty(opts.data)
        D = run_formC_b(struct('seeds', opts.seeds, 'arm', opts.arm));
    else
        D = opts.data;
    end
    runs  = D.runs;
    n_seed = numel(runs);

    % ------------------------------------------------------------------
    % [2] Constants -- taken from the run's own ctrl_const / params, never
    %     retyped (the controller's own xi_bar route is reproduced here)
    % ------------------------------------------------------------------
    cc = runs{1}.ctrl_const;
    P  = runs{1}.meta.params_value;
    a_cov   = cc.a_cov;
    C_dpmr  = cc.C_dpmr;
    C_n     = cc.C_n;
    K_var   = cc.K_var;
    IF_abc  = cc.IF_abc(:);
    d_delay = cc.d;

    R_radius = P.common.R;
    a_o      = P.ctrl.Ts / (P.ctrl.gamma * R_radius);
    kappa_T  = 4 * (P.ctrl.k_B * P.ctrl.T / R_radius) * a_o;
    s2n_nd   = P.ctrl.sigma2_noise(:) / R_radius^2;
    xi_bar   = (C_n / C_dpmr) * s2n_nd / kappa_T;
    a_nom    = runs{1}.a_nom;                       % [um/pN] = a_o * R
    assert(abs(a_nom - a_o * R_radius) < 1e-9 * a_nom, ...
           'verify_formC_am_r22:aNom', 'a_nom does not match a_o*R');

    local_print_constants(cc, IF_abc, kappa_T, xi_bar, ax, axl, d_delay);

    % ------------------------------------------------------------------
    % [3] Stack the per-seed series (normalized gain units)
    % ------------------------------------------------------------------
    t   = runs{1}.tout(:);
    N   = numel(t);
    A_wm = zeros(N, n_seed);    % raw readout a_bar_wm
    A_tr = zeros(N, n_seed);    % true a_bar
    R2_u = zeros(N, n_seed);    % R(2,2) the filter actually used
    A_ht = zeros(N, n_seed);    % a_bar_hat (posterior)
    gate = false(N, n_seed);
    for s = 1:n_seed
        A_wm(:, s) = runs{s}.a_xm_out(:, ax)   / a_nom;
        A_tr(:, s) = runs{s}.a_true_out(:, ax) / a_nom;
        R2_u(:, s) = runs{s}.R2_out(:, ax);
        A_ht(:, s) = runs{s}.a_bar_hat_out(:, ax);   % posterior a_bar (R2 is built on it)
        gate(:, s) = logical(runs{s}.gate_out(:, ax));
    end

    % SAMPLE 1 IS THE CONTROLLER'S INIT-ONLY CALL (formC_b returns early at
    % :772 with an all-zero diag), so every logged series starts with a
    % placeholder. Dropping it is mandatory: left in, it seeds the offline LPF
    % at zero (0.5 s of recovery at a_det = 0.005) and makes the first whitened
    % increment meaningless. Signature that it is the init sample and not a
    % real zero: a_xm, h_bar and P are ALL exactly zero on that row.
    assert(all(A_wm(1, :) == 0), 'verify_formC_am_r22:initRow', ...
           'row 1 is expected to be the init-only call (all-zero diag)');
    A_wm = A_wm(2:end, :);  A_tr = A_tr(2:end, :);
    R2_u = R2_u(2:end, :);  gate = gate(2:end, :);  A_ht = A_ht(2:end, :);
    t    = t(2:end);        N    = numel(t);

    if ~isempty(opts.save_stack)
        cfg = D.cfg;  seeds = opts.seeds;
        save(opts.save_stack, 'A_wm', 'A_tr', 'R2_u', 'A_ht', 'gate', 't', 'cc', ...
             'kappa_T', 's2n_nd', 'xi_bar', 'a_nom', 'cfg', 'seeds', 'ax', '-v7.3');
        fprintf('stack saved: %s\n', opts.save_stack);
    end
    end   % <- end of the "run vs reload" branch

    % Whitened increment, rebuilt EXACTLY as the controller builds it
    % (formC_b:829). The first retained sample has no predecessor and is
    % dropped in turn.
    Y2  = A_wm(2:end, :) - (1 - a_cov) * A_wm(1:end-1, :);
    t2  = t(2:end);

    % ------------------------------------------------------------------
    % [4] Theory, evaluated per step at a_true (the physics), never at a_hat
    % ------------------------------------------------------------------
    a_tr_mean  = mean(A_tr, 2);
    IF_eff     = if_eff_local(a_tr_mean, s2n_nd(ax), IF_abc, C_dpmr, C_n, kappa_T);
    gap        = a_tr_mean + xi_bar(ax);

    var_raw_th = K_var .* IF_eff .* gap.^2;             % Var(a_bar_wm)
    var_y2_th  = 2 * a_cov^2 .* gap(2:end).^2;          % Var(y2), PER SAMPLE (no IF)
    R2_used_th = 2 * a_cov^2 .* IF_eff(2:end) .* gap(2:end).^2;   % what R2 carries

    var_raw_ms = var(A_wm, 0, 2);
    var_y2_ms  = var(Y2,   0, 2);

    % ------------------------------------------------------------------
    % [4b] The three pieces R(2,2) is made of, evaluated where the controller
    %      evaluates them (at the POSTERIOR a_bar_hat, not at a_true):
    %        A = 2 a_cov^2 (a_hat + xi)^2      per-sample variance of y2
    %        A * IF_eff                        + serial-correlation penalty
    %        + a_cov^2 * d * Q44               + the gain moved during the delay
    %      The last piece is recovered by difference from the LOGGED R2, so the
    %      reconstruction is also its own cross-check.
    % ------------------------------------------------------------------
    a_ht_mean = mean(A_ht, 2);
    IF_hat    = if_eff_local(a_ht_mean, s2n_nd(ax), IF_abc, C_dpmr, C_n, kappa_T);
    R2_A      = 2 * a_cov^2 .* (a_ht_mean + xi_bar(ax)).^2;
    R2_AIF    = R2_A .* IF_hat;
    R2_tot    = mean(R2_u, 2);
    R2_delay  = R2_tot - R2_AIF;

    % ------------------------------------------------------------------
    % [5] Offline LPF cascade on the SAME readout
    % ------------------------------------------------------------------
    n_beta = numel(opts.betas);
    A_md      = cell(n_beta, 1);
    var_md_ms = cell(n_beta, 1);
    var_md_th = cell(n_beta, 1);
    IF2       = zeros(n_beta, 1);
    for j = 1:n_beta
        b = opts.betas(j);
        s2 = (1 - b) * (1 - a_cov);
        IF2(j) = 1 + 2 * s2 / (1 - s2);                 % crude AR(1) IF2 (build's form)
        M = zeros(N, n_seed);
        for s = 1:n_seed
            % zi chosen so the LPF output starts exactly at the readout, the
            % same mean-preserving prefill the 6-state controller used.
            M(:, s) = filter(b, [1, -(1 - b)], A_wm(:, s), (1 - b) * A_wm(1, s));
        end
        A_md{j}      = M;
        var_md_ms{j} = var(M, 0, 2);
        var_md_th{j} = (b / (2 - b)) * IF2(j) * var_raw_th;
    end

    % ------------------------------------------------------------------
    % [6] Console: wiring checks FIRST, ratios second
    % ------------------------------------------------------------------
    win = local_windows(D.cfg);
    ih  = t >= win.hold(1) & t <= win.hold(2);          % final hold, quasi-stationary
    ih2 = t2 >= win.hold(1) & t2 <= win.hold(2);

    rho_raw = lag1_fluct(A_wm(ih, :));
    rho_y2  = lag1_fluct(Y2(ih2, :));
    fprintf('\n=== wiring checks (final hold, %d seeds) ===\n', n_seed);
    fprintf('lag-1 autocorr  a_bar_wm %+.4f   (1-a_cov = %.4f expected)\n', rho_raw, 1 - a_cov);
    fprintf('lag-1 autocorr  y2       %+.4f   (whitened: must be far below that)\n', rho_y2);
    tau_max = 8;
    pr_raw = rho_profile(A_wm(ih, :), tau_max);
    pr_y2  = rho_profile(Y2(ih2, :), tau_max);
    fprintf('rho(tau)  tau =');   fprintf(' %6d', 1:tau_max); fprintf('\n');
    fprintf('  a_bar_wm      ');  fprintf(' %+6.3f', pr_raw); fprintf('\n');
    fprintf('  y2            ');  fprintf(' %+6.3f', pr_y2);  fprintf('\n');
    fprintf('  (1-a_cov)^tau ');  fprintf(' %+6.3f', (1 - a_cov).^(1:tau_max)); fprintf('\n');
    tau_int = 30;
    pr_long = rho_profile(Y2(ih2, :), tau_int);
    IF_meas = 1 + 2 * sum(pr_long);
    fprintf(['IF (correlation penalty), final hold: measured 1+2*sum(rho) = %.3f  vs  ', ...
             'IF_eff used = %.3f  (%+.1f %%)\n'], IF_meas, IF_hat(end), ...
            100 * (IF_hat(end) / IF_meas - 1));
    fprintf(['R2 pieces at the trough: per-sample %.4g  x IF %.3f  + delay %.4g  ', ...
             '= %.4g  (logged %.4g)\n'], R2_A(end), IF_hat(end), R2_delay(end), ...
            R2_AIF(end) + R2_delay(end), R2_tot(end));
    fprintf('delay share of R2: %.1f %% at the trough, %.1f %% at the start\n', ...
            100 * R2_delay(end) / R2_tot(end), 100 * R2_delay(1) / R2_tot(1));
    fprintf(['estimator noise: a cross-seed variance from %d seeds scatters by ', ...
             '%.1f %% (1-sigma);\n   the figures time-average 0.1 s on top of that ', ...
             '(a_bar_wm keeps ~1/a_cov = %d steps of\n   memory, so that window holds ', ...
             'only ~%d independent draws; y2 is white, so it holds ~%d).\n'], ...
            n_seed, 100 * sqrt(2 / (n_seed - 1)), round(1 / a_cov), ...
            round(0.1 / (t(2) - t(1)) * a_cov / 2), round(0.1 / (t(2) - t(1))));
    fprintf('gate-off fraction (axis %c): %.2f %%   [G3 h_bar_safe = %.2f]\n', ...
            axl(ax), 100 * mean(gate(:), 'all'), cc.h_bar_safe);

    % Where the LPF cascade's error comes from: IF2 assumes rho_axm(tau) =
    % (1-a_cov)^tau, but the readout is an EWMA of a COLORED input and decays
    % slower than that. Rebuilding IF2 from the measured rho says how much of
    % the cascade ratio that accounts for -- the 2026-06 "L1 crude AR(1)"
    % defect, measured in this scenario instead of assumed.
    L_if2 = 600;
    rho_raw_long = rho_profile(A_wm(ih, :), L_if2);
    fprintf('\nIF2 (LPF cascade), final hold:\n');
    for j = 1:n_beta
        b = opts.betas(j);
        IF2_meas_j = 1 + 2 * sum(((1 - b) .^ (1:L_if2)) .* rho_raw_long);
        fprintf(['  a_det %.3f : model %.2f  vs  from measured rho %.2f  ', ...
                 '(x%.3f -- compare the cascade row below)\n'], ...
                b, IF2(j), IF2_meas_j, IF2_meas_j / IF2(j));
    end

    fprintf('\n=== variance ratios (measured / theory), median per window ===\n');
    fprintf('%-28s %17s %17s %17s\n', 'quantity  [agg | med]', 'descent', 'osc', 'hold');
    print_ratio('Var(a_bar_wm) / closed form', var_raw_ms, var_raw_th, t, win);
    print_ratio('Var(y2) / per-sample form',   var_y2_ms,  var_y2_th,  t2, win);
    for j = 1:n_beta
        print_ratio(sprintf('Var(a_m_det) b=%.3f / casc', opts.betas(j)), ...
                    var_md_ms{j}, var_md_th{j}, t, win);
    end
    print_ratio('R2_used / Var(y2) measured', mean(R2_u(2:end, :), 2), var_y2_ms, t2, win);
    fprintf(['note: the last row is NOT a defect -- R2 deliberately carries the ', ...
             'serial-correlation\n      inflation IF_eff (%.3f at the trough) plus ', ...
             'a_cov^2*d*Q44, on top of the\n      per-sample variance.\n'], IF_eff(end));

    % ------------------------------------------------------------------
    % [7] Figures
    % ------------------------------------------------------------------
    if opts.save_fig
        sf = opts.seed_fig;
        if isempty(sf); sf = 1; else; sf = find(opts.seeds == sf, 1); end
        FS = 18; LFS = 14; AXLW = 1.2;
        fs = 1 / (t(2) - t(1));
        % Time-average window. The cross-seed variance itself scatters by
        % sqrt(2/(N-1)) = 22.6% at N = 40, and a_bar_wm carries ~1/a_cov = 20
        % steps of memory, so a 25 ms window would hold only ~2 independent
        % draws. 0.1 s holds ~8 for the readout and ~160 for the whitened
        % channel, while still resolving the 1 Hz shape.
        SM = 0.1;                                    % [s] time-average window
        COL_RAW = [0.72 0.86 0.97];                  % faint background = raw scatter
        COL_TRUE = [0.8 0 0]; COL_MEAS = [0.45 0.72 0.95]; COL_HAT = [0 0.2 0.9];
        COL_TH   = [0 0.55 0.2];

        % --- FIG 1: whitening, before | after (single seed, shared y) ---
        % Zoomed to half a second in the oscillation: over a full run the two
        % panels are solid ink and the point (memory vs no memory) is invisible.
        % Shared y is kept -- the whitened channel really is ~3x noisier per
        % sample, and hiding that with separate axes would misrepresent the
        % trade the whitening makes.
        ZW = [2.0, 2.5];
        f = new_fig([80 80 1200 520]);
        tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
        iz = t >= ZW(1) & t <= ZW(2);  iz2 = t2 >= ZW(1) & t2 <= ZW(2);
        yl = [min([A_wm(iz, sf); Y2(iz2, sf) / a_cov]), ...
              max([A_wm(iz, sf); Y2(iz2, sf) / a_cov])];
        nexttile; hold on;
        h1 = plot(t, A_wm(:, sf), '-', 'Color', COL_MEAS, 'LineWidth', 0.8, ...
                  'DisplayName', 'a_{m} readout (before)');
        h2 = plot(t, a_tr_mean, '-', 'Color', COL_TRUE, 'LineWidth', 2.4, ...
                  'DisplayName', 'a_{true}');
        ylim(yl); xlim(ZW);
        ylabel(sprintf('a_%c / a_o', axl(ax)), 'FontSize', FS, 'FontWeight', 'bold');
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        legend([h2 h1], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
               'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
        nexttile; hold on;
        h3 = plot(t2, Y2(:, sf) / a_cov, '-', 'Color', COL_MEAS, 'LineWidth', 0.8, ...
                  'DisplayName', 'y_2 / a_{cov}  (after)');
        h4 = plot(t, a_tr_mean, '-', 'Color', COL_TRUE, 'LineWidth', 2.4, ...
                  'DisplayName', 'a_{true}');
        ylim(yl); xlim(ZW);
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        legend([h4 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
               'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
        save_fig(f, out_dir, 'fig1_whiten_before_after.png');

        % --- FIG 2: raw R22 ---
        f = new_fig([80 80 1180 560]);
        hold on;
        plot(t, var_raw_ms, '-', 'Color', COL_RAW, 'LineWidth', 0.5, ...
             'HandleVisibility', 'off');
        h1 = plot(t, smooth_t(var_raw_ms, fs, SM), '-', 'Color', COL_MEAS, ...
                  'LineWidth', 2.0, ...
                  'DisplayName', sprintf('measured (%d seeds, %.0f ms avg)', n_seed, SM * 1e3));
        h2 = plot(t, var_raw_th, '-', 'Color', COL_TH, 'LineWidth', 3.0, ...
                  'DisplayName', 'K_{var} IF_{eff} (a+\xi)^2');
        xlim([t(1) t(end)]);
        ylabel(sprintf('Var(a_{m,%c})  [-]', axl(ax)), 'FontSize', FS, 'FontWeight', 'bold');
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        legend([h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
               'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
        save_fig(f, out_dir, 'fig2_r22_raw_time.png');

        % --- FIG 3 / 4: variance vs a_bar, binned along a (the 2026-06 view) ---
        % The time plots answer "how big"; these answer "how does it depend on
        % a". Bins are uniform along a_true (NOT time slices), the error bar is
        % a jackknife over SEEDS, and both panels share the same x range and
        % plain decimal ticks so the readout and the whitened channel can be
        % read against each other.
        NBIN = 28;
        a_lo = min(a_tr_mean); a_hi = max(a_tr_mean);
        edges = linspace(a_lo, a_hi, NBIN + 1);
        a_grid = linspace(a_lo, a_hi, 200).';
        IF_grid = if_eff_local(a_grid, s2n_nd(ax), IF_abc, C_dpmr, C_n, kappa_T);

        [cR, vR, eR] = binned_var_jk(A_wm, a_tr_mean, edges);
        f = new_fig([80 80 1000 620]);
        hold on;
        hT = plot(a_grid, K_var * IF_grid .* (a_grid + xi_bar(ax)).^2, '-', ...
                  'Color', COL_TH, 'LineWidth', 3.0, ...
                  'DisplayName', 'K_{var} IF_{eff} (a+\xi)^2');
        hM = errorbar(cR, vR, eR, 'o', 'Color', COL_HAT, 'MarkerFaceColor', COL_HAT, ...
                      'MarkerSize', 6, 'LineWidth', 1.0, 'CapSize', 3, 'LineStyle', 'none', ...
                      'DisplayName', sprintf('measured, a = â arm (%d seeds)', n_seed));
        set(gca, 'XScale', 'log', 'YScale', 'log');
        style_scatter(gca, a_lo, a_hi, FS, AXLW, axl(ax), ...
                      sprintf('Var(a_{m,%c})  [-]', axl(ax)));
        legend([hT hM], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
               'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        save_fig(f, out_dir, 'fig3_r22_raw_vs_a.png');

        [cW, vW, eW] = binned_var_jk(Y2, a_tr_mean(2:end), edges);
        f = new_fig([80 80 1000 620]);
        hold on;
        hT = plot(a_grid, 2 * a_cov^2 * (a_grid + xi_bar(ax)).^2, '-', ...
                  'Color', COL_TH, 'LineWidth', 3.0, ...
                  'DisplayName', '2 a_{cov}^2 (a+\xi)^2  per-sample');
        hM = errorbar(cW, vW, eW, 'o', 'Color', COL_HAT, 'MarkerFaceColor', COL_HAT, ...
                      'MarkerSize', 6, 'LineWidth', 1.0, 'CapSize', 3, 'LineStyle', 'none', ...
                      'DisplayName', sprintf('measured, a = â arm (%d seeds)', n_seed));
        set(gca, 'XScale', 'log', 'YScale', 'log');
        style_scatter(gca, a_lo, a_hi, FS, AXLW, axl(ax), ...
                      sprintf('Var(y_{2,%c})  [-]', axl(ax)));
        legend([hT hM], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
               'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        save_fig(f, out_dir, 'fig4_r22_whitened_vs_a.png');

        % --- FIG 6: LPF before / after, single seed ---
        f = new_fig([80 80 1180 560]);
        hold on;
        hh = plot(t, A_wm(:, sf), '-', 'Color', COL_MEAS, 'LineWidth', 0.8, ...
                  'DisplayName', 'a_{m} readout');
        hs = gobjects(n_beta, 1);
        shades = [0 0.2 0.9; 0.45 0.1 0.6];
        for j = 1:n_beta
            hs(j) = plot(t, A_md{j}(:, sf), '-', 'Color', shades(min(j, 2), :), ...
                         'LineWidth', 2.0, ...
                         'DisplayName', sprintf('a_{m,det}  a_{det} = %.3f', opts.betas(j)));
        end
        ht = plot(t, a_tr_mean, '-', 'Color', COL_TRUE, 'LineWidth', 2.4, ...
                  'DisplayName', 'a_{true}');
        xlim([t(1) t(end)]);
        ylabel(sprintf('a_%c / a_o', axl(ax)), 'FontSize', FS, 'FontWeight', 'bold');
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        legend([ht; hh; hs(:)], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
               'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
        save_fig(f, out_dir, 'fig6_lpf_before_after.png');

        % --- FIG 7: LPF cascade variance, one figure per a_det ---
        for j = 1:n_beta
            f = new_fig([80 80 1180 560]);
            hold on;
            plot(t, var_md_ms{j}, '-', 'Color', COL_RAW, 'LineWidth', 0.5, ...
                 'HandleVisibility', 'off');
            h1 = plot(t, smooth_t(var_md_ms{j}, fs, SM), '-', 'Color', COL_MEAS, ...
                      'LineWidth', 2.0, ...
                      'DisplayName', sprintf('measured (%d seeds, %.0f ms avg)', ...
                                             n_seed, SM * 1e3));
            h2 = plot(t, var_md_th{j}, '-', 'Color', COL_TH, 'LineWidth', 3.0, ...
                      'DisplayName', sprintf('[b/(2-b)] IF_2 Var(a_m),  IF_2 = %.1f', IF2(j)));
            xlim([t(1) t(end)]);
            ylabel(sprintf('Var(a_{m,det,%c})  [-]', axl(ax)), 'FontSize', FS, 'FontWeight', 'bold');
            xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
            legend([h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
                   'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
            set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
            save_fig(f, out_dir, sprintf('fig7_r22_lpf_adet%g.png', opts.betas(j)));
        end
        % --- FIG 5: what R(2,2) is made of ---
        f = new_fig([80 80 1180 560]);
        hold on;
        hm = plot(t2, smooth_t(var_y2_ms, fs, SM), '-', 'Color', COL_RAW, ...
                  'LineWidth', 2.0, 'DisplayName', 'measured Var(y_2)');
        h1 = plot(t, R2_A, '-', 'Color', COL_TH, 'LineWidth', 2.5, ...
                  'DisplayName', '(1) per-sample  2a_{cov}^2(\ita\rm+\xi)^2');
        h2 = plot(t, R2_AIF, '-', 'Color', COL_MEAS, 'LineWidth', 2.5, ...
                  'DisplayName', '(2) + correlation \times IF_{eff}');
        h3 = plot(t, R2_tot, '--', 'Color', COL_HAT, 'LineWidth', 2.0, ...
                  'DisplayName', '(3) + delay = R(2,2)');
        xlim([t(1) t(end)]);
        ylabel(sprintf('R(2,2)_%c  [-]', axl(ax)), 'FontSize', FS, 'FontWeight', 'bold');
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        legend([hm h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
               'FontSize', LFS - 2, 'FontWeight', 'bold', 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
        save_fig(f, out_dir, 'fig5_r22_decomposition.png');

        fprintf('\nfigures -> %s\n', out_dir);
    end

    out = struct('t', t, 't2', t2, 'A_wm', A_wm, 'Y2', Y2, 'A_tr', A_tr, ...
                 'var_raw_ms', var_raw_ms, 'var_raw_th', var_raw_th, ...
                 'var_y2_ms', var_y2_ms, 'var_y2_th', var_y2_th, ...
                 'R2_used_th', R2_used_th, 'R2_used', mean(R2_u, 2), ...
                 'var_md_ms', {var_md_ms}, 'var_md_th', {var_md_th}, ...
                 'R2_A', R2_A, 'R2_AIF', R2_AIF, 'R2_tot', R2_tot, ...
                 'R2_delay', R2_delay, 'IF_meas', IF_meas, ...
                 'IF_eff', IF_eff, 'IF2', IF2, 'betas', opts.betas, ...
                 'rho_raw', rho_raw, 'rho_y2', rho_y2, 'out_dir', out_dir, ...
                 'seeds', opts.seeds, 'D', D, 'n_seed', n_seed);
end


function local_print_constants(cc, IF_abc, kappa_T, xi_bar, ax, axl, d_delay)
%LOCAL_PRINT_CONSTANTS  The chain's offline scalars, straight from the run.
    fprintf('\n=== constants (from the run) ===\n');
    fprintf('a_cov %.4g | a_pd %.4g | C_dpmr %.6f | C_n %.6f | K_var %.6f\n', ...
            cc.a_cov, cc.a_pd, cc.C_dpmr, cc.C_n, cc.K_var);
    fprintf('IF_abc [%.5g %.5g %.5g] | kappa_T %.6g | xi_bar(%c) %.4g | d %d\n', ...
            IF_abc(1), IF_abc(2), IF_abc(3), kappa_T, axl(ax), xi_bar(ax), d_delay);
    fprintf('amlpf_var_factor (production) %.6g\n', ...
            get_field_or(cc, 'amlpf_var_factor', 1));
end


function [ctr, val, sem] = binned_var_jk(X, a_ref, edges)
%BINNED_VAR_JK  Cross-seed variance binned along a_bar, with a jackknife SEM.
%   X      [N x S] per-seed series      a_ref [N x 1] the a_bar of each step
%   Bins are uniform in a_bar, so a slow descent and a fast oscillation
%   contribute to the same bin whenever they visit the same gain -- that is the
%   point: the bin's spread then reflects the estimator, not the schedule.
%   The error bar is a delete-one-SEED jackknife (the seeds are the independent
%   replicates; neighbouring time samples are not), computed from running sums
%   so the leave-one-out variances cost one pass:
%       var_(s)[k] = (S2 - x_s^2 - (S1 - x_s)^2/(n-1)) / (n-2)
    n = size(X, 2);
    S1 = sum(X, 2);  S2 = sum(X.^2, 2);
    v_all = (S2 - S1.^2 / n) / (n - 1);                       % [N x 1]
    v_loo = (S2 - X.^2 - (S1 - X).^2 / (n - 1)) / (n - 2);    % [N x S]
    nb = numel(edges) - 1;
    ctr = zeros(nb, 1); val = zeros(nb, 1); sem = zeros(nb, 1);
    for b = 1:nb
        in = a_ref >= edges(b) & a_ref < edges(b + 1);
        if b == nb; in = in | a_ref == edges(end); end
        if ~any(in); ctr(b) = NaN; val(b) = NaN; sem(b) = NaN; continue; end
        ctr(b) = mean(a_ref(in));
        val(b) = mean(v_all(in));
        th = mean(v_loo(in, :), 1);                            % [1 x S]
        sem(b) = sqrt((n - 1) / n * sum((th - mean(th)).^2));
    end
    keep = ~isnan(ctr);
    ctr = ctr(keep); val = val(keep); sem = sem(keep);
end


function style_scatter(ax_h, a_lo, a_hi, FS, AXLW, axl_c, ylab)
%STYLE_SCATTER  Shared x range and plain decimal ticks, so the two scatter
%   figures can be read against one another (the 2026-06 alignment fix: the
%   default exponent labelling made x and z look like different variables).
    xlim(ax_h, [a_lo * 0.95, a_hi * 1.05]);
    xlabel(ax_h, sprintf('a_%c / a_o', axl_c), 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax_h, ylab, 'FontSize', FS, 'FontWeight', 'bold');
    ax_h.XAxis.Exponent = 0;
    xtickformat(ax_h, '%.2f');
    set(ax_h, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
    grid(ax_h, 'off');
end


function y = smooth_t(v, fs, win_s)
%SMOOTH_T  Moving average over TIME. A cross-seed variance from N seeds has a
%   relative scatter of sqrt(2/(N-1)) (22.6% at N = 40) -- that is the
%   estimator's own noise, not a model mismatch. Neighbouring time points are
%   near-independent realizations of the same underlying variance, so averaging
%   over a short window buys back what more seeds would. Window 0.025 s = 40
%   samples at 1600 Hz, the same choice verify_axm_cdpmr_6state uses.
    w = max(1, round(win_s * fs));
    y = movmean(v, w, 'omitnan');
end


function IF = if_eff_local(a_bar, sigma2_n, IF_abc, C_dpmr, C_n, kappa_T)
%IF_EFF_LOCAL  Element-wise copy of the controller's IF_eff (formC_b:1372-1375).
    sxT = kappa_T * a_bar;
    num = sxT.^2 * IF_abc(1) + 2 * sxT * sigma2_n * IF_abc(2) + sigma2_n^2 * IF_abc(3);
    den = (C_dpmr * sxT + C_n * sigma2_n).^2;
    IF  = 1 + 2 * num ./ den;
end


function r = lag1_fluct(X)
%LAG1_FLUCT  Lag-1 autocorrelation of the FLUCTUATION (cross-seed mean removed
%   first, so a common deterministic drift cannot masquerade as memory).
    Xf = X - mean(X, 2);
    a = Xf(1:end-1, :); b = Xf(2:end, :);
    a = a(:) - mean(a(:)); b = b(:) - mean(b(:));
    r = (a' * b) / sqrt((a' * a) * (b' * b));
end


function w = local_windows(cfg)
%LOCAL_WINDOWS  Phase windows [s], same boundaries the drivers use.
    t1 = cfg.t_hold;
    t2 = t1 + cfg.t_descend_override;
    t3 = t2 + cfg.n_cycles / cfg.frequency;
    w = struct('descent', [t1, t2], 'osc', [t2 + 0.25, t3], 'hold', [t3 + 0.25, cfg.T_sim]);
end


function print_ratio(name, meas, theory, t, win)
%PRINT_RATIO  Aggregate ratio (window mean of measured / window mean of theory)
%   and, after it, the pointwise median. With few seeds the pointwise ratio is
%   chi-squared distributed and its MEDIAN is biased low (0.79 at 4 seeds), so
%   the aggregate is the one to read; both are printed so the bias is visible.
    fprintf('%-28s', name);
    ws = {win.descent, win.osc, win.hold};
    for i = 1:3
        idx = t >= ws{i}(1) & t <= ws{i}(2);
        agg = mean(meas(idx), 'omitnan') / mean(theory(idx), 'omitnan');
        med = median(meas(idx) ./ theory(idx), 'omitnan');
        fprintf(' %8.3f | %6.3f', agg, med);
    end
    fprintf('\n');
end


function pr = rho_profile(X, tau_max)
%RHO_PROFILE  Fluctuation autocorrelation at lags 1..tau_max.
    Xf = X - mean(X, 2);
    v = Xf(:);
    n = size(Xf, 1);
    pr = zeros(1, tau_max);
    for tau = 1:tau_max
        a = Xf(1:n-tau, :); b = Xf(1+tau:n, :);
        a = a(:) - mean(a(:)); b = b(:) - mean(b(:));
        pr(tau) = (a' * b) / sqrt((a' * a) * (b' * b));
    end
    if isempty(v); pr(:) = NaN; end
end


function f = new_fig(pos)
    f = figure('Position', pos, 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
end


function save_fig(f, out_dir, name)
    exportgraphics(f, fullfile(out_dir, name), 'Resolution', 150);
    close(f);
end


function v = get_field_or(s, f, dflt)
    if isfield(s, f) && ~isempty(s.(f)); v = s.(f); else; v = dflt; end
end
