function out = plot_amlpf_compare_6state(baseline_freq_dir, lpf_dir, opts)
%PLOT_AMLPF_COMPARE_6STATE  a_m_det LPF study output figures (P5, read-only).
%
%   out = plot_amlpf_compare_6state(baseline_freq_dir, lpf_dir)
%   out = plot_amlpf_compare_6state(baseline_freq_dir, lpf_dir, opts)
%
%   Compares the baseline gain_compare run (no a_m LPF) against the a_m_det
%   LPF rerun (run_amlpf_rerun_6state.m), producing the three study figures
%   defined in reference/eq17_analysis/investigations/am_lpf_r22_design.md §6:
%
%     fig_am_vs_amdet.png        (1) representative seed: raw a_xm vs the
%                                    offline-reconstructed a_m_det vs a_true,
%                                    x (top) / z (bottom). Shows smoothing+lag.
%     fig_ahat_compare.png       (2) arm B cross-seed mean a_hat +/- across-seed
%                                    sigma band, baseline vs LPF, vs a_true.
%     fig_ahat_relerr.png        (2) rel-error decomposition: bias (mean) +
%                                    spread (sigma) band, baseline vs LPF, with
%                                    far/near numbers printed on the plot.
%     fig_tracking_det_compare.png (3) z tracking error e: det component
%                                    (cross-seed mean, top) + stochastic
%                                    (cross-seed std, bottom): baseline / LPF /
%                                    a=a_true floor.
%
%   Also writes amlpf_compare_summary.md (baseline vs LPF table + a headline
%   comparison to the 2026-04-16 prior result).
%
%   Inputs:
%     baseline_freq_dir : dir with baseline runs.mat (arms A + B), e.g.
%                         test_results/gain_compare/f1Hz
%     lpf_dir           : dir with the LPF rerun runs_lpf.mat (arm B only).
%                         With opts.use_preview=true, loads runs_lpf_preview.mat.
%
%   opts (all optional):
%     use_preview  load runs_lpf_preview.mat (runs_p) instead of runs_lpf.mat (false)
%     a_det        LPF EWMA weight (default: file's saved a_det, fallback 0.005)
%     out_dir      output dir (default = lpf_dir)
%     save_fig     true (default)
%     rep_seed     representative seed index for fig (1) (default 1)
%
%   The a=a_true arm is taken from the baseline file (the LPF is a no-op on
%   the true-gain arm: its control law never touches the measurement). The
%   baseline and LPF arm-B runs share seeds and per-seed noise realizations,
%   so figures (2) and (3) use the intersection of non-diverged seeds in BOTH.
%
%   See also: run_amlpf_rerun_6state, verify_axm_cdpmr_6state,
%             plot_var_ahat_6state, analyze_gain_6state

    % ----------------------------------------------------------------
    % Input guards
    % ----------------------------------------------------------------
    if nargin < 1 || isempty(baseline_freq_dir)
        error('plot_amlpf_compare_6state:badInput', 'baseline_freq_dir is required');
    end
    if nargin < 2 || isempty(lpf_dir)
        error('plot_amlpf_compare_6state:badInput', 'lpf_dir is required');
    end
    if nargin < 3; opts = struct(); end
    if ~isfield(opts, 'use_preview'); opts.use_preview = false; end
    if ~isfield(opts, 'save_fig');    opts.save_fig    = true;  end
    if ~isfield(opts, 'rep_seed');    opts.rep_seed    = 1;     end
    % LPF data file: 6-state path uses run_amlpf_rerun's 'runs_lpf.mat' (arm B
    % only); 5-state path reuses compare_gain's 'runs.mat' (arms A+B, use .B).
    if ~isfield(opts, 'lpf_file');    opts.lpf_file    = 'runs_lpf.mat'; end
    if ~isfield(opts, 'det_only');    opts.det_only    = false; end   % fig3: keep only top (det) panel
    if ~isfield(opts, 'no_movmean');  opts.no_movmean  = false; end   % fig3: skip time-smoothing (raw ensemble mean)
    if ~isfield(opts, 'show_seed_n'); opts.show_seed_n = 0; end       % fig2: overlay N single-seed a_hat (raw)

    if isfield(opts, 'out_dir') && ~isempty(opts.out_dir)
        out_dir = opts.out_dir;
    else
        out_dir = lpf_dir;
    end
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end

    % wall_effect on the path for parity with the sibling analyzers (a_true is
    % read from a_true_out, so no wall correction is actually evaluated here)
    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(fileparts(script_dir));
    addpath(fullfile(project_root, 'model', 'wall_effect'));

    % ----------------------------------------------------------------
    % Load baseline (arms A + B) and LPF (arm B only)
    % ----------------------------------------------------------------
    base_path = fullfile(baseline_freq_dir, 'runs.mat');
    if ~exist(base_path, 'file')
        error('plot_amlpf_compare_6state:noFile', 'baseline runs.mat not found: %s', base_path);
    end
    SB     = load(base_path);            % runs, cfg, opts, ...
    runs_b = SB.runs;
    cfg    = SB.cfg;

    if opts.use_preview
        lpf_path = fullfile(lpf_dir, 'runs_lpf_preview.mat');
    else
        lpf_path = fullfile(lpf_dir, opts.lpf_file);
    end
    if ~exist(lpf_path, 'file')
        error('plot_amlpf_compare_6state:noFile', 'LPF file not found: %s', lpf_path);
    end
    SL = load(lpf_path);
    if opts.use_preview
        assert(isfield(SL, 'runs_p'), ...
            'plot_amlpf_compare_6state: preview file missing runs_p (got %s)', lpf_path);
        lpf_B = SL.runs_p.B;
    else
        assert(isfield(SL, 'runs'), ...
            'plot_amlpf_compare_6state: LPF file missing runs (got %s)', lpf_path);
        lpf_B = SL.runs.B;
    end

    % a_det: opts override -> file value -> 0.005 fallback
    if isfield(opts, 'a_det') && ~isempty(opts.a_det)
        a_det = opts.a_det;
    elseif isfield(SL, 'a_det') && ~isempty(SL.a_det)
        a_det = SL.a_det;
    else
        a_det = 0.005;
    end

    % ----------------------------------------------------------------
    % Reference grid + physical constants (mirror plot_var_ahat_6state)
    % ----------------------------------------------------------------
    assert(~runs_b.A.det.diverged && ~isempty(runs_b.A.det.simOut), ...
        'plot_amlpf_compare_6state: baseline a=a_true det run diverged/empty');
    so_ref = runs_b.A.det.simOut;
    P      = so_ref.meta.params_value;
    R_phys = P.common.R;
    w_hat  = P.wall.w_hat(:);
    pz     = P.wall.pz;
    Ts     = P.common.Ts;
    fs     = 1 / Ts;
    f      = cfg.frequency;

    t_e   = so_ref.tout(2:end);            % [N x 1] aligned time
    pd_al = so_ref.p_d_out(2:end, :);     % [N x 3] desired trajectory
    N     = numel(t_e);
    T_END = ceil(t_e(end));

    % Desired-trajectory h_bar and far/near (Guard-3) split
    h_bar_d = (pd_al * w_hat - pz) / R_phys;     % [N x 1]
    t_osc0    = cfg.t_hold + cfg.t_descend_override;
    osc_dur   = cfg.n_cycles / cfg.frequency;
    t_osc1    = t_osc0 + osc_dur;
    t_discard = min(1 / cfg.frequency, 0.5 * osc_dur);
    H_BAR_GATE = 1.5;
    W.osc  = (t_e >= t_osc0 + t_discard & t_e < t_osc1);
    W.far  = W.osc & (h_bar_d >= H_BAR_GATE);
    W.near = W.osc & (h_bar_d <  H_BAR_GATE);

    % smoothing window: <= 1/8 osc cycle (mirror analyzer fig_motion_var)
    w_mm = max(3, round(min(0.025, 1 / (8 * f)) * fs));

    % ----------------------------------------------------------------
    % Paired non-diverged seeds (baseline arm B vs LPF arm B), by position.
    % run_amlpf_rerun reuses the baseline seed list in order, so position i
    % is the same seed / same noise realization in both files.
    % ----------------------------------------------------------------
    nzB_base = runs_b.B.noisy;
    nzB_lpf  = lpf_B.noisy;
    Np_max   = min(numel(nzB_base), numel(nzB_lpf));
    pair_ok  = false(1, Np_max);
    for p = 1:Np_max
        pair_ok(p) = ~nzB_base(p).diverged && ~nzB_lpf(p).diverged;
    end
    pair_pos = find(pair_ok);
    assert(~isempty(pair_pos), ...
        'plot_amlpf_compare_6state: no paired non-diverged arm-B seed in baseline AND LPF');
    Np = numel(pair_pos);

    % arm A (a=a_true) floor seeds (own non-diverged set)
    nzA      = runs_b.A.noisy;
    a_ok     = find(~[nzA.diverged]);
    assert(~isempty(a_ok), 'plot_amlpf_compare_6state: all baseline a=a_true seeds diverged');

    % getters on the aligned grid (mirror analyzer e[k]=p_d[k+1]-p_true[k])
    get_ahat = @(so) so.diag.a_hat(2:end, :);
    get_atru = @(so) so.a_true_out(2:end, :);
    get_e    = @(so) so.p_d_out(2:end, :) - so.p_true_out(1:end-1, :);

    % ----------------------------------------------------------------
    % (2) a_hat ensemble: baseline vs LPF (paired seeds), each vs OWN a_true
    % ----------------------------------------------------------------
    ahB_base = stack_simout(nzB_base, pair_pos, get_ahat, N);   % [N x 3 x Np]
    atB_base = stack_simout(nzB_base, pair_pos, get_atru, N);
    ahB_lpf  = stack_simout(nzB_lpf,  pair_pos, get_ahat, N);
    atB_lpf  = stack_simout(nzB_lpf,  pair_pos, get_atru, N);

    base.ah_mean = mean(ahB_base, 3);  base.ah_std = std(ahB_base, 0, 3);
    lpf.ah_mean  = mean(ahB_lpf,  3);  lpf.ah_std  = std(ahB_lpf,  0, 3);
    at_ref       = mean(atB_base, 3);                       % red a_true reference

    relB = (ahB_base - atB_base) ./ atB_base * 100;          % [N x 3 x Np]
    relL = (ahB_lpf  - atB_lpf ) ./ atB_lpf  * 100;
    base.bias = mean(relB, 3);  base.spread = std(relB, 0, 3);   % [N x 3]
    lpf.bias  = mean(relL, 3);  lpf.spread  = std(relL, 0, 3);

    % window scalars (far / near, x=col1 / z=col3)
    ah = struct();
    for cfgname = {'base', 'lpf'}
        cn = cfgname{1};
        if strcmp(cn, 'base'); src = base; else; src = lpf; end
        for ax = [1 3]
            ah.(cn).bias_far(ax)    = mean(src.bias(W.far,  ax));
            ah.(cn).bias_near(ax)   = mean(src.bias(W.near, ax));
            ah.(cn).spread_far(ax)  = mean(src.spread(W.far,  ax));
            ah.(cn).spread_near(ax) = mean(src.spread(W.near, ax));
        end
    end

    % ----------------------------------------------------------------
    % (3) tracking error e: det (cross-seed mean) + stochastic (cross-seed std)
    % ----------------------------------------------------------------
    eB_base = stack_simout(nzB_base, pair_pos, get_e, N);    % [N x 3 x Np]
    eB_lpf  = stack_simout(nzB_lpf,  pair_pos, get_e, N);
    eA      = stack_simout(nzA,      a_ok,     get_e, N);    % a=a_true floor

    trk = struct();
    trk.base.det = mean(eB_base, 3);  trk.base.sto = std(eB_base, 0, 3);
    trk.lpf.det  = mean(eB_lpf,  3);  trk.lpf.sto  = std(eB_lpf,  0, 3);
    trk.flo.det  = mean(eA,      3);  trk.flo.sto  = std(eA,      0, 3);

    tk = struct();
    for cfgname = {'base', 'lpf', 'flo'}
        cn = cfgname{1};
        src = trk.(cn);
        for ax = [1 3]
            tk.(cn).det_rms_far(ax)  = rms(src.det(W.far,  ax)) * 1e3;   % [nm]
            tk.(cn).det_rms_near(ax) = rms(src.det(W.near, ax)) * 1e3;
            tk.(cn).sto_far(ax)      = mean(src.sto(W.far,  ax)) * 1e3;
            tk.(cn).sto_near(ax)     = mean(src.sto(W.near, ax)) * 1e3;
        end
    end

    % ----------------------------------------------------------------
    % (1) representative-seed a_m_det reconstruction (LPF run)
    % ----------------------------------------------------------------
    rep = opts.rep_seed;
    if rep < 1 || rep > numel(nzB_lpf) || nzB_lpf(rep).diverged
        rep = find(~[nzB_lpf.diverged], 1);     % fallback to first non-diverged
    end
    so_rep    = nzB_lpf(rep).simOut;
    rep_axm   = so_rep.diag.a_xm(2:end, :);     % aligned real a_xm
    rep_amd   = recon_am_det(rep_axm, a_det, so_rep.a_true_out(1, :));  % init = a_x_init
    rep_atru  = so_rep.a_true_out(2:end, :);

    % ----------------------------------------------------------------
    % Console summary
    % ----------------------------------------------------------------
    fprintf('[amlpf:%gHz] a_det=%.4g  paired seeds=%d  rep seed idx=%d\n', ...
            f, a_det, Np, rep);
    axn = 'xyz';
    for ax = [1 3]
        fprintf(['[amlpf:%gHz a_hat axis=%c] far  bias base/LPF=%+.1f/%+.1f%% ', ...
                 'spread base/LPF=%.1f/%.1f%% | near bias=%+.1f/%+.1f%% ', ...
                 'spread=%.1f/%.1f%%\n'], f, axn(ax), ...
                ah.base.bias_far(ax),   ah.lpf.bias_far(ax), ...
                ah.base.spread_far(ax), ah.lpf.spread_far(ax), ...
                ah.base.bias_near(ax),  ah.lpf.bias_near(ax), ...
                ah.base.spread_near(ax), ah.lpf.spread_near(ax));
    end
    for ax = [1 3]
        fprintf(['[amlpf:%gHz track axis=%c] far  detRMS base/LPF=%.1f/%.1f nm ', ...
                 'stoch base/LPF=%.1f/%.1f nm | near detRMS=%.1f/%.1f nm ', ...
                 'stoch=%.1f/%.1f nm\n'], f, axn(ax), ...
                tk.base.det_rms_far(ax),  tk.lpf.det_rms_far(ax), ...
                tk.base.sto_far(ax),      tk.lpf.sto_far(ax), ...
                tk.base.det_rms_near(ax), tk.lpf.det_rms_near(ax), ...
                tk.base.sto_near(ax),     tk.lpf.sto_near(ax));
    end

    % ----------------------------------------------------------------
    % Figures (analyze_gain_6state round-2 locked style)
    % ----------------------------------------------------------------
    COL_TRUE2 = [0.8 0 0];                % a_true / a=a_true floor (red)
    COL_HAT2  = [0 0.2 0.9];              % baseline (blue)
    COL_LPF   = [0.90 0.45 0];            % LPF (orange)
    COL_RAW   = [0.45 0.55 0.95 0.30];    % raw a_xm faint background (fig 1)
    FS2 = 18; LFS2 = 14; AXLW2 = 2.0;
    COLS = [1 3]; AXL = 'xz';
    tcol = t_e(:);

    if opts.save_fig
        % ===== (1) fig_am_vs_amdet : rep seed, raw a_xm vs a_m_det vs a_true =====
        f1 = figure('Position', [80 80 1100 720], 'Color', 'w', ...
                    'NumberTitle', 'off', 'Visible', 'off');
        tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
        glbl = {'a_x', 'a_z'};
        for r = 1:2
            c = COLS(r); nexttile; hold on;
            hm = plot(t_e, rep_axm(:, c), '-', 'Color', COL_RAW, ...
                      'LineWidth', 0.9, 'DisplayName', 'a_m');
            hd = plot(t_e, rep_amd(:, c), '-', 'Color', COL_HAT2, ...
                      'LineWidth', 2.5, 'DisplayName', 'a_{m,det}');
            ht = plot(t_e, rep_atru(:, c), '-', 'Color', COL_TRUE2, ...
                      'LineWidth', 2.0, 'DisplayName', 'a_{true}');
            xlim([0 T_END]);
            ylim([0, 1.25 * max(rep_atru(:, c))]);
            ylabel(sprintf('%s  (\\mum/pN)', glbl{r}), 'FontSize', FS2, 'FontWeight', 'bold');
            if r == 1
                legend([hm hd ht], 'Location', 'northoutside', 'Orientation', ...
                       'horizontal', 'FontSize', LFS2, 'FontWeight', 'bold', 'Box', 'on');
            end
            if r == 2
                xlabel('Time (sec)', 'FontSize', FS2, 'FontWeight', 'bold');
            end
            set(gca, 'FontSize', FS2, 'FontWeight', 'bold', 'LineWidth', AXLW2, 'Box', 'on');
            grid off;
        end
        exportgraphics(f1, fullfile(out_dir, 'fig_am_vs_amdet.png'), 'Resolution', 150);
        close(f1);

        % ===== (2a) fig_ahat_compare : a_hat mean +/- sigma, baseline vs LPF =====
        f2 = figure('Position', [80 80 1100 720], 'Color', 'w', ...
                    'NumberTitle', 'off', 'Visible', 'off');
        tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
        for r = 1:2
            c = COLS(r); nexttile; hold on;
            % baseline band (blue) then LPF band (orange), bands excluded from legend
            ybB = [base.ah_mean(:, c) - base.ah_std(:, c); ...
                   flipud(base.ah_mean(:, c) + base.ah_std(:, c))];
            fill([tcol; flipud(tcol)], ybB, COL_HAT2, 'FaceAlpha', 0.16, ...
                 'EdgeColor', 'none', 'HandleVisibility', 'off');
            ybL = [lpf.ah_mean(:, c) - lpf.ah_std(:, c); ...
                   flipud(lpf.ah_mean(:, c) + lpf.ah_std(:, c))];
            fill([tcol; flipud(tcol)], ybL, COL_LPF, 'FaceAlpha', 0.16, ...
                 'EdgeColor', 'none', 'HandleVisibility', 'off');
            % optional: overlay individual single-seed a_hat (raw, KF-smooth but unaveraged)
            if opts.show_seed_n > 0
                ns = min(opts.show_seed_n, size(ahB_base, 3));
                for is = 1:ns
                    plot(tcol, ahB_base(:, c, is), '-', 'Color', [0.45 0.45 0.45 0.5], ...
                         'LineWidth', 0.6, 'HandleVisibility', 'off');
                end
            end
            hT = plot(tcol, at_ref(:, c), '-', 'Color', COL_TRUE2, ...
                      'LineWidth', 2.0, 'DisplayName', 'a_{true}');
            hB = plot(tcol, base.ah_mean(:, c), '-', 'Color', COL_HAT2, ...
                      'LineWidth', 2.0, 'DisplayName', 'â base');
            hL = plot(tcol, lpf.ah_mean(:, c), '-', 'Color', COL_LPF, ...
                      'LineWidth', 2.0, 'DisplayName', 'â LPF');
            xlim([0 T_END]); ylim([0, 1.25 * max(at_ref(:, c))]);
            ylabel(sprintf('a_%c  (\\mum/pN)', AXL(r)), 'FontSize', FS2, 'FontWeight', 'bold');
            if r == 1
                legend([hB hL hT], 'Location', 'northoutside', 'Orientation', ...
                       'horizontal', 'FontSize', LFS2, 'FontWeight', 'bold', 'Box', 'on');
            end
            if r == 2
                xlabel('Time (sec)', 'FontSize', FS2, 'FontWeight', 'bold');
            end
            set(gca, 'FontSize', FS2, 'FontWeight', 'bold', 'LineWidth', AXLW2, 'Box', 'on');
            grid off;
        end
        exportgraphics(f2, fullfile(out_dir, 'fig_ahat_compare.png'), 'Resolution', 150);
        close(f2);

        % ===== (2b) fig_ahat_relerr : bias +/- spread, baseline vs LPF =====
        f3 = figure('Position', [80 80 1100 720], 'Color', 'w', ...
                    'NumberTitle', 'off', 'Visible', 'off');
        tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
        for r = 1:2
            c = COLS(r); nexttile; hold on;
            ybB = [base.bias(:, c) - base.spread(:, c); ...
                   flipud(base.bias(:, c) + base.spread(:, c))];
            fill([tcol; flipud(tcol)], ybB, COL_HAT2, 'FaceAlpha', 0.14, ...
                 'EdgeColor', 'none', 'HandleVisibility', 'off');
            ybL = [lpf.bias(:, c) - lpf.spread(:, c); ...
                   flipud(lpf.bias(:, c) + lpf.spread(:, c))];
            fill([tcol; flipud(tcol)], ybL, COL_LPF, 'FaceAlpha', 0.14, ...
                 'EdgeColor', 'none', 'HandleVisibility', 'off');
            yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
            hB = plot(tcol, base.bias(:, c), '-', 'Color', COL_HAT2, ...
                      'LineWidth', 2.0, 'DisplayName', 'base');
            hL = plot(tcol, lpf.bias(:, c), '-', 'Color', COL_LPF, ...
                      'LineWidth', 2.0, 'DisplayName', 'LPF');
            xlim([0 T_END]);
            ylabel(sprintf('(â_%c - a_{true})/a_{true}  (%%)', AXL(r)), ...
                   'FontSize', FS2, 'FontWeight', 'bold');
            % far/near numbers on the plot (study spec §6 explicitly requests this)
            note = sprintf(['far  bias b/L %+.0f/%+.0f%%  spread %.0f/%.0f%%\n', ...
                            'near bias b/L %+.0f/%+.0f%%  spread %.0f/%.0f%%'], ...
                           ah.base.bias_far(c),   ah.lpf.bias_far(c), ...
                           ah.base.spread_far(c), ah.lpf.spread_far(c), ...
                           ah.base.bias_near(c),  ah.lpf.bias_near(c), ...
                           ah.base.spread_near(c), ah.lpf.spread_near(c));
            text(0.015, 0.96, note, 'Units', 'normalized', 'FontSize', 11, ...
                 'FontWeight', 'bold', 'VerticalAlignment', 'top', ...
                 'BackgroundColor', 'w', 'EdgeColor', [0.6 0.6 0.6], 'Margin', 3);
            if r == 1
                legend([hB hL], 'Location', 'northoutside', 'Orientation', ...
                       'horizontal', 'FontSize', LFS2, 'FontWeight', 'bold', 'Box', 'on');
            end
            if r == 2
                xlabel('Time (sec)', 'FontSize', FS2, 'FontWeight', 'bold');
            end
            set(gca, 'FontSize', FS2, 'FontWeight', 'bold', 'LineWidth', AXLW2, 'Box', 'on');
            grid off;
        end
        exportgraphics(f3, fullfile(out_dir, 'fig_ahat_relerr.png'), 'Resolution', 150);
        close(f3);

        % ===== (3) fig_tracking_det_compare : z det (top) + stochastic (bottom) =====
        zc = 3;
        w_trk = w_mm; if opts.no_movmean; w_trk = 1; end   % w_trk=1 -> raw ensemble mean, no time smoothing
        f4 = figure('Position', [80 80 1100 720], 'Color', 'w', ...
                    'NumberTitle', 'off', 'Visible', 'off');
        n_tile = 2; if opts.det_only; n_tile = 1; end
        tiledlayout(n_tile, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

        % top: det component = cross-seed mean <e_z[k]> (nm), smoothed
        nexttile; hold on;
        dB = movmean(trk.base.det(:, zc) * 1e3, w_trk);
        dL = movmean(trk.lpf.det(:, zc)  * 1e3, w_trk);
        dF = movmean(trk.flo.det(:, zc)  * 1e3, w_trk);
        hBd = plot(t_e, dB, '-', 'Color', COL_HAT2,  'LineWidth', 2.5, 'DisplayName', 'base');
        hLd = plot(t_e, dL, '-', 'Color', COL_LPF,   'LineWidth', 2.0, 'DisplayName', 'LPF');
        hFd = plot(t_e, dF, '-', 'Color', COL_TRUE2, 'LineWidth', 1.0, 'DisplayName', 'a_{true}');
        yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
        xlim([0 T_END]);
        ylabel('\langle\deltaz\rangle_{det}  (nm)', 'FontSize', FS2, 'FontWeight', 'bold');
        legend([hBd hLd hFd], 'Location', 'northoutside', 'Orientation', ...
               'horizontal', 'FontSize', LFS2, 'FontWeight', 'bold', 'Box', 'on');
        if opts.det_only
            xlabel('Time (sec)', 'FontSize', FS2, 'FontWeight', 'bold');
        end
        set(gca, 'FontSize', FS2, 'FontWeight', 'bold', 'LineWidth', AXLW2, 'Box', 'on');
        grid off;

        % bottom: stochastic = cross-seed std(e_z[k]) (nm), smoothed (skipped if det_only)
        if ~opts.det_only
            nexttile; hold on;
            sB = movmean(trk.base.sto(:, zc) * 1e3, w_trk);
            sL = movmean(trk.lpf.sto(:, zc)  * 1e3, w_trk);
            sF = movmean(trk.flo.sto(:, zc)  * 1e3, w_trk);
            plot(t_e, sB, '-', 'Color', COL_HAT2,  'LineWidth', 2.5, 'DisplayName', 'base');
            plot(t_e, sL, '-', 'Color', COL_LPF,   'LineWidth', 2.0, 'DisplayName', 'LPF');
            plot(t_e, sF, '-', 'Color', COL_TRUE2, 'LineWidth', 1.0, 'DisplayName', 'a_{true}');
            xlim([0 T_END]);
            ylabel('\sigma_{seed}(\deltaz)  (nm)', 'FontSize', FS2, 'FontWeight', 'bold');
            xlabel('Time (sec)', 'FontSize', FS2, 'FontWeight', 'bold');
            set(gca, 'FontSize', FS2, 'FontWeight', 'bold', 'LineWidth', AXLW2, 'Box', 'on');
            grid off;
        end

        exportgraphics(f4, fullfile(out_dir, 'fig_tracking_det_compare.png'), 'Resolution', 150);
        close(f4);

        fprintf('[amlpf:%gHz] wrote 4 figures -> %s\n', f, out_dir);
    end

    % ----------------------------------------------------------------
    % Summary markdown
    % ----------------------------------------------------------------
    write_summary_md(fullfile(out_dir, 'amlpf_compare_summary.md'), ...
                     f, a_det, Np, baseline_freq_dir, lpf_dir, ah, tk);

    % ----------------------------------------------------------------
    % Return struct
    % ----------------------------------------------------------------
    out          = struct();
    out.freq     = f;
    out.a_det    = a_det;
    out.n_paired = Np;
    out.rep_seed = rep;
    out.t_e      = t_e;
    out.ahat     = ah;        % bias/spread far/near per config (x,z)
    out.track    = tk;        % det RMS / stochastic std far/near per config (x,z)
    out.base     = base;      % a_hat mean/std/bias/spread [N x 3]
    out.lpf      = lpf;
    out.trk      = trk;       % det/sto [N x 3] per config + floor
    out.W        = W;
    out.out_dir  = out_dir;
end   % main function


% ====================================================================
% LOCAL: stack_simout — stack a getter result over seed positions
% ====================================================================
function stk = stack_simout(nz, pos, getter, N)
%STACK_SIMOUT  [N x 3 x numel(pos)] stack of getter(nz(pos(i)).simOut).
    Ns  = numel(pos);
    stk = zeros(N, 3, Ns);
    for i = 1:Ns
        stk(:, :, i) = getter(nz(pos(i)).simOut);
    end
end


% ====================================================================
% LOCAL: recon_am_det — offline EWMA reconstruction of a_m_det
% ====================================================================
function amd = recon_am_det(a_xm, a_det, init)
%RECON_AM_DET  a_m_det EWMA matching the controller exactly.
%   amd(1) = (1-a_det)*init + a_det*a_xm(1); amd(k) = (1-a_det)*amd(k-1)+a_det*a_xm(k).
%   init = a_m_det just before the first aligned step (prefill a_x_init, passed as
%   a_true_out(1,:)); falls back to a_xm(1,:) if omitted.
    if nargin < 3 || isempty(init); init = a_xm(1, :); end
    amd       = zeros(size(a_xm));
    amd(1, :) = (1 - a_det) .* init + a_det .* a_xm(1, :);
    for k = 2:size(a_xm, 1)
        amd(k, :) = (1 - a_det) .* amd(k - 1, :) + a_det .* a_xm(k, :);
    end
end


% ====================================================================
% LOCAL: write_summary_md
% ====================================================================
function write_summary_md(path, f, a_det, Np, base_dir, lpf_dir, ah, tk)
%WRITE_SUMMARY_MD  baseline vs LPF table + headline vs 2026-04-16 prior.
    fid = fopen(path, 'w');
    if fid == -1
        warning('plot_amlpf_compare_6state:writeErr', ...
                'Cannot open %s for writing; summary skipped.', path);
        return;
    end

    fprintf(fid, '# a_m_det LPF compare : %g Hz\n\n', f);
    fprintf(fid, 'a_det = %.4g | paired non-diverged seeds = %d\n\n', a_det, Np);
    fprintf(fid, 'baseline: `%s`\n\nLPF: `%s`\n\n', base_dir, lpf_dir);

    % --- headline vs 2026-04-16 prior (7-state positioning: a_hat std -40%, ---
    %     tracking unchanged). z far is the primary evidence channel.
    sp_b = ah.base.spread_far(3);  sp_l = ah.lpf.spread_far(3);
    dt_b = tk.base.det_rms_far(3); dt_l = tk.lpf.det_rms_far(3);
    sp_chg = pct_change(sp_b, sp_l);
    dt_chg = pct_change(dt_b, dt_l);
    sp_verdict = verdict_word(sp_chg, 'spread');
    dt_verdict = verdict_word(dt_chg, 'det');
    if dt_chg <= -5
        prior = 'BREAKS the prior (LPF det shrinks tracking, not just a_hat)';
    else
        prior = 'REPRODUCES the prior (a_hat changes but tracking det does not)';
    end
    fprintf(fid, '## Headline (vs 2026-04-16 prior: a_hat std -40%%, tracking unchanged)\n\n');
    fprintf(fid, '- (2) a_hat spread (z, far): baseline %.1f%% -> LPF %.1f%%  (%+.0f%%) => %s\n', ...
            sp_b, sp_l, sp_chg, sp_verdict);
    fprintf(fid, '- (3) tracking det RMS (z, far): baseline %.1f nm -> LPF %.1f nm  (%+.0f%%) => %s\n', ...
            dt_b, dt_l, dt_chg, dt_verdict);
    fprintf(fid, '- **Verdict: %s.**\n\n', prior);

    % --- (2) a_hat bias / spread table ---
    fprintf(fid, '## (2) a_hat bias / spread [%%] (cross-seed, paired)\n\n');
    fprintf(fid, '| metric | config | x far | x near | z far | z near |\n');
    fprintf(fid, '|--------|--------|-------|--------|-------|--------|\n');
    fprintf(fid, '| bias   | base | %+.1f | %+.1f | %+.1f | %+.1f |\n', ...
            ah.base.bias_far(1), ah.base.bias_near(1), ah.base.bias_far(3), ah.base.bias_near(3));
    fprintf(fid, '| bias   | LPF  | %+.1f | %+.1f | %+.1f | %+.1f |\n', ...
            ah.lpf.bias_far(1),  ah.lpf.bias_near(1),  ah.lpf.bias_far(3),  ah.lpf.bias_near(3));
    fprintf(fid, '| spread | base | %.1f | %.1f | %.1f | %.1f |\n', ...
            ah.base.spread_far(1), ah.base.spread_near(1), ah.base.spread_far(3), ah.base.spread_near(3));
    fprintf(fid, '| spread | LPF  | %.1f | %.1f | %.1f | %.1f |\n\n', ...
            ah.lpf.spread_far(1),  ah.lpf.spread_near(1),  ah.lpf.spread_far(3),  ah.lpf.spread_near(3));
    fprintf(fid, ['bias = mean((a_hat-a_true)/a_true); spread = std over seeds of the same, ', ...
                  'window-averaged. far = h_bar>=1.5, near = h_bar<1.5 (Guard-3).\n\n']);

    % --- (3) tracking error table ---
    fprintf(fid, '## (3) tracking error [nm]\n\n');
    fprintf(fid, '| metric | config | x far | x near | z far | z near |\n');
    fprintf(fid, '|--------|--------|-------|--------|-------|--------|\n');
    cns = {'base', 'lpf', 'flo'};
    cls = {'base', 'LPF', 'a_true'};
    for i = 1:3
        cn = cns{i};
        fprintf(fid, '| det RMS | %s | %.1f | %.1f | %.1f | %.1f |\n', cls{i}, ...
                tk.(cn).det_rms_far(1), tk.(cn).det_rms_near(1), ...
                tk.(cn).det_rms_far(3), tk.(cn).det_rms_near(3));
    end
    for i = 1:3
        cn = cns{i};
        fprintf(fid, '| stoch std | %s | %.1f | %.1f | %.1f | %.1f |\n', cls{i}, ...
                tk.(cn).sto_far(1), tk.(cn).sto_near(1), ...
                tk.(cn).sto_far(3), tk.(cn).sto_near(3));
    end
    fprintf(fid, ['\ndet = cross-seed ensemble mean <e[k]> (systematic, gain mismatch); ', ...
                  'stoch = cross-seed std(e[k]). a_true row = a=a_true floor (baseline arm A).\n']);
    fclose(fid);
end


% ====================================================================
% LOCAL: pct_change / verdict_word — small summary helpers
% ====================================================================
function p = pct_change(a, b)
%PCT_CHANGE  percent change from a (baseline) to b (LPF); 0 if a ~ 0.
    if abs(a) < eps
        p = 0;
    else
        p = (b - a) / a * 100;
    end
end


function w = verdict_word(chg, kind)
%VERDICT_WORD  qualitative reduction verdict for a percent change.
    if chg <= -5
        w = sprintf('%s REDUCED', kind);
    elseif chg >= 5
        w = sprintf('%s INCREASED', kind);
    else
        w = sprintf('%s ~unchanged', kind);
    end
end
