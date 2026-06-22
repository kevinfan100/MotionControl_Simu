function results = verify_scale_calib_6state(freq, opts)
%VERIFY_SCALE_CALIB_6STATE  P1 scale-calibration feasibility test (a_xm channel).
%
%   results = verify_scale_calib_6state(freq)
%   results = verify_scale_calib_6state(freq, opts)
%
%   OFFLINE, READ-ONLY post-processing of an existing gain_compare run. Answers
%   one question: is the a_xm feedback channel good enough to pin the motion-gain
%   SCALE (P1), the easiest of the three model-error layers (P1 scale / P2 shape /
%   P3 coordinate)? Scale is the only thing estimable by pooling ALL heights,
%   because dividing a_xm by the model shape removes the height dependence:
%
%       a_x(h) = a_nom * g(h),   g(h) = 1/c(h_bar)        (model)
%       r[k]   = a_xm[k] / a_x_model(h_bar[k])  ~  kappa  (height-independent)
%
%   Here a_x_model is the unit-scale skeleton a_nom/c(h_bar) and the unknown scale
%   kappa = a_nom_true / a_nom_assumed. Because both a_xm and the true gain
%   a_true_out are logged per seed, an injected kappa is purely a denominator
%   choice (a_nom_assumed = a_nom_true/kappa), NO sim rerun:
%
%       r[k] = a_xm[k] / (a_x_model(h_bar[k]) / kappa) = kappa * (a_xm[k]/a_true[k])
%       kappa_hat = fold-mean over (osc samples x seeds) of r        (linear in kappa)
%
%   Folding (temporal averaging over the osc window + ensemble averaging over
%   seeds) collapses the per-sample chi-squared noise of a_xm. The recovery error
%   of kappa_hat is the a_xm bias (~<1.5% for the a=a_true arm); its standard
%   error shrinks as 1/sqrt(Ns).
%
%   The same per-height ratio r_bar(h_bar) is a free P2 (shape) detector:
%     - FLAT  -> shape g(h) consistent, only scale matters (sim a=a_true arm).
%     - TREND -> a height-dependent deviation (e.g. the a=a_hat arm's near-wall
%                bias, or, with opts.demo_shift, an injected coordinate error).
%
%   Channel-independence: the a_xm IIR chain (paper2025 Eq.9-13 + linear
%   inversion) is byte-identical across the 6/5/4-state controllers, so a result
%   obtained from the 6-state gain_compare runs.mat applies to the 4-state too.
%
%   INPUTS
%     freq : trajectory frequency in Hz (default 1). Selects
%            test_results/gain_compare/f<freq>Hz/runs.mat.
%     opts : optional struct
%       data_root   source dir holding f<freq>Hz/runs.mat
%                   (default <project>/test_results/gain_compare)
%       out_root    output root (default <project>/test_results/scale_calib)
%       n_bin       h_bar bins for the r_bar(h_bar) diagnostic (default 15)
%       kappa_sweep injected true-scale values (default [0.8 0.9 1.0 1.1 1.2])
%       seed_grid   seed counts for the SNR-vs-seeds figure
%                   (default [25 50 100 200], capped at available Ns)
%       demo_shift  wall-offset error in units of R applied to the model skeleton
%                   ONLY (h_bar_model = h_bar_d - demo_shift) to demonstrate the
%                   r_bar(h_bar) shape/coordinate detector (default 0 = off)
%       save_fig    write PNG figures (default true)
%       verbose     print progress (default true)
%
%   OUTPUTS (written to test_results/scale_calib/f<freq>Hz):
%     fig_scale_recover.png  kappa_hat vs kappa_true (y=x), arms A/B, x & z
%     fig_ratio_vs_h.png     r_bar(h_bar) at kappa=1, arms A/B (flat vs trend)
%     fig_snr_vs_seeds.png   SE(kappa_hat) vs seed count, with 1/sqrt(Ns) ref
%     scale_calib_summary.md per-axis/per-arm kappa_hat, recovery, flatness
%     scale_calib_verify.mat all computed arrays
%
%   See also: verify_axm_cdpmr_6state, plot_q55_6state, compare_gain_6state

    % ----------------------------------------------------------------
    % Input handling
    % ----------------------------------------------------------------
    if nargin < 1 || isempty(freq); freq = 1; end
    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'n_bin');       opts.n_bin       = 15;                 end
    if ~isfield(opts, 'kappa_sweep'); opts.kappa_sweep = [0.8 0.9 1.0 1.1 1.2]; end
    if ~isfield(opts, 'seed_grid');   opts.seed_grid   = [25 50 100 200];    end
    if ~isfield(opts, 'demo_shift');  opts.demo_shift  = 0;                  end
    if ~isfield(opts, 'save_fig');    opts.save_fig    = true;               end
    if ~isfield(opts, 'verbose');     opts.verbose     = true;               end

    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(fileparts(script_dir));
    addpath(fullfile(project_root, 'model', 'wall_effect'));
    if ~isfield(opts, 'data_root')
        opts.data_root = fullfile(project_root, 'test_results', 'gain_compare');
    end
    if ~isfield(opts, 'out_root')
        opts.out_root = fullfile(project_root, 'test_results', 'scale_calib');
    end
    freq_name = sprintf('f%gHz', freq);
    out_dir   = fullfile(opts.out_root, freq_name);
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end

    % ----------------------------------------------------------------
    % Load
    % ----------------------------------------------------------------
    runs_path = fullfile(opts.data_root, freq_name, 'runs.mat');
    if ~exist(runs_path, 'file')
        error('verify_scale_calib_6state:noFile', 'runs.mat not found: %s', runs_path);
    end
    S    = load(runs_path);
    runs = S.runs;
    cfg  = S.cfg;
    clear S;

    % Pre-flight asserts (mirror verify_axm_cdpmr_6state)
    ok_ref = find(~[runs.A.noisy.diverged], 1);
    assert(~isempty(ok_ref), 'verify_scale_calib_6state: all a=a_true seeds diverged');
    so_ref = runs.A.noisy(ok_ref).simOut;
    assert(isfield(so_ref, 'diag') && isfield(so_ref.diag, 'a_xm'), ...
        ['verify_scale_calib_6state: simOut.diag.a_xm missing. ', ...
         'Re-run compare_gain_6state with collect_diag=true.']);
    assert(isfield(so_ref, 'a_true_out'), ...
        'verify_scale_calib_6state: simOut.a_true_out missing');

    % ----------------------------------------------------------------
    % Constants + aligned grid (e[k] = pd[k+1]-p[k] convention -> use 2:end)
    % ----------------------------------------------------------------
    P       = runs.A.det.simOut.meta.params_value;
    R_phys  = P.common.R;
    w_hat   = P.wall.w_hat(:);
    pz      = P.wall.pz;
    a_nom   = P.common.Ts / P.common.gamma_N;     % a_nom_true [um/pN]

    t_e     = so_ref.tout(2:end);
    pd_al   = so_ref.p_d_out(2:end, :);
    N       = numel(t_e);
    h_bar_d = (pd_al * w_hat - pz) / R_phys;       % [N x 1] desired-trajectory h_bar

    % Oscillation analysis window (drop 1st cycle transient; mirror verify_axm)
    t_osc0    = cfg.t_hold + cfg.t_descend_override;
    osc_dur   = cfg.n_cycles / cfg.frequency;
    t_osc1    = t_osc0 + osc_dur;
    t_discard = min(1 / cfg.frequency, 0.5 * osc_dur);
    osc       = (t_e >= t_osc0 + t_discard) & (t_e < t_osc1);   % [N x 1] logical
    n_osc     = sum(osc);
    assert(n_osc > 0, 'verify_scale_calib_6state: empty osc window');

    % ----------------------------------------------------------------
    % Model skeleton a_x_model = a_nom / c(h_bar_model), unit scale (kappa=1).
    % h_bar_model = desired h_bar minus an optional injected coordinate error
    % (opts.demo_shift, in units of R) used ONLY to exercise the shape detector.
    % x,y use c_para; z uses c_perp (wall normal).
    % ----------------------------------------------------------------
    hbm = max(h_bar_d - opts.demo_shift, 1.001);
    c_pa = zeros(N, 1);
    c_pe = zeros(N, 1);
    for ki = 1:N
        [c_pa(ki), c_pe(ki)] = calc_correction_functions(hbm(ki));
    end
    a_model = [a_nom ./ c_pa, a_nom ./ c_pa, a_nom ./ c_pe];   % [N x 3] unit-scale

    % ----------------------------------------------------------------
    % Per-arm extraction + per-seed scale estimates
    %   ratio_true[k,c,s] = a_xm / a_true_out   (CHANNEL-ONLY: isolates a_xm)
    %   ratio_mod [k,c,s] = a_xm / a_model      (REALISTIC: model skeleton)
    %   per-seed kappa_unit = mean over osc window of the ratio
    % ----------------------------------------------------------------
    ARMS = {'A', 'B'};
    Vr   = struct();
    for ai = 1:2
        arm = ARMS{ai};
        nz  = runs.(arm).noisy;
        ok  = find(~[nz.diverged]);
        assert(~isempty(ok), 'verify_scale_calib_6state: all arm %s seeds diverged', arm);
        Ns  = numel(ok);

        axm = zeros(N, 3, Ns);
        atr = zeros(N, 3, Ns);
        for si = 1:Ns
            s = ok(si);
            axm(:, :, si) = nz(s).simOut.diag.a_xm(2:end, :);
            atr(:, :, si) = nz(s).simOut.a_true_out(2:end, :);
        end

        ratio_true = axm ./ atr;                                % [N x 3 x Ns]
        ratio_mod  = axm ./ reshape(a_model, [N, 3, 1]);        % broadcast model

        % per-seed scale (mean of ratio over osc window) -> [3 x Ns]
        ksd_true = squeeze(mean(ratio_true(osc, :, :), 1));
        ksd_mod  = squeeze(mean(ratio_mod(osc, :, :),  1));
        if Ns == 1   % squeeze collapses the seed dim; restore [3 x 1]
            ksd_true = ksd_true(:); ksd_mod = ksd_mod(:);
        end

        Vr.(arm).Ns         = Ns;
        Vr.(arm).ratio_true = ratio_true;
        Vr.(arm).ratio_mod  = ratio_mod;
        Vr.(arm).a_xm_ens   = mean(axm, 3);                     % [N x 3]
        Vr.(arm).a_true_ens = mean(atr, 3);                     % [N x 3]
        Vr.(arm).ksd_true   = ksd_true;                         % [3 x Ns]
        Vr.(arm).ksd_mod    = ksd_mod;                          % [3 x Ns]
        Vr.(arm).kappa_unit_true = mean(ksd_true, 2);           % [3 x 1] channel-only
        Vr.(arm).kappa_unit_mod  = mean(ksd_mod,  2);           % [3 x 1] realistic
        Vr.(arm).se_unit_true    = std(ksd_true, 0, 2) / sqrt(Ns);
        Vr.(arm).se_unit_mod     = std(ksd_mod,  0, 2) / sqrt(Ns);

        if opts.verbose
            fprintf(['[scale_calib:%gHz arm %s] Ns=%d  kappa_unit(true-anchor) ', ...
                     'x=%.4f z=%.4f  SE x=%.2g z=%.2g\n'], freq, arm, Ns, ...
                     Vr.(arm).kappa_unit_true(1), Vr.(arm).kappa_unit_true(3), ...
                     Vr.(arm).se_unit_true(1),    Vr.(arm).se_unit_true(3));
        end
    end

    % ----------------------------------------------------------------
    % kappa recovery sweep: kappa_hat(kappa) = kappa * kappa_unit (linear).
    % Channel-only anchor is the headline (purest test of the a_xm channel).
    % ----------------------------------------------------------------
    ks = opts.kappa_sweep(:).';                                 % [1 x nK]
    recover = struct();
    for ai = 1:2
        arm = ARMS{ai};
        recover.(arm).kappa_true = ks;
        recover.(arm).kappa_hat  = Vr.(arm).kappa_unit_true * ks;          % [3 x nK]
        recover.(arm).kappa_se   = Vr.(arm).se_unit_true    * ks;          % [3 x nK]
    end

    % ----------------------------------------------------------------
    % r_bar(h_bar): channel-only ratio folded by h_bar bin (kappa=1).
    % Mean over (osc samples in bin x seeds); SEM across seeds.
    % FLAT -> consistent; TREND -> shape/coordinate deviation (P2/P3 detector).
    % ----------------------------------------------------------------
    rbar = struct();
    for ai = 1:2
        arm = ARMS{ai};
        rbar.(arm) = bin_ratio_vs_h(Vr.(arm).ratio_true, h_bar_d, osc, opts.n_bin);
        % flatness: slope of a least-squares line through r_bar(h_bar), per axis
        rbar.(arm).slope = lsq_slope(rbar.(arm).x, rbar.(arm).rmean);       % [1 x 3]
    end

    % ----------------------------------------------------------------
    % SNR vs seeds: SE(kappa_hat) vs subsampled seed count (channel-only).
    % Uses arm A per-seed scale estimates; expect SE ~ 1/sqrt(Ns).
    % ----------------------------------------------------------------
    Ns_A    = Vr.A.Ns;
    sg      = opts.seed_grid(opts.seed_grid <= Ns_A);
    if isempty(sg); sg = Ns_A; end
    sg      = unique([sg, Ns_A]);
    se_grid = zeros(numel(sg), 3);
    for gi = 1:numel(sg)
        m = sg(gi);
        se_grid(gi, :) = std(Vr.A.ksd_true(:, 1:m), 0, 2).' / sqrt(m);
    end

    % ================================================================
    % FIGURES (locked style mirrors verify_axm_cdpmr_6state / plot_q55_6state)
    % ================================================================
    COL_REF  = [0.4 0.4 0.4];      % y=x / unity reference (gray)
    COL_A    = [0.8 0 0];          % arm A (a=a_true)  red
    COL_B    = [0 0.2 0.9];        % arm B (a=a_hat)   blue
    COL_TH   = [0 0.6 0];          % reference / theory (green)
    FS = 18; LFS = 14; AXLW = 2.0;
    COLS = [1 3]; AXL = 'xz';

    if opts.save_fig
        % ---- fig_scale_recover: kappa_hat vs kappa_true (y=x recovery) ----
        f1 = figure('Position', [80 80 1100 760], 'Color', 'w', ...
                    'NumberTitle', 'off', 'Visible', 'off');
        tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
        for r = 1:2
            c = COLS(r); nexttile; hold on;
            lo = min(ks) * 0.95; hi = max(ks) * 1.05;
            hRef = plot([lo hi], [lo hi], '--', 'Color', COL_REF, ...
                        'LineWidth', 1.5, 'DisplayName', 'y = x');
            hA = errorbar(ks, recover.A.kappa_hat(c, :), recover.A.kappa_se(c, :), 'o', ...
                          'Color', COL_A, 'MarkerFaceColor', COL_A, 'MarkerSize', 8, ...
                          'LineWidth', 1.5, 'CapSize', 5, 'LineStyle', 'none', ...
                          'DisplayName', 'a = a_{true}');
            hB = errorbar(ks, recover.B.kappa_hat(c, :), recover.B.kappa_se(c, :), 's', ...
                          'Color', COL_B, 'MarkerFaceColor', COL_B, 'MarkerSize', 8, ...
                          'LineWidth', 1.5, 'CapSize', 5, 'LineStyle', 'none', ...
                          'DisplayName', 'a = â');
            xlim([lo hi]); ylim([lo hi]);
            ylabel(sprintf('$\\hat{\\kappa}_{%c}$', AXL(r)), ...
                   'Interpreter', 'latex', 'FontSize', FS, 'FontWeight', 'bold');
            if r == 1
                legend([hRef hA hB], 'Location', 'northoutside', ...
                       'Orientation', 'horizontal', 'FontSize', LFS, ...
                       'FontWeight', 'bold', 'Box', 'on');
            end
            if r == 2
                xlabel('\kappa_{true} (injected scale)', 'FontSize', FS, 'FontWeight', 'bold');
            end
            set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
            grid off;
        end
        exportgraphics(f1, fullfile(out_dir, 'fig_scale_recover.png'), 'Resolution', 150);
        close(f1);
        if opts.verbose; fprintf('[scale_calib] wrote fig_scale_recover.png\n'); end

        % ---- fig_ratio_vs_h: r_bar(h_bar) flatness (kappa=1), arms A/B ----
        f2 = figure('Position', [80 80 1100 760], 'Color', 'w', ...
                    'NumberTitle', 'off', 'Visible', 'off');
        tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
        for r = 1:2
            c = COLS(r); nexttile; hold on;
            hRef = yline(1, '--', 'Color', COL_TH, 'LineWidth', 2.0, ...
                         'DisplayName', 'unity (consistent)');
            hA = errorbar(rbar.A.x, rbar.A.rmean(:, c), rbar.A.rsem(:, c), 'o-', ...
                          'Color', COL_A, 'MarkerFaceColor', COL_A, 'MarkerSize', 6, ...
                          'LineWidth', 1.5, 'CapSize', 4, 'DisplayName', 'a = a_{true}');
            hB = errorbar(rbar.B.x, rbar.B.rmean(:, c), rbar.B.rsem(:, c), 's-', ...
                          'Color', COL_B, 'MarkerFaceColor', COL_B, 'MarkerSize', 6, ...
                          'LineWidth', 1.5, 'CapSize', 4, 'DisplayName', 'a = â');
            ylabel(sprintf('$\\bar r_{%c}(\\bar h)=a_{xm}/a_{true}$', AXL(r)), ...
                   'Interpreter', 'latex', 'FontSize', FS, 'FontWeight', 'bold');
            if r == 1
                legend([hRef hA hB], 'Location', 'northoutside', ...
                       'Orientation', 'horizontal', 'FontSize', LFS, ...
                       'FontWeight', 'bold', 'Box', 'on');
            end
            if r == 2
                xlabel('$\bar h = h/R$', 'Interpreter', 'latex', ...
                       'FontSize', FS, 'FontWeight', 'bold');
            end
            set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
            grid off;
        end
        exportgraphics(f2, fullfile(out_dir, 'fig_ratio_vs_h.png'), 'Resolution', 150);
        close(f2);
        if opts.verbose; fprintf('[scale_calib] wrote fig_ratio_vs_h.png\n'); end

        % ---- fig_snr_vs_seeds: SE(kappa_hat) vs Ns, with 1/sqrt(Ns) ref ----
        f3 = figure('Position', [80 80 1100 760], 'Color', 'w', ...
                    'NumberTitle', 'off', 'Visible', 'off');
        tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
        for r = 1:2
            c = COLS(r); nexttile; hold on;
            se_ref = se_grid(end, c) * sqrt(sg(end) ./ sg(:));   % anchored at full Ns
            hRef = plot(sg, se_ref, '--', 'Color', COL_REF, 'LineWidth', 2.0, ...
                        'DisplayName', '1/\surd{N_s}');
            hM = plot(sg, se_grid(:, c), 'o-', 'Color', COL_A, ...
                      'MarkerFaceColor', COL_A, 'MarkerSize', 8, 'LineWidth', 2.0, ...
                      'DisplayName', 'measured SE');
            set(gca, 'XScale', 'log', 'YScale', 'log');
            ylabel(sprintf('SE($\\hat{\\kappa}_{%c}$)', AXL(r)), ...
                   'Interpreter', 'latex', 'FontSize', FS, 'FontWeight', 'bold');
            if r == 1
                legend([hM hRef], 'Location', 'northoutside', ...
                       'Orientation', 'horizontal', 'FontSize', LFS, ...
                       'FontWeight', 'bold', 'Box', 'on');
            end
            if r == 2
                xlabel('N_{seeds}', 'FontSize', FS, 'FontWeight', 'bold');
            end
            xlim([min(sg) * 0.9, max(sg) * 1.1]);
            set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
            grid off;
        end
        exportgraphics(f3, fullfile(out_dir, 'fig_snr_vs_seeds.png'), 'Resolution', 150);
        close(f3);
        if opts.verbose; fprintf('[scale_calib] wrote fig_snr_vs_seeds.png\n'); end
    end

    % ================================================================
    % SAVE .mat
    % ================================================================
    save(fullfile(out_dir, 'scale_calib_verify.mat'), ...
         'Vr', 'recover', 'rbar', 'se_grid', 'sg', 'ks', ...
         'h_bar_d', 'osc', 'a_model', 'a_nom', 'cfg');
    if opts.verbose; fprintf('[scale_calib] wrote scale_calib_verify.mat\n'); end

    % ================================================================
    % WRITE SUMMARY .md
    % ================================================================
    write_summary_md(fullfile(out_dir, 'scale_calib_summary.md'), ...
                     freq, Vr, rbar, sg, se_grid, opts);
    if opts.verbose; fprintf('[scale_calib] wrote scale_calib_summary.md\n'); end

    % ================================================================
    % RETURN STRUCT
    % ================================================================
    results          = struct();
    results.Vr       = Vr;
    results.recover  = recover;
    results.rbar     = rbar;
    results.se_grid  = se_grid;
    results.sg       = sg;
    results.ks       = ks;
    results.a_nom    = a_nom;
    results.out_dir  = out_dir;

    if opts.verbose; fprintf('[scale_calib] done -> %s\n', out_dir); end
end   % main


% ====================================================================
% LOCAL: bin_ratio_vs_h
% ====================================================================
function B = bin_ratio_vs_h(ratio, h_bar_d, mask, n_bin)
%BIN_RATIO_VS_H  Fold ratio[N,3,Ns] by h_bar bin over masked samples.
%   Returns B.x [nb x 1] bin-center h_bar, B.rmean [nb x 3] mean over
%   (samples-in-bin x seeds), B.rsem [nb x 3] SEM across seeds (per-seed
%   bin mean -> std/sqrt(Ns)). Bins with < MIN_COUNT samples are dropped.
    MIN_COUNT = 3;
    [~, ~, Ns] = size(ratio);
    idx   = find(mask(:));
    hb    = h_bar_d(idx);
    if isempty(hb)
        B.x = zeros(0,1); B.rmean = zeros(0,3); B.rsem = zeros(0,3); return;
    end
    edges = linspace(min(hb), max(hb), n_bin + 1);
    x = nan(n_bin, 1); rmean = nan(n_bin, 3); rsem = nan(n_bin, 3);
    for b = 1:n_bin
        if b < n_bin
            in = hb >= edges(b) & hb < edges(b+1);
        else
            in = hb >= edges(b) & hb <= edges(b+1);
        end
        sel = idx(in);
        if numel(sel) < MIN_COUNT; continue; end
        x(b) = mean(h_bar_d(sel));
        sub  = ratio(sel, :, :);                        % [m x 3 x Ns]
        rmean(b, :) = mean(reshape(sub, [], 3), 1);     % over samples + seeds
        if Ns >= 2
            per_seed = squeeze(mean(sub, 1));           % [3 x Ns]
            rsem(b, :) = std(per_seed, 0, 2).' / sqrt(Ns);
        else
            rsem(b, :) = 0;
        end
    end
    keep    = ~isnan(x);
    B.x     = x(keep);
    B.rmean = rmean(keep, :);
    B.rsem  = rsem(keep, :);
end


% ====================================================================
% LOCAL: lsq_slope
% ====================================================================
function m = lsq_slope(x, Y)
%LSQ_SLOPE  Least-squares slope of each column of Y vs x (NaN-safe). [1 x ncol].
    x = x(:);
    ncol = size(Y, 2);
    m = nan(1, ncol);
    for c = 1:ncol
        y = Y(:, c);
        good = ~isnan(x) & ~isnan(y);
        if sum(good) < 2; continue; end
        xc = x(good) - mean(x(good));
        yc = y(good) - mean(y(good));
        denom = sum(xc.^2);
        if denom > 0; m(c) = sum(xc .* yc) / denom; end
    end
end


% ====================================================================
% LOCAL: write_summary_md
% ====================================================================
function write_summary_md(path, freq, Vr, rbar, sg, se_grid, opts)
%WRITE_SUMMARY_MD  P1 scale-calibration feasibility summary.
    fid = fopen(path, 'w');
    if fid == -1
        warning('verify_scale_calib_6state:writeErr', 'Cannot open %s', path);
        return;
    end
    AX = {'x', 'y', 'z'};

    fprintf(fid, '# P1 scale-calibration feasibility : %g Hz\n\n', freq);
    fprintf(fid, ['freq=%.4g Hz | a=a_true = %d seeds | a=â = %d seeds | ', ...
                  'osc window only | demo_shift=%.3g R\n\n'], ...
            freq, Vr.A.Ns, Vr.B.Ns, opts.demo_shift);
    fprintf(fid, ['Estimator: kappa_hat = fold-mean over (osc samples x seeds) of ', ...
                  'a_xm / a_true (channel-only anchor). Scale is a linear parameter, ', ...
                  'so an injected kappa is recovered as kappa_hat = kappa * kappa_unit; ', ...
                  'the substance is kappa_unit (bias) and its SE (noise floor).\n\n']);

    % --- headline: kappa_unit (channel-only) + recovery error ---
    fprintf(fid, '## Headline - can a_xm pin the scale? (a=a_true arm, channel-only)\n\n');
    fprintf(fid, '| axis | kappa_unit | recovery err | SE | rel SE |\n');
    fprintf(fid, '|------|-----------|--------------|----|--------|\n');
    for c = 1:3
        ku = Vr.A.kappa_unit_true(c);
        se = Vr.A.se_unit_true(c);
        fprintf(fid, '| %s | %.4f | %+.2f%% | %.2g | %.2f%% |\n', ...
                AX{c}, ku, (ku - 1) * 100, se, se / ku * 100);
    end
    fprintf(fid, ['\nRead: kappa_unit ~ 1 means the a_xm channel recovers the true scale; ', ...
                  'the offset from 1 is the known a_xm bias (target <1.5%% on x/z desc/far). ', ...
                  'rel SE is the post-fold noise floor (folding collapses the ~15-31%% ', ...
                  'per-sample chi-squared noise).\n\n']);

    % --- realistic (model-skeleton) anchor, for comparison ---
    fprintf(fid, '## Realistic anchor (a_xm / model skeleton a_nom/c(h_bar_d))\n\n');
    fprintf(fid, '| axis | kappa_unit (model) | vs channel-only |\n');
    fprintf(fid, '|------|--------------------|------------------|\n');
    for c = 1:3
        km = Vr.A.kappa_unit_mod(c);
        kt = Vr.A.kappa_unit_true(c);
        fprintf(fid, '| %s | %.4f | %+.2f%% |\n', AX{c}, km, (km - kt) / kt * 100);
    end
    fprintf(fid, ['\nGap = model-skeleton vs true-position anchor = tracking / position-noise ', ...
                  'coupling (h_bar_d vs h_bar_true). Small gap confirms the desired-trajectory ', ...
                  'skeleton is an adequate stand-in for the unknown true height.\n\n']);

    % --- shape diagnostic: r_bar(h_bar) flatness slope ---
    fprintf(fid, '## Shape detector - r_bar(h_bar) slope (P2/P3)\n\n');
    fprintf(fid, '| arm | slope_x | slope_z |\n');
    fprintf(fid, '|-----|---------|---------|\n');
    fprintf(fid, '| a=a_true | %+.4f | %+.4f |\n', rbar.A.slope(1), rbar.A.slope(3));
    fprintf(fid, '| a=â      | %+.4f | %+.4f |\n\n', rbar.B.slope(1), rbar.B.slope(3));
    fprintf(fid, ['Read: a=a_true slope ~ 0 -> model shape consistent (only P1 scale). ', ...
                  'a=â shows a near-wall trend = the self-consistent gain-mismatch bias ', ...
                  '(a height-dependent deviation the same detector would flag for a P2/P3 ', ...
                  'shape/coordinate error; set opts.demo_shift>0 to inject one).\n\n']);

    % --- SNR vs seeds ---
    fprintf(fid, '## SNR vs seeds - SE(kappa_hat) (a=a_true, z axis)\n\n');
    fprintf(fid, '| N_seeds | SE_x | SE_z |\n');
    fprintf(fid, '|---------|------|------|\n');
    for gi = 1:numel(sg)
        fprintf(fid, '| %d | %.2g | %.2g |\n', sg(gi), se_grid(gi, 1), se_grid(gi, 3));
    end
    fprintf(fid, ['\nExpect SE ~ 1/sqrt(N_seeds): a 4x seed increase should roughly halve SE. ', ...
                  'This is the folding lever that makes a noisy per-sample channel usable for ', ...
                  'a single scalar.\n\n']);

    % --- verdict ---
    fprintf(fid, '## Verdict\n\n');
    fprintf(fid, ['If kappa_unit is within a few %% of 1 with rel SE well under 1%%, the a_xm ', ...
                  'feedback channel is MORE than sufficient for P1 scale calibration ', ...
                  '(grey-box). Free-form a''(h) shape estimation (P2) remains the harder, ', ...
                  'SNR-bound problem -- gated on this result.\n']);
    fclose(fid);
end
