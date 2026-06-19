function out = verify_r22_amlpf_6state(freq_dir, opts)
%VERIFY_R22_AMLPF_6STATE  P3 of the a_m_det LPF study (am_lpf_r22_design.md).
%   Validates the cascade R22_new closed form WITHOUT a closed-loop rerun, by
%   reconstructing a_m_det offline from the baseline a=a_true arm (which is
%   LPF-invariant: control uses a_true, so the a_xm/a_m_det chain is a passive
%   observer at the strict lambda_c pole).
%
%   out = verify_r22_amlpf_6state(freq_dir, opts)
%
%   freq_dir : baseline gain_compare dir holding runs.mat (e.g.
%              test_results/gain_compare/f1Hz)
%   opts.a_det    : LPF EWMA weight                       (default 0.005)
%   opts.n_bin    : a-axis bins for scatter               (default 28)
%   opts.save_fig : write PNGs                            (default true)
%   opts.out_dir  : output dir (default test_results/am_lpf/f<f>Hz/r22_verify)
%
%   Verifies (cross-seed ensemble var on the a=a_true arm; deterministic a(t)
%   cancels across seeds, leaving the stochastic part the formulas predict):
%     V-i   Var(a_xm)    ~= K_var*IF_eff*(a_true+xi)^2          (re-confirms axm_cdpmr)
%     V-ii  rho_axm(tau) ~= (1-a_cov)^tau                       (AR(1) approx for IF2)
%     V-iii Var(a_m_det) ~= amlpf_var_factor*K_var*IF_eff*(a_true+xi)^2  (HEADLINE)
%           amlpf_var_factor = [a_det/(2-a_det)] * IF2,  IF2 = 1 + 2 s2/(1-s2),
%           s2 = (1-a_det)(1-a_cov).
%
%   Figures: fig_r22_var_time, fig_r22_var_scatter, fig_rho_axm + summary.md.
%
%   See also: verify_axm_cdpmr_6state, run_amlpf_rerun_6state, build_eq17_6state_constants

    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'a_det')    || isempty(opts.a_det);    opts.a_det = 0.005; end
    if ~isfield(opts, 'n_bin')    || isempty(opts.n_bin);    opts.n_bin = 28;    end
    if ~isfield(opts, 'save_fig') || isempty(opts.save_fig); opts.save_fig = true; end
    if ~isfield(opts, 'verbose')  || isempty(opts.verbose);  opts.verbose = true;  end

    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(fileparts(script_dir));
    addpath(fullfile(project_root, 'model'), fullfile(project_root, 'model', 'config'), ...
            fullfile(project_root, 'model', 'wall_effect'), script_dir);

    % ---------------- Load ----------------
    runs_path = fullfile(freq_dir, 'runs.mat');
    assert(exist(runs_path, 'file') == 2, 'runs.mat not found: %s', runs_path);
    S = load(runs_path);
    runs = S.runs; cfg = S.cfg;

    ok_ref = find(~[runs.A.noisy.diverged], 1);
    assert(~isempty(ok_ref), 'verify_r22_amlpf: all a=a_true noisy seeds diverged');
    so_ref = runs.A.noisy(ok_ref).simOut;
    assert(isfield(so_ref, 'diag') && isfield(so_ref.diag, 'a_xm'), ...
           'verify_r22_amlpf: simOut.diag.a_xm missing (re-run with collect_diag)');

    % ---------------- Constants ----------------
    P      = runs.A.det.simOut.meta.params_value;
    kBT    = P.ctrl.k_B * P.ctrl.T;
    R_phys = P.common.R;
    w_hat  = P.wall.w_hat(:);
    pz     = P.wall.pz;
    Ts     = P.common.Ts;
    fs     = 1 / Ts;
    sig2_n = cfg.meas_noise_std(:).^2;       % [3x1]
    cc     = so_ref.ctrl_const;
    C_dpmr = cc.C_dpmr; C_n = cc.C_n;
    K_var  = cc.K_var; IF_abc = cc.IF_abc(:); xi_ax = cc.xi_per_axis(:).';
    a_cov  = cfg.a_cov;
    a_det  = opts.a_det;

    % cascade factor (same formula as build_eq17_6state_constants)
    K_var2 = a_det / (2 - a_det);
    s2     = (1 - a_det) * (1 - a_cov);
    IF2    = 1 + 2 * s2 / (1 - s2);
    amlpf_var_factor = K_var2 * IF2;

    f = cfg.frequency;
    if ~isfield(opts, 'out_dir') || isempty(opts.out_dir)
        opts.out_dir = fullfile(project_root, 'test_results', 'am_lpf', ...
                                sprintf('f%gHz', f), 'r22_verify');
    end
    if opts.save_fig && ~exist(opts.out_dir, 'dir'); mkdir(opts.out_dir); end

    if opts.verbose
        fprintf('[r22_amlpf] a_cov=%.4g a_det=%.4g | IF2=%.4f K_var2=%.5f -> amlpf_var_factor=%.5f\n', ...
                a_cov, a_det, IF2, K_var2, amlpf_var_factor);
        fprintf('[r22_amlpf] (predicts Var(a_m_det)/Var(a_xm) = %.4f -> std ratio %.3f)\n', ...
                amlpf_var_factor, sqrt(amlpf_var_factor));
    end

    % ---------------- Aligned grid + windows ----------------
    t_e   = so_ref.tout(2:end);
    pd_al = so_ref.p_d_out(2:end, :);
    N     = numel(t_e);
    h_bar_d   = (pd_al * w_hat - pz) / R_phys;
    t_osc0    = cfg.t_hold + cfg.t_descend_override;
    osc_dur   = cfg.n_cycles / cfg.frequency;
    t_osc1    = t_osc0 + osc_dur;
    t_discard = min(1 / cfg.frequency, 0.5 * osc_dur);
    H_BAR_GATE = 1.5;
    W.desc = (t_e >= cfg.t_hold       & t_e < t_osc0);
    W.osc  = (t_e >= t_osc0+t_discard & t_e < t_osc1);
    W.near = W.osc & (h_bar_d < H_BAR_GATE);
    W.far  = W.osc & (h_bar_d >= H_BAR_GATE);

    % ---------------- Ensemble extraction (arm A; arm B diagnostic) ----------------
    ARMS = {'A', 'B'};
    V = struct(); stacks = struct();
    for ai = 1:2
        arm = ARMS{ai};
        nz = runs.(arm).noisy; ok = find(~[nz.diverged]);
        assert(~isempty(ok), 'verify_r22_amlpf: all arm %s seeds diverged', arm);
        Ns = numel(ok);
        axm_loc   = zeros(N, 3, Ns);
        amdet_loc = zeros(N, 3, Ns);
        atr_loc   = zeros(N, 3, Ns);
        for si = 1:Ns
            s = ok(si);
            axm_aln = nz(s).simOut.diag.a_xm(2:end, :);    % aligned real a_xm (drop init-call row)
            amd_aln = recon_am_det(axm_aln, a_det, nz(s).simOut.a_true_out(1, :));  % init = a_x_init
            axm_loc(:, :, si)   = axm_aln;
            amdet_loc(:, :, si) = amd_aln;
            atr_loc(:, :, si)   = nz(s).simOut.a_true_out(2:end, :);
        end
        V.(arm).var_axm   = var(axm_loc,   0, 3);    % [N x 3] cross-seed Var(a_xm)
        V.(arm).var_amdet = var(amdet_loc, 0, 3);    % [N x 3] cross-seed Var(a_m_det)
        V.(arm).a_amdet   = mean(amdet_loc, 3);
        V.(arm).a_true    = mean(atr_loc,  3);
        V.(arm).Ns        = Ns;
        stacks.(arm).axm   = axm_loc;
        stacks.(arm).amdet = amdet_loc;
    end

    % ---------------- Theory ----------------
    IF_mat = zeros(N, 3);
    for c = 1:3
        IF_mat(:, c) = if_eff_eval(IF_abc, C_dpmr, C_n, kBT, V.A.a_true(:, c), sig2_n(c));
    end
    var_axm_theory   = K_var .* IF_mat .* (V.A.a_true + xi_ax).^2;        % V-i
    var_amdet_theory = amlpf_var_factor .* var_axm_theory;               % V-iii (crude AR(1) IF2)

    % L1 fix: a-dependent rigorous IF2 (am_lpf_r22_design.md §3.3), replacing the
    % crude constant IF2 (AR(1)) inside amlpf_var_factor. Evaluate per-axis on an
    % a-grid + pchip interp to a_true.
    lpf_gain  = a_det / (2 - a_det);
    IF2_crude = amlpf_var_factor / lpf_gain;            % constant (AR(1))
    var_amdet_theory_rig = zeros(N, 3);
    for c = 1:3
        ac  = V.A.a_true(:, c);
        ag  = linspace(min(ac), max(ac), 40).';
        i2g = compute_if2_rigorous(ag, sig2_n(c), cc.lambda_c, cc.a_pd, a_cov, a_det, kBT);
        var_amdet_theory_rig(:, c) = lpf_gain .* interp1(ag, i2g, ac, 'pchip', 'extrap') .* var_axm_theory(:, c);
    end

    % ---------------- rho_axm(tau): cross-seed fluctuation autocorr (V-ii) ----------------
    % Use the descent window (quasi-static a) for the cleanest stationary estimate.
    Tmax = 40;
    rho_meas = nan(Tmax + 1, 3);
    wd = find(W.desc);
    if numel(wd) > Tmax + 5
        for c = 1:3
            % subtract per-time cross-seed mean -> stochastic fluctuation (removes deterministic a(t))
            d = squeeze(stacks.A.axm(wd, c, :));
            d = d - mean(d, 2);
            r0 = mean(mean(d .^ 2, 2));
            for tau = 0:Tmax
                seg = mean(mean(d(1:end-tau, :) .* d(1+tau:end, :), 2));
                rho_meas(tau + 1, c) = seg / r0;
            end
        end
    end
    rho_model = ((1 - a_cov) .^ (0:Tmax)).';   % [Tmax+1 x 1]

    % ---------------- Per-window ratios ----------------
    WIN = {'desc', 'osc', 'near', 'far'};
    stat = struct();
    for wi = 1:numel(WIN)
        wn = WIN{wi}; idx = W.(wn);
        for ai = 1:2
            arm = ARMS{ai};
            if ~any(idx)
                stat.(arm).(wn).ratio_axm       = nan(1, 3);
                stat.(arm).(wn).ratio_amdet     = nan(1, 3);
                stat.(arm).(wn).ratio_amdet_rig = nan(1, 3);
                stat.(arm).(wn).amdet_bias      = nan(1, 3);
                continue;
            end
            stat.(arm).(wn).ratio_axm       = mean(V.(arm).var_axm(idx, :), 1)   ./ mean(var_axm_theory(idx, :), 1);
            stat.(arm).(wn).ratio_amdet     = mean(V.(arm).var_amdet(idx, :), 1) ./ mean(var_amdet_theory(idx, :), 1);
            stat.(arm).(wn).ratio_amdet_rig = mean(V.(arm).var_amdet(idx, :), 1) ./ mean(var_amdet_theory_rig(idx, :), 1);
            stat.(arm).(wn).amdet_bias  = (mean(V.(arm).a_amdet(idx, :), 1) - mean(V.(arm).a_true(idx, :), 1)) ...
                                          ./ mean(V.(arm).a_true(idx, :), 1);
        end
    end

    if opts.verbose
        fprintf('[r22_amlpf] IF2 crude(AR1)=%.2f const. Var(a_m_det) ratio (a=a_true), target ~1.0:\n', IF2_crude);
        fprintf('   window |  crude  x   y   z   |  rigorous x   y   z\n');
        for wi = 1:numel(WIN)
            rc = stat.A.(WIN{wi}).ratio_amdet;
            rr = stat.A.(WIN{wi}).ratio_amdet_rig;
            fprintf('   %-5s  | %.3f %.3f %.3f |  %.3f %.3f %.3f\n', ...
                    WIN{wi}, rc(1), rc(2), rc(3), rr(1), rr(2), rr(3));
        end
    end

    % ---------------- Figures ----------------
    if opts.save_fig
        COL_AXM   = [0.45 0.55 0.95];   % a_xm (light blue)
        COL_AMDET = [0 0.2 0.9];    % a_m_det (blue)
        COL_TH    = [0 0.6 0];      % theory (green)
        FS2 = 18; LFS2 = 14; AXLW2 = 2.0;
        COLS = [1 3]; AXL = 'xz';
        w_mm = max(3, round(min(0.025, 1 / (8 * f)) * fs));
        SCALE = 1e6;   % um^2 -> nm^2
        sc_mask = W.desc | W.osc;

        % FIG 1: time series (z, x) Var(a_xm) and Var(a_m_det) vs theory
        fig = figure('Visible', 'off', 'Position', [80 80 1100 720]);
        tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
        for r = 1:2
            c = COLS(r); nexttile; hold on;
            plot(t_e, movmean(V.A.var_axm(:, c), w_mm)   * SCALE, '-', 'Color', COL_AXM, 'LineWidth', 1.5);
            plot(t_e, movmean(var_axm_theory(:, c), w_mm) * SCALE, '--', 'Color', COL_AXM, 'LineWidth', 1.5);
            plot(t_e, movmean(V.A.var_amdet(:, c), w_mm) * SCALE, '-', 'Color', COL_AMDET, 'LineWidth', 2.5);
            plot(t_e, movmean(var_amdet_theory(:, c), w_mm) * SCALE, '-', 'Color', COL_TH, 'LineWidth', 3.0);
            hold off; xlabel('t [s]', 'FontSize', FS2, 'FontWeight', 'bold');
            ylabel(sprintf('var(a_{m,%s}) [nm^2]', AXL(r)), 'FontSize', FS2, 'FontWeight', 'bold');
            if r == 1
                legend({'Var(a_m) meas', 'Var(a_m) theory', 'Var(a_{m,det}) meas', 'Var(a_{m,det}) theory'}, ...
                       'Location', 'northoutside', 'Orientation', 'horizontal', ...
                       'FontSize', LFS2, 'FontWeight', 'bold', 'Box', 'on');
            end
            set(gca, 'FontSize', FS2, 'FontWeight', 'bold', 'LineWidth', AXLW2, 'Box', 'on'); grid off;
        end
        title(tl, sprintf('R22 cascade verify  f=%gHz  a\\_det=%.4g  (a=a_{true} arm)', f, a_det), ...
              'FontSize', FS2, 'FontWeight', 'bold');
        exportgraphics(fig, fullfile(opts.out_dir, 'fig_r22_var_time.png'), 'Resolution', 150);
        close(fig);

        % FIG 2: scatter Var(a_m_det) vs a (binned + jackknife SEM) + theory
        fig = figure('Visible', 'off', 'Position', [80 80 1100 720]);
        tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
        for r = 1:2
            c = COLS(r); nexttile; hold on;
            B = a_bin_points(V.A.a_true(:, c), V.A.var_amdet(:, c), squeeze(stacks.A.amdet(:, c, :)), sc_mask, opts.n_bin, SCALE);
            [as, is] = sort(V.A.a_true(sc_mask, c));
            th  = var_amdet_theory(sc_mask, c) * SCALE;     th  = th(is);
            thr = var_amdet_theory_rig(sc_mask, c) * SCALE; thr = thr(is);
            plot(as, th,  '-',  'Color', COL_TH,       'LineWidth', 3.0);
            plot(as, thr, '--', 'Color', [0.85 0.3 0], 'LineWidth', 2.5);
            errorbar(B.x, B.y, B.sem, 'o', 'Color', COL_AMDET, 'MarkerFaceColor', COL_AMDET, ...
                     'MarkerSize', 6, 'LineWidth', 1.0, 'CapSize', 3);
            hold off; xlabel(sprintf('a_{%s} [\\mum/pN]', AXL(r)), 'FontSize', FS2, 'FontWeight', 'bold');
            ylabel(sprintf('var(a_{m,det,%s}) [nm^2]', AXL(r)), 'FontSize', FS2, 'FontWeight', 'bold');
            if r == 1
                legend({'theory (AR1)', 'theory (rigorous)', 'a=a_{true} (binned \pm SEM)'}, 'Location', 'northoutside', ...
                       'Orientation', 'horizontal', 'FontSize', LFS2, 'FontWeight', 'bold', 'Box', 'on');
            end
            ax = gca; ax.XAxis.Exponent = 0; xtickformat('%.3f');
            set(gca, 'FontSize', FS2, 'FontWeight', 'bold', 'LineWidth', AXLW2, 'Box', 'on'); grid off;
        end
        title(tl, sprintf('Var(a_{m,det}) vs a  f=%gHz  a\\_det=%.4g', f, a_det), 'FontSize', FS2, 'FontWeight', 'bold');
        exportgraphics(fig, fullfile(opts.out_dir, 'fig_r22_var_scatter.png'), 'Resolution', 150);
        close(fig);

        % FIG 3: rho_axm(tau) measured vs (1-a_cov)^tau
        fig = figure('Visible', 'off', 'Position', [80 80 1100 720]);
        tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
        for r = 1:2
            c = COLS(r); nexttile; hold on;
            plot(0:Tmax, rho_model, '-', 'Color', COL_TH, 'LineWidth', 3.0);
            plot(0:Tmax, rho_meas(:, c), 'o', 'Color', COL_AMDET, 'MarkerFaceColor', COL_AMDET, 'MarkerSize', 6);
            hold off; xlabel('\tau [steps]', 'FontSize', FS2, 'FontWeight', 'bold');
            ylabel(sprintf('\\rho_{a_m,%s}(\\tau)', AXL(r)), 'FontSize', FS2, 'FontWeight', 'bold');
            if r == 1
                legend({'(1-a\_cov)^\tau', 'measured (a=a_{true})'}, 'Location', 'northoutside', ...
                       'Orientation', 'horizontal', 'FontSize', LFS2, 'FontWeight', 'bold', 'Box', 'on');
            end
            set(gca, 'FontSize', FS2, 'FontWeight', 'bold', 'LineWidth', AXLW2, 'Box', 'on'); grid off;
        end
        title(tl, sprintf('a_xm autocorrelation (AR(1) approx check)  f=%gHz', f), 'FontSize', FS2, 'FontWeight', 'bold');
        exportgraphics(fig, fullfile(opts.out_dir, 'fig_rho_axm.png'), 'Resolution', 150);
        close(fig);

        write_summary(opts.out_dir, f, a_cov, a_det, IF2, amlpf_var_factor, stat, rho_meas, rho_model);
    end

    out = struct('amlpf_var_factor', amlpf_var_factor, 'IF2', IF2, 'stat', stat, ...
                 'rho_meas', rho_meas, 'rho_model', rho_model, 'out_dir', opts.out_dir);
end


% ==================== Locals ====================
function amd = recon_am_det(axm, b, init)
%RECON_AM_DET  Offline EWMA(b) of a_xm matching the controller's a_m_det exactly.
%   amd(1) = (1-b)*init + b*axm(1); amd(k) = (1-b)*amd(k-1) + b*axm(k).
%   init = a_m_det just before the first aligned step (= prefill a_x_init, passed
%   as a_true_out(1,:)); falls back to axm(1,:) if omitted.
    if nargin < 3 || isempty(init); init = axm(1, :); end
    amd = zeros(size(axm));
    amd(1, :) = (1 - b) * init + b * axm(1, :);
    for k = 2:size(axm, 1)
        amd(k, :) = (1 - b) * amd(k-1, :) + b * axm(k, :);
    end
end


function write_summary(out_dir, f, a_cov, a_det, IF2, amlpf_var_factor, stat, rho_meas, rho_model)
    fid = fopen(fullfile(out_dir, 'r22_verify_summary.md'), 'w');
    fprintf(fid, '# R22 cascade (a_m_det LPF) verification — f%gHz\n\n', f);
    fprintf(fid, '- a_cov=%.4g, a_det=%.4g, IF2=%.4f, **amlpf_var_factor=%.5f** ', a_cov, a_det, IF2, amlpf_var_factor);
    fprintf(fid, '(predicts Var(a_m_det)/Var(a_xm)=%.4f, std ratio %.3f)\n\n', amlpf_var_factor, sqrt(amlpf_var_factor));
    fprintf(fid, 'Headline (a=a_true arm, target ratio ~= 1.0):\n\n');
    WIN = {'desc', 'osc', 'near', 'far'};
    fprintf(fid, '## V-iii Var(a_m_det) ratio (meas/theory), a=a_true. crude=AR(1), rig=a-dependent\n\n');
    fprintf(fid, '| window | crude x | crude y | crude z | rig x | rig y | rig z |\n');
    fprintf(fid, '|---|---|---|---|---|---|---|\n');
    for wi = 1:numel(WIN)
        rc = stat.A.(WIN{wi}).ratio_amdet;
        rr = stat.A.(WIN{wi}).ratio_amdet_rig;
        fprintf(fid, '| %s | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f |\n', ...
                WIN{wi}, rc(1), rc(2), rc(3), rr(1), rr(2), rr(3));
    end
    fprintf(fid, '\n## V-i Var(a_xm) ratio (re-confirm axm_cdpmr), a=a_true\n\n');
    fprintf(fid, '| window | x | y | z |\n|---|---|---|---|\n');
    for wi = 1:numel(WIN)
        r = stat.A.(WIN{wi}).ratio_axm;
        fprintf(fid, '| %s | %.3f | %.3f | %.3f |\n', WIN{wi}, r(1), r(2), r(3));
    end
    fprintf(fid, '\n## V-ii rho_axm(tau) vs (1-a_cov)^tau (z-axis)\n\n');
    fprintf(fid, '| tau | model | meas_z |\n|---|---|---|\n');
    for t = [1 2 5 10 20]
        fprintf(fid, '| %d | %.3f | %.3f |\n', t, rho_model(t+1), rho_meas(t+1, 3));
    end
    fprintf(fid, '\nNote: a=a_true arm validates the formula (loop at strict lambda_c, ');
    fprintf(fid, 'measurement chain is a passive observer). a=a_hat arm carries the ');
    fprintf(fid, 'self-consistent estimation bias (diagnostic only).\n');
    fclose(fid);
end


function B = a_bin_points(a_axis, vps, stk_full, mask, n_bin, scale)
%A_BIN_POINTS Bin per-sample cross-seed variance by a-value; binned mean + jackknife SEM.
    MIN_BIN_COUNT = 3;
    idx = find(mask(:));
    a = a_axis(idx);
    if isempty(a)
        B.x = zeros(0, 1); B.y = zeros(0, 1); B.sem = zeros(0, 1); return;
    end
    edges = linspace(min(a), max(a), n_bin + 1);
    B.x = nan(n_bin, 1); B.y = nan(n_bin, 1); B.sem = nan(n_bin, 1);
    for b = 1:n_bin
        if b < n_bin
            in_bin = a >= edges(b) & a < edges(b+1);
        else
            in_bin = a >= edges(b) & a <= edges(b+1);
        end
        sel = idx(in_bin);
        if numel(sel) < MIN_BIN_COUNT; continue; end
        B.x(b)   = mean(a_axis(sel));
        B.y(b)   = mean(vps(sel)) * scale;
        B.sem(b) = jackknife_var_sem(stk_full(sel, :)) * scale;
    end
    keep = ~isnan(B.x);
    B.x = B.x(keep); B.y = B.y(keep); B.sem = B.sem(keep);
end


function sem = jackknife_var_sem(stk)
%JACKKNIFE_VAR_SEM Jackknife SE of the mean cross-seed variance.
    [W, Ns] = size(stk);
    if Ns < 2 || W < 1; sem = 0; return; end
    jk = zeros(1, Ns);
    for si = 1:Ns
        keep = true(1, Ns); keep(si) = false;
        vk = var(stk(:, keep), 0, 2);
        jk(si) = mean(vk);
    end
    sem = sqrt((Ns - 1) / Ns * sum((jk - mean(jk)).^2));
end


function IF = if_eff_eval(IF_abc, C_dpmr, C_n, kBT, a, sigma2_nx)
%IF_EFF_EVAL Exact color-inflation factor (mirror controller / verify_axm_cdpmr).
    sxT = 4 * kBT * a;
    num = sxT.^2 * IF_abc(1) + 2 * sxT * sigma2_nx * IF_abc(2) + sigma2_nx^2 * IF_abc(3);
    den = (C_dpmr * sxT + C_n * sigma2_nx).^2;
    IF  = 1 + 2 * num ./ den;
end
