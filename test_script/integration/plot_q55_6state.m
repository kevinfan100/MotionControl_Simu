function plot_q55_6state(freq, opts)
%PLOT_Q55_6STATE  Visualize Q55 closed-form verification from gain_compare data (read-only).
%
%   plot_q55_6state(freq)
%   plot_q55_6state(freq, opts)
%
%   Reads test_results/gain_compare/f<freq>Hz/runs.mat WITHOUT touching the
%   production analyze_gain_6state pipeline, and writes two figures into the
%   same folder (test_results/ is gitignored):
%
%     fig_q55_scatter.png  var(delta a_ram) vs gain a, log-log, x (top) /
%                          z (bottom). Per-bin cross-seed measurement (osc
%                          window, h_bar-binned) + jackknife-over-seeds SEM,
%                          overlaid on the closed-form theory curve.
%     fig_q55_time.png     pointwise var(delta a_ram) vs time, x / z. Raw
%                          pointwise (chi-squared fuzz) background + movmean
%                          main line + theory curve.
%
%   Q55 closed form (the EKF gain-state process noise, = the one-step
%   increment of the random gain fluctuation):
%       var(delta a_ram,i) = [2/(1+lambda_c)] * (a_i * K_h,i / R)^2 * sigma2_dh
%       sigma2_dh = 4 kB T a_perp           (wall-normal kick, shared 3 axes)
%   Theory is evaluated along the DESIRED trajectory (a_pd, K_h at h_bar_d):
%   no simulation privilege, so the curve is a genuine prediction. The
%   measured side stacks per-seed a_true_out (realized gain at the true
%   position) and takes the across-seed variance of its one-step increment.
%
%   Uses the a=a_true arm (opts.arm='A', DEFAULT): the closed loop sits
%   strictly at lambda_c there, so the closed form is exact. The a=a_hat arm
%   has a self-consistent gain-mismatch bias and is for diagnosis only.
%
%   opts (all optional):
%     arm        'A' (a=a_true verification arm; DEFAULT) | 'B' (a=a_hat)
%     data_root  source dir for f<freq>Hz/runs.mat (default test_results/gain_compare)
%     runs, cfg  preloaded structs -> skip the disk load (fast iteration)
%     n_bin      h_bar bins for the scatter (default 14)
%     save_fig   true (default)
%
%   See also: analyze_gain_6state, verify_axm_cdpmr_6state, plot_var_ahat_6state

    if nargin < 1 || isempty(freq); freq = 1; end
    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'arm');      opts.arm      = 'A';   end
    if ~isfield(opts, 'save_fig'); opts.save_fig = true;  end
    if ~isfield(opts, 'n_bin');    opts.n_bin    = 14;    end

    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(fileparts(script_dir));
    addpath(fullfile(project_root, 'model', 'wall_effect'));
    if ~isfield(opts, 'data_root')
        opts.data_root = fullfile(project_root, 'test_results', 'gain_compare');
    end
    out_dir = fullfile(opts.data_root, sprintf('f%gHz', freq));

    % --- load (or accept preloaded) ---
    if isfield(opts, 'runs') && isfield(opts, 'cfg')
        runs = opts.runs; cfg = opts.cfg;
    else
        S = load(fullfile(out_dir, 'runs.mat'));   % runs, cfg, opts, ...
        runs = S.runs; cfg = S.cfg; clear S;
    end

    % --- physical constants + aligned time grid (e[k] = pd[k+1]-p[k] convention) ---
    P     = runs.A.det.simOut.meta.params_value;
    R     = P.common.R; w_hat = P.wall.w_hat; pz = P.wall.pz;
    Ts    = P.common.Ts; gamma_N = P.common.gamma_N;
    kBT   = P.ctrl.k_B * P.ctrl.T;  a_nom = Ts / gamma_N;
    lc    = cfg.lambda_c;
    t_e   = runs.A.det.simOut.tout(2:end);
    pd_al = runs.A.det.simOut.p_d_out(2:end, :);
    h_bar_d = (pd_al * w_hat - pz) / R;            % [N-1 x 1] desired-trajectory h_bar

    % --- deterministic gain skeleton + wall sensitivity along desired traj ---
    % a_pd,i = a_nom / C_i(h_bar_d); K_h,i = (1/C_i) dC_i/dh_bar. x=y=para, z=perp.
    hbd = max(h_bar_d, 1.001);
    Nd  = numel(hbd);
    c_pa = zeros(Nd, 1); c_pe = zeros(Nd, 1); Kpa = zeros(Nd, 1); Kpe = zeros(Nd, 1);
    for ki = 1:Nd
        [c_pa(ki), c_pe(ki), drv] = calc_correction_functions(hbd(ki), true);
        Kpa(ki) = drv.K_h_para; Kpe(ki) = drv.K_h_perp;
    end
    a_pd  = [a_nom ./ c_pa, a_nom ./ c_pa, a_nom ./ c_pe];   % [N-1 x 3]
    Kh_pd = [Kpa, Kpa, Kpe];                                  % [N-1 x 3]

    % --- osc analysis window (drop 1st cycle transient; single-cycle cap) ---
    t_osc0    = cfg.t_hold + cfg.t_descend_override;
    osc_dur   = cfg.n_cycles / cfg.frequency;
    t_osc1    = t_osc0 + osc_dur;
    t_discard = min(1 / cfg.frequency, 0.5 * osc_dur);
    osc = (t_e >= t_osc0 + t_discard) & (t_e < t_osc1);       % [N-1 x 1] logical

    % --- stack a_true (realized gain) across non-diverged seeds of the arm ---
    nz = runs.(opts.arm).noisy;
    ok = find(~[nz.diverged]);
    assert(~isempty(ok), 'plot_q55_6state: all arm-%s noisy seeds diverged', opts.arm);
    Ns = numel(ok);
    stack = zeros(numel(t_e), 3, Ns);
    for si = 1:Ns
        stack(:, :, si) = nz(ok(si)).simOut.a_true_out(2:end, :);
    end
    corr  = 1 - 1/Ns;                            % ensemble-mean self-subtraction deflation
    a_ram = stack - mean(stack, 3);              % [N-1 x 3 x Ns] random gain fluctuation
    d_ram = diff(a_ram, 1, 1);                   % [N-2 x 3 x Ns] one-step increment

    % --- theory (closed form, along desired trajectory) ---
    s2dh = 4 * kBT * a_pd(:, 3);                 % [N-1 x 1] wall-normal kick (shared 3 axes)
    base = (a_pd .* Kh_pd / R).^2 .* s2dh;       % [N-1 x 3]
    inc_factor = 2 / (1 + lc);                   % closed-form increment factor (Q55)
    th_incr_pw = inc_factor * base(1:end-1, :);  % [N-2 x 3] aligned to earlier endpoint

    % --- measured pointwise increment variance (cross-seed, deflated) ---
    meas_incr_pw = var(d_ram, 0, 3) / corr;      % [N-2 x 3]

    % --- osc-window h_bar-binned measured increment (for scatter) ---
    vmask = osc(1:end-1) & osc(2:end);           % in-osc adjacent increment pairs
    steps = find(vmask);
    hb_s  = h_bar_d(steps);
    nb    = opts.n_bin; edges = linspace(min(hb_s), max(hb_s), nb + 1);
    [bin_apar, bin_aperp]       = deal(nan(nb, 1));
    [bin_meas, bin_sem, bin_th] = deal(nan(nb, 2));    % col 1 = x, col 2 = z
    for b = 1:nb
        hi  = edges(b + 1) + (b == nb) * 1e-9;
        sel = steps(hb_s >= edges(b) & hb_s < hi);
        if numel(sel) < 3; continue; end
        for axn = [1 3]
            vs = squeeze(var(d_ram(sel, axn, :), 0, 1)) / corr;   % [Ns x 1] per-seed
            jk = zeros(Ns, 1);
            for j = 1:Ns; jk(j) = mean(vs([1:j-1, j+1:Ns])); end  % jackknife-over-seeds
            c = (axn == 3) + 1;
            bin_meas(b, c) = mean(vs);
            bin_sem(b, c)  = sqrt((Ns - 1) / Ns * sum((jk - mean(jk)).^2));
            bin_th(b, c)   = inc_factor * mean(base(sel, axn));
        end
        bin_apar(b)  = mean(a_pd(sel, 1));
        bin_aperp(b) = mean(a_pd(sel, 3));
    end

    % --- smooth theory curve over h_bar grid (scatter overlay) ---
    hg = linspace(min(hb_s) * 0.99, max(hb_s) * 1.01, 200).';
    [apar_g, aperp_g, thx_g, thz_g] = deal(zeros(size(hg)));
    for i = 1:numel(hg)
        [cpa, cpe, dv] = calc_correction_functions(hg(i), true);
        ap = a_nom / cpa; az = a_nom / cpe;
        apar_g(i) = ap; aperp_g(i) = az;
        thx_g(i) = inc_factor * (ap * dv.K_h_para / R)^2 * 4 * kBT * az;
        thz_g(i) = inc_factor * (az * dv.K_h_perp / R)^2 * 4 * kBT * az;
    end

    % --- console summary: osc-pooled measured/theory ratio per axis ---
    axc = 'xyz';
    for axn = [1 3]
        c = (axn == 3) + 1; v = ~isnan(bin_meas(:, c));
        ratio = sum(bin_meas(v, c)) / sum(bin_th(v, c));
        fprintf('[q55:%gHz arm=%s axis=%c] osc-pooled meas/theory = %.3f (Ns=%d)\n', ...
                freq, opts.arm, axc(axn), ratio, Ns);
    end

    if ~opts.save_fig; return; end

    % --- shared EXP style (mirrors analyze_gain_6state make_figs) ---
    COL_TH = [0 0.6 0]; COL_M = [0.8 0 0];
    FS = 18; LFS = 14; AXLW = 2.0; axlab = 'xz';

    % ===== FIG 1: scatter var(delta a_ram) vs a, log-log, x / z =====
    f1 = figure('Position', [80 80 1100 760], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for r = 1:2
        nexttile; hold on;
        if r == 1; ag = apar_g; tg = thx_g; av = bin_apar; else; ag = aperp_g; tg = thz_g; av = bin_aperp; end
        mv = bin_meas(:, r); sv = bin_sem(:, r); v = ~isnan(av) & mv > 0;
        hT = plot(ag * 1e3, tg * 1e6, '-', 'Color', COL_TH, 'LineWidth', 3.0, ...
                  'DisplayName', 'Theory  [2/(1+\lambda_c)](aK_h/R)^2\sigma^2_{\deltah}');
        hM = errorbar(av(v) * 1e3, mv(v) * 1e6, sv(v) * 1e6, 's', 'Color', COL_M, ...
                  'MarkerFaceColor', COL_M, 'MarkerSize', 8, 'LineWidth', 1.5, 'CapSize', 5, ...
                  'DisplayName', sprintf('%g Hz measured (%d seeds)', freq, Ns));
        set(gca, 'XScale', 'log', 'YScale', 'log');
        ylabel(sprintf('var(\\deltaa_{ram,%c})  (nm/pN)^2', axlab(r)), 'FontSize', FS, 'FontWeight', 'bold');
        if r == 1
            legend([hT hM], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
                   'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
            xlabel('a_x  (nm/pN)', 'FontSize', FS, 'FontWeight', 'bold');
        else
            xlabel('a_z  (nm/pN)', 'FontSize', FS, 'FontWeight', 'bold');
        end
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    end
    exportgraphics(f1, fullfile(out_dir, 'fig_q55_scatter.png'), 'Resolution', 150);
    close(f1);

    % ===== FIG 2: pointwise increment variance vs time, x / z =====
    fsr   = 1 / (t_e(2) - t_e(1));
    w_mm  = max(3, round(min(0.025, 1/(8*freq)) * fsr));    % movmean window [samples]
    t_inc = t_e(1:end-1); T_END = ceil(t_e(end)); cols = [1 3];
    f2 = figure('Position', [80 80 1100 760], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for r = 1:2
        c = cols(r); nexttile; hold on;
        mv = meas_incr_pw(:, c) * 1e6; tv = th_incr_pw(:, c) * 1e6;
        plot(t_inc, mv, '-', 'Color', [0.8 0 0 0.22], 'LineWidth', 0.5, 'HandleVisibility', 'off');
        hM = plot(t_inc, movmean(mv, w_mm), '-', 'Color', COL_M, 'LineWidth', 1.4, ...
                  'DisplayName', 'a_{true} increment (meas)');
        hT = plot(t_inc, tv, '-', 'Color', COL_TH, 'LineWidth', 3.0, 'DisplayName', 'Theory');
        xlim([0 T_END]); ym = max(mv) * 1.15; if ~isfinite(ym) || ym <= 0; ym = 1; end; ylim([0 ym]);
        ylabel(sprintf('var(\\deltaa_{ram,%c})  (nm/pN)^2', axlab(r)), 'FontSize', FS, 'FontWeight', 'bold');
        if r == 1
            legend([hT hM], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
                   'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        end
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    end
    xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    exportgraphics(f2, fullfile(out_dir, 'fig_q55_time.png'), 'Resolution', 150);
    close(f2);

    fprintf('[q55:%gHz arm=%s] Ns=%d -> %s\n', freq, opts.arm, Ns, out_dir);
end
