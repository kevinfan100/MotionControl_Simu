function out = verify_eq17_5state_aprime_L1(opts)
%VERIFY_EQ17_5STATE_APRIME_L1  Main (L1) estimability check: can the 5-state
%   controller recover the gain slope a'_x = da_x/dh from a_xm alone, WITHOUT the
%   wall model, and how accurately?
%
%   out = verify_eq17_5state_aprime_L1()
%   out = verify_eq17_5state_aprime_L1(struct('seeds',1:5,'kappa_list',[0.1 1 10]))
%
%   OPEN-LOOP design (isolates "is the information there" from the closed-loop
%   self-consistency / +11% a_xm confound):
%     - run_pure_simulation with use_true_gain=true: the CONTROL LAW uses the
%       true gain (a_ctrl = a_true), so tracking is the known-good behaviour and
%       a_xm is the clean (arm-A, ~unbiased) measurement.
%     - the EKF still estimates a_x (slot 4) and a'_x (slot 5) from a_xm; a'_hat
%       (= diag.delta_a_hat) NEVER feeds the control -> open-loop estimate.
%   Score a'_hat(t) against the true slope a'_true(h) = -a*K_h/R computed from the
%   true position (used ONLY for scoring; the estimator never sees c(h_bar)).
%
%   Metrics (per axis; z = wall-normal is the battleground):
%     - corr  : Pearson corr(a'_hat, a'_true) over the oscillation window
%               (does a'_hat track the SHAPE of a'(h)?).
%     - relerr_near : near-wall (h_bar<1.8) median |a'_hat - a'_true|/|a'_true|.
%     - spread_near : across-seed std(a'_hat)/mean|a'_true| near wall (SNR floor).
%   Swept over Q55 = kappa * baseline to expose the lag(bias)-noise(variance)
%   trade-off (kappa small -> a'_hat lags a'(h); kappa large -> a'_hat noisy).
%
%   NOTE: dynamic-only (no commit). Trajectory mirrors verify_eq17_4state.
%
%   See also: motion_control_law_eq17_5state_aprime, verify_eq17_5state_aprime_L0,
%             verify_eq17_4state, calc_correction_functions

    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'seeds');      opts.seeds = 1:5;          end
    if ~isfield(opts, 'frequency');  opts.frequency = 1.0;      end
    if ~isfield(opts, 'T_sim');      opts.T_sim = 4.0;          end
    if ~isfield(opts, 'kappa_list'); opts.kappa_list = [0.1 1 10]; end
    if ~isfield(opts, 'verbose');    opts.verbose = true;       end
    if ~isfield(opts, 'make_figs');  opts.make_figs = true;     end
    if ~isfield(opts, 'use_true_gain'); opts.use_true_gain = true; end  % true=open-loop (arm A); false=closed-loop (L2)

    here = fileparts(mfilename('fullpath'));
    proj = fullfile(here, '..', '..');
    addpath(genpath(proj));

    f  = opts.frequency;
    vb = opts.verbose;
    out = struct('frequency', f, 'seeds', opts.seeds, 'kappa_list', opts.kappa_list);

    if vb
        fprintf('==== verify_eq17_5state_aprime_L1 (open-loop estimability) ====\n');
        fprintf('  %d seeds, %g Hz near-wall, T=%g s, kappa sweep = [%s]\n', ...
                numel(opts.seeds), f, opts.T_sim, num2str(opts.kappa_list));
    end

    nK = numel(opts.kappa_list);
    res = cell(nK, 1);
    base = [];                                   % kappa=1 ensemble (for the figure)
    for ik = 1:nK
        kappa = opts.kappa_list(ik);
        res{ik} = run_kappa(f, opts.T_sim, opts.seeds, kappa, opts.use_true_gain);
        res{ik}.kappa = kappa;
        if abs(kappa - 1) < 1e-9
            base = res{ik};
        end
    end
    if isempty(base); base = res{1}; end          % fallback for the figure
    out.res = res;

    % --------------------------- report ---------------------------
    if vb
        fprintf('\n[a''_hat vs true a''(h) -- open-loop, mean over seeds]\n');
        fprintf('  %-8s | %-22s | %-26s | %-24s\n', 'kappa', ...
                'corr(a''hat,a''true) xyz', 'near-wall |relerr| %% xyz', 'near spread %% xyz');
        for ik = 1:nK
            r = res{ik};
            fprintf('  %-8.3g | %6.2f %6.2f %6.2f      | %7.1f %7.1f %7.1f          | %6.1f %6.1f %6.1f\n', ...
                    r.kappa, r.corr, 100*r.relerr_near, 100*r.spread_near);
        end
        fprintf('  (z is wall-normal; corr->1 = tracks a''(h) shape; relerr/spread = accuracy/SNR floor)\n');
        % verdict line on the baseline kappa=1, z axis
        fprintf('\n[verdict @ kappa=1, z-axis]\n');
        fprintf('  corr_z = %.2f, near-wall |relerr|_z = %.0f%%, near spread_z = %.0f%%\n', ...
                base.corr(3), 100*base.relerr_near(3), 100*base.spread_near(3));
    end

    % --------------------------- figure ---------------------------
    if opts.make_figs
        out_dir = fullfile(proj, 'test_results', 'verify_5state_aprime_L1', sprintf('f%gHz', f));
        out.fig_path = plot_aprime_estimability(base, out_dir, f);
        if vb; fprintf('\n[figure] %s\n', out.fig_path); end
        if nK >= 2
            out.fig_compare = plot_aprime_kappa_compare(res, out_dir, f);
            if vb; fprintf('[figure] %s\n', out.fig_compare); end
        end
    end
end


function fig_path = plot_aprime_kappa_compare(res, out_dir, freq)
%PLOT_APRIME_KAPPA_COMPARE  z-axis a'_hat for each Q55 scale kappa vs true a'(h):
%   visualises lag (small kappa) -> track (mid) -> noise (large kappa).
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end
    FS = 18; LFS = 12; AXLW = 2.0;
    az = 3;
    t  = res{1}.t;
    w_mm = max(3, round(0.02 / res{1}.Ts));
    sm = @(v) movmean(v, w_mm);
    nK = numel(res);
    cmap = [0 0.45 0.74; 0.85 0.33 0.10; 0.49 0.18 0.56; 0.47 0.67 0.19];

    f1 = figure('Position', [80 80 1150 780], 'Color', 'w', ...
                'NumberTitle', 'off', 'Visible', 'off');

    subplot(3,1,1:2); hold on;
    hT = plot(t, res{1}.aprime_true(:,az), '-', 'Color', [0 0 0], 'LineWidth', 2.8, ...
              'DisplayName', 'a''_{true}(h)');
    hh = gobjects(nK, 1);
    for ik = 1:nK
        col = cmap(min(ik, size(cmap,1)), :);
        hh(ik) = plot(t, sm(res{ik}.aprime_hat(:,az)), '-', 'Color', col, 'LineWidth', 1.6, ...
                      'DisplayName', sprintf('\\kappa=%g  (corr_z=%.2f)', res{ik}.kappa, res{ik}.corr(az)));
    end
    ylabel('a''_z  (1/pN)', 'FontSize', FS, 'FontWeight', 'bold');
    legend([hT; hh], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

    subplot(3,1,3); hold on;
    plot(t, res{1}.h_bar, '-', 'Color', [0.25 0.25 0.25], 'LineWidth', 1.8, 'HandleVisibility', 'off');
    yline(1.8, 'r--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    ylabel('h / R', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

    fig_path = fullfile(out_dir, sprintf('fig_aprime_kappa_compare_z_f%gHz.png', freq));
    exportgraphics(f1, fig_path, 'Resolution', 150);
    close(f1);
end


% ====================== per-kappa runner ======================

function r = run_kappa(f, T_sim, seeds, kappa, use_true_gain)
%RUN_KAPPA  a'_hat vs true a'(h) aggregated over seeds for one Q55 scale.
%   use_true_gain=true  -> open-loop (control uses true gain; a'_hat rides along)
%   use_true_gain=false -> closed-loop (a'_hat feeds the gain feedforward = L2)
    cfg = build_cfg(f, T_sim);
    cfg.Q_aprime_factor = kappa;

    ns = numel(seeds);
    ah_stack = []; at_stack = []; hb_stack = [];
    t_vec = []; Ts = [];
    tr = nan(ns, 3); maxe = 0;
    for si = 1:ns
        ro = struct('seed', seeds(si), 'verbose', false, 'collect_diag', true, ...
                    'use_true_gain', use_true_gain);
        s = run_pure_simulation(cfg, ro);
        P = s.meta.params_value;
        Ts = P.common.Ts;
        aprime_hat  = s.diag.delta_a_hat;              % [N x 3]  slot-5 estimate
        aprime_true = aprime_true_from_pos(s.p_true_out, P);   % [N x 3]
        w = P.wall.w_hat(:);
        h_bar = (s.p_true_out * w - P.wall.pz) / P.common.R;   % [N x 1]
        e = s.p_d_out(2:end, :) - s.p_true_out(1:end-1, :);    % tracking error [um]
        tr(si, :) = 1e3 * std(e, 0, 1);                        % [nm]
        maxe = max(maxe, max(abs(e(:))));

        ah_stack = cat(3, ah_stack, aprime_hat);
        at_stack = cat(3, at_stack, aprime_true);
        hb_stack = cat(2, hb_stack, h_bar);
        if si == 1
            N = size(aprime_hat, 1);
            t_vec = (0:N-1)' * Ts;
        end
    end
    r_track_std = mean(tr, 1);
    r_diverged  = maxe > 0.5;

    ah_ens = mean(ah_stack, 3);          % [N x 3] ensemble-mean a'_hat
    at_ens = mean(at_stack, 3);          % [N x 3] ensemble-mean a'_true
    hb_ens = mean(hb_stack, 2);          % [N x 1]

    % oscillation window (skip hold + descend transient): t > t_osc_start
    t_osc_start = 1.6;
    osc = t_vec > t_osc_start;
    near = osc & (hb_ens < 1.8);

    corr   = zeros(1, 3);
    relerr = zeros(1, 3);
    spread = zeros(1, 3);
    for ax = 1:3
        a = ah_ens(osc, ax); b = at_ens(osc, ax);
        if std(a) > 0 && std(b) > 0
            c = corrcoef(a, b); corr(ax) = c(1, 2);
        else
            corr(ax) = NaN;
        end
        if any(near)
            num = abs(ah_ens(near, ax) - at_ens(near, ax));
            den = abs(at_ens(near, ax));
            relerr(ax) = median(num ./ max(den, eps));
            % across-seed spread of a'_hat near wall, normalised by |a'_true|
            sd = std(ah_stack(near, ax, :), 0, 3);          % per-time across-seed std
            spread(ax) = mean(sd) / max(mean(abs(at_ens(near, ax))), eps);
        else
            relerr(ax) = NaN; spread(ax) = NaN;
        end
    end

    r.corr        = corr;
    r.relerr_near = relerr;
    r.spread_near = spread;
    r.track_std_nm = r_track_std;
    r.diverged     = r_diverged;
    r.t           = t_vec;
    r.aprime_hat  = ah_ens;
    r.aprime_true = at_ens;
    r.h_bar       = hb_ens;
    r.Ts          = Ts;
end


function ap = aprime_true_from_pos(p_pos, P)
%APRIME_TRUE_FROM_POS  True slope a'(h) = -a*K_h/R per axis from positions.
%   [N x 3]; x,y use c_para/K_h_para; z uses c_perp/K_h_perp. h_bar from w_hat.
    R = P.common.R; Ts = P.common.Ts; gam = P.ctrl.gamma; a_nom = Ts / gam;
    w = P.wall.w_hat(:); pz = P.wall.pz;
    N = size(p_pos, 1);
    h_bar = (p_pos * w - pz) / R;
    ap = zeros(N, 3);
    for k = 1:N
        hb = max(h_bar(k), 1.001);
        [c_para, c_perp, dv] = calc_correction_functions(hb, true);
        a_para = a_nom / c_para;  a_perp = a_nom / c_perp;
        ap(k, 1) = -a_para * dv.K_h_para / R;
        ap(k, 2) = -a_para * dv.K_h_para / R;
        ap(k, 3) = -a_perp * dv.K_h_perp / R;
    end
end


% ====================== figure ======================

function fig_path = plot_aprime_estimability(base, out_dir, freq)
%PLOT_APRIME_ESTIMABILITY  z-axis open-loop estimability:
%   (1) a'_hat (ensemble) vs true a'(h)
%   (2) error a'_hat - a'_true
%   (3) h_bar(t) phase context (trough = near wall = large a').
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end
    FS = 18; LFS = 13; AXLW = 2.0;
    COL_T = [0 0 0]; COL_H = [0.85 0.33 0.10]; COL_E = [0 0.45 0.74];
    az = 3;                                   % z axis (wall-normal)
    t  = base.t;
    w_mm = max(3, round(0.02 / base.Ts));     % ~20 ms zero-phase smooth
    sm = @(v) movmean(v, w_mm);

    f1 = figure('Position', [80 80 1100 900], 'Color', 'w', ...
                'NumberTitle', 'off', 'Visible', 'off');

    subplot(3,1,1); hold on;
    hT = plot(t, base.aprime_true(:,az),    '-', 'Color', COL_T, 'LineWidth', 2.5, 'DisplayName', 'a''_{true}(h)');
    hH = plot(t, sm(base.aprime_hat(:,az)), '-', 'Color', COL_H, 'LineWidth', 1.8, 'DisplayName', 'a''_{hat} (open-loop)');
    ylabel('a''_z  (1/pN)', 'FontSize', FS, 'FontWeight', 'bold');
    legend([hT hH], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

    subplot(3,1,2); hold on;
    plot(t, zeros(size(t)), 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    plot(t, sm(base.aprime_hat(:,az) - base.aprime_true(:,az)), '-', 'Color', COL_E, 'LineWidth', 1.8);
    ylabel('a''_z err (1/pN)', 'FontSize', FS, 'FontWeight', 'bold');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

    subplot(3,1,3); hold on;
    plot(t, base.h_bar, '-', 'Color', [0.25 0.25 0.25], 'LineWidth', 1.8, 'HandleVisibility', 'off');
    yline(1.8, 'r--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    ylabel('h / R', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

    fig_path = fullfile(out_dir, sprintf('fig_aprime_estimability_z_f%gHz.png', freq));
    exportgraphics(f1, fig_path, 'Resolution', 150);
    close(f1);
end


% ====================== shared scenario ======================

function cfg = build_cfg(f, T_sim)
%BUILD_CFG  1 Hz near-wall osc (mirrors verify_eq17_4state build_cfg).
    cfg = user_config();
    cfg.eq17_variant   = '5state_aprime';
    cfg.trajectory_type = 'osc';
    cfg.h_init   = 50;
    cfg.h_bottom = 2.7;                       % h_bar trough ~1.2 (near wall)
    cfg.amplitude = 2.5;
    cfg.frequency = f;
    cfg.n_cycles  = 2 * f;
    cfg.t_hold    = 0.5;
    cfg.t_descend_override = 1.0;
    cfg.T_sim     = T_sim;
    pc = physical_constants();
    cfg.h_min     = 1.05 * pc.R;
    cfg.ctrl_enable = true;
    cfg.thermal_enable = true;
    cfg.meas_noise_enable = true;
    cfg.lambda_c = 0.7;
    cfg.a_pd  = 0.05;
    cfg.a_cov = 0.05;
    cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];
    cfg.suppress_xD = true;
    cfg.h_bar_safe = 1;                        % G3 unreachable (trough h_bar=1.2)
    cfg.use_am_lpf = false;
    cfg.a_det      = 0.005;
end
