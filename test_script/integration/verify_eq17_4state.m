function out = verify_eq17_4state(opts)
%VERIFY_EQ17_4STATE  End-to-end check of the 4-state (gain-rate feedforward)
%   controller against the 5-state, on the 1 Hz near-wall trajectory that
%   exposes the fast-motion a_hat lag the 4-state is meant to remove.
%
%   out = verify_eq17_4state()                 % defaults: 5 seeds, 1 Hz, T=4 s
%   out = verify_eq17_4state(struct('seeds',1:10,'frequency',1.0))
%
%   Three parts:
%     (1) SMOKE   : run the 4-state once; assert it executes + does not diverge.
%     (2) 4-vs-5  : same seeds, same scenario; compare tracking std, a_hat error
%                   vs a_true, and a_hat lag (cross-correlation, z axis). The
%                   design hypothesis: the 4-state's feedforward reduces the
%                   near-wall z a_hat LAG vs the 5-state.
%     (3) RESIDUAL: caveat-1 quantification. The feedforward uses the DESIRED
%                   height h_bar_d; the residual a_nom/c(h_bar_true) -
%                   a_nom/c(h_bar_d) (true vs desired operating point) is the
%                   error the feedforward cannot remove. Reported far-vs-near.
%
%   NOTE: dynamic-only (no commit). Mirrors compare_gain_6state's scenario.
%
%   See also: motion_control_law_eq17_4state, motion_control_law_eq17_5state,
%             run_pure_simulation, compare_gain_6state

    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'seeds');     opts.seeds = 1:5;      end
    if ~isfield(opts, 'frequency'); opts.frequency = 1.0;  end
    if ~isfield(opts, 'T_sim');     opts.T_sim = 4.0;      end
    if ~isfield(opts, 'verbose');   opts.verbose = true;   end
    if ~isfield(opts, 'make_figs'); opts.make_figs = true; end

    here = fileparts(mfilename('fullpath'));
    proj = fullfile(here, '..', '..');
    addpath(genpath(proj));

    f = opts.frequency;
    cfg4 = build_cfg(f, opts.T_sim, '4state');
    cfg5 = build_cfg(f, opts.T_sim, '5state');

    out = struct('frequency', f, 'seeds', opts.seeds);

    % ---------------------------------------------------------------
    % (1) SMOKE: single 4-state noisy run must execute + not diverge
    % ---------------------------------------------------------------
    ro = struct('seed', opts.seeds(1), 'verbose', false, 'collect_diag', true, ...
                'use_true_gain', false);
    s4 = run_pure_simulation(cfg4, ro);
    e_smoke = s4.p_d_out(2:end,:) - s4.p_true_out(1:end-1,:);
    out.smoke.ran = true;
    out.smoke.max_abs_err_um = max(abs(e_smoke(:)));
    out.smoke.diverged = out.smoke.max_abs_err_um > 0.5;
    assert(~out.smoke.diverged, ...
        'verify_eq17_4state:smokeDiverged', ...
        '4-state SMOKE diverged: |e|_max = %.3g um', out.smoke.max_abs_err_um);
    if opts.verbose
        fprintf('[smoke] 4-state ran; |e|_max = %.1f nm (no divergence)\n', ...
                1e3 * out.smoke.max_abs_err_um);
    end

    % ---------------------------------------------------------------
    % (2) 4-vs-5 comparison over seeds
    % ---------------------------------------------------------------
    A4 = run_variant(cfg4, opts.seeds);
    A5 = run_variant(cfg5, opts.seeds);
    out.cmp.track_std_nm = struct('s4', A4.track_std_nm, 's5', A5.track_std_nm);
    out.cmp.ahat_relerr  = struct('s4', A4.ahat_relerr,  's5', A5.ahat_relerr);
    out.cmp.ahat_lag_ms  = struct('s4', A4.ahat_lag_ms,  's5', A5.ahat_lag_ms);

    if opts.verbose
        ax = {'x','y','z'};
        fprintf('\n[4-vs-5 @ %gHz, %d seeds]   (mean over seeds)\n', f, numel(opts.seeds));
        fprintf('  %-22s %8s %8s %8s\n', 'metric / axis', ax{:});
        fprintf('  %-22s %8.1f %8.1f %8.1f\n', 'track std 4-state [nm]', A4.track_std_nm);
        fprintf('  %-22s %8.1f %8.1f %8.1f\n', 'track std 5-state [nm]', A5.track_std_nm);
        fprintf('  %-22s %8.1f %8.1f %8.1f\n', 'a_hat |err| 4-st [%]', 100*A4.ahat_relerr);
        fprintf('  %-22s %8.1f %8.1f %8.1f\n', 'a_hat |err| 5-st [%]', 100*A5.ahat_relerr);
        fprintf('  %-22s %8.1f %8.1f %8.1f\n', 'a_hat lag 4-st [ms]', A4.ahat_lag_ms);
        fprintf('  %-22s %8.1f %8.1f %8.1f\n', 'a_hat lag 5-st [ms]', A5.ahat_lag_ms);
        fprintf('  (design hypothesis: 4-state z lag < 5-state z lag)\n');
    end

    % time-domain comparison figure (z axis: a_hat tracking + error + h_bar)
    if opts.make_figs
        out_dir = fullfile(proj, 'test_results', 'verify_4state', sprintf('f%gHz', f));
        out.fig_path = plot_time_compare(A4, A5, out_dir, f);
        if opts.verbose
            fprintf('\n[figure] %s\n', out.fig_path);
        end
    end

    % ---------------------------------------------------------------
    % (3) RESIDUAL (caveat 1): feedforward uses h_bar_d (desired), true gain
    %     uses h_bar_true. residual = a_nom/c(h_bar_true) - a_nom/c(h_bar_d).
    %     Reported as % of a_true, split far (h_bar>3) vs near (h_bar<2).
    % ---------------------------------------------------------------
    out.residual = feedforward_residual(s4, proj);
    if opts.verbose
        r = out.residual;
        fprintf('\n[caveat-1 feedforward residual: a_det(h_d) vs a_det(h_true)]\n');
        fprintf('  far  (h_bar>3): median |resid|/a_true = %.2f%% (z), %.2f%% (xy)\n', ...
                100*r.far_z, 100*r.far_xy);
        fprintf('  near (h_bar<2): median |resid|/a_true = %.2f%% (z), %.2f%% (xy)\n', ...
                100*r.near_z, 100*r.near_xy);
        fprintf('  -> residual grows near wall (expected); compare to a_hat |err| above.\n');
    end
end


% ====================== helpers ======================

function cfg = build_cfg(f, T_sim, variant)
%BUILD_CFG  osc near-wall scenario (mirrors compare_gain_6state build_config).
    cfg = user_config();
    cfg.eq17_variant   = variant;            % '4state' | '5state'
    cfg.trajectory_type = 'osc';
    cfg.h_init   = 50;                        % [um]
    cfg.h_bottom = 2.7;                       % [um] -> h_bar = 1.2 (near wall)
    cfg.amplitude = 2.5;                      % [um] -> h_bar in [1.2, 3.42]
    cfg.frequency = f;
    cfg.n_cycles  = 2 * f;                    % osc duration = 2 s
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
    cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];   % [um]
    cfg.suppress_xD = true;
    cfg.h_bar_safe = 1;                       % G3 unreachable (trough h_bar=1.2)
    cfg.use_am_lpf = false;
    cfg.a_det      = 0.005;
end


function A = run_variant(cfg, seeds)
%RUN_VARIANT  Aggregate tracking std + a_hat error/lag over seeds, and keep the
%   seed-ensemble-mean a_hat / a_true time series (for the time-domain figure).
    ns = numel(seeds);
    track = nan(ns, 3); relerr = nan(ns, 3); lag = nan(ns, 3);
    ah_stack = []; at_stack = [];
    t_vec = []; hbar_d = []; Ts = [];
    for si = 1:ns
        ro = struct('seed', seeds(si), 'verbose', false, 'collect_diag', true, ...
                    'use_true_gain', false);
        s = run_pure_simulation(cfg, ro);
        % tracking error (aligned): p_d[k+1] - p_true[k]
        e = s.p_d_out(2:end,:) - s.p_true_out(1:end-1,:);   % [N-1 x 3] [um]
        track(si,:) = 1e3 * std(e, 0, 1);                   % [nm]
        % a_hat vs a_true (aligned to a_true_out(2:end))
        a_true = s.a_true_out(2:end,:);                     % [N-1 x 3]
        a_hat  = s.diag.a_hat(2:end,:);                     % [N-1 x 3]
        relerr(si,:) = median(abs(a_hat - a_true) ./ a_true, 1);
        Ts = s.meta.params_value.common.Ts;
        % a_hat lag via cross-correlation (per axis), in ms
        for ax = 1:3
            lag(si,ax) = xcorr_lag_ms(a_true(:,ax), a_hat(:,ax), Ts);
        end
        ah_stack = cat(3, ah_stack, a_hat);
        at_stack = cat(3, at_stack, a_true);
        if si == 1
            N = size(a_true, 1);
            t_vec = (1:N)' * Ts;                            % aligned time [sec]
            P = s.meta.params_value;
            w = P.wall.w_hat(:);
            hbar_d = (s.p_d_out(2:end,:) * w - P.wall.pz) / P.common.R;   % desired h_bar [N-1]
        end
    end
    A.track_std_nm = mean(track, 1);
    A.ahat_relerr  = mean(relerr, 1);
    A.ahat_lag_ms  = mean(lag, 1);
    A.a_hat_ens    = mean(ah_stack, 3);    % seed-ensemble-mean a_hat [N-1 x 3]
    A.a_true_ens   = mean(at_stack, 3);    % seed-ensemble-mean a_true [N-1 x 3]
    A.t            = t_vec;
    A.hbar_d       = hbar_d;
    A.Ts           = Ts;
end


function lag_ms = xcorr_lag_ms(a_true, a_hat, Ts)
%XCORR_LAG_MS  Positive lag => a_hat lags a_true. De-meaned, +/-200-sample window.
    x = a_true - mean(a_true);
    y = a_hat  - mean(a_hat);
    if std(x) < eps || std(y) < eps
        lag_ms = 0; return;
    end
    maxlag = min(200, numel(x)-1);
    [c, lags] = xcorr(y, x, maxlag, 'coeff');   % y delayed wrt x => peak at +lag
    [~, im] = max(c);
    lag_ms = lags(im) * Ts * 1e3;
end


function r = feedforward_residual(s, proj)
%FEEDFORWARD_RESIDUAL  caveat-1: a_det(h_bar_true) - a_det(h_bar_d), %a_true.
    addpath(genpath(proj));
    P  = s.meta.params_value;
    R  = P.common.R;  Ts = P.common.Ts;  gam = P.ctrl.gamma;
    a_nom = Ts / gam;
    w = P.wall.w_hat(:);  pz = P.wall.pz;
    N = size(s.p_d_out, 1);
    h_bar_d    = (s.p_d_out    * w - pz) / R;        % desired height [N]
    h_bar_true = (s.p_true_out * w - pz) / R;        % true height [N]
    a_det_d    = zeros(N,3); a_det_true = zeros(N,3); a_true = zeros(N,3);
    for k = 1:N
        hd = max(h_bar_d(k), 1.001);  ht = max(h_bar_true(k), 1.001);
        [cpd, cppd] = calc_correction_functions(hd);
        [cpt, cppt] = calc_correction_functions(ht);
        a_det_d(k,:)    = [a_nom/cpd,  a_nom/cpd,  a_nom/cppd];
        a_det_true(k,:) = [a_nom/cpt,  a_nom/cpt,  a_nom/cppt];
        a_true(k,:)     = a_det_true(k,:);
    end
    resid = abs(a_det_true - a_det_d) ./ a_true;      % relative residual
    far  = h_bar_true > 3;  near = h_bar_true < 2;
    r.far_z   = median(resid(far,  3));
    r.far_xy  = median(mean(resid(far,  1:2), 2));
    r.near_z  = median(resid(near, 3));
    r.near_xy = median(mean(resid(near, 1:2), 2));
end


function fig_path = plot_time_compare(A4, A5, out_dir, freq)
%PLOT_TIME_COMPARE  z-axis time-domain comparison (4-state vs 5-state):
%   (1) a_hat tracking: a_true / a_hat 5-state / a_hat 4-state
%   (2) a_hat error (a_hat - a_true): 5-state vs 4-state
%   (3) desired h_bar(t) for phase context (trough = near wall; mid = max |dh/dt|)
%   Seed-ensemble means; a light CENTERED (zero-phase) movmean smooths the
%   chi-squared noise without distorting the lag. Style mirrors plot_q55_6state.
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end
    FS = 18; LFS = 13; AXLW = 2.0;
    COL_T = [0 0 0]; COL5 = [0.85 0.33 0.10]; COL4 = [0 0.45 0.74];
    az = 3;                                  % z axis (wall-normal)
    t  = A4.t;
    w_mm = max(3, round(0.02 / A4.Ts));      % ~20 ms centered window (zero phase)
    sm = @(v) movmean(v, w_mm);

    f1 = figure('Position', [80 80 1100 900], 'Color', 'w', ...
                'NumberTitle', 'off', 'Visible', 'off');

    % --- Row 1: a_hat tracking (z) ---
    subplot(3,1,1); hold on;
    hT = plot(t, A4.a_true_ens(:,az)*1e3,    '-', 'Color', COL_T, 'LineWidth', 2.5, 'DisplayName', 'a_{true}');
    h5 = plot(t, sm(A5.a_hat_ens(:,az))*1e3, '-', 'Color', COL5,  'LineWidth', 1.8, 'DisplayName', 'a_{hat} 5-state');
    h4 = plot(t, sm(A4.a_hat_ens(:,az))*1e3, '-', 'Color', COL4,  'LineWidth', 1.8, 'DisplayName', 'a_{hat} 4-state');
    ylabel('a_z  (nm/pN)', 'FontSize', FS, 'FontWeight', 'bold');
    legend([hT h5 h4], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

    % --- Row 2: a_hat error (z) ---
    subplot(3,1,2); hold on;
    plot(t, zeros(size(t)), 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    e5 = plot(t, sm(A5.a_hat_ens(:,az)-A5.a_true_ens(:,az))*1e3, '-', 'Color', COL5, 'LineWidth', 1.8, 'DisplayName', '5-state');
    e4 = plot(t, sm(A4.a_hat_ens(:,az)-A4.a_true_ens(:,az))*1e3, '-', 'Color', COL4, 'LineWidth', 1.8, 'DisplayName', '4-state');
    ylabel('a_z err (nm/pN)', 'FontSize', FS, 'FontWeight', 'bold');
    legend([e5 e4], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

    % --- Row 3: desired h_bar (phase context) ---
    subplot(3,1,3); hold on;
    plot(t, A4.hbar_d, '-', 'Color', [0.25 0.25 0.25], 'LineWidth', 1.8, 'HandleVisibility', 'off');
    ylabel('h_d / R', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

    fig_path = fullfile(out_dir, sprintf('fig_4v5_ahat_z_f%gHz.png', freq));
    exportgraphics(f1, fig_path, 'Resolution', 150);
    close(f1);
end
