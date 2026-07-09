function out = verify_eq17_5state_aprime_selfmod_L1(opts)
%VERIFY_EQ17_5STATE_APRIME_SELFMOD_L1  Does use_selfmod=true help or hurt
%   a'_x estimability and tracking in the MOTION scenario (1 Hz near-wall
%   oscillation)? Companion to verify_5state_aprime_selfmod_signflip.m,
%   which only tested the static hold. use_selfmod adds a dxh3-driven term
%   to F_e(4,5)/H(2,5) (see reference/eq17_analysis/derivation/
%   5state_aprime_unified.tex Sec.3-5,7) specifically to restore
%   hold-observability; this checks it does not regress the ALREADY-WORKING
%   motion-observability case (verify_eq17_5state_aprime_L1.m baseline).
%
%   Same 1 Hz near-wall osc scenario, Q55 kappa, and open-loop
%   (use_true_gain=true) harness as verify_eq17_5state_aprime_L1.m, but
%   sweeping use_selfmod={false,true} at fixed kappa=1 instead of sweeping
%   kappa. use_selfmod=false at kappa=1 reproduces that file's baseline exactly.
%
%   out = verify_eq17_5state_aprime_selfmod_L1()
%   out = verify_eq17_5state_aprime_selfmod_L1(struct('seeds',1:10))
%
%   See also: verify_eq17_5state_aprime_L1, verify_5state_aprime_selfmod_signflip

    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'seeds');      opts.seeds = 1:10;    end
    if ~isfield(opts, 'frequency');  opts.frequency = 1.0; end
    if ~isfield(opts, 'T_sim');      opts.T_sim = 4.0;     end
    if ~isfield(opts, 'kappa');      opts.kappa = 1;       end
    if ~isfield(opts, 'verbose');    opts.verbose = true;  end
    if ~isfield(opts, 'use_true_gain'); opts.use_true_gain = true; end
    if ~isfield(opts, 'make_figs');  opts.make_figs = false; end

    here = fileparts(mfilename('fullpath'));
    proj = fullfile(here, '..', '..');
    addpath(genpath(proj));

    f  = opts.frequency;
    vb = opts.verbose;
    out = struct('frequency', f, 'seeds', opts.seeds, 'kappa', opts.kappa);

    if vb
        fprintf('==== verify_eq17_5state_aprime_selfmod_L1 (motion scenario, selfmod on/off) ====\n');
        fprintf('  %d seeds, %g Hz near-wall osc, T=%g s, kappa=%g\n', ...
                numel(opts.seeds), f, opts.T_sim, opts.kappa);
    end

    r_off = run_variant(f, opts.T_sim, opts.seeds, opts.kappa, false, opts.use_true_gain);
    r_on  = run_variant(f, opts.T_sim, opts.seeds, opts.kappa, true,  opts.use_true_gain);
    out.baseline  = r_off;
    out.selfmod   = r_on;

    if vb
        fprintf('\n[a''_hat vs true a''(h) -- motion scenario, mean over seeds]\n');
        fprintf('  %-14s | %-22s | %-26s | %-24s | %-18s\n', 'use_selfmod', ...
                'corr(a''hat,a''true) xyz', 'near-wall |relerr| %% xyz', 'near spread %% xyz', 'track std nm xyz');
        fprintf('  %-14s | %6.2f %6.2f %6.2f      | %7.1f %7.1f %7.1f          | %6.1f %6.1f %6.1f | %6.1f %6.1f %6.1f\n', ...
                'false (base)', r_off.corr, 100*r_off.relerr_near, 100*r_off.spread_near, r_off.track_std_nm);
        fprintf('  %-14s | %6.2f %6.2f %6.2f      | %7.1f %7.1f %7.1f          | %6.1f %6.1f %6.1f | %6.1f %6.1f %6.1f\n', ...
                'true (selfmod)', r_on.corr, 100*r_on.relerr_near, 100*r_on.spread_near, r_on.track_std_nm);
        fprintf('  (z is wall-normal; corr->1 = tracks a''(h) shape; regression = selfmod corr_z drops or track_std_z grows a lot)\n');

        dcorr_z  = r_on.corr(3) - r_off.corr(3);
        dtrack_z = r_on.track_std_nm(3) - r_off.track_std_nm(3);
        fprintf('\n[verdict @ z-axis] d(corr_z) = %+.3f, d(track_std_z) = %+.2f nm -> %s\n', ...
                dcorr_z, dtrack_z, ternary_str(dcorr_z > -0.05 && dtrack_z < 5));
    end
    out.dcorr_z  = r_on.corr(3) - r_off.corr(3);
    out.dtrack_z = r_on.track_std_nm(3) - r_off.track_std_nm(3);
    out.no_regression = out.dcorr_z > -0.05 && out.dtrack_z < 5;

    if opts.make_figs
        loop_str = 'openloop'; if ~opts.use_true_gain; loop_str = 'closedloop'; end
        out_dir = fullfile(proj, 'test_results', 'verify_5state_aprime_selfmod_L1', ...
                           sprintf('f%gHz_%s', f, loop_str));
        out.fig_path = plot_selfmod_compare(r_off, r_on, out_dir, f, loop_str);
        if vb; fprintf('\n[figure] %s\n', out.fig_path); end
    end
end


function fig_path = plot_selfmod_compare(r_off, r_on, out_dir, freq, loop_str)
%PLOT_SELFMOD_COMPARE  z-axis a'_hat vs true, tracking error, and h_bar
%   context: use_selfmod=false vs true, for one loop mode (open/closed).
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end
    FS = 18; LFS = 12; AXLW = 2.0;
    az = 3;
    COL_T = [0 0 0]; COL_OFF = [0 0.45 0.74]; COL_ON = [0.85 0.33 0.10];
    w_mm = max(3, round(0.02 / r_off.Ts));
    sm = @(v) movmean(v, w_mm);

    f1 = figure('Position', [80 80 1150 900], 'Color', 'w', ...
                'NumberTitle', 'off', 'Visible', 'off');

    subplot(3,1,1); hold on;
    hT   = plot(r_off.t, r_off.aprime_true(:,az), '-', 'Color', COL_T, 'LineWidth', 2.5, ...
                'DisplayName', 'a''_{true}(h)');
    hOff = plot(r_off.t, sm(r_off.aprime_hat(:,az)), '-', 'Color', COL_OFF, 'LineWidth', 1.6, ...
                'DisplayName', sprintf('use\\_selfmod=false (corr_z=%.2f)', r_off.corr(az)));
    hOn  = plot(r_on.t,  sm(r_on.aprime_hat(:,az)),  '-', 'Color', COL_ON,  'LineWidth', 1.6, ...
                'DisplayName', sprintf('use\\_selfmod=true (corr_z=%.2f)', r_on.corr(az)));
    ylabel('a''_z  (1/pN)', 'FontSize', FS, 'FontWeight', 'bold');
    legend([hT hOff hOn], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

    subplot(3,1,2); hold on;
    plot(r_off.t_e, 1e3*r_off.e_ens(:,az), '-', 'Color', COL_OFF, 'LineWidth', 1.4, ...
         'DisplayName', sprintf('false (std=%.1f nm)', r_off.track_std_nm(az)));
    plot(r_on.t_e,  1e3*r_on.e_ens(:,az),  '-', 'Color', COL_ON,  'LineWidth', 1.4, ...
         'DisplayName', sprintf('true (std=%.1f nm)', r_on.track_std_nm(az)));
    ylabel('tracking err_z (nm)', 'FontSize', FS, 'FontWeight', 'bold');
    legend('Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

    subplot(3,1,3); hold on;
    plot(r_off.t, r_off.h_bar, '-', 'Color', [0.25 0.25 0.25], 'LineWidth', 1.8, 'HandleVisibility', 'off');
    yline(1.8, 'r--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    ylabel('h / R', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

    fig_path = fullfile(out_dir, sprintf('fig_selfmod_compare_z_f%gHz_%s.png', freq, loop_str));
    exportgraphics(f1, fig_path, 'Resolution', 150);
    close(f1);
end


function r = run_variant(f, T_sim, seeds, kappa, use_selfmod, use_true_gain)
%RUN_VARIANT  a'_hat vs true a'(h) aggregated over seeds for one use_selfmod setting.
    cfg = build_cfg(f, T_sim);
    cfg.Q_aprime_factor = kappa;
    cfg.use_selfmod = use_selfmod;

    ns = numel(seeds);
    ah_stack = []; at_stack = []; hb_stack = []; e_stack = [];
    t_vec = []; t_e_vec = []; Ts = [];
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
        e_stack  = cat(3, e_stack, e);
        if si == 1
            N = size(aprime_hat, 1);
            t_vec   = (0:N-1)' * Ts;
            t_e_vec = (0:size(e,1)-1)' * Ts;
        end
    end
    r.track_std_nm = mean(tr, 1);
    r.diverged      = maxe > 0.5;
    r.e_ens         = mean(e_stack, 3);   % [N-1 x 3] ensemble-mean tracking error [um]
    r.t_e           = t_e_vec;

    ah_ens = mean(ah_stack, 3);          % [N x 3] ensemble-mean a'_hat
    at_ens = mean(at_stack, 3);          % [N x 3] ensemble-mean a'_true
    hb_ens = mean(hb_stack, 2);          % [N x 1]

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
            sd = std(ah_stack(near, ax, :), 0, 3);
            spread(ax) = mean(sd) / max(mean(abs(at_ens(near, ax))), eps);
        else
            relerr(ax) = NaN; spread(ax) = NaN;
        end
    end

    r.corr        = corr;
    r.relerr_near = relerr;
    r.spread_near = spread;
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


function cfg = build_cfg(f, T_sim)
%BUILD_CFG  1 Hz near-wall osc (identical to verify_eq17_5state_aprime_L1.m).
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


function s = ternary_str(b)
    if b; s = 'PASS (no regression)'; else; s = 'FAIL (regression)'; end
end
