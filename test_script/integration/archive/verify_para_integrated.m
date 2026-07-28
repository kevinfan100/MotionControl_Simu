function results = verify_para_integrated()
%VERIFY_PARA_INTEGRATED  Closed-loop 5-seed test of the INTEGRATED para-c 4-state
%   EKF (motion_control_law_eq17_4state_para, slot 4 = wall-shape constant theta).
%
%   results = verify_para_integrated()
%
%   EXPLORATORY. Standalone harness (does NOT use run_pure_simulation). Mirrors
%   the closed-loop loop of plot_para_aprime.m (step_dynamics / trajectory_generator
%   / calc_thermal_force, sensor-delay buffer length d+1), but calls the NEW
%   integrated controller each step -- its EKF slot4=theta provides a_ctrl
%   internally, so there is NO a_ctrl_override hook (the decoupled study used one).
%
%   Trajectory: descent (0-1 s) -> oscillation (1-4 s), no holds (t_hold=0).
%   5 seeds, full thermal + measurement noise.
%
%   Metrics (per seed + aggregate):
%     - tracking std x/y/z [nm] over the oscillation band (t >= 1 s)
%     - min h_bar (true) over the whole run  (>1 = no wall crash)
%     - z a' RMS [%] over the in-data band  h_bar in [1.5, 3.33]
%       (a'_est at theta_hat[k] vs true a', both evaluated at the TRUE h_bar so
%        the metric isolates model/theta error from sensor-noise on h_bar)
%     - z a-level bias [%] over the same band
%     - theta_hat_z final (last step), aggregate mean +/- std across seeds
%     - divergence flag (NaN/Inf or runaway tracking)
%
%   Saves results to test_results/para_integrated/para_integrated_results.mat
%   (gitignored). Prints a per-seed + aggregate table to the console.
%
%   See also: motion_control_law_eq17_4state_para, plot_para_aprime,
%             build_eq17_6state_constants

    [sd, ~, ~] = fileparts(mfilename('fullpath'));
    proj = fileparts(fileparts(sd));
    addpath(genpath(fullfile(proj, 'model')));
    od = fullfile(proj, 'test_results', 'para_integrated');
    if ~exist(od, 'dir'); mkdir(od); end

    % ---------------- config (per team-lead spec) ----------------
    config = user_config();
    config.trajectory_type   = 'osc';
    config.h_init            = 20;
    config.h_bottom          = 2.5;
    config.amplitude         = 2.5;
    config.frequency         = 1;
    config.n_cycles          = 3;
    config.t_hold            = 0;
    config.T_sim             = 4.0;
    config.ctrl_enable       = true;
    config.meas_noise_enable = true;
    config.thermal_enable    = true;
    config.lambda_c          = 0.7;
    config.a_pd              = 0.05;
    config.a_cov             = 0.05;
    config.meas_noise_std    = [0.00062; 0.00057; 0.00331];

    n_seeds   = 5;
    band_lo   = 1.5;       % in-data band lower h_bar
    band_hi   = 3.33;      % in-data band upper h_bar
    t_osc     = 1.0;       % oscillation starts (descent = 1/frequency = 1 s)

    % ---------------- per-seed storage ----------------
    track_std = zeros(3, n_seeds);   % [um] -> *1e3 for nm at print
    min_hbar  = zeros(1, n_seeds);
    aprime_z_rms = zeros(1, n_seeds);   % clean: est & true both at TRUE h_bar
    aprime_z_rms_meas = zeros(1, n_seeds);  % decoupled-style: est at MEASURED h_bar
    a_z_bias     = zeros(1, n_seeds);
    theta_z_fin  = zeros(1, n_seeds);
    diverged     = false(1, n_seeds);
    max_abs_e    = zeros(1, n_seeds);

    for s = 1:n_seeds
        % Reset all stateful functions, seed, then build params (so thermal.seed
        % is deterministic per outer seed).
        clear motion_control_law_eq17_4state_para trajectory_generator calc_thermal_force;
        rng(s);
        P = calc_simulation_params(config).Value;

        Ts = P.common.Ts; gN = P.common.gamma_N; R = P.common.R;
        w  = P.wall.w_hat; pz = P.wall.pz; a_nom = Ts / gN;

        eq = struct('lambda_c', config.lambda_c, 'option', 'A_MA2_full', ...
                    'sigma2_n_s', (config.meas_noise_std(:)).^2, ...
                    'kBT', P.ctrl.k_B * P.ctrl.T, 't_warmup_kf', 0, ...
                    'h_bar_safe', band_lo, 'd', 2, 'a_cov', config.a_cov, ...
                    'a_pd', config.a_pd, 'iir_warmup_mode', 'prefill');
        ctrl_const = build_eq17_6state_constants(eq);
        ctrl_const.theta_init = [0.5625; 0.5625; 1.125];   % x,y Goldman 9/16 ; z Brenner 9/8
        ctrl_const.P_theta0   = 9;
        d = ctrl_const.d;

        N = round(config.T_sim / Ts) + 1;
        p_curr  = P.common.p0;
        p_m_buf = repmat(p_curr, 1, d + 1);   % sensor-delay buffer (length d+1)
        pd_ctrl = p_curr;

        % logs
        tL    = zeros(N, 1);
        eL    = zeros(3, N);     % tracking error p_curr - p_d
        hbtL  = zeros(N, 1);     % TRUE h_bar
        hbmL  = zeros(N, 1);     % controller (delayed measured) h_bar
        thzL  = zeros(N, 1);     % theta_hat_z (posterior)
        bad   = false;

        for k = 1:N
            t = (k - 1) * Ts;
            [pd_kp1, del_pd] = trajectory_generator(t, P);
            pd_k  = pd_ctrl;
            p_m_del = p_m_buf(:, 1);

            [f_d_k, ~, diag_k] = motion_control_law_eq17_4state_para(del_pd, pd_k, p_m_del, P, ctrl_const);

            % advance true dynamics
            if P.thermal.enable > 0.5
                f_th = calc_thermal_force(p_curr, P);
            else
                f_th = zeros(3, 1);
            end
            p_curr = step_dynamics(p_curr, f_d_k + f_th, P, Ts);

            if any(~isfinite(p_curr)) || any(~isfinite(f_d_k))
                bad = true; tL(k) = t + Ts; break;
            end

            % logs (true position now at t+Ts, desired pd_kp1 also at t+Ts)
            tL(k)    = t + Ts;
            eL(:, k) = p_curr - pd_kp1;
            hbtL(k)  = (dot(p_curr, w) - pz) / R;
            hbmL(k)  = diag_k.h_bar;
            thzL(k)  = diag_k.theta_hat(3);

            % measurement + buffer shift
            if config.meas_noise_enable
                nm = config.meas_noise_std(:) .* randn(3, 1);
            else
                nm = zeros(3, 1);
            end
            p_m_buf = [p_m_buf(:, 2:end), p_curr + nm];
            pd_ctrl = pd_kp1;
        end

        if bad
            diverged(s) = true;
            kmax = find(tL > 0, 1, 'last');
            track_std(:, s) = NaN; min_hbar(s) = NaN;
            aprime_z_rms(s) = NaN; a_z_bias(s) = NaN;
            theta_z_fin(s) = thzL(max(kmax - 1, 1)); max_abs_e(s) = Inf;
            fprintf('seed %d: DIVERGED at t=%.3f s\n', s, tL(kmax));
            continue;
        end

        % ---- metrics ----
        osc   = tL >= t_osc;
        track_std(:, s) = std(eL(:, osc), 0, 2);
        min_hbar(s)     = min(hbtL(hbtL > 0));
        max_abs_e(s)    = max(abs(eL(:)));
        theta_z_fin(s)  = thzL(N);

        % z a' / a over in-data band (true h_bar in [band_lo, band_hi], osc)
        band = osc & (hbtL >= band_lo) & (hbtL <= band_hi);
        idx  = find(band);
        relE_ap = zeros(numel(idx), 1);
        relE_ap_m = zeros(numel(idx), 1);
        relE_a  = zeros(numel(idx), 1);
        for ii = 1:numel(idx)
            kk = idx(ii);
            hbt = max(hbtL(kk), 1.001);
            [~, cpz, dz] = calc_correction_functions(hbt, true);
            aT  = a_nom / cpz;
            apT = (1 / R) * (-a_nom / cpz^2) * dz.dc_perp_dh;
            % (a) clean: estimate at the SAME true h_bar with theta_hat[kk]
            hq  = max(hbt - 1, 0.05);
            phi = 1 / hq;
            ce  = 1 + thzL(kk) * phi;
            dce = -thzL(kk) * phi^2;             % dc_est/dh_bar
            aE  = a_nom / ce;
            apE = (1 / R) * (-a_nom / ce^2) * dce;
            relE_ap(ii) = (apE - apT) / apT;
            relE_a(ii)  = (aE - aT) / aT;
            % (b) decoupled-style: estimate at MEASURED (controller) h_bar
            hqm  = max(hbmL(kk) - 1, 0.05);
            phim = 1 / hqm;
            cem  = 1 + thzL(kk) * phim;
            dcem = -thzL(kk) * phim^2;
            apEm = (1 / R) * (-a_nom / cem^2) * dcem;
            relE_ap_m(ii) = (apEm - apT) / apT;
        end
        aprime_z_rms(s)      = 100 * sqrt(mean(relE_ap.^2));
        aprime_z_rms_meas(s) = 100 * sqrt(mean(relE_ap_m.^2));
        a_z_bias(s)          = 100 * mean(relE_a);

        fprintf(['seed %d: track std [nm] x=%.1f y=%.1f z=%.1f | min hbar=%.3f | ' ...
                 'z a'' RMS=%.1f%% (meas-h %.1f%%) | z a bias=%+.1f%% | theta_z_fin=%.3f\n'], ...
                s, 1e3 * track_std(1, s), 1e3 * track_std(2, s), 1e3 * track_std(3, s), ...
                min_hbar(s), aprime_z_rms(s), aprime_z_rms_meas(s), a_z_bias(s), theta_z_fin(s));
    end

    % ---------------- aggregate ----------------
    ok = ~diverged;
    agg = struct();
    agg.track_std_nm_mean = 1e3 * mean(track_std(:, ok), 2);
    agg.min_hbar_worst    = min(min_hbar(ok));
    agg.aprime_z_rms_mean = mean(aprime_z_rms(ok));
    agg.aprime_z_rms_meas_mean = mean(aprime_z_rms_meas(ok));
    agg.a_z_bias_mean     = mean(a_z_bias(ok));
    agg.theta_z_mean      = mean(theta_z_fin(ok));
    agg.theta_z_std       = std(theta_z_fin(ok));
    agg.n_diverged        = sum(diverged);

    fprintf('\n==== AGGREGATE (%d/%d stable) ====\n', sum(ok), n_seeds);
    fprintf('tracking std [nm]: x=%.1f y=%.1f z=%.1f\n', ...
            agg.track_std_nm_mean(1), agg.track_std_nm_mean(2), agg.track_std_nm_mean(3));
    fprintf('worst min h_bar  : %.3f  (>1 = no wall crash)\n', agg.min_hbar_worst);
    fprintf('z a'' RMS (true-h): %.1f%%\n', agg.aprime_z_rms_mean);
    fprintf('z a'' RMS (meas-h): %.1f%%  (decoupled-study style)\n', agg.aprime_z_rms_meas_mean);
    fprintf('z a-level bias   : %+.1f%%\n', agg.a_z_bias_mean);
    fprintf('theta_hat_z final: %.3f +/- %.3f\n', agg.theta_z_mean, agg.theta_z_std);
    fprintf('diverged seeds   : %d\n', agg.n_diverged);

    % ---------------- save ----------------
    results = struct();
    results.config       = config;
    results.track_std    = track_std;
    results.min_hbar     = min_hbar;
    results.aprime_z_rms = aprime_z_rms;
    results.aprime_z_rms_meas = aprime_z_rms_meas;
    results.a_z_bias     = a_z_bias;
    results.theta_z_fin  = theta_z_fin;
    results.diverged     = diverged;
    results.max_abs_e    = max_abs_e;
    results.agg          = agg;
    save(fullfile(od, 'para_integrated_results.mat'), 'results');
    fprintf('saved results to %s\n', fullfile(od, 'para_integrated_results.mat'));
end
