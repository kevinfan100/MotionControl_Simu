function results = run_v2_h50_e2e(opts)
%RUN_V2_H50_E2E  V2 H=50 positioning 5-seed CI verification (eq17_7state)
%
%   results = run_v2_h50_e2e()
%   results = run_v2_h50_e2e(opts)
%
%   End-to-end verification wrapper for the v2 eq17_7state controller at
%   h_init = 50 um, positioning trajectory, T_sim = 5 s. Mirrors the
%   sigma-ratio-filter qr-branch verify_qr_positioning_run pattern: runs
%   N seeds, aggregates per-axis tracking std and a_hat statistics, and
%   compares against the Phase 7 oracle (predict_closed_loop_var_eq17_v2).
%
%   This script is the **scaffold** invoked by Wave 4 Agent F. It does not
%   modify any model code; it only orchestrates run_pure_simulation calls.
%
%   ----- Inputs (struct opts, all optional) -----
%       opts.seeds      - integer vector of RNG seeds (default [1, 2, 3, 4, 5])
%       opts.T_sim      - simulation duration [sec]   (default 5)
%       opts.h_init     - h_init [um]                  (default 50)
%       opts.t_warmup   - start time for stats [sec]   (default 0.5)
%       opts.verbose    - print per-seed progress      (default true)
%       opts.run_oracle - call predict_closed_loop_var_eq17_v2 oracle for
%                         comparison (default true; requires model/diag on path)
%
%   ----- Output (struct results) -----
%       results.config              - effective config struct (last seed)
%       results.metrics_per_seed    - struct with raw per-seed metrics:
%                                       .tracking_std        N x 3 [um]
%                                       .a_hat_mean          N x 3 [um/pN]
%                                       .a_hat_std           N x 3 [um/pN]
%                                       .s2_dx_observed      N x 3 [um^2]
%                                     (axis order: [x, y, z])
%       results.aggregate           - struct with cross-seed aggregation:
%                                       .tracking_std_mean   1 x 3 [um]
%                                       .tracking_std_std    1 x 3 [um]
%                                       .a_hat_mean_mean     1 x 3 [um/pN]
%                                       .a_hat_bias_pct      1 x 3 [%]
%                                       .a_hat_std_mean      1 x 3 [um/pN]
%                                       .s2_dx_obs_mean      1 x 3 [um^2]
%       results.oracle              - (if run_oracle) struct from
%                                     predict_closed_loop_var_eq17_v2:
%                                       .s2_dx_pred          1 x 3 [um^2]
%                                       .ratio_obs_to_pred   1 x 3 [-]
%       results.meta                - timestamp, driver version, etc.
%
%   ----- Workflow -----
%       1. Set up base config: positioning, h_init=50, T_sim=5,
%          meas_noise + thermal both ON.
%       2. Compute per-axis a_nom from physical_constants and wall geometry
%          (used for a_hat bias %).
%       3. For each seed: call run_pure_simulation, extract metrics over
%          t >= opts.t_warmup window.
%       4. Aggregate: cross-seed mean and std; print summary table.
%       5. If oracle requested: build oracle opts struct from ctrl_const
%          + first-seed time-mean Q77/R_axis, compute predicted s2_dx,
%          report ratio.
%
%   ----- Notes -----
%       * ekf_out column order (per motion_control_law_eq17_7state):
%             col 1 = a_hat_x  (axis 1, tangential)
%             col 2 = a_hat_z  (axis 3, normal)
%             col 3 = a_hat_y  (axis 2, tangential)
%             col 4 = h_bar
%         Wrapper rearranges to per-axis [x, y, z] order in metrics.
%       * Trajectory mode: trajectory_type='positioning' stays at h_init
%         (verified in trajectory_generator.m line 56: type > 1.5 branch).
%       * No simulation is invoked when this file is checkcode'd; only
%         when the user explicitly calls run_v2_h50_e2e().
%
%   See also: run_pure_simulation, predict_closed_loop_var_eq17_v2,
%             motion_control_law_eq17_7state

    % --- 0. Defaults ---
    if nargin < 1 || isempty(opts); opts = struct(); end
    if ~isfield(opts, 'seeds');      opts.seeds      = [1, 2, 3, 4, 5]; end
    if ~isfield(opts, 'T_sim');      opts.T_sim      = 5;               end
    if ~isfield(opts, 'h_init');     opts.h_init     = 50;              end
    if ~isfield(opts, 't_warmup');   opts.t_warmup   = 0.5;             end
    if ~isfield(opts, 'verbose');    opts.verbose    = true;            end
    if ~isfield(opts, 'run_oracle'); opts.run_oracle = true;            end

    % --- 1. Path setup (mirror run_simulation.m) ---
    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(script_dir);
    addpath(fullfile(project_root, 'model'));
    addpath(fullfile(project_root, 'model', 'config'));
    addpath(fullfile(project_root, 'model', 'wall_effect'));
    addpath(fullfile(project_root, 'model', 'thermal_force'));
    addpath(fullfile(project_root, 'model', 'trajectory'));
    addpath(fullfile(project_root, 'model', 'controller'));
    addpath(fullfile(project_root, 'model', 'pure_matlab'));
    addpath(fullfile(project_root, 'model', 'diag'));

    % --- 2. Base config ---
    config = user_config();
    config.h_init           = opts.h_init;
    config.h_bottom         = opts.h_init;             % no descent in positioning
    config.amplitude        = 0;                       % no oscillation
    config.trajectory_type  = 'positioning';            % hold at h_init
    config.T_sim            = opts.T_sim;
    config.ctrl_enable      = true;
    config.meas_noise_enable = true;
    config.thermal_enable   = true;
    config.lambda_c         = 0.7;
    config.a_pd             = 0.05;
    config.a_cov            = 0.05;
    config.sigma2_w_fD      = 0;        % Phase 5 §5.4 baseline
    % Match run_simulation.m h=50 noise std (Wave 1 audit)
    config.meas_noise_std   = [0.00062; 0.000057; 0.00331];
    % Note: config.controller_type is unused by run_pure_simulation (driver
    % is hard-wired to motion_control_law_eq17_7state). Kept for trace only.
    config.controller_type  = 7;

    % --- 3. Resolve params once for nominal a per-axis (used in bias %) ---
    params = calc_simulation_params(config);
    P = params.Value;

    Ts        = P.common.Ts;
    gamma_N   = P.common.gamma_N;
    R_radius  = P.common.R;
    w_hat     = P.wall.w_hat;
    pz_wall   = P.wall.pz;
    p0        = P.common.p0;

    a_nom_freespace = Ts / gamma_N;                     % [um/pN]
    h_init_um  = dot(p0, w_hat) - pz_wall;
    h_bar_init = max(h_init_um / R_radius, 1.001);
    [c_para0, c_perp0] = calc_correction_functions(h_bar_init);
    % Per-axis nominal: x,y use c_para; z uses c_perp (standard wall geom).
    a_nom_per_axis = [a_nom_freespace / c_para0, ...
                      a_nom_freespace / c_para0, ...
                      a_nom_freespace / c_perp0];       % 1 x 3, [x, y, z]

    % --- 4. Seed loop ---
    n_seeds = length(opts.seeds);
    tracking_std_seed   = zeros(n_seeds, 3);
    a_hat_mean_seed     = zeros(n_seeds, 3);
    a_hat_std_seed      = zeros(n_seeds, 3);
    s2_dx_obs_seed      = zeros(n_seeds, 3);

    n_warmup = round(opts.t_warmup / Ts);
    if opts.verbose
        fprintf('[run_v2_h50_e2e] T_sim=%.2fs, h_init=%g um, n_seeds=%d, t_warmup=%.2fs\n', ...
                opts.T_sim, opts.h_init, n_seeds, opts.t_warmup);
    end

    for s = 1:n_seeds
        seed = opts.seeds(s);
        run_opts.seed    = seed;
        run_opts.verbose = false;

        if opts.verbose
            fprintf('  seed %d/%d (=%d) ...\n', s, n_seeds, seed);
        end

        simOut = run_pure_simulation(config, run_opts);

        % Stats window: t >= opts.t_warmup (skip warmup)
        N_samp = length(simOut.tout);
        idx = (n_warmup + 1):N_samp;

        % Tracking error per axis [um]: (p_d - p_m)
        del_p = simOut.p_d_out(idx, :) - simOut.p_m_out(idx, :);
        tracking_std_seed(s, :) = std(del_p, 0, 1);     % 1 x 3
        s2_dx_obs_seed(s, :)    = var(del_p, 0, 1);     % 1 x 3 [um^2]

        % Extract a_hat from ekf_out; remap col order [x, z, y] -> [x, y, z]
        a_hat_x = simOut.ekf_out(idx, 1);   % col 1 = a_hat_x
        a_hat_z = simOut.ekf_out(idx, 2);   % col 2 = a_hat_z
        a_hat_y = simOut.ekf_out(idx, 3);   % col 3 = a_hat_y
        a_hat_mean_seed(s, :) = [mean(a_hat_x), mean(a_hat_y), mean(a_hat_z)];
        a_hat_std_seed(s, :)  = [std(a_hat_x),  std(a_hat_y),  std(a_hat_z) ];
    end

    % --- 5. Aggregate across seeds ---
    aggregate.tracking_std_mean = mean(tracking_std_seed, 1);
    aggregate.tracking_std_std  = std(tracking_std_seed, 0, 1);
    aggregate.a_hat_mean_mean   = mean(a_hat_mean_seed, 1);
    aggregate.a_hat_bias_pct    = (aggregate.a_hat_mean_mean - a_nom_per_axis) ...
                                  ./ a_nom_per_axis * 100;
    aggregate.a_hat_std_mean    = mean(a_hat_std_seed, 1);
    aggregate.s2_dx_obs_mean    = mean(s2_dx_obs_seed, 1);

    % --- 6. Optional oracle prediction ---
    oracle = struct();
    if opts.run_oracle
        % Build oracle opts from ctrl_const + per-seed-mean Q77/R for h=50.
        % For h=50 positioning: Q77 ~ 0 (frozen a, ḣ = ḧ = 0), R(2,2) intrinsic
        % from Phase 6 §4.1 with current a_hat_mean. Q77, R driven mostly by
        % theory; pull from controller's offline ctrl_const.
        eq17_opts.lambda_c    = config.lambda_c;
        eq17_opts.option      = 'A_MA2_full';
        eq17_opts.sigma2_n_s  = (config.meas_noise_std(:)).^2;
        eq17_opts.kBT         = P.ctrl.k_B * P.ctrl.T;
        eq17_opts.t_warmup_kf = 0.2;
        eq17_opts.h_bar_safe  = 1.5;
        eq17_opts.d           = 2;
        eq17_opts.a_cov       = config.a_cov;
        eq17_opts.a_pd        = config.a_pd;
        eq17_opts.sigma2_w_fD = config.sigma2_w_fD;
        ctrl_const            = build_eq17_constants(eq17_opts);

        % R(2,2) intrinsic per axis at h=50 with a = aggregate.a_hat_mean_mean
        a_used = aggregate.a_hat_mean_mean;      % 1x3, [x,y,z]
        R_axis = ctrl_const.a_cov * ctrl_const.IF_var ...
                 * (a_used + ctrl_const.xi_per_axis(:).').^2;
        Q77_axis = [0, 0, 0];                     % positioning baseline

        oracle_opts = struct( ...
            'lambda_c',    config.lambda_c, ...
            'a_x_axis',    a_used, ...
            'h_bar',       h_bar_init, ...
            'sigma2_n_s',  eq17_opts.sigma2_n_s(:).', ...
            'k_B',         P.ctrl.k_B, ...
            'T',           P.ctrl.T, ...
            'Q77_axis',    Q77_axis, ...
            'R_axis',      R_axis, ...
            'a_cov',       config.a_cov, ...
            'a_pd',        config.a_pd, ...
            'C_dpmr',      ctrl_const.C_dpmr, ...
            'C_n',         ctrl_const.C_n, ...
            'IF_var',      ctrl_const.IF_var, ...
            'sigma2_w_fD', config.sigma2_w_fD, ...
            'scenario',    'positioning' );

        [s2_dx_pred, s2_e_xD_pred, s2_e_a_pred, diag_pred] = ...
            predict_closed_loop_var_eq17_v2(oracle_opts);

        oracle.s2_dx_pred       = s2_dx_pred(:).';     % 1x3
        oracle.s2_e_xD_pred     = s2_e_xD_pred(:).';
        oracle.s2_e_a_pred      = s2_e_a_pred(:).';
        oracle.diag             = diag_pred;
        oracle.ratio_obs_to_pred = aggregate.s2_dx_obs_mean ./ s2_dx_pred(:).';
    end

    % --- 7. Pack output ---
    results.config = config;
    results.metrics_per_seed = struct( ...
        'tracking_std', tracking_std_seed, ...
        'a_hat_mean',   a_hat_mean_seed, ...
        'a_hat_std',    a_hat_std_seed, ...
        's2_dx_observed', s2_dx_obs_seed );
    results.aggregate = aggregate;
    results.oracle    = oracle;
    results.meta = struct( ...
        'driver',          'run_v2_h50_e2e', ...
        'driver_version',  '1.0', ...
        'a_nom_per_axis',  a_nom_per_axis, ...
        'h_bar_init',      h_bar_init, ...
        'n_warmup',        n_warmup, ...
        'timestamp',       char(datetime('now')) );

    % --- 8. Print summary ---
    if opts.verbose
        fprintf('\n=== V2 H=50 Positioning %d-seed CI Results ===\n', n_seeds);
        fprintf('Tracking std (mean +/- std across seeds, nm):\n');
        fprintf('  x: %.2f +/- %.2f\n', aggregate.tracking_std_mean(1)*1e3, ...
                                         aggregate.tracking_std_std(1)*1e3);
        fprintf('  y: %.2f +/- %.2f\n', aggregate.tracking_std_mean(2)*1e3, ...
                                         aggregate.tracking_std_std(2)*1e3);
        fprintf('  z: %.2f +/- %.2f\n', aggregate.tracking_std_mean(3)*1e3, ...
                                         aggregate.tracking_std_std(3)*1e3);
        fprintf('a_hat bias (cross-seed mean, %% of a_nom):\n');
        fprintf('  x: %+.2f%%\n', aggregate.a_hat_bias_pct(1));
        fprintf('  y: %+.2f%%\n', aggregate.a_hat_bias_pct(2));
        fprintf('  z: %+.2f%%\n', aggregate.a_hat_bias_pct(3));
        fprintf('a_hat std (mean across seeds, [um/pN]):\n');
        fprintf('  x: %.4e\n', aggregate.a_hat_std_mean(1));
        fprintf('  y: %.4e\n', aggregate.a_hat_std_mean(2));
        fprintf('  z: %.4e\n', aggregate.a_hat_std_mean(3));
        if opts.run_oracle
            fprintf('Observed/Predicted s2_dx ratio:\n');
            fprintf('  x: %.3f   (obs %.3e, pred %.3e [um^2])\n', ...
                    oracle.ratio_obs_to_pred(1), ...
                    aggregate.s2_dx_obs_mean(1), oracle.s2_dx_pred(1));
            fprintf('  y: %.3f   (obs %.3e, pred %.3e [um^2])\n', ...
                    oracle.ratio_obs_to_pred(2), ...
                    aggregate.s2_dx_obs_mean(2), oracle.s2_dx_pred(2));
            fprintf('  z: %.3f   (obs %.3e, pred %.3e [um^2])\n', ...
                    oracle.ratio_obs_to_pred(3), ...
                    aggregate.s2_dx_obs_mean(3), oracle.s2_dx_pred(3));
        end
        fprintf('=== End of V2 H=50 5-seed CI ===\n');
    end

end
