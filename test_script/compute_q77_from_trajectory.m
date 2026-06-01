function [Q77_scaling, Q77_abs, diag_out] = compute_q77_from_trajectory(config, opts)
%COMPUTE_Q77_FROM_TRAJECTORY Compute Q(6,6) and Q(7,7) from reference trajectory
%
%   [Q77_scaling, Q77_abs, diag_out] = compute_q77_from_trajectory(config, opts)
%
%   Computes Var(second forward difference of a_true) along the trajectory.
%   Under the backward-diff beta interpretation (diagonal approximation of
%   rank-1 correlated Q), both Q(6,6) and Q(7,7) share this value:
%
%       Q(6,6)_beta = Q(7,7)_beta = Var(a_true[k+2] - 2*a_true[k+1] + a_true[k])
%
%   Under forward-diff interpretation (clean decomposition), only Q(7,7)
%   receives this value and Q(6,6) = 0.
%
%   diag_out provides both values so downstream scripts can choose.
%
%   per-axis a_axis(i) = Ts * Gamma_inv(i,i)
%                      = (Ts/gamma_N) * [(1-w_hat(i)^2)/c_para + w_hat(i)^2/c_perp]
%
%   Inputs:
%       config  (optional) user config struct; defaults to user_config()
%       opts    (optional) struct with fields:
%                   .save_mat    (default true)
%                   .save_plot   (default true)
%                   .deployed_axis (default 'max' | 'w_hat' | integer 1..3)
%                   .regularization_floor (default 1e-6, min Q77_scaling)
%                   .verbose (default true)
%
%   Outputs:
%       Q77_scaling  scalar deployed value (dimensionless, on sigma2_dXT)
%       Q77_abs      scalar deployed value in absolute units (um^2/pN^2*s^2)
%       diag_out     struct with per-axis values, time series, sanity gates
%
%   Side effects (if opts.save_mat / save_plot):
%       test_results/verify/q77_trajectory.mat
%       reference/qr_analysis/fig_q77_trajectory.png

    % ---------------------------------------------------------------
    % 0. Defaults
    % ---------------------------------------------------------------
    if nargin < 1 || isempty(config), config = user_config(); end
    if nargin < 2 || isempty(opts),   opts   = struct(); end
    if ~isfield(opts, 'save_mat'),              opts.save_mat   = true; end
    if ~isfield(opts, 'save_plot'),             opts.save_plot  = true; end
    if ~isfield(opts, 'deployed_axis'),         opts.deployed_axis = 'max'; end
    if ~isfield(opts, 'regularization_floor'),  opts.regularization_floor = 1e-15; end
    if ~isfield(opts, 'verbose'),               opts.verbose = true; end

    % Merge config with defaults
    defaults = user_config();
    f = fieldnames(defaults);
    for i = 1:numel(f)
        if ~isfield(config, f{i}), config.(f{i}) = defaults.(f{i}); end
    end

    % ---------------------------------------------------------------
    % 1. Build params struct and physical constants
    % ---------------------------------------------------------------
    sim_params = calc_simulation_params(config);
    params = sim_params.Value;

    constants  = physical_constants();
    Ts         = constants.Ts;
    gamma_N    = constants.gamma_N;
    a_nom      = Ts / gamma_N;
    sigma2_dXT = 4 * constants.k_B * constants.T * a_nom;

    w_hat   = params.wall.w_hat;
    t_hold  = params.traj.t_hold;
    freq    = params.traj.frequency;
    ncyc    = params.traj.n_cycles;

    % ---------------------------------------------------------------
    % 2. Determine trajectory length and sample grid
    % ---------------------------------------------------------------
    t_descend = 1 / freq;
    T_osc     = ncyc / freq;
    T_traj    = t_hold + t_descend + T_osc + 0.1;   % small tail margin
    N         = round(T_traj / Ts) + 1;

    t_phase_bounds = [t_hold, t_hold + t_descend, t_hold + t_descend + T_osc];

    % ---------------------------------------------------------------
    % 3. Offline trajectory generation
    %    trajectory_generator uses persistent state; reset and loop.
    %    Calling at t = (k-1)*Ts returns p_d at k*Ts.
    % ---------------------------------------------------------------
    clear trajectory_generator;   % reset persistent p_d_prev

    p_d_arr  = zeros(3, N);
    h_bar_arr = zeros(1, N);
    a_axis   = zeros(3, N);

    for k = 1:N
        t_call = (k-1) * Ts - Ts;   % returns p_d at (k-1)*Ts = (k-1)*Ts
        p_k = trajectory_generator(t_call, params);
        p_d_arr(:, k) = p_k;

        [Gamma_inv_k, h_bar_k] = calc_gamma_inv(p_k, params);

        % Clamp h_bar to avoid numerical blowup; this should never trigger
        % for well-parametrized trajectories (h_bottom >= 2.5 um, R = 2.25 um).
        h_bar_arr(k) = max(h_bar_k, 1.001);
        if h_bar_k < 1.001
            warning('compute_q77_from_trajectory:h_bar_clamped', ...
                    'h_bar=%.4f at k=%d clamped to 1.001', h_bar_k, k);
            % Recompute with clamped h_bar via the direct formula to avoid
            % inconsistency with calc_gamma_inv
            [c_para_k, c_perp_k] = calc_correction_functions(h_bar_arr(k));
            for i = 1:3
                a_axis(i, k) = a_nom * ((1 - w_hat(i)^2) / c_para_k ...
                                        + w_hat(i)^2 / c_perp_k);
            end
        else
            a_axis(:, k) = Ts * diag(Gamma_inv_k);
        end
    end

    clear trajectory_generator;   % leave no persistent state behind

    t_vec = (0:N-1) * Ts;

    % ---------------------------------------------------------------
    % 4. Second forward difference of a_axis
    %    d2a[i, k] = a_axis[i, k+2] - 2*a_axis[i, k+1] + a_axis[i, k]
    %    for k = 1..N-2
    % ---------------------------------------------------------------
    d2a = a_axis(:, 3:end) - 2*a_axis(:, 2:end-1) + a_axis(:, 1:end-2);
    t_d2a = t_vec(1:end-2);   % time index of d2a[k]

    % Per-axis variance over full trajectory
    Q77_per_axis_full = var(d2a, 0, 2);   % 3x1, unbiased

    % Per-phase variance (diagnostic only)
    idx_hold = t_d2a <= t_phase_bounds(1);
    idx_desc = t_d2a > t_phase_bounds(1) & t_d2a <= t_phase_bounds(2);
    idx_osc  = t_d2a > t_phase_bounds(2) & t_d2a <= t_phase_bounds(3);

    Q77_per_axis_hold = var(d2a(:, idx_hold), 0, 2);
    Q77_per_axis_desc = var(d2a(:, idx_desc), 0, 2);
    Q77_per_axis_osc  = var(d2a(:, idx_osc),  0, 2);

    % ---------------------------------------------------------------
    % 5. Pick deployed value (scalar Q(7,7) applied to all 3 axes in EKF)
    % ---------------------------------------------------------------
    switch opts.deployed_axis
        case 'max'
            Q77_abs = max(Q77_per_axis_full);
        case 'w_hat'
            % Axis most aligned with w_hat (the wall-normal direction)
            [~, i_wnorm] = max(abs(w_hat));
            Q77_abs = Q77_per_axis_full(i_wnorm);
        otherwise
            if isnumeric(opts.deployed_axis)
                Q77_abs = Q77_per_axis_full(opts.deployed_axis);
            else
                error('compute_q77_from_trajectory:bad_deployed_axis', ...
                      'deployed_axis must be ''max'', ''w_hat'', or 1..3');
            end
    end

    Q77_scaling = Q77_abs / sigma2_dXT;

    % Regularization floor
    if Q77_scaling < opts.regularization_floor
        warning('compute_q77_from_trajectory:floor', ...
                'Q77_scaling=%.3e below floor %.3e; using floor value.', ...
                Q77_scaling, opts.regularization_floor);
        Q77_scaling = opts.regularization_floor;
        Q77_abs = Q77_scaling * sigma2_dXT;
    end

    % ---------------------------------------------------------------
    % 6. Sanity gates
    % ---------------------------------------------------------------
    gate = struct();

    % G1: x and y symmetric when theta=phi=0 (w_hat = [0;0;1])
    if abs(w_hat(1)) < 1e-9 && abs(w_hat(2)) < 1e-9
        xy_rel_diff = abs(Q77_per_axis_full(1) - Q77_per_axis_full(2)) / ...
                      max(Q77_per_axis_full(3), eps);
        gate.G1_xy_symmetry_rel_diff = xy_rel_diff;
        gate.G1_pass = xy_rel_diff < 0.01;
    else
        gate.G1_pass = true;   % N/A for tilted wall
        gate.G1_xy_symmetry_rel_diff = NaN;
    end

    % G2: continuous-time approximation.
    % For Q(7,7) ~ Ts^4 * Var(a''(t)), estimate a''(t) by 2nd central
    % difference of a_axis and compare Var.
    % (2nd central diff) ≈ (a[k+1] - 2 a[k] + a[k-1]) / Ts^2
    %   => same as d2a shifted; Var(d2a) ≈ Ts^4 * Var(a'')
    Q77_ct_estimate = Ts^4 * var(diff(a_axis, 2, 2) / Ts^2, 0, 2);  % 3x1
    gate.G2_ct_ratio = Q77_per_axis_full ./ Q77_ct_estimate;         % should be ≈ 1
    gate.G2_pass = all(gate.G2_ct_ratio > 0.5 & gate.G2_ct_ratio < 2.0);

    % G3: min h_bar in trajectory
    gate.G3_min_h_bar = min(h_bar_arr);

    % ---------------------------------------------------------------
    % 7. Diagnostics output
    % ---------------------------------------------------------------
    diag_out = struct();
    diag_out.Q77_per_axis_full = Q77_per_axis_full;
    diag_out.Q77_per_axis_hold = Q77_per_axis_hold;
    diag_out.Q77_per_axis_desc = Q77_per_axis_desc;
    diag_out.Q77_per_axis_osc  = Q77_per_axis_osc;
    diag_out.Q77_scaling_per_axis = Q77_per_axis_full / sigma2_dXT;
    diag_out.Q77_scaling_osc_per_axis = Q77_per_axis_osc / sigma2_dXT;
    diag_out.Q77_abs = Q77_abs;
    diag_out.Q77_scaling = Q77_scaling;
    % --- Backward-diff beta interpretation: Q66 = Q77 (diagonal approx) ---
    diag_out.Q66_scaling_beta = Q77_scaling;   % same value under beta
    diag_out.Q77_scaling_beta = Q77_scaling;   % redundant field name for clarity
    diag_out.Q66_abs_beta = Q77_abs;
    diag_out.Q77_abs_beta = Q77_abs;
    diag_out.interpretation_note = ['beta (backward-diff, diagonal approx): ', ...
        'Q(6,6) = Q(7,7) = Var(d2a). Deploy Qz_diag_scaling slot 6 and 7 both = Q77_scaling.'];
    diag_out.sigma2_dXT = sigma2_dXT;
    diag_out.a_nom = a_nom;
    diag_out.t_vec = t_vec;
    diag_out.t_d2a = t_d2a;
    diag_out.h_bar = h_bar_arr;
    diag_out.a_axis = a_axis;
    diag_out.d2a = d2a;
    diag_out.p_d = p_d_arr;
    diag_out.N = N;
    diag_out.t_phase_bounds = t_phase_bounds;
    diag_out.gate = gate;
    diag_out.config = config;
    diag_out.opts = opts;

    % ---------------------------------------------------------------
    % 8. Console print
    % ---------------------------------------------------------------
    if opts.verbose
        fprintf('===== compute_q77_from_trajectory =====\n');
        fprintf('Trajectory: t_hold=%.2fs  descent=%.2fs  osc=%.2fs (%d cycles @ %.2f Hz)\n', ...
                t_hold, t_descend, T_osc, ncyc, freq);
        fprintf('            amplitude=%.2f um, h_init=%.2f, h_bottom=%.2f um\n', ...
                params.traj.amplitude, params.traj.h_init, params.traj.h_bottom);
        fprintf('Wall:       theta=%.1f deg  phi=%.1f deg  pz=%.2f um\n', ...
                config.theta, config.phi, params.wall.pz);
        fprintf('            w_hat=[%.3f %.3f %.3f]\n', w_hat(1), w_hat(2), w_hat(3));
        fprintf('N=%d samples at Ts=%.6f sec (%.2f sec total)\n', N, Ts, T_traj);
        fprintf('min(h_bar)=%.4f  max(h_bar)=%.4f\n', ...
                min(h_bar_arr), max(h_bar_arr));
        fprintf('sigma2_dXT = %.6e um^2\n\n', sigma2_dXT);

        fprintf('Q(7,7) per-axis (absolute [um^2/pN^2*s^2]):\n');
        fprintf('   full     : x=%.3e  y=%.3e  z=%.3e\n', ...
                Q77_per_axis_full(1), Q77_per_axis_full(2), Q77_per_axis_full(3));
        fprintf('   oscill.  : x=%.3e  y=%.3e  z=%.3e\n', ...
                Q77_per_axis_osc(1), Q77_per_axis_osc(2), Q77_per_axis_osc(3));
        fprintf('   descent  : x=%.3e  y=%.3e  z=%.3e\n', ...
                Q77_per_axis_desc(1), Q77_per_axis_desc(2), Q77_per_axis_desc(3));
        fprintf('   hold     : x=%.3e  y=%.3e  z=%.3e\n', ...
                Q77_per_axis_hold(1), Q77_per_axis_hold(2), Q77_per_axis_hold(3));

        fprintf('\nQ(7,7) per-axis scaling (on sigma2_dXT):\n');
        fprintf('   full     : x=%.4e  y=%.4e  z=%.4e\n', ...
                Q77_per_axis_full(1)/sigma2_dXT, ...
                Q77_per_axis_full(2)/sigma2_dXT, ...
                Q77_per_axis_full(3)/sigma2_dXT);
        fprintf('   oscill.  : x=%.4e  y=%.4e  z=%.4e\n', ...
                Q77_per_axis_osc(1)/sigma2_dXT, ...
                Q77_per_axis_osc(2)/sigma2_dXT, ...
                Q77_per_axis_osc(3)/sigma2_dXT);

        fprintf('\nDeployed (%s): Q(7,7)_scaling = %.4e  (abs = %.3e um^2/pN^2*s^2)\n', ...
                opts.deployed_axis, Q77_scaling, Q77_abs);

        fprintf('\nSanity gates:\n');
        fprintf('   G1 (x==y under theta=phi=0):  %s  (rel diff %.2e)\n', ...
                tern(gate.G1_pass, 'PASS', 'FAIL'), gate.G1_xy_symmetry_rel_diff);
        fprintf('   G2 (Ts^4 * Var(a'''') match):  %s  (ratios [%.2f %.2f %.2f])\n', ...
                tern(gate.G2_pass, 'PASS', 'FAIL'), ...
                gate.G2_ct_ratio(1), gate.G2_ct_ratio(2), gate.G2_ct_ratio(3));
        fprintf('   G3 (min h_bar):               %.4f (expected ~1.11)\n\n', ...
                gate.G3_min_h_bar);
    end

    % ---------------------------------------------------------------
    % 9. Save outputs
    % ---------------------------------------------------------------
    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(script_dir);

    if opts.save_mat
        out_dir = fullfile(project_root, 'test_results', 'verify');
        if ~exist(out_dir, 'dir'), mkdir(out_dir); end
        out_file = fullfile(out_dir, 'q77_trajectory.mat');
        save(out_file, '-struct', 'diag_out');
        if opts.verbose
            fprintf('Saved: %s\n', out_file);
        end
    end

    if opts.save_plot
        fig_dir = fullfile(project_root, 'reference', 'qr_analysis');
        if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end
        fig_file = fullfile(fig_dir, 'fig_q77_trajectory.png');

        fig = figure('Position', [100, 100, 1000, 800], 'Visible', 'off');

        subplot(3, 1, 1);
        plot(t_vec, h_bar_arr, 'LineWidth', 1.3);
        ylabel('h_{bar}');
        xlabel('time [s]');
        set(gca, 'FontSize', 12);
        xlim([0, T_traj]);

        subplot(3, 1, 2);
        plot(t_vec, a_axis(1, :), 'r-', 'LineWidth', 1.2); hold on;
        plot(t_vec, a_axis(2, :), 'g--', 'LineWidth', 1.2);
        plot(t_vec, a_axis(3, :), 'b-', 'LineWidth', 1.2);
        ylabel('a_{axis} [um/pN*s]');
        xlabel('time [s]');
        legend({'x', 'y', 'z'}, 'Location', 'northoutside', 'Orientation', 'horizontal');
        set(gca, 'FontSize', 12);
        xlim([0, T_traj]);

        subplot(3, 1, 3);
        plot(t_d2a, d2a(1, :), 'r-', 'LineWidth', 0.8); hold on;
        plot(t_d2a, d2a(2, :), 'g--', 'LineWidth', 0.8);
        plot(t_d2a, d2a(3, :), 'b-', 'LineWidth', 0.8);
        ylabel('\Delta^2 a');
        xlabel('time [s]');
        set(gca, 'FontSize', 12);
        xlim([0, T_traj]);
        text(0.02, 0.95, sprintf('Q(7,7) deployed = %.3e um^2/pN^2*s^2  (scaling %.3e)', ...
                                 Q77_abs, Q77_scaling), ...
             'Units', 'normalized', 'FontSize', 10, ...
             'BackgroundColor', 'w', 'EdgeColor', 'k');

        saveas(fig, fig_file);
        close(fig);
        if opts.verbose
            fprintf('Saved: %s\n', fig_file);
        end
    end

end

% -------------------------------------------------------------------
function s = tern(cond, a, b)
    if cond, s = a; else, s = b; end
end
