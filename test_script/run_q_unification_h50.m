function results = run_q_unification_h50(opts)
%RUN_Q_UNIFICATION_H50  Q unification 5-seed positioning at h=50 um.
%
%   Drives 5-seed positioning at h_init=50, T_sim=5 with the latest
%   user_config (Q66_OL_mode=true, force_Q77_zero=true, Fe Row-3 patched).
%   Computes per-axis tracking RMS, a_hat bias/std, R22 predicted vs
%   empirical ratio (window=300 steps), and steady-state P_a/P_dx.
%   Saves summary.mat + summary.md + 3 PNGs into
%   test_results/q_unification_h50/.
%
%   Notes:
%     * Uses run_pure_simulation with opts.collect_diag=true.
%     * R22 predicted (per axis, time-series) = R22_prefactor *
%       IF_eff_per_axis(ax) * (a_hat(t) + xi_per_axis(ax))^2
%       (intrinsic term — matches the motion_control_law_eq17_core R(2,2)
%       assembly minus the delay_R2_factor*Q77 contribution, which is zero
%       under force_Q77_zero=true).
%     * R22 empirical: sliding-window var(a_xm) with window=300 steps
%       (Hann/rectangular rectangular kernel for variance is fine; we
%       use plain centered windowed var via movvar).
%     * Steady-state averages taken on t > 1 s.
%
%   See: test_script/run_v2_h50_e2e.m for the structural template, and
%        test_script/learn_variance/verify_R22_intrinsic.m for the
%        R22 ratio formula.

    if nargin < 1 || isempty(opts); opts = struct(); end
    if ~isfield(opts, 'seeds');       opts.seeds       = [1, 2, 3, 4, 5]; end
    if ~isfield(opts, 'T_sim');       opts.T_sim       = 5;               end
    if ~isfield(opts, 'h_init');      opts.h_init      = 50;              end
    if ~isfield(opts, 't_ss');        opts.t_ss        = 1.0;             end  % steady-state cutoff [s]
    if ~isfield(opts, 'r22_window');  opts.r22_window  = 300;             end  % samples
    if ~isfield(opts, 'verbose');     opts.verbose     = true;            end

    % --- Path setup ---
    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(script_dir);
    addpath(fullfile(project_root, 'model'));
    addpath(fullfile(project_root, 'model', 'config'));
    addpath(fullfile(project_root, 'model', 'wall_effect'));
    addpath(fullfile(project_root, 'model', 'thermal_force'));
    addpath(fullfile(project_root, 'model', 'trajectory'));
    addpath(fullfile(project_root, 'model', 'controller'));
    addpath(fullfile(project_root, 'model', 'dual_track'));
    addpath(fullfile(project_root, 'model', 'diag'));

    out_dir = fullfile(project_root, 'test_results', 'q_unification_h50');
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end

    % --- Base config (positioning, h=50) ---
    config = user_config();
    config.h_init            = opts.h_init;
    config.h_bottom          = opts.h_init;          % no descent
    config.amplitude         = 0;                    % no oscillation
    config.trajectory_type   = 'positioning';
    config.T_sim             = opts.T_sim;
    config.ctrl_enable       = true;
    config.meas_noise_enable = true;
    config.thermal_enable    = true;
    config.lambda_c          = 0.7;
    config.a_pd              = 0.05;
    config.a_cov             = 0.05;
    config.sigma2_w_fD       = 0;
    config.meas_noise_std    = [0.00062; 0.000057; 0.00331];  % matches run_v2_h50_e2e
    config.controller_type   = 7;

    % --- Resolve params + a_nom per axis ---
    params = calc_simulation_params(config);
    P = params.Value;
    Ts        = P.common.Ts;
    gamma_N   = P.common.gamma_N;
    R_radius  = P.common.R;
    w_hat     = P.wall.w_hat;
    pz_wall   = P.wall.pz;
    p0        = P.common.p0;

    a_nom_freespace = Ts / gamma_N;                          % [um/pN]
    h_init_um   = dot(p0, w_hat) - pz_wall;
    h_bar_init  = max(h_init_um / R_radius, 1.001);
    [c_para0, c_perp0] = calc_correction_functions(h_bar_init);
    a_nom_per_axis = [a_nom_freespace / c_para0, ...
                      a_nom_freespace / c_para0, ...
                      a_nom_freespace / c_perp0];            % 1x3 [x,y,z]

    n_seeds = numel(opts.seeds);
    n_warmup = round(opts.t_ss / Ts);

    if opts.verbose
        fprintf('[run_q_unification_h50] seeds=%s T_sim=%.2fs h_init=%g um\n', ...
                mat2str(opts.seeds), opts.T_sim, opts.h_init);
        fprintf('  Q66_OL_mode=%d  force_Q77_zero=%d  iir_warmup_mode=%s\n', ...
                config.Q66_OL_mode, config.force_Q77_zero, config.iir_warmup_mode);
        fprintf('  a_nom_per_axis (x,y,z) [um/pN] = [%.4e, %.4e, %.4e]\n', ...
                a_nom_per_axis);
    end

    % --- Allocate per-seed storage ---
    %   metric arrays: rows=seed, cols=axis [x,y,z]
    tracking_rms      = zeros(n_seeds, 3);
    a_hat_norm_bias   = zeros(n_seeds, 3);
    a_hat_norm_std    = zeros(n_seeds, 3);
    P_a_ss            = zeros(n_seeds, 3);
    P_dx_ss           = zeros(n_seeds, 3);
    R22_ratio_ss      = zeros(n_seeds, 3);

    % --- Time-series storage (for plotting) ---
    %   We need: per-axis del_p, a_hat/a_true, R22 ratio, sigma2_dxr_hat
    N_total = round(opts.T_sim / Ts) + 1;
    tout_master  = (0:N_total-1)' * Ts;
    track_err_ts = nan(N_total, 3, n_seeds);   % p_d - p_m   [um]
    a_hat_ratio_ts = nan(N_total, 3, n_seeds); % a_hat / a_true
    R22_ratio_ts   = nan(N_total, 3, n_seeds); % pred / emp
    sigma2_dxr_ts  = nan(N_total, 3, n_seeds);

    % --- Seed loop ---
    for s = 1:n_seeds
        seed = opts.seeds(s);
        run_opts.seed         = seed;
        run_opts.verbose      = false;
        run_opts.collect_diag = true;

        if opts.verbose
            fprintf('  seed %d/%d (=%d) ... ', s, n_seeds, seed);
            t_start = tic;
        end

        simOut = run_pure_simulation(config, run_opts);

        if opts.verbose
            fprintf('done (%.1fs)\n', toc(t_start));
        end

        % --- Sanity print of constants on first seed only ---
        if s == 1 && opts.verbose
            cc = simOut.ctrl_const;
            fprintf('  ctrl_const.R22_prefactor   = %.6f\n', cc.R22_prefactor);
            fprintf('  ctrl_const.IF_eff_per_axis = [%.4f, %.4f, %.4f]\n', ...
                    cc.IF_eff_per_axis(1), cc.IF_eff_per_axis(2), cc.IF_eff_per_axis(3));
            fprintf('  ctrl_const.xi_per_axis     = [%.3e, %.3e, %.3e]\n', ...
                    cc.xi_per_axis(1), cc.xi_per_axis(2), cc.xi_per_axis(3));
            fprintf('  ctrl_const.force_Q77_zero  = %d\n', cc.force_Q77_zero);
            fprintf('  ctrl_const.Q66_OL_mode     = %d\n', cc.Q66_OL_mode);
        end

        % --- Extract arrays ---
        tout = simOut.tout;
        N    = numel(tout);
        del_p = simOut.p_d_out - simOut.p_m_out;                  % [N x 3, um]

        % a_hat order in ekf_out: col 1=x, col 2=z, col 3=y. Reorder -> [x,y,z]
        a_hat_xyz = [simOut.ekf_out(:, 1), simOut.ekf_out(:, 3), simOut.ekf_out(:, 2)];

        % a_xm and sigma2_dxr_hat in diag come per axis in [x,y,z] order
        % (diag_k.a_xm(:) packs axes 1..3 i.e. [x,y,z] per controller convention).
        a_xm           = simOut.diag.a_xm;             % [N x 3]
        sigma2_dxr_hat = simOut.diag.sigma2_dxr_hat;   % [N x 3]
        P_a_log        = simOut.diag.P_a;              % [N x 3]
        P_dx_log       = simOut.diag.P_dx;             % [N x 3]

        % --- Predicted R22 (intrinsic, time-series) ---
        cc = simOut.ctrl_const;
        % R22_pred(t, ax) = R22_prefactor * IF_eff_per_axis(ax) * (a_hat(t, ax) + xi(ax))^2
        R22_pred = (cc.R22_prefactor .* cc.IF_eff_per_axis(:).') ...
                   .* (a_hat_xyz + cc.xi_per_axis(:).').^2;
        % NOTE: this is R22 in the a_xm-space (matches verify_R22_intrinsic.m
        % 'a_xm-space' branch).  Empirical = sliding-window var of a_xm.

        % --- Empirical R22 (sliding-window var of a_xm) ---
        % movvar(x, w) returns variance over a window of w samples
        % (centered, edges shrink). We bound w_eff = opts.r22_window.
        R22_emp = movvar(a_xm, opts.r22_window, 0, 1);   % [N x 3]
        % Guard against div-by-zero in early steps where window is partial
        R22_emp(R22_emp <= 0) = NaN;
        R22_ratio = R22_pred ./ R22_emp;                 % [N x 3]

        % --- Steady-state windows (t > t_ss) ---
        idx_ss = (tout >= opts.t_ss);

        % Tracking RMS per axis
        tracking_rms(s, :) = sqrt(mean(del_p(idx_ss, :).^2, 1));

        % a_hat normalized bias and std
        a_ratio = a_hat_xyz ./ a_nom_per_axis;
        a_hat_norm_bias(s, :) = mean(a_ratio(idx_ss, :), 1) - 1;
        a_hat_norm_std(s, :)  = std(a_ratio(idx_ss, :),  0, 1);

        % P_a / P_dx steady-state
        P_a_ss(s, :)  = mean(P_a_log(idx_ss, :),  1);
        P_dx_ss(s, :) = mean(P_dx_log(idx_ss, :), 1);

        % R22 ratio steady-state (drop NaNs)
        for ax = 1:3
            r = R22_ratio(idx_ss, ax);
            r = r(isfinite(r));
            if isempty(r)
                R22_ratio_ss(s, ax) = NaN;
            else
                R22_ratio_ss(s, ax) = mean(r);
            end
        end

        % --- Store time-series for plotting ---
        track_err_ts(1:N, :, s)   = del_p;
        a_hat_ratio_ts(1:N, :, s) = a_ratio;
        R22_ratio_ts(1:N, :, s)   = R22_ratio;
        sigma2_dxr_ts(1:N, :, s)  = sigma2_dxr_hat;
    end

    % --- Aggregate (mean and std across seeds) ---
    aggregate = struct();
    aggregate.tracking_rms_mean    = mean(tracking_rms, 1);
    aggregate.tracking_rms_std     = std(tracking_rms, 0, 1);
    aggregate.a_hat_norm_bias_mean = mean(a_hat_norm_bias, 1);
    aggregate.a_hat_norm_bias_std  = std(a_hat_norm_bias, 0, 1);
    aggregate.a_hat_norm_std_mean  = mean(a_hat_norm_std, 1);
    aggregate.a_hat_norm_std_std   = std(a_hat_norm_std, 0, 1);
    aggregate.P_a_ss_mean          = mean(P_a_ss, 1);
    aggregate.P_a_ss_std           = std(P_a_ss, 0, 1);
    aggregate.P_dx_ss_mean         = mean(P_dx_ss, 1);
    aggregate.P_dx_ss_std          = std(P_dx_ss, 0, 1);
    aggregate.R22_ratio_ss_mean    = mean(R22_ratio_ss, 1);
    aggregate.R22_ratio_ss_std     = std(R22_ratio_ss, 0, 1);

    % --- Pack results ---
    results.config            = config;
    results.opts              = opts;
    results.a_nom_per_axis    = a_nom_per_axis;
    results.metrics_per_seed  = struct( ...
        'tracking_rms',     tracking_rms, ...
        'a_hat_norm_bias',  a_hat_norm_bias, ...
        'a_hat_norm_std',   a_hat_norm_std, ...
        'P_a_ss',           P_a_ss, ...
        'P_dx_ss',          P_dx_ss, ...
        'R22_ratio_ss',     R22_ratio_ss );
    results.aggregate         = aggregate;
    results.meta = struct( ...
        'driver',         'run_q_unification_h50', ...
        'driver_version', '1.0', ...
        'timestamp',      char(datetime('now')), ...
        'project_root',   project_root );

    % --- Save .mat ---
    summary_path = fullfile(out_dir, 'summary.mat');
    save(summary_path, '-struct', 'results');
    if opts.verbose
        fprintf('Saved summary.mat -> %s\n', summary_path);
    end

    % --- Save summary.md ---
    write_summary_md(fullfile(out_dir, 'summary.md'), results, opts);

    % --- Plot figures ---
    plot_tracking_per_axis(tout_master, track_err_ts, opts, out_dir);
    plot_a_hat_per_axis(tout_master, a_hat_ratio_ts, opts, out_dir);
    plot_R22_ratio_per_axis(tout_master, R22_ratio_ts, opts, out_dir);

    if opts.verbose
        print_summary(results);
    end
end


function write_summary_md(md_path, results, opts)
    seeds = opts.seeds;
    agg = results.aggregate;

    fid = fopen(md_path, 'w');
    onCleanupObj = onCleanup(@() fclose(fid));

    fprintf(fid, '# Q-unification h=50 positioning (5-seed)\n\n');
    fprintf(fid, '- Generated: %s\n', results.meta.timestamp);
    fprintf(fid, '- Seeds: %s\n', mat2str(seeds));
    fprintf(fid, '- T_sim: %.2f s   steady-state window: t > %.2f s\n', ...
            opts.T_sim, opts.t_ss);
    fprintf(fid, '- h_init: %.1f um   trajectory: positioning (amplitude=0)\n', ...
            opts.h_init);
    fprintf(fid, '- Q66_OL_mode = %d, force_Q77_zero = %d, iir_warmup_mode = %s\n', ...
            results.config.Q66_OL_mode, results.config.force_Q77_zero, ...
            results.config.iir_warmup_mode);
    fprintf(fid, '- a_nom_per_axis [um/pN]: x=%.4e, y=%.4e, z=%.4e\n\n', ...
            results.a_nom_per_axis(1), results.a_nom_per_axis(2), results.a_nom_per_axis(3));

    fprintf(fid, '## Per-axis summary (mean +/- std across 5 seeds)\n\n');
    fprintf(fid, '| Metric | x | y | z |\n');
    fprintf(fid, '|--------|---|---|---|\n');
    fprintf(fid, '| Tracking RMS [nm]              | %s | %s | %s |\n', ...
            fmt_pm(agg.tracking_rms_mean(1)*1e3, agg.tracking_rms_std(1)*1e3), ...
            fmt_pm(agg.tracking_rms_mean(2)*1e3, agg.tracking_rms_std(2)*1e3), ...
            fmt_pm(agg.tracking_rms_mean(3)*1e3, agg.tracking_rms_std(3)*1e3));
    fprintf(fid, '| a_hat normalized bias [%%]      | %s | %s | %s |\n', ...
            fmt_pm(agg.a_hat_norm_bias_mean(1)*100, agg.a_hat_norm_bias_std(1)*100), ...
            fmt_pm(agg.a_hat_norm_bias_mean(2)*100, agg.a_hat_norm_bias_std(2)*100), ...
            fmt_pm(agg.a_hat_norm_bias_mean(3)*100, agg.a_hat_norm_bias_std(3)*100));
    fprintf(fid, '| std(a_hat / a_true)            | %s | %s | %s |\n', ...
            fmt_pm(agg.a_hat_norm_std_mean(1), agg.a_hat_norm_std_std(1)), ...
            fmt_pm(agg.a_hat_norm_std_mean(2), agg.a_hat_norm_std_std(2)), ...
            fmt_pm(agg.a_hat_norm_std_mean(3), agg.a_hat_norm_std_std(3)));
    fprintf(fid, '| R22 ratio (pred / emp)         | %s | %s | %s |\n', ...
            fmt_pm(agg.R22_ratio_ss_mean(1), agg.R22_ratio_ss_std(1)), ...
            fmt_pm(agg.R22_ratio_ss_mean(2), agg.R22_ratio_ss_std(2)), ...
            fmt_pm(agg.R22_ratio_ss_mean(3), agg.R22_ratio_ss_std(3)));
    fprintf(fid, '| P_a steady-state               | %s | %s | %s |\n', ...
            fmt_pm_sci(agg.P_a_ss_mean(1), agg.P_a_ss_std(1)), ...
            fmt_pm_sci(agg.P_a_ss_mean(2), agg.P_a_ss_std(2)), ...
            fmt_pm_sci(agg.P_a_ss_mean(3), agg.P_a_ss_std(3)));
    fprintf(fid, '| P_dx steady-state              | %s | %s | %s |\n\n', ...
            fmt_pm_sci(agg.P_dx_ss_mean(1), agg.P_dx_ss_std(1)), ...
            fmt_pm_sci(agg.P_dx_ss_mean(2), agg.P_dx_ss_std(2)), ...
            fmt_pm_sci(agg.P_dx_ss_mean(3), agg.P_dx_ss_std(3)));

    fprintf(fid, '## Per-seed raw values\n\n');
    fprintf(fid, '### Tracking RMS [nm]\n\n');
    fprintf(fid, '| seed | x | y | z |\n|------|---|---|---|\n');
    for s = 1:numel(seeds)
        fprintf(fid, '| %d | %.2f | %.2f | %.2f |\n', seeds(s), ...
                results.metrics_per_seed.tracking_rms(s, :)*1e3);
    end
    fprintf(fid, '\n### a_hat normalized bias [%%]\n\n');
    fprintf(fid, '| seed | x | y | z |\n|------|---|---|---|\n');
    for s = 1:numel(seeds)
        fprintf(fid, '| %d | %+.3f | %+.3f | %+.3f |\n', seeds(s), ...
                results.metrics_per_seed.a_hat_norm_bias(s, :)*100);
    end
    fprintf(fid, '\n### R22 ratio (pred / emp) steady-state\n\n');
    fprintf(fid, '| seed | x | y | z |\n|------|---|---|---|\n');
    for s = 1:numel(seeds)
        fprintf(fid, '| %d | %.3f | %.3f | %.3f |\n', seeds(s), ...
                results.metrics_per_seed.R22_ratio_ss(s, :));
    end

    fprintf(fid, '\n## Notes\n');
    fprintf(fid, '- R22 ratio computed as R22_predicted / R22_empirical.\n');
    fprintf(fid, '  predicted = R22_prefactor * IF_eff_per_axis * (a_hat + xi)^2.\n');
    fprintf(fid, '  empirical = movvar(a_xm, window=%d).\n', opts.r22_window);
    fprintf(fid, '- Target: R22 ratio ~ 1.0; a_hat bias < 5%%; tracking RMS sub-um.\n');
    %#ok<*PRTCAL>
end


function s = fmt_pm(m, sd)
    s = sprintf('%.3f +/- %.3f', m, sd);
end

function s = fmt_pm_sci(m, sd)
    s = sprintf('%.3e +/- %.2e', m, sd);
end


function plot_tracking_per_axis(t, track_err_ts, opts, out_dir)
    ax_names = {'x', 'y', 'z'};
    colors = lines(numel(opts.seeds));
    fig = figure('Position', [50 50 1500 800], 'Color', 'w', 'Visible', 'off');
    tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ax = 1:3
        nexttile;
        hold on;
        for s = 1:numel(opts.seeds)
            plot(t, track_err_ts(:, ax, s) * 1e3, ...
                 'Color', [colors(s,:), 0.6], 'LineWidth', 0.9);
        end
        xline(opts.t_ss, 'k--', 'LineWidth', 1.2, 'Alpha', 0.5);
        grid on; box on;
        set(gca, 'FontSize', 12);
        ylabel(sprintf('e_%s [nm]', ax_names{ax}), 'FontSize', 13);
        if ax == 1
            title(sprintf('Tracking error e = p_d - p_m  (h_{init}=%g um, 5 seeds)', ...
                          opts.h_init), 'FontSize', 14);
        end
        if ax == 3
            xlabel('Time [s]', 'FontSize', 13);
        end
        xlim([0 opts.T_sim]);
        legend(arrayfun(@(s) sprintf('seed %d', s), opts.seeds, ...
                        'UniformOutput', false), ...
               'Location', 'eastoutside', 'FontSize', 9);
    end
    out_path = fullfile(out_dir, 'tracking_per_axis.png');
    exportgraphics(fig, out_path, 'Resolution', 150);
    close(fig);
    fprintf('Saved %s\n', out_path);
end


function plot_a_hat_per_axis(t, a_hat_ratio_ts, opts, out_dir)
    ax_names = {'x', 'y', 'z'};
    colors = lines(numel(opts.seeds));
    fig = figure('Position', [50 50 1500 800], 'Color', 'w', 'Visible', 'off');
    tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ax = 1:3
        nexttile;
        hold on;
        for s = 1:numel(opts.seeds)
            plot(t, a_hat_ratio_ts(:, ax, s), ...
                 'Color', [colors(s,:), 0.6], 'LineWidth', 0.9);
        end
        yline(1.0, 'k-', 'LineWidth', 1.3, 'Alpha', 0.7);
        yline(1.05, 'k--', 'LineWidth', 1.0, 'Alpha', 0.4);
        yline(0.95, 'k--', 'LineWidth', 1.0, 'Alpha', 0.4);
        xline(opts.t_ss, 'k--', 'LineWidth', 1.0, 'Alpha', 0.5);
        grid on; box on;
        set(gca, 'FontSize', 12);
        ylabel(sprintf('a_{hat,%s} / a_{true,%s}', ax_names{ax}, ax_names{ax}), ...
               'FontSize', 13);
        if ax == 1
            title(sprintf('a_{hat} / a_{true} per axis  (h_{init}=%g um, 5 seeds)', ...
                          opts.h_init), 'FontSize', 14);
        end
        if ax == 3
            xlabel('Time [s]', 'FontSize', 13);
        end
        xlim([0 opts.T_sim]);
        ylim([0.7 1.3]);
        legend(arrayfun(@(s) sprintf('seed %d', s), opts.seeds, ...
                        'UniformOutput', false), ...
               'Location', 'eastoutside', 'FontSize', 9);
    end
    out_path = fullfile(out_dir, 'a_hat_per_axis.png');
    exportgraphics(fig, out_path, 'Resolution', 150);
    close(fig);
    fprintf('Saved %s\n', out_path);
end


function plot_R22_ratio_per_axis(t, R22_ratio_ts, opts, out_dir)
    ax_names = {'x', 'y', 'z'};
    colors = lines(numel(opts.seeds));
    fig = figure('Position', [50 50 1500 800], 'Color', 'w', 'Visible', 'off');
    tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ax = 1:3
        nexttile;
        hold on;
        for s = 1:numel(opts.seeds)
            plot(t, R22_ratio_ts(:, ax, s), ...
                 'Color', [colors(s,:), 0.6], 'LineWidth', 0.9);
        end
        yline(1.0, 'k-', 'LineWidth', 1.3, 'Alpha', 0.7);
        yline(1.2, 'k--', 'LineWidth', 1.0, 'Alpha', 0.4);
        yline(0.8, 'k--', 'LineWidth', 1.0, 'Alpha', 0.4);
        xline(opts.t_ss, 'k--', 'LineWidth', 1.0, 'Alpha', 0.5);
        grid on; box on;
        set(gca, 'FontSize', 12);
        ylabel(sprintf('R_{22,%s}^{pred} / R_{22,%s}^{emp}', ...
                       ax_names{ax}, ax_names{ax}), 'FontSize', 13);
        if ax == 1
            title(sprintf('R_{22} predicted / empirical (window=%d steps), h_{init}=%g um', ...
                          opts.r22_window, opts.h_init), 'FontSize', 14);
        end
        if ax == 3
            xlabel('Time [s]', 'FontSize', 13);
        end
        xlim([0 opts.T_sim]);
        ylim([0 3]);
        legend(arrayfun(@(s) sprintf('seed %d', s), opts.seeds, ...
                        'UniformOutput', false), ...
               'Location', 'eastoutside', 'FontSize', 9);
    end
    out_path = fullfile(out_dir, 'R22_ratio_per_axis.png');
    exportgraphics(fig, out_path, 'Resolution', 150);
    close(fig);
    fprintf('Saved %s\n', out_path);
end


function print_summary(results)
    agg = results.aggregate;
    fprintf('\n=== Q-unification h=50 5-seed positioning ===\n');
    fprintf('Tracking RMS [nm] (mean +/- std across seeds):\n');
    fprintf('  x: %.2f +/- %.2f\n', agg.tracking_rms_mean(1)*1e3, ...
                                     agg.tracking_rms_std(1)*1e3);
    fprintf('  y: %.2f +/- %.2f\n', agg.tracking_rms_mean(2)*1e3, ...
                                     agg.tracking_rms_std(2)*1e3);
    fprintf('  z: %.2f +/- %.2f\n', agg.tracking_rms_mean(3)*1e3, ...
                                     agg.tracking_rms_std(3)*1e3);
    fprintf('a_hat normalized bias [%%]:\n');
    fprintf('  x: %+.3f +/- %.3f\n', agg.a_hat_norm_bias_mean(1)*100, ...
                                     agg.a_hat_norm_bias_std(1)*100);
    fprintf('  y: %+.3f +/- %.3f\n', agg.a_hat_norm_bias_mean(2)*100, ...
                                     agg.a_hat_norm_bias_std(2)*100);
    fprintf('  z: %+.3f +/- %.3f\n', agg.a_hat_norm_bias_mean(3)*100, ...
                                     agg.a_hat_norm_bias_std(3)*100);
    fprintf('std(a_hat / a_true):\n');
    fprintf('  x: %.4f +/- %.4f\n', agg.a_hat_norm_std_mean(1), agg.a_hat_norm_std_std(1));
    fprintf('  y: %.4f +/- %.4f\n', agg.a_hat_norm_std_mean(2), agg.a_hat_norm_std_std(2));
    fprintf('  z: %.4f +/- %.4f\n', agg.a_hat_norm_std_mean(3), agg.a_hat_norm_std_std(3));
    fprintf('R22 ratio (pred/emp), steady-state:\n');
    fprintf('  x: %.3f +/- %.3f\n', agg.R22_ratio_ss_mean(1), agg.R22_ratio_ss_std(1));
    fprintf('  y: %.3f +/- %.3f\n', agg.R22_ratio_ss_mean(2), agg.R22_ratio_ss_std(2));
    fprintf('  z: %.3f +/- %.3f\n', agg.R22_ratio_ss_mean(3), agg.R22_ratio_ss_std(3));
    fprintf('P_a steady-state:\n');
    fprintf('  x: %.3e +/- %.2e\n', agg.P_a_ss_mean(1), agg.P_a_ss_std(1));
    fprintf('  y: %.3e +/- %.2e\n', agg.P_a_ss_mean(2), agg.P_a_ss_std(2));
    fprintf('  z: %.3e +/- %.2e\n', agg.P_a_ss_mean(3), agg.P_a_ss_std(3));
    fprintf('P_dx steady-state:\n');
    fprintf('  x: %.3e +/- %.2e\n', agg.P_dx_ss_mean(1), agg.P_dx_ss_std(1));
    fprintf('  y: %.3e +/- %.2e\n', agg.P_dx_ss_mean(2), agg.P_dx_ss_std(2));
    fprintf('  z: %.3e +/- %.2e\n', agg.P_dx_ss_mean(3), agg.P_dx_ss_std(3));
    fprintf('=== End ===\n');
end
