function results = compare_xD_estimation_6state(scenarios, opts)
%COMPARE_XD_ESTIMATION_6STATE  6-state (full disturbance estimation) vs
%   5-state (disturbance state removed) controller comparison.
%
%   results = compare_xD_estimation_6state()
%   results = compare_xD_estimation_6state(scenarios, opts)
%
%   Answers "what happens to the control result if the lumped-disturbance
%   estimate is removed?" by running the same scenario with two controllers:
%     - '6state' (suppress_xD=false): EKF estimates delta_x_D^d AND the
%       control law subtracts it (full Vpersonal form).
%     - '5state': the disturbance state is removed from the EKF entirely
%       (F_e Row-3 loses the -1 coupling, H/Q/Pf shrink to 5-dim, control
%       law has no -x_hat_D term).
%
%   Both forms run on the SAME seeds. Per scenario it writes the
%   eq17_6state_h50-style figure pair (overlaid for the two forms):
%       fig1_gain_estimation.png  - a_x, a_z : True / 6-state a_hat / 5-state a_hat
%       fig2_tracking_error.png   - per-axis error 6-state vs 5-state
%                                    + 6-state delta_x_D^d on right axis
%   plus summary.md / summary.mat, into
%       test_results/eq17_6v5_compare/<scenario>/
%
%   Scenarios (default both):
%     'h50'    : positioning, hold at h=50 um (amplitude=0). 5 seeds x 5 s.
%     'osc1hz' : 1 Hz oscillation centred at h=50 um (h_bottom=47.5,
%                amplitude=2.5 -> h in [47.5,52.5], far from wall, gate-free).
%                5 seeds x 6 s.
%
%   opts (all optional): seeds, save_fig (true), verbose (true).
%
%   See also: motion_control_law_eq17_5state, motion_control_law_eq17_6state,
%             verify_eq17_6state, run_pure_simulation

    if nargin < 1 || isempty(scenarios); scenarios = {'h50', 'osc1hz'}; end
    if ischar(scenarios) || isstring(scenarios); scenarios = cellstr(scenarios); end
    if nargin < 2 || isempty(opts); opts = struct(); end
    if ~isfield(opts, 'seeds');    opts.seeds = 1:5;     end
    if ~isfield(opts, 'save_fig'); opts.save_fig = true; end
    if ~isfield(opts, 'verbose');  opts.verbose = true;  end

    % --- paths ---
    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(fileparts(script_dir));
    addpath(fullfile(project_root, 'model'), fullfile(project_root, 'model', 'config'), ...
            fullfile(project_root, 'model', 'wall_effect'), ...
            fullfile(project_root, 'model', 'thermal_force'), ...
            fullfile(project_root, 'model', 'trajectory'), ...
            fullfile(project_root, 'model', 'controller'), ...
            fullfile(project_root, 'model', 'dual_track'), script_dir);

    results = struct('scenario', {}, 'agg6', {}, 'agg5', {}, 'out_dir', {});
    for si = 1:numel(scenarios)
        scen = lower(scenarios{si});
        sc   = scenario_defaults(scen);
        cfg  = build_config(sc, opts);

        params = calc_simulation_params(cfg); P = params.Value;
        Ts = P.common.Ts; gamma_N = P.common.gamma_N; R_radius = P.common.R;
        w_hat = P.wall.w_hat; pz_wall = P.wall.pz; a_nom = Ts / gamma_N;
        n_warmup = round(sc.t_warmup / Ts);

        % pre-flight: desired trajectory respects h_min override
        [is_safe, h_min_actual, t_crit] = check_trajectory_safety(P);
        assert(is_safe, 'compare_xD:%s trajectory unsafe: h_min=%.3f um @ t=%.2f s', ...
               scen, h_min_actual, t_crit);

        if opts.verbose
            fprintf('[compare_xD:%s] type=%s h=%g amp=%g f=%g T=%.1fs seeds=%s\n', ...
                    scen, sc.traj_type, sc.h_init, cfg.amplitude, cfg.frequency, ...
                    cfg.T_sim, mat2str(opts.seeds));
        end

        % --- run both forms on the same seeds ---
        [agg6, samp6] = run_form('6state', cfg, opts.seeds, n_warmup, ...
                                 w_hat, pz_wall, R_radius, a_nom, cfg.a_pd);
        [agg5, samp5] = run_form('5state', cfg, opts.seeds, n_warmup, ...
                                 w_hat, pz_wall, R_radius, a_nom, cfg.a_pd);
        if agg6.n_div > 0 || agg5.n_div > 0
            fprintf('  WARNING: diverged runs  6st=%d  5st=%d (excluded from stats)\n', ...
                    agg6.n_div, agg5.n_div);
        end

        % --- output ---
        out_dir = fullfile(project_root, 'test_results', 'eq17_6v5_compare', scen);
        if ~exist(out_dir, 'dir'); mkdir(out_dir); end
        save(fullfile(out_dir, 'summary.mat'), 'agg6', 'agg5', 'sc', 'cfg', 'opts');
        write_summary_md(fullfile(out_dir, 'summary.md'), scen, sc, opts, agg6, agg5);
        print_report(scen, sc, opts, agg6, agg5);
        if opts.save_fig
            meta = struct('name', scen, 'kind', sc.kind);
            make_compare_figs(samp6, samp5, agg6, agg5, meta, out_dir);
        end
        if opts.verbose; fprintf('  saved %s\n', out_dir); end

        results(end+1) = struct('scenario', scen, 'agg6', agg6, 'agg5', agg5, ...
                                'out_dir', out_dir); %#ok<AGROW>
    end
end


% ====================================================================
function sc = scenario_defaults(scen)
    switch scen
        case 'h50'
            sc = struct('traj_type', 'positioning', 'kind', 'positioning', ...
                        'h_init', 50, 'h_bottom', 50, 'amplitude', 0, ...
                        'frequency', 1, 'n_cycles', 0, 't_hold', 0.5, 't_descend', 0, ...
                        'T_sim', 5, 't_warmup', 0.5, 'h_bar_safe', 1.5, 'h_min_factor', 1.5);
        case 'osc1hz'
            % Near-wall 1 Hz descending oscillation (= compare_gain osc scenario):
            % hold@50 -> cosine descent (1 s) -> 2 cycles between trough h_bottom
            % (h_bar=1.2) and peak h_bottom+2*amplitude=7.7 um (h_bar=3.42) ->
            % hold@trough. a_true varies ~3x across the cycle (gain tracking is
            % meaningful here). Gate-free (h_bar_safe=1) keeps y_2 active through
            % the trough. Plot/eval window = oscillation onward (t >= t_warmup).
            sc = struct('traj_type', 'osc', 'kind', 'osc', ...
                        'h_init', 50, 'h_bottom', 2.7, 'amplitude', 2.5, ...
                        'frequency', 1, 'n_cycles', 2, 't_hold', 0.5, 't_descend', 1.0, ...
                        'T_sim', 4, 't_warmup', 1.5, 'h_bar_safe', 1, 'h_min_factor', 1.05);
        otherwise
            error('compare_xD_estimation_6state:badScenario', ...
                  'Unknown scenario "%s" (use h50 | osc1hz).', scen);
    end
end


function cfg = build_config(sc, opts) %#ok<INUSD>
    cfg = user_config();
    cfg.h_init          = sc.h_init;
    cfg.h_bottom        = sc.h_bottom;
    cfg.amplitude       = sc.amplitude;
    cfg.frequency       = sc.frequency;
    cfg.n_cycles        = sc.n_cycles;
    cfg.trajectory_type = sc.traj_type;
    cfg.T_sim           = sc.T_sim;
    cfg.t_hold          = sc.t_hold;
    if sc.t_descend > 0
        cfg.t_descend_override = sc.t_descend;   % decouple descent from 1/f
    end
    cfg.ctrl_enable     = true;
    cfg.meas_noise_enable = true;
    cfg.thermal_enable  = true;
    cfg.lambda_c        = 0.7;
    cfg.a_pd            = 0.05;
    cfg.a_cov           = 0.05;
    cfg.meas_noise_std  = [0.00062; 0.00057; 0.00331];   % [um] per-axis sensor std
    cfg.suppress_xD     = false;     % 6-state arm uses the disturbance estimate;
                                     % 5-state arm ignores this flag (no term).
    cfg.h_bar_safe      = sc.h_bar_safe;             % Guard-3 threshold (1 = gate-free here)
    pc = physical_constants();
    cfg.h_min           = sc.h_min_factor * pc.R;    % scenario-local h_min floor
end


function [agg, sample] = run_form(variant, cfg, seeds, n_warmup, w_hat, pz, R, a_nom, a_pd)
%RUN_FORM  Run one controller form across seeds; return aggregate + sample
%   (first non-diverged seed). Diverged noisy runs (crash / |aligned error| >
%   0.5 um) are counted and excluded from the statistics.
    cfg.eq17_variant = variant;
    ns = numel(seeds);
    trk = []; ah_m = []; ah_sd = []; at_m = []; xD_am = [];
    n_div = 0;
    sample = struct(); have_sample = false;
    for s = 1:ns
        ro = struct('seed', seeds(s), 'verbose', false, 'collect_diag', true);
        ok = true;
        try
            out = run_pure_simulation(cfg, ro);
        catch
            ok = false;
        end
        if ok
            e = out.p_d_out(2:end, :) - out.p_true_out(1:end-1, :);   % aligned physical error
            if ~all(isfinite(e(:))) || max(abs(e(:))) > 0.5
                ok = false;
            end
        end
        if ~ok
            n_div = n_div + 1;
            continue;
        end
        N = numel(out.tout); idx = (n_warmup + 1):N;
        trk(end+1, :)  = std(out.p_d_out(idx, :) - out.p_m_out(idx, :), 0, 1) * 1e3; %#ok<AGROW>
        a_true = local_a_true(out.p_true_out, w_hat, pz, R, a_nom);
        a_hat  = out.diag.a_hat;
        ah_m(end+1, :)  = mean(a_hat(idx, :), 1);   %#ok<AGROW>
        ah_sd(end+1, :) = std(a_hat(idx, :), 0, 1); %#ok<AGROW>
        at_m(end+1, :)  = mean(a_true(idx, :), 1);  %#ok<AGROW>
        xD_am(end+1, :) = mean(abs(out.diag.x_D_hat(idx, :)), 1); %#ok<AGROW>
        if ~have_sample
            sample = build_sample(out, idx, a_true, a_pd, w_hat, pz);
            have_sample = true;
        end
    end

    agg.trk_mean      = mean(trk, 1);
    agg.a_hat_mean    = mean(ah_m, 1);
    agg.a_true_mean   = mean(at_m, 1);
    agg.a_hat_bias    = (agg.a_hat_mean - agg.a_true_mean) ./ agg.a_true_mean * 100;
    agg.a_hat_relstd  = mean(ah_sd, 1) ./ agg.a_hat_mean * 100;
    agg.xD_absmean    = mean(xD_am, 1);
    agg.n_div         = n_div;
    agg.n_ok          = size(trk, 1);
    agg.variant       = variant;
end


function a_true = local_a_true(p_true, w_hat, pz, R, a_nom)
    N = size(p_true, 1);
    h_true = (p_true * w_hat - pz) / R;
    a_para = zeros(N, 1); a_perp = zeros(N, 1);
    for k = 1:N
        hb = max(h_true(k), 1.001);
        [cpa, cpe] = calc_correction_functions(hb);
        a_para(k) = a_nom / cpa;
        a_perp(k) = a_nom / cpe;
    end
    a_true = [a_para, a_para, a_perp];
end


function sample = build_sample(out, idx, a_true, a_pd, w_hat, pz)
    N = numel(out.tout);
    p_d = out.p_d_out; p_m = out.p_m_out;
    err = (p_m - p_d) * 1e3;                     % nm

    d = 2;
    del_pm  = zeros(N, 3);
    for k = d+1:N, del_pm(k, :) = p_d(k-d, :) - p_m(k, :); end
    del_pmd = zeros(N, 3);
    for k = 2:N, del_pmd(k, :) = (1 - a_pd) * del_pmd(k-1, :) + a_pd * del_pm(k, :); end
    del_pmd_plot = -del_pmd * 1e3;               % nm, sign-matched to err

    h_d    = out.p_d_out * w_hat - pz;           % desired height along wall normal [um]
    h_true = out.p_true_out * w_hat - pz;        % actual (noise-free) height [um]

    logical_idx = false(N, 1); logical_idx(idx) = true;
    sample = struct('t', out.tout(:), 'idx', logical_idx, ...
                    'a_xm', out.diag.a_xm, 'a_hat', out.diag.a_hat, 'a_true', a_true, ...
                    'err', err, 'del_pmd', del_pmd_plot, 'xD_nm', out.diag.x_D_hat * 1e3, ...
                    'h_d', h_d, 'h_true', h_true);
end


function print_report(scen, sc, opts, agg6, agg5)
    fprintf('\n===== compare_xD_estimation_6state:%s (%d seeds, %.0fs, %s) =====\n', ...
            scen, numel(opts.seeds), sc.T_sim, sc.traj_type);
    fprintf('seeds OK (non-diverged)  6st: %d/%d   5st: %d/%d\n', ...
            agg6.n_ok, numel(opts.seeds), agg5.n_ok, numel(opts.seeds));
    fprintf('                          x        y        z\n');
    fprintf('tracking std [nm]  6st: %7.2f %7.2f %7.2f\n', agg6.trk_mean);
    fprintf('                   5st: %7.2f %7.2f %7.2f\n', agg5.trk_mean);
    fprintf('a_hat bias [%%]     6st: %7.2f %7.2f %7.2f\n', agg6.a_hat_bias);
    fprintf('                   5st: %7.2f %7.2f %7.2f\n', agg5.a_hat_bias);
    fprintf('a_hat rel-std [%%]  6st: %7.2f %7.2f %7.2f\n', agg6.a_hat_relstd);
    fprintf('                   5st: %7.2f %7.2f %7.2f\n', agg5.a_hat_relstd);
    fprintf('|delta_x_D^d| [um] 6st: %7.2e %7.2e %7.2e  (5st: none)\n', agg6.xD_absmean);
    fprintf('---------------------------------------------------------------\n');
end


function write_summary_md(path, scen, sc, opts, agg6, agg5)
    fid = fopen(path, 'w');
    fprintf(fid, '# compare_xD_estimation_6state : %s\n\n', scen);
    fprintf(fid, '6-state (full disturbance estimation, suppress_xD=false) vs 5-state ');
    fprintf(fid, '(disturbance state removed), %s (h=%g, amp=%g, f=%g), %d seeds x %.0fs.\n\n', ...
            sc.traj_type, sc.h_init, sc.amplitude, sc.frequency, numel(opts.seeds), sc.T_sim);
    fprintf(fid, '| metric | form | x | y | z |\n|---|---|---|---|---|\n');
    fprintf(fid, '| tracking std (p_d-p_m) [nm] | 6-state | %.2f | %.2f | %.2f |\n', agg6.trk_mean);
    fprintf(fid, '| | 5-state | %.2f | %.2f | %.2f |\n', agg5.trk_mean);
    fprintf(fid, '| a_hat bias vs a_true [%%] | 6-state | %.2f | %.2f | %.2f |\n', agg6.a_hat_bias);
    fprintf(fid, '| | 5-state | %.2f | %.2f | %.2f |\n', agg5.a_hat_bias);
    fprintf(fid, '| a_hat rel-std [%%] | 6-state | %.2f | %.2f | %.2f |\n', agg6.a_hat_relstd);
    fprintf(fid, '| | 5-state | %.2f | %.2f | %.2f |\n', agg5.a_hat_relstd);
    fprintf(fid, '| \\|delta_x_D^d\\| mean [um] | 6-state | %.2e | %.2e | %.2e |\n', agg6.xD_absmean);
    fprintf(fid, '| | 5-state | (no disturbance state) | | |\n');
    fprintf(fid, '\na_hat mean 6st [x y z] = [%.4g %.4g %.4g], 5st = [%.4g %.4g %.4g], a_true = [%.4g %.4g %.4g] um/pN.\n', ...
            agg6.a_hat_mean, agg5.a_hat_mean, agg6.a_true_mean);
    fclose(fid);
end


% ====================================================================
function make_compare_figs(sig6, sig5, agg6, agg5, meta, out_dir)
%MAKE_COMPARE_FIGS  Overlaid 6-state vs 5-state figures (EXP/thesis style).
%   Plotted curves are the first non-diverged seed; the bias/std/tracking
%   numbers in the titles are the N-seed AGGREGATE (agg6/agg5), so they match
%   summary.md (single-seed numbers wander run-to-run via a_hat LF drift).
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end

    COL_REF = [0 0.6 0];        % green : True / Desired
    COL_6   = [0.8 0 0];        % red   : 6-state estimate / error
    COL_5   = [0.3 0.3 0.85];   % blue  : 5-state estimate / error
    COL_DST = [0.85 0.45 0];    % orange: 6-state delta_x_D^d
    FS = 18; LFS = 13; LW = 2.0; LR = 3.0; LO = 2.0;

    t = sig6.t;
    t0 = 0; t1 = max(t);                 % full time range (hold -> descent -> osc)
    is_osc = isfield(meta, 'kind') && strcmpi(meta.kind, 'osc');
    gain_cols = [1 3]; gain_lbl = {'a_x', 'a_z'};

    % unified gain y-range across BOTH gain axes (a_x, a_z) and both forms
    gv = [sig6.a_true(:, 1); sig6.a_true(:, 3); ...
          sig6.a_hat(:, 1);  sig6.a_hat(:, 3); ...
          sig5.a_hat(:, 1);  sig5.a_hat(:, 3)];
    gv = gv(isfinite(gv));
    gylim = [max(0, min(gv) * 0.9), max(gv) * 1.05];

    % ============== FIG 1 : gain estimation (a_x, a_z) ==============
    f1 = figure('Position', [80 80 1100 720], 'Color', 'w', ...
                'Name', sprintf('6v5 %s : gain', meta.name), 'NumberTitle', 'off');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for r = 1:2
        c = gain_cols(r);
        nexttile; hold on;
        plot(t, sig6.a_true(:, c), '-', 'Color', COL_REF, 'LineWidth', LR, 'DisplayName', 'True');
        plot(t, sig6.a_hat(:, c),  '-', 'Color', COL_6,   'LineWidth', LO, 'DisplayName', '6-state $\hat{a}$');
        plot(t, sig5.a_hat(:, c),  '--','Color', COL_5,   'LineWidth', LO, 'DisplayName', '5-state $\hat{a}$');

        if is_osc
            % 1 Hz: no analysis std/bias numbers (user request)
            title(gain_lbl{r}, 'FontSize', FS - 1, 'FontWeight', 'bold');
        else
            title(sprintf('%s:   6st bias %+.2f%% std %.2f%%    |    5st bias %+.2f%% std %.2f%%', ...
                  gain_lbl{r}, agg6.a_hat_bias(c), agg6.a_hat_relstd(c), ...
                  agg5.a_hat_bias(c), agg5.a_hat_relstd(c)), ...
                  'FontSize', FS - 4, 'FontWeight', 'bold');
        end

        ylabel(sprintf('%s  (\\mum/pN)', gain_lbl{r}), 'FontSize', FS, 'FontWeight', 'bold');
        ylim(gylim);                                  % unified across a_x, a_z
        if r == 2, xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
        if r == 1
            legend('Location', 'northoutside', 'Orientation', 'horizontal', ...
                   'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'off', 'Interpreter', 'latex');
        end
        xlim([t0 t1]);
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', LW, 'Box', 'on'); grid off;
        ax = gca; ax.YAxis.Exponent = 0; ytickformat(ax, '%.3f');   % decimal ticks (no x10^-3)
    end
    disable_toolbars(f1);
    exportgraphics(f1, fullfile(out_dir, 'fig1_gain_estimation.png'), 'Resolution', 150);

    % ============== FIG 2 : tracking error + disturbance (3x1) ==============
    f2 = figure('Position', [80 80 1100 920], 'Color', 'w', ...
                'Name', sprintf('6v5 %s : tracking', meta.name), 'NumberTitle', 'off');
    tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ev = [sig6.err(:); sig5.err(:)]; ev = ev(isfinite(ev));
    ymax = max(abs(ev));                                 % full time range
    axn  = {'x', 'y', 'z'};
    for k = 1:3
        nexttile;
        yyaxis left; hold on;
        h6 = plot(t, sig6.err(:, k), '-',  'Color', COL_6, 'LineWidth', 0.7, 'DisplayName', '6-state error');
        h5 = plot(t, sig5.err(:, k), '-',  'Color', COL_5, 'LineWidth', 0.7, 'DisplayName', '5-state error');
        ylabel(sprintf('%s error (nm)', axn{k}), 'FontSize', FS, 'FontWeight', 'bold');
        ylim([-ymax ymax] * 1.05); set(gca, 'YColor', 'k');

        yyaxis right;
        hd = plot(t, sig6.xD_nm(:, k), '-', 'Color', COL_DST, 'LineWidth', LO, 'DisplayName', '6-state \delta x_D^d');
        ylabel('\delta x_D^d (nm)', 'FontSize', FS - 2, 'FontWeight', 'bold'); set(gca, 'YColor', COL_DST);
        xdm = max(0.5, max(abs(sig6.xD_nm(:, k))));
        ylim([-xdm xdm] * 1.2);

        if is_osc
            % 1 Hz: no analysis std numbers (user request)
            title(axn{k}, 'FontSize', FS - 1, 'FontWeight', 'bold');
        else
            title(sprintf('%s:  6st std %.2f / 5st std %.2f nm   |6st \\delta x_D^d| %.2f nm', ...
                  axn{k}, agg6.trk_mean(k), agg5.trk_mean(k), ...
                  agg6.xD_absmean(k) * 1e3), 'FontSize', FS - 4, 'FontWeight', 'bold');
        end
        if k == 3, xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
        if k == 1
            legend([h6 h5 hd], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
                   'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'off');
        end
        xlim([t0 t1]);
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', LW, 'Box', 'on'); grid off;
    end
    disable_toolbars(f2);
    exportgraphics(f2, fullfile(out_dir, 'fig2_tracking_error.png'), 'Resolution', 150);
    close(f1); close(f2);

    % ===== FIG 3 : trajectory + gain overview (osc scenarios only) =====
    %   Makes the descent+oscillation visible (left axis, Desired height) and
    %   shows the gain it drives + the EKF tracking it (right axis). Full time
    %   range so hold -> descent -> oscillation is all shown.
    if isfield(meta, 'kind') && strcmpi(meta.kind, 'osc') && isfield(sig6, 'h_d')
        f3 = figure('Position', [80 80 1100 500], 'Color', 'w', ...
                    'Name', sprintf('6v5 %s : trajectory', meta.name), 'NumberTitle', 'off');
        hold on;
        yyaxis left;
        plot(t, sig6.h_d, '-', 'Color', COL_REF, 'LineWidth', LR, 'DisplayName', 'Desired z');
        ylabel('z  (\mum)', 'FontSize', FS, 'FontWeight', 'bold');
        ylim([0 max(sig6.h_d) * 1.15]); set(gca, 'YColor', 'k');

        yyaxis right;
        plot(t, sig6.a_true(:, 3), '-',  'Color', COL_6, 'LineWidth', LO, 'DisplayName', '$a_{\mathrm{true}}$ (z)');
        plot(t, sig6.a_hat(:, 3),  '--', 'Color', COL_5, 'LineWidth', LO, 'DisplayName', '$\hat{a}$ (z, 6-state)');
        ylabel('a_z  (\mum/pN)', 'FontSize', FS, 'FontWeight', 'bold'); set(gca, 'YColor', COL_5);
        ylim(gylim);                                          % same gain range as fig1
        gax = gca; gax.YAxis(2).Exponent = 0; ytickformat('%.3f');  % decimal ticks (right ruler active)

        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        title(sprintf('%s: descent 50 \\rightarrow %.1f \\mum then %g Hz osc (seed 1)', ...
              meta.name, min(sig6.h_d), 1), 'FontSize', FS - 3, 'FontWeight', 'bold');
        legend('Location', 'northoutside', 'Orientation', 'horizontal', ...
               'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'off', 'Interpreter', 'latex');
        xlim([0 max(t)]);
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', LW, 'Box', 'on'); grid off;
        disable_toolbars(f3);
        exportgraphics(f3, fullfile(out_dir, 'fig3_trajectory.png'), 'Resolution', 150);
        close(f3);
    end
end


function disable_toolbars(fig)
    axs = findall(fig, 'Type', 'Axes');
    for ii = 1:numel(axs)
        axs(ii).Toolbar.Visible = 'off';
    end
end
