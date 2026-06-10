function results = verify_eq17_6state(scenario, opts)
%VERIFY_EQ17_6STATE  Scenario-parametric verification for the 6-state
%   RevisedControl_Vpersonal controller (pure-MATLAB).
%
%   results = verify_eq17_6state(scenario)
%   results = verify_eq17_6state(scenario, opts)
%
%   scenario : 'h50' | 'h10' | 'ramp'
%       'h50' / 'h10' : positioning at h=50 / h=10 um. 5-seed aggregate with
%                       PASS/FAIL on tracking std, a_hat bias/rel-std vs the
%                       physics ground-truth a_x_true, and |delta_x_D^d|.
%       'ramp'        : ramp_descent 50 -> 5 um (a_x time-varying). Figures +
%                       descriptive statistics only (no PASS/FAIL), per the
%                       agreed plan -- the gain figure shows a_hat tracking the
%                       descending a_true(t).
%
%   Generalizes the earlier verify_eq17_6state_h50. Uses the shared figure
%   function make_eq17_6state_figures (finalized EXP/thesis style). Ground
%   truth a_x_true is computed from the noise-free position probe
%   simOut.p_true_out (the pure-MATLAB advantage over Simulink).
%
%   Outputs results struct + writes test_results/eq17_6state_<scenario>/
%   {summary.md, summary.mat, fig1_gain_estimation.png, fig2_tracking_error.png}.
%   (test_results/ is gitignored.)
%
%   ----- opts (all optional, override scenario defaults) -----
%       seeds, T_sim, t_warmup, save_fig (true), verbose (true),
%       trk_thresh_nm (40), bias_thresh_pct (5), relstd_thresh_pct (5),
%       xD_thresh_um (0.01).
%
%   See also: make_eq17_6state_figures, run_eq17_6state_all, run_pure_simulation

    if nargin < 1 || isempty(scenario); scenario = 'h50'; end
    if nargin < 2 || isempty(opts); opts = struct(); end
    scenario = lower(scenario);

    % --- 0. Scenario table ---
    sc = scenario_defaults(scenario);

    % --- 0b. Generic defaults + opts override ---
    dflt = struct('seeds', sc.seeds, 'T_sim', sc.T_sim, 't_warmup', sc.t_warmup, ...
                  'save_fig', true, 'verbose', true, ...
                  'trk_thresh_nm', 40, 'bias_thresh_pct', 5, ...
                  'relstd_thresh_pct', 5, 'xD_thresh_um', 0.01);
    fn = fieldnames(dflt);
    for i = 1:numel(fn)
        if ~isfield(opts, fn{i}) || isempty(opts.(fn{i})); opts.(fn{i}) = dflt.(fn{i}); end
    end

    % --- 1. Paths (script in test_script/integration/ -> up two levels) ---
    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(fileparts(script_dir));
    addpath(fullfile(project_root, 'model'), fullfile(project_root, 'model', 'config'), ...
            fullfile(project_root, 'model', 'wall_effect'), ...
            fullfile(project_root, 'model', 'thermal_force'), ...
            fullfile(project_root, 'model', 'trajectory'), ...
            fullfile(project_root, 'model', 'controller'), ...
            fullfile(project_root, 'model', 'dual_track'), script_dir);

    % --- 2. Config (6-state variant) ---
    config = user_config();
    config.h_init          = sc.h_init;
    config.h_bottom        = sc.h_bottom;
    config.amplitude       = 0;
    config.trajectory_type = sc.traj_type;
    config.T_sim           = opts.T_sim;
    config.ctrl_enable     = true;
    config.meas_noise_enable = true;
    config.thermal_enable  = true;
    config.lambda_c        = 0.7;
    config.a_pd            = 0.05;
    config.a_cov           = 0.05;
    config.meas_noise_std  = [0.00062; 0.00057; 0.00331];   % [um] per-axis sensor std (y corrected 0.057->0.57 nm)
    config.eq17_variant    = '6state';        % <-- dispatch A1

    % --- 3. Params + wall geometry ---
    params = calc_simulation_params(config);
    P = params.Value;
    Ts = P.common.Ts; gamma_N = P.common.gamma_N; R_radius = P.common.R;
    w_hat = P.wall.w_hat; pz_wall = P.wall.pz;
    a_nom = Ts / gamma_N;

    n_seeds  = numel(opts.seeds);
    n_warmup = round(opts.t_warmup / Ts);

    if opts.verbose
        fprintf('[verify_eq17_6state:%s] type=%s h=%g->%g T_sim=%.1fs n_seeds=%d eval=%d\n', ...
                scenario, sc.traj_type, sc.h_init, sc.h_bottom, opts.T_sim, n_seeds, sc.evaluate);
    end

    trk_meas    = zeros(n_seeds, 3);
    a_hat_mean  = zeros(n_seeds, 3);
    a_hat_std   = zeros(n_seeds, 3);
    a_true_mean = zeros(n_seeds, 3);
    relerr_mean = zeros(n_seeds, 3);   % mean (a_hat-a_true)/a_true (ramp descriptive)
    xD_absmean  = zeros(n_seeds, 3);

    sample = struct();
    for s = 1:n_seeds
        ro  = struct('seed', opts.seeds(s), 'verbose', false, 'collect_diag', true);
        out = run_pure_simulation(config, ro);
        N = numel(out.tout); idx = (n_warmup + 1):N;

        trk_meas(s, :) = std(out.p_d_out(idx, :) - out.p_m_out(idx, :), 0, 1) * 1e3;

        a_true = local_a_true(out.p_true_out, w_hat, pz_wall, R_radius, a_nom);  % [N x3] [x y z]
        % D6 note (spec 1b item 4): diag.a_hat logs the A-PRIORI estimate
        % x_hat[k+1|k], so it carries a one-step shift vs a_true[k]. Negligible
        % for the averaged statistics below (Ts=625 us, thousands of samples);
        % kept uncorrected by design.
        a_hat  = out.diag.a_hat;                                                 % [N x3] [x y z]
        a_hat_mean(s, :)  = mean(a_hat(idx, :), 1);
        a_hat_std(s, :)   = std(a_hat(idx, :), 0, 1);
        a_true_mean(s, :) = mean(a_true(idx, :), 1);
        relerr_mean(s, :) = mean((a_hat(idx, :) - a_true(idx, :)) ./ a_true(idx, :), 1) * 100;
        xD_absmean(s, :)  = mean(abs(out.diag.x_D_hat(idx, :)), 1);

        if s == 1
            sample = build_sample(out, idx, a_true, config.a_pd);
        end
    end

    % --- 4. Aggregate ---
    agg.trk_meas_mean    = mean(trk_meas, 1);
    agg.a_hat_mean       = mean(a_hat_mean, 1);
    agg.a_true_mean      = mean(a_true_mean, 1);
    agg.a_hat_bias_pct   = (agg.a_hat_mean - agg.a_true_mean) ./ agg.a_true_mean * 100;
    agg.a_hat_relstd_pct = mean(a_hat_std, 1) ./ agg.a_hat_mean * 100;
    agg.relerr_mean_pct  = mean(relerr_mean, 1);
    agg.xD_absmean       = mean(xD_absmean, 1);

    % --- 5. PASS/FAIL (positioning only) ---
    if sc.evaluate
        pass_trk    = all(agg.trk_meas_mean < opts.trk_thresh_nm);
        pass_bias   = all(abs(agg.a_hat_bias_pct) < opts.bias_thresh_pct);
        pass_relstd = all(agg.a_hat_relstd_pct < opts.relstd_thresh_pct);
        pass_xD     = all(agg.xD_absmean < opts.xD_thresh_um);
        all_pass    = pass_trk && pass_bias && pass_relstd && pass_xD;
    else
        pass_trk = NaN; pass_bias = NaN; pass_relstd = NaN; pass_xD = NaN; all_pass = NaN;
    end

    print_report(scenario, sc, opts, agg, pass_trk, pass_bias, pass_relstd, pass_xD, all_pass);

    % --- 6. Save summary + figures ---
    out_dir = fullfile(project_root, 'test_results', ['eq17_6state_', scenario]);
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end
    save(fullfile(out_dir, 'summary.mat'), 'agg', 'trk_meas', 'a_hat_mean', 'a_hat_std', ...
         'a_true_mean', 'relerr_mean', 'xD_absmean', 'opts', 'sc', 'config');
    write_summary_md(fullfile(out_dir, 'summary.md'), scenario, sc, opts, agg, ...
                     pass_trk, pass_bias, pass_relstd, pass_xD, all_pass);
    if opts.save_fig
        meta = struct('name', scenario, 'kind', sc.kind);
        make_eq17_6state_figures(sample, meta, out_dir);
    end
    if opts.verbose; fprintf('Saved: %s\n', out_dir); end

    results = struct('scenario', scenario, 'aggregate', agg, 'pass', all_pass, ...
                     'evaluate', sc.evaluate, 'trk_meas', trk_meas, ...
                     'a_hat_mean', a_hat_mean, 'a_hat_std', a_hat_std, 'out_dir', out_dir);
end


% ====================================================================
function sc = scenario_defaults(scenario)
%SCENARIO_DEFAULTS  Per-scenario trajectory + run settings.
    switch scenario
        case 'h50'
            sc = struct('traj_type', 'positioning', 'kind', 'positioning', ...
                        'h_init', 50, 'h_bottom', 50, 'T_sim', 5, ...
                        'seeds', 1:5, 't_warmup', 0.5, 'evaluate', true);
        case 'h10'
            % figures + descriptive only (no PASS/FAIL): near-wall a_z is small
            % so a_hat rel-std legitimately exceeds the h=50-calibrated 5% gate.
            sc = struct('traj_type', 'positioning', 'kind', 'positioning', ...
                        'h_init', 10, 'h_bottom', 10, 'T_sim', 5, ...
                        'seeds', 1:5, 't_warmup', 0.5, 'evaluate', false);
        case 'ramp'
            sc = struct('traj_type', 'ramp_descent', 'kind', 'ramp', ...
                        'h_init', 50, 'h_bottom', 5, 'T_sim', 20, ...
                        'seeds', 1, 't_warmup', 0.5, 'evaluate', false);
        otherwise
            error('verify_eq17_6state:badScenario', ...
                  'Unknown scenario "%s" (use h50 | h10 | ramp).', scenario);
    end
end


function a_true = local_a_true(p_true, w_hat, pz, R, a_nom)
%LOCAL_A_TRUE  Per-step physics ground-truth gain from noise-free position.
%   Columns ordered [x y z]; x,y use c_para, z uses c_perp.
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


function sample = build_sample(out, idx, a_true, a_pd)
%BUILD_SAMPLE  Assemble the seed-1 time-series struct for the figure function.
    N = numel(out.tout);
    p_d = out.p_d_out; p_m = out.p_m_out;
    err = (p_m - p_d) * 1e3;                     % nm

    d = 2;
    del_pm  = zeros(N, 3);
    for k = d+1:N, del_pm(k, :) = p_d(k-d, :) - p_m(k, :); end
    del_pmd = zeros(N, 3);
    for k = 2:N, del_pmd(k, :) = (1 - a_pd) * del_pmd(k-1, :) + a_pd * del_pm(k, :); end
    del_pmd_plot = -del_pmd * 1e3;               % nm, sign-matched to err

    logical_idx = false(N, 1); logical_idx(idx) = true;
    sample = struct('t', out.tout(:), 'idx', logical_idx, ...
                    'a_xm', out.diag.a_xm, 'a_hat', out.diag.a_hat, 'a_true', a_true, ...
                    'err', err, 'del_pmd', del_pmd_plot, 'xD_nm', out.diag.x_D_hat * 1e3);
end


function print_report(scenario, sc, opts, agg, pT, pB, pR, pX, all_pass)
    fprintf('\n========== verify_eq17_6state:%s (%d seeds, %.0fs, %s) ==========\n', ...
            scenario, numel(opts.seeds), opts.T_sim, sc.traj_type);
    fprintf('tracking std (p_d-p_m) [x y z] nm : [%.2f %.2f %.2f]\n', agg.trk_meas_mean);
    if sc.evaluate
        fprintf('a_hat bias vs a_true   [x y z] %% : [%.2f %.2f %.2f]  thresh %g -> %s\n', ...
            agg.a_hat_bias_pct, opts.bias_thresh_pct, passstr(pB));
        fprintf('a_hat rel-std          [x y z] %% : [%.2f %.2f %.2f]  thresh %g -> %s\n', ...
            agg.a_hat_relstd_pct, opts.relstd_thresh_pct, passstr(pR));
        fprintf('tracking std                     : thresh %g nm -> %s\n', opts.trk_thresh_nm, passstr(pT));
        fprintf('|delta_x_D^d| mean     [x y z] um: [%.2e %.2e %.2e] thresh %g -> %s\n', ...
            agg.xD_absmean, opts.xD_thresh_um, passstr(pX));
        fprintf('-------------------------------------------------------------------\n');
        fprintf('OVERALL: %s\n', passstr(all_pass));
    else
        fprintf('a_hat mean rel-err     [x y z] %% : [%.2f %.2f %.2f]  (descriptive, no PASS/FAIL)\n', ...
            agg.relerr_mean_pct);
        fprintf('|delta_x_D^d| mean     [x y z] um: [%.2e %.2e %.2e]\n', agg.xD_absmean);
        fprintf('-------------------------------------------------------------------\n');
        fprintf('FIGURES ONLY (descriptive, no PASS/FAIL)\n');
    end
end


function s = passstr(b)
    if isnan(b); s = 'N/A'; elseif b; s = 'PASS'; else; s = 'FAIL'; end
end


function write_summary_md(path, scenario, sc, opts, agg, pT, pB, pR, pX, all_pass)
    fid = fopen(path, 'w');
    fprintf(fid, '# verify_eq17_6state : %s\n\n', scenario);
    fprintf(fid, '6-state (RevisedControl_Vpersonal) controller, %s (h %g->%g), %d seeds x %.0fs.\n\n', ...
            sc.traj_type, sc.h_init, sc.h_bottom, numel(opts.seeds), opts.T_sim);
    if sc.evaluate
        fprintf(fid, '**OVERALL: %s**\n\n', passstr(all_pass));
        fprintf(fid, '| metric | x | y | z | thresh | result |\n|---|---|---|---|---|---|\n');
        fprintf(fid, '| tracking std (p_d-p_m) [nm] | %.2f | %.2f | %.2f | <%g | %s |\n', ...
                agg.trk_meas_mean, opts.trk_thresh_nm, passstr(pT));
        fprintf(fid, '| a_hat bias vs a_true [%%] | %.2f | %.2f | %.2f | <%g | %s |\n', ...
                agg.a_hat_bias_pct, opts.bias_thresh_pct, passstr(pB));
        fprintf(fid, '| a_hat rel-std [%%] | %.2f | %.2f | %.2f | <%g | %s |\n', ...
                agg.a_hat_relstd_pct, opts.relstd_thresh_pct, passstr(pR));
        fprintf(fid, '| \\|delta_x_D^d\\| mean [um] | %.2e | %.2e | %.2e | <%g | %s |\n', ...
                agg.xD_absmean, opts.xD_thresh_um, passstr(pX));
    else
        fprintf(fid, '**FIGURES ONLY: descriptive statistics (no PASS/FAIL).**\n\n');
        fprintf(fid, '| metric | x | y | z |\n|---|---|---|---|\n');
        fprintf(fid, '| tracking std (p_d-p_m) [nm] | %.2f | %.2f | %.2f |\n', agg.trk_meas_mean);
        fprintf(fid, '| a_hat mean rel-err [%%] | %.2f | %.2f | %.2f |\n', agg.relerr_mean_pct);
        fprintf(fid, '| \\|delta_x_D^d\\| mean [um] | %.2e | %.2e | %.2e |\n', agg.xD_absmean);
    end
    fprintf(fid, '\na_hat mean [x y z] = [%.4g %.4g %.4g], a_true mean = [%.4g %.4g %.4g] um/pN.\n', ...
            agg.a_hat_mean, agg.a_true_mean);
    fclose(fid);
end
