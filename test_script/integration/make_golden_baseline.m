function baseline = make_golden_baseline(tag, opts)
%MAKE_GOLDEN_BASELINE  Freeze full simulation outputs for packaging A/B gates.
%
%   baseline = make_golden_baseline(tag)
%   baseline = make_golden_baseline(tag, opts)
%
%   tag : snapshot label, e.g. 'pre_d6' | 'post_d6'  (default 'baseline')
%
%   Runs the three packaging scenarios with a FULLY EXPLICIT config (every
%   controller-relevant field pinned below -- do NOT rely on user_config /
%   apply_qr_preset defaults: the 'frozen_correct' preset silently sets
%   a_cov = 0.005) and saves the complete time series of every run plus
%   summary statistics to:
%
%       test_results/pack_baseline/golden_<tag>.mat        (gitignored)
%
%   This .mat is the answer key for the standalone-packaging equivalence
%   gates (see standalone/PACKAGING_PLAN.md section 6). Compare two
%   snapshots with compare_baselines('pre_d6', 'post_d6').
%
%   Scenarios (PACKAGING_PLAN.md decision 11):
%       h50      positioning h=50 um, seeds 1:5, T=5 s   (quantitative ref)
%       h10      positioning h=10 um, seeds 1:5, T=5 s
%       ramp2p7  ramp_descent 50 -> 2.7 um (h_bar = 1.2), seeds 1:3, T=20 s
%                (package envelope boundary; requires h_min = 1.2*R)
%
%   opts (optional): .scenarios (cellstr subset), .verbose (true)
%
%   See also: compare_baselines, verify_eq17_6state, run_pure_simulation

    if nargin < 1 || isempty(tag); tag = 'baseline'; end
    if nargin < 2 || isempty(opts); opts = struct(); end
    if ~isfield(opts, 'scenarios') || isempty(opts.scenarios)
        opts.scenarios = {'h50', 'h10', 'ramp2p7'};
    end
    if ~isfield(opts, 'verbose') || isempty(opts.verbose); opts.verbose = true; end

    % --- Paths (script in test_script/integration/ -> up two levels) ---
    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(fileparts(script_dir));
    addpath(fullfile(project_root, 'model'), fullfile(project_root, 'model', 'config'), ...
            fullfile(project_root, 'model', 'wall_effect'), ...
            fullfile(project_root, 'model', 'thermal_force'), ...
            fullfile(project_root, 'model', 'trajectory'), ...
            fullfile(project_root, 'model', 'controller'), ...
            fullfile(project_root, 'model', 'dual_track'), script_dir);

    % --- Snapshot provenance ---
    [~, git_head] = system(sprintf('git -C "%s" rev-parse --short HEAD', project_root));
    baseline = struct();
    baseline.tag        = tag;
    baseline.git_head   = strtrim(git_head);
    baseline.matlab_ver = version;
    baseline.created    = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
    baseline.runs       = {};

    for isc = 1:numel(opts.scenarios)
        sc_name = opts.scenarios{isc};
        sc = scenario_table(sc_name);

        % --- Config: fully explicit (mirrors verify_eq17_6state section 2,
        %     plus h_min for the ramp2p7 envelope) ---
        config = user_config();
        config.h_init            = sc.h_init;
        config.h_bottom          = sc.h_bottom;
        config.h_min             = sc.h_min;       % [um] envelope floor
        config.amplitude         = 0;
        config.trajectory_type   = sc.traj_type;
        config.T_sim             = sc.T_sim;
        config.ctrl_enable       = true;
        config.meas_noise_enable = true;
        config.thermal_enable    = true;
        config.lambda_c          = 0.7;
        config.a_pd              = 0.05;           % IIR mean-EWMA pole
        config.a_cov             = 0.05;           % IIR var-EWMA pole (preset trap: keep explicit)
        config.meas_noise_std    = [0.00062; 0.00057; 0.00331];  % [um] (y = 0.57 nm, corrected)
        config.eq17_variant      = '6state';

        % Exact physical/wall values for stats (same path as verify script)
        params = calc_simulation_params(config);
        Pv = params.Value;
        geo = struct('Ts', Pv.common.Ts, 'R', Pv.common.R, ...
                     'a_nom', Pv.common.Ts / Pv.common.gamma_N, ...
                     'w_hat', Pv.wall.w_hat, 'pz', Pv.wall.pz);

        for s = 1:numel(sc.seeds)
            seed = sc.seeds(s);
            ro = struct('seed', seed, 'verbose', false, 'collect_diag', true);
            t0 = tic;
            out = run_pure_simulation(config, ro);
            wall_sec = toc(t0);

            run = struct();
            run.scenario = sc_name;
            run.seed     = seed;
            run.config   = config;
            run.wall_sec = wall_sec;
            run.out      = out;                    % full time series incl. diag
            run.stats    = run_stats(out, geo);
            baseline.runs{end+1} = run;

            if opts.verbose
                fprintf('[golden:%s] %s seed=%d  trk[nm]=[%.2f %.2f %.2f]  a_bias[%%]=[%.2f %.2f %.2f]  (%.1fs)\n', ...
                        tag, sc_name, seed, run.stats.trk_std_nm, run.stats.a_bias_pct, wall_sec);
            end
        end
    end

    out_dir = fullfile(project_root, 'test_results', 'pack_baseline');
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end
    out_path = fullfile(out_dir, ['golden_', tag, '.mat']);
    save(out_path, 'baseline', '-v7.3');
    if opts.verbose
        fprintf('[golden:%s] %d runs saved -> %s (HEAD %s)\n', ...
                tag, numel(baseline.runs), out_path, baseline.git_head);
    end
end


% ====================================================================
function sc = scenario_table(name)
%SCENARIO_TABLE  Packaging scenario set (PACKAGING_PLAN.md decision 11).
    R = 2.25;                                       % [um] probe radius
    switch lower(name)
        case 'h50'
            sc = struct('traj_type', 'positioning', 'h_init', 50, 'h_bottom', 50, ...
                        'h_min', 1.5 * R, 'T_sim', 5, 'seeds', 1:5);
        case 'h10'
            sc = struct('traj_type', 'positioning', 'h_init', 10, 'h_bottom', 10, ...
                        'h_min', 1.5 * R, 'T_sim', 5, 'seeds', 1:5);
        case 'ramp2p7'
            % Package envelope boundary: h_bottom = 1.2*R = 2.7 um (h_bar 1.2).
            % Mother-repo truth INCLUDES the 3-guard behavior (G3 latches for
            % h_bar < 1.5 tail) -- recorded as-is.
            sc = struct('traj_type', 'ramp_descent', 'h_init', 50, 'h_bottom', 1.2 * R, ...
                        'h_min', 1.2 * R, 'T_sim', 20, 'seeds', 1:3);
        otherwise
            error('make_golden_baseline:badScenario', ...
                  'Unknown scenario "%s" (use h50 | h10 | ramp2p7).', name);
    end
end


function st = run_stats(out, geo)
%RUN_STATS  Summary statistics (mirrors verify_eq17_6state aggregation).
%   geo: Ts, R, a_nom, w_hat, pz from calc_simulation_params (exact values).
    n_warmup = round(0.5 / geo.Ts);
    N = numel(out.tout); idx = (n_warmup + 1):N;

    st = struct();
    st.trk_std_nm = std(out.p_d_out(idx, :) - out.p_m_out(idx, :), 0, 1) * 1e3;

    % Physics ground truth from the noise-free probe (same as verify script)
    h_true = (out.p_true_out * geo.w_hat - geo.pz) / geo.R;
    a_para = zeros(N, 1); a_perp = zeros(N, 1);
    for k = 1:N
        hb = max(h_true(k), 1.001);
        [cpa, cpe] = calc_correction_functions(hb);
        a_para(k) = geo.a_nom / cpa;
        a_perp(k) = geo.a_nom / cpe;
    end
    a_true = [a_para, a_para, a_perp];

    a_hat = out.diag.a_hat;                         % a-priori (D6) or posterior (pre-D6)
    st.a_hat_mean  = mean(a_hat(idx, :), 1);
    st.a_hat_std   = std(a_hat(idx, :), 0, 1);
    st.a_true_mean = mean(a_true(idx, :), 1);
    st.a_bias_pct  = (st.a_hat_mean - st.a_true_mean) ./ st.a_true_mean * 100;
    st.xD_absmean  = mean(abs(out.diag.x_D_hat(idx, :)), 1);
    st.h_bar_min   = min(h_true);
end
