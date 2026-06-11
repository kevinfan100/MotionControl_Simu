function results = compare_gain_oracle_6state(freqs, opts)
%COMPARE_GAIN_ORACLE_6STATE Run the gain-oracle A/B matrix (design doc
%   reference/eq17_analysis/gain_oracle_ab_design.md §2-§3, §7 Layer 0).
%
%   results = compare_gain_oracle_6state()            % freqs = [1 5 10]
%   results = compare_gain_oracle_6state(freqs, opts)
%
%   Round-2 defaults (gate-free, 100 seeds):
%     freqs      : [1 5 10] Hz
%     seeds      : 1:100
%     T_sim      : 4.0 s
%     n_cyc_per_s: 2   (osc duration = n_cyc_per_s seconds, design §12.1)
%     h_bar_safe : 1   (G3 unreachable; trough h_bar = 1.2, design §12.1)
%     out_root   : test_results/gain_oracle_ab_nogate/
%
%   Per frequency: 2 arms x (1 det run + numel(seeds) noisy runs), all
%   collect_diag. Arm A = gain_oracle (true time-varying gain in the
%   control law), arm B = EKF gain. Both arms suppress_xD. Layer 0
%   assertions (incl. gate-free check, design §12.7 acceptance #6) run
%   before saving. Output:
%       test_results/gain_oracle_ab_nogate/f<f>Hz/runs.mat        (production)
%       test_results/gain_oracle_ab_nogate/f<f>Hz-smoke/runs.mat  (smoke mode)
%
%   opts fields:
%     seeds        (1:100)  - RNG seeds for noisy runs
%     T_sim        (4.0)    - simulation duration [s]
%     n_cyc_per_s  (2)      - oscillation cycles per second of osc phase
%     h_bar_safe   (1)      - Guard-3 threshold (1 = gate-free for h_bar>=1.2)
%     verbose      (true)
%     out_root     (test_results/gain_oracle_ab_nogate/)
%     smoke        (false;  true -> T_sim=2.0 + seeds=1, saved under -smoke
%                           suffix so production data is never overwritten)
%
%   To reproduce the Round-1 gated matrix:
%     compare_gain_oracle_6state([1 2 5], struct( ...
%         'seeds', 1:20, 'T_sim', 7.0, 'n_cyc_per_s', 5, ...
%         'h_bar_safe', 1.5, ...
%         'out_root', fullfile(project_root,'test_results','gain_oracle_ab')))
%
%   See also: analyze_gain_oracle_6state, run_pure_simulation

    if nargin < 1 || isempty(freqs); freqs = [1 5 10]; end
    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'seeds');       opts.seeds = 1:100;     end
    if ~isfield(opts, 'T_sim');       opts.T_sim = 4.0;       end
    if ~isfield(opts, 'n_cyc_per_s'); opts.n_cyc_per_s = 2;   end   % osc duration [s] (design §12.1)
    if ~isfield(opts, 'h_bar_safe');  opts.h_bar_safe = 1;    end   % gate-free B-prime (design §12.1)
    if ~isfield(opts, 'verbose'); opts.verbose = true; end
    if ~isfield(opts, 'smoke');   opts.smoke = false;  end
    if opts.smoke
        opts.T_sim = 2.0; opts.seeds = 1;
    end

    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(fileparts(script_dir));
    addpath(fullfile(project_root, 'model'), fullfile(project_root, 'model', 'config'), ...
            fullfile(project_root, 'model', 'wall_effect'), ...
            fullfile(project_root, 'model', 'thermal_force'), ...
            fullfile(project_root, 'model', 'trajectory'), ...
            fullfile(project_root, 'model', 'controller'), ...
            fullfile(project_root, 'model', 'dual_track'), script_dir);
    if ~isfield(opts, 'out_root')
        opts.out_root = fullfile(project_root, 'test_results', 'gain_oracle_ab_nogate');
    end

    results = struct('freq', {}, 'out_dir', {}, 'layer0', {}, 'n_diverged', {});
    for fi = 1:numel(freqs)
        f = freqs(fi);
        cfg = build_config(f, opts);

        % --- pre-flight: desired trajectory respects h_min override ---
        params = calc_simulation_params(cfg);
        [is_safe, h_min_actual, t_crit] = check_trajectory_safety(params.Value);
        assert(is_safe, 'Trajectory unsafe: h_min_actual=%.3f um at t=%.2f s', ...
               h_min_actual, t_crit);

        if opts.verbose
            fprintf('[compare_gain_oracle:%gHz] T_sim=%.1fs seeds=%s\n', ...
                    f, cfg.T_sim, mat2str(opts.seeds));
        end

        % --- run matrix ---
        runs = struct();
        for arm = 'AB'
            oracle = (arm == 'A');
            cfg_det = cfg;
            cfg_det.thermal_enable = false; cfg_det.meas_noise_enable = false;
            runs.(arm).det = run_one(cfg_det, 0, oracle, true);
            for s = 1:numel(opts.seeds)
                runs.(arm).noisy(s) = run_one(cfg, opts.seeds(s), oracle, false);
            end
        end

        % --- Layer 0 assertions ---
        layer0 = layer0_checks(runs, cfg, opts);

        % --- divergence bookkeeping (persisted with the runs) ---
        n_div = count_diverged(runs);
        diverged_list = build_diverged_list(runs);
        if opts.verbose && n_div > 0
            for di = 1:numel(diverged_list)
                dl = diverged_list(di);
                det_tag = '';
                if dl.is_det; det_tag = ' (det)'; end
                fprintf('    DIVERGED arm %c seed %d%s: %s\n', ...
                        dl.arm, dl.seed, det_tag, dl.reason);
            end
        end

        % --- save (smoke output isolated from production dirs) ---
        if opts.smoke
            out_dir = fullfile(opts.out_root, sprintf('f%gHz-smoke', f));
        else
            out_dir = fullfile(opts.out_root, sprintf('f%gHz', f));
        end
        if ~exist(out_dir, 'dir'); mkdir(out_dir); end
        save(fullfile(out_dir, 'runs.mat'), 'runs', 'cfg', 'opts', 'layer0', ...
             'n_div', 'diverged_list', '-v7.3');
        if opts.verbose
            fprintf('  saved %s  (diverged runs: %d)\n', out_dir, n_div);
        end
        results(end+1) = struct('freq', f, 'out_dir', out_dir, ...
                                'layer0', layer0, 'n_diverged', n_div); %#ok<AGROW>
    end
end


function cfg = build_config(f, opts)
%BUILD_CONFIG osc_aggr scenario (design doc §3) at frequency f.
    cfg = user_config();
    cfg.eq17_variant   = '6state';
    cfg.trajectory_type = 'osc';
    cfg.h_init   = 50;            % [um] h_bar ~ 22
    cfg.h_bottom = 2.7;           % [um] h_bar = 1.2 (below gate 1.5)
    cfg.amplitude = 2.5;          % [um] -> h_bar in [1.2, 3.42], gate-crossing
    cfg.frequency = f;
    cfg.n_cycles  = opts.n_cyc_per_s * f;   % osc duration = n_cyc_per_s seconds
    cfg.t_hold    = 0.5;
    cfg.t_descend_override = 1.0; % decouple descent from 1/f
    cfg.T_sim     = opts.T_sim;
    pc = physical_constants();
    cfg.h_min     = 1.05 * pc.R;  % scenario-local override (global default 1.5R blocks h_bar=1.2)
    cfg.ctrl_enable = true;
    cfg.thermal_enable = true;
    cfg.meas_noise_enable = true;
    cfg.lambda_c = 0.7;
    cfg.a_pd  = 0.05;
    cfg.a_cov = 0.05;
    cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];   % [um]
    cfg.suppress_xD = true;       % both arms (design §2)
    cfg.h_bar_safe = opts.h_bar_safe;   % Round 2: 1 -> G3 unreachable (trough h_bar = 1.2)
end


function rec = run_one(cfg, seed, oracle, is_det)
%RUN_ONE Single run wrapped in try/catch (crash = diverged, design §7.3).
    if nargin < 4; is_det = false; end
    ro = struct('seed', seed, 'verbose', false, 'collect_diag', true, ...
                'gain_oracle', oracle);
    rec = struct('seed', seed, 'oracle', oracle, 'is_det', is_det, ...
                 'diverged', false, 'diverge_reason', '', 'simOut', []);
    try
        rec.simOut = run_pure_simulation(cfg, ro);
    catch err
        rec.diverged = true;
        rec.diverge_reason = sprintf('crash: %s', err.message);
        return;
    end
    % post-run divergence scan on aligned physical error (design §6.0/§7.3)
    e = rec.simOut.p_d_out(2:end, :) - rec.simOut.p_true_out(1:end-1, :);
    if max(abs(e(:))) > 0.5
        rec.diverged = true;
        rec.diverge_reason = sprintf('|e|_max = %.3g um > 0.5 um', max(abs(e(:))));
    end
end


function layer0 = layer0_checks(runs, cfg, opts)
%LAYER0_CHECKS Design §7 Layer 0: p_d identity + wiring assertions.
%   Gate-free assertion (design §12.7 acceptance #6) is added when
%   cfg.h_bar_safe < h_bottom/R so Guard 3 is unreachable for noisy runs.
    ref = first_ok_run(runs);
    assert(~isempty(ref), 'Layer0: every run crashed — nothing to analyze');
    pd_ref = ref.simOut.p_d_out;
    layer0 = struct('pd_identical', true, 'wiring_A', true, 'wiring_B', true, ...
                    'n_checked', 0, 'n_skipped_A', 0, 'n_skipped_B', 0);

    R_phys = ref.simOut.meta.params_value.common.R;
    gate_free_expected = isfield(cfg, 'h_bar_safe') && ...
        cfg.h_bar_safe < cfg.h_bottom / R_phys * (1 - 1e-9);

    for arm = 'AB'
        recs = [runs.(arm).det, runs.(arm).noisy];
        for r = recs
            if isempty(r.simOut)                  % crashed run: skip checks
                layer0.(sprintf('n_skipped_%c', arm)) = ...
                    layer0.(sprintf('n_skipped_%c', arm)) + 1;
                continue;
            end
            % Skip diverged runs with populated simOut: wiring assertions
            % are meaningless for numerically blown runs (NaN EKF states
            % can cause max(NaN...) == 0 to be false, firing spuriously).
            if r.diverged
                layer0.(sprintf('n_skipped_%c', arm)) = ...
                    layer0.(sprintf('n_skipped_%c', arm)) + 1;
                continue;
            end
            layer0.n_checked = layer0.n_checked + 1;
            assert(isequal(size(r.simOut.p_d_out), size(pd_ref)) && ...
                   max(abs(r.simOut.p_d_out(:) - pd_ref(:))) == 0, ...
                   'Layer0: p_d_out differs (arm %c seed %d)', arm, r.seed);
            if arm == 'A'
                assert(max(abs(r.simOut.diag.a_ctrl_used(:) - r.simOut.a_true_out(:))) == 0, ...
                       'Layer0: arm A a_ctrl_used ~= a_true_out (seed %d)', r.seed);
            else
                d = abs(r.simOut.diag.a_ctrl_used(2:end, :) - r.simOut.diag.a_hat(1:end-1, :));
                assert(max(d(:)) == 0, ...
                       'Layer0: arm B a_ctrl_used ~= a_hat posterior[k-1] (seed %d)', r.seed);
            end
            % design §12.7 acceptance #6: expected-gate-free NOISY runs must
            % never gate. The no-noise det run is exempt: Guard 2 latches by
            % design when sigma2_dxr -> 0 (known behavior, design §12.0).
            if gate_free_expected && ~r.is_det
                assert(~any(r.simOut.diag.gate_active(:)), ...
                       'Layer0: gate fired in expected-gate-free run (arm %c seed %d)', ...
                       arm, r.seed);
            end
        end
    end
    if opts.verbose; fprintf('  Layer 0: PASS\n'); end
end


function rec = first_ok_run(runs)
    % p_d_out is trajectory-deterministic (same cfg both arms), so any
    % non-crashed run from either arm is a valid p_d reference.
    rec = [];
    for arm = 'AB'
        all_r = [runs.(arm).det, runs.(arm).noisy];
        for r = all_r
            if ~isempty(r.simOut); rec = r; return; end
        end
    end
end


function n = count_diverged(runs)
    n = 0;
    for arm = 'AB'
        all_r = [runs.(arm).det, runs.(arm).noisy];
        n = n + sum([all_r.diverged]);
    end
end


function lst = build_diverged_list(runs)
%BUILD_DIVERGED_LIST One entry per diverged run: {arm, seed, is_det, reason}.
    lst = struct('arm', {}, 'seed', {}, 'is_det', {}, 'reason', {});
    for arm = 'AB'
        all_r = [runs.(arm).det, runs.(arm).noisy];
        for r = all_r
            if r.diverged
                lst(end+1) = struct('arm', arm, 'seed', r.seed, ...
                                    'is_det', r.is_det, ...
                                    'reason', r.diverge_reason); %#ok<AGROW>
            end
        end
    end
end
