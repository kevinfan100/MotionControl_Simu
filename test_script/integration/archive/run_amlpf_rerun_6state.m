function out_path = run_amlpf_rerun_6state(baseline_freq_dir, opts)
%RUN_AMLPF_RERUN_6STATE  P4 of the a_m_det LPF study (am_lpf_r22_design.md).
%   Re-runs the gain_compare a=a_hat arm (arm B) with use_am_lpf=true, reusing
%   the EXACT baseline cfg so the only difference vs baseline is the LPF. Same
%   seed -> same thermal/noise realization (paired comparison). Arm A (a=a_true)
%   is UNAFFECTED by the LPF, so it is taken from the baseline file by the
%   downstream plot/verify scripts (not re-run here).
%
%   out_path = run_amlpf_rerun_6state(baseline_freq_dir, opts)
%
%   baseline_freq_dir : dir holding the baseline runs.mat (e.g.
%                       test_results/gain_compare/f1Hz)
%   opts.a_det        : LPF EWMA weight                  (default 0.005)
%   opts.seeds        : seed subset                       (default = baseline's)
%   opts.out_dir      : output dir                        (default test_results/am_lpf/f<f>Hz)
%   opts.preview_n    : save a preview after this many noisy seeds (default 20)
%
%   Saves: <out_dir>/runs_lpf.mat (final) + <out_dir>/runs_lpf_preview.mat
%   Each holds: runs.B.det, runs.B.noisy(1:N), cfg_lpf, seeds, a_det, n_div.
%
%   See also: compare_gain_6state, run_pure_simulation, verify_r22_amlpf_6state

    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'a_det') || isempty(opts.a_det); opts.a_det = 0.005; end
    if ~isfield(opts, 'preview_n') || isempty(opts.preview_n); opts.preview_n = 20; end

    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(fileparts(script_dir));
    addpath(fullfile(project_root, 'model'), fullfile(project_root, 'model', 'config'), ...
            fullfile(project_root, 'model', 'wall_effect'), ...
            fullfile(project_root, 'model', 'thermal_force'), ...
            fullfile(project_root, 'model', 'trajectory'), ...
            fullfile(project_root, 'model', 'controller'), ...
            fullfile(project_root, 'model', 'dual_track'), script_dir);

    % --- Load baseline cfg + seeds verbatim (cheap matfile partial load) ---
    base_mat = fullfile(baseline_freq_dir, 'runs.mat');
    assert(exist(base_mat, 'file') == 2, 'baseline runs.mat not found: %s', base_mat);
    m = matfile(base_mat);
    cfg_lpf = m.cfg;
    base_opts = m.opts;
    if ~isfield(opts, 'seeds') || isempty(opts.seeds)
        opts.seeds = base_opts.seeds;
    end

    % --- Enable the LPF (only difference vs baseline) ---
    cfg_lpf.use_am_lpf = true;
    cfg_lpf.a_det      = opts.a_det;

    f = cfg_lpf.frequency;
    if ~isfield(opts, 'out_dir') || isempty(opts.out_dir)
        opts.out_dir = fullfile(project_root, 'test_results', 'am_lpf', sprintf('f%gHz', f));
    end
    if ~exist(opts.out_dir, 'dir'); mkdir(opts.out_dir); end

    fprintf('[amlpf_rerun:%gHz] arm B (a=a_hat), use_am_lpf=true a_det=%.4g, seeds=%d\n', ...
            f, opts.a_det, numel(opts.seeds));

    % --- det run (no thermal / no meas noise) ---
    cfg_det = cfg_lpf;
    cfg_det.thermal_enable = false; cfg_det.meas_noise_enable = false;
    runs.B.det = local_run_one(cfg_det, 0, false, true);

    % --- noisy seeds ---
    n_seed = numel(opts.seeds);
    for s = 1:n_seed
        runs.B.noisy(s) = local_run_one(cfg_lpf, opts.seeds(s), false, false);
        if mod(s, 10) == 0
            fprintf('  seed %d/%d done\n', s, n_seed);
        end
        if s == opts.preview_n
            n_div_p = sum([runs.B.det.diverged, runs.B.noisy(1:s).diverged]);
            seeds = opts.seeds(1:s); a_det = opts.a_det;
            runs_p.B.det = runs.B.det; runs_p.B.noisy = runs.B.noisy(1:s);
            save(fullfile(opts.out_dir, 'runs_lpf_preview.mat'), ...
                 'runs_p', 'cfg_lpf', 'seeds', 'a_det', 'n_div_p', '-v7.3');
            fprintf('  saved preview (%d seeds) -> %s\n', s, opts.out_dir);
        end
    end

    n_div = sum([runs.B.det.diverged, runs.B.noisy.diverged]);
    seeds = opts.seeds; a_det = opts.a_det;
    out_path = fullfile(opts.out_dir, 'runs_lpf.mat');
    save(out_path, 'runs', 'cfg_lpf', 'seeds', 'a_det', 'n_div', '-v7.3');
    fprintf('  saved %s  (diverged: %d)\n', out_path, n_div);
end


function rec = local_run_one(cfg, seed, use_true_gain, is_det)
%LOCAL_RUN_ONE  Mirror of compare_gain_6state/run_one (try/catch + divergence).
    ro = struct('seed', seed, 'verbose', false, 'collect_diag', true, ...
                'use_true_gain', use_true_gain);
    rec = struct('seed', seed, 'use_true_gain', use_true_gain, 'is_det', is_det, ...
                 'diverged', false, 'diverge_reason', '', 'simOut', []);
    try
        rec.simOut = run_pure_simulation(cfg, ro);
    catch err
        rec.diverged = true;
        rec.diverge_reason = sprintf('crash: %s', err.message);
        return;
    end
    e = rec.simOut.p_d_out(2:end, :) - rec.simOut.p_true_out(1:end-1, :);
    if max(abs(e(:))) > 0.5
        rec.diverged = true;
        rec.diverge_reason = sprintf('|e|_max = %.3g um > 0.5 um', max(abs(e(:))));
    end
end
