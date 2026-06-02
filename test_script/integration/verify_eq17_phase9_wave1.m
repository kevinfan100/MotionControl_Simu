function run_phase9_wave1()
%RUN_PHASE9_WAVE1  Driver for Phase 9 R(2,2) validation Wave 1 (12 sims).
%
%   run_phase9_wave1()
%
%   Calls run_R22_validation 12 times per the Wave 1 sim matrix:
%
%     V1 (5 sims): seeds {1..5}, a_cov=0.05, sigma2_n_factor=1.0
%     V2 a_cov sweep (4 sims): seed=1, a_cov in {0.01, 0.02, 0.10, 0.20},
%         sigma2_n_factor=1.0  (a_cov=0.05 reuses V1 seed1)
%     V3 sigma^2_n sweep (3 sims): seed=1, a_cov=0.05,
%         sigma2_n_factor in {0.5, 2.0, 4.0}  (factor=1 reuses V1 seed1)
%
%   Each sim is saved to:
%     reference/eq17_analysis/phase9_results_{tag}_seed{i}.mat
%
%   No analysis is performed here — Agent Z consumes the .mat files.

    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(script_dir);
    addpath(script_dir);                               % run_R22_validation
    out_dir = fullfile(project_root, 'reference', 'eq17_analysis');
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end

    % -----------------------------------------------------------------
    % Wave 1 sim matrix: cell array of {tag, scenario_struct, seed}
    % -----------------------------------------------------------------
    matrix = {};

    % V1 main: 5 seeds, a_cov=0.05, sigma2_n_factor=1.0
    for s = 1:5
        sc = struct('a_cov', 0.05, 'sigma2_n_factor', 1.0);
        matrix{end+1} = {'V1', sc, s}; %#ok<AGROW>
    end

    % V2 a_cov sweep (seed=1, sigma2_n_factor=1.0); a_cov=0.05 reuses V1 seed1
    a_cov_values = [0.01, 0.02, 0.10, 0.20];
    a_cov_tags   = {'V2_acov0p01', 'V2_acov0p02', 'V2_acov0p1', 'V2_acov0p2'};
    for i = 1:numel(a_cov_values)
        sc = struct('a_cov', a_cov_values(i), 'sigma2_n_factor', 1.0);
        matrix{end+1} = {a_cov_tags{i}, sc, 1}; %#ok<AGROW>
    end

    % V3 sigma2_n sweep (seed=1, a_cov=0.05); factor=1 reuses V1 seed1
    sn_values = [0.5, 2.0, 4.0];
    sn_tags   = {'V3_sigma2n0p5', 'V3_sigma2n2', 'V3_sigma2n4'};
    for i = 1:numel(sn_values)
        sc = struct('a_cov', 0.05, 'sigma2_n_factor', sn_values(i));
        matrix{end+1} = {sn_tags{i}, sc, 1}; %#ok<AGROW>
    end

    n_sims = numel(matrix);
    fprintf('[run_phase9_wave1] Total sims: %d\n', n_sims);
    fprintf('[run_phase9_wave1] Output dir: %s\n', out_dir);

    t_global = tic;
    for i = 1:n_sims
        tag      = matrix{i}{1};
        scenario = matrix{i}{2};
        seed     = matrix{i}{3};

        out_file = fullfile(out_dir, sprintf('phase9_results_%s_seed%d.mat', tag, seed));
        fprintf('  [%2d/%2d] tag=%-18s seed=%d  a_cov=%.3f  s2n_factor=%.2f ... ', ...
                i, n_sims, tag, seed, scenario.a_cov, scenario.sigma2_n_factor);

        t_sim = tic;
        results = run_R22_validation(scenario, seed); %#ok<NASGU>
        elapsed = toc(t_sim);

        save(out_file, '-struct', 'results', '-v7.3');
        fprintf('done (%.1fs)\n', elapsed);
    end

    fprintf('[run_phase9_wave1] All sims complete in %.1fs\n', toc(t_global));
end
