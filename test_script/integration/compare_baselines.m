function report = compare_baselines(tagA, tagB)
%COMPARE_BASELINES  Systematic comparison of two golden baseline snapshots.
%
%   report = compare_baselines(tagA, tagB)
%
%   Loads test_results/pack_baseline/golden_<tag>.mat for both tags, matches
%   runs by (scenario, seed), and reports per run:
%       max_abs_dpm   max |p_m_A - p_m_B|            [um]  (0 -> bit-exact)
%       rms_rel_dpm   rms(p_m_A - p_m_B)/rms(p_m_A)        (relative drift)
%       d_trk_nm      tracking-std delta [x y z]     [nm]
%       d_abias_pct   a_hat bias delta   [x y z]     [pct points]
%
%   Use cases:
%       compare_baselines('pre_d6', 'post_d6')   quantify the D6 migration delta
%       compare_baselines('post_d6', 'pack_b1')  packaging verbatim gate (expect 0)
%
%   See also: make_golden_baseline

    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(fileparts(script_dir));
    base_dir = fullfile(project_root, 'test_results', 'pack_baseline');

    A = load_tag(base_dir, tagA);
    B = load_tag(base_dir, tagB);

    fprintf('\n===== compare_baselines: %s (HEAD %s)  vs  %s (HEAD %s) =====\n', ...
            tagA, A.git_head, tagB, B.git_head);
    fprintf('%-9s %-5s %-12s %-12s %-26s %-26s\n', 'scenario', 'seed', ...
            'max|dp_m|um', 'rms_rel', 'd_trk_nm [x y z]', 'd_abias_pct [x y z]');

    report = struct('rows', {{}});
    for i = 1:numel(A.runs)
        ra = A.runs{i};
        rb = find_match(B.runs, ra.scenario, ra.seed);
        if isempty(rb)
            fprintf('%-9s %-5d (no match in %s)\n', ra.scenario, ra.seed, tagB);
            continue;
        end

        dpm = ra.out.p_m_out - rb.out.p_m_out;
        row = struct();
        row.scenario    = ra.scenario;
        row.seed        = ra.seed;
        row.max_abs_dpm = max(abs(dpm(:)));
        row.rms_rel_dpm = rms(dpm(:)) / max(rms(ra.out.p_m_out(:)), eps);
        row.d_trk_nm    = rb.stats.trk_std_nm - ra.stats.trk_std_nm;
        row.d_abias_pct = rb.stats.a_bias_pct - ra.stats.a_bias_pct;
        report.rows{end+1} = row;

        fprintf('%-9s %-5d %-12.3e %-12.3e [%6.2f %6.2f %6.2f]    [%6.2f %6.2f %6.2f]\n', ...
                row.scenario, row.seed, row.max_abs_dpm, row.rms_rel_dpm, ...
                row.d_trk_nm, row.d_abias_pct);
    end
    if numel(B.runs) ~= numel(A.runs)
        fprintf('NOTE: run-count mismatch (%s: %d, %s: %d) -- unmatched runs ignored.\n', ...
                tagA, numel(A.runs), tagB, numel(B.runs));
    end
    fprintf('================================================================\n');
end


function S = load_tag(base_dir, tag)
    p = fullfile(base_dir, ['golden_', tag, '.mat']);
    if ~exist(p, 'file')
        error('compare_baselines:missing', 'Snapshot not found: %s', p);
    end
    L = load(p, 'baseline');
    S = L.baseline;
end


function r = find_match(runs, scenario, seed)
    r = [];
    for i = 1:numel(runs)
        if strcmp(runs{i}.scenario, scenario) && runs{i}.seed == seed
            r = runs{i};
            return;
        end
    end
end
