% PURPOSE (2026-10-07, branch test/conly-meng-law): stage-0 / stage-2 instrument of the Meng-law line
%   (plan: agent_docs/eq17/conly_meng_law_plan.md). Runs the committed c-only arm (law f1p2, ramp_tri 1 Hz, plane init,
%   plane + cell, seed 1) with the CURRENT control law and stores the full per-step logs; with opts.compare = true it re-runs
%   and asserts isequaln against the stored logs (the flag-off bit-identity gate of stage 2).
%   Output test_results/temp_conly_meng/baseline_<plant>.mat (gitignored, regenerable).
function out = run_conly_meng_baseline(opts)
    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'compare'); opts.compare = false; end
    if ~isfield(opts, 'plants');  opts.plants  = {'plane', 'cell'}; end
    if ~isfield(opts, 'seeds');   opts.seeds   = 1; end
    if ~isfield(opts, 'cc_extra'); opts.cc_extra = struct(); end       % stage 2: controller flags on top of the f1p2 recipe (e.g. the Meng-law flag, OFF)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(here);
    od = fullfile(root, 'test_results', 'temp_conly_meng');  if ~exist(od, 'dir'); mkdir(od); end
    FIELDS = {'a_hat', 'c_hat', 'd_hat', 'innov1', 'innov2', 'h_true', 'a_true', 'S1', 'S2', 'sP_c', 'sP_a'};
    out = struct();
    for ip = 1:numel(opts.plants)
        pl = opts.plants{ip};
        o = struct('law', 'f1p2', 'plants', {{pl}}, 'seeds', opts.seeds, 'tag_suffix', '_mengbase');
        if ~isempty(fieldnames(opts.cc_extra)); o.cc_extra = opts.cc_extra; end
        R = run_plaw_walls(o);
        tag = fieldnames(R);  res = R.(tag{1});
        B = struct();  for f = FIELDS; if isfield(res, f{1}); B.(f{1}) = res.(f{1}); end; end
        B.tag = tag{1};  B.seeds = opts.seeds;  B.commit = local_git_head(root);
        fn = fullfile(od, sprintf('baseline_%s.mat', pl));
        if opts.compare
            assert(exist(fn, 'file') == 2, 'no stored baseline %s', fn);
            A = load(fn);
            same = true;
            for f = FIELDS
                if ~isfield(A, f{1}); continue; end
                eq = isequaln(A.(f{1}), B.(f{1}));
                if ~eq
                    same = false;
                    fprintf('  DIFF %-7s %-7s max|d| = %.3e\n', pl, f{1}, max(abs(A.(f{1})(:) - B.(f{1})(:))));
                end
            end
            verdict = {'FAIL', 'PASS'};
            fprintf('BIT-IDENTITY %-6s vs baseline (%s): %s\n', pl, A.commit, verdict{same + 1});
            out.(pl) = same;
        else
            save(fn, '-struct', 'B');
            fprintf('BASELINE %-6s stored -> %s (commit %s): min h_true %.4f | NaN %d | e_a end %+.4f | c_hat end %+.4f\n', ...
                    pl, fn, B.commit, min(B.h_true(:)), nnz(~isfinite(B.a_hat(:))), ...
                    mean(B.a_true(end, :) - B.a_hat(end, :)), mean(B.c_hat(end, :)));
            out.(pl) = B;
        end
    end
end

function h = local_git_head(root)
    [st, s] = system(sprintf('git -C "%s" rev-parse --short HEAD', root));
    if st == 0; h = strtrim(s); else; h = 'unknown'; end
end
