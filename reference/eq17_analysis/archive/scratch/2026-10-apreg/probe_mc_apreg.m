% STATUS: SCRATCH -- 2026-10-16: scores for run_mc_apreg (plane a_d 0.07 h* 1.078; cell a_d 0.55 h* 1.334, 0.60 h* 1.654). Per seed: contact, NaN, min h, time when
%   h first < h* + 0.1 R, last 0.5 s mean h - h*, a_true - a_d, e_a = a_true - a_hat, law slope at the end vs true slope at h*.
function probe_mc_apreg()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'test_results', 'temp_1009_ws');
    CASES = {'plane', '', 1.078; 'cell', '_ad055', 1.334; 'cell', '_ad060', 1.654};
    for arm = {'diff', 'state'}
        for ic = 1:size(CASES, 1)
            wall = CASES(ic, 1);  HS = CASES{ic, 3};
            S = load(fullfile(od, sprintf('apreg_%s_%s%s.mat', arm{1}, wall{1}, CASES{ic, 2})));  L = S.t > 5.5;
            fprintf('\n[%s, %s, a_d %.2f]  seed | contact | NaN | min h | t(h < h*+0.1) | h - h* | a_true - a_d | e_a | a''hat end\n', arm{1}, wall{1}, S.a_d);
            for q = 1:numel(S.seeds)
                h = S.h_true(:, q);  i1 = find(h < HS + 0.1, 1);  t1 = NaN;  if ~isempty(i1); t1 = S.t(i1); end
                fprintf('   %d | %d | %d | %.3f | %.3f | %+.4f | %+.4f | %+.4f | %.3f\n', S.seeds(q), S.contact(q), nnz(isnan(S.a_hat(:, q)) & ~isnan(S.t)), ...
                        min(h), t1, mean(h(L)) - HS, mean(S.a_true(L, q)) - S.a_d, mean(S.a_true(L, q) - S.a_hat(L, q)), S.ap_hat(end, q));
            end
        end
    end
end
