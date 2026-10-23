% STATUS: SCRATCH -- 2026-10-18: scores, a' state regulation: a law + speed cap (tau 0.1, run_mc_apreg 'state') vs u law (run_mc_apreg_u,
%   tau 0.5 / 1.0, no cap). Per case: contacts, and over the survivors: time to h < h* + 0.1, min h, last 0.5 s h - h*, e_a.
function probe_mc_apreg_u()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'test_results', 'temp_1009_ws');
    CASES = {'plane', '', 1.078; 'cell', '_ad060', 1.654; 'cell', '_ad055', 1.334};
    ARMS = {'state', 'a'' st: a law + cap 0.1'; 'stateu050', 'a'' st: u law 0.5'; 'stateu100', 'a'' st: u law 1.0'; ...
            'stateuslope100', 'a'' st u1.0, true slope'; 'stateua100', 'a'' st u1.0, true a'; 'wsu050', 'w_s st: u law 0.5'; 'wsu100', 'w_s st: u law 1.0'};
    fprintf('case        | arm                        | contact | t(h<h*+0.1) mean | min h  | h - h* mean (sd)  | e_a mean (sd)\n');
    for ic = 1:size(CASES, 1)
        for ia = 1:size(ARMS, 1)
            fn = fullfile(od, sprintf('apreg_%s_%s%s.mat', ARMS{ia, 1}, CASES{ic, 1}, CASES{ic, 2}));  if ~exist(fn, 'file'); continue; end
            S = load(fn);  HS = CASES{ic, 3};
            L = S.t > 5.5;  ok = find(~S.contact);  t1 = NaN(1, numel(ok));  dh = t1;  ea = t1;  mh = t1;
            for j = 1:numel(ok)
                q = ok(j);  h = S.h_true(:, q);  i1 = find(h < HS + 0.1, 1);  if ~isempty(i1); t1(j) = S.t(i1); end
                dh(j) = mean(h(L)) - HS;  ea(j) = mean(S.a_true(L, q) - S.a_hat(L, q));  mh(j) = min(h);
            end
            fprintf('%-5s %.2f  | %-26s | %d/5     | %5.2f s          | %.3f  | %+.3f (%.3f)   | %+.4f (%.4f)\n', CASES{ic, 1}, S.a_d, ARMS{ia, 2}, ...
                    nnz(S.contact), mean(t1, 'omitnan'), min(mh), mean(dh), std(dh), mean(ea), std(ea));
        end
    end
end
