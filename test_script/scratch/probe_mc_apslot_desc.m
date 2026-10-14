% STATUS: SCRATCH -- 2026-10-14: metrics for run_mc_apslot_compare(..., 'desc') (constant-velocity 22.2 -> 1.10 R in 1 s, hold 0.2 s),
%   5-seed mean |e|, e = a_true - a_hat, windows h_d > 6 / 6-2 / 2-1.1 R (t <= 1 s) and hold (t > 1 s); score = mean of the 8 numbers
%   (both walls). Also the signed hold mean. Sorted by score.
function probe_mc_apslot_desc(arms)
    od = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'test_results', 'temp_1009_ws');
    if nargin < 1 || isempty(arms)
        arms = {'s8'};  for pr = [2 5 20 50]; for pf = [5 10 20 50]; arms{end+1} = sprintf('apP%03dF%03d', pr, pf); end; end
    end
    WL = {'plane', 'cell'};  M = zeros(numel(arms), 8);  Hs = zeros(numel(arms), 2);
    for ia = 1:numel(arms)
        for iw = 1:2
            S = load(fullfile(od, sprintf('apcmp_%s_%s_desc.mat', arms{ia}, WL{iw})));  t = S.t;  e = S.a_true - S.a_hat;
            wd = mean(S.wd, 2);  wd(1) = wd(2);  m = mean(abs(e), 2);  d = t <= 1;
            M(ia, (iw - 1) * 4 + (1:4)) = [mean(m(d & wd > 6)), mean(m(d & wd <= 6 & wd > 2)), mean(m(d & wd <= 2)), mean(m(t > 1))];
            Hs(ia, iw) = mean(mean(e(t > 1, :)));
        end
    end
    sc = mean(M, 2);  [~, o] = sort(sc);
    fprintf('arm         | plane: >6R  6-2R  2-1.1R  hold | cell: >6R  6-2R  2-1.1R  hold | score | hold signed plane / cell\n');
    for i = o'
        fprintf('%-11s | %.4f %.4f %.4f %.4f | %.4f %.4f %.4f %.4f | %.4f | %+.4f / %+.4f\n', arms{i}, M(i, :), sc(i), Hs(i, :));
    end
end
