% STATUS: SCRATCH -- 2026-10-14: metrics for run_mc_apslot_compare (5-seed means). e = a_true - a_hat.
%   troughs t = 1, 1.5, 2, 2.5, 3 s and tops 1.25 .. 2.75 s (+-5 ms, signed mean); mean |e| on the descent windows h_d > 6 / 6-2 / 2-1.1 R
%   (t <= 1 s) and on the triangle (1-3 s); seed sd of the triangle mean |e|.
function probe_mc_apslot_compare(arms, plants)
    if nargin < 1 || isempty(arms); arms = {'conly', 's8', 'ap05', 'ap20'}; end
    if nargin < 2 || isempty(plants); plants = {'plane', 'cell'}; end
    od = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'test_results', 'temp_1009_ws');
    for ip = 1:numel(plants)
        fprintf('\n[%s] arm   | descent |e| >6 / 6-2 / 2-1.1 R | troughs 1 1.5 2 2.5 3 s       | tops 1.25 1.75 2.25 2.75 s  | triangle |e| (seed sd)\n', plants{ip});
        for ia = 1:numel(arms)
            S = load(fullfile(od, sprintf('apcmp_%s_%s.mat', arms{ia}, plants{ip})));  t = S.t;  e = S.a_true - S.a_hat;  wd = S.wd;  wd(1, :) = wd(2, :);
            w = @(tq) mean(mean(e(abs(t - tq) <= 0.005, :)));
            d = t <= 1;  m = mean(abs(e), 2);  wdm = mean(wd, 2);
            ds = [mean(m(d & wdm > 6)), mean(m(d & wdm <= 6 & wdm > 2)), mean(m(d & wdm <= 2))];
            tri = t > 1;  ts = mean(abs(e(tri, :)), 1);
            fprintf('  %-6s| %.4f %.4f %.4f            | %s| %s| %.4f (%.4f)\n', arms{ia}, ds, sprintf('%+.3f ', arrayfun(w, 1:0.5:3)), ...
                    sprintf('%+.3f ', arrayfun(w, 1.25:0.5:2.75)), mean(ts), std(ts));
        end
    end
end
