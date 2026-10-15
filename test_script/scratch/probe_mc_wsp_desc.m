% STATUS: SCRATCH -- 2026-10-15 (user): does freeing the exponent (a' = 1/(h - w_s)^p) bring w_s-hat closer to the PHYSICAL wall
%   (plane 0, cell -1.05)? Constant-velocity descent 22.2 -> 1.10 R in 1 s, hold 0.2 s (run_mc_apslot_compare 'desc'), 5 seeds.
%   Prints w_s-hat (mean +- seed sd) when h_d first reaches 6 / 3 / 2 / 1.5 / 1.1 R and at the end, p-hat at the end, mean |e_a| windows.
function probe_mc_wsp_desc(arms)
    if nargin < 1; arms = {'s8fc', 'f4fc'}; end
    od = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'test_results', 'temp_1009_ws');
    WL = {'plane', 'cell'};  WS = [0, -1.05];  HQ = [6 3 2 1.5 1.1];
    for iw = 1:2
        fprintf('\n[%s] physical w_s = %+.2f | w_s-hat at h_d = 6 / 3 / 2 / 1.5 / 1.1 R / end (mean +- sd) | p-hat end | |e_a| >6 6-2 2-1.1 hold\n', WL{iw}, WS(iw));
        for ia = 1:numel(arms)
            S = load(fullfile(od, sprintf('apcmp_%s_%s_desc.mat', arms{ia}, WL{iw})));  t = S.t;  wd = mean(S.wd, 2);  wd(1) = wd(2);
            ix = [arrayfun(@(h) find(wd <= h & t > 0.01, 1), HQ), numel(t)];
            ws = arrayfun(@(i) sprintf('%+.3f+-%.3f', mean(S.w_hat(i, :)), std(S.w_hat(i, :))), ix, 'UniformOutput', false);
            m = mean(abs(S.a_true - S.a_hat), 2);  d = t <= 1;
            ea = [mean(m(d & wd > 6)), mean(m(d & wd <= 6 & wd > 2)), mean(m(d & wd <= 2)), mean(m(t > 1))];
            fprintf('  %-5s %s | %.3f+-%.3f | %.4f %.4f %.4f %.4f\n', arms{ia}, strjoin(ws, ' '), mean(S.p_hat(end, :)), std(S.p_hat(end, :)), ea);
        end
    end
end
