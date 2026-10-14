% STATUS: SCRATCH -- 2026-10-14 (user-approved test): criterion 2 for formC_b fe_cmd_eval -- does the motion phase get worse?
%   Section 8 setting (22.2 -> 1.10 R in 1 s, hold 0.2 s, w_s-hat[0] = -1, prior 0.3, 10 seeds; run_1009_wsonly), base '_sw030' vs '_sw030fc'.
%   Per wall: 10-seed mean |e_a| = |a_true - a_hat| and |a'-hat - a'| in the descent windows h_d > 6 / 6-2 / 2-1.1 R, and the 0.2 s hold.
%   Pass = no window worse than base by more than 10 %.
function probe_mc_fecmd_motion()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(genpath(fullfile(root, 'model')));  addpath(here);
    od = fullfile(root, 'test_results', 'temp_1009_ws');  WL = {'plane', 'cell'};  PC = {[], @cell_cperp_delta};  AR = {'_sw030', '_sw030fc'};
    for iw = 1:2
        fprintf('\n[%s] mean |e_a|: >6R  6-2R  2-1.1R  hold | mean |a''hat - a''|: >6R  6-2R  2-1.1R\n', WL{iw});
        M = zeros(2, 7);
        for ia = 1:2
            S = load(fullfile(od, sprintf('wsonly_%s_m1%s.mat', WL{iw}, AR{ia})));  t = S.t;  wd = S.wd;  wd(1) = wd(2);  dsc = t <= 1;
            ea = mean(abs(S.a_true - S.a_hat), 2);
            hb = max(S.h_true, 1.001);  ap = local_ap(hb, PC{iw});  ea1 = mean(abs(1 ./ (wd - S.w_hat).^2 - ap), 2);
            W = {dsc & wd > 6, dsc & wd <= 6 & wd > 2, dsc & wd <= 2, t > 1};
            M(ia, :) = [cellfun(@(m) mean(ea(m)), W), cellfun(@(m) mean(ea1(m)), W(1:3))];
            fprintf('  %-10s %.4f %.4f %.4f %.4f | %.5f %.4f %.4f\n', AR{ia}, M(ia, :));
        end
        fprintf('  ratio fc/base %s  -> %s\n', sprintf('%.2f ', M(2, :) ./ M(1, :)), local_v(all(M(2, :) <= 1.10 * M(1, :))));
    end
end

function s = local_v(ok)
    s = 'FAIL';  if ok; s = 'PASS'; end
end

function ap = local_ap(h, pcp)
    d = 1e-4;  ap = (local_a(h + d, pcp) - local_a(h - d, pcp)) / (2 * d);
end

function a = local_a(h, pcp)
    h = max(h, 1.001);
    if isempty(pcp); a = zeros(size(h));  for i = 1:numel(h); [~, c] = calc_correction_functions(h(i)); a(i) = 1 / c; end
    else; a = 1 ./ pcp(h); end
end
