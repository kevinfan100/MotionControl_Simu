% STATUS: SCRATCH -- 2026-10-14 (user: explain Fig 9.5): is the end gain error e_a = a - a_hat of each w_s-hat start the slope error
%   integrated along the descent?  C(t) = sum_k (a'_true(h[k]) - a'_hat[k]) * (h_d[k+1] - h_d[k]),  a'_hat = 1/(h_d - w_s-hat)^2 (S8),
%   a'_true = slope of the plant at the true height. Prints e_a and C at 0.1 / 0.3 / 1 / 3 / 6 s, and the descent share h_d > 2 R.
%   Data: run_mc_s9_gainreg files gainreg_<plant>_tau100<init tag>_T6<sfx>.mat.
function probe_mc_s9_slope_integral(plant, sfx)
    if nargin < 1; plant = 'cell'; end
    if nargin < 2; sfx = '_ad520_fc'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'test_results', 'temp_1009_ws');  WT = {'_wm3', '', '_wp3'};  W0 = [-3 0 3];  TQ = [0.1 0.3 1 3 6];
    fprintf('[%s%s] e_a = a - a_hat  vs  C = cumulative (a''_true - a''_hat) dh_d   at t = %s s\n', plant, sfx, sprintf('%g ', TQ));
    for iw = 1:3
        S = load(fullfile(od, sprintf('gainreg_%s_tau100%s_T6%s.mat', plant, WT{iw}, sfx)));  t = S.t;
        wd = S.wd;  wd(1) = wd(2);  hb = max(S.h_true, 1.001);
        apt = local_ap(hb, plant);  aph = 1 ./ max(wd - S.w_hat, 0.5).^2;
        dh = [diff(wd); 0];  C = cumsum((apt - aph) .* dh);  e = S.a_true - S.a_hat;  e = e - e(2);  % both start at 0 (a_hat seeded at the plant value at 22.2 R)
        ix = arrayfun(@(tq) find(t >= min(tq, t(end)), 1), TQ);
        far = find(wd <= 2, 1);
        fprintf('  w_s-hat[0] %+d | e_a %s | C %s | C at h_d = 2 R %+.4f (t %.2f s)\n', W0(iw), sprintf('%+.4f ', e(ix)), sprintf('%+.4f ', C(ix)), C(far), t(far));
    end
end

function ap = local_ap(h, plant)
    d = 1e-4;  ap = (local_a(h + d, plant) - local_a(h - d, plant)) / (2 * d);
end

function a = local_a(h, plant)
    if strcmp(plant, 'cell'); a = 1 ./ cell_cperp_delta(h); return; end
    a = zeros(size(h));  for i = 1:numel(h); [~, c] = calc_correction_functions(h(i)); a(i) = 1 / c; end
end
