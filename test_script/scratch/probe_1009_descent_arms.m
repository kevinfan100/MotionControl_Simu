% STATUS: SCRATCH -- 2026-10-13, descent-phase a-bar error of the unit-numerator w_s estimator (motion_ctrl S8) against its variants and
%   c-only, same descent (22.2 -> 1.10 R in 1 s, hold 0.2 s), 10 seeds, e_a = a_true - a_hat (user 10-13: "can it track faster / with less
%   error on the descent?"). Windows by commanded height: far h_d > 6, mid 6..2, near 2..1.10 (t <= 1 s), hold t > 1 s.
%   Columns: mean over seeds of mean |e_a| per window, the mean e_a (sign) in the near window, worst |seed-mean e_a| on the descent,
%   and the w_s-hat excursion (seed mean of the max w_s-hat before 1 s).
function probe_1009_descent_arms()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  od = fullfile(root, 'test_results', 'temp_1009_ws');
    A = { 'c-only (S6, sqrtPfix 0.15)', 'wallshift_p0', 'wallshift_m1.05'
          'S8 w_s, prior 1.0, start 0',  'wsonly_plane_p0', 'wsonly_cell_p0'
          'S8 w_s, prior 1.0, start -1', 'wsonly_plane_m1', 'wsonly_cell_m1'
          'A: prior 0.3, start 0',       'wsonly_plane_p0_sw030', 'wsonly_cell_p0_sw030'
          'A: prior 0.3, start -1',      'wsonly_plane_m1_sw030', 'wsonly_cell_m1_sw030'
          'B: numerator 9/8, start 0',   'wsonly_plane_p0_n98', 'wsonly_cell_p0_n98'
          'B: numerator 9/8, start -1',  'wsonly_plane_m1_n98', 'wsonly_cell_m1_n98' };
    WL = {'plane', 'cell'};
    for iw = 1:2
        fprintf('\n[%s]  %-30s | |e_a| desc   far    mid    near   hold | near mean e_a | worst | w_s-hat max | end w_s-hat\n', WL{iw}, 'arm');
        for ia = 1:size(A, 1)
            fn = fullfile(od, [A{ia, 1 + iw} '.mat']);  if ~exist(fn, 'file'); fprintf('  %-38s missing\n', A{ia, 1}); continue; end
            S = load(fn);  t = S.t;  wd = S.wd;  wd(1) = wd(2);  e = S.a_true - S.a_hat;  m = mean(e, 2);
            dsc = t <= 1;  far = dsc & wd > 6;  mid = dsc & wd <= 6 & wd > 2;  nr = dsc & wd <= 2;  hd = t > 1;
            g = @(msk) mean(mean(abs(e(msk, :)), 1));
            wmax = NaN;  wend = NaN;  if isfield(S, 'w_hat'); wmax = mean(max(S.w_hat(dsc, :), [], 1));  wend = mean(S.w_hat(end, :)); end
            fprintf('  %-38s | %.4f  %.4f %.4f %.4f %.4f | %+.4f        | %.3f | %+6.2f      | %+.3f\n', A{ia, 1}, g(dsc), g(far), g(mid), g(nr), g(hd), ...
                    mean(m(nr)), max(abs(m(dsc))), wmax, wend);
        end
    end
end
