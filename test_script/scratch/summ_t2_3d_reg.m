% STATUS: SCRATCH -- 2026-10-20, per-seed summary of run_t2_3d_reg (user: all tests at a_d = 0.1). Motion first:
%   approach / regulation (0 - t_scan): contact, min h, time when |theta-hat_y - theta_y| < 2 deg for good, |a_N - a_d| at 6-7 s;
%   scan (t_scan - end): mean and range of h - h*, a_N-hat - a_N; a'-dips = time with a-hat'_N < a'_N / 5 (where the step jumps).
%   | run: summ_t2_3d_reg('ad010')
function summ_t2_3d_reg(tag)
    if nargin < 1; tag = 'ad010'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    S = load(fullfile(root, 'test_results', 'temp_3d_wsonly', ['temp_t2_' tag '.mat']));  o = S.R.opts;
    fprintf('T2 %s: wall %g deg, a_d %.2f, tau_a %.1f s, scan %.3f R/step from %.1f s\n', tag, o.theta_y, o.a_d, o.tau_a, o.du_d, o.t_scan);
    fprintf('seed | contact | min h  | theta_y<2deg from | end e_thy  e_thx | |a_N-a_d| 6-7 s | scan h-h* mean [min max] | scan aN-hat-aN | a''-dip time | angle on at\n');
    for ir = 1:numel(S.R.runs)                                  % cell loop: diverged runs carry an extra 'fail' field
        r = S.R.runs{ir};
        k = 1:r.kend;  t = r.t(k);  tl = NaN;  if isfield(r, 't_latch'); tl = r.t_latch; end
        bad = find(abs(r.thy(k) - o.theta_y) >= 2, 1, 'last');  tc = NaN;  if ~isempty(bad) && bad < numel(k); tc = t(bad + 1); end
        w67 = t >= 6 & t < 7;  ws = t >= o.t_scan;
        dh = r.h(k) - r.h_star;  dA = r.aN(k) - r.aN_true(k);
        dip = sum(r.apN(k) < r.apN_true(k) / 5 & t > 0.5) * (t(2) - t(1));
        if any(ws)
            fprintf('%4d | %7d | %.3f  | %6.2f s          | %+5.2f  %+5.2f  | %.4f          | %+.3f [%+.3f %+.3f]      | %+.4f        | %.3f s      | %.3f s\n', r.seed, r.contact, ...
                    min(r.h(k)), tc, o.theta_y - r.thy(r.kend), -r.thx(r.kend), mean(abs(r.aN_true(w67) - o.a_d)), mean(dh(ws)), min(dh(ws)), max(dh(ws)), mean(dA(ws)), dip, tl);
        else
            fprintf('%4d | %7d | %.3f  | contact at %.3f s\n', r.seed, r.contact, min(r.h(k)), r.t_contact);
        end
    end
end
