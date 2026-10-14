% STATUS: SCRATCH -- 2026-10-13, why the gain regulation of motion_ctrl S9 is slower than designed (plane, seed 1).
%   Per step the law asks for a gain change (1-lambda_a)(a_d - a_hat); the plant delivers about a'(1-lambda_a)(a_d - a_hat)/a_hat'.
%   So the realized fraction of the designed rate = kappa * g, kappa = a'/a_hat' (slope), g = (a_d - a_hat)/(a_d - a) (gain estimate).
%   Measured fraction m = (ln e(t0) - ln e(t1)) / (-(steps) ln lambda_a), e = a - a_d smoothed. Windows in units of tau_a.
%   Data: run_mc_s9_gainreg.m, gainreg_plane_tau<ms>.mat.
function probe_mc_s9_slowdown(taus_ms)
    if nargin < 1; taus_ms = [100 300]; end
    here = fileparts(mfilename('fullpath'));  od = fullfile(fileparts(fileparts(here)), 'test_results', 'temp_1009_ws');
    W = [0 0.5; 0.5 1; 1 2; 2 4; 4 7];
    for tm = taus_ms
        S = load(fullfile(od, sprintf('gainreg_plane_tau%03d.mat', tm)));  t = S.t;  tau = S.tau_a;  nsm = 20;
        e = movmean(S.a_true - S.a_d, nsm);  eh = movmean(S.a_hat - S.a_d, nsm);  kap = S.ap_true ./ S.ap_hat;
        fprintf('\ntau_a = %.1f s (lambda_a %.5f)\n  window [tau] | t [s]       | a - a_d  | a_hat - a_d | g = (a_d-a_hat)/(a_d-a) | kappa = a''/a_hat'' | kappa*g | measured m | h - h_d [R]\n', tau, S.lambda_a);
        for i = 1:size(W, 1)
            ix = t >= W(i, 1) * tau & t < W(i, 2) * tau & t > 0.002;  i0 = find(ix, 1);  i1 = find(ix, 1, 'last');
            g = mean(eh(ix)) / mean(e(ix));  kp = median(kap(ix));
            m = (log(abs(e(i0))) - log(abs(e(i1)))) / (-(i1 - i0) * log(S.lambda_a));
            fprintf('  %3.1f-%3.1f      | %.3f-%.3f | %+.4f  | %+.4f     | %.3f                   | %.3f              | %.3f   | %.3f      | %+.4f\n', ...
                    W(i, :), t(i0), t(i1), mean(e(ix)), mean(eh(ix)), g, kp, kp * g, m, mean(S.h_true(ix) - S.wd(ix)));
        end
    end
end
