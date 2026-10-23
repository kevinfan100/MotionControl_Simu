% STATUS: ACTIVE (scratch) | PURPOSE: attribution ledger for the oracle-arm
%   residual bias, on the 100-seed compact files (arm100_btrue / arm100_aptrue).
%   The per-step change of a_bar_hat is exactly three terms:
%       predict (slope x displacement) + l41*e1 + l42*e2
%   All three are reconstructable from the logs. This script:
%     A. total bias vs time, 100-seed mean +/- SEM band (both arms)
%     B. update-term ledger: cumulative sum(l41*e1) and sum(l42*e2), seed mean
%     C. reference-jitter check: bias measured against a_true(true h) vs
%        a_true(cmd h) -- the "ruler" swap (hypothesis 3)
%     D. honesty profile sd(t)/sqrtP44(t), CORRECT normalization
%        (P_a_out stores sqrt(P) in physical units -> divide by ad per seed)
%   Residual of the ledger (A - B - C-explained part) = propagation-side
%   (hypotheses 1/2), to be split by the displacement rebuild if nonzero.
function out = ledger_arm100_bias()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    A = {'aptrue', 'btrue'}; out = struct();
    for a = 1:2
        D = load(fullfile(od, ['arm100_' A{a} '.mat']));
        t = D.t; nS = numel(D.seeds); N = numel(t);
        E  = D.a_bar_hat_out - D.a_true_norm;              % ruler = true height
        % --- C. ruler swap: a_true at the COMMANDED height ------------------
        % a_true(cmd)/ad: rebuild from c_perp at hd (deterministic, one column)
        wq = max(D.hd, 1.0005); cp = zeros(N,1);
        for k = 1:N; [~, cp(k)] = calc_correction_functions(wq(k)); end
        at_cmd = (1 ./ cp);                                 % a_bar_true(cmd)
        % normalize like a_true_norm: a_true_norm = a_true/ad, and
        % a_true = a_o / c_perp with a_o/ad = a_bar at t=1 far field.
        % Use per-seed far-field ratio so scales match exactly:
        sc = mean(D.a_true_norm(1:200,:), 1) ./ mean(at_cmd(1:200));  % 1 x nS
        Ecmd = D.a_bar_hat_out - at_cmd * sc;               % ruler = cmd height
        % --- B. update ledger ----------------------------------------------
        % l41*e1 is ALREADY in a_bar units (closure regression coeff 0.994,
        % 2026-08-31); do NOT divide by ad.
        U1n = cumsum(D.K_a_y1_out .* D.innov_y1_out, 1);    % y1 net push on a_bar_hat
        U2n = cumsum(D.K_a_y2_out .* D.innov_y2_out, 1);    % y2 net push
        % predict-side residual: E = PR + U1 + U2  (cumulative identity)
        PR  = E - U1n - U2n;                                % propagation minus truth
        % --- D. honesty (correct units) ------------------------------------
        sdE  = std(D.a_bar_hat_out, 0, 2);
        sPn  = mean(D.P_a_out ./ D.ad, 2);                  % sqrt(P44)/ad
        % --- windows --------------------------------------------------------
        m1 = t >= 7.5 & t <= 10.5; m2 = t > 10.5;
        r = struct();
        for w = {m1,'nearwall'; m2,'hold'}'
            m = w{1}; tag = w{2};
            r.(tag) = struct( ...
              'bias',      mean(E(m,:),'all'),   'bias_sem', std(mean(E(m,:),1))/sqrt(nS), ...
              'bias_cmd',  mean(Ecmd(m,:),'all'),'bias_cmd_sem', std(mean(Ecmd(m,:),1))/sqrt(nS), ...
              'u1',        mean(mean(U1n(m,:),1)), 'u1_sem', std(mean(U1n(m,:),1))/sqrt(nS), ...
              'u2',        mean(mean(U2n(m,:),1)), 'u2_sem', std(mean(U2n(m,:),1))/sqrt(nS), ...
              'pr',        mean(mean(PR(m,:),1)),  'pr_sem', std(mean(PR(m,:),1))/sqrt(nS), ...
              'sd',        mean(sdE(m)), 'sqrtP', mean(sPn(m)), 'honesty', mean(sdE(m))/mean(sPn(m)));
            fprintf('[%s] %-8s bias %+.5f (%.5f) | ruler-cmd %+.5f (%.5f) | PR %+.5f (%.5f) + U1 %+.5f (%.5f) + U2 %+.5f (%.5f) | sd %.5f sqrtP %.5f honesty %.2f\n', ...
              A{a}, tag, r.(tag).bias, r.(tag).bias_sem, r.(tag).bias_cmd, r.(tag).bias_cmd_sem, ...
              r.(tag).pr, r.(tag).pr_sem, r.(tag).u1, r.(tag).u1_sem, r.(tag).u2, r.(tag).u2_sem, ...
              r.(tag).sd, r.(tag).sqrtP, r.(tag).honesty);
        end
        out.(A{a}) = struct('t', t, 'E', E, 'Ecmd', Ecmd, 'U1', U1n, 'U2', U2n, 'PR', PR, ...
                            'sdE', sdE, 'sP', sPn, 'hd', D.hd, 'win', r);
        clear D E Ecmd U1n U2n PR;
    end
end
