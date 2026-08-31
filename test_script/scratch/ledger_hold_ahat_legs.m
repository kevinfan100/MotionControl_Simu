function out = ledger_hold_ahat_legs(seeds, arms)
%LEDGER_HOLD_AHAT_LEGS  Where does a_hat's +23 % come from in the end hold?
%   Split the posterior a_hat increment over a segment into its three legs:
%       law leg   sum_k a'(a_hat, b_hat) * [Delta_w_d + (1 - lambda_c) * dx_hat3 / R]   (predict; Delta_w_d = 0 in a hold)
%       y1 leg    sum_k K1(4) * innov_y1
%       y2 leg    sum_k K2(4) * innov_y2
%   residual = observed - (law + y1 + y2)  ->  MA(2) memory feedthrough + clamps.
%   Identity check first (rule 13), attribution second.
%   Also prints the quadrature-ratchet candidate  sum_k 2 b^2 (1-a)^3 dw^2 = -sum a'' dw^2
%   (Euler / left-endpoint error of the law integral, memory 2026-08-06 / 08-11), against the
%   observed drift -- a [measured] number, not an explanation.
%
%   out = ledger_hold_ahat_legs(1:8);
% STATUS: ACTIVE | closure check C follow-up (2026-08-30)

    if nargin < 1 || isempty(seeds); seeds = 1:8; end
    if nargin < 2 || isempty(arms); arms = [1 2]; end
    ax = 3;  ns = numel(seeds);
    pc = physical_constants();  R = pc.R;  lam = 0.7;
    ARMS = {'best', struct('arm','best'); 'best+lawq', struct('arm','best','law_err_q',true)};
    ARMS = ARMS(arms, :);
    out = struct();
    for a = 1:size(ARMS,1)
        o = ARMS{a,2};  o.ap_src = 'post';  o.seeds = seeds;  o.verbose = false;
        O = run_formC_b(o);
        G = @(f) cell2mat(reshape(cellfun(@(r) r.(f)(:, ax), O.runs, 'UniformOutput', false), 1, []));
        a_nom = O.runs{1}.a_nom;  t = O.runs{1}.tout(:);
        ah = G('a_bar_hat_out');  at = G('a_true_out')/a_nom;  bh = G('b_hat_out');
        dx3 = G('delta_x_hat_3_out')/R;                 % [R]
        hd = O.runs{1}.h_bar_d_out(:);  dwd = [0; diff(hd)];   % Delta_wbar_d used by predict at step k = w_d[k]-w_d[k-1]
        K1 = G('K_a_y1_out'); n1 = G('innov_y1_out'); K2 = G('K_a_y2_out'); n2 = G('innov_y2_out');
        SEG = {'oscillate', t > 1.60 & t < 3.40; 'hold end', t > 3.70};
        fprintf('\n=== %s, z, %d seeds (a_o units; %% = of trough a_true) ===\n', ARMS{a,1}, ns);
        fprintf('%-10s %9s %9s | %9s %9s %9s %9s | %9s %9s\n', 'segment', 'd a_hat', 'd a_true', 'law leg', 'y1 leg', 'y2 leg', 'resid', 'ratchet', 'sum dw^2');
        for g = 1:2
            idx = find(SEG{g,2});  k0 = idx(1);  k1 = idx(end);
            kk = (k0+1):k1;
            dw   = dwd(kk) + (1 - lam) * dx3(kk-1, :);          % predict increment [R]: commanded + tracking-error pull
            ap   = bh(kk-1, :) .* (1 - ah(kk-1, :)).^2;         % a'(a_hat[k-1]) in the law
            law  = sum(ap .* dw, 1);
            y1   = sum(K1(kk, :) .* n1(kk, :), 1);
            y2   = sum(K2(kk, :) .* n2(kk, :), 1);
            dobs = ah(k1, :) - ah(k0, :);  dtru = at(k1, :) - at(k0, :);
            res  = dobs - (law + y1 + y2);
            rat  = sum(2 * bh(kk-1,:).^2 .* (1 - ah(kk-1,:)).^3 .* dw.^2, 1);   % -a'' dw^2 summed: left-endpoint (Euler) quadrature error of the law
            aT   = mean(at(idx, :), 'all');
            M = @(v) mean(v);  S = @(v) std(v)/sqrt(ns);
            fprintf('%-10s %+9.4f %+9.4f | %+9.4f %+9.4f %+9.4f %+9.4f | %+9.4f %9.2e\n', SEG{g,1}, ...
                    M(dobs), M(dtru), M(law), M(y1), M(y2), M(res), M(rat), M(sum(dw.^2,1)));
            fprintf('%-10s %9.4f %9.4f | %9.4f %9.4f %9.4f %9.4f | %9.4f   (SEM)\n', '', ...
                    S(dobs), S(dtru), S(law), S(y1), S(y2), S(res), S(rat));
            fprintf('%-10s %+8.1f%% %+8.1f%% | %+8.1f%% %+8.1f%% %+8.1f%% %+8.1f%% | %+8.1f%%   (of trough a_true %.4f)\n', '', ...
                    100*[M(dobs) M(dtru) M(law) M(y1) M(y2) M(res) M(rat)]/aT, aT);
            out.(matlab.lang.makeValidName([ARMS{a,1} '_' SEG{g,1}])) = struct('dobs',dobs,'dtru',dtru,'law',law,'y1',y1,'y2',y2,'res',res,'rat',rat);
        end
    end
end
