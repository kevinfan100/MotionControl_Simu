% PURPOSE (2026-10-07, branch test/conly-meng-law): instrument check of calc_readout_consts_kfloop.m before it is wired in.
%   (1) K1 = 1: the KF-closed loop collapses to the raw-measurement law, so C_T / C_N / IF sums / echo S must reproduce
%       build_eq17_6state_constants (C_dpmr, C_n, IF_abc, IF_abc_white, y2_mirror) and the formC_b init echo at the same
%       (lambda_c, a_pd, a_cov) -- rel. diff < 1e-6.
%   (2) the no-LP closed form C_T = 2 + 1/(1 - lc^2), C_N = 1 + K1 is covered by check_conly_meng_law.py (S7 numeric block);
%       a_pd -> 0 leaves the LP state marginal here (singular Stein), so it is not repeated with this tool.
%   (3) print the table at the house point (lc 0.7, a_pd = a_cov 0.05) for K1 of the z axis far field / 2 R / valley.
function check_readout_consts_kfloop()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    lc = 0.7;  apd = 0.05;  a_cov = 0.05;  d = 2;
    pc = physical_constants();  kBT = pc.k_B * pc.T;  V = {'FAIL', 'PASS'};
    % --- (1) K1 = 1 against the production constants
    o = struct('lambda_c', lc, 'sigma2_n_s', (0.00331)^2 * ones(3, 1), 'kBT', kBT, 'a_cov', a_cov, 'a_pd', apd, 'd', d);
    cc = build_eq17_6state_constants(o);
    T1 = calc_readout_consts_kfloop(lc, apd, a_cov, d, 1.0);
    rel = @(x, y) abs(x - y) ./ max(abs(y), 1e-300);
    chk = {'C_T = C_dpmr',       T1.C_T,              cc.C_dpmr; ...
           'C_N = C_n',          T1.C_N,              cc.C_n; ...
           'IF_abc(1)',          T1.IF_abc(1),        cc.IF_abc(1); ...
           'IF_abc(2)',          T1.IF_abc(2),        cc.IF_abc(2); ...
           'IF_abc(3)',          T1.IF_abc(3),        cc.IF_abc(3); ...
           'IF_abc_white(1)',    T1.IF_abc_white(1),  cc.IF_abc_white(1); ...
           'IF_abc_white(2)',    T1.IF_abc_white(2),  cc.IF_abc_white(2); ...
           'IF_abc_white(3)',    T1.IF_abc_white(3),  cc.IF_abc_white(3)};
    npass = 0;
    for i = 1:size(chk, 1)
        r = rel(chk{i, 2}, chk{i, 3});  ok = r < 1e-6;  npass = npass + ok;
        fprintf('  %-4s %-18s kfloop %.8g  production %.8g  rel %.1e\n', V{ok + 1}, chk{i, 1}, chk{i, 2}, chk{i, 3}, r);
    end
    % echo S against the formC_b init block (same formula, raw law)
    [S_T_old, S_n_old] = local_echo_old(lc, apd);
    for pair = {{'S_T', T1.S_T, S_T_old}, {'S_n', T1.S_n, S_n_old}}
        p = pair{1};  r = rel(p{2}, p{3});  ok = r < 1e-6;  npass = npass + ok;
        fprintf('  %-4s %-18s kfloop %.8g  formC_b init %.8g  rel %.1e\n', V{ok + 1}, p{1}, p{2}, p{3}, r);
    end
    fprintf('%d / %d checks pass\n', npass, size(chk, 1) + 2);
    % --- (3) house table
    kappa_T = 4 * kBT * (pc.Ts / (pc.gamma_N * pc.R)) / pc.R;   % normalization-boundary.md U0
    r = (0.00331 / pc.R)^2;
    T = calc_readout_consts_kfloop(lc, apd, a_cov, d);
    fprintf('\n  house point lc %.1f a_pd %.2f a_cov %.2f, z axis sigma_n 3.31 nm, kappa_T %.3e, r %.3e\n', lc, apd, a_cov, kappa_T, r);
    fprintf('  %-10s %-6s %-8s %-8s %-8s %-8s %-8s\n', 'a_bar', 'q/r', 'K1', 'C_T', 'C_N', 'S_T', 'S_n');
    for ab = [1.0, 0.35, 0.087]
        q = kappa_T * ab;  K1 = T.K1_of(q, r);
        fprintf('  %-10.3f %-6.1f %-8.4f %-8.4f %-8.4f %-8.4f %-8.4f\n', ab, q / r, K1, ...
                interp1(T.K1, T.C_T, K1), interp1(T.K1, T.C_N, K1), interp1(T.K1, T.S_T, K1), interp1(T.K1, T.S_n, K1));
    end
    fprintf('  production constants: C_dpmr %.4f C_n %.4f S_T %.4f S_n %.4f\n', cc.C_dpmr, cc.C_n, S_T_old, S_n_old);
end

function [S_echo_T, S_echo_n] = local_echo_old(lambda_c, a_pd)
% verbatim of motion_control_law_formC_b.m init (y2_echo_corr block), raw-measurement law
    alE = 1 - lambda_c; epE = 1e-4;
    vE = zeros(2, 3); gE_list = [1, 1/(1+epE), 1/(1-epE)];
    for iN = 1:2
        for iG = 1:3
            gE = gE_list(iG);
            AE = zeros(6); BqE = zeros(6,1); BnE = zeros(6,1);
            AE(1,1)=1; AE(1,3)=-gE*alE; AE(1,4)=-gE*alE; AE(1,5)=-gE*alE;
            BnE(1)=-gE*alE; BqE(1)=1;
            AE(2,1)=1; AE(3,2)=1;
            AE(4,3)=-alE; AE(4,4)=-alE; AE(4,5)=-alE; BnE(4)=-alE;
            AE(5,4)=1;
            AE(6,3)=a_pd; AE(6,6)=1-a_pd; BnE(6)=a_pd;
            if iN == 1; QE = BqE*BqE.'; extraE = 0;
            else;       QE = BnE*BnE.'; extraE = (1-a_pd)^2; end
            XE = reshape((eye(36) - kron(AE,AE)) \ QE(:), 6, 6);
            cE = zeros(1,6); cE(3) = 1-a_pd; cE(6) = -(1-a_pd);
            vE(iN,iG) = cE*XE*cE.' + extraE;
        end
    end
    S_echo_T = (log(vE(1,2)) - log(vE(1,3))) / (2*epE);
    S_echo_n = (log(vE(2,2)) - log(vE(2,3))) / (2*epE);
end
