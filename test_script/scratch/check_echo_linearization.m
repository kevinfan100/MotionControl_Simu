% STATUS: ACTIVE (scratch) | PURPOSE: how large is the linearisation error of
%   the y2 echo model (H2 x (1-S), S from the 6-state mismatched-loop Lyapunov
%   at g = 1) once the loop runs at the near-wall gain ratio g = a/a_hat = 0.84
%   (ledger L21)? Also the bare C_dpmr(lambda) sensitivity for comparison.
%   Same AE/BqE/BnE matrices as motion_control_law_formC_b.m :425-445.
%   2026-08-26, memory project_lambda_eff_treatment_in_estimator.
function check_echo_linearization()
    lc = 0.7; apd = 0.05; al = 1 - lc; ep = 1e-4;
    names = {'thermal S_T', 'sensor  S_n'};
    for iN = 1:2
        S = (log(local_var(1/(1+ep), iN, al, apd)) ...
           - log(local_var(1/(1-ep), iN, al, apd))) / (2*ep);
        fprintf('%s  S = %+.4f\n', names{iN}, S);
        for g = [0.9 0.8388 0.8]
            r = local_var(g, iN, al, apd) / local_var(1, iN, al, apd);
            fprintf('  g=%.4f  lam_eff=%.4f  exact %.4f | g^-S %.4f | 1-S ln g %.4f\n', ...
                    g, 1 - g*al, r, g^(-S), 1 - S*log(g));
        end
    end
    cd0 = local_cdpmr(0.7, apd);
    for l = [0.7 0.748 0.76]
        fprintf('lam=%.3f  C_dpmr(closed,a_pd) %.4f ratio %.4f | bare 2+1/(1-l^2) ratio %.4f\n', ...
                l, local_cdpmr(l, apd), local_cdpmr(l, apd)/cd0, ...
                (2+1/(1-l^2)) / (2+1/(1-0.49)));
    end
end

function v = local_var(g, iN, al, apd)
    AE = zeros(6); BqE = zeros(6,1); BnE = zeros(6,1);
    AE(1,1)=1; AE(1,3)=-g*al; AE(1,4)=-g*al; AE(1,5)=-g*al; BnE(1)=-g*al; BqE(1)=1;
    AE(2,1)=1; AE(3,2)=1;
    AE(4,3)=-al; AE(4,4)=-al; AE(4,5)=-al; BnE(4)=-al;
    AE(5,4)=1;
    AE(6,3)=apd; AE(6,6)=1-apd; BnE(6)=apd;
    if iN == 1; QE = BqE*BqE.'; extra = 0;
    else;       QE = BnE*BnE.'; extra = (1-apd)^2; end
    XE = reshape((eye(36) - kron(AE,AE)) \ QE(:), 6, 6);
    cE = zeros(1,6); cE(3) = 1-apd; cE(6) = -(1-apd);
    v = cE*XE*cE.' + extra;
end

function c = local_cdpmr(l, a)                       % calc_ctrl_params.m :164-171
    oma = 1 - a; D = 1 - oma*l;
    c = oma^2 * (2*oma*(1-l)/D + 2/((2-a)*(1+l)*D));
end
