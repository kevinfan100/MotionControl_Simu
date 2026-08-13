function check_formB_echo_coefficient()
%CHECK_FORMB_ECHO_COEFFICIENT  Audit the y2 self-echo factor (1-S) of formB/formC.
%   FORK OF model/controller/motion_control_law_formB_ws.m @ 2f2fef6 (init block
%   :356-380 and the per-step blend :1068) | PURPOSE: decide whether the echo
%   factor that multiplies H2 must use the VARIANCE-level S (code: divisor
%   a_bar + xi_bar) or the READOUT-level S (divisor a_bar) | EXPIRES: when the
%   coefficient question is settled in formB_ws.tex | 產線改動不會自動跟上
%
%   The derivation defines (formB_ws.tex:509-527)
%       S := dln sigma^2_dwr / dln a_hat_ctrl  =>  S = (S_T*a + S_n*xi)/(a + xi)
%   and applies (1-S) to H2. But H2 needs the sensitivity of the READOUT
%       a_wm = (sigma^2_dwr - C_n*sigma2_nw) / (C_dpmr*kappa_T)
%   to the TRUE gain at fixed applied gain. The subtracted noise floor does not
%   change the derivative but does change the level it is divided by.
%
%   This script recomputes v_T(g), v_n(g) from the SAME Lyapunov model the
%   controller uses, then finite-differences the readout in both arguments
%   INDEPENDENTLY (no chain rule), and compares against the two candidates.
%
%   Structural check available for free: scaling a_true and a_hat together
%   leaves g untouched, so the two sensitivities must sum to exactly 1.

    lambda_c = 0.7;  a_pd = 0.05;
    C_dpmr = 3.160954;  C_n = 1.109329;
    kappa_T = 4.97555e-05;
    sigma2_nw = [7.59309e-08; 6.41778e-08; 2.16417e-06];   % x, y, z  [-]
    xi_bar    = (C_n / C_dpmr) * sigma2_nw / kappa_T;

    fprintf('=== 0. Lyapunov model reproduces the controller constants ===\n');
    fprintf('v_T(g=1) = %.6f   vs C_dpmr = %.6f   (rel %.2e)\n', ...
            vg(1, 1, lambda_c, a_pd), C_dpmr, abs(vg(1,1,lambda_c,a_pd)-C_dpmr)/C_dpmr);
    fprintf('v_n(g=1) = %.6f   vs C_n     = %.6f   (rel %.2e)\n', ...
            vg(1, 2, lambda_c, a_pd), C_n, abs(vg(1,2,lambda_c,a_pd)-C_n)/C_n);

    ep = 1e-4;
    S_T = (log(vg(1/(1+ep),1,lambda_c,a_pd)) - log(vg(1/(1-ep),1,lambda_c,a_pd))) / (2*ep);
    S_n = (log(vg(1/(1+ep),2,lambda_c,a_pd)) - log(vg(1/(1-ep),2,lambda_c,a_pd))) / (2*ep);
    fprintf('S_T = %+.6f   S_n = %+.6f   (controller init: +0.328123 / -0.243727)\n\n', S_T, S_n);

    fprintf('=== 1. Direct finite difference of the READOUT (no chain rule) ===\n');
    fprintf('%-4s %-7s %-9s | %-10s %-10s | %-10s %-10s %-10s\n', ...
            'ax', 'a_bar', 'xi/a', 'dY/da_true', 'dY/da_hat', 'sum', '1-S_code', '1-S_read');
    for ax = [1 3]
        for a0 = [0.95 0.47]
            xi = xi_bar(ax);  s2n = sigma2_nw(ax);
            % readout as an explicit function of (a_true, a_hat)
            Y = @(at, ah) ( vg(at/ah, 1, lambda_c, a_pd) * kappa_T * at ...
                          + vg(at/ah, 2, lambda_c, a_pd) * s2n ...
                          - C_n * s2n ) / (C_dpmr * kappa_T);
            h = 1e-6 * a0;
            dY_dat = (Y(a0 + h, a0) - Y(a0 - h, a0)) / (2*h);   % H2 needs THIS
            dY_dah = (Y(a0, a0 + h) - Y(a0, a0 - h)) / (2*h);
            S_code = (S_T*a0 + S_n*xi) / (a0 + xi);             % controller :1068
            S_read = (S_T*a0 + S_n*xi) / a0;                    % candidate fix
            fprintf('%-4d %-7.2f %-9.4f | %-10.6f %-10.6f | %-10.8f %-10.6f %-10.6f\n', ...
                    ax, a0, xi/a0, dY_dat, dY_dah, dY_dat + dY_dah, 1 - S_code, 1 - S_read);
        end
    end

    fprintf('\n=== 2. Unbiasedness at g=1 (readout must return a_bar exactly) ===\n');
    for ax = [1 3]
        a0 = 0.95;  s2n = sigma2_nw(ax);
        Y0 = (vg(1,1,lambda_c,a_pd)*kappa_T*a0 + vg(1,2,lambda_c,a_pd)*s2n - C_n*s2n) ...
             / (C_dpmr*kappa_T);
        fprintf('ax=%d  a_wm(g=1) - a_bar = %+.3e\n', ax, Y0 - a0);
    end

    fprintf('\n=== 3. Filter-level consequence on the z axis ===\n');
    for a0 = [0.95 0.47]
        xi = xi_bar(3);
        S_code = (S_T*a0 + S_n*xi)/(a0 + xi);
        S_read = (S_T*a0 + S_n*xi)/a0;
        fprintf('a_bar=%.2f : (1-S) code %.5f vs correct %.5f  -> H2 high by %.2f%%, y2 info high by %.2f%%\n', ...
                a0, 1-S_code, 1-S_read, ...
                100*((1-S_code)/(1-S_read) - 1), 100*(((1-S_code)/(1-S_read))^2 - 1));
    end
end


function v = vg(gE, iN, lambda_c, a_pd)
%VG  Residual-variance gain of the mismatched loop, per unit driving variance.
%   iN = 1 thermal (kick into position), iN = 2 sensor noise (enters through the
%   control action plus a direct feedthrough). Verbatim structure of
%   motion_control_law_formB_ws.m:360-377 with g left free.
    alE = 1 - lambda_c;
    AE = zeros(6); BqE = zeros(6,1); BnE = zeros(6,1);
    AE(1,1)=1; AE(1,3)=-gE*alE; AE(1,4)=-gE*alE; AE(1,5)=-gE*alE;
    BnE(1)=-gE*alE; BqE(1)=1;
    AE(2,1)=1; AE(3,2)=1;
    AE(4,3)=-alE; AE(4,4)=-alE; AE(4,5)=-alE; BnE(4)=-alE;
    AE(5,4)=1;
    AE(6,3)=a_pd; AE(6,6)=1-a_pd; BnE(6)=a_pd;
    if iN == 1
        QE = BqE*BqE.';  extraE = 0;
    else
        QE = BnE*BnE.';  extraE = (1-a_pd)^2;
    end
    XE = reshape((eye(36) - kron(AE,AE)) \ QE(:), 6, 6);
    cE = zeros(1,6); cE(3) = 1-a_pd; cE(6) = -(1-a_pd);
    v = cE*XE*cE.' + extraE;
end
