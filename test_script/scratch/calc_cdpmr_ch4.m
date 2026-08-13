% STATUS: ACTIVE (scratch) | PURPOSE: derive the variance-channel calibration
%   constants for the Ch4 law (4.4)+estimator closed loop, by exact stationary
%   Lyapunov of the 10-state loop. The journal reuses the (3.6)-loop formula
%   (11) for the (4.4) system (cites [19]); measured on Scenario 0 this
%   miscalibration reads a_xm high by ~1.9x. Instrument is validated first on
%   the (3.6) loop against the known closed forms.
%   Var(dx_m) = C_dpmr * (4kBT a) + C_n * sigma_n^2  (per axis)
% EXPIRES: Scenario-0 adjudication (spec: meng_ch4_spec_ledger.md sections 8-9)
function out = calc_cdpmr_ch4(lc, lambda_f, ratio)
    if nargin < 1 || isempty(lc);       lc = 0.4;   end
    if nargin < 2 || isempty(lambda_f); lambda_f = 0.98; end
    if nargin < 3 || isempty(ratio);    ratio = 0.35;   end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    pc = physical_constants(); kBT = pc.k_B*pc.T; a_N = pc.Ts/pc.gamma_N;

    % ---- instrument validation: (3.6) loop vs closed forms ----
    fprintf('--- instrument validation: law (3.6) loop ---\n');
    for lcv = [0.4 0.7 1e-9]
        mu = 1-lcv;
        A = [1 0 -mu  mu  mu; 1 0 0 0 0; 0 1 0 0 0; 0 0 mu -mu -mu; 0 0 0 1 0];
        DT = lyap_ss(A, [-1;0;0;0;0]);  Dn = lyap_ss(A, [-mu;0;0;mu;0]);
        fprintf('lc=%.1g: C_dpmr=%.4f (closed %.4f)  C_n=%.4f (closed %.4f)\n', ...
            lcv, DT(3,3), 2+1/(1-lcv^2), Dn(3,3)+1, 2/(1+lcv));
    end

    % ---- (4.4) loop constants, per axis ----
    fprintf('\n--- law (4.4) loop, lc=%.2f lambda_f=%.2f ---\n', lc, lambda_f);
    mu = 1-lc;
    Fs = [0 1 0 0 0 0 0;0 0 1 0 0 0 0;0 0 lc 0 0 0 0;0 0 0 1 1 0 0; ...
          0 0 0 0 1 0 0;0 0 0 0 0 1 1;0 0 0 0 0 0 1];
    Ferr = Fs; Ferr(3,:) = [0 0 1 -1 0 0 0];
    H2 = [1 0 0 0 0 0 0; 0 0 0 0 0 1 -2];
    SN = [0.0007 0.0007 0.0023];
    cc = build_eq17_constants(struct('lambda_c',lc,'sigma2_n_s',SN(:).^2, ...
                                     'kBT',kBT,'t_warmup_kf',0));
    % rows: x, y, z at a_N; plus z at a_N/ratio (region-2 K dependence check)
    cases = [1 a_N; 2 a_N; 3 a_N; 3 a_N/ratio];
    out = struct('C_dpmr', zeros(4,1), 'C_n', zeros(4,1), 'y2leak', zeros(4,1));
    for i = 1:4
        ax = cases(i,1); a = cases(i,2); sn2 = SN(ax)^2;
        Q = zeros(7); Q(3,3) = 4*kBT*a;
        R22 = cc.R22_prefactor * cc.IF_eff * (a + cc.xi_per_axis(ax))^2;
        Rm = [sn2 0; 0 R22];
        P = eye(7);
        for it = 1:20000
            Pf = Ferr*P*Ferr' + Q; Pf = 0.5*(Pf+Pf');
            S = H2*Pf*H2' + Rm; K = (Pf*H2')/S;
            P = (eye(7)-K*H2)*Pf/lambda_f; P = 0.5*(P+P');
        end
        A = zeros(10);
        A(1,1)=1; A(1,6)=-mu; A(1,7)=1; A(2,1)=1; A(3,2)=1;
        A(4:10,4:10) = (eye(7)-K*H2)*Fs;
        A(4:10,3) = K(:,1);
        ST = lyap_ss(A, [-1; zeros(9,1)]);
        Sn = lyap_ss(A, [zeros(3,1); K(:,1)]);
        Sy = lyap_ss(A, [zeros(3,1); K(:,2)]);
        out.C_dpmr(i) = ST(3,3);
        out.C_n(i)    = Sn(3,3) + 1;
        out.y2leak(i) = Sy(3,3)*R22/sn2;
        fprintf('case %d (ax %d, a=%.5f, sn=%.1f nm): C_dpmr44=%.4f  C_n44=%.4f  y2leak=%.3g\n', ...
            i, ax, a, 1e3*SN(ax), out.C_dpmr(i), out.C_n(i), out.y2leak(i));
    end
    fprintf('cross-check: C_dpmr44/C_dpmr36 = %.3f (Scenario-0 measured sigma^2 inflation ~1.91)\n', ...
        out.C_dpmr(3)/(2+1/(1-lc^2)));
end

function S = lyap_ss(A, B)
    S = zeros(size(A));
    for it = 1:60000
        Sn = A*S*A' + B*B';
        if max(abs(Sn(:)-S(:))) < 1e-16*max(1,max(abs(Sn(:)))), S = Sn; return; end
        S = Sn;
    end
end
