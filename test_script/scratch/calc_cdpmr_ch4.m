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
    fprintf('--- instrument validation: law (3.6) loop (raw variance) ---\n');
    for lcv = [0.4 0.7 1e-9]
        mu = 1-lcv;
        A = [1 0 -mu  mu  mu; 1 0 0 0 0; 0 1 0 0 0; 0 0 mu -mu -mu; 0 0 0 1 0];
        DT = lyap_ss(A, [-1;0;0;0;0]);  Dn = lyap_ss(A, [-mu;0;0;mu;0]);
        fprintf('lc=%.1g: C_dpmr=%.4f (closed %.4f)  C_n=%.4f (closed %.4f)\n', ...
            lcv, DT(3,3), 2+1/(1-lcv^2), Dn(3,3)+1, 2/(1+lcv));
    end

    % ---- instrument validation 2: (3.6)+LP readout vs the PRODUCTION
    %      closed forms (calc_ctrl_params.m:150-181, a_pd low-pass composed) ----
    fprintf('--- instrument validation: (3.6)+LP readout vs production eff forms ---\n');
    alpha = 0.05;   % a_pd
    for lcv = [0.4 0.7]
        mu = 1-lcv;  oma = 1-alpha;  D0 = 1 - oma*lcv;
        Cd_closed = oma^2 * (2*oma*(1-lcv)/D0 + 2/((2-alpha)*(1+lcv)*D0));
        Cn_closed = oma^2 * (2/(2-alpha) + 2*oma^2*alpha*(1-lcv)/((2-alpha)*D0) ...
                             + 2*(1-lcv)^2/((2-alpha)*(1+lcv)*D0));
        % 6-state: [dx(j+1) dx(j) dx(j-1) af(j) af(j-1) m]
        A6 = [1 0 -mu  mu  mu 0; 1 0 0 0 0 0; 0 1 0 0 0 0; ...
              0 0 mu -mu -mu 0; 0 0 0 1 0 0; 0 0 alpha 0 0 oma];
        Bw = [-1;0;0;0;0;0];  Bn = [-mu;0;0;mu;0;alpha];
        Cv = oma * [0 0 1 0 0 -1];    % dx_r = (1-alpha)(s3 + n - m)
        ST = lyap_ss(A6, Bw);  Sn = lyap_ss(A6, Bn);
        Cd_l = Cv*ST*Cv';  Cn_l = Cv*Sn*Cv' + oma^2;
        fprintf('lc=%.1f: C_dpmr_eff=%.4f (closed %.4f)  C_n_eff=%.4f (closed %.4f)\n', ...
            lcv, Cd_l, Cd_closed, Cn_l, Cn_closed);
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
        A = zeros(11);
        A(1,1)=1; A(1,6)=-mu; A(1,7)=1; A(2,1)=1; A(3,2)=1;
        A(4:10,4:10) = (eye(7)-K*H2)*Fs;
        A(4:10,3) = K(:,1);
        alpha = cc.a_pd;  oma = 1-alpha;
        A(11,3) = alpha;  A(11,11) = oma;         % LP mean m of dx_m
        Bw = [-1; zeros(10,1)];
        Bn = [zeros(3,1); K(:,1); alpha];
        By = [zeros(3,1); K(:,2); 0];
        Cv = oma * [0 0 1 zeros(1,7) -1];          % dx_r = (1-a)(s3 + n - m)
        ST = lyap_ss(A, Bw);  Sn = lyap_ss(A, Bn);  Sy = lyap_ss(A, By);
        out.C_dpmr_raw(i) = ST(3,3);
        out.C_n_raw(i)    = Sn(3,3) + 1;
        out.C_dpmr(i) = Cv*ST*Cv';                 % v2: what the EWMA sees
        out.C_n(i)    = Cv*Sn*Cv' + oma^2;
        out.y2leak(i) = (Cv*Sy*Cv')*R22/sn2;
        fprintf('case %d (ax %d, a=%.5f, sn=%.1f nm): raw C_dpmr44=%.4f C_n44=%.4f | v2(LP) C_dpmr=%.4f C_n=%.4f | y2leak=%.3g\n', ...
            i, ax, a, 1e3*SN(ax), out.C_dpmr_raw(i), out.C_n_raw(i), ...
            out.C_dpmr(i), out.C_n(i), out.y2leak(i));
    end
    fprintf('cross-check raw: C_dpmr44/C_dpmr36 = %.3f (Scenario-0 measured ~1.91)\n', ...
        out.C_dpmr_raw(3)/(2+1/(1-lc^2)));
    fprintf('LP under-read on (4.4): v2/raw = %.4f (frozen-arm measured ~0.91)\n', ...
        out.C_dpmr(3)/out.C_dpmr_raw(3));

    % ---- v3: Jensen inflation of the loop variance under 1/a-hat noise ----
    % V(gamma): EWMA-visible variance when the TRUE loop gain is gamma = a/a_hat
    % (plant-side columns scale by gamma; the estimator's K, R22 stay at the
    % believed operating point). J(sigma) = E_eps[ V(1/(1-eps)) ] / V(1) with
    % eps ~ N(0, sigma^2), eps = (a - a_hat)/a read at runtime as sqrt(P66)/a_hat.
    fprintf('\n--- v3 Jensen factor (z axis, a = a_N) ---\n');
    a = a_N; sn2 = SN(3)^2;
    Q = zeros(7); Q(3,3) = 4*kBT*a;
    R22 = cc.R22_prefactor * cc.IF_eff * (a + cc.xi_per_axis(3))^2;
    Rm = [sn2 0; 0 R22];
    P = eye(7);
    for it = 1:20000
        Pf = Ferr*P*Ferr' + Q; Pf = 0.5*(Pf+Pf');
        S = H2*Pf*H2' + Rm; K = (Pf*H2')/S;
        P = (eye(7)-K*H2)*Pf/lambda_f; P = 0.5*(P+P');
    end
    alpha = cc.a_pd;  oma = 1-alpha;
    Vg = @(g) local_Vg(g, mu, K, H2, Fs, alpha, oma);
    V1 = Vg(1);
    gam_grid = 0.6:0.05:1.8;  Vrel = zeros(size(gam_grid));
    for i = 1:numel(gam_grid); Vrel(i) = Vg(gam_grid(i))/V1; end
    fprintf('V(gamma)/V(1): '); fprintf('%.3g ', Vrel); fprintf('\n');
    c2 = (Vg(1.05)+Vg(0.95)-2*V1)/V1/0.05^2/2;
    fprintf('curvature c2 = %.2f;  c1 = %.2f\n', c2, (Vg(1.05)-Vg(0.95))/V1/0.1);
    % J(sigma) by Gauss-Hermite (20-node) with instability clipping
    [xg, wg] = local_gh(20);
    sig_grid = 0:0.02:0.30;  J = ones(size(sig_grid));
    for si = 2:numel(sig_grid)
        s_ = sig_grid(si); acc = 0; wacc = 0;
        for q = 1:20
            eps_ = sqrt(2)*s_*xg(q);
            g_ = 1/(1-eps_);
            if g_ > 0 && g_ < 1.75
                v_ = Vg(g_)/V1;
                if v_ < 50; acc = acc + wg(q)*v_; wacc = wacc + wg(q); end
            end
        end
        J(si) = acc/wacc;
    end
    out.jensen = struct('sigma_grid', sig_grid, 'J', J, 'c2', c2);
    fprintf('J(sigma): '); fprintf('%.3f ', J); fprintf('\n');
end


function V = local_Vg(g, mu, K, H2, Fs, alpha, oma)
%LOCAL_VG  EWMA-visible variance (thermal-unit) at true loop gain g = a/a_hat.
    A = zeros(11);
    A(1,1)=1; A(1,6)=-mu*g; A(1,7)=g; A(2,1)=1; A(3,2)=1;
    A(4:10,4:10) = (eye(7)-K*H2)*Fs;
    A(4:10,3) = K(:,1);
    A(11,3) = alpha; A(11,11) = oma;
    Cv = oma * [0 0 1 zeros(1,7) -1];
    ST = lyap_ss(A, [-1; zeros(10,1)]);
    V = Cv*ST*Cv';
end


function [x, w] = local_gh(n)
%LOCAL_GH  Gauss-Hermite nodes/weights via the Golub-Welsch companion.
    b = sqrt((1:n-1)/2);
    T = diag(b,1) + diag(b,-1);
    [ev, ed] = eig(T);
    x = diag(ed);  w = (ev(1,:).^2)' * sqrt(pi);
    [x, ix] = sort(x); w = w(ix);
    w = w / sqrt(pi);         % normalize to a probability measure
end

function S = lyap_ss(A, B)
    S = zeros(size(A));
    for it = 1:60000
        Sn = A*S*A' + B*B';
        if max(abs(Sn(:)-S(:))) < 1e-16*max(1,max(abs(Sn(:)))), S = Sn; return; end
        S = Sn;
    end
end
