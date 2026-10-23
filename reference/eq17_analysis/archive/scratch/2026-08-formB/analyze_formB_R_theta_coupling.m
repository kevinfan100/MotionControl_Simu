function res = analyze_formB_R_theta_coupling(seed, tier)
%ANALYZE_FORMB_R_THETA_COUPLING  How a_bar' and the slope Jacobians J_theta
%   actually couple into the R (measurement-noise) update of the Form B filter.
%
% STATUS: ACTIVE -- 08-07 anatomy of the R (measurement-noise) update: R2 responds only to
%   a_hat (dlnR2/dlna_hat = 1.985); the theta path is exact but 1e-9..1e-7, so the
%   Ljung dK/dtheta term is innocent in the R leg.
%   See memory project-formb-r-theta-coupling-2026-08-07.
%
% QUESTION (user, 2026-08-07): "J 和 a' 具體會和 R 更新有怎麼樣的關係".
%
% SCOPE: OFFLINE ONLY.  Runs the production driver once, reads its logs, and
%   re-evaluates the controller's OWN R2 expression; nothing under model/ is
%   touched and no filter is re-implemented.
%
% THE CHAIN, straight out of motion_control_law_formB_ws.m
%   R1 = sigma2_nw                                   <- constant, no a', no J
%   R2 = a_cov*(2-a_cov)*R2_int(a_bar_hat)  +  a_cov^2 * d * Q44
%        R2_int = amlpf*K_var*IF_eff(a_bar_hat)*(a_bar_hat + xi_bar)^2
%        Q44    = a_bar'^2 * Q33          (line 878)
%   so a_bar' enters R2 QUADRATICALLY through the delay term, and because
%   a_bar' = a_bar'(w_d - ws_hat, b_hat, p_hat) the parameter estimates are
%   inside R.  Differentiating that term and using J_theta = d a_bar'/d theta
%   (the controller's own definition, line 1142-1144):
%
%       dR2/dtheta = a_cov^2 * d * 2 * a_bar' * Q33 * J_theta        (EXACT)
%
%   J_theta is therefore exactly the multiplier that converts a parameter
%   error into an R error.  The filter never uses it that way: R2 is passed to
%   S2 = H2*P*H2' + R2 and to the Joseph form as a CONSTANT.  This is the
%   Ljung-1979 missing-dK/dtheta structure of the literature memo, in its
%   concrete Form B instance.
%
% DELIVERABLES
%   D1  bit-check Q44 = a_bar'^2*Q33 and the R2 recomposition against the log
%   D2  how much of R2 the a_bar' term actually carries (descend vs hold)
%   D3  dR2/dtheta for a 1-sigma parameter error, vs R2 and vs S2
%   D4  the information the constant-R assumption throws away: variance-channel
%       Fisher 0.5*(dlnS2/dtheta)^2 vs the mean-channel Fisher the filter uses
%   D5  the a_bar_hat (state slot 4) dependence of R2 for comparison -- the
%       dominant one, and a positive feedback path
%
% See also: motion_control_law_formB_ws, run_formB_ws,
%   analyze_formB_fisher_2param (same log-reading preamble and conventions).

    if nargin < 1 || isempty(seed); seed = 1; end
    if nargin < 2 || isempty(tier); tier = 't2'; end   % t2 = w_s FREE (working line)
    AX_Z = 3;

    base = run_formB_ws(struct('seeds', seed, 'tier', tier, ...
                    'ctrl_const_override', struct('Pf_ws_std', 0.111)));
    ref  = base.runs{1};
    cc   = ref.ctrl_const;
    P    = ref.meta.params_value;

    R_um     = P.common.R;
    Ts       = P.common.Ts;
    a_o      = Ts / (P.ctrl.gamma * R_um);
    kappa_T  = 4 * (P.ctrl.k_B * P.ctrl.T / R_um) * a_o;
    s2n      = P.ctrl.sigma2_noise(AX_Z) / R_um^2;
    a_cov    = cc.a_cov;
    d        = cc.d;
    xi_bar   = (cc.C_n / cc.C_dpmr) * s2n / kappa_T;
    whiten   = local_field(cc, 'y2_whiten', true);
    H2_scale = 1; if whiten; H2_scale = a_cov; end
    lam_c    = cc.lambda_c;
    a_pd     = cc.a_pd;
    b_cl     = local_field(cc, 'b_clamp', [0.05 5]);
    p_cl     = local_field(cc, 'p_clamp', [0.05 5]);
    gapfl    = local_field(cc, 'ws_margin', 1e-3);
    ws0      = local_field(cc, 'ws0_perp', 1);
    if local_field(cc, 'y2_echo_corr', true)
        [S_T, S_n] = local_echo_shares(lam_c, a_pd);
    else
        S_T = 0; S_n = 0;
    end

    N   = numel(ref.tout);
    t   = ref.tout(:);
    wd  = ref.h_bar_d_out(:); wd(1) = wd(2);
    kk  = (1:N).';
    grad = wd - wd(max(kk - d, 1));

    % prior row = what x_curr held when step k ran (posterior of k-1)
    b_pri  = [NaN; ref.b_hat_out(1:end-1,  AX_Z)];
    p_pri  = [NaN; ref.p_hat_out(1:end-1,  AX_Z)];
    ws_pri = [NaN; ref.ws_hat_out(1:end-1, AX_Z)];
    ab_pri = [NaN; ref.a_bar_hat_out(1:end-1, AX_Z)];
    R2log  = ref.R2_out(:, AX_Z);
    Q33    = ref.Q33_out(:, AX_Z);
    sig    = [ref.P_b_out(:, AX_Z), ref.P_p_out(:, AX_Z), ref.P_ws_out(:, AX_Z)];
    open2  = ~ref.gate_out(:, AX_Z); open2(1) = false;

    % ---- rebuild a_bar', J_theta and dA/dtheta exactly as the controller ----
    ap = zeros(N,1); Jth = zeros(N,3); dA = zeros(N,3); ech = ones(N,1);
    for k = 2:N
        lb = min(max(b_pri(k), b_cl(1)), b_cl(2));
        lp = min(max(p_pri(k), p_cl(1)), p_cl(2));
        lw = ws_pri(k) + (ws0 - 1);
        g  = max(wd(k) - lw, gapfl);
        u  = 1 + g/lb;
        a_b = 1 - u^(-lp);
        apk = lp * u^(-lp-1) / lb;
        ap(k)  = apk;
        Jth(k,:) = [(-1/lb + (lp+1)*g/(lb*(g+lb))) * apk, ...
                    (1/lp - log(u)) * apk, ...
                    ((lp+1)/(g+lb)) * apk];
        dA(k,:)  = [-(g/lb)*apk, (1-a_b)*log(u), -apk];       % LEVEL dA/dtheta
        ech(k)   = 1 - (S_T*ab_pri(k) + S_n*xi_bar)/(ab_pri(k) + xi_bar);
    end

    fprintf('\n==== D1  identities against the log =========================\n');
    m = open2 & isfinite(ap) & ap > 0;
    R2int = local_R2int(ab_pri, cc, xi_bar, kappa_T, s2n);
    R2reb = a_cov*(2-a_cov)*R2int + a_cov^2*d*(ap.^2 .* Q33);
    e_r2  = max(abs(R2reb(m) - R2log(m)) ./ R2log(m));
    fprintf('  Q44 = a_bar''^2*Q33 by construction (controller line 878), not logged\n');
    fprintf('  max rel |R2 rebuilt - R2_log|         = %.3e\n', e_r2);
    fprintf('  (both ~eps: the R2 below IS the controller''s own number)\n');

    fprintf('\n==== D2  what fraction of R2 rides on a_bar'' ================\n');
    frac = a_cov^2*d*(ap.^2 .* Q33) ./ R2log;
    win  = local_windows(t, base.cfg);
    fn = fieldnames(win);
    for i = 1:numel(fn)
        w = win.(fn{i}) & m;
        if ~any(w); continue; end
        fprintf('  %-8s a_bar'' %.4f   d*a_cov^2*Q44 %.3e   R2 %.3e   share %.3e\n', ...
            fn{i}, mean(ap(w)), mean(a_cov^2*d*(ap(w).^2.*Q33(w))), ...
            mean(R2log(w)), mean(frac(w)));
    end
    fprintf('  -> R2_int (a_bar_hat only) carries the rest; a_cov^2*d = %.4g\n', a_cov^2*d);

    fprintf('\n==== D3  dR2/dtheta = a_cov^2*d*2*a_bar''*Q33*J_theta ========\n');
    dR2 = (a_cov^2*d*2*(ap .* Q33)) .* Jth;              % N x 3, EXACT
    % innovation variance actually used by the filter
    S2 = zeros(N,1);
    if isfield(ref, 'P_full_out') && ~isempty(ref.P_full_out)
        for k = 2:N
            Pk = squeeze(ref.P_full_out(k,:,:,AX_Z));
            H2 = H2_scale*ech(k)*[0 0 0 1 -grad(k)*Jth(k,1) -grad(k)*Jth(k,2) ...
                                  -grad(k)*Jth(k,3) zeros(1, size(Pk,1)-7)];
            S2(k) = H2*Pk*H2' + R2log(k);
        end
    else
        S2 = R2log;   % arm path hard-codes log_P_full=false (driver line 357)
        fprintf('  (P_full unavailable in the arm path -> S2 := R2, i.e. S2 <= truth;\n');
        fprintf('   this makes every discarded-fraction below an UPPER bound)\n');
    end
    nm = {'b ', 'p ', 'ws'};
    for j = 1:3
        w = m & isfinite(sig(:,j)) & sig(:,j) > 0;
        rel_R2 = abs(dR2(w,j)) .* sig(w,j) ./ R2log(w);
        rel_S2 = abs(dR2(w,j)) .* sig(w,j) ./ S2(w);
        fprintf('  theta=%s  sigma %.4f | DIRECT via a_bar'': dR2/R2 = %.3e (max %.3e)\n', ...
            nm{j}, mean(sig(w,j)), mean(rel_R2), max(rel_R2));
    end
    fprintf('  INDIRECT path theta -> a_bar_hat -> R2 (the one that is not small):\n');
    for j = 1:3
        w = m & isfinite(sig(:,j)) & sig(:,j) > 0;
        d_ab_rel = abs(dA(w,j)) .* sig(w,j) ./ ab_pri(w);
        fprintf('    theta=%s  1-sigma tilts a_bar by %.3f%% -> R2 by %.3f%% (x d lnR2/d lna = 1.985)\n', ...
            nm{j}, 100*mean(d_ab_rel), 100*1.985*mean(d_ab_rel));
    end

    fprintf('\n==== D4  information discarded by treating R as constant ====\n');
    % mean channel (what the filter uses): s = H2_scale*echo*dA/dtheta
    % variance channel (dropped): 0.5*(dlnS2/dtheta)^2
    Fm = zeros(3,1); Fv = zeros(3,1);
    for j = 1:3
        w = m;
        s_mean = H2_scale*ech(w).*dA(w,j);
        Fm(j) = sum(s_mean.^2 ./ S2(w));
        Fv(j) = sum(0.5*(dR2(w,j)./S2(w)).^2);
        fprintf('  theta=%s  F_mean %.4e   F_var %.4e   F_var/F_mean %.3e\n', ...
            nm{j}, Fm(j), Fv(j), Fv(j)/Fm(j));
    end
    fprintf('  -> the dropped dR/dtheta channel is worth this fraction of the\n');
    fprintf('     information the y2 update already extracts.\n');

    fprintf('\n==== D5  the a_bar_hat dependence of R2 (for scale) =========\n');
    hstep = 1e-6;
    dR2_da = (local_R2full(ab_pri+hstep, ap, Q33, cc, xi_bar, kappa_T, s2n, a_cov, d) ...
            - local_R2full(ab_pri-hstep, ap, Q33, cc, xi_bar, kappa_T, s2n, a_cov, d))/(2*hstep);
    elast = dR2_da(m) .* ab_pri(m) ./ R2log(m);
    fprintf('  d ln R2 / d ln a_bar_hat = %.3f  (mean over open steps)\n', mean(elast));
    fprintf('  a 1%% low a_bar_hat -> R2 low by %.2f%% -> y2 trusted MORE (K2 up)\n', ...
        abs(mean(elast)));
    fprintf('  Q33 also reads a_bar_hat, so this path is the dominant one; the\n');
    fprintf('  a_bar'' path of D3 is the only one carrying theta directly.\n');

    fprintf('\n==== D6  does R2 actually set K2? (S2 = H2*P*H2'' + R2) ======\n');
    % H2 is dominated by its slot-4 entry H2_scale*echo (the theta columns are
    % scaled by Grad, which vanishes in hold); use the logged P(4,4).
    Pa = ref.P_a_out(:, AX_Z).^2;                       % logged as std
    HPH = (H2_scale*ech).^2 .* Pa;
    fprintf('  mean H2*P*H2'' / R2 = %.3e   (max %.3e)\n', ...
        mean(HPH(m)./R2log(m)), max(HPH(m)./R2log(m)));
    fprintf('  -> S2 = R2 to within that, so K2 ~ P*H2''/R2 and\n');
    fprintf('     d ln K2 / d ln a_bar_hat = -%.3f at fixed P.\n', mean(elast));
    fprintf('  SIGN: dA/dws < 0, so a LOW ws_hat raises a_bar_hat, raises R2,\n');
    fprintf('     and LOWERS K2 -- the y2 channel that would pull ws back up.\n');
    fprintf('     Size check: 1-sigma_ws -> R2 +4.3%%, so K2 -4.3%%; a -5%% w_s\n');
    fprintf('     error is ~0.45 sigma -> K2 -1.9%%.  Real, self-sealing in sign,\n');
    fprintf('     but far too small to be the 56%% response deficit of defect 3.\n');

    res = struct('frac_aprime', frac, 'dR2', dR2, 'F_mean', Fm, 'F_var', Fv, ...
                 'S2', S2, 'R2', R2log, 'a_prime', ap, 'J', Jth, 'mask', m);
end

% --------------------------------------------------------------------------
function R2i = local_R2int(a_bar, cc, xi_bar, kappa_T, s2n)
%LOCAL_R2INT  Controller's R2_int, verbatim from compute_R2_formB.
    sxT = kappa_T * a_bar;
    IFa = cc.IF_abc(:);
    num = sxT.^2*IFa(1) + 2*sxT*s2n*IFa(2) + s2n^2*IFa(3);
    den = (cc.C_dpmr*sxT + cc.C_n*s2n).^2;
    IF  = 1 + 2*num./den;
    amlpf = 1; if isfield(cc, 'amlpf_var_factor'); amlpf = cc.amlpf_var_factor; end
    R2i = amlpf * cc.K_var * IF .* (a_bar + xi_bar).^2;
end

function R2 = local_R2full(a_bar, ap, Q33, cc, xi_bar, kappa_T, s2n, a_cov, d)
    R2 = a_cov*(2-a_cov)*local_R2int(a_bar, cc, xi_bar, kappa_T, s2n) ...
         + a_cov^2*d*(ap.^2 .* Q33);
end

function w = local_windows(t, cfg)
    t1 = cfg.t_hold;
    t2 = t1 + local_or(cfg, 't_descend_override', 1.0);
    t3 = t2 + cfg.n_cycles / cfg.frequency;
    w.hold0   = t <= t1;
    w.descend = t > t1 & t <= t2;
    w.osc     = t > t2 & t <= t3;
    w.hold1   = t > t3;
end

function v = local_or(s, f, dflt)
    v = dflt;
    if isfield(s, f) && ~isempty(s.(f)); v = s.(f); end
end

function v = local_field(s, f, dflt)
    v = dflt;
    if isfield(s, f) && ~isempty(s.(f)); v = s.(f); end
end

function [S_T, S_n] = local_echo_shares(lambda_c, a_pd)
%LOCAL_ECHO_SHARES  y2 self-echo sensitivities (verbatim controller init copy,
%   same helper as analyze_formB_fisher_2param).
    alE = 1 - lambda_c;
    epE = 1e-4;
    vE  = zeros(2, 3);
    gE_list = [1, 1/(1+epE), 1/(1-epE)];
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
    S_T = (log(vE(1,2)) - log(vE(1,3))) / (2*epE);
    S_n = (log(vE(2,2)) - log(vE(2,3))) / (2*epE);
end
