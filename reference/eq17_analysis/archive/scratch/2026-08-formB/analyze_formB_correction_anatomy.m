function res = analyze_formB_correction_anatomy(seed, tier)
%ANALYZE_FORMB_CORRECTION_ANATOMY  Where the gain law a_bar, its slope a_bar'
%   and the parameter Jacobians J_theta actually act inside the KF CORRECTION
%   (measurement-update) step of the Form B filter.
%
% STATUS: ACTIVE -- 08-07 anatomy of the correction step: H1 carries no law at all, H2's J
%   column is always multiplied by Grad (|B/A| = 0.1-0.4 %), so 99.6 % of the
%   parameter update comes through P built in predict.
%   See memory project-formb-r-theta-coupling-2026-08-07.
%
% QUESTION (user, 2026-08-07): "函數設定的和它的導數 J 具體會怎麼樣影響到 KF
%   的回授更正部分?  H? R?"
%
% SCOPE: OFFLINE ONLY.  Runs the production arm once to capture its exact
%   (cfg, ctrl_const), replays the SAME seed through the driver's test-ladder
%   entry with log_P_full so P is available, checks the replay is bit-identical,
%   then dissects the update step.  Nothing under model/ is touched.
%
% THE CORRECTION STEP, verbatim from motion_control_law_formB_ws.m
%   x = [dw1 dw2 dw3  a_bar_w  b  p  w_s]   (+ m1 m2 when ma2_aug)
%
%   (a) y1 = dw_m                                       (line 955)
%         H1 = [1 0 0 0 0 0 0]        <- the law is NOT here at all
%         K1 = P*H1'/S1 ,  S1 = P(1,1) + R1 ,  R1 = sigma2_nw  (constant)
%       so y1 can only move b/p/w_s through P(theta,1), a cross-covariance
%       that exists ONLY because F_e row 4 coupled them during predict:
%         F_e(4,:) = [0 0 (1-lc)a'  1+a'F_dw   J_b*M  J_p*M  J_ws*M]
%
%   (b) y2 = whitened gain readout                      (line 981)
%         H2 = a_cov*(1-S_echo)*[0 0 0 1  -Grad*J_b  -Grad*J_p  -Grad*J_ws]
%         y2_pred = a_cov*( a_bar_hat - (1-S_echo)*a_bar'*Grad )
%         S2 = H2*P*H2' + R2 ,  K2 = P*H2'/S2
%       J_theta appears in H EXACTLY once, always multiplied by Grad (the
%       d-step COMMANDED travel).  a_bar' appears in the nonlinear predicted
%       measurement, i.e. inside the innovation, not in H*x.
%
%   So (P*H2')_theta splits into two competing terms:
%       A = P(theta,4)                       accumulated, built by F_e row 4
%       B = -Grad*sum_phi P(theta,phi)*J_phi instantaneous, via H's theta cols
%   D3 measures A vs B.  In every hold Grad = 0 exactly, so B vanishes and the
%   whole parameter correction is A.
%
% DELIVERABLES
%   D1 replay check + H row contents
%   D2 which channel actually moves each parameter: y1 vs y2 correction budget
%   D3 A vs B split of the y2 parameter gain
%   D4 hold vs motion: what survives when Grad = 0
%   D5 R's role: S1 = P(1,1)+R1 vs S2 = H2*P*H2'+R2, i.e. what R actually gates
%
% See also: analyze_formB_R_theta_coupling (the R side of the same question),
%   analyze_formB_fisher_2param (information accounting).

    if nargin < 1 || isempty(seed); seed = 1; end
    if nargin < 2 || isempty(tier); tier = 't2'; end
    AX = 3;

    arm = run_formB_ws(struct('seeds', seed, 'tier', tier, ...
                       'ctrl_const_override', struct('Pf_ws_std', 0.111)));
    cfg = arm.cfg;  cc = arm.runs{1}.ctrl_const;
    ref = run_formB_ws(cfg, struct('seed', seed, 'ctrl_const_override', cc, ...
                                   'log_P_full', true));

    fprintf('\n==== D1  replay check + H rows ==============================\n');
    e_rep = max(abs(ref.ws_hat_out(:,AX) - arm.runs{1}.ws_hat_out(:,AX)));
    fprintf('  max |ws_hat replay - arm| = %.3e  (0 = same run, P now logged)\n', e_rep);

    P    = ref.meta.params_value;
    R_um = P.common.R;
    Ts   = P.common.Ts;
    a_o  = Ts / (P.ctrl.gamma * R_um);
    kT   = 4 * (P.ctrl.k_B * P.ctrl.T / R_um) * a_o;
    s2n  = P.ctrl.sigma2_noise(AX) / R_um^2;
    a_cov = cc.a_cov;  d = cc.d;
    xi   = (cc.C_n / cc.C_dpmr) * s2n / kT;
    H2s  = 1; if local_f(cc,'y2_whiten',true); H2s = a_cov; end
    bcl  = local_f(cc,'b_clamp',[0.05 5]); pcl = local_f(cc,'p_clamp',[0.05 5]);
    gfl  = local_f(cc,'ws_margin',1e-3);   ws0 = local_f(cc,'ws0_perp',1);
    if local_f(cc,'y2_echo_corr',true)
        [S_T, S_n] = local_echo_shares(cc.lambda_c, cc.a_pd);
    else
        S_T = 0; S_n = 0;
    end

    N  = numel(ref.tout);  t = ref.tout(:);
    wd = ref.h_bar_d_out(:); wd(1) = wd(2);
    kk = (1:N).';
    grad = wd - wd(max(kk-d,1));
    b_pri  = [NaN; ref.b_hat_out(1:end-1,AX)];
    p_pri  = [NaN; ref.p_hat_out(1:end-1,AX)];
    ws_pri = [NaN; ref.ws_hat_out(1:end-1,AX)];
    ab_pri = [NaN; ref.a_bar_hat_out(1:end-1,AX)];
    R2  = ref.R2_out(:,AX);  R1 = s2n;
    open2 = ~ref.gate_out(:,AX); open2(1) = false;
    np  = size(ref.P_full_out, 2);

    ap = zeros(N,1); J = zeros(N,3); ech = ones(N,1);
    for k = 2:N
        lb = min(max(b_pri(k),bcl(1)),bcl(2));
        lp = min(max(p_pri(k),pcl(1)),pcl(2));
        g  = max(wd(k) - (ws_pri(k)+ws0-1), gfl);
        u  = 1 + g/lb;
        ap(k) = lp*u^(-lp-1)/lb;
        J(k,:) = [(-1/lb + (lp+1)*g/(lb*(g+lb))), (1/lp - log(u)), ((lp+1)/(g+lb))]*ap(k);
        ech(k) = 1 - (S_T*ab_pri(k) + S_n*xi)/(ab_pri(k) + xi);
    end
    fprintf('  H1 = [1 0 ... 0]                       -> no a_bar, no a'', no J\n');
    fprintf('  H2 = %.4f*echo*[0 0 0 1 -Grad*J_b -Grad*J_p -Grad*J_ws]\n', H2s);
    fprintf('  mean echo = %.4f (y2 self-echo 1-S), mean |Grad| = %.4f R\n', ...
        mean(ech(2:end)), mean(abs(grad)));

    % ---- per-step gains -------------------------------------------------
    % CAVEAT: P_full is logged as the POSTERIOR of step k; the filter forms K1
    % from P_pred and K2 from P after the y1 update.  The K1/K2 below are
    % therefore INDICATIVE (right order, right structure) -- ratios that share
    % the same P (D3) are unaffected; absolute budgets (D2) are not exact.
    % The one exact channel split available from the logs is dws_y1_out.
    K1 = zeros(N,np); K2 = zeros(N,np); S1v = zeros(N,1); S2v = zeros(N,1);
    Aterm = zeros(N,3); Bterm = zeros(N,3);
    for k = 2:N
        Pk = squeeze(ref.P_full_out(k,:,:,AX));
        H1 = [1, zeros(1,np-1)];
        S1v(k) = Pk(1,1) + R1;
        K1(k,:) = (Pk*H1.')/S1v(k);
        if open2(k)
            H2 = H2s*ech(k)*[0 0 0 1 -grad(k)*J(k,1) -grad(k)*J(k,2) -grad(k)*J(k,3) ...
                             zeros(1,np-7)];
            S2v(k) = H2*Pk*H2.' + R2(k);
            K2(k,:) = (Pk*H2.')/S2v(k);
            for j = 1:3
                th = 4+j;
                Aterm(k,j) = Pk(th,4);
                Bterm(k,j) = -grad(k)*(Pk(th,5)*J(k,1) + Pk(th,6)*J(k,2) + Pk(th,7)*J(k,3));
            end
        end
    end
    m2 = open2 & isfinite(S2v) & S2v > 0;
    m1 = (1:N).' > 1;

    fprintf('\n==== D2  which channel moves each parameter =================\n');
    i1 = ref.innov_y1_out(:,AX); i2 = ref.innov_y2_out(:,AX);
    nm = {'b ','p ','ws'};
    for j = 1:3
        th = 4+j;
        c1 = sum(abs(K1(m1,th).*i1(m1)));
        c2 = sum(abs(K2(m2,th).*i2(m2)));
        fprintf('  %s  sum|K1*innov1| = %.4e   sum|K2*innov2| = %.4e   y2/y1 = %.3f\n', ...
            nm{j}, c1, c2, c2/c1);
    end
    fprintf('  (y1 has NO law in H: its whole parameter authority is P(theta,1),\n');
    fprintf('   which F_e row 4 built from a_bar'' and J during predict)\n');

    fprintf('\n==== D3  y2 parameter gain: accumulated A vs instantaneous B ==\n');
    for j = 1:3
        w = m2 & abs(Aterm(:,j)) > 0;
        r = abs(Bterm(w,j))./abs(Aterm(w,j));
        fprintf('  %s  |B/A| mean %.3e  max %.3e   (A = P(theta,4), B = -Grad*P(theta,5:7)*J)\n', ...
            nm{j}, mean(r), max(r));
    end
    fprintf('  -> J''s DIRECT appearance in H is worth this much of the parameter\n');
    fprintf('     gain; the rest is P(theta,4), i.e. J acting through F_e/predict.\n');

    fprintf('\n==== D4  hold vs motion (Grad = 0 exactly in hold) ==========\n');
    win = local_win(t, cfg); fn = fieldnames(win);
    for i = 1:numel(fn)
        w = win.(fn{i}) & m2;
        if ~any(w); continue; end
        fprintf('  %-8s |Grad| %.4f  |K2(ws)| %.3e  |K2(b)| %.3e  |K2(a)| %.3e\n', ...
            fn{i}, mean(abs(grad(w))), mean(abs(K2(w,7))), mean(abs(K2(w,5))), ...
            mean(abs(K2(w,4))));
    end

    fprintf('\n==== D4b NET drift attribution (F_e rows 5-7 = identity, so the\n');
    fprintf('         whole theta history IS the sum of the two corrections) ==\n');
    % EXACT for w_s: the controller logs K1(7)*innov1 per step (dws_y1).
    dws_tot = ref.ws_hat_out(end,AX) - ref.ws_hat_out(1,AX);
    dws_1   = sum(ref.dws_y1_out(:,AX));
    fprintf('  ws  total %+.4e = y1 %+.4e (logged, exact) + y2 %+.4e (residual)\n', ...
        dws_tot, dws_1, dws_tot - dws_1);
    fprintf('      -> y2 carries %.1f%% of the net w_s move, and it is the NEGATIVE one\n', ...
        100*abs(dws_tot - dws_1)/(abs(dws_1) + abs(dws_tot - dws_1)));
    fprintf('  b/p: no per-channel log exists; the K-based estimate is indicative only\n');

    fprintf('\n==== D5  what R actually gates =============================\n');
    P11 = ref.P_full_out(:,1,1,AX);
    fprintf('  y1: S1 = P(1,1) + R1 ; mean P(1,1)/R1 = %.3f  -> R1 shares control\n', ...
        mean(P11(m1))/R1);
    fprintf('  y2: S2 = H2*P*H2'' + R2 ; mean H2PH2''/R2 = %.3e -> R2 alone sets K2\n', ...
        mean((S2v(m2)-R2(m2))./R2(m2)));
    fprintf('  R1 = sigma2_nw constant; R2 reads a_bar_hat only (elasticity ~1.985),\n');
    fprintf('  a_bar'' enters R2 only via the inert d*Q44 term (see the R-coupling script).\n');

    res = struct('K1',K1,'K2',K2,'A',Aterm,'B',Bterm,'S1',S1v,'S2',S2v, ...
                 'J',J,'a_prime',ap,'grad',grad,'mask2',m2);
end

% --------------------------------------------------------------------------
function v = local_f(s,f,dflt)
    v = dflt; if isfield(s,f) && ~isempty(s.(f)); v = s.(f); end
end

function w = local_win(t,cfg)
    t1 = cfg.t_hold; t2 = t1 + cfg.t_descend_override;
    t3 = t2 + cfg.n_cycles/cfg.frequency;
    w.hold0 = t <= t1; w.descend = t > t1 & t <= t2;
    w.osc = t > t2 & t <= t3; w.hold1 = t > t3;
end

function [S_T, S_n] = local_echo_shares(lambda_c, a_pd)
%LOCAL_ECHO_SHARES  y2 self-echo sensitivities (verbatim controller init copy).
    alE = 1 - lambda_c; epE = 1e-4; vE = zeros(2,3);
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
    S_T = (log(vE(1,2)) - log(vE(1,3)))/(2*epE);
    S_n = (log(vE(2,2)) - log(vE(2,3)))/(2*epE);
end
