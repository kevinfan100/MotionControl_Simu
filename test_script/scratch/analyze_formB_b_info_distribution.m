function res = analyze_formB_b_info_distribution(seed)
%ANALYZE_FORMB_B_INFO_DISTRIBUTION  Where along the trajectory does the y2
%   channel's information about b actually come from, and how do Form B and
%   Form C compare on the SAME trajectory?
%
% STATUS: ACTIVE -- where along the trajectory the y2 channel's information about b comes
%   from, B vs C on the SAME trajectory. Feeds the coupling ledger.
%   See memory project-formb-coupling-ledger-2026-08-09.
%
% WHY.  Every "Form C is better" number so far was quoted at the trough,
%   because an earlier audit found 68% of the y2 information sits in the final
%   trough hold.  But that 68% was measured for w_s, NOT for b.  b's
%   sensitivity is nowhere near zero in the far field, and the descent is long,
%   so b's information may well be descent-dominated.  This script measures it
%   instead of assuming it.
%
% WHAT IS COMPUTED.  Per open-gate step, the y2 level-channel information
%       F_b[k] = s_b[k]^2 / R2[k] ,   s_b = H2_scale * echo * dA/db
%   with dA/db taken from each form at the SAME logged (w_bar_d, w_s_hat):
%       Form B   dA/db = -(gap/b_hat) * a_bar'          (controller's own form)
%       Form C   dA/db = -1/(1 + gap)                   (b-free)
%   Same trajectory, same R2, same gate, same echo -- so the ratio is a like
%   for like statement about the two laws, not about two different runs.
%
%   Also pulled out, for the per-seed-scatter question: P(w_s, a_bar) from the
%   logged covariance, to see whether the w_s coupling swings with the
%   commanded motion the way P(b, a_bar) does.
%
% SCOPE: OFFLINE.  t2 arm (both b and w_s free) replayed with log_P_full.
%   Nothing under model/ is touched.
%
%   See also: analyze_formB_Jb_coupling, analyze_formB_fisher_2param

    if nargin < 1 || isempty(seed); seed = 1; end
    AX = 3;

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    fig_dir = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');

    arm = run_formB_ws(struct('seeds', seed, 'tier', 't2', ...
                       'ctrl_const_override', struct('Pf_ws_std', 0.111)));
    cfg = arm.cfg;  cc = arm.runs{1}.ctrl_const;
    ref = run_formB_ws(cfg, struct('seed', seed, 'ctrl_const_override', cc, ...
                                   'log_P_full', true));
    fprintf('\n[b info]  replay check: max |ws_hat replay - arm| = %.3e\n', ...
        max(abs(ref.ws_hat_out(:,AX) - arm.runs{1}.ws_hat_out(:,AX))));

    P  = ref.meta.params_value;
    a_cov = cc.a_cov;
    H2s = 1; if local_f(cc,'y2_whiten',true); H2s = a_cov; end
    kT  = 4*(P.ctrl.k_B*P.ctrl.T/P.common.R)*(P.common.Ts/(P.ctrl.gamma*P.common.R));
    s2n = P.ctrl.sigma2_noise(AX)/P.common.R^2;
    xi  = (cc.C_n/cc.C_dpmr)*s2n/kT;
    if local_f(cc,'y2_echo_corr',true)
        [S_T, S_n] = local_echo_shares(cc.lambda_c, cc.a_pd);
    else
        S_T = 0; S_n = 0;
    end
    b_cl = local_f(cc,'b_clamp',[0.05 5]); p_cl = local_f(cc,'p_clamp',[0.05 5]);
    gfl  = local_f(cc,'ws_margin',1e-3);   ws0  = local_f(cc,'ws0_perp',1);

    N  = numel(ref.tout); t = ref.tout(:);
    wd = ref.h_bar_d_out(:); wd(1) = wd(2);
    R2 = ref.R2_out(:,AX);
    open2 = ~ref.gate_out(:,AX); open2(1) = false;
    b_pri  = [NaN; ref.b_hat_out(1:end-1,AX)];
    p_pri  = [NaN; ref.p_hat_out(1:end-1,AX)];
    ws_pri = [NaN; ref.ws_hat_out(1:end-1,AX)];
    ab_pri = [NaN; ref.a_bar_hat_out(1:end-1,AX)];

    dB = zeros(N,1); dC = zeros(N,1); gap = zeros(N,1); ech = ones(N,1);
    for k = 2:N
        lb = min(max(b_pri(k),b_cl(1)),b_cl(2));
        lp = min(max(p_pri(k),p_cl(1)),p_cl(2));
        g  = max(wd(k) - (ws_pri(k)+ws0-1), gfl);
        ap = lp*(1+g/lb)^(-lp-1)/lb;
        gap(k) = g;
        dB(k)  = -(g/lb)*ap;             % Form B, controller's own dA_db
        dC(k)  = -1/(1+g);               % Form C
        ech(k) = 1 - (S_T*ab_pri(k) + S_n*xi)/(ab_pri(k) + xi);
    end

    w = open2 & isfinite(dB);
    FB = zeros(N,1); FC = zeros(N,1);
    FB(w) = (H2s*ech(w).*dB(w)).^2 ./ R2(w);
    FC(w) = (H2s*ech(w).*dC(w)).^2 ./ R2(w);
    cB = cumsum(FB); cC = cumsum(FC);

    % ---- phase breakdown -------------------------------------------------
    t1 = cfg.t_hold; t2 = t1 + cfg.t_descend_override;
    t3 = t2 + cfg.n_cycles/cfg.frequency;
    ph = {'hold0', t<=t1; 'descend', t>t1&t<=t2; 'osc', t>t2&t<=t3; 'hold1', t>t3};
    fprintf('\n  b 的 y2 資訊沿軌跡的分布\n');
    fprintf('  段        gap 範圍         Form B 佔比   Form C 佔比\n');
    for i = 1:4
        m = ph{i,2} & w;
        if ~any(m); continue; end
        fprintf('  %-8s %5.2f - %5.2f    %8.1f%%    %8.1f%%\n', ph{i,1}, ...
            min(gap(m)), max(gap(m)), 100*sum(FB(m))/sum(FB(w)), 100*sum(FC(m))/sum(FC(w)));
    end
    fprintf('\n  總資訊比  Form C / Form B = %.2f x\n', sum(FC(w))/sum(FB(w)));
    fprintf('  對照:谷底單點的靈敏度比 (C/B)^2 = %.2f x\n', ...
        (dC(find(abs(gap-1)<0.02 & w,1))/dB(find(abs(gap-1)<0.02 & w,1)))^2);
    sb0 = ref.P_b_out(2,AX); sbE = ref.P_b_out(end,AX);
    fprintf('  B 實測 sigma_b %.5f -> %.5f (縮 %.2f%%);  資訊 F_B = %.4e, 1/F_B = %.4e\n', ...
        sb0, sbE, 100*(1-sbE/sb0), sum(FB(w)), 1/sum(FB(w)));

    % ---- the per-seed question: does P(ws, a) swing with the motion? -----
    Pwa = squeeze(ref.P_full_out(:,7,4,AX));
    Pba = squeeze(ref.P_full_out(:,5,4,AX));

    % ---- figure ---------------------------------------------------------
    CB = [0 0.2 0.9]; CC = [0.45 0.55 0.95]; CR = [0.8 0 0]; CG = [0.45 0.45 0.45];
    FS = 20; LFS = 15; AXLW = 2.0; LW = 2.2;
    f = figure('Position',[80 80 1100 820],'Color','w','NumberTitle','off','Visible','off');
    tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

    nexttile; hold on;
    h1 = plot(t, cB/cB(end), '-', 'Color', CB, 'LineWidth', LW, 'DisplayName', 'Form B');
    h2 = plot(t, cC/cC(end), '-', 'Color', CC, 'LineWidth', LW, 'DisplayName', 'Form C');
    for x = [t1 t2 t3]
        xline(x, '--', 'Color', CG, 'LineWidth', 1.5, 'HandleVisibility', 'off');
    end
    ylabel('cumulative $b$ info', 'Interpreter','latex','FontSize',FS);
    legend([h1 h2],'Interpreter','latex','Location','northoutside', ...
           'Orientation','horizontal','FontSize',LFS,'Box','on');
    set(gca,'FontSize',FS,'LineWidth',AXLW,'Box','on', ...
            'TickLabelInterpreter','latex','XTickLabel',[]);
    xlim([0 t(end)]); ylim([0 1.05]); grid off;

    nexttile; hold on;
    h3 = plot(t, Pba, '-', 'Color', CB, 'LineWidth', LW, 'DisplayName', '$P(b,\bar{a})$');
    h4 = plot(t, Pwa, '-', 'Color', CR, 'LineWidth', LW, 'DisplayName', '$P(\bar{w}_s,\bar{a})$');
    yline(0,'-','Color',CG,'LineWidth',1.0,'HandleVisibility','off');
    for x = [t1 t2 t3]
        xline(x, '--', 'Color', CG, 'LineWidth', 1.5, 'HandleVisibility', 'off');
    end
    xlabel('$t$ \ [s]','Interpreter','latex','FontSize',FS);
    legend([h3 h4],'Interpreter','latex','Location','northoutside', ...
           'Orientation','horizontal','FontSize',LFS,'Box','on');
    set(gca,'FontSize',FS,'LineWidth',AXLW,'Box','on','TickLabelInterpreter','latex');
    xlim([0 t(end)]); grid off;

    out = fullfile(fig_dir,'formB_b_info_distribution.png');
    exportgraphics(f,out,'Resolution',200,'Padding','figure'); close(f);
    fprintf('  wrote %s\n', out);

    res = struct('t',t,'FB',FB,'FC',FC,'cB',cB,'cC',cC,'gap',gap, ...
                 'Pba',Pba,'Pwa',Pwa,'mask',w);
end

% --------------------------------------------------------------------------
function v = local_f(s,f,dflt)
    v = dflt; if isfield(s,f) && ~isempty(s.(f)); v = s.(f); end
end

function [S_T, S_n] = local_echo_shares(lambda_c, a_pd)
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
